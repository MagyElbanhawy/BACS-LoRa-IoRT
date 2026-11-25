"""ROS 2 node that runs the BACS scheduler with real constraint candidates.

Unlike scheduler_node.py (which sends deterministic test frames), this node:
  1. Generates realistic inter-robot SLAM constraint candidates via
     ConstraintSource (one every ~2 seconds).
  2. On each TX tick (every 11 s for 1% duty-cycle compliance at SF7/BW125),
     calls the scheduler's ranking logic to pick the BEST constraint from the
     pool, according to the active policy (FIFO / BACS / BACS+ / RANDOM).
  3. Serializes the selected constraint into a 52-byte LoRa payload and
     transmits it via TSeqHexSerialSender.
  4. Publishes the scheduler decision to /bacs/scheduler so ros2 bag can
     capture which constraint was selected and why.

This is the node used by record_bacs_sessions.sh for the 4-policy comparison.
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial as pyserial
except ImportError:
    pyserial = None

from .scheduler import DutyCycleBudget, Scheduler, lora_airtime_s
from .transport import TSeqHexSerialSender
from .constraint_source import ConstraintSource


class BacsSchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__('bacs_scheduler')

        # ---- Parameters -----------------------------------------------------
        self.declare_parameter('policy',         'FIFO')
        self.declare_parameter('config',        '')
        self.declare_parameter('serial_dev',    '/dev/ttyACM0')
        self.declare_parameter('serial_baud',   115200)
        self.declare_parameter('session_dir',   '')
        self.declare_parameter('session_id',    '')
        self.declare_parameter('tx_period_s',   11.0)
        self.declare_parameter('payload_bytes', 52)
        self.declare_parameter('constraint_seed', 42)
        self.declare_parameter('constraint_period_s', 2.0)
        self.declare_parameter('trust_threshold', 0.5)

        policy          = self.get_parameter('policy').value
        serial_dev      = self.get_parameter('serial_dev').value
        serial_baud     = int(self.get_parameter('serial_baud').value)
        session_dir     = self.get_parameter('session_dir').value
        session_id      = self.get_parameter('session_id').value
        tx_period_s     = float(self.get_parameter('tx_period_s').value)
        payload_bytes   = int(self.get_parameter('payload_bytes').value)
        seed            = int(self.get_parameter('constraint_seed').value)
        gen_period_s    = float(self.get_parameter('constraint_period_s').value)
        trust_threshold = float(self.get_parameter('trust_threshold').value)

        if not session_dir:
            session_dir = os.path.expanduser('~/hardware_runs/default_session')
        session_dir = Path(session_dir)
        session_dir.mkdir(parents=True, exist_ok=True)

        if pyserial is None:
            self.get_logger().error('pyserial not installed: pip install pyserial')
            raise SystemExit(1)

        self.get_logger().info(f'opening serial: {serial_dev}@{serial_baud}')
        port = pyserial.Serial(serial_dev, serial_baud, timeout=2.0)

        self.sender = TSeqHexSerialSender(port=port, session_dir=session_dir)
        self.scheduler = Scheduler(
            policy,
            DutyCycleBudget(),
            trust_threshold=trust_threshold,
        )

        self.constraint_source = ConstraintSource(
            seed=seed,
            generation_period_s=gen_period_s,
        )

        # ---- Publishers -----------------------------------------------------
        self.scheduler_pub     = self.create_publisher(String, '/bacs/scheduler',               10)
        self.raw_pub           = self.create_publisher(String, '/bacs/serial/raw',              10)
        self.clock_offset_pub  = self.create_publisher(String, '/bacs/scheduler/clock_offset', 10)
        self.candidate_pub     = self.create_publisher(String, '/bacs/candidates',              10)

        # ---- State -----------------------------------------------------------
        self._policy = policy
        self._session_id = session_id
        self._payload_bytes = payload_bytes
        self._pool: list = []           # pending constraint candidates
        self._t0_ns = time.time_ns()

        # ---- Timers ----------------------------------------------------------
        # Constraint generation timer: runs faster than TX so the pool grows.
        self.gen_timer = self.create_timer(gen_period_s, self._generate)
        # TX timer: every tx_period_s, select and transmit one constraint.
        self.tx_timer = self.create_timer(tx_period_s, self._tick)

        self.get_logger().info(
            f'session_id={session_id} policy={policy} tx_period_s={tx_period_s} '
            f'gen_period_s={gen_period_s} seed={seed} ready'
        )

    def _generate(self) -> None:
        """Generate new constraint candidates and add them to the pool."""
        now_ns = time.monotonic_ns()
        new = self.constraint_source.maybe_generate(now_ns)
        for c in new:
            self._pool.append(c)
            # Publish every candidate so the bag captures what was available.
            msg = String()
            msg.data = (
                f'{{"event":"CANDIDATE","seq":{c.sequence},'
                f'"robot_i":"{c.robot_i}","robot_j":"{c.robot_j}",'
                f'"trust":{c.predicted_trust:.4f},'
                f'"info":{c.information_score:.4f},'
                f'"pair_count":{c.pair_constraints}}}'
            )
            self.candidate_pub.publish(msg)

    def _pick_one(self, now_ns: int):
        """Use the scheduler's ranking to pick the single best constraint."""
        if not self._pool:
            return None
        # Filter by trust threshold (same as scheduler.select()).
        eligible = [
            c for c in self._pool
            if c.predicted_trust >= self.scheduler.trust_threshold
        ]
        if not eligible:
            return None
        # Order by the scheduler's policy-specific scoring.
        ordered = self.scheduler._ordered(eligible, now_ns)
        return ordered[0] if ordered else None

    def _tick(self) -> None:
        """Select one constraint, transmit it, and remove it from the pool."""
        now_ns = time.monotonic_ns()
        chosen = self._pick_one(now_ns)

        elapsed_s = (time.time_ns() - self._t0_ns) / 1e9

        if chosen is None:
            self.get_logger().warning(
                f'[{elapsed_s:7.2f}s] no eligible constraint (pool={len(self._pool)})'
            )
            return

        # Record airtime for just this one constraint (not the whole pool).
        airtime = lora_airtime_s(chosen.payload_bytes)
        now_s = now_ns / 1e9
        if not self.scheduler.budget.can_send(now_s, airtime):
            self.get_logger().warning(
                f'[{elapsed_s:7.2f}s] duty-cycle budget exhausted, skipping seq={chosen.sequence}'
            )
            return
        self.scheduler.budget.record(now_s, airtime)

        # Serialize and transmit.
        payload = ConstraintSource.serialize(chosen)
        ack = self.sender.transmit(seq=chosen.sequence, payload=payload)

        if ack is None:
            self.get_logger().warning(
                f'[{elapsed_s:7.2f}s] seq={chosen.sequence}: no ack from MR'
            )
            return

        # Remove the transmitted constraint from the pool.
        self._pool = [c for c in self._pool if c.sequence != chosen.sequence]

        # Publish the scheduler decision.
        score = self.scheduler.score(chosen, now_ns)
        print(
            f'[{elapsed_s:7.2f}s] policy={self._policy} '
            f'seq={chosen.sequence} trust={chosen.predicted_trust:.3f} '
            f'info={chosen.information_score:.3f} score={score:.6f} '
            f'pool_remaining={len(self._pool)} '
            f'-> TX,{ack.mr_timestamp_ms},{ack.seq},{ack.field3}',
            flush=True,
        )

        msg = String()
        msg.data = (
            f'{{"event":"SELECT","session":"{self._session_id}",'
            f'"policy":"{self._policy}","seq":{ack.seq},'
            f'"mr_ts_ms":{ack.mr_timestamp_ms},'
            f'"trust":{chosen.predicted_trust:.4f},'
            f'"info":{chosen.information_score:.4f},'
            f'"score":{score:.6f},'
            f'"pool_remaining":{len(self._pool)}}}'
        )
        self.scheduler_pub.publish(msg)

        raw = String()
        raw.data = ack.raw
        self.raw_pub.publish(raw)

        co = String()
        co.data = f'{{"seq":{ack.seq},"mr_ts_ms":{ack.mr_timestamp_ms}}}'
        self.clock_offset_pub.publish(co)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BacsSchedulerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.sender.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
