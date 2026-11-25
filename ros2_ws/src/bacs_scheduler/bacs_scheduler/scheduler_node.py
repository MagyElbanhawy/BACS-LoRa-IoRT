"""ROS 2 node for a numbered, duty-cycle-safe LoRa link test.

This node intentionally does not claim to execute FIFO/BACS/BACS+/RANDOM.
Those policies require a queue of real constraint candidates and their trust,
information and observability fields.  This link test sends deterministic,
numbered frames and records genuine remote ACK, RSSI and SNR evidence.
"""

from __future__ import annotations

import json
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

from .scheduler import lora_airtime_s
from .transport import TSeqHexSerialSender


class SchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__("bacs_link_test")

        self.declare_parameter("policy", "LINK_TEST")
        self.declare_parameter("serial_dev", "/dev/ttyACM0")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("session_dir", "")
        self.declare_parameter("session_id", "")
        self.declare_parameter("tx_period_s", 11.0)
        self.declare_parameter("payload_bytes", 52)
        self.declare_parameter("duty_cycle", 0.01)

        policy = str(self.get_parameter("policy").value).upper()
        serial_dev = self.get_parameter("serial_dev").value
        serial_baud = int(self.get_parameter("serial_baud").value)
        session_dir = self.get_parameter("session_dir").value
        session_id = self.get_parameter("session_id").value
        tx_period_s = float(self.get_parameter("tx_period_s").value)
        payload_bytes = int(self.get_parameter("payload_bytes").value)
        duty_cycle = float(self.get_parameter("duty_cycle").value)

        if policy != "LINK_TEST":
            raise ValueError(
                "this node generates numbered link-test frames, not policy "
                "candidates; use policy=LINK_TEST"
            )
        if payload_bytes != 52:
            raise ValueError("LINK_TEST_V2 firmware requires a 52-byte on-air frame")
        if not 0 < duty_cycle <= 1:
            raise ValueError("duty_cycle must be in (0, 1]")

        airtime_s = lora_airtime_s(payload_bytes)
        min_period_s = airtime_s / duty_cycle
        if tx_period_s + 1e-9 < min_period_s:
            raise ValueError(
                f"tx_period_s={tx_period_s:.3f} violates the {duty_cycle:.2%} "
                f"limit: {payload_bytes} bytes require at least "
                f"{min_period_s:.3f} s between transmissions"
            )

        if not session_dir:
            session_dir = os.path.expanduser("~/hardware_runs/default_link_test")
        session_dir = Path(session_dir)
        session_dir.mkdir(parents=True, exist_ok=True)

        if pyserial is None:
            raise RuntimeError("pyserial not installed: pip install pyserial")

        self.get_logger().info(f"opening serial: {serial_dev}@{serial_baud}")
        port = pyserial.Serial(serial_dev, serial_baud, timeout=0.1)
        self.sender = TSeqHexSerialSender(port=port, session_dir=session_dir)

        self.scheduler_pub = self.create_publisher(String, "/bacs/scheduler", 10)
        self.raw_pub = self.create_publisher(String, "/bacs/serial/raw", 10)

        self._seq = 0
        self._session_id = session_id
        self._airtime_s = airtime_s
        self._start_mono_ns = time.monotonic_ns()
        self.timer = self.create_timer(tx_period_s, self._tick)

        self.get_logger().info(
            f"session_id={session_id} type=LINK_TEST numbered_payloads=true "
            f"tx_period_s={tx_period_s:.3f} airtime_s={airtime_s:.6f} "
            f"duty_cycle={airtime_s / tx_period_s:.4%} ready"
        )

    @staticmethod
    def _numbered_payload(seq: int) -> bytes:
        """Return 48 deterministic bytes visibly labelled with the sequence."""
        label = f"LINK-{seq:010d}|".encode("ascii")
        return (label * ((48 + len(label) - 1) // len(label)))[:48]

    def _tick(self) -> None:
        self._seq += 1
        ack = self.sender.transmit(
            seq=self._seq, payload=self._numbered_payload(self._seq)
        )
        elapsed_s = (time.monotonic_ns() - self._start_mono_ns) / 1e9

        if ack is None:
            self.get_logger().warning(
                f"[{elapsed_s:7.2f}s] label={self._seq:06d}: no remote ACK"
            )
            return

        print(
            f"[{elapsed_s:7.2f}s] label={self._seq:06d} DELIVERED "
            f"ack_rssi={ack.rssi_dbm} ack_snr={ack.snr_db}",
            flush=True,
        )

        event = {
            "session": self._session_id,
            "type": "LINK_TEST",
            "label": self._seq,
            "status": "DELIVERED",
            "ack_device_uptime_ms": ack.device_uptime_ms,
            "ack_rssi_dbm": float(ack.rssi_dbm),
            "ack_snr_db": float(ack.snr_db),
            "configured_airtime_s": self._airtime_s,
        }
        msg = String()
        msg.data = json.dumps(event, separators=(",", ":"))
        self.scheduler_pub.publish(msg)

        raw = String()
        raw.data = ack.raw
        self.raw_pub.publish(raw)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sender.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
