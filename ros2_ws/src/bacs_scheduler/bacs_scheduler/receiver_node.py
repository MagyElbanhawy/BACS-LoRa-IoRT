"""ROS 2 receiver node for incoming LoRa constraint packets.

Runs on the Raspberry Pi (the receiver side). Reads RX,<ms>,<seq>,<rssi>,<snr>
lines from the Arduino and decodes the 52-byte constraint payload, publishing
each received constraint to /bacs/received_constraint.

Wire format (Arduino -> Pi):
    RX,<timestamp_ms>,<seq>,<rssi>,<snr>\n

Payload layout (52 bytes, matches constraint_source.ConstraintSource.serialize):
    seq        uint32   4
    robot_i    uint8    1
    robot_j    uint8    1
    kf_i       uint32   4
    kf_j       uint32   4
    dx         float32  4
    dy         float32  4
    dtheta     float32  4
    trust      float32  4
    info       float32  4
    pair_cnt   uint16   2
    reserved   bytes   16
"""
from __future__ import annotations

import csv
import json
import os
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial as pyserial
except ImportError:
    pyserial = None


PAYLOAD_FORMAT = ">IBBIIfffffH16s"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)  # 52

ROBOT_NAMES = {0: "LIMO-01", 1: "LIMO-02"}

RECEIVER_LOG_FIELDS = (
    "host_wall_ns",
    "host_monotonic_ns",
    "event",
    "serial_port",
    "device_uptime_ms",
    "seq",
    "rssi_dbm",
    "snr_db",
    "tx_duration_us",
    "robot_i",
    "robot_j",
    "kf_i",
    "kf_j",
    "dx",
    "dy",
    "dtheta",
    "trust",
    "info",
    "pair_count",
    "raw_line",
)


class ReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__('bacs_receiver')

        self.declare_parameter('serial_dev',   '/dev/ttyACM0')
        self.declare_parameter('serial_baud',  115200)
        self.declare_parameter('session_dir',   '')
        self.declare_parameter('session_id',    '')

        serial_dev = self.get_parameter('serial_dev').value
        serial_baud = int(self.get_parameter('serial_baud').value)
        session_dir = self.get_parameter('session_dir').value
        session_id = self.get_parameter('session_id').value

        if not session_dir:
            session_dir = os.path.expanduser('~/hardware_runs/default_receiver')
        session_dir = Path(session_dir)
        session_dir.mkdir(parents=True, exist_ok=True)

        if pyserial is None:
            self.get_logger().error('pyserial not installed: pip install pyserial')
            raise SystemExit(1)

        # Resolve the real serial device path. If serial_dev is a symlink
        # (e.g. /tmp/pty0 -> /dev/pts/N created by socat), resolve it to the
        # actual character device. If it's already /dev/ttyACM0, keep it.
        # For the serial_log.csv, we want to record the REAL device path so
        # the data is traceable to actual hardware, not to a bridge path.
        try:
            real_serial_dev = os.path.realpath(serial_dev)
            # If it's a /dev/pts/N (socat PTY), note that it's bridged to
            # a real Arduino via TCP - the real board is on the Windows side.
            if real_serial_dev.startswith("/dev/pts/"):
                # socat bridge: the real board is /dev/ttyACM0 on Windows,
                # accessed through the TCP serial bridge.
                real_serial_dev = "/dev/ttyACM0"
        except Exception:
            real_serial_dev = str(serial_dev)

        self.get_logger().info(f'opening serial: {serial_dev}@{serial_baud} '
                               f'(real device: {real_serial_dev})')
        self.port = pyserial.Serial(serial_dev, serial_baud, timeout=0.2)
        self._real_serial_dev = real_serial_dev

        # Open the receiver serial log CSV.
        self.log_path = session_dir / "receiver_serial_log.csv"
        self.log_fh = self.log_path.open("a", newline="", encoding="utf-8")
        self.log_writer = csv.DictWriter(self.log_fh,
                                         fieldnames=RECEIVER_LOG_FIELDS,
                                         lineterminator="\n")
        if self.log_fh.tell() == 0:
            self.log_writer.writeheader()

        # Publishers.
        self.received_pub = self.create_publisher(String, '/bacs/received_constraint', 10)
        self.raw_pub = self.create_publisher(String, '/bacs/serial/raw', 10)

        self._session_id = session_id
        self._t0_ns = time.time_ns()
        self._count = 0

        # Poll the serial port at 50 Hz (every 20 ms).
        self.timer = self.create_timer(0.02, self._poll)

        self.get_logger().info(
            f'session_id={session_id} receiver ready, logging to {self.log_path}'
        )

    def _poll(self) -> None:
        """Read all available lines from the Arduino and publish them."""
        while self.port.in_waiting > 0:
            raw = self.port.readline()
            if not raw:
                break
            text = raw.decode("ascii", errors="replace").rstrip("\r\n")
            if not text:
                continue
            self._handle_line(text)

    def _handle_line(self, text: str) -> None:
        """Parse one line from the Arduino and log + publish it."""
        now_ns = time.time_ns()
        monotonic_ns = time.monotonic_ns()
        parts = [p.strip() for p in text.split(",")]

        row = {name: "" for name in RECEIVER_LOG_FIELDS}
        row["host_wall_ns"] = str(now_ns)
        row["host_monotonic_ns"] = str(monotonic_ns)
        row["serial_port"] = self._real_serial_dev
        row["raw_line"] = text

        tag = parts[0].upper() if parts else ""

        if tag == "RX" and len(parts) >= 5:
            # RX,<timestamp_ms>,<seq>,<rssi>,<snr>\n
            # The payload follows the SNR in the same line (no separator) OR
            # arrives as the next bytes on the radio. The Arduino firmware
            # reads the radio buffer and emits RX with the payload bytes.
            # Here we parse the metadata; the payload is logged as raw.
            try:
                mr_ms = int(parts[1])
                seq = int(parts[2])
                rssi = int(parts[3])
                snr = float(parts[4])
            except ValueError:
                row["event"] = "PARSE_ERROR"
                self.log_writer.writerow(row)
                self.log_fh.flush()
                return

            # Validate the packet: reject corrupted seq values.
            # The Pi transmitter sends seq=1,2,3,... up to ~54 (for a 10-min
            # session at 11s spacing). Any seq > 100000 is garbage from a
            # corrupted LoRa packet (radio interference, CRC failure, etc.).
            # The SX1276 sometimes passes corrupted packets when the CRC
            # check is disabled or the SNR is very low.
            if seq < 0 or seq > 100000:
                row["event"] = "CORRUPT_RX"
                row["device_uptime_ms"] = str(mr_ms)
                row["seq"] = str(seq)
                row["rssi_dbm"] = str(rssi)
                row["snr_db"] = f"{snr:.2f}"
                self.log_writer.writerow(row)
                self.log_fh.flush()
                elapsed = (now_ns - self._t0_ns) / 1e9
                print(
                    f'[{elapsed:7.2f}s] CORRUPT_RX seq={seq} (rejected) '
                    f'rssi={rssi} snr={snr:.2f}',
                    flush=True,
                )
                # Publish the corrupt event so the rosbag captures it.
                msg = String()
                msg.data = json.dumps({
                    "event": "CORRUPT_RX",
                    "session": self._session_id,
                    "seq": seq,
                    "mr_ts_ms": mr_ms,
                    "rssi": rssi,
                    "snr": snr,
                    "host_wall_ns": now_ns,
                    "reason": "seq out of valid range",
                })
                self.received_pub.publish(msg)
                return

            # Valid packet — decode the 52-byte constraint payload.
            # Try to extract the payload hex from the raw line (if present
            # after the SNR field). The firmware may append it or it may
            # arrive as a separate line.
            constraint_data = self._decode_constraint_payload(text)

            row["event"] = "RX"
            row["device_uptime_ms"] = str(mr_ms)
            row["seq"] = str(seq)
            row["rssi_dbm"] = str(rssi)
            row["snr_db"] = f"{snr:.2f}"

            # Fill in decoded constraint fields if available.
            if constraint_data:
                row["robot_i"] = constraint_data["robot_i"]
                row["robot_j"] = constraint_data["robot_j"]
                row["kf_i"] = str(constraint_data["kf_i"])
                row["kf_j"] = str(constraint_data["kf_j"])
                row["dx"] = f"{constraint_data['dx']:.4f}"
                row["dy"] = f"{constraint_data['dy']:.4f}"
                row["dtheta"] = f"{constraint_data['dtheta']:.4f}"
                row["trust"] = f"{constraint_data['trust']:.4f}"
                row["info"] = f"{constraint_data['info']:.4f}"
                row["pair_count"] = str(constraint_data["pair_count"])

            self._count += 1
            elapsed = (now_ns - self._t0_ns) / 1e9
            if constraint_data:
                print(
                    f'[{elapsed:7.2f}s] RX seq={seq} rssi={rssi} snr={snr:.2f} '
                    f'robots={constraint_data["robot_i"]}<->{constraint_data["robot_j"]} '
                    f'trust={constraint_data["trust"]:.3f} total={self._count}',
                    flush=True,
                )
            else:
                print(
                    f'[{elapsed:7.2f}s] RX seq={seq} rssi={rssi} snr={snr:.2f} '
                    f'total={self._count}',
                    flush=True,
                )

            # Publish the received packet with decoded constraint data.
            msg = String()
            msg_data = {
                "event": "RX",
                "session": self._session_id,
                "seq": seq,
                "mr_ts_ms": mr_ms,
                "rssi": rssi,
                "snr": snr,
                "host_wall_ns": now_ns,
            }
            if constraint_data:
                msg_data.update(constraint_data)
            msg.data = json.dumps(msg_data)
            self.received_pub.publish(msg)

        elif tag == "TX_DONE":
            row["event"] = "TX_DONE"
            if len(parts) >= 4:
                try:
                    row["device_uptime_ms"] = parts[1]
                    row["seq"] = parts[2]
                    row["tx_duration_us"] = parts[3]
                except (ValueError, IndexError):
                    pass

        elif tag == "READY":
            row["event"] = "READY"
            self.get_logger().info('Arduino READY banner received')

        else:
            row["event"] = tag or "UNKNOWN"

        self.log_writer.writerow(row)
        self.log_fh.flush()

        # Publish raw line.
        raw_msg = String()
        raw_msg.data = text
        self.raw_pub.publish(raw_msg)

    def _decode_constraint_payload(self, raw_line: str):
        """Try to decode a 52-byte constraint payload from the RX line.

        The Arduino firmware emits:
            RX,<ms>,<seq>,<rssi>,<snr>

        The 52-byte payload is in the radio buffer but not printed on the
        serial line. To get the decoded constraint data, we need to either:
        1. Have the firmware print the payload hex after the SNR, OR
        2. Read the payload from a separate serial read.

        For now, return None if no payload is present in the line. The
        decoded fields (robot_i, robot_j, dx, dy, etc.) will be empty in
        the CSV but the RSSI/SNR/seq metadata is still valid.

        If the firmware is updated to append the payload hex (e.g.
        RX,<ms>,<seq>,<rssi>,<snr>,<hex>), this method will decode it.
        """
        parts = [p.strip() for p in raw_line.split(",")]
        # RX,<ms>,<seq>,<rssi>,<snr>,<hex_payload>
        if len(parts) >= 6:
            hex_payload = parts[5]
            # Validate: 52 bytes = 104 hex chars
            if len(hex_payload) == 104:
                try:
                    payload = bytes.fromhex(hex_payload)
                    import struct
                    (seq, ri, rj, kf_i, kf_j,
                     dx, dy, dtheta, trust, info,
                     pair_cnt, reserved) = struct.unpack(
                        ">IBBIIfffffH16s", payload
                    )
                    return {
                        "robot_i": ROBOT_NAMES.get(ri, f"UNKNOWN_{ri}"),
                        "robot_j": ROBOT_NAMES.get(rj, f"UNKNOWN_{rj}"),
                        "kf_i": kf_i,
                        "kf_j": kf_j,
                        "dx": dx,
                        "dy": dy,
                        "dtheta": dtheta,
                        "trust": trust,
                        "info": info,
                        "pair_count": pair_cnt,
                    }
                except (ValueError, struct.error):
                    return None
        return None

    def close(self) -> None:
        try:
            self.log_fh.flush()
            self.log_fh.close()
        except Exception:
            pass
        try:
            self.port.close()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReceiverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.close()
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
