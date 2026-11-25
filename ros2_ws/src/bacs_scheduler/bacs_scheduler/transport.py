"""Serial transport for the two-board LoRa link-test firmware.

The local ``TX_DONE`` event is not a delivery acknowledgement.  A transmission
is successful only when the remote board receives the data frame and returns an
over-the-air ``ACK_RX`` frame.  Device timestamps are unsynchronised uptime
values; all host durations use ``time.monotonic_ns()``.
"""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import IO, Optional, Protocol


class Rylr998Transport(Protocol):
    def transmit(self, payload: bytes) -> None: ...


SERIAL_LOG_FIELDS = (
    "host_wall_ns",
    "host_monotonic_ns",
    "direction",
    "device_uptime_ms",
    "seq",
    "rssi_dbm",
    "snr_db",
    "tx_duration_us",
    "payload_hex",
    "raw_line",
)

TIMING_LOG_FIELDS = (
    "seq",
    "host_handoff_wall_ns",
    "host_handoff_monotonic_ns",
    "host_tx_done_wall_ns",
    "host_tx_done_monotonic_ns",
    "host_ack_wall_ns",
    "host_ack_monotonic_ns",
    "device_tx_done_uptime_ms",
    "device_tx_duration_us",
    "delivery_status",
    "host_rtt_ms",
)


@dataclass
class SerialLine:
    """One parsed line emitted by the board firmware."""

    direction: str
    device_uptime_ms: int
    seq: int
    raw: str
    rssi_dbm: str = ""
    snr_db: str = ""
    tx_duration_us: Optional[int] = None

    # Compatibility alias for callers that displayed the old field name.
    @property
    def mr_timestamp_ms(self) -> int:
        return self.device_uptime_ms


@dataclass
class TSeqHexSerialSender:
    """Send numbered frames and wait for a genuine remote delivery ACK."""

    port: object
    session_dir: Path
    tx_timeout_s: float = 3.0
    _serial_log_fh: IO = field(init=False, default=None, repr=False)
    _serial_log_writer: csv.DictWriter = field(init=False, default=None, repr=False)
    _timing_log_fh: IO = field(init=False, default=None, repr=False)
    _timing_log_writer: csv.DictWriter = field(init=False, default=None, repr=False)

    def __post_init__(self) -> None:
        self.session_dir = Path(self.session_dir)
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self._serial_log_fh = open(
            self.session_dir / "serial_log.csv", "a", newline="", encoding="utf-8"
        )
        self._serial_log_writer = csv.DictWriter(
            self._serial_log_fh, fieldnames=SERIAL_LOG_FIELDS
        )
        if self._serial_log_fh.tell() == 0:
            self._serial_log_writer.writeheader()

        self._timing_log_fh = open(
            self.session_dir / "timing_log.csv", "a", newline="", encoding="utf-8"
        )
        self._timing_log_writer = csv.DictWriter(
            self._timing_log_fh, fieldnames=TIMING_LOG_FIELDS
        )
        if self._timing_log_fh.tell() == 0:
            self._timing_log_writer.writeheader()

    def transmit(self, seq: int, payload: bytes) -> Optional[SerialLine]:
        """Transmit and return only after the remote board's ACK is received.

        ``payload`` is the 48-byte application section.  The firmware prepends
        the full four-byte sequence number to produce the configured 52-byte
        on-air data frame.
        """
        if len(payload) != 48:
            raise ValueError(f"link-test payload must be 48 bytes, got {len(payload)}")

        hex_payload = payload.hex()
        line = f"T,{seq},{hex_payload}\n".encode("ascii")
        handoff_wall_ns = time.time_ns()
        handoff_mono_ns = time.monotonic_ns()
        self.port.write(line)
        self._log_serial(
            direction="HOST_TX",
            device_uptime_ms="",
            seq=seq,
            payload_hex=hex_payload,
            raw_line=line.decode("ascii").rstrip("\n"),
        )

        tx_done: Optional[SerialLine] = None
        tx_done_wall_ns = ""
        tx_done_mono_ns = ""
        deadline = time.monotonic() + self.tx_timeout_s
        while time.monotonic() < deadline:
            raw = self._readline()
            if raw is None:
                continue
            parsed = self._parse_mr_line(raw)
            if parsed is None or parsed.seq != seq:
                continue

            if parsed.direction == "MR_TX_DONE":
                tx_done = parsed
                tx_done_wall_ns = time.time_ns()
                tx_done_mono_ns = time.monotonic_ns()
                continue

            if parsed.direction == "MR_DELIVERY_ACK":
                # Reject a stale/synthetic ACK that was not preceded by this
                # sequence's local TX_DONE event.
                if tx_done is None:
                    continue
                ack_wall_ns = time.time_ns()
                ack_mono_ns = time.monotonic_ns()
                self._record_timing(
                    seq=seq,
                    handoff_wall_ns=handoff_wall_ns,
                    handoff_mono_ns=handoff_mono_ns,
                    tx_done=tx_done,
                    tx_done_wall_ns=tx_done_wall_ns,
                    tx_done_mono_ns=tx_done_mono_ns,
                    ack_wall_ns=ack_wall_ns,
                    ack_mono_ns=ack_mono_ns,
                    status="DELIVERED",
                )
                return parsed

            if parsed.direction == "MR_ACK_TIMEOUT":
                self._record_timing(
                    seq=seq,
                    handoff_wall_ns=handoff_wall_ns,
                    handoff_mono_ns=handoff_mono_ns,
                    tx_done=tx_done,
                    tx_done_wall_ns=tx_done_wall_ns,
                    tx_done_mono_ns=tx_done_mono_ns,
                    ack_wall_ns="",
                    ack_mono_ns="",
                    status="ACK_TIMEOUT",
                )
                return None

        self._record_timing(
            seq=seq,
            handoff_wall_ns=handoff_wall_ns,
            handoff_mono_ns=handoff_mono_ns,
            tx_done=tx_done,
            tx_done_wall_ns=tx_done_wall_ns,
            tx_done_mono_ns=tx_done_mono_ns,
            ack_wall_ns="",
            ack_mono_ns="",
            status="HOST_TIMEOUT",
        )
        return None

    def drain(self, timeout_s: float = 0.05) -> list[SerialLine]:
        out: list[SerialLine] = []
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self._readline()
            if raw is None:
                continue
            parsed = self._parse_mr_line(raw)
            if parsed is not None:
                out.append(parsed)
        return out

    def close(self) -> None:
        for fh in (self._serial_log_fh, self._timing_log_fh):
            if fh is not None and not fh.closed:
                fh.flush()
                fh.close()

    def _readline(self) -> Optional[str]:
        try:
            data = self.port.readline()
        except Exception:
            return None
        if not data:
            return None
        if isinstance(data, bytes):
            text = data.decode("ascii", errors="replace")
        else:
            text = str(data)
        return text.rstrip("\r\n")

    def _parse_mr_line(self, raw: str) -> Optional[SerialLine]:
        parts = [part.strip() for part in raw.split(",")]
        if not parts:
            return None
        tag = parts[0].upper()
        try:
            if tag == "TX_DONE" and len(parts) == 4:
                parsed = SerialLine(
                    "MR_TX_DONE", int(parts[1]), int(parts[2]), raw,
                    tx_duration_us=int(parts[3]),
                )
            elif tag == "ACK_RX" and len(parts) == 5:
                parsed = SerialLine(
                    "MR_DELIVERY_ACK", int(parts[1]), int(parts[2]), raw,
                    rssi_dbm=parts[3], snr_db=parts[4],
                )
            elif tag == "ACK_TIMEOUT" and len(parts) == 3:
                parsed = SerialLine("MR_ACK_TIMEOUT", int(parts[1]), int(parts[2]), raw)
            elif tag == "RX" and len(parts) == 5:
                parsed = SerialLine(
                    "MR_RX", int(parts[1]), int(parts[2]), raw,
                    rssi_dbm=parts[3], snr_db=parts[4],
                )
            elif tag == "ACK_TX" and len(parts) == 4:
                parsed = SerialLine(
                    "MR_ACK_TX", int(parts[1]), int(parts[2]), raw,
                    tx_duration_us=int(parts[3]),
                )
            else:
                self._log_serial(
                    direction="MR_OTHER",
                    device_uptime_ms="",
                    seq="",
                    raw_line=raw,
                )
                return None
        except (ValueError, IndexError):
            self._log_serial(
                direction="MR_PARSE_ERROR",
                device_uptime_ms="",
                seq="",
                raw_line=raw,
            )
            return None

        self._log_serial(
            direction=parsed.direction,
            device_uptime_ms=parsed.device_uptime_ms,
            seq=parsed.seq,
            rssi_dbm=parsed.rssi_dbm,
            snr_db=parsed.snr_db,
            tx_duration_us=parsed.tx_duration_us if parsed.tx_duration_us is not None else "",
            raw_line=raw,
        )
        return parsed

    def _log_serial(
        self, *, direction: str, device_uptime_ms, seq,
        rssi_dbm="", snr_db="", tx_duration_us="", payload_hex="", raw_line: str,
    ) -> None:
        self._serial_log_writer.writerow({
            "host_wall_ns": time.time_ns(),
            "host_monotonic_ns": time.monotonic_ns(),
            "direction": direction,
            "device_uptime_ms": device_uptime_ms,
            "seq": seq,
            "rssi_dbm": rssi_dbm,
            "snr_db": snr_db,
            "tx_duration_us": tx_duration_us,
            "payload_hex": payload_hex,
            "raw_line": raw_line.replace("\r", "\\r").replace("\n", "\\n"),
        })
        self._serial_log_fh.flush()

    def _record_timing(
        self, *, seq: int, handoff_wall_ns: int, handoff_mono_ns: int,
        tx_done: Optional[SerialLine], tx_done_wall_ns, tx_done_mono_ns,
        ack_wall_ns, ack_mono_ns, status: str,
    ) -> None:
        rtt_ms = ""
        if ack_mono_ns != "":
            rtt_ms = (int(ack_mono_ns) - handoff_mono_ns) / 1_000_000
        self._timing_log_writer.writerow({
            "seq": seq,
            "host_handoff_wall_ns": handoff_wall_ns,
            "host_handoff_monotonic_ns": handoff_mono_ns,
            "host_tx_done_wall_ns": tx_done_wall_ns,
            "host_tx_done_monotonic_ns": tx_done_mono_ns,
            "host_ack_wall_ns": ack_wall_ns,
            "host_ack_monotonic_ns": ack_mono_ns,
            "device_tx_done_uptime_ms": tx_done.device_uptime_ms if tx_done else "",
            "device_tx_duration_us": tx_done.tx_duration_us if tx_done else "",
            "delivery_status": status,
            "host_rtt_ms": rtt_ms,
        })
        self._timing_log_fh.flush()


class _BytesPort:
    """Small in-memory serial port used by unit tests."""

    def __init__(self) -> None:
        self.tx = bytearray()
        self._rx: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.tx.extend(data)
        return len(data)

    def readline(self) -> bytes:
        if not self._rx:
            return b""
        return self._rx.pop(0)

    def feed(self, line: str) -> None:
        self._rx.append(line.encode("ascii"))


__all__ = [
    "Rylr998Transport",
    "TSeqHexSerialSender",
    "SerialLine",
    "SERIAL_LOG_FIELDS",
    "TIMING_LOG_FIELDS",
    "_BytesPort",
]
