#!/usr/bin/env python3
"""MR radio serial endpoint for bench sessions.

Implements the radio side of the MR firmware wire protocol defined in
``hardware/firmware/MR/MR.ino``:

    Host -> MR :  T,<seq>,<hex_payload>\\n
    MR   -> Host:  TX,<timestamp_ms>,<seq>,SENT\\n
                  (and, with real radios, RX,<timestamp_ms>,<seq>,<rssi>,<snr>)

Start this process on the far end of the serial link BEFORE launching the
scheduler node or ``scripts/record_session.sh``. Example with a socat PTY
pair (/tmp/pty0 <-> /tmp/pty1):

    socat -d -d pty,raw,echo=0,link=/tmp/pty0 pty,raw,echo=0,link=/tmp/pty1 &
    python3 tools/mr_endpoint.py --port /tmp/pty1

Then point the scheduler at the other end:

    export BACS_SERIAL_DEV=/tmp/pty0
    ./scripts/record_session.sh --dry-run   # or a real capture

Like the firmware, the endpoint prints a ``READY`` banner on startup and
answers every well-formed ``T,<seq>,<hex>`` line with
``TX,<millis>,<seq>,SENT`` terminated by CRLF (Serial.println semantics).

``millis()`` semantics: the Arduino's ``millis()`` counts milliseconds since
boot. This endpoint therefore uses ``time.monotonic_ns()`` (boot-relative,
non-wall-clock) so its timestamps share the same basis as the firmware's,
which is what ``clock_offsets.txt`` analysis expects.

I/O note: PTY endpoints (socat ``link=`` paths) are opened with plain
blocking ``os.open`` plus raw termios settings. pyserial's O_NONBLOCK open
can hit EAGAIN on PTY write queues on some kernels, which delays or drops
the TX acknowledgements; the blocking raw fd has none of those stalls.
Real ``/dev/tty*`` devices still go through pyserial for proper baud setup.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time

try:
    import serial as pyserial
except ImportError:  # pyserial only needed for real /dev/tty* devices
    pyserial = None


class RawFdPort:
    """Blocking raw serial port for PTY paths (write/readline contract)."""

    def __init__(self, path: str) -> None:
        # O_NOCTTY + blocking (no O_NONBLOCK): a non-blocking PTY slave fd can
        # raise EAGAIN on write() even when the queue has room, stalling the
        # TX acknowledgement path by seconds on some kernels.
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        try:
            attrs = termios.tcgetattr(self.fd)
            # raw line discipline: no echo, no canonical buffering, no signals
            attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                          | termios.ISTRIP | termios.INLCR | termios.IGNCR
                          | termios.ICRNL | termios.IXON | termios.IXOFF
                          | termios.IXANY)
            attrs[1] &= ~termios.OPOST
            attrs[3] &= ~(termios.ECHO | termios.ECHOE | termios.ECHOK
                          | termios.ECHONL | termios.ICANON | termios.ISIG
                          | termios.IEXTEN)
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except termios.error:
            pass  # not a tty (e.g. redirected file) - keep plain fd semantics

    def write(self, data: bytes) -> int:
        return os.write(self.fd, data)

    def flush(self) -> None:
        pass  # blocking raw fd: os.write already reached the kernel

    def readline(self) -> bytes:
        """One '\\n'-terminated line; '' when nothing arrives within 0.5 s."""
        buf = bytearray()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if not r:
                continue
            chunk = os.read(self.fd, 4096)
            if not chunk:
                break
            buf.extend(chunk)
            if b"\n" in chunk:
                break
        return bytes(buf)

    def close(self) -> None:
        os.close(self.fd)


class PySerialPort:
    """Thin adapter exposing the same contract over a pyserial port."""

    def __init__(self, path: str, baud: int, read_timeout_s: float) -> None:
        if pyserial is None:
            raise SystemExit(
                f"ERROR: {path} needs pyserial (pip3 install pyserial); "
                "or use a PTY path, which needs no pyserial."
            )
        self._port = pyserial.Serial(path, baud, timeout=read_timeout_s)

    def write(self, data: bytes) -> int:
        return self._port.write(data)

    def flush(self) -> None:
        self._port.flush()

    def readline(self) -> bytes:
        return self._port.readline()

    def close(self) -> None:
        self._port.close()


def open_port(path: str, baud: int, read_timeout_s: float):
    """Raw fd for PTY/bench paths, pyserial for real /dev/tty* devices."""
    if os.path.basename(path).startswith("tty"):
        return PySerialPort(path, baud, read_timeout_s)
    return RawFdPort(path)


def millis_ms() -> int:
    """Arduino-style milliseconds since process start (monotonic clock)."""
    return time.monotonic_ns() // 1_000_000


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="MR radio serial endpoint (speaks the MR.ino wire protocol)."
    )
    ap.add_argument("--port", default="/tmp/pty1",
                    help="serial device to serve on (default: /tmp/pty1)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="line baud rate, used for /dev/tty* only "
                         "(default: 115200)")
    ap.add_argument("--payload-len", type=int, default=52,
                    help="expected payload length in bytes, for logging only "
                         "(default: 52, matches MR.ino PAYLOAD_LEN)")
    ap.add_argument("--read-timeout", type=float, default=0.5,
                    help="serial read timeout in seconds (default: 0.5)")
    return ap.parse_args()


def handle_t_command(line: str, payload_len: int) -> str | None:
    """Parse ``T,<seq>,<hex>`` and build the ``TX,<ms>,<seq>,SENT`` reply.

    Returns ``None`` if the line is malformed (the firmware would ignore it
    too). Mirrors MR.ino: seq goes into buf[0], the hex payload is zero-padded
    to PAYLOAD_LEN-1, and the ack is emitted the instant endPacket() returns.
    """
    parts = line.split(",", 2)
    if len(parts) != 3 or parts[0].strip().upper() != "T":
        return None

    seq_str, hex_str = parts[1].strip(), parts[2].strip()
    try:
        seq = int(seq_str)
    except ValueError:
        return None

    nbytes = len(hex_str) // 2          # odd trailing nibble ignored, as in firmware
    if nbytes < payload_len - 1:
        nbytes = payload_len - 1        # firmware zero-pads the buffer
    elif nbytes > payload_len - 1:
        nbytes = payload_len - 1        # firmware reads at most PAYLOAD_LEN-1

    t0 = millis_ms()
    return f"TX,{t0},{seq},SENT"


def main() -> int:
    args = parse_args()
    port = open_port(args.port, args.baud, args.read_timeout)

    t_start = millis_ms()

    def elapsed_s() -> str:
        return f"{(millis_ms() - t_start) / 1000.0:8.2f}s"

    # READY banner - identical to MR.ino setup(); the scheduler node and the
    # transport both tolerate/consume it as an unparsed line.
    port.write(b"READY\r\n")
    port.flush()
    print(f"[{elapsed_s()}] READY on {args.port} "
          f"(payload_len={args.payload_len}); Ctrl-C to stop.", flush=True)

    while True:
        try:
            raw = port.readline()
        except KeyboardInterrupt:
            break
        if not raw:
            continue

        line = raw.decode("ascii", errors="replace").rstrip("\r\n")
        if not line:
            continue

        reply = handle_t_command(line, args.payload_len)
        if reply is None:
            print(f"[{elapsed_s()}] ! unparsed line: {line!r}", flush=True)
            continue

        port.write((reply + "\r\n").encode("ascii"))
        port.flush()

        seq = reply.split(",")[2]
        nbytes = len(line.split(",", 2)[2].strip()) // 2
        print(f"[{elapsed_s()}] T,seq={seq} bytes={nbytes:2d} -> {reply}", flush=True)

    port.close()
    print(f"[{elapsed_s()}] closed.", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
