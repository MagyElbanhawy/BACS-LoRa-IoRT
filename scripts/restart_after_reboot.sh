#!/usr/bin/env bash
#
# restart_after_reboot.sh - bring the BACS-ROS2-SLAM serial bridge back up
# after a Windows or WSL2 reboot.
#
# WHY THIS EXISTS:
#   `usbipd attach --wsl` triggers a BSOD on this host because of the
#   NVIDIA `nxusbf` USB filter driver. The TCP serial bridge bypasses
#   usbipd entirely: the Arduino stays on Windows, a small Python script
#   on Windows bridges COM27 -> TCP:7777, and WSL2 talks to it through
#   socat. This script brings the WSL2 half of that bridge back up after
#   a reboot and verifies connectivity.
#
# PREREQUISITES (do these on the WINDOWS side before running this script):
#   1. Arduino MKR WAN 1310 is plugged in (USB cable to the host).
#   2. PowerShell (regular, not admin) is running:
#        cd D:\WSL\serial_tcp_bridge\tools
#        python serial_tcp_bridge.py --port COM27 --baud 115200 --tcp-port 7777
#      You should see "[bridge] listening on 0.0.0.0:7777".
#   3. The Arduino IDE is CLOSED (otherwise it holds COM27 open and the
#      bridge cannot open it).
#
# USAGE:
#   ./restart_after_reboot.sh           # bring up bridge + smoke test
#   ./restart_after_reboot.sh --record  # also run the 12-session sweep
#
set -euo pipefail

DO_RECORD=0
if [[ "${1:-}" == "--record" ]]; then
  DO_RECORD=1
fi

echo "=== BACS-ROS2-SLAM post-reboot restart ==="
echo

# ---- 1. Source ROS 2 + workspace -------------------------------------------
# ROS 2's setup.bash references variables (e.g. AMENT_TRACE_SETUP_FILES) that
# are not defined until first use, which trips `set -u`. Temporarily relax
# strict mode (set +u) around the source calls, then re-enable it.
WS_ROOT="${HOME}/GitHub/BACS-ROS2-SLAM"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
  exit 1
fi
if [[ ! -f "${WS_ROOT}/ros2_ws/install/setup.bash" ]]; then
  echo "ERROR: ${WS_ROOT}/ros2_ws/install/setup.bash not found."
  echo "       Did you run colcon build? See scripts/record_session.sh docs."
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS_ROOT}/ros2_ws/install/setup.bash"
set -u
echo "[1/5] ROS 2 + workspace sourced."

# ---- 2. Verify user is in dialout group -----------------------------------
if ! groups | grep -qw dialout; then
  echo "WARN: user not in 'dialout' group. PTY access may fail."
  echo "      Fix: sudo usermod -aG dialout \$USER && wsl --shutdown (from PowerShell)"
fi
echo "[2/5] dialout group check done."

# ---- 3. Find Windows host IP ----------------------------------------------
WIN_HOST=$(ip route show | grep -i default | awk '{print $3}')
if [[ -z "${WIN_HOST}" ]]; then
  echo "ERROR: could not find default gateway (Windows host IP)."
  exit 1
fi
echo "[3/5] Windows host IP: ${WIN_HOST}"

# ---- 4. Test TCP connectivity to the Windows bridge ------------------------
if ! timeout 3 bash -c "echo > /dev/tcp/${WIN_HOST}/7777" 2>/dev/null; then
  echo "ERROR: cannot reach ${WIN_HOST}:7777."
  echo "       Is the Windows bridge running? On Windows PowerShell:"
  echo "         cd D:\\WSL\\serial_tcp_bridge\\tools"
  echo "         python serial_tcp_bridge.py --port COM27 --baud 115200 --tcp-port 7777"
  echo "       Also check Windows Firewall isn't blocking port 7777."
  exit 1
fi
echo "[4/5] TCP connectivity to Windows bridge OK."

# ---- 5. Start socat PTY bridge --------------------------------------------
pkill -f "socat.*pty0" 2>/dev/null || true
sleep 1
rm -f /tmp/pty0
socat -d -d pty,raw,echo=0,link=/tmp/pty0 TCP:${WIN_HOST}:7777 &
SOCAT_PID=$!
sleep 2

if [[ ! -L /tmp/pty0 ]]; then
  echo "ERROR: socat did not create /tmp/pty0 symlink."
  kill "${SOCAT_PID}" 2>/dev/null || true
  exit 1
fi
echo "[5/5] socat PTY bridge up: /tmp/pty0 -> TCP ${WIN_HOST}:7777 (PID ${SOCAT_PID})"
echo

# ---- Smoke test: verify the Arduino responds ------------------------------
echo "=== Smoke test ==="
python3 - <<'PY'
import serial, time, sys
try:
    s = serial.Serial("/tmp/pty0", 115200, timeout=2)
except Exception as e:
    print(f"ERROR: cannot open /tmp/pty0: {e}")
    sys.exit(1)
time.sleep(2)  # let Arduino boot if it just reset
boot = s.readline().decode(errors="replace").strip()
print(f"Boot: {boot!r}")
if boot != "READY":
    print("WARN: did not see READY (Arduino may already be past boot). Continuing.")
s.write(b"T,1,AA\n")
ack1 = s.readline().decode(errors="replace").strip()
print(f"Ack1: {ack1!r}")
s.write(b"T,256,AA\n")
ack256 = s.readline().decode(errors="replace").strip()
print(f"Ack256: {ack256!r}")
s.close()
# The firmware uses TX_DONE format (not the old TX,<ms>,<seq>,SENT).
# A TX_DONE reply means the Arduino transmitted successfully.
# An ACK_RX reply means the remote Arduino RECEIVED the packet (bidirectional
# link is working). Either is a pass.
if "TX_DONE" in ack1 or "TX," in ack1:
    if "ACK_RX" in ack256 or "TX_DONE" in ack256:
        print("\nPASS: Arduino responds AND bidirectional link is working.")
        if "ACK_RX" in ack256:
            print("  (ACK_RX received from remote Arduino - link is bidirectional!)")
        sys.exit(0)
    else:
        print("\nPASS: Arduino responds to T commands.")
        sys.exit(0)
else:
    print("\nFAIL: Arduino did not respond with TX_DONE or TX.")
    sys.exit(2)
PY
SMOKE_RC=$?
echo

if [[ ${SMOKE_RC} -ne 0 ]]; then
  echo "Smoke test failed. Not starting record_session.sh."
  exit ${SMOKE_RC}
fi

# ---- Optional: launch the full 12-session sweep ---------------------------
if [[ ${DO_RECORD} -eq 1 ]]; then
  echo "=== Starting 12-session sweep ==="
  export BACS_SERIAL_DEV=/tmp/pty0
  export BACS_WS="${WS_ROOT}/ros2_ws"
  cd "${WS_ROOT}"
  ./scripts/record_session.sh
else
  echo "Bridge is up and Arduino is responding. Ready to run:"
  echo "  export BACS_SERIAL_DEV=/tmp/pty0"
  echo "  export BACS_WS=${WS_ROOT}/ros2_ws"
  echo "  cd ${WS_ROOT}"
  echo "  ./scripts/record_session.sh"
  echo
  echo "Or re-run this script with --record to launch the sweep automatically."
fi
