#!/usr/bin/env bash
#
# run_12_sessions_rx.sh - Runs 12 BACS scheduling sessions on the receiver.
#
# Runs 4 policies (FIFO, BACS, BACS+, RANDOM) x 3 rounds = 12 sessions.
# Each session is 10 minutes (600s). Sessions are interleaved to match
# the transmitter.
#
# Session numbering: HWS-BACS-001 through HWS-BACS-012
#   001-003: FIFO    round 1, 2, 3
#   004-006: BACS    round 1, 2, 3
#   007-009: BACS+   round 1, 2, 3
#   010-012: RANDOM  round 1, 2, 3
#
# Output: ~/hardware_runs/HWS-BACS-NNN/ for each session
#
# Usage:
#   ./run_12_sessions_rx.sh                 # full 12-session sweep
#   ./run_12_sessions_rx.sh --dry-run       # print the plan only
#   ./run_12_sessions_rx.sh --duration 600   # override duration
#   ./run_12_sessions_rx.sh --serial-dev /tmp/pty0
#
set -euo pipefail

DURATION_S=600
SERIAL_DEV="${BACS_SERIAL_DEV:-/tmp/pty0}"
SERIAL_BAUD=115200
ROBOT_NAME="LIMO-02"
DRY_RUN=0
RUNS_ROOT="${HOME}/hardware_runs"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RECEIVER_SCRIPT="${SCRIPT_DIR}/record_receiver.sh"

# Resolve real serial device for the manifest.
REAL_SERIAL_DEV="/dev/ttyACM0"
if [[ "${SERIAL_DEV}" == "/dev/ttyACM"* ]]; then
  REAL_SERIAL_DEV="${SERIAL_DEV}"
fi

# Policy -> session number mapping (interleaved for fairness)
POLICIES=("FIFO" "BACS" "BACS+" "RANDOM")

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)        DRY_RUN=1; shift ;;
    --duration)       DURATION_S="$2"; shift 2 ;;
    --serial-dev)     SERIAL_DEV="$2"; shift 2 ;;
    --serial-baud)    SERIAL_BAUD="$2"; shift 2 ;;
    --robot)          ROBOT_NAME="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

# Re-resolve real device.
if [[ "${SERIAL_DEV}" == "/dev/ttyACM"* ]]; then
  REAL_SERIAL_DEV="${SERIAL_DEV}"
fi

log() { printf '[rx-sweep %s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >&2; }

log "BACS-ROS2-SLAM 12-session receiver sweep"
log "  policies:   ${POLICIES[*]}"
log "  rounds:     3"
log "  total:      12 sessions"
log "  duration:   ${DURATION_S}s per session"
log "  serial:     ${SERIAL_DEV}@${SERIAL_BAUD} (real: ${REAL_SERIAL_DEV})"
log "  robot:      ${ROBOT_NAME}"
log "  output:     ${RUNS_ROOT}/HWS-BACS-001..012"
if (( DRY_RUN )); then
  log "  mode:       DRY-RUN"
fi
echo "---" >&2

session_num=0
for round in 1 2 3; do
  for policy in "${POLICIES[@]}"; do
    session_num=$((session_num + 1))
    printf -v sid "HWS-BACS-%03d" "${session_num}"
    out_dir="${RUNS_ROOT}/${sid}"

    log "=== session ${sid} | policy=${policy} | round ${round} | ${DURATION_S}s ==="

    if (( DRY_RUN )); then
      log "  [dry-run] would run: ${RECEIVER_SCRIPT} --serial-dev ${SERIAL_DEV} --duration ${DURATION_S} --session-id ${sid}"
      continue
    fi

    # Run the receiver for this session.
    "${RECEIVER_SCRIPT}" \
        --serial-dev "${SERIAL_DEV}" \
        --serial-baud "${SERIAL_BAUD}" \
        --duration "${DURATION_S}" \
        --session-id "${sid}" \
        --root "${RUNS_ROOT}" \
        --robot "${ROBOT_NAME}"

    log "  ${sid} complete. Cooling down 10s..."
    sleep 10
  done
done

log "DONE. Captured ${session_num} sessions under ${RUNS_ROOT}/."

# Print summary.
log ""
log "=== Session Summary ==="
for d in "${RUNS_ROOT}"/HWS-BACS-0*; do
  [[ -f "${d}/manifest.json" ]] || continue
  sid=$(basename "${d}")
  # Count RX events.
  rx_count=0
  if [[ -f "${d}/receiver_serial_log.csv" ]]; then
    rx_count=$(($(wc -l < "${d}/receiver_serial_log.csv") - 1))
  fi
  bag_size=0
  if [[ -d "${d}/rosbag.db3" ]]; then
    bag_size=$(du -m "${d}/rosbag.db3" 2>/dev/null | cut -f1 || echo 0)
  fi
  log "  ${sid}: rx_events=${rx_count} rosbag=${bag_size}MB"
done
