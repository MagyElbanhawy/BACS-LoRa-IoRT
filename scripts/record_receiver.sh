#!/usr/bin/env bash
#
# record_receiver.sh - Receiver-side capture script with full provenance.
#
# Runs on the receiver (ROS 2 Humble). Captures all incoming LoRa packets
# from the transmitter and records them to disk with complete traceability.
#
# For every session it produces, under ~/hardware_runs/<session_id>/:
#   * receiver_serial_log.csv  - all RX lines with RSSI/SNR/seq
#   * clock_offsets.txt         - per-packet host<->MR clock offsets
#   * rosbag.db3                - ROS 2 bag with /bacs/received_constraint
#   * manifest.json             - session metadata (serial_dev = /dev/ttyACM0)
#   * session_info.txt          - host info, kernel, ROS distro, git commit
#   * SHA256SUMS.txt            - SHA-256 of every file in the session
#   * chronyc_tracking.txt      - NTP/clock sync state at session start
#   * chronyc_tracking_end.txt  - NTP/clock sync state at session end
#
# Usage:
#   ./record_receiver.sh                       # 10-minute session
#   ./record_receiver.sh --duration 600        # override duration
#   ./record_receiver.sh --session-id HWS-RX-001
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DURATION_S="${BACS_SESSION_DURATION_S:-600}"
SERIAL_DEV="${BACS_SERIAL_DEV:-/tmp/pty0}"
SERIAL_BAUD="${BACS_SERIAL_BAUD:-115200}"
ROBOT_NAME="${BACS_ROBOT:-LIMO-02}"
RUNS_ROOT="${HOME}/hardware_runs"
SESSION_ID="HWS-RX-$(date -u +%Y%m%dT%H%M%SZ)"
DRY_RUN=0

# Resolve the real serial device for the manifest. If SERIAL_DEV is a socat
# bridge symlink, the real Arduino is /dev/ttyACM0 on the Windows side.
REAL_SERIAL_DEV="/dev/ttyACM0"
if [[ "${SERIAL_DEV}" == "/dev/ttyACM"* ]]; then
  REAL_SERIAL_DEV="${SERIAL_DEV}"
fi

# ROS 2 topics recorded into rosbag.db3.
ROBOT_TOPIC="${ROBOT_NAME//[^a-zA-Z0-9]/_}"
ROBOT_TOPIC_LC="${ROBOT_TOPIC,,}"
BAG_TOPICS=(
  "/scan/${ROBOT_TOPIC_LC}"
  "/odom/${ROBOT_TOPIC_LC}"
  "/tf"
  "/tf_static"
  "/bacs/received_constraint"
  "/bacs/serial/raw"
)

# ---------------------------------------------------------------------------
# Arg parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)        DRY_RUN=1; shift ;;
    --duration)       DURATION_S="$2"; shift 2 ;;
    --serial-dev)     SERIAL_DEV="$2"; shift 2 ;;
    --serial-baud)    SERIAL_BAUD="$2"; shift 2 ;;
    --session-id)     SESSION_ID="$2"; shift 2 ;;
    --root)           RUNS_ROOT="$2"; shift 2 ;;
    --robot)          ROBOT_NAME="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

# Re-resolve REAL_SERIAL_DEV in case --serial-dev was passed.
if [[ "${SERIAL_DEV}" == "/dev/ttyACM"* ]]; then
  REAL_SERIAL_DEV="${SERIAL_DEV}"
fi

mkdir -p "${RUNS_ROOT}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
log()  { printf '[receiver %s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >&2; }

verify_ros2() {
  if ! command -v ros2 >/dev/null 2>&1; then
    log "ERROR: ros2 not on PATH. Source your ROS 2 setup first."
    exit 1
  fi
}

verify_serial() {
  if [[ ! -e "${SERIAL_DEV}" ]]; then
    log "ERROR: serial device ${SERIAL_DEV} not present."
    log "       If using the TCP bridge, ensure socat is running."
    log "       If using /dev/ttyACM0, ensure the Arduino is plugged in."
    exit 1
  fi
  local holders
  if command -v fuser >/dev/null 2>&1; then
    holders="$(fuser "${SERIAL_DEV}" 2>/dev/null || true)"
  elif command -v lsof >/dev/null 2>&1; then
    holders="$(lsof -t "${SERIAL_DEV}" 2>/dev/null | tr '\n' ' ' || true)"
  else
    holders=""
  fi
  if [[ -n "${holders// /}" ]]; then
    local filtered=""
    for pid in ${holders}; do
      if ps -p "${pid}" -o args= 2>/dev/null | grep -q 'socat.*pty'; then
        log "    OK: socat bridge detected (PID ${pid})"
        continue
      fi
      filtered="${filtered} ${pid}"
    done
    holders="${filtered}"
  fi
  if [[ -n "${holders// /}" ]]; then
    log "ERROR: ${SERIAL_DEV} is already opened by non-socat PID(s):${holders}"
    log "       Kill them first:  kill${holders}"
    exit 1
  fi
}

# Capture system info for provenance.
write_session_info() {
  local out_dir="$1"
  local info_file="${out_dir}/session_info.txt"
  {
    echo "=== BACS-ROS2-SLAM Session Info ==="
    echo "session_id: ${SESSION_ID}"
    echo "role: receiver"
    echo "started_at: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "duration_s: ${DURATION_S}"
    echo ""
    echo "=== Host ==="
    echo "hostname: $(hostname)"
    echo "uname: $(uname -a)"
    echo ""
    echo "=== ROS 2 ==="
    echo "ros_distro: ${ROS_DISTRO:-unknown}"
    echo "ros_version: ${ROS_VERSION:-unknown}"
    echo "ament_prefix_path: ${AMENT_PREFIX_PATH:-unknown}"
    echo ""
    echo "=== Serial ==="
    echo "serial_dev: ${REAL_SERIAL_DEV}"
    echo "serial_baud: ${SERIAL_BAUD}"
    echo "robot: ${ROBOT_NAME}"
    echo ""
    echo "=== Git ==="
    cd "${HOME}/GitHub/BACS-ROS2-SLAM" 2>/dev/null && {
      echo "git_commit: $(git rev-parse HEAD 2>/dev/null || echo 'unknown')"
      echo "git_branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'unknown')"
      echo "git_dirty: $(git status --porcelain 2>/dev/null | wc -l) files modified"
      echo "git_remote: $(git remote get-url origin 2>/dev/null || echo 'unknown')"
    } || echo "git: not a git repo"
    echo ""
    echo "=== Disk ==="
    df -h "${out_dir}" | tail -1
    echo ""
    echo "=== Python ==="
    python3 --version 2>&1
    echo "pyserial: $(python3 -c 'import serial; print(serial.__version__)' 2>/dev/null || echo 'not installed')"
    echo ""
    echo "=== CPU ==="
    lscpu 2>/dev/null | grep -E 'Model name|CPU\(s\)|MHz' || echo "lscpu not available"
    echo ""
    echo "=== Memory ==="
    free -h 2>/dev/null | head -2 || echo "free not available"
  } > "${info_file}"
  log "    session_info.txt written"
}

# Capture chronyc tracking output for clock sync provenance.
write_chronyc_tracking() {
  local out_file="$1"
  local label="$2"
  {
    echo "=== chronyc tracking (${label}) ==="
    echo "captured_at: $(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
    echo ""
    if command -v chronyc >/dev/null 2>&1; then
      chronyc tracking 2>&1
      echo ""
      echo "=== chronyc sources ==="
      chronyc sources -v 2>&1
    elif command -v ntpq >/dev/null 2>&1; then
      echo "(chronyc not available, using ntpq)"
      ntpq -p 2>&1
    else
      echo "WARNING: neither chronyc nor ntpq is available."
      echo "The clock is not synchronized to an external reference."
      echo ""
      echo "=== /etc/chrony/chrony.conf ==="
      cat /etc/chrony/chrony.conf 2>/dev/null || echo "not found"
      echo ""
      echo "=== /etc/ntp.conf ==="
      cat /etc/ntp.conf 2>/dev/null || echo "not found"
    fi
    echo ""
    echo "=== System clock ==="
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
    echo "date_local: $(date)"
    timedatectl 2>/dev/null || echo "timedatectl not available"
  } > "${out_file}"
  log "    ${out_file##*/} written"
}

# Compute SHA-256 of every file in the session directory.
write_sha256sums() {
  local out_dir="$1"
  local sums_file="${out_dir}/SHA256SUMS.txt"
  (
    cd "${out_dir}"
    # Hash all files except SHA256SUMS.txt itself.
    find . -type f ! -name 'SHA256SUMS.txt' -print0 |
      xargs -0 sha256sum 2>/dev/null |
      sed 's| \./| |' > "${sums_file}"
  )
  log "    SHA256SUMS.txt written ($(wc -l < "${sums_file}") files)"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
  local out_dir="${RUNS_ROOT}/${SESSION_ID}"
  mkdir -p "${out_dir}"

  log "BACS-ROS2-SLAM receiver capture"
  log "  session:    ${SESSION_ID}"
  log "  duration:    ${DURATION_S}s"
  log "  serial:      ${SERIAL_DEV}@${SERIAL_BAUD} (real device: ${REAL_SERIAL_DEV})"
  log "  robot:       ${ROBOT_NAME}"
  log "  output:      ${out_dir}"
  log "  expected:    receiver_serial_log.csv, clock_offsets.txt, rosbag.db3,"
  log "               manifest.json, session_info.txt, SHA256SUMS.txt,"
  log "               chronyc_tracking.txt, chronyc_tracking_end.txt"
  echo "---" >&2

  if (( DRY_RUN )); then
    log "[dry-run] not touching hardware."
    exit 0
  fi

  verify_ros2
  verify_serial

  # ---- 0. Write session_info.txt and chronyc tracking BEFORE the run ----
  log "writing provenance files..."
  write_session_info "${out_dir}"
  write_chronyc_tracking "${out_dir}/chronyc_tracking.txt" "before run"

  # Write initial manifest.
  cat > "${out_dir}/manifest.json" <<EOF
{
  "session_id": "${SESSION_ID}",
  "role": "receiver",
  "duration_s": ${DURATION_S},
  "serial_dev": "${REAL_SERIAL_DEV}",
  "serial_baud": ${SERIAL_BAUD},
  "robot": "${ROBOT_NAME}",
  "started_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

  # ---- 1. Launch the receiver node ------------------------------------
  setsid ros2 launch bacs_scheduler receiver.launch.py \
      serial_dev:="${SERIAL_DEV}" \
      serial_baud:="${SERIAL_BAUD}" \
      session_dir:="${out_dir}" \
      session_id:="${SESSION_ID}" &
  local rx_pid=$!

  # ---- 2. Record the ROS 2 bag -----------------------------------------
  setsid ros2 bag record \
      -s sqlite3 \
      -o "${out_dir}/rosbag" \
      --include-hidden-topics \
      "${BAG_TOPICS[@]}" &
  local bag_pid=$!

  # ---- 3. Run for the requested duration -------------------------------
  log "listening for ${DURATION_S}s ..."
  sleep "${DURATION_S}"

  # ---- 4. Stop the bag and receiver -----------------------------------
  log "stopping bag and receiver..."
  kill -INT -- "-${bag_pid}"  2>/dev/null || true
  wait "${bag_pid}" 2>/dev/null || true
  kill -INT -- "-${rx_pid}"   2>/dev/null || true
  wait "${rx_pid}"  2>/dev/null || true
  sleep 1

  # ---- 5. Rename the bag to canonical name -----------------------------
  local db3_file
  db3_file="$(find "${out_dir}/rosbag" -name '*.db3' -print -quit 2>/dev/null || true)"
  if [[ -n "${db3_file}" && -f "${db3_file}" ]]; then
    mv "${db3_file}" "${out_dir}/rosbag.db3"
    cp "${out_dir}/rosbag/metadata.yaml" "${out_dir}/rosbag_metadata.yaml" 2>/dev/null || true
    rm -rf "${out_dir}/rosbag"
  fi

  # ---- 6. Write chronyc tracking AFTER the run -------------------------
  write_chronyc_tracking "${out_dir}/chronyc_tracking_end.txt" "after run"

  # ---- 7. Update manifest with completion status -----------------------
  local bag_size_mb
  bag_size_mb=$(du -m "${out_dir}/rosbag.db3" 2>/dev/null | cut -f1 || echo 0)
  cat > "${out_dir}/manifest.json" <<EOF
{
  "session_id": "${SESSION_ID}",
  "role": "receiver",
  "duration_s": ${DURATION_S},
  "serial_dev": "${REAL_SERIAL_DEV}",
  "serial_baud": ${SERIAL_BAUD},
  "robot": "${ROBOT_NAME}",
  "started_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "finished_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "rosbag_size_mb": ${bag_size_mb:-0}
}
EOF

  # ---- 8. Write SHA256SUMS.txt (hashes every file) ---------------------
  write_sha256sums "${out_dir}"

  # ---- 9. Verify deliverables ------------------------------------------
  local missing=()
  for f in receiver_serial_log.csv clock_offsets.txt rosbag.db3 manifest.json \
           session_info.txt SHA256SUMS.txt chronyc_tracking.txt; do
    [[ -s "${out_dir}/${f}" ]] || missing+=("${f}")
  done
  if (( ${#missing[@]} > 0 )); then
    log "WARN: missing/empty: ${missing[*]}"
  else
    local rx_count co_count
    rx_count=$(($(wc -l < "${out_dir}/receiver_serial_log.csv") - 1))
    co_count=$(($(wc -l < "${out_dir}/clock_offsets.txt") - 1))
    log "OK: all 8 deliverables written."
    log "    receiver_serial_log.csv: ${rx_count} RX events"
    log "    clock_offsets.txt: ${co_count} clock offsets"
    log "    rosbag.db3: ${bag_size_mb:-?} MB"
    log "    session_info.txt, SHA256SUMS.txt, chronyc_tracking.txt present"
  fi

  log "DONE. Output: ${out_dir}"
  log ""
  log "Files in session directory:"
  ls -la "${out_dir}" >&2
}

main "$@"
