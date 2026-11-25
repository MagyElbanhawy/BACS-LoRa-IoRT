#!/usr/bin/env bash
#
# record_session.sh - numbered LoRa physical link-test capture script.
#
# Records numbered LINK_TEST sessions. This script does not claim to exercise
# FIFO, BACS, BACS+, or RANDOM because it generates deterministic test frames,
# not a queue of real constraint candidates.
#
# For every session it produces, under ~/hardware_runs/<session_id>/:
#   * serial_log.csv    - raw MR serial log (one row per observed line)
#   * rosbag.db3        - ROS 2 bag with fused poses (/odom, /tf, /map,
#                         /bacs/scheduler) and the /vicon ground truth topics
#   * timing_log.csv    - host-monotonic TX_DONE and remote-ACK timing evidence
#
# Usage:
#   ./record_session.sh                 # three numbered link-test sessions
#   ./record_session.sh --dry-run       # print the plan, do not touch hardware
#   ./record_session.sh --duration 60    # override 30 s per session
#   ./record_session.sh --tx-period 11
#
# This script is the "raw serial logging via a dedicated failsafe script"
# promised in NON_REPRODUCIBLE_RESULTS.md §3.

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DURATION_S="${BACS_SESSION_DURATION_S:-120}"
TX_PERIOD_S="${BACS_TX_PERIOD_S:-11.0}"
REPEATS=3
RUNS_ROOT="${HOME}/hardware_runs"
SERIAL_DEV="${BACS_SERIAL_DEV:-/dev/ttyACM0}"
SERIAL_BAUD="${BACS_SERIAL_BAUD:-115200}"
ROBOT_NAME="${BACS_ROBOT:-LIMO-01}"
DRY_RUN=0
INTER_SESSION_COOLDOWN_S=5

# ROS 2 topics recorded into rosbag.db3. The fused-pose topics are the SLAM
# output of robot_localization / RTAB-Map; the scheduler topic is logged so
# that every select() decision is captured alongside the fused poses.
#
# ROS 2 topic names must match [a-zA-Z0-9_/~{}] (no hyphens). Robot names
# often contain hyphens (e.g. "LIMO-01"), so we sanitize by replacing any
# non-alphanumeric character with '_' when building topic names. The original
# ROBOT_NAME is kept for display/logging only.
ROBOT_TOPIC="${ROBOT_NAME//[^a-zA-Z0-9]/_}"   # LIMO-01 -> LIMO_01
ROBOT_TOPIC_LC="${ROBOT_TOPIC,,}"             # LIMO_01  -> limo_01
BAG_TOPICS=(
  "/odom/${ROBOT_TOPIC_LC}"
  "/tf"
  "/tf_static"
  "/scan/${ROBOT_TOPIC_LC}"
  "/vicon/${ROBOT_TOPIC_LC}/pose"
  "/bacs/scheduler"
  "/bacs/serial/raw"
)

# ---------------------------------------------------------------------------
# Arg parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)        DRY_RUN=1; shift ;;
    --duration)       DURATION_S="$2"; shift 2 ;;
    --tx-period)      TX_PERIOD_S="$2"; shift 2 ;;
    --repeats)        REPEATS="$2"; shift 2 ;;
    --root)           RUNS_ROOT="$2"; shift 2 ;;
    --serial-dev)     SERIAL_DEV="$2"; shift 2 ;;
    --serial-baud)    SERIAL_BAUD="$2"; shift 2 ;;
    --robot)          ROBOT_NAME="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "${RUNS_ROOT}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
log()  { printf '[record_session %s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >&2; }
iso()  { date -u +%Y%m%dT%H%M%SZ; }

verify_ros2() {
  if ! command -v ros2 >/dev/null 2>&1; then
    log "ERROR: ros2 not on PATH. Source your ROS 2 setup first."
    exit 1
  fi
}

verify_serial() {
  if [[ ! -e "${SERIAL_DEV}" ]]; then
    log "ERROR: serial device ${SERIAL_DEV} not present."
    log "       Set BACS_SERIAL_DEV or pass --serial-dev /dev/XXXX."
    exit 1
  fi

  # Refuse to start if another process is holding the port open - a leftover
  # `cat /dev/ttyACM0 &` from a manual smoke test will steal bytes from the
  # link-test node and produce truncated TX_DONE/ACK_RX evidence.
  #
  # EXCEPTION: when the serial device is a socat PTY symlink (e.g.
  # /tmp/pty0 -> /dev/pts/N), socat is *supposed* to hold the PTY master open
  # - that's the bridge between the scheduler and the Windows TCP serial
  # proxy. The scheduler opens the PTY slave, and socat shuttles bytes to
  # the real Arduino on Windows. So we only reject readers that are NOT
  # the socat bridge itself.
  local holders
  if command -v fuser >/dev/null 2>&1; then
    holders="$(fuser "${SERIAL_DEV}" 2>/dev/null || true)"
  elif command -v lsof >/dev/null 2>&1; then
    holders="$(lsof -t "${SERIAL_DEV}" 2>/dev/null | tr '\n' ' ' || true)"
  else
    holders=""
  fi

  if [[ -n "${holders// /}" ]]; then
    # Filter out the socat bridge - it is the legitimate PTY master when
    # SERIAL_DEV is a /tmp/ptyN symlink created by the operator.
    local filtered=""
    for pid in ${holders}; do
      if ps -p "${pid}" -o args= 2>/dev/null | grep -q 'socat.*pty'; then
        log "    OK: socat bridge detected (PID ${pid}) - scheduler will share the PTY slave."
        continue
      fi
      filtered="${filtered} ${pid}"
    done
    holders="${filtered}"
  fi

  if [[ -n "${holders// /}" ]]; then
    log "ERROR: ${SERIAL_DEV} is already opened by non-socat PID(s):${holders}"
    log "       Kill them first:  kill${holders}"
    log "       (a leftover 'cat /dev/ttyACM0 &' will steal ack bytes from the scheduler)"
    exit 1
  fi
}

# Build stable numeric labels. No random or policy name is embedded in a link
# test label, preventing these captures from being mistaken for policy runs.
build_plan() {
  for ((r=1; r<=REPEATS; r++)); do
    printf '%03d:%d\n' "${r}" "${r}"
  done
}

run_one_session() {
  local label="$1" round="$2"
  local session_id="HWS-LINK-${label}-$(iso)"
  local out_dir="${RUNS_ROOT}/${session_id}"
  mkdir -p "${out_dir}"

  log "=== session ${session_id} | type=LINK_TEST | ${DURATION_S}s ==="
  log "    output: ${out_dir}"
  log "    expected files: serial_log.csv, timing_log.csv, rosbag.db3"

  if (( DRY_RUN )); then
    log "    [dry-run] skipping hardware capture."
    return 0
  fi

  verify_ros2
  verify_serial

  # ---- 1. Launch the numbered LINK_TEST node ------------------------------
  log "    serial: ${SERIAL_DEV}@${SERIAL_BAUD}"

  BACS_SESSION_ID="${session_id}" \
  BACS_SESSION_DIR="${out_dir}" \
  BACS_SERIAL_DEV="${SERIAL_DEV}" \
  BACS_SERIAL_BAUD="${SERIAL_BAUD}" \
  BACS_POLICY="LINK_TEST" \
  setsid ros2 launch bacs_scheduler session.launch.py \
      policy:="LINK_TEST" \
      serial_dev:="${SERIAL_DEV}" \
      serial_baud:="${SERIAL_BAUD}" \
      session_dir:="${out_dir}" \
      session_id:="${session_id}" \
      tx_period_s:="${TX_PERIOD_S}" &
  local sched_pid=$!

  # Fail fast if parameter validation, serial opening, or node startup fails.
  # Without this guard the bag recorder would otherwise produce an empty run
  # for the full requested duration after the scheduler had already died.
  sleep 2
  if ! kill -0 "${sched_pid}" 2>/dev/null; then
    wait "${sched_pid}" 2>/dev/null || true
    log "ERROR: link-test node failed during startup; capture aborted."
    return 1
  fi

  # ---- 2. Record the ROS 2 bag (fused poses + scheduler events) -----------
  # Use the sqlite3 storage id so the deliverable matches the historical
  # HWS-002/HWS-003/HWS-005 artefacts (db3 is the primary source of truth).
  # NOTE: in ROS 2 Humble the flag is -s / --storage, NOT --storage-plugin.
  # NOTE: ros2 bag record rejects combining -a (all) with an explicit topic
  # list, so we record ONLY the BAG_TOPICS list (these include the scheduler
  # topics and the SLAM/Vicon topics of interest). --all is not used.
  # NOTE: topic names with hyphens (e.g. /odom/limo-01) are invalid in ROS 2;
  # BAG_TOPICS already sanitises ROBOT_NAME to alphanumerics+underscore.
  setsid ros2 bag record \
      -s sqlite3 \
      -o "${out_dir}/rosbag" \
      --include-hidden-topics \
      "${BAG_TOPICS[@]}" &
  local bag_pid=$!

  # ---- 3. Run for the requested wall-clock duration -----------------------
  log "    filming for ${DURATION_S}s ..."
  local stop_at=$((SECONDS + DURATION_S))
  while (( SECONDS < stop_at )); do
    sleep 1
    if ! kill -0 "${sched_pid}" 2>/dev/null; then
      log "ERROR: link-test node died during capture; stopping the bag."
      kill -INT -- "-${bag_pid}" 2>/dev/null || true
      wait "${bag_pid}" 2>/dev/null || true
      wait "${sched_pid}" 2>/dev/null || true
      return 1
    fi
  done

  # ---- 4. Stop the bag first (flushes all messages) then the scheduler -----
  # Use process-group kills (negative PID) because `ros2 launch` and
  # `ros2 bag record` spawn child processes that the parent's SIGINT does
  # NOT propagate to on its own. `setsid` above put each in its own group.
  kill -INT -- "-${bag_pid}"   2>/dev/null || true
  wait "${bag_pid}" 2>/dev/null || true
  kill -INT -- "-${sched_pid}" 2>/dev/null || true
  wait "${sched_pid}" 2>/dev/null || true
  # Belt-and-braces: kill any straggler ros2/scheduler_node processes that
  # might still be holding the serial PTY (would block the next session).
  pkill -INT -f "ros2 bag record.*${out_dir//\//\\/}" 2>/dev/null || true
  pkill -INT -f "scheduler_node.*${session_id}"        2>/dev/null || true
  sleep 1

  # ---- 5. Rename the bag directory to the canonical rosbag.db3 name -------
  local db3_file
  db3_file="$(find "${out_dir}/rosbag" -name '*.db3' -print -quit 2>/dev/null || true)"
  if [[ -n "${db3_file}" && -f "${db3_file}" ]]; then
    mv "${db3_file}" "${out_dir}/rosbag.db3"
    cp "${out_dir}/rosbag/metadata.yaml" "${out_dir}/rosbag_metadata.yaml" 2>/dev/null || true
    rm -rf "${out_dir}/rosbag"
  fi

  # ---- 6. Verify the three required deliverables exist --------------------
  local missing=()
  for f in serial_log.csv timing_log.csv rosbag.db3; do
    [[ -s "${out_dir}/${f}" ]] || missing+=("${f}")
  done
  if (( ${#missing[@]} > 0 )); then
    log "    WARN: missing/empty: ${missing[*]}"
  else
    log "    OK: serial_log.csv, timing_log.csv, rosbag.db3 all written."
  fi

  log "    cooling down ${INTER_SESSION_COOLDOWN_S}s ..."
  sleep "${INTER_SESSION_COOLDOWN_S}"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
  log "BACS-ROS2-SLAM numbered physical link-test capture"
  log "  type:       LINK_TEST (not a scheduler-policy run)"
  log "  repeats:    ${REPEATS}"
  log "  duration:   ${DURATION_S}s per session"
  log "  tx period:  ${TX_PERIOD_S}s (52-byte SF7/BW125 duty-cycle safe)"
  log "  output:     ${RUNS_ROOT}/<session_id>/{serial_log.csv,timing_log.csv,rosbag.db3}"
  log "  serial:     ${SERIAL_DEV}@${SERIAL_BAUD}"
  log "  robot:      ${ROBOT_NAME}"
  if (( DRY_RUN )); then
    log "  mode:       DRY-RUN (no hardware will be touched)"
  fi
  echo "---" >&2

  local total=0
  while IFS=: read -r label round; do
    total=$((total+1))
    run_one_session "${label}" "${round}"
  done < <(build_plan)

  log "DONE. Captured ${total} sessions under ${RUNS_ROOT}/."
  if (( DRY_RUN )); then
    log "(dry-run: no files were actually written.)"
  fi
}

main "$@"
