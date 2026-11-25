#!/usr/bin/env bash
#
# record_bacs_sessions.sh - BACS policy comparison capture script.
#
# Runs the four scheduler policies (FIFO, BACS, BACS+, RANDOM) with real
# constraint candidates from the ICP scan-matching pipeline. Sessions are
# NUMBERED (HWS-BACS-001 through HWS-BACS-012) - no policy names in the
# session ID, per the reproducibility convention.
#
# For every session it produces, under ~/hardware_runs/<session_id>/:
#   * serial_log.csv    - raw MR serial log (one row per observed line)
#   * timing_log.csv    - host-monotonic TX_DONE and remote-ACK timing
#   * rosbag.db3        - ROS 2 bag with:
#                          - /scan/<robot>      (EAI T-mini Pro 2D LiDAR)
#                          - /odom/<robot>      (wheel odometry)
#                          - /tf, /tf_static    (transform tree)
#                          - /vicon/<robot>/pose (ground truth)
#                          - /bacs/scheduler     (scheduler SELECT events)
#                          - /bacs/candidates   (all generated candidates)
#                          - /bacs/serial/raw    (raw MR serial lines)
#                          - /bacs/scheduler/clock_offset
#   * manifest.json     - session metadata (policy, seed, durations)
#
# A 10-minute session produces roughly 50-150 MB per robot of rosbag data,
# depending on LiDAR scan rate and point density.
#
# Usage:
#   ./record_bacs_sessions.sh                 # full 12-session sweep (10 min each)
#   ./record_bacs_sessions.sh --dry-run       # print the plan, do not touch hardware
#   ./record_bacs_sessions.sh --duration 600  # override 600 s per session
#   ./record_bacs_sessions.sh --policies FIFO,BACS
#   ./record_bacs_sessions.sh --seed 42
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DURATION_S="${BACS_SESSION_DURATION_S:-600}"       # 10 minutes default
TX_PERIOD_S="${BACS_TX_PERIOD_S:-11.0}"            # duty-cycle-safe at SF7/BW125
CONSTRAINT_SEED="${BACS_CONSTRAINT_SEED:-42}"
CONSTRAINT_PERIOD_S="${BACS_CONSTRAINT_PERIOD_S:-2.0}"
POLICIES=("FIFO" "BACS" "BACS+" "RANDOM")
REPEATS=3
RUNS_ROOT="${HOME}/hardware_runs"
SERIAL_DEV="${BACS_SERIAL_DEV:-/tmp/pty0}"
SERIAL_BAUD="${BACS_SERIAL_BAUD:-115200}"
ROBOT_NAME="${BACS_ROBOT:-LIMO-01}"
SOURCE_WS="${BACS_WS:-${HOME}/GitHub/BACS-ROS2-SLAM/ros2_ws}"
DRY_RUN=0
INTER_SESSION_COOLDOWN_S=10

# ROS 2 topics recorded into rosbag.db3. Includes the full sensor suite
# (LiDAR, odometry, TF, Vicon) plus all scheduler event topics.
# Topic names with hyphens are sanitized (LIMO-01 -> limo_01).
ROBOT_TOPIC="${ROBOT_NAME//[^a-zA-Z0-9]/_}"
ROBOT_TOPIC_LC="${ROBOT_TOPIC,,}"
BAG_TOPICS=(
  "/odom/${ROBOT_TOPIC_LC}"
  "/tf"
  "/tf_static"
  "/scan/${ROBOT_TOPIC_LC}"
  "/vicon/${ROBOT_TOPIC_LC}/pose"
  "/bacs/scheduler"
  "/bacs/scheduler/clock_offset"
  "/bacs/serial/raw"
  "/bacs/candidates"
)

# ---------------------------------------------------------------------------
# Arg parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)             DRY_RUN=1; shift ;;
    --duration)            DURATION_S="$2"; shift 2 ;;
    --tx-period)           TX_PERIOD_S="$2"; shift 2 ;;
    --seed)                CONSTRAINT_SEED="$2"; shift 2 ;;
    --constraint-period)   CONSTRAINT_PERIOD_S="$2"; shift 2 ;;
    --policies)            IFS=',' read -r -a POLICIES <<< "$2"; shift 2 ;;
    --repeats)             REPEATS="$2"; shift 2 ;;
    --root)                RUNS_ROOT="$2"; shift 2 ;;
    --serial-dev)          SERIAL_DEV="$2"; shift 2 ;;
    --serial-baud)         SERIAL_BAUD="$2"; shift 2 ;;
    --robot)               ROBOT_NAME="$2"; shift 2 ;;
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
log()  { printf '[record_bacs %s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >&2; }
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
  # Allow socat bridge to hold the PTY master (it's the legitimate bridge
  # between WSL2 and the Windows TCP serial proxy).
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
    exit 1
  fi
}

# Build the per-session interleaved plan with NUMBERED session IDs.
# For REPEATS=3 and 4 policies this produces 12 sessions:
#   HWS-BACS-001: FIFO,    HWS-BACS-002: BACS,    HWS-BACS-003: BACS+,  HWS-BACS-004: RANDOM
#   HWS-BACS-005: FIFO,    HWS-BACS-006: BACS,    HWS-BACS-007: BACS+,  HWS-BACS-008: RANDOM
#   HWS-BACS-009: FIFO,    HWS-BACS-010: BACS,    HWS-BACS-011: BACS+,  HWS-BACS-012: RANDOM
build_plan() {
  local plan=()
  local n=0
  for ((r=1; r<=REPEATS; r++)); do
    for p in "${POLICIES[@]}"; do
      n=$((n+1))
      printf -v num '%03d' "${n}"
      plan+=("${num}:${p}:${r}")
    done
  done
  printf '%s\n' "${plan[@]}"
}

run_one_session() {
  local num="$1" policy="$2" round="$3"
  local session_id="HWS-BACS-${num}"
  local out_dir="${RUNS_ROOT}/${session_id}"
  mkdir -p "${out_dir}"

  log "=== session ${session_id} | policy=${policy} | round ${round} | ${DURATION_S}s ==="
  log "    output: ${out_dir}"
  log "    expected files: serial_log.csv, timing_log.csv, rosbag.db3, manifest.json"
  log "    seed=${CONSTRAINT_SEED} constraint_period=${CONSTRAINT_PERIOD_S}s tx_period=${TX_PERIOD_S}s"

  # Write the session manifest so the policy is recoverable from the
  # numbered session directory without needing to parse log files.
  cat > "${out_dir}/manifest.json" <<EOF
{
  "session_id": "${session_id}",
  "session_number": ${num},
  "policy": "${policy}",
  "round": ${round},
  "duration_s": ${DURATION_S},
  "tx_period_s": ${TX_PERIOD_S},
  "constraint_seed": ${CONSTRAINT_SEED},
  "constraint_period_s": ${CONSTRAINT_PERIOD_S},
  "serial_dev": "${SERIAL_DEV}",
  "serial_baud": ${SERIAL_BAUD},
  "robot": "${ROBOT_NAME}",
  "started_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

  if (( DRY_RUN )); then
    log "    [dry-run] skipping hardware capture."
    return 0
  fi

  verify_ros2
  verify_serial

  log "    serial: ${SERIAL_DEV}@${SERIAL_BAUD}"

  # ---- 1. Launch the BACS scheduler node with real constraints -----------
  # Uses scheduler_node_bacs which generates constraint candidates via
  # ConstraintSource and uses Scheduler.select() to pick which to transmit.
  setsid ros2 launch bacs_scheduler session_bacs.launch.py \
      policy:="${policy}" \
      serial_dev:="${SERIAL_DEV}" \
      serial_baud:="${SERIAL_BAUD}" \
      session_dir:="${out_dir}" \
      session_id:="${session_id}" \
      tx_period_s:="${TX_PERIOD_S}" \
      constraint_seed:="${CONSTRAINT_SEED}" \
      constraint_period_s:="${CONSTRAINT_PERIOD_S}" &
  local sched_pid=$!

  # ---- 2. Record the ROS 2 bag (LiDAR + odom + TF + scheduler) -----------
  # A 10-minute session at 10 Hz LiDAR produces ~50-150 MB per robot.
  setsid ros2 bag record \
      -s sqlite3 \
      -o "${out_dir}/rosbag" \
      --include-hidden-topics \
      "${BAG_TOPICS[@]}" &
  local bag_pid=$!

  # ---- 3. Run for the requested wall-clock duration -----------------------
  log "    filming for ${DURATION_S}s ..."
  sleep "${DURATION_S}"

  # ---- 4. Stop the bag first (flushes all messages) then the scheduler -----
  kill -INT -- "-${bag_pid}"   2>/dev/null || true
  wait "${bag_pid}" 2>/dev/null || true
  kill -INT -- "-${sched_pid}" 2>/dev/null || true
  wait "${sched_pid}" 2>/dev/null || true
  pkill -INT -f "ros2 bag record.*${out_dir//\//\\/}" 2>/dev/null || true
  pkill -INT -f "scheduler_node_bacs.*${session_id}"    2>/dev/null || true
  sleep 1

  # ---- 5. Rename the bag directory to the canonical rosbag.db3 name -------
  local db3_file
  db3_file="$(find "${out_dir}/rosbag" -name '*.db3' -print -quit 2>/dev/null || true)"
  if [[ -n "${db3_file}" && -f "${db3_file}" ]]; then
    mv "${db3_file}" "${out_dir}/rosbag.db3"
    cp "${out_dir}/rosbag/metadata.yaml" "${out_dir}/rosbag_metadata.yaml" 2>/dev/null || true
    rm -rf "${out_dir}/rosbag"
  fi

  # ---- 6. Update manifest with completion status --------------------------
  local bag_size_mb
  bag_size_mb=$(du -m "${out_dir}/rosbag.db3" 2>/dev/null | cut -f1 || echo 0)
  cat >> "${out_dir}/manifest.json" <<EOF
{
  "session_id": "${session_id}",
  "session_number": ${num},
  "policy": "${policy}",
  "round": ${round},
  "duration_s": ${DURATION_S},
  "tx_period_s": ${TX_PERIOD_S},
  "constraint_seed": ${CONSTRAINT_SEED},
  "constraint_period_s": ${CONSTRAINT_PERIOD_S},
  "serial_dev": "${SERIAL_DEV}",
  "serial_baud": ${SERIAL_BAUD},
  "robot": "${ROBOT_NAME}",
  "started_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "finished_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "rosbag_size_mb": ${bag_size_mb:-0}
}
EOF

  # ---- 7. Verify the three required deliverables exist --------------------
  local missing=()
  for f in serial_log.csv rosbag.db3 clock_offsets.txt; do
    [[ -s "${out_dir}/${f}" ]] || missing+=("${f}")
  done
  if (( ${#missing[@]} > 0 )); then
    log "    WARN: missing/empty: ${missing[*]}"
  else
    log "    OK: serial_log.csv, rosbag.db3, clock_offsets.txt all written."
    log "    rosbag size: ${bag_size_mb:-?} MB"
  fi

  log "    cooling down ${INTER_SESSION_COOLDOWN_S}s ..."
  sleep "${INTER_SESSION_COOLDOWN_S}"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
  log "BACS-ROS2-SLAM BACS policy comparison capture"
  log "  sessions:   ${#POLICIES[@]} policies x ${REPEATS} rounds = $((${#POLICIES[@]} * REPEATS)) sessions"
  log "  duration:   ${DURATION_S}s per session (10 min default)"
  log "  tx period:  ${TX_PERIOD_S}s (1% duty-cycle at SF7/BW125)"
  log "  seed:       ${CONSTRAINT_SEED} (same for all policies - fair comparison)"
  log "  output:     ${RUNS_ROOT}/HWS-BACS-NNN/{serial_log.csv,rosbag.db3,clock_offsets.txt,manifest.json}"
  log "  serial:     ${SERIAL_DEV}@${SERIAL_BAUD}"
  log "  robot:      ${ROBOT_NAME}"
  log "  topics:     ${BAG_TOPICS[*]}"
  if (( DRY_RUN )); then
    log "  mode:       DRY-RUN (no hardware will be touched)"
  fi
  echo "---" >&2

  local total=0
  while IFS=: read -r num policy round; do
    total=$((total+1))
    run_one_session "${num}" "${policy}" "${round}"
  done < <(build_plan)

  log "DONE. Captured ${total} sessions under ${RUNS_ROOT}/."
  log "Session manifest (policy mapping):"
  for d in ${RUNS_ROOT}/HWS-BACS-0*; do
    [[ -f "${d}/manifest.json" ]] || continue
    local sid policy
    sid=$(basename "${d}")
    policy=$(python3 -c "import json; print(json.load(open('${d}/manifest.json'))['policy'])" 2>/dev/null || echo "?")
    log "  ${sid}: policy=${policy}"
  done
  if (( DRY_RUN )); then
    log "(dry-run: no files were actually written.)"
  fi
}

main "$@"
