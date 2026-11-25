#!/usr/bin/env bash
#
# run_12_sessions_tx.sh - Runs 12 BACS scheduling sessions on the transmitter.
#
# Runs 4 policies (FIFO, BACS, BACS+, RANDOM) x 3 rounds = 12 sessions.
# Each session is 10 minutes (600s). Sessions are interleaved so
# environmental drift is shared evenly across policies.
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
#   ./run_12_sessions_tx.sh                 # full 12-session sweep
#   ./run_12_sessions_tx.sh --dry-run       # print the plan only
#   ./run_12_sessions_tx.sh --duration 600   # override duration
#
set -euo pipefail

DURATION_S=600
TX_PERIOD_S=11.0
SEED=42
PORT=/dev/ttyACM0
BAUD=115200
DRY_RUN=0
RUNS_ROOT="${HOME}/hardware_runs"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TX_SCRIPT="${SCRIPT_DIR}/../tools/pi_scheduler_tx.py"

# Policy -> session number mapping (interleaved for fairness)
# Round 1: 001=FIFO, 002=BACS, 003=BACS+, 004=RANDOM
# Round 2: 005=FIFO, 006=BACS, 007=BACS+, 008=RANDOM
# Round 3: 009=FIFO, 010=BACS, 011=BACS+, 012=RANDOM
POLICIES=("FIFO" "BACS" "BACS+" "RANDOM")

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)    DRY_RUN=1; shift ;;
    --duration)   DURATION_S="$2"; shift 2 ;;
    --tx-period)  TX_PERIOD_S="$2"; shift 2 ;;
    --seed)       SEED="$2"; shift 2 ;;
    --port)       PORT="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

log() { printf '[tx-sweep %s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >&2; }

log "BACS-ROS2-SLAM 12-session transmitter sweep"
log "  policies:   ${POLICIES[*]}"
log "  rounds:     3"
log "  total:      12 sessions"
log "  duration:   ${DURATION_S}s per session"
log "  tx_period:  ${TX_PERIOD_S}s"
log "  seed:       ${SEED}"
log "  port:       ${PORT}@${BAUD}"
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
      log "  [dry-run] would run: python3 ${TX_SCRIPT} --port ${PORT} --policy ${policy} --duration ${DURATION_S} --tx-period ${TX_PERIOD_S} --seed ${SEED} --session-id ${sid} --output-dir ${out_dir}"
      continue
    fi

    mkdir -p "${out_dir}"

    # Run the transmitter for this session.
    python3 "${TX_SCRIPT}" \
        --port "${PORT}" \
        --baud "${BAUD}" \
        --policy "${policy}" \
        --duration "${DURATION_S}" \
        --tx-period "${TX_PERIOD_S}" \
        --seed "${SEED}" \
        --session-id "${sid}" \
        --output-dir "${out_dir}"

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
  policy=$(python3 -c "import json; print(json.load(open('${d}/manifest.json'))['policy'])" 2>/dev/null || echo "?")
  tx_count=$(python3 -c "import json; print(json.load(open('${d}/manifest.json')).get('packets_transmitted', '?'))" 2>/dev/null || echo "?")
  log "  ${sid}: policy=${policy} packets_tx=${tx_count}"
done
