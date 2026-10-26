#!/usr/bin/env bash
# Software-in-the-loop flight: sim, flight computer and ground station as three
# processes talking UDP over loopback, with the ground station sending ARM.
#
#   scripts/sitl.sh -o out/nominal
#   scripts/sitl.sh -o out/alt --fault alt_dropout@24:6
#   scripts/sitl.sh -o out/rt --rt          # SCHED_FIFO (needs root / CAP_SYS_NICE)
#   scripts/sitl.sh -o out/gui --gui        # live plots instead of headless
#
# Outputs in the run directory: flight.fclog, fc_stats.json, gs_stats.json,
# sim_summary.json, telemetry.csv, truth.csv, *.log
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD=${BUILD:-build/release}
OUT=out/sitl
FAULTS=()
FC_ARGS=()
GS_MODE=(--headless)
SEED=42

while [[ $# -gt 0 ]]; do
    case "$1" in
    -o) OUT=$2; shift 2 ;;
    --fault) FAULTS+=(--fault "$2"); shift 2 ;;
    --seed) SEED=$2; shift 2 ;;
    --rt) FC_ARGS+=(--rt); shift ;;
    --spin-us) FC_ARGS+=(--spin-us "$2"); shift 2 ;;
    --gui) GS_MODE=(); shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
    esac
done

mkdir -p "$OUT"
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

FC_CMD=("$BUILD/flight_computer" --log "$OUT/flight.fclog" --stats "$OUT/fc_stats.json" --exit-after 4 "${FC_ARGS[@]}")
if [[ " ${FC_ARGS[*]} " == *" --rt "* && $EUID -ne 0 ]]; then FC_CMD=(sudo "${FC_CMD[@]}"); fi
"${FC_CMD[@]}" >"$OUT/fc.log" 2>&1 &
PIDS+=($!)
FC_PID=$!

python3 ground/ground_station.py "${GS_MODE[@]}" --no-console --auto-arm 2 --exit-on-terminal 3 \
    --stats "$OUT/gs_stats.json" --csv "$OUT/telemetry.csv" >"$OUT/gs.log" 2>&1 &
PIDS+=($!)
GS_PID=$!

sleep 0.5
"$BUILD/fc_sim" --seed "$SEED" --truth "$OUT/truth.csv" --summary "$OUT/sim_summary.json" \
    --max-time 180 "${FAULTS[@]}" >"$OUT/sim.log" 2>&1
wait "$GS_PID" || true
wait "$FC_PID" || true
trap - EXIT

echo "== $OUT"
grep -h -E '^\[sim\] (LANDED|CRASH|CHUTE|NO_)' "$OUT/sim.log" || tail -2 "$OUT/sim.log"
grep -h -E '^\[fc\] (t=|final|wake|period)' "$OUT/fc.log"
python3 - "$OUT/gs_stats.json" <<'EOF'
import json, sys
s = json.load(open(sys.argv[1]))
t, u = s["telemetry"], s["uplink"]
print(f"[gs] telemetry {t['received']} rx, {t['dropped']} lost, {t['corrupt']} corrupt -> {t['delivery_pct']}% delivered;"
      f" uplink {u['acked']}/{u['commands']} acked, {u['retransmissions']} retransmissions; final {s['final_state']}")
EOF
