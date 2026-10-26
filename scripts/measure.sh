#!/usr/bin/env bash
# Full measurement campaign behind the numbers in README.md. Needs root for
# tc netem and SCHED_FIFO (passwordless sudo is fine). Takes ~12 minutes.
#
#   BUILD=build/release scripts/measure.sh [OUT_DIR]
#
# Writes every run's raw outputs under OUT_DIR and a summary to OUT_DIR/summary.md.
set -euo pipefail
cd "$(dirname "$0")/.."
export BUILD=${BUILD:-build/release}
OUT=${1:-out/measurements}
SUDO=""
[[ $EUID -ne 0 ]] && SUDO=sudo
mkdir -p "$OUT"
trap '$SUDO bash scripts/netem.sh off >/dev/null' EXIT

echo "### 1. Loop timing (nominal flights)"
for i in 1 2 3; do bash scripts/sitl.sh -o "$OUT/timing_default_$i"; done
for i in 1 2 3; do bash scripts/sitl.sh -o "$OUT/timing_rt_$i" --rt; done
for i in 1 2 3; do bash scripts/sitl.sh -o "$OUT/timing_rt_spin_$i" --rt --spin-us 500; done

echo "### 2. Fault injection (real-time SITL)"
bash scripts/sitl.sh -o "$OUT/fault_alt_dropout" --fault alt_dropout@24:6
bash scripts/sitl.sh -o "$OUT/fault_imu_stuck" --fault imu_stuck@24:30
bash scripts/sitl.sh -o "$OUT/fault_dual" --fault imu_stuck@14:30 --fault alt_dropout@15:30
bash scripts/sitl.sh -o "$OUT/fault_sensor_blackout" --fault sensor_blackout@12:1

echo "### 3. Ground links under 20% netem loss"
$SUDO bash scripts/netem.sh on 20
bash scripts/sitl.sh -o "$OUT/netem20_flight"
# Uplink stress: flight computer + sim idle on the pad; PING is accepted in any state.
"$BUILD/flight_computer" --log none --max-time 900 >"$OUT/stress_fc.log" 2>&1 &
FC=$!
"$BUILD/fc_sim" --max-time 900 >/dev/null 2>&1 &
SIM=$!
sleep 1
python3 ground/uplink_stress.py --count 2000 --json "$OUT/uplink_netem20.json" >/dev/null || true
$SUDO bash scripts/netem.sh off
python3 ground/uplink_stress.py --count 2000 --json "$OUT/uplink_clean.json" >/dev/null || true
kill "$FC" "$SIM" 2>/dev/null || true
wait "$FC" "$SIM" 2>/dev/null || true

echo "### 3b. Ground links under 20% loss + 1% bit corruption (CRC must catch it)"
$SUDO bash scripts/netem.sh on 20 1
bash scripts/sitl.sh -o "$OUT/netem20_corrupt1_flight"
"$BUILD/flight_computer" --log none --max-time 900 --stats "$OUT/uplink_corrupt_fc_stats.json" \
    >"$OUT/stress_corrupt_fc.log" 2>&1 &
FC=$!
"$BUILD/fc_sim" --max-time 900 >/dev/null 2>&1 &
SIM=$!
sleep 1
python3 ground/uplink_stress.py --count 2000 --json "$OUT/uplink_netem20_corrupt1.json" >/dev/null || true
kill -INT "$FC" 2>/dev/null || true
kill "$SIM" 2>/dev/null || true
wait "$FC" "$SIM" 2>/dev/null || true
$SUDO bash scripts/netem.sh off

echo "### 4. Log replay of every real-time flight"
: >"$OUT/replay.txt"
for log in "$OUT"/*/flight.fclog; do
    if "$BUILD/fc_replay" "$log" --tol 0 >/dev/null; then r=MATCH; else r=DIVERGED; fi
    echo "$r $log" | tee -a "$OUT/replay.txt"
done

echo "### 5. Test counts"
(cd "$BUILD" && ctest -N | tail -1) | tee "$OUT/ctest.txt"
python3 -m unittest discover -s ground 2>&1 | grep -E '^Ran' | tee "$OUT/pytest.txt"

python3 scripts/summarize.py "$OUT" | tee "$OUT/summary.md"
