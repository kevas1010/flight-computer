#!/usr/bin/env bash
# Re-records the golden flight logs used by the replay regression tests.
# Only run this when a behaviour change is intended, and review the printed
# transitions against the previous ones before committing.
set -euo pipefail
cd "$(dirname "$0")/.."
BUILD=${BUILD:-build/release}
mkdir -p tests/data
for s in nominal alt_dropout imu_stuck dual_fault; do
    "$BUILD/fc_closedloop" --scenario "$s" --log "tests/data/$s.fclog"
done
