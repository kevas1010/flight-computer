#!/usr/bin/env bash
# One command from a fresh clone to a full results table, on any Ubuntu/Debian
# machine: x86-64 laptop, Raspberry Pi 4/5 (64-bit OS), or WSL2.
#
#   git clone https://github.com/kevas1010/flight-computer.git
#   cd flight-computer
#   bash scripts/bootstrap.sh            # deps + build + tests + measurement campaign (~15 min)
#   bash scripts/bootstrap.sh --no-measure   # stop after the tests
#
# Needs sudo (apt, tc netem, SCHED_FIFO). Results land in out/measurements-<host>-<date>/.
set -euo pipefail
cd "$(dirname "$0")/.."

MEASURE=1
for arg in "$@"; do
    case "$arg" in
    --no-measure) MEASURE=0 ;;
    *) echo "usage: $0 [--no-measure]" >&2; exit 2 ;;
    esac
done

step() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

step "Machine"
. /etc/os-release
echo "OS: $PRETTY_NAME | kernel $(uname -r) | arch $(uname -m) | $(nproc) cores | $(free -g | awk '/Mem:/{print $2}') GB RAM"
grep -qi microsoft /proc/version && echo "Note: this is WSL2 (a virtual machine); timing numbers will reflect that."

step "Installing dependencies"
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    build-essential cmake ninja-build libgtest-dev python3 python3-matplotlib iproute2 kmod git >/dev/null
# tc netem lives in the sch_netem kernel module; Ubuntu ships it in linux-modules-extra.
if ! sudo modprobe sch_netem 2>/dev/null; then
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "linux-modules-extra-$(uname -r)" >/dev/null 2>&1 || true
    sudo modprobe sch_netem 2>/dev/null || echo "WARNING: sch_netem unavailable; netem runs will fail"
fi
echo "g++ $(g++ -dumpfullversion), cmake $(cmake --version | head -1 | awk '{print $3}'), python $(python3 -c 'import platform; print(platform.python_version())')"

# Keep parallel compiles within memory on small boards (each g++ needs ~0.7 GB with ASan).
MEM_GB=$(free -g | awk '/Mem:/{print $2}')
JOBS=$(nproc)
if [ "$MEM_GB" -lt 2 ]; then JOBS=1; elif [ "$MEM_GB" -lt 4 ]; then JOBS=2; fi
export CMAKE_BUILD_PARALLEL_LEVEL=$JOBS

step "Release build ($JOBS parallel jobs)"
cmake --preset release -DFC_WERROR=ON >/dev/null
cmake --build --preset release

step "Release tests"
ctest --preset release
for log in tests/data/*.fclog; do build/release/fc_replay "$log" | tail -1; done

step "AddressSanitizer + UBSan build and tests"
cmake --preset asan -DFC_WERROR=ON >/dev/null
cmake --build --preset asan
ctest --preset asan

step "Ground-station tests"
python3 -m unittest discover -s ground

if [ "$MEASURE" -eq 0 ]; then
    step "Done (measurement skipped)"
    exit 0
fi

# The campaign uses sudo for netem and SCHED_FIFO over ~15 minutes; keep the
# credential fresh so it never stops at a password prompt.
sudo -v
( while true; do sudo -n true; sleep 50; done ) 2>/dev/null &
KEEPALIVE=$!
trap 'kill $KEEPALIVE 2>/dev/null' EXIT

if grep -qi microsoft /proc/version; then PLATFORM=wsl2; else PLATFORM=baremetal; fi
OUT="results/$(date +%Y%m%d-%H%M)-$(hostname)-$PLATFORM"
step "Measurement campaign -> $OUT (about 15 minutes; keep the machine otherwise idle)"
BUILD=build/release bash scripts/measure.sh "$OUT"

step "Comparison across every result set in results/"
python3 scripts/compare.py

step "Done"
echo "New result set: $OUT (summary.md, machine.txt, raw JSON; flight logs stay local)"
echo "To keep it in your GitHub history next to the earlier runs:"
echo "  python3 scripts/compare.py --update-readme"
echo "  git add results README.md && git commit -m \"Add $PLATFORM measurements from $(hostname)\" && git push"
