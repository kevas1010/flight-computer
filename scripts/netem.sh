#!/usr/bin/env bash
# Impairs only the ground links on loopback with tc netem:
#   telemetry   fc -> gs   udp dport 5600
#   uplink      gs -> fc   udp dport 5601
#   ACKs        fc -> gs   udp sport 5601
# The sim <-> flight computer sensor/actuator links (5500/5501) are untouched,
# so the vehicle keeps flying while the radio link degrades.
#
#   sudo scripts/netem.sh on 20          # 20% loss each way
#   sudo scripts/netem.sh on 20 0.5      # plus 0.5% bit corruption
#   sudo scripts/netem.sh off | status
set -euo pipefail
DEV=${DEV:-lo}

case "${1:-status}" in
on)
    LOSS=${2:-20}
    CORRUPT=${3:-0}
    tc qdisc del dev "$DEV" root 2>/dev/null || true
    # Bands 1-3 keep the default priomap; band 4 is the impaired one.
    tc qdisc add dev "$DEV" root handle 1: prio bands 4 priomap 1 2 2 2 1 2 0 0 1 1 1 1 1 1 1 1
    NETEM="loss ${LOSS}%"
    if [[ "$CORRUPT" != "0" ]]; then NETEM="$NETEM corrupt ${CORRUPT}%"; fi
    # shellcheck disable=SC2086
    tc qdisc add dev "$DEV" parent 1:4 handle 40: netem $NETEM
    for match in "dport 5600" "dport 5601" "sport 5601"; do
        # shellcheck disable=SC2086
        tc filter add dev "$DEV" parent 1:0 protocol ip prio 1 u32 \
            match ip protocol 17 0xff match ip $match 0xffff flowid 1:4
    done
    echo "netem on $DEV: $NETEM on udp 5600/5601"
    ;;
off)
    tc qdisc del dev "$DEV" root 2>/dev/null || true
    echo "netem off on $DEV"
    ;;
status)
    tc qdisc show dev "$DEV"
    tc filter show dev "$DEV" 2>/dev/null || true
    ;;
*)
    echo "usage: $0 on [LOSS%] [CORRUPT%] | off | status" >&2
    exit 2
    ;;
esac
