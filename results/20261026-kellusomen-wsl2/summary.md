# Measurements (20261026-kellusomen-wsl2)

```
host: kellusomen
os: Ubuntu 24.04.3 LTS
kernel: 6.6.87.2-microsoft-standard-WSL2 (x86_64)
cpu: Intel(R) Core(TM) Ultra 7 255H x 16
platform: WSL2 (virtual machine)
date: 2026-10-26T05:13Z
commit: ad7ba95
note: fresh clone of github.com/kevas1010/flight-computer at ad7ba95, run via scripts/bootstrap.sh
```

## Loop timing, 100 Hz loop, nominal flights

| run | SCHED_FIFO | spin | frames | wake p50 us | wake p99 us | wake max us | jitter p50 us | jitter p99 us | jitter max us | overruns |
|---|---|---|---|---|---|---|---|---|---|---|
| timing_default_1 | False | 0 | 4138 | 94.4 | 944.3 | 1619.0 | 23.6 | 297.1 | 1560.2 | 0 |
| timing_default_2 | False | 0 | 4134 | 300.0 | 919.1 | 1088.7 | 25.2 | 311.5 | 843.2 | 0 |
| timing_default_3 | False | 0 | 4145 | 282.9 | 526.7 | 712.7 | 26.9 | 259.7 | 436.4 | 0 |
| timing_rt_1 | True | 0 | 4143 | 295.6 | 518.3 | 2646.9 | 28.4 | 303.5 | 2265.3 | 0 |
| timing_rt_2 | True | 0 | 4141 | 286.3 | 449.7 | 754.3 | 26.0 | 233.2 | 618.7 | 0 |
| timing_rt_3 | True | 0 | 4142 | 294.9 | 421.2 | 879.5 | 21.8 | 186.6 | 574.8 | 0 |
| timing_rt_spin_1 | True | 500 | 4142 | 0.0 | 50.2 | 433.3 | 0.1 | 88.8 | 433.6 | 0 |
| timing_rt_spin_2 | True | 500 | 4143 | 0.0 | 55.6 | 381.3 | 0.1 | 113.5 | 381.4 | 0 |
| timing_rt_spin_3 | True | 500 | 4142 | 0.0 | 125.1 | 312.7 | 6.5 | 181.3 | 311.6 | 0 |

## Flights (real-time SITL, three processes over UDP)

| run | outcome | touchdown m/s | apogee m | FC final state | abort reason | telemetry delivered | uplink acked |
|---|---|---|---|---|---|---|---|
| fault_alt_dropout | LANDED | 1.07 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| fault_dual | CHUTE_LANDING | 6.55 | 565 | ABORT | nav_loss | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| fault_imu_stuck | LANDED | 1.04 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| fault_sensor_blackout | CHUTE_LANDING | 6.55 | 404 | ABORT | sensor_link | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| netem20_corrupt1_flight | LANDED | 1.05 | 624 | LANDED | none | 78.663% (431 lost, 16 corrupt) | 1/1 (0 retx) |
| netem20_flight | LANDED | 1.05 | 624 | LANDED | none | 78.458% (436 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_default_1 | LANDED | 1.05 | 625 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_default_2 | LANDED | 1.06 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_default_3 | LANDED | 1.05 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_1 | LANDED | 1.05 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_2 | LANDED | 1.06 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_3 | LANDED | 1.05 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_spin_1 | LANDED | 1.05 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_spin_2 | LANDED | 1.05 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_spin_3 | LANDED | 1.05 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |

## Uplink (stop-and-wait ARQ), 2000 PING commands

| link | delivered | failed | first-try | mean tx/cmd | retransmissions | latency p50 ms | p99 ms | max ms | corrupt ACKs rejected (gs) | corrupt commands rejected (fc) |
|---|---|---|---|---|---|---|---|---|---|---|
| uplink_clean | 2000/2000 (100.0%) | 0 | 100.0% | 1.0 | 0 | 9.98 | 10.24 | 10.51 | 0 | - |
| uplink_netem20 | 2000/2000 (100.0%) | 0 | 64.0% | 1.577 | 1155 | 10.04 | 210.08 | 420.12 | 0 | - |
| uplink_netem20_corrupt1 | 2000/2000 (100.0%) | 0 | 65.2% | 1.538 | 1077 | 10.03 | 210.03 | 360.01 | 5 | 8 |

## replay.txt

```
MATCH out/measurements-kellusomen-20261026-0513/fault_alt_dropout/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/fault_dual/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/fault_imu_stuck/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/fault_sensor_blackout/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/netem20_corrupt1_flight/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/netem20_flight/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_default_1/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_default_2/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_default_3/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_rt_1/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_rt_2/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_rt_3/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_rt_spin_1/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_rt_spin_2/flight.fclog
MATCH out/measurements-kellusomen-20261026-0513/timing_rt_spin_3/flight.fclog
```

## ctest.txt

```
Total Tests: 88
```

## pytest.txt

```
Ran 13 tests in 0.014s
```
