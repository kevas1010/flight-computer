# Measurements (scripts/measure.sh, Ubuntu 24.04 on WSL2, 2026-10-25)

## Loop timing, 100 Hz loop, nominal flights

| run | SCHED_FIFO | spin | frames | wake p50 us | wake p99 us | wake max us | jitter p50 us | jitter p99 us | jitter max us | overruns |
|---|---|---|---|---|---|---|---|---|---|---|
| timing_default_1 | False | 0 | 4155 | 612.0 | 858.6 | 3029.7 | 27.0 | 411.9 | 2529.8 | 0 |
| timing_default_2 | False | 0 | 4165 | 222.7 | 480.3 | 2099.2 | 23.4 | 324.9 | 1911.1 | 0 |
| timing_default_3 | False | 0 | 4158 | 286.1 | 544.5 | 972.7 | 28.9 | 309.7 | 652.9 | 0 |
| timing_rt_1 | True | 0 | 4168 | 293.1 | 612.8 | 2218.5 | 34.2 | 362.5 | 1796.8 | 0 |
| timing_rt_2 | True | 0 | 4163 | 78.2 | 588.6 | 1815.4 | 22.2 | 399.7 | 1741.9 | 0 |
| timing_rt_3 | True | 0 | 4158 | 297.9 | 910.6 | 1676.5 | 35.0 | 446.4 | 1358.7 | 0 |
| timing_rt_spin_1 | True | 500 | 4153 | 0.0 | 101.5 | 1215.8 | 0.2 | 137.8 | 1216.1 | 0 |
| timing_rt_spin_2 | True | 500 | 4166 | 0.0 | 131.5 | 3163.3 | 0.2 | 209.7 | 3163.6 | 0 |
| timing_rt_spin_3 | True | 500 | 4159 | 0.0 | 120.1 | 3495.7 | 0.2 | 195.2 | 3496.1 | 0 |

## Flights (real-time SITL, three processes over UDP)

| run | outcome | touchdown m/s | apogee m | FC final state | abort reason | telemetry delivered | uplink acked |
|---|---|---|---|---|---|---|---|
| fault_alt_dropout | LANDED | 1.07 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| fault_dual | CHUTE_LANDING | 6.55 | 547 | ABORT | nav_loss | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| fault_imu_stuck | LANDED | 1.04 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| fault_sensor_blackout | CHUTE_LANDING | 6.55 | 387 | ABORT | sensor_link | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| netem20_flight | LANDED | 1.06 | 624 | LANDED | none | 81.086% (383 lost, 0 corrupt) | 1/1 (1 retx) |
| timing_default_1 | LANDED | 1.07 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_default_2 | LANDED | 1.06 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_default_3 | LANDED | 1.06 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_1 | LANDED | 1.05 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_2 | LANDED | 1.06 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_3 | LANDED | 1.06 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_spin_1 | LANDED | 1.05 | 621 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_spin_2 | LANDED | 1.05 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |
| timing_rt_spin_3 | LANDED | 1.06 | 624 | LANDED | none | 100.0% (0 lost, 0 corrupt) | 1/1 (0 retx) |

## Uplink (stop-and-wait ARQ), 2000 PING commands

| link | delivered | failed | first-try | mean tx/cmd | retransmissions | latency p50 ms | p99 ms | max ms |
|---|---|---|---|---|---|---|---|---|
| uplink_clean | 2000/2000 (100.0%) | 0 | 100.0% | 1.0 | 0 | 9.97 | 10.53 | 12.5 |
| uplink_netem20 | 2000/2000 (100.0%) | 0 | 63.65% | 1.576 | 1152 | 10.05 | 210.01 | 310.23 |

## replay.txt

```
MATCH /root/fcruns/measurements/fault_alt_dropout/flight.fclog
MATCH /root/fcruns/measurements/fault_dual/flight.fclog
MATCH /root/fcruns/measurements/fault_imu_stuck/flight.fclog
MATCH /root/fcruns/measurements/fault_sensor_blackout/flight.fclog
MATCH /root/fcruns/measurements/netem20_flight/flight.fclog
MATCH /root/fcruns/measurements/timing_default_1/flight.fclog
MATCH /root/fcruns/measurements/timing_default_2/flight.fclog
MATCH /root/fcruns/measurements/timing_default_3/flight.fclog
MATCH /root/fcruns/measurements/timing_rt_1/flight.fclog
MATCH /root/fcruns/measurements/timing_rt_2/flight.fclog
MATCH /root/fcruns/measurements/timing_rt_3/flight.fclog
MATCH /root/fcruns/measurements/timing_rt_spin_1/flight.fclog
MATCH /root/fcruns/measurements/timing_rt_spin_2/flight.fclog
MATCH /root/fcruns/measurements/timing_rt_spin_3/flight.fclog
```

## ctest.txt

```
Total Tests: 88
```

## pytest.txt

```
Ran 13 tests in 0.021s
```
