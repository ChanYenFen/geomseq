# native vs Python -- where each call's time goes

`native` is the algorithm alone (`benchmarks/native/bench_core`). `python` is the same call through the wrapper (`benchmarks/python/run.py`): ctypes, marshaling and wrapper included. `bridge` is the difference.

| | |
|---|---|
| Native toolchain | MSVC 1929 |
| Python | 3.10.6 on Windows AMD64 |
| DLL under test | `fd6ad632ec57ae5e...` built 2026-09-03T14:18:23 |

## `build_turn_waypoints`

| geometry | theta_max_deg | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|
| straight | 30 | 2 | 0.141 us | 6.05 us | 5.91 us | 2.3% |
| straight | 10 | 2 | 0.154 us | 5.76 us | 5.61 us | 2.7% |
| straight | 5 | 2 | 0.150 us | 8.18 us | 8.03 us | 1.8% |
| straight | 1 | 2 | 0.168 us | 6.28 us | 6.11 us | 2.7% |
| right_angle | 30 | 6 | 0.234 us | 7.00 us | 6.76 us | 3.3% |
| right_angle | 10 | 12 | 0.266 us | 8.55 us | 8.29 us | 3.1% |
| right_angle | 5 | 20 | 0.297 us | 12.24 us | 11.95 us | 2.4% |
| right_angle | 1 | 92 | 0.619 us | 22.69 us | 22.08 us | 2.7% |
| hairpin | 30 | 8 | 0.191 us | 7.02 us | 6.83 us | 2.7% |
| hairpin | 10 | 20 | 0.252 us | 9.43 us | 9.18 us | 2.7% |
| hairpin | 5 | 38 | 0.330 us | 14.94 us | 14.61 us | 2.2% |
| hairpin | 1 | 182 | 0.944 us | 41.45 us | 40.50 us | 2.3% |

## `redistribute_lookups`

| band | corners | mode | input_n | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|---|---|
| 2-8 | 0 | 0 | 101 | 367 | 1.711 us | 16.30 us | 14.59 us | 10.5% |
| 2-8 | 0 | 0 | 1001 | 367 | 1.711 us | 16.90 us | 15.19 us | 10.1% |
| 2-8 | 0 | 0 | 10001 | 367 | 1.711 us | 19.00 us | 17.29 us | 9.0% |
| 2-8 | 0 | 0 | 100001 | 367 | 1.711 us | 60.00 us | 58.29 us | 2.9% |
| 8-20 | 0 | 0 | 10001 | 102 | 0.506 us | 31.30 us | 30.79 us | 1.6% |
| 2-8 | 0 | 0 | 10001 | 367 | 1.711 us | 16.80 us | 15.09 us | 10.2% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.729 us | 48.00 us | 41.27 us | 14.0% |
| 0.2-0.8 | 0 | 0 | 10001 | 3657 | 16.701 us | 124.30 us | 107.60 us | 13.4% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.729 us | 48.40 us | 41.67 us | 13.9% |
| 0.5-2 | 10 | 0 | 10001 | 1465 | 6.675 us | 84.80 us | 78.12 us | 7.9% |
| 0.5-2 | 100 | 0 | 10001 | 1462 | 6.438 us | 131.20 us | 124.76 us | 4.9% |
| 0.5-2 | 1000 | 0 | 10001 | 1557 | 5.974 us | 375.70 us | 369.73 us | 1.6% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.729 us | 47.00 us | 40.27 us | 14.3% |
| 0.5-2 | 0 | 1 | 10001 | 714 | 4.962 us | 27.40 us | 22.44 us | 18.1% |

`input_n` is a Python-side axis only -- the native call does not receive it, so one native row is compared against every Python row that varies it.

