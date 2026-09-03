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
| straight | 30 | 2 | 0.141 us | 5.91 us | 5.77 us | 2.4% |
| straight | 10 | 2 | 0.154 us | 6.01 us | 5.86 us | 2.6% |
| straight | 5 | 2 | 0.150 us | 9.20 us | 9.05 us | 1.6% |
| straight | 1 | 2 | 0.168 us | 6.96 us | 6.79 us | 2.4% |
| right_angle | 30 | 6 | 0.234 us | 6.57 us | 6.34 us | 3.6% |
| right_angle | 10 | 12 | 0.266 us | 7.47 us | 7.20 us | 3.6% |
| right_angle | 5 | 20 | 0.297 us | 11.49 us | 11.19 us | 2.6% |
| right_angle | 1 | 92 | 0.619 us | 20.45 us | 19.83 us | 3.0% |
| hairpin | 30 | 8 | 0.191 us | 6.98 us | 6.79 us | 2.7% |
| hairpin | 10 | 20 | 0.252 us | 9.53 us | 9.28 us | 2.6% |
| hairpin | 5 | 38 | 0.330 us | 15.38 us | 15.05 us | 2.1% |
| hairpin | 1 | 182 | 0.944 us | 40.54 us | 39.60 us | 2.3% |

## `redistribute_lookups`

| band | corners | mode | input_n | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|---|---|
| 2-8 | 0 | 0 | 101 | 367 | 1.711 us | 15.80 us | 14.09 us | 10.8% |
| 2-8 | 0 | 0 | 1001 | 367 | 1.711 us | 16.90 us | 15.19 us | 10.1% |
| 2-8 | 0 | 0 | 10001 | 367 | 1.711 us | 17.50 us | 15.79 us | 9.8% |
| 2-8 | 0 | 0 | 100001 | 367 | 1.711 us | 40.90 us | 39.19 us | 4.2% |
| 8-20 | 0 | 0 | 10001 | 102 | 0.506 us | 19.50 us | 18.99 us | 2.6% |
| 2-8 | 0 | 0 | 10001 | 367 | 1.711 us | 15.60 us | 13.89 us | 11.0% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.729 us | 44.30 us | 37.57 us | 15.2% |
| 0.2-0.8 | 0 | 0 | 10001 | 3657 | 16.701 us | 105.40 us | 88.70 us | 15.8% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.729 us | 45.50 us | 38.77 us | 14.8% |
| 0.5-2 | 10 | 0 | 10001 | 1465 | 6.675 us | 53.40 us | 46.73 us | 12.5% |
| 0.5-2 | 100 | 0 | 10001 | 1462 | 6.438 us | 86.50 us | 80.06 us | 7.4% |
| 0.5-2 | 1000 | 0 | 10001 | 1557 | 5.974 us | 318.80 us | 312.83 us | 1.9% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.729 us | 46.60 us | 39.87 us | 14.4% |
| 0.5-2 | 0 | 1 | 10001 | 714 | 4.962 us | 27.70 us | 22.74 us | 17.9% |

`input_n` is a Python-side axis only -- the native call does not receive it, so one native row is compared against every Python row that varies it.

