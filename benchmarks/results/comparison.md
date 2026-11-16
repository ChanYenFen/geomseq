# native vs Python -- where each call's time goes

`native` is the algorithm alone (`benchmarks/native/bench_core`). `python` is the same call through the wrapper (`benchmarks/python/run.py`): ctypes, marshaling and wrapper included. `bridge` is the difference.

| | |
|---|---|
| Native toolchain | MSVC 1929 |
| Python | 3.10.6 on Windows AMD64 |
| DLL under test | `fd1b56857adbd1cd...` built 2026-11-16T11:54:37 |

## `build_turn_waypoints`

| geometry | theta_max_deg | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|
| straight | 30 | 2 | 0.115 us | 5.68 us | 5.57 us | 2.0% |
| straight | 10 | 2 | 0.126 us | 5.63 us | 5.51 us | 2.2% |
| straight | 5 | 2 | 0.121 us | 6.40 us | 6.28 us | 1.9% |
| straight | 1 | 2 | 0.140 us | 5.94 us | 5.80 us | 2.4% |
| right_angle | 30 | 6 | 0.215 us | 6.57 us | 6.35 us | 3.3% |
| right_angle | 10 | 12 | 0.252 us | 7.38 us | 7.13 us | 3.4% |
| right_angle | 5 | 20 | 0.283 us | 9.09 us | 8.81 us | 3.1% |
| right_angle | 1 | 92 | 0.593 us | 21.22 us | 20.63 us | 2.8% |
| hairpin | 30 | 8 | 0.181 us | 6.54 us | 6.36 us | 2.8% |
| hairpin | 10 | 20 | 0.240 us | 8.63 us | 8.39 us | 2.8% |
| hairpin | 5 | 38 | 0.308 us | 11.92 us | 11.61 us | 2.6% |
| hairpin | 1 | 182 | 0.925 us | 36.06 us | 35.13 us | 2.6% |

## `redistribute_arc_lengths`

| band | corners | mode | input_n | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|---|---|
| 2-8 | 0 | 0 | 101 | 367 | 1.754 us | 14.30 us | 12.55 us | 12.3% |
| 2-8 | 0 | 0 | 1001 | 367 | 1.754 us | 22.70 us | 20.95 us | 7.7% |
| 2-8 | 0 | 0 | 10001 | 367 | 1.754 us | 14.70 us | 12.95 us | 11.9% |
| 2-8 | 0 | 0 | 100001 | 367 | 1.754 us | 24.00 us | 22.25 us | 7.3% |
| 8-20 | 0 | 0 | 10001 | 102 | 0.499 us | 7.60 us | 7.10 us | 6.6% |
| 2-8 | 0 | 0 | 10001 | 367 | 1.754 us | 15.00 us | 13.25 us | 11.7% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.856 us | 44.40 us | 37.54 us | 15.4% |
| 0.2-0.8 | 0 | 0 | 10001 | 3657 | 17.008 us | 102.20 us | 85.19 us | 16.6% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.856 us | 45.40 us | 38.54 us | 15.1% |
| 0.5-2 | 10 | 0 | 10001 | 1465 | 6.667 us | 50.80 us | 44.13 us | 13.1% |
| 0.5-2 | 100 | 0 | 10001 | 1462 | 6.433 us | 73.00 us | 66.57 us | 8.8% |
| 0.5-2 | 1000 | 0 | 10001 | 1557 | 5.822 us | 299.90 us | 294.08 us | 1.9% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.856 us | 44.40 us | 37.54 us | 15.4% |
| 0.5-2 | 0 | 1 | 10001 | 714 | 5.075 us | 25.60 us | 20.52 us | 19.8% |

`input_n` is a Python-side axis only -- the native call does not receive it, so one native row is compared against every Python row that varies it.

## `point_in_curve`

| n | v | shape | where | tol | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|---|---|---|
| 1000 | 1000 | circle | uniform | 0 | -- | 98.755 us | 844.50 us | 745.75 us | 11.7% |
| 2000 | 1000 | circle | uniform | 0 | -- | -- | 1350.90 us | -- | -- |
| 4000 | 1000 | circle | uniform | 0 | -- | -- | 2303.20 us | -- | -- |
| 8000 | 1000 | circle | uniform | 0 | -- | 705.400 us | 4243.20 us | 3537.80 us | 16.6% |
| 16000 | 1000 | circle | uniform | 0 | -- | -- | 8817.20 us | -- | -- |
| 32000 | 1000 | circle | uniform | 0 | -- | -- | 17304.90 us | -- | -- |
| 64000 | 1000 | circle | uniform | 0 | -- | 6991.900 us | 35390.80 us | 28398.90 us | 19.8% |
| 8000 | 100 | circle | uniform | 0 | -- | 698.640 us | 4086.30 us | 3387.66 us | 17.1% |
| 8000 | 250 | circle | uniform | 0 | -- | -- | 4062.00 us | -- | -- |
| 8000 | 1000 | circle | uniform | 0 | -- | 705.400 us | 4780.30 us | 4074.90 us | 14.8% |
| 8000 | 4000 | circle | uniform | 0 | -- | -- | 5678.80 us | -- | -- |
| 8000 | 16000 | circle | uniform | 0 | -- | 1425.100 us | 11698.40 us | 10273.30 us | 12.2% |
| 8000 | 1000 | circle | uniform | 0 | -- | 705.400 us | 4311.60 us | 3606.20 us | 16.4% |
| 8000 | 1000 | circle | spread | 0 | -- | 157.750 us | 3770.60 us | 3612.85 us | 4.2% |
| 8000 | 1000 | circle | inside | 0 | -- | 678.000 us | 4375.40 us | 3697.40 us | 15.5% |
| 8000 | 1000 | circle | outside | 0 | -- | 44.901 us | 3440.00 us | 3395.10 us | 1.3% |
| 8000 | 1000 | circle | uniform | 0 | -- | 705.400 us | 4218.90 us | 3513.50 us | 16.7% |
| 8000 | 1000 | comb | uniform | 0 | -- | 27199.200 us | 30192.10 us | 2992.90 us | 90.1% |
| 8000 | 1000 | circle | uniform | 0 | -- | 705.400 us | 4191.30 us | 3485.90 us | 16.8% |
| 8000 | 1000 | circle | uniform | 1 | -- | -- | 4259.40 us | -- | -- |
| 8000 | 1000 | circle | uniform | 10 | -- | -- | 5300.20 us | -- | -- |
| 8000 | 1000 | circle | uniform | 100 | -- | -- | 8795.40 us | -- | -- |

