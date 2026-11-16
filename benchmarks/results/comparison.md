# native vs Python -- where each call's time goes

`native` is the algorithm alone (`benchmarks/native/bench_core`). `python` is the same call through the wrapper (`benchmarks/python/run.py`): ctypes, marshaling and wrapper included. `bridge` is the difference.

| | |
|---|---|
| Native toolchain | MSVC 1929 |
| Python | 3.10.6 on Windows AMD64 |
| DLL under test | `a168d6ca0191d2cb...` built 2026-11-16T08:25:00 |

## `build_turn_waypoints`

| geometry | theta_max_deg | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|
| straight | 30 | 2 | 0.121 us | 5.81 us | 5.69 us | 2.1% |
| straight | 10 | 2 | 0.133 us | 5.90 us | 5.77 us | 2.2% |
| straight | 5 | 2 | 0.129 us | 6.44 us | 6.31 us | 2.0% |
| straight | 1 | 2 | 0.144 us | 6.26 us | 6.12 us | 2.3% |
| right_angle | 30 | 6 | 0.220 us | 6.38 us | 6.16 us | 3.4% |
| right_angle | 10 | 12 | 0.257 us | 7.43 us | 7.18 us | 3.5% |
| right_angle | 5 | 20 | 0.289 us | 9.43 us | 9.14 us | 3.1% |
| right_angle | 1 | 92 | 0.584 us | 21.23 us | 20.65 us | 2.7% |
| hairpin | 30 | 8 | 0.181 us | 6.99 us | 6.81 us | 2.6% |
| hairpin | 10 | 20 | 0.242 us | 8.77 us | 8.53 us | 2.8% |
| hairpin | 5 | 38 | 0.303 us | 11.73 us | 11.43 us | 2.6% |
| hairpin | 1 | 182 | 0.912 us | 36.02 us | 35.11 us | 2.5% |

## `redistribute_arc_lengths`

| band | corners | mode | input_n | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|---|---|
| 2-8 | 0 | 0 | 101 | 367 | 1.685 us | 13.40 us | 11.72 us | 12.6% |
| 2-8 | 0 | 0 | 1001 | 367 | 1.685 us | 14.00 us | 12.32 us | 12.0% |
| 2-8 | 0 | 0 | 10001 | 367 | 1.685 us | 15.30 us | 13.62 us | 11.0% |
| 2-8 | 0 | 0 | 100001 | 367 | 1.685 us | 39.70 us | 38.02 us | 4.2% |
| 8-20 | 0 | 0 | 10001 | 102 | 0.476 us | 7.70 us | 7.22 us | 6.2% |
| 2-8 | 0 | 0 | 10001 | 367 | 1.685 us | 15.40 us | 13.72 us | 10.9% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.615 us | 45.00 us | 38.38 us | 14.7% |
| 0.2-0.8 | 0 | 0 | 10001 | 3657 | 16.869 us | 110.80 us | 93.93 us | 15.2% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.615 us | 48.10 us | 41.48 us | 13.8% |
| 0.5-2 | 10 | 0 | 10001 | 1465 | 6.849 us | 53.10 us | 46.25 us | 12.9% |
| 0.5-2 | 100 | 0 | 10001 | 1462 | 6.367 us | 75.10 us | 68.73 us | 8.5% |
| 0.5-2 | 1000 | 0 | 10001 | 1557 | 5.981 us | 323.00 us | 317.02 us | 1.9% |
| 0.5-2 | 0 | 0 | 10001 | 1464 | 6.615 us | 47.30 us | 40.68 us | 14.0% |
| 0.5-2 | 0 | 1 | 10001 | 714 | 4.960 us | 27.90 us | 22.94 us | 17.8% |

`input_n` is a Python-side axis only -- the native call does not receive it, so one native row is compared against every Python row that varies it.

## `point_in_curve`

| n | v | shape | where | tol | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|---|---|---|---|
| 1000 | 1000 | circle | uniform | 0 | -- | 4565.930 us | 5091.20 us | 525.27 us | 89.7% |
| 2000 | 1000 | circle | uniform | 0 | -- | -- | 10086.30 us | -- | -- |
| 4000 | 1000 | circle | uniform | 0 | -- | -- | 20319.40 us | -- | -- |
| 8000 | 1000 | circle | uniform | 0 | -- | 35824.450 us | 40812.60 us | 4988.15 us | 87.8% |
| 16000 | 1000 | circle | uniform | 0 | -- | -- | 80033.70 us | -- | -- |
| 32000 | 1000 | circle | uniform | 0 | -- | -- | 157058.00 us | -- | -- |
| 64000 | 1000 | circle | uniform | 0 | -- | 288934.500 us | 314279.30 us | 25344.80 us | 91.9% |
| 8000 | 100 | circle | uniform | 0 | -- | 3986.384 us | 7399.80 us | 3413.42 us | 53.9% |
| 8000 | 250 | circle | uniform | 0 | -- | -- | 12522.00 us | -- | -- |
| 8000 | 1000 | circle | uniform | 0 | -- | 35824.450 us | 39658.90 us | 3834.45 us | 90.3% |
| 8000 | 4000 | circle | uniform | 0 | -- | -- | 146858.70 us | -- | -- |
| 8000 | 16000 | circle | uniform | 0 | -- | 587519.700 us | 589630.60 us | 2110.90 us | 99.6% |
| 8000 | 1000 | circle | uniform | 0 | -- | 35824.450 us | 38940.00 us | 3115.55 us | 92.0% |
| 8000 | 1000 | circle | spread | 0 | -- | 4190.100 us | 7442.00 us | 3251.90 us | 56.3% |
| 8000 | 1000 | circle | inside | 0 | -- | 37238.450 us | 39558.20 us | 2319.75 us | 94.1% |
| 8000 | 1000 | circle | outside | 0 | -- | 64.732 us | 3463.40 us | 3398.67 us | 1.9% |
| 8000 | 1000 | circle | uniform | 0 | -- | 35824.450 us | 39135.30 us | 3310.85 us | 91.5% |
| 8000 | 1000 | comb | uniform | 0 | -- | 38677.700 us | 42853.20 us | 4175.50 us | 90.3% |
| 8000 | 1000 | circle | uniform | 0 | -- | 35824.450 us | 39379.40 us | 3554.95 us | 91.0% |
| 8000 | 1000 | circle | uniform | 1 | -- | -- | 40444.80 us | -- | -- |
| 8000 | 1000 | circle | uniform | 10 | -- | -- | 38333.60 us | -- | -- |
| 8000 | 1000 | circle | uniform | 100 | -- | -- | 34761.70 us | -- | -- |

