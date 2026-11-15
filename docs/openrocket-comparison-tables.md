
# dual_parachute

OpenRocket 23.09 — 1 stage(s); import warnings: 6.

- import: material "Ripstop nylon, lightweight, 3 mil" (0.035653924 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match
- import: material "Spectra #200 [Round 1.5 mm, 1/16 in]" (0.000656168 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: material "Ripstop Nylon, 1.9 oz 5 mil" (0.058906483 kg/m²) is not in the model's table: using Ripstop nylon (0.067) and scaling the part masses to match
- import: material "0555 Nylon webbing #440 [flat 1/2 x 3/64 in (12.7 x 1 mm)]" (0.004090114 kg/m) is not in the model's table: using Nylon elastic cord (flat 6 mm, 1/4 in) (0.0045) and scaling the part masses to match
- import: material "Delrin" (1420 kg/m³) is not in the model's table: using Blue tube (1300) and scaling the part masses to match
- import: material "Fiberglass" (40.827859344 kg/m³) is not in the model's table: using Balsa (170) and scaling the part masses to match

## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [H669-P] | 1.613 | 1.613 | -0.0% | 0.9629 | 0.9640 | 0.019 | 1.517 | 1.517 | 0.9344 | 0.9355 |
| [H242-P] | 1.626 | 1.626 | -0.0% | 0.9666 | 0.9675 | 0.016 | 1.511 | 1.511 | 0.9326 | 0.9337 |
| [J570-P] | 2.247 | 2.247 | -0.0% | 1.0257 | 1.0264 | 0.014 | 1.699 | 1.699 | 0.9532 | 0.9543 |
| [H999-P] | 1.692 | 1.692 | -0.0% | 0.9790 | 0.9800 | 0.018 | 1.548 | 1.548 | 0.9409 | 0.9420 |
| [I1299-P] | 1.783 | 1.783 | -0.0% | 0.9944 | 0.9954 | 0.017 | 1.591 | 1.591 | 0.9497 | 0.9508 |
| [G64-P] | 1.512 | 1.512 | -0.0% | 0.9343 | 0.9354 | 0.020 | 1.449 | 1.449 | 0.9130 | 0.9142 |

## Static aerodynamics, α = 0° (configuration [H669-P], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 1.2037 | 1.2038 | +0.001 | 14.631 | 14.631 | -0.0% | 0.8018 | 0.8051 | +0.4% | 0.514/0.167/0.121 | 0.684/0.000/0.121 |
| 0.30 | 1.2047 | 1.2047 | +0.001 | 14.711 | 14.711 | +0.0% | 0.8148 | 0.8183 | +0.4% | 0.509/0.174/0.132 | 0.687/0.000/0.132 |
| 0.50 | 1.2066 | 1.2066 | +0.001 | 14.879 | 14.879 | -0.0% | 0.8388 | 0.8424 | +0.4% | 0.501/0.185/0.152 | 0.690/0.000/0.152 |
| 0.80 | 1.2161 | 1.2161 | +0.001 | 15.327 | 15.327 | -0.0% | 0.8969 | 0.9052 | +0.9% | 0.481/0.212/0.203 | 0.697/0.005/0.203 |
| 0.90 | 1.2208 | 1.2209 | +0.001 | 15.543 | 15.543 | -0.0% | 0.9224 | 0.9495 | +2.9% | 0.472/0.225/0.225 | 0.701/0.023/0.225 |
| 1.00 | 1.2260 | 1.2258 | -0.004 | 15.802 | 15.778 | -0.2% | 0.9359 | 1.0391 | +11.0% | 0.447/0.239/0.250 | 0.690/0.100/0.250 |
| 1.20 | 1.2347 | 1.2336 | -0.019 | 16.230 | 16.113 | -0.7% | 0.9030 | 0.9958 | +10.3% | 0.408/0.286/0.208 | 0.662/0.126/0.208 |
| 1.50 | 1.2198 | 1.2198 | +0.001 | 14.443 | 14.443 | -0.0% | 0.7947 | 0.8645 | +8.8% | 0.366/0.262/0.167 | 0.633/0.065/0.167 |
| 2.00 | 1.1457 | 1.1457 | +0.000 | 10.032 | 10.032 | -0.0% | 0.7008 | 0.7555 | +7.8% | 0.299/0.277/0.125 | 0.581/0.050/0.125 |

## Static aerodynamics, α = 2° (configuration [H669-P], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 1.1721 | 1.1721 | +0.001 | 15.823 | 15.823 | -0.0% | 0.8110 | 0.8144 | +0.4% | 0.514/0.167/0.121 | 0.692/0.000/0.123 |
| 0.30 | 1.1731 | 1.1732 | +0.001 | 15.904 | 15.904 | -0.0% | 0.8241 | 0.8277 | +0.4% | 0.509/0.174/0.132 | 0.694/0.000/0.133 |
| 0.50 | 1.1752 | 1.1753 | +0.001 | 16.072 | 16.072 | -0.0% | 0.8484 | 0.8521 | +0.4% | 0.501/0.185/0.152 | 0.698/0.000/0.154 |
| 0.80 | 1.1849 | 1.1849 | +0.001 | 16.520 | 16.520 | -0.0% | 0.9072 | 0.9156 | +0.9% | 0.481/0.212/0.203 | 0.705/0.005/0.206 |
| 0.90 | 1.1897 | 1.1898 | +0.001 | 16.736 | 16.736 | -0.0% | 0.9330 | 0.9604 | +2.9% | 0.472/0.225/0.225 | 0.709/0.023/0.228 |
| 1.00 | 1.1951 | 1.1948 | -0.004 | 17.000 | 16.976 | -0.1% | 0.9466 | 1.0511 | +11.0% | 0.447/0.239/0.250 | 0.698/0.101/0.253 |
| 1.20 | 1.2049 | 1.2037 | -0.022 | 17.524 | 17.406 | -0.7% | 0.9134 | 1.0072 | +10.3% | 0.408/0.286/0.208 | 0.669/0.127/0.211 |
| 1.50 | 1.1907 | 1.1907 | +0.000 | 15.957 | 15.957 | -0.0% | 0.8039 | 0.8744 | +8.8% | 0.366/0.262/0.167 | 0.640/0.066/0.169 |
| 2.00 | 1.1120 | 1.1120 | +0.000 | 11.412 | 11.412 | -0.0% | 0.7089 | 0.7641 | +7.8% | 0.299/0.277/0.125 | 0.588/0.050/0.126 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.4141 \| 0.4180 | 0.0010 \| 0.0000 | 0.1317 \| 0.1317 | 0.0878 \| 0.2633 | 0.0023 \| 0.0052 | 0.6369 \| 0.8183 |
| 0.90 | 0.3840 \| 0.3876 | 0.0010 \| 0.0232 | 0.2253 \| 0.2253 | 0.1011 \| 0.3033 | 0.0044 \| 0.0100 | 0.7158 \| 0.9495 |
| 1.00 | 0.3636 \| 0.3670 | 0.0010 \| 0.0995 | 0.2500 \| 0.2500 | 0.1038 \| 0.3115 | 0.0049 \| 0.0111 | 0.7233 \| 1.0391 |
| 1.20 | 0.3318 \| 0.3350 | 0.0378 \| 0.1258 | 0.2083 \| 0.2083 | 0.1043 \| 0.3129 | 0.0061 \| 0.0138 | 0.6883 \| 0.9958 |
| 2.00 | 0.2429 \| 0.2452 | 0.0003 \| 0.0495 | 0.1250 \| 0.1250 | 0.1032 \| 0.3097 | 0.0115 \| 0.0260 | 0.4829 \| 0.7555 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Simulation 1 | apogee (m) | 593.509 | 592.449 | -0.2% |
| Simulation 1 | apogee (m), BS without the exhaust base-drag credit | 593.509 | 591.705 | -0.3% |
| Simulation 1 | max velocity (m/s) | 135.494 | 135.553 | +0.0% |
| Simulation 1 | max velocity (m/s), BS without the credit | 135.494 | 135.421 | -0.1% |
| Simulation 1 | max Mach | 0.398 | 0.398 | +0.1% |
| Simulation 1 | time to apogee (s) | 10.329 | 10.308 | -0.2% |
| Simulation 1 | rail-exit velocity (m/s, OR at 2 ms step) | 52.942 | 53.306 | +0.7% |
| Simulation 1 | stability off rail (cal) | 4.465 | 4.428 | -0.8% |
| Simulation 1 | ground-hit velocity (m/s) | 4.171 | 4.187 | +0.4% |
| Simulation 1 | flight time (s) | 61.761 | 64.839 | +5.0% |
| Simulation 2 | apogee (m) | 698.305 | 700.826 | +0.4% |
| Simulation 2 | apogee (m), BS without the exhaust base-drag credit | 698.305 | 696.623 | -0.2% |
| Simulation 2 | max velocity (m/s) | 140.840 | 141.508 | +0.5% |
| Simulation 2 | max velocity (m/s), BS without the credit | 140.840 | 140.977 | +0.1% |
| Simulation 2 | max Mach | 0.414 | 0.416 | +0.5% |
| Simulation 2 | time to apogee (s) | 11.440 | 11.455 | +0.1% |
| Simulation 2 | rail-exit velocity (m/s, OR at 2 ms step) | 29.138 | 29.216 | +0.3% |
| Simulation 2 | stability off rail (cal) | 4.309 | 4.284 | -0.6% |
| Simulation 2 | ground-hit velocity (m/s) | 4.198 | 4.179 | -0.5% |
| Simulation 2 | flight time (s) | 68.955 | 70.583 | +2.4% |
| Simulation 3 | apogee (m) | 2224.990 | 2214.532 | -0.5% |
| Simulation 3 | apogee (m), BS without the exhaust base-drag credit | 2224.990 | 2169.502 | -2.5% |
| Simulation 3 | max velocity (m/s) | 388.574 | 390.571 | +0.5% |
| Simulation 3 | max velocity (m/s), BS without the credit | 388.574 | 382.980 | -1.4% |
| Simulation 3 | max Mach | 1.147 | 1.153 | +0.5% |
| Simulation 3 | time to apogee (s) | 17.382 | 17.313 | -0.4% |
| Simulation 3 | rail-exit velocity (m/s, OR at 2 ms step) | 53.757 | 54.013 | +0.5% |
| Simulation 3 | stability off rail (cal) | 3.276 | 3.252 | -0.7% |
| Simulation 3 | ground-hit velocity (m/s) | 4.466 | 4.432 | -0.8% |
| Simulation 3 | flight time (s) | 129.657 | 130.304 | +0.5% |
| Simulation 4 | apogee (m) | 898.191 | 896.780 | -0.2% |
| Simulation 4 | apogee (m), BS without the exhaust base-drag credit | 898.191 | 895.379 | -0.3% |
| Simulation 4 | max velocity (m/s) | 190.929 | 191.125 | +0.1% |
| Simulation 4 | max velocity (m/s), BS without the credit | 190.929 | 190.846 | -0.0% |
| Simulation 4 | max Mach | 0.561 | 0.562 | +0.1% |
| Simulation 4 | time to apogee (s) | 12.219 | 12.188 | -0.2% |
| Simulation 4 | rail-exit velocity (m/s, OR at 2 ms step) | 62.646 | 63.161 | +0.8% |
| Simulation 4 | stability off rail (cal) | 4.207 | 4.167 | -1.0% |
| Simulation 4 | ground-hit velocity (m/s) | 4.139 | 4.230 | +2.2% |
| Simulation 4 | flight time (s) | 76.647 | 78.657 | +2.6% |
| Simulation 5 | apogee (m) | 1159.623 | 1157.854 | -0.2% |
| Simulation 5 | apogee (m), BS without the exhaust base-drag credit | 1159.623 | 1155.510 | -0.4% |
| Simulation 5 | max velocity (m/s) | 242.545 | 243.227 | +0.3% |
| Simulation 5 | max velocity (m/s), BS without the credit | 242.545 | 242.739 | +0.1% |
| Simulation 5 | max Mach | 0.713 | 0.715 | +0.3% |
| Simulation 5 | time to apogee (s) | 13.462 | 13.446 | -0.1% |
| Simulation 5 | rail-exit velocity (m/s, OR at 2 ms step) | 69.387 | 69.670 | +0.4% |
| Simulation 5 | stability off rail (cal) | 3.940 | 3.898 | -1.1% |
| Simulation 5 | ground-hit velocity (m/s) | 4.291 | 4.288 | -0.1% |
| Simulation 5 | flight time (s) | 87.043 | 89.403 | +2.7% |
| Simulation 6 | apogee (m) | 227.408 | 227.820 | +0.2% |
| Simulation 6 | apogee (m), BS without the exhaust base-drag credit | 227.408 | 226.950 | -0.2% |
| Simulation 6 | max velocity (m/s) | 58.536 | 58.601 | +0.1% |
| Simulation 6 | max velocity (m/s), BS without the credit | 58.536 | 58.509 | -0.0% |
| Simulation 6 | max Mach | 0.172 | 0.172 | +0.1% |
| Simulation 6 | time to apogee (s) | 7.310 | 7.337 | +0.4% |
| Simulation 6 | rail-exit velocity (m/s, OR at 2 ms step) | 18.911 | 18.916 | +0.0% |
| Simulation 6 | stability off rail (cal) | 4.865 | 4.840 | -0.5% |
| Simulation 6 | ground-hit velocity (m/s) | 4.080 | 4.093 | +0.3% |
| Simulation 6 | flight time (s) | 45.225 | 47.387 | +4.8% |

# pods_powered

OpenRocket 23.09 — 2 stage(s); import warnings: 18.

- import: material "Paper, spiral kraft glassine, Estes avg, bulk" (894.4 kg/m³) is not in the model's table: using Kraft phenolic (950) and scaling the part masses to match
- import: material "Polyethylene film, HDPE, 1.0 mil, bare" (0.0235 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match
- import: material "Carpet Thread" (0.00033 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: material "Nylon" (212.73036595434928 kg/m³) is not in the model's table: using Balsa (170) and scaling the part masses to match
- import: material "Fiber, bulk" (657 kg/m³) is not in the model's table: using Cardboard (680) and scaling the part masses to match
- import: material "Paper, spiral kraft glassine, Estes avg, bulk" (894.4 kg/m³) is not in the model's table: using Kraft phenolic (950) and scaling the part masses to match
- import: material "Polyethylene film, HDPE, 1.0 mil, bare" (0.0235 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match
- import: material "Carpet Thread" (0.00033 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: Parachute, plastic, preassembled, 10 in., PN 2262: deployment "never" is not modelled; it deploys 1e6 s after launch
- import: material "Elastic rubber band (flat 3.2 mm, 1/8 in)" (0.00297638 kg/m) is not in the model's table: using Kevlar line (1/8 in) (0.0035) and scaling the part masses to match
- import: Side Motor Tube: the body tube is its own motor mount; imported as a massless inner tube of its bore (13.1 mm)
- import: material "Paper, spiral kraft glassine, Estes avg, bulk" (894.4 kg/m³) is not in the model's table: using Kraft phenolic (950) and scaling the part masses to match
- import: material "Polyethylene film, HDPE, 1.0 mil, bare" (0.0235 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match
- import: material "Carpet Thread" (0.00033 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: material "Elastic rubber band (flat 3.2 mm, 1/8 in)" (0.00297638 kg/m) is not in the model's table: using Kevlar line (1/8 in) (0.0035) and scaling the part masses to match
- import: material "Pine" (530 kg/m³) is not in the model's table: using Basswood (500) and scaling the part masses to match
- import: Sustainer Motor Tube: its motor reaches 6.4 mm past a stage joint; the motor was moved that far forward so the stage separates at the joint
- import: stage 1 separation "never" is not modelled; it separates at burnout

## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [C6-7; 217A3-4,B6-0] | 0.148 | 0.150 | +1.6% | 0.5708 | 0.5716 | 0.022 | 0.125 | 0.127 | 0.5407 | 0.5427 |
| [C6-7; B6-0] | 0.131 | 0.133 | +1.8% | 0.5430 | 0.5443 | 0.039 | 0.114 | 0.117 | 0.5185 | 0.5211 |
| [C6-5; None] | 0.112 | 0.115 | +2.0% | 0.5050 | 0.5073 | 0.068 | 0.102 | 0.104 | 0.4864 | 0.4900 |
| [None; 217A10-3,B6-4] | 0.118 | 0.120 | +2.0% | 0.5376 | 0.5404 | 0.082 | 0.105 | 0.107 | 0.5069 | 0.5107 |

## Static aerodynamics, α = 0° (configuration [C6-7; 217A3-4,B6-0], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.6664 | 0.6665 | +0.005 | 26.015 | 26.015 | +0.0% | 1.2567 | 1.2577 | +0.1% | 0.656/0.521/0.080 | 1.097/0.082/0.078 |
| 0.30 | 0.6669 | 0.6671 | +0.005 | 26.296 | 26.296 | +0.0% | 1.2810 | 1.2820 | +0.1% | 0.651/0.544/0.086 | 1.107/0.089/0.085 |
| 0.50 | 0.6680 | 0.6681 | +0.005 | 26.898 | 26.898 | +0.0% | 1.3294 | 1.3304 | +0.1% | 0.640/0.589/0.100 | 1.128/0.104/0.099 |
| 0.80 | 0.5339 | 0.6732 | +4.091 | 28.652 | 28.652 | -0.0% | 1.4491 | 1.4570 | +0.5% | 0.614/0.701/0.133 | 1.180/0.146/0.131 |
| 0.90 | 0.4644 | 0.6757 | +6.209 | 29.590 | 29.590 | +0.0% | 1.5020 | 1.5338 | +2.1% | 0.603/0.751/0.148 | 1.203/0.185/0.146 |
| 1.00 | 0.3816 | 0.6780 | +8.710 | 30.753 | 30.643 | -0.4% | 1.5421 | 1.6721 | +8.4% | 0.571/0.807/0.164 | 1.210/0.300/0.162 |
| 1.20 | -0.0491 | 0.6770 | +21.334 | 32.730 | 32.196 | -1.6% | 1.4954 | 1.6121 | +7.8% | 0.521/0.837/0.137 | 1.170/0.308/0.135 |
| 1.50 | -0.6342 | 0.6627 | +38.102 | 28.840 | 28.840 | +0.0% | 1.3481 | 1.4344 | +6.4% | 0.467/0.771/0.109 | 1.127/0.200/0.108 |
| 2.00 | 0.6909 | 0.6412 | -1.461 | 19.075 | 19.075 | +0.0% | 1.2152 | 1.2805 | +5.4% | 0.382/0.751/0.082 | 1.049/0.151/0.081 |

## Static aerodynamics, α = 2° (configuration [C6-7; 217A3-4,B6-0], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.6577 | 0.6578 | +0.004 | 27.228 | 27.218 | -0.0% | 1.2711 | 1.2722 | +0.1% | 0.656/0.521/0.080 | 1.109/0.083/0.079 |
| 0.30 | 0.6583 | 0.6584 | +0.004 | 27.509 | 27.499 | -0.0% | 1.2957 | 1.2967 | +0.1% | 0.651/0.544/0.086 | 1.120/0.090/0.086 |
| 0.50 | 0.6595 | 0.6596 | +0.004 | 28.111 | 28.101 | -0.0% | 1.3447 | 1.3456 | +0.1% | 0.640/0.589/0.100 | 1.141/0.105/0.100 |
| 0.80 | 0.5314 | 0.6650 | +3.924 | 29.865 | 29.855 | -0.0% | 1.4657 | 1.4737 | +0.5% | 0.614/0.701/0.133 | 1.194/0.147/0.133 |
| 0.90 | 0.4647 | 0.6677 | +5.964 | 30.804 | 30.793 | -0.0% | 1.5193 | 1.5515 | +2.1% | 0.603/0.751/0.148 | 1.217/0.187/0.147 |
| 1.00 | 0.3847 | 0.6702 | +8.388 | 31.977 | 31.857 | -0.4% | 1.5598 | 1.6913 | +8.4% | 0.571/0.807/0.164 | 1.224/0.304/0.164 |
| 1.20 | -0.0359 | 0.6698 | +20.734 | 34.165 | 33.621 | -1.6% | 1.5126 | 1.6306 | +7.8% | 0.521/0.837/0.137 | 1.183/0.311/0.136 |
| 1.50 | -0.5909 | 0.6562 | +36.640 | 30.764 | 30.754 | -0.0% | 1.3635 | 1.4508 | +6.4% | 0.467/0.771/0.109 | 1.140/0.202/0.109 |
| 2.00 | 0.6796 | 0.6326 | -1.380 | 20.703 | 20.692 | -0.0% | 1.2291 | 1.2952 | +5.4% | 0.382/0.751/0.082 | 1.061/0.153/0.082 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.4772 \| 0.4801 | 0.0950 \| 0.0894 | 0.0858 \| 0.0852 | 0.3163 \| 0.6260 | 0.0010 \| 0.0013 | 0.9753 \| 1.2820 |
| 0.90 | 0.4425 \| 0.4453 | 0.1594 \| 0.1847 | 0.1468 \| 0.1457 | 0.3795 \| 0.7558 | 0.0019 \| 0.0024 | 1.1301 \| 1.5338 |
| 1.00 | 0.4190 \| 0.4216 | 0.1764 \| 0.3001 | 0.1629 \| 0.1617 | 0.3936 \| 0.7862 | 0.0021 \| 0.0027 | 1.1539 \| 1.6721 |
| 1.20 | 0.3824 \| 0.3848 | 0.1974 \| 0.3077 | 0.1358 \| 0.1348 | 0.3855 \| 0.7817 | 0.0025 \| 0.0032 | 1.1036 \| 1.6121 |
| 2.00 | 0.2799 \| 0.2817 | 0.0922 \| 0.1510 | 0.0815 \| 0.0809 | 0.3626 \| 0.7616 | 0.0042 \| 0.0053 | 0.8204 \| 1.2805 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Powered Pods with Recovery Devices | apogee (m) | 112.030 | 338.104 | +201.8% |
| Powered Pods with Recovery Devices | apogee (m), BS without the exhaust base-drag credit | 112.030 | 333.469 | +197.7% |
| Powered Pods with Recovery Devices | max velocity (m/s) | 79.943 | 90.310 | +13.0% |
| Powered Pods with Recovery Devices | max velocity (m/s), BS without the credit | 79.943 | 88.746 | +11.0% |
| Powered Pods with Recovery Devices | max Mach | 0.235 | 0.266 | +13.1% |
| Powered Pods with Recovery Devices | time to apogee (s) | 2.314 | 8.034 | +247.2% |
| Powered Pods with Recovery Devices | rail-exit velocity (m/s, OR at 2 ms step) | 5.056 | 5.024 | -0.6% |
| Powered Pods with Recovery Devices | stability off rail (cal) | 2.868 | 2.841 | -0.9% |
| Powered Pods with Recovery Devices | ground-hit velocity (m/s) | 4.658 | 4.608 | -1.1% |
| Powered Pods with Recovery Devices | flight time (s) | 20.984 | 78.435 | +273.8% |
| Unpowered Pods | apogee (m) | 95.951 | 277.869 | +189.6% |
| Unpowered Pods | apogee (m), BS without the exhaust base-drag credit | 95.951 | 274.579 | +186.2% |
| Unpowered Pods | max velocity (m/s) | 71.581 | 82.338 | +15.0% |
| Unpowered Pods | max velocity (m/s), BS without the credit | 71.581 | 81.188 | +13.4% |
| Unpowered Pods | max Mach | 0.210 | 0.242 | +15.1% |
| Unpowered Pods | time to apogee (s) | 2.531 | 7.698 | +204.2% |
| Unpowered Pods | rail-exit velocity (m/s, OR at 2 ms step) | 15.441 | 15.227 | -1.4% |
| Unpowered Pods | stability off rail (cal) | 3.805 | 3.763 | -1.1% |
| Unpowered Pods | ground-hit velocity (m/s) | 4.544 | 4.608 | +1.4% |
| Unpowered Pods | flight time (s) | 17.980 | 64.608 | +259.3% |
| Sustainer Only [Turn Off Booster] | apogee (m) | 130.994 | 210.311 | +60.5% |
| Sustainer Only [Turn Off Booster] | apogee (m), BS without the exhaust base-drag credit | 130.994 | 208.284 | +59.0% |
| Sustainer Only [Turn Off Booster] | max velocity (m/s) | 50.423 | 71.494 | +41.8% |
| Sustainer Only [Turn Off Booster] | max velocity (m/s), BS without the credit | 50.423 | 70.769 | +40.4% |
| Sustainer Only [Turn Off Booster] | max Mach | 0.148 | 0.210 | +41.9% |
| Sustainer Only [Turn Off Booster] | time to apogee (s) | 5.417 | 6.540 | +20.7% |
| Sustainer Only [Turn Off Booster] | rail-exit velocity (m/s, OR at 2 ms step) | 17.874 | 21.772 | +21.8% |
| Sustainer Only [Turn Off Booster] | stability off rail (cal) | 4.883 | 1.970 | -59.7% |
| Sustainer Only [Turn Off Booster] | ground-hit velocity (m/s) | 5.278 | 4.608 | -12.7% |
| Sustainer Only [Turn Off Booster] | flight time (s) | 29.744 | 51.908 | +74.5% |
| BoosterOnly [Turn off Sustainer] | apogee (m) | 115.636 | 118.721 | +2.7% |
| BoosterOnly [Turn off Sustainer] | apogee (m), BS without the exhaust base-drag credit | 115.636 | 118.122 | +2.2% |
| BoosterOnly [Turn off Sustainer] | max velocity (m/s) | 52.688 | 51.849 | -1.6% |
| BoosterOnly [Turn off Sustainer] | max velocity (m/s), BS without the credit | 52.688 | 51.681 | -1.9% |
| BoosterOnly [Turn off Sustainer] | max Mach | 0.155 | 0.152 | -1.6% |
| BoosterOnly [Turn off Sustainer] | time to apogee (s) | 4.697 | 4.810 | +2.4% |
| BoosterOnly [Turn off Sustainer] | rail-exit velocity (m/s, OR at 2 ms step) | 31.062 | 30.524 | -1.7% |
| BoosterOnly [Turn off Sustainer] | stability off rail (cal) | 4.226 | 4.100 | -3.0% |
| BoosterOnly [Turn off Sustainer] | ground-hit velocity (m/s) | 4.479 | 36.016 | +704.0% |
| BoosterOnly [Turn off Sustainer] | flight time (s) | 30.161 | 10.281 | -65.9% |

# pods_winglets

OpenRocket 23.09 — 1 stage(s); import warnings: 8.

- import: material "Paper, spiral kraft glassine, Estes avg, bulk" (894.4 kg/m³) is not in the model's table: using Kraft phenolic (950) and scaling the part masses to match
- import: material "Polyethylene film, HDPE, 1.0 mil, bare" (0.0235 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match
- import: material "Carpet Thread" (0.00033 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: material "Elastic braided cord (flat 3.2 mm, 1/8 in)" (0.00106 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: material "Paper, spiral kraft glassine, Estes avg, bulk" (894.4 kg/m³) is not in the model's table: using Kraft phenolic (950) and scaling the part masses to match
- import: Pod tail: a flipped nose cone (tail cone) is imported as a conical transition to a point
- import: material "Balsa, bulk, 7lb/ft3" (112 kg/m³) is not in the model's table: using Balsa (170) and scaling the part masses to match
- import: material "Fiber, bulk" (657 kg/m³) is not in the model's table: using Cardboard (680) and scaling the part masses to match

## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [A8-3] | 0.073 | 0.072 | -0.9% | 0.2506 | 0.2494 | -0.034 | 0.069 | 0.068 | 0.2415 | 0.2402 |
| [B6-4] | 0.077 | 0.076 | -0.8% | 0.2597 | 0.2587 | -0.030 | 0.071 | 0.071 | 0.2477 | 0.2465 |
| [C6-5] | 0.082 | 0.081 | -0.8% | 0.2688 | 0.2680 | -0.026 | 0.071 | 0.070 | 0.2470 | 0.2457 |
| [C12-6] | 0.082 | 0.082 | -0.8% | 0.2687 | 0.2678 | -0.026 | 0.072 | 0.071 | 0.2485 | 0.2473 |
| [D16-6] | 0.084 | 0.084 | -0.7% | 0.2720 | 0.2711 | -0.025 | 0.072 | 0.071 | 0.2483 | 0.2471 |

## Static aerodynamics, α = 0° (configuration [A8-3], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.3421 | 0.3425 | +0.011 | 22.835 | 22.853 | +0.1% | 0.7965 | 0.7935 | -0.4% | 0.430/0.245/0.121 | 0.632/0.040/0.121 |
| 0.30 | 0.3424 | 0.3427 | +0.011 | 23.070 | 23.087 | +0.1% | 0.8183 | 0.8115 | -0.8% | 0.427/0.260/0.132 | 0.636/0.044/0.132 |
| 0.50 | 0.3430 | 0.3433 | +0.011 | 23.572 | 23.587 | +0.1% | 0.8652 | 0.8509 | -1.7% | 0.420/0.293/0.152 | 0.645/0.054/0.152 |
| 0.80 | 0.3478 | 0.3483 | +0.015 | 25.022 | 25.032 | +0.0% | 1.0045 | 0.9798 | -2.5% | 0.403/0.398/0.203 | 0.670/0.106/0.203 |
| 0.90 | 0.3504 | 0.3509 | +0.017 | 25.789 | 25.794 | +0.0% | 1.1024 | 1.0746 | -2.5% | 0.396/0.481/0.225 | 0.685/0.164/0.225 |
| 1.00 | 0.3529 | 0.3535 | +0.017 | 26.637 | 26.549 | -0.3% | 1.1109 | 1.2232 | +10.1% | 0.375/0.486/0.250 | 0.680/0.293/0.250 |
| 1.20 | 0.3558 | 0.3562 | +0.011 | 26.533 | 26.096 | -1.6% | 1.0861 | 1.1866 | +9.2% | 0.342/0.536/0.208 | 0.646/0.332/0.208 |
| 1.50 | 0.3502 | 0.3511 | +0.027 | 20.028 | 20.028 | +0.0% | 0.9510 | 1.0053 | +5.7% | 0.306/0.478/0.167 | 0.610/0.228/0.167 |
| 2.00 | 0.3360 | 0.3368 | +0.023 | 13.637 | 13.637 | +0.0% | 0.8443 | 0.8579 | +1.6% | 0.250/0.469/0.125 | 0.552/0.181/0.125 |

## Static aerodynamics, α = 2° (configuration [A8-3], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.3398 | 0.3402 | +0.010 | 23.487 | 23.501 | +0.1% | 0.8056 | 0.8026 | -0.4% | 0.430/0.245/0.121 | 0.639/0.041/0.123 |
| 0.30 | 0.3401 | 0.3405 | +0.010 | 23.722 | 23.735 | +0.1% | 0.8277 | 0.8208 | -0.8% | 0.427/0.260/0.132 | 0.643/0.045/0.133 |
| 0.50 | 0.3407 | 0.3411 | +0.010 | 24.224 | 24.235 | +0.0% | 0.8752 | 0.8606 | -1.7% | 0.420/0.293/0.152 | 0.652/0.054/0.154 |
| 0.80 | 0.3456 | 0.3461 | +0.014 | 25.674 | 25.679 | +0.0% | 1.0161 | 0.9910 | -2.5% | 0.403/0.398/0.203 | 0.678/0.108/0.206 |
| 0.90 | 0.3481 | 0.3487 | +0.016 | 26.441 | 26.442 | +0.0% | 1.1150 | 1.0869 | -2.5% | 0.396/0.481/0.225 | 0.693/0.166/0.228 |
| 1.00 | 0.3507 | 0.3513 | +0.016 | 27.296 | 27.204 | -0.3% | 1.1236 | 1.2372 | +10.1% | 0.375/0.486/0.250 | 0.688/0.296/0.253 |
| 1.20 | 0.3537 | 0.3540 | +0.009 | 27.330 | 26.890 | -1.6% | 1.0986 | 1.2002 | +9.2% | 0.342/0.536/0.208 | 0.654/0.336/0.211 |
| 1.50 | 0.3481 | 0.3490 | +0.026 | 21.145 | 21.141 | -0.0% | 0.9619 | 1.0168 | +5.7% | 0.306/0.478/0.167 | 0.617/0.231/0.169 |
| 2.00 | 0.3335 | 0.3342 | +0.021 | 14.561 | 14.556 | -0.0% | 0.8540 | 0.8677 | +1.6% | 0.250/0.469/0.125 | 0.558/0.183/0.126 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.2744 \| 0.2955 | 0.0236 \| 0.0441 | 0.1317 \| 0.1317 | 0.2433 \| 0.3281 | 0.0107 \| 0.0121 | 0.6836 \| 0.8115 |
| 0.90 | 0.2545 \| 0.2740 | 0.0392 \| 0.1644 | 0.2253 \| 0.2253 | 0.3496 \| 0.3979 | 0.0110 \| 0.0130 | 0.8796 \| 1.0746 |
| 1.00 | 0.2409 \| 0.2594 | 0.0433 \| 0.2927 | 0.2500 \| 0.2500 | 0.3483 \| 0.4077 | 0.0112 \| 0.0133 | 0.8937 \| 1.2232 |
| 1.20 | 0.2199 \| 0.2368 | 0.0792 \| 0.3318 | 0.2083 \| 0.2083 | 0.3469 \| 0.3961 | 0.0154 \| 0.0136 | 0.8697 \| 1.1866 |
| 2.00 | 0.1610 \| 0.1733 | 0.0213 \| 0.1813 | 0.1250 \| 0.1250 | 0.3376 \| 0.3641 | 0.0081 \| 0.0141 | 0.6530 \| 0.8579 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Simulation 1 | apogee (m) | 32.217 | 32.707 | +1.5% |
| Simulation 1 | apogee (m), BS without the exhaust base-drag credit | 32.217 | 32.654 | +1.4% |
| Simulation 1 | max velocity (m/s) | 24.886 | 25.154 | +1.1% |
| Simulation 1 | max velocity (m/s), BS without the credit | 24.886 | 25.126 | +1.0% |
| Simulation 1 | max Mach | 0.073 | 0.074 | +1.1% |
| Simulation 1 | time to apogee (s) | 2.814 | 2.815 | +0.0% |
| Simulation 1 | rail-exit velocity (m/s, OR at 2 ms step) | 15.080 | 15.215 | +0.9% |
| Simulation 1 | stability off rail (cal) | 2.884 | 2.922 | +1.3% |
| Simulation 1 | ground-hit velocity (m/s) | 4.286 | 4.323 | +0.9% |
| Simulation 1 | flight time (s) | 10.252 | 10.120 | -1.3% |
| Simulation 2 | apogee (m) | 90.589 | 91.770 | +1.3% |
| Simulation 2 | apogee (m), BS without the exhaust base-drag credit | 90.589 | 91.337 | +0.8% |
| Simulation 2 | max velocity (m/s) | 45.701 | 46.321 | +1.4% |
| Simulation 2 | max velocity (m/s), BS without the credit | 45.701 | 46.147 | +1.0% |
| Simulation 2 | max Mach | 0.134 | 0.136 | +1.4% |
| Simulation 2 | time to apogee (s) | 4.329 | 4.348 | +0.4% |
| Simulation 2 | rail-exit velocity (m/s, OR at 2 ms step) | 17.008 | 17.103 | +0.6% |
| Simulation 2 | stability off rail (cal) | 2.570 | 2.604 | +1.3% |
| Simulation 2 | ground-hit velocity (m/s) | 4.355 | 4.404 | +1.1% |
| Simulation 2 | flight time (s) | 24.981 | 25.055 | +0.3% |
| Simulation 3 | apogee (m) | 199.750 | 202.724 | +1.5% |
| Simulation 3 | apogee (m), BS without the exhaust base-drag credit | 199.750 | 200.804 | +0.5% |
| Simulation 3 | max velocity (m/s) | 72.072 | 73.345 | +1.8% |
| Simulation 3 | max velocity (m/s), BS without the credit | 72.072 | 72.548 | +0.7% |
| Simulation 3 | max Mach | 0.212 | 0.216 | +1.8% |
| Simulation 3 | time to apogee (s) | 6.212 | 6.228 | +0.3% |
| Simulation 3 | rail-exit velocity (m/s, OR at 2 ms step) | 17.423 | 17.548 | +0.7% |
| Simulation 3 | stability off rail (cal) | 2.286 | 2.318 | +1.4% |
| Simulation 3 | ground-hit velocity (m/s) | 4.353 | 4.394 | +0.9% |
| Simulation 3 | flight time (s) | 51.415 | 51.985 | +1.1% |
| Simulation 4 | apogee (m) | 207.522 | 209.840 | +1.1% |
| Simulation 4 | apogee (m), BS without the exhaust base-drag credit | 207.522 | 208.433 | +0.4% |
| Simulation 4 | max velocity (m/s) | 102.743 | 104.100 | +1.3% |
| Simulation 4 | max velocity (m/s), BS without the credit | 102.743 | 103.508 | +0.7% |
| Simulation 4 | max Mach | 0.302 | 0.306 | +1.3% |
| Simulation 4 | time to apogee (s) | 5.721 | 5.749 | +0.5% |
| Simulation 4 | rail-exit velocity (m/s, OR at 2 ms step) | 16.928 | 17.064 | +0.8% |
| Simulation 4 | stability off rail (cal) | 2.276 | 2.302 | +1.1% |
| Simulation 4 | ground-hit velocity (m/s) | 4.428 | 4.425 | -0.1% |
| Simulation 4 | flight time (s) | 51.903 | 52.148 | +0.5% |
| Simulation 5 | apogee (m) | 245.702 | 248.568 | +1.2% |
| Simulation 5 | apogee (m), BS without the exhaust base-drag credit | 245.702 | 246.768 | +0.4% |
| Simulation 5 | max velocity (m/s) | 127.051 | 128.864 | +1.4% |
| Simulation 5 | max velocity (m/s), BS without the credit | 127.051 | 127.989 | +0.7% |
| Simulation 5 | max Mach | 0.374 | 0.379 | +1.4% |
| Simulation 5 | time to apogee (s) | 5.941 | 5.968 | +0.5% |
| Simulation 5 | rail-exit velocity (m/s, OR at 2 ms step) | 19.457 | 19.638 | +0.9% |
| Simulation 5 | stability off rail (cal) | 2.177 | 2.206 | +1.3% |
| Simulation 5 | ground-hit velocity (m/s) | 4.364 | 4.422 | +1.3% |
| Simulation 5 | flight time (s) | 60.943 | 61.465 | +0.9% |

# simple_conical_nose

OpenRocket 23.09 — 1 stage(s); import warnings: 0.


## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [A8-3] | 0.062 | 0.061 | -1.2% | 0.2568 | 0.2557 | -0.042 | 0.058 | 0.058 | 0.2502 | 0.2490 |
| [B4-4] | 0.064 | 0.064 | -1.1% | 0.2614 | 0.2604 | -0.038 | 0.058 | 0.058 | 0.2499 | 0.2487 |
| [C6-3] | 0.069 | 0.068 | -1.1% | 0.2682 | 0.2674 | -0.033 | 0.058 | 0.057 | 0.2486 | 0.2474 |
| [C6-5] | 0.069 | 0.068 | -1.1% | 0.2682 | 0.2674 | -0.033 | 0.058 | 0.057 | 0.2486 | 0.2474 |
| [C6-7] | 0.069 | 0.068 | -1.1% | 0.2682 | 0.2674 | -0.033 | 0.058 | 0.057 | 0.2486 | 0.2474 |

## Static aerodynamics, α = 0° (configuration [A8-3], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.3228 | 0.3228 | +0.000 | 11.886 | 11.886 | -0.0% | 0.6137 | 0.6177 | +0.6% | 0.418/0.074/0.121 | 0.484/0.012/0.121 |
| 0.30 | 0.3231 | 0.3232 | +0.000 | 11.970 | 11.970 | +0.0% | 0.6302 | 0.6339 | +0.6% | 0.415/0.084/0.132 | 0.490/0.012/0.132 |
| 0.50 | 0.3239 | 0.3239 | +0.000 | 12.147 | 12.147 | -0.0% | 0.6681 | 0.6712 | +0.5% | 0.408/0.108/0.152 | 0.506/0.012/0.152 |
| 0.80 | 0.3276 | 0.3276 | +0.000 | 12.636 | 12.636 | -0.0% | 0.8091 | 0.8108 | +0.2% | 0.392/0.214/0.203 | 0.588/0.020/0.203 |
| 0.90 | 0.3295 | 0.3295 | +0.000 | 12.880 | 12.880 | -0.0% | 0.9535 | 0.9546 | +0.1% | 0.385/0.344/0.225 | 0.686/0.043/0.225 |
| 1.00 | 0.3316 | 0.3315 | -0.004 | 13.186 | 13.159 | -0.2% | 1.0068 | 1.0069 | +0.0% | 0.364/0.393/0.250 | 0.633/0.124/0.250 |
| 1.20 | 0.3357 | 0.3352 | -0.018 | 13.875 | 13.740 | -1.0% | 0.9823 | 0.9754 | -0.7% | 0.332/0.442/0.208 | 0.615/0.152/0.208 |
| 1.50 | 0.3335 | 0.3335 | +0.000 | 12.781 | 12.781 | -0.0% | 0.8490 | 0.8477 | -0.2% | 0.298/0.384/0.167 | 0.593/0.088/0.167 |
| 2.00 | 0.3136 | 0.3136 | +0.000 | 8.959 | 8.959 | +0.0% | 0.7412 | 0.7418 | +0.1% | 0.243/0.373/0.125 | 0.549/0.068/0.125 |

## Static aerodynamics, α = 2° (configuration [A8-3], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.3174 | 0.3174 | +0.000 | 12.570 | 12.570 | -0.0% | 0.6208 | 0.6248 | +0.6% | 0.418/0.074/0.121 | 0.490/0.012/0.123 |
| 0.30 | 0.3178 | 0.3178 | +0.000 | 12.654 | 12.654 | +0.0% | 0.6375 | 0.6412 | +0.6% | 0.415/0.084/0.132 | 0.496/0.012/0.133 |
| 0.50 | 0.3186 | 0.3186 | +0.000 | 12.831 | 12.831 | -0.0% | 0.6758 | 0.6789 | +0.5% | 0.408/0.108/0.152 | 0.512/0.012/0.154 |
| 0.80 | 0.3223 | 0.3223 | +0.000 | 13.320 | 13.320 | -0.0% | 0.8183 | 0.8201 | +0.2% | 0.392/0.214/0.203 | 0.595/0.020/0.206 |
| 0.90 | 0.3242 | 0.3242 | +0.000 | 13.564 | 13.564 | -0.0% | 0.9645 | 0.9656 | +0.1% | 0.385/0.344/0.225 | 0.694/0.044/0.228 |
| 1.00 | 0.3263 | 0.3262 | -0.004 | 13.875 | 13.847 | -0.2% | 1.0184 | 1.0184 | +0.0% | 0.364/0.393/0.250 | 0.640/0.125/0.253 |
| 1.20 | 0.3307 | 0.3302 | -0.019 | 14.646 | 14.510 | -0.9% | 0.9936 | 0.9866 | -0.7% | 0.332/0.442/0.208 | 0.622/0.154/0.211 |
| 1.50 | 0.3290 | 0.3290 | +0.000 | 13.743 | 13.743 | +0.0% | 0.8587 | 0.8574 | -0.2% | 0.298/0.384/0.167 | 0.600/0.089/0.169 |
| 2.00 | 0.3085 | 0.3085 | +0.000 | 9.805 | 9.805 | +0.0% | 0.7497 | 0.7503 | +0.1% | 0.243/0.373/0.125 | 0.555/0.069/0.126 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.3065 \| 0.3151 | 0.0123 \| 0.0123 | 0.1317 \| 0.1317 | 0.0470 \| 0.1422 | 0.0388 \| 0.0325 | 0.5363 \| 0.6339 |
| 0.90 | 0.2843 \| 0.2922 | 0.0433 \| 0.0433 | 0.2253 \| 0.2253 | 0.1184 \| 0.3565 | 0.0454 \| 0.0373 | 0.7167 \| 0.9546 |
| 1.00 | 0.2691 \| 0.2767 | 0.1240 \| 0.1240 | 0.2500 \| 0.2500 | 0.1054 \| 0.3175 | 0.0473 \| 0.0387 | 0.7960 \| 1.0069 |
| 1.20 | 0.2457 \| 0.2525 | 0.1524 \| 0.1524 | 0.2083 \| 0.2083 | 0.1070 \| 0.3221 | 0.0550 \| 0.0400 | 0.7683 \| 0.9754 |
| 2.00 | 0.1798 \| 0.1849 | 0.0681 \| 0.0681 | 0.1250 \| 0.1250 | 0.1068 \| 0.3211 | 0.0479 \| 0.0426 | 0.5276 \| 0.7418 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Simulation 1 | apogee (m) | 55.273 | 56.571 | +2.3% |
| Simulation 1 | apogee (m), BS without the exhaust base-drag credit | 55.273 | 56.294 | +1.8% |
| Simulation 1 | max velocity (m/s) | 30.819 | 31.430 | +2.0% |
| Simulation 1 | max velocity (m/s), BS without the credit | 30.819 | 31.334 | +1.7% |
| Simulation 1 | max Mach | 0.091 | 0.092 | +2.0% |
| Simulation 1 | time to apogee (s) | 3.569 | 3.624 | +1.5% |
| Simulation 1 | rail-exit velocity (m/s, OR at 2 ms step) | 15.679 | 15.788 | +0.7% |
| Simulation 1 | stability off rail (cal) | 2.769 | 2.802 | +1.2% |
| Simulation 1 | ground-hit velocity (m/s) | 4.056 | 4.043 | -0.3% |
| Simulation 1 | flight time (s) | 17.426 | 17.173 | -1.4% |
| Simulation 2 | apogee (m) | 143.814 | 147.067 | +2.3% |
| Simulation 2 | apogee (m), BS without the exhaust base-drag credit | 143.814 | 145.719 | +1.3% |
| Simulation 2 | max velocity (m/s) | 55.671 | 56.888 | +2.2% |
| Simulation 2 | max velocity (m/s), BS without the credit | 55.671 | 56.470 | +1.4% |
| Simulation 2 | max Mach | 0.164 | 0.167 | +2.2% |
| Simulation 2 | time to apogee (s) | 5.331 | 5.509 | +3.4% |
| Simulation 2 | rail-exit velocity (m/s, OR at 2 ms step) | 17.989 | 18.151 | +0.9% |
| Simulation 2 | stability off rail (cal) | 2.588 | 2.619 | +1.2% |
| Simulation 2 | ground-hit velocity (m/s) | 4.134 | 4.038 | -2.3% |
| Simulation 2 | flight time (s) | 40.891 | 42.088 | +2.9% |
| Simulation 3 - too short delay | apogee (m) | 288.033 | 297.287 | +3.2% |
| Simulation 3 - too short delay | apogee (m), BS without the exhaust base-drag credit | 288.033 | 292.829 | +1.7% |
| Simulation 3 - too short delay | max velocity (m/s) | 98.510 | 101.498 | +3.0% |
| Simulation 3 - too short delay | max velocity (m/s), BS without the credit | 98.510 | 99.521 | +1.0% |
| Simulation 3 - too short delay | max Mach | 0.290 | 0.299 | +3.1% |
| Simulation 3 - too short delay | time to apogee (s) | 5.492 | 5.591 | +1.8% |
| Simulation 3 - too short delay | rail-exit velocity (m/s, OR at 2 ms step) | 17.771 | 17.970 | +1.1% |
| Simulation 3 - too short delay | stability off rail (cal) | 2.292 | 2.318 | +1.1% |
| Simulation 3 - too short delay | ground-hit velocity (m/s) | 4.063 | 4.017 | -1.1% |
| Simulation 3 - too short delay | flight time (s) | 76.460 | 79.361 | +3.8% |
| Simulation 4 | apogee (m) | 327.388 | 334.970 | +2.3% |
| Simulation 4 | apogee (m), BS without the exhaust base-drag credit | 327.388 | 329.418 | +0.6% |
| Simulation 4 | max velocity (m/s) | 98.510 | 101.498 | +3.0% |
| Simulation 4 | max velocity (m/s), BS without the credit | 98.510 | 99.521 | +1.0% |
| Simulation 4 | max Mach | 0.290 | 0.299 | +3.1% |
| Simulation 4 | time to apogee (s) | 7.336 | 7.487 | +2.1% |
| Simulation 4 | rail-exit velocity (m/s, OR at 2 ms step) | 17.771 | 17.970 | +1.1% |
| Simulation 4 | stability off rail (cal) | 2.292 | 2.318 | +1.1% |
| Simulation 4 | ground-hit velocity (m/s) | 4.018 | 4.017 | -0.0% |
| Simulation 4 | flight time (s) | 87.990 | 90.497 | +2.8% |
| Simulation 5 | apogee (m) | 330.640 | 337.321 | +2.0% |
| Simulation 5 | apogee (m), BS without the exhaust base-drag credit | 330.640 | 331.395 | +0.2% |
| Simulation 5 | max velocity (m/s) | 98.510 | 101.498 | +3.0% |
| Simulation 5 | max velocity (m/s), BS without the credit | 98.510 | 99.521 | +1.0% |
| Simulation 5 | max Mach | 0.290 | 0.299 | +3.1% |
| Simulation 5 | time to apogee (s) | 7.860 | 7.888 | +0.4% |
| Simulation 5 | rail-exit velocity (m/s, OR at 2 ms step) | 17.771 | 17.970 | +1.1% |
| Simulation 5 | stability off rail (cal) | 2.292 | 2.318 | +1.1% |
| Simulation 5 | ground-hit velocity (m/s) | 4.061 | 4.017 | -1.1% |
| Simulation 5 | flight time (s) | 88.652 | 90.524 | +2.1% |

# simple_model_rocket

OpenRocket 23.09 — 1 stage(s); import warnings: 0.


## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [A8-3] | 0.065 | 0.064 | -1.1% | 0.2480 | 0.2469 | -0.043 | 0.061 | 0.060 | 0.2412 | 0.2400 |
| [B4-4] | 0.067 | 0.066 | -1.1% | 0.2527 | 0.2517 | -0.040 | 0.061 | 0.060 | 0.2409 | 0.2397 |
| [C6-3] | 0.071 | 0.071 | -1.0% | 0.2598 | 0.2590 | -0.034 | 0.060 | 0.060 | 0.2396 | 0.2383 |
| [C6-5] | 0.071 | 0.071 | -1.0% | 0.2598 | 0.2590 | -0.034 | 0.060 | 0.060 | 0.2396 | 0.2383 |
| [C6-7] | 0.071 | 0.071 | -1.0% | 0.2598 | 0.2590 | -0.034 | 0.060 | 0.060 | 0.2396 | 0.2383 |

## Static aerodynamics, α = 0° (configuration [A8-3], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.3194 | 0.3194 | +0.000 | 11.886 | 11.886 | -0.0% | 0.6176 | 0.6207 | +0.5% | 0.433/0.063/0.121 | 0.499/0.000/0.121 |
| 0.30 | 0.3198 | 0.3198 | +0.000 | 11.970 | 11.970 | +0.0% | 0.6339 | 0.6368 | +0.4% | 0.430/0.073/0.132 | 0.505/0.000/0.132 |
| 0.50 | 0.3206 | 0.3206 | +0.000 | 12.147 | 12.147 | -0.0% | 0.6716 | 0.6739 | +0.3% | 0.423/0.096/0.152 | 0.521/0.000/0.152 |
| 0.80 | 0.3244 | 0.3244 | +0.000 | 12.636 | 12.636 | -0.0% | 0.8046 | 0.8162 | +1.4% | 0.406/0.196/0.203 | 0.602/0.011/0.203 |
| 0.90 | 0.3263 | 0.3263 | +0.000 | 12.880 | 12.880 | -0.0% | 0.9252 | 0.9645 | +4.2% | 0.398/0.302/0.225 | 0.700/0.039/0.225 |
| 1.00 | 0.3285 | 0.3284 | -0.004 | 13.186 | 13.159 | -0.2% | 0.8970 | 1.0202 | +13.7% | 0.377/0.270/0.250 | 0.646/0.124/0.250 |
| 1.20 | 0.3327 | 0.3323 | -0.019 | 13.875 | 13.740 | -1.0% | 0.8798 | 0.9875 | +12.3% | 0.344/0.327/0.208 | 0.627/0.152/0.208 |
| 1.50 | 0.3303 | 0.3303 | +0.000 | 12.781 | 12.781 | -0.0% | 0.7724 | 0.8586 | +11.2% | 0.308/0.297/0.167 | 0.604/0.088/0.167 |
| 2.00 | 0.3090 | 0.3090 | +0.000 | 8.959 | 8.959 | +0.0% | 0.6821 | 0.7507 | +10.1% | 0.252/0.305/0.125 | 0.558/0.068/0.125 |

## Static aerodynamics, α = 2° (configuration [A8-3], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.3135 | 0.3135 | +0.000 | 12.603 | 12.603 | -0.0% | 0.6247 | 0.6279 | +0.5% | 0.433/0.063/0.121 | 0.505/0.000/0.123 |
| 0.30 | 0.3139 | 0.3139 | +0.000 | 12.687 | 12.687 | -0.0% | 0.6412 | 0.6441 | +0.4% | 0.430/0.073/0.132 | 0.511/0.000/0.133 |
| 0.50 | 0.3147 | 0.3147 | +0.000 | 12.864 | 12.864 | -0.0% | 0.6793 | 0.6816 | +0.3% | 0.423/0.096/0.152 | 0.527/0.000/0.154 |
| 0.80 | 0.3186 | 0.3186 | +0.000 | 13.353 | 13.353 | -0.0% | 0.8138 | 0.8255 | +1.4% | 0.406/0.196/0.203 | 0.609/0.011/0.206 |
| 0.90 | 0.3205 | 0.3205 | +0.000 | 13.597 | 13.597 | -0.0% | 0.9358 | 0.9755 | +4.2% | 0.398/0.302/0.225 | 0.708/0.040/0.228 |
| 1.00 | 0.3227 | 0.3226 | -0.004 | 13.908 | 13.880 | -0.2% | 0.9073 | 1.0319 | +13.7% | 0.377/0.270/0.250 | 0.654/0.125/0.253 |
| 1.20 | 0.3273 | 0.3268 | -0.020 | 14.679 | 14.543 | -0.9% | 0.8899 | 0.9989 | +12.3% | 0.344/0.327/0.208 | 0.634/0.154/0.211 |
| 1.50 | 0.3254 | 0.3254 | +0.000 | 13.776 | 13.776 | -0.0% | 0.7812 | 0.8684 | +11.2% | 0.308/0.297/0.167 | 0.611/0.089/0.169 |
| 2.00 | 0.3035 | 0.3035 | +0.000 | 9.838 | 9.838 | -0.0% | 0.6899 | 0.7593 | +10.1% | 0.252/0.305/0.125 | 0.564/0.069/0.126 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.3213 \| 0.3303 | 0.0012 \| 0.0000 | 0.1317 \| 0.1317 | 0.0470 \| 0.1422 | 0.0388 \| 0.0325 | 0.5400 \| 0.6368 |
| 0.90 | 0.2980 \| 0.3063 | 0.0012 \| 0.0391 | 0.2253 \| 0.2253 | 0.1184 \| 0.3565 | 0.0454 \| 0.0373 | 0.6883 \| 0.9645 |
| 1.00 | 0.2821 \| 0.2900 | 0.0012 \| 0.1240 | 0.2500 \| 0.2500 | 0.1054 \| 0.3175 | 0.0473 \| 0.0387 | 0.6861 \| 1.0202 |
| 1.20 | 0.2575 \| 0.2647 | 0.0379 \| 0.1524 | 0.2083 \| 0.2083 | 0.1070 \| 0.3221 | 0.0550 \| 0.0400 | 0.6658 \| 0.9875 |
| 2.00 | 0.1885 \| 0.1938 | 0.0004 \| 0.0681 | 0.1250 \| 0.1250 | 0.1068 \| 0.3211 | 0.0479 \| 0.0426 | 0.4685 \| 0.7507 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Simulation 1 | apogee (m) | 51.111 | 52.293 | +2.3% |
| Simulation 1 | apogee (m), BS without the exhaust base-drag credit | 51.111 | 52.058 | +1.9% |
| Simulation 1 | max velocity (m/s) | 29.334 | 29.835 | +1.7% |
| Simulation 1 | max velocity (m/s), BS without the credit | 29.334 | 29.752 | +1.4% |
| Simulation 1 | max Mach | 0.086 | 0.088 | +1.7% |
| Simulation 1 | time to apogee (s) | 3.480 | 3.508 | +0.8% |
| Simulation 1 | rail-exit velocity (m/s, OR at 2 ms step) | 15.272 | 15.357 | +0.6% |
| Simulation 1 | stability off rail (cal) | 2.991 | 3.026 | +1.2% |
| Simulation 1 | ground-hit velocity (m/s) | 4.127 | 4.139 | +0.3% |
| Simulation 1 | flight time (s) | 15.999 | 15.861 | -0.9% |
| Simulation 2 | apogee (m) | 136.516 | 139.493 | +2.2% |
| Simulation 2 | apogee (m), BS without the exhaust base-drag credit | 136.516 | 138.283 | +1.3% |
| Simulation 2 | max velocity (m/s) | 53.307 | 54.325 | +1.9% |
| Simulation 2 | max velocity (m/s), BS without the credit | 53.307 | 53.959 | +1.2% |
| Simulation 2 | max Mach | 0.157 | 0.160 | +1.9% |
| Simulation 2 | time to apogee (s) | 5.317 | 5.428 | +2.1% |
| Simulation 2 | rail-exit velocity (m/s, OR at 2 ms step) | 17.563 | 17.742 | +1.0% |
| Simulation 2 | stability off rail (cal) | 2.808 | 2.838 | +1.1% |
| Simulation 2 | ground-hit velocity (m/s) | 4.100 | 4.133 | +0.8% |
| Simulation 2 | flight time (s) | 38.325 | 39.354 | +2.7% |
| Simulation 3 - too short delay | apogee (m) | 280.501 | 289.562 | +3.2% |
| Simulation 3 - too short delay | apogee (m), BS without the exhaust base-drag credit | 280.501 | 285.449 | +1.8% |
| Simulation 3 - too short delay | max velocity (m/s) | 95.334 | 97.983 | +2.8% |
| Simulation 3 - too short delay | max velocity (m/s), BS without the credit | 95.334 | 96.207 | +0.9% |
| Simulation 3 - too short delay | max Mach | 0.280 | 0.288 | +2.8% |
| Simulation 3 - too short delay | time to apogee (s) | 5.441 | 5.603 | +3.0% |
| Simulation 3 - too short delay | rail-exit velocity (m/s, OR at 2 ms step) | 17.400 | 17.523 | +0.7% |
| Simulation 3 - too short delay | stability off rail (cal) | 2.498 | 2.526 | +1.1% |
| Simulation 3 - too short delay | ground-hit velocity (m/s) | 4.122 | 4.113 | -0.2% |
| Simulation 3 - too short delay | flight time (s) | 73.099 | 75.817 | +3.7% |
| Simulation 4 | apogee (m) | 319.372 | 326.860 | +2.3% |
| Simulation 4 | apogee (m), BS without the exhaust base-drag credit | 319.372 | 321.681 | +0.7% |
| Simulation 4 | max velocity (m/s) | 95.334 | 97.983 | +2.8% |
| Simulation 4 | max velocity (m/s), BS without the credit | 95.334 | 96.207 | +0.9% |
| Simulation 4 | max Mach | 0.280 | 0.288 | +2.8% |
| Simulation 4 | time to apogee (s) | 7.346 | 7.494 | +2.0% |
| Simulation 4 | rail-exit velocity (m/s, OR at 2 ms step) | 17.400 | 17.523 | +0.7% |
| Simulation 4 | stability off rail (cal) | 2.498 | 2.526 | +1.1% |
| Simulation 4 | ground-hit velocity (m/s) | 4.231 | 4.113 | -2.8% |
| Simulation 4 | flight time (s) | 84.253 | 86.644 | +2.8% |
| Simulation 5 | apogee (m) | 322.470 | 329.101 | +2.1% |
| Simulation 5 | apogee (m), BS without the exhaust base-drag credit | 322.470 | 323.566 | +0.3% |
| Simulation 5 | max velocity (m/s) | 95.334 | 97.983 | +2.8% |
| Simulation 5 | max velocity (m/s), BS without the credit | 95.334 | 96.207 | +0.9% |
| Simulation 5 | max Mach | 0.280 | 0.288 | +2.8% |
| Simulation 5 | time to apogee (s) | 7.859 | 7.879 | +0.2% |
| Simulation 5 | rail-exit velocity (m/s, OR at 2 ms step) | 17.400 | 17.523 | +0.7% |
| Simulation 5 | stability off rail (cal) | 2.498 | 2.526 | +1.1% |
| Simulation 5 | ground-hit velocity (m/s) | 4.050 | 4.113 | +1.6% |
| Simulation 5 | flight time (s) | 84.842 | 86.630 | +2.1% |

# spaceport_final

OpenRocket 23.09 — 2 stage(s); import warnings: 4.

- import: material "Kevlar 12-strand (10 mm, 3/8 in)" (0.05952655 kg/m) is not in the model's table: using Tubular nylon (11 mm, 7/16 in) (0.013) and scaling the part masses to match
- import: material "Ripstop nylon, ultra lightweight, 2 mil" (0.035653924 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match
- import: material "Spectra #200 [Round 1.5 mm, 1/16 in]" (0.000656168 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: material "Kevlar 12-strand (11 mm, 7/16 in)" (0.07440819 kg/m) is not in the model's table: using Tubular nylon (11 mm, 7/16 in) (0.013) and scaling the part masses to match

## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [M1939-P; O5500-P] | 53.408 | 53.408 | +0.0% | 3.1438 | 3.1442 | 0.002 | 37.973 | 37.973 | 2.9353 | 2.9358 |
| [N1000-P; O5500-P] | 57.196 | 57.196 | +0.0% | 3.0748 | 3.0752 | 0.002 | 39.492 | 39.492 | 2.9043 | 2.9048 |
| [M1800-4; O5500-P] | 52.479 | 52.479 | +0.0% | 3.1161 | 3.1176 | 0.010 | 37.101 | 37.101 | 2.8928 | 2.8939 |

## Static aerodynamics, α = 0° (configuration [M1939-P; O5500-P], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 3.5158 | 3.5160 | +0.002 | 18.519 | 18.519 | +0.0% | 0.6255 | 0.6279 | +0.4% | 0.502/0.002/0.121 | 0.507/0.000/0.121 |
| 0.30 | 3.5186 | 3.5188 | +0.002 | 18.626 | 18.626 | +0.0% | 0.6377 | 0.6401 | +0.4% | 0.498/0.008/0.132 | 0.508/0.000/0.132 |
| 0.50 | 3.5243 | 3.5246 | +0.002 | 18.848 | 18.848 | +0.0% | 0.6638 | 0.6662 | +0.4% | 0.490/0.022/0.152 | 0.514/0.000/0.153 |
| 0.80 | 3.5487 | 3.5489 | +0.001 | 19.441 | 19.441 | +0.0% | 0.7549 | 0.7608 | +0.8% | 0.470/0.082/0.203 | 0.554/0.003/0.203 |
| 0.90 | 3.5609 | 3.5611 | +0.001 | 19.727 | 19.727 | +0.0% | 0.8378 | 0.8598 | +2.6% | 0.462/0.151/0.225 | 0.615/0.020/0.225 |
| 1.00 | 3.5736 | 3.5730 | -0.004 | 20.090 | 20.058 | -0.2% | 0.8119 | 0.9073 | +11.7% | 0.437/0.125/0.250 | 0.564/0.093/0.250 |
| 1.20 | 3.5822 | 3.5787 | -0.022 | 21.003 | 20.846 | -0.7% | 0.7838 | 0.8683 | +10.8% | 0.399/0.177/0.208 | 0.541/0.119/0.208 |
| 1.50 | 3.5196 | 3.5198 | +0.001 | 19.314 | 19.314 | +0.0% | 0.6783 | 0.7407 | +9.2% | 0.357/0.154/0.167 | 0.514/0.060/0.167 |
| 2.00 | 3.3577 | 3.3579 | +0.001 | 13.176 | 13.176 | +0.0% | 0.5849 | 0.6326 | +8.2% | 0.292/0.168/0.125 | 0.463/0.045/0.125 |

## Static aerodynamics, α = 2° (configuration [M1939-P; O5500-P], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 3.4511 | 3.4513 | +0.001 | 19.993 | 19.993 | -0.0% | 0.6327 | 0.6351 | +0.4% | 0.502/0.002/0.121 | 0.512/0.000/0.123 |
| 0.30 | 3.4540 | 3.4542 | +0.001 | 20.099 | 20.099 | -0.0% | 0.6450 | 0.6475 | +0.4% | 0.498/0.008/0.132 | 0.514/0.000/0.133 |
| 0.50 | 3.4601 | 3.4603 | +0.001 | 20.321 | 20.321 | -0.0% | 0.6714 | 0.6739 | +0.4% | 0.490/0.022/0.152 | 0.520/0.000/0.154 |
| 0.80 | 3.4845 | 3.4847 | +0.001 | 20.914 | 20.914 | -0.0% | 0.7636 | 0.7695 | +0.8% | 0.470/0.082/0.203 | 0.561/0.003/0.206 |
| 0.90 | 3.4967 | 3.4970 | +0.001 | 21.200 | 21.200 | -0.0% | 0.8474 | 0.8696 | +2.6% | 0.462/0.151/0.225 | 0.622/0.020/0.228 |
| 1.00 | 3.5098 | 3.5091 | -0.004 | 21.570 | 21.538 | -0.1% | 0.8212 | 0.9177 | +11.7% | 0.437/0.125/0.250 | 0.571/0.094/0.253 |
| 1.20 | 3.5224 | 3.5188 | -0.023 | 22.615 | 22.459 | -0.7% | 0.7928 | 0.8783 | +10.8% | 0.399/0.177/0.208 | 0.547/0.120/0.211 |
| 1.50 | 3.4660 | 3.4662 | +0.001 | 21.233 | 21.233 | -0.0% | 0.6861 | 0.7492 | +9.2% | 0.357/0.154/0.167 | 0.520/0.061/0.169 |
| 2.00 | 3.2959 | 3.2960 | +0.001 | 14.910 | 14.910 | -0.0% | 0.5916 | 0.6399 | +8.2% | 0.292/0.168/0.125 | 0.468/0.046/0.126 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.3999 \| 0.4030 | 0.0009 \| 0.0000 | 0.1317 \| 0.1317 | 0.0259 \| 0.1037 | 0.0015 \| 0.0017 | 0.5599 \| 0.6401 |
| 0.90 | 0.3708 \| 0.3737 | 0.0009 \| 0.0195 | 0.2253 \| 0.2253 | 0.0595 \| 0.2380 | 0.0028 \| 0.0032 | 0.6593 \| 0.8598 |
| 1.00 | 0.3511 \| 0.3538 | 0.0009 \| 0.0931 | 0.2500 \| 0.2500 | 0.0517 \| 0.2068 | 0.0031 \| 0.0036 | 0.6568 \| 0.9073 |
| 1.20 | 0.3205 \| 0.3229 | 0.0377 \| 0.1191 | 0.2083 \| 0.2083 | 0.0534 \| 0.2136 | 0.0037 \| 0.0043 | 0.6236 \| 0.8683 |
| 2.00 | 0.2346 \| 0.2364 | 0.0003 \| 0.0451 | 0.1250 \| 0.1250 | 0.0547 \| 0.2187 | 0.0063 \| 0.0074 | 0.4209 \| 0.6326 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Simulation 1 | apogee (m) | 9087.899 | 8960.084 | -1.4% |
| Simulation 1 | apogee (m), BS without the exhaust base-drag credit | 9087.899 | 8605.368 | -5.3% |
| Simulation 1 | max velocity (m/s) | 442.014 | 442.826 | +0.2% |
| Simulation 1 | max velocity (m/s), BS without the credit | 442.014 | 425.561 | -3.7% |
| Simulation 1 | max Mach | 1.370 | 1.373 | +0.2% |
| Simulation 1 | time to apogee (s) | 43.818 | 43.427 | -0.9% |
| Simulation 1 | rail-exit velocity (m/s, OR at 2 ms step) | 36.417 | 36.478 | +0.2% |
| Simulation 1 | stability off rail (cal) | 2.514 | 2.502 | -0.5% |
| Simulation 1 | ground-hit velocity (m/s) | 7.045 | 7.050 | +0.1% |
| Simulation 1 | flight time (s) | 371.180 | 368.200 | -0.8% |
| Simulation 2 | apogee (m) | 10718.493 | 10776.800 | +0.5% |
| Simulation 2 | apogee (m), BS without the exhaust base-drag credit | 10718.493 | 9992.495 | -6.8% |
| Simulation 2 | max velocity (m/s) | 409.962 | 412.342 | +0.6% |
| Simulation 2 | max velocity (m/s), BS without the credit | 409.962 | 388.942 | -5.1% |
| Simulation 2 | max Mach | 1.282 | 1.291 | +0.7% |
| Simulation 2 | time to apogee (s) | 48.482 | 48.556 | +0.2% |
| Simulation 2 | rail-exit velocity (m/s, OR at 2 ms step) | 14.642 | 14.698 | +0.4% |
| Simulation 2 | stability off rail (cal) | 2.874 | 2.867 | -0.2% |
| Simulation 2 | ground-hit velocity (m/s) | 7.313 | 7.318 | +0.1% |
| Simulation 2 | flight time (s) | 402.540 | 403.663 | +0.3% |
| Simulation 4 | apogee (m) | 8045.253 | 7905.307 | -1.7% |
| Simulation 4 | apogee (m), BS without the exhaust base-drag credit | 8045.253 | 7643.608 | -5.0% |
| Simulation 4 | max velocity (m/s) | 430.057 | 430.235 | +0.0% |
| Simulation 4 | max velocity (m/s), BS without the credit | 430.057 | 414.927 | -3.5% |
| Simulation 4 | max Mach | 1.323 | 1.323 | +0.0% |
| Simulation 4 | time to apogee (s) | 39.964 | 39.504 | -1.1% |
| Simulation 4 | rail-exit velocity (m/s, OR at 2 ms step) | 23.166 | 23.179 | +0.1% |
| Simulation 4 | stability off rail (cal) | 2.638 | 2.622 | -0.6% |
| Simulation 4 | ground-hit velocity (m/s) | 7.086 | 7.091 | +0.1% |
| Simulation 4 | flight time (s) | 338.203 | 334.877 | -1.0% |

# spaceport_subscale_rev4

OpenRocket 23.09 — 2 stage(s); import warnings: 3.

- import: material "Ripstop nylon, lightweight, 3 mil" (0.035653924 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match
- import: material "Spectra #200 [Round 1.5 mm, 1/16 in]" (0.000656168 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: material "Ripstop nylon, ultra lightweight, 2 mil" (0.035653924 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match

## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [J425-14; K535-14] | 9.132 | 9.176 | +0.5% | 1.3416 | 1.3450 | 0.044 | 8.023 | 8.067 | 1.2614 | 1.2656 |
| [H550-14; K535-14] | 8.817 | 8.861 | +0.5% | 1.3464 | 1.3498 | 0.045 | 7.896 | 7.940 | 1.2623 | 1.2667 |
| [H550-14; J425-14] | 8.184 | 8.228 | +0.5% | 1.2778 | 1.2819 | 0.054 | 7.644 | 7.688 | 1.2303 | 1.2350 |
| [I500-P; I500-P] | 8.397 | 8.441 | +0.5% | 1.2693 | 1.2734 | 0.054 | 7.901 | 7.945 | 1.2385 | 1.2430 |
| [H550-14; J1799-P] | 8.664 | 8.708 | +0.5% | 1.3349 | 1.3361 | 0.015 | 7.897 | 7.941 | 1.2646 | 1.2677 |
| [H550-14; K2050-0] | 8.836 | 8.880 | +0.5% | 1.3484 | 1.3499 | 0.020 | 8.034 | 8.078 | 1.2791 | 1.2822 |
| [H550-14; J800-18] | 8.639 | 8.683 | +0.5% | 1.3306 | 1.3343 | 0.049 | 7.849 | 7.894 | 1.2578 | 1.2622 |
| [H550-14; K2050-0] | 8.836 | 8.880 | +0.5% | 1.3484 | 1.3499 | 0.020 | 8.034 | 8.078 | 1.2791 | 1.2822 |

## Static aerodynamics, α = 0° (configuration [J425-14; K535-14], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 1.7056 | 1.7056 | +0.001 | 16.769 | 16.769 | +0.0% | 0.6682 | 0.6706 | +0.4% | 0.544/0.002/0.121 | 0.549/0.000/0.121 |
| 0.30 | 1.7074 | 1.7075 | +0.001 | 16.928 | 16.928 | +0.0% | 0.6842 | 0.6867 | +0.4% | 0.540/0.012/0.132 | 0.555/0.000/0.132 |
| 0.50 | 1.7113 | 1.7114 | +0.001 | 17.267 | 17.267 | +0.0% | 0.7206 | 0.7231 | +0.3% | 0.531/0.037/0.152 | 0.570/0.000/0.152 |
| 0.80 | 1.7252 | 1.7252 | +0.001 | 18.236 | 18.236 | -0.0% | 0.8625 | 0.8754 | +1.5% | 0.510/0.149/0.203 | 0.661/0.011/0.203 |
| 0.90 | 1.7320 | 1.7320 | +0.001 | 18.740 | 18.740 | +0.0% | 1.0058 | 1.0471 | +4.1% | 0.501/0.280/0.225 | 0.783/0.039/0.225 |
| 1.00 | 1.7387 | 1.7382 | -0.006 | 19.262 | 19.204 | -0.3% | 0.9542 | 1.0802 | +13.2% | 0.474/0.230/0.250 | 0.706/0.124/0.250 |
| 1.20 | 1.7372 | 1.7345 | -0.035 | 18.587 | 18.303 | -1.5% | 0.9348 | 1.0521 | +12.6% | 0.433/0.294/0.208 | 0.691/0.152/0.208 |
| 1.50 | 1.6663 | 1.6663 | +0.000 | 13.029 | 13.029 | -0.0% | 0.8371 | 0.9269 | +10.7% | 0.388/0.282/0.167 | 0.672/0.088/0.167 |
| 2.00 | 1.5491 | 1.5491 | +0.000 | 9.119 | 9.119 | -0.0% | 0.7472 | 0.8171 | +9.4% | 0.317/0.305/0.125 | 0.624/0.068/0.125 |

## Static aerodynamics, α = 2° (configuration [J425-14; K535-14], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 1.6691 | 1.6691 | +0.001 | 18.252 | 18.252 | -0.0% | 0.6759 | 0.6783 | +0.4% | 0.544/0.002/0.121 | 0.556/0.000/0.123 |
| 0.30 | 1.6711 | 1.6712 | +0.001 | 18.411 | 18.411 | -0.0% | 0.6921 | 0.6945 | +0.4% | 0.540/0.012/0.132 | 0.561/0.000/0.133 |
| 0.50 | 1.6754 | 1.6754 | +0.001 | 18.750 | 18.750 | -0.0% | 0.7289 | 0.7314 | +0.3% | 0.531/0.037/0.152 | 0.577/0.000/0.154 |
| 0.80 | 1.6899 | 1.6899 | +0.001 | 19.718 | 19.718 | -0.0% | 0.8724 | 0.8855 | +1.5% | 0.510/0.149/0.203 | 0.669/0.011/0.206 |
| 0.90 | 1.6971 | 1.6971 | +0.001 | 20.223 | 20.223 | -0.0% | 1.0174 | 1.0591 | +4.1% | 0.501/0.280/0.225 | 0.792/0.040/0.228 |
| 1.00 | 1.7043 | 1.7038 | -0.007 | 20.749 | 20.691 | -0.3% | 0.9651 | 1.0926 | +13.2% | 0.474/0.230/0.250 | 0.714/0.125/0.253 |
| 1.20 | 1.7027 | 1.6997 | -0.039 | 20.158 | 19.875 | -1.4% | 0.9455 | 1.0642 | +12.6% | 0.433/0.294/0.208 | 0.699/0.154/0.211 |
| 1.50 | 1.6305 | 1.6305 | +0.000 | 14.796 | 14.796 | -0.0% | 0.8467 | 0.9375 | +10.7% | 0.388/0.282/0.167 | 0.680/0.089/0.169 |
| 2.00 | 1.5148 | 1.5149 | +0.000 | 10.768 | 10.768 | -0.0% | 0.7558 | 0.8265 | +9.4% | 0.317/0.305/0.125 | 0.631/0.069/0.126 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.4657 \| 0.4693 | 0.0012 \| 0.0000 | 0.1317 \| 0.1317 | 0.0214 \| 0.0856 | 0.0000 \| 0.0000 | 0.6200 \| 0.6867 |
| 0.90 | 0.4319 \| 0.4352 | 0.0012 \| 0.0391 | 0.2253 \| 0.2253 | 0.0869 \| 0.3475 | 0.0000 \| 0.0000 | 0.7452 \| 1.0471 |
| 1.00 | 0.4089 \| 0.4121 | 0.0012 \| 0.1240 | 0.2500 \| 0.2500 | 0.0735 \| 0.2941 | 0.0000 \| 0.0000 | 0.7336 \| 1.0802 |
| 1.20 | 0.3732 \| 0.3761 | 0.0379 \| 0.1524 | 0.2083 \| 0.2083 | 0.0788 \| 0.3153 | 0.0000 \| 0.0000 | 0.6983 \| 1.0521 |
| 2.00 | 0.2732 \| 0.2753 | 0.0004 \| 0.0681 | 0.1250 \| 0.1250 | 0.0872 \| 0.3486 | 0.0000 \| 0.0000 | 0.4857 \| 0.8171 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Simulation 1 | apogee (m) | 2219.892 | 2219.011 | -0.0% |
| Simulation 1 | apogee (m), BS without the exhaust base-drag credit | 2219.892 | 2205.939 | -0.6% |
| Simulation 1 | max velocity (m/s) | 197.187 | 197.816 | +0.3% |
| Simulation 1 | max velocity (m/s), BS without the credit | 197.187 | 196.835 | -0.2% |
| Simulation 1 | max Mach | 0.585 | 0.587 | +0.3% |
| Simulation 1 | time to apogee (s) | 24.250 | 24.247 | -0.0% |
| Simulation 1 | rail-exit velocity (m/s, OR at 2 ms step) | 16.943 | 16.911 | -0.2% |
| Simulation 1 | stability off rail (cal) | 4.904 | 4.852 | -1.1% |
| Simulation 1 | ground-hit velocity (m/s) | 7.535 | 7.531 | -0.1% |
| Simulation 1 | flight time (s) | 176.785 | 177.386 | +0.3% |
| Simulation 2 | apogee (m) | 1572.458 | 1567.574 | -0.3% |
| Simulation 2 | apogee (m), BS without the exhaust base-drag credit | 1572.458 | 1559.501 | -0.8% |
| Simulation 2 | max velocity (m/s) | 143.991 | 144.024 | +0.0% |
| Simulation 2 | max velocity (m/s), BS without the credit | 143.991 | 143.451 | -0.4% |
| Simulation 2 | max Mach | 0.427 | 0.427 | +0.0% |
| Simulation 2 | time to apogee (s) | 20.081 | 20.065 | -0.1% |
| Simulation 2 | rail-exit velocity (m/s, OR at 2 ms step) | 17.312 | 17.252 | -0.3% |
| Simulation 2 | stability off rail (cal) | 4.842 | 4.790 | -1.1% |
| Simulation 2 | ground-hit velocity (m/s) | 7.429 | 7.425 | -0.1% |
| Simulation 2 | flight time (s) | 137.680 | 137.662 | -0.0% |
| Simulation 3 | apogee (m) | 645.574 | 640.971 | -0.7% |
| Simulation 3 | apogee (m), BS without the exhaust base-drag credit | 645.574 | 640.311 | -0.8% |
| Simulation 3 | max velocity (m/s) | 89.526 | 89.371 | -0.2% |
| Simulation 3 | max velocity (m/s), BS without the credit | 89.526 | 89.303 | -0.2% |
| Simulation 3 | max Mach | 0.264 | 0.263 | -0.2% |
| Simulation 3 | time to apogee (s) | 14.731 | 14.704 | -0.2% |
| Simulation 3 | rail-exit velocity (m/s, OR at 2 ms step) | 14.205 | 14.178 | -0.2% |
| Simulation 3 | stability off rail (cal) | 5.739 | 5.679 | -1.0% |
| Simulation 3 | ground-hit velocity (m/s) | 7.429 | 7.425 | -0.1% |
| Simulation 3 | flight time (s) | 76.471 | 76.535 | +0.1% |
| Simulation 4 | apogee (m) | 1104.149 | 1086.048 | -1.6% |
| Simulation 4 | apogee (m), BS without the exhaust base-drag credit | 1104.149 | 1083.546 | -1.9% |
| Simulation 4 | max velocity (m/s) | 137.331 | 136.514 | -0.6% |
| Simulation 4 | max velocity (m/s), BS without the credit | 137.331 | 136.282 | -0.8% |
| Simulation 4 | max Mach | 0.405 | 0.403 | -0.6% |
| Simulation 4 | time to apogee (s) | 18.770 | 18.704 | -0.4% |
| Simulation 4 | rail-exit velocity (m/s, OR at 2 ms step) | 16.450 | 16.354 | -0.6% |
| Simulation 4 | stability off rail (cal) | 5.830 | 5.768 | -1.1% |
| Simulation 4 | ground-hit velocity (m/s) | 7.589 | 7.585 | -0.1% |
| Simulation 4 | flight time (s) | 106.814 | 105.798 | -1.0% |
| Simulation 5 | apogee (m) | 1222.485 | 1215.454 | -0.6% |
| Simulation 5 | apogee (m), BS without the exhaust base-drag credit | 1222.485 | 1213.259 | -0.8% |
| Simulation 5 | max velocity (m/s) | 133.724 | 133.548 | -0.1% |
| Simulation 5 | max velocity (m/s), BS without the credit | 133.724 | 133.354 | -0.3% |
| Simulation 5 | max Mach | 0.395 | 0.395 | -0.1% |
| Simulation 5 | time to apogee (s) | 17.085 | 17.037 | -0.3% |
| Simulation 5 | rail-exit velocity (m/s, OR at 2 ms step) | 32.339 | 32.379 | +0.1% |
| Simulation 5 | stability off rail (cal) | 5.118 | 5.075 | -0.8% |
| Simulation 5 | ground-hit velocity (m/s) | 7.429 | 7.425 | -0.1% |
| Simulation 5 | flight time (s) | 113.645 | 113.744 | +0.1% |
| Simulation 6 | apogee (m) | 1635.273 | 1627.307 | -0.5% |
| Simulation 6 | apogee (m), BS without the exhaust base-drag credit | 1635.273 | 1623.380 | -0.7% |
| Simulation 6 | max velocity (m/s) | 160.793 | 160.602 | -0.1% |
| Simulation 6 | max velocity (m/s), BS without the credit | 160.793 | 160.274 | -0.3% |
| Simulation 6 | max Mach | 0.476 | 0.475 | -0.1% |
| Simulation 6 | time to apogee (s) | 18.935 | 18.889 | -0.2% |
| Simulation 6 | rail-exit velocity (m/s, OR at 2 ms step) | 32.638 | 32.636 | -0.0% |
| Simulation 6 | stability off rail (cal) | 4.883 | 4.844 | -0.8% |
| Simulation 6 | ground-hit velocity (m/s) | 7.429 | 7.425 | -0.1% |
| Simulation 6 | flight time (s) | 140.371 | 140.010 | -0.3% |
| Simulation 7 | apogee (m) | 1413.527 | 1407.918 | -0.4% |
| Simulation 7 | apogee (m), BS without the exhaust base-drag credit | 1413.527 | 1402.377 | -0.8% |
| Simulation 7 | max velocity (m/s) | 138.612 | 138.594 | -0.0% |
| Simulation 7 | max velocity (m/s), BS without the credit | 138.612 | 138.178 | -0.3% |
| Simulation 7 | max Mach | 0.410 | 0.410 | -0.0% |
| Simulation 7 | time to apogee (s) | 18.873 | 18.832 | -0.2% |
| Simulation 7 | rail-exit velocity (m/s, OR at 2 ms step) | 20.099 | 20.073 | -0.1% |
| Simulation 7 | stability off rail (cal) | 5.066 | 5.004 | -1.2% |
| Simulation 7 | ground-hit velocity (m/s) | 7.429 | 7.425 | -0.1% |
| Simulation 7 | flight time (s) | 127.189 | 127.007 | -0.1% |
| Simulation 8 | apogee (m) | 1639.328 | 1631.846 | -0.5% |
| Simulation 8 | apogee (m), BS without the exhaust base-drag credit | 1639.328 | 1627.226 | -0.7% |
| Simulation 8 | max velocity (m/s) | 181.315 | 181.177 | -0.1% |
| Simulation 8 | max velocity (m/s), BS without the credit | 181.315 | 180.754 | -0.3% |
| Simulation 8 | max Mach | 0.536 | 0.535 | -0.1% |
| Simulation 8 | time to apogee (s) | 18.265 | 18.237 | -0.2% |
| Simulation 8 | rail-exit velocity (m/s, OR at 2 ms step) | 32.638 | 32.636 | -0.0% |
| Simulation 8 | stability off rail (cal) | 4.883 | 4.844 | -0.8% |
| Simulation 8 | ground-hit velocity (m/s) | 7.429 | 7.425 | -0.1% |
| Simulation 8 | flight time (s) | 139.688 | 139.605 | -0.1% |

# tube_fin_rocket

OpenRocket 23.09 — 1 stage(s); import warnings: 4.

- import: material "Paper, spiral kraft glassine, Estes avg, bulk" (894.4 kg/m³) is not in the model's table: using Kraft phenolic (950) and scaling the part masses to match
- import: material "Polyethylene film, HDPE, 1.0 mil, bare" (0.0235 kg/m²) is not in the model's table: using Mylar (0.021) and scaling the part masses to match
- import: material "Carpet Thread" (0.00033 kg/m) is not in the model's table: using Braided nylon (2 mm, 1/16 in) (0.001) and scaling the part masses to match
- import: Body tube: the body tube is its own motor mount; imported as a massless inner tube of its bore (24.1 mm)

## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [D12-7] | 0.076 | 0.077 | +0.2% | 0.4528 | 0.4529 | 0.003 | 0.055 | 0.055 | 0.4163 | 0.4165 |

## Static aerodynamics, α = 0° (configuration [D12-7], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.4966 | 0.4966 | +0.000 | 39.847 | 39.847 | +0.0% | 1.7931 | 1.7977 | +0.3% | 0.636/1.036/0.121 | 1.676/0.000/0.121 |
| 0.30 | 0.4966 | 0.4966 | +0.000 | 39.847 | 39.847 | +0.0% | 1.7811 | 1.7867 | +0.3% | 0.631/1.018/0.132 | 1.655/0.000/0.132 |
| 0.50 | 0.4966 | 0.4966 | +0.000 | 39.847 | 39.847 | +0.0% | 1.8140 | 1.8200 | +0.3% | 0.621/1.041/0.152 | 1.667/0.000/0.152 |
| 0.80 | 0.4785 | 0.4966 | +0.730 | 39.847 | 39.847 | +0.0% | 1.9104 | 1.9221 | +0.6% | 0.596/1.111/0.203 | 1.714/0.005/0.203 |
| 0.90 | 0.4785 | 0.4966 | +0.730 | 39.847 | 39.847 | +0.0% | 1.9546 | 1.9865 | +1.6% | 0.585/1.144/0.225 | 1.736/0.025/0.225 |
| 1.00 | 0.4785 | 0.4966 | +0.730 | 39.847 | 39.847 | +0.0% | 1.9860 | 2.0956 | +5.5% | 0.554/1.182/0.250 | 1.743/0.103/0.250 |
| 1.20 | 0.4785 | 0.4966 | +0.730 | 39.847 | 39.847 | +0.0% | 2.2060 | 2.0671 | -6.3% | 0.506/1.492/0.208 | 1.729/0.129/0.208 |
| 1.50 | 0.4785 | 0.4966 | +0.730 | 39.847 | 39.847 | +0.0% | 1.8244 | 1.9492 | +6.8% | 0.453/1.205/0.167 | 1.714/0.068/0.167 |
| 2.00 | 0.3878 | 0.4966 | +4.389 | 39.847 | 39.847 | +0.0% | 1.5839 | 1.8401 | +16.2% | 0.370/1.089/0.125 | 1.663/0.052/0.125 |

## Static aerodynamics, α = 2° (configuration [D12-7], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.4918 | 0.4918 | +0.000 | 40.906 | 40.906 | -0.0% | 1.8137 | 1.8183 | +0.3% | 0.636/1.036/0.121 | 1.696/0.000/0.123 |
| 0.30 | 0.4918 | 0.4918 | +0.000 | 40.906 | 40.906 | -0.0% | 1.8016 | 1.8072 | +0.3% | 0.631/1.018/0.132 | 1.674/0.000/0.133 |
| 0.50 | 0.4918 | 0.4918 | +0.000 | 40.906 | 40.906 | -0.0% | 1.8348 | 1.8409 | +0.3% | 0.621/1.041/0.152 | 1.687/0.000/0.154 |
| 0.80 | 0.4741 | 0.4918 | +0.711 | 40.906 | 40.906 | -0.0% | 1.9323 | 1.9441 | +0.6% | 0.596/1.111/0.203 | 1.733/0.005/0.206 |
| 0.90 | 0.4741 | 0.4918 | +0.711 | 40.906 | 40.906 | -0.0% | 1.9770 | 2.0093 | +1.6% | 0.585/1.144/0.225 | 1.756/0.025/0.228 |
| 1.00 | 0.4741 | 0.4918 | +0.711 | 40.906 | 40.906 | -0.0% | 2.0088 | 2.1196 | +5.5% | 0.554/1.182/0.250 | 1.763/0.104/0.253 |
| 1.20 | 0.4741 | 0.4918 | +0.711 | 40.906 | 40.906 | -0.0% | 2.2314 | 2.0908 | -6.3% | 0.506/1.492/0.208 | 1.749/0.131/0.211 |
| 1.50 | 0.4741 | 0.4918 | +0.711 | 40.906 | 40.906 | -0.0% | 1.8454 | 1.9715 | +6.8% | 0.453/1.205/0.167 | 1.734/0.069/0.169 |
| 2.00 | 0.3858 | 0.4918 | +4.275 | 40.906 | 40.906 | -0.0% | 1.6021 | 1.8612 | +16.2% | 0.370/1.089/0.125 | 1.682/0.052/0.126 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.4441 \| 0.4488 | 0.0010 \| 0.0000 | 0.1317 \| 0.1317 | 0.1970 \| 1.1817 | 0.0220 \| 0.0245 | 0.7959 \| 1.7867 |
| 0.90 | 0.4119 \| 0.4162 | 0.0010 \| 0.0252 | 0.2253 \| 0.2253 | 0.2156 \| 1.2936 | 0.0226 \| 0.0261 | 0.8764 \| 1.9865 |
| 1.00 | 0.3899 \| 0.3941 | 0.0010 \| 0.1028 | 0.2500 \| 0.2500 | 0.2204 \| 1.3220 | 0.0228 \| 0.0266 | 0.8842 \| 2.0956 |
| 1.20 | 0.3559 \| 0.3597 | 0.0378 \| 0.1293 | 0.2083 \| 0.2083 | 0.2620 \| 1.3426 | 0.0319 \| 0.0271 | 0.8959 \| 2.0671 |
| 2.00 | 0.2606 \| 0.2633 | 0.0003 \| 0.0519 | 0.1250 \| 0.1250 | 0.1970 \| 1.3718 | 0.0163 \| 0.0281 | 0.5991 \| 1.8401 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Simulation 1 | apogee (m) | 282.881 | 288.281 | +1.9% |
| Simulation 1 | apogee (m), BS without the exhaust base-drag credit | 282.881 | 282.324 | -0.2% |
| Simulation 1 | max velocity (m/s) | 119.412 | 122.916 | +2.9% |
| Simulation 1 | max velocity (m/s), BS without the credit | 119.412 | 119.227 | -0.2% |
| Simulation 1 | max Mach | 0.351 | 0.362 | +3.0% |
| Simulation 1 | time to apogee (s) | 5.903 | 5.949 | +0.8% |
| Simulation 1 | rail-exit velocity (m/s, OR at 2 ms step) | 18.539 | 18.583 | +0.2% |
| Simulation 1 | stability off rail (cal) | 1.883 | 1.865 | -1.0% |
| Simulation 1 | ground-hit velocity (m/s) | 3.884 | 3.899 | +0.4% |
| Simulation 1 | flight time (s) | 75.437 | 72.877 | -3.4% |

# two_stage

OpenRocket 23.09 — 2 stage(s); import warnings: 8.

- import: material "Paper, kraft glassine, LOC tube avg" (855.2 kg/m³) is not in the model's table: using Kraft phenolic (950) and scaling the part masses to match
- import: material "Paper, kraft, LOC coupler avg" (789.6 kg/m³) is not in the model's table: using Cardboard (680) and scaling the part masses to match
- import: material "Nylon fabric, ripstop, 1.9 oz actual" (0.0589 kg/m²) is not in the model's table: using Ripstop nylon (0.067) and scaling the part masses to match
- import: material "Paper, kraft, LOC coupler stiffener avg" (603.3 kg/m³) is not in the model's table: using Birch plywood (630) and scaling the part masses to match
- import: material "Delrin" (1420 kg/m³) is not in the model's table: using Blue tube (1300) and scaling the part masses to match
- import: material "Nylon Paracord, 110 lb, 1/16 in. dia." (0.0016 kg/m) is not in the model's table: using Nylon elastic cord (2 mm, 1/16 in) (0.0018) and scaling the part masses to match
- import: material "[material:Aircraft plywood (LOC)]" (724.9956534840001 kg/m³) is not in the model's table: using Cardboard (680) and scaling the part masses to match
- import: stage 1 separation at the ignition of the upper stage is approximated by burnout

## Mass and CG at liftoff

| configuration | OR mass (kg) | BS mass | Δ | OR CG (m) | BS CG | Δ (cal) | OR burnout mass | BS | OR burnout CG | BS |
|---|---|---|---|---|---|---|---|---|---|---|
| [H148R-0; H148R-0] | 2.574 | 2.572 | -0.1% | 1.3302 | 1.3307 | 0.004 | 2.279 | 2.276 | 1.2878 | 1.2882 |
| [I59WN-P; I357T-14] | 2.793 | 2.790 | -0.1% | 1.3300 | 1.3304 | 0.004 | 2.341 | 2.339 | 1.2861 | 1.2865 |

## Static aerodynamics, α = 0° (configuration [H148R-0; H148R-0], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 1.5404 | 1.5404 | +0.000 | 28.592 | 28.592 | -0.0% | 0.8769 | 0.8793 | +0.3% | 0.416/0.339/0.121 | 0.758/0.000/0.121 |
| 0.30 | 1.5423 | 1.5423 | +0.000 | 29.064 | 29.064 | -0.0% | 0.8946 | 0.8972 | +0.3% | 0.413/0.350/0.132 | 0.766/0.000/0.132 |
| 0.50 | 1.5463 | 1.5463 | +0.000 | 30.105 | 30.105 | -0.0% | 0.9292 | 0.9322 | +0.3% | 0.406/0.370/0.152 | 0.779/0.000/0.152 |
| 0.80 | 1.5640 | 1.5640 | +0.000 | 33.435 | 33.435 | +0.0% | 1.0142 | 1.0393 | +2.5% | 0.390/0.421/0.203 | 0.814/0.022/0.203 |
| 0.90 | 1.5735 | 1.5735 | +0.000 | 35.450 | 35.450 | -0.0% | 1.0520 | 1.1168 | +6.2% | 0.383/0.444/0.225 | 0.829/0.062/0.225 |
| 1.00 | 1.5830 | 1.5824 | -0.006 | 37.659 | 37.411 | -0.7% | 1.0822 | 1.2397 | +14.6% | 0.363/0.470/0.250 | 0.835/0.155/0.250 |
| 1.20 | 1.5824 | 1.5786 | -0.037 | 34.995 | 33.792 | -3.4% | 1.0695 | 1.2240 | +14.4% | 0.331/0.530/0.208 | 0.827/0.189/0.208 |
| 1.50 | 1.4791 | 1.4791 | +0.000 | 16.507 | 16.507 | -0.0% | 0.9824 | 1.1055 | +12.5% | 0.297/0.519/0.167 | 0.819/0.120/0.167 |
| 2.00 | 1.3977 | 1.3977 | +0.000 | 11.364 | 11.364 | -0.0% | 0.9115 | 1.0104 | +10.8% | 0.242/0.544/0.125 | 0.790/0.095/0.125 |

## Static aerodynamics, α = 2° (configuration [H148R-0; H148R-0], ISA sea level)

| Mach | CP OR (m) | CP BS | ΔCP (cal) | CNα OR | CNα BS | Δ | Cd OR | Cd BS | Δ | OR f/p/b | BS f+fins+lugs / p / b |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 1.5258 | 1.5258 | +0.000 | 29.527 | 29.527 | -0.0% | 0.8869 | 0.8894 | +0.3% | 0.416/0.339/0.121 | 0.767/0.000/0.123 |
| 0.30 | 1.5278 | 1.5279 | +0.000 | 29.999 | 29.999 | -0.0% | 0.9049 | 0.9075 | +0.3% | 0.413/0.350/0.132 | 0.774/0.000/0.133 |
| 0.50 | 1.5322 | 1.5322 | +0.000 | 31.040 | 31.040 | -0.0% | 0.9398 | 0.9429 | +0.3% | 0.406/0.370/0.152 | 0.788/0.000/0.154 |
| 0.80 | 1.5507 | 1.5508 | +0.000 | 34.369 | 34.369 | -0.0% | 1.0259 | 1.0512 | +2.5% | 0.390/0.421/0.203 | 0.823/0.023/0.206 |
| 0.90 | 1.5607 | 1.5608 | +0.000 | 36.385 | 36.385 | -0.0% | 1.0640 | 1.1296 | +6.2% | 0.383/0.444/0.225 | 0.839/0.063/0.228 |
| 1.00 | 1.5708 | 1.5701 | -0.007 | 38.599 | 38.352 | -0.6% | 1.0946 | 1.2540 | +14.6% | 0.363/0.470/0.250 | 0.844/0.157/0.253 |
| 1.20 | 1.5695 | 1.5655 | -0.040 | 36.047 | 34.844 | -3.3% | 1.0818 | 1.2380 | +14.4% | 0.331/0.530/0.208 | 0.837/0.191/0.211 |
| 1.50 | 1.4619 | 1.4619 | +0.000 | 17.817 | 17.817 | -0.0% | 0.9937 | 1.1182 | +12.5% | 0.297/0.519/0.167 | 0.828/0.121/0.169 |
| 2.00 | 1.3785 | 1.3785 | +0.000 | 12.518 | 12.518 | -0.0% | 0.9220 | 1.0220 | +10.8% | 0.242/0.544/0.125 | 0.799/0.096/0.126 |

## Drag by category, α = 0° (OR | BS)

| Mach | body friction | body pressure | base | fins | lugs/buttons | total |
|---|---|---|---|---|---|---|
| 0.30 | 0.3049 \| 0.3086 | 0.0015 \| 0.0000 | 0.1317 \| 0.1317 | 0.1512 \| 0.4536 | 0.0014 \| 0.0033 | 0.5908 \| 0.8972 |
| 0.90 | 0.2828 \| 0.2862 | 0.0015 \| 0.0622 | 0.2253 \| 0.2253 | 0.1789 \| 0.5368 | 0.0028 \| 0.0063 | 0.6913 \| 1.1168 |
| 1.00 | 0.2677 \| 0.2710 | 0.0015 \| 0.1550 | 0.2500 \| 0.2500 | 0.1856 \| 0.5567 | 0.0031 \| 0.0070 | 0.7079 \| 1.2397 |
| 1.20 | 0.2443 \| 0.2473 | 0.0382 \| 0.1886 | 0.2083 \| 0.2083 | 0.1903 \| 0.5709 | 0.0039 \| 0.0088 | 0.6850 \| 1.2240 |
| 2.00 | 0.1789 \| 0.1811 | 0.0004 \| 0.0952 | 0.1250 \| 0.1250 | 0.1974 \| 0.5923 | 0.0075 \| 0.0168 | 0.5092 \| 1.0104 |

## Flights (no wind, ISA)

| simulation | metric | OpenRocket | Bowshock | Δ |
|---|---|---|---|---|
| Simulation 1 | apogee (m) | 678.622 | 675.279 | -0.5% |
| Simulation 1 | apogee (m), BS without the exhaust base-drag credit | 678.622 | 670.843 | -1.1% |
| Simulation 1 | max velocity (m/s) | 159.034 | 159.163 | +0.1% |
| Simulation 1 | max velocity (m/s), BS without the credit | 159.034 | 157.944 | -0.7% |
| Simulation 1 | max Mach | 0.468 | 0.469 | +0.1% |
| Simulation 1 | time to apogee (s) | 11.122 | 11.021 | -0.9% |
| Simulation 1 | rail-exit velocity (m/s, OR at 2 ms step) | 19.886 | 19.791 | -0.5% |
| Simulation 1 | stability off rail (cal) | 2.185 | 2.166 | -0.9% |
| Simulation 1 | ground-hit velocity (m/s) | 5.940 | 5.940 | -0.0% |
| Simulation 1 | flight time (s) | 63.387 | 64.163 | +1.2% |
| Simulation 2 | apogee (m) | 1382.540 | 1401.565 | +1.4% |
| Simulation 2 | apogee (m), BS without the exhaust base-drag credit | 1382.540 | 1379.537 | -0.2% |
| Simulation 2 | max velocity (m/s) | 175.571 | 177.197 | +0.9% |
| Simulation 2 | max velocity (m/s), BS without the credit | 175.571 | 175.548 | -0.0% |
| Simulation 2 | max Mach | 0.518 | 0.523 | +1.0% |
| Simulation 2 | time to apogee (s) | 16.315 | 16.300 | -0.1% |
| Simulation 2 | rail-exit velocity (m/s, OR at 2 ms step) | 15.262 | 15.314 | +0.3% |
| Simulation 2 | stability off rail (cal) | 2.128 | 2.117 | -0.5% |
| Simulation 2 | ground-hit velocity (m/s) | 6.062 | 6.062 | +0.0% |
| Simulation 2 | flight time (s) | 102.058 | 104.447 | +2.3% |

## Summary

| rocket | worst CP Δ (cal) | worst CNα Δ | worst Cd Δ (M≤2) | apogee Δ | max v Δ |
|---|---|---|---|---|---|
| dual_parachute | -0.022 | -0.7% | +11.0% | -0.5% | +0.5% |
| pods_powered | +39.324 | -1.7% | +8.4% | +201.8% | +41.8% |
| pods_winglets | +0.027 | -1.7% | +10.1% | +1.5% | +1.8% |
| simple_conical_nose | -0.019 | -1.0% | -2.3% | +3.2% | +3.0% |
| simple_model_rocket | -0.020 | -1.0% | +13.7% | +3.2% | +2.8% |
| spaceport_final | -0.024 | -0.8% | +11.7% | -1.7% | +0.6% |
| spaceport_subscale_rev4 | -0.043 | -1.6% | +13.2% | -1.6% | -0.6% |
| tube_fin_rocket | +4.389 | -0.0% | -29.5% | +1.9% | +2.9% |
| two_stage | -0.053 | -3.8% | +14.7% | +1.4% | +0.9% |
