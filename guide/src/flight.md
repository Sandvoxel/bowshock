# Flight conditions

A viscous analysis (and fin drag, and a motor) needs the static state of the air the rocket flies
through. `FlightConditions` carries an `Atmosphere`:

```rust
use bowshock::{Atmosphere, FlightConditions};

let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 3000.0 } };
let air = flight.atmosphere.state().unwrap();
println!("T {:.2} K, p {:.0} Pa, rho {:.4} kg/m^3, a {:.2} m/s, mu {:.3e} Pa s",
    air.temperature_k, air.pressure_pa, air.density, air.sound_speed, air.viscosity);
assert!((air.temperature_k - 268.65).abs() < 1e-9);
```

## Atmosphere

| Variant | Meaning | Range |
| --- | --- | --- |
| `Atmosphere::Isa { altitude_m }` | International Standard Atmosphere at a geometric altitude [m] | 0 to 20 000 m |
| `Atmosphere::Explicit { temperature_k, pressure_pa }` | static temperature [K] and pressure [Pa] given directly, for wind tunnels and other non-standard air | 100 to 400 K; pressure finite and > 0 |

`Atmosphere::state()` returns `Error::InvalidInput` outside these ranges:

```rust
use bowshock::Atmosphere;

assert!(Atmosphere::Isa { altitude_m: 25_000.0 }.state().is_err());
assert!(Atmosphere::Explicit { temperature_k: 300.0, pressure_pa: 0.0 }.state().is_err());
```

## AirState

`Atmosphere::state()` computes an `AirState`, all fields in SI:

| Field | Unit | How |
| --- | --- | --- |
| `temperature_k` | K | ISA: 288.15 K minus 0.0065 K/m up to 11 km, 216.65 K above; explicit: as given |
| `pressure_pa` | Pa | ISA troposphere power law, stratosphere exponential; explicit: as given |
| `density` | kg/m^3 | `p / (R T)`, `R = 287.05287 J/(kg K)` |
| `sound_speed` | m/s | `sqrt(1.4 R T)` |
| `viscosity` | Pa s | Sutherland's law, `mu = 1.716e-5 (T / 273.15)^1.5 (273.15 + 110.4) / (T + 110.4)` |

## Reynolds number

You do not give a Reynolds number or a velocity; the Mach number of each sweep point does that.
For a point at Mach `M`, the free-stream speed is `U = M a`, and the solver's Reynolds number is
based on the speed of sound and the nose diameter `d_nose`:

```text
Re_c = rho a d_nose / mu          (the flow solver's reference Reynolds number)
Re_L = rho M a L / mu             (length-based Reynolds number at Mach M, e.g. L = fin mean chord for fins)
```

so the Reynolds number grows linearly with Mach at fixed altitude. The Sutherland constant and
the temperature enter the solver's viscosity law the same way as in `AirState`.

```rust
use bowshock::{Atmosphere, FlightConditions};

let air = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } }.atmosphere.state().unwrap();
let (mach, length) = (0.8, 1.0);
let re = air.density * mach * air.sound_speed * length / air.viscosity;
println!("Re at Mach {mach} over {length} m: {re:.3e}");
assert!(re > 1e7 && re < 2e7);
```

To reproduce a wind-tunnel test, give the tunnel's static temperature and pressure with
`Atmosphere::Explicit`; compute them from the stagnation temperature and the Mach number
(`T = T0 / (1 + 0.2 M^2)`) and from the unit Reynolds number (`p = rho R T`,
`rho = Re_per_m mu / (M a)`).
Each Mach number then needs its own `Analysis`, since the conditions change with Mach.
