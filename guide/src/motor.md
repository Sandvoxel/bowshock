# Motor-on base flow

With the motor running, the base of the rocket is not a dead-air wake: the exhaust jet leaves the
nozzle exit in the centre of the base and the annulus around it sees the jet's influence. bowshock
models this by solving the jet as part of the base flow. Power-off and power-on drag are two
analyses of the same body.

## MotorSpec

```rust
use bowshock_core::MotorSpec;

let motor = MotorSpec {
    exit_diameter: 0.06,
    exit_mach: 2.5,
    exit_pressure_pa: 101_325.0,
    total_temperature_k: 1500.0,
};
motor.validate(0.08).unwrap(); // base diameter 0.08 m
```

| Field | Unit | Rule |
| --- | --- | --- |
| `exit_diameter` | m | finite, > 0, at most 0.95 of the base diameter |
| `exit_mach` | - | in [1, 5] (the jet leaves the nozzle supersonic) |
| `exit_pressure_pa` | Pa | static pressure at the exit; finite and > 0 |
| `total_temperature_k` | K | total temperature of the jet, in [200, 4000] |

`MotorSpec::validate(base_diameter)` takes the [base diameter](body.md#base-diameter) [m] and
returns `InvalidInput` with the field named (`motor.exit_diameter`, `motor.exit_mach`,
`motor.exit_pressure_pa`, `motor.total_temperature_k`); `Analysis::run` calls it.

```rust
use bowshock_core::MotorSpec;

let motor = MotorSpec { exit_diameter: 0.08, exit_mach: 2.5, exit_pressure_pa: 101_325.0, total_temperature_k: 1500.0 };
let err = motor.validate(0.08).unwrap_err();
assert!(err.to_string().starts_with("motor.exit_diameter"));
```

## The jet model

The jet is **air with gamma = 1.4** (cold-air model), leaving the base through a circular nozzle
exit of `exit_diameter`, centred on the axis, at a **uniform supersonic axial state**: no nozzle
boundary layer, no lip angle, no real-gas exhaust species. From the free-stream air
(temperature `T`, pressure `p`, density `rho`, speed of sound `a`) and the motor:

```text
T_e = T_t / (1 + 0.2 M_e^2)          exit static temperature
p_e = exit_pressure_pa               exit static pressure
rho_e = p_e / (R T_e)                (the solver uses T_e/T, p_e/p, ratios of the free stream)
u_e = M_e sqrt(1.4 R T_e)            exit speed
```

The solver works in free-stream units (`rho = 1`, `a = 1`, `p = 1/gamma`, `T = 1`); the exit
state in these units is `[p_e/p / (T_e/T), M_e sqrt(T_e/T), 0, p_e/p / gamma]` (density, axial
velocity, radial velocity, pressure).

**Jet turbulence** (RANS): turbulence intensity 1 % of the exit speed, `k = 1.5 (0.01 u_e)^2`, and
`omega` such that the eddy-to-molecular viscosity ratio at the exit is 10,
`omega = rho_e k / (10 mu_e)`, with `mu_e` Sutherland's viscosity at `T_e`.

In the grid, the base face is split: rows within the exit radius are an inflow with the jet state;
the rest is the wall of the annulus. The jet is part of the solved base wake.

## Requirements

A motor needs

- `Analysis::viscous(flight)` (the jet is a RANS boundary): `InvalidInput("motor-on analysis needs Analysis::viscous")` otherwise;
- the base wake: `Wake::Sting` gives `InvalidInput("motor-on analysis needs the base wake")`, and
  the default of a viscous analysis is the base wake;
- a dimensional body from `Analysis::new`.

## Drag with the motor on

- `Drag::base` is the drag of the **annulus only** (the base area outside the nozzle exit), from the
  solver; `Drag::base_modelled` is `false`. The jet's momentum and pressure thrust is **not** a drag
  term and is not in `total`; account for thrust separately.
- `PointDetail::cp_base` is the area-mean base pressure coefficient over the annulus.
- `Polar::motor_on` is `true`.

## Power-off and power-on

The power-off curve is the same body without `.motor(..)` and with the viscous base wake. Run both
and compare:

```rust,no_run
use bowshock_core::{
    AftSpec, Analysis, Atmosphere, BodySpec, FlightConditions, MotorSpec, NoseShape, NoseSpec, Resolution,
};

fn main() -> Result<(), bowshock_core::Error> {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    };
    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };
    let motor = MotorSpec {
        exit_diameter: 0.06,
        exit_mach: 2.5,
        exit_pressure_pa: 101_325.0,
        total_temperature_k: 1500.0,
    };
    let machs = [0.5, 0.8, 1.5];

    let off = Analysis::new(&spec)?.resolution(Resolution::Coarse).viscous(flight);
    let on = off.clone().motor(motor);

    let power_off = off.run(&machs)?;
    let power_on = on.run(&machs)?;
    assert!(!power_off.motor_on && power_on.motor_on);

    for (a, b) in power_off.points.iter().zip(&power_on.points) {
        println!(
            "M {}: power-off {:.4} (base {:.4}), power-on {:.4} (annulus base {:.4})",
            a.mach, a.drag.total, a.drag.base, b.drag.total, b.drag.base
        );
    }
    Ok(())
}
```

The exit pressure and total temperature are properties of your motor at the flight condition;
the exit pressure usually changes with altitude when the nozzle is not ideally expanded. Run one
analysis per condition.

## In the app: the CFD table's power-on part

The app's CFD table (Analyze, `BOWSHOCK_ENGINE=3d` or the legacy engine) also solves the jet of the stack's motor, and the
flight simulator flies it while the motor burns. The motor files carry a case diameter and a thrust
curve and no nozzle, so the jet is derived (`analyze::cfd::plan::jet_of`): an air jet at exit Mach 2.5 and
3000 K total temperature whose thrust is the motor's average (impulse over burn time),

```text
F = p_e A_e (1 + gamma M_e^2) - p_sl A_e        (static thrust, p_sl = 101 325 Pa)
A_e = F / (gamma M_e^2 p_sl)                    (the exit matched to sea level), kept between 0.5 and 0.8 of the case diameter
p_e = (F / A_e + p_sl) / (1 + gamma M_e^2)      (at the kept area)
```

so it does not depend on the flight altitude (the solver puts it in the free stream's units: the exit-to-ambient pressure
ratio rises as the rocket climbs). At the Mach numbers 0.5, 0.9, 1.3 and 2.3 that the table has, the body alone (no fins:
the axisymmetric wedge) is solved with the jet on and with it off on the table's grid, and the ratios of friction,
nose/aft pressure and base drag are stored with the table (`CfdTable::power_on`). The flight multiplies the table's parts by
the ratios, linear between the nodes and held beyond them, and no longer takes the motor's exit area out of the base drag. A
table without a power-on part for the motor it flies (solved before the motor changed) keeps that old correction and the
Simulate screen says so. The fins' drag is not scaled by the ratios (the finned sector with the jet differs from the body's by
at most 5 % in the base-drag ratio, DESIGN3D item 58).

## Limits

- The jet is a cold-air model with a uniform exit state; real exhaust has a different gamma and
  molecular weight, and a nonuniform exit profile.
- One nozzle, centred on the axis, with `exit_diameter` at most 0.95 of the base diameter.
- The jet cannot be combined with `Wake::Sting`.

## Accuracy

**Not validated: the comparison with NASA TP-2005 fails.** The test case is a strut-mounted
ogive-cylinder (6.35 cm, base at 82.55 cm) with cold air jets through a 5.08 cm exit at
Mach 1.65-2.50, exit pressures 8-13 times the free-stream pressure (full table in
[Validation](validation.md#motor-on-base-pressure-nasa-tp-2005-d2)):

- at `Resolution::Medium` all 11 points converge (`Stationary`), but the base pressure is
  0.59-1.05 p_inf where the data have 0.95-1.67 p_inf (errors 0.20-0.96 p_inf), and it falls with
  exit Mach where the data rise: the plume-induced separation that lifts the measured base
  pressure above p_inf is not reproduced.

Use motor-on results only to see the trend of a modestly under-expanded jet, and check `quality`
and the base Cp for plausibility.
