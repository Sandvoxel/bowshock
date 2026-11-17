# Results

`Analysis::run` returns a `Polar`. All drag coefficients are dimensionless and referenced to the
area of the [reference diameter](body.md#reference-diameter-and-reference-area).

```rust,no_run
use bowshock_core::{AftSpec, Analysis, BodySpec, NoseShape, NoseSpec, Quality, Resolution};

fn main() -> Result<(), bowshock_core::Error> {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }],
    };
    let polar = Analysis::new(&spec)?.resolution(Resolution::Coarse).run(&[0.8, 1.2])?;
    for p in &polar.points {
        if p.quality == Quality::Failed {
            eprintln!("M {}: failed ({:?})", p.mach, p.detail.stop_reason);
            continue;
        }
        println!("M {}: CD {:.4} ({:?}, {} iterations)", p.mach, p.drag.total, p.quality, p.detail.iterations);
        for w in &p.warnings {
            println!("  warning: {w:?}");
        }
    }
    Ok(())
}
```

## Polar

| Field | Type | Meaning |
| --- | --- | --- |
| `points` | `Vec<PolarPoint>` | one per requested Mach number, in the requested order |
| `backend` | `BackendUsed` | `Cpu` or `Gpu`: what ran the sweep |
| `fallback` | `Option<String>` | why the GPU was not used under `Backend::Auto` |
| `motor_on` | `bool` | the analysis had a [motor](motor.md) |

## PolarPoint

| Field | Meaning |
| --- | --- |
| `mach` | Mach number |
| `alpha` | angle of attack [rad]; always 0.0 |
| `drag` | the [`Drag`](#drag) coefficients |
| `detail` | the [`PointDetail`](#pointdetail) solver summary |
| `quality` | [`Quality`](#quality) of the point |
| `warnings` | [`Warning`](#warnings)s: conditions that make the point less reliable than its quality suggests |
| `cd_uncertainty` | `Some(d)`, `d` the absolute difference of the total drag to the neighbouring grid level, with `estimate_uncertainty(true)`; else `None` |
| `field` | `Some(`[`Field`](#field)`)`, the final flow field of the point, with `keep_fields(true)`; else `None` |

## Drag

| Field | Meaning |
| --- | --- |
| `forebody_pressure` | pressure drag of the nose and the aft sections |
| `friction` | skin friction; `None` for Euler analyses |
| `base` | base drag: from the solver for a viscous analysis with a base wake, otherwise a correlation. With a motor: the annulus around the nozzle exit only; thrust is not a drag term |
| `base_modelled` | `true` when `base` is the correlation (Euler analyses, and viscous analyses with `Wake::Sting`) |
| `fins` | `Some(FinDrag { friction, pressure })` when [fins](fins.md) were given |
| `total` | `forebody_pressure + friction + base + fins.friction + fins.pressure`; non-finite only for a `Failed` point |

## PointDetail

`PointDetail` is the solver summary of one point. Lengths are in metres and coefficients are
referenced to the reference area, like `Drag`.

| Field | Meaning |
| --- | --- |
| `iterations` | iterations run |
| `residual_drop_orders` | orders of magnitude the residual dropped from its peak to the last iteration |
| `stop_reason` | why the iteration stopped ([`StopReason`](#stopreason)) |
| `cd_nose` | pressure drag of the nose |
| `cd_aft` | pressure drag of each aft section, one entry per `AftSpec`, in order |
| `cd_friction` | skin friction; 0 for the Euler equations |
| `cd_base_solver` | base drag computed by the solver; NaN without a base wake |
| `cp_base` | area-mean base pressure coefficient (the annulus with a motor); NaN without a base wake |
| `wall` | `Vec<WallCp>`: body-surface faces in axial order (not the sting extension) |
| `y_plus_max` | largest `y+` of the first wall cell; NaN for the Euler equations |
| `farfield_dp` | largest relative pressure deviation from the free stream at the outer boundaries |

`Drag::forebody_pressure` equals `cd_nose + sum(cd_aft)`.

### WallCp

`WallCp { x, r, cp }`: axial position `x` [m] (from the nose tip), radius `r` [m] and pressure
coefficient `cp` (relative to the free-stream static pressure and dynamic pressure) of one face of the
body surface. `x` and `r` are at the face centre. Use it to plot the surface pressure.

## Quality

Derived from how the iteration stopped:

| `Quality` | Meaning | What to do |
| --- | --- | --- |
| `Converged` | the residual dropped by the requested number of orders | use it |
| `Stationary` | the residual did not drop enough but the monitored quantity (drag or base pressure) is stationary, or (viscous) the drag oscillates in a bounded cycle with a stationary mean and the point is taken at the mean | usable; typical at transonic Mach where residuals stall 3 to 4 orders down, and for subsonic viscous points (see `Warning::OscillatingDrag`) |
| `Stalled` | the iteration cap was reached without any stop criterion | do not trust it; raise `max_iterations`, use `Resolution::Coarse` to check, or check for a `Warning::UnsteadyWake` |
| `Failed` | non-finite or non-physical state (negative density or pressure), or non-finite drag | discard; try another resolution, or a Mach number nearby |

## Warnings

| `Warning` | Raised when | Meaning |
| --- | --- | --- |
| `SharpCornerTransonic` | Mach in [0.7, 1.05) and a convex profile corner sharper than 2 deg | the shoulder suction peak grows with grid refinement, so the drag is grid-sensitive; check with `estimate_uncertainty` |
| `UnsteadyWake { cp_base_range }` | base wake, residual did not drop (`stop_reason != ResidualDrop`), and the base Cp varies by at least 1e-3 over the monitor window | the wake is unsteady; the steady solution is only a time average |
| `FarFieldDisturbed { dp }` | outer-boundary pressure deviates from free stream by more than 2 % | the domain may be too small for this body/Mach |
| `WallResolution { y_plus_max }` | `y+` of the first cell above 5 | the wall-resolved boundary layer is not resolved; friction is suspect |
| `OscillatingDrag { cd_range }` | a viscous point stopped by `MeanStationary` | the total drag oscillated by `cd_range` (reference area) over the last 12000 iterations; the reported state is the one where the drag crossed the cycle mean (the steady RANS solution does not exist; treat `cd_range` as the band of the result) |

## StopReason

`PointDetail::stop_reason`:

| `StopReason` | Meaning |
| --- | --- |
| `ResidualDrop` | the residual dropped by the requested number of orders |
| `CdStationary` | the forebody or total drag is stationary over the monitor window |
| `BasePressureStationary` | the base pressure is stationary over the monitor window |
| `MeanStationary` | viscous runs: the total drag oscillates in a bounded limit cycle (peak-to-peak at most 5 % of the mean) whose mean over two consecutive 6000-iteration windows agrees to 0.2 %; the run stops where the drag crosses that mean |
| `ResidualPlateau` | the residual stopped decreasing |
| `MaxIterations` | the iteration cap was reached |
| `NonFinite` | the state became non-finite or non-physical |

## Field

`Field` is the flow the solver left at the end of a point: the grid, which cells are blocked, and the primitive state in each
cell. It is the state the reported drag was integrated from, on the grid of the requested resolution and wake treatment (with
`estimate_uncertainty(true)` the point is also solved one grid level away; that field is not kept). It is off unless asked for
(`keep_fields(true)`): on the 1.1 m example body a field is about 0.5 MB on the coarse viscous grid and 1.1 MB on the medium one.

| Field | Meaning |
| --- | --- |
| `mach`, `iteration`, `wake` | the free-stream Mach number, the iterations done, the grid (`Wake::Sting` or `Wake::Base`) |
| `ni`, `nj` | cells along `i` (downstream) and `j` (away from the axis) |
| `x`, `r` | node coordinates in metres, `(ni + 1) * (nj + 1)` each, node `(i, j)` at `j * (ni + 1) + i`; `x` runs from the nose tip, `r` from the axis |
| `solid` | per cell, `ni * nj`, cell `(i, j)` at `j * ni + i`: blocked (inside the body or the sting), not part of the flow |
| `rho`, `u`, `v`, `p` | per cell: density over the free-stream density, axial and radial velocity over the free-stream speed of sound, pressure over `rho_inf c_inf^2` |
| `freestream` | `[rho, u, v, p]` of the free stream in the same units, `[1, Mach, 0, 1 / GAMMA]`, which is all that is needed to form coefficients |

`mach_numbers()`, `pressure_coefficients()` and `density_gradient()` derive the Mach number, the pressure coefficient and the
magnitude of the density gradient per cell (NaN in blocked cells), `wall()` the pressure coefficient along the body surface by the
rule the point's own `detail.wall` uses, and `surface()` the lower edge of the flow, the body's outline. `Analysis::on_field(every,
f)` hands a snapshot of the run in flight to `f` every `every` iterations, see [Progress and cancelling](sweeps.md#progress-and-cancelling).

## serde

With the `serde` feature, the inputs (`BodySpec`, `NoseSpec`, `NoseShape`, `AftSpec`,
`Atmosphere`, `FlightConditions`, `FinSpec`, `FinProfile`, `MotorSpec`) and the results (`Polar`,
`PolarPoint`, `PointDetail`, `WallCp`, `Field`, `Drag`, `FinDrag`, `Quality`, `Warning`, `BackendUsed`,
`StopReason`) implement `Serialize` and `Deserialize`.

```rust
# #[cfg(feature = "serde")]
# fn main() -> Result<(), Box<dyn std::error::Error>> {
use bowshock_core::{AftSpec, BodySpec, NoseShape, NoseSpec};

let spec = BodySpec {
    nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
    aft: vec![AftSpec::Tube { length: 0.60 }],
};
let json = serde_json::to_string(&spec)?;
let back: BodySpec = serde_json::from_str(&json)?;
assert_eq!(back, spec);
# Ok(())
# }
# #[cfg(not(feature = "serde"))]
# fn main() {}
```

JSON has no NaN: `serde_json` writes a NaN `f64` as `null`. The fields that are documented to
hold NaN (`PointDetail::cd_base_solver`, `PointDetail::cp_base`, `PointDetail::y_plus_max` and a
`Failed` point's `Drag::total`) read `null` back as NaN, so a `Polar` round-trips through JSON.
Other `f64` fields are not NaN-tolerant.
