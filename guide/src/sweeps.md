# Mach sweeps

`Analysis` is a builder. `Analysis::new(&spec)?` starts from defaults, each method returns the
`Analysis` (so calls chain), and `run(&machs)` solves one flow per Mach number and returns a
[`Polar`](results.md). `run` takes `&self`; clone an `Analysis` to run variants.

```rust,no_run
use bowshock::{
    AftSpec, Analysis, Atmosphere, Backend, BodySpec, FlightConditions, NoseShape, NoseSpec, Resolution,
};

fn main() -> Result<(), bowshock::Error> {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }],
    };
    let base = Analysis::new(&spec)?.resolution(Resolution::Medium).backend(Backend::Auto);

    // Euler: forebody pressure drag + modelled base drag
    let euler = base.clone().run(&[0.5, 0.9, 1.1, 2.0])?;

    // RANS: adds skin friction, base drag from the solved wake
    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };
    let rans = base.viscous(flight).run(&[0.5, 0.9, 1.1, 2.0])?;

    println!("{} {}", euler.points.len(), rans.points.len());
    Ok(())
}
```

## Mach numbers

Every Mach number must be finite and in [0.1, 3.0], the list must not be empty; otherwise `run`
returns `InvalidInput`. Internally the points are solved in ascending Mach order; the result is
in the order you gave. Duplicates are allowed.

## Builder methods

| Method | Default | Meaning |
| --- | --- | --- |
| `reference_diameter(d_m)` | largest diameter | [reference diameter](body.md#reference-diameter-and-reference-area) [m] of the coefficients |
| `resolution(r)` | `Medium` | grid level |
| `backend(b)` | `Auto` | where to run |
| `viscous(flight)` | Euler | RANS instead of Euler, at these [flight conditions](flight.md) |
| `wake(w)` | sting (Euler), base (viscous) | treatment of the region behind the body |
| `max_iterations(n)` | per-Mach default | cap on the solver iterations of each point |
| `warm_start(on)` | off | start each Mach number from the previous converged field |
| `estimate_uncertainty(on)` | off | also run one grid level away |
| `fins(f)` | none | add [fin drag](fins.md) |
| `motor(m)` | none | [motor-on](motor.md) base flow |

### Resolution

`Resolution::{Coarse, Medium, Fine}` are grid levels 0, 1, 2. Each level roughly doubles the
number of cells along the body; `Coarse` is for checks and sweeps with many points, `Medium` is
the default, `Fine` for final numbers. The cost grows with the level
([Performance](performance.md)). Grids are checked before the solve: a body whose grid is too stretched returns
`Error::Grid`.

### Backend

`Backend::Auto` runs on the GPU if there is one and otherwise on the CPU (reason in
`Polar::fallback`); a GPU that fails during the sweep, or a GPU solver panic, falls back to the CPU
the same way. `Backend::Cpu` forces the CPU. `Backend::Gpu` returns an error when no GPU is
available. The GPU and CPU solvers solve the same equations; on the validation cases they agree
to the tolerances the repository tests ([Validation](validation.md)). The backend that ran is in
`Polar::backend`.

### Euler or RANS

Without `viscous`, the analysis solves the **Euler equations** (inviscid):

- drag = forebody pressure drag (nose and aft sections) + base drag;
- `drag.friction` is `None`;
- the base drag is a correlation, `0.12 + 0.13 M^2` for `M < 1` and `0.25 / M` for `M >= 1`
  (OpenRocket technical documentation, eq. 3.94, after Fleeman), scaled by the base-to-nose
  area ratio; `drag.base_modelled` is `true`. The inviscid wake has no viscous mixing layer, so
  its own base pressure is not physical;
- the default wake treatment is a sting: the body's last tube continues to the outflow, no base.

With `viscous(flight)` it solves the **Reynolds-averaged Navier-Stokes equations** with the
k-omega SST turbulence model, on a wall-resolved grid (first cell below `y+` = 1 on the validation body), fully
turbulent, smooth walls:

- `drag.friction` is `Some(..)` from the wall shear;
- with the base wake (the default), `drag.base` comes from the solver, `base_modelled` is `false`;
- `Warning::WallResolution` flags a point whose first cell reached `y+ > 5`.

Viscous analyses need an `Analysis` built with `Analysis::new` (dimensional input) and a valid
atmosphere.

### Wake

`Wake::Base` solves a finite body with a flat base and a wake region behind it. `Wake::Sting`
continues the body as a cylinder to the outflow boundary (the sting-mounted configuration of wind
tunnel models): there is no base, and the base drag of a sting body is the correlation above. Set
it only to override the default. A [motor](motor.md) needs `Wake::Base`.

### Iteration cap

`max_iterations(n)` lowers the per-point iteration cap of whichever numerics is chosen for that
Mach number and level (it never raises it). Use a small cap for smoke tests; capped points that
do not converge report `Quality::Stalled`.

### Warm start

`warm_start(true)` starts each Mach number from the converged field of the previous one. It is
**off by default**: measured on the solver, warm-started points needed 2 to 5 times more
iterations than cold starts at every Mach spacing from 0.05 to 0.3, and the GPU implicit scheme
diverges on a warm start to Mach 0.8. A warm-started point that ends `Failed` or `Stalled` is
re-run cold, and the point after a `Failed` point always starts cold. Turn it on only after
measuring that it helps your sweep.

### Uncertainty estimate

`estimate_uncertainty(true)` also runs the sweep one grid level away (level - 1, or level + 1 on
the coarsest level) and reports the absolute
difference of the total drag in `PolarPoint::cd_uncertainty`. It costs a second sweep. It is a
grid-sensitivity indicator, not a confidence interval: it is `None` when either point failed or
the option is off.

## What `run` rejects

| Condition | Error |
| --- | --- |
| no Mach numbers, or one outside [0.1, 3.0] | `InvalidInput` |
| reference diameter not finite and > 0 | `InvalidInput` |
| fins without `viscous`, or invalid `FinSpec` | `InvalidInput` |
| motor without `viscous`, with `Wake::Sting`, or invalid `MotorSpec` | `InvalidInput` |
| grid too stretched for the body | `Grid` |
| `Backend::Gpu` without a GPU (or without the `gpu` feature) | `Gpu` / `Unsupported` |
| a solver invariant failed (a bug; panics are caught) | `Internal` |

`run` does not panic on valid or invalid input; solver problems at a Mach number show up as
`Quality::Failed` on that point, not as an `Err`.
