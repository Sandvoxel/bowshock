# Getting started

## Add the crate

bowshock is used from git:

```toml
[dependencies]
bowshock-core = { git = "https://github.com/Sandvoxel/bowshock" }
```

You need a recent stable Rust toolchain. Cargo features:

| Feature | Default | Effect |
| --- | --- | --- |
| `gpu` | on | the GPU solver, through Luma (wgpu, Vulkan) |
| `serde` | off | `Serialize` / `Deserialize` for the input and result types ([Results](results.md#serde)) |

For a CPU-only build, with no Luma or wgpu dependency:

```toml
[dependencies]
bowshock-core = { git = "https://github.com/Sandvoxel/bowshock", default-features = false }
```

With `serde`:

```toml
[dependencies]
bowshock-core = { git = "https://github.com/Sandvoxel/bowshock", features = ["serde"] }
```

## GPU and CPU

By default (`Backend::Auto`) a sweep runs on the GPU when one is available and on the CPU
otherwise. A software Vulkan driver (Mesa llvmpipe) counts as no GPU, because the CPU solver is
faster than the software device. When the CPU ran instead of the GPU, `Polar::backend` is
`BackendUsed::Cpu` and `Polar::fallback` holds the reason as a string. `Backend::Cpu` forces the
CPU, `Backend::Gpu` makes a missing GPU an error. Without the `gpu` feature only the CPU exists
and `Backend::Gpu` returns `Error::Unsupported`. See [Mach sweeps](sweeps.md#backend).

## A first analysis

A 0.10 m tangent-ogive body: 0.30 m nose, 0.60 m tube, then a 0.10 m boattail to 0.08 m diameter,
in sea-level standard air, with the RANS equations (friction and base drag from the solution):

```rust,no_run
use bowshock_core::{
    AftSpec, Analysis, Atmosphere, BodySpec, FlightConditions, NoseShape, NoseSpec, Resolution,
};

fn main() -> Result<(), bowshock_core::Error> {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    };
    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };

    let polar = Analysis::new(&spec)?
        .resolution(Resolution::Coarse)
        .viscous(flight)
        .run(&[0.6, 0.9, 1.2])?;

    println!("backend: {:?}", polar.backend);
    if let Some(why) = &polar.fallback {
        println!("GPU not used: {why}");
    }
    println!("mach, cd_pressure, cd_friction, cd_base, cd_total, quality");
    for p in &polar.points {
        println!(
            "{}, {:.4}, {:.4}, {:.4}, {:.4}, {:?}",
            p.mach,
            p.drag.forebody_pressure,
            p.drag.friction.unwrap_or(0.0),
            p.drag.base,
            p.drag.total,
            p.quality
        );
    }
    Ok(())
}
```

All coefficients refer to the largest cross-section of the body (0.10 m diameter here) and are
dimensionless. The same example without `.viscous(flight)` solves the Euler equations: no friction
(`drag.friction` is `None`), and the base drag comes from a correlation (`drag.base_modelled` is
`true`).

The points come back in the order you gave the Mach numbers. Check `quality` on every point before
using `drag.total`; see [Results](results.md#quality).

The repository has the same flow as a runnable program: `cargo run --release -p bowshock-core --example polar`.
