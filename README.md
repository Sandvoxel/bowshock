# bowshock

Rocket drag from CFD, as a Rust library. `bowshock-core` computes the zero-lift drag of an axisymmetric rocket from Mach 0.1 to 3:
forebody pressure, skin friction and base drag from the axisymmetric Euler or RANS (k-omega SST) equations, solved on the GPU
(Luma / wgpu, Vulkan) or the CPU. Fin drag comes from an empirical model, and a motor-on base flow includes the exhaust jet. The
solver itself is the separate crate [`bowshock-cfd`](crates/bowshock-cfd/README.md).

```toml
[dependencies]
bowshock-core = { git = "https://github.com/Sandvoxel/bowshock" }
```

```rust,no_run
use bowshock_core::{AftSpec, Analysis, Atmosphere, BodySpec, FinProfile, FinSpec, FlightConditions, NoseShape, NoseSpec};

fn main() -> Result<(), bowshock_core::Error> {
    let body = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    };
    let fins = FinSpec { count: 4, root_chord: 0.12, tip_chord: 0.06, span: 0.08, sweep: 0.06, thickness: 0.003, profile: FinProfile::Rounded };
    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };

    let polar = Analysis::new(&body)?.fins(fins).viscous(flight).run(&[0.5, 0.9, 1.5])?;
    for p in &polar.points {
        println!("M {:.2}: CD {:.4} ({:?})", p.mach, p.drag.total, p.quality);
    }
    Ok(())
}
```

- Guide: <https://sandvoxel.github.io/bowshock/> (getting started, body and fin input, motor-on runs, results, validation, limits).
- API reference: <https://sandvoxel.github.io/bowshock/main/api/bowshock_core/>.
- Limits in short: axisymmetric bodies, zero angle of attack, fully turbulent smooth walls; fins are empirical and under-predicted
  against wind-tunnel data; motor-on base pressure is not validated. Details and measured errors are in the guide's validation chapter.
- Working on bowshock itself: [developer notes and validation drivers](crates/bowshock-validate/README.md).

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
