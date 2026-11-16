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

## Desktop app

`crates/bowshock` is the Bowshock rocket builder (iced): build a rocket from a parts bin with KSP-style drag and drop,
edit every dimension with handles or the inspector, and fly it in **Simulate** or sweep it in **Analyze**. Its physics are
the engineering models in `bowshock-sim` (Barrowman CP with body lift, component drag build-up, RK4 rigid-body flight with pitch/yaw/roll dynamics),
fed by the rocket document in `bowshock-model` and the thrust curves in `bowshock-motors`. The UI follows the design kit in
`crates/bowshock/design/` (start at its `README.md`); module layout is in `crates/bowshock/src/ARCHITECTURE.md`.

```sh
cargo run --release -p bowshock               # open the app (Ctrl/Cmd+E loads the example rocket)
cargo run --release -p bowshock -- --example  # start with the example rocket
cargo run -q -p bowshock -- drive --example crates/bowshock/scripts/shell.drive out/shell   # scripted run + screenshots
```

| Crate | What it is |
|---|---|
| `bowshock` | the desktop app |
| `bowshock-model` | the rocket document: parts, materials, mass/CG, configurations, stages, the binary `.bowshock` format (`BOWSHOCK` magic + version, deflated CBOR) |
| `bowshock-sim` | Barrowman aerodynamics, flight simulation and analyses; `from_model` maps a document onto it |
| `bowshock-motors` | RASP `.eng` parser, the bundled ThrustCurve.org motor catalogue (every listed motor with a curve; data: [ThrustCurve.org](https://www.thrustcurve.org)) and a ThrustCurve.org search/download client |
| `bowshock-core`, `bowshock-cfd`, `bowshock-reference` | CFD drag library (not yet wired into the app) |

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
