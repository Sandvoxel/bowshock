# Bowshock

Open-source rocket aerodynamics CFD in Rust. Bowshock has a CPU f64 reference solver and a GPU solver built on
Luma/wgpu. Both solve the 2D axisymmetric Euler equations (cell-centred finite
volume, MUSCL + HLLC blended with Rusanov in shocks, SSP-RK3 with local time stepping). The solvers are validated against
Taylor-Maccoll cone solutions and against the NACA RM A54H23 ogive-cylinder experiment (Perkins & Jorgensen, M = 1.98).

## Capabilities and limits

Current:
- Supersonic, inviscid, axisymmetric flow at zero angle of attack.
- Cone and tangent-ogive-cylinder bodies on structured body-fitted grids.
- GPU solver in f32 storage that converges like the f64 CPU solver on the 12 cone validation cases (see `bowshock-gpu/README.md`).

Not yet supported: transonic and subsonic flow (the far-field boundary conditions are not suitable), viscous effects, angle of attack.

Roadmap: transonic far-field boundary conditions, subsonic convergence, boundary layer, aerodynamic coefficient tables.

## Quick start

```sh
cargo test --release                                  # unit/integration tests (GPU tests need a Vulkan device)
cargo run --release --bin cone -- --quick             # Taylor-Maccoll cone validation (CPU)
cargo run --release --bin ogive -- --sweep            # ogive-cylinder validation M = 1.98 (+ Mach sweep)
cargo run --release --bin cone_gpu                    # GPU vs CPU vs Taylor-Maccoll
cargo run --release -p bowshock-cfd --example animate # Mach-number animation of the ogive-cylinder (see the example's docs for flags)
```

Results are written to `out/` (git-ignored).

## Crates

| crate | description | docs |
|---|---|---|
| `bowshock-cfd` (lib `bowshock_cfd`) | CPU f64 reference solver, Taylor-Maccoll and shock-expansion theory, validation drivers `cone`, `ogive` | [README](bowshock-cfd/README.md) |
| `bowshock-gpu` (lib `bowshock_gpu`) | f32 Luma/WGSL port of the solver, validated against the CPU reference; driver `cone_gpu` | [README](bowshock-gpu/README.md) |

`bowshock-gpu` depends on [Luma](https://github.com/flipnoteman/Luma) (`main` branch) via git. To develop against a local Luma checkout, add a `[patch."https://github.com/flipnoteman/Luma.git"]` entry pointing at it.

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
