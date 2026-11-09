# Bowshock

Open-source rocket aerodynamics CFD in Rust. Bowshock has a CPU f64 reference solver and a GPU solver built on
Luma/wgpu. Both solve the 2D axisymmetric Euler equations (cell-centred finite
volume, MUSCL + HLLC blended with Rusanov in shocks, SSP-RK3 with local time stepping). The solvers are validated against
Taylor-Maccoll cone solutions, the NACA RM A54H23 ogive-cylinder experiment (Perkins & Jorgensen, M = 1.98), and
transonic ogive-cylinder pressure and nose-drag data (NASA TN D-3516, NACA RM L53K17) from M = 0.7 to 1.5.

## Capabilities and limits

Current:
- Inviscid, axisymmetric flow at zero angle of attack, transonic and supersonic (M ≈ 0.7 and up).
- Characteristic (Riemann-invariant) far-field and pressure-outflow boundaries; large-domain stretched grids for
  cone, tangent-ogive, power, parabolic and Haack noses on a cylinder (sting to the outflow, so no base flow).
- Forebody pressure (wave) drag vs Mach, including the transonic drag rise.
- GPU solver in f32 storage with double-float evaluation that matches the f64 CPU solver on the cone, ogive and
  transonic cases and runs ~8–16× faster per iteration (see `bowshock-gpu/README.md`).

Known limits:
- Subsonic/transonic residuals stall ~3–4 orders down; runs stop when CD is stationary (≤0.4 % change vs 80k-iteration runs).
- Sharp shoulders (cone-cylinder) are not grid-converged below M ≈ 1.05.
- No viscous effects (skin friction, boundary-layer displacement), no base flow, no angle of attack.

Roadmap: subsonic convergence (implicit/multigrid) and low-Mach accuracy, base/wake region, boundary layer and skin
friction, aerodynamic coefficient tables.

## Quick start

```sh
cargo test --release                                  # unit/integration tests (GPU tests need a Vulkan device)
cargo run --release --bin cone -- --quick             # Taylor-Maccoll cone validation (CPU)
cargo run --release --bin ogive -- --sweep            # ogive-cylinder validation M = 1.98 (+ Mach sweep)
cargo run --release --bin transonic                   # transonic Mach sweep M = 0.7–1.5 vs TN D-3516 / RM L53K17 (CPU)
cargo run --release --bin cone_gpu                    # GPU vs CPU vs Taylor-Maccoll
cargo run --release --bin ogive_gpu                   # ogive-cylinder on the GPU vs CPU
cargo run --release --bin transonic_gpu               # transonic sweep on the GPU vs CPU
cargo run --release -p bowshock-cfd --example animate # Mach-number animation of the ogive-cylinder (see the example's docs for flags)
```

Results are written to `out/` (git-ignored).

## Crates

| crate | description | docs |
|---|---|---|
| `bowshock-cfd` (lib `bowshock_cfd`) | CPU f64 reference solver, characteristic BCs, large-domain body grids, Taylor-Maccoll and shock-expansion theory, validation drivers `cone`, `ogive`, `transonic` | [README](bowshock-cfd/README.md) |
| `bowshock-gpu` (lib `bowshock_gpu`) | f32/double-float Luma/WGSL port of the solver, validated against the CPU reference; drivers `cone_gpu`, `ogive_gpu`, `transonic_gpu` | [README](bowshock-gpu/README.md) |

`bowshock-gpu` depends on [Luma](https://github.com/flipnoteman/Luma) (`main` branch) via git. To develop against a local Luma checkout, add a `[patch."https://github.com/flipnoteman/Luma.git"]` entry pointing at it.

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
