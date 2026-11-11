# Bowshock

Open-source rocket aerodynamics CFD in Rust. Bowshock has a CPU f64 reference solver and a GPU solver built on
Luma/wgpu. Both solve the 2D axisymmetric Euler equations (cell-centred finite volume, MUSCL + HLLC blended with Rusanov
in shocks, SSP-RK3 with local time stepping or a line-implicit scheme) and, for viscous drag, the Reynolds-averaged
Navier-Stokes equations with the k-omega SST turbulence model. The solvers are validated against Taylor-Maccoll cone
solutions, the NACA RM A54H23 ogive-cylinder experiment (Perkins & Jorgensen, M = 1.98), and transonic ogive-cylinder
pressure and nose-drag data (NASA TN D-3516, NACA RM L53K17) from M = 0.7 to 1.5. The viscous terms are checked against
flat-plate skin friction (Blasius, a turbulent correlation) and the NASA TM X-2439 wall and base pressures.

## Capabilities and limits

Current:
- Facade crate `bowshock` (user API): `Analysis::new(&BodySpec)?.run(&machs)?` returns zero-lift drag vs Mach as `Drag { forebody_pressure, friction, base, base_modelled, total }`, with a `Quality` flag (`Converged`, `Stationary`, `Stalled`, `Failed`) and `Warning`s per point.
- Bodies: a nose (cone, tangent ogive, power, parabolic or Haack series) followed by any sequence of tubes and conical transitions, in metres (dimensional input; lengths are non-dimensionalised by the nose diameter internally).
- Inviscid, axisymmetric flow at zero angle of attack, from M ≈ 0.1 (low-Mach flux correction) through transonic to supersonic.
- Characteristic (Riemann-invariant) far-field and pressure-outflow boundaries; large-domain stretched grids for cone, tangent-ogive, power, parabolic and Haack noses, followed by the piecewise aft body (sting to the outflow, or a flat base with a wake region).
- Forebody pressure (wave) drag vs Mach, including the transonic drag rise.
- Subsonic bodies with a sting or a flat blunt base and wake (line-implicit convergence scheme). Inviscid base drag comes from an empirical free-flight correlation (OpenRocket technical documentation, eq. 3.94), because the inviscid wake gives no physical base pressure.
- Viscous analysis `Analysis::viscous(FlightConditions)`: wall-resolved, steady, fully turbulent RANS (k-omega SST) on the same grids, with an ISA atmosphere from 0 to 20 km (or an explicit temperature and pressure). Friction drag and base pressure come from the solution.
- Checks of the viscous terms, measured: laminar flat-plate skin friction within +0.64 % (x = 0.5) and +1.63 % (x = 0.9) of Blasius; SST flat-plate skin friction -4.25 % against `0.37 / log10(Re_x)^2.584` at Re_x = 4.8e6, first-cell y+ = 0.36.
- GPU solver in f32 storage with double-float evaluation that matches the f64 CPU solver on the cone, ogive and transonic cases and runs ~8–16× faster per iteration (see `bowshock-gpu/README.md`). RANS runs on the GPU too. The GPU is the default backend of the facade (`gpu` cargo feature, on by default), with an automatic CPU fallback when the GPU cannot be used (`Polar::fallback` says why); `--no-default-features` builds CPU-only.
- Inviscid robustness matrix (3 bodies x 10 Mach numbers from 0.3 to 2.5, Coarse, GPU): 30 of 30 points Converged or Stationary, about 20 s in total.

Known limits:
- Zero angle of attack only: `alpha` is reported in every result and is always 0.
- Axisymmetric bodies only (no fins).
- Inviscid runs model base drag with a Mach-only correlation; the inviscid wake gives no physical base pressure.
- RANS is steady and fully turbulent (no transition model). It is validated on flat plates (laminar against Blasius, SST against a turbulent correlation) and against the Fox TM X-2439 wall and base pressures, not against total-drag measurements. Body check (tangent ogive 0.30 x 0.10 m + 0.60 m tube, ISA sea level, Mach 0.5, Coarse): friction drag 0.0900 (0.964 of the flat-plate estimate over the wetted area), base Cp -0.123, total CD 0.213, y+ max 0.65, about 27 s on the CPU.
- Fox TM X-2439 (`cargo run --release --bin subsonic -- --quick --validate --rans`, table `out/subsonic/tables_validate_rans.md`): RANS base Cp +0.049 to +0.072 against measured +0.069 to +0.132 (Euler: -0.037 to +0.066). The Reynolds numbers per Mach number are interpolated from the range quoted in the report, not verified.
- Warm-started sweeps are off by default: they took 2–5× more iterations than cold starts at every Mach spacing tried (0.05 to 0.3), and the GPU implicit scheme diverges on a warm start to Mach 0.8. Opt in with `.warm_start(true)`; a warm point that ends Failed or Stalled is re-run cold.
- Subsonic/transonic residuals stall ~3–4 orders down; runs stop when CD is stationary (≤0.4 % change vs 80k-iteration runs).
- Sharp shoulders (cone-cylinder) are not grid-converged below M ≈ 1.05.

Roadmap: CLI and outputs (M5), GPU performance (M6), docs and release (M7); see `ROADMAP.md`.

## Quick start

```sh
cargo test --release                                  # unit/integration tests (GPU tests need a Vulkan device)
cargo run --release --bin cone -- --quick             # Taylor-Maccoll cone validation (CPU)
cargo run --release --bin ogive -- --sweep            # ogive-cylinder validation M = 1.98 (+ Mach sweep)
cargo run --release --bin transonic                   # transonic Mach sweep M = 0.7–1.5 vs TN D-3516 / RM L53K17 (CPU)
cargo run --release --bin cone_gpu                    # GPU vs CPU vs Taylor-Maccoll
cargo run --release --bin ogive_gpu                   # ogive-cylinder on the GPU vs CPU
cargo run --release --bin transonic_gpu               # transonic sweep on the GPU vs CPU
cargo run --release --bin subsonic -- --quick         # subsonic sweep M = 0.1–0.8, base on/off, potential-flow checks (CPU)
cargo run --release --bin subsonic_gpu                # subsonic sweep on the GPU vs CPU (--validate: NASA TM X-2439 bodies)
cargo run --release -p bowshock-cases --example animate # Mach-number animation of the ogive-cylinder (see the example's docs for flags)
cargo run --release -p bowshock --example polar        # inviscid and viscous drag polar of an example body (GPU, CPU fallback)
cargo test --release -p bowshock --test sweep -- --ignored --nocapture   # 30-point robustness matrix (GPU)
```

Results are written to `out/` (git-ignored).

## Crates

| crate | description | docs |
|---|---|---|
| `bowshock` (lib `bowshock`) | User API: body specification, grids, Mach-sweep `Analysis` (Euler or RANS), CPU and GPU runners | |
| `bowshock-core` (lib `bowshock_core`) | CPU f64 reference solver: gas model, HLLC/MUSCL face kernels, characteristic BCs, grids, SSP-RK3, line-implicit scheme, multigrid | [README](bowshock-core/README.md) |
| `bowshock-reference` (lib `bowshock_reference`) | Reference solutions: Taylor-Maccoll cones, tangent-cone and shock-expansion theory, axisymmetric potential flow | [README](bowshock-core/README.md) |
| `bowshock-cases` (lib `bowshock_cases`) | Validation cases (cone / ogive / transonic / subsonic), experimental data and their post-processing | [README](bowshock-core/README.md) |
| `bowshock-gpu` (lib `bowshock_gpu`) | f32/double-float Luma/WGSL port of the solver, validated against the CPU reference | [README](bowshock-gpu/README.md) |
| `bowshock-gpu-cases` (lib `bowshock_gpu_cases`) | GPU runners of the validation cases side by side with the CPU runs; GPU-vs-CPU tests | [README](bowshock-gpu/README.md) |
| `bowshock-cli` | Validation drivers `cone`, `ogive`, `transonic`, `subsonic` (CPU) and `cone_gpu`, `ogive_gpu`, `transonic_gpu`, `subsonic_gpu` | |

`bowshock-gpu` depends on [Luma](https://github.com/flipnoteman/Luma) (`main` branch) via git. To develop against a local Luma checkout, add a `[patch."https://github.com/flipnoteman/Luma.git"]` entry pointing at it.

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
