# bowshock-gpu

GPU solver for the axisymmetric Euler equations of `bowshock-core`, written in plain [Luma](https://github.com/flipnoteman/Luma) array ops (lazy
arrays 0.3: views, region assigns, records, `cache()`, `select`, `Array<Df32>`, `iterate_all`, `iterate_until_all`): no custom WGSL. Each function
mirrors the CPU routine of the same name in `bowshock-core` expression by expression; the doc comment states the equation. It replaced the
hand-written WGSL kernels of the first GPU port (Luma 0.2.4, see "History" at the end).

```text
bowshock-gpu/
  src/lib.rs        `Precision`, `GpuError`, `GpuSolver` (the handle the other crates use); `f32s` / `df32s` include the solver once per precision
  src/body.rs       the explicit solver: gas, flux, bc, solver (MUSCL + HLLC / Rusanov blend + low-Mach correction, all `Bc`, solid block, SSP-RK3)
  src/implicit.rs   the line-implicit scheme `Scheme::Implicit`
  src/viscous.rs   viscous terms (`LazySolver::set_viscous`): gradients, Sutherland viscosity, face fluxes, hoop stress, diffusive time step, `2 lam I`
  src/turbulence.rs  the Menter SST (2003) k-omega update and its two scalar line solves
  src/control.rs    run control: `set_config` (a sweep point), `base_cells`, accessors
  examples/bench.rs, bench_viscous.rs, eval_loop.rs, luma_issues.rs      speed tables (inviscid, viscous), where the `eval` loop spends its time, what still applies of the Luma issues
  ISSUES.md         issue list (Luma and this crate)
bowshock-gpu-cases/
  src/{cone,ogive,run}.rs     CPU / GPU runners of the validation cases with identical stopping criteria, comparison metrics
  tests/explicit.rs           explicit scheme vs the CPU, every `Staging` and both precisions; free-stream preservation, large grid, six orders, ogive wall Cp
  tests/implicit.rs           line-implicit scheme vs the CPU (sting, base block, Fox body), options of `ImplicitParams`, loop forms
  tests/loops.rs              the three loop forms compute the same iteration; threads on `default_context()`
  tests/runs.rs               whole runs through `bowshock::gpu::GpuRunner` vs `bowshock::run::run_on` (transonic, subsonic, warm start, converged implicit)
  tests/viscous.rs            laminar plate G1, SST plate G2, G3 body (solid block) vs the CPU, explicit and implicit, stagings, converged G3 drag
bowshock/src/gpu.rs           `GpuRunner`: the GPU run of a `bowshock::case::Case`, same stop rules and post-processing as the CPU runner
```

```text
cargo test --release -p bowshock-gpu-cases                       # accuracy vs the CPU solver, loop forms agree, whole runs
cargo run --release -p bowshock-gpu --example bench [-- --quick]  # speed tables (--quick skips 400x200; --profile-only, --staging Fusion|Lines|Fluxes|Split)
cargo run --release -p bowshock-gpu --example bench_viscous       # the viscous implicit scheme on the G3 body (--laminar, --levels 0,1, --staging ..)
cargo run --release -p bowshock-gpu --example bench -- --implicit # the line-implicit scheme only (--no-implicit skips it in the default run); LUMA_TRIDIAGONAL=thomas|cyclic
cargo run --release -p bowshock-gpu --example eval_loop           # where the time of the `eval` loop form goes
cargo run --release -p bowshock-gpu --example luma_issues         # what still applies of the Luma problems below
cargo run --release --bin cone_gpu | ogive_gpu | transonic_gpu | subsonic_gpu    # validation + benchmark drivers (bowshock-cli; CSV in out/gpu*/)
```

## API

```rust
let ctx = luma::default_context()?;
let mut gpu = GpuSolver::from_host(&ctx, &host, Precision::Extended)?;   // host: a bowshock_core Solver (grid, config, block, scheme, state)
gpu.set_config(&cfg)?;          // free stream, CFL, limiter, first_order_iters, bc: a sweep point, state untouched
gpu.reset()?;                   // free stream, iteration 0, CFL ramp restarted   (warm start: `set_iter(first_order_iters)` + `restart_ramp()`)
let history = gpu.run(50)?;     // density residual of each of the 50 iterations, recorded on the GPU
let u = gpu.state_f64()?;       // conserved state as f64;  `base_cells()`: the two cells behind each base-wall row
                                // viscous: `from_host` of a viscous host (or `set_viscous(Some(v), &grid)` after `set_block`), `turb_state()`, `turb_residual()`
```

`Precision::F32` is `f32s::LazySolver` (`Array<f32>` everywhere), `Precision::Extended` is `df32s::LazySolver` (state, residual and time step in
double-float `Array<Df32>`). Supported: `Scheme::Explicit` and `Scheme::Implicit`, with or without a solid block, low-Mach correction, and the viscous terms
(laminar or Menter SST) of `bowshock_core::viscous` / `turbulence`. Not supported: the multigrid and smoothed schemes (`Unsupported`). `bowshock::gpu::GpuRunner` (facade) is the runner of a
`Case`, Euler or RANS: `Backend::Auto` uses it when `luma::default_context()` opens a real GPU (a software Vulkan device counts as none).

The `LazySolver`s are also public (`bowshock_gpu::{f32s, df32s}::LazySolver`) for the benchmarks and the tests that vary the `Staging` or the loop form
(`run` = `iterate_until_all` with the residual recorded, `run_replay` = `iterate_all`, `run_eval` = `eval_all` every iteration).
The step is traced and its kernels compiled when a run starts, and the free stream and the limiter phase are constants of the traced step, so a
run after `set_config` with another free stream traces (and, if Luma's program cache misses, compiles) its kernels again, so a sweep point pays the "First call" cost below (0.05 s f32, 0.9 s double-float on a 100x50 cone grid); `run(n)` calls with the same configuration reuse them. [INFERENCE: the cache miss is not measured separately.]

## What is in it

* `src/body.rs`: the solver, written once. `lib.rs` includes it twice: `f32s` (`Array<f32>`) and `df32s` (state, residual and time
  step in double-float `Array<Df32>`, `Precision::Extended`). The state is a record of arrays of the element
  type (`[Array<S>; 6]`: `rho, rho u, rho v, E`, the density residual of the step and the CFL of the next one) that goes through
  `iterate_all` / `iterate_until_all` as it is, `f32` or `Df32`. Covered: `Scheme::Explicit` and `Scheme::Implicit`, with or without a solid block (`set_block`:
  base/wake grids; block ghosts, base-wall face stencils and solid faces/cells are region assigns, no masks), `Limiter::{FirstOrder, Minmod, VanAlbada}`, `face_flux_lm` (HLLC blended with Rusanov by the shock sensor,
  optional low-Mach correction), all `Bc` variants (`Freestream`, `Extrapolate`, `SlipWall`, `Characteristic`, `PressureOutflow`),
  the axisymmetric pressure source, local time step, SSP-RK3 with the first-order switch (one traced loop per limiter phase).
  Viscous runs are in `src/viscous.rs` / `src/turbulence.rs` (below). Not covered: multigrid and smoothed schemes.
* `src/implicit.rs`: the line-implicit scheme `Scheme::Implicit(ImplicitParams)` (`bowshock_core::implicit`): approximate factorization
  `(I - dt J_i)(I - dt J_j)`, the second-order residual on the right, the first-order flux Jacobians with the Roe-type dissipation
  matrix (shock-sensor blend, entropy fix) on the left, ghost-state Jacobians (exact for the free stream, extrapolation and the slip
  wall, central differences with relative step 1e-3 for the characteristic and pressure-outflow far fields), CFL ramp in the loop
  state, defect correction, positivity damping. A 4x4 block is sixteen arrays (`Blk`, with exact 0 / 1 entries that cost nothing);
  the line solves are Luma's `solve_block_tridiagonal` on `[16, ni, nj]` stacks. The left-hand side is f32 in both precisions.
* `src/viscous.rs`: `LazySolver::set_viscous` (mirrors `bowshock_core::viscous`: Green-Gauss gradients, Sutherland viscosity, the corrected face gradients with the
  ghost-mirror geometry, the viscous face fluxes, the hoop stress, the diffusive time step and the `2 lam I` of the implicit operator).
  `src/turbulence.rs`: the Menter SST (2003) update (`bowshock_core::turbulence`): gradients, blending functions, face data, residual and line
  coefficients, the two scalar line solves (`solve_tridiagonal`), the clipped update and the eddy viscosity.
* Tests: `bowshock-gpu-cases/tests/`; examples: `examples/`.

The math reads like the CPU code, expression by expression (`body.rs`, HLLC star state; `c(x)` is the constant `x` rounded to the
element type, `&` borrows an array):

```rust
/// `F_K + S_K (U*_K - U_K)`, `U*_K = fac [1, u + (S_M - u_n) n_x, v + (S_M - u_n) n_r, E / rho + (S_M - u_n) (S_M + p / (rho (S_K - u_n)))]`
let star = |k: &Side, s: &A, s_un: &A| -> Rec {
    let [r, u, v, p] = k.w;
    let (f, e) = (&k.f, &k.e);
    let fac = r * s_un / (s - sm);
    let sm_un = sm - &k.un;
    [
        &f[0] + s * (&fac - r),
        &f[1] + s * (&fac * (u + &sm_un * nx) - r * u),
        &f[2] + s * (&fac * (v + &sm_un * nr) - r * v),
        &f[3] + s * (&fac * (e / r + &sm_un * (sm + p / (r * s_un))) - e),
    ]
};
```

CPU branches (`if sl >= 0 { .. } else if ..`, `if t == 0.0 { return c }`, `if z >= 1.0`) become `select`. The ghost cells are region
assigns into one padded primitive record (`body.rs`, `assign_ghosts` and the start of `eval_residual`):

```rust
let p = cons_to_prim(u.clone());                              // [ni, nj] primitives (a record `[A; 4]`)
let w = p.pad([(2, 2), (2, 2)], PadMode::Edge);               // [ni + 4, nj + 4], still lazy
w.slice(0, ..2).slice(1, 2..nj + 2).assign(ghost)?;           // per boundary: wall reflection, characteristic, pressure outflow, ...
```

`Fields::assign` writes the four fields at once; because `w` is a pending expression nothing is launched, `w` becomes
`select(region, ghost, old)` and fuses into whatever reads it. A face reads its four cells as four `slice`s of `w`, the residual is
`fi.slice(0, 1..) * w.slice(0, 1..) - fi.slice(0, ..ni) * ..`. The geometry (normals, lengths, weights, `1/V`, areas) is uploaded
once from the f64 `Grid`, rounded to `f32` or `Df32`. Handles that are meant to be the same lazy array (the primitives passed on
as the first-order reconstruction, `rho` of the Riemann ghost state, ...) are `.clone()`s; no place needs an independent copy
(`&a * 1.0`).

## Accuracy

Tests in `bowshock-gpu-cases/tests/explicit.rs` (`cone_state_after_20_iterations_matches_cpu`: 40x20 cone grid, M = 2, 15 degree cone, minmod and van
Albada; `characteristic_far_field_matches_cpu_on_a_perturbed_state`: far-field box, M = 0.9 and 1.3, characteristic and fixed-pressure outflow;
`transonic_state_after_20_iterations_matches_cpu`: ogive-cylinder 144x70, M = 0.9; `low_mach_correction_matches_cpu`: the box at M = 0.3;
`base_block_explicit_state_matches_cpu`: base and Fox grids with the solid block): CPU and GPU solver start from the same state, run 20 iterations (the
first 5 first order), and are compared by `max |dU|/(1+|U|)` (state) and `max |r_gpu - r_cpu| / r_cpu` over the 20 residual-history values.
`cargo test --release -p bowshock-gpu-cases -- --nocapture` prints every row for all four stagings (default staging below; the other stagings give the same
numbers up to a few percent of the f32 rows).

| case | f32 state / residual | df32 state / residual |
|---|---|---|
| cone 40x20 minmod | 1.0e-6 / 9.0e-7 | 4.1e-14 / 3.0e-14 |
| cone 40x20 van Albada | 9.0e-7 / 8.1e-7 | 1.8e-10 / 2.1e-11 |
| box M = 0.9 characteristic | 8.9e-7 / 1.9e-6 | 2.7e-14 / 1.7e-14 |
| box M = 0.9 pressure outflow | 9.4e-7 / 1.9e-6 | 2.7e-14 / 2.0e-14 |
| box M = 1.3 characteristic | 9.6e-7 / 4.6e-7 | 3.0e-14 / 3.1e-14 |
| box M = 1.3 pressure outflow | 9.6e-7 / 4.6e-7 | 3.0e-14 / 3.1e-14 |
| transonic M = 0.9 144x70 | 7.6e-7 / 1.0e-6 | 3.5e-14 / 3.4e-14 |
| box M = 0.3 low-Mach van Albada | 9.0e-7 / 7.0e-7 | 2.2e-14 / 2.1e-14 |
| **target** | 1e-5 / 1e-4 | 3e-7 / 2e-7 (5e-7 transonic) |

All 8 cases pass in all four stagings and both precisions. The f32 rows are at the f32 round-off of the CPU comparison. The double-float solver uploads
the geometry as `Df32` computed from the f64 grid, so it agrees with the CPU to ~1e-14 relative (the 1.8e-10 of the van Albada cone is two orders above its
siblings [INFERENCE: sign decisions of the limiter on round-off-level differences in the uniform far field, 1e-16 on the CPU vs 1e-14 in double-float; not
investigated]). The three loop forms (`iterate_until_all`, `iterate_all`, `eval_all` loop) agree (`tests/loops.rs`: double-float bit-identical, f32 within
6.4e-7 of each other), and four threads sharing `default_context()` each reproduce the state of a private-context run
(`threads_on_the_default_context_agree`). The residual history of the double-float solver is recorded on the GPU as two `f32` records, `rn.to_f32()` and the
low word `(rn - rn.to_f32().to_df32()).to_f32()`, because the history of `iterate_until_all` is `f32`; with only the high word the history would carry a 6e-8
relative rounding, which would hide the 1e-14 agreement.

`GpuSolver` on the same kinds of set-ups: `extended_precision_reaches_six_orders_on_a_small_grid` (the double-float solver converges a 60x30 cone to the CPU's 6
orders), `ogive_state_and_wall_cp_match_cpu`, `characteristic_far_field_preserves_the_free_stream`, `large_grid_residual_history_matches_cpu` (400x200).
`tests/runs.rs` compares whole runs of the facade runner with the CPU runner (iteration counts, residual history, wall Cp, CD, base Cp): transonic cold and warm
start, subsonic sting and base cases, converged implicit states (iteration counts within `0.05 cpu + 5`, field within 1e-4 p_inf), cold restart on a reused
solver, `set_config` moving the free stream.

### Implicit scheme

`tests/implicit.rs`: sting case of `subsonic_case` (M = 0.5 ogive-cylinder, grid level 0 = 144x70, van Albada, low-Mach correction, `Numerics::subsonic`'s
`Scheme::Implicit`: CFL ramp 1 -> 30 at 1.02 per iteration, Roe-type left-hand side, characteristic far field with finite-difference ghost Jacobians, two
defect-correction passes), first 5 iterations first order, `Staging::Fluxes`, from the same state as the CPU solver (`LazySolver::from_host`). The left-hand side
is f32 in both precisions; `R` and the update are in the element type, so the fixed point is the element type's.

| 30 iterations (`implicit_scheme_state_matches_cpu`) | state `max abs(dU) / (1 + abs(U))` | residual history `max abs(r - r_cpu) / r_cpu` | tolerance |
|---|---|---|---|
| lazy f32 | 1.0e-6 | 1.8e-6 | 1e-3 |
| lazy df32 | 3.1e-9 | 4.3e-8 | 1e-4 |

With the solid block (the same test, 30 iterations; state / residual history; `subsonic_case(.., true)` is the base case with `j_sting = 0`, the Fox TM X-2439
body `fox_case(false, 0.6, 0)` has a sting and a boattail):

| case | lazy f32 | lazy df32 |
|---|---|---|
| base (`j_sting = 0`) | 1.0e-6 / 1.6e-6 | 2.4e-8 / 6.9e-8 |
| Fox sting + boattail | 1.1e-6 / 4.5e-7 | 3.6e-8 / 5.9e-8 |

The agreement is far inside the tolerances because the f32 left-hand side only steers the correction `dU = (LHS)^-1 R`: an f32 error in the left-hand side
scales the update by 1e-7, it does not enter `R`. The options of `ImplicitParams` (`implicit_options_and_loop_forms`, df32, 30 iterations): `roe = false`
3.5e-9, `lines = false` 2.5e-8, `implicit_bc = false` 3.4e-9, `sub_iters = 1` 5.5e-9, `MinDirection` with growth 1.2 and CFL 20 3.5e-6 (state; the residual
history, which has dropped to 2.5e-3 of its start, 5e-4), each against the CPU run with the same parameters. The three loop forms give identical states and
histories in both precisions: the line solves are traced and replayed like any other kernel.

Converged states (`implicit_scheme_converged_state_matches_cpu` in `tests/runs.rs`, double-float, `GpuRunner` and `bowshock::run::run_on` to 5 orders of
density-residual drop): M = 0.3 and 0.6 agree in iteration count (484 / 484 and 377 / 377 in the first measurement of this solver) and field
(`max abs(p - p_cpu) / p_inf` 1.3e-8 and 1.6e-7).

### Viscous terms and the SST model

`tests/viscous.rs`: CPU `Solver` and GPU solver start from the same state (`GpuSolver::from_host`: grid, block, viscous parameters, wall distance and the turbulence
state of the host) and run the same iterations (the first 5 first order); `max |dU| / (1 + |U|)` and the maximum relative difference of the residual history (from the
second iteration):

| case (iterations) | lazy f32 state / residual | lazy df32 state / residual |
|---|---|---|
| G1 laminar plate, implicit (30) | 5.4e-6 / 9.9e-4 | 1.2e-7 / 4.7e-7 |
| G1 laminar plate, explicit (30) | | 4.0e-13 / 2.1e-11 |
| G2 SST plate, implicit (50) | 3.8e-5 / 3.4e-2 | 1.5e-7 / 8.2e-5 |
| G2 SST plate, explicit (50) | | 6.8e-12 / 8.3e-10 |
| G3 body (solid block, no-slip body surface and base wall, SST), implicit (30) | 1.0e-6 / 4.4e-6 | 2.2e-8 / 6.4e-8 |
| G3 body, explicit (30) | | 1.6e-11 / 1.7e-13 |

(Tolerances of the tests: state 1e-3 for the laminar plate and the body, mean state 1e-3 and `k`, `omega` 1e-2 for the SST plate, 1e-6 for the explicit rows.)
`k`, `omega`, `mu_t` of the cells with `k > 10 k_inf` (the plate, 2012 cells): f32 1.3e-3 / 3.8e-4 / 1.2e-3, df32 5.4e-6 / 1.4e-6 / 4.0e-6; the G3 body (12422 cells where
`k` has moved off the free stream): f32 4.0e-6 / 1.3e-5, df32 2.7e-7 / 4.7e-7; the density residual norm of `k` agrees to the printed digits (G2: 6.504e-4 both). The
explicit scheme is round-off (all of it in `S`, with the f32 left-hand side only in the turbulence line solves: `k`, `omega` 1e-7 over all 9792 cells of the plate). The
implicit df32 rows are 1e-7, not 1e-14: the f32 left-hand side steers the update, and the viscous wall cells (first cell height 4e-6 on the plate, so very high aspect ratios
[INFERENCE: not isolated]) amplify its 1e-7 error more than the inviscid grids do (3e-9). `converged_body_total_drag_matches_cpu` (`#[ignore]`: the CPU's stop rule
`CD stationary` after 4000 iterations, 4.1 orders; the GPU solver runs the same 4000 iterations from the free stream in 3.3 s (f32) / 4.0 s (df32), the CPU about 27 s):
total CD 0.21300 on all three (f32 +0.001 %, df32 -0.000 %), `cd_friction` 0.09003, `cd_base` 0.12257, `cp_base` -0.1226, `y+` max 0.648, each equal to the printed digits;
through the facade (`bowshock/tests/viscous.rs::g3_gpu_matches_cpu`, `Analysis` on `Backend::Cpu` and `Backend::Gpu`) the same numbers.

What is covered: `Scheme::Explicit` and `Scheme::Implicit`, laminar and SST, with and without the solid block. The no-slip walls are those of `bowshock_core::viscous::Walls`
(the domain `jmin` wall from `wall_i_start`, the body surface from `wall_i_start`, every base-wall face, the sting surface), applied at all four mirror sites
(`assign_ghosts`, `assign_block_ghosts`, `block_fluxes`, the implicit assembly with its `diag(1, -1, -1, 1)` Jacobians). The loop state grows to ten arrays (`k`, `omega`,
`mu_t`, the `k` residual; one-element dummies without SST), so the traced loops carry the turbulence state like the CFL. The block must be declared before the viscous
terms, and the boundary conditions of a viscous run are fixed by `set_viscous` (`GpuSolver::set_config` refuses another `bc`).

## Speed

RTX 3090, Vulkan, 15 degree cone at M = 2, minmod, MUSCL from the first iteration, 2000 iterations from the free stream (as `cone_gpu --bench`, whose
grids and settings these are), best of 3, microseconds per iteration, `bench --quick` (the 400x200 grid was not re-measured). Steady state: the first call of
every loop is run before the clock starts, so compilation and tracing are not in these numbers. CPU = f64 `Solver` on 32 rayon threads (500 iterations).
Lazy = `LazySolver::run` (`iterate_until_all`, residual recorded every iteration, one wait per 50 iterations) or `run_replay` (`iterate_all`, no history),
whichever is faster for the best staging. The GPU was shared with other jobs during these runs (the lazy numbers moved by about 10 % between runs).

| grid | CPU f64 | lazy f32 | lazy df32 |
|---|---|---|---|
| 100x50 | 602 | 33.2 | 190.6 |
| 200x100 | 1304 | 44.7 | 260.2 |

Lazy solver by staging and loop form (`until` = `iterate_until_all` + residual history, `replay` = `iterate_all`, `eval` = `eval_all` after every
iteration, 300 iterations):

| grid | staging | f32 until | f32 replay | f32 eval | df32 until | df32 replay | df32 eval |
|---|---|---|---|---|---|---|---|
| 100x50 | Fusion | 166 | 140 | 4176 | 732 | 702 | 4760 |
| 100x50 | Lines | 84 | 66 | 3546 | 512 | 481 | 3509 |
| 100x50 | Fluxes | 53 | **33** | 1756 | 209 | **191** | 1809 |
| 100x50 | Split | 62 | 43 | 1683 | 215 | 192 | 1706 |
| 200x100 | Fusion | 221 | 196 | 4646 | 1081 | 1025 | 4918 |
| 200x100 | Lines | 107 | 86 | 3559 | 574 | 543 | 5076 |
| 200x100 | Fluxes | 62 | **45** | 2529 | 430 | **260** | 1701 |
| 200x100 | Split | 104 | 60 | 1704 | 419 | 327 | 1701 |

(The 200x100 `Fluxes` df32 `until` / `eval` cells and the `Split` cells of that grid carry the run-to-run noise of the shared GPU:
an earlier run of the same binary gave 267 / 240 for `Fluxes` replay / until and 258 / 234 for `Split`.)

* **Staging** (`LazySolver::staging`, defaults `Fluxes` for both precisions): `Fusion` has no `cache()` at all, which is the code
  the equations say, and is the slowest everywhere: 3-4.4x the time of `Fluxes` in f32 and 2.5-4x in double-float, and its
  double-float kernels take 9 s to compile on the first call (`Lines` 6 s, `Fluxes` 0.9 s). `Lines` evaluates the padded primitive
  record once; `Fluxes` also evaluates the face fluxes; `Split` also the reconstructed states and the shock weight.
  Every `cache()` left in `body.rs` has its measured reason in a comment next to it.
* **Why `Fusion` is slow** (profile, below): the padded record `w` is `select(region, ghost, old)` four times over, so every kernel
  that reads `w` at a stencil cell recomputes the primitives and all four ghost expressions (about 100 operations each for the
  characteristic far field) at that cell; the face flux is read at four stencil cells in each direction and the stage updates nest
  in each other, so the kernel exceeds the fusion limits (`1024 operations`, `1537 inlined operations > 1536`) and is cut 19-33
  times per step into 67-91 kernels that all recompute what the others compute. Raising the limits (`LUMA_FUSION_MAX_NODES=4096
  LUMA_FUSION_MAX_INLINED=16384`) gives 47 launches and is not faster than `Fluxes` in the profile (178-205 us of GPU time against
  109-171 us). Minimal repro: `examples/luma_issues.rs`,
  section 1.
* **Loop form**: `iterate_all` replay is fastest; the residual history of `iterate_until_all` costs about 20 us per iteration; the
  plain `eval` loop re-builds the step in Rust every iteration and is 40-55x slower in f32 (see "The eval loop").
First call (construction + compile + trace + first 50 iterations), seconds, 100x50 (kernels are specialised to the grid shape):

| | lazy f32 `Fusion` | lazy f32 `Lines` | lazy f32 `Fluxes` | lazy df32 `Fusion` | lazy df32 `Lines` | lazy df32 `Fluxes` | lazy df32 `Split` |
|---|---|---|---|---|---|---|---|
| first call | 0.49 | 0.21 | 0.05 | 9.33 | 6.06 | 0.90 | 0.38 |

### Launches per SSP-RK3 step (`Profile`, 200x100, `bench` profile section)

| | f32 `Fusion` | f32 `Lines` | f32 `Fluxes` | f32 `Split` | df32 `Fusion` | df32 `Lines` | df32 `Fluxes` | df32 `Split` |
|---|---|---|---|---|---|---|---|---|
| launches | 67 | 26 | **14** | 20 | 67 | 26 | **14** | 20 |

(Fusion cuts per step: 19, 8, 0, 0. The first measurement after the update, with Luma's working tree as it was a few hours
earlier, gave 91 / 30 / 14 / 20: the `Fusion` and `Lines` counts move with Luma's fusion heuristics, `Fluxes` and `Split` did not.)
The 14 launches of `Fluxes` are 3 per Runge-Kutta stage (the padded record with the previous stage update and residual fused into
it, the i fluxes, the j fluxes), the time step, the final state of the step and the three kernels of the residual norm (a fused
sum, the second reduction pass, `sqrt(sum / N)`). One launch per residual evaluation is not reachable: the fluxes read the
padded record at four cells and the residual reads the fluxes at two, so each of the three needs its own evaluated buffer or
recomputes its inputs (`Fusion`). The reference (Luma's `bench_pde` Euler step, first order) is one kernel because its stencil is
one cell wide and has no nonlinear ghost expression.

### The eval loop

`LazySolver::run_eval` builds the step in Rust and `eval_all`s it every iteration: 1.7-1.8 ms per iteration at 200x100 with `Fluxes`. `cargo run --release -p bowshock-gpu --example eval_loop` splits it up (us per iteration, 200x100):

| precision | staging | loop total | build the step (Rust only, nothing evaluated) | rest of the loop (`eval_all`) | launches / step |
|---|---|---|---|---|---|
| f32 | Fluxes | 1687 | 1498 | 189 | 14 |
| df32 | Fluxes | 1724 | 1454 | 268 | 14 |
| f32 | Fusion | 4058 | 4053 | 5 | 67 |

So 90 % of an iteration is the host building the step's nodes (about 3000 of them [INFERENCE: from the operation counts of the
fused kernels]; `luma::jit::create` and the program measuring behind it lead a callgrind profile of `eval_loop --trace-only`), not
launching: no kernel is compiled inside the loop (the first iteration takes 40 ms f32 / 850 ms df32 for compilation, after 20
warm-up iterations the average over 300 is the build time plus 0.2 ms), and the 14 launches cost 65-150 us of GPU time in f32
(profile, shared GPU). It is not our code: computing `el`, `un`, `c` and the physical flux once per face side (`Side`) instead of
three times changed the build time from 1.55 to 1.50 ms (3 %). Luma's per-operation build cost is 0.7-1.2 us and grows with the
size of the expression (`luma_issues` section 2). `iterate_all` / `iterate_until_all` trace the step once and replay it, which is
what the 33-45 us per iteration of the tables above are.

### Line-implicit scheme

`cargo run --release -p bowshock-gpu --example bench -- --implicit` (RTX 3090, shared with other jobs): sting case of `subsonic_case`, M = 0.5, tangent
ogive + 6 d cylinder, grid levels 0 (144x70) and 1 (288x140), `Numerics::subsonic` (van Albada, low-Mach, `Scheme::Implicit`: CFL ramp
1 -> 30, two defect-correction passes), 200 iterations from the free stream (the first 100 first order, so both limiter phases are
compiled before the clock), best of 3, microseconds per iteration. CPU = f64 `Solver::step` on 32 rayon threads; lazy `until` = `iterate_until_all` with the residual recorded, `replay` =
`iterate_all`. Luma picks the algorithm of the line solves (default):

| case | level | grid | CPU f64 | lazy f32 until | lazy f32 replay | lazy df32 until | lazy df32 replay |
|---|---|---|---|---|---|---|---|
| sting | 0 | 144x70 | 2361 | 227.1 | 180.6 | 259.8 | 217.6 |
| sting | 1 | 288x140 | 10868 | 447.0 | 405.9 | 516.4 | 466.9 |
| base (solid block) | 0 | 156x76 | 2520 | 302.9 | 194.7 | 313.5 | 264.0 |
| base (solid block) | 1 | 312x152 | 10977 | 493.6 | 446.0 | 618.5 | 565.9 |

(GPU columns measured with the line-solve stacks built by `luma::stack_slice` (see "Luma issues", 11); with the 16-part `concat` they were 273.1 / 226.6 / 310.0 / 266.9, 582.5 /
540.5 / 639.4 / 590.5, 281.9 / 239.8 / 349.9 / 304.0 and 683.7 / 598.3 / 768.8 / 712.0. The CPU column is the earlier measurement.)

(A later run of the sting rows, with the base rows added; the CPU column moves by 2x between runs on the shared machine, the earlier run
of this table had 3592 / 7711.) The base case is the ogive-cylinder with the flat base and wake block (`j_sting = 0`); its f32 step is
27 launches, 204 us of GPU time (df32: 28 launches, 286 us; 256 / 339 us with `concat`), the line solves are the same four `CyclicTridiagonal` launches with
`starts: true` (44.1 + 38.2 us).

The lazy scheme is 5-30x faster than the CPU solver (10 / 18 times at level 1 in f32 / df32 replay). `LUMA_TRIDIAGONAL` forces the algorithm of the line solves (the other columns do not depend on it):

| `LUMA_TRIDIAGONAL` | level | lazy f32 until / replay | lazy df32 until / replay |
|---|---|---|---|
| unset (Luma chooses) | 0 | 276.6 / 236.4 | 316.3 / 266.3 |
| `cyclic` | 0 | 263.5 / 225.4 | 306.2 / 262.5 |
| `thomas` | 0 | 700.6 / 656.6 | 737.8 / 692.3 |
| unset (Luma chooses) | 1 | 610.6 / 552.9 | 654.9 / 625.7 |
| `cyclic` | 1 | 591.7 / 544.0 | 649.6 / 596.9 |
| `thomas` | 1 | 1567.2 / 1514.0 | 1624.0 / 1568.8 |

The default is cyclic reduction at both levels (its timings are the forced-`cyclic` ones within the noise of the shared GPU).

Where the time of one lazy f32 step goes (level 0, `profile_step`: 27 launches, 240 us of GPU time, 0 fusion cuts; the df32 step is 28
launches, 278 us), summed over the kernels of the profile:

| kernels | launches | GPU us |
|---|---|---|
| line solves: `CyclicTridiagonal { block: 4, workgroup: 32, shared_rows: 144 }` (i lines, 42.9) and `shared_rows: 70` (j lines, 38.8) | 4 | 81.7 |
| the 16-part `concat` that builds the `[16, ni, nj]` stacks of `lo`, `d`, `up` of both directions (issue 11; with `luma::stack_slice`: 6 launches, about 20 us) | 6 | 70.2 |
| ghost-state Jacobians at the line ends (32 outputs on `[1, nj]` / `[ni, 1]`) | 2 | 16.5 |
| blocks `lo`, `d`, `up` (48 outputs) | 2 | 13.7 |
| face matrices `FL`, `FR` (32 outputs) | 2 | 8.3 |
| 4-part `concat` (the right-hand side stacks; `stack_slice`: about 3 us) | 2 | 6.5 |
| 4-output kernels on `[144, 70]` (pseudo-time step, `dt R`) [INFERENCE: from the kernel shapes] | 1 | 9.4 |
| residual: padded state 5.9, face fluxes 9.5, cell residual 3.5, norm 8.0 | 5 | 26.9 |
| positivity damping and update, CFL | 2 | 6.8 |

The four line solves of a step are two passes (`sub_iters = 2`) of the i-line solve and the j-line solve. With `LUMA_TRIDIAGONAL=thomas`
they are `Tridiagonal { block: 4, workgroup: 8 }`, 128.5 us each (514 us of a 679 us step): one invocation per line (70 / 144 lines)
with a dependent chain of 144 / 70 block eliminations, as expected. `until` is about 40 us above `replay`: the residual history, as for
the explicit scheme.

### Viscous terms and the SST model

`cargo run --release -p bowshock-gpu --example bench_viscous [-- --laminar] [--staging ..]` (RTX 3090, shared with other jobs): `Case::viscous` on the G3 body (tangent ogive 0.30 x
0.10 m + 0.60 m tube, ISA 0 m, Mach 0.5, flat base and wake block, `Scheme::Implicit(ImplicitParams::VISCOUS)`, van Albada with the low-Mach correction), grid levels 0
(156x122) and 1 (312x122), 200 iterations from the free stream (the first 100 first order), best of 3, microseconds per iteration; CPU = `Solver::step` (rayon), lazy `until` /
`replay` as above. Default staging `Fluxes`. The line-solve stacks are built with `luma::stack_slice`.

| model | level | grid | CPU f64 | lazy f32 until / replay | lazy df32 until / replay | CPU / lazy f32 replay |
|---|---|---|---|---|---|---|
| SST | 0 | 156x122 | 6744 | 438 / 387 | 574 / 519 | 17x |
| SST | 1 | 312x122 | 12411 | 678 / 626 | 854 / 797 | 20x |
| laminar | 0 | 156x122 | 4351 | 354 / 304 | 442 / 386 | 14x |
| laminar | 1 | 312x122 | 7683 | 570 / 513 | 701 / 616 | 15x |

(The CPU column moves by 2x between runs on the shared machine.) **Double-float is cheap here**: `df32s` costs 1.3x of `f32s` (SST replay), against 2.5-3.2x for the explicit inviscid
kernels, because the left-hand side and the line solves, which are f32 in both, are most of the step. The viscous terms and SST in `S` are therefore the default in both precisions (no
f32 special case): the arithmetic that sets the fixed point is in the element type, only the pseudo-time-step operators (`2 lam I`, the diffusive time step, the two scalar line
solves) are f32. Staging (SST, level 0, `replay`, f32 / df32, measured with `concat` stacks): `Fluxes` 480 / 612, `Lines` 499 / 621, `Split` 498 / 625, `Fusion` 577 / 958 (first call 2.8 s /
14 s). `Fluxes` stays the default.

One SST step, level 0 (`profile_step`): 65 kernel launches, 440 us of GPU time (f32; df32 580 us; level 1: 687 us f32), 1 fusion cut (`1537 inlined operations > 1536` in the
dissipation blocks). The biggest kernels, f32 level 0:

| kernels | launches | GPU us |
|---|---|---|
| mean-flow line solves: `CyclicTridiagonal { block: 4, shared_rows: 156 }` (i lines) and `shared_rows: 122` (j lines), 3 passes each | 6 | 133.3 |
| 4-output kernels (the `[4, ni, nj]` right-hand side stacks, pseudo-time step) | 2 | 27.0 |
| the `[16, ni, nj]` stacks of `lo`, `d`, `up` (`FusedStack`) | 6 | 25.4 |
| turbulence line solves: `CyclicTridiagonal { block: 1 }`, one per scalar and direction | 4 | 25.5 |
| assembly blocks (48, 48, 32, 56 outputs) | 4 | ~46 |
| residual norm (`FusedReduce`) | 1 | 9.6 |

## History: the hand-written WGSL kernels

The first GPU port (`GpuSolver` on Luma 0.2.4 with custom WGSL kernels: `shaders.rs`, `shaders_ext.rs`, `shaders_impl.rs`, `shaders_visc.rs`; modes
`Direct` / `Guarded`, precisions `F32` / `Compensated` / `Extended`, f32 geometry with a closure-defect correction) was replaced by this crate. Last
measurement of both on the same machine (microseconds per iteration, conditions as in "Speed"; custom = `Mode::Guarded`, history kernel on, `run(50)` chunks):

| grid | CPU f64 | custom F32 | custom Extended | lazy f32 | lazy df32 | lazy f32 / custom F32 | lazy df32 / custom Extended |
|---|---|---|---|---|---|---|---|
| 100x50 | 602 | 28.9 | 75.2 | 33.2 | 190.6 | 1.15x | 2.5x |
| 200x100 | 1304 | 33.7 | 81.9 | 44.7 | 260.2 | 1.3x | 3.2x |

Line-implicit scheme, sting case (level 0 = 144x70 and level 1 = 288x140) and base case (156x76, 312x152): custom F32 / Extended 105.8 / 143.7, 260.6 / 286.4,
96.9 / 142.9, 274.0 / 302.9 against lazy f32 / df32 replay 226.6 / 266.9, 540.5 / 590.5, 239.8 / 304.0, 598.3 / 712.0: the array-op scheme is 2.1-2.5x (f32) and
1.8-2.4x (df32) the hand-written kernels, 5-30x faster than the CPU solver. In accuracy the double-float array-op solver agrees with the CPU to ~1e-14 where the
`Extended` kernels agreed to ~5e-8 (the lazy solver uploads `Df32` geometry computed from the f64 grid, `Extended` kept f32 normals / weights / volumes plus a
closure-defect correction). Viscous implicit step on the G3 body (SST, level 0 / 1, replay): custom F32 / Extended 263 / 319 and 377 / 428 against lazy f32 / df32 480 / 612 and 764 / 942 us with `concat` stacks (1.8-2.2x), 387 / 519 us at level 0 with `luma::stack_slice` (1.5x / 1.6x). With `stack_slice` the inviscid implicit step is 1.7x (f32) and 1.5x (df32) the custom time at level 0 (180.6 / 217.6 against 105.8 / 143.7 us). The array-op solver is 2.0k lines (`lib.rs`, `body.rs`, `implicit.rs`, `control.rs`) against 3.5k lines for the kernels without viscous terms (`lib.rs` and three `shaders*.rs`) and reads like the CPU routines. The viscous terms and the SST model of the kernels (`shaders_visc.rs`) are not ported yet.

## Luma issues

See also `ISSUES.md` (the running issue list).

All four bugs the prototype hit are fixed in Luma (each has a regression test there; `CHANGELOG.md`, `[Unreleased]`), and the
workarounds are removed:

1. **A `Df32` kernel whose only library calls are comparisons cannot bind its uniform: fixed.** `Array::<Df32>::abs`,
   `minimum`, `maximum`, `lt` of two arrays bind correctly (fix: the guard uniform of Df32 kernels with only
   `abs`/`minimum`/`maximum`/comparisons). `df32s` uses plain `abs` / `lt` / `minimum` / `maximum` again (`aabs`, `alt`, `amin`,
   `amax` are gone). `luma_issues` printed the values `[1, 2, 3]`, `[1, -2, -3]`, `[1, 1, 0]` for `a.abs()`, `a.minimum(&b)`,
   `a.lt(&b)` before this file was shrunk.
2. **`x - x.to_f32().to_df32()` is 0 when fused: fixed** (casts to and from `Df32` are guarded against reassociation).
   `(x - x.to_f32().to_df32()).to_f32()` fused gives `x.lo` (`[-9.934108e-9, -1.9868216e-8]` for `[1/3, 2/3]`); `df32s::split`
   (with its `cache()`) is gone, and the residual history's low word uses exactly this expression.
3. **Input-buffer count after an evaluation: fixed** (buffers below a `cache()`d part no longer count toward the limit; the
   input-buffer limit is the device's, up to 256, not 16). The old repro prints 12 launches, or 15 with `s` cached, no cuts.
4. **Wrong results with several threads on `default_context()`: fixed** (each thread evaluates only its own lazy arrays);
   `bowshock-gpu-cases/tests/loops.rs::threads_on_the_default_context_agree` runs four threads on it (the other tests keep a `Context::new()` each, for
   isolation).
5. **API gaps: closed.** `Array: Clone` (a clone is another handle on the same data: all `dup` uses were shared handles, none needed
   `&a * 1.0`); `Fields` and `iterate_all` / `iterate_until_all` take `Df32` states (no `hi` / `lo` packing, no `pack` / `unpack` of
   pairs); `concat` is lazy but computes every part at every cell, so ghost layers are region assigns now. Still true: no
   `Operand<f64> for f64` scalar on comparisons, `minimum` / `maximum` or `select` (a native `f64` variant would need `Array::full`
   constants; not done, the optional item of the task); the `iterate_until_all` history is `f32`.

What still applies (`cargo run --release -p bowshock-gpu --example luma_issues`):

6. **Ghost layers written as region assigns into a lazy padded array cost operations at every reader.** The array is
   `select(region, ghost, old)`; a kernel that reads it at a stencil cell evaluates the ghost expression there, for each of the four
   sides, although the region is 2 cells wide. With ghost expressions of 100 operations the kernel reading the padded record at
   four offsets per direction is cut at `1537 inlined operations > 1536` and a one-kernel expression becomes four launches
   (`luma_issues` section 1: `ghost_ops` 0 / 10 / 40: 2 launches, 0 cuts; 100: 4 launches, 2 cuts). Expected: the cost of a region
   assign proportional to the region (e.g. the select lowered to a branch that evaluates the right-hand side only inside the region,
   or counted by the region's share of the kernel). In the solver this is why `Fusion` needs the padded record evaluated (`Lines`).
7. **Building a lazy expression costs 0.7-1.2 us per operation, rising with the expression size** (`luma_issues` section 2: 100
   operations 0.66 us each, 400: 1.2 us, 900: 1.1 us), which is 90 % of the `eval` loop form (see above). Expected: a flat, lower
   per-node cost (the program is measured at every node creation: `luma::jit::create`, `program::collect`).
8. **The residual norm of a step is three launches** (fused sum, second reduction pass, `sqrt(sum / N)` on the one-element result): 3
   of the 14 launches of a step. Expected: the scalar epilogue fused into the reduction's second pass.
9. **`Profile::splits` lists the cuts at node creation only** (unchanged from before): the "evaluated on its own first" decisions
   do not appear; the launch count of the step is what shows them.
10. **First call of a big `Df32` kernel**: 9 s of compile for `Fusion` / 6 s for `Lines` (0.9 s for `Fluxes`) at the first launch,
    `bench --staging Fusion` (fresh process, `__GL_SHADER_DISK_CACHE=0` to see it every time).
11. **Operands of `solve_block_tridiagonal` must be one `[16, ni, nj]` array, and there was no way to build it from sixteen `[ni, nj]`
    arrays other than `concat`: fixed by `luma::stack` / `stack_slice` (Luma 90e68d6, used since: the six stacks of a step cost about 20 us instead of 70-105 us, the implicit
    step 191 instead of 242 us of GPU time at level 0, 438 instead of 520 us for the viscous one).** What was measured with `concat`: `luma::concat::<f32, 16>(0, parts)` over the evaluated, reshaped parts costs 11.7 us per stack at 144x70
    (161k elements; a bandwidth-bound copy is 2 us, the launch itself 2-3 us): every output element selects among 16 loads. An implicit
    step needs six stacks (`lo`, `d`, `up` of two directions): 70 us of the 240 us GPU time of the step (29 %), as much as the four line
    solves (82 us). The alternative tried, `zeros([16, ni, nj]) + 0.0` as a lazy base and sixteen `slice(0, k..k + 1).assign(part)`,
    is 20x slower (4200-5100 us per iteration instead of 236-277; `src/implicit.rs::stack`). Expected: a `stack` operation that writes
    each part once (or the solvers taking the blocks as a record `[Array<f32>; 16]` / a strided view).
12. **`reshape` evaluates its operand** (it shares the buffer, so the array has to exist): a lazy `slice(0, k..k + 1)` of a `[4, ni, nj]`
    solve result cannot be reshaped to `[ni, nj]` without a copy kernel. The solution is unstacked with `take` and four precomputed
    index arrays (`4 (i + ni j) + k`), which stays lazy and fuses into the next kernel. Expected: a lazy `reshape` of a view (or of a
    unit-extent axis).
13. **No `is_finite`**: the positivity damping of the implicit step tests `|q| < 1e30` (NaN and infinity compare false), which relies on
    NaN comparing false in the generated WGSL. Nothing in Luma promises that, and the damped branch (`alpha < 1`) is not checked
    separately: the tests do not force a step that needs it.


## Going further

* **Viscous terms and SST: done** (`src/viscous.rs`, `src/turbulence.rs`, see "Viscous terms and the SST model" above): Green-Gauss gradients, the face-gradient correction and the
  viscous fluxes are slices and arithmetic as predicted; SST needed `tanh` (f32 even in `df32s`: `Array<Df32>` has no `tanh`), `sqrt`, Luma's scalar tridiagonal solves and a
  wall-distance array uploaded from `bowshock_core`.
* **`iterate_until_all` as the run loop.** It already is one here (`LazySolver::run` is one per limiter phase). The convergence rules of
  `bowshock::run::stop_reason` map onto it: `ResidualDrop` = `h["residual"].peak() / h["residual"].last() >= 1e6` (Luma's `peak` is over all steps, the CPU
  rule over steps after 20: use `h.step()` to gate), the monitor "stationary" rules = `window_max(k) - window_min(k) < tol` of a recorded Cd / base-Cp,
  `NonFinite` = `last().le(x).not()`. What does not map: the limiter switch (the traced step is specific to the limiter: one loop per phase, or the CFL ramp as
  a `luma::Scalar`, which changes without recompiling), warm starts across a Mach sweep (the free stream could be `Scalar`s, again no recompile; today a new free
  stream retraces and recompiles), and `StopReason` variety beyond one predicate index per rule.
* **Speed.** The remaining gap to the old hand-written kernels (14 launches per step against ~7, and the traffic of the padded record and the fluxes) is small in
  f32; in double-float it is the arithmetic: the flux kernels are 700-operation `Df32` expressions.
