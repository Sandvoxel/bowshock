# bowshock-gpu — f32-storage Luma/WGSL port of bowshock-core

GPU port of the CPU reference solver (`../bowshock-core`: 2D axisymmetric Euler, structured body-fitted grid,
cell-centred FV, MUSCL (minmod / van Albada) + HLLC blended with Rusanov in shocks (`flux::face_flux`),
SSP-RK3 with local time stepping, CFL 0.5, or the line-implicit convergence scheme of M2) on [Luma](../../luma) custom kernels. The state lives in **f32 arrays**; how it is
accumulated and how the residual is evaluated is selectable (`Precision`: `F32` = the original plain-f32 port, `Compensated`,
`Extended` = default, see "The f32 residual floor"). With the default the GPU solver converges like the f64 CPU solver (6 orders on
all 12 cone validation cases). The solver takes any structured axisymmetric grid and BC set of bowshock-core: cone, tangent-ogive cylinder (M = 1.98), the transonic
nose-cylinder case on the large-domain grid with characteristic far-field BCs ("Ogive-cylinder" and "Transonic" below) and, since M2, the subsonic (M ≈ 0.1 – 0.8) nose-cylinder cases with and without a flat blunt base and wake
("Subsonic" below), with the **line-implicit convergence scheme**, the **low-Mach flux correction** and the **solid-block / base-wall handling** of bowshock-core. The CPU solver (f64, rayon) is the reference at run time: `cone_gpu`, `ogive_gpu`, `transonic_gpu` and `subsonic_gpu` run both and compare them.

```
bowshock-gpu/
  src/shaders.rs        WGSL kernels (COMMON = translation of bowshock-core gas.rs/flux.rs; one source per kernel), f32 / compensated update
  src/shaders_ext.rs    double-float (df) WGSL: error-free arithmetic + the df translation of the face kernels and the residual (+ low-Mach correction, f32-geometry closure correction)
  src/shaders_impl.rs   WGSL of the line-implicit scheme: 4x4 block library (flux Jacobian, Roe-type |A|, ghost Jacobians), block assembly, block-tridiagonal line solves, defect correction, positivity-damped update
  src/shaders_visc.rs   WGSL of the viscous terms and the SST model (cell gradients, viscous face fluxes, turbulence gradients / coefficients / update) and the host-side geometry they need
  src/lib.rs            GpuSolver (Mode x Precision): uploads geometry once, records the kernels, residual history, state read-back
bowshock-gpu-cases/
  src/cone.rs           cone runners (CPU loop, GPU loop with identical stopping criteria), post-processing, comparison metrics
  src/run.rs            case-independent CPU / GPU iteration loops (`Monitor` = the stopping criteria), GPU-vs-CPU field difference
  src/ogive.rs          ogive-cylinder runners, wall post-processing (CD, Cp), comparison metrics
  (`bowshock::gpu::GpuRunner`, `bowshock/src/gpu.rs`: GPU run of a `bowshock::case::Case`; cold / warm-started Mach sweeps, same stop rules and post-processing as the CPU `bowshock::run::run_on`)
bowshock-cli/
  src/bin/cone_gpu.rs   validation + benchmark driver (writes out/gpu/*.csv)
  src/bin/ogive_gpu.rs  ogive validation + benchmark driver (writes out/gpu_ogive/*.csv)
  src/bin/transonic_gpu.rs  transonic Mach-sweep validation + benchmark driver (writes out/gpu_transonic/*.csv; `--scheme implicit|explicit`)
  src/bin/subsonic_gpu.rs   subsonic Mach-sweep (base on/off) + Fox TM X-2439 validation + benchmark driver (writes out/gpu_subsonic/*.csv)
bowshock-gpu-cases/
  tests/gpu_vs_cpu.rs   cargo tests (face kernels f32 + df on random states, low-Mach face kernels, 20-iteration state comparison for every mode x precision (cone, ogive, transonic grids),
                        400x200 residual history, small-grid regression test of the residual floor, characteristic far-field BC: free-stream preservation, perturbed state, transonic run + warm start,
                        base block (explicit), line-implicit scheme (sting + base, state after 30 it, short runs, converged state, transonic warm start), cold restart)
  examples/f32_cpu_emulation.rs  CPU experiment: f64 arithmetic with the state constrained to f32 (floor diagnosis)
  examples/f32_diag.rs           GPU experiment: update size vs ulp, f64 vs f32 evaluation of the GPU state, where the residual lives
  examples/f32_floor.rs          GPU experiment: f32 floor vs CFL
  examples/imp_profile.rs        GPU timing: per-kernel time of one line-implicit iteration on the subsonic sting case (best of 20 profiled runs)
```

Run: `cargo test --release -p bowshock-gpu-cases`, `cargo run --release --bin cone_gpu` (everything: 12 cases x {CPU, GPU-guarded extended, GPU-direct
extended, GPU-guarded f32, GPU-guarded compensated}, steady benchmark, Luma launch-overhead + sync-interval studies; ~4 min; `--quick`
skips 400x200, `--limiter minmod|vanalbada`, `--chunk K`, `--bench [--no-cpu]` only the timing studies),
`cargo run --release --bin ogive_gpu [-- --quick] [--direct] [--no-bench] [--bench-only] [--limiter ..] [--chunk K]` (ogive-cylinder, M = 1.98, coarse / medium / fine; ~1.5 min),
`cargo run --release --bin transonic_gpu [-- --quick | --level 0|1|2] [--body cone|karman|lvhaack|sqrt|ogive] [--mach m1,m2,..] [--warm] [--scheme explicit|implicit] [--lowmach on|off] [--no-bench] [--bench-only] ...` (the Mach sweep of the CPU `transonic` bin, same options; minutes),
`cargo run --release --bin subsonic_gpu [-- --quick | --level 0|1|2] [--body ogive|cone|karman|lvhaack|sqrt|parabola] [--mach 0.1,..] [--base on|off|both] [--warm] [--lowmach on|off] [--scheme implicit|explicit] [--no-cpu] [--no-bench] [--bench-only] [--validate]` (the Mach sweep of the CPU `subsonic` bin, same options; ~5 min on level 0, ~30 min on level 1).

## Design

* **State** `Array<[f32;4]>` (`vec4<f32>`) per cell: conserved `u`, `u0` (RK start), primitives `w` (with 2 ghost layers,
  direct mode only), face fluxes `fi`/`fj`, residual `res`, plus `Array<f32>` `dt`, `rho_res` (stage-1 `d rho/dt`). The compensated and
  extended precisions add the low words `ulo`, `u0lo` (same layout), and (extended) the fluxes / primitives are stored as `(hi, lo)` pairs of vec4.
  Geometry is uploaded once from the bowshock-core grid (`bowshock_cases::cone::build_solver(...).grid`): `iface`/`jface` =
  `(nx, nr, len, len*r_mid)` per face (weight computed in f64, then rounded), `cell` = `(A, V, Cx, Cr)` where `(Cx, Cr)` is the closure defect of the cell's f32 face data (see "Geometry closure correction").
* **Uniform block** (`Params`, encase struct of scalars, 144 B) is passed with every launch (Luma copies it into its own
  arena slot per launch): sizes, limiter code (0 first order during the start-up, 1 minmod, 2 van Albada),
  RK stage and stage coefficients, CFL, free stream (f32 + its low words), BC codes, residual-history slot, the low-Mach flag, the solid-block extents (`blk_i, blk_j, blk_s` = `Block {i_end, j_end, j_sting}`),
  the implicit-scheme parameters (pseudo-time CFL of this iteration, entropy fix, flags, pass index) and a runtime `zero` (see
  "compiler pitfalls"). So iteration-dependent parameters (also the implicit CFL ramp) need no extra upload and everything stays in one recorded batch.
* **Two formulations** (`Mode`), same physics, same WGSL functions:
  * `Direct` — line-by-line mirror of `Solver::eval_residual`/`step`: per RK stage `prim` (cons→prim of interior cells),
    `ghost` (BC fill, 2 layers), `flux_i`, `flux_j` (`face_flux` per face), `residual` (cell residual, stage 1 also `dt`
    and `rho_res`), `update` (RK stage). 17 launches per iteration incl. the residual history.
  * `Guarded` (default, optimised) — no primitive/ghost arrays: one `flux_guarded` kernel for i- and j-faces computes the
    stencil primitives from `u` on the fly, boundary ghost states are evaluated lazily by index guards (identical BC
    logic to `fill_ghosts`); one fused `residual_update` kernel (cell residual + RK stage update, residual not stored).
    7 launches per iteration.
* **Three precisions** (`Precision`, orthogonal to `Mode`; why: next section):
  * `F32` — the original port: `u = a (u + dt res) + b u0` rounded to f32, residual in f32.
  * `Compensated` — state = `hi + lo` (two f32 arrays), RK stage in *increment form* `u = u0 + a ((u - u0) + dt res)` (valid because
    `a + b = 1`; only small numbers go through f32 arithmetic) with an error-free (two-sum) addition into `(hi, lo)`. Fluxes still
    evaluated in f32 on `hi`. Cost: 2 extra arrays, a few flops in the update kernel.
  * `Extended` (default) — `Compensated` plus a **double-float (~48 bit) evaluation of the whole residual**: cons→prim of the stencil,
    MUSCL limiter + reconstruction, HLLC, Rusanov blend and shock sensor (`shaders_ext::dface_flux`, a one-to-one translation of
    `flux::face_flux`), the cell flux sum and the axisymmetric source. Only the geometry (normals, face weights, cell area/volume: f32
    inputs), the local time step and the final f32 residual that enters `dt * res` stay f32.
* **Residual history** `sqrt(sum(rho_res^2)/N)` per iteration (needed to apply the CPU stopping criteria
  iteration by iteration): `hist_partial` (direct) / per-workgroup partial sums inside `residual_update` (guarded),
  then `hist_final` writes `hist[slot]`. The host reads `hist` (the only sync) every K iterations (default 50, max 256).
  Luma's own `Array<f32>::norm()` reduction of `rho_res` is used as an independent cross-check (tests; `run_luma_norm`
  in the benchmark) and agrees with the history kernels to 1e-6 relative.
* **Stopping**: the criteria of `run_cone` (6 orders below the peak after iteration 20, plateau test from iteration
  8000, non-finite) are evaluated on the history; the GPU may run up to K-1 iterations past the iteration where the
  criterion fired (reported as `stop` vs `exec`; the final state is the one after `exec` iterations).
* **f32 specifics**: `max(ch2, 1e-300)` in HLLC → `1e-30`; van Albada eps `1e-14` kept; shock-sensor thresholds are read
  from `bowshock_core::flux::SHOCK_SENSOR_LO/HI` when the WGSL source is generated (the df code has them, `GAMMA`, `GAMMA-1`, `1/(GAMMA-1)` and
  the van Albada eps as f64-rounded hi/lo constants).
* **Boundary conditions** (`Config::bc` = `[imin, imax, jmin, jmax]`, converted per launch to the `bc0..bc3` uniforms; the solver is case-independent and takes any structured axisymmetric `bowshock_core::grid::Grid`: cone, ogive,
  large-domain transonic grid, base/wake grid): `Freestream` (0), `Extrapolate` (1), `SlipWall` (2, also the axis), `Characteristic` (3, Riemann-invariant far field of `bowshock_core::bc`) and `PressureOutflow` (4). The ghost-state functions get the
  outward-normal sign of their boundary (`-1` for imin/jmin, `+1` for imax/jmax). The characteristic ghost depends on the first interior cell only (both ghost layers get the same state), so it is computed lazily by the guarded kernels like the
  other BCs; in the extended precision it is evaluated in df (no `pow`: `rho = rho_ref (c_b/c_ref)^5` for gamma = 1.4). Mach number sweeps change the free stream between runs without touching the state (`GpuSolver::config_mut`).
* **Low-Mach correction** (`GpuSolver::set_low_mach`, `Params::low_mach`; mirrors `flux::low_mach_states` / `face_flux_lm`): after the MUSCL reconstruction the jump of the face-normal velocity between the left and right state is scaled by
  `z = clamp(max(M_L, M_R), 0.05, 1)` before HLLC / Rusanov (Thornber et al. 2008). WGSL `low_mach_states`, `reconstruct_lm` (f32) and `dlow_mach_states`, `dreconstruct_lm` (df) inside `face_flux` / `dface_flux`; with the flag off the kernels are the previous ones.
* **Solid block (base / wake grids)** (`GpuSolver::set_block`, `Params::blk_*`; mirrors `Solver::block`, `eval_residual`): cells `i < i_end && j < j_end` (body) and `i >= i_end && j < j_sting` (sting) are never updated and never enter the residual norm
  (`n_active`, the history is `sqrt(sum rho_res^2 / n_active)`); in the guarded flux kernel (f32 and df): zero flux inside the block, the solid cells below the body / sting surface are the mirror image of the fluid cells above it (`prim_j`, = `fill_block_ghosts`),
  and the base-wall face `i = i_end` and its neighbour build their solid stencil cells on the fly as the reflection of the fluid cells about the base-wall normal (the two corner cells need different mirrors for the two directions, hence not stored). Guarded formulation only.
* **Line-implicit convergence scheme** (`GpuSolver::set_scheme(Scheme::Implicit(ImplicitParams))`, `from_host` takes it from the host `Solver`; mirrors `bowshock_core::implicit`, `shaders_impl.rs`; guarded formulation only; multigrid and implicit residual smoothing of the CPU crate are not ported:
  they measured worse there). One iteration = 14 launches with the default `sub_iters = 3` (flux, residual, `hist_final`, 2 block assemblies, 2 line solves, then per extra pass `apply` + 2 line solves, update):
  1. `flux_ext` / `flux_guarded`: face fluxes of the second-order scheme (the explicit kernels, unchanged);
  2. `residual_store(_ext)`: cell residual (df sum collapsed to f32 `res`), pseudo-time step `dt = CFL A / (L_i + L_j)` with the CFL of this iteration (`cfl0 · growth^k` up to `cfl_max`, `k` = iterations since the last `reset` / `restart_ramp`), `rho_res`, workgroup partial sums; `hist_final`;
  3. `assemble_i`, `assemble_j` (`[nj, ni]`, `[ni, nj]`): the 4x4 blocks `lo, d, up` of `I - dt J_dir` per cell from the first-order flux Jacobians with the Roe-type dissipation `|A|` (quadratic polynomial through the three eigenvalues, entropy fix 0.1 c) blended to `lambda I` with the face-flux shock weight,
     the axisymmetric pressure-source Jacobian in the j factor, and the ghost-state Jacobians `T = dU_ghost/dU` (exact for free stream / extrapolation / slip wall and for the reflection of the solid block, central difference for `Characteristic` and `PressureOutflow`) added to the first / last block of every line. All f32 (only the left-hand side);
  4. `line_i`, `line_j`: block-tridiagonal solve by parallel cyclic reduction, one workgroup per line (`nj` i lines, `ni` j lines; lines of the solid block start at its fluid cells): the rows are normalised to `x_k + L x_{k-1} + U x_{k+1} = f`, then `log2 m` forward levels eliminate the odd rows from the even ones (in place, rows in workgroup memory, one barrier per level) and as many backward levels recover them; it solves `(I - dt J_i) X = dt R`, then `(I - dt J_j) dU = X`;
  5. `sub_iters - 1` defect-correction passes against the unfactored operator: `apply` forms `x = dt R - (A_i dU + A_j dU - dU)` from the stored blocks, `line_i`, `line_j` solve again (the reduction is redone, nothing is stored between the passes) and add to `dU`;
  6. `implicit_update`: `U += alpha dU` with the positivity damping of the CPU (halve `alpha` up to 7 times until rho, p > 0), in the compensated precisions with the error-free `df_add`.
  The residual `R` is evaluated in the arithmetic of the selected precision (df in `Extended`), the left-hand side only steers the iteration, so the fixed point is that of the explicit scheme (`implicit_scheme_converged_state_matches_cpu`: the explicit GPU residual of the implicit solution is at the converged level). Blocks are stored as four column vec4 (Luma arrays hold vec4 at most) in
  6 block arrays of `4 ni nj` vec4 (`lo`, `d`, `up` per direction) and one `9 ni nj` scratch of the line solves (lines whose rows do not fit in 16 KiB of workgroup memory keep only every `ks`-th row there and the others in this scratch); i-line blocks are stored transposed (`i nj + j`).
* **Geometry closure correction** (`Extended` precision): the f32 face weights / normals of a cell do not satisfy `sum_f w n = (0, A)` exactly (relative error 6e-8), so a perfectly uniform flow has a spurious residual of order `p · 6e-8 · w / V`, large in
  the high-aspect-ratio cells of the large-domain grids near the axis and in the far field (residual `1e-8` in `d rho / dt` units there). The host sums the defect `(Cx, Cr) = sum_f w n - (0, A)` of every cell in f64 from the very f32 numbers the GPU uses and stores it in `cell.zw` (the former `x_c, r_c`, unused by the kernels); the extended residual kernels add `Fx Cx + Fr Cr`
  (`dgeo_corr`: the Euler flux of the cell state for unit normals in x and r) to the flux sum, i.e. they cancel the effect of the rounding on a uniform flow to df accuracy. Effect: the GPU residual floor on the subsonic cases fell from `1.5e-8` to `1.7e-10` (the CPU's own floor, 2e-10), and the GPU–CPU agreement of every extended-precision test improved (below).
* **Viscous terms and the SST model** (`GpuSolver::set_viscous(Some(Viscous), &grid)`, `from_host` takes it from the host `Solver`; mirrors `bowshock_core::viscous` / `turbulence`, `shaders_visc.rs`; `Mode::Guarded` + `Scheme::Implicit` only, else `GpuError::Unsupported`). `Params::vflags` (bit 1 viscous, bit 2 SST), `wall_i0`, `mu_inf = 1/re_c`, `suth`, `k_inf`, `om_inf` (the struct is 144 B).
  The no-slip faces are the CPU's (`Walls`): the plate wall from `wall_i0`, the body surface, base wall and sting surface of a block; `wall_ghost` replaces `reflect_wall` at every wall mirror of the flux kernels (f32 and df) and the assembly (state and Jacobian), and is exactly `reflect_wall` for the Euler equations (the kernels of an inviscid run are unchanged).
  Host uploads, computed in f64 from the `Grid` with the ghost-mirror rule of `viscous::stencil` and stored as f32: per face `(e_x, e_r, 1/|d|, r_m)` (`vgi`, `vgj`), per cell `(1/A, r_c, d_wall, 0)` (`bowshock_core::turbulence::wall_distances`), and the state `turb = (k, omega, mu_t, mu)`; `GpuSolver::reset` returns it to the free stream (as the CPU runner does for a cold start).
  Per iteration (see the table in `shaders_visc.rs`): `vgrad`, `vflux` after the inviscid flux kernel, the residual kernel (viscous variant) subtracts `fv`, adds `-t_tt A/V` and the diffusive `dt` term, the assembly adds `2 lam I` and the no-slip Jacobians; then, for SST, `turb_grad`, `turb_coef`, the two scalar line solves with the existing cyclic-reduction kernels
  (4x4 blocks that are diagonal) and `turb_update`, all before the mean-flow update because the CPU turbulence update works from the primitives of the start of the iteration.

## Kernel list

| kernel | problem size | mirrors (bowshock-core) | mode / precision |
|---|---|---|---|
| `prim` | `[ni, nj]` | `eval_residual` step 1 (`cons_to_prim`) | direct |
| `ghost` | `[ni + nj]` | `fill_ghosts` / `ghost_state` | direct |
| `flux_i`, `flux_j` | `[ni+1, nj]`, `[ni, nj+1]` | `face_flux` per face (reconstruct → HLLC → Rusanov blend) | direct |
| `residual` | `[ni, nj]` | cell residual + source `pA/V`, stage 1: local `dt`, `rho_res` | direct |
| `update` | `[ni, nj]` | `Solver::stage` (`u = a (u + dt res) + b u0`; compensated: increment form into `hi + lo`) | direct |
| `hist_partial` → `hist_final` | `[128*256]`, `[256]` | `density_residual` per iteration | direct |
| `flux_guarded` | `[ni+1, 2nj+1]` | `face_flux` for i- and j-faces with lazy ghosts | guarded |
| `residual_update` | `[ni, nj]` | residual + `stage` fused, stage 1: `dt`, `rho_res`, workgroup partial sums | guarded |
| `hist_final` | `[256]` | `density_residual` per iteration | guarded |
| `prim_ext`, `ghost_ext`, `flux_i_ext`, `flux_j_ext`, `residual_ext` | as the f32 kernels | the same functions in df; `update` is the compensated one | direct, extended |
| `flux_ext`, `residual_update_ext` | as `flux_guarded`, `residual_update` | the same functions in df | guarded, extended |
| `residual_store`, `residual_store_ext` | `[ni, nj]` | `eval_residual` + `compute_dt` (implicit: residual stored, `dt` of the pseudo-time CFL, workgroup partial sums) | implicit |
| `assemble_i`, `assemble_j` | `[nj, ni]`, `[ni, nj]` | `implicit::assemble` (`cell_blocks`, `abs_jacobian`, `ghost_jacobian`) | implicit |
| `line_i`, `line_j` | `[nj]`, `[ni]` | `implicit::solve_lines` (`block_thomas`), pass 0 factorises, later passes reuse the factors | implicit |
| `apply` | `[ni, nj]` | `implicit::apply_factor` + defect `dt R - (A_i dU + A_j dU - dU)` | implicit |
| `implicit_update` | `[ni, nj]` | update of `implicit::step` with positivity damping | implicit |
| `vgrad` | `[ni, nj]` | `viscous::cell_pass` (cell gradients of `u, v, T`, `mu(T)`) | viscous |
| `vflux` | `[ni+1, 2nj+1]` | `viscous::face_pass` (compact face gradient, stress tensor, heat flux into `fvi`, `fvj`) | viscous |
| `residual_store(_ext)` viscous variants, `assemble_i/j` viscous variants | as above | `F - Fv`, hoop stress `t_tt`, diffusive `dt`; no-slip ghost + Jacobian `diag(1,-1,-1,1)`, `2 lam I` | viscous |
| `turb_grad` | `[ni, nj]` | `turbulence::scalar_gradients` + `blends` (`F1`, `X`) | SST |
| `turb_coef` | `[ni, nj]` | `turbulence::face_pass` + `cell_terms` (residual, `diag0`, the six diagonal blocks, `tres`) | SST |
| `line_i`, `line_j` on the turbulence blocks | `[nj]`, `[ni]` | `solve_lines` with diagonal blocks (k, omega in entries 0, 5) | SST |
| `turb_update` | `[ni, nj]` | clipped update of `k`, `omega` and `mu_t` (`turbulence::update`) | SST |
| `face_test`, `face_test_ext` | `[n]` | test only: reconstruct / HLLC / `face_flux` on arbitrary states (f32 / df) | tests |

WGSL functions (COMMON): `cons_to_prim`, `sound_speed`, `physical_flux`, `reflect_wall`, `ghost_state`, `limited_slope`
(first order / minmod / van Albada), `reconstruct` (with the positivity fallback), `hllc`, `rusanov`, `shock_weight`, `face_flux`; `df_add` (compensated
state update). `shaders_ext`: df arithmetic `ts/qts/tp/dadd/dsub/dmul/dmulf/ddiv/dsqrt/dmin/dmax/dabs` and the df versions `dcons_to_prim`, `dslope`,
`dreconstruct`, `dhllc`, `drusanov`, `dshock_weight`, `dface_flux`, `dlow_mach_states`, `dreconstruct_lm`, `dgeo_corr`. `shaders_impl`: `inv4`, `flux_jacobian`, `abs_jacobian`, `reflection`, `ghost_jac`, `cell_blocks`.
Far-field BCs (COMMON, f32): `characteristic_ghost`, `pressure_outflow_ghost`, `far_ghost` (ghost states of `bowshock_core::bc`; `ghost_state` dispatches on the BC code); df versions in `shaders_ext`: `dcharacteristic_ghost`, `dpressure_outflow_ghost`, `dfar_ghost`.
Viscous runs: COMMON `wall_ghost` (`reflect_wall` or the no-slip `(rho, -u, -v, p)`), `noslip_jface`, `noslip_iface` (the no-slip classification of `bowshock_core::viscous::Walls`), df `dwall_ghost`; `shaders_visc`: `iface_sides`, `jface_sides` (the two sides of a face with ghosts), `visc_mu`, `iface_kw`, `jface_kw`, `strain2`, `blend_args`, `tface`, `dir_terms`.

## Results

All numbers below are produced by `cargo run --release --bin cone_gpu` (RTX 3090 via Luma/wgpu/Vulkan; CPU = bowshock-core, f64,
rayon, 32 threads; M = 2, CFL 0.5, 300 first-order start-up iterations, convergence criteria of `run_cone`; GPU = default `Precision::Extended` unless a column says
f32 / compensated). Output CSVs: `out/gpu/` (`cp_<angle>deg_<ni>x<nj>_<limiter>.csv`: GPU/CPU/Taylor-Maccoll Cp per station incl. relative difference;
`residual_*.csv`: GPU and CPU density residual per iteration; `summary.csv`: one row per case incl. the f32 / compensated results).
The CPU side reproduces bowshock-core's own reference table exactly (iteration counts 2785/3918/5880 and 1900/3824/5185 for minmod 10°/15°,
1352/2240/4080 and 1518/2531/4654 for van Albada; all 6.0 orders).

### Summary

* **Convergence** — with the default precision (`Extended`) the GPU density residual drops **6.0 orders in all 12 cases** and the run stops on the CPU's own criterion
  after 1352 – 5654 iterations (CPU: 1352 – 5880; per case within 3.9 %). The original f32 solver (`F32`) stalled at 3.9 – 4.6 orders,
  the compensated state alone at 4.3 – 5.0. Cause, evidence and fix: "The f32 residual floor" below.
* **Accuracy** — averaged cone Cp (0.4 ≤ x/L ≤ 0.9, wall-extrapolated) of the GPU differs from the CPU by **2.2e-06 % … 3.2e-05 %** in all 12 cases (before:
  0.0014 … 0.011 %; target 0.1 %); per-station relative Cp difference ≤ 1.8e-04 % in the averaging window and ≤ 3.0e-04 % over all stations (tip region included;
  before ≤ 0.014 %); shock angle identical; the Cp error w.r.t. Taylor-Maccoll is that of the CPU to the printed digits (e.g. 10°, van Albada, 400×200: GPU −0.015 %, CPU −0.015 %;
  before −0.003 % vs −0.015 %). The field difference at the end of the runs is ≤ 2.8e-06 p∞ (max), rms ≤ 1.2e-07 p∞, max relative density difference ≤ 1.3e-06
  (before: ≤ 2.1e-04 / 2.6e-05). The systematic GPU-higher-than-CPU Cp offset of the f32 solver (it grew with refinement: it was the f32 state sitting on its noise floor) is gone.
  `Direct` and `Guarded` GPU formulations agree to the printed digits (columns "direct-mode ..." below). The remaining GPU–CPU difference is the f32 rounding of the geometry (see the floor section).
* **Performance** (steady state, best of 3, sync every K = 50 iterations, per-iteration residual history included; Luma main f29290fb): CPU 596 / 1211 / 2960 µs, GPU-guarded **66.9 / 74.2 / 235.8 µs**
  (8.9x / 16.3x / 12.6x faster), GPU-direct 83.5 / 94.6 / 233.8 µs (7.1x / 12.8x / 12.7x) for 100×50 / 200×100 / 400×200.
  The extended precision costs **2.86x / 2.58x / 3.93x** of the f32 guarded solver (23.4 / 28.8 / 60.0 µs; which is 25x / 42x / 49x faster than the CPU). Time to each solver's own stop (6 orders for both now; iterations × steady cost):
  0.09 – 1.33 s on the GPU vs 0.81 – 17.40 s on the CPU, i.e. 8.8x – 17.0x in favour of the GPU-guarded solver (7.1x – 13.3x direct). Before, the GPU ran ≥ 8001 iterations until its plateau test fired
  without reaching the criterion. Setup (kernel build + upload): ≤ 33 ms, first-launch warm-up ≤ 16 ms, context creation 0.2 s (the first time the GPU is touched).
* **The solver is device-bound now**: with Luma's launch cost down to ≈ 0.8 µs (it was ≈ 8.5 µs when the first version of these tables was made) 100×50 and 200×100 still cost almost the same (66.9 / 74.2 µs extended) but the host recording of an iteration
  is only 9 µs (guarded) / 24 µs (direct); fusing 17 launches/iteration (direct) into 7 (guarded) now gains 1.1 – 1.3x for the extended precision at 100×50 – 200×100 and nothing at 400×200, and the extended precision costs 2.6 – 3.9x the f32 solver on every grid.

### Accuracy: GPU (extended precision, default) vs CPU (f64) and vs Taylor-Maccoll; before = original f32 solver, compensated = compensated state only
| cone | grid | limiter | Cp GPU | Cp CPU | Cp TM | GPU err vs TM % | CPU err vs TM % | avg Cp diff GPU-CPU % | max rel Cp diff (0.4<=x/L<=0.9) % | max rel Cp diff (all stations) % | beta GPU [deg] | beta CPU [deg] | direct-mode avg Cp diff vs CPU % | direct-mode max rel Cp diff (window) % | f32 (before) avg Cp diff % | f32 (before) max rel Cp diff (window) % | compensated avg Cp diff % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 0.10411 | 0.10411 | 0.10447 | -0.347 | -0.347 | -8.9e-6 | 1.2e-4 | 1.7e-4 | 31.253 | 31.253 | -8.9e-6 | 1.2e-4 | +2.8e-3 | 4.4e-3 | +5.7e-5 |
| 10 deg | 200x100 | minmod | 0.10434 | 0.10434 | 0.10447 | -0.128 | -0.128 | -1.7e-5 | 1.0e-4 | 3.0e-4 | 31.263 | 31.263 | -1.7e-5 | 1.0e-4 | +5.3e-3 | 6.9e-3 | +9.6e-5 |
| 10 deg | 400x200 | minmod | 0.10441 | 0.10441 | 0.10447 | -0.056 | -0.056 | +3.2e-5 | 1.8e-4 | 2.8e-4 | 31.241 | 31.241 | +3.2e-5 | 1.8e-4 | +9.4e-3 | 1.2e-2 | +6.6e-5 |
| 15 deg | 100x50 | minmod | 0.20169 | 0.20169 | 0.20225 | -0.274 | -0.274 | -3.6e-6 | 5.0e-5 | 9.2e-5 | 33.994 | 33.994 | -3.6e-6 | 5.0e-5 | +1.7e-3 | 2.4e-3 | +4.7e-5 |
| 15 deg | 200x100 | minmod | 0.20195 | 0.20195 | 0.20225 | -0.145 | -0.145 | -2.7e-5 | 6.7e-5 | 1.7e-4 | 33.944 | 33.944 | -2.7e-5 | 6.7e-5 | +3.0e-3 | 3.9e-3 | +7.1e-6 |
| 15 deg | 400x200 | minmod | 0.20208 | 0.20208 | 0.20225 | -0.081 | -0.081 | -1.5e-5 | 6.7e-5 | 7.3e-5 | 33.925 | 33.925 | -1.5e-5 | 6.7e-5 | +5.2e-3 | 6.9e-3 | +2.3e-5 |
| 10 deg | 100x50 | vanalbada | 0.10433 | 0.10433 | 0.10447 | -0.138 | -0.138 | +4.9e-6 | 6.3e-5 | 1.1e-4 | 31.228 | 31.228 | +4.9e-6 | 6.3e-5 | +2.4e-3 | 3.4e-3 | +5.2e-5 |
| 10 deg | 200x100 | vanalbada | 0.10443 | 0.10443 | 0.10447 | -0.041 | -0.041 | +3.8e-6 | 6.7e-5 | 1.0e-4 | 31.238 | 31.238 | +3.8e-6 | 6.7e-5 | +6.2e-3 | 8.8e-3 | +9.9e-5 |
| 10 deg | 400x200 | vanalbada | 0.10446 | 0.10446 | 0.10447 | -0.015 | -0.015 | +2.3e-5 | 9.0e-5 | 1.2e-4 | 31.227 | 31.227 | +2.3e-5 | 9.0e-5 | +1.1e-2 | 1.4e-2 | +6.2e-5 |
| 15 deg | 100x50 | vanalbada | 0.20209 | 0.20209 | 0.20225 | -0.079 | -0.079 | -2.2e-6 | 4.0e-5 | 9.9e-5 | 33.972 | 33.972 | -2.2e-6 | 4.0e-5 | +1.4e-3 | 2.0e-3 | +2.8e-5 |
| 15 deg | 200x100 | vanalbada | 0.20216 | 0.20216 | 0.20225 | -0.043 | -0.043 | -6.7e-6 | 4.9e-5 | 4.9e-5 | 33.942 | 33.942 | -6.7e-6 | 4.9e-5 | +3.4e-3 | 4.8e-3 | +4.6e-5 |
| 15 deg | 400x200 | vanalbada | 0.20220 | 0.20220 | 0.20225 | -0.024 | -0.024 | -4.2e-6 | 5.0e-5 | 5.5e-5 | 33.925 | 33.925 | -4.2e-6 | 5.0e-5 | +6.2e-3 | 8.4e-3 | +5.6e-5 |

### Field difference GPU (guarded, extended) - CPU at the end of the runs (f32 = original f32 solver)
| cone | grid | limiter | max abs dp/p_inf | rms dp/p_inf | max rel drho/rho | f32: max abs dp/p_inf | f32: rms dp/p_inf | f32: max rel drho/rho |
|---|---|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 3.61e-7 | 7.10e-8 | 3.12e-7 | 1.41e-5 | 5.19e-6 | 2.18e-5 |
| 10 deg | 200x100 | minmod | 7.64e-7 | 1.02e-7 | 5.43e-7 | 2.30e-5 | 9.73e-6 | 2.72e-5 |
| 10 deg | 400x200 | minmod | 9.54e-7 | 1.02e-7 | 6.42e-7 | 5.22e-5 | 1.96e-5 | 6.96e-5 |
| 15 deg | 100x50 | minmod | 5.39e-7 | 7.83e-8 | 2.47e-7 | 1.48e-5 | 6.04e-6 | 1.86e-5 |
| 15 deg | 200x100 | minmod | 1.04e-6 | 1.23e-7 | 6.56e-7 | 5.21e-5 | 1.15e-5 | 3.31e-5 |
| 15 deg | 400x200 | minmod | 1.13e-6 | 1.09e-7 | 7.39e-7 | 1.32e-4 | 2.25e-5 | 9.24e-5 |
| 10 deg | 100x50 | vanalbada | 1.07e-6 | 1.01e-7 | 7.76e-7 | 1.18e-5 | 4.59e-6 | 2.08e-5 |
| 10 deg | 200x100 | vanalbada | 1.13e-6 | 9.49e-8 | 7.98e-7 | 2.98e-5 | 1.22e-5 | 3.09e-5 |
| 10 deg | 400x200 | vanalbada | 1.18e-6 | 7.91e-8 | 8.39e-7 | 7.22e-5 | 2.37e-5 | 6.28e-5 |
| 15 deg | 100x50 | vanalbada | 9.10e-7 | 9.84e-8 | 4.42e-7 | 1.26e-5 | 4.82e-6 | 1.80e-5 |
| 15 deg | 200x100 | vanalbada | 1.07e-6 | 8.29e-8 | 5.13e-7 | 6.45e-5 | 1.26e-5 | 4.27e-5 |
| 15 deg | 400x200 | vanalbada | 2.78e-6 | 8.83e-8 | 1.34e-6 | 2.15e-4 | 2.63e-5 | 1.38e-4 |

### Convergence (density residual drop from its peak after iteration 20; stop = criterion of `run_cone`)
| cone | grid | limiter | CPU iters | CPU stop | CPU drop (orders) | CPU final res | GPU iters (stop/exec) | GPU stop | GPU drop (orders) | GPU final res | GPU-direct iters (stop/exec) | GPU-direct stop | GPU-direct drop | f32 (before): stop | f32 drop | f32 tail geo-mean res | compensated: stop | compensated drop | compensated tail geo-mean res |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 2785 | residual drop reached | 6.0 | 1.39e-6 | 2816/2850 | residual drop reached | 6.0 | 1.39e-6 | 2816/2850 | residual drop reached | 6.0 | residual plateau | 4.1 | 1.15e-4 | residual plateau | 4.5 | 4.35e-5 |
| 10 deg | 200x100 | minmod | 3918 | residual drop reached | 6.0 | 2.37e-6 | 3765/3800 | residual drop reached | 6.0 | 2.37e-6 | 3765/3800 | residual drop reached | 6.0 | residual plateau | 3.9 | 2.78e-4 | residual plateau | 4.4 | 1.05e-4 |
| 10 deg | 400x200 | minmod | 5880 | residual drop reached | 6.0 | 3.77e-6 | 5654/5700 | residual drop reached | 6.0 | 3.76e-6 | 5654/5700 | residual drop reached | 6.0 | residual plateau | 3.9 | 5.25e-4 | residual plateau | 4.3 | 1.90e-4 |
| 15 deg | 100x50 | minmod | 1900 | residual drop reached | 6.0 | 2.29e-6 | 1900/1900 | residual drop reached | 6.0 | 2.28e-6 | 1900/1900 | residual drop reached | 6.0 | residual plateau | 4.4 | 8.08e-5 | residual plateau | 4.8 | 3.46e-5 |
| 15 deg | 200x100 | minmod | 3824 | residual drop reached | 6.0 | 3.75e-6 | 3814/3850 | residual drop reached | 6.0 | 3.75e-6 | 3814/3850 | residual drop reached | 6.0 | residual plateau | 4.3 | 2.02e-4 | residual plateau | 4.6 | 8.61e-5 |
| 15 deg | 400x200 | minmod | 5185 | residual drop reached | 6.0 | 5.81e-6 | 5099/5100 | residual drop reached | 6.0 | 5.82e-6 | 5099/5100 | residual drop reached | 6.0 | residual plateau | 4.1 | 4.19e-4 | residual plateau | 4.6 | 1.64e-4 |
| 10 deg | 100x50 | vanalbada | 1352 | residual drop reached | 6.0 | 1.33e-6 | 1352/1400 | residual drop reached | 6.0 | 1.34e-6 | 1352/1400 | residual drop reached | 6.0 | residual plateau | 4.3 | 6.29e-5 | residual plateau | 4.8 | 2.45e-5 |
| 10 deg | 200x100 | vanalbada | 2240 | residual drop reached | 6.0 | 2.32e-6 | 2240/2250 | residual drop reached | 6.0 | 2.33e-6 | 2240/2250 | residual drop reached | 6.0 | residual plateau | 4.3 | 1.10e-4 | residual plateau | 4.7 | 4.66e-5 |
| 10 deg | 400x200 | vanalbada | 4080 | residual drop reached | 6.0 | 3.67e-6 | 4080/4100 | residual drop reached | 6.0 | 3.69e-6 | 4080/4100 | residual drop reached | 6.0 | residual plateau | 4.2 | 2.28e-4 | residual plateau | 4.6 | 9.12e-5 |
| 15 deg | 100x50 | vanalbada | 1518 | residual drop reached | 6.0 | 2.22e-6 | 1518/1550 | residual drop reached | 6.0 | 2.22e-6 | 1518/1550 | residual drop reached | 6.0 | residual plateau | 4.6 | 6.15e-5 | residual plateau | 5.0 | 2.53e-5 |
| 15 deg | 200x100 | vanalbada | 2531 | residual drop reached | 6.0 | 3.70e-6 | 2531/2550 | residual drop reached | 6.0 | 3.71e-6 | 2531/2550 | residual drop reached | 6.0 | residual plateau | 4.6 | 9.93e-5 | residual plateau | 4.9 | 4.81e-5 |
| 15 deg | 400x200 | vanalbada | 4654 | residual drop reached | 6.0 | 5.73e-6 | 4654/4700 | residual drop reached | 6.0 | 5.75e-6 | 4654/4700 | residual drop reached | 6.0 | residual plateau | 4.4 | 2.25e-4 | residual plateau | 4.8 | 9.37e-5 |

### Residual of the converged CPU f64 state evaluated by the GPU solver in f32 (original) and in extended precision
(f32: 20 iterations from the CPU state; extended: the GPU operator has f32-rounded geometry (normals, face weights, volumes), so it differs from the CPU operator by a fixed small amount, which is what this measures)
| cone | grid | limiter | CPU final residual (f64) | f32 residual, iteration 1 | f32 residual, geo-mean of 20 | extended residual, iteration 1 | extended residual, geo-mean of 20 |
|---|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 1.39e-6 | 2.11e-5 | 2.35e-5 | 7.71e-6 | 2.87e-6 |
| 10 deg | 200x100 | minmod | 2.37e-6 | 3.88e-5 | 4.72e-5 | 1.56e-5 | 5.91e-6 |
| 10 deg | 400x200 | minmod | 3.77e-6 | 7.37e-5 | 9.15e-5 | 3.05e-5 | 1.14e-5 |
| 15 deg | 100x50 | minmod | 2.29e-6 | 2.11e-5 | 2.34e-5 | 8.32e-6 | 3.46e-6 |
| 15 deg | 200x100 | minmod | 3.75e-6 | 3.97e-5 | 4.67e-5 | 1.65e-5 | 6.78e-6 |
| 15 deg | 400x200 | minmod | 5.81e-6 | 7.50e-5 | 9.00e-5 | 3.26e-5 | 1.26e-5 |
| 10 deg | 100x50 | vanalbada | 1.33e-6 | 1.92e-5 | 2.11e-5 | 7.66e-6 | 1.92e-6 |
| 10 deg | 200x100 | vanalbada | 2.32e-6 | 3.63e-5 | 3.98e-5 | 1.56e-5 | 3.69e-6 |
| 10 deg | 400x200 | vanalbada | 3.67e-6 | 6.99e-5 | 7.70e-5 | 3.05e-5 | 7.04e-6 |
| 15 deg | 100x50 | vanalbada | 2.22e-6 | 1.93e-5 | 2.07e-5 | 8.31e-6 | 2.32e-6 |
| 15 deg | 200x100 | vanalbada | 3.70e-6 | 3.66e-5 | 4.00e-5 | 1.64e-5 | 4.51e-6 |
| 15 deg | 400x200 | vanalbada | 5.73e-6 | 7.08e-5 | 7.66e-5 | 3.26e-5 | 8.28e-6 |

### Steady per-iteration cost (15 deg, minmod, 2000 iterations from the free stream, best of 3, sync every K=50; Luma main f29290fb, RTX 3090, load average 7 at the start)
| grid | cells | CPU f64 rayon us/iter | GPU-guarded extended us/iter | guarded compensated | guarded f32 | GPU-direct extended | direct f32 | guarded extended speed-up vs CPU | direct extended speed-up | extended / f32 cost (guarded) |
|---|---|---|---|---|---|---|---|---|---|---|
| 100x50 | 5000 | 596 | 66.9 | 24.4 | 23.4 | 83.5 | 44.2 | 8.9x | 7.1x | 2.86x |
| 200x100 | 20000 | 1211 | 74.2 | 30.5 | 28.8 | 94.6 | 51.0 | 16.3x | 12.8x | 2.58x |
| 400x200 | 80000 | 2960 | 235.8 | 75.5 | 60.0 | 233.8 | 99.1 | 12.6x | 12.7x | 3.93x |

### Total time to the stopping criterion of each solver (iterations to stop x steady us/iter above; GPU extended and CPU both stop at 6 orders; the f32 solver (before) ends at its plateau test)
| cone | grid | limiter | CPU iters | CPU s | GPU-guarded ext iters (stop) | GPU-guarded ext s | GPU-direct ext s | guarded speed-up | direct speed-up | f32 guarded iters (stop) | f32 guarded s | f32 orders reached |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 2785 | 1.66 | 2816 | 0.19 | 0.24 | 8.8x | 7.1x | 8001 | 0.19 | 4.1 |
| 10 deg | 200x100 | minmod | 3918 | 4.74 | 3765 | 0.28 | 0.36 | 17.0x | 13.3x | 8001 | 0.23 | 3.9 |
| 10 deg | 400x200 | minmod | 5880 | 17.40 | 5654 | 1.33 | 1.32 | 13.1x | 13.2x | 8001 | 0.48 | 3.9 |
| 15 deg | 100x50 | minmod | 1900 | 1.13 | 1900 | 0.13 | 0.16 | 8.9x | 7.1x | 8001 | 0.19 | 4.4 |
| 15 deg | 200x100 | minmod | 3824 | 4.63 | 3814 | 0.28 | 0.36 | 16.4x | 12.8x | 8001 | 0.23 | 4.3 |
| 15 deg | 400x200 | minmod | 5185 | 15.35 | 5099 | 1.20 | 1.19 | 12.8x | 12.9x | 9001 | 0.54 | 4.1 |
| 10 deg | 100x50 | vanalbada | 1352 | 0.81 | 1352 | 0.09 | 0.11 | 8.9x | 7.1x | 8001 | 0.19 | 4.3 |
| 10 deg | 200x100 | vanalbada | 2240 | 2.71 | 2240 | 0.17 | 0.21 | 16.3x | 12.8x | 8001 | 0.23 | 4.3 |
| 10 deg | 400x200 | vanalbada | 4080 | 12.08 | 4080 | 0.96 | 0.95 | 12.6x | 12.7x | 8001 | 0.48 | 4.2 |
| 15 deg | 100x50 | vanalbada | 1518 | 0.90 | 1518 | 0.10 | 0.13 | 8.9x | 7.1x | 8001 | 0.19 | 4.6 |
| 15 deg | 200x100 | vanalbada | 2531 | 3.07 | 2531 | 0.19 | 0.24 | 16.3x | 12.8x | 8001 | 0.23 | 4.6 |
| 15 deg | 400x200 | vanalbada | 4654 | 13.78 | 4654 | 1.10 | 1.09 | 12.6x | 12.7x | 9001 | 0.54 | 4.4 |

### One-time GPU setup (kernel build + geometry/state upload) and first-launch warm-up, seconds (context creation 0.26 s); extended precision
| cone | grid | limiter | guarded setup | guarded warm-up | direct setup | direct warm-up |
|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 0.018 | 0.006 | 0.033 | 0.000 |
| 10 deg | 200x100 | minmod | 0.003 | 0.003 | 0.001 | 0.000 |
| 10 deg | 400x200 | minmod | 0.011 | 0.016 | 0.003 | 0.004 |
| 15 deg | 100x50 | minmod | 0.000 | 0.001 | 0.000 | 0.000 |
| 15 deg | 200x100 | minmod | 0.002 | 0.003 | 0.000 | 0.001 |
| 15 deg | 400x200 | minmod | 0.008 | 0.014 | 0.003 | 0.001 |
| 10 deg | 100x50 | vanalbada | 0.000 | 0.000 | 0.000 | 0.000 |
| 10 deg | 200x100 | vanalbada | 0.001 | 0.006 | 0.000 | 0.001 |
| 10 deg | 400x200 | vanalbada | 0.008 | 0.014 | 0.003 | 0.001 |
| 15 deg | 100x50 | vanalbada | 0.000 | 0.001 | 0.000 | 0.000 |
| 15 deg | 200x100 | vanalbada | 0.001 | 0.006 | 0.000 | 0.001 |
| 15 deg | 400x200 | vanalbada | 0.008 | 0.014 | 0.002 | 0.001 |

### Wall time of the convergence runs themselves (iteration loop incl. history readback every K=50; setup excluded; measured while other heavy jobs kept the machine at load average 20 - 60, so the CPU column is inflated by ~1.5 - 2x and everything is noisy: use the steady benchmark above)
| cone | grid | limiter | CPU iters | CPU us/iter | GPU-guarded ext us/iter | GPU-direct ext us/iter | GPU iters exec (stop) | CPU total s | GPU-guarded ext total s | GPU-direct ext total s | f32 guarded total s (8000+ its) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 2785 | 1037 | 131 | 88 | 2850 (2816) | 2.89 | 0.37 | 0.25 | 0.22 |
| 10 deg | 200x100 | minmod | 3918 | 1871 | 158 | 97 | 3800 (3765) | 7.33 | 0.60 | 0.37 | 0.27 |
| 10 deg | 400x200 | minmod | 5880 | 4499 | 276 | 217 | 5700 (5654) | 26.45 | 1.57 | 1.24 | 0.51 |
| 15 deg | 100x50 | minmod | 1900 | 970 | 147 | 98 | 1900 (1900) | 1.84 | 0.28 | 0.19 | 0.24 |
| 15 deg | 200x100 | minmod | 3824 | 1968 | 137 | 104 | 3850 (3814) | 7.53 | 0.53 | 0.40 | 0.37 |
| 15 deg | 400x200 | minmod | 5185 | 5655 | 300 | 219 | 5100 (5099) | 29.32 | 1.53 | 1.12 | 0.63 |
| 10 deg | 100x50 | vanalbada | 1352 | 1066 | 75 | 97 | 1400 (1352) | 1.44 | 0.10 | 0.14 | 0.24 |
| 10 deg | 200x100 | vanalbada | 2240 | 1942 | 206 | 116 | 2250 (2240) | 4.35 | 0.46 | 0.26 | 0.31 |
| 10 deg | 400x200 | vanalbada | 4080 | 4670 | 324 | 252 | 4100 (4080) | 19.05 | 1.33 | 1.03 | 0.53 |
| 15 deg | 100x50 | vanalbada | 1518 | 756 | 75 | 111 | 1550 (1518) | 1.15 | 0.12 | 0.17 | 0.24 |
| 15 deg | 200x100 | vanalbada | 2531 | 1783 | 185 | 112 | 2550 (2531) | 4.51 | 0.47 | 0.29 | 0.31 |
| 15 deg | 400x200 | vanalbada | 4654 | 4653 | 323 | 250 | 4700 (4654) | 21.66 | 1.52 | 1.18 | 0.60 |

### Luma launch overhead: 4000 dependent launches of `a[i] += 1` (one storage binding, luma grid uniform only)
| invocations | host record us/launch | record+sync us/launch |
|---|---|---|
| 64 | 0.86 | 1.23 |
| 5000 | 0.75 | 1.30 |
| 80000 | 0.83 | 2.06 |
| 1000000 | 0.93 | 11.01 |

### GPU per-iteration timing study (15 deg, minmod, from free stream, 1000 iterations, f32 precision; first-order phase off)
Residual history = per-iteration density residual (history kernels), read back every K iterations.
| mode | grid | hist, sync K=1 | K=10 | K=50 | K=250 | Luma norm() every 50 (no hist kernels) | no residual at all, record+sync | host recording time only |
|---|---|---|---|---|---|---|---|---|
| Direct | 100x50 | 120.3 us | 66.7 us | 45.9 us | 40.7 us | 42.2 us | 35.2 us | 23.8 us |
| Direct | 200x100 | 126.8 us | 73.2 us | 50.9 us | 45.8 us | 46.5 us | 40.7 us | 23.3 us |
| Direct | 400x200 | 177.5 us | 120.6 us | 102.1 us | 94.3 us | 94.8 us | 88.1 us | 25.4 us |
| Guarded | 100x50 | 89.8 us | 32.8 us | 24.9 us | 18.6 us | 23.4 us | 15.5 us | 9.2 us |
| Guarded | 200x100 | 90.1 us | 36.9 us | 28.7 us | 22.2 us | 27.2 us | 19.2 us | 9.4 us |
| Guarded | 400x200 | 122.1 us | 66.9 us | 59.6 us | 52.8 us | 57.7 us | 48.7 us | 8.7 us |

Reading the study (f32 precision, Luma main): with `no residual at all` the guarded solver costs 15.5 / 19.2 / 48.7 µs per iteration for 100×50 / 200×100 / 400×200 (host recording alone: 9.2 / 9.4 / 8.7 µs = 7 launches × 1.3 µs), the direct solver
35.2 / 40.7 / 88.1 µs (host recording ≈ 24 µs = 17 launches × 1.4 µs): both are device-side bound now, not host bound. The history kernel adds ≈ 9 – 11 µs per iteration to the guarded solver (24.9 / 28.7 / 59.6 µs at K = 50);
a synchronisation costs ≈ 0.07 ms at K = 1, ≈ 0.14 ms at K = 10 and ≈ 0.3 ms at K = 50 (guarded 100×50: (time/iter − time/iter at K = 250) × K), and K = 250 brings the guarded solver within 3 – 4 µs of the no-residual cost.
`Luma norm() every 50` (Luma's `Array<f32>::norm`, no history kernels) costs 23.4 / 27.2 / 57.7 µs, 1.5 – 2 µs less than the history kernels at K = 50, and only gives the last iteration's residual, so the history kernels remain the default. The extended
solver adds only device time, see the steady-state table above.

## Ogive-cylinder, M = 1.98 (`ogive_gpu`)

The solver needed no change for this case: the grid (`bowshock_cases::ogive::ogive_grid` via `OgiveCase::build_solver()`: 114×50 / 228×100 / 456×200 cells, free-stream inflow `imin` and outer boundary `jmax`, extrapolation outflow `imax`, slip wall + axis `jmin`)
and `Config` are uploaded exactly like the cone's (`GpuSolver::new_with(&host.grid, &host.cfg, ..)`); the run loops (`run.rs`) and the wall post-processing (`ogive.rs`, a copy of the private `bowshock_cases::ogive::postprocess`) are case-independent.
Same stopping criterion as `run_ogive` (6 orders from the peak after iteration 20), minmod, CFL 0.5, 300 first-order iterations, default precision (`Extended`), guarded formulation (`--direct` also runs the direct one; it agrees to the printed digits).
Output: `out/gpu_ogive/` (`cp_M1.98_<grid>.csv`: GPU/CPU wall Cp, `residual_*.csv`, `cp_compare_M1.98.csv`: orifice stations, `summary.csv`).

### Accuracy, GPU (extended) vs CPU (f64), `cargo run --release --bin ogive_gpu -- --direct`
| grid | cells | CD GPU | CD CPU | CD diff % | CD first-cell diff % | nose mean Cp diff % | max rel Cp diff, nose, abs(Cp) >= 0.05, % | max abs ΔCp (whole wall) | max Δp/p∞ | rms Δp/p∞ | max Δρ/ρ |
|---|---|---|---|---|---|---|---|---|---|---|---|
| coarse 114×50 | 5700 | 0.09565 | 0.09565 | −5.3e-6 | −7.1e-6 | −3.1e-6 | 8.6e-5 | 2.8e-7 | 7.34e-7 | 1.18e-7 | 7.21e-7 |
| coarse, direct mode | | 0.09565 | | −5.3e-6 | −7.1e-6 | −3.1e-6 | 8.6e-5 | 2.8e-7 | 7.34e-7 | 1.18e-7 | 7.21e-7 |
| medium 228×100 | 22800 | 0.09694 | 0.09694 | +1.1e-6 | +2.0e-6 | +5.9e-6 | 1.1e-4 | 3.8e-7 | 1.18e-6 | 1.57e-7 | 1.98e-6 |
| medium, direct mode | | 0.09694 | | +1.1e-6 | +2.0e-6 | +5.9e-6 | 1.1e-4 | 3.8e-7 | 1.18e-6 | 1.57e-7 | 1.98e-6 |
| fine 456×200 | 91200 | 0.09772 | 0.09772 | +2.2e-7 | +2.3e-7 | +8.1e-7 | 1.2e-4 | 3.4e-7 | 1.35e-6 | 1.67e-7 | 1.24e-6 |
| fine, direct mode | | 0.09772 | | +2.2e-7 | +2.3e-7 | +8.1e-7 | 1.2e-4 | 3.4e-7 | 1.35e-6 | 1.67e-7 | 1.24e-6 |

The forebody pressure drag CD (base-area reference, nose 0 ≤ x ≤ 3 d, extrapolated-wall Cp) and the averaged nose Cp agree with the CPU to **≤ 5.3e-6 %** (cone target: avg Cp diff ≪ 0.01 %; the cone cases give 2e-6 … 3e-5 %); the wall Cp differs by ≤ 3.8e-7 absolute everywhere
(≤ 1.2e-4 % relative where |Cp| ≥ 0.05; Cp passes through zero near x/d = 2.3 where a relative difference means nothing, so those faces are excluded from the relative column), the field by ≤ 1.4e-6 p∞. The CPU result itself against the experiment
(Perkins & Jorgensen, NACA RM A54H23, Table I(a)) and theory is the subject of the bowshock-core README (CD_forebody: experiment 0.1067 with tip extrapolated, shock-expansion 0.0625, tangent-cone 0.1158; CFD 0.0957 / 0.0969 / 0.0977).

### Cp at the 15 orifice stations: experiment, shock-expansion, CPU and GPU (the GPU reproduces the CPU to the 4 printed digits at every station and grid)
| x/d | Cp exp | shock-exp. | CPU coarse | GPU coarse | CPU medium | GPU medium | CPU fine | GPU fine |
|---|---|---|---|---|---|---|---|---|
| 0.22 | 0.260 | 0.252 | 0.2418 | 0.2418 | 0.2493 | 0.2493 | 0.2535 | 0.2535 |
| 0.56 | 0.212 | 0.187 | 0.1983 | 0.1983 | 0.2003 | 0.2003 | 0.2019 | 0.2019 |
| 0.89 | 0.165 | 0.130 | 0.1530 | 0.1530 | 0.1536 | 0.1536 | 0.1543 | 0.1543 |
| 1.22 | 0.115 | 0.078 | 0.1096 | 0.1096 | 0.1104 | 0.1104 | 0.1108 | 0.1108 |
| 1.61 | 0.074 | 0.023 | 0.0631 | 0.0631 | 0.0640 | 0.0640 | 0.0644 | 0.0644 |
| 2.06 | 0.026 | -0.034 | 0.0164 | 0.0164 | 0.0169 | 0.0169 | 0.0171 | 0.0171 |
| 2.50 | -0.014 | -0.083 | -0.0227 | -0.0227 | -0.0227 | -0.0227 | -0.0227 | -0.0227 |
| 2.95 | -0.038 | -0.126 | -0.0573 | -0.0573 | -0.0574 | -0.0574 | -0.0574 | -0.0574 |
| 3.62 | -0.030 | -0.131 | -0.0413 | -0.0413 | -0.0414 | -0.0414 | -0.0413 | -0.0413 |
| 4.06 | -0.017 | -0.131 | -0.0320 | -0.0320 | -0.0320 | -0.0320 | -0.0321 | -0.0321 |
| 4.50 | -0.012 | -0.131 | -0.0252 | -0.0252 | -0.0253 | -0.0253 | -0.0253 | -0.0253 |
| 5.39 | -0.009 | -0.131 | -0.0162 | -0.0162 | -0.0163 | -0.0163 | -0.0163 | -0.0163 |
| 6.28 | -0.001 | -0.131 | -0.0111 | -0.0111 | -0.0112 | -0.0112 | -0.0112 | -0.0112 |
| 7.17 | 0.000 | -0.131 | -0.0079 | -0.0079 | -0.0080 | -0.0080 | -0.0080 | -0.0080 |
| 8.06 | 0.006 | -0.131 | -0.0058 | -0.0058 | -0.0059 | -0.0059 | -0.0059 | -0.0059 |

### Convergence (stop = criterion of `run_ogive`)
| grid | CPU iters | CPU stop | CPU drop (orders) | CPU final res | GPU iters (stop/exec) | GPU stop | GPU drop (orders) | GPU final res |
|---|---|---|---|---|---|---|---|---|
| coarse 114×50 | 3505 | residual drop reached | 6.0 | 7.08e-7 | 3501/3550 | residual drop reached | 6.0 | 7.08e-7 |
| medium 228×100 | 4060 | residual drop reached | 6.0 | 1.44e-6 | 4073/4100 | residual drop reached | 6.0 | 1.44e-6 |
| fine 456×200 | 6452 | residual drop reached | 6.0 | 2.49e-6 | 6447/6450 | residual drop reached | 6.0 | 2.49e-6 |

### Timing (Luma main f29290fb, RTX 3090; `ogive_gpu --bench-only`: 1000 iterations from the free stream, best of 3, sync every 50, history kernels on; load average 7 – 17)
| grid | cells | CPU f64 rayon µs/iter | GPU-guarded extended µs/iter | GPU-direct extended | guarded f32 | guarded extended speed-up vs CPU | direct extended speed-up | extended / f32 cost | CPU s to stop | GPU-guarded s to stop | speed-up to stop |
|---|---|---|---|---|---|---|---|---|---|---|---|
| coarse 114×50 | 5700 | 636 | 76.3 | 85.8 | 23.6 | 8.3x | 7.4x | 3.24x | 2.23 | 0.27 | 8.3x |
| medium 228×100 | 22800 | 1299 | 84.2 | 97.0 | 27.5 | 15.4x | 13.4x | 3.06x | 5.27 | 0.34 | 15.5x |
| fine 456×200 | 91200 | 3302 | 252.7 | 241.3 | 71.9 | 13.1x | 13.7x | 3.52x | 21.30 | 1.63 | 13.1x |

"to stop" = iterations to the stopping criterion (CPU 3505 / 4060 / 6452, GPU 3501 / 4073 / 6447) × the steady cost. Measured wall times of the convergence runs (loop incl. history read-back, setup excluded) in the final full run, taken while the machine was at load average 35 – 47 (CPU inflated): CPU 3.45 / 8.48 / 32.45 s,
GPU-guarded 0.50 / 0.50 / 1.96 s. GPU setup (kernel build + upload) ≤ 18 ms, first-launch warm-up ≤ 18 ms.

## Transonic: large-domain grid + characteristic far field (`transonic_gpu`)

Case, grid and stopping rules are those of `bowshock_cases::transonic` (nose-cylinder body of the CPU `transonic` bin, default `cone`, nose fineness 3 + 6 d cylinder, domain 10 body lengths upstream / radially and 5 downstream, level 0 / 1 / 2 = 144×70 / 288×140 / 576×280 cells,
`Bc::Characteristic` on inflow plane, outer boundary and outflow plane, `SlipWall` on axis and body, minmod, CFL 0.5, 300 first-order iterations on a cold start; stop at 6 orders of residual drop or, from iteration 4000 on, when the CD (sampled every 50 iterations) varies by < 2e-4 over 2000 iterations after ≥ 2 orders).
**GPU side:** the far-field BCs (`Bc::Characteristic`, `Bc::PressureOutflow`) are ghost-state functions of the first interior cell and the outward normal (WGSL `characteristic_ghost`, `pressure_outflow_ghost`, f32 and df); the solver itself is unchanged.
`bowshock::gpu::GpuRunner::run_on` is the counterpart of `bowshock::run::run_on` (same arguments, cold start or warm start from the previous Mach number with the cold-start residual peak as reference; the free stream is switched with `GpuSolver::config_mut` without touching the state): it records 50 iterations,
reads the residual history and the state, evaluates the CD sample with `run::wall_distribution`, applies `run::stop_reason` per iteration and finishes with `run::finish`, i.e. the CPU's own stop rules and post-processing. Sweep: M = 0.7, 0.8, 0.85, 0.9, 0.95, 0.98, 1.0, 1.02, 1.05, 1.1, 1.2, 1.5, cold start for each (`--warm` for a warm-started sweep).
Output: `out/gpu_transonic/` (`sweep_<tag>.csv`, `cp_<tag>_M<m>.csv`, `residual_*.csv`, `cd_history_*.csv`, `steady_<tag>.csv`).

### Accuracy: forebody CD vs Mach, GPU (guarded, extended) vs CPU (f64) — cone-cylinder, level 1 (288×140)
| M | CD GPU | CD CPU | GPU − CPU | (GPU − CPU)/CPU % | Cp_min GPU | Cp_min CPU | max abs ΔCp (wall) | CPU iters | GPU iters (stop/exec) | stop rule | Stoney (digitised) | tangent-cone / T-M |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.7 | 0.00531 | 0.00531 | -1.0e-08 | -1.9e-04 | -0.7204 | -0.7204 | 2.2e-06 | 19500 | 19500/19500 | CD stationary | - | - |
| 0.8 | 0.01295 | 0.01295 | +1.9e-07 | +1.5e-03 | -0.8821 | -0.8821 | 1.5e-06 | 19000 | 19000/19000 | CD stationary | 0.020 | - |
| 0.85 | 0.02632 | 0.02632 | +1.7e-07 | +6.5e-04 | -0.8681 | -0.8681 | 7.5e-07 | 22000 | 22000/22000 | CD stationary | 0.025 | - |
| 0.9 | 0.04715 | 0.04715 | +2.7e-07 | +5.7e-04 | -0.7538 | -0.7538 | 8.6e-07 | 17500 | 17500/17500 | CD stationary | 0.030 | - |
| 0.95 | 0.07737 | 0.07737 | +1.7e-07 | +2.2e-04 | -0.6451 | -0.6451 | 5.1e-07 | 19000 | 19000/19000 | CD stationary | 0.060 | - |
| 0.98 | 0.10425 | 0.10425 | +1.8e-07 | +1.7e-04 | -0.5819 | -0.5819 | 5.9e-07 | 30000 | 30000/30000 | CD stationary | 0.090 | - |
| 1 | 0.13014 | 0.13014 | +1.4e-07 | +1.1e-04 | -0.5391 | -0.5391 | 3.0e-07 | 7000 | 7000/7000 | CD stationary | 0.110 | - |
| 1.02 | 0.15040 | 0.15040 | +1.3e-07 | +8.6e-05 | -0.5008 | -0.5008 | 3.5e-07 | 26000 | 26000/26000 | CD stationary | 0.130 | - |
| 1.05 | 0.16262 | 0.16262 | +1.6e-07 | +9.8e-05 | -0.4481 | -0.4481 | 4.0e-07 | 20000 | 20000/20000 | CD stationary | 0.160 | 0.20587 |
| 1.1 | 0.15503 | 0.15503 | +1.0e-07 | +6.5e-05 | -0.3746 | -0.3746 | 7.1e-07 | 11000 | 11000/11000 | CD stationary | 0.160 | 0.16197 |
| 1.2 | 0.13604 | 0.13604 | +4.0e-08 | +2.9e-05 | -0.2823 | -0.2823 | 1.6e-06 | 5500 | 5500/5500 | CD stationary | 0.140 | 0.13872 |
| 1.5 | 0.11231 | 0.11231 | +7.0e-08 | +6.2e-05 | -0.1664 | -0.1664 | 6.1e-07 | 4000 | 4000/4000 | CD stationary | - | 0.11315 |

Level 0 (144×70): max |CD_GPU − CD_CPU| = 6.0e-08 (relative 3.7e-04 %), max |ΔCp| over the wall 5.8e-07; level 1 (288×140): max |ΔCD| = 2.7e-07 (relative 1.5e-03 %, at M = 0.8 where CD = 0.013), max |ΔCp| 2.2e-06.
The GPU stops after the same number of iterations as the CPU at all 24 sweep points (both levels; the stop rule is evaluated on the GPU residual history, which agrees with the CPU's), by "CD stationary" except M = 1.5 on level 0 (6 orders reached). The Stoney column is the digitised free-flight data of
`bowshock_cases::transonic::stoney` (NACA RM L53K17, Fig. 5, ±0.01), shown for reference only: the comparison with experiment is the CPU README's; this crate only reproduces the CPU solution. Level 0 table: `out/gpu_transonic/sweep_cone_L0.csv`.

### Timing
Steady cost at M = 0.9 (`transonic_gpu --bench-only`, 1000 iterations from the free stream, best of 3, sync every 50, history kernels on; Luma main f29290fb, RTX 3090, load average 7 – 17):

| grid | cells | CPU f64 rayon µs/iter | GPU-guarded extended µs/iter | GPU-direct extended | guarded f32 | guarded extended speed-up vs CPU | direct extended speed-up | extended / f32 cost |
|---|---|---|---|---|---|---|---|---|
| L0 144×70 | 10080 | 940 | 89.5 | 103.9 | 26.2 | 10.5x | 9.1x | 3.41x |
| L1 288×140 | 40320 | 2121 | 145.1 | 152.9 | 33.6 | 14.6x | 13.9x | 4.32x |

Time to convergence of the whole 12-point sweep (iterations × steady cost; 127 078 iterations on level 0, 200 500 on level 1, same count on CPU and GPU up to the ≤ 49 extra GPU iterations of the last chunk):
level 0: CPU 119 s, GPU 11.4 s (10.5x); level 1: CPU 425 s, GPU 29 s (14.6x); per Mach number 0.3 – 1.7 s on the GPU (level 0), 0.6 – 4.4 s (level 1) vs 3 – 18 s / 8 – 64 s on the CPU (e.g. M = 0.98: 16 500 / 30 000 iterations).
Measured wall time of the sweeps themselves (the loop including the residual-history and state read-back for the CD sample every 50 iterations; taken while the machine was at load average 20 – 60, CPU side inflated up to ~2x, see Caveats): level 0: CPU 271 s, GPU 15.2 s (17.8x, 2129 / 120 µs per iteration);
level 1: CPU 639 s, GPU 41.0 s (15.6x, 3188 / 204 µs per iteration). The GPU loop's per-iteration cost is ≈ 1.3 – 1.4x the steady cost at both levels (state download every 50 iterations plus the load) [INFERENCE: not separated].

### Far-field BC validation (tests)
* `characteristic_far_field_preserves_the_free_stream` (M = 0.9 and 1.3, 40×20 box, 30 iterations from the uniform state): CPU deviation 2e-16; GPU 8.6e-8 / 6.6e-8 (extended), ≤ 4e-7 (f32): geometry rounding only, no growth.
* `characteristic_far_field_matches_cpu_on_a_perturbed_state`: Riemann, free-stream and extrapolation branches and the pressure outflow (M = 0.9, 1.3 × `Characteristic`/`PressureOutflow` outflow × both formulations × f32/extended): state ≤ 5.2e-8 (extended) / 1.1e-6 (f32).
* `transonic_state_after_20_iterations_matches_cpu`, `transonic_run_and_warm_start_match_cpu`: the real case at M = 0.9 / warm start 0.95: state ≤ 1.1e-7, CD ≤ 1.6e-7 relative, wall Cp ≤ 2.7e-7 (see "Tests").

## The f32 residual floor: cause, evidence, fix

**Problem (original port, `Precision::F32`).** The GPU density residual stalled 3.9 – 4.6 orders below its peak (floor 6e-5 … 5.3e-4), the floor
roughly doubled per grid refinement and scaled like 1/CFL, while the f64 CPU solver reaches the 6-orders criterion and keeps dropping (10.9 orders
after 8000 iterations on 100×50).

### Cause — confirmed by experiment

Two f32 effects, both of size ulp·|F|/Δx (fluxes are O(1), the residual is their cell-to-cell difference divided by the cell size):

1. **Quantisation of the state update.** At the floor `|dt·res|` is of the order of one ulp of `u`; updates below half an ulp round away and
   the iteration cannot move any more. Measured on the GPU state after 8000 iterations (`examples/f32_diag.rs`, 15°, minmod; ρ component,
   ρu and E alike):

   | grid | median of abs(dt·res) / ulp(u) | cells with abs(dt·res) < ½ ulp (update lost) |
   |---|---|---|
   | 100×50 | 0.39 | 56 % |
   | 200×100 | 0.30 | 63 % |
   | 400×200 | 0.36 | 59 % |

   This sets the floor to `res ≈ ulp(u)/dt`, and `dt ∝ CFL·Δx` — hence floor ∝ 1/Δx and ∝ 1/CFL (`f32_floor`, table at the end of this section).
2. **Rounding of the state the residual is evaluated on, and f32 arithmetic in the flux.** Even with the updates accumulated perfectly,
   evaluating the residual of a converged field on its f32-rounded copy gives a residual far above 1e-6 relative, because every cell value
   carries ±½ ulp of noise and the residual differentiates it. The flux arithmetic itself adds noise of the same size.

**Evidence 1 — CPU emulation (f64 arithmetic, only the state constrained; `examples/f32_cpu_emulation.rs`, 15°, minmod, 8000 iterations).**

| grid | f64 state (reference): orders below peak, min residual | state stored in f32 after every RK stage | f64 state, residual evaluated on `f32(u)` | GPU `F32` (geo-mean of its. 7000-8000) |
|---|---|---|---|---|
| 100×50 | 10.9, 3.0e-11 | **4.47**, 7.7e-5 (geo-mean 8.5e-5) | 5.08, 1.9e-5 (2.3e-5) | 4.45, 8.05e-5 |
| 200×100 | 7.0 (still converging), 3.6e-7 | **4.14**, 2.7e-4 (2.9e-4) | 4.89, 4.8e-5 (5.8e-5) | 4.27, 2.02e-4 |
| 400×200 | 6.6 (still converging), 1.5e-6 | **4.03**, 5.4e-4 (5.5e-4) | 4.72, 1.1e-4 (1.2e-4) | 4.14, 4.21e-4 |

Constraining only the state to f32 — with exact f64 flux arithmetic — reproduces the GPU floor to within 5 – 45 %; so the floor is caused by the f32
state, not by noise of the flux arithmetic alone. Removing the update quantisation (middle-right column: state kept in f64, only the evaluation sees
`f32(u)`) improves the floor by 3.6 – 5 × but leaves 4.7 – 5.1 orders: the evaluation-on-rounded-state noise (cause 2) remains.

**Evidence 2 — the GPU state itself (`examples/f32_diag.rs`, 8000 iterations, same state evaluated by the f64 CPU solver).**

| grid | precision | GPU residual, geo-mean its. 7000-8000 (orders below peak) | f64 CPU residual of the same state | rms(R_f32 − R_f64) of the GPU's own f32 evaluation |
|---|---|---|---|---|
| 100×50 | `F32` | 8.05e-5 (4.45) | 8.5e-5 | 1.9e-5 |
| 100×50 | `Compensated` | 3.39e-5 (4.83) | 3.1e-5 | 2.4e-5 |
| 100×50 | `Extended` | **6.6e-11 (10.5)** | 8.0e-6 (*) | 8.0e-6 (*) |
| 200×100 | `F32` | 2.02e-4 (4.27) | 2.0e-4 | 3.7e-5 |
| 200×100 | `Compensated` | 8.57e-5 (4.64) | 8.1e-5 | 4.4e-5 |
| 200×100 | `Extended` | **4.5e-7 (6.92)**, still falling | 1.6e-5 (*) | 1.6e-5 (*) |
| 400×200 | `F32` | 4.21e-4 (4.14) | 4.2e-4 | 7.7e-5 |
| 400×200 | `Compensated` | 1.64e-4 (4.55) | 1.6e-4 | 8.7e-5 |
| 400×200 | `Extended` | **1.55e-6 (6.57)**, still falling | 3.2e-5 (*) | 3.2e-5 (*) |

(*) The `Extended` GPU solver converges its *own* discrete operator, which has f32-rounded geometry (face normals, weights, cell volumes); the f64
CPU operator evaluated at that state differs from it by a fixed 8e-6 / 1.6e-5 / 3.2e-5 (∝ 1/Δx again: relative f32 geometry error 6e-8 × flux / Δx).
This is a perturbation of the discrete problem, not a convergence limit; its effect on the solution is ≤ 3e-6 in p/p∞ (table "Field difference").

The residual of the f32 states is the same whether evaluated in f32 or f64 (4th column ≈ 3rd column of the same row), i.e. the GPU is not "converged but measured
wrongly": the f32 state really is that far from a discrete steady state. The floor is spread over the shock layer (the wall-side 40 % of the rows carry 64 – 77 % of Σρ_res²; no single cell more than 1 %), it is not a
boundary or limiter hot spot.

### Fix — in the priority order of the assignment

* **(a) delta / perturbation form.** The RK stage is now computed as `u = u0 + a ((u − u0) + dt·res)` (valid because `a + b = 1`): only small
  numbers (increments) go through f32 arithmetic, so their relative precision is the full 24 bits instead of being swamped by the ulp of `u`. A
  fixed reference state (free stream or a per-cell reference) was *not* used: it cannot help cause 2, because the flux function needs the sum
  `u_ref + du` and that sum has to be rounded to f32 again (the residual of a smooth field is a difference of nearly equal fluxes; no f32 evaluation of
  `F(u_ref + du)` is better than ulp(u)·∂F/∂u).
* **(b) compensated accumulation.** The sum `u0 + increment` is a two-sum into a `(hi, lo)` pair of f32 arrays (`Precision::Compensated`). This removes
  cause 1 completely (update column of the table above: the lost increments are kept in `lo`) and improves the floor 2.4 – 2.6 × (4.45 → 4.83, 4.27 → 4.64, 4.14 → 4.55
  orders at 100×50 … 400×200 vs `F32`), but, as Evidence 1 predicts (5.1 / 4.9 / 4.7 orders at best), it cannot reach 6 orders: cause 2 is untouched.
* **(c) double-float (df) evaluation** (`Precision::Extended`, the default). Cause 2 can only be removed by evaluating the residual with more
  precision than f32, from the full `(hi, lo)` state. The "critical sums" are not enough: the noise enters through the primitive variables,
  the reconstruction and the HLLC arithmetic as much as through the flux sum, so the whole `face_flux` chain (cons→prim of the stencil, limiter,
  MUSCL, HLLC, Rusanov blend, shock sensor) and the cell sum + source term are evaluated in df (`shaders_ext.rs`, one-to-one with `flux.rs`). df numbers are `hi + lo` pairs of
  f32 (~48 bits), built from two-sum / two-product (with `fma`); division and square root use one Newton correction. Accuracy of the df
  face kernels against the f64 `bowshock-core` functions on 20 000 random stencils: **6e-13** relative (the f32 kernels: 1.2e-5), see the test
  `extended_face_kernels_match_cpu_to_1e9`.

### Result — before / after

Orders of magnitude the density residual drops below its peak (peak after iteration 20), all 12 cases, guarded formulation. `F32` / `Compensated` run until the plateau test (iteration 8001+); their floor is the geometric mean residual over the last 1000 iterations:

| cone | grid | limiter | CPU f64 | GPU `F32` (before) | GPU `Compensated` | GPU `Extended` (default) | `Extended` iterations (CPU iterations) |
|---|---|---|---|---|---|---|---|
| 10° | 100x50 | minmod | 6.0 | 4.1 (floor 1.2e-04) | 4.5 (floor 4.3e-05) | **6.0** | 2816 (2785) |
| 10° | 200x100 | minmod | 6.0 | 3.9 (floor 2.8e-04) | 4.4 (floor 1.0e-04) | **6.0** | 3765 (3918) |
| 10° | 400x200 | minmod | 6.0 | 3.9 (floor 5.3e-04) | 4.3 (floor 1.9e-04) | **6.0** | 5654 (5880) |
| 15° | 100x50 | minmod | 6.0 | 4.4 (floor 8.1e-05) | 4.8 (floor 3.5e-05) | **6.0** | 1900 (1900) |
| 15° | 200x100 | minmod | 6.0 | 4.3 (floor 2.0e-04) | 4.6 (floor 8.6e-05) | **6.0** | 3814 (3824) |
| 15° | 400x200 | minmod | 6.0 | 4.1 (floor 4.2e-04) | 4.6 (floor 1.6e-04) | **6.0** | 5099 (5185) |
| 10° | 100x50 | vanalbada | 6.0 | 4.3 (floor 6.3e-05) | 4.8 (floor 2.5e-05) | **6.0** | 1352 (1352) |
| 10° | 200x100 | vanalbada | 6.0 | 4.3 (floor 1.1e-04) | 4.7 (floor 4.7e-05) | **6.0** | 2240 (2240) |
| 10° | 400x200 | vanalbada | 6.0 | 4.2 (floor 2.3e-04) | 4.6 (floor 9.1e-05) | **6.0** | 4080 (4080) |
| 15° | 100x50 | vanalbada | 6.0 | 4.6 (floor 6.2e-05) | 5.0 (floor 2.5e-05) | **6.0** | 1518 (1518) |
| 15° | 200x100 | vanalbada | 6.0 | 4.6 (floor 9.9e-05) | 4.9 (floor 4.8e-05) | **6.0** | 2531 (2531) |
| 15° | 400x200 | vanalbada | 6.0 | 4.4 (floor 2.3e-04) | 4.8 (floor 9.4e-05) | **6.0** | 4654 (4654) |

All 12 cases reach the CPU's 6-orders criterion with the default precision, in the same number of iterations as the CPU (within ±5 %, the residual histories
agree: the GPU solves the same discrete problem to ~1e-6), in both formulations (`Guarded`, `Direct`). The 15° / minmod / 100×50 case continues to 10.5 orders (see Evidence 2). Accuracy, with the CPU as reference, improved
by 2 – 3 orders: avg Cp difference 2e-8 … 3e-7 (relative; before: 1.4e-5 … 1.1e-4) and field difference ≤ 2.8e-6 p/p∞ (before ≤ 2.2e-4); see the accuracy tables below.

**Performance cost** (steady state, 15°, minmod, best of 3, history kernels on, sync every 50, Luma main f29290fb): 100x50: 66.9 µs/iter extended vs 23.4 µs f32 (2.86x cost; CPU 596 µs → speed-up 8.9x instead of 25x); 200x100: 74.2 µs/iter extended vs 28.8 µs f32 (2.58x cost; CPU 1211 µs → speed-up 16.3x instead of 42x); 400x200: 235.8 µs/iter extended vs 60.0 µs f32 (3.93x cost; CPU 2960 µs → speed-up 12.6x instead of 49x) (compensated: 24.4, 30.5, 75.5 µs). With the much cheaper Luma launches (0.8 µs instead of 8.5 µs) the f32 solver became device-bound at all sizes and the df arithmetic of the extended precision, which used to hide behind the launch overhead on the small grids (1.4x cost on the old Luma), costs 2.6 – 3.9x. See the steady-state table below.

### Compiler pitfalls found on the way (important for anybody doing error-free arithmetic in WGSL/Luma)

The first df implementation was *not better than f32* (face-kernel error 8e-6): the Vulkan driver's compiler (NVIDIA RTX 3090) treats WGSL float
arithmetic as associative and contracts `a*b + c` into `fma`. Two-sum `e = (a - (s - bb)) + (b - bb)` with `bb = s - a` was simplified to
`0`, and `s + a*b`-style sums used the exact product while the error term used the rounded one (error counted twice). There is no `precise` / `NoContraction` in WGSL. Fix: every operand
and intermediate of an error-free transformation goes through `opq(x) = bitcast<f32>(bitcast<u32>(x) | p.zero)` where `p.zero` is a uniform
that is always 0 (`Params::zero`), which the compiler cannot see through (`shaders::common`, `shaders_ext::ts/qts/tp`; same for `df_add` of the
compensated update — whose first version had the same defect and gained only ~1.6× instead of 2.4×). The test
`extended_face_kernels_match_cpu_to_1e9` fails if a driver / compiler change breaks the transformations again. `fma` itself is honoured.
The accurate df add was replaced by the cheaper "sloppy" df add and division uses 2 instead of 3 correction steps (same 6e-13 accuracy, 20 – 30 % faster).

### f32 floor vs CFL (original `F32` precision; `cargo run --release -p bowshock-gpu-cases --example f32_floor`, GPU only, 15°, minmod, 6000 iterations)

The floor scales like 1/CFL, i.e. like 1/dt (update quantisation), noise in the flux evaluation alone would be CFL-independent:

| grid | CFL | peak residual | geo-mean residual, iterations 5000-6000 | floor x CFL |
|---|---|---|---|---|
| 100x50 | 0.125 | 4.04e0 | 4.72e-4 | 5.89e-5 |
| 100x50 | 0.25 | 3.09e0 | 2.12e-4 | 5.29e-5 |
| 100x50 | 0.5 | 2.29e0 | 8.03e-5 | 4.01e-5 |
| 100x50 | 1 | 1.65e0 | 5.66e-2 (not converging) | - |
| 200x100 | 0.125 | 6.14e0 | 3.57e-2 (not yet converged after 6000 it.) | - |
| 200x100 | 0.25 | 4.85e0 | 6.14e-4 | 1.53e-4 |
| 200x100 | 0.5 | 3.75e0 | 2.07e-4 | 1.03e-4 |
| 200x100 | 1 | 2.90e0 | 1.01e-1 (not converging) | - |

The product floor × CFL is roughly constant (4 – 6e-5 at 100×50, 1.0 – 1.5e-4 at 200×100) and the floor doubles with refinement, as the update-quantisation estimate `ulp(u)/dt` predicts
(`dt ∝ CFL·Δx`); together with Evidence 1 and 2 above this confirms the diagnosis (it was an inference before). With `Precision::Extended` the floor is gone: there is no CFL-dependent plateau, the residual keeps falling to 1e-10 (100×50).

## Tests (`cargo test --release -p bowshock-gpu-cases`, 10 tests, all pass, ~6 s)

1. `hllc_and_reconstruction_match_cpu_on_random_states`: 20 000 random states (smooth neighbours with 0.3 % / 3 % perturbations and unrelated
   states; random normals) through the f32 GPU `reconstruct` + `hllc` + `face_flux` kernels vs `bowshock_core::flux` on the same f32 inputs, first
   order / minmod / van Albada: max reconstructed-state error 2.2e-7, max HLLC flux error 1.2e-5, max `face_flux` error 1.2e-5 (relative to the
   largest flux component, floor 1), 12 529 of the samples inside the Rusanov blend range.
2. `extended_face_kernels_match_cpu_to_1e9`: the same samples through the df kernels (`eval_faces_ext`): max state error 1e-14, max HLLC flux error
   4.5e-13, max `face_flux` error 6.1e-13 (asserted < 1e-9). Also guards the compiler workaround (`opq`): if the driver reassociates / contracts the
   error-free transformations the error jumps back to ~1e-5.
3. `state_after_20_iterations_matches_cpu`: 40×20 cone grid, 20 iterations (first 5 first-order, which also exercises the per-launch limiter
   uniform), minmod and van Albada, `Direct` and `Guarded` x `F32`, `Compensated`, `Extended`: max |ΔU|/(1+|U|) ≤ 1.1e-6 (f32) / 2.3e-7 (compensated) /
   1.3e-7 (extended, asserted < 3e-7; the remainder is the f32 geometry), max relative pressure difference ≤ 1.6e-6 / 4.9e-7 / 1.8e-7, density-residual
   history ≤ 1.0e-6 / 2.4e-7 / 6.5e-8 relative; Luma's `norm()` of the stage-1 density residual equals the history kernels' value (≤ 1e-6).
4. `large_grid_residual_history_matches_cpu`: 400×200, 3 iterations (> 128 workgroups in the partial-sum reduction), both modes, default
   precision: residual ≤ 1.0e-7, state ≤ 1.1e-7.
5. `extended_precision_reaches_six_orders_on_a_small_grid` (regression test of the residual floor, ~2 s): 60×30 cone grid, 15°, minmod, guarded. `F32`
   stalls at 4.5 orders and `Compensated` at 4.9 orders within 4000 iterations (asserted < 5.5), `Extended` reaches 6.2 orders after 1250
   iterations (asserted ≥ 6) and its state agrees with the f64 CPU solution after the same number of iterations to 2e-7 (asserted < 1e-5).
6. `ogive_state_and_wall_cp_match_cpu`: ogive-cylinder, M = 1.98, coarse grid 114×50 (`bowshock_cases::ogive::OgiveCase::level(1.98, 0)`: free-stream inflow, extrapolation outflow, slip wall /
   axis, free-stream outer boundary; tip node at x = 0 and junction node on the grid): 20 iterations (5 first order), `Direct`/`Guarded` × `F32`/`Extended` vs CPU: max |ΔU|/(1+|U|) ≤ 1.2e-6 (f32) / 1.4e-7 (extended, asserted < 3e-7),
   residual history ≤ 6.0e-7 / 7.1e-8 (asserted < 2e-7); then 500 iterations: forebody CD relative difference −5.9e-8, max |ΔCp| over the whole wall 1.0e-7 (asserted < 1e-5 each).
7. `characteristic_far_field_preserves_the_free_stream`: 40×20 axisymmetric box (stretched grid, axis `jmin`, characteristic BC on imin / imax / jmax), 30 iterations from the uniform free stream at M = 0.9 (all far-field faces subsonic)
   and M = 1.3: the CPU keeps it to 2e-16; the GPU (both formulations, `F32` and `Extended`) keeps it to 8.6e-8 / 6.6e-8 (extended) and 4.0e-7 / 9.4e-8 (f32) — the f32 geometry rounding, no growth, no spurious waves from the BC.
8. `characteristic_far_field_matches_cpu_on_a_perturbed_state`: the same box with a smooth density / pressure / transverse-velocity perturbation of the free stream (so that the Riemann, free-stream and extrapolation branches and the pressure outflow
   are all hit), 20 iterations, M = 0.9 and 1.3 × outflow `Characteristic` and `PressureOutflow` × `Direct`/`Guarded` × `F32`/`Extended`: state ≤ 5.2e-8 (extended) / 1.1e-6 (f32), residual history ≤ 6.3e-8 / 8.9e-7 relative.
9. `transonic_state_after_20_iterations_matches_cpu`: the transonic case (`transonic_case(0.9, 0, ..)`, ogive-cylinder in the large domain, 144×70, characteristic far field), 20 iterations, both formulations, f32 / extended:
   state ≤ 1.1e-7 (extended, asserted < 3e-7) / 8.0e-7 (f32), residual history ≤ 1.7e-7 (asserted < 5e-7) / 8.6e-7.
10. `transonic_run_and_warm_start_match_cpu`: `GpuRunner::run_on` vs `bowshock::run::run_on` with the same arguments, 200 iterations at M = 0.9 (cold start) and then 200 iterations at M = 0.95 warm-started from that field
    (`r_ref` = cold-start peak): residual history ≤ 3.2e-7 / 2.4e-6 relative, forebody CD relative difference −1.2e-7 / +1.6e-7, max |ΔCp| ≤ 2.7e-7, CD-history samples ≤ 6.5e-8 (asserted < 1e-4 / 1e-5 / 1e-5 / 1e-6).
11. `viscous_laminar_plate_state_matches_cpu`: the laminar flat plate of the CPU gate G1 (annulus formulation, Mach 0.2, `re_c = 1e6`, `Scheme::Implicit(ImplicitParams::VISCOUS)`, van Albada, low Mach, 5 first-order iterations), 30 iterations from the free stream: max |ΔU|/(1+|U|) 6.4e-6 (f32) / 1.4e-7 (extended)
    (asserted ≤ 1e-3). One iteration from a developed state, kernel by kernel (checked while porting): cell gradients 1.6e-6 of the largest value, `mu` 4e-7, `dt` 3e-7, state after the iteration 3e-6 (f32) / 2e-8 (extended).
12. `viscous_sst_plate_state_matches_cpu`: the SST plate of G2 (`re_c = 2.5e7`, `dr_wall = 4e-6`), 50 iterations: mean |ΔU|/(1+|U|) 1.3e-6 (f32) / 2.0e-8 (extended) (asserted ≤ 1e-3); `k`, `omega` on the 2012 cells with `k > 10 k_inf`: max relative difference 1.6e-3, 4.0e-4 (f32) / 1.2e-5, 4.3e-6 (extended) (asserted ≤ 1e-2).
    One iteration from a developed state: `k` 3e-6, `omega` 3e-6, `mu_t` 4e-6 of the largest value (f32), i.e. the turbulence coefficients, the two scalar line solves and the clipped update are at f32 round-off.
13. `viscous_body_total_drag_matches_cpu` (`#[ignore]`, ~30 s, `-- --ignored`): the G3 body (tangent ogive 0.30 x 0.10 m + 0.60 m tube, ISA 0 m, Mach 0.5, coarse grid, flat base, SST) run by the CPU to its own stop (CD stationary after 4000 iterations) and by the GPU (extended) for the same number of
    iterations: total CD 0.21300 on both (+0.000 %), `cd_friction` 0.09003, `cd_base` 0.12257, `cp_base` -0.1226, `y+_max` 0.648 (asserted within 1 %).

## Luma friction / feedback

Everything worked without a Luma bug: no wrong result, no panic, poisoned arrays never triggered, argument/binding checking at launch was helpful.
Observations, most important first (the first two decided the structure of the solver; item 9 was found while fixing the f32 floor):

1. **Host cost per launch (older Luma): ≈ 8.2 – 8.6 µs, independent of problem size** (4000 dependent trivial launches, 64 … 80 000 invocations; record+sync 9 µs). `Context::launch` created a new compute pass per dispatch, allocated a `Vec` of bindings,
   hashed a `BindKey` into a `HashMap` and took fresh arena slots for every uniform. With 17 launches per iteration the direct solver was host-bound at every grid size (≈ 150 µs/iter); only fusing kernels (7 launches) helped.
   **On the current main (commit 8bc26d3 "Cut launch overhead ...") the same measurement gives 0.75 – 0.93 µs per launch recorded and 1.2 – 2.1 µs record+sync (table above)**: the guarded solver records an iteration (7 launches) in ≈ 9 µs and the direct one (17 launches) in ≈ 24 µs, so
   neither is host-bound any more (steady costs 23 – 60 µs / 44 – 101 µs in f32). The df arithmetic of the extended precision now shows: 2.6 – 3.9 × the f32 cost at the same launch count.
2. **No public non-blocking submit; batches go out only every 256 dispatches or at a sync** (older Luma). `Context::flush` was `pub(crate)`; with 7 launches × K iterations the GPU received nothing until 256 dispatches were recorded
   [INFERENCE from the sync-cost numbers: ≈ 1 ms per sync at K = 50 then]. Luma commit 8bc26d3 ("add flush, profiling, vector reductions, scoped reflection") makes `Context::flush` public (verified in `context.rs`); `bowshock-gpu` does not use it yet.
   On main the measured sync cost is ≈ 0.3 ms at K = 50 ((24.9 − 18.6) µs × 50, guarded f32, 100×50) and ≈ 0.07 ms at K = 1. A non-blocking read-back (`to_vec_async` / a future) would still let the history of chunk n be fetched while chunk n+1 runs.
3. **Binding reflection covers all module-level globals, not those used by the entry point** (`build_core` iterates `module.global_variables`;
   verified: a module with `fn k1` using only `b` still reports bindings `[a, b]` for `k1`). A shared WGSL module with several entry points needing different
   bindings is therefore impossible; I build one source (COMMON + bindings + `main`) per kernel, i.e. compile the shared functions 8 times
   (cheap here: all pipelines + uploads take ≤ 20 ms also with the df sources, but it makes a "library of kernels" awkward). Per-entry-point reflection (naga
   already knows which globals an entry point uses) would fix this.
4. **No custom reduction / vec4 math.** The built-in ops and `sum/max/norm` exist only for `Array<f32|u32>`; `Array<[f32;4]>` can only be created, read
   back and bound. I needed (a) a scalar `rho_res` array written by my residual kernel only so that Luma's `norm()` can be used,
   and (b) my own history kernels (partial sums + final) to get the residual of *every* iteration without a sync per iteration
   (`norm()` synchronises and reads back on every call, and allocates scratch through the pool). A `reduce_into(&mut Array<f32>, slot)`-style
   non-blocking reduction (result stays on the GPU) would make the history kernels unnecessary.
5. **Uniforms work very well**: per-launch values (limiter code, RK stage and coefficients, history slot) in an encase struct are copied at call
   time and every launch of a batch keeps its own; this is what lets 3 RK stages × K iterations be recorded in one batch with changing parameters.
   Limitation encountered: nothing in Luma helps with `array<vec4<f32>>`/`[f32;4]` fields in uniform structs (I used scalar fields only; not tested).
6. **Storage-binding count**: the fused guarded residual kernel binds 10 storage buffers + 1 uniform (12 + 1 with the compensated / extended state). Luma requests `adapter.limits()` so this works on
   the 3090 (a 30-binding kernel also built), but it exceeds WebGPU's default of 8 per stage (portability warning; worth documenting).
7. **No GPU timing facility** (timestamp queries / per-kernel time). I had to infer host-bound vs device-bound from record-only vs record+sync timings
   and trivial-kernel launches. A `Context::profile()` or per-launch timestamp option would make this much easier.
8. Minor: `ProblemSize` only for `[u32; N]` (an integer literal needs to infer to `u32`); `Array::write` needs `&mut` even though it only records an upload
   (fine); `launch` takes `Vec<KernelArg>` (a small allocation per launch; part of item 1).

9. **No control over float semantics (`precise` / `NoContraction`), no f64.** Error-free transformations (two-sum, two-product) needed for double-float
   arithmetic are silently destroyed by the driver compiler (reassociation of `(a + b) - a`, contraction of `a*b + c` into `fma`); WGSL has no way to forbid
   it and naga does not emit `NoContraction`. I had to route values through a runtime-zero integer OR (`opq`, see "Compiler pitfalls"). A WGSL attribute
   (or a Luma kernel option) that makes arithmetic strict, and/or `f64` arrays (`wgpu::Features::SHADER_F64` where the adapter has it; not tried), would
   make extended precision much simpler. Also worth documenting: Luma users who need reproducible float summation face the same compiler behaviour.

WGSL/naga pitfalls met (not Luma): `1e-300` is not an f32 literal (replaced with `1e-30`); a user function named `reflect` would shadow a
builtin (renamed `reflect_wall`); `workgroupBarrier()` must be in uniform control flow, so the fused residual kernel has no early `return` before the reduction.

## Caveats

* Timings were taken on a shared desktop (browser, compositor, language server) and, in part, while the bowshock-core side was running its own heavy CPU jobs (load average 20 – 60 on 32 threads). The steady benchmarks (best of 3,
  `cone_gpu --bench`, `ogive_gpu --bench-only`, `transonic_gpu --bench-only`) were run at load average 7 – 17 and are the reference; "time to the stopping criterion" tables multiply iteration counts by these steady costs. Per-run wall times
  (cone "Wall time of the convergence runs themselves", the sweep timings of `transonic_gpu`) were taken under the heavier load: the CPU side is inflated by up to ~2x there (e.g. 4 ms/iter vs 2.1 ms/iter on the 288×140 transonic grid at load 7 – 17),
  the GPU side by 1.2 – 1.5x.
* All timing tables were regenerated on Luma main f29290fb (Cargo.lock; the previous tables were taken on an older Luma with ≈ 8.5 µs of host time per launch, see "Luma friction"). The accuracy and convergence tables of the cone cases are unchanged to the printed digits (checked by re-running `cone_gpu`).
* The NVIDIA driver printed `NVVM compilation failed: 3` once, in a combined bench session (`cone_gpu --bench`, `ogive_gpu --bench-only`, `transonic_gpu --bench-only` back to back); I could not attribute it to one binary and it did not reproduce when `ogive_gpu --bench-only --quick` and `cone_gpu --bench --quick --no-cpu` were re-run alone. All results of that session were correct (tests and tables agree with the CPU).
* The CPU run in `cone.rs` repeats `run_cone`'s loop and stopping logic using the public `Solver::step` (so the final CPU state is available) and copies the
  private `postprocess` of `bowshock_cases::cone`; it reproduces bowshock-core's reference iteration counts exactly. If bowshock-core's criteria change, `Monitor` must follow.
* GPU runs execute up to K−1 = 49 iterations past the iteration where the criterion fired (`exec` vs `stop` in the tables); the final state is the one after `exec`.
* `Precision::Extended` depends on the driver honouring the opaque-identity workaround (`opq`, see "Compiler pitfalls"); it was verified on the RTX 3090 / Vulkan only, and
  `extended_face_kernels_match_cpu_to_1e9` is the canary on other devices or driver versions. It also needs `fma` to be a true fused multiply-add (two-product).
* The extended GPU operator uses f32 geometry (face normals, weights, cell area/volume: converted from the f64 grid once), so it differs from the CPU operator by ≈ 6e-8 relative; the
  GPU converges its own discrete problem. Before M2 this showed as 1e-6-level field differences and residuals of the CPU operator evaluated on the GPU state of 8e-6 … 3e-5 (see "The f32 residual floor") and, on the large-domain subsonic grids, as a residual floor of 1.5e-8 that
  stopped the GPU at 5.4 – 5.7 of 6 orders. M2 adds the first-order correction of the geometry's closure defect ("Geometry closure correction"): it removes the residual floor of a uniform flow (floor 1.7e-10 = the CPU's), the iteration counts of the GPU now equal the CPU's, and every
  extended-precision GPU–CPU comparison improved 2 – 10x (cone / ogive / transonic tests and runs). What remains is the second-order effect of the f32 geometry on non-uniform flow (≈ 1e-7). Uploading the geometry as hi/lo pairs would remove it as well.
* The GPU solver mirrors bowshock-core as of its SSP-RK3 + HLLC/Rusanov-blend version (`flux::face_flux`, `Solver::step`/`stage`) plus the M2 additions (`flux::face_flux_lm`, `Solver::block`, `Scheme::Implicit`); BCs, limiters, grid and
  first-order start-up are as before. Multigrid and implicit residual smoothing (`Scheme::Multigrid`, `Scheme::Smoothed`) are not ported (`GpuSolver::set_scheme` returns an error): they measured worse than the line-implicit scheme on the CPU.
* The implicit scheme exists in the guarded formulation only (`set_scheme` / `set_block` return an error for `Mode::Direct`). Its left-hand side is assembled in f32 and the 4x4 blocks are inverted without pivoting (the pivot blocks `I - dt J` are diagonally dominant); the CPU uses f64 and pivoted
  Gauss-Jordan. Both converge to the same fixed point; the iterates differ at the 1e-7 level and a nonlinear transient (shock formation, base-flow unsteadiness) amplifies that, so the GPU / CPU residual histories of the *unsteady* base-flow runs and of long transonic transients agree only until
  the residual has dropped 3 – 4 orders (tests compare the short transients and the converged states).
* `ogive.rs` / `run.rs`: the CPU runs use the public `Solver::step` / `bowshock::run::run_on` stopping rules (`run::Monitor` = `run_ogive`'s criteria; the transonic GPU loop calls
  `bowshock::run::stop_reason`, the very function `run_on` uses, and `finish` for the post-processing, so both sides stop and post-process identically); the ogive wall post-processing is a copy of the private
  `bowshock_cases::ogive::postprocess`. The transonic GPU loop samples the forebody CD every 50 iterations and therefore synchronises every `bowshock::gpu::SYNC` = 50 iterations (the CPU's sampling interval).
* The characteristic far-field ghost state is evaluated with the integer power `rho = rho_ref (c_b / c_ref)^5` instead of the CPU's `powf` form (identical mathematically for gamma = 1.4,
  asserted when the shaders are generated: `shaders::characteristic_exponent`); the difference is ~1e-16 relative in the df evaluation.
* **Viscous terms and SST** exist for `Mode::Guarded` + `Scheme::Implicit` only (anything else returns `GpuError::Unsupported` from `set_viscous` / `set_scheme`). Only the mean-flow inviscid residual keeps its double-float evaluation; the viscous fluxes, the gradients, the turbulence equations
  and the implicit left-hand side are plain f32 on the high part of the state. The one-iteration energy residual of the thinnest wall cells therefore differs from the CPU's by up to 1.3e-4 (laminar plate, `dr = 2e-4`) / 8e-3 (SST plate, `dr = 4e-6`) of its largest value: the temperature `T = gamma p / rho` of the f32 state carries ~3e-7 absolute noise,
  which the heat-conduction term turns into a flux error of about `kappa dT/dr` with `dT/dr` ~ noise / `dr` and divides by the cell height again (an estimate of 2e-3 for the worst SST cell, 2.2e-3 measured). The momentum and continuity residuals, the gradients, the viscosity, `dt`, `k`, `omega` and `mu_t` agree to f32 round-off, and converged fields and drag agree to the printed digits (tests 11 - 13).
