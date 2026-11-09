# bowshock-gpu — f32-storage Luma/WGSL port of bowshock-cfd

GPU port of the CPU reference solver (`../bowshock-cfd`: 2D axisymmetric Euler, structured body-fitted grid,
cell-centred FV, MUSCL (minmod / van Albada) + HLLC blended with Rusanov in shocks (`flux::face_flux`),
SSP-RK3 with local time stepping, CFL 0.5) on [Luma](../../luma) custom kernels. The state lives in **f32 arrays**; how it is
accumulated and how the residual is evaluated is selectable (`Precision`: `F32` = the original plain-f32 port, `Compensated`,
`Extended` = default, see "The f32 residual floor"). With the default the GPU solver converges like the f64 CPU solver (6 orders on
all 12 validation cases). The CPU solver (f64, rayon) is the reference at run time: `cone_gpu` runs both and compares them.

```
bowshock-gpu/
  src/shaders.rs        WGSL kernels (COMMON = translation of bowshock-cfd gas.rs/flux.rs; one source per kernel), f32 / compensated update
  src/shaders_ext.rs    double-float (df) WGSL: error-free arithmetic + the df translation of the face kernels and the residual
  src/lib.rs            GpuSolver (Mode x Precision): uploads geometry once, records the kernels, residual history, state read-back
  src/cone.rs           cone runners (CPU loop, GPU loop with identical stopping criteria), post-processing, comparison metrics
  src/bin/cone_gpu.rs   validation + benchmark driver (writes out/gpu/*.csv)
  tests/gpu_vs_cpu.rs   cargo tests (face kernels f32 + df on random states, 20-iteration state comparison for every mode x precision,
                        400x200 residual history, small-grid regression test of the residual floor)
  examples/f32_cpu_emulation.rs  CPU experiment: f64 arithmetic with the state constrained to f32 (floor diagnosis)
  examples/f32_diag.rs           GPU experiment: update size vs ulp, f64 vs f32 evaluation of the GPU state, where the residual lives
  examples/f32_floor.rs          GPU experiment: f32 floor vs CFL
```

Run: `cargo test --release -p bowshock-gpu`, `cargo run --release --bin cone_gpu` (everything: 12 cases x {CPU, GPU-guarded extended, GPU-direct
extended, GPU-guarded f32, GPU-guarded compensated}, steady benchmark, Luma launch-overhead + sync-interval studies; ~4 min; `--quick`
skips 400x200, `--limiter minmod|vanalbada`, `--chunk K`, `--bench [--no-cpu]` only the timing studies).

## Design

* **State** `Array<[f32;4]>` (`vec4<f32>`) per cell: conserved `u`, `u0` (RK start), primitives `w` (with 2 ghost layers,
  direct mode only), face fluxes `fi`/`fj`, residual `res`, plus `Array<f32>` `dt`, `rho_res` (stage-1 `d rho/dt`). The compensated and
  extended precisions add the low words `ulo`, `u0lo` (same layout), and (extended) the fluxes / primitives are stored as `(hi, lo)` pairs of vec4.
  Geometry is uploaded once from the bowshock-cfd grid (`bowshock_cfd::cone::build_solver(...).grid`): `iface`/`jface` =
  `(nx, nr, len, len*r_mid)` per face (weight computed in f64, then rounded), `cell` = `(A, V, x_c, r_c)`.
* **Uniform block** (`Params`, encase struct of scalars, 96 B) is passed with every launch (Luma copies it into its own
  arena slot per launch): sizes, limiter code (0 first order during the 300-iteration start-up, 1 minmod, 2 van Albada),
  RK stage and stage coefficients, CFL, free stream (f32 + its low words), BC codes, residual-history slot, and a runtime `zero` (see
  "compiler pitfalls"). So iteration-dependent parameters need no extra upload and everything stays in one recorded batch.
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
  from `bowshock_cfd::flux::SHOCK_SENSOR_LO/HI` when the WGSL source is generated (the df code has them, `GAMMA`, `GAMMA-1`, `1/(GAMMA-1)` and
  the van Albada eps as f64-rounded hi/lo constants).

## Kernel list

| kernel | problem size | mirrors (bowshock-cfd) | mode / precision |
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
| `face_test`, `face_test_ext` | `[n]` | test only: reconstruct / HLLC / `face_flux` on arbitrary states (f32 / df) | tests |

WGSL functions (COMMON): `cons_to_prim`, `sound_speed`, `physical_flux`, `reflect_wall`, `ghost_state`, `limited_slope`
(first order / minmod / van Albada), `reconstruct` (with the positivity fallback), `hllc`, `rusanov`, `shock_weight`, `face_flux`; `df_add` (compensated
state update). `shaders_ext`: df arithmetic `ts/qts/tp/dadd/dsub/dmul/dmulf/ddiv/dsqrt/dmin/dmax/dabs` and the df versions `dcons_to_prim`, `dslope`,
`dreconstruct`, `dhllc`, `drusanov`, `dshock_weight`, `dface_flux`.

## Results

All numbers below are produced by `cargo run --release --bin cone_gpu` (RTX 3090 via Luma/wgpu/Vulkan; CPU = bowshock-cfd, f64,
rayon, 32 threads; M = 2, CFL 0.5, 300 first-order start-up iterations, convergence criteria of `run_cone`; GPU = default `Precision::Extended` unless a column says
f32 / compensated). Output CSVs: `out/gpu/` (`cp_<angle>deg_<ni>x<nj>_<limiter>.csv`: GPU/CPU/Taylor-Maccoll Cp per station incl. relative difference;
`residual_*.csv`: GPU and CPU density residual per iteration; `summary.csv`: one row per case incl. the f32 / compensated results).
The CPU side reproduces bowshock-cfd's own reference table exactly (iteration counts 2785/3918/5880 and 1900/3824/5185 for minmod 10°/15°,
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
* **Performance** (steady state, best of 3, sync every K = 50 iterations, per-iteration residual history included): CPU 601 / 1206 / 2930 µs, GPU-guarded **128.6 / 130.8 / 262.1 µs**
  (4.7x / 9.2x / 11.2x faster), GPU-direct 238.6 / 232.3 / 375.6 µs (2.5x / 5.2x / 7.8x) for 100×50 / 200×100 / 400×200.
  The extended precision costs **1.43x / 1.40x / 1.88x** of the f32 guarded solver (89.8 / 93.5 / 139.7 µs; which was 6.7× / 12.9× / 21.0× faster than the CPU). Time to each solver's own stop (6 orders for both now):
  0.17 – 1.48 s on the GPU vs 0.81 – 17.23 s on the CPU, i.e. 4.6× – 11.6× in favour of the GPU-guarded solver (2.5× – 8.1× direct). Before, the GPU ran ≥ 8001 iterations until its plateau test fired
  without reaching the criterion. Setup (kernel build + upload): ≤ 12 ms, first-launch warm-up ≤ 14 ms, context creation 0.2 s (the first time the GPU is touched).
* **Small grids are Luma-host-bound**: 100×50 and 200×100 cost almost the same (128.6 / 130.8 µs) because a launch costs ≈ 8.5 µs of host time regardless of its size (see "Luma friction");
  the df arithmetic is hidden behind it there (1.4×) and shows up at 400×200 (1.9×). Fusing 17 launches/iteration (direct) into 7 (guarded) is a 1.5 – 1.9× gain at all sizes.

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

### Steady per-iteration cost (15 deg, minmod, 2000 iterations from the free stream, best of 3, sync every K=50)
| grid | cells | CPU f64 rayon us/iter | GPU-guarded extended us/iter | guarded compensated | guarded f32 | GPU-direct extended | direct f32 | guarded extended speed-up vs CPU | direct extended speed-up | extended / f32 cost (guarded) |
|---|---|---|---|---|---|---|---|---|---|---|
| 100x50 | 5000 | 601 | 128.6 | 84.8 | 89.8 | 238.6 | 198.7 | 4.7x | 2.5x | 1.43x |
| 200x100 | 20000 | 1206 | 130.8 | 88.9 | 93.5 | 232.3 | 201.6 | 9.2x | 5.2x | 1.40x |
| 400x200 | 80000 | 2930 | 262.1 | 154.8 | 139.7 | 375.6 | 232.8 | 11.2x | 7.8x | 1.88x |

### Total time to the stopping criterion of each solver (iterations to stop x steady us/iter; GPU extended and CPU both stop at 6 orders; the f32 solver (before) ends at its plateau test)
| cone | grid | limiter | CPU iters | CPU s | GPU-guarded ext iters (stop) | GPU-guarded ext s | GPU-direct ext s | guarded speed-up | direct speed-up | f32 guarded iters (stop) | f32 guarded s | f32 orders reached |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 2785 | 1.67 | 2816 | 0.36 | 0.67 | 4.6x | 2.5x | 8001 | 0.72 | 4.1 |
| 10 deg | 200x100 | minmod | 3918 | 4.72 | 3765 | 0.49 | 0.87 | 9.6x | 5.4x | 8001 | 0.75 | 3.9 |
| 10 deg | 400x200 | minmod | 5880 | 17.23 | 5654 | 1.48 | 2.12 | 11.6x | 8.1x | 8001 | 1.12 | 3.9 |
| 15 deg | 100x50 | minmod | 1900 | 1.14 | 1900 | 0.24 | 0.45 | 4.7x | 2.5x | 8001 | 0.72 | 4.4 |
| 15 deg | 200x100 | minmod | 3824 | 4.61 | 3814 | 0.50 | 0.89 | 9.2x | 5.2x | 8001 | 0.75 | 4.3 |
| 15 deg | 400x200 | minmod | 5185 | 15.19 | 5099 | 1.34 | 1.91 | 11.4x | 7.9x | 9001 | 1.26 | 4.1 |
| 10 deg | 100x50 | vanalbada | 1352 | 0.81 | 1352 | 0.17 | 0.32 | 4.7x | 2.5x | 8001 | 0.72 | 4.3 |
| 10 deg | 200x100 | vanalbada | 2240 | 2.70 | 2240 | 0.29 | 0.52 | 9.2x | 5.2x | 8001 | 0.75 | 4.3 |
| 10 deg | 400x200 | vanalbada | 4080 | 11.96 | 4080 | 1.07 | 1.53 | 11.2x | 7.8x | 8001 | 1.12 | 4.2 |
| 15 deg | 100x50 | vanalbada | 1518 | 0.91 | 1518 | 0.20 | 0.36 | 4.7x | 2.5x | 8001 | 0.72 | 4.6 |
| 15 deg | 200x100 | vanalbada | 2531 | 3.05 | 2531 | 0.33 | 0.59 | 9.2x | 5.2x | 8001 | 0.75 | 4.6 |
| 15 deg | 400x200 | vanalbada | 4654 | 13.64 | 4654 | 1.22 | 1.75 | 11.2x | 7.8x | 9001 | 1.26 | 4.4 |

### One-time GPU setup (kernel build + geometry/state upload) and first-launch warm-up, seconds (context creation 0.24 s); extended precision
| cone | grid | limiter | guarded setup | guarded warm-up | direct setup | direct warm-up |
|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 0.012 | 0.001 | 0.020 | 0.001 |
| 10 deg | 200x100 | minmod | 0.003 | 0.003 | 0.001 | 0.001 |
| 10 deg | 400x200 | minmod | 0.010 | 0.014 | 0.003 | 0.002 |
| 15 deg | 100x50 | minmod | 0.000 | 0.003 | 0.000 | 0.001 |
| 15 deg | 200x100 | minmod | 0.002 | 0.003 | 0.000 | 0.001 |
| 15 deg | 400x200 | minmod | 0.008 | 0.011 | 0.002 | 0.002 |
| 10 deg | 100x50 | vanalbada | 0.000 | 0.001 | 0.000 | 0.001 |
| 10 deg | 200x100 | vanalbada | 0.000 | 0.002 | 0.000 | 0.001 |
| 10 deg | 400x200 | vanalbada | 0.007 | 0.011 | 0.003 | 0.002 |
| 15 deg | 100x50 | vanalbada | 0.000 | 0.001 | 0.000 | 0.001 |
| 15 deg | 200x100 | vanalbada | 0.002 | 0.003 | 0.000 | 0.001 |
| 15 deg | 400x200 | vanalbada | 0.008 | 0.010 | 0.002 | 0.001 |

### Wall time of the convergence runs themselves (iteration loop incl. history readback every K=50; setup excluded; shared desktop, noisy: see the steady benchmark above)
| cone | grid | limiter | CPU iters | CPU us/iter | GPU-guarded ext us/iter | GPU-direct ext us/iter | GPU iters exec (stop) | CPU total s | GPU-guarded ext total s | GPU-direct ext total s | f32 guarded total s (8000+ its) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 deg | 100x50 | minmod | 2785 | 723 | 117 | 189 | 2850 (2816) | 2.01 | 0.33 | 0.54 | 0.53 |
| 10 deg | 200x100 | minmod | 3918 | 1215 | 233 | 255 | 3800 (3765) | 4.76 | 0.89 | 0.97 | 0.87 |
| 10 deg | 400x200 | minmod | 5880 | 3032 | 423 | 409 | 5700 (5654) | 17.83 | 2.41 | 2.33 | 1.17 |
| 15 deg | 100x50 | minmod | 1900 | 594 | 128 | 260 | 1900 (1900) | 1.13 | 0.24 | 0.49 | 1.10 |
| 15 deg | 200x100 | minmod | 3824 | 2023 | 230 | 263 | 3850 (3814) | 7.74 | 0.88 | 1.01 | 1.15 |
| 15 deg | 400x200 | minmod | 5185 | 6058 | 409 | 410 | 5100 (5099) | 31.41 | 2.09 | 2.09 | 1.27 |
| 10 deg | 100x50 | vanalbada | 1352 | 594 | 132 | 211 | 1400 (1352) | 0.80 | 0.18 | 0.30 | 0.68 |
| 10 deg | 200x100 | vanalbada | 2240 | 1225 | 244 | 225 | 2250 (2240) | 2.74 | 0.55 | 0.51 | 0.72 |
| 10 deg | 400x200 | vanalbada | 4080 | 3177 | 409 | 511 | 4100 (4080) | 12.96 | 1.68 | 2.09 | 1.66 |
| 15 deg | 100x50 | vanalbada | 1518 | 989 | 191 | 265 | 1550 (1518) | 1.50 | 0.30 | 0.41 | 0.97 |
| 15 deg | 200x100 | vanalbada | 2531 | 2247 | 281 | 368 | 2550 (2531) | 5.69 | 0.72 | 0.94 | 1.28 |
| 15 deg | 400x200 | vanalbada | 4654 | 3355 | 383 | 448 | 4700 (4654) | 15.62 | 1.80 | 2.11 | 1.26 |

### Luma launch overhead: 4000 dependent launches of `a[i] += 1` (one storage binding, luma grid uniform only)
| invocations | host record us/launch | record+sync us/launch |
|---|---|---|
| 64 | 8.59 | 9.19 |
| 5000 | 8.12 | 8.76 |
| 80000 | 8.23 | 8.95 |
| 1000000 | 12.20 | 16.81 |

### GPU per-iteration timing study (15 deg, minmod, from free stream, 1000 iterations, f32 precision; first-order phase off)
Residual history = per-iteration density residual (history kernels), read back every K iterations.
| mode | grid | hist, sync K=1 | K=10 | K=50 | K=250 | Luma norm() every 50 (no hist kernels) | no residual at all, record+sync | host recording time only |
|---|---|---|---|---|---|---|---|---|
| Direct | 100x50 | 319.7 us | 286.4 us | 199.7 us | 175.0 us | 188.5 us | 159.5 us | 157.2 us |
| Direct | 200x100 | 322.9 us | 298.7 us | 206.9 us | 178.3 us | 147.9 us | 141.3 us | 135.6 us |
| Direct | 400x200 | 333.1 us | 306.2 us | 219.7 us | 194.0 us | 201.2 us | 172.4 us | 149.7 us |
| Guarded | 100x50 | 133.3 us | 96.0 us | 84.2 us | 58.3 us | 89.6 us | 46.7 us | 43.9 us |
| Guarded | 200x100 | 153.1 us | 108.8 us | 101.5 us | 70.6 us | 95.0 us | 52.4 us | 49.4 us |
| Guarded | 400x200 | 192.6 us | 141.9 us | 134.1 us | 105.3 us | 131.2 us | 92.8 us | 60.0 us |

Reading the study (f32 precision): with `no residual at all` the guarded solver costs 46.7 / 52.4 / 92.8 µs per iteration for 100×50 / 200×100 / 400×200
(host recording alone: 43.9 / 49.4 / 60.0 µs = 7 launches × 6 – 9 µs); the direct solver 159 / 141 / 172 µs (17 launches ≈ 135 – 157 µs of host
time). The history kernel adds one launch per iteration; a synchronisation costs ≈ 0.08 ms at K = 1, ≈ 0.4 ms at K = 10 and ≈ 1.3 ms at K = 50 (guarded 100×50: (time/iter − time/iter at K = 250) × K), growing with the number of dispatches that are only submitted at the sync (up to 255, see the batching item below); K = 250 brings the
guarded solver within 12 – 18 µs of the no-residual cost. `Luma norm() every 50` (Luma's `Array<f32>::norm`, no history kernels)
costs about the same as the history kernels at K = 50 (90 / 95 / 131 µs vs 84 / 102 / 134 µs per iteration, guarded) and only gives the last iteration's residual, so the history kernels are the default. The extended
solver adds only device time, see the steady-state table above.

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
  face kernels against the f64 `bowshock-cfd` functions on 20 000 random stencils: **6e-13** relative (the f32 kernels: 1.2e-5), see the test
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

**Performance cost** (steady state, 15°, minmod, best of 3, history kernels on, sync every 50): 100x50: 128.6 µs/iter extended vs 89.8 µs f32 (1.43x cost; CPU 601 µs → speed-up 4.7x instead of 6.7x); 200x100: 130.8 µs/iter extended vs 93.5 µs f32 (1.40x cost; CPU 1206 µs → speed-up 9.2x instead of 12.9x); 400x200: 262.1 µs/iter extended vs 139.7 µs f32 (1.88x cost; CPU 2930 µs → speed-up 11.2x instead of 21.0x) (compensated: 84.8, 88.9, 154.8 µs). The extended precision is host-launch-bound on the small grids (7 launches/iteration, same count as f32) and device-bound at 400x200, where the df arithmetic costs ~2 x. See the steady-state table below.

### Compiler pitfalls found on the way (important for anybody doing error-free arithmetic in WGSL/Luma)

The first df implementation was *not better than f32* (face-kernel error 8e-6): the Vulkan driver's compiler (NVIDIA RTX 3090) treats WGSL float
arithmetic as associative and contracts `a*b + c` into `fma`. Two-sum `e = (a - (s - bb)) + (b - bb)` with `bb = s - a` was simplified to
`0`, and `s + a*b`-style sums used the exact product while the error term used the rounded one (error counted twice). There is no `precise` / `NoContraction` in WGSL. Fix: every operand
and intermediate of an error-free transformation goes through `opq(x) = bitcast<f32>(bitcast<u32>(x) | p.zero)` where `p.zero` is a uniform
that is always 0 (`Params::zero`), which the compiler cannot see through (`shaders::common`, `shaders_ext::ts/qts/tp`; same for `df_add` of the
compensated update — whose first version had the same defect and gained only ~1.6× instead of 2.4×). The test
`extended_face_kernels_match_cpu_to_1e9` fails if a driver / compiler change breaks the transformations again. `fma` itself is honoured.
The accurate df add was replaced by the cheaper "sloppy" df add and division uses 2 instead of 3 correction steps (same 6e-13 accuracy, 20 – 30 % faster).

### f32 floor vs CFL (original `F32` precision; `cargo run --release -p bowshock-gpu --example f32_floor`, GPU only, 15°, minmod, 6000 iterations)

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

## Tests (`cargo test --release -p bowshock-gpu`, 5 tests, all pass)

1. `hllc_and_reconstruction_match_cpu_on_random_states`: 20 000 random states (smooth neighbours with 0.3 % / 3 % perturbations and unrelated
   states; random normals) through the f32 GPU `reconstruct` + `hllc` + `face_flux` kernels vs `bowshock_cfd::flux` on the same f32 inputs, first
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

## Luma friction / feedback

Everything worked without a Luma bug: no wrong result, no panic, poisoned arrays never triggered, argument/binding checking at launch was helpful.
Observations, most important first (the first two decided the structure of the solver; item 9 was found while fixing the f32 floor):

1. **Host cost per launch ≈ 8.2 – 8.6 µs, independent of problem size** (`launch_overhead` above: 4000 dependent trivial launches,
   64 … 80 000 invocations; record+sync 9 µs). `Context::launch` creates a new compute pass per dispatch, allocates a `Vec` of bindings,
   hashes a `BindKey` (cloned `wgpu::Buffer`s) into a `HashMap`, and takes fresh arena slots for the grid uniform and every uniform argument.
   With 17 launches per iteration the direct solver is host-bound at every grid size (≈ 150 µs/iter); only fusing kernels (7 launches)
   helped. Suggestions: one compute pass for consecutive launches of a batch (or at least for launches without barriers), cache the bind-group
   key without cloning buffers, reuse the grid-uniform slot when the size is unchanged.
2. **No public non-blocking submit; batches go out only every 256 dispatches or at a sync.** `Context::flush` is `pub(crate)`. For a pipeline
   like 7 launches × K iterations the GPU receives nothing until 256 dispatches are recorded, and the sync then waits for the unsubmitted remainder serially
   [INFERENCE from the code and the sync-cost numbers above: ≈ 1 ms per sync at K = 50]. A `ctx.flush()` (submit without waiting), a smaller adaptive auto-flush threshold, and a non-blocking
   read-back (`to_vec_async` / a future, so the residual history of chunk n is fetched while chunk n+1 runs) would remove most of the sync cost.
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

* Timings were taken on a shared desktop (browser, compositor, language server running); bowshock-cfd author's heavy CPU jobs had finished before the run
  but the load average was still decaying at its start. The steady benchmark (best of 3 × 2000 iterations) is the reference; the per-run wall times
  in "Wall time of the convergence runs themselves" are noisier (e.g. the CPU's 1379 µs/iter for the very first case vs ≈ 610 µs for the same grid later).
* The CPU run in `cone.rs` repeats `run_cone`'s loop and stopping logic using the public `Solver::step` (so the final CPU state is available) and copies the
  private `postprocess` of `bowshock_cfd::cone`; it reproduces bowshock-cfd's reference iteration counts exactly. If bowshock-cfd's criteria change, `Monitor` must follow.
* GPU runs execute up to K−1 = 49 iterations past the iteration where the criterion fired (`exec` vs `stop` in the tables); the final state is the one after `exec`.
* `Precision::Extended` depends on the driver honouring the opaque-identity workaround (`opq`, see "Compiler pitfalls"); it was verified on the RTX 3090 / Vulkan only, and
  `extended_face_kernels_match_cpu_to_1e9` is the canary on other devices or driver versions. It also needs `fma` to be a true fused multiply-add (two-product).
* The extended GPU operator uses f32 geometry (face normals, weights, cell area/volume: converted from the f64 grid once), so it differs from the CPU operator by ≈ 6e-8 relative; the
  GPU converges its own discrete problem (hence 1e-6-level field differences and residuals of the CPU operator evaluated on the GPU state of 8e-6 … 3e-5, see "The f32 residual floor"). Uploading the geometry as hi/lo pairs would remove it; not needed for the 6-orders target.
* The GPU solver mirrors bowshock-cfd as of its SSP-RK3 + HLLC/Rusanov-blend version (`flux::face_flux`, `Solver::step`/`stage`); BCs, limiters, grid and
  first-order start-up are as before.
