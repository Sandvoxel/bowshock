# bowshock-lazy

Experiment: bowshock's explicit solver (`Solver::eval_residual` + `rk3_step`) written in plain Luma array ops, no custom
kernels, compared with the hand-written WGSL kernels (`bowshock-gpu`) and the f64 CPU reference (`bowshock-core`).

The unreleased lazy array API of Luma (`/home/sandvoxel/Documents/luma`, 0.3.0 working tree) is used: views (`slice`, `pad`,
`concat`), `cache()`, `select`, `Array<Df32>`, `iterate_all`, `iterate_until_all`, `Profile::splits`. The crate is **not** a member of
the bowshock workspace (empty `[workspace]`); `bowshock-gpu` keeps its own Luma 0.2.4 (git) in the same dependency graph without
conflict (it is a dev-dependency, `luma_old` is that same Luma under another name, needed to open its `Context`).

```text
cargo test --release                                  # accuracy vs the CPU solver, loop forms agree
cargo run --release --example bench [-- --quick]      # speed tables (--quick skips 400x200; --profile-only, --staging Lines|Split|Fusion)
cargo run --release --example luma_issues             # minimal reproductions of the Luma problems below
```

## What is in it

* `src/body.rs`: the solver, written once. `lib.rs` includes it twice: `f32s` (`Array<f32>`) and `df32s` (state, residual and time
  step in double-float `Array<Df32>`, the counterpart of `Precision::Extended`; the state lives between steps as an `f32` pair
  `hi + lo`, see "Luma issues" 5). Covered: grids without a solid block, `Scheme::Explicit`, `Limiter::{FirstOrder, Minmod,
  VanAlbada}`, `face_flux_lm` (HLLC blended with Rusanov by the shock sensor, optional low-Mach correction), all `Bc` variants
  (`Freestream`, `Extrapolate`, `SlipWall`, `Characteristic`, `PressureOutflow`), the axisymmetric pressure source, local time
  step, SSP-RK3 with the first-order switch (one traced loop per limiter phase). Not covered: blocks (base/wake grids), implicit
  and multigrid schemes, viscous terms.
* `tests/accuracy.rs`, `tests/loops.rs`; `examples/bench.rs`, `examples/luma_issues.rs`.

The math reads like the CPU code, expression by expression (`body.rs`, HLLC star state; `c(x)` is the constant `x` rounded to the
element type, `&` borrows an array):

```rust
/// `F_K + S_K (U*_K - U_K)`, `U*_K = fac [1, u + (S_M - u_n) n_x, v + (S_M - u_n) n_r, E / rho + (S_M - u_n) (S_M + p / (rho (S_K - u_n)))]`
let star = |w: &Rec, f: &Rec, un: &A, e: &A, s: &A, s_un: &A| -> Rec {
    let [r, u, v, p] = w;
    let fac = r * s_un / (s - sm);
    let sm_un = sm - un;
    [
        &f[0] + s * (&fac - r),
        &f[1] + s * (&fac * (u + &sm_un * nx) - r * u),
        &f[2] + s * (&fac * (v + &sm_un * nr) - r * v),
        &f[3] + s * (&fac * (e / r + &sm_un * (sm + p / (r * s_un))) - e),
    ]
};
```

CPU branches (`if sl >= 0 { .. } else if ..`, `if t == 0.0 { return c }`, `if z >= 1.0`) become `select`; the ghost layers of each
boundary are small arrays (`pad(Edge | Mirror)` + `slice` read the edge / mirror cell, the characteristic and pressure-outflow states are
array expressions on them), joined with the interior by `concat` into the cell lines `[ni + 4, nj]` / `[ni, nj + 4]`; a face reads its
four cells as four `slice`s of the line, the residual is `fi.slice(0, 1..) * w.slice(0, 1..) - fi.slice(0, ..ni) * ..`. The geometry
(normals, lengths, weights, `1/V`, areas) is uploaded once from the f64 `Grid`, rounded to `f32` or `Df32`.

## Accuracy

Same set-ups as `bowshock-gpu-cases/tests/gpu_vs_cpu.rs` (`state_after_20_iterations_matches_cpu`: 40x20 cone grid, M = 2, 15 degree
cone, minmod and van Albada; `characteristic_far_field_matches_cpu_on_a_perturbed_state`: far-field box, M = 0.9 and 1.3,
characteristic and fixed-pressure outflow; `transonic_state_after_20_iterations_matches_cpu`: ogive-cylinder 144x70, M = 0.9; plus
the low-Mach correction on the box at M = 0.3): CPU and lazy solver start from the same state, run 20 iterations (the first 5 first
order), and are compared by `max |dU|/(1+|U|)` (state) and `max |r_lazy - r_cpu| / r_cpu` over the 20 residual-history values.
`cargo test --release -- --nocapture` prints every row (default staging below; the other stagings give the same numbers up to
a few percent of the f32 rows). Target = the tolerance the hand-written kernels' tests use for the same precision; "custom" =
the numbers `Mode::Guarded` prints in those tests on this machine.

| case | lazy f32 state / residual | custom `F32` state / residual | lazy df32 state / residual | custom `Extended` state / residual |
|---|---|---|---|---|
| cone 40x20 minmod | 1.0e-6 / 9.0e-7 | 1.1e-6 / 8.2e-7 | 4.1e-14 / 3.0e-14 | 4.5e-8 / 3.7e-8 |
| cone 40x20 van Albada | 9.0e-7 / 8.1e-7 | 1.0e-6 / 1.0e-6 | 1.8e-10 / 2.1e-11 | 4.3e-8 / 4.8e-8 |
| box M = 0.9 characteristic | 8.9e-7 / 1.9e-6 | 9.8e-7 / 7.3e-7 | 2.7e-14 / 1.7e-14 | 1.7e-8 / 4.0e-8 |
| box M = 0.9 pressure outflow | 9.4e-7 / 1.9e-6 | 9.9e-7 / 7.0e-7 | 2.7e-14 / 2.0e-14 | 1.6e-8 / 5.5e-8 |
| box M = 1.3 characteristic | 9.6e-7 / 4.6e-7 | 1.0e-6 / 6.5e-7 | 3.0e-14 / 3.1e-14 | 2.0e-8 / 4.7e-8 |
| box M = 1.3 pressure outflow | 9.6e-7 / 4.6e-7 | 1.0e-6 / 6.5e-7 | 3.0e-14 / 3.1e-14 | 2.0e-8 / 4.7e-8 |
| transonic M = 0.9 144x70 | 7.6e-7 / 1.0e-6 | 8.0e-7 / 8.6e-7 | 3.5e-14 / 3.4e-14 | 6.2e-8 / 1.0e-7 |
| box M = 0.3 low-Mach van Albada | 9.0e-7 / 7.0e-7 | (not in that test file) | 2.2e-14 / 2.1e-14 | |
| **target** | 1e-5 / 1e-4 | | 3e-7 / 2e-7 (5e-7 transonic) | |

All 8 cases pass in all three stagings and both precisions. The f32 rows are at the f32 round-off of the CPU comparison, like the custom
kernels'. The double-float rows are 6 orders better than the custom `Extended` kernels because the lazy solver uploads the geometry
as `Df32` computed from the f64 grid, where `Extended` keeps f32 normals / weights / volumes (plus a closure-defect correction); the
arithmetic itself is the same double-float (~1e-14 relative). The 1.8e-10 of the van Albada cone is two orders above its siblings
[INFERENCE: sign decisions of the limiter on round-off-level differences in the uniform far field, 1e-16 on the CPU vs 1e-14 in
double-float; not investigated]. The three loop forms (`iterate_until_all`, `iterate_all`, `eval_all` loop) give bit-identical
states and residual histories (`tests/loops.rs`). The residual history of the double-float solver is recorded on the GPU as two `f32`
records (`hi`, `lo`), because the history of `iterate_until_all` is `f32` only.

## Speed

RTX 3090, Vulkan, 15 degree cone at M = 2, minmod, MUSCL from the first iteration, 2000 iterations from the free stream (as
`cone_gpu --bench`, whose grids and settings these are), best of 3, microseconds per iteration. Steady state: the first call of every
loop is run before the clock starts, so compilation and tracing are not in these numbers. Custom = `GpuSolver`, `Mode::Guarded`,
history kernel on, `run(50)` chunks; CPU = f64 `Solver` on 32 rayon threads (500 iterations). Lazy = `LazySolver::run`
(`iterate_until_all`, residual recorded every iteration, one wait per 50 iterations) or `run_replay` (`iterate_all`, no history),
whichever is faster for the best staging; the full grid of variants is below.

| grid | CPU f64 | custom F32 | custom Extended | lazy f32 | lazy df32 | lazy f32 / custom F32 | lazy df32 / custom Extended |
|---|---|---|---|---|---|---|---|
| 100x50 | 613 | 29.0 | 74.9 | 97.2 | 243.5 | 3.4x | 3.3x |
| 200x100 | 1285 | 33.5 | 81.1 | 109.2 | 278.4 | 3.3x | 3.4x |
| 400x200 | 3199 | 61.7 | 258.2 | 158.6 | 618.8 | 2.6x | 2.4x |

Lazy solver by staging and loop form (`until` = `iterate_until_all` + residual history, `replay` = `iterate_all`, `eval` =
`eval_all` after every iteration, 300 iterations):

| grid | staging | f32 until | f32 replay | f32 eval | df32 until | df32 replay | df32 eval |
|---|---|---|---|---|---|---|---|
| 100x50 | Fusion | 140 | 124 | 2075 | 291 | 270 | 2782 |
| 100x50 | Lines | 116 | **97** | 2330 | 264 | 246 | 2843 |
| 100x50 | Split | 131 | 113 | 2056 | 270 | **243** | 2349 |
| 200x100 | Fusion | 155 | 138 | 2243 | 367 | 334 | 2904 |
| 200x100 | Lines | 132 | **109** | 2310 | 334 | 307 | 2839 |
| 200x100 | Split | 145 | 122 | 2039 | 303 | **278** | 2347 |
| 400x200 | Fusion | 212 | 189 | 2240 | 723 | 684 | 2797 |
| 400x200 | Lines | 181 | **159** | 2276 | 718 | 681 | 2729 |
| 400x200 | Split | 209 | 186 | 2036 | 642 | **619** | 2574 |

* **Loop form**: `iterate_all` replay is fastest; the residual history of `iterate_until_all` costs 15-30 us per iteration; the plain
  `eval` loop (the step is rebuilt in Rust every iteration, 70-100 launches) is 20-30x slower and host bound
  (2-3 ms/iteration at every grid size). Replay traces `step` once per call, so each limiter phase is a separate loop.
* **Staging** (`LazySolver::staging`) is how much of the order of evaluation is spelled out with `cache()`: `Fusion` evaluates only
  grid-level arrays (primitives, every ghost layer, the lines, the fluxes, the residual); `Lines` additionally merges the ghost layers
  of a direction into one array and evaluates the state of every Runge-Kutta stage; `Split` additionally evaluates the reconstructed
  states and the shock weight before the Riemann solvers (two kernels per direction instead of one). The face flux as one expression
  (`Lines`) is fastest in f32; in double-float the 700-operation kernel is slower than two 350-operation kernels (`Split`) from
  200x100 on. Defaults follow this table (`f32s`: `Lines`, `df32s`: `Split`).
* The small grids are launch-bound for both solvers: 100x50 and 200x100 cost nearly the same (lazy f32 97 / 109 us, custom 29 / 33 us).
  At 400x200 the lazy solver is 2.6x / 2.4x of the custom kernels. [INFERENCE: the custom kernels read the state, the four-cell stencil and
  the geometry once per stage in two kernels; the lazy solver additionally writes and reads the padded lines (`wi`, `wj`: 4 fields
  each), the primitives, the fluxes and the residual.]

First call (construction + compile + trace + first 50 iterations), `__GL_SHADER_DISK_CACHE=0`, fresh process, 100x50 (compile times
depend on the grid shape: kernels are specialised to the dimensions, a second grid size compiles again; with the driver's shader
cache warm all of them are 0.01-0.1 s):

| | custom F32 | custom Extended | lazy f32 `Lines` | lazy f32 `Fusion` | lazy df32 `Split` | lazy df32 `Lines` | lazy df32 `Fusion` |
|---|---|---|---|---|---|---|---|
| seconds | 0.17-0.20 | 3.5-5.3 | 0.20 | 0.17 | 0.75 | 1.31 | 1.36 |

### What one iteration became (`Profile::splits`, 200x100)

`bench` prints the per-kernel profile of one residual evaluation and the launch counts of one step for every staging (`--profile-only`
for just that). Launches per residual evaluation / per SSP-RK3 step (GPU time inside the profiled step, every launch in its own pass,
so it is longer than the wall time of the replay): f32 `Fusion` 30 / 95 (353 us), `Lines` **18 / 68** (251 us), `Split` 20 / 74
(281 us); df32 `Fusion` 30 / 106 (666 us), `Lines` 18 / 76 (572 us), `Split` **20 / 82** (529 us). The custom `Guarded` kernels launch about
7 per iteration. `Profile::splits` is empty for all of these, because the shipped code has been shaped until it stopped producing
cuts; the first, natural version of the same equations (one expression per face, caches only after the primitives and the fluxes) became
**166 launches per residual evaluation, 505 per step, with 6 recorded cuts per evaluation** (`17 input buffers > 16` at the HLLC
`select` chain, `22` and `20` at the blend `add`): the 4-output flux expression shares nearly every node between its outputs, and
every cut, together with Luma's "a node another live array also reads is evaluated on its own first" [INFERENCE: the mechanism], split it into one-operation
kernels (`Fused(Sub)`, `Fused(Abs)`, `Fused(Mul)`, ... about 3 us each, GPU time ~ launch count x 3 us). What the engine did well:
the whole face flux (MUSCL, HLLC, Rusanov, shock blend: ~115 operations, 4 outputs, 6 buffers) fuses into one kernel
if its inputs are plain buffers (`Lines`), and the reconstruction of 8 outputs from 16 stencil views is one kernel (`Split`).
The 18 launches of a residual evaluation in `Lines` (f32): primitives 1, the line arrays 2 (`concat`, 4 outputs each), the two
fluxes 2, the residual 1, one more elementwise kernel (the time step, first stage only), and ten small kernels for the ghost layers
(arrays of 2 cells per row: the grouped `concat` of each direction, and copies of constants and of the extrapolated / mirrored
edge views, ~3 us each: the launch floor, not work). Rules that decided the shape (all found by profiling):

1. `cache()` the arrays that many expressions read (primitives, lines, fluxes, stage states); never `cache()` one after the other
   several arrays that share most of their operations and are functions of an already cached record (`cache(&s[0]); cache(&s[1]);
   cache(&s[2])` of the three HLLC wave speeds turned 11 launches into 49).
2. Do not use a view that an already evaluated kernel consumed in a new expression next to that kernel's outputs (Luma issue 3):
   `Split` evaluates the shock weight, which reads the same stencil views, in the same group as the reconstructed states.
3. A copy handle (`dup`, an identity `slice`) of a pending expression is a read of it through a view, which Luma documents as
   evaluating the expression when it is not small ("the third reader evaluates it"). Removing the `dup`s of the HLLC star state did not
   change the launch count here; consume values instead of copying them where the math allows.

## Luma issues

`cargo run --release --example luma_issues` reproduces 1-3. Luma checked out at `/home/sandvoxel/Documents/luma` (working tree
with the `[Unreleased]` lazy API, not modified).

1. **A `Df32` kernel whose only library calls are comparisons cannot bind its uniform.** `Array::<Df32>::from_slice(..).abs().to_vec()`,
   `a.minimum(&b)`, `a.maximum(&b)`, `a.lt(&b)` (the other array-array comparisons were not tried) fail with `Gpu("Validation Error ... In
   Device::create_bind_group: Number of bindings in bind group descriptor (3) does not match the number of bindings defined in the
   bind group layout (2)")`. Expected: the values. Cause: `src/jit/program.rs:471-481` (`calls_library`) gives every program with
   a `df_*` expression a guard uniform (`Fused.key.guard`), but `df_abs`, `df_lt`, `df_min`, `df_max` (`src/builtin.rs:384-389`) never
   read it (only `luma_o`, used by `df_add`, `df_mul`, `df_div`, `df_sqrt`, does); a program with no other uniform has then no binding
   for it in the shader (`src/jit/codegen.rs:397-401` emits the library, the unused `p.sN` is dropped by naga). It works when the kernel
   has a scalar operand (`a.minimum(Df32::from(0.5))`) or any `+ - * / sqrt`. Worked around in `df32s` (`aabs`, `alt`, `amin`, `amax`
   always carry a subtraction or a scalar compare: `src/lib.rs`); kernels that fuse such an op with arithmetic are fine [INFERENCE: which is
   why it showed up in the solver only for kernels the fusion engine had cut down to comparisons].
2. **`x - x.to_f32().to_df32()` is 0 when fused into one kernel.** With `x: Array<Df32>`, `lo = (&x - x.to_f32().to_df32()).to_f32()`
   evaluates to `[0, 0]` (wanted `x.lo`); it is right when `x.to_f32().cache()` is evaluated first. The cast is `({value}.x +
   {value}.y)` (`src/jit/codegen.rs:210`), not passed through `luma_o` like the library's arithmetic, so [INFERENCE] the shader compiler
   reassociates `x.hi - (x.hi + x.lo)`. Worked around in `df32s::split`; this is how a double-float value is split into the `f32`
   pair that `Fields` / `iterate*` need (see 5). Unguarded `Df32 -> f32` casts are not safe next to library arithmetic.
3. **Input-buffer count of a fused expression is wrong after an evaluation.** An expression of four arrays over 14 distinct buffers
   (8 cached `l`/`r`, 2 normals, 3 cached `s`, 1 `w`, each also read through views) reports `21 input buffers > 16` and is cut into
   76 launches instead of 12, but only when the three `s` arrays were evaluated with `cache()` first (`examples/luma_issues.rs`, third
   section: `s evaluated: false: 12 kernel launches, 0 cuts` vs `s evaluated: true: 76 kernel launches, 37 cuts, first
   "21 input buffers > 16"`). Expected: 12 launches, no cuts. Not diagnosed further (`measure` in `src/jit/mod.rs:801` counts buffer ids
   transitively; some operand of the evaluated arrays seems to keep its pre-evaluation inputs). In the solver this was the
   `cache(&s[0..2])` + shock weight combination above.
4. **Wrong results when several threads use the global `default_context()`** (observed, not reduced to a minimal repro). With the
   default parallel `cargo test`, the accuracy tests, each tracing / replaying its own loops on `luma::default_context()`, failed
   (state errors 1e-3 to 1e-1, one run of the f32 box, df32 cone and df32 low-Mach case each); with `--test-threads=1`, or one
   `luma::Context::new()` per test (what the tests now do), all pass. `tests/iterate_until_all.rs` in Luma says the same ("every test
   has a context of its own"); the recordings are per context, but concurrent use of one context should not be silently wrong.
5. **API gaps** (not bugs): `Array` is not `Clone`, so shared handles need a view (`slice(0, ..)`, see rule 3); `Fields`, `iterate_all`
   and `iterate_until_all` take `f32` arrays only, so a `Df32` state has to live as an `hi`/`lo` pair of `f32` arrays and be converted
   each step (casts, `hi`/`lo` splits and a few extra kernels: 76 against 68 launches per step in `Lines`, plus issue 2); `Operand<f64> for f64` does not exist, so a port to native `f64`
   arrays would need constant arrays in every `gt`/`maximum`/`select` (not tried; the RTX 3090 has `SHADER_F64`); the `iterate_until_all`
   history is `f32`; `concat` selects between parts that are all evaluated (a ghost expression inlined into a `concat` is computed at
   every cell), so ghost layers must be evaluated into small arrays first; `Profile::splits` lists the cuts at node creation only, not
   the "evaluated on its own first" decisions that dominated the launch count.

## Going further

* **Implicit scheme (`solve_block_tridiagonal`).** The primitive fits the line-implicit scheme of `bowshock-core::implicit` (4x4 blocks,
  `block <= 8`, per-line starts `solve_block_tridiagonal_from` for the solid-block grids), but with three consequences: the arrays
  must be stacked `[4, ni, nj]` / `[16, ni, nj]` along axis 0 (`axis` is 1 or 2), so the state would live in one array per record and be
  built with `concat`; the solve is `f32` only (the custom implicit scheme keeps its left-hand side in f32 too, so this is no loss); and it
  runs **one invocation per line** (block Thomas, 4x4 pivoted blocks): 100-400 invocations for these grids, each a dependent chain of
  100-400 block eliminations. The custom kernels use cyclic reduction (parallel along the line) for exactly this reason, so I expect the
  solves, not the assembly, to dominate. The assembly itself (`flux_jacobian`, dissipation matrices, wall Jacobians) is 16 entries x 3
  blocks x 2 directions of array expressions, i.e. several kernels of <= 8 outputs / 16 inputs each, which the staging rules above
  govern. Not tried.
* **`iterate_until_all` as the run loop.** It already is one here (`LazySolver::run` is one per limiter phase). The convergence rules of
  `bowshock::run::stop_reason` map onto it: `ResidualDrop` = `h["residual"].peak() / h["residual"].last() >= 1e6` (Luma's `peak` is over
  all steps, the CPU rule over steps after 20: use `h.step()` to gate), the monitor "stationary" rules = `window_max(k) - window_min(k) <
  tol` of a recorded Cd / base-Cp (`record("cd", ..)` with `sum_where` over the wall faces), `NonFinite` = `last().le(x).not()`.
  What does not map: the limiter switch (the traced step is specific to the limiter: one loop per phase, or the CFL ramp as a
  `luma::Scalar`, which changes without recompiling), warm starts across a Mach sweep (the free stream could be `Scalar`s, again no
  recompile), and `StopReason` variety beyond one predicate index per rule.
* **Speed.** The remaining gap to the hand-written kernels is mostly launch count (68-82 per step against ~7), then the traffic of
  the intermediate arrays. Merging the eight small ghost kernels (one kernel that writes the padded line array in place with `assign`)
  and fusing the stage updates into the primitive kernels are the next steps; a single fused kernel for the whole residual is not
  expressible because the line arrays are read through shifted views of a computed array (Luma evaluates those first, as designed).
* **Blocks and viscous terms.** Solid-block grids need the masks of `Block` (`select` on `index(axis)` comparisons) and the base-wall
  reflections; Green-Gauss gradients and the face-gradient correction are slices and arithmetic; SST needs only `tanh`, `sqrt`, scalar
  tridiagonal solves (`solve_tridiagonal`) and a wall-distance array. None of it was tried.
