# bowshock-lazy

Experiment: bowshock's explicit solver (`Solver::eval_residual` + `rk3_step`) written in plain Luma array ops, no custom
kernels, compared with the hand-written WGSL kernels (`bowshock-gpu`) and the f64 CPU reference (`bowshock-core`).

The unreleased lazy array API of Luma (`/home/sandvoxel/Documents/luma`, 0.3.0 working tree, `[Unreleased]` in its changelog) is
used: views (`slice`, `pad`, `flip`), region `assign`, records (`Fields`), `cache()`, `select`, `Array<Df32>`, `iterate_all`,
`iterate_until_all`, `Profile::splits`. The crate is **not** a member of the bowshock workspace (empty `[workspace]`);
`bowshock-gpu` keeps its own Luma 0.2.4 (git) in the same dependency graph without conflict (it is a dev-dependency, `luma_old`
is that same Luma under another name, needed to open its `Context`).

```text
cargo test --release                                  # accuracy vs the CPU solver, loop forms agree, threads on default_context()
cargo run --release --example bench [-- --quick]      # speed tables (--quick skips 400x200; --profile-only, --staging Fusion|Lines|Fluxes|Split)
cargo run --release --example eval_loop               # where the time of the `eval` loop form goes
cargo run --release --example luma_issues             # what still applies of the Luma problems below
```

## What is in it

* `src/body.rs`: the solver, written once. `lib.rs` includes it twice: `f32s` (`Array<f32>`) and `df32s` (state, residual and time
  step in double-float `Array<Df32>`, the counterpart of `Precision::Extended`). The state is a record of arrays of the element
  type (`[Array<S>; 5]`: `rho, rho u, rho v, E` and the density residual of the step) that goes through `iterate_all` /
  `iterate_until_all` as it is, `f32` or `Df32`. Covered: `Scheme::Explicit` with or without a solid block (`set_block`: base/wake
  grids; block ghosts, base-wall face stencils and solid faces/cells are region assigns, no masks), `Limiter::{FirstOrder,
  Minmod, VanAlbada}`, `face_flux_lm` (HLLC blended with Rusanov by the shock sensor, optional low-Mach correction), all `Bc`
  variants (`Freestream`, `Extrapolate`, `SlipWall`, `Characteristic`, `PressureOutflow`), the axisymmetric pressure source, local
  time step, SSP-RK3 with the first-order switch (one traced loop per limiter phase). Not covered:
  implicit and multigrid schemes, viscous terms.
* `tests/accuracy.rs`, `tests/loops.rs`; `examples/bench.rs`, `examples/eval_loop.rs`, `examples/luma_issues.rs`.

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

Same set-ups as `bowshock-gpu-cases/tests/gpu_vs_cpu.rs` (`state_after_20_iterations_matches_cpu`: 40x20 cone grid, M = 2, 15 degree
cone, minmod and van Albada; `characteristic_far_field_matches_cpu_on_a_perturbed_state`: far-field box, M = 0.9 and 1.3,
characteristic and fixed-pressure outflow; `transonic_state_after_20_iterations_matches_cpu`: ogive-cylinder 144x70, M = 0.9; plus
the low-Mach correction on the box at M = 0.3): CPU and lazy solver start from the same state, run 20 iterations (the first 5 first
order), and are compared by `max |dU|/(1+|U|)` (state) and `max |r_lazy - r_cpu| / r_cpu` over the 20 residual-history values.
`cargo test --release -- --nocapture` prints every row for all four stagings (default staging below; the other stagings give the
same numbers up to a few percent of the f32 rows). Target = the tolerance the hand-written kernels' tests use for the same precision;
"custom" = the numbers `Mode::Guarded` prints in those tests on this machine. Unchanged by the update to the fixed Luma.

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

All 8 cases pass in all four stagings and both precisions. The f32 rows are at the f32 round-off of the CPU comparison, like the
custom kernels'. The double-float rows are 6 orders better than the custom `Extended` kernels because the lazy solver uploads the
geometry as `Df32` computed from the f64 grid, where `Extended` keeps f32 normals / weights / volumes (plus a closure-defect
correction); the arithmetic itself is the same double-float (~1e-14 relative). The 1.8e-10 of the van Albada cone is two orders
above its siblings [INFERENCE: sign decisions of the limiter on round-off-level differences in the uniform far field, 1e-16 on the
CPU vs 1e-14 in double-float; not investigated]. The three loop forms (`iterate_until_all`, `iterate_all`, `eval_all` loop) agree
(`tests/loops.rs`: double-float bit-identical, f32 within 6.4e-7 of each other), and four threads sharing `default_context()` each
reproduce the state of a private-context run (`threads_on_the_default_context_agree`). The residual history of the double-float
solver is recorded on the GPU as two `f32` records, `rn.to_f32()` and the low word `(rn - rn.to_f32().to_df32()).to_f32()`, because
the history of `iterate_until_all` is `f32`; with only the high word the history would carry a 6e-8 relative rounding, which is
inside the targets (2e-7) but would hide the 1e-14 agreement.

## Speed

RTX 3090, Vulkan, 15 degree cone at M = 2, minmod, MUSCL from the first iteration, 2000 iterations from the free stream (as
`cone_gpu --bench`, whose grids and settings these are), best of 3, microseconds per iteration, `bench --quick` (the 400x200 grid
was not re-measured). Steady state: the first call of every loop is run before the clock starts, so compilation and tracing are not
in these numbers. Custom = `GpuSolver`, `Mode::Guarded`, history kernel on, `run(50)` chunks; CPU = f64 `Solver` on 32 rayon
threads (500 iterations). Lazy = `LazySolver::run` (`iterate_until_all`, residual recorded every iteration, one wait per 50
iterations) or `run_replay` (`iterate_all`, no history), whichever is faster for the best staging. The GPU was shared with other
jobs during these runs (the custom columns moved by up to 2x between runs, the lazy numbers by about 10 %).

**Before** (the tree as it was, `Dup` / `pack` workarounds, `concat` ghosts, hand-staged `cache()`s, on the fixed Luma; the numbers
of the earlier measurement on this tree, 73.3 / 84.1 and 214.9 / 246.9, agree):

| grid | CPU f64 | custom F32 | custom Extended | lazy f32 | lazy df32 | lazy f32 / custom F32 | lazy df32 / custom Extended |
|---|---|---|---|---|---|---|---|
| 100x50 | 624 | 29.0 | 76.5 | 73.5 | 217.9 | 2.5x | 2.8x |
| 200x100 | 1306 | 51.7 | 82.7 | 84.4 | 249.7 | 1.6x | 3.0x |

**After** (this tree: ghosts as region assigns, `Df32` state, default staging `Fluxes`):

| grid | CPU f64 | custom F32 | custom Extended | lazy f32 | lazy df32 | lazy f32 / custom F32 | lazy df32 / custom Extended |
|---|---|---|---|---|---|---|---|
| 100x50 | 602 | 28.9 | 75.2 | 33.2 | 190.6 | 1.15x | 2.5x |
| 200x100 | 1304 | 33.7 | 81.9 | 44.7 | 260.2 | 1.3x | 3.2x |

(The custom F32 at 200x100 reads 33.7 here and 51.7 before: the same code, a shared GPU. Against the same custom number the
lazy f32 solver went from 2.5x / 2.5x to 1.15x / 1.3x; lazy df32 is unchanged within the noise, see below.)

Lazy solver by staging and loop form (`until` = `iterate_until_all` + residual history, `replay` = `iterate_all`, `eval` =
`eval_all` after every iteration, 300 iterations); before:

| grid | staging | f32 until | f32 replay | f32 eval | df32 until | df32 replay | df32 eval |
|---|---|---|---|---|---|---|---|
| 100x50 | Fusion | 140 | 123 | 1820 | 289 | 267 | 2217 |
| 100x50 | Lines | 92 | **74** | 1825 | 237 | 218 | 2043 |
| 100x50 | Split | 107 | 87 | 1841 | 268 | 277 | 2165 |
| 200x100 | Fusion | 204 | 156 | 2431 | 363 | 330 | 2452 |
| 200x100 | Lines | 105 | **84** | 1991 | 302 | 277 | 2146 |
| 200x100 | Split | 121 | 98 | 2101 | 288 | **250** | 2179 |

after:

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
* The small grids are launch-bound for the custom solver: 100x50 and 200x100 cost nearly the same. The lazy f32 solver is within 1.15x
  / 1.3x of it. In double-float the lazy solver is 2.5-3.2x the custom `Extended` kernels: its flux kernels are one 700-operation
  `Df32` expression each (profile: 2 x 20-25 us of the step's GPU time), the custom ones keep the geometry in `f32`.

First call (construction + compile + trace + first 50 iterations), seconds, 100x50 (kernels are specialised to the grid shape; the
disk cache of the driver was warm for the before runs, so the compile time of the new, bigger kernels is the honest part):

| | lazy f32 `Fusion` | lazy f32 `Lines` | lazy f32 `Fluxes` | lazy df32 `Fusion` | lazy df32 `Lines` | lazy df32 `Fluxes` | lazy df32 `Split` | custom F32 | custom Extended |
|---|---|---|---|---|---|---|---|---|---|
| after | 0.49 | 0.21 | 0.05 | 9.33 | 6.06 | 0.90 | 0.38 | 0.01 | 0.02 |
| before (old code: `Fusion` / `Lines` / `Split`) | 0.07 | 0.06 | | 0.07 | 0.06 | | 0.05 | 0.01 | 0.01 |

### Launches per SSP-RK3 step (`Profile`, 200x100, `bench` profile section)

| | f32 `Fusion` | f32 `Lines` | f32 `Fluxes` | f32 `Split` | df32 `Fusion` | df32 `Lines` | df32 `Fluxes` | df32 `Split` |
|---|---|---|---|---|---|---|---|---|
| before (concat ghosts, hand-staged) | 94 | 47 | | 53 | 101 | 52 | | 58 |
| after | 67 | 26 | **14** | 20 | 67 | 26 | **14** | 20 |

(Fusion cuts per step after: 19, 8, 0, 0. The first measurement after the update, with Luma's working tree as it was a few hours
earlier, gave 91 / 30 / 14 / 20: the `Fusion` and `Lines` counts move with Luma's fusion heuristics, `Fluxes` and `Split` did not.)
The 14 launches of `Fluxes` are 3 per Runge-Kutta stage (the padded record with the previous stage update and residual fused into
it, the i fluxes, the j fluxes), the time step, the final state of the step and the three kernels of the residual norm (a fused
sum, the second reduction pass, `sqrt(sum / N)`). One launch per residual evaluation is not reachable: the fluxes read the
padded record at four cells and the residual reads the fluxes at two, so each of the three needs its own evaluated buffer or
recomputes its inputs (`Fusion`). The reference (Luma's `bench_pde` Euler step, first order) is one kernel because its stencil is
one cell wide and has no nonlinear ghost expression.

### The eval loop

`LazySolver::run_eval` builds the step in Rust and `eval_all`s it every iteration: 1.7-1.8 ms per iteration at 200x100 with `Fluxes`
(before: 1.8-2.4 ms with 47-58 launches). `cargo run --release --example eval_loop` splits it up (us per iteration, 200x100):

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

## Luma issues

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
   `tests/loops.rs::threads_on_the_default_context_agree` runs four threads on it (the other tests keep a `Context::new()` each, for
   isolation).
5. **API gaps: closed.** `Array: Clone` (a clone is another handle on the same data: all `dup` uses were shared handles, none needed
   `&a * 1.0`); `Fields` and `iterate_all` / `iterate_until_all` take `Df32` states (no `hi` / `lo` packing, no `pack` / `unpack` of
   pairs); `concat` is lazy but computes every part at every cell, so ghost layers are region assigns now. Still true: no
   `Operand<f64> for f64` scalar on comparisons, `minimum` / `maximum` or `select` (a native `f64` variant would need `Array::full`
   constants; not done, the optional item of the task); the `iterate_until_all` history is `f32`.

What still applies (`cargo run --release --example luma_issues`):

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

## Going further

* **Implicit scheme (`solve_block_tridiagonal`).** The primitive fits the line-implicit scheme of `bowshock-core::implicit` (4x4 blocks,
  `block <= 8`, per-line starts `solve_block_tridiagonal_from` for the solid-block grids), but with three consequences: the arrays
  must be stacked `[4, ni, nj]` / `[16, ni, nj]` along axis 0 (`axis` is 1 or 2), so the state would live in one array per record and be
  built with `concat`; the solve is `f32` only (the custom implicit scheme keeps its left-hand side in f32 too, so this is no loss); and it
  runs **one invocation per line** (block Thomas, 4x4 pivoted blocks): 100-400 invocations for these grids, each a dependent chain of
  100-400 block eliminations. The custom kernels use cyclic reduction (parallel along the line) for exactly this reason, so I expect the
  solves, not the assembly, to dominate. Not tried.
* **`iterate_until_all` as the run loop.** It already is one here (`LazySolver::run` is one per limiter phase). The convergence rules of
  `bowshock::run::stop_reason` map onto it: `ResidualDrop` = `h["residual"].peak() / h["residual"].last() >= 1e6` (Luma's `peak` is over
  all steps, the CPU rule over steps after 20: use `h.step()` to gate), the monitor "stationary" rules = `window_max(k) - window_min(k) <
  tol` of a recorded Cd / base-Cp (`record("cd", ..)` with `sum_where` over the wall faces), `NonFinite` = `last().le(x).not()`.
  What does not map: the limiter switch (the traced step is specific to the limiter: one loop per phase, or the CFL ramp as a
  `luma::Scalar`, which changes without recompiling), warm starts across a Mach sweep (the free stream could be `Scalar`s, again no
  recompile), and `StopReason` variety beyond one predicate index per rule.
* **Speed.** The remaining gap to the hand-written kernels (14 launches per step against ~7, and the traffic of the padded record and
  the fluxes) is small in f32; in double-float it is the arithmetic: the flux kernels are 700-operation `Df32` expressions.
* **Viscous terms.** Green-Gauss gradients and the face-gradient correction are slices and arithmetic; SST needs only `tanh`, `sqrt`, scalar
  tridiagonal solves (`solve_tridiagonal`) and a wall-distance array. None of it was tried.
