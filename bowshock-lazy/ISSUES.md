# bowshock-lazy: issue list

Problems met while porting the solver to Luma array ops. `Luma` = needs a Luma change (reported in
`~/Documents/luma/Temp.md`), `lazy` = this crate, `core` = the CPU reference.

| # | Area | Status | Issue |
|---|---|---|---|
| 1 | Luma | reported 2026-11-13 | Process exit hangs (100% CPU in `libnvidia-gpucomp`) or corrupts the heap (`free(): chunks in smallbin corrupted`) while the NVIDIA driver is still compiling a pipeline; intermittent (1 in 3 `bench --quick --staging Fluxes` runs). Blocks shipping the lazy path in `bowshock::Analysis`, not development. |
| 2 | Luma | reported 2026-11-13 | Region assigns into a lazy expression grow it past the fusion limit: with a solid block (base grid 156x76) one SSP-RK3 step is, f32: 16 launches / ~70 µs with `Fluxes` (2 cuts `1537 inlined operations > 1536` at the `assign`s) vs 12 / 51 µs on a block-free grid of the same size, and 89 launches / ~330-380 µs with `Fusion` (3 cuts) vs 26 / 100 µs; Df32: `Fluxes` 19 launches / 178 µs, `Fusion` 96 launches / 2041 µs (6 cuts: op limit at the zero / `dt` assigns, "recomputing 9 times costs more than storing" at the base-flux assign). The assigned regions are a few hundred cells, but every reader inlines both branches. |
| 3 | lazy | fixed | `zero_region` uploaded a fresh `zeros_on` array per call; now a scalar `assign(c(0.0))` (same launches / cuts). |
| 4 | Luma | reported 2026-11-13 | No `stack`: building the `[16, ni, nj]` block arrays for `solve_block_tridiagonal` with `concat::<f32, 16>(0, ..)` of `[1, ni, nj]` parts costs 11.7 µs per stack at 144x70 (expected ~2 µs, a bandwidth-bound copy); six per implicit step = 70 µs, 29% of the step, as much as the four line solves. Region-assigning the 16 parts into a zeros base was ~20x slower. A `stack` op or block solves taking `[Array; 16]` / `[Array; 4]` would remove it. |
| 5 | Luma | reported 2026-11-13 | `reshape` (`array.rs:708`) evaluates its operand: `solve_result.slice(0, k..k+1).reshape([ni, nj])` materialises. Worked around with `take` and precomputed index arrays (stays lazy). |
| 6 | Luma | reported 2026-11-13 | No `is_finite`: the positivity damping uses `abs().lt(1e30)`, relying on NaN comparing false in the generated WGSL. |
| 7 | lazy | open | Implicit step 2.1-2.5x slower than the hand-written kernels (µs/iter, replay): sting 144x70 f32 227 vs 106, Df32 267 vs 144 (Extended); 288x140 541 vs 261, 591 vs 286; base 156x76 240 vs 97, 304 vs 143; 312x152 598 vs 274, 712 vs 303. CPU 2400-11000. Per level-0 step: line solves 82 µs (Luma's cyclic reduction; Thomas 2.5x slower; the per-line starts of the block grid cost nothing), `concat` stacks 70 µs (#4), the rest assembly. |
