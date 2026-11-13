# bowshock-lazy: issue list

Problems met while porting the solver to Luma array ops. `Luma` = needs a Luma change (reported in
`~/Documents/luma/Temp.md`), `lazy` = this crate, `core` = the CPU reference.

| # | Area | Status | Issue |
|---|---|---|---|
| 1 | Luma | reported 2026-11-13 | Process exit hangs (100% CPU in `libnvidia-gpucomp`) or corrupts the heap (`free(): chunks in smallbin corrupted`) while the NVIDIA driver is still compiling a pipeline; intermittent (1 in 3 `bench --quick --staging Fluxes` runs). Blocks shipping the lazy path in `bowshock::Analysis`, not development. |
| 2 | Luma | open | Region assigns into a lazy expression grow it past the fusion limit: with a solid block (base grid 156x76, f32) one SSP-RK3 step is 16 launches / 67 µs with `Fluxes` (2 cuts `1537 inlined operations > 1536` at the zeroing `assign`s) vs 12 / 51 µs on a block-free grid of the same size; with `Fusion` 89 launches / 379 µs (3 cuts at the block-ghost `assign`) vs 26 / 100 µs. The zeroed / ghost regions are a few hundred cells, but the assign makes every reader inline both branches. |
| 3 | lazy | fixed | `zero_region` uploaded a fresh `zeros_on` array per call; now a scalar `assign(c(0.0))` (same launches / cuts). |
