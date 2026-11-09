# M2 (subsonic) — work in progress

Branch `subsonic-wip`. Not reviewed; do not merge to `main` until finished and verified. Delete this file on merge.

## State when paused (2026-11-09)
- Workspace builds (`cargo build --release --all-targets`, 0 errors). Full test run was not re-checked before pushing.
- bowshock-cfd (~74%): convergence acceleration (`implicit.rs`, `multigrid.rs`), low-Mach flux work (`flux.rs`), base/wake grid (`grid.rs`, `body_grid.rs`), panel-method check (`potential.rs`), `subsonic.rs` + `bin/subsonic.rs`, experiment examples (`accel`, `basetest`, `fox`, `lowmach`, `pot`, `pulse`, `resloc`, `trace`). Was running the convergence-scheme comparison.
- bowshock-gpu (~72%): implicit-scheme WGSL (`shaders_impl.rs`) and solver wiring done; its tests passed. Was running benchmark sweeps.

## Remaining
1. bowshock-cfd: finish scheme comparison and pick the scheme (measure iterations/wall time vs explicit SSP-RK3); subsonic sweep M 0.1–0.8 with base on/off; switch `transonic` bin to the new scheme; subsonic validation vs a published NACA/NASA dataset (cite; no invented data) + panel-method check; README 'Subsonic' section; update 'Transonic' convergence numbers.
2. bowshock-gpu: mirror any final CPU changes; `subsonic_gpu` bin; update `transonic_gpu`; GPU-vs-CPU tests; timings; README.
3. Verify: `cargo test --release`; cone/ogive/transonic results unchanged or improvement explained with before/after numbers; then clean up experiment examples that aren't worth keeping and merge.

## Notes
- Previous agent runs died on fallback model server network errors (`192.168.1.101:8787`), not code issues.
- Agent transcripts from this session were on the original machine only.
