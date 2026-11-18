# GPU solver handoff (gpu3d, branch cfd-3d-gpu)

Hardware of every number: local RTX 3090 (24 GB), remote RTX 3080 Laptop (192.168.1.88); f32 unless noted. Job ids are gpuq ids (`~/.cache/bowshock-gpuq/done/<id>/log`).

## 1. What it is, where it lives

`crates/bowshock-cfd/src/gpu/` (Luma lazy arrays, one source, instantiated twice: `f32s` and `df32s` modules via `include!` in `mod.rs`):

| file | mirrors (CPU) | content |
|---|---|---|
| `common.rs`, `body.rs`, `implicit_common.rs`, `implicit.rs`, `viscous.rs`, `turbulence.rs`, `control.rs` | 2D solver | axisymmetric engine (untouched by the 3D work) |
| `body3.rs` | `solver3.rs` | `LazySolver3`: geometry (`Geo3`), state (`State3` = 11 arrays: U x5, residual, CFL, k, omega, mu_t, force monitor), explicit SSP-RK3, loops (`run_monitored`), force monitor, snapshot/restore, axis-cell residual norm and wedge axis dt cap |
| `implicit3.rs` | `implicit3.rs` | 5x5 block line-implicit step (3 factors, `sub_iters` sweeps), low-Mach P, `pseudo_dt3`, defect correction |
| `viscous3.rs` | `viscous3.rs` | cell pass, viscous face fluxes, traction faces |
| `turbulence3.rs` | `turbulence3.rs` | SST update, wall state (smooth law, Aupoix, rough law + runaway guard = CPU `wall_state`) |
| `rans3.rs` | `solver3/rans.rs`, `jet.rs` | residual with viscous terms, motor jet cells |
| `stages3.rs` | - | per-stage profile (`profile_stages3`) |
| `memory3.rs` | - | persistent-array ledger (`memory_report`) |
| `mod.rs` | - | `GpuSolver3` facade over the two precisions |

Runner: `crates/bowshock-core/src/engine/gpu3.rs` (`GpuRunner3::new(case, prec)`, `run_on`, forces, `y_plus_report`). Product uses `Precision::F32` (`analysis.rs`).
Examples (`crates/bowshock-validate/examples/`): `bench3` (--stages, --mem, LOOP / MONITOR-CHECK lines), `finned_table`, `mem_stages`, `wedge_rans`, `half_probe`. Scripts: `tools/finned_table.sh`, `finned_rough.sh`, `finned_cfl.sh`, `finned_sub.sh`. Tests: `crates/bowshock-gpu-cases/tests/*` (CPU-vs-GPU agreement), gates in `crates/bowshock-core/tests/{wedge_gates,fin_gates,fin_lift_gates}`.

## 2. Precision rules
- `S` precision = f32 or df32 (double-float) for state, residual, fluxes; the implicit left-hand side is **always f32** (`F`, `lhs()`).
- f32 is the product. RANS wedge: CD identical to 5 digits (G-V3 0.213277 both; G3 M0.5 0.26710 both). Agreement with the f64 CPU (tests): laminar/SST wedge f32 <= 2e-6 state, df32 <= 3e-8; sector f32 3e-6, df32 2e-7; rough wall cells f32 2.2e-6 / df32 1.3e-13 (`s5a_turb`).
- Never put a non-GPU-resident requirement into a traced step (next section).

## 3. The compiled loop and the host-clear trap
Luma records one step and replays it. Anything inside `step3` that needs a real buffer or a host clear (`Array::zeros_on` + `put`, `to_vec`, downloads, host constants built per call) **aborts the recording**, and Luma then re-traces and runs the whole step eagerly every iteration. That was the 54x slowdown (RANS wedge 32.5 -> 0.60 ms/iteration f32, 28.4 -> 1.08 ms df32; fix `09a8c0b`, replacing zeros+put by gather+select in `wall_state_pass3` / `wall_speed_grid3`). Rules: build zeros as `c(0.0) * &p[0]`, scatter with `take` + `select`, create geometry arrays only in constructors/setters. Check with `bench3`: the LOOP line must show replay wall ~ replay GPU time (wedge 0.6 vs 0.67 ms); a wall many times the GPU time means an abort (`LUMA_TRACE_ABORT=1` prints the backtrace, Luma main).
`LazySolver3::run_monitored` uses `luma::compile_until_all` (one trace per (limiter, staging, wedge switches, sync), `CompiledLoop3`); the setters that change what the step reads (`set_config`, `set_low_mach`, `set_jet`, `set_viscous*`, `set_force_monitor`) drop it. Chunks cost their launches only (wedge 20 ms per 50 its).

## 4. Forces in the step graph
`set_force_monitor` (runner sets it with the wall faces): the step computes [Fx, Fz, Mz] of the monitored classes (pressure on cut faces + viscous face fluxes, sting/jet excluded) from fluxes it already has; state slot 10; recorded per iteration in the loop history (hi/lo words in df32). Entry i belongs to the state step i started from (one iteration old). `run_on` feeds the mean rule / CN samples from it in chunks of 50; the single exact `lift_forces` is the result's. Agreement with the exact reduction 8e-8 f32 / 1.5e-15 df32 (bench3 MONITOR-CHECK).

## 5. CFL controller (product default)
Policy in `engine/run3.rs` (`CflControl`, `CflEvent`, `CFL_FLOOR` 5, `CFL_BACKOFF` 0.7, `RECENT_RESTORE` 10, `MAX_ROLLBACKS` 12, `Numerics3::cfl_control`, `Case3::without_cfl_control`). GPU: `GpuRunner3::run_on` takes `GpuSolver3::snapshot()` (shares device buffers, no copy) after each good 50-it chunk (two kept), a chunk with residual not finite / > 1e5 or non-finite monitor is not fed to the stop rule, the runner returns to the snapshot with cfl_max x 0.7 (`set_config` first: it drops the compiled loop, a recompile of 1-8 s). Start cap = engine default (50 below Mach 1, 20 above). Events in `PointResult3.cfl_events`. Measured (Draft sector, start 50): 1-3 rollbacks, 180-370 iterations each, CD within 0.2 % of fixed cfl; wall -26..-58 % vs fixed 10 (phys3d campaign in DESIGN3D). Late blow-ups at cfl 20 start behind the base / at fin roots (see DESIGN3D item 41 and phys3d's paragraphs).

## 6. Memory
Persistent arrays (`GpuSolver3::memory_report`, `bench3 --mem`, `mem_stages`): G-F1 sector 677 376 cells f32 = 412 MiB (638 B/cell: Geo3 faces 73, solid-face masks 68, LGeo3 187, VGeo3 134, SST 123, state 32); df32 527 MiB. Process peak in the runner path: 607 MiB after `GpuRunner3::new`, +2437 MiB for the compiled loop = 3050 MiB (job `0-20261117-0-gpu3d-memst2`); Coarse example sector (4 fins) 3306 MiB with sub_iters 3, 2986 MiB with sub_iters 2; Draft 0.8-1.45 GiB.
Why 7.5 kB/cell is transient: each recorded plan keeps one buffer per materialised intermediate for its whole life (Luma `Plan.retired` is returned to the pool only when the plan is dropped; nothing is reused inside a recording); the step writes ~530 full-size arrays (250 in the block-assembly `Fused[50]` launches: face matrices FL/FR 2 x 25 per face per direction). **A second recorded plan in the same process costs another 2.4 GiB** (replay loop, eval loop, stage profiles): `bench3 --mem` shows 5570 MiB for that reason; the product path runs one compiled loop at a time (a limiter-phase switch replaces the first loop, +0). Drop a `GpuRunner3` before building the next in the same process.

## 7. Luma
Workspace pin: `luma = { git = "https://github.com/flipnoteman/Luma.git", rev = "5a592c4c53137ff93e4d0437818d47a24c1f3fbc" }` (on origin/main; has `compile_until_all`, shape-independent kernels). On the recorded-loop path it is 5-9 % faster than crates.io 0.4.1 (sector f32 10.9 vs 11.95 ms). It builds on the remote 3080 worker (rustc 1.100, needs GitHub): job `0-20261117-150217-21533` built Luma from the rev and ran s5b 5/5. Luma main spills to host above 80 % of device memory instead of failing (slow, not fatal). Do not patch Luma from here.

## 8. gpuq
`tools/gpuq/gpuq submit --host local|remote|any --name N [--prio 0-9] --timeout S -- CMD`; `wait ID`, `status`; logs `~/.cache/bowshock-gpuq/done/<id>/log`. GPU runs only through it.
- Priority = id prefix `N-`; 0-2 before unprefixed, 3-9 after. **Prio 0 only for jobs under about 2 minutes** (the director's rule); long jobs prio 1+.
- Fairness (`efa4704f54`): within a priority class the worktree (job `repo`) served longest ago goes first; `$GPUQ_DIR/served`. **The workers run the main checkout's copy of the script**: a change in a worktree takes effect only when merged there and the workers restarted (done 17:46 local / 16:56 remote on 2026-11-17).
- Per-worktree build dirs: each job copies its snapshot into a stable run dir `run/wt-<key>` and builds in `target-local/wt-<key>`; the job log prints `Compiling bowshock-* (...)`: check it names the run dir of its own worktree.
- Staleness hazard 1 (stale target artifacts): `rsync -a` snapshots kept old mtimes, so cargo's freshness check ran stale code from a shared target dir; fixed by `-rlpgoD --checksum` into the run dir (proved on a tiny crate: code A vs code B).
- Staleness hazard 2 (stale snapshots by id): a job's snapshot is taken at submit time; code edited after `submit` is not in it, and renaming a queued id (priority tricks) must move the snap dir too. Never rename ids to jump the queue.
- Memory per process: `tools/gpuq/gpumem --label X -- CMD` (nvidia-smi per-pid peak).

## 9. Measured numbers (commit / job)
- RANS wedge 19 032 cells, 3090: 0.60 ms/it f32, 1.08 df32 (`0-20261117-102542-25081`, after `09a8c0b`); stage table `0-20261117-102720-3688`. Sector 677 376 cells step 10.9 ms f32 (90 launches), 15.9 df32.
- Ten-Mach RANS wedge table (G-V4, f32, 2D stop rule): 14.2 s, 20.4k iterations, all 10 PASS (`0-20261117-0-gpu3d-26785`; 2D facade 18 s).
- G-F1 sector (ANF 4 fins, 677 k): M0.5 7100 its 89.6 s (sub 3) / 6800 its 59.1 s (sub 2), M2.0 6940 its 78.7 s / 7000 its 57.5 s; fin friction ratio 0.9194 (Coarse), 0.9191 (64 chordwise cells), 0.9278 (Fine, y+ 1): KNOWN line (DESIGN3D item 41; jobs `1-20261117-162824-5005`, `-30463`, remote gate `1-20261117-155237-32257`).
- GPU test suite (bowshock-gpu-cases, 57 tests) green on the pinned Luma: `0-20261117-0-gpu3d-suite2`.
- Wall treatment rough GPU vs CPU: fin friction identical to 4 digits, G-F1 400 its (`/tmp` logs of finned_rough.sh; M0.5 0.014611 vs 0.01461, M2 0.01037 vs 0.01037).
- **Example-rocket numbers on the 4-fin helper (WRONG, the example has 3 fins)**: Draft/Coarse ten-Mach tables (`...-t1-draft20`, `...-t2-coarse20`), chord 20/32, controller rows, rough/smooth rows, memory 2986/810 MiB. Body, base, forebody roughly right, fin friction/pressure 4/3 too high. 3-fin reruns: `prod3-draft`, `prod3-coarse`, `c3-chord`, `c3-rough` (see the final report for what finished).

### 9b. Example rocket, THREE fins, controller default, Coarse (job `1-20261117-192608-12615` = prod3-coarse, commit a641f8ee6f + 6d009f5188, 3090 f32, sector 191x140x32 = 855 680 cells, `tools/finned_table.sh coarse`, sub_iters 2, start cap 50 below Mach 1 / 20 above)
Total wall 1165 s (19.4 min) for the ten Mach numbers, all MeanStationary, peak 3118 MiB (subsonic) / 2986 MiB. Per Mach: iterations (+wasted), rollbacks -> final cap, wall s; CD = body fr + fin fr + forebody p + base + fin p:
- 0.1: 13030 (+82), 3 -> 17.15, 145.9 s; 0.43833 = 0.20605 + 0.03413 - 0.00056 + 0.10895 + 0.08977
- 0.5: 9960 (+58), 2 -> 24.5, 112.7 s; 0.41003 = 0.15402 + 0.02462 + 0.00255 + 0.11308 + 0.11575
- 0.8: 10490 (+94), 2 -> 24.5, 117.3 s; 0.41107 = 0.13806 + 0.02181 + 0.00300 + 0.12827 + 0.11993
- 0.9: 13340 (+107), 3 -> 17.15, 149.0 s; 0.42204 = 0.13317 + 0.02085 + 0.00699 + 0.15363 + 0.10740
- 1.0: 12520 (+0), 0, 124.4 s; 0.57103 = 0.12852 + 0.02017 + 0.05280 + 0.24286 + 0.12667
- 1.1: 11730, 0, 116.6 s; 0.58792 = 0.12386 + 0.01869 + 0.10496 + 0.21989 + 0.12052
- 1.3: 10270, 0, 102.4 s; 0.58499 = 0.11687 + 0.01739 + 0.11039 + 0.21358 + 0.12677
- 1.7: 8660, 0, 88.2 s; 0.53630 = 0.10420 + 0.01682 + 0.10526 + 0.18276 + 0.12726
- 2.3: 8600 (+16), 1 -> 14, 90.7 s; 0.45313 = 0.08780 + 0.01492 + 0.09808 + 0.14152 + 0.11081
- 3.0: 11710 (+39), 1 -> 14, 117.8 s; 0.38035 = 0.07238 + 0.01216 + 0.09210 + 0.10648 + 0.09723
Draft with three fins (prod3-draft `1-20261117-192608-14774`), the 3-fin chord 20 vs 32 rerun (c3-chord `1-20261117-192621-8782`) and the 3-fin rough rows (c3-rough `1-20261117-192622-4771`): were queued and had NOT run when this was written (only the 3-fin Coarse table above exists). The GPU gate jobs rc3 / rc4 / rc5 (solver3d's) never ran on the integration candidate 3ec6e84 (queue); the sections above are marked with their own snapshots, not that candidate.

## 10. Not ported / not verified (honest list)
- Euler/viscous diagonal line solve on viscous cases: refused (`from_host` returns Unsupported); the diagonal default diverges on the finned half domain (runner takes Block).
- Motor jet: ported (`set_jet`, `jet_fix_*`); G-J 11/11 PASS on the GPU, `motor_jet_wedge_matches_cpu` green after comparing only the cells the CPU holds (CPU zeroes jet cells after a step) and an f32 target of 5e-5 on the Euler explicit row; jet with `Block` + SST agreement is at the test's tolerances, not tighter.
- df32 viscous finned sectors: only the memory ledger and small agreement tests; the product and gates run f32.
- Half-domain / incidence gates on the GPU (G-L1, G-F5, G-S): runs happen on solver3d's and phys3d's jobs (rc4/rc5); the first runs at 4ff3ba7 diverged (cfl) or ran out of memory on the 3080 laptop; the controller was not in place then.
- Euler table Mach 0.5 row of G-V4 fails (limit cycle, stop rule; not a GPU issue).
- No batching of Mach numbers (estimate below); no grid sequencing; the controller has no cap recovery (cap only goes down).
- The 4-fin example-rocket numbers above were never rerun on 3 fins for chord 20 vs 32 (the DRAFT_CHORD_CELLS 32 decision rests on them).
- A rollback recompiles the loop (1-8 s on f32/df32 respectively; df32 first compile up to 28 s).

## 11. Open performance items, in order
1. **Buffer reuse inside a Luma recording** (Plan.retired not reused): the cheapest big cut, 2.4 GiB per plan; luma-perf has the question.
2. **Batching** for finless tables: ten Mach numbers as ten k layers of one array. The wedge step is latency-bound (line solves are 140-156 lines = one thread each, 34-67 us; elementwise ~5-10 us); estimate 0.66 -> ~1.2 ms per step for ten points (~5x). Needs per-layer free stream / cfl / residual norm / stop state, no flux between layers (existing k-plane treatment per layer), per-layer Re and jet, one grid for all (`Analysis::grid_mach`). Not built; the wedge table is already 14 s.
3. **sub_iters**: wedge 1 below Mach 1 (1.8x), finned sector 2 (24-32 % wall, CD +-0.07 %), 3 elsewhere; 1 is unstable on finned sectors and supersonic wedges.
4. **A better implicit scheme**: the 3-factor AF has a CFL limit between 30 and 40; phys3d's DDADI (stable at cfl 100 on the CPU) has a period-2 limit cycle unless omega <= 0.7 (no net gain); a cell-local CFL / positivity damping in the base-wake and fin-root cells is the suggested cheaper attack.
5. Fusion cuts (3 f32 / 5 df32) and `cache()` placement are not the bottleneck any more.

## 12. Not run at hand-off
- 3-fin Draft ten-Mach table (`prod3-draft`), 3-fin chord 20 vs 32 (`c3-chord`), 3-fin rough rows (`c3-rough`): cancelled in the queue. Consequently `DRAFT_CHORD_CELLS` 32 rests on the 4-fin comparison only, and the Draft-vs-Coarse statements in DESIGN3D item 41 are 4-fin numbers.
- The GPU gate set on the integration candidate 3ec6e84 (rc3/rc4/rc5): never ran. Last full GPU suite: 57 tests green on cfd-3d-gpu before the controller default, `0-20261117-0-gpu3d-suite2`; the controller default (6d009f5188) was checked by CPU unit tests (case3 / run3, 11/11) and by the 3-fin Coarse table above, not by the GPU suite or the gates.
