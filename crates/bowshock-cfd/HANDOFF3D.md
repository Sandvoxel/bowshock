# HANDOFF3D: the 3D viscous solver on `cfd-3d` (single entry point)

Written by the integration owner (`solver3d`) at hand-off, 2026-11-17 evening. Nothing is pushed. The detailed record is `DESIGN3D.md` (section L, items 21-42); the
per-session sections are `handoff/gpu.md` (gpu3d), `handoff/physics.md` (phys3d), `handoff/app.md` (app-integrate), `handoff/luma.md` (luma-perf), merged from their
branches when they were reported (a missing file means that session had not reported when this was written). Every number below carries its source (gpuq job id,
commit, test); "not re-measured" marks a number from an earlier tree.

## 0. Update 2026-11-18 (branch `perf5`, merged into `cfd-3d`): Luma v0.5.0, the full solve in about 4 minutes, the supersonic finned drag fixed

Sections 1-9 below are the 2026-11-17 hand-off and stay as written; where this section disagrees, this section is current. Detail: DESIGN3D "Solve time: from 41 minutes
to about 4" and items 43-44.

- **Luma pinned at v0.5.0** (`7fafed6`: CUDA through the driver's NVVM, graph-replayed gated loops, shared transient buffers). `gpu_name` asks Luma (`Context::device`).
  Sector step time is the same on CUDA and wgpu (8.5 ms); CUDA is slower on small grids (Luma issue #6). Device peak of the full solve 8.9 GiB (was 10.3).
- **Full Coarse solve of the example rocket** (10 sector points + 8 stability points, RTX 3090, f32, `engine3d_finned_timing`): 2 481 s (old pin, fixed cfl 10) /
  1 654 s (v0.5.0, controller) -> **257 s** on the merged tip (39 300 iterations, job `3-20261118-043226-29707`; 245 s before the TE fan). What did it, in order of effect:
  - DDADI (omega 1) with the implicit face dissipation x 1.7 at `cfl_max` 100 on every non-wedge viscous domain (`Case3::with_tier`, `DDADI_CFL_MAX`, `DDADI_DIS_BOOST`);
  - the extrapolated-drift stop rule (`StopReason::Settled`);
  - one stability angle (2 degrees; CN_alpha and x_cp from it);
  - the axisymmetric seed of every cold finned sector (`Analysis::seed_axisymmetric`, default on).
- **GPU scaling** (the same tree, grids and 38 300 iterations on both; jobs `1-20261118-024355-31072`, `2-20261118-025211-28376`): RTX 3080 Laptop 16 GB
  (48 SMs, 130 W, 384 GB/s) 490.5 s against the RTX 3090 (82 SMs, 1.8 GHz under load, 936 GB/s) 244.6 s: x2.01. Sector and stability steps go x2.06-2.10
  (8.38 -> 17.61 ms at Mach 0.5): the SM-count x clock ratio (about 2.0-2.1 if the laptop holds about 1.5 GHz; not measured), not the bandwidth ratio (x2.44), so
  the step is SM-throughput / latency bound at these sizes, not DRAM bound. The 20 k-cell seed wedges go only x1.3-1.45 (launch latency). With the measured step
  ratio x2.08, `T = S / throughput + C` over the two leaves C about 17 s of the 3090's 245 s unscaled (seed wedges about 21 s of wall, host grid build 4.6 s, upload
  5.6 s, partly hidden by the wedges' own scaling): a GPU with twice the 3090's SM throughput would take about 131 s (x1.87).
- **CFL controller** also rolls back on a collapsing state (density or pressure below 1e-3 of the free stream, a device reduction per chunk) and, on a repeated failure,
  goes back past the restored state (`ad152aa`): the unseeded Mach 1.3 sector at CFL 100 ended `Failed` without it.
- **Supersonic finned drag:** the 9-12 % shortfall of the Finner was a 3-10x axial cell-size jump right behind the flush fin trailing edge. A graded fan there
  (`be3c9b8`) gives G-F5 against the zero-cant free-flight singles of DREV-TM-9703 Table VIII (`anf::CD0`, re-sourced; pooled Table IX is `CD0_POOLED`, INFO):
  Mach 1.85 -0.7 %, 2.364 -3.7 %, 2.741 +1.3 %, 2.970 -1.2 % (Lift tier, job `1-20261118-035444-11012`). Refuted on the way: more cells across the fin thickness
  (`fin_cells_across`, opt-in, item 43). Measured: a finer leading edge (48 / 64 chord cells) adds +2 / +3 %; an isothermal wall at T_inf adds friction x1.16 / x1.35 at
  Mach 1.8 / 3.1 (Van Driest II), opt-in only.
- **Production-tier accuracy** (the product runs CD0 on the Coarse sector, the stability angle on the Lift half domain; DESIGN3D item 46): Coarse sector CD0 of the
  Finner against the zero-cant free flight **+4.6 % (Mach 0.766), -1.9 % (1.85), -4.4 % (2.364), +0.1 % (2.741), -2.7 % (2.970)**; the sting-mounted tunnel
  points +8 / +9 % (INFO). CN_alpha (Lift tier, G-L1): supersonic +3 to +10 %, x_cp within 0.25 cal everywhere (0.12 cal subsonic).
- **Grid convergence of the Finner sector** (branch `gridconv`, DESIGN3D item 47; `finned_table --refine S / --chord N / --drift-tol X`, `Case3::with_refinement`; 6 similar grids
  of 0.25-3.4 M cells, r 1.3-1.46 because r 1.5-2 needs more than the 3090's 24 GB): the Coarse CD0 is **not grid-converged**. The fin pressure (leading-edge face: 32 chord
  cells, 0.0094 d, against a face about 0.004 d tall) is 2.8 / 4.4 / 6.8 % of CD0 too low at Mach 0.766 / 1.85 / 2.741; forebody pressure, base, body friction and the
  trailing-edge face are converged to a few per cent, fin friction is 2-7 % high. The grid-converged model over-predicts free flight by **+5.6 / +2.6 / +6.5 %**, so the
  Coarse +4.6 / -1.9 / +0.1 % is a cancellation (stop rule +1.0 / +0.6 / +0.3, discretisation -2.0 / -5.0 / -6.6, model +5.6 / +2.6 / +6.5 points). The default stop
  (`drift_tol` 5e-3) is itself 0.3-1.0 % high. Do not tune the model to the Coarse CD0; 64 chord cells (+20 % cells, same iterations, `Settled`) bring it within 0.8 %
  of the converged value, but 96 or more chord cells stop on `MeanStationary` at Mach 0.766 / 1.85.
- **2026-11-19 (branch `lumabump`):**
  - Luma pinned at `231c986` (#9: in-recording reuse, exact-size sharing, `memory_stats`/`trim`). Trims around every trace and on a grid-shape change, a memory guard
    (`device_bytes3`, refuses with a clear error; `BOWSHOCK_GPU_MEM_GUARD=off`), CFL snapshots on the host. Full-solve device peak 8.9 -> 4.1 GiB; 6.8 M cells now fit
    on the 3090 (item 49; follow-up on Luma #9).
  - **Coarse and Fine take 64 fin chord cells** (`COARSE_CHORD_CELLS`; Draft and Lift keep 32): full solve 254.5 -> **266.6 s** on the 3090 (544 s on the 3080 Laptop),
    38 800 iterations. The Coarse CD0 is now the grid-converged model, +6.4 / +2.6 / +6.7 % over free flight at Mach 0.766 / 1.85 / 2.741 (fully turbulent).
  - Prescribed transition, opt-in (`Case3::with_transition`, item 50): a laminar nose run of 6-9 calibres closes the gap on the smooth Finner model; the default stays
    fully turbulent (painted, rail-buttoned rockets trip near the tip; it is the conservative side).
  - Tunnel like-for-like (item 51): G-L1 tunnel points at `Re_L` 4e6 with the 0.63 sting; slope definition, Re and sting move subsonic CN_alpha by only about -1.5 %.
  - G-S at 64 chord cells (item 52): 2 cells across the fin in both domains; Mach 0.8 is `FAIL KNOWN` (+2.3 / +4.6 %): the half domain's free fins carry an
    asymmetric, oscillating trailing-edge wake the symmetric sector cannot; Mach 2 agrees to 0.003 %.
  - **Warm start across Mach numbers (branch `warmsweep`, item 59):** `Analysis::warm_start` / `BOWSHOCK_WARM_START`, opt-in, default off. Neighbour field rescaled to the new Mach number (`engine/warm3.rs`), CFL kept, per-domain order and limits (`WarmTuning::of`). Subsonic sector 0.5-0.9 saves 13 % of its iterations and 12 % wall (CD -0.07 % at 0.8/0.9, -1.3 % at 0.5 from 0.1, hence the factor-2 neighbour limit); the supersonic points and the transonic half domain are not faster than the seeded/cold start (the first design was +2 100 it). Not recommended as default (item 59); no full run on the final settings.
- **Still open:**
  - subsonic CN_alpha: +13.4 % against the tunnel at Mach 0.79 in-plane (G-L1 FAIL, 13 % band), +11 % against free flight (±14-17 % in the data); at the steady-RANS /
    data limit (items 46, 51);
  - subsonic and supersonic CD0 +3-7 % fully turbulent on a smooth model (transition, item 50);
  - the Arcas supersonic fin increment (KNOWN: the test article had anchors, a lip and grit strips the model lacks).
- **Gates on `lumabump` with 64 chord cells** (local 3090 / remote 3080 Laptop): wedge 60 PASS; fins G-F1 and 14 G-F4 PASS, 4 G-F4 KNOWN (Arcas), G-S Mach 2 PASS,
  Mach 0.8 FAIL KNOWN; lift G-F5 10/10 PASS, G-L1 30 PASS and 1 FAIL (above); gpucases all pass; CPU workspace (without gpu-cases) 1576 pass; clippy and rustfmt clean.

## 1. State on one screen

**Works (verified on the candidate or its close ancestors, sources below):**
- One 3D code path for body, fin and base drag and for lift / centre of pressure: finless wedge (single azimuthal cell, 2 degrees), 4-fin sector, half domains at
  incidence; Euler and RANS (SST, wall-resolved or wall-function), CPU f64 and GPU (Luma, f32 default, df32 optional).
- Finless wedge reproduces the 2D solver to 0.1-0.3 % on the frozen reference (G-V3 CD 0.213279 vs 0.213002, friction +0.01 %, base +0.23 %; G-V4 RANS table all rows
  within 3 %); the Euler wedge reproduces the 2D Euler (G-W1 62 lines PASS on 562edd6).
- The sector is a valid zero-incidence shortcut: G-S sector vs half domain N = 4 / 3, Mach 0.8 / 2: -0.81 / -0.79 / -0.42 / -0.32 % (band 1 %), gpuq job
  0-20261117-151300-3912 (tree of cfd-3d 15:13, cfl 10 on every viscous grid).
- Motor jet (TP-2005 base pressure ratios G-J 11 lines PASS, 562edd6 CPU).

**Does not work / FAIL or KNOWN (evidence in section 4):** supersonic fin drag is below both references (Arcas increment -45 %, Finner CD0 -9..-12 %); Finner subsonic CD0
is +10..+12 % high (G-F5 Mach 0.5 / 0.7); Finner CN_alpha at Mach 0.79 is +13.3 / +13.8 % against a 13 % band (G-L1); G-F1 fin friction at Mach 0.5 is 0.919 of the flat
plate against 0.92 (KNOWN); the inviscid wake at Mach 0.5 cycles (not gated).

**Unverified on the final tree:** the GPU gate set. The shared 3090 queue did not run the rc3 / rc4 / rc5 gate jobs on the candidate in three hours (prio 1 behind
longer jobs of other sessions); they were cancelled at hand-off. `fixtures/release_candidate_gates.txt` therefore has one section per snapshot and marks what is
missing. CPU verification of the final tree is in section 9.

## 2. How to run it

- **Engine switch:** `BOWSHOCK_ENGINE=3d` (or `solver3`) selects the 3D engine in `bowshock_core::Analysis` (`analysis.rs`, `Engine::from_env`); the default is still
  `Engine::Legacy` (the 2D route). The facade tiers are `Tier::{Draft, Coarse, Fine, Lift}` (`engine/case3.rs`): **Draft** (opt-in quick tier: wall functions on body and
  fins, y+ 40, axial counts x0.6, domain 5 L / 3 L, 32 chord cells; Mach >= 0.8 with default levers, below 0.8 `Case3::draft_subsonic`), **Coarse** (table tier, wall
  resolved, fin stack y+ ~15), **Fine** (y+ ~1, 39 azimuth cells), **Lift** (half domain at incidence, fin stack y+ 60). The app's Quick button is `app-integrate`'s (see
  `handoff/app.md`; the Quick tier's subsonic bias is open, section 8).
- **CFL:** `Numerics3.cfl_control` (default on): the runners' rollback controller (`run3::CflControl`, floor `CFL_FLOOR` 5, `MAX_ROLLBACKS`) starts at the engine CFL
  (50 below Mach 1, 20 above) and rolls back x0.7 on a non-finite or > 1e5 residual; `Case3::without_cfl_control()` = controller off and the fixed cap of 10 on half domains
  and sectors (what pins numerics on the CPU in tests).
- **Gate tests:** `tools/release_gates.sh wedge|fingates|fins|lift|gpucases|cpu` prints the GATE / INFO lines of a group; `tools/collect_gates.py` builds the table.
  The tests are `#[ignore]`: `BOWSHOCK_WEDGE_BACKEND=gpu BOWSHOCK_FIN_TIER=coarse|lift cargo test --release -p bowshock-core --test fin_lift_gates -- --ignored --nocapture <name>`.
- **Studies:** `tests/arcas_study.rs` (`STUDY_MACH`, `STUDY_TIER`, `STUDY_BODY=anf`, `STUDY_LE`, `STUDY_Z0`, `STUDY_FIN_T/CHORD/DW`; prints `STUDY`, `STUDY-EDGES`,
  `STUDY-BASE`, `STUDY-FINFRICTION` lines), `tests/part_estimates.rs`, scripts `tools/{arcas_study,anf_1832,anf_le,part_study,finfric_study,gpu_mem_study}.sh`.
- **GPU queue (`tools/gpuq/gpuq`):** `gpuq submit --host local --prio N --name NAME --timeout S -- CMD`; **prio 0 on the local 3090 only for jobs under about 2 minutes**,
  longer jobs prio 1; the claim order alternates between worktrees inside a priority class; `gpuq status`, `gpuq log ID`; a status loop must test the words
  `queued|running`. **A job snapshots the repository at submission** (a script edited afterwards is not picked up, the code run is the snapshot's): this cost several
  re-submissions. Logs: `~/.cache/bowshock-gpuq/done/<id>/log`; lines starting `PERF ` are appended to `perf/ledger.jsonl` of the submitting repo.
  `tools/gpuq/gpumem` wraps a command and prints a `MEM {...}` peak line (process peak, not the desktop's 2.3 GiB baseline).

## 3. Branches and worktrees (nothing pushed)

- `cfd-3d` = integration (this branch). `cfd-3d-gpu` (gpu3d), `cfd-3d-phys` (phys3d), `cfd-3d-viz` (app, facade, viewer, Quick tier; app-integrate): merged into
  `cfd-3d` repeatedly, last merge at the time of writing in the log below. `cfd-3d-luma` and `cfd-3d-lumaperf` are the Luma side (luma-perf; Luma is pinned by git
  rev `5a592c4` in the workspace `Cargo.toml`).
- `ui` (main checkout `/home/sandvoxel/Documents/bowshock`) holds only the merge of `origin/ui`; the main checkout has an updated `tools/gpuq` (and `tools/gpuq/gpumem`)
  and the user's two uncommitted files (`crates/bowshock-core/src/analysis.rs`, `perf/ledger.jsonl`): do not touch them.
- Worktrees that can be deleted once their branch is merged and the sibling session is done: `bowshock-cfd-3d-gpu`, `-phys`, `-viz`, `-luma`, `-lumaperf`,
  `-lumaperf0`, `-lumaperf1` (detached), and the scratch `/tmp/solver3d_bis`, `/tmp/solver3d_dry`, `/tmp/gpu3d-turb`. The `exp/*`, `bowshock-ideas`, `-pdf`, `-remote`,
  `-ui-*` worktrees belong to earlier work, not to this project.

## 4. Gate table (candidate code `3ec6e84`; harness `562edd6`; per-section snapshots in `fixtures/release_candidate_gates.txt`)

| gate | PASS | FAIL | KNOWN | INFO | snapshot of the lines |
|---|---|---|---|---|---|
| G-W1 (wedge Euler vs frozen 2D) | 62 | 0 | 0 | 3 | 562edd6 CPU |
| G-W4 (cone, ogive, smoke) | 10 | 0 | 0 | 3 (smoke cases name their covering tests) | 562edd6 CPU |
| G-J (TP-2005 jets) | 11 | 0 | 0 | 0 | 562edd6 CPU |
| G-V3 (G3 body, CPU and GPU) | 8 | 0 | 0 | 2 | 0922310 GPU, job 0-20261117-173948-14374 |
| G-V4 (example table, Arcas fins off) | 29 | 0 | 0 | 30 | 0922310 GPU, same job (table-euler Mach >= 1.2 not re-run after the euler_wedge fix) |
| G-V5 (GPU vs CPU tiny cases) | 23 | 0 | 0 | 0 | 0922310 GPU, same job |
| G-F5 (Finner CD0, Lift) | 0 | 4 | 8 | 11 | 0922310 GPU, job 0-20261117-173949-12378 |
| G-L1 (Finner CN_alpha, x_cp, Lift) | 29 | 2 | 0 | 16 | same job |
| G-S (sector vs half) | 4 | 0 | 0 | 4 | job 0-20261117-151300-3912 |
| G-F1 (fin friction) | 1 | 1 (Mach 0.5, KNOWN in code since gpu d609c95) | | | job 0-20261117-154122-7481 (b9d3501) |
| G-F4 (Arcas fins on / off) | MISSING as GATE lines | | KNOWN markers in code (Mach 1.5 / 1.8 fins-on and increment) | | last measurements: job 0-20261117-164014-25405 |
| G-F2, G-F3 (Euler fins) | MISSING on the candidate | | | | meshes of all built-in profiles bit-identical to the earlier PASS runs (grid3 unit test) |
| bowshock-gpu-cases suite | last: 57 tests 0 failed on cfd-3d-gpu 73e91b4 (gpu3d's report, job suite2-gpucases); NOT RUN on the candidate | | | | |
| G-V1, G-V2, G-P1 | not harness checks: plates are `bowshock-cfd` `plate3::tests` (PASS in the candidate's nextest), the finned table is `examples/finned_table` | | | | |

**Every FAIL and KNOWN, measured / reference / band:**
- G-F5 KNOWN (`DESIGN3D-L-item-38`), Finner CD0, sourced section, Lift tier: Mach 1.832 0.5298 / 0.585 (-9.4 %), 2.375 0.4297 / 0.484 (-11.2 %), 2.718 0.3844 / 0.435
  (-11.6 %), 3.147 0.3390 / 0.373 (-9.1 %), band 8 % (both roll phases within 0.4 %). Coarse sector (job 0-20261117-164014-25405): 0.5146 / 0.4185 / 0.3761 / 0.3310.
  Reference now re-sourced (DESIGN3D item 44): zero-cant single fits 0.567 / 0.478 / 0.400 / 0.3805 at Mach 1.850 / 2.364 / 2.741 / 2.970 (the Mach 3.147 pooled value was cant 0/2/4). On the 128-chord Coarse sector Mach 1.832 reads 0.5333 (-5.9 % against 0.567), default 0.5148 (-9.2 %). Part by part the 3D parts match independent estimates and the solver is 8 % above their sum
  (DESIGN3D item 40): not a biased solver part; the missing physics is unidentified.
- G-F5 FAIL (plain), Mach 0.5 / 0.7: 0.5077 (between), 0.5055 (inplane) / 0.460 (+10.4 / +9.9 %); 0.5152 / 0.5105 / 0.460 (+12.0 / +11.0 %), band 8 %. Not investigated;
  phys3d's section may add a controller re-run.
- G-L1 FAIL, Finner CN_alpha at Mach 0.79: 18.43 (between) / 18.52 (inplane) vs 16.27 per rad, +13.3 / +13.8 % against 13 %. The same fin section change (0.1-chord bevel ->
  the sourced full-chord wedge) moved this point from +7.6 % to +13.3 % (the earlier run, phys3d); no explanation. The other 30 lines pass.
- G-F4 KNOWN (`DESIGN3D-L-item-37`), Arcas, Coarse sector, C_A without base: Mach 1.5 fins on 0.4169 / 0.505 (-17.4 %), fins off 0.3243 / 0.340 (-4.6 %, PASS), increment
  0.0926 / 0.165 (-44 %); Mach 1.8 fins on 0.4380 / ... see item 37 (gpu3d's run, -16 %); bands fins on 15 %, increment 30 %. Independent estimate of the increment (Ackeret wave
  + flat plate + anchors) 0.0909: the 3D agrees with it to 2 %; the report exceeds it by 0.074 / 0.038 / 0.026 / 0.009 at Mach 1.5 / 1.8 / 2.3 / 2.96.
- G-F1 Mach 0.5 (KNOWN): 0.013424 / 0.014593 = 0.9199 against [0.92, 1.12] (gpu3d: 0.9194 Coarse, 0.9278 Fine, 64 chord cells 0.9191: grid independent).
- Inviscid wake at Mach 0.5 (G-V4 `table-euler-M0.50`): printed INFO, not gated: the 2D reference stops on its iteration cap (10 000) and the wedge on the mean-stationary rule;
  a cycling wake has no steady CD (DESIGN3D item 8).

## 5. Measured performance and memory

- Finless RANS wedge: G-V3 2 900 iterations 2.7 s on the GPU (RTX 3090, job 0-20261117-173948-14374, 0922310), 11.8 s on the CPU; the 10-Mach RANS wedge table 14.2 s
  (gpu3d, cfd-3d-gpu 73e91b4).
- 4-fin Coarse sector (670 410 cells, f32): 52-100 s per point at 5 000-8 600 iterations (jobs 0-20261117-164014-25405, 0-20261117-190528-5343 family, 3090); Fine
  sector (968 k cells) 484 s for 12 051 iterations (job 0-20261117-122154-31351); Lift half-domain G-L1 + G-F5 4 311 s for the full Finner table (0922310).
- Device memory (gpumem, f32 / df32, process peak, desktop 2 327 MiB not counted; job solver3d-mem2 on 210a2fd): finless Coarse wedge 175 / 201 MiB; Euler sector 116 k
  323 / 389; Draft sector 156 k **810 / 1 179**; Coarse sector 670 k **2 922 / 4 347** MiB (4.4 kB per cell; persistent arrays 412 MiB by gpu3d's ledger, the rest is
  transient buffers of the compiled step, one set per recorded plan in Luma). A process that keeps two recorded plans alive adds ~2.4 GiB (gpu3d: bench3 --mem 5 570 MiB).
- From the brief and **not re-measured by me on the final tree** (see `handoff/app.md`, `handoff/gpu.md`): the app's full Coarse solve 41 min at fixed CFL 10 and the
  controller's measured saving; 10.3 GiB peak on the half domains; Draft numbers of the example rocket (DESIGN3D item 41).

## 6. The default is NOT flipped and the 2D solver is NOT deleted

**Flip** (app-integrate owns it; their list is in `handoff/app.md`): make `Engine::Solver3` the default in `Analysis` (`BOWSHOCK_ENGINE=legacy` stays as the escape), the
app's CFD table / flow viewer on `Field3`, `SOLVER_REV` 7. **Before the flip:** the GPU gate set on the flip candidate (section 1: it does not exist on one snapshot),
the app group, serde and doc tests (section 9).

**Deletion** (mine, after the flip; inventory in DESIGN3D items 31 and 42, dry-run scratch `/tmp/solver3d_dry` with `/tmp/solver3d_dry_apply.py`): about 17 400 lines. Steps:
(a) delete the 2D files of item 31 (cfd `solver, implicit, viscous, turbulence, multigrid, bc, gas, flux`, `gpu/{body,implicit,viscous,turbulence,control}`; core
`engine/{case,run,gpu,numerics}`, `field.rs`); (b) in `gpu/mod.rs` drop the 2D `Inner`/`GpuSolver` block and the five `include!`s, in `gpu/common.rs` drop the unused `concat` import,
`crate::sst::Viscous` in `gpu/mod.rs`; (c) `analysis.rs` 2D route (app-integrate), `lib.rs` `pub mod field`, `case3.rs` `Numerics3::from_2d` / `Case3::wedge_of`; (d) tests: the
2D parity tests listed in item 31(b), phys3d's `tests/viscous3.rs` 2D harness, gpu3d's `tests/wedge_gates/gpu.rs` `engine_case_of`, my `tests/wedge.rs`
`the_wedge_of_a_2d_rans_case_is_the_wedge_built_with_the_tier`; `bowshock-gpu-cases` 2D runners and tests; validate examples `bench, bench_viscous, cfl_scan, eval_loop,
luma_issues, perfcheck, ref2d`; (e) docs (item 42 list). Already done as separate green commits: every 3D module's dependency on a 2D module moved out (items 31, 42; last
`ee22d78`). **True before deleting:** the flip is in and CI-green; `ref2d.json` fixture header says it was generated before the deletion; the CPU wedge harness reads
only that file (it does).

## 7. Corrections a reader of earlier messages must know

- The Arcas flat-back fin section (DESIGN3D item 33) is **refuted** by the NASA source (TN D-4014 p. 4: trapezoidal **double-wedge** fins, Fig. 1(b)); `arcas_fin_spec` stays
  `DoubleWedge`; the G-F4 shortfall is therefore real and KNOWN, not a geometry error.
- The Finner fin section was an **assumed** 0.1-chord 21.8 degree bevel (G-F5 +20-27 % high, "no shortfall" in item 37's first text, which used an under-resolved Euler
  sector). It is now the sourced full-chord wedge with a 0.004 cal leading edge (DRDC TM 2002-136 read second hand, BRL 711, Abiza et al.; item 38); the Finner is
  **-9..-12 %**, not +20 %. The mesher's blunt-edge floor went 25 % -> 10 % of t/2 (no built-in `FinProfile` changes).
- gpu3d's 4-fin example helper: the example rocket has **three** fins; the 4 was copied from `grid_census` (cfd-3d-gpu a641f8e); finned example tables made with 4 fins are
  not the example rocket's.
- The G-W1 Arcas Mach 2 Euler failure and its fix (300 first-order iterations only below Mach 1.2; bisected to 5f89ed1) and the one_entry frozen bits that moved with the 8-column
  trailing-edge taper (CA of the pressure +3.9 % / +2.7 %).
- The 5.2-5.6 GiB device-memory figure was several recorded plans in one process; the product path is 2.9 GiB (f32 Coarse sector).
- **gpuq staleness hazards:** a job snapshots the tree at submission (script edits are not seen); a watcher loop must test `queued|running`; removing a queue entry is the
  only cancel; the "done" lines contain neither word; prio 0 starves everything else (2-minute rule).

## 8. Open problems, priority order

1. **CFL limit of the three-factor step and the controller** (phys3d, gpu3d: `handoff/physics.md`): the viscous half domain and sectors diverge above CFL ~10-35 depending on case;
   the 8-column trailing-edge taper fixed one cause (DESIGN3D phys3d items), a second limit sits in the wake behind the fin azimuths; the rollback controller makes the cap per case
   but costs iterations (settling time ~ 1/CFL).
2. **Angle-of-attack memory and time** (Lift half domain: 10.3 GiB peak per the brief; G-L1 + G-F5 4 311 s on the 3090).
3. **Supersonic fin-drag shortfall against two references** (Arcas -45 % increment, Finner -9..-12 %): independent estimates give the 3D within 8 %; the references exceed them.
   Untested: the SST compressibility correction (research item 9: Sarkar / Wilcox dilatation dissipation as a flag in `turbulence3.rs` and the GPU kernels, run on the finless wedge and the Finner),
   the 2 degree cant of the tested Finner fins (about 2-3 % of CD0), open fin slots / strips of the Arcas model, the Finner table's provenance (resolved: pooled over cant 0/2/4, DESIGN3D item 44; the zero-cant fits are lower, the chord resolution of the fin leading edge is worth +3.6 % at Mach 1.832).
4. **Subsonic CN_alpha high** (Finner Mach 0.79 +13.3 %; the earlier +8 % mean) and Finner subsonic CD0 +10..+12 %.
5. **Quick (Draft) tier subsonic bias:** Mach 0.5 Draft with default levers -5.1 %, +2.3 % with the safe variant (`Case3::draft_subsonic`); supersonic +0.6 % (item 32).
6. **Fully turbulent walls / transition:** all walls are fully turbulent (SST), fin friction on a 1.3e6 chord may be transitional in the references.
7. Pods, lugs, fin cant, several fin sets: not built (`FinProfile`, `Fins3` carry one fin set; S11 / S12 cover fins anywhere on the aft body and symmetry classes only).
8. **Luma buffer reuse** (luma-perf: `handoff/luma.md`): one set of transient buffers per recorded plan (~2.4 GiB on the Coarse sector); reuse inside a recording would cut ~2 GiB.

## 9. What was and was not verified on the final sha

- **Verified on the final sha (push):** `cargo check --workspace --all-targets` clean, `cargo +stable fmt --all --check` clean, `cargo test --release -p bowshock-core --test wedge_gates` self-tests 5 passed (including `a_gate_without_a_runner_method_fails`). Root `Cargo.toml` pins Luma by git rev 5a592c4 (no path patch). Nothing else was re-run.
- **Verified on ancestors (not re-run on the final sha):** on `3ec6e84` family (`/tmp/solver3d_ci2.log`, before the last test-harness / docs / phys merge): `cargo fmt --check`, `cargo clippy
  --workspace --all-targets -D warnings`, app group 1188 tests (one run showed 1 failure that did not reproduce, test not identified), cfd / reference / cases / core release
  345 tests (the one failure, `the_wedge_of_a_2d_rans_case_is_the_wedge_built_with_the_tier`, fixed in 3ec6e84 and re-run green together with the lib tests), serde_roundtrip, `subsonic_smoke_with_base`, doc tests, `cargo doc`, the three CI drive smokes (ancestor `9a2b18e` / `0922310`).
  The CPU verification chain on the last merge (phys handoff, docs only) was **cancelled** at hand-off.
- **Not run on the candidate:** the GPU gate groups (queue), G-F2 / G-F3, G-F4 as GATE lines, the gpu-cases suite (section 4).
- **Not merged:** `cfd-3d-lumaperf` 8d5fd93 (conflicts in `gpu/body3.rs`, `gpu/rans3.rs`; see the stub `handoff/luma.md`). Merged: cfd-3d-gpu 76172b9 (`handoff/gpu.md`), cfd-3d-phys 84b2333 (`handoff/physics.md`), cfd-3d-viz a7ff98a (`handoff/app.md`; app-integrate ran fmt, clippy, serde, remote/worker/validate/model 213, doc 20, app 796 on its own merge of f271e83: not re-run here).
