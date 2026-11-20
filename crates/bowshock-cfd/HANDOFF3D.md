# HANDOFF3D: the 3D viscous solver on `cfd-3d` (single entry point)

Written by the integration owner (`solver3d`) at hand-off, 2026-11-17 evening. Nothing is pushed. The detailed record is `DESIGN3D.md` (section L, items 21-42); the
per-session sections are `handoff/gpu.md` (gpu3d), `handoff/physics.md` (phys3d), `handoff/app.md` (app-integrate), `handoff/luma.md` (luma-perf), merged from their
branches when they were reported (a missing file means that session had not reported when this was written). Every number below carries its source (gpuq job id,
commit, test); "not re-measured" marks a number from an earlier tree.

## 0. Current state (integration branch `integrate` on `cfd-3d` c3cb0d8, 2026-11-19)

Sections 1-9 below are the 2026-11-17 hand-off and stay as written; where this section disagrees, this section is current. Detail: DESIGN3D items 43-64. Integrated, in order: `lumapin2` (53), `draftfirst` (57), `validate2` (55), `subdrag` / `boattail` (62, 64), `flightval` (56, 57b, 57c), `product`, `compcorr` (54), `warmsweep` (59), `wakedt` (60), `motoron` (58). `multigrid` is parked on its branch.

**Product defaults.**
- `Engine::DEFAULT = Solver3` (`Analysis::legacy(&body)` / `BOWSHOCK_ENGINE=legacy` ask for the old engine); no GPU: the app falls back to Legacy with a note. OpenRocket export (Analyze > Export > `OpenRocket CFD`, drag and stability CSV).
- Luma pinned at `620c688` (item 53), one trim on a grid-shape change, device-memory guard as a staircase; Coarse and Fine take 64 fin chord cells, the Lift half domain 2 cells across the fin; rounded leading edges resolved by 4 cells (`LE_RADIUS_CELLS`), sweep zone, positivity floor in the GPU damping, collapsed state is `Failed` (items 57b, 57c); diameter steps (`AftSpec::Step`).
- Grid sizing on the shared grid (item 57): drag nodes `max(own Mach, 1.0)`, angle nodes their own Mach; draft first (a missing or stale table is solved on the Draft grid, drag only, then on Coarse; `BOWSHOCK_DRAFT_FIRST=off`).
- **Uncertainty bands** (`bowshock_core::uncertainty`, one documented table; a model that reads high puts the truth below the value): CD: subsonic (< Mach 0.95) the model reads high by up to 20 % for a boattailed body or one whose skin friction is >= 0.6 of the drag (items 55 G-X4/G-X6, 62, 64), 6.7 % for a flat-based fin-dominated one, and low by up to 5 %; transonic (0.95-1.2) high 6 % / low 17 % (item 55: Arcas fins on, AEDC); supersonic high 7 % / low 10 %; plus stop rule 1 %, grid 0.8 % (Draft: +6.8 % high side), wall temperature up to +6 % from Mach 1.5, reference scatter 1 %. CN_alpha: grid 5 % (item 57) plus model bias, subsonic high 21 %, supersonic high 13 % / low 1 %. x_cp 0.3 cal both sides; the margin lower bound subtracts 0.3 cal. `CfdTable::boattail` carries the body feature; Analyze shows `Cd band`, `CNα band`, `Margin lower bound`, `Not meshed: lugs, buttons` and, with a boattail below Mach 0.95, `Boattail: subsonic drag may read high`.

**Opt-in, all default off:** solved jet / power-on base drag (`BOWSHOCK_POWER_ON`, `cfd::POWER_ON`, item 58: no power-on node is solved by default, the flight keeps the exit-area correction; the solved effect is x2 on the base subsonic, the opposite sign, TP-2005 fails); warm start across Mach numbers (`BOWSHOCK_WARM_START`, item 59; a warm chain uses one grid sized for the table's highest Mach); k-rate cap `ImplicitParams::dt_k_cap` (item 60); SST compressibility corrections (item 54; the GPU-vs-CPU agreement tests passed on the 3080 Laptop, job `0-20261119-172844-24898`); prescribed transition (50); isothermal walls (45).

**Accuracy** (Coarse sector CD0 / Lift half domain, GPU f32, items 46, 47, 51, 55): Basic Finner against free flight +4.6 / -1.9 / -4.4 / +0.1 / -2.7 % at Mach 0.766 / 1.85 / 2.364 / 2.741 / 2.970 (grid-converged model +5.6 / +2.6 / +6.7 %, fully turbulent smooth), transonic +1.8 / +5.3 % at Mach 1.056 / 1.332; CN_alpha supersonic +0..+13 %, subsonic +11..+21 %; x_cp within 0.3 cal. Finless Arcas long model subsonic +16 % (boattail pressure drag of the base-wake solve, items 62, 64); Arcas fins on transonic -17 %; Yuska C_A0 +19..+34 % (KNOWN, item 55).

**Gate tally of this integration** (3090 local, 3080 Laptop remote; `tools/release_gates.sh`): wedge 60 PASS; fins G-F1 + 14 G-F4 PASS, 4 G-F4 KNOWN (Arcas Mach 1.5 / 1.8), G-S Mach 2 PASS (2), Mach 0.8 FAIL KNOWN (2), G-F5s 7 PASS (new: the Coarse sector against the Finner CD0, +7.0 / +1.8 / +5.3 / +3.2 / +1.7 / +7.0 / +4.5 % at Mach 0.766 / 1.056 / 1.332 / 1.85 / 2.364 / 2.741 / 2.970, band 8 %); lift G-F5 14 PASS (10 before: Mach 1.056 and 1.332 added), G-L1 30 PASS + 1 FAIL (Mach 0.79 in plane, +13.4 % against 13 %, pre-existing); gpucases all pass; xsector (local, job `0-20261119-214911-22492`): G-X4 4 PASS + 2 KNOWN (fins off Mach 0.6 / 0.8, +16.8 / +16.0 %, as item 55), G-X6 6 PASS + 2 KNOWN (Mach 0.80 / 0.85), **G-X5 16 KNOWN, of which 7 are garbage** (Yuska A Mach 2.09 / 2.38, B Mach 2.00 / 2.38 / 2.78, C Mach 2.00 read C_A0 58 / -220 / -778 / -1105 / -159 / -811: collapsed trailing-edge-face pressures, stop `non-finite residual` or a `settled` collapsed state; item 55 measured +19..+34 % at all 16 points before the integration) **[REGRESSION, not diagnosed: suspects the positivity floor of the GPU damping (item 57c), the round-edge / sweep-zone grid changes (57b) or the 64-chord-cell / sizing changes, all merged after item 55's runs; bisect with `release_gates.sh xsector` / `gate_x5` on `validate2` against `flightval`]**; the gate marks them KNOWN, so the tally hides it. xlift (remote, job `0-20261119-214912-25588`, 59 min): G-X1 13 PASS + 6 KNOWN (x_cp Mach 0.6-1.2, CN_alpha Mach 0.8), G-X2 37 PASS + 3 KNOWN (Yuska A x_cp Mach 3.47 both rolls, C CN_alpha Mach 2.38), G-X3 16 PASS + 3 KNOWN (CN_alpha Mach 0.85 / 0.92 / 0.94): as item 55. CPU workspace without gpu-cases: 1635 PASS, clippy `-D warnings` and rustfmt clean.

**Timings.** Full Coarse solve of the example rocket through the app path (3090, f32, shared grid with the item 57 sizing, 10 drag + 8 angle points, 28 runs, no power-on node): **289.8 s**, 42 750 iterations, process peak 3.6 GiB (`0-20261119-214911-22492`, idle CPU; 314.5 s under heavy CPU load before). 3080 Laptop: not re-measured here (924 s on the shared grid before the sizing, item 57).

**Fixed costs** (item 69, branch `fixedcost`): the next run's host solver and GPU solver are built on a thread while the GPU iterates (`Analysis::hint_next`, `BOWSHOCK_NO_FIXEDCOST=1` / `BOWSHOCK_NO_RUNNER_AHEAD=1` switch it off), wall distance kept with the cached grid, snapshots into recycled buffers; A/B on the 3090 in one job (`0-20261120-000013-24689`): **286.9 s -> 266.7 s**, fixed costs 35.5 -> 17.2 s, results identical, process peak 3.6 -> 5.5 GiB; the seed cap does not pay (item 69).
**Stop rule and the limit cycle** (item 68, `periodstop`, 2026-11-19): the subsonic base wake's cycle (period 11-28 iterations) is 0.02-0.10 % in CA / CN, 4-10 times below the stop tolerance: the stop does not wait for it but for the geometric tail of the base-wake adjustment (0.93 per 100 iterations), and the original drift rule already stops at 0.2-0.45 % error, about where an oracle reaches 0.3-0.5 %. `Numerics3::drift_period_aware` (OPT-IN, default off; `BOWSHOCK_DRIFT_PERIODS=1` on `Numerics3::viscous`) measures the period, ends blocks after whole periods and tests the extrapolated limit of consecutive blocks: worst error over block phases on the recorded histories 1.39 -> 0.38 %, but 43 350 against 42 750 iterations on the full solve (no trend per point, +-500), CD / CN per point within 0.09 / 0.18 % except Mach 0.1 CD -0.48 %. The recorded per-iteration histories (sector and half domain, Mach 0.5-1.0, 14 000 iterations, columns iteration CA CN CN*x_cp CFL) are in `~/Documents/periodstop_histories.tgz` (outside the repository); the GPU runner's 30-40 iterations past the stop (about 7 s of 290) is the one saving seen and belongs to `gpu3.rs`.

**Gate harness guard.** A gate under a `gpuq` job refuses the CPU backend (`GPUQ_JOB` set and `BOWSHOCK_WEDGE_BACKEND` not `gpu`: panic with the fix; `fin_gates/common.rs`, `wedge_gates.rs`; the `cpu` group of `release_gates.sh` unsets it). `gpuq` must export `GPUQ_JOB=$id` in the job's environment (`run_job`: local `GPUQ_JOB="$id" CARGO_TARGET_DIR=...`, remote `GPUQ_JOB=$id PATH=...`).

**Flight-data validation** (item 56): 3D tables exist for 2 of 6 rockets (Don't Debate This +7.7 %, Rabia +7.0 %), legacy for all six (mean +12.7 %), build-up +11.9 %, RASAero II +3.1 %; the 3D tables of Byrum, Torrent, Cancer Descending and Ion Drive were not produced (subsonic points that do not settle, no table stored).
- **Flight robustness** (item 70, Coarse app path, merged tree): Ion Drive 9/10 drag + 8/8 angle points converge, Torrent 8/10 + 8/8 (was 22 300 iterations at Mach 0.8, now 1 270), Don't Debate This 10/10 drag (angle Mach 0.8 falls back after 52 820 iterations); Byrum (68 deg SQUARE 0.25 in: Coarse vacuum at the swept flat edge; thinner fin or 4 cells across the fin settle) and Cancer Descending (61 deg: unsteady subsonic wake, 8 600-23 200 iterations then `Fallback`) do not converge; Rabia supersonic per item 66. The 3D tables of this run were lost (relative `FLIGHTVAL_OUT`): regenerate with an absolute path before filling item 56's 3D column.

**Positivity floor reverted (ArcasLL, 2026-11-20, DESIGN3D 57c amended / item 67):** the Yuska G-X5 collapse (A / B Mach 2.38 `C_A0` -220 / -1105) was the `damping3` floor of item 57c (it freezes a cell that would fall under 1e-2 of the free stream; floor off gives the pre-integration 0.4848 / 0.4794 / 0.4476 bit for bit; the mesher is innocent), not the sweep zone / round-edge refinement. `damping3` is the plain positivity rule again, the guard of `quality3` is `unphysical_cell` (not positive, or below 1e-3 of the free stream at local Mach above 20). Consequence: supersonic Rabia (Draft Mach 2.3: 7 700 it, 8 rollbacks, Failed) collapses again and falls back until the swept-leading-edge mesh fix; no variant of the damping (relative limit, floor with fallback, other floors) keeps both Yuska and Rabia.

**Swept thick leading edges (item 66, sweptle):** the example rocket's grids are bit-identical; the fold of Cancer Descending / Torrent and the 2-10 degree slivers above the tip of Byrum are fixed in the mesher (`ramp_need`, `FADE_TRIGGER`); Rabia Coarse M 1.3 / 2.3 settle in about 1 200 iterations only with the opt-in `Grid3Params::fin_le_rows` (rounded fins keep the root's first chord cell down the span), which breaks its Mach 0.8 point, so the default grid still fails Rabia M 1.3 (non-finite) and Draft M 2.3; no cell size settles all three.

**Open:** Yuska G-X5 collapsed states after the integration (above); subsonic CN_alpha +11..+21 % high; fully turbulent CD0 +3-7 %; Arcas supersonic fin increment and Yuska C_A0 (KNOWN); boattail pressure drag subsonic reads high and the GPU Euler + sting divergence is localised, not fixed (item 64); Coarse supersonic Rabia and the subsonic Byrum / Torrent points (57b, 57c); full solve on 6 GB cards (the guard refuses the shared angle nodes); solved jet unvalidated (TP-2005).

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

- **Engine switch:** `BOWSHOCK_ENGINE=3d` (or `solver3`) selects the 3D engine in `bowshock_core::Analysis` (`analysis.rs`, `Engine::from_env`); `Engine::DEFAULT` is `Solver3`
  since the product flip (section 0; a machine without a GPU falls back to `Engine::Legacy` with a note, `BOWSHOCK_ENGINE=legacy` forces it). The facade tiers are `Tier::{Draft, Coarse, Fine, Lift}` (`engine/case3.rs`): **Draft** (opt-in quick tier: wall functions on body and
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
- **2026-11-19 (SlowRocket, DESIGN3D item 57b, commit after 2daff43):** Rabia subsonic is fixed (Coarse M 0.8 570 iterations / 19 s instead of 19 250 / 276 s; grid3 `ZONE_REACH`, `LE_RADIUS_CELLS`, `quality3` fails collapsed states). NOT fixed: Rabia M >= 1.3 (Coarse M 1.3 NonFinite after 8 rollbacks; Draft M 2.3 Failed). Not run: the six-flight validation table (needs the supersonic points), Coarse example C_D per Mach before/after (Draft M 0.8 identical). `tools/sr_final.sh` is the confirming batch; `tests/diag56.rs` the diagnostic (grid quality `DIAG_GRID`, `DIAG_SNAP`, cell scan `DIAG_CELL`, numerics knobs).
- **2026-11-19 (SlowRocket, item 57c):** the GPU update damping has a positivity floor (1e-2 of the free stream): Draft Rabia M 0.8 / 1.3 / 2.3 settle in 770 / 670 / 870 iterations, example Draft unchanged. Coarse supersonic Rabia is still bad (stalls / clipped cell; `quality3` now fails it). Jobs `0-20261119-201315-12026` (gates lift + fins, 3D tables of rabia, torrent, dont-debate-this) and `0-20261119-201315-25233` (legacy tables of all six, 3D tables of the other three) hold the validation numbers: copy their `out/*.json` into `assets/validation/flights/tables/` and print the table (item 57c). The G-F5 / G-L1 / G-F1 / G-F4 / G-S lines of the first job are the check that the floor left the gates unchanged: compare with item 52 / 38 values.
