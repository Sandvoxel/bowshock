# Hand-off: the facade, the route, the app (branch `cfd-3d-viz`, owner `app-integrate`)

Everything below was true at the final commit named in the last section. Numbers come from gpuq jobs whose ids are given; nothing here is estimated unless marked `[INFERENCE]`.

## 1. The facade (`bowshock-core`, `analysis.rs`, `analysis/solver3.rs`)

* `Analysis::new(&BodySpec)` then builders (`resolution`, `backend`, `fins`, `viscous(FlightConditions)`, `roughness`, `drag_tolerance`, `grid_mach`, `keep_fields`, `warm_start`, `max_iterations`, `on_progress`, `on_field3`) and `run(&[impl Into<Condition>]) -> Polar`.
* `Condition { mach, alpha_deg }` (`From<f64>` gives zero incidence). Points come back in the order asked. Zero incidence with fins is the sector, with no fins the wedge; an angle of attack is the base-wake half domain.
* `Engine { Legacy, Solver3 }`. `Engine::DEFAULT` (`analysis.rs`) is `Legacy`; `BOWSHOCK_ENGINE=3d|solver3|legacy` overrides it. The engine travels in `AnalysisSpec` (serde default = legacy).
* `Resolution { Draft, Coarse, Medium, Fine }`. Draft is the 3D engine's `Tier::draft` (wall functions on body and fins, about a quarter of the cells) at zero incidence with the viscous terms; everywhere else (Euler, the angle points, the legacy engine) it behaves as Coarse. Coarse is the default tier; Medium/Fine are Refine.
* Coefficients are on the reference area (`area_ratio = (d_nose/d_ref)^2`). At an angle of attack `Drag` holds the axial-force split and its total is `Forces.ca`.
* `Drag { forebody_pressure, friction: Option, base, base_modelled, fins: Option<FinDrag{friction,pressure}>, total }`: forebody and base are pressure only; `friction` is body friction incl. base friction; `total` = sum of the five. `friction` is `None` for Euler.
* `PointDetail` NaN rules: `cd_nose` NaN at an angle; `cd_base_solver`, `cp_base` NaN without a base wake; `y_plus_max` NaN for Euler (the GPU leaves fin-tip faces out, as the CPU); `cd_friction` 0 for Euler. New: `cfl_limit: Option<f64>` (the CFL limit of the run that gave the point; `None` legacy) and `rollbacks: Vec<Rollback{iteration, restored_to, cfl_before, cfl_after}>`; `attempts()` = 1 + rollbacks.
* `Warning::RestartedAtLowerCfl { rollbacks, cfl_from, cfl_to }` is pushed when the solver's CFL controller (`CflControl`, both runners) took the run back.
* Hidden builders: `fin_outline(FinOutline::{Trapezoid, Ellipse, Polygon})` (in `AnalysisSpec`, serde default) and `mesh_check()` (builds the coarse Euler sector; the app calls it per fin set so a mesher refusal becomes the plan's reason text).
* `Field3` extras from the viscous run: `eddy_ratio` per cell (mu_t/mu), `WallFace.cf` (skin-friction coefficient vector over the free-stream q), CPU from `rans()`/`wall_tractions`, GPU from `turb_state` + `viscous_flux`. Euler: `eddy_ratio` empty, `cf` zero.
* Backend: `Backend::Auto` tries the GPU and falls back to the CPU with a note; the CPU solves viscous too. A motor request under `Solver3` is `Error::Unsupported` (the jet patch is not done). The facade no longer retries a NonFinite run itself: the CFL controller in the runners does (see section 5).
* Tests: `crates/bowshock-core/tests/solver3.rs` (facade, viscous CPU wedge/sector/half domain with every `Drag` field finite, outlines, Draft, rollbacks), `tests/wedge_gates/facade.rs` (gates through the facade; `BOWSHOCK_WEDGE_BACKEND=facade BOWSHOCK_FACADE_BACKEND=cpu|gpu`, `BOWSHOCK_FACADE_ONLY=<id>`).

## 2. The route and THE FLIP

With `BOWSHOCK_ENGINE=3d` the whole app already runs on the 3D engine. The flip, in order:

1. Preconditions: the GPU fin gates pass; the Jet patch exists (or the app keeps refusing motors under the 3D engine and says so); `Engine::Solver3` has been the one the release gate (`tools/release_gates.sh`) runs.
2. `Engine::DEFAULT = Engine::Solver3` (`crates/bowshock-core/src/analysis.rs`). This is the only line that flips the default; `from_env` still lets `legacy` through.
3. App constants (`crates/bowshock/src/analyze/cfd/plan.rs`): `SOLVER_REV_3D = 1001` and `LIFT_REV_3D = 1001` take the place of `SOLVER_REV` / `LIFT_REV`; bump them whenever the numbers change meaning (a table stored before reads as stale).
4. Then delete the legacy path, in this order: `Words(false)` and the `Words` bool (`cfd/wording.rs`; the guard test lists the words a screen string may not use); `solves_3d()` and `shared_grid()` (`cfd/run.rs`), `FinSpec::validate_for(engine)` (legacy 3..=8 fins, 3D 1..=16), the legacy `SOLVER_REV`/`LIFT_REV`, `fin_outline`'s `three_d()` branches in `cfd/plan.rs`, the 2D axisymmetric solver and its facade route, `flow_view/axi.rs` (the one adapter `field3_from_2d`; nothing else names the 2D `Field`), the 2D snapshots (`FieldRef::from_2d`, `on_field`), `crates/bowshock-core/src/engine` 2D files and `bowshock-cases` 2D builders. The route comment at the top of `analysis.rs` lists what is shared.
5. `BOWSHOCK_CFD_RESTRICT=lo,hi` (`analyze/compute.rs`) is a diagnostic hook, keep or delete.

## 3. Remote protocol 3 and the Luma pin

* `bowshock-remote::proto::PROTOCOL = 3`: the spec carries the engine (an older worker would silently solve with the legacy engine, so it is refused). `AnalysisSpec`, `PolarPoint`/`PointDetail` (`cfl_limit`, `rollbacks`), `Warning` and `CfdPoint` are all serde-additive (`#[serde(default)]`).
* Workers must be built from the same source key and the same Luma: the workspace pins `luma = { git = "https://github.com/flipnoteman/Luma.git", rev = "5a592c4c53137ff93e4d0437818d47a24c1f3fbc" }` (root `Cargo.toml`). A worker on another Luma revision traces the loop differently (the 2-hour blow-up of the old rev was a host array clear that made Luma re-trace every iteration).
* Round trip: `bowshock-remote/tests/loopback.rs` (compares JSON, with `residual_drop_orders` rounded to 9 digits).

## 4. The model's additive fields (`bowshock-model/src/cfd.rs`)

`CfdGrid::Draft` (new variant, shown as "draft grid"), `CfdPoint.restart: Option<CfdRestart{rollbacks, cfl_from, cfl_to}>` (`skip_serializing_if` none; old tables read as no restart), `CfdRestart` exported from `lib.rs`. Nothing else changed; a table stored before reads fine.

## 5. The app

* One solve: the drag nodes and the angle nodes run in `solve_sized` (`cfd/run.rs`); `node_analysis(job, NodeKind, session, altitude, grid_mach, shared_grid)` builds the `Analysis` of both kinds (viscous, roughness from the plan, `drag_tolerance` for drag nodes, `grid_mach` shared grid, the fin outline).
* Buttons (`analyze/view.rs`): Re-solve = Coarse (default), Refine = Medium (3D engine and legacy), Quick = Draft (`Message::Quick`, 3D engine only, tooltip `DRAFT_TOOLTIP`). Grid labels "coarse / draft / medium grid".
* `wording.rs` owns every string that names the method (`Words(bool)`); its guard test (`only wording.rs names the solver`) scans the screen strings for words such as "axisymmetric", "Euler", "3D solve". New: `restart_note`, `restart_tag`, `DRAFT_TOOLTIP`.
* Plan (`cfd/plan.rs`): `Plan.outline` (elliptical fin -> `FinOutline::Ellipse`, freeform fin -> `Polygon` found by span and thickness in the document; the equivalent-trapezoid notes appear only under the legacy engine); mesher refusals become the reason text of that fin set (left to the build-up); the outline is in the geometry hash only when not a trapezoid.
* Viewer (`analyze/flow_view/*`): the contract is `Field3` (`bowshock-core/src/field3.rs`; spec in `flow_view/DESIGN3D.md`: nose-diameter solver units scaled to metres, +z is the lee side, tiles carry a full 3x3 matrix, wall faces in `wall_faces` order). The 2D side-view mode is drawn from the same `Field3` (two meridian sheets); the axisymmetric solver's `Field` reaches it through `flow_view/axi.rs::field3_from_2d` (delete with the 2D solver). Sheets: Mach, Cp, Schlieren, Eddy ratio (range = 95th percentile of fluid cells, rounded to one significant figure); walls: Wall Cp, Skin friction (range = 95th percentile of |cf|); "Whole rocket" tiling on by default, "Tile sheets" off.
* How a failed, retried or rolled-back point reaches the user today:
  * NonFinite with no recovery: `Quality::Failed` -> the point is dropped, the table interpolates it and says "Mach X did not converge and is interpolated from its neighbours"; fewer than 2 points gives "the solver did not converge at enough Mach numbers".
  * A run the CFL controller took back and then converged: stored with `CfdPoint.restart`, a table note ("Mach 0.50 converged after a restart at a lower CFL limit (10.0 to 7.0): ...") and a tag in the point list of the solve screen ("converged · restarted at CFL 7.0", warning tone). `Warning::RestartedAtLowerCfl` is on the facade point.
  * Before this work the facade itself repeated a NonFinite point once with the CFL halved and said nothing (seen at Mach 0.5, job `0-20261117-142308-13772`: NonFinite at ~250 iterations, retry converged in 3700 iterations, CD 0.5049, 47.5 s, no note). That retry is removed; the controller (`CflControl`, `PointResult3.cfl_events`) replaces it. `[INFERENCE]` no Mach 0.5 point has been re-run with the controller since the merge.
  * Not done: a retried point's quality is not lowered, only flagged; the lift points (`CfdLiftPoint`) carry no restart flag.

## 6. Measured app-path numbers (3090 unless noted; BOWSHOCK_ENGINE=3d, viscous, 60 um body and fin roughness, altitude 120 m, 3-fin example)

* Full Coarse solve, 10 drag points + 16 angle points (8 Mach x 2/6 deg), job `1-20261117-172927-2425` (cfd-3d-gpu 47c9cec, fixed CFL 10): 2481 s = 41.4 min wall, 226 000 iterations, fixed costs 46 s, all Stationary, no failures; peak device memory 10.3 GiB (`gpumem`, whole test process, set by the 0.8-1.4 M-cell half domains). Drag points 384 674-855 680 cells at 5.4-10 ms/it, angle points 0.81-1.41 M cells at 10.5-16.2 ms/it.
* Quick (Draft) drag table alone, same job, 10 points: 354 s total, 2.7-3.0 ms/it, 10 900-14 750 iterations per point. The Quick drag table WITH the angles was not run (job `1-20261117-201838-14039` cancelled).
* Restricted Coarse table M 1.0/1.1/1.3/1.7, rough vs smooth, job `0-20261117-172926-9096` (CFL 10): 625.6 s rough, 633.5 s smooth, 13-20k iterations at 9.2 ms/it.
* Earlier restricted table before the fin-tip fix, `0-20261117-142308-12983`: Coarse 387.7 s (3.3 GiB), Draft 92.5 s (0.88 GiB); its fin friction was wrong (rough fin-tip faces ran away, y+max 1e5; fixed by cfd-3d-phys a0b6040 and the GPU mirror 47c9cec).
* Unit of `x_cp`: metres from the nose tip (`Forces.x_cp = r.x_cp * d_nose`; the LIFT lines printed 0.68-0.93 m on the 1.14 m rocket).

Rough vs smooth (job 9096, Coarse, CD / body friction / fin friction): M1.0 0.6485 | 0.5706 / 0.2002 | 0.1261 / 0.0326 | 0.0203; M1.1 0.6627 | 0.5904 / 0.1949 | 0.1215 / 0.0301 | 0.0187; M1.3 0.6499 | 0.5837 / 0.1859 | 0.1146 / 0.0286 | 0.0174; M1.7 0.6039 | 0.5385 / 0.1667 | 0.1021 / 0.0285 | 0.0169. Friction ratios 1.59-1.69 (a rough flat plate at this chord: 1.6-1.7). y+max 16.7-26.2.

### Coarse (3D, viscous, app path) next to the legacy table (2D RANS + empirical fins), per Mach

Columns: CD (the facade total, without the build-up "other" so that both columns are the solver's own: the 3D rows are the test's CD minus its 'other' of 0.003-0.026), body friction, forebody pressure, base, fins (friction + pressure for 3D; friction + pressure of the empirical model for legacy). Legacy: job `20261117-085411-30827` (130 s, 10 points, legacy engine, 3090). 3D: job `1-20261117-172927-2425`.

| Mach | CD 3D | CD legacy | body fric 3D | legacy | forebody 3D | legacy | base 3D | legacy | fins 3D | legacy (fric+press) |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 0.541 | 0.518 | 0.243 | 0.246 | 0.0018 | 0.0024 | 0.114 | 0.110 | 0.183 | 0.159 |
| 0.50 | 0.500 | 0.494 | 0.221 | 0.223 | 0.0035 | 0.0024 | 0.116 | 0.115 | 0.159 | 0.153 |
| 0.80 | 0.500 | 0.511 | 0.211 | 0.212 | 0.0043 | 0.0032 | 0.128 | 0.127 | 0.157 | 0.168 |
| 0.90 | 0.504 | 0.532 | 0.206 | 0.208 | 0.0087 | 0.0055 | 0.150 | 0.144 | 0.140 | 0.175 |
| 1.00 | 0.646 | 0.639 | 0.200 | 0.203 | 0.054 | 0.051 | 0.233 | 0.202 | 0.158 | 0.182 |
| 1.10 | 0.662 | 0.680 | 0.195 | 0.198 | 0.106 | 0.104 | 0.209 | 0.196 | 0.152 | 0.182 |
| 1.30 | 0.650 | 0.672 | 0.186 | 0.188 | 0.112 | 0.110 | 0.198 | 0.190 | 0.154 | 0.183 |
| 1.70 | 0.604 | 0.628 | 0.167 | 0.169 | 0.107 | 0.106 | 0.174 | 0.170 | 0.156 | 0.184 |
| 2.30 | 0.510 | 0.557 | 0.139 | 0.140 | 0.100 | 0.099 | 0.135 | 0.137 | 0.137 | 0.181 |
| 3.00 | 0.420 | 0.486 | 0.111 | 0.112 | 0.094 | 0.093 | 0.099 | 0.102 | 0.115 | 0.177 |

Legacy fin columns are the sum of its fin friction and fin pressure (e.g. M1.0 0.0354 + 0.1470). The app's table CD adds the build-up 'other' (lugs and rail buttons, 0.003-0.026) to the 3D CD, so the app shows 0.544 ... 0.444 where this table has 0.541 ... 0.420. The two agree on body friction, forebody and base within a few per cent; the fins differ most above Mach 2, where the solved fins give less drag than the empirical model.
CN and x_cp are given for the 3D Coarse solve only (the legacy Euler angle values were not re-run in this task): at 2 deg / 6 deg, CN M0.5 0.535 / 1.595 (x_cp 0.903 m), M0.8 0.556 / 1.683 (0.914), M0.95 0.640 / 1.942 (0.925), M1.1 0.619 / 1.825 (0.923), M1.3 0.537 / 1.589 (0.899), M1.7 0.418 / 1.262 (0.833), M2.3 0.352 / 1.068 (0.758), M3.0 0.280 / 0.915 (0.680).

## 7. Screenshots

* `out/finned-3d-restricted/*.png` (3000x2200, from job `0-20261117-142309-5638`, restricted Mach 1.0-1.7, BEFORE the fin-tip fix and before the 95th-percentile colour ranges): coarse default tiled scene, eddy-ratio sheet in the fin plane, skin-friction walls, the same on the Quick grid.
* Earlier Euler screenshots: `out/finned-3d/`, `out/finless-3d/`, `out/flow-view*`, `out/field3`. `out` is git-ignored; copy before the worktree is removed.
* Not taken: the full viscous solve's screenshot set with the new ranges and the fixed fin friction (`crates/bowshock/scripts/finned-3d-viscous.drive` and `finned-3d-viscous-restricted.drive` exist; the full one takes ~45 min of the 3090). 6-degree and zero-incidence-tiled shots of a viscous solve were never taken.

## 8. Flaky tests and what was done

* `bowshock-remote/tests/loopback.rs` JSON compare: the CPU residual is a parallel sum, last bits differ run to run -> `residual_drop_orders` rounded to 9 digits.
* `bowshock-core/tests/serde_roundtrip.rs::from_spec_of_spec_gives_the_same_run`: same cause (failed 1 in 4), same fix.
* App tests fail only under heavy machine load (timeouts); never assert the bits of the 3D CPU residual.
* The gpuq same-worktree mtime trap: `touch` the sources before each `gpuq submit`, else a job reuses the previous binary (check the log for "Compiling bowshock").

## 9. What is not done

* The flip itself (Engine::DEFAULT), the Jet patch (motors under the 3D engine), deleting the legacy path.
* Quick with angles, the full-solve screenshot set, a Mach 0.5 point re-run with the controller (and rolled-back points on the lift tier are not flagged), legacy CN/x_cp comparison, OpenRocket comparison (no harness at hand), a restart flag on `CfdLiftPoint`.
* The numbers in sections 6 were at fixed CFL 10; the product default became the engine's CFL (50/20) WITH the controller on cfd-3d-gpu 6d009f5188, which I did not re-time.
* y+max of the Draft tier reaches 61-230 at Mach 1.7-3.0 (job `1-20261117-172927-2425`): it is a wall-function tier, but I did not check whether the report counts fin-tip or wall-function faces.

## 9b. CI state at the end

fmt and clippy clean; app suite 796/796 under the default engine (3d run before the last merge: 796/796); bowshock-cfd/reference/cases/core: 318 passed and ONE failure after the last merge of cfd-3d (562edd6, solver3d's harness change): `wedge_gates::a_gate_without_a_runner_method_fails` now sees `INFO` lines ('not a harness runner check; covered by no covering test named') where it expects every line to FAIL. It is in solver3d's harness (`wedge_gates.rs` / its G-W1 runner-less path), not in the facade; the test file of mine (`wedge_gates/facade.rs`) is not involved. serde_roundtrip 7/7, remote/worker/validate/model 213/213, doc tests 20/20. The rest of that run (26 tests) did not run (fail-fast) and the app under BOWSHOCK_ENGINE=3d was not re-run after that merge.

## 10. Final commit

See the last line of the message that accompanies this file (the sha is in the git log of `cfd-3d-viz`).
