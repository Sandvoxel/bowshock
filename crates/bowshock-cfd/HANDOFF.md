# CFD handoff: 2D state after the `cfd-2d` round, and the next step (3D)

The `CFD table` method in the desktop app's Analyze screen calls this solver through `bowshock-core`; Simulate can fly on the
stored table. The `cfd-2d` branch (2026-11-15, RTX 3090) made the 2D path faster, rough-walled and usable in flights. This note
covers what exists, what was measured, and what is still open before and for the 3D solver.

## What the solver can do today

- The solver is axisymmetric (Euler / RANS k-ω SST in the meridian plane, rough walls) and gives zero-lift drag only, in three
  parts: forebody pressure, skin friction and base. It gives no angle of attack, no normal force and no CP. Fins are empirical.
  See the root `ROADMAP.md`, items 1–2.
- The app therefore takes drag from CFD, and takes static margin, CP and CNα from the Extended method (Barrowman with body lift),
  labelled as such in the UI. Simulate can fly on the CFD drag (per flight).

## App integration

| Area | File | State |
|---|---|---|
| Document → solver mapping | `crates/bowshock/src/analyze/cfd.rs` (`plan`) | Done. Maps one axisymmetric body plus one trapezoidal fin set. Other fin sets, lugs and buttons go to the build-up method's drag, listed in `Plan::notes`. Pods, tube fins, diameter steps, a missing nose and noses outside the solver's range give `Err(reason)` |
| Mach nodes, cache key, staleness | `analyze/cfd.rs` (`nodes`, `table_key`, `status`) | Done. The key is an FNV-1a hash of geometry, altitude (Reynolds number) and nodes. Status is Fresh, Stale, Missing or Unsupported |
| Solve | `analyze/cfd.rs` (`solve`), `analyze/mod.rs` (`spawn_solve`) | Done. One Mach at a time on a worker thread, with progress events and cancel between points. Uses the GPU through Luma and falls back to CPU, saying so in the provenance line |
| Below the first node | `analyze/cfd.rs` (`mach_points`, `below`) | Done. Uses the first node's drag, part by part, scaled by the Extended method's Mach trend; shaded in the plots |
| Storage | `crates/bowshock-model/src/cfd.rs` (`CfdStore`, `CfdTable`, `CfdPoint`, `CfdQuality`) | Done. Saved inside the binary `.bowshock` document (`BOWSHOCK` magic + u16 version + deflated CBOR, see `bowshock-model/src/format.rs`), keyed per stage, and survives undo/redo |
| UI | `analyze/view.rs`, `plots.rs`, `readout.rs`, `compute.rs` | Done. Provenance reads `Solving k / n · GPU (Luma)`, then `10 Mach points · zero-lift · solved on GPU (Luma) · coarse grid · 6 min ago · M<0.3 scaled from Extended`. A stale table shows Re-solve; a coarse table shows Refine. Plot labels read `· Extended (CFD is zero-lift)` or `· CFD` |
| Tests | `analyze/cfd.rs` | Mapping, unsupported reasons, key stability, nodes, interpolation, blend, stale/fresh, cancel. Two tests are `#[ignore]`d because they solve for real: `a_tiny_grid_solves_and_is_sane` and `the_example_table_solves_and_agrees_with_extended` (prints Mach, margin and Cd for Extended and CFD side by side; `BOWSHOCK_CFD_FIXTURE=<path>` saves the solved example) |
| Drive script | `crates/bowshock/scripts/cfd.drive` | Run on `cfd-2d` (screenshots of the CFD screens, Refine, the shaded band) |
| Build profile | root `Cargo.toml` | `[profile.dev.package.*] opt-level = 3` for bowshock-core/cfd/reference, luma, wgpu*, naga. A debug solver is about 4× slower without it |

### Done on `cfd-2d`

| Change | Where | Measured |
|---|---|---|
| Relative drag stop rule (`Analysis::drag_tolerance`, opt-in; app uses 0.2 % over 500 iterations) | `bowshock-core` `engine/run.rs::stop_reason` | Example table 71 s -> 22 s (smooth), Cd within 0.2 % of the old rule; worst point 0.39 % from a 12 000-iteration solution |
| Refine: re-solve the table on the medium grid; `CfdTable.grid` | `analyze/{cfd,mod,view}.rs`, `bowshock-model` | - |
| Below the first node (Mach 0.1, or the range start): first node scaled part by part by Extended's Mach trend, shaded band | `cfd::mach_points`, `PlotGroup.before` | - |
| Base-pressure guard: base Cp below vacuum -> `Failed` (ISSUES #14) | `analysis.rs::quality` | - |
| Wall roughness: Aupoix (2015) Colebrook k-omega SST wall condition, CPU + GPU; app passes the parts' finish (wetted-area mean), fins floored at OpenRocket's roughness limit | `turbulence.rs::aupoix_wall`, `viscous.rs`, `gpu/`, `Analysis::roughness`, `FinSpec::roughness` | Flat plate fully rough +0.2 % vs Mills & Hang; transitional within 1.6 % of a Colebrook log-wake reference; example table 31 s, all points Stationary |
| Simulate flies on the CFD table (per-flight `Drag: Extended / CFD table`): table parts + build-up Reynolds change, base x build-up motor-on/off ratio | `bowshock-sim/src/drag_table.rs`, `Aero::drag`, `simulate/` | Example Nominal flight: apogee 619 m (Extended) -> 645 m (CFD) |
| OpenRocket comparison (ignored test `cfd_against_openrocket`, OpenRocket 23.09 jar in `~/.local/share/openrocket`) | `analyze/cfd.rs` tests | See below |

CFD vs OpenRocket 23.09 (bundled examples `dual_parachute`, `simple_model_rocket`, `tarc_payload`, 60 um paint, sea level):
subsonic (M 0.5-0.9) -9 to -15 % (was -24 to -30 % with smooth walls); M 1.0-1.3 -9 to +12 %; M 1.7-3.0 -6 to +18 %. Extended is
within 1-4 % of OpenRocket subsonic (it is the same build-up) and +8 to +21 % transonic. What is left subsonic is mostly base
drag (CFD 0.11 against the empirical 0.12 + 0.13 M^2 = 0.15) and OpenRocket's fully rough friction assumption, which overstates
friction at these k_s+ (about 20-50).

### Still open in 2D

- Power-on drag from CFD (a jet in the base flow, ISSUES #13): the solver can run a motor-on base flow, but under-expanded jets
  give base pressures far from the data, and nozzle-exit data (diameter, Mach, pressure, temperature) is not in `.eng` files.
  Flights use the build-up's base reduction as a ratio instead.
- The roughness ramp (`ROUGH_RAMP_CELLS` = 8, about 0.25 diameters at Coarse) and smooth base / sting walls are stability
  measures (shorter ramps oscillate at the wall start transonic); rough runs take 1.3-2x the iterations of smooth ones.
- Simulate's drag note recomputes the CFD plan on every render for flights on `CFD table`; cache it if it shows in frame time.
- Analyze's Dispersion analysis and Monte Carlo from Simulate fly on whatever `SimInput.drag_tables` the caller gives; Analyze
  Dispersion does not attach a table yet.
- Mac timing not re-measured: the 3090 is about 10x an M1 Max per iteration, so the table should take about 5 min there.

## Timings

- Apple M1 Max, before `cfd-2d` (GPU via Luma/Metal, coarse grid, example rocket): 28-41 s per Mach point, each about
  4000-4500 iterations at 7-10 ms ending on `CdStationary` (the old rule cannot fire before iteration 4000); the 10-node table
  about 5.5 min. Running Mach points in parallel gives no speedup (GPU-bound).
- RTX 3090: about 0.8 ms per iteration (coarse, viscous, df32). Example table 71 s before the stop rule, 22 s after, 31 s with
  rough walls. M0.1 converges in about 2750 iterations with the relative rule (its base wake has a slow ~1 % limit cycle).
- The medium grid (Refine) is about 3.5x a coarse point; Cd about 1 % different.

## Grid used by the app

Mach only, 10 nodes: 0.1, 0.5, 0.8, 0.9, 1.0, 1.1, 1.3, 1.7, 2.3, 3.0 for the default range 0.1-3.0, denser through transonic.
Nodes are clipped to the condition range, and an interior node within 0.12 of an end is dropped. Coarse grid by default.

## Decisions taken on `cfd-2d`

1. Below the first node: Extended-trend scaling, shaded.
2. Coarse grid by default, Refine for medium.
3. The app's stop rule aims at 1 % Cd (0.2 % range over 500 iterations).
4. 3D: a full 3D solver, with the 2D axisymmetric solver kept as the fast fallback (Quick mode).

## Next: 3D (angle of attack, normal force, CP, fins)

User decision: full 3D, 2D kept as the fallback. Proposed shape (discussed, not yet agreed in detail):

- Drag stays 2D in both modes (validated, cheap); 3D supplies CN, CNα, CP and Cm at a few Mach nodes and one or two α.
- Mesh: the current (x, r) grid revolved into a structured (x, r, θ) grid with fins as zero-thickness walls on constant-θ
  planes, half domain about the pitch plane. Reuses MUSCL/HLLC, the line-implicit scheme (Luma's block-tridiagonal solves take
  5x5 blocks along any axis of an array of up to 4 dimensions), multigrid and the array layout. Needs a polar-axis treatment.
- Open questions for the user: Euler first or RANS; α sampling (one small α vs several); Quick/Full selection in the app;
  validation set (Army-Navy Basic Finner for CNα/CP across Mach, ogive-cylinder normal-force data, OpenRocket).

The app is ready to take lift data. `analyze/cfd.rs` already has `alpha_points`, which currently uses Extended's change with
angle of attack. To use 3D results:

- Give `CfdPoint` CN, CNα, the CP station and Cm per (Mach, α). Add new fields with `#[serde(default)]` so saved files still load;
  renames or restructures need a `FORMAT_VERSION` bump, a migration step and a new golden file (rules in `format.rs`).
- Add the α nodes to `nodes` and `table_key`.
- Switch the margin, CP and CNα panels from `Extended` to `CFD` where the table has the data. The labels are produced in
  `analyze/plots.rs` and `readout.rs`.

What to validate against:

- OpenRocket: `tools/openrocket-harness` drives OpenRocket 23.09 (jar at `~/.local/share/openrocket/OpenRocket-23.09.jar` on
  the Linux machine, `OPENROCKET_JAR`) and dumps CP, CNα and Cd versus Mach at α = 0° and 2° for any `.ork` file; the ignored
  test `cfd_against_openrocket` compares the CFD table with it.
- The existing validation cases in `bowshock-cases`, `bowshock-gpu-cases` and `bowshock-validate`.
- Wind-tunnel normal-force and CP data for ogive-cylinder and finned bodies. Pick these when the 3D work starts.
