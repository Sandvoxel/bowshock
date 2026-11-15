# CFD handoff: app integration status and the next step (3D)

The `CFD table` method in the desktop app's Analyze screen calls this solver through `bowshock-core`. That work was paused on the
M1 Max laptop because solves were slow. CFD development, starting with a 3D solver, continues on a machine with a faster GPU. This
note covers what exists, what was measured, and what is still open.

## What the solver can do today

- The solver is axisymmetric (Euler / RANS k-ω SST in the meridian plane) and gives zero-lift drag only, in three parts:
  forebody pressure, skin friction and base. It gives no angle of attack, no normal force and no CP. Fins are empirical. See the
  root `ROADMAP.md`, items 1–2.
- The app therefore takes drag from CFD, and takes static margin, CP and CNα from the Extended method (Barrowman with body lift),
  labelled as such in the UI.

## App integration

| Area | File | State |
|---|---|---|
| Document → solver mapping | `crates/bowshock/src/analyze/cfd.rs` (`plan`) | Done. Maps one axisymmetric body plus one trapezoidal fin set. Other fin sets, lugs and buttons go to the build-up method's drag, listed in `Plan::notes`. Pods, tube fins, diameter steps, a missing nose and noses outside the solver's range give `Err(reason)` |
| Mach nodes, cache key, staleness | `analyze/cfd.rs` (`nodes`, `table_key`, `status`) | Done. The key is an FNV-1a hash of geometry, altitude (Reynolds number) and nodes. Status is Fresh, Stale, Missing or Unsupported |
| Solve | `analyze/cfd.rs` (`solve`), `analyze/mod.rs` (`spawn_solve`) | Done. One Mach at a time on a worker thread, with progress events and cancel between points. Uses the GPU through Luma and falls back to CPU, saying so in the provenance line |
| Below the first node | `analyze/cfd.rs` (`sweep_cd`) | Done. Uses the first node's drag, part by part, scaled by the Extended method's Mach trend |
| Storage | `crates/bowshock-model/src/cfd.rs` (`CfdStore`, `CfdTable`, `CfdPoint`, `CfdQuality`) | Done. Saved inside the binary `.bowshock` document (`BOWSHOCK` magic + u16 version + deflated CBOR, see `bowshock-model/src/format.rs`), keyed per stage, and survives undo/redo |
| UI | `analyze/view.rs`, `plots.rs`, `readout.rs`, `compute.rs` | Done. Provenance reads `Solving k / n · GPU (Luma)`, then `10 Mach points · zero-lift · solved on GPU (Luma) · 6 min ago · M<0.2 scaled from Extended`. A stale table shows Re-solve. Plot labels read `· Extended (CFD is zero-lift)` or `· CFD` |
| Tests | `analyze/cfd.rs` | Mapping, unsupported reasons, key stability, nodes, interpolation, blend, stale/fresh, cancel. Two tests are `#[ignore]`d because they solve for real: `a_tiny_grid_solves_and_is_sane` and `the_example_table_solves_and_agrees_with_extended` (prints Mach, margin and Cd for Extended and CFD side by side; `BOWSHOCK_CFD_FIXTURE=<path>` saves the solved example) |
| Drive script | `crates/bowshock/scripts/cfd.drive` | Written but never run end to end |
| Build profile | root `Cargo.toml` | `[profile.dev.package.*] opt-level = 3` for bowshock-core/cfd/reference, luma, wgpu*, naga. A debug solver is about 4× slower without it |

### Still to do in the app

- The full example table has never finished (stopped at 2 of 10 points), so there is no Extended-vs-CFD Cd table and no
  screenshots yet. To produce them:
  `cargo test --release -p bowshock -- --ignored the_example_table --nocapture`, then run `cfd.drive`.
- The plots don't shade the region below M 0.2 where drag is scaled from Extended. It is only mentioned in the provenance line.
  Shading would need a band like the existing `beyond` band in `widgets/plot`.
- Below M 0.2 the stability analysis uses the first node's drag without scaling it. This is minor.

## Timings (Apple M1 Max, GPU via Luma/Metal, coarse grid, example rocket)

- Release build, per Mach point: M0.2 36 s, M0.3 41 s, M0.5 31 s, M0.9 36 s, M1.5 32 s, M2.5 28 s. Each takes about 4000–4500
  iterations at 7–10 ms, ending on the `CdStationary` stop rule.
- The medium grid takes 126 s at M0.3, with Cd about 1 % different from the coarse grid.
- At M0.1 a point takes about 2 minutes. The subsonic base wake settles into a limit cycle, so only `MeanStationary` stops it,
  after about 14 000 iterations. M0.15 takes 25 s. This is why the app's table starts at M0.2.
- Running Mach points in parallel gives no speedup, because the solve is GPU-bound.
- The app's default 10-node table therefore takes about 5.5 min on this machine.

## Grid used by the app

The grid is Mach only, because there is no angle of attack, with 10 nodes: 0.2, 0.5, 0.8, 0.9, 1.0, 1.1, 1.3, 1.7, 2.3, 3.0. It
is denser through transonic. Nodes are clipped to the condition range, and an interior node within 0.12 of an end is dropped. The
grid resolution is coarse.

## Open decisions

1. Below M0.2, use Extended-trend scaling (implemented), hold the first node's value, or cap the solver's iterations at M0.1.
2. Use the coarse or the medium grid as the default.
3. Whether to shade the scaled region in the plots.

## Next: 3D (angle of attack, normal force, CP, fins)

The app is ready to take lift data. `analyze/cfd.rs` already has `alpha_points`, which currently uses Extended's change with
angle of attack. To use 3D results:

- Give `CfdPoint` CN, CNα, the CP station and Cm per (Mach, α). Add new fields with `#[serde(default)]` so saved files still load;
  renames or restructures need a `FORMAT_VERSION` bump, a migration step and a new golden file (rules in `format.rs`).
- Add the α nodes to `nodes` and `table_key`.
- Switch the margin, CP and CNα panels from `Extended` to `CFD` where the table has the data. The labels are produced in
  `analyze/plots.rs` and `readout.rs`.

Two routes, from the discussion with the user:

| Option | Gives | Cost per point (vs today) | Notes |
|---|---|---|---|
| First-harmonic crossflow (linearised cos θ mode on the existing 2D mesh) | Body CNα and CP at small α | about 2× | Body only, fins stay empirical. The hard parts are the linearised turbulence and base-wake terms |
| Full 3D solver | Body and fins at any α, true CP, fin interference | 10–100× | Needs a 3D mesher that handles fins, 3D GPU kernels, and validation |

What to validate against:

- OpenRocket: `tools/openrocket-harness` drives OpenRocket 23.09 and dumps CP, CNα and Cd versus Mach at α = 0° and 2° for
  any `.ork` file. `tools/compare-openrocket/run.sh` runs the comparison, and `docs/openrocket-comparison.md` holds the current
  tables.
- The existing validation cases in `bowshock-cases`, `bowshock-gpu-cases` and `bowshock-validate`.
- Wind-tunnel normal-force and CP data for ogive-cylinder and finned bodies. Pick these when the 3D work starts.
