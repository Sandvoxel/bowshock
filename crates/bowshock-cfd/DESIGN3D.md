# DESIGN3D: one 3D viscous solver for drag, fin drag and lift (design + staged plan, no code written)

Author: `solver3d`, 2026-11-17. Branch `cfd-3d`, HEAD 008d69d (merge of `origin/ui` 6375190; the design was written against the pre-merge tree and section F re-read after it). Tags: **[M: src]** measured / read from a file,
**[E]** my estimate (method given), **[R]** read in a paper or page tonight, **[S]** abstract or search summary only (not read in full).
Lengths in nose diameters `d` unless stated. "2D" = `crates/bowshock-cfd/src/{solver,flux,implicit,viscous,turbulence,multigrid,grid,bc,gas}.rs`
+ `gpu/{body,implicit,viscous,turbulence,control}.rs` + facade `bowshock-core/src/engine/{case,run,gpu,numerics}.rs`, `fins.rs`, `field.rs`.

## 0. Provenance and what I corrected

* My session cwd is the main worktree (`/home/sandvoxel/Documents/bowshock`, branch `cfd-perf`); about half of my first reads went there. I diffed every file I
  relied on against `bowshock-cfd-3d`: differences are rustfmt plus features that exist here and not there: `LineSolve3::DiagonalCorrected`
  (`implicit3/diag3.rs`, `Numerics3::implicit_for`), 2D low-Mach preconditioning (`ImplicitParams::low_mach_precond`, ISSUES #19), `FinProfile::{Hexagonal,Biconvex}`,
  `Nose::Blunt`, `field.rs`, `AnalysisSpec`. Numbers below are from the `cfd-3d` versions; section F was re-read after the merge (008d69d).
* Facts the brief did not give and the design depends on:
  1. The 2D base grid is **not** a body in a sting grid: `body_grid.rs::base_domain_grid` makes one H-grid in which the body interior is the solid `Block`
     (`solver.rs: Block{i_end,j_end,j_sting}`, cells `i<i_end && j<j_end`), the base is the `i = i_end` face of that block, and the wake rows `j<j_end, i>=i_end`
     reach the axis. The 3D mesher (`engine/grid3.rs`, `Case3::new -> sting_grid`) revolves the *sting* grid instead (no base).
  2. 2D viscous implicit left-hand side has **no viscous Jacobian**: `implicit.rs::cell_blocks` adds the diffusive spectral radius `2 lam I`,
     `lam = gamma (mu/Pr + mu_t/Pr_t)/(rho |d|)` (`viscous.rs::face_lambdas`) to the Roe dissipation of each face. Nothing more has to be ported.
  3. 3D has **no low-Mach preconditioning** of the implicit side (2D has: Mach 0.10 14 100 -> 1 200 iterations [M: ISSUES #19]); the 3D `low_mach` flag is the flux
     correction only (`flux3.rs::low_mach_states3`). A subsonic 3D RANS table needs it (section D).
  4. CFL limits: 3D Euler `cfl_max` 10 (`Numerics3::auto`; 15/20/30 diverged on coarse ANF), 2D RANS 50 / 20 supersonic (`numerics.rs: RANS_IMPLICIT*`).
  5. Wall distance is brute force over wall segments (`turbulence.rs::wall_distance`); fine in 2D, not in 3D (section C).
  6. Consumers of the 2D path the brief did not list: `field.rs::Field` (meridian field built from the 2D `Solver`/`Wake`, kept by `Analysis::keep_fields`, streamed live by `on_field`, drawn by the app's
     `analyze/flow_view.rs` + `flow_view/`); `bowshock-remote` (`proto.rs`: `Job::{Drag, Lift}`, `WorkerMsg::{Drag(Polar), Lift(LiftPolar)}`, `PROTOCOL` 1, frames <= 256 MB) and `bowshock-worker`, which is
     rebuilt on the remote from an embedded source set of the crates `bowshock-{cfd,core,reference,remote,worker}` hashed with SHA-256 (`sourceset.rs: CRATES`), so every solver change re-keys it and a **new crate would have to be added to `CRATES`**.
  7. Stored-result constants live in `crates/bowshock/src/analyze/cfd/plan.rs`: `SOLVER_REV` 5 (l.7), `LIFT_REV` 4 (l.27), `SHARED_GRID` false (l.16), `LIFT_ALPHAS` [2, 6] deg (l.31), `LIFT_INTERIOR` 8 Mach nodes (l.29).
     `bowshock-model::FORMAT_VERSION` is 1 and additive `#[serde(default)]` fields need no bump (`format.rs` rules). The app solves **one Mach node per call** (`analyze/cfd/run.rs`: `analysis.run(&[mach])` l.561,
     `run_lift(&[mach], &LIFT_ALPHAS)` l.280, nodes shared out over local and remote GPUs by `run_fleet`), so grid, wall distance and kernels must be reusable across calls (section E).
  8. `FinSpec::validate` accepts `count` 3..=8; the document allows 1..=16 (`geometry.rs: MAX_COUNT`), per-fin angles, cant, three sections
     (`FinSection`), three planforms, lugs, buttons, pods (`RadialKind`).
  9. CI (`.github/workflows/ci.yml`): `cargo +stable fmt --all --check` (`rustfmt.toml`: `max_width = 160`, `use_small_heuristics = "Max"`) and `cargo +stable clippy --workspace --all-targets -- -D warnings` (stable 1.100): every stage must pass both.
  10. Luma is **crates.io 0.4.1** (`Cargo.toml` l.19, `Cargo.lock` registry, checksum abf4a362...), no copy in the tree. The local Luma repo's `CHANGELOG.md` "Unreleased" section (`permute_axes`, shared-expression kernel cuts, wider strided Thomas solves up to 1.5x, 48 KiB cyclic lines) is **not** in 0.4.1: the 3D step numbers below are for the released Luma; the Unreleased gains arrive with the next release and a pin bump.

## 1. Design in twelve lines

1. One solver: `Solver3` (CPU f64 reference, Luma GPU port) becomes RANS (SST-2003, Aupoix rough walls, automatic all-y+ wall treatment) on a **revolved base-wake grid**
   (`base_domain_grid` -> `grid3::revolve`), body interior and fin layers as `SolidBox`, base and wake included.
2. One grid family, three domains: **wedge** (nk = 1, angle `dtheta` default 6 deg, replaces the axisymmetric solver), **sector** (theta in [0, pi/N], fin mid-plane to
   mid-gap, symmetry planes both ends, valid for every N = 1..16, odd and even), **half domain** (alpha != 0, pitch-plane symmetry, fin roll phase 0 or pi/N).
3. Axis ghost generalised to `AxisMap` (source cell(s) + 2x2 velocity matrix per k): `-I` same cell for the wedge and even N, mirrored interpolated cell for odd N.
4. Forces by wall class (nose, aft, base, fin side, fin LE, fin TE, fin tip), pressure + shear, moments (pitch, roll), all from the same wall-face list.
5. Wall treatment: automatic (omega blending, blended u_tau; Esch-Menter 2003 style): body y+ ~ 1 (parity with 2D and gates), fin sides y+ ~ 15 on the Coarse tier.
6. Implicit: existing three-factor AF (block on CPU, `DiagonalCorrected` on GPU) + diffusive spectral radius + SST scalar line solves + **new** 5x5 low-Mach
   preconditioning below Mach 1; wedge mode drops the k factor. Stop rule: relative CD (and CN, Cm at alpha) range over a window.
7. Perf: f32/df32 residual as today (LHS f32), shared grid per table, **grid sequencing** (semi-coarsening in i, j; fin layer keeps its k) as the main time lever.
8. Facade: `Analysis::run(&[Condition{mach, alpha}])` is the only solve entry; `run_lift`, `LiftPoint/LiftPolar`, `fins.rs`, 2D engine and 2D solver are deleted.
9. Validation first, deletion last: S0 freezes 2D reference numbers into a fixture; the wedge must reproduce them before anything is removed (section G, H).
10. Research outcome: keep SST-2003 + HLLC; adopt all-y+ wall treatment and grid sequencing; **defer** gamma transition, QCR, curvature, compressibility, overset/cut-cell.
11. Estimated cost: wedge table ~20-45 s; 4-fin sector coarse table 2.7-4 min (2.5 x faster with S9); alpha table 3-5 min on a lift tier. All [E], section E.
12. Smallest useful end state (I): S0-S6 = finless wedge + trapezoidal fin sector at alpha = 0 on the GPU, gated; 2D stays until the gates pass.

## A. Grid and topology

### A1 Base and wake

| item | 2D today | 3D design |
|---|---|---|
| meridian grid | `base_domain_grid(BaseGridParams) -> (Grid, Block)`: `n_up 28f, n_nose 48f, n_seg 40f/24f, n_wake 40f, n_gap = layer_count(re-rs, y1), nj_out = layer_count(r_far-re, y1)` (`case.rs: Case::new/viscous`, f = 2^level) | unchanged; the same meridian `Grid` is revolved: `revolve(g2, nk, theta)` (`grid3.rs:198`). Parity with the wedge is by construction |
| body interior | `Block` (never updated) | `SolidBox{i:0..i_end, j:0..j_end, k:0..nk}` (`grid3.rs: SolidBox`); no new solver code: faces fluid/solid are walls (`solver3.rs: Family::fluxes`, `run3.rs: accumulate` already integrates them) |
| base wall | `i = i_end` block face, no-slip (`viscous.rs: Walls::noslip_iface`) | the same faces, wall class `Base`; pressure + shear counted in the force breakdown |
| axis | j = j_end for `i < n_up` (upstream line), j = 0 for `i >= i_end` (wake) | `Grid3.axis_cells: usize` (one run at j = 0) becomes `axis: Vec<AxisRun{j, i0, i1}>` (two runs); degenerate faces already carry `area == 0` and zero flux (`fluxes`) |
| motor jet | `NozzleRows{r_exit,n_jet}` rows + `Nozzle` state patch on the base face | **only a boundary patch**: faces `i = i_end, j < n_jet, all k` get a jet-state ghost (+ k, omega values), `Walls::jet_iface` rule. ~300 lines CPU+GPU. Physics unchanged (ISSUES #13 open: under-expanded jets give base pressures far from data). Must be ported before the 2D deletion (S9b) |
| waste | solid block is ~27 % of 2D cells [E: (28+48+40)/156 columns x 52/140 rows] | same 27 % but multiplied by nk: lever `gap` growth 1.2 -> 1.3 (52 -> 36 rows, -11 % cells) [E]; accepted until S10 |

### A2 Domains and the generalised axis ghost

`theta_k in [0, Theta]`, nodes `(x, r sin th, r cos th)` as `revolve`. Fin mid-plane at theta = 0 (k = 0), mid-gap at `Theta = pi/N` (k = nk): both are reflection
planes of the whole body for any N (bisector of two neighbours), so the sector rule is uniform: **one half fin on the k = 0 plane** (the existing odd-N half fin on `k = nk`
in `engine/grid3.rs::layers`, mirrored). The solid layer sits on the symmetry plane, which then needs no BC inside the fin.

| domain | N (fins) | `Theta` | k planes | axis map from `theta_k + pi` (group D_N folded into [0, Theta]) | nk |
|---|---|---|---|---|---|
| wedge | 0 (or finless) | `dtheta` = 6 deg | symmetry both | same cell, `M = -I` (exact for axisymmetric flow) | 1 |
| sector | N even | pi/N | symmetry both | rotation by pi is in D_N: same cell, `M = -I` | 12..40 |
| sector | N odd | pi/N | symmetry both | `rot(pi) = R(pi/N) * (group element)`: mirrored cell at `Theta - theta`, `M = R(((N-1)/2)(2pi/N)) Refl(pi/N)`, det -1; 2-cell linear interpolation unless the k grid is mirror-symmetric | 12..40 |
| half | alpha != 0 | pi | symmetry (pitch plane) | existing `axis_k` mirror, `M = diag(-1, 1)` (current `fill_ghosts`: negate `v`) | 40..100 |
| half | roll phase | fins at `phi0 + 2 pi m/N`, `phi0 in {0, pi/N}` (the pattern must be mirror-symmetric about the pitch plane) | | the mesher takes `phi0`; `phi0 = 0` puts half fins on both k planes (N even) or one (N odd) | |

* `Solver3::new` computes `axis_k[i*nk+k]` by azimuth search (`solver3.rs:99-116`); it becomes `AxisMap{k0,k1,w,m:[f64;4]}` per (run, i, k); GPU: the existing host-index
  `take` plus a 2x2 multiply on `(v, w)` (`gpu/body3.rs` axis ghost). Test: sector (N = 4) and the two-fin half domain at alpha = 0 must agree (G-W3); odd N: N = 3 vs N = 1.
* The ghost matters only for the second-order reconstruction and the gradient of the first row beside the axis (face j = 0 has zero area).
* **N = 0..8 table.** 0: wedge. 1: sector pi (one fin, one side). 2: pi/2. 3: pi/3 (odd path). 4: pi/4 (ANF, Arcas). 5: 36 deg (odd). 6: 30 deg. 7: 25.7 deg (odd). 8: 22.5 deg. N up to 16 uses
  the same code (11.25 deg).
* Wedge stiffness: with nk = 1 the k faces are reflecting planes. The k-face fluxes **stay** (pressure and, for viscous flow, the normal stress `tau_nn`: the hoop stress of
  2D `viscous.rs::hoop_stress` is exactly the gradient across the mirrored ghost, no special code), but the k line solve and `L_k` in `compute_dt` are dropped (`nk == 1` switch),
  otherwise `V/(c A_k) = r dtheta/c` makes every near-wall and axis cell 10x stiffer than in 2D.
* `dtheta` default 6 deg: flat-face chord error `1 - sin(dtheta/2)/(dtheta/2) = 1.1e-4` [E: analytic; README measured 3.6e-4 for 32 cells over pi, ratio 0.27 per halving [M: README 3D "Flat-face chord error"]].

### A3 Resolution

| where | spacing | how | cost [E] |
|---|---|---|---|
| body wall (j) | y+ ~ 1, growth 1.2 | `Case::viscous`: `y1 = 1/(re_c M_max sqrt(cf/2))`, `layer_count`; same meridian grid as the wedge | 0 (shared) |
| fin sides (k) | first cell `y1_fin = y+_fin nu/u_tau` at the root radius `R` (`dtheta_1 = y1_fin/R`), growth `q = 1.25` inside the boundary layer `delta ~ 0.37 c Re_c^-0.2`, `q = 1.35` to mid-gap | new k-spacing in `engine/grid3.rs` (replaces uniform `nh`, `gaps` in `layers`) | nk = `n_BL + n_out + 1 solid` |
| tip (j) | node row on the tip (exists: `s_remap`), nominal spacing above (no clustering; tip face is ~4 % of fin wetted area for ANF) | tip wall treated by the all-y+ BC | 0 |
| LE / TE (i) | `32 << level` cells over the root chord, clustered both ends (exists: `two_sided_nodes`) | faces with normal x carry no streamwise shear | 0 |
| fin-body junction | no special treatment: wall rows (j) and fin-side rows (k) refine the corner cell in both directions | SST corner separation is the known weak point, see Research | 0 |

nk for the Arcas fin at M 2 (`arcas.rs`: chord 0.086 m, d 0.0572 m, Re/m 9.84e6): `delta` = 2.1e-3 m = 0.036 d, `y1(y+=1)` = 2.5e-6 m, `y1(y+=15)` = 3.7e-5 m [E: flat-plate
Cf 0.003, T 190 K] -> `n_BL` = 24 (y+ 1) or 12 (y+ 15), `n_out` = 5, so **nk = 30 or 18**.

| cells | Level 0 (Coarse) | Level 1 (Medium) |
|---|---|---|
| wedge (nk = 1) | 156 x 140 = 21 840 [M: ledger `2d-table-example`; `field.rs` doc 21 216 / 42 432 for the 1.1 m body] | ~43 000 [E: 2 x ni, +4 rows] |
| sector N = 4, fin y+ 15 (nk 18) | 176 x 140 x 18 = 443 000 (`ni` = 156 + ~20 for the three-piece fin segment) [E] | 352 x 144 x 24 = 1.2 M [E] |
| sector N = 4, fin y+ 1 (nk 30) | 739 000 [E] | 2.0 M [E] |
| half domain N = 4, phase pi/N (2 fins, nk ~ 70) | 176 x 140 x 70 = 1.7 M [E] | 352 x 144 x 93 = 4.7 M [E] (2.8-4.7 GB: borderline, not offered) |
| half domain, lift tier (body y+ ~ 60, ni 150, nj 100, nk ~ 32) | 480 000 [E] | - |
| existing 3D Euler half domain (ANF) | 210 x 70 x 24 = 352 800 [M: ledger `3d-anf-L0-implicit`] | 2 587 200 [M] |

### A4 Is the one-cell layer with the 25 % floor usable for edges?

No, for sharp edges. `engine/grid3.rs::half_thickness` ends in `h.max(0.25 * r)` (`r = t/2`), so every node column of the layer has thickness >= `0.25 t`: a sharp wedge LE (ANF
`Custom [[0,0],[0.1,1],[1,1]]`, Arcas `DoubleWedge`, `Airfoil` tip) becomes a **flat face of width 0.25 t** normal to x at the LE node column (and at the TE of a sharp
section). Cost at M 2, ANF: `(cp_stag - cp_wedge) * 0.02 d * 1 d * 4 fins / (pi/4 d^2)` = `(1.66 - 0.5) * 0.102` = **+0.12 CD, ~22 % of the ANF CD0 ~ 0.55 at M 2** [E: Rayleigh pitot
`cp_stag` = 1.66, oblique-shock `cp_wedge` ~ 0.5, thickness 0.08 d; CD0 interpolated between the DREV range values 0.585 (M 1.83) and 0.484 (M 2.38), G-F5]. It is harmless for Square (a real blunt face) and for the blunt TE of the ANF.
Cheapest fix (S6, ~120 lines in `half_thickness`/`fill_theta`): floor `0.02 r` at nodes that are really sharp (zero-area face, already supported: `fluxes` returns 0 for `area == 0`),
keep the floor only where the section is blunt; relax the layer width from the nominal `2 alpha0` to the fin width over >= 4 cells ahead of the LE and behind the TE (the
needle cell stays valid: a taper, not a step); circle arcs of `Rounded` come out as the polygon of the clustered nodes (already node-defined, not a staircase: the fin side is the
node surface `theta_f +- h(x_i)/r_j`, faces between node columns are slanted). Gates G-F2 (double wedge vs oblique shock / Ackeret) and G-F3 decide whether more is needed.

### A5 Geometry scope: what the document can hold (item 3), ordered by value

Document inventory: `PartKind{NoseCone(7 shapes), BodyTube, Transition}`, `RadialKind{FinSet, EllipticalFin, FreeformFin, LaunchLug, RailButton, Pod}`, `FinSection{Square, Rounded,
Airfoil}`, `RadialPart{count<=16, angle, cant, angles per copy, overrides}`, shoulders (inside), stages (truncated vehicles), motors. `analyze/cfd/plan.rs::plan` refuses pods, tube fins,
a missing nose, diameter steps, Haack C > 1/3, power n outside 0.5..1; diverts to the build-up: extra fin sets, lugs/buttons, freeform/elliptical fins (trapezoid equivalents).

| # | item | representation | cost / gate | status tonight [E] |
|---|---|---|---|---|
| 1 | axisymmetric stack: all noses, tubes, transitions, boattail, base, roughness, power-on patch | revolved meridian grid, wedge | S1-S5, S9b; G-V4 | yes |
| 2 | trapezoidal fin set N = 1..16, sections Square/Rounded/Airfoil/DoubleWedge/Hexagonal/Biconvex/Custom, sweep, taper, thickness | sector layer + x-remap per row (exists) | S6; G-F1..F5 | yes |
| 3 | alpha != 0: CN, CP, Cm, CD(alpha), fin roll phase 0 / pi/N | half domain | S7; G-L1/2 | coarse tier yes, fine tier probably not |
| 4 | elliptical, freeform fins (chordwise single-interval outlines) | x-remap per row with tabulated `x_le(r), x_te(r)` + r-remap per column for a curved tip so the tip is a node row | ~500 lines, gate vs trapezoid limit and Arcas | stretch |
| 5 | several fin sets, per-fin angles, mixed counts | symmetry classifier -> {sector gcd, half, full 2pi}; one `SolidBox` per set; sets on different segments only | ~400 lines | stretch |
| 6 | diameter steps | one-cell-wide chamfer (very steep ramp) in the meridian grid, valid hexahedra | ~200 lines, gate backward/forward step vs Hoerner/NASA data | stretch |
| 7 | fin cant | k lines sheared along the chord by `dtheta = (x - x_c) tan(eps)/r` (<= 15 deg, `Inspector` limit); no mirror symmetry -> periodic sector `2pi/N` or full domain; needs a periodic k seam: **Luma has no periodic block-tridiagonal** [M: `luma/src/linalg.rs`], seam coupling explicit (lagged) as fallback | ~600 lines | no; also gives roll moment `Cl` |
| 8 | launch lugs, rail buttons | sub-grid (1-5 mm in a boundary layer ~ 1 cell): **cannot be done well** body-fitted; keep the build-up add-on (`CfdPoint.other`); a roughness-patch model is the cheap upgrade | - | no |
| 9 | pods, strap-ons, tube fins | bodies beside the axis: **cannot be done** with one revolved grid; needs overset (OVERFLOW-style near-body grids + donor search; GPU Chimera exchange is the hard part [R: Jackson 2024]) or a cut-cell/IB Cartesian solver | new subsystem, weeks | no (stay `Err(reason)`) |
| 10 | transition (gamma), Tu input, QCR, compressibility | turbulence module as a scalar list | Research | no |

## B. Wall-resolved or wall functions

| | wall-resolved only (y+ <= 2) | all-y+ automatic treatment (**recommended**) | wall functions only (y+ 30-300) |
|---|---|---|---|
| body, wedge | = 2D (parity gates, G3 y+max 0.648 [M: README]) | same grid, y+ ~ 1 | nj 140 -> 115 [E: `ln10/ln1.2` rows per decade] |
| fin sides nk | 30 | **18** (y+ 15) | 14 |
| accuracy | SST plate within 5 % at y+ <= 2, degrades above 5 [R: NASA GRC WIND SST flat-plate study] | y+-insensitive 0.1..100 by design [S: Esch-Menter 2003 via CFD-wiki, Grotjans-Menter 1998]; gate G-V2 measures it here | friction +-8..15 % in shock/pressure-gradient regions [E]; breaks wedge parity |
| stiffness | cell aspect 1e4 at the wall | 1e3 | 1e2 |
| effort | 0 | ~60 lines CPU + GPU (`omega_w = sqrt(omega_vis^2 + omega_log^2)`, `u_tau = (u_vis^4 + u_log^4)^(1/4)`, wall shear `tau_w = rho u_tau^2`, `mu_t,w` for the face, ghost trick of `turbulence.rs` kept) | ~150 |
| roughness | Aupoix 2015 `aupoix_wall` (exists) | the same function replaces `omega_log` when `ks+ > 0`; roughness makes y+ < ks+ meaningless anyway | natural |

Recommendation: first implementation = automatic treatment, **body y+ ~ 1 and fin sides y+ ~ 15 on Coarse, ~ 3 on Medium**. What the alternative buys: pure wall functions would
save another 20 % of cells and lose parity; pure resolved costs +65 % cells (nk 18 -> 30) for the same answer within the gate bands. The Menter-Esch y+ sweep is a gate
(G-V2: Cf within 5 % of the y+ = 0.5 value at y+ in {1, 5, 15, 30}); if it fails, fall back to y+ 3 on fins and accept nk 26.

## C. Physics port

| piece | 2D source | 3D action | reuse |
|---|---|---|---|
| gas, Sutherland, `PRANDTL`, `PRANDTL_T`, `Viscous` | `viscous.rs`, `solver.rs` | `Viscous` moves to `viscous3.rs` unchanged (nose diameter units, `re_c`, `sutherland`, `roughness`, `wall_i_start`) | verbatim |
| cell gradients | `cell_pass`: Green-Gauss with `phi_f = (phi_c + phi_nb)/2`, meridian plane | same on hexahedra: `g = (1/V) sum_f phi_f S_f` for `(u, v, w, T)` (+ `k`, `omega`); Cartesian 3-vectors, so no hoop terms, no `1/r` | generalised |
| face gradient | `face_pass`: `g_f = gbar + ((phi_R - phi_L)/dist - gbar.e) e`, ghost = mirror image of the cell centre in the face centre | identical formula with 3-vectors; `dist`, `e` per face precomputed (`VDir` -> `VDir3`) | generalised |
| viscous flux | `Fv.n` with `txx, trr, txr`, `t_tt` | full tensor `tau_ij = mu_e(2 S_ij - 2/3 div delta)`, `q = -kappa grad T`, energy `u.tau + q`; nk = 1 recovers 2D to `O(dtheta^2)` | rewritten |
| walls | `Walls` (jface at `j_end`, base iface, sting) | `Walls3`: **any face between fluid and solid is no-slip** (body, base, fin sides/LE/TE/tip); symmetry planes and axis stay slip; wall ghost `(rho, -u, -v, -w, p)` adiabatic | rewritten (smaller) |
| SST-2003 | `turbulence.rs` constants `A1, BETA_STAR, SIGMA_*, BETA1/2, GAMMA1/2`, `F1/F2`, `mu_t = min(rho a1 k/max(a1 w, S F2), 1e5/re_c)`, `P = min(mu_t S^2, 10 beta* rho k w)` | `S^2 = 2 S_ij S_ij` full 3D; `X = 2 rho sigma_w2 grad k . grad w / w`; `k_inf = 9e-9`, `omega_inf = 1e-6 re_c` unchanged | constants verbatim, kernels generalised |
| wall distance | brute force over segments (`wall_distance`) | exact **point-quad distance by BVH/bucket grid once per grid** on the CPU (0.4-0.8 M queries x log 12k [E: < 1 s]); the grid is Mach-independent when sized for `mach_max` (`Analysis::grid_mach`) so it is shared by the table | rewritten |
| Aupoix rough walls | `aupoix_wall(nu, abs(u_c), d, ks)`, ramp `ROUGH_RAMP_CELLS = 8` along the body | the same function on every no-slip face (fin faces carry `FinSpec::roughness`, fins floored per `Analysis::roughness` logic); body ramp kept (stability measure), fin ramp over the first 4 chordwise cells [E] | verbatim |
| low Mach | flux correction `z = clamp(max(M_L, M_R), LOW_MACH_FLOOR, 1)` (`low_mach_states3` exists) | keep; add 5x5 Weiss-Smith preconditioning of the implicit side below Mach 1 (section D) | ported from `implicit.rs: LowMach` |
| frame limiting | `reconstruct3` limits `(v_r, v_t)` in the cylindrical frame at the face centroid | keep (needed for wedge exactness; harmless with fins) | verbatim |
| freestream, far field | `characteristic_ghost3` | keep; for alpha != 0 the free-stream vector is `freestream3(M, alpha)` | verbatim |

Reused verbatim: limiters (`flux.rs::limited_slope`, `shock_weight`), HLLC/Rusanov/blend (`flux3.rs`), `bc3.rs`, SST constants and Aupoix, `Viscous`, `ImplicitParams`. Rewritten: gradients/fluxes
(3-vector), `Walls3`, wall distance, turbulence transport (three directions), force integration (viscous). The turbulence module is written as a **list of scalar transport equations**
(`k`, `omega`, later `gamma`), so transition is an addition, not a rewrite.

## D. Implicit scheme

* Both line solves keep their structure: `implicit3.rs::step` (block, `du_block`) and `diag3::du3` (diagonalised, GPU default via `Numerics3::implicit_for(gpu)`). Viscous terms enter as
  the 2D does: the Roe dissipation of each face gets `+ 2 lam_f I` (block) or the eigenvalue becomes `|lambda| + 2 lam_f` (diagonal form: a scalar add, no new eigensystem), `lam_f =
  gamma (mu_f/Pr + mu_t,f/Pr_t)/(rho_f |d|)`; `compute_dt` adds `diffusion_rate` per direction (`viscous.rs::diffusion_rate`). Wall faces: ghost Jacobian `diag(1,-1,-1,-1,1)`.
* Turbulence: loosely coupled after the mean-flow update (as 2D), scalar tridiagonal solves in **three** directions per scalar (2D: two): 6 solves per iteration (`solve_tridiagonal_parts`).
* **Low-Mach preconditioning (new, 5x5).** `implicit.rs::LowMach/precond_matrix/precond_inverse` are 4x4; the 5x5 `P` is the same with `v` and `w`. Block path: copy of the 2D
  construction. Diagonal path: the eigensystem of `P^-1 A` changes (speeds `u', c'` of `precond_speeds`), so `diag3` needs the preconditioned `T`; **risk**. Fallback: below Mach 1 use `Block` on the GPU
  too (+63 % step, 3151 vs 1935 us [M: README 3D]), still far cheaper than 14 100 vs 1 200 iterations [M: ISSUES #19].
* Stability limits known: 3D Euler `cfl_max` 10 for every Mach (diverges at 15 on coarse ANF [M: `case3.rs` doc]); `LineSolve3::DiagonalCorrected` diverges on the f64 CPU at 10, stable at 5 [M: ISSUES #20] so the
  CPU reference uses `Block`; 2D RANS `cfl_max` 50, 20 supersonic (axis-adjacent wake rows oscillate above CFL ~ 40, ISSUES #12 [M]). Plan: start 3D RANS at `cfl_max` 5 (subsonic 10 with
  preconditioning), supersonic 5, `DtMode::Sum`, `first_order_iters` 100, `sub_iters` 2; raise per gate on iteration counts. Last resorts already named in the README: k direction point-implicit.
* Axis rows in 3D: sliver cells `r dtheta` wide; the k factor is implicit so the CFL is not limited by them, but `compute_dt` sums `L_k`; the wedge drops it (A2).
* **Stop rule for drag in 3D**: extend `StopRule3` (`run3.rs`: samples every 50, `window`/floor 250, CN) with the 2D monitor of `engine/run.rs::stop_reason`: monitor vector `[CD, CN, Cm]`, stationary when
  `(max-min)/|mean|` of each over 500 iterations is below 0.2 % (CD) / 0.2 % (CN) after >= 2 orders of residual drop, plus the 2D mean-stationary rule for limit cycles
  (`Numerics::mean_window`, base wake ~1 % cycle at M 0.1 before preconditioning). Wedge: CD only. Force monitor = on-device wall-force reduction (E), not a host download of `u`.

## E. GPU port in Luma array ops

* Convention kept: every GPU routine mirrors the CPU routine expression by expression (`gpu/body3.rs` header). New files: `gpu/viscous3.rs`, `gpu/turbulence3.rs`, `gpu/force3.rs`; edited:
  `body3.rs` (axis map, `Walls3` masks, wedge mode), `implicit3.rs` (viscous dissipation, preconditioning), `mod.rs` (arrays, state grows by `k, omega, mu_t, res_k, res_w` as in 2D).
* Arrays per cell (f32-equivalents) [E: counted from the 2D/3D structs]: state 5, padded primitive 5, inviscid fluxes 3 x 5, viscous fluxes 3 x 4, gradients 12 + 6 (`k`, `omega`), `mu, mu_t` 2,
  `k, omega, res` 6, wall distance 1, geometry (3 face normals + 3 areas + volume + centroid + `face_rot` + `VDir3`) ~ 40, implicit face matrices (cached `Fused[50]` x 3 in the block path
  [M: README]) 0-150. **0.6-1.0 KB/cell** (df32 x ~1.6) -> sector coarse 0.3-0.45 GB, half domain 1.0-1.7 GB, Level 1 sector 0.8-1.2 GB; target < 4 GB holds for everything offered. To be measured in S5.
* Launches per step: 3D Euler 27-30 [M: ledger], 2D viscous 65 (Euler 27 + 38 for viscous and SST) [M: ISSUES #10]; 3D viscous SST estimated **80-100** [E: 27 + 38 + third direction].
* Time per step on the RTX 3090:

| case | cells | step | basis |
|---|---|---|---|
| 3D Euler ANF, f32, `DiagonalCorrected` | 352 800 | **1.94 ms** (5.5 ns/cell) | [M: README 3D, ISSUES #20] |
| same, `Block` | 352 800 | 3.15 ms (f32), 4.00 ms (df32) | [M: ledger 2026-11-16] |
| 2D RANS wedge-equivalent, df32 | 21 840 | 0.82 ms (launch-bound) | [M: 17.1 s / 20 950 iterations, ISSUES #19] |
| 2D viscous marginal cost / cell vs inviscid | - | 14.9 vs 5.5 ns/cell = 2.7x (f32, level 0 -> 1) | [M: ISSUES #10: 480 -> 764 us for 19 032 -> 38 064 cells] |
| 3D wedge RANS | 21 840 | 1.0-1.3 ms | [E: 2D + 20-40 % launches] |
| 3D RANS sector coarse | 443 000 | 5.3-8 ms f32 (12-18 ns/cell), +25 % df32 | [E: 1.94 ms x 2.7 scaled; line solves are 73 % of the Euler step and do not scale with the viscous terms, hence 12 not 15] |
| 3D RANS half domain | 1.7 M | 20-31 ms | [E] |

* **Seconds per Mach point / table** [E; iterations 2 000-3 000 wedge (2D: 20 950 per 10 points with preconditioning [M]), 3 000 sector (3D Euler needs 765 on average [M: ledger], RANS and CFL 5-10 cost 3-5x), 2 000 half domain]:
  wedge 2-4 s point, 20-45 s table; sector 16-24 s point, **2.7-4.0 min table**; with S9 sequencing (x0.4) 1.1-1.6 min; half domain coarse 41-61 s point (16 points 11-16 min), lift tier 12-17 s point (3-4.5 min).
  The 5-minute target holds in the estimate only with fin y+ ~ 15, <= 3 000 iterations and one shared grid per table; with y+ 1 (nk 30) it fails (4.5-6.7 min) without S9.
* f32 vs df32: today the facade runs 2D RANS in df32 (`Precision::Extended`) and 3D lift in f32 (CN within 0.007 %, f32 residual plateau at 4.5 orders [M: README 3D]). Decide by gate in S9: f32 if
  CD within 0.2 % of df32 on all gate cases (df32 costs +25 % [M]).
* Luma: the tree builds the **released 0.4.1** (section 0 item 10); all measured numbers here are for it. Not in 0.4.1 (local Luma `CHANGELOG.md` Unreleased): `permute_axes`, shared-expression kernel cuts (3D implicit step 4.86 -> 3.28 ms uncached, 220 fusion
  cuts -> 29 launches), strided Thomas up to 1.5x, 48 KiB cyclic lines; the Luma agent should cut a release so S9 can pin it. **Missing or wanted** (for the Luma agent): (1) periodic block-tridiagonal / cyclic parts solve (cant, periodic sectors) - none in
  `linalg.rs`; (2) strided restriction (`slice` with step) or a 2x2x2 averaging op for grid sequencing (today: sum of 8 offset slices, 8 launches per field [E]); (3) compensated `sum` for f32 so f32 runs reach the 6-order residual criterion [M: Temp.md];
  (4) factor-once block solves (Temp.md suggestion 1, ~0.35 ms of 3.15 ms); (5) a way to mark launches independent. Repro for any of them: `lift3d --case anf --level 0 --bench 200`.
* **Per-call reuse.** The app calls `Analysis::run` once per Mach node (section 0 item 7), so a point must not pay the grid, the wall distance, the upload and the kernel build again: a process-wide cache (`OnceLock<Mutex<..>>` in the engine) keyed by the hash of
  `(body, fins, tier, grid_mach, domain)` holds `Grid3` + wall distance + `AxisMap` + the `GpuRunner3` (its `set_config` already changes free stream, CFL, limiter without touching geometry [M: README 3D]). `Analysis::grid_mach` (exists) makes all nodes of a table one grid shape
  (`SHARED_GRID` is `false` today because the 2D grid moved Cd by +2.5 % at M 0.1; the 3D grid is sized for `mach_max` by design). Per-point setup then 0.5-2 s [E: grid build O(cells) + upload], kernel build once per process (9.5 s for the 3D Euler solver [M: Temp.md]; viscous ~2-3x [E]).
* GPU ghost/force plumbing: wall faces as host-built index lists, `take` of two cells per face, class masks, `sum_where` per class: 7 classes x 6 components every 50 iterations (< 0.1 ms).

## F. Forces, results, facade, what happens to the old code

**Wall forces.** Every fluid/solid face and `Base` face: `dF = -(p - p_inf) n_out A + tau_w`, `tau_w` from the viscous flux `Fv.n` of the face (momentum components), pressure extrapolated from the first two cells as
`run3.rs::lift_forces_upto` does (`pw = p0 + (p0-p1) d0/(d1-d0)`); classes: `Nose` (x <= nose length), `Aft`, `Base`, `FinSide`, `FinLE`, `FinTE`, `FinTip`. Outputs per class: `[Fx, Fy, Fz]` pressure and shear separately,
moments about a given reference (pitch `Mz`, roll `Mx` for cant later). Coefficients on `S_ref`; wedge/sector forces scaled by `2pi/Theta` (wedge: x and radial parts only, `Fy = Fz = 0` by symmetry), half domain by 2.

**Types** (`bowshock-core/src/analysis.rs`, serde under feature `serde`):
* `Drag`: unchanged names; `fins: Option<FinDrag{friction, pressure}>` now holds the **direct** fin-surface drag (sides + tip friction; LE/TE/side pressure); interference shows up in the body parts. New `Drag.fin_increment: Option<f64>` (`#[serde(default)]`) =
  `CD(finned) - CD(finless wedge at the same Mach)`, computed only with `Analysis::fin_increment(true)` (a wedge point is 20 x cheaper than a sector point). `base_modelled` stays for Euler.
* `PolarPoint`: `alpha` becomes real (not "always 0"); new `forces: Forces{cn, cy, cm, cl_roll, x_cp, ca}` (`#[serde(default)]`, zeros/NaN at alpha = 0 where CN = 0). `PointDetail.cd_aft`, `wall: Vec<WallCp>` stay (wedge: body wall row; with fins: the fin mid-plane / pitch-plane row).
* **One entry**: `Analysis::run(&[Condition{mach, alpha_deg}]) -> Polar`. Deleted without shim: `run_lift`, `LiftPoint`, `LiftPolar`, `Analysis::lift_case_hook`, `Wake::Sting` + `Analysis::wake` (base wake is the only grid), `engine/{case,run,gpu,numerics}.rs`
  (the 3D `case3/run3/gpu3/grid3` are renamed to those names), `fins.rs::fin_drag` and `Analysis::fins`'s empirical use (the builder keeps the geometry). `Backend::{Cpu,Gpu,Auto}` and `Resolution` stay;
  `Resolution` maps to `(level, tier)` where the tier fixes y+ of walls and the nk policy.
* `field.rs`: see F2. The meridian `Field` (`x, r, ni, nj, solid, rho, u, v, p`) stays for the existing viewer and is **derived** from the 3D solution (mid-gap plane `k = nk-1` of a sector, pitch plane of the half domain, the one cell of the wedge with `v_r` from the Cartesian pair); `Field.wake` is removed.
  `Analysis::engine_cases` and `initial_field` (free-stream field) change to the new case type; `keep_fields`/`on_field`/`SNAPSHOT_INTERVAL` 500 keep their contracts.
* `AnalysisSpec` (`analysis.rs`, serde/CBOR mirror used by `Analysis::spec/from_spec` and the worker): drop `wake`; new knobs (`fin_increment`, `tier`, `field_detail`) are added with `#[serde(default)]` and to `bowshock-core/tests/serde_roundtrip.rs`
  (`an_analysis_spec_round_trips_through_json_and_rebuilds_the_analysis`, `from_spec_of_spec_gives_the_same_run`, `viscous_polar_round_trips_equal`, `nan_fields_of_an_euler_point_come_back_as_nan` extended with a finned alpha point). `bowshock-remote/src/proto.rs`:
  `Job::{Drag, Lift}` -> `Job::Solve{spec, conditions}`, `WorkerMsg::{Drag, Lift}` -> `WorkerMsg::Solve(Polar)`, `client.rs`/`analyze/cfd/remote.rs` `Session::{drag, lift}` -> `solve`; **`PROTOCOL` 1 -> 2** at S8 (the `Hello` handshake already refuses another number; the worker re-keys
  itself from the source hash, so no manual rebuild step). No new crate is added, so `sourceset::CRATES` is unchanged.
* App/model: `analyze/cfd/run.rs` (`solve`, `solve_lift`, `reduce_lift` which maps `solver::LiftPoint` -> `CfdLiftPoint`) and `share.rs` nodes become one `run(&[Condition])` per node (alpha 0 on the sector, `LIFT_ALPHAS` on the half domain); `CfdPoint` keeps
  `{mach, cd, friction, pressure, base, fins, other, quality}` with `fins` = direct fin drag and `other` shrunk to lugs/buttons/unsolved sets, plus `#[serde(default)] interference: f64`; `CfdLiftPoint` (`bowshock-model/src/cfd.rs`) gains `#[serde(default)] cd: Option<f64>, cm: Option<f64>` so
  Simulate can fly CD(alpha). Additive, so `FORMAT_VERSION` stays 1. `analyze/cfd/plan.rs::plan` drops the "equivalent trapezoid" notes when S11 lands; `fin_spec` allows `count` 1..=16.
* **Stored results to bump** (`analyze/cfd/plan.rs`): `SOLVER_REV` 5 -> 6 at S6 (fin drag changes), -> 7 at S10 (2D removed: every stored table is stale); `LIFT_REV` 4 -> 5 at S7; `PROTOCOL` 1 -> 2 at S8; `Plan::hash` and the `analyze/cfd/tests` follow.

**What happens to `fins.rs`**: removed at S8 (after G-F4/F5 pass). `bowshock-reference::fin_wave` (Ackeret) and `base_drag` stay as references for gates; `bowshock-sim` build-up keeps its own fin model (Extended method), untouched.
**Analysis::run and run_lift become one path**: both call `Case::new(mach, alpha, body, fins, tier)` -> domain classifier (wedge / sector / half / later full) -> `Solver3` CPU or `GpuRunner3` -> `PointResult` with forces; progress, cancel, `on_progress`, `on_field` keep their contracts (`progress.rs`, `SAMPLE_INTERVAL` 50).

### F2 3D field and wall-surface export (owed to the flow viewer; contract to be agreed with `app-integrate`, this is the solver side's proposal)

Assumed contract: structured node coordinates, cell primitives, solid mask, wall faces with Cp, skin friction and a part tag, and the symmetry of the solved domain so the viewer can replicate it. Proposed (in `bowshock-core/src/field.rs`, serde, `f32`):
```
Field3  { mach, alpha, iteration, domain: Domain, ni, nj, nk, x, y, z: Vec<f32> /* nodes, (ni+1)(nj+1)(nk+1), i fastest, metres */,
          solid: Vec<bool>, rho, u, v, w, p: Vec<f32> /* cells, solver units like Field */, mu_t: Option<Vec<f32>>, freestream: [f64; 5], surface: Surface }
Domain  { Wedge{theta} | Sector{n, theta, phase} | Half{n, phase} | Full }      // viewer replicates: rotations by 2*theta about x + mirror in the k planes (Wedge: any angle)
Surface { faces: Vec<WallFace> }  WallFace { c: [f32;3], n: [f32;3] /* out of the body */, area: f32, cp: f32, cf: f32 /* |tau_w|/q_inf */, tau: [f32;3], tag: PartTag }
PartTag { Nose, Aft(segment), Base, FinSide(set), FinLE, FinTE, FinTip, Jet }
```
* `Surface` is built from **the same wall-face list and the same extrapolated wall pressure/shear as the force integration** (section F, wall forces), so the viewer's `sum(cp n A)` reproduces the reported forces (a unit test, G-A). `cf` from the first-cell shear `(mu + mu_t,w) U_t / d`.
* Size [E: 5 prim x 4 B + 1 B solid per cell, 12 B per node, 48 B per face]: wedge 21.8 k cells 0.5 MB (today's `Field`); sector coarse 0.443 M cells ~16 MB; half domain 1.7 M cells ~62 MB. A 10-point sector table is 160 MB in the session, a half-domain table 620 MB, and `proto.rs` frames are capped at 256 MB:
  `Polar` can no longer carry every field. The field leaves `PolarPoint` on the wire: `WorkerMsg::PointField{condition, Field3}` per point (<= 70 MB), `Polar.points[i].field` stays `None` remotely; `field_detail` selects `Surface` only (1 MB), a meridian `Field`, or `Field3` with optional stride-2 cells.
* Live snapshots (`on_field`, every 500 iterations): GPU read-back of 5 arrays is ~9 MB (1-2 ms [E]); send the `Surface` + meridian slice live, `Field3` only at the end.
* Delivery: S8 gives the derived `Field` + `Surface` (the viewer keeps working and gets part-tagged wall Cp/Cf); S8b (+300 lines) gives `Field3` once the contract is signed. Not started before the director sends the agreed contract.

### F3 The 2D deletion stage (S10) and what migrates

| deleted | migrates / replaced by |
|---|---|
| `bowshock-cfd`: `solver.rs, implicit.rs, viscous.rs, turbulence.rs, multigrid.rs, bc.rs, gas.rs` + `gpu/{body,implicit,viscous,turbulence,control}.rs` (~4 600 + ~2 950 lines) | **first move shared pieces**: limiters/`shock_weight`/`LOW_MACH_FLOOR` (`flux.rs`) -> `limiter.rs`; `GAMMA` -> `gas3.rs`; SST constants + `aupoix_wall` -> `sst.rs`; `Grid` + `cone_grid`/`geometric_offsets` stay as the meridian generator; `ImplicitParams/DtMode/LineSolve3` -> `implicit3.rs`; `gpu/body.rs` helpers (`S, c, up, to_f64, A`, `G1`, `no_state`, `Residuals`) -> `gpu/common.rs` (body3.rs includes them today) |
| `engine/{case,run,gpu,numerics}.rs`, `Wake`, 2D `Case`, `PointResult` (2D), `Analysis` 2D paths | 3D engine renamed; `StopReason`, `WallPoint`, `BasePoint` (`result.rs`) kept |
| `bowshock-cases` (`cone`, `ogive`, `transonic`, `subsonic`), `bowshock-validate/src/bin/*_gpu`, `bowshock-gpu-cases` (`cone.rs, ogive.rs, run.rs`, tests `explicit, implicit, implicit_lowmach, loops, runs, viscous`) | each case is re-run on the wedge (Euler first, in S1; RANS in S5) against its original tolerance (README tables: Taylor-Maccoll cone, Cp vs shock expansion, Stoney, Fox TM X-2439, G1/G2/G3); a case that the wedge reproduces is re-pointed (`Solver3` / `GpuSolver3`), the rest is deleted with its README table. `bowshock-gpu-cases/tests/viscous.rs` -> `viscous3.rs` (GPU vs CPU) |
| `bowshock-validate/examples` `bench, bench_viscous, eval_loop, perfcheck` | replaced by `bench3` (wedge + sector step, `PERF` ledger lines) |
| `bowshock-core/tests`: `viscous.rs` (G3, motor, roughness), `validation.rs` (Arcas, TP-2005), `long_body.rs`, `field.rs`, `sweep.rs`, `robustness.rs` | **keep all**, they run through the facade on the wedge path; `lift.rs`, `run_lift.rs` merge into `analysis.rs` tests |
| what the wedge must match first | S0 fixture (`ref2d.json`): G3 CD 0.21300 (friction 0.09003, base 0.12257, cp_base -0.1226, y+max 0.648 [M: README]) CPU = GPU; the 10-Mach example table (CD per Mach, 3 parts); Arcas fins-off sweep; TP-2005 jet cases; G1 plate; G2 plate (Cf within 5 % of the correlation, y+ < 2); rough plate G2r (within 10 % of Mills-Hang); 2D Euler base cases at M 0.5/0.9/1.3/2.0 |

## G. Validation: exact gates (each an `#[ignore]`d test or example with a number; `tools/gpuq/gpuq submit` runs them unattended, one process per case: ISSUES #1 can hang a process at exit, retry once)

| id | stage | case (reuse / new) | pass |
|---|---|---|---|
| G-W1 | S1 | **new** `wedge_euler_matches_2d`: finless ogive-cylinder + boattail + tube + transition bodies (`tests/viscous.rs::spec`, `arcas_body`, `long_body`), M 0.5, 0.9, 1.3, 2.0, Euler, 2D base grid vs wedge | total pressure CD and each of nose/aft/base within 2 % (abs 0.002 near zero) |
| G-W2 | S1 | wedge angle 2, 6, 12 deg | CD within 0.3 % |
| G-W3 | S1 | N = 4 sector vs two-fin half domain at alpha = 0 (Euler, Arcas-like fins); N = 3 vs N = 1 sector; free-stream preservation on revolved base grid (`solver3.rs` test pattern, 1e-15) | CD within 1 %; half-domain CN <= 1e-4; freestream <= 1e-13 |
| G-W4 | S1 | existing `bowshock-cases` cone (Taylor-Maccoll), ogive (shock expansion), transonic, subsonic tables on the wedge | original tolerance of each table (README) |
| G-V1 | S3 | `viscous.rs::g1_laminar_flat_plate_matches_blasius` on a 3D box plate; wedge laminar body vs 2D CPU state | Cf within 3 % of Blasius; wedge vs 2D state `max abs(dU)/(1+abs(U))` <= 5e-4 at 2 deg, ratio 4 per halving |
| G-V2 | S4 | `turbulence.rs` G2 plate (Re_x 4.85e6, Mach 0.2) on 3D plate; y+ sweep {0.5, 1, 5, 15, 30}; G2r rough plate (Mills-Hang) | Cf within 5 % of the correlation at y+ <= 2, within 5 % of the y+ 0.5 value at 15 and 30; rough within 10 % |
| G-V3 | S4/S5 | `g3_ogive_cylinder_friction_and_base_pressure` + README G3 (CD 0.21300) CPU and GPU | CD within 2 %, friction within 3 %, base within 4 %, y+max < 3 |
| G-V4 | S5 | S0 fixture: example 10-Mach table, subsonic/transonic/supersonic, Euler and RANS | each CD within 2 % of the 2D value (3 % at M 0.9-1.1, where the 2D wake cycles) |
| G-V5 | S5 | GPU vs CPU: tiny wedge and tiny sector (`euler3d.rs` pattern, 30 iterations from free stream), f32 and df32 | state `max abs(dU)/(1+abs(U))` <= 1e-3 / 1e-4 implicit (README 3D table: 1.5e-6 / 2.9e-8 today), `k, omega` <= 1e-2 / 1e-3 |
| G-F1 | S6 | fin skin friction: thin fin (t/c 2 %, N = 4), zero thickness limit, M 0.5 and 2, Re_c 1e6, fully turbulent: direct `FinDrag.friction` vs `2 Cf(Re_c, M) * A_fin * N` (Mills-Hang / Van Driest II compressible Cf) | within 10 % (band [0.92, 1.12]) |
| G-F2 | S6 | sharp double-wedge fin, Euler, M 1.5, 2, 3: **pressure** drag vs `bowshock_reference::fin_wave` / shock-expansion for a wedge | within 8 % (this decides A4) |
| G-F3 | S6 | blunt Square fin: LE stagnation drag at M 0.5, 1.5, 3 vs `fins.rs` formulas (Niskanen 3.85-3.94, then deleted: copy the three numbers into the test) | within 25 %, reported; Rounded < Square at M 0.5 |
| G-F4 | S6 | `validation.rs::arcas_axial_force` through the new path: fins on/off, NASA TN D-4013 (M 0.6..1.2) / D-4014 (1.5..2.96) [M: `bowshock-reference::arcas`]; fins-on `C_A` without base, and the **increment on - off** | on: existing tolerance 15 %; increment: within 30 % or 0.03 abs at M 0.6, 0.8, 1.5, 1.8, 2.3, 2.96 (data increments 0.078, 0.099, 0.165, 0.116, 0.091, 0.063, reading +-0.005..0.012); M 0.9-1.2 reported |
| G-F5 | S7 | Army-Navy Basic Finner CD0 (`Case3::anf_body/anf_fins`, 0 deg roll, fully turbulent, Re from the tests): M 0.5-0.7: 0.46 [R: Bhagwandin-Sahu via Abiza et al.]; supersonic range fits DREV-TM-9703 Table IX C_X0 0.585 / 0.484 / 0.435 / 0.373 at M 1.832 / 2.375 / 2.718 / 3.147 [S: quoted in fusionspace-eridanus issue #1513] | within 8 % (shots scatter 3-7 %; range models are canted 2 deg); transonic reported |
| G-L1 | S7 | ANF CN_alpha, x_cp at alpha 2 and 6 deg on the digitised WT/FF tables (`bowshock-reference::anf`, +-3 % digitisation, tunnel-to-tunnel spread 11 % / 0.4 cal) | **no worse than the Euler gate** [M: `out/gate/anf_3d.md`: WT 5/5 and FF 2/3 gated points pass]: 13 % CN_alpha, 0.25 cal x_cp at M <= 0.8 and >= 1.5; transonic reported; plus roll 0 and 45 deg |
| G-L2 | S7 | `lift.rs` tests: CN(+a) = -CN(-a), CN(0) = 0 (<= 1e-4), pitching moment consistent with `x_cp` | as test |
| G-P1 | S9 | example finned rocket, 10 Mach nodes (`analyze/cfd/plan.rs::nodes`), coarse tier, RTX 3090 alone, shared grid | table wall <= 300 s; peak device memory < 4 GB (`nvidia-smi`); wedge table <= 60 s |
| G-A | every stage | `cargo nextest run --workspace` (CPU tests), `tools/gpuq` GPU tests, `cargo +stable fmt --all --check`, `cargo +stable clippy --workspace --all-targets -- -D warnings` (CI, stable 1.100; the one existing error `analyze/flow_view/prepared.rs:283` belongs to the app agent), `tests/serde_roundtrip.rs` extended with every new serde field | green |
| report | S8/S9 | `tools/compare-openrocket/run.sh` + `tools/openrocket-harness` (jar at `~/.local/share/openrocket/OpenRocket-23.09.jar`): fin-drag increment of the CFD vs OpenRocket on `simple_model_rocket`, `dual_parachute`, `tarc_payload` | not gated; table in `docs/openrocket-comparison.md` |

## H. Stages (each ends compiling, tested, committed, independently verifiable)

**Order: CPU first per physics feature, GPU immediately after, not "all CPU then all GPU".** Reason: a 3D RANS CPU run is ~0.5 M cells x ~1 us x 3 000 iterations ~ 25 min per point [E: 2D CPU 4 000 iterations of 12 k cells = 27 s,
README], so the CPU solver can only be an oracle on tiny grids; every number that matters (Arcas, ANF, tables) needs the GPU. Fastest route to a *validated GPU result* is S0 -> S1 -> S2 -> S3 -> S4 -> S5 (wedge G3 on the GPU = 2D value);
S3 (CPU laminar) and a GPU laminar port can overlap, and S4 (CPU SST) overlaps the GPU laminar port. Mechanical = a weaker implementer can do it from this spec (Sonnet-class); **bold** = needs me.

| stage | content | files | size [E] | main risk -> fallback | mech. |
|---|---|---|---|---|---|
| S0 | freeze 2D reference: `ref2d.json` (cases of the last row of F3) + generator example + gate test skeletons | `bowshock-validate/examples/ref2d.rs`, `bowshock-core/tests/wedge_gates.rs`, `bowshock-cfd/fixtures/` | 350 | none (2D unchanged) | yes |
| S1 | CPU topology, Euler: axis runs + `AxisMap`, `revolve_base` (Block -> SolidBox), wedge mesher (`nk = 1`, `dtheta`), sector `layers` (half fin at k = 0, N 1..16, odd interpolation), roll phase, wedge mode (no k factor), force classes incl. base | `grid3.rs`, `solver3.rs`, `implicit3.rs`, `engine/{grid3,case3,run3}.rs`, `body_grid.rs` | 1 100 | axis ghost for odd N / wedge conditioning -> `nk = 2` wedge; odd N half domain only | **no** |
| S2 | GPU of S1: axis map, wedge mode, class forces on device; G-W1/3 on GPU vs 2D GPU Euler | `gpu/{body3,implicit3,force3,mod}.rs`, `gpu3.rs`, `tests/euler3d.rs` | 600 | Luma `take` + 2x2 multiply fusion -> host index as today | yes |
| S3 | CPU laminar viscous: gradients, face fluxes, `Walls3`, implicit viscous dissipation, `compute_dt` diffusion; box-plate and wedge tests | `viscous3.rs`, `solver3.rs`, `implicit3.rs`, `grid3.rs` | 800 | wedge equivalence tolerance (hoop gradient) -> tighten ghost distance rule | no |
| S4 | CPU SST + all-y+ wall treatment + Aupoix + BVH wall distance + 5x5 low-Mach (block); G-V2, G-V3 (CPU, 4 000 iterations) | `turbulence3.rs`, `walldist3.rs`, `implicit3.rs` | 1 100 | convergence of SST in thin k cells; stability at CFL 5 -> `first_order_iters` up, CFL 2 | **no** |
| S5 | GPU viscous + SST + preconditioning + wall-force reduction; G-V4, G-V5; memory and launch counts measured; **first validated GPU result** | `gpu/{viscous3,turbulence3,force3}.rs`, `body3.rs`, `implicit3.rs`, `diag3` GPU | 1 800 | Luma fusion cuts (cap 16 384 nodes, README #7), preconditioned diagonal form -> `Block` below Mach 1 on GPU | partly (mirror of CPU) |
| S6 | fins in RANS: fin k-stack (`y1_fin`, growth), sharp-edge floor + taper (A4), fin wall classes, roughness on fins, `Analysis::run` takes fins into the solver (empirical add-on still present, compared) | `engine/grid3.rs`, `solver3.rs`, `run3.rs`, `analysis.rs` | 800 | LE/TE spurious drag (G-F2) -> finer chord clustering; SST LE overproduction -> Kato-Launder flag | **no** |
| S7 | alpha != 0 viscous half domain, roll phase, lift tier, CN/CP/Cm from forces; ANF gates | `engine/{grid3,case3}.rs`, `run3.rs`, `analysis.rs` | 500 | iteration count at 1.7 M cells -> lift tier | no |
| S8 | **one path**: `Analysis::run(&[Condition])`, delete `run_lift`, `LiftPoint/LiftPolar`, `fins.rs`; `AnalysisSpec` knobs; `proto.rs` `Job::Solve` + `PROTOCOL` 2; `analyze/cfd/{run,share,remote,plan}.rs` one `run` per node; `CfdPoint/CfdLiftPoint` serde-default fields; derived meridian `Field` + `Surface` | `analysis.rs`, `field.rs`, `bowshock-model/src/cfd.rs`, `analyze/cfd/*`, `bowshock-remote/src/{proto,client}.rs`, `tests/serde_roundtrip.rs` | 1 200 | app tests that assert 2D numbers -> regenerate fixtures | yes (after S7) |
| S8b | `Field3` + `PointField` streaming (F2), after the director sends the agreed contract | `field.rs`, `proto.rs`, engine | 300 | frame size (256 MB cap) -> `Surface`-only or stride-2 fields | yes |
| S9 | perf: grid sequencing (semi-coarsening i, j), shared grid per table, tier tuning, f32 vs df32, memory; G-P1 | `engine/gpu3.rs`, `engine/grid3.rs`, `gpu/mod.rs` | 700 | fin layer alignment on the coarse grid (indices even) -> sequence only the finless wedge and the body | **no** |
| S9b | motor-on jet patch (port of 2D `Nozzle`) | `solver3.rs`, `gpu/body3.rs`, `engine/grid3.rs` | 300 | none (physics as 2D) | yes |
| S10 | delete 2D and migrate (F3); `SOLVER_REV` 7; README/HANDOFF/ISSUES rewrite | see F3 | -9 000 / +600 | a validation case the wedge does not reproduce -> drop it with its table, say so | yes |
| S11 | elliptical + freeform fins | `engine/grid3.rs`, `analyze/cfd/plan.rs` | 500 | corner cells at LE-tip -> keep equivalent trapezoid note | no |
| S12 | several fin sets, per-fin angles, symmetry classifier | `engine/{grid3,case3}.rs`, `plan.rs` | 400 | domain growth -> full 2 pi only on demand | no |
| S13 | diameter steps (chamfer) | `body_grid.rs`, `plan.rs` | 200 | cell skew -> reject steps > x mm | yes |
| later | cant + periodic sector (needs Luma), transition gamma, QCR, compressibility, lugs/buttons patch, pods (overset) | | | | |

S6 and S7 can swap if the director wants lift before drag; S9 may start in parallel with S7 (separate files) in a second worktree.

## I. Risks and the smallest useful end state

| risk (likelihood x damage) | mitigation |
|---|---|
| 3D RANS CFL 5 and thin-cell stiffness make 5-10 k iterations per point -> table 10+ min | S9 sequencing; preconditioning; fin y+ 15; shared grid; measure iteration counts in S5 on the wedge first |
| diagonal + preconditioning (S5) does not compose | `Block` below Mach 1 on GPU (+63 % step) |
| SST at fin LE/junction over-produces `k`, spurious fin pressure drag | G-F2/G-F3 early in S6; Kato-Launder production flag; sharper floor |
| axis (polar) rows diverge at high CFL in the supersonic wake (ISSUES #12 analogue) | `cfl_max` 5 supersonic; first-order axis rows |
| Luma fusion limits / compile time for a 100-launch step (cold start ~10-30 s [E; 9.5 s for 3D Euler, Temp.md]); the tree pins released 0.4.1, the Unreleased kernel-cut / Thomas gains are not in it | split kernels with `cache()` as `implicit.rs` does; ask the Luma agent for a release and bump the pin at S9 |
| reference data is weak: ANF digitised (+-3 %, spread 11 %), DREV tables only quoted second-hand, Arcas read from figures (+-0.005..0.02) | gates carry the data uncertainty; transonic reported not gated; scans of DRDC TM 2002-136 (web.archive.org, p518050.pdf) are the upgrade path |
| fully turbulent walls overpredict friction at model-rocket Re (laminar fins, Re_c 2e5-1e6) - the largest systematic error for the target user | disclosed (Research); gamma model is the first post-tonight item |
| NVIDIA exit hang (ISSUES #1), shared-state merges, long GPU jobs | one process per gate, retry once, commit per stage |
| time: S0-S10 is ~14 000 lines | stage order puts value first; S10/S11-S13 are optional |
| 3D field exports (16-62 MB per point) exceed what `Polar` and the 256 MB frame cap carry; the viewer contract is not signed | per-point `PointField` frames, `Surface`-only default, S8b after the contract (F2) |

**Smallest useful end state if the plan does not fit:** S0-S6, no deletion. Finless bodies keep the 2D solver (D3 original); finned bodies at alpha = 0 get the sector RANS with direct fin drag replacing the empirical add-on for N = 3..8 trapezoidal fins (`Analysis::run` routes on `fins.is_some()`), gated by G-F1..F4; the existing Euler `run_lift` is untouched. If S5 fails: S0-S4 stays as a CPU reference, nothing user-visible changes.

## Research (what I looked at, what I adopt tonight, what I defer)

Ranked by accuracy gained per unit of risk; "tonight" means inside S1-S9.

| # | topic | evidence | gain | risk | decision |
|---|---|---|---|---|---|
| 1 | automatic all-y+ wall treatment | omega/u_tau blending makes SST insensitive to y+ 0.1..100 [S: Esch-Menter 2003; Grotjans-Menter 1998; CFD-wiki]; WIND SST flat plate grid-independent at y+ 2, degrades beyond 5 [R: NASA GRC fpturb01] | fin nk 30 -> 18 (-40 % cells), Coarse tier possible | low | **adopt** (S4) |
| 2 | baseline model SST-2003 + Aupoix roughness | Menter-Kuntz-Langtry 2003 [R via TMR]; ARL/Kestrel/Cobalt ANF-class RANS use SST [R: Shelton-Martin-Silva NTRS 20180006293; ARL-TR-9939 abstract]; 2D parity | parity | none | **keep** |
| 3 | flux | HLLC + MUSCL + shock sensor + low-Mach velocity-jump scaling (exists); Roe low-Mach fix (Rieper 2011), Thornber 2008, HLLC+ (SISC 2019) [S] agree: fix the normal-velocity jump | none needed | - | **keep** |
| 4 | implicit side | three-factor AF with diagonalisation (Pulliam-Chaussee 1981) + defect correction (exists); Weiss-Smith preconditioning measured 14 100 -> 1 200 iterations at M 0.1 [M: ISSUES #19]; fine-grain parallel linear iterations on structured GPU grids [S: Kashi-Nadarajah, Comput. Fluids 299, 2025] favour line/ILU-type smoothers | subsonic points 10x | medium (diag form) | **port preconditioning (S4/S5)**; JFNK/GMRES **defer** |
| 5 | convergence acceleration | grid sequencing = standard in structured RANS (OVERFLOW/CFL3D practice) [S]; FAS multigrid more gain, but a 1-cell solid fin layer cannot be coarsened in k: semi-coarsening in i, j keeps it | 2-3x wall time [E] | medium | **adopt sequencing (S9)**, FAS defer |
| 6 | transition: Menter gamma (1 transport eq., local, Galilean-invariant; `Tu_L` from `k, omega, d_w`; `Re_theta,c` correlation; gamma-Re_theta is the 2-eq predecessor, Langtry-Menter 2009) | [R: NASA TMR "SST-2003-Menter-Gamma-2015" page, 2026-02-17; Menter et al., FTaC 95, 2015]; low-Re airfoils/props need transition models [S: ICAS 2024 paper 0327; FTaC 2022 gamma-Re_theta very-low-Re] | at Re_L 5e5 laminar Cf 0.0019 vs turbulent 0.0052 (2.7x) [E: Schlichting flat plate]: **largest systematic gain for model-rocket Re**; OpenRocket also treats fins as laminar below `Re_crit` | high: needs `Tu_inf`, roughness-induced transition (not in the model), no unattended low-Re drag data in hand; adds a scalar (+15 % step [E]) | **defer; hook built in (scalar list); first item after tonight** |
| 7 | QCR2000 / SA-RC-QCR for junction flow | improves corner separation, DPW-7 premature side-of-body separation [R: EUCASS2023-055; S: AIAA J. 2024 10.2514/1.J064610; R: NASA juncture-flow NTRS 20200002764] | < 1-2 % CD at alpha 0 [E] | +10 % step; calibration | defer, opt-in flag after G-F4 |
| 8 | rotation/curvature (SST-RC, Smirnov-Menter 2009), Kato-Launder production | tip/body vortices at alpha > 0; Kato-Launder is part of the gamma model [R: TMR] | CN, tip loss [E] small | low | defer; Kato-Launder as a flag if G-F3 fails |
| 9 | compressibility correction in the supersonic wake | SST overspreads compressible shear layers -> base pressure at M >= 2 | possibly the largest CD error at M 2-3 (base is 30-50 % of CD) [E] | changes 2D parity | evaluate after S5 against TP-2005/Arcas; defer |
| 10 | SA-neg | standard in DPW; no benefit for this class; SST has the Aupoix wall condition here | none | - | reject |
| 11 | geometry | structured body-fitted + revolved base grid + layered fins (exists): cheap, wall-resolved, GPU-regular. Overset (OVERFLOW; GPU port 2024 [R: Jackson, NTRS 20230016176, AIAA 2024-0042] found Chimera exchange the hard part) is the route for pods/lugs but a new subsystem. Immersed boundary / cut cell: geometry-free, but skin friction on IB walls oscillates unless non-equilibrium wall models are used [S: "Coupling of turbulence wall models and immersed boundaries on Cartesian grids"]; fins t/c ~ 1-2 % need cells << t | covers items 1-7 of A5 | - | **adopt structured**; overset for pods later; IB only as plan B |
| 12 | rocket-specific evidence | OpenRocket overpredicts sounding-rocket CD 12-73 % vs Fluent [S: KTH diva-1881325]; fin pressure drag is the overpredicted part [S]; ANF CD 0.46 within 1.2-2.8 % by LBM WMLES at M 0.5-0.7 [R: Abiza et al.] | target | - | gates G-F4/F5 |

Sources (URLs): TMR gamma `https://tmbwg.github.io/turbmodels/menter_gamma_3eqn.html`; Menter et al. `https://link.springer.com/article/10.1007/s10494-015-9622-4`; Shelton et al.
`https://ntrs.nasa.gov/api/citations/20180006293/downloads/20180006293.pdf`; Abiza et al. `https://www.3ds.com/assets/invest/2024-02/prediction_of_finned_projectile_aerodynamics_using_a_lattice-boltzmann_method_cfd_solution.pdf`;
Basic Finner tables `https://github.com/nrdptel/fusionspace-eridanus/blob/main/docs/research/basic-finner-transonic-tables.md`, issue `.../issues/1513`; DREV-TM-9703 `https://apps.dtic.mil/sti/citations/ADA636861`;
NASA WIND SST plate `https://www.grc.nasa.gov/www/wind/valid/fpturb/fpturb01/fpturb01.html`; CFD-wiki `https://www.cfd-online.com/Wiki/Near-wall_treatment_for_k-omega_models`; DPW-7 `https://www.eucass.eu/doi/EUCASS2023-055.pdf`;
juncture flow `https://ntrs.nasa.gov/api/citations/20200002764/downloads/20200002764.pdf`; QCR corner `https://arc.aiaa.org/doi/10.2514/1.J064610`; OVERFLOW GPU `https://ntrs.nasa.gov/api/citations/20230016176/downloads/202401_SciTech_Jackson_Final_v3.pdf`;
Rieper `https://www.sciencedirect.com/science/article/abs/pii/S0021999111001689`; HLLC+ `https://epubs.siam.org/doi/10.1137/18M119032X`; Kashi-Nadarajah `https://www.sciencedirect.com/science/article/pii/S0045793025001744` (fetch blocked, title/abstract from search);
KTH `https://kth.diva-portal.org/smash/get/diva2:1881325/FULLTEXT01.pdf`. Earlier classics cited from memory (not fetched): Menter 1994 AIAA J 32(8); Pulliam-Chaussee 1981 JCP 39; Aupoix 2015 J Fluids Eng 137 (in repo: `turbulence.rs::aupoix_wall`).

## J. Measurements still to take (no cargo while the merge is open; first things in S0/S1)

1. 3D RANS iteration counts per point (needs S5); 2. device memory per cell (S5); 3. launches per step with viscous + SST (S5 `profile_step`); 4. Luma HEAD vs the pinned copy on the 3D Euler step (1.94 ms f32 today with the old Luma);
5. wall-distance BVH time on 0.5 M cells (S4); 6. f32 vs df32 CD on gates (S9); 7. `dtheta` sensitivity (G-W2).

## K. Director decisions (2026-11-17)

Approved with these decisions; they supersede the sections above where they differ.

1. **Order for tonight:** S1 -> S5 (wedge parity on the GPU), S6 fins, S7 alpha, S8 one path, S9 perf (table <= 300 s), S10 delete 2D. After that: S14 gamma transition (opt-in), S11-S13, S15 (item 5 below). Stretch items never pre-empt S8-S10.
2. **Turbulence baseline (answer to Q1):** fully turbulent walls are the baseline tonight (the standard assumption for painted rockets; OpenRocket's too) and must be disclosed in the docs. Gamma transition is the first physics item after S10. Keep the scalar-list structure of the turbulence module so it is an addition.
3. **Tier policy (answer to Q2) approved:** `Resolution::Coarse` on a finned body = automatic wall treatment with fin y+ ~ 15; the alpha table runs on the lift tier and must not be worse than the existing Euler ANF gate (`out/gate/anf_3d.md`: WT 5/5, FF 2/3 gated points).
4. **Field export: ONE contract, `Field3`**, owned by the app agent as new files `crates/bowshock-core/src/field3.rs` and `engine/field3.rs` (it builds a `Field3` from `Grid3` + state + wall faces; `solver3d` adapts and wires it in S8). **No derived meridian `Field`**: the viewer gets a 2D mode fed from `Field3`, and `field.rs::Field` is deleted with the 2D solver in S10 (this replaces the "derived meridian `Field` + `Surface`" of F and F2 and the S8/S8b split). Wall faces are exported as quads (node indices) from the same face list as the force integration, with cp, the shear vector and a part tag; remote: the final field per point in its own frame; live snapshots local only. The final type comes from the director when the app agent's design is in; nothing in S1-S7 depends on it.
5. **Solids are a general per-cell mask end to end** (solver, GPU, forces, export) **plus a per-cell part id written by the mesher** (body, fin set n, appendage n). Wall classes come from part id + face direction, not from geometry tests such as `x <= nose length` inside the solver. Only the mesher knows boxes. This keeps the door open for S15: rasterised appendages (lugs, rail buttons, later pods as blockage) on the same grid, tiled by the sector symmetry but with their force counted once.
6. **Work split** (four sessions in parallel, each in its own worktree, file ownership strict):
   * `solver3d` (worktree `bowshock-cfd-3d`, branch `cfd-3d` = the integration branch): topology and mesher, `Solver3` integration of every physics piece, CPU implicit, engine (`engine/*` except `gpu3.rs`), facade (`analysis.rs`). Critical path: S1 now; then integrate the viscous kernels (S3) and the turbulence kernels (S4); S6; S7; the solver side of S8; S10.
   * `phys3d` (branch `cfd-3d-phys`): standalone CPU physics kernels as NEW files with unit tests, written against today's `Grid3` / `solid` mask / padded `w` conventions and independent of the axis representation: `viscous3.rs`, `walldist3.rs`, `turbulence3.rs` (SST-2003 + automatic wall treatment + Aupoix), `lowmach5.rs` (5x5 preconditioning matrices). `solver3d` integrates them into `Solver3` / `implicit3.rs` and does not write those kernels; any signature `solver3d` needs that differs from the 2D pattern is raised with the director early.
   * `gpu3d` (branch `cfd-3d-gpu`): S0 now (2D reference fixture, gate test skeletons), then every GPU port (`gpu/*`, `engine/gpu3.rs`, `bowshock-gpu-cases`, benches) trailing the CPU stages by one, and the perf work. `solver3d` writes no GPU code. CPU kernels stay plain per-face / per-cell functions, and each stage ends with a "port notes" list in the commit body (function -> what changed) so the port is mechanical.
   * `app-integrate` (branch `cfd-3d-viz`): viewer, app, later the app/remote side of S8.
   * The director directs merges into `cfd-3d` at stage boundaries; `solver3d` runs them in its worktree when told.
7. **Luma:** an A/B of the local Luma HEAD against crates.io 0.4.1 is running; if it passes the GPU suites the branch gets a `[patch.crates-io]` to a Luma worktree where Luma bugs are fixed by a separate agent. A crates.io release cannot be cut. Luma issues keep being reported to the director with a repro.
8. **GPU jobs of all sessions go through `tools/gpuq`** so they do not overlap and timings stay valid.
9. **Every stage, without exception:** `cargo nextest` for the crates touched, `cargo +stable fmt --all --check`, `cargo +stable clippy --workspace --all-targets -- -D warnings`, commit(s) on `cfd-3d`, never push. A numerics problem unresolved after ~30 minutes is written up with its evidence and either put to one Opus-class sub-agent or reported to the director; Sonnet/Haiku-class sub-agents only for mechanical slices, never Fable-class.
10. **Shell rule:** the session cwd is the main checkout (`/home/sandvoxel/Documents/bowshock`, branch `ui`, with the user's uncommitted files): nothing is written or run there; every command uses absolute paths or `cd /home/sandvoxel/Documents/bowshock-cfd-3d`.

## L. S1 as built (2026-11-18): what differs from sections A-H

1. **Wedge `k` faces carry the pressure only, and the pressure source is implicit in the `j` factor.** A Riemann flux across the wedge's two `k` faces (against the cell's own mirror image) carries acoustic waves round the wedge: explicit time stepping amplified the radial momentum ~70x per iteration at the first axis cell. The `k` faces of a wedge now carry `[0, p n]` (the axisymmetric hoop term, exact for the cell's pressure), `compute_dt` has no `L_k`, and the implicit `k` factor is the identity. The pressure source's Jacobian `(I - dt J)_mom += dt/V (A1 n1 - A0 n0) dp/dU`, `dp/dU = (g-1) [|u|^2/2, -u, -v, -w, 1]`, goes into the diagonal block of the **`j`** factor, as in 2D (`implicit::cell_blocks` `src`; `implicit3::assemble3`, `if wedge && dir == 1`). Measured on the CPU (f64, `Block`, base-wake wedge, M 0.5): without the block the iteration is stable to `cfl_max` ~3 and blows up at 6; with the same block as a separate `k` factor it diverged by iteration 50 at `cfl_max` 10; with the block in the `j` diagonal it converges at `cfl_max` 10 (`Numerics3::auto`) in 350-1600 iterations at all four Mach numbers. The GPU measurements of `gpu3d` (ISSUES #22: no `k` factor and no source block: stable only to `cfl_max` 1; keeping the `k` factor: stable at 10) agree: the missing piece is the implicit pressure source, either as a one-cell `k` line (GPU) or as the `j` diagonal block (CPU). `DiagonalCorrected` solves `i` and `j` diagonally and carries the pressure block only in its block defect operator (`diag3::du3`); untested. **For S3 (inference, not yet exercised):** the wedge's viscous hoop terms (`-2 mu v_r/r^2`, the hoop strain in the dissipation) have no `k` flux to live in; the viscous kernel needs an explicit axisymmetric source for the wedge, or `k` faces that carry the viscous flux against the mirror image (pressure only for the inviscid part).
2. **Cells upstream of the nose are dead cells, not walls.** The 2D body block has cells at negative radius upstream of the tip (nodes clamped to the axis by `revolve`). They are `PART_DEAD`: never updated, not walls, so the axis ghost shows through them (the second-order stencil of the axis row reads padded ghost states). `Grid3.solid` boxes still list them (the GPU masks solid boxes).
3. **Solids are a per-cell part id** (`Grid3.part`, `Part::{Body(segment), Sting, Fin(n), Appendage(n)}`, `PART_FLUID = 0`, `PART_DEAD = 255`); wall classes come from the part and the face direction (`Wall::{Body(s), Base, Sting, FinSide, FinLe, FinTe, FinTip, Appendage}`), never from geometry tests. On the sting grid the body is the `j = 0` wall, so those cells have no part: `Grid3.wall_j0` lists the `(i range, Part)` runs. `Grid3.axis_cells` is replaced by `Grid3.axis: Vec<AxisRun>` (a base-wake grid has two axis runs, upstream of the body and in the wake).
4. **Axis ghost.** `axis3::AxisMap` builds per axis cell the ghost's source: wedge: the cell itself, matrix `-I` on `(v, w)`; half domain: the mirror cell, `diag(-1, 1)` (the mirror plane images); sector, even `n`: the same cell with `-I`; odd `n`: the mirrored cell, interpolated linearly in the azimuth when the folded azimuth is not a cell centre (`fold` is the dihedral-group folding). At most two sources per ghost, each a `k`, a weight and a 2x2 matrix on `(v, w)`.
5. **Wedge angle default 2 deg, not 6 deg, and G-W2 is not met at 12 deg.** The flat-faced wedge cell has area = chord, volume = the polygon's, so the radial flux divergence is divided by `cos(d/2)`; the drag has a bias ~ `-0.17 d^2` to `-0.27 d^2` (G-W2). The same polygon bias shows in a half domain: 8 cells of 22.5 deg on the base grid are 2.9 % low at M 1.3, 24 cells of 7.5 deg 0.4 %. `Domain3::Wedge { dtheta }` stays free; correcting it exactly needs the true curved-sector areas and volumes (arc for the scalar fluxes, chord for the pressure vector) and is not done.
6. **Mesher.** `Domain3::{Half { phase: FinPhase::{Between, InPlane} }, Sector, Wedge { dtheta }}`; `Grid3Params.base` selects the revolved base-wake grid (finless in S1; fins on it are S6). Sector layers: the half fin sits on `k = 0` (`Part::Fin(0)`), the mid-gap plane on `k = nk` at `theta = pi/N`, N = 1..16 (N = 1 is the half domain). Sector `nk` counts the fin layer.
7. **Forces** are integrated per wall class (`run3::wall_forces`, `drag_parts`) and scaled by `2 pi / theta_span` (wedge: `2 pi / dtheta`; sector: `2 pi / (2 pi / N) = N`). Wedge and sector report `cn = 0`, `x_cp = NaN`. `run3_on` waits on `ca` instead of `cn` for wedges, sectors and for any half domain at zero incidence (CN is round-off there and never "stationary": the half domain on the base grid ran to its iteration cap before).
8. **The 2D Euler wake is not always steady.** The 2D solver's own cone-cylinder run at M 0.5 hits its iteration cap (10 000) with a base `CD` that has no steady value, so G-W1 compares nose and aft only there; at M 0.9 the cone-cylinder's base `CD` differs by 2.4 % from the 2D one with both stationary (total -1.84 %).
9. **G-W3 "N = 3 vs N = 1" is not an invariant.** The per-fin fin drag of a three-fin sector is 6 % (M 0.8) to 8 % (M 2) above a one-fin sector's on cells of 15 and 20 deg (fins interfere through the body flow; the coarse cells carry the polygon bias). The nose drag, supersonic, is invariant: N = 1..6 give 0.10170, 0.10230, 0.10271, 0.10340, 0.10379, 0.10405 (a smooth trend, no odd/even alternation, so the odd-`n` ghost is no outlier). The test is the nose-drag one.

### S1 gates, measured (CPU f64, Euler, level 0; `crates/bowshock-core/tests/wedge.rs`)

| gate | case | measured | band |
|---|---|---|---|
| G-W1 | boattailed ogive-cylinder, 2D vs 2 deg wedge, total CD, M 0.5 / 0.9 / 1.3 / 2.0 | +0.02 / -0.78 / -0.03 / -0.03 % (every part within 0.002 abs or 2 % of the total) | 2 % |
| G-W1 | cone-cylinder (ANF body), M 0.9 / 1.3 / 2.0 | -1.84 / -0.03 / -0.03 % (M 0.5: nose +0.00488 vs +0.00490, base not compared: 2D unsteady) | 2 % |
| G-W2 | 1 / 6 / 12 deg against 2 deg, M 0.9 | +0.016 / -0.175 / -0.758 % | 0.3 % (design) |
| G-W2 | same, M 2.0 | +0.025 / -0.267 / -1.172 % | 0.3 % (design): met to 6 deg, not at 12 deg |
| G-W3 | N = 2 InPlane sector vs half domain (same cells), M 0.8 / 2.0 | +0.02 / 0.00 % | 1 % |
| G-W3 | N = 4 Between, M 0.8 / 2.0 | -3.72 / 0.00 % (the half domain's fin cell straddles the plane the sector's ends at) | 1 % (design): met at M 2, not at M 0.8 |
| G-W3 | N = 3 InPlane (grids not replicas; odd ghost), M 0.8 / 2.0 | -1.30 / +0.59 % | 1 % (design): met at M 2 |
| G-W3 | half domain CN at alpha = 0 (N = 4, N = 2; M 0.8, 2.0) | <= 1.3e-8 | 1e-4 |
| G-W3 | free stream on wedge, base half, N = 4 and N = 3 sectors, N = 4 InPlane half | max abs dU <= 7.6e-15 | 1e-13 |
| G-W3 | half domain on the base grid, 24 cells, vs wedge, M 1.3 / 2.0 | -0.40 / -0.40 % | 1.5 % |
| G-W4 (sample) | 10 deg cone Cp vs Taylor-Maccoll, M 2 / 3 | -0.75 / -0.47 % | 3 % (2D table: see `bowshock-cases`) |

Not run on the wedge in S1: the remaining `bowshock-cases` tables (ogive shock expansion, transonic, subsonic) and the other cone angles: `gpu3d`'s `wedge_gates.rs` runs G-W4 against the frozen 2D fixture once its CPU runner is filled in.
