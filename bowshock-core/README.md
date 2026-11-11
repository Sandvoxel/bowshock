# bowshock-core — CPU reference solver (2D axisymmetric Euler)

Pure-Rust (f64, `rayon` for data-parallel loops) reference solver that the Luma/wgpu GPU kernels
will be validated against. Kernels are plain per-face / per-cell functions of their inputs with no
hidden state, so they port 1:1 to WGSL.

```
bowshock/
  Cargo.toml                 workspace
  bowshock-core/src/gas.rs         ideal gas, prim<->cons, physical flux, reflection
  bowshock-core/src/flux.rs        limiters (minmod, van Albada), MUSCL reconstruct, HLLC, Rusanov, shock sensor/blend   <- face kernels
  bowshock-core/src/grid.rs        structured grid metrics + algebraic cone grid
  bowshock-core/src/solver.rs      BCs/ghost cells, face flux pass, cell residual+dt pass, SSP-RK3
  bowshock-core/src/bc.rs          characteristic (Riemann-invariant) far-field + static-pressure outflow ghost states   <- BC kernels
  bowshock-core/src/implicit.rs    line-implicit (block-tridiagonal, approximate-factorization) pseudo-time scheme, Jacobians, ghost-state linearisation   <- convergence kernels
  bowshock-core/src/multigrid.rs   FAS multigrid (measured, not selected: see "Subsonic")
  bowshock-reference/src/taylor_maccoll.rs   oblique shock + Taylor-Maccoll (RK4) reference
  bowshock-reference/src/shock_expansion.rs  tangent-ogive geometry, tangent-cone and shock-expansion theory
  bowshock-reference/src/potential.rs   incompressible axisymmetric potential flow (axial source method) + Goethert rule: independent subsonic reference
  bowshock-cases/src/cone.rs        cone case set-up and post-processing
  bowshock-cases/src/ogive.rs       ogive-cylinder grid, case runner, wall post-processing, Perkins-Jorgensen data
  bowshock-cases/src/body_grid.rs   large-domain stretched grid for nose-cylinder bodies (ogive, cone, power/parabolic/Haack noses)
  bowshock-cases/src/transonic.rs   `transonic_case`, Stoney (RM L53K17) and Czarnecki-Monta (TN D-3516) data
  bowshock-cases/src/subsonic.rs    subsonic case runner (sting / flat base + wake), drag breakdown, NASA TM X-2439 data
  bowshock-cases/examples/diag.rs   convergence diagnostics (per-cell residual map -> out/diag/)
  bowshock-cli/src/bin/cone.rs      validation driver
  bowshock-cli/src/bin/ogive.rs     ogive validation driver -> out/ogive/
  bowshock-cli/src/bin/transonic.rs   transonic Mach sweep driver -> out/transonic/
  bowshock-cli/src/bin/subsonic.rs    subsonic Mach sweep, TM X-2439 validation, low-Mach study -> out/subsonic/
  out/                       CSV output of `cone`
```

## Equations

Axisymmetric Euler in the meridian plane (x axial, r radial), `U = (rho, rho u, rho v, E)`,
`E = p/(g-1) + rho (u²+v²)/2`, `g = 1.4`. Written with the r-weighted conservation form

```
d(r U)/dt + d(r F)/dx + d(r G)/dr = S,     S = (0, 0, p, 0)
```

(`S` is the only non-conservative term: the radial-momentum pressure source). Integrating over a
planar cell of area `A` in the (x, r) plane gives the exact discrete form used in the code:

```
V dU/dt = - Σ_faces (len · r_mid) (F·n)  +  (0, 0, p A, 0),      V = ∫ r dA
```

* `len`, `n` = planar length and unit normal of the face in the (x, r) plane, `r_mid` = mean radius
  of the two end nodes (exact for a straight edge), so every face has the weight `w = len·r_mid`.
* `V` is the exact integral of `r` over the quadrilateral (two triangles). `A` is its planar area.
* `Σ r_mid n_r len = A` holds exactly for a closed polygon (Green), so the source `pA` is the exact
  counterpart of the pressure flux: uniform pressure/axial flow is preserved to round-off on any grid.
* Faces on the axis have `r_mid = 0`, hence zero flux weight — the zero-area axis faces need no
  special treatment and there is no `1/r` singularity (cell volumes are `V > 0`).

Non-dimensionalisation: `rho_inf = 1`, `c_inf = 1` (⇒ `p_inf = 1/g`), `u_inf = M`.
`Cp = (p - p_inf) / (½ rho_inf u_inf²)`.

## Discretisation

* Cell-centred FV on a structured (i, j) body-fitted grid (i along the body, j from wall to far field).
* **MUSCL** on primitive variables `(rho, u, v, p)` with index-space limited slopes: `minmod`
  (default) or smooth `van Albada` (`--limiter vanalbada`). Positivity guard: a face whose
  reconstructed `rho` or `p` is ≤ 0 falls back to first order. The first 300 iterations are first order
  (start-up robustness, impulsive start from the free stream).
* **HLLC** flux (Toro) with Einfeldt/Roe-averaged wave speed estimates, **blended with Rusanov inside shocks**:
  `F = (1-θ) F_HLLC + θ F_Rusanov`, `θ = smoothstep((s - 0.002)/(0.02 - 0.002))`, `s = |p_R - p_L| / min(p_R, p_L)`
  of the two cell-centred pressures adjacent to the face (`flux::face_flux`; see "Convergence stall").
* **SSP-RK3** (Shu–Osher), **local time stepping** `dt = CFL·A / (½ Σ_f (|u·n|+c) len)`, CFL = 0.5 default.
* Residual monitor: L2 norm of `d rho/dt` (divided by √N). "Orders of drop" = peak (after iteration 20)
  over final.
* Per iteration: (1) cons→prim + ghost fill, (2) flux per i-face and per j-face, (3) per-cell residual
  and `dt`, (4) RK stage update. Each pass is an independent per-face or per-cell kernel.

## Boundary conditions (2 ghost layers, no corner ghosts needed)

| boundary | condition |
|---|---|
| inflow (imin) | supersonic inflow: ghost = fixed free stream |
| outflow (imax) | supersonic outflow: zero-gradient extrapolation |
| wall / axis (jmin) | slip wall: ghost layer k = interior cell k mirrored with the velocity reflected about the local face normal (`v_n → −v_n`). The same BC serves the symmetry axis `r = 0` for `x < 0` (normal = ±r) whose faces have zero weight. |
| far field (jmax) | fixed free-stream ghost (valid because the outer boundary lies outside the shock and the flow is supersonic) |

Selectable per boundary (`Config::bc`) in addition to the above (details in "Transonic" below):

| `Bc` | condition |
|---|---|
| `Characteristic` | Riemann-invariant far field, regime per face by the interior normal Mach number (supersonic inflow = free stream, subsonic in/out = 1-D characteristic ghost, supersonic outflow = extrapolation). Valid on inflow, outflow and outer boundaries. |
| `PressureOutflow` | fixed static pressure `p_inf` on subsonic outflow faces (`ρ, u, v` extrapolated), extrapolation if the face Mach number is ≥ 1. |

## Grid (`grid::cone_grid`)

Cone nose at origin, half-angle `θc`, domain `x ∈ [-0.1 L, L]`, `L = 1`. Vertical grid lines `x = x_i`
(a node sits exactly at `x = 0`); lower boundary `r = 0` for `x < 0` and `r = x tan θc` for `x ≥ 0`;
upper boundary the straight line from `(−0.1, 0.4 h)` to `(L, h)` with `h = 1.25 L tan β_TM` (outside the shock).
Optional tanh wall clustering (`cluster`, default 0 = uniform; not used in the results below).

## Taylor-Maccoll reference (`taylor_maccoll.rs`)

Oblique-shock relations give the state behind a shock of angle β; the Taylor-Maccoll ODE
(`Vr' = Vθ`, `Vθ' = (Vθ² Vr − k(2Vr + Vθ cotθ)) / (k − Vθ²)`, `k = (g−1)/2 (1 − Vr² − Vθ²)`, velocities in
units of `V_max`) is integrated with RK4 (step 1e-5 rad) from β down to `Vθ = 0` (cone surface); β is
bisected until that angle equals `θc`. Surface `p/p_inf` follows from the post-shock total pressure and
`T/T0 = 1 − Vr²`.
Reference values, verified against independent sources:

* M = 2, 15°: β = 33.9147°, p/p∞ = 1.56629, Cp = 0.20225, M_surf = 1.7069 (MatthewBonanni/mallard
  `docs/tutorials/cone.md`, from its `taylor_maccoll.py`) — reproduced (unit test).
* M = 2.35, 10°: M_surf = 2.1468, p/p∞ = 1.374 (NASA NPARC validation archive "10 Degree Cone at
  Mach 2.35", Euler CFD rows) — reproduced (unit test). **Note:** that NASA page's own "theory" row
  (β = 27.1843°, p3/p1 = 1.4234) contradicts both its CFD rows and the M = 2/15° reference above, so it
  is not used.
* M = 2, 10°: β = 31.206° (the ~31.2° quoted in tables), p/p∞ = 1.2925, Cp = 0.10447, M_surf = 1.834.

## How to run

```
cargo test --release                        # unit tests (incl. a ~5 s convergence regression, BC / grid / transonic smoke tests)
cargo run --release --bin cone              # M=2, 10° and 15°, grids 100x50, 200x100, 400x200 (≈2 min per limiter)
cargo run --release --bin cone -- --quick   # skip the 400x200 grid
cargo run --release --bin cone -- --limiter vanalbada [--mach 2.5]
cargo run --release -p bowshock-cases --example diag -- 10 200 100 minmod 8000   # residual map -> out/diag/
cargo run --release --bin ogive -- --sweep  # ogive-cylinder validation M=1.98 (+ Mach sweep), ≈7 min; --quick skips the fine grid
cargo run --release --bin transonic -- --body ogive --level 1   # transonic Mach sweep 0.7 – 1.5 -> out/transonic/ (see "Transonic")
cargo run --release --bin subsonic -- --quick            # M = 0.1 ... 0.8, base on/off, level 0 -> out/subsonic/ (see "Subsonic")
cargo run --release --bin subsonic -- --validate         # NASA TM X-2439 bodies at M = 0.404 / 0.608 / 0.831
cargo run --release --bin subsonic -- --lowmach-study    # low-Mach correction on/off vs potential flow, M = 0.1 ... 0.5
```

CSV files go to `out/`: `cp_<angle>deg_<ni>x<nj>_<limiter>.csv` (x/L, p/p∞, Cp from the wall-extrapolated
pressure, Cp of the first cell, Taylor-Maccoll Cp) and `residual_<...>.csv` (iteration, density residual).

Unit tests: HLLC and Rusanov consistency `F(U,U) = F(U)` for random states/normals, shock-blend weights, stationary
contact, limiter properties, free-stream preservation (50 RK3 steps on a strongly curved, stretched grid with an axis boundary,
drift < 1e-12; and `max|dU/dt| < 1e-12` on a curved grid with all-Dirichlet boundaries), oblique-shock
table value, the three Taylor-Maccoll references above, and a convergence regression (10° cone, 200×100, minmod must
reach 6 orders within 6000 iterations with Cp within 0.3 % of Taylor-Maccoll; it plateaued at 4 orders before the fix).

## Post-processing

* Cone Cp = mean over wall faces with `0.4 ≤ x/L ≤ 0.9` (away from the tip and the outflow) of the wall pressure
  extrapolated linearly from cell centres j = 0, 1 to the wall face (the first-cell value is also in the CSV).
* Shock angle: per column in the same window the steepest cell-to-cell pressure gradient (parabolic sub-cell
  refinement) gives a shock point; the angle is `atan` of the slope of a least-squares line `r = a + b x`
  through those points.

## Validation results (M = 2, γ = 1.4, CFL 0.5, SSP-RK3 + HLLC/Rusanov blend + MUSCL)

Taylor-Maccoll: 10° → β = 31.206°, Cp = 0.10447; 15° → β = 33.915°, Cp = 0.20225.

**minmod** (default)

| cone | grid | Cp CFD | Cp TM | Cp err % | β CFD [°] | β err [°] | residual drop | iters |
|---|---|---|---|---|---|---|---|---|
| 10° | 100×50 | 0.10411 | 0.10447 | −0.35 | 31.253 | +0.047 | 6.0 orders | 2785 |
| 10° | 200×100 | 0.10434 | 0.10447 | −0.13 | 31.263 | +0.057 | 6.0 orders | 3918 |
| 10° | 400×200 | 0.10441 | 0.10447 | −0.06 | 31.241 | +0.035 | 6.0 orders | 5880 |
| 15° | 100×50 | 0.20169 | 0.20225 | −0.27 | 33.994 | +0.079 | 6.0 orders | 1900 |
| 15° | 200×100 | 0.20195 | 0.20225 | −0.15 | 33.944 | +0.030 | 6.0 orders | 3824 |
| 15° | 400×200 | 0.20208 | 0.20225 | −0.08 | 33.925 | +0.011 | 6.0 orders | 5185 |

**van Albada**

| cone | grid | Cp CFD | Cp err % | β CFD [°] | β err [°] | residual drop | iters |
|---|---|---|---|---|---|---|---|
| 10° | 100×50 | 0.10433 | −0.14 | 31.228 | +0.022 | 6.0 orders | 1352 |
| 10° | 200×100 | 0.10443 | −0.04 | 31.238 | +0.032 | 6.0 orders | 2240 |
| 10° | 400×200 | 0.10446 | −0.01 | 31.227 | +0.021 | 6.0 orders | 4080 |
| 15° | 100×50 | 0.20209 | −0.08 | 33.972 | +0.057 | 6.0 orders | 1518 |
| 15° | 200×100 | 0.20216 | −0.04 | 33.942 | +0.027 | 6.0 orders | 2531 |
| 15° | 400×200 | 0.20220 | −0.02 | 33.925 | +0.011 | 6.0 orders | 4654 |

(The runs stop as soon as 6 orders are reached; see "Convergence stall" for how far they continue.)

The 2 % Cp target is met with large margin (≤ 0.4 % even on 100×50), and the error decreases with
refinement (≈ 2× per grid doubling, i.e. ~first-order-like for the wall extrapolation + shock smearing).
Shock angle is within 0.13° everywhere and converges as well. Cp is always slightly *below* TM
(the numerical solution has a small entropy layer/tip transient; on the finest grid the deficit is 0.02–0.08 %).

Caveats (honest notes):

* **Convergence.** All 12 runs reach 6 orders (see "Convergence stall"). The plateau stop (mean log-residual over the
  last 2000 iterations improves by < 0.05 decades vs the previous 2000, checked from iteration 8000) is kept as a safety
  net but no longer triggers. minmod converges more slowly than van Albada (non-differentiable limiter): its residual
  keeps falling after 6 orders but slowly (≈ 0.4–0.5 orders per 4000 iterations at 8000–12000 on 400×200).
* The solution has no stagnation/positivity problems (`min rho = 1`, `min p = p∞` in the output: free-stream values
  are the minima).
* The far-field is a fixed free stream; it is only valid while the shock stays inside the domain (guaranteed here by the
  1.25× margin).
* Index-space MUSCL slopes ignore grid stretching; the validation grids are uniform in each direction (no clustering).
* The tip cell region is intentionally excluded from the averages (x/L < 0.4).

## Convergence stall (before: minmod 10° 200×100 and 400×200 stopped at 4.1 orders, van Albada 15° 400×200 at 1.5 orders)

**Evidence** (`examples/diag.rs`: runs a case with no stopping rule, averages `dρ/dt` per cell over a trailing window,
writes `out/diag/resmap_*.csv`; measurements below are from the pre-fix code):

* The residual was **not steady** (mean part of Σrms² ≈ 0 %, 100 % oscillatory — a limit cycle) and lived in the **shock
  cells** (78 % of the Σrms² for minmod 10° 200×100, 89 % for van Albada 15° 400×200). Tip, axis, wall, far field and
  outflow were quiet (≤ 3 %).
* For van Albada 15° 400×200 the rms residual **grew exponentially with x** along the shock, from 1e-10 at x = 0.37 to
  2e2 at x = 0.64 (blocks of 20 columns), then saturated: a convective instability of the discrete shock structure that
  saturates as a limit cycle. Pressure probes across the shock showed waves travelling through the shock layer.
  Per-cell |dρ/dt| up to 4 (≈ 1e-3 relative change per step).
* The pure first-order scheme (same HLLC) converged to 1e-13 on the same grid; HLLC with the Rusanov flux in *all* faces
  converged to 1e-8 for van Albada 15° 400×200; HLL (which still resolves contact/shear) only halved the amplitude.
  So the culprit is the Riemann solver's lack of damping of contact/shear waves combined with second-order reconstruction in
  a captured oblique shock that is not aligned with the grid — not start-up (3000 first-order iterations: same),
  CFL (0.25: same plateau), or the limiter formula (van Albada converges where minmod does not and vice versa).
* minmod 10° 200×100 was additionally a *slow growth* of the residual (1e-4 at iteration 3000 → 2.3e-4 at 8000) that
  decayed instead of growing at CFL 0.25 (consistent with `|G|² − 1 ∝ CFL⁴`) and was absent with SSP-RK3 at the same CFL. SSP-RK2's amplification factor
  has `|G|² = 1 + y⁴/4 > 1` on the imaginary axis, so weakly damped, nearly central modes of the limited second-order
  operator grow. The shock-cell part of that residual remained with RK3 alone and disappeared with the Rusanov blend.

**Fix** (numerics change; the GPU solver must mirror both):

1. `flux::face_flux`: HLLC blended with Rusanov by a smooth (smoothstep) pressure-jump shock sensor, θ = 0 for relative
   jumps < 0.2 %, θ = 1 for ≥ 2 % (Quirk-type hybrid, "HLLC-HLL"/carbuncle-cure style). The shock cells get full
   dissipation of all wave families, smooth flow keeps pure HLLC, and the weight is C¹ so the steady state is a
   smooth fixed point (a hard switch would itself chatter).
2. SSP-RK3 instead of SSP-RK2 (3 residual evaluations per iteration, same CFL 0.5).

**Result**: 6 orders for all 12 cases (tables above); Cp error equal or smaller at the printed precision (minmod 10° coarse −0.40 → −0.35 %,
10° medium −0.15 → −0.13 %, 15° coarse −0.30 → −0.27 %; van Albada 10° −0.15/−0.04/−0.02 → −0.14/−0.04/−0.01 %) and β within 0.08° everywhere. Long runs (no stopping rule) keep converging: minmod 15° 400×200 residual
2e-3 / 1.5e-6 / 6.5e-7 at iterations 4000 / 8000 / 12000, 10° 400×200 5.6e-5 / 1.2e-6 / 3.6e-7, 10° 200×100 down to 3e-9 at
24000; van Albada 15° 400×200 reaches a 7e-8 floor, 95 % of it at the axis/tip singularity (x < 0.1). Cost per iteration is
~1.5× (3 stages), partly recovered by fewer iterations for the converging cases.

Before the fix (earlier README table), residual drop / iterations: minmod 10° 100×50 6.0 / 5423, 200×100 4.1 / 8001,
400×200 4.1 / 8001; 15° 6.0 / 2678, 6.0 / 5509, 6.0 / 5416; van Albada 10° 6.0 / 1293, 6.0 / 2211, 6.0 / 4048; 15° 6.0 / 1431,
6.0 / 2489, **1.5 / 8001**.

## Ogive-cylinder validation (`ogive` bin, `ogive.rs`, `shock_expansion.rs`)

Additive to the cone case: the cone grid, solver, flux and cone runner are untouched (the `cone --quick` CSVs were regenerated
after this work and are byte-identical to the previous ones).

### Sources

* **Experimental data (used):** E. W. Perkins and L. H. Jorgensen, *Comparison of Experimental and Theoretical Normal-Force
  Distributions (Including Reynolds Number Effects) on an Ogive-Cylinder Body at Mach Number 1.98*, NACA RM A54H23 (1954),
  NTRS 19930088493 (`https://ntrs.nasa.gov/citations/19930088493`). Model (Fig. 1, p. 25): fineness-ratio-3 circular-arc tangent
  ogive nose (arc radius 9.25 d, i.e. `(L² + R²)/2R` with `L = 3d`, `R = d/2`), cylinder afterbody to 10 d, `d = 1.125 in`,
  23 pressure orifices in one meridian. Zero-angle-of-attack pressures: **Table I(a)** (p. 20, `α = 0°`, `Re = 0.39e6 / in`,
  `M = 1.98`; 15 stations x/d = 0.22 … 8.06, of which 8 lie on the nose, x/d ≤ 2.95). Stated Cp uncertainty ±0.005 (p. 6);
  the pressure variation around the body at α = 0 was < ±0.002 so one value per station is tabulated; Reynolds-number
  effects at α = 0 are called negligible by the authors; transition at 7.5 body diameters from the vertex (schlieren), i.e. the
  nose and most of the afterbody are laminar. The scanned table is not OCR-able; **the 30 numbers were transcribed by eye from a 300 dpi
  render of the page** (3 decimals as printed), so a misread digit is possible (they agree with the experimental circles in Fig. 3 of the
  report at a glance; that figure was not digitised).
* **Inspected, not used quantitatively:** NACA Report 1386 (Perkins, Jorgensen, Sommer, *Investigation of the drag of various
  axially symmetric nose shapes of fineness ratio 3 for Mach numbers from 1.24 to 7.4*, NTRS 19930091022) has measured fore-drag
  vs Mach (Fig. 16), but (a) the circular-arc ogive tested is "L-V ogive" with L/D = 2.93, not the tangent ogive L/D = 3, (b) the measured
  fore-drag includes skin friction (the report compares against wave drag + friction) and, in Fig. 12(c), is referenced to volume^(2/3) rather than base area,
  (c) only plots are available. A wave-drag-vs-Mach comparison against data would therefore need friction subtraction and digitisation
  with errors larger than the effects of interest, so none is made. **No experimental Mach variation for the exact geometry was retrieved;
  the Mach sweep below is CFD vs theory only.**
* **Theory (independent, own implementation, `shock_expansion.rs`):**
  * *tangent-cone*: Cp of the Taylor-Maccoll cone whose half angle equals the local surface slope (for slopes < 1° the 1° value is scaled linearly with the slope). Defined on the nose only.
  * *shock-expansion* (basic, planar Prandtl-Meyer variant): Taylor-Maccoll tip cone (half angle `asin(L/ρ) = 18.925°`, β = 37.17°, Cp = 0.2983 at M = 1.98), then isentropic
    Prandtl-Meyer expansion through `θ_tip − θ_local` along the surface. It ignores shock curvature, vorticity and the relief of the axisymmetric flow, which is known
    to make it over-expand on bodies of revolution with large total turning (confirmed here: it is far too low on the aft nose and on the cylinder, Cp → −0.131 at M = 1.98).
    A second-order version (Syvertson-Dennis / Van Dyke second-order theory, which the report's Fig. 3 shows agreeing with the data) was **not** implemented.
    The two methods bracket the CFD: tangent-cone is an upper, basic shock-expansion a lower bound, at every Mach number run.

### Geometry and grid (`ogive::ogive_grid`)

Generalisation of `cone_grid`: `d = 1`, tip at the origin, nose profile `r(x) = sqrt(ρ² − (L − x)²) − (ρ − R)`, `ρ = 9.25`, cylinder `r = 0.5` up to `x = 9` (the
data end at x/d = 8.06; the model was 10 d long). Vertical lines `x = x_i` (nodes exactly at the tip and the nose-cylinder junction, 0.5 d upstream of the tip), lower boundary axis / body, wall-clustered radial
distribution (`tanh`, strength 2), straight outer boundary from `(−0.5, 2)` to `(9, h_end)` with `h_end = 1.1 (L tan β_tip + 6 tan(μ + 3°))` (tip-cone shock angle over the nose, Mach angle + 3° over the cylinder).
Reported "shock margin" = smallest `(h − r_shock)/h` over x > 0.5 (shock = outermost cell > 2 % off free-stream pressure): 0.10 – 0.14 for all runs, i.e. the shock stays inside the free-stream far-field.
Optional tip clustering `--stretch s` (`x = L (k/n)^s` on the nose). Grids: coarse / medium / fine = 114×50 / 228×100 / 456×200 cells
(12 / 24 / 48 cells per diameter along the body, 36/72/144 cells on the 3 d nose). Same solver and settings as the cone (HLLC/Rusanov blend, minmod, SSP-RK3, CFL 0.5,
BCs: free-stream inflow and far field, extrapolated outflow, slip wall/axis), stopping rule 6 orders of density residual (all runs reached it: 3505 – 9237 iterations, 3 – 52 s).

### Post-processing

Wall Cp = pressure extrapolated linearly from cells j = 0, 1 to the wall face (first-cell Cp also tabulated), evaluated at the experimental stations by linear interpolation in x.
Forebody pressure drag `CD = (1/πR²) ∫ Cp dA_x = ∫₀¹ Cp d(r²/R²)` over the nose 0 ≤ x ≤ 3 d (base-area reference; positive = drag). CFD: sum over nose wall faces; theory: 4000 (shock-expansion) /
400 (tangent-cone) midpoint panels. Experiment: trapezoid in `s = (r/R)²` through the 8 nose orifices plus two **assumed** end values: tip Cp (x = 0, unmeasured; the first orifice is at x/d = 0.22, `s = 0.021`) and
tail Cp (x = 3). Nominal: tip = linear extrapolation of the first two orifices (0.291; the Taylor-Maccoll tip-cone value is 0.298), tail = the x/d = 2.95 value (−0.038). Bounds: tip Cp ∈ [0.260, 0.298], tail Cp ∈ [−0.038, 0],
and ±0.005 (the stated Cp uncertainty, which maps one-to-one into CD because `∫ ds = 1`).

### Results, M = 1.98 (CSV: `out/ogive/cp_M1.98_<grid>.csv`, `cp_compare_M1.98.csv`, `exp_cp_M1.98.csv`, `drag_summary.csv`; tables: `out/ogive/tables.md`)

| x/d | Cp exp | shock-exp. | tangent-cone | CFD coarse 114x50 | CFD medium 228x100 | CFD fine 456x200 |
|---|---|---|---|---|---|---|
| 0.22 | 0.260 | 0.252 | 0.262 | 0.242 | 0.249 | 0.253 |
| 0.56 | 0.212 | 0.187 | 0.210 | 0.198 | 0.200 | 0.202 |
| 0.89 | 0.165 | 0.130 | 0.165 | 0.153 | 0.154 | 0.154 |
| 1.22 | 0.115 | 0.078 | 0.124 | 0.110 | 0.110 | 0.111 |
| 1.61 | 0.074 | 0.023 | 0.083 | 0.063 | 0.064 | 0.064 |
| 2.06 | 0.026 | -0.034 | 0.044 | 0.016 | 0.017 | 0.017 |
| 2.50 | -0.014 | -0.083 | 0.016 | -0.023 | -0.023 | -0.023 |
| 2.95 | -0.038 | -0.126 | 0.001 | -0.057 | -0.057 | -0.057 |
| 3.62 | -0.030 | -0.131 | - | -0.041 | -0.041 | -0.041 |
| 4.06 | -0.017 | -0.131 | - | -0.032 | -0.032 | -0.032 |
| 4.50 | -0.012 | -0.131 | - | -0.025 | -0.025 | -0.025 |
| 5.39 | -0.009 | -0.131 | - | -0.016 | -0.016 | -0.016 |
| 6.28 | -0.001 | -0.131 | - | -0.011 | -0.011 | -0.011 |
| 7.17 | 0.000 | -0.131 | - | -0.008 | -0.008 | -0.008 |
| 8.06 | 0.006 | -0.131 | - | -0.006 | -0.006 | -0.006 |

Cp error vs experiment (model − experiment), nose = 8 stations x/d ≤ 2.95, all = 15 stations:

| model | nose RMS | nose max | all RMS | all max |
|---|---|---|---|---|
| shock-expansion | 0.0525 | 0.0881 | 0.0918 | 0.1366 |
| tangent-cone (nose only) | 0.0189 | 0.0387 | - | - |
| CFD coarse 114x50 | 0.0130 | 0.0193 | 0.0122 | 0.0193 |
| CFD medium 228x100 | 0.0114 | 0.0194 | 0.0114 | 0.0194 |
| CFD fine 456x200 | 0.0106 | 0.0194 | 0.0109 | 0.0194 |

Forebody pressure drag (base-area reference):

| source | CD_forebody | vs experiment nominal |
|---|---|---|
| experiment (Table I(a), tip extrapolated) | 0.1067 (bounds 0.1014 … 0.1118) | - |
| shock-expansion | 0.0625 | −41.4 % |
| tangent-cone | 0.1158 | +8.6 % |
| CFD coarse 114x50 | 0.0956 (first-cell Cp: 0.0965) | −10.4 % |
| CFD medium 228x100 | 0.0969 (first-cell Cp: 0.0974) | −9.2 % |
| CFD fine 456x200 | 0.0977 (first-cell Cp: 0.0980) | −8.4 % |

With `--stretch 1.6` (`out/ogive_stretch1.6/`, coarse/medium only) CD = 0.0967 / 0.0977 (vs 0.0956 / 0.0969 uniform): the tip-clustered medium grid equals the uniform fine grid; not needed for the results above.

### Results, Mach sweep (CFD vs theory only, no data; CSV `cp_M<M>_<grid>.csv`)

| M | tip cone β [deg] | shock-expansion | tangent-cone | CFD medium | CFD fine | fine vs tangent-cone |
|---|---|---|---|---|---|---|
| 1.5 | 48.04 | 0.0124 | 0.1366 | 0.1028 | 0.1039 | −24.0 % |
| 3 | 28.63 | 0.0794 | 0.0977 | 0.0885 | 0.0891 | −8.8 % |
| 4 | 25.44 | 0.0799 | 0.0898 | 0.0831 | 0.0837 | −6.8 % |

The two theories bracket the CFD at every Mach number; the shock-expansion estimate converges to the CFD as M grows (−4.5 % at M = 4) and is meaningless at M = 1.5 (tip cone close to detachment, large over-expansion).

### Findings and uncertainties

* **Numerics are converged; the remaining difference to the data is not a grid effect.** Coarse → fine changes CD by 2.2 % and the Cp curve by ≤ 0.004 except at the first station (x/d = 0.22: 0.242 → 0.253, still rising
  with refinement, so the tip-region Cp is not fully resolved at the fine grid); the extrapolated-wall and first-cell Cp differ by 0.0002 – 0.0003 in CD. Residuals dropped 6 orders on every grid.
* **Inviscid CFD vs experiment:** nose-station Cp RMS 0.011, max 0.019 (several times the ±0.005 stated uncertainty); forebody CD 8.4 % below the nominal experimental value and 3.6 % below its lower bound
  (0.1014). The CFD is below the data on the whole nose (0.01 – 0.02) and over-expands at the shoulder (x/d = 2.95: −0.057 vs −0.038), and stays ≈ 0.007 – 0.015 below the data on the cylinder up to 8 d.
  Tangent-cone is above the data (+8.6 % CD, above the upper bound), so the data lie between the two inviscid estimates.
  **[INFERENCE, not tested here]** the sign and distribution (positive offset growing with run length, smoothed shoulder expansion) are what a growing laminar boundary-layer displacement effect gives; the solver is inviscid, so
  the Euler result should not be expected to reproduce the data to within the ±0.005 Cp stated uncertainty. A viscous (boundary-layer-corrected or Navier-Stokes) check would be needed to confirm this.
* Experimental CD uncertainty: the assumed tip/tail Cp end values change CD by < 0.001 (they cover only `s < 0.021` and `s > 0.99`), so the bounds [0.1014, 0.1118] (±5 %) are dominated by the ±0.005 Cp uncertainty. The table values were transcribed by eye (see above).
* The GPU port should be validated against the CFD, not the experiment: the CPU CFD (f64) is the reference for kernels; the experiment/theory comparison characterises the physics (Euler vs viscous flow), not the port.

## Transonic (M ≈ 0.7 – 1.5): characteristic far field, large-domain grid, validation (`transonic` bin, `bc.rs`, `body_grid.rs`, `transonic.rs`)

Additive: `cone_grid`, `ogive_grid`, the cone/ogive runners, flux and the existing BCs are untouched. The `cone --quick` / `ogive --quick`
CSVs of the Cp and wall data are byte-identical to the previous ones (the residual-history files differ only at the 3e-16 relative level, from the
order of the parallel `Σ r²` reduction; iteration counts are unchanged), and `cargo test --release -p bowshock-core` passes (28 tests).

### Far-field boundary conditions (`bc.rs`, `Bc::Characteristic`, `Bc::PressureOutflow`)

Pure functions of the first interior cell, the free stream and the **outward** unit face normal; both ghost layers get the same boundary state (the GPU port
mirrors them kernel-for-kernel). With `u_n = v·n`, `c` the sound speed:

```
R_out = u_n,i + 2 c_i/(g-1)   (u_n + c wave, from the interior)       R_in = u_n,e - 2 c_e/(g-1)   (u_n - c wave, from the free stream)
u_n,b = (R_out + R_in)/2,   c_b = (g-1)/4 (R_out - R_in)
entropy s = p/rho^g and tangential velocity: interior if u_n,b >= 0 (outflow), free stream otherwise (inflow)
rho_b = (c_b²/(g s))^(1/(g-1)),   p_b = rho_b c_b²/g,   v_b = v_ref + (u_n,b - u_n,ref) n
```

The regime is chosen **per face from the interior normal Mach number** `M_n = v_i·n/c_i` (`bc::regime`): `M_n ≤ −1` supersonic inflow → ghost = free stream;
`−1 < M_n < 0` subsonic inflow and `0 ≤ M_n < 1` subsonic outflow → the Riemann ghost above; `M_n ≥ 1` supersonic outflow → zero-gradient. A uniform stream is a fixed point for any
normal (to 1e-13) in every regime, so the BC emits no spurious waves; `Bc::PressureOutflow` is the alternative outflow: subsonic faces take `ρ, u, v` from the interior and `p = p_inf`
(supersonic faces extrapolate). The old `Freestream` / `Extrapolate` BCs stay available and are selected per boundary in `Config::bc`. Tests: regime selection by `M_n` (incl. the
sign flip with the outward normal), ghost = free stream/interior for supersonic faces, Riemann invariants and entropy/tangential-velocity sources reproduced, free-stream fixed point
for 16 normals × 7 Mach numbers, uniform flow preserved to < 1e-12 for 100 RK3 steps on a strongly curved stretched grid with the characteristic BCs on three sides at M = 0.5, 0.9, 0.98, 1.2
(both outflow variants), and a pressure pulse leaving a subsonic domain (> 90 % of the perturbation gone through the BCs).

### Large-domain grid (`body_grid::large_domain_grid`)

H-grid like the existing ones (vertical lines, axis for `x < 0`, flat outer boundary `r = r_far`), `d = 1`, tip at the origin. Bodies: `Nose::{TangentOgive, Cone, Power, Parabolic, Haack}` on a cylinder.
Default domain (`transonic_case`, body length `L = L_nose + 6 d = 9 d`): inflow plane and outer boundary **10 L = 90 d** away, outflow plane 5 L = 45 d behind the shoulder
(`--domain n` rescales the 10 L). x spacing: geometric growth upstream of the tip, `tanh` clustering on the nose at the tip **and** the shoulder with end cell `dx_tip`
(0.03/0.015/0.0075 d for grid level 0/1/2), geometric growth along the cylinder and on the extension. r spacing: geometric from the wall (`dr_wall` = 0.03/0.015/0.0075 d).
Levels 0/1/2 = 144×70 / 288×140 / 576×280 cells.
**Afterbody choice: a sting.** The cylinder simply continues to the outflow plane (6 d inside the geometric cylinder segment, then the stretched extension). There is no base and no wake, so base pressure
(ill-defined in Euler) cannot contaminate the forebody, and the zero-lift pressure drag of the body is the nose drag only (the cylinder has no axial projected area); the outflow plane is far away and coarse, and only
exercised by the characteristic BC. The forebody CD is reported referenced to the base area, `CD = ∫₀¹ Cp d(r²/R²)` over the nose (extrapolated-wall Cp; the first-cell variant is also in the CSV).
Limitation: MUSCL slopes are limited in index space (they ignore the stretching); growth ratios are 1.08 / 1.04 / 1.02 radially and 1.25 / 1.12 / 1.06 upstream of the tip for levels 0/1/2.

### Case runner and convergence (`bowshock::run::run_on`, `stop_reason`)

Same scheme as before (HLLC/Rusanov blend, minmod, SSP-RK3, local dt, CFL 0.5, 300 first-order iterations on a cold start). Stop rule: density-residual drop of 6 orders (from the peak after iteration 20), **or**
(every 500 iterations, from iteration 4000) forebody CD varied by < 2e-4 over the last 2000 iterations once the residual has dropped ≥ 2 orders ("CD stationary"). `--warm` starts every Mach number from the
previous converged field (ascending order; the drop criterion is then relative to the cold-start peak). No implicit residual smoothing was added (the iteration counts below did not require it).

### Data sources and their limits

* **Czarnecki & Monta, NASA TN D-3516 (1966), NTRS 19660024157, Fig. 5 (p. 27)** — Cp(x) on a 3-caliber tangent-ogive cylinder (nose 3.00 d, ogive radius 9.25 d, d = 4.096 in, total 12.2 d, sting-mounted), α = 0, M = 0.7, 0.9, 1.0, 1.1, 1.2,
  Langley 8-ft transonic pressure tunnel (slotted), fully turbulent (No. 60 carborundum 0.75 in from the tip), no Reynolds-number effect reported, no wall corrections. **Digitised from the plot by eye** (34 stations × 5 Mach numbers, embedded as
  `transonic::czarnecki_monta`): ±0.01 Cp for x ≥ 6 in, ±0.02 on the nose (x < 5 in), ±0.015 within 1 in of a shock. The authors state that the slotted-wall reflection of the nose shock lands on the body at x ≈ 30 in (M = 1.1) and 48 in (M = 1.2), and the last points
  (x ≥ 44 in) are near the base/sting (not described in what was read), so the statistics below use stations x ≤ 44.1 in (x ≤ 28.8 in at M = 1.1). The model is the `ogive` body.
* **Stoney, NACA RM L53K17 (1954), NTRS 19930087953, Fig. 5 (p. 14)** — nose pressure drag CD (maximum-frontal-area = base-area reference) of eight fineness-ratio-3 noses vs Mach number 0.8 – 1.25, free flight (helium gun, no tunnel interference), Re = 8–15 × 10⁶, turbulent.
  **Digitised from a poor scan** (curves, not tabulated): ±0.01 in CD and in M (more where curves overlap); embedded as `transonic::stoney` for the cone (9.46°), `x^1/2`, parabola K = 1, Von Kármán and LV-Haack. The "nose pressure drag" is derived by the author:
  measured total CD − tare (afterbody + fins + base: 0.025 subsonic, ≈ 0.065 supersonic, Fig. 4, anchored at M = 1.2 on the Taylor-Maccoll cone drag) − calculated turbulent friction, so it carries an extra ≈ ±0.01 tare uncertainty (stated accuracy ±0.008 in total CD, ±0.005 in M).
  Above M ≈ 1.225 the curves are fairings to the Ames data (NACA Rep. 1386) and second-order theory, not measurements (those rows are flagged and not used).
* **Inspected, not used:** NACA Rep. 1386 / RM A52H28 (supersonic only, M ≥ 1.24); Swihart & Whitcomb RM L53H04 (tabulated Cp on a fineness-12 rounded fuselage body without cylinder, wall-interfered at M ≥ 1.02; partly transcribed during the search, not used); NACA TN 4234
  (parabolic-arc bodies, plots only); NACA Rep. 1242 (Solomon, 20–30° cone-cylinders, too blunt); NASA TN D-6580 / D-7331 (supercritical blunt bodies).
* **Transonic-similarity rule:** no verified tabulated similarity-rule drag prediction for a cone/ogive-cylinder was found, and none is given (an unverified formula would be fabricated data). The theory references used are Taylor-Maccoll (exact for the cone at supersonic speed)
  and tangent-cone, valid only where the flow is attached and supersonic behind the shock (they agree with the CFD to 5 % from M = 1.1 up for the 9.46° cone, see below).

### Results

All runs: cold start, grid levels L0/L1/L2 = 144×70 / 288×140 / 576×280, minmod. CSV/tables in `out/transonic/` (`sweep_<body>_L<n>.csv`, `compare_<body>_L<n>.csv`, `cp_<body>_L<n>_M<m>.csv`, `residual_<body>_L<n>_M<m>.csv`
(density residual + CD every 50 iterations), `cp_compare_ogive_L<n>_M<m>.csv`, `tables_<body>_L<n>.md`).

**1. Surface Cp, tangent ogive cylinder vs TN D-3516** (CFD wall Cp extrapolated from cells 0/1, linearly interpolated at the orifices; RMS / max of CFD − data in Cp, "nose" = 8 orifices x/d ≤ 3, "valid" = 31 stations (25 at M = 1.1) as defined above)

| M | grid | nose RMS | nose max | valid RMS | valid max | shoulder suction peak: data | CFD |
|---|---|---|---|---|---|---|---|
| 0.7 | L1 | 0.0144 | 0.0334 | 0.0092 | 0.0334 | −0.116 | −0.130 |
| 0.9 | L1 | 0.0187 | 0.0556 | 0.0129 | 0.0556 | −0.167 | −0.180 |
| 0.9 | L2 | 0.0181 | 0.0540 | 0.0124 | 0.0540 | −0.167 | −0.179 |
| 1.0 | L1 | 0.0071 | 0.0138 | 0.0172 | 0.0415 | −0.241 | −0.248 |
| 1.0 | L2 | 0.0079 | 0.0158 | 0.0180 | 0.0379 | −0.241 | −0.248 |
| 1.1 | L1 | 0.0201 | 0.0416 | 0.0165 | 0.0416 | −0.172 | −0.172 |
| 1.1 | L2 | 0.0213 | 0.0439 | 0.0168 | 0.0439 | −0.172 | −0.173 |
| 1.2 | L1 | 0.0076 | 0.0137 | 0.0083 | 0.0162 | −0.131 | −0.140 |
| 1.2 | L2 | 0.0083 | 0.0167 | 0.0086 | 0.0167 | −0.131 | −0.140 |

(L0 = 144×70 gives the same numbers to ±0.005, e.g. M = 0.9: 0.0185 / 0.0530 / 0.0126 / 0.0530; M = 1.0: 0.0066 / 0.0104 / 0.0171 / 0.0469.) The grid change L1 → L2 moves the statistics by ≤ 0.0033, the data uncertainty is 0.01 – 0.02: **the Cp field is grid-converged to
within the data uncertainty and agrees with the data to RMS 0.007 – 0.021** (the stated tunnel Mach non-uniformity, ±0.005 – 0.02 in M, is a comparable source of Cp scatter). Where the CFD misses the data by more than the digitisation uncertainty:
(a) the shoulder at M = 0.7 / 0.9: the data peak earlier (x ≈ 9.8 in) and recover faster aft of the peak (CFD peak at x ≈ 10.5 in; largest errors at x = 11.8 in: −0.033 / −0.056) and the CFD suction peak is 0.013 – 0.014 deeper — **[INFERENCE]** boundary-layer displacement
(the data are fully turbulent) and the sting, not tested; (b) M = 1.0 downstream of the shoulder: the data have a recompression shock at x ≈ 24 in (Cp −0.042 → +0.026), the free-air inviscid CFD recompresses smoothly with no shock: it is 0.013 – 0.037 above the data at x = 16.8 – 22.8 in (before the shock) and 0.028 – 0.042 below it at x = 24.7 – 28.8 in (after it);
(c) the nose tip at M = 1.1: CFD +0.04 above the data at x = 1.0 and 1.95 in (digitisation ±0.02; the grit trip sits at 0.75 in); (d) M = 1.1, x > 30 in: the data contain the tunnel-reflected shock (Cp +0.06), absent in free air (excluded from "valid"). Peak suction agrees to 0.007 at M = 1.0, 0.000 at 1.1, 0.009 at 1.2.

**2. Nose pressure drag vs Mach number.** Smooth noses (no shoulder corner) are the clean test: the Von Kármán nose, CFD L1 vs Stoney Fig. 5 (digitised, ±0.01 plus ≈ ±0.01 tare uncertainty):

| M | CD CFD L1 | Stoney | CFD − Stoney |
|---|---|---|---|
| 0.7 – 0.9 | −0.0016 … −0.0010 | ≈ 0 (subsonic; assumed tare) | ≈ −0.001 |
| 0.95 | 0.0079 | 0.000 | +0.008 |
| 0.98 | 0.0285 | 0.018 (interpolated) | +0.011 |
| 1.00 | 0.0445 | 0.030 | +0.015 |
| 1.02 | 0.0616 | 0.046 (interpolated) | +0.016 |
| 1.05 | 0.0791 | 0.070 | +0.009 |
| 1.10 | 0.0857 | 0.075 | +0.011 |
| 1.20 | 0.0871 | 0.080 | +0.007 |
| 1.50 | 0.0841 | (faired, not used) | |

The shape of the drag rise (onset at M ≈ 0.95, steep rise to M ≈ 1.05, plateau 0.08 – 0.09 above M = 1.1, nearly flat towards 1.5) is reproduced; the CFD lies **systematically 0.007 – 0.016 above** the digitised data, i.e. at the 1 – 1.5 σ level of the combined read-off + tare uncertainty
(about ±0.014), so the agreement is to the uncertainty of the data but not better than that. The cone-cylinder (9.46° half angle, sharp shoulder corner), CD vs Stoney's cone curve:

| M | L0 | L1 | L2 | Stoney cone | L1 − Stoney | Taylor-Maccoll cone |
|---|---|---|---|---|---|---|
| 0.8 | 0.0137 | 0.0129 | | 0.020 | −0.007 | |
| 0.85 | 0.0217 | 0.0263 | | 0.025 | +0.001 | |
| 0.9 | 0.0398 | 0.0471 | 0.0529 | 0.030 | +0.017 | |
| 0.95 | 0.0701 | 0.0774 | | 0.060 | +0.017 | |
| 0.98 | 0.0960 | 0.1042 | | 0.090 | +0.014 | |
| 1.0 | 0.1215 | 0.1301 | 0.1351 | 0.110 | +0.020 | |
| 1.02 | 0.1434 | 0.1504 | | 0.130 | +0.020 | |
| 1.05 | 0.1545 | 0.1626 | 0.1669 | 0.160 | +0.003 | 0.2059 |
| 1.1 | 0.1491 | 0.1550 | | 0.160 | −0.005 | 0.1620 |
| 1.2 | 0.1333 | 0.1360 | 0.1374 | 0.140 | −0.004 | 0.1387 |
| 1.5 | 0.1114 | 0.1123 | | (faired) | | 0.1132 |

**Caveat for the cone-cylinder below M ≈ 1.05:** the sharp 9.46° shoulder corner is a singularity of the inviscid problem (the suction peak grows with refinement: `Cp_min` = −0.695 / −0.754 / −0.791 for L0/L1/L2 at M = 0.9), so the subsonic and
low-transonic CD does **not** converge with the grid (M = 0.9: 0.0398 → 0.0471 → 0.0529; M = 1.0: 0.1215 → 0.1301 → 0.1351, successive increments shrink by ≈ 0.6 – 0.8 per refinement, i.e. slowly) and drifts away from the digitised data (+0.02 and rising); the agreement at L0 is partly fortuitous.
At supersonic Mach numbers the corner cannot influence the nose and the CFD converges to Taylor-Maccoll (M = 1.2: −3.9 % / −2.0 % / −1.0 % for L0/L1/L2; M = 1.5: −1.5 % / −0.8 %). At M = 1.05 – 1.1 Taylor-Maccoll over-predicts the CFD (by 27 % / 4.5 % at L1): there the flow behind the nose shock is not supersonic enough for the cone solution to hold.
Tangent-cone on the ogive nose is an upper bound (M = 1.2: 0.1725 vs CFD 0.106; M = 1.5: 0.1366 vs 0.103), as already seen at M = 1.98. Smooth noses (ogive, Kármán) are grid-insensitive: tangent-ogive CD L0 / L1 / L2 at M = 1.0: 0.0434 / 0.0438 / 0.0447, M = 1.2: 0.1038 / 0.1061 / 0.1074.

The tangent parabola `K = 1` nose (`r/R = 2η − η²`, shoulder-tangent, the shape in Stoney's set closest to the tangent ogive; `--body parabola`) — CFD vs the digitised middle panel of Fig. 5 (the hardest panel to read; ±0.01, more where curves cross):

| M | CD L0 | CD L1 | Stoney parabola | L1 − Stoney |
|---|---|---|---|---|
| 0.7 – 0.9 | −0.0029 … −0.0026 | −0.0019 … −0.0010 | ≈ 0 (subsonic) | ≈ −0.001 |
| 0.95 | 0.0013 | 0.0026 | 0.010 | −0.007 |
| 0.98 | 0.0188 | 0.0242 | 0.025 (interpolated) | −0.001 |
| 1.00 | 0.0441 | 0.0446 | 0.045 | −0.000 |
| 1.02 | 0.0680 | 0.0655 | 0.063 (interpolated) | +0.003 |
| 1.05 | 0.0866 | 0.0875 | 0.090 | −0.003 |
| 1.10 | 0.0986 | 0.1008 | 0.105 | −0.004 |
| 1.20 | 0.1037 | 0.1060 | 0.120 (Van Dyke second-order marker 0.12) | −0.014 |
| 1.50 | 0.1014 | 0.1028 | (faired, not used) | |

Here the CFD sits within the ±0.01 read-off uncertainty (before the tare uncertainty) at every Mach number except M = 1.2 (−0.014) — the drag-rise Mach number and magnitude (0.045 at M = 1.0, 0.09 at 1.05) are reproduced. The Von Kármán nose above is
offset by +0.007 – 0.016 and the parabola by −0.014 … +0.003, so the CFD-minus-data differences of the smooth noses lie within ±0.016 with no consistent sign across shapes: an uncertainty-level agreement, not a validation to better than ≈ 0.015 in CD.

Tangent-ogive CD (base-area reference, nose only; no data for this exact shape) at L1: −0.0019 (0.7), −0.0017 (0.8), −0.0010 (0.9), 0.0022 (0.95), 0.0235 (0.98), 0.0438 (1.0), 0.0647 (1.02), 0.0868 (1.05), 0.1004 (1.1), 0.1061 (1.2), 0.1034 (1.5); the subsonic value is ≈ 0 as it must be
(d'Alembert: the nose drag of a closed body is cancelled by the afterbody; the residual −0.002 is the numerical drag of the nose-only integral).

**3. Convergence behaviour (honest numbers).** Subsonic and transonic runs do **not** reach 6 orders: the density residual falls ≈ 3 – 4 orders (L1: 2.6 – 4.2, L0: 3.1 – 5.3 orders for 0.7 ≤ M ≤ 1.2) and then sits on a floor of ≈ 5e-6 – 3e-4 (peak ≈ 0.1) set by the weakly
unsteady shoulder shock; only M = 1.5 (L0) reached 6 orders. Iterations to the stop rule: L0 3 600 – 19 000 (M = 1.2 / 1.5: 3.6 – 4 k; M = 0.95, 0.98, 1.02: 16.5 – 19 k), L1 4 000 – 30 000 (typical 17 – 22 k), L2 9 000 – 34 000 for the Mach numbers run
(cone M = 0.9: 29 k, 1.0: 11.5 k, 1.05: 33 k, 1.2: 9 k; ogive 9 – 34 k). Cost per iteration ≈ 1 ms (L0, 10 k cells) on 32 threads, ≈ 8 ms for L2 (161 k cells) on a loaded machine: a full L1 sweep takes 10 – 20 min, i.e. much slower than the supersonic cases but not the 10⁵+ iterations anticipated.
The CD-stationary stop is a **weak** convergence statement (CD drifts < 2e-4 per 2000 iterations; the last-window CD range is 0.9 – 1.9e-4). Checks against much longer runs (cone, L0, no early stop, 80 000 iterations, residual 4.8 – 6.8 orders): CD at the stop → CD after the long run:
M = 0.9: 0.03979 → 0.03973, 0.95: 0.07014 → 0.06983, 1.0: 0.12146 → 0.12138, 1.05: 0.15455 → 0.15469 (8 orders, 32 k iterations): ≤ 0.4 %. At L1, M = 0.95: 0.0774 → 0.0779 after 150 000 iterations (residual 2e-6, CD ±0.0005 oscillation).
**Warm start does not help and can mislead:** the L1 warm-start sweep needed 19 500 – 60 000 iterations per Mach number for M < 1 (59 500 at M = 0.8 vs 19 000 cold; 60 000 = the iteration cap reached at 0.85 and 0.9) because the instantaneous free-stream change launches a large transient through the 90 d far field, and between M = 0.85 and 0.98 it landed on a different or still-unsteady
state (CD range up to 5 × 10⁻³; M = 0.95: 0.0561 after 26 500 iterations / 2.3 orders vs 0.0774 cold, converged to 0.0779 in the 150 000-iteration cold run). At L0 the warm chain with long runs agrees with the cold values (0.8: 0.01367, 0.85: 0.02156, 0.9: 0.03987, 0.95: 0.0705 with a limit cycle of range 4 × 10⁻³, 1.0: 0.12085)
but needs 44 000 – 60 000 iterations. For M ≥ 1.0 the warm and cold L1 CD agree to ≤ 0.003 (M = 1.0: 0.1277 vs 0.1301, 1.05: 0.1635 vs 0.1626, 1.1: 0.1560 vs 0.1550, ≥ 1.2 identical) except at M = 1.02 (0.1574 vs 0.1504). Use cold starts for the transonic range (the default; `--warm` is available).
**Domain size:** (cone, L0, M = 0.9 / 1.0 / 1.05 / 1.2) with the inflow/outer boundary at 5 / 10 / 20 body lengths CD = 0.03966 / 0.03979 / 0.03987, 0.12141 / 0.12146 / 0.12155, 0.15461 / 0.15455 / 0.15451, 0.13325 / 0.13329 / 0.13332: ≤ 2 × 10⁻⁴ (the residual far-field disturbance in the boundary cells is |Δp/p∞| ≈ 1e-3 on the 10 L grid).

### Uncertainty summary

* Data: Cp ±0.01 – 0.02 (digitised), CD ±0.01 (digitised) ± ≈ 0.01 (tare), plus tunnel interference (TN D-3516, M ≥ 1.0) — the comparisons cannot discriminate CFD errors below those levels.
* Numerics: Cp grid-converged to ≤ 0.003 (ogive, L1 → L2), smooth-nose CD to ≤ 0.002; cone-cylinder subsonic/low-transonic CD not grid-converged (corner singularity, see above); steady-state error ≲ 0.4 % of CD (long-run check), warm-started sweeps not trusted at M = 0.85 – 0.98.
* Physics not modelled: viscosity (boundary-layer displacement at the shoulder, turbulent shock/boundary-layer interaction), the base and sting of the experiments; the wall-interference of the tunnel data.

### Reproduce

```
cargo run --release --bin transonic -- --body ogive --level 1            # Cp vs TN D-3516 (M = 0.7 … 1.5), tables_ogive_L1.md, ≈ 10 min
cargo run --release --bin transonic -- --body karman --level 1           # CD vs Stoney (Von Kármán), ≈ 20 min;  --body cone|parabola|sqrt|lvhaack likewise
cargo run --release --bin transonic -- --quick                            # cone, level 0, ≈ 2 min;  --level 2 --mach 0.9,1.0,1.05,1.2 for the fine grid;  --warm, --domain n, --outflow pressure
```

### Still open

Viscous effects (integral boundary layer or RANS, needed for the shoulder shock/boundary-layer interaction and the sting),
a far-field vortex/source correction (the domain study shows < 2e-4 sensitivity so it is not needed here), and a tabulated transonic-similarity reference.

## Subsonic (M ≈ 0.1 – 0.8): line-implicit scheme, low-Mach correction, base / wake, modelled base drag

Data: `subsonic_gpu --base both` (grid level 1, GPU guarded extended precision vs this f64 CPU solver) and `subsonic_gpu --validate`; CSV and tables in `out/gpu_subsonic/` (`sweep_*.csv`, `tables_validate.md`).

### Setup

* Body: tangent ogive nose l/d 3 + 6 d cylinder. Base off: the cylinder continues as a sting to the outflow. Base on: flat base plus wake.
* Grid level 1: 288 × 140 (40 320 cells) base off, 312 × 152 (47 424 cells) base on. Domain: inflow / outer boundary 10 body lengths, outflow 5 body lengths; characteristic far field.
* Scheme: line-implicit, van Albada limiter, low-Mach correction on.
* Stop rules: 6-order residual drop, or "base pressure stationary" for base-on cases; iteration limit `10 000 << level` (`SubsonicCase::with_body`), i.e. 20 000 at level 1.

### Results, ogive + cylinder, level 1, cold starts (GPU values)

| M | base | CD nose | CD base (Euler) | CD total (Euler) | Cp_b | CD base corr. | CD total corr. | GPU iters | stop | drop [orders] | max abs dCp GPU–CPU (wall) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.1 | off | -0.00189 | +0.00000 | -0.00189 | - | +0.00000 | -0.00189 | 9587 | residual drop reached | 6.0 | 3.9e-6 |
| 0.2 | off | -0.00176 | +0.00000 | -0.00176 | - | +0.00000 | -0.00176 | 20000 | max iterations | 4.1 | 4.4e-6 |
| 0.3 | off | -0.00165 | +0.00000 | -0.00165 | - | +0.00000 | -0.00165 | 20000 | max iterations | 4.7 | 1.1e-7 |
| 0.4 | off | -0.00156 | +0.00000 | -0.00156 | - | +0.00000 | -0.00156 | 3586 | residual drop reached | 6.0 | 1.8e-6 |
| 0.5 | off | -0.00148 | +0.00000 | -0.00148 | - | +0.00000 | -0.00148 | 3261 | residual drop reached | 6.0 | 3.6e-7 |
| 0.6 | off | -0.00142 | +0.00000 | -0.00142 | - | +0.00000 | -0.00142 | 1881 | residual drop reached | 6.0 | 2.0e-7 |
| 0.7 | off | -0.00137 | +0.00000 | -0.00137 | - | +0.00000 | -0.00137 | 1361 | residual drop reached | 6.0 | 4.6e-7 |
| 0.8 | off | -0.00132 | +0.00000 | -0.00132 | - | +0.00000 | -0.00132 | 876 | residual drop reached | 6.0 | 1.6e-6 |
| 0.1 | on | -0.00358 | +0.00899 | +0.00541 | -0.0092 | +0.12130 | +0.11772 | 4000 | base pressure stationary | 4.8 | 4.5e-7 |
| 0.2 | on | -0.00339 | +0.01368 | +0.01030 | -0.0137 | +0.12520 | +0.12181 | 4000 | base pressure stationary | 4.8 | 1.6e-7 |
| 0.3 | on | -0.00324 | +0.02438 | +0.02114 | -0.0244 | +0.13170 | +0.12846 | 4000 | base pressure stationary | 5.1 | 8.2e-8 |
| 0.4 | on | -0.00314 | +0.01981 | +0.01668 | -0.0310 | +0.14080 | +0.13766 | 20000 | max iterations | 1.8 | 4.0e-6 |
| 0.5 | on | -0.00306 | +0.05220 | +0.04915 | -0.0358 | +0.15250 | +0.14944 | 20000 | max iterations | 1.4 | 3.8e-6 |
| 0.6 | on | -0.00299 | +0.03320 | +0.03021 | -0.0389 | +0.16680 | +0.16381 | 20000 | max iterations | 1.4 | 2.8e-7 |
| 0.7 | on | -0.00292 | +0.04594 | +0.04302 | -0.0392 | +0.18370 | +0.18078 | 20000 | max iterations | 1.4 | 4.0e-8 |
| 0.8 | on | -0.00284 | +0.04531 | +0.04247 | -0.0450 | +0.20320 | +0.20036 | 20000 | max iterations | 1.7 | 1.2e-2 |

CD is referenced to the cross-section area π R²; Cp_b is the base pressure coefficient.

GPU vs CPU: max |ΔCp| (wall) ≤ 4.5e-6 on 15 of 16 rows. M 0.8 base-on reaches 1.2e-2 (CD base 0.04531 GPU vs 0.04895 CPU, CD total 0.04247 vs 0.04612). That case, like every base-on case with M ≥ 0.4, ends on `max iterations` (20 000) at 1.4 – 1.8 orders: the inviscid wake stays unsteady, and long unsteady transients of the f32-LHS GPU and the f64 CPU implicit iterations diverge (the gpu README's "Caveats" describe this). Base-off M 0.2 and 0.3 also end on `max iterations` (4.1 and 4.7 orders) but agree with the CPU to 4.4e-6 and 1.1e-7.

### Base drag: Euler wake vs modelled

* The inviscid (Euler) wake has no viscous mixing layer, so its base pressure is not physical: Cp_b stays slightly negative (-0.0092 at M 0.1 to -0.0450 at M 0.8), and CD base (Euler) is not monotone in M (0.02438 at M 0.3, 0.01981 at M 0.4, 0.05220 at M 0.5, 0.03320 at M 0.6) because the wake does not settle (see the convergence column above).
* Modelled base drag: `bowshock_reference::base_drag::base_drag_coefficient(M)` = 0.12 + 0.13 M² for M < 1, 0.25 / M for M ≥ 1, referenced to the base area. Source: OpenRocket technical documentation (S. Niskanen; based on his 2009 M.Sc. thesis), eq. 3.94, after Fleeman, *Tactical Missile Design*.
* It is scaled by (r_b / R)² to the reference area; 0 without base; NaN (printed `-`) for sting-mounted bases (the TM X-2439 configurations below), because it is a free-flight correlation. It is reported next to, not instead of, the Euler values (`SubsonicResult::cd_base_model`, `cd_total_model`). Here the full disc gives r_b = R, so `CD base corr.` = 0.12 + 0.13 M² (M 0.5: 0.15250).

### Validation: NASA TM X-2439 (Fox 1971), sting-mounted nose-cylinder-boattail bodies, level 1

Config 4 = ogive, config 2 = cone. RMS of the wall Cp against digitised data over the fore body + cylinder (x ≤ 3.4 ogive, x ≤ 4.2 cone).

| config | M | GPU iters | CPU iters | RMS fore+cyl GPU | RMS fore+cyl CPU | base Cp GPU | base Cp CPU | data base Cp | CD total GPU | CD total CPU | max abs dCp GPU–CPU |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 4 (ogive) | 0.404 | 4000 | 4000 | 0.0170 | 0.0170 | +0.0564 | +0.0564 | +0.0691 | +0.0028 | +0.0028 | 1.3e-6 |
| 4 (ogive) | 0.608 | 20000 | 20000 | 0.0227 | 0.0227 | +0.0643 | +0.0643 | +0.0836 | +0.0066 | +0.0066 | 3.1e-5 |
| 4 (ogive) | 0.831 | 20000 | 20000 | 0.0596 | 0.0596 | +0.0859 | +0.0859 | +0.1317 | +0.0339 | +0.0339 | 4.6e-6 |
| 2 (cone) | 0.404 | 4000 | 4000 | 0.0907 | 0.0907 | +0.0569 | +0.0569 | +0.0685 | +0.0083 | +0.0083 | 2.0e-4 |
| 2 (cone) | 0.608 | 20000 | 4000 | 0.0871 | 0.0871 | +0.0647 | +0.0647 | +0.0813 | +0.0127 | +0.0131 | 3.3e-2 |
| 2 (cone) | 0.831 | 20000 | 20000 | 0.0614 | 0.0614 | +0.0852 | +0.0852 | +0.1290 | +0.0601 | +0.0601 | 6.9e-6 |

All six configurations complete. GPU vs CPU max |ΔCp| ≤ 3.1e-5 except cone M 0.404 (2.0e-4) and cone M 0.608 (3.3e-2). At cone M 0.608 the CPU stopped on "base pressure stationary" at 4000 iterations while the GPU ran to the 20 000 limit (1.8 orders): the same unsteady-transient divergence as the M 0.8 base-on sweep row. The CFD base Cp is below the data in all six configurations (by 0.0116 – 0.0458). `CD base corr.` and `CD total corr.` print `-` for all six (sting-mounted).

### Timing (level 1, 32 CPU rayon threads vs RTX 3090 GPU, time to the stopping criterion)

* Whole sweep (16 runs): CPU 1555.7 s, GPU 60.13 s, speed-up 25.9×; per run 12.2× (M 0.8 base off) to 29.2× (M 0.8 base on). Per iteration at M 0.5 (1000 iterations from the free stream): CPU 8001 µs (base off) / 9882 µs (base on), GPU extended 291.9 / 312.1 µs, i.e. 27.4× / 31.7×; the extended-precision GPU costs 1.20× the pure-f32 variant.
* Validation: GPU 1.7 – 8.3 s per configuration against 46.8 – 240.3 s on the CPU. Full runs: sweep 28 min, validate 15 min wall (both include the CPU runs).

### Known limit

The level-2 sting case at M 0.5 stalls at 3.8 orders. It ends on `max iterations` at both 10 000 and 40 000 iterations, so it is a convergence floor, not an iteration limit.
