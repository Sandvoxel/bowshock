# bowshock-cfd — CPU reference solver (2D axisymmetric Euler)

Pure-Rust (f64, `rayon` for data-parallel loops) reference solver that the Luma/wgpu GPU kernels
will be validated against. Kernels are plain per-face / per-cell functions of their inputs with no
hidden state, so they port 1:1 to WGSL.

```
bowshock/
  Cargo.toml                 workspace
  bowshock-cfd/src/gas.rs         ideal gas, prim<->cons, physical flux, reflection
  bowshock-cfd/src/flux.rs        limiters (minmod, van Albada), MUSCL reconstruct, HLLC, Rusanov, shock sensor/blend   <- face kernels
  bowshock-cfd/src/grid.rs        structured grid metrics + algebraic cone grid
  bowshock-cfd/src/solver.rs      BCs/ghost cells, face flux pass, cell residual+dt pass, SSP-RK3
  bowshock-cfd/src/taylor_maccoll.rs   oblique shock + Taylor-Maccoll (RK4) reference
  bowshock-cfd/src/cone.rs        cone case set-up and post-processing
  bowshock-cfd/src/bin/cone.rs    validation driver
  bowshock-cfd/src/shock_expansion.rs  tangent-ogive geometry, tangent-cone and shock-expansion theory
  bowshock-cfd/src/ogive.rs       ogive-cylinder grid, case runner, wall post-processing, Perkins-Jorgensen data
  bowshock-cfd/src/bin/ogive.rs   ogive validation driver -> out/ogive/
  bowshock-cfd/examples/diag.rs   convergence diagnostics (per-cell residual map -> out/diag/)
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
cd cfd
cargo test --release                        # 17 unit tests (incl. a ~5 s convergence regression)
cargo run --release --bin cone              # M=2, 10° and 15°, grids 100x50, 200x100, 400x200 (≈2 min per limiter)
cargo run --release --bin cone -- --quick   # skip the 400x200 grid
cargo run --release --bin cone -- --limiter vanalbada [--mach 2.5]
cargo run --release -p bowshock-cfd --example diag -- 10 200 100 minmod 8000   # residual map -> out/diag/
cargo run --release --bin ogive -- --sweep  # ogive-cylinder validation M=1.98 (+ Mach sweep), ≈7 min; --quick skips the fine grid
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

### Next: transonic (not done; out of scope)

What the current solver lacks and what I recommend, in order:

1. **Far-field / inflow-outflow BCs.** Fixed free-stream ghosts and zero-gradient outflow are only valid when every boundary is supersonic and the bow shock stays inside the domain. For M ≲ 1.3 the shock detaches far upstream
   (and for M < 1 there is none, with a large subsonic perturbation field). Replace them with characteristic (Riemann-invariant) non-reflecting BCs, sub/supersonic switched per face by the sign of the normal Mach number, and an
   outer boundary 20 – 50 body lengths away (or a far-field vortex/source correction) with strong radial stretching; the grid generator must stop sizing the domain from the attached-cone shock angle (`h_end` above).
2. **Base / afterbody.** The body currently runs through the outflow plane, so there is no base. Transonic ogive-cylinders need either the cylinder extended long enough to be unaffected (done here) or a real base with a
   wake: an H-grid with a base face and a wake block (or an O/C topology), plus a sting if comparing to sting-mounted data. Inviscid base pressure is ill-defined (set by numerical dissipation), so report forebody + cylinder pressure drag and
   add base drag from an empirical base-pressure correlation, or move to a viscous model. Shoulder shock/boundary-layer interaction (flow on ogive-cylinders at M ≈ 0.8 – 1.2) is also a viscous problem: expect Euler to mis-place the shoulder shock and
   over-predict its strength; at least a coupled integral boundary layer, ideally RANS (Spalart-Allmaras), is needed for credible transonic drag.
3. **Convergence acceleration.** The explicit local-time-step SSP-RK3 at CFL 0.5 needs 3 – 9 k iterations at M ≥ 1.5 here; with subsonic regions the acoustic transit time grows like `1/(1 − M)` and the far field is large, so expect 10⁵+ iterations.
   Recommended: FAS multigrid with the existing RK3 as smoother (Jameson-style; it maps to GPUs as plain stencil kernels and keeps the one-kernel-per-pass structure), optionally with implicit residual smoothing; a point-/line-implicit
   or colour-ordered LU-SGS is the alternative but parallelises worse. Low-Mach preconditioning is not required down to M ≈ 0.5. Keep the Rusanov-blend shock sensor; add entropy fix/positivity checks for shock-induced separation-free but strong transonic shocks.
4. **Validation data.** Not searched in this work; NACA Report 1386 (above) covers M = 1.24 – 7.4 but with the caveats given.
