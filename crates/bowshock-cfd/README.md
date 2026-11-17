# bowshock-cfd — axisymmetric Euler / RANS solver (f64 CPU reference, Luma GPU port)

Pure-Rust (f64, `rayon` for data-parallel loops) solver of the axisymmetric Euler and Reynolds-averaged Navier-Stokes (k-omega SST)
equations, plus the same solver on the GPU behind the cargo feature `gpu` (on by default; `bowshock_cfd::gpu`, see "GPU solver" below).
Kernels are plain per-face / per-cell functions of their inputs with no hidden state, so they port 1:1 to array operations.

```
Cargo.toml                 workspace
crates/bowshock/            desktop app
crates/bowshock-core/       user-facing library (facade over the solver)
  crates/bowshock-cfd/src/gas.rs         ideal gas, prim<->cons, physical flux, reflection
  crates/bowshock-cfd/src/flux.rs        limiters (minmod, van Albada), MUSCL reconstruct, HLLC, Rusanov, shock sensor/blend   <- face kernels
  crates/bowshock-cfd/src/grid.rs        structured grid metrics + algebraic cone grid
  crates/bowshock-cfd/src/solver.rs      BCs/ghost cells, face flux pass, cell residual+dt pass, SSP-RK3
  crates/bowshock-cfd/src/bc.rs          characteristic (Riemann-invariant) far-field + static-pressure outflow ghost states   <- BC kernels
  crates/bowshock-cfd/src/implicit.rs    line-implicit (block-tridiagonal, approximate-factorization) pseudo-time scheme, Jacobians, ghost-state linearisation   <- convergence kernels
  crates/bowshock-cfd/src/multigrid.rs   FAS multigrid (measured, not selected: see "Subsonic")
  crates/bowshock-reference/src/taylor_maccoll.rs   oblique shock + Taylor-Maccoll (RK4) reference
  crates/bowshock-reference/src/shock_expansion.rs  tangent-ogive geometry, tangent-cone and shock-expansion theory
  crates/bowshock-reference/src/potential.rs   incompressible axisymmetric potential flow (axial source method) + Goethert rule: independent subsonic reference
  crates/bowshock-cases/src/cone.rs        cone case set-up and post-processing
  crates/bowshock-cases/src/ogive.rs       ogive-cylinder grid, case runner, wall post-processing, Perkins-Jorgensen data
  crates/bowshock-cases/src/body_grid.rs   large-domain stretched grid for nose-cylinder bodies (ogive, cone, power/parabolic/Haack noses)
  crates/bowshock-cases/src/transonic.rs   `transonic_case`, Stoney (RM L53K17) and Czarnecki-Monta (TN D-3516) data
  crates/bowshock-cases/src/subsonic.rs    subsonic case runner (sting / flat base + wake), drag breakdown, NASA TM X-2439 data
  crates/bowshock-cases/examples/diag.rs   convergence diagnostics (per-cell residual map -> out/diag/)
  crates/bowshock-validate/src/bin/cone.rs      validation driver
  crates/bowshock-validate/src/bin/ogive.rs     ogive validation driver -> out/ogive/
  crates/bowshock-validate/src/bin/transonic.rs   transonic Mach sweep driver -> out/transonic/
  crates/bowshock-validate/src/bin/subsonic.rs    subsonic Mach sweep, TM X-2439 validation, low-Mach study -> out/subsonic/
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
order of the parallel `Σ r²` reduction; iteration counts are unchanged), and `cargo test --release -p bowshock-cfd` passes (28 tests).

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

### Case runner and convergence (`bowshock_core::run::run_on`, `stop_reason`)

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
* Stop rules: 6-order residual drop, or "base pressure stationary" for base-on cases; iteration limit `10 000 << level` (`Numerics::subsonic`), i.e. 20 000 at level 1.

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
* Modelled base drag: `bowshock_reference::base_drag::base_drag_coefficient(M)` = 0.12 + 0.13 M² for M < 1, 0.25 / M for M ≥ 1, referenced to the base area. Source: S. Niskanen, M.Sc. thesis (2009), eq. 3.94, after Fleeman, *Tactical Missile Design*.
* It is scaled by (r_b / R)² to the reference area; 0 without base; NaN (printed `-`) for sting-mounted bases (the TM X-2439 configurations below), because it is a free-flight correlation. It is reported next to, not instead of, the Euler values (`PointResult::cd_base_model`, `cd_total_model`). Here the full disc gives r_b = R, so `CD base corr.` = 0.12 + 0.13 M² (M 0.5: 0.15250).

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


# GPU solver (feature `gpu`)

The GPU solver for the axisymmetric Euler / RANS equations of the CPU modules, written in plain [Luma](https://github.com/flipnoteman/Luma) array ops (lazy
arrays 0.3: views, region assigns, records, `cache()`, `select`, `Array<Df32>`, `iterate_all`, `iterate_until_all`): no custom WGSL. Each function
mirrors the CPU routine of the same name in this crate expression by expression; the doc comment states the equation. It replaced the
hand-written WGSL kernels of the first GPU port (Luma 0.2.4, see "History" at the end).

```text
crates/bowshock-cfd/src/gpu/
  mod.rs            `Precision`, `GpuError`, `GpuSolver` (the handle the other crates use); `f32s` / `df32s` include the solver once per precision
  body.rs           the explicit solver: gas, flux, bc, solver (MUSCL + HLLC / Rusanov blend + low-Mach correction, all `Bc`, solid block, SSP-RK3)
  implicit.rs       the line-implicit scheme `Scheme::Implicit`
  viscous.rs        viscous terms (`LazySolver::set_viscous`): gradients, Sutherland viscosity, face fluxes, hoop stress, diffusive time step, `2 lam I`
  turbulence.rs     the Menter SST (2003) k-omega update and its two scalar line solves
  control.rs        run control: `set_config` (a sweep point), `base_cells`, accessors
crates/bowshock-cfd/ISSUES.md   issue list (Luma and this crate)
crates/bowshock-validate/examples/bench.rs, bench_viscous.rs, eval_loop.rs, luma_issues.rs      speed tables (inviscid, viscous), where the `eval` loop spends its time, what still applies of the Luma issues
crates/bowshock-gpu-cases/
  src/{cone,ogive,run}.rs     CPU / GPU runners of the validation cases with identical stopping criteria, comparison metrics
  tests/explicit.rs           explicit scheme vs the CPU, every `Staging` and both precisions; free-stream preservation, large grid, six orders, ogive wall Cp
  tests/implicit.rs           line-implicit scheme vs the CPU (sting, base block, Fox body), options of `ImplicitParams`, loop forms
  tests/loops.rs              the three loop forms compute the same iteration; threads on `default_context()`
  tests/runs.rs               whole runs through `bowshock_core::gpu::GpuRunner` vs `bowshock_core::run::run_on` (transonic, subsonic, warm start, converged implicit)
  tests/viscous.rs            laminar plate G1, SST plate G2, G3 body (solid block) vs the CPU, explicit and implicit, stagings, converged G3 drag
crates/bowshock-core/src/gpu.rs           `GpuRunner`: the GPU run of a `bowshock_core::case::Case`, same stop rules and post-processing as the CPU runner
```

```text
cargo test --release -p bowshock-gpu-cases                       # accuracy vs the CPU solver, loop forms agree, whole runs
cargo run --release -p bowshock-validate --example bench [-- --quick]  # speed tables (--quick skips 400x200; --profile-only, --staging Fusion|Lines|Fluxes|Split)
cargo run --release -p bowshock-validate --example bench_viscous       # the viscous implicit scheme on the G3 body (--laminar, --levels 0,1, --staging ..)
cargo run --release -p bowshock-validate --example bench -- --implicit # the line-implicit scheme only (--no-implicit skips it in the default run); LUMA_TRIDIAGONAL=thomas|cyclic
cargo run --release -p bowshock-validate --example eval_loop           # where the time of the `eval` loop form goes
cargo run --release -p bowshock-validate --example luma_issues         # what still applies of the Luma problems below
cargo run --release --bin cone_gpu | ogive_gpu | transonic_gpu | subsonic_gpu    # validation + benchmark drivers (bowshock-validate; CSV in out/gpu*/)
```

## API

```rust
let ctx = luma::default_context()?;
let mut gpu = GpuSolver::from_host(&ctx, &host, Precision::Extended)?;   // host: a `bowshock_cfd::solver::Solver` (grid, config, block, scheme, state)
gpu.set_config(&cfg)?;          // free stream, CFL, limiter, first_order_iters, bc: a sweep point, state untouched
gpu.reset()?;                   // free stream, iteration 0, CFL ramp restarted   (warm start: `set_iter(first_order_iters)` + `restart_ramp()`)
let history = gpu.run(50)?;     // density residual of each of the 50 iterations, recorded on the GPU
let u = gpu.state_f64()?;       // conserved state as f64;  `base_cells()`: the two cells behind each base-wall row
                                // viscous: `from_host` of a viscous host (or `set_viscous(Some(v), &grid)` after `set_block`), `turb_state()`, `turb_residual()`
```

`Precision::F32` is `f32s::LazySolver` (`Array<f32>` everywhere), `Precision::Extended` is `df32s::LazySolver` (state, residual and time step in
double-float `Array<Df32>`). Supported: `Scheme::Explicit` and `Scheme::Implicit`, with or without a solid block, low-Mach correction, and the viscous terms
(laminar or Menter SST) of `bowshock_cfd::viscous` / `turbulence`. Not supported: the multigrid and smoothed schemes (`Unsupported`). `bowshock_core::gpu::GpuRunner` (facade) is the runner of a
`Case`, Euler or RANS: `Backend::Auto` uses it when `luma::default_context()` opens a real GPU (a software Vulkan device counts as none).

The `LazySolver`s are also public (`bowshock_cfd::gpu::{f32s, df32s}::LazySolver`) for the benchmarks and the tests that vary the `Staging` or the loop form
(`run` = `iterate_until_all` with the residual recorded, `run_replay` = `iterate_all`, `run_eval` = `eval_all` every iteration).
The step is traced and its kernels compiled when a run starts, and the free stream and the limiter phase are constants of the traced step, so a
run after `set_config` with another free stream traces (and, if Luma's program cache misses, compiles) its kernels again, so a sweep point pays the "First call" cost below (0.05 s f32, 0.9 s double-float on a 100x50 cone grid); `run(n)` calls with the same configuration reuse them. [INFERENCE: the cache miss is not measured separately.]

## What is in it

* `src/gpu/body.rs`: the solver, written once. `gpu/mod.rs` includes it twice: `f32s` (`Array<f32>`) and `df32s` (state, residual and time
  step in double-float `Array<Df32>`, `Precision::Extended`). The state is a record of arrays of the element
  type (`[Array<S>; 6]`: `rho, rho u, rho v, E`, the density residual of the step and the CFL of the next one) that goes through
  `iterate_all` / `iterate_until_all` as it is, `f32` or `Df32`. Covered: `Scheme::Explicit` and `Scheme::Implicit`, with or without a solid block (`set_block`:
  base/wake grids; block ghosts, base-wall face stencils and solid faces/cells are region assigns, no masks), `Limiter::{FirstOrder, Minmod, VanAlbada}`, `face_flux_lm` (HLLC blended with Rusanov by the shock sensor,
  optional low-Mach correction), all `Bc` variants (`Freestream`, `Extrapolate`, `SlipWall`, `Characteristic`, `PressureOutflow`),
  the axisymmetric pressure source, local time step, SSP-RK3 with the first-order switch (one traced loop per limiter phase).
  A nozzle exit on the base face (`set_nozzle`, after `set_block` and before `set_viscous`; `Solver::set_nozzle`) replaces the first base rows by a
  fixed supersonic inflow state: the flux stencils, the viscous ghost, the implicit left-hand side and the `k`, `omega` ghost select the jet
  state on those rows with `Array<bool>` masks, and the wall distance leaves the jet faces out; without a nozzle no mask is built.
  Viscous runs are in `src/gpu/viscous.rs` / `src/gpu/turbulence.rs` (below). Not covered: multigrid and smoothed schemes.
* `src/gpu/implicit.rs`: the line-implicit scheme `Scheme::Implicit(ImplicitParams)` (`bowshock_cfd::implicit`): approximate factorization
  `(I - dt J_i)(I - dt J_j)`, the second-order residual on the right, the first-order flux Jacobians with the Roe-type dissipation
  matrix (shock-sensor blend, entropy fix) on the left, ghost-state Jacobians (exact for the free stream, extrapolation and the slip
  wall, central differences with relative step 1e-3 for the characteristic and pressure-outflow far fields), CFL ramp in the loop
  state, defect correction, positivity damping. A 4x4 block is sixteen arrays (`Blk`, with exact 0 / 1 entries that cost nothing);
  the line solves are Luma's `solve_block_tridiagonal` on `[16, ni, nj]` stacks. The left-hand side is f32 in both precisions.
* Low-Mach preconditioning (`ImplicitParams::low_mach_precond`, `Numerics::viscous` below Mach 1): `P` (Weiss-Smith) replaces `I` in both factors, the defect operator and between the sweeps, dissipation `P |P^-1 A|`, preconditioned pseudo-time step (`gpu/implicit.rs` `LmGpu`, CPU `implicit.rs` `LowMach`). The right-hand side is unchanged. Example drag table (10 viscous points, RTX 3090, df32): 34 050 -> 20 950 iterations, Mach 0.10 14 100 -> 1 200 at the steady CD. Test: `tests/implicit_lowmach.rs` (GPU vs CPU, 1e-3 f32 / 1e-4 df32).
* `src/gpu/viscous.rs`: `LazySolver::set_viscous` (mirrors `bowshock_cfd::viscous`: Green-Gauss gradients, Sutherland viscosity, the corrected face gradients with the
  ghost-mirror geometry, the viscous face fluxes, the hoop stress, the diffusive time step and the `2 lam I` of the implicit operator).
  `src/gpu/turbulence.rs`: the Menter SST (2003) update (`bowshock_cfd::turbulence`): gradients, blending functions, face data, residual and line
  coefficients, the two scalar line solves (`solve_tridiagonal`), the clipped update and the eddy viscosity.
* Tests: `crates/bowshock-gpu-cases/tests/`; examples: `crates/bowshock-validate/examples/`.

The math reads like the CPU code, expression by expression (`body.rs`, HLLC star state; `c(x)` is the constant `x` rounded to the
element type, `&` borrows an array):

```rust
/// `F_K + S_K (U*_K - U_K)`, `U*_K = fac [1, u + (S_M - u_n) n_x, v + (S_M - u_n) n_r, E / rho + (S_M - u_n) (S_M + p / (rho (S_K - u_n)))]`
let star = |k: &Side, s: &A, s_un: &A| -> Rec {
    let [r, u, v, p] = k.w;
    let (f, e) = (&k.f, &k.e);
    let fac = r * s_un / (s - sm);
    let sm_un = sm - &k.un;
    [
        &f[0] + s * (&fac - r),
        &f[1] + s * (&fac * (u + &sm_un * nx) - r * u),
        &f[2] + s * (&fac * (v + &sm_un * nr) - r * v),
        &f[3] + s * (&fac * (e / r + &sm_un * (sm + p / (r * s_un))) - e),
    ]
};
```

CPU branches (`if sl >= 0 { .. } else if ..`, `if t == 0.0 { return c }`, `if z >= 1.0`) become `select`. The ghost cells are region
assigns into one padded primitive record (`body.rs`, `assign_ghosts` and the start of `eval_residual`):

```rust
let p = cons_to_prim(u.clone());                              // [ni, nj] primitives (a record `[A; 4]`)
let w = p.pad([(2, 2), (2, 2)], PadMode::Edge);               // [ni + 4, nj + 4], still lazy
w.slice(0, ..2).slice(1, 2..nj + 2).assign(ghost)?;           // per boundary: wall reflection, characteristic, pressure outflow, ...
```

`Fields::assign` writes the four fields at once; because `w` is a pending expression nothing is launched, `w` becomes
`select(region, ghost, old)` and fuses into whatever reads it. A face reads its four cells as four `slice`s of `w`, the residual is
`fi.slice(0, 1..) * w.slice(0, 1..) - fi.slice(0, ..ni) * ..`. The geometry (normals, lengths, weights, `1/V`, areas) is uploaded
once from the f64 `Grid`, rounded to `f32` or `Df32`. Handles that are meant to be the same lazy array (the primitives passed on
as the first-order reconstruction, `rho` of the Riemann ghost state, ...) are `.clone()`s; no place needs an independent copy
(`&a * 1.0`).

## Accuracy

Tests in `crates/bowshock-gpu-cases/tests/explicit.rs` (`cone_state_after_20_iterations_matches_cpu`: 40x20 cone grid, M = 2, 15 degree cone, minmod and van
Albada; `characteristic_far_field_matches_cpu_on_a_perturbed_state`: far-field box, M = 0.9 and 1.3, characteristic and fixed-pressure outflow;
`transonic_state_after_20_iterations_matches_cpu`: ogive-cylinder 144x70, M = 0.9; `low_mach_correction_matches_cpu`: the box at M = 0.3;
`base_block_explicit_state_matches_cpu`: base and Fox grids with the solid block): CPU and GPU solver start from the same state, run 20 iterations (the
first 5 first order), and are compared by `max |dU|/(1+|U|)` (state) and `max |r_gpu - r_cpu| / r_cpu` over the 20 residual-history values.
`cargo test --release -p bowshock-gpu-cases -- --nocapture` prints every row for all four stagings (default staging below; the other stagings give the same
numbers up to a few percent of the f32 rows).

| case | f32 state / residual | df32 state / residual |
|---|---|---|
| cone 40x20 minmod | 1.0e-6 / 9.0e-7 | 4.1e-14 / 3.0e-14 |
| cone 40x20 van Albada | 9.0e-7 / 8.1e-7 | 1.8e-10 / 2.1e-11 |
| box M = 0.9 characteristic | 8.9e-7 / 1.9e-6 | 2.7e-14 / 1.7e-14 |
| box M = 0.9 pressure outflow | 9.4e-7 / 1.9e-6 | 2.7e-14 / 2.0e-14 |
| box M = 1.3 characteristic | 9.6e-7 / 4.6e-7 | 3.0e-14 / 3.1e-14 |
| box M = 1.3 pressure outflow | 9.6e-7 / 4.6e-7 | 3.0e-14 / 3.1e-14 |
| transonic M = 0.9 144x70 | 7.6e-7 / 1.0e-6 | 3.5e-14 / 3.4e-14 |
| box M = 0.3 low-Mach van Albada | 9.0e-7 / 7.0e-7 | 2.2e-14 / 2.1e-14 |
| **target** | 1e-5 / 1e-4 | 3e-7 / 2e-7 (5e-7 transonic) |

All 8 cases pass in all four stagings and both precisions. The f32 rows are at the f32 round-off of the CPU comparison. The double-float solver uploads
the geometry as `Df32` computed from the f64 grid, so it agrees with the CPU to ~1e-14 relative (the 1.8e-10 of the van Albada cone is two orders above its
siblings [INFERENCE: sign decisions of the limiter on round-off-level differences in the uniform far field, 1e-16 on the CPU vs 1e-14 in double-float; not
investigated]). The three loop forms (`iterate_until_all`, `iterate_all`, `eval_all` loop) agree (`tests/loops.rs`: double-float bit-identical, f32 within
6.4e-7 of each other), and four threads sharing `default_context()` each reproduce the state of a private-context run
(`threads_on_the_default_context_agree`). The residual history of the double-float solver is recorded on the GPU as two `f32` records, `rn.to_f32()` and the
low word `(rn - rn.to_f32().to_df32()).to_f32()`, because the history of `iterate_until_all` is `f32`; with only the high word the history would carry a 6e-8
relative rounding, which would hide the 1e-14 agreement.

`GpuSolver` on the same kinds of set-ups: `extended_precision_reaches_six_orders_on_a_small_grid` (the double-float solver converges a 60x30 cone to the CPU's 6
orders), `ogive_state_and_wall_cp_match_cpu`, `characteristic_far_field_preserves_the_free_stream`, `large_grid_residual_history_matches_cpu` (400x200).
`tests/runs.rs` compares whole runs of the facade runner with the CPU runner (iteration counts, residual history, wall Cp, CD, base Cp): transonic cold and warm
start, subsonic sting and base cases, converged implicit states (iteration counts within `0.05 cpu + 5`, field within 1e-4 p_inf), cold restart on a reused
solver, `set_config` moving the free stream.

### Implicit scheme

`tests/implicit.rs`: sting case of `subsonic_case` (M = 0.5 ogive-cylinder, grid level 0 = 144x70, van Albada, low-Mach correction, `Numerics::subsonic`'s
`Scheme::Implicit`: CFL ramp 1 -> 30 at 1.02 per iteration, Roe-type left-hand side, characteristic far field with finite-difference ghost Jacobians, two
defect-correction passes), first 5 iterations first order, `Staging::Fluxes`, from the same state as the CPU solver (`LazySolver::from_host`). The left-hand side
is f32 in both precisions; `R` and the update are in the element type, so the fixed point is the element type's.

| 30 iterations (`implicit_scheme_state_matches_cpu`) | state `max abs(dU) / (1 + abs(U))` | residual history `max abs(r - r_cpu) / r_cpu` | tolerance |
|---|---|---|---|
| lazy f32 | 1.0e-6 | 1.8e-6 | 1e-3 |
| lazy df32 | 3.1e-9 | 4.3e-8 | 1e-4 |

With the solid block (the same test, 30 iterations; state / residual history; `subsonic_case(.., true)` is the base case with `j_sting = 0`, the Fox TM X-2439
body `fox_case(false, 0.6, 0)` has a sting and a boattail):

| case | lazy f32 | lazy df32 |
|---|---|---|
| base (`j_sting = 0`) | 1.0e-6 / 1.6e-6 | 2.4e-8 / 6.9e-8 |
| Fox sting + boattail | 1.1e-6 / 4.5e-7 | 3.6e-8 / 5.9e-8 |

The agreement is far inside the tolerances because the f32 left-hand side only steers the correction `dU = (LHS)^-1 R`: an f32 error in the left-hand side
scales the update by 1e-7, it does not enter `R`. The options of `ImplicitParams` (`implicit_options_and_loop_forms`, df32, 30 iterations): `roe = false`
3.5e-9, `lines = false` 2.5e-8, `implicit_bc = false` 3.4e-9, `sub_iters = 1` 5.5e-9, `MinDirection` with growth 1.2 and CFL 20 3.5e-6 (state; the residual
history, which has dropped to 2.5e-3 of its start, 5e-4), each against the CPU run with the same parameters. The three loop forms give identical states and
histories in both precisions: the line solves are traced and replayed like any other kernel.

Converged states (`implicit_scheme_converged_state_matches_cpu` in `tests/runs.rs`, double-float, `GpuRunner` and `bowshock_core::run::run_on` to 5 orders of
density-residual drop): M = 0.3 and 0.6 agree in iteration count (484 / 484 and 377 / 377 in the first measurement of this solver) and field
(`max abs(p - p_cpu) / p_inf` 1.3e-8 and 1.6e-7).

### Viscous terms and the SST model

`tests/viscous.rs`: CPU `Solver` and GPU solver start from the same state (`GpuSolver::from_host`: grid, block, viscous parameters, wall distance and the turbulence
state of the host) and run the same iterations (the first 5 first order); `max |dU| / (1 + |U|)` and the maximum relative difference of the residual history (from the
second iteration):

| case (iterations) | lazy f32 state / residual | lazy df32 state / residual |
|---|---|---|
| G1 laminar plate, implicit (30) | 5.4e-6 / 9.9e-4 | 1.2e-7 / 4.7e-7 |
| G1 laminar plate, explicit (30) | | 4.0e-13 / 2.1e-11 |
| G2 SST plate, implicit (50) | 3.8e-5 / 3.4e-2 | 1.5e-7 / 8.2e-5 |
| G2 SST plate, explicit (50) | | 6.8e-12 / 8.3e-10 |
| G3 body (solid block, no-slip body surface and base wall, SST), implicit (30) | 1.0e-6 / 4.4e-6 | 2.2e-8 / 6.4e-8 |
| G3 body, explicit (30) | | 1.6e-11 / 1.7e-13 |

(Tolerances of the tests: state 1e-3 for the laminar plate and the body, mean state 1e-3 and `k`, `omega` 1e-2 for the SST plate, 1e-6 for the explicit rows.)
`k`, `omega`, `mu_t` of the cells with `k > 10 k_inf` (the plate, 2012 cells): f32 1.3e-3 / 3.8e-4 / 1.2e-3, df32 5.4e-6 / 1.4e-6 / 4.0e-6; the G3 body (12422 cells where
`k` has moved off the free stream): f32 4.0e-6 / 1.3e-5, df32 2.7e-7 / 4.7e-7; the density residual norm of `k` agrees to the printed digits (G2: 6.504e-4 both). The
explicit scheme is round-off (all of it in `S`, with the f32 left-hand side only in the turbulence line solves: `k`, `omega` 1e-7 over all 9792 cells of the plate). The
implicit df32 rows are 1e-7, not 1e-14: the f32 left-hand side steers the update, and the viscous wall cells (first cell height 4e-6 on the plate, so very high aspect ratios
[INFERENCE: not isolated]) amplify its 1e-7 error more than the inviscid grids do (3e-9). `converged_body_total_drag_matches_cpu` (`#[ignore]`: the CPU's stop rule
`CD stationary` after 4000 iterations, 4.1 orders; the GPU solver runs the same 4000 iterations from the free stream in 3.3 s (f32) / 4.0 s (df32), the CPU about 27 s):
total CD 0.21300 on all three (f32 +0.001 %, df32 -0.000 %), `cd_friction` 0.09003, `cd_base` 0.12257, `cp_base` -0.1226, `y+` max 0.648, each equal to the printed digits;
through the facade (`crates/bowshock-core/tests/viscous.rs::g3_gpu_matches_cpu`, `Analysis` on `Backend::Cpu` and `Backend::Gpu`) the same numbers.

**Wall roughness** (`Viscous::roughness` = equivalent sand grain `ks` in nose diameters, `0` = smooth, bit-identical to before): the cells above the rough wall (the body surface; the base wall and the sting stay smooth) take the Aupoix (2015) "Colebrook" wall state `aupoix_wall(nu, |u_c|, d, ks)` (`turbulence.rs`, mirrored in `src/gpu/turbulence.rs`): `k_w = k_w+ u_tau^2`, `omega_w = min(omega_w+ u_tau^2 / nu, 60 nu / (0.075 d^2))`, `nu_t,w = min(k_w / omega_w, 1e5 nu)`, with `u_tau^2 = (nu + nu_t,w) |u_c| / d` found by 16 fixed-point iterations per cell. The `k` / `omega` ghosts are `2 k_w - k_c`, `2 omega_w - omega_c`, the face eddy viscosity of a no-slip face is `mu_t,w = rho nu_t,w` (`Solver::rough_wall`, read by `viscous::face_mu`; on the GPU `VFlow::rough`), so the discrete wall shear is `(mu + mu_t,w) u_c / d` (also in `bowshock_core::engine::run::friction`). The roughness is phased in over the first 8 wall cells (`ks min(1, (i + 1 - wall_i_start) / 8)`). Three things were needed for the solver to converge with it (found on the G2r plate and the ogive-cylinder at Coarse, Mach 0.3-3, 60 um, `ks / d` 6e-4): (1) 4 fixed-point iterations leave the start error of `u_tau^2` (up to `1 + nu_t,w / nu`) unresolved and the plate's residual stuck at 0.5-1 (5 or more converge); (2) on the base wall (cells 1.5e-2 thick against `ks`) the smooth `omega_w` bound is active, `nu_t,w ~ u_tau^2` and the iteration diverges (`mu_t,w / mu` 1e31 in the first step): the rough state is for the body surface only and `nu_t,w` is capped; (3) a roughness that starts as a step at the wall start feeds `k_w` with the low rough `omega_w` into the slip cells ahead of it (a column of `mu_t / mu` 15-45 against 3 smooth) and that column oscillates with period 2 at M 0.9-1.3 (80 000 iterations, stalled): ramps of 2 and 3 cells still do, 4 and 16 do not. Not the cause (tested): lagging the wall state (0.2 under-relaxation), under-relaxing `mu_t`, `lam` x2, the implicit operators (all of them already read the face `mu_t,w`). G2r (`sst_rough_flat_plate_cf`, ignored, G2 plate with `ks = 1e-3 x`, `x = 0.97`; the residual drops 4 orders in 350 iterations): local `cf` 5.409e-3, `ks+` 250, against Mills & Hang (1983) `(3.476 + 0.707 ln(x / ks))^-2.46` = 5.398e-3 (+0.2 %; Schlichting's fully rough formula, 6.272e-3, reads 14 % higher). Transitional trend of `cf(0.97)` (smooth 2.617e-3, `sst_rough_flat_plate_trend`, ignored): `ks/x` 1e-5 (`ks+` 1.8) 2.830e-3, 1e-4 (`ks+` 20.3, `k_w+` 0.36, `omega_w+` 4.73, `nu_t,w/nu` 0.075) 3.553e-3. GPU body check (Coarse ogive-cylinder + transition, `Backend::Gpu`, 60 um): all 13 Mach numbers 0.3-3 end `Stationary` in 4000-10 000 iterations (smooth 4000-5000, 14 050 at M 3), total CD 0.193 (M 0.8) / 0.288 (M 2.3) against 0.142 / 0.258 smooth (friction 0.089 -> 0.139 and 0.057 -> 0.090, base 0.0228 -> 0.0226 and 0.0798 -> 0.0755). The GPU port is checked against the CPU on the rough plate by `sst_rough_plate_state_matches_cpu_{f32,df32}` (same tolerances as the smooth plate; df32 state max 1.7e-7, f32 3.0e-5).

What is covered: `Scheme::Explicit` and `Scheme::Implicit`, laminar and SST, with and without the solid block. The no-slip walls are those of `bowshock_cfd::viscous::Walls`
(the domain `jmin` wall from `wall_i_start`, the body surface from `wall_i_start`, every base-wall face, the sting surface), applied at all four mirror sites
(`assign_ghosts`, `assign_block_ghosts`, `block_fluxes`, the implicit assembly with its `diag(1, -1, -1, 1)` Jacobians). The loop state grows to ten arrays (`k`, `omega`,
`mu_t`, the `k` residual; one-element dummies without SST), so the traced loops carry the turbulence state like the CFL. The block must be declared before the viscous
terms, and the boundary conditions of a viscous run are fixed by `set_viscous` (`GpuSolver::set_config` refuses another `bc`).

## Speed

RTX 3090, Vulkan, 15 degree cone at M = 2, minmod, MUSCL from the first iteration, 2000 iterations from the free stream (as `cone_gpu --bench`, whose
grids and settings these are), best of 3, microseconds per iteration, `bench --quick` (the 400x200 grid was not re-measured). Steady state: the first call of
every loop is run before the clock starts, so compilation and tracing are not in these numbers. CPU = f64 `Solver` on 32 rayon threads (500 iterations).
Lazy = `LazySolver::run` (`iterate_until_all`, residual recorded every iteration, one wait per 50 iterations) or `run_replay` (`iterate_all`, no history),
whichever is faster for the best staging. The GPU was shared with other jobs during these runs (the lazy numbers moved by about 10 % between runs).

| grid | CPU f64 | lazy f32 | lazy df32 |
|---|---|---|---|
| 100x50 | 602 | 33.2 | 190.6 |
| 200x100 | 1304 | 44.7 | 260.2 |

Lazy solver by staging and loop form (`until` = `iterate_until_all` + residual history, `replay` = `iterate_all`, `eval` = `eval_all` after every
iteration, 300 iterations):

| grid | staging | f32 until | f32 replay | f32 eval | df32 until | df32 replay | df32 eval |
|---|---|---|---|---|---|---|---|
| 100x50 | Fusion | 166 | 140 | 4176 | 732 | 702 | 4760 |
| 100x50 | Lines | 84 | 66 | 3546 | 512 | 481 | 3509 |
| 100x50 | Fluxes | 53 | **33** | 1756 | 209 | **191** | 1809 |
| 100x50 | Split | 62 | 43 | 1683 | 215 | 192 | 1706 |
| 200x100 | Fusion | 221 | 196 | 4646 | 1081 | 1025 | 4918 |
| 200x100 | Lines | 107 | 86 | 3559 | 574 | 543 | 5076 |
| 200x100 | Fluxes | 62 | **45** | 2529 | 430 | **260** | 1701 |
| 200x100 | Split | 104 | 60 | 1704 | 419 | 327 | 1701 |

(The 200x100 `Fluxes` df32 `until` / `eval` cells and the `Split` cells of that grid carry the run-to-run noise of the shared GPU:
an earlier run of the same binary gave 267 / 240 for `Fluxes` replay / until and 258 / 234 for `Split`.)

* **Staging** (`LazySolver::staging`, defaults `Fluxes` for both precisions): `Fusion` has no `cache()` at all, which is the code
  the equations say, and is the slowest everywhere: 3-4.4x the time of `Fluxes` in f32 and 2.5-4x in double-float, and its
  double-float kernels take 9 s to compile on the first call (`Lines` 6 s, `Fluxes` 0.9 s). `Lines` evaluates the padded primitive
  record once; `Fluxes` also evaluates the face fluxes; `Split` also the reconstructed states and the shock weight.
  Every `cache()` left in `body.rs` has its measured reason in a comment next to it.
* **Why `Fusion` is slow** (profile, below): the padded record `w` is `select(region, ghost, old)` four times over, so every kernel
  that reads `w` at a stencil cell recomputes the primitives and all four ghost expressions (about 100 operations each for the
  characteristic far field) at that cell; the face flux is read at four stencil cells in each direction and the stage updates nest
  in each other, so the kernel exceeds the fusion limits (`1024 operations`, `1537 inlined operations > 1536`) and is cut 19-33
  times per step into 67-91 kernels that all recompute what the others compute. Raising the limits (`LUMA_FUSION_MAX_NODES=4096
  LUMA_FUSION_MAX_INLINED=16384`) gives 47 launches and is not faster than `Fluxes` in the profile (178-205 us of GPU time against
  109-171 us). Minimal repro: `examples/luma_issues.rs`,
  section 1.
* **Loop form**: `iterate_all` replay is fastest; the residual history of `iterate_until_all` costs about 20 us per iteration; the
  plain `eval` loop re-builds the step in Rust every iteration and is 40-55x slower in f32 (see "The eval loop").
First call (construction + compile + trace + first 50 iterations), seconds, 100x50 (kernels are specialised to the grid shape):

| | lazy f32 `Fusion` | lazy f32 `Lines` | lazy f32 `Fluxes` | lazy df32 `Fusion` | lazy df32 `Lines` | lazy df32 `Fluxes` | lazy df32 `Split` |
|---|---|---|---|---|---|---|---|
| first call | 0.49 | 0.21 | 0.05 | 9.33 | 6.06 | 0.90 | 0.38 |

### Launches per SSP-RK3 step (`Profile`, 200x100, `bench` profile section)

| | f32 `Fusion` | f32 `Lines` | f32 `Fluxes` | f32 `Split` | df32 `Fusion` | df32 `Lines` | df32 `Fluxes` | df32 `Split` |
|---|---|---|---|---|---|---|---|---|
| launches | 67 | 26 | **14** | 20 | 67 | 26 | **14** | 20 |

(Fusion cuts per step: 19, 8, 0, 0. The first measurement after the update, with Luma's working tree as it was a few hours
earlier, gave 91 / 30 / 14 / 20: the `Fusion` and `Lines` counts move with Luma's fusion heuristics, `Fluxes` and `Split` did not.)
The 14 launches of `Fluxes` are 3 per Runge-Kutta stage (the padded record with the previous stage update and residual fused into
it, the i fluxes, the j fluxes), the time step, the final state of the step and the three kernels of the residual norm (a fused
sum, the second reduction pass, `sqrt(sum / N)`). One launch per residual evaluation is not reachable: the fluxes read the
padded record at four cells and the residual reads the fluxes at two, so each of the three needs its own evaluated buffer or
recomputes its inputs (`Fusion`). The reference (Luma's `bench_pde` Euler step, first order) is one kernel because its stencil is
one cell wide and has no nonlinear ghost expression.

### The eval loop

`LazySolver::run_eval` builds the step in Rust and `eval_all`s it every iteration: 1.7-1.8 ms per iteration at 200x100 with `Fluxes`. `cargo run --release -p bowshock-validate --example eval_loop` splits it up (us per iteration, 200x100):

| precision | staging | loop total | build the step (Rust only, nothing evaluated) | rest of the loop (`eval_all`) | launches / step |
|---|---|---|---|---|---|
| f32 | Fluxes | 1687 | 1498 | 189 | 14 |
| df32 | Fluxes | 1724 | 1454 | 268 | 14 |
| f32 | Fusion | 4058 | 4053 | 5 | 67 |

So 90 % of an iteration is the host building the step's nodes (about 3000 of them [INFERENCE: from the operation counts of the
fused kernels]; `luma::jit::create` and the program measuring behind it lead a callgrind profile of `eval_loop --trace-only`), not
launching: no kernel is compiled inside the loop (the first iteration takes 40 ms f32 / 850 ms df32 for compilation, after 20
warm-up iterations the average over 300 is the build time plus 0.2 ms), and the 14 launches cost 65-150 us of GPU time in f32
(profile, shared GPU). It is not our code: computing `el`, `un`, `c` and the physical flux once per face side (`Side`) instead of
three times changed the build time from 1.55 to 1.50 ms (3 %). Luma's per-operation build cost is 0.7-1.2 us and grows with the
size of the expression (`luma_issues` section 2). `iterate_all` / `iterate_until_all` trace the step once and replay it, which is
what the 33-45 us per iteration of the tables above are.

### Line-implicit scheme

`cargo run --release -p bowshock-validate --example bench -- --implicit` (RTX 3090, shared with other jobs): sting case of `subsonic_case`, M = 0.5, tangent
ogive + 6 d cylinder, grid levels 0 (144x70) and 1 (288x140), `Numerics::subsonic` (van Albada, low-Mach, `Scheme::Implicit`: CFL ramp
1 -> 30, two defect-correction passes), 200 iterations from the free stream (the first 100 first order, so both limiter phases are
compiled before the clock), best of 3, microseconds per iteration. CPU = f64 `Solver::step` on 32 rayon threads; lazy `until` = `iterate_until_all` with the residual recorded, `replay` =
`iterate_all`. Luma picks the algorithm of the line solves (default):

| case | level | grid | CPU f64 | lazy f32 until | lazy f32 replay | lazy df32 until | lazy df32 replay |
|---|---|---|---|---|---|---|---|
| sting | 0 | 144x70 | 2292 | 189.3 | 145.1 | 229.6 | 184.0 |
| sting | 1 | 288x140 | 10348 | 378.7 | 325.5 | 426.1 | 382.9 |
| base (solid block) | 0 | 156x76 | 2505 | 216.0 | 163.6 | 281.4 | 230.7 |
| base (solid block) | 1 | 312x152 | 10891 | 415.4 | 358.1 | 540.0 | 487.2 |

After the Luma `bowshock-fixes` port (Mask arrays, `solve_block_tridiagonal_parts`, one `solve_tridiagonal_parts` for k and omega, `is_finite`, Df32 `tanh`); the table above is that run, best of two. Before it (Luma c709621 with `stack_slice`, same session) the rows were 237.7 / 194.8 / 273.7 / 224.9, 464.6 / 414.5 / 524.9 / 472.7, 253.7 / 201.4 / 325.9 / 270.3 and 507.0 / 449.4 / 652.8 / 577.6. One implicit step as the eval loop launches it (level 0): sting f32 19 launches / 149 us (before 27 / 198), sting df32 20 / 196 (28 / 238), base f32 19 / 168 (27 / 210), base df32 20 / 375 (28 / 294: the Df32 flux and residual kernels of the block case got slower, see ISSUES 11). The six `FusedStack` launches and the two right-hand-side stacks are gone; the four line solves are `SolveParts` launches (base f32: 22.9 + 18.2 us per direction pair, as the `CyclicTridiagonal` launches before).

(GPU columns measured with the line-solve stacks built by `luma::stack_slice` (see "Luma issues", 11); with the 16-part `concat` they were 273.1 / 226.6 / 310.0 / 266.9, 582.5 /
540.5 / 639.4 / 590.5, 281.9 / 239.8 / 349.9 / 304.0 and 683.7 / 598.3 / 768.8 / 712.0. The CPU column is the earlier measurement.)

(A later run of the sting rows, with the base rows added; the CPU column moves by 2x between runs on the shared machine, the earlier run
of this table had 3592 / 7711.) The base case is the ogive-cylinder with the flat base and wake block (`j_sting = 0`); its f32 step is
27 launches, 204 us of GPU time (df32: 28 launches, 286 us; 256 / 339 us with `concat`), the line solves are the same four `CyclicTridiagonal` launches with
`starts: true` (44.1 + 38.2 us).

The lazy scheme is 5-30x faster than the CPU solver (10 / 18 times at level 1 in f32 / df32 replay). `LUMA_TRIDIAGONAL` forces the algorithm of the line solves (the other columns do not depend on it):

| `LUMA_TRIDIAGONAL` | level | lazy f32 until / replay | lazy df32 until / replay |
|---|---|---|---|
| unset (Luma chooses) | 0 | 276.6 / 236.4 | 316.3 / 266.3 |
| `cyclic` | 0 | 263.5 / 225.4 | 306.2 / 262.5 |
| `thomas` | 0 | 700.6 / 656.6 | 737.8 / 692.3 |
| unset (Luma chooses) | 1 | 610.6 / 552.9 | 654.9 / 625.7 |
| `cyclic` | 1 | 591.7 / 544.0 | 649.6 / 596.9 |
| `thomas` | 1 | 1567.2 / 1514.0 | 1624.0 / 1568.8 |

The default is cyclic reduction at both levels (its timings are the forced-`cyclic` ones within the noise of the shared GPU).

Where the time of one lazy f32 step goes (level 0, `profile_step`: 27 launches, 240 us of GPU time, 0 fusion cuts; the df32 step is 28
launches, 278 us), summed over the kernels of the profile:

| kernels | launches | GPU us |
|---|---|---|
| line solves: `CyclicTridiagonal { block: 4, workgroup: 32, shared_rows: 144 }` (i lines, 42.9) and `shared_rows: 70` (j lines, 38.8) | 4 | 81.7 |
| the 16-part `concat` that builds the `[16, ni, nj]` stacks of `lo`, `d`, `up` of both directions (issue 11; with `luma::stack_slice`: 6 launches, about 20 us) | 6 | 70.2 |
| ghost-state Jacobians at the line ends (32 outputs on `[1, nj]` / `[ni, 1]`) | 2 | 16.5 |
| blocks `lo`, `d`, `up` (48 outputs) | 2 | 13.7 |
| face matrices `FL`, `FR` (32 outputs) | 2 | 8.3 |
| 4-part `concat` (the right-hand side stacks; `stack_slice`: about 3 us) | 2 | 6.5 |
| 4-output kernels on `[144, 70]` (pseudo-time step, `dt R`) [INFERENCE: from the kernel shapes] | 1 | 9.4 |
| residual: padded state 5.9, face fluxes 9.5, cell residual 3.5, norm 8.0 | 5 | 26.9 |
| positivity damping and update, CFL | 2 | 6.8 |

The four line solves of a step are two passes (`sub_iters = 2`) of the i-line solve and the j-line solve. With `LUMA_TRIDIAGONAL=thomas`
they are `Tridiagonal { block: 4, workgroup: 8 }`, 128.5 us each (514 us of a 679 us step): one invocation per line (70 / 144 lines)
with a dependent chain of 144 / 70 block eliminations, as expected. `until` is about 40 us above `replay`: the residual history, as for
the explicit scheme.

### Viscous terms and the SST model

`cargo run --release -p bowshock-validate --example bench_viscous [-- --laminar] [--staging ..]` (RTX 3090, shared with other jobs): `Case::viscous` on the G3 body (tangent ogive 0.30 x
0.10 m + 0.60 m tube, ISA 0 m, Mach 0.5, flat base and wake block, `Scheme::Implicit(ImplicitParams::VISCOUS)`, van Albada with the low-Mach correction), grid levels 0
(156x122) and 1 (312x122), 200 iterations from the free stream (the first 100 first order), best of 3, microseconds per iteration; CPU = `Solver::step` (rayon), lazy `until` /
`replay` as above. Default staging `Fluxes`. The line-solve stacks are built with `luma::stack_slice`.

| model | level | grid | CPU f64 | lazy f32 until / replay | lazy df32 until / replay | CPU / lazy f32 replay |
|---|---|---|---|---|---|---|
| SST | 0 | 156x122 | 6724 | 378 / 319 | 508 / 447 | 21x |
| SST | 1 | 312x122 | 12454 | 554 / 487 | 713 / 645 | 26x |
| laminar | 0 | 156x122 | 4351 | 354 / 304 | 442 / 386 | 14x |
| laminar | 1 | 312x122 | 7683 | 570 / 513 | 701 / 616 | 15x |

(The CPU column moves by 2x between runs on the shared machine; the Luma `bowshock-fixes` run is the table above, before it the SST rows were 460 / 398 | 589 / 526 at level 0 and 700 / 637 | 866 / 808 at level 1.) **Double-float is cheap here**: `df32s` costs 1.3x of `f32s` (SST replay), against 2.5-3.2x for the explicit inviscid
kernels, because the left-hand side and the line solves, which are f32 in both, are most of the step. The viscous terms and SST in `S` are therefore the default in both precisions (no
f32 special case): the arithmetic that sets the fixed point is in the element type, only the pseudo-time-step operators (`2 lam I`, the diffusive time step, the two scalar line
solves) are f32. Staging (SST, level 0, `replay`, f32 / df32, measured with `concat` stacks): `Fluxes` 480 / 612, `Lines` 499 / 621, `Split` 498 / 625, `Fusion` 577 / 958 (first call 2.8 s /
14 s). `Fluxes` stays the default.

One SST step, level 0 (`profile_step`): after the Luma `bowshock-fixes` port (block line solves on parts, scalar solves of both turbulence equations in one launch, no stacking) 34 kernel launches, 337 us of GPU time (f32; df32 478 us; level 1: 497 us f32), before it 65 launches / 445 us (f32; df32 595 us; level 1 689 us), 1 fusion cut (`1537 inlined operations > 1536` in the
dissipation blocks). The biggest kernels, f32 level 0:

| kernels | launches | GPU us |
|---|---|---|
| mean-flow line solves: `CyclicTridiagonal { block: 4, shared_rows: 156 }` (i lines) and `shared_rows: 122` (j lines), 3 passes each | 6 | 133.3 |
| 4-output kernels (the `[4, ni, nj]` right-hand side stacks, pseudo-time step) | 2 | 27.0 |
| the `[16, ni, nj]` stacks of `lo`, `d`, `up` (`FusedStack`) | 6 | 25.4 |
| turbulence line solves: `CyclicTridiagonal { block: 1 }`, one per scalar and direction | 4 | 25.5 |
| assembly blocks (48, 48, 32, 56 outputs) | 4 | ~46 |
| residual norm (`FusedReduce`) | 1 | 9.6 |

## History: the hand-written WGSL kernels

The first GPU port (`GpuSolver` on Luma 0.2.4 with custom WGSL kernels: `shaders.rs`, `shaders_ext.rs`, `shaders_impl.rs`, `shaders_visc.rs`; modes
`Direct` / `Guarded`, precisions `F32` / `Compensated` / `Extended`, f32 geometry with a closure-defect correction) was replaced by this crate. Last
measurement of both on the same machine (microseconds per iteration, conditions as in "Speed"; custom = `Mode::Guarded`, history kernel on, `run(50)` chunks):

| grid | CPU f64 | custom F32 | custom Extended | lazy f32 | lazy df32 | lazy f32 / custom F32 | lazy df32 / custom Extended |
|---|---|---|---|---|---|---|---|
| 100x50 | 602 | 28.9 | 75.2 | 33.2 | 190.6 | 1.15x | 2.5x |
| 200x100 | 1304 | 33.7 | 81.9 | 44.7 | 260.2 | 1.3x | 3.2x |

Line-implicit scheme, sting case (level 0 = 144x70 and level 1 = 288x140) and base case (156x76, 312x152): custom F32 / Extended 105.8 / 143.7, 260.6 / 286.4,
96.9 / 142.9, 274.0 / 302.9 against lazy f32 / df32 replay 226.6 / 266.9, 540.5 / 590.5, 239.8 / 304.0, 598.3 / 712.0: the array-op scheme is 2.1-2.5x (f32) and
1.8-2.4x (df32) the hand-written kernels, 5-30x faster than the CPU solver. In accuracy the double-float array-op solver agrees with the CPU to ~1e-14 where the
`Extended` kernels agreed to ~5e-8 (the lazy solver uploads `Df32` geometry computed from the f64 grid, `Extended` kept f32 normals / weights / volumes plus a
closure-defect correction). Viscous implicit step on the G3 body (SST, level 0 / 1, replay): custom F32 / Extended 263 / 319 and 377 / 428 against lazy f32 / df32 480 / 612 and 764 / 942 us with `concat` stacks (1.8-2.2x), 387 / 519 us at level 0 with `luma::stack_slice` (1.5x / 1.6x). With `stack_slice` the inviscid implicit step is 1.7x (f32) and 1.5x (df32) the custom time at level 0 (180.6 / 217.6 against 105.8 / 143.7 us). The array-op solver is 2.0k lines (`lib.rs`, `body.rs`, `implicit.rs`, `control.rs`) against 3.5k lines for the kernels without viscous terms (`lib.rs` and three `shaders*.rs`) and reads like the CPU routines. The viscous terms and the SST model of the kernels (`shaders_visc.rs`) are not ported yet.

## Luma issues

See also `ISSUES.md` (the running issue list).

All four bugs the prototype hit are fixed in Luma (each has a regression test there; `CHANGELOG.md`, `[Unreleased]`), and the
workarounds are removed:

1. **A `Df32` kernel whose only library calls are comparisons cannot bind its uniform: fixed.** `Array::<Df32>::abs`,
   `minimum`, `maximum`, `lt` of two arrays bind correctly (fix: the guard uniform of Df32 kernels with only
   `abs`/`minimum`/`maximum`/comparisons). `df32s` uses plain `abs` / `lt` / `minimum` / `maximum` again (`aabs`, `alt`, `amin`,
   `amax` are gone). `luma_issues` printed the values `[1, 2, 3]`, `[1, -2, -3]`, `[1, 1, 0]` for `a.abs()`, `a.minimum(&b)`,
   `a.lt(&b)` before this file was shrunk.
2. **`x - x.to_f32().to_df32()` is 0 when fused: fixed** (casts to and from `Df32` are guarded against reassociation).
   `(x - x.to_f32().to_df32()).to_f32()` fused gives `x.lo` (`[-9.934108e-9, -1.9868216e-8]` for `[1/3, 2/3]`); `df32s::split`
   (with its `cache()`) is gone, and the residual history's low word uses exactly this expression.
3. **Input-buffer count after an evaluation: fixed** (buffers below a `cache()`d part no longer count toward the limit; the
   input-buffer limit is the device's, up to 256, not 16). The old repro prints 12 launches, or 15 with `s` cached, no cuts.
4. **Wrong results with several threads on `default_context()`: fixed** (each thread evaluates only its own lazy arrays);
   `crates/bowshock-gpu-cases/tests/loops.rs::threads_on_the_default_context_agree` runs four threads on it (the other tests keep a `Context::new()` each, for
   isolation).
5. **API gaps: closed.** `Array: Clone` (a clone is another handle on the same data: all `dup` uses were shared handles, none needed
   `&a * 1.0`); `Fields` and `iterate_all` / `iterate_until_all` take `Df32` states (no `hi` / `lo` packing, no `pack` / `unpack` of
   pairs); `concat` is lazy but computes every part at every cell, so ghost layers are region assigns now. Still true: no
   `Operand<f64> for f64` scalar on comparisons, `minimum` / `maximum` or `select` (a native `f64` variant would need `Array::full`
   constants; not done, the optional item of the task); the `iterate_until_all` history is `f32`.

What still applies (`cargo run --release -p bowshock-validate --example luma_issues`):

6. **Ghost layers written as region assigns into a lazy padded array cost operations at every reader.** The array is
   `select(region, ghost, old)`; a kernel that reads it at a stencil cell evaluates the ghost expression there, for each of the four
   sides, although the region is 2 cells wide. With ghost expressions of 100 operations the kernel reading the padded record at
   four offsets per direction is cut at `1537 inlined operations > 1536` and a one-kernel expression becomes four launches
   (`luma_issues` section 1: `ghost_ops` 0 / 10 / 40: 2 launches, 0 cuts; 100: 4 launches, 2 cuts). Expected: the cost of a region
   assign proportional to the region (e.g. the select lowered to a branch that evaluates the right-hand side only inside the region,
   or counted by the region's share of the kernel). In the solver this is why `Fusion` needs the padded record evaluated (`Lines`).
7. **Building a lazy expression costs 0.7-1.2 us per operation, rising with the expression size** (`luma_issues` section 2: 100
   operations 0.66 us each, 400: 1.2 us, 900: 1.1 us), which is 90 % of the `eval` loop form (see above). Expected: a flat, lower
   per-node cost (the program is measured at every node creation: `luma::jit::create`, `program::collect`).
8. **The residual norm of a step is three launches** (fused sum, second reduction pass, `sqrt(sum / N)` on the one-element result): 3
   of the 14 launches of a step. Expected: the scalar epilogue fused into the reduction's second pass.
9. **`Profile::splits` lists the cuts at node creation only** (unchanged from before): the "evaluated on its own first" decisions
   do not appear; the launch count of the step is what shows them.
10. **First call of a big `Df32` kernel**: 9 s of compile for `Fusion` / 6 s for `Lines` (0.9 s for `Fluxes`) at the first launch,
    `bench --staging Fusion` (fresh process, `__GL_SHADER_DISK_CACHE=0` to see it every time).
11. **Operands of `solve_block_tridiagonal` must be one `[16, ni, nj]` array, and there was no way to build it from sixteen `[ni, nj]`
    arrays other than `concat`: fixed by `luma::stack` / `stack_slice` (Luma 90e68d6, used since: the six stacks of a step cost about 20 us instead of 70-105 us, the implicit
    step 191 instead of 242 us of GPU time at level 0, 438 instead of 520 us for the viscous one).** What was measured with `concat`: `luma::concat::<f32, 16>(0, parts)` over the evaluated, reshaped parts costs 11.7 us per stack at 144x70
    (161k elements; a bandwidth-bound copy is 2 us, the launch itself 2-3 us): every output element selects among 16 loads. An implicit
    step needs six stacks (`lo`, `d`, `up` of two directions): 70 us of the 240 us GPU time of the step (29 %), as much as the four line
    solves (82 us). The alternative tried, `zeros([16, ni, nj]) + 0.0` as a lazy base and sixteen `slice(0, k..k + 1).assign(part)`,
    is 20x slower (4200-5100 us per iteration instead of 236-277; `src/implicit.rs::stack`). Expected: a `stack` operation that writes
    each part once (or the solvers taking the blocks as a record `[Array<f32>; 16]` / a strided view).
12. **`reshape` evaluates its operand** (it shares the buffer, so the array has to exist): a lazy `slice(0, k..k + 1)` of a `[4, ni, nj]`
    solve result cannot be reshaped to `[ni, nj]` without a copy kernel. The solution is unstacked with `take` and four precomputed
    index arrays (`4 (i + ni j) + k`), which stays lazy and fuses into the next kernel. Expected: a lazy `reshape` of a view (or of a
    unit-extent axis).
13. **No `is_finite`**: the positivity damping of the implicit step tests `|q| < 1e30` (NaN and infinity compare false), which relies on
    NaN comparing false in the generated WGSL. Nothing in Luma promises that, and the damped branch (`alpha < 1`) is not checked
    separately: the tests do not force a step that needs it.


## Going further

* **Viscous terms and SST: done** (`src/viscous.rs`, `src/turbulence.rs`, see "Viscous terms and the SST model" above): Green-Gauss gradients, the face-gradient correction and the
  viscous fluxes are slices and arithmetic as predicted; SST needed `tanh` (f32 even in `df32s`: `Array<Df32>` has no `tanh`), `sqrt`, Luma's scalar tridiagonal solves and a
  wall-distance array uploaded from `bowshock_cfd`.
* **`iterate_until_all` as the run loop.** It already is one here (`LazySolver::run` is one per limiter phase). The convergence rules of
  `bowshock_core::run::stop_reason` map onto it: `ResidualDrop` = `h["residual"].peak() / h["residual"].last() >= 1e6` (Luma's `peak` is over all steps, the CPU
  rule over steps after 20: use `h.step()` to gate), the monitor "stationary" rules = `window_max(k) - window_min(k) < tol` of a recorded Cd / base-Cp,
  `NonFinite` = `last().le(x).not()`. What does not map: the limiter switch (the traced step is specific to the limiter: one loop per phase, or the CFL ramp as
  a `luma::Scalar`, which changes without recompiling), warm starts across a Mach sweep (the free stream could be `Scalar`s, again no recompile; today a new free
  stream retraces and recompiles), and `StopReason` variety beyond one predicate index per rule.
* **Speed.** The remaining gap to the old hand-written kernels (14 launches per step against ~7, and the traffic of the padded record and the fluxes) is small in
  f32; in double-float it is the arithmetic: the flux kernels are 700-operation `Df32` expressions.

# 3D Euler solver (`gas3`, `grid3`, `flux3`, `bc3`, `solver3`, `implicit3`; GPU `gpu/body3.rs`, `gpu/implicit3.rs`)

The 3D Euler stability solver of the bowshock plan: the finite-volume scheme of the axisymmetric solver on a Cartesian hexahedral grid of the half
domain around a sting-mounted body with thick fins, f64 on the CPU and the same code in Luma array ops on the GPU (f32 and double-float). It gives
normal force and centre of pressure at incidence (`bowshock_core::engine::{case3, run3, gpu3}`, `Analysis::run_lift`); the 2D solver stays the drag source.

```
crates/bowshock-cfd/src/gas3.rs         Cons3 / Prim3 = [rho, rho u, rho v, rho w, E] / [rho, u, v, w, p], physical flux, reflection, free stream
crates/bowshock-cfd/src/grid3.rs        Grid3: nodes, i / j / k faces (area vector, centroid), volumes, SolidBox, `revolve`
crates/bowshock-cfd/src/flux3.rs        MUSCL (limited in the face's cylindrical frame, `face_rot`), HLLC, Rusanov, shock blend, low-Mach
crates/bowshock-cfd/src/bc3.rs          characteristic far field, static-pressure outflow
crates/bowshock-cfd/src/solver3.rs      Solver3: ghost fill, face fluxes, cell residual, local dt, SSP-RK3 (Scheme3::Explicit)
crates/bowshock-cfd/src/implicit3.rs    three-factor approximate-factorisation scheme (Scheme3::Implicit), 5x5 Jacobians, block Thomas
crates/bowshock-cfd/src/gpu/body3.rs    LazySolver3: the explicit scheme in array ops; GpuSolver3 (mod.rs) is the precision-independent handle
crates/bowshock-cfd/src/gpu/implicit3.rs  the implicit scheme in array ops (5x5 blocks as 25 arrays, `solve_block_tridiagonal_parts::<25, 5>`)
crates/bowshock-core/src/engine/grid3.rs  the mesher (Grid3Params, Fins3, body_grid3): axisymmetric sting grid revolved, one-cell-thick fin layers
```

## Conventions

* Velocities are Cartesian: x axial, y lateral, z vertical. `alpha > 0` means the air moves towards +z (`freestream3`): the windward side is -z and a
  positive normal force points to +z. Non-dimensionalisation as in 2D: `rho = 1`, `c = 1`, `p = 1/g`, `u = M`.
* The grid is the half domain `theta in [0, pi]` (`(x, r sin theta, r cos theta)`; `theta = 0` is the +z side): the pitch plane `y = 0` is a symmetry
  plane at both ends (`k = 0` and `k = nk`). Cell, face and node layouts are those of `Grid3` (`i` fastest); the GPU arrays are `[ni, nj, nk]` with the
  same order, `[ni + 4, nj + 4, nk + 4]` with two ghost layers on every side for the padded primitives (no corner ghosts: every stencil is 1D).
* Faces are flat quadrilaterals: `S = (d1 x d2) / 2` of the diagonals, `V = (1/3) sum c_f . S_f`. Every cell closes (`sum S = 0` to 1e-13 of the largest
  face), so a uniform flow is preserved to round-off on any grid (`freestream_is_preserved`: 4e-16 on a perturbed grid, explicit and implicit, M 0.8 at 6 degrees).
* **Flat-face chord error.** A revolved grid replaces the cylinder by a prism with `nk` flat sides per half domain: the wall faces see the body radius
  reduced by up to `R (1 - cos(pi / 2 nk))`. The wall Cp of a 15 degree cone against the 2D solver on the same meridian grid
  (`wall_cp_converges_to_the_2d_solution`, M 2, both converged 6 orders) is `e(16) = 1.35e-3`, `e(32) = 3.6e-4` (ratio 0.27: second-order in the chord).

## Boundaries (two ghost layers per side, as the 2D `NG = 2`)

|boundary|ghost state|
|---|---|
|`i = 0`, `i = ni`, `j = nj`|`characteristic_ghost3` with the outward normal of the boundary face (Riemann invariants on the normal velocity, the tangential velocity vector from the upwind side); both layers get the same state|
|`j = 0`, `i < axis_cells` (the axis)|layer `m` = `mirror_y` (negate `v`) of the cell `(i, m - 1, k')`, `k'` the cell whose azimuth contains `pi - theta_c(k)` (`axis_k`, computed once); the axis faces have no area|
|`j = 0`, `i >= axis_cells` (body, sting)|slip wall: `reflect3` about the face normal|
|`k = 0`, `k = nk`|symmetry planes: `reflect3` about the face normal|

## Fins (`Grid3::solid`, `SolidBox`)

Fins are one-cell-thick layers of solid cells in `k` (the mesher moves the layer edges to the fin thickness over the planform, so the fin faces are flat planes
through the axis). Solid cells are never updated, have `res = dt = 0` and are left out of the residual norm. A face between a fluid cell and a solid cell is a
wall face: the stencil states on the solid side are the reflections of the fluid cells, `(p0, p1) = (R(m1), R(m2))` about the face normal (`R` =
`reflect3`); a face between two solid cells carries no flux; a fluid face whose outer stencil cell is solid sees the reflection of the adjacent fluid cell about
the face between them. The half fin of an odd fin count lies on the `k = nk` symmetry plane (nothing special). In the implicit scheme a solid cell is an
identity row (`d = I`, `lo = up = 0`, `dt = 0`, so a line through a fin solves as independent segments) and the fluid cell next to it drops that off-diagonal
block `B` and adds `B T` to its diagonal with `T = I - 2 n n^T` (on the momenta) of the wall face.

## Face fluxes and the frame limiting

`face_flux_lm3` is the 2D `face_flux_lm` with a normal `n` and two tangential components carried by the HLLC contact. The reconstruction limits `(rho, u, v_r,
v_t, p)` with `v_r = v sin(phi) + w cos(phi)`, `v_t = v cos(phi) - w sin(phi)` in the cylindrical frame about the x axis at the face centroid (`face_rot`, one
`[sin, cos]` per face, precomputed): limiting the Cartesian `v`, `w` would break the axisymmetry of a flow at `alpha = 0` (the limiter acts per component), the
frame limiting keeps it to round-off (`axisymmetric_flow_stays_axisymmetric`: azimuthal velocity 5e-16 for minmod, 5e-11 for van Albada, whose `1e-14`
regularisation is discontinuous at `b = 0`) and makes the 3D flux in the `theta = 0` plane equal the 2D one to 1e-14. A side whose slopes all vanish or whose
reconstructed density / pressure is not positive is the cell state itself (first order).

## Implicit scheme (`Scheme3::Implicit`)

The 2D scheme (`implicit.rs`) with three factors solved i, then j, then k, 5x5 blocks, no source term: `(I - dt J_i) X = dt R`, `(I - dt J_j) Y = X`,
`(I - dt J_k) dU = Y`; `R` is the full second-order residual, `J` the first-order Jacobians with the entropy-fixed Roe `|A|` blended to `lambda I` by the shock
sensor; `sub_iters` defect-correction passes against `I - dt J_i - dt J_j - dt J_k` (`r = b - (A_i + A_j + A_k - 2 I) du`); update halved up to 8 times until
`rho, p > 0`; the CFL ramps `cfl0 -> cfl_max` by `growth` per iteration. Ghost Jacobians on the first / last cell of each line: characteristic far field frozen (`T = 0`, CPU and GPU alike: the f32
finite-difference Jacobian at the kink of the characteristic branch switch destabilised the GPU left-hand side at CFL ~ 6, ANF M 2.48 nk 48: a far-field corner
cell lost its pressure and the run froze with a constant residual), slip wall and symmetry planes `T = I - 2 n n^T`, the axis `T = 0` (its faces have no area, so the block that `T` multiplies is zero).

* **Diagonalised line solve (`ImplicitParams::line_solve = LineSolve3::DiagonalCorrected`, selected by `Numerics3::auto`; `Block` is the default of `ImplicitParams::DEFAULT`).** Per cell the factor `I - dt J_d` becomes `T (I + dt delta Lambda) T^-1` (`implicit3/diag3.rs`, GPU `DiagFac3` / `Eig` in `gpu/implicit3.rs`): five scalar tridiagonal solves per line, and one defect-correction pass against the block operator. ANF level 0, f32, `cfl_max` 10, RTX 3090: step 3151 -> 1935 us, same iterations and CN (within 0.01 %) as `Block`, 1.56x to the stop rule. One diagonal pass without the correction settles 0.25 % off at M 0.8 and is not offered. The tangent pair of the eigensystem breaks ties towards the lower axis within 1e-4 on CPU and GPU. **The f64 CPU solver diverges with this scheme at `cfl_max` 10 on the tiny test grid (alpha 0 and -2 at M 2; stable at `cfl_max` 5)**; see ISSUES #20.
* **`cfl_max` 10 for every Mach number** (`Numerics3::auto`; the 2D value is 30 subsonic / 20 supersonic): the three-factor scheme diverged on the coarse ANF
  grid once the ramp passed about 15 (it did at `cfl_max` 20 and 30; 10 and 5 converge). If a finer grid diverges at 10 as well, drop to 5; the plan's
  last resort is to treat the k direction point-implicit (k Jacobians on the diagonal only).
* **Stop rule and precision of the facade (overnight A/B, `lift3d --sweep`, ANF level 0, 17 Mach x alpha 2, 6; deltas against the 500-window / 1000-floor
  rule in df32, which was the default before).** CN settles to 1e-4 within 200 to 300 iterations, so the old window of 500 with a floor of 1000 mostly counted
  iterations after convergence (13 of 16 points of the example table stopped exactly on the floor). `Numerics3::auto` now uses a **window of 250 with a floor of
  250** (`window`, `stationary_min_iters`, `warm_min_iters`, knobs `--window/--stat-min/--warm-min` of `lift3d`): table 36 288 -> 21 783 iterations, max |dCN|
  0.17 % (M 0.966), max |dx_cp| 0.0024 cal; example lift 20 400 -> 12 250 iterations, max |dCN| 0.18 %, |dx_cp| 0.4 mm. A window of 200 fails (0.54 - 0.68 % at
  M 0.95 / 0.966, where CN cycles by +-0.3 % for 2000+ iterations: a physical-looking limit cycle of the transonic shock, not slow convergence). Not adopted:
  floor 500 only (-13 % iterations), warm start without restarting the CFL ramp (the ramp is ~115 of the iterations: -3 %), requiring x_cp stationarity
  (changed nothing), a mean-of-windows rule for the cycling CN (never fired), `cfl_max` 15 (-6 to -9 % iterations, +-0.2 % CN) and 20 (an example point stalls, 1 % CN).
  The facade's GPU lift path runs **`Precision::F32`** (`analysis.rs`): CN agrees with df32 to 0.007 % and x_cp to 1e-4 cal on the whole table (M 0.8 and 2.0
  included), 0.02 % on the example; step 4.2 against 5.0 ms. f32 never reaches the 6-order residual drop (plateau near 4.5 orders), so points that used to end
  `Converged` now end `Stationary` at the first full window.

## GPU

* `LazySolver3` (`f32s`, `df32s`) mirrors `Solver3` expression by expression: the state `[rho, rho u, rho v, rho w, E]`, the residual of the step and the CFL of the
  next one (`State3 = [A; 7]`) are loop state of `iterate_until_all`; the density residual is recorded on the GPU (double-float as two words) and read once per `sync` iterations.
  The ghosts are region assigns into the padded primitives (the axis ghost a `take` with the host-computed partner index), the face fluxes are three
  kernels (one per direction, `Staging::Fluxes`).
* The faces next to fin cells are recomputed from the wall stencils and written over the cached fluxes: per direction one box of face indices that contains all of
  them, case masks from the host (`SolidFaces3`: zero flux, solid before / after the face, solid on both sides of the stencil), one region assign. Faces of the box
  that see no solid cell keep their own stencil, so the box costs a few percent of the faces (ANF).
* The implicit left-hand side is f32 in both precisions (as in 2D): Jacobians, dissipation, ghost Jacobians (central finite difference with the relative step
  1e-3: the CPU's `1e-7` is below f32 resolution), `dt`, the three `solve_block_tridiagonal_parts::<25, 5>` per factor pass and the defect correction. A 5x5 block is
  25 arrays (`Blk5`, structural zeros and ones are not arrays); the solves read the parts without stacking.
* `set_config` changes the free stream, CFL, limiter, `first_order_iters`, low-Mach flag and scheme between runs without touching the state or the geometry;
  changing the scheme (or its parameters) restarts the CFL ramp, a change of the free stream does not (`restart_ramp`).

### Accuracy against the f64 `Solver3` (`cargo test --release -p bowshock-gpu-cases --test euler3d -- --nocapture`)

Tiny ANF-like case (`body_grid3`, 106 x 24 x 8 cells, 2 solid boxes: 10 degree cone-cylinder, four square fins of chord 1 d, span 1 d, thickness 0.08 d, sting
grid counts `n_up 10, n_nose 16, n_seg [12], n_ext 10, nj 24`), 2 degrees, 30 iterations with the first 5 first order from the free stream. `max |dU| / (1 + |U|)`, the
worst case over the stagings:

|scheme|M|f32|df32|target f32 / df32|
|---|---|---|---|---|
|explicit, cold|0.5 (van Albada, low-Mach)|1.2e-6|5.5e-11|1e-5 / 3e-7|
|explicit, cold|2 (minmod)|1.8e-6|8.6e-14|1e-5 / 3e-7|
|explicit, after 150 CPU iterations|0.5 / 2|1.9e-6 / 2.3e-6|1.5e-10 / 9.8e-14|1e-5 / 3e-7|
|implicit, `cfl_max` 10 (`Numerics3::auto`)|0.5|1.1e-6|5.7e-9|1e-3 / 1e-4|
|implicit, `cfl_max` 10|2|1.5e-6|2.9e-8|1e-3 / 1e-4|
|implicit, steep ramp (`cfl0 3`, `growth 1.25`, `cfl_max` 10)|0.5 / 2|7.6e-5 / 5.2e-5|9.4e-8 / 6.0e-8|1e-3 / 1e-4|

Every solid-face pattern (one-cell-wide box, two layers with one fluid cell between them, a wider box, a half layer on the `k = nk` plane; `solid_box_patterns_*`):
explicit 1.8e-6 (f32) / 4.4e-14 (df32); implicit with frozen ghost Jacobians (`implicit_bc = false`, steep ramp, 30 iterations) 7.5e-7 / 6.2e-8; the characteristic far field is frozen on both sides, so CPU and GPU use the same operator there. Free-stream preservation on the GPU (perturbed far-field box, M 0.8 at 6 degrees, 5 iterations): explicit 1.7e-7 (f32) /
9.4e-15 (df32), implicit 3.7e-7 / 1.0e-14.

### Speed (`lift3d --case anf|ogive --level 0|1 --precision f32|df32 --bench N`; `PERF` lines in `perf/ledger.jsonl`)

Implicit step, M 2, 2 degrees, second-order from the first iteration, replay of 200 iterations after 20 warm-up (RTX 3090; the RTX 3080 Laptop numbers are the
`remote` rows of the ledger):

|case|cells|f32 3090|df32 3090|
|---|---|---|---|
|ANF level 0 (187 x 70 x 24, 2 solid boxes), E2, luma before 152c127|314 160|10.26 ms|10.94 ms|
|ANF level 0, transposed i factor (P1 workaround), luma before 152c127|314 160|4.62 ms|5.36 ms|
|ANF level 0, luma 152c127, solved in place (187 x 70 x 24)|314 160|4.22 ms|4.97 ms|
|ANF level 0 (210 x 70 x 24, 2 solid boxes), before the P2 pass (`d54beda`, current luma)|352 800|3.91 ms|4.75 ms|
|ANF level 0, P2: lazy blocks in the line solves, flux-difference defect correction (current)|352 800|**3.15 ms**|**4.00 ms**|
|ANF level 0 on the RTX 3080 Laptop (`remote`), before / after P2|352 800|8.70 / 6.94 ms|10.58 / 8.91 ms|
|ANF level 1 (385 x 140 x 48, RTX 3090), before / after P2|2 587 200|31.8 / 26.6 ms|37.4 / 32.2 ms|
|fin-less ogive level 0 (144 x 70 x 24), E2|241 920|-|6.85 ms|

**Layout of the line solves.** Luma's parts solve ran one invocation per line, and along axis 0 the neighbouring lines are `n` elements apart: the 80 loads per
step did not coalesce (the 1680 i lines of 187 cells: 3.3 ms per solve, against 0.45 ms along axis 1 or 2 on the same data). The P1 workaround assembled and solved
the i factor on transposed arrays (i solve 0.45 ms, step 10.9 -> 5.4 ms). Luma 152c127 ("contiguous lines always cyclic") solves those lines in place in 0.32 ms
(`SolveParts` Cyclic, workgroup 128), and the step is faster without the transposes (4.97 against 5.36 ms): the workaround was removed. Time to the stop rule on ANF
level 0, M 2, 2 degrees (`converge3d`, df32): 345 iterations, 2.2 s; `sub_iters` 1 needs 398 iterations and the same 2.2 s, so it stays 2. Launches per step: tiny
case explicit 33 (f32) / 30 (df32).

**P2 pass (ISSUES.md #18).** Two changes to the left-hand side of `implicit3.rs`, no change of the result (the ANF sweep M 0.79 / 1.99 / 2.48 at 2 and 6 degrees: same
iteration counts, CN within 3e-9, x_cp within 4e-9 calibers): the blocks `lo`, `d`, `up` of the three factors are not evaluated (the 75-array assembly kernels,
0.84 ms, are gone; `SolveParts` computes them from the face matrices where it reads a cell), and the first defect-correction pass is `N_i (X - dU) + N_j (Y - dU)`
from the intermediates of the solves with the products `N_d z` in flux-difference form (two kernels over the face matrices instead of three block products). Profile of
one f32 step now (3090, 27 launches, 3.13 ms): the six line solves 2.29 ms (73 %: j 2 x 0.42, k 2 x 0.38, i 0.36 + 0.33), the three face-matrix kernels 0.32 ms, the two
`N_d z` kernels 0.23 ms, the rest 0.29 ms; the solves are the Luma part (`~/Documents/luma/Temp.md`). Staging stays `Fluxes` (f32 3.15 / `Lines` 3.16 / `Fusion` 3.16 /
`Split` 3.28 ms; df32 3.99 / 4.98 / 4.98 / 4.04 ms).

### Run

```
cargo test --release -p bowshock-cfd --lib          # CPU (solver3, implicit3, grid3, flux3, bc3): grids, kernels, free stream, symmetry, 2D consistency, fin walls
tools/gpuq/gpuq submit --host local -- cargo test --release -p bowshock-gpu-cases --test euler3d -- --nocapture
tools/gpuq/gpuq submit --host local -- cargo run --release -p bowshock-validate --example lift3d -- --case anf --level 0 --precision df32 --bench 200
```

### The S1 topologies on the GPU (stage S2: `gpu/body3.rs`, `gpu/implicit3.rs`, `tests/s2.rs`, `examples/{bench3,cfl_scan,sector_vs_half}.rs`)

The GPU solver mirrors the CPU `Solver3` on the S1 grids (`Grid3`: per-cell parts, axis runs, `Symmetry::{Half, Sector, Wedge}`) expression by expression:

* **Axis runs** (`Geo3::axis`, one `AxisGeo3` per `AxisRun`): the ghost of the two rows below the axis is the weighted two-source state of `axis3::AxisMap` (`rho`, `u`, `p` weighted, `(v, w)` through the
  2x2 matrix of each source, a one-source ghost has weight 0 on the second): same cell with `-I` (wedge and even-`N` sector), the mirror cell (half domain), interpolation (odd `N`). The columns of
  `j = 0` that are not on an axis run keep the slip-wall ghost.
* **Masks**: `PART_DEAD` cells (upstream of a revolved nose, never updated) are in the update mask but are no walls; solid cells with a part are the walls of the outer-stencil classes and of the force classes.
* **Wedge** (automatic from `Symmetry::Wedge`, `LazySolver3::{skip_k, no_dt_k, k_wall_flux, k_src}`): the flux through the two k planes is the cell's own pressure on the face normal `[0, p n, 0]`, no `L_k` in the
  time step (`MinDirection`: `2 min(l_i, l_j)`), no k line, and the 2D-style pressure-source block `dt/V (A1 n1 - A0 n0) dp` on the diagonal of the j factor (`factor3`, applied in `n_apply3`, so
  that the `DiagonalCorrected` defect carries it as on the CPU).
* **Forces on the device** (`set_wall_faces`, `wall_forces`): the wall faces of `run3::wall_faces` (class, cells `a`, `b`, `k = d0 / (d1 - d0)`, weights) as arrays, the wall pressure `p_a + k (p_a - p_b)`,
  the class sums (all faces and the faces up to the base station) as masked reductions, one download of `6 n_classes` numbers every 50 iterations; the stop rule monitors `ca` for the wedge, a sector and a
  half domain at zero incidence. No state download in a run.

Checks (`cargo test --release -p bowshock-gpu-cases --test s2`, through gpuq, `max |dU| / (1 + |U|)` against the f64 `Solver3` on tiny grids, 30 iterations explicit / block, 20 diagonal, targets of
`euler3d.rs`): wedge (M 0.5, 2), base half domain at 2 degrees, sectors N = 4 and 3 (`nk` 6, 7): explicit f32 1.6e-6 - 2.0e-5 / df32 6e-13 - 1e-13 (the one-cell wedge amplifies the f32 round-off, target 5e-5 there),
block 7.9e-7 - 3.9e-5 / 1.9e-8 - 7.2e-8, diagonal 5.6e-7 - 2.0e-5 / 2.3e-8 - 5.2e-8; the device forces equal the host `wall_forces` (class by class, all faces and cut) to 1e-13 (df32) and 3.8e-5 (f32).

**`cfl_max` on the wedge** (`cfl_scan`, Euler base-wake grid of the example rocket 156 x 76 x 1, the facade stop rule, GPU f32 `Block`; the CPU f64 gives the same iteration counts and `ca` to 4 digits):

|Mach|`cfl_max` 5|10|20|50|
|---|---|---|---|---|
|2|400 it|350|350|350 (`ca` 0.30036 / 0.30036 / 0.30035 / 0.30052)|
|0.9|1750|1050|8000 (cap, `ca` 0.1625)|8000 (cap, 0.1951)|

`DiagonalCorrected` is non-finite at iteration 72-88 at every `cfl_max`, on the CPU as on the GPU (ISSUES.md #24); the gates run the wedge with `Block`. `cfl_max` 10 stays the default (above it the transonic M 0.9 point
does not settle).

**Gates** (`BOWSHOCK_WEDGE_BACKEND=gpu cargo test --release -p bowshock-core --test wedge_gates -- --ignored --nocapture gate_w1 gate_w4`, through gpuq; the runner is `tests/wedge_gates/gpu.rs`): 99 PASS, 4 FAIL.
G-W1: all Euler base cases within 2 % except `table-euler-M0.90` (cd_total 0.17052 against 0.17480, -2.4 %; the CPU wedge gives the same 0.17052 as the GPU) and the M 0.5 cases, which the 2D reference ran to its
iteration cap and the gate does not gate. G-W4: the four cones, the two ogives, the transonic smoke case and the subsonic base case pass; the two subsonic smoke cases fail only on the name of the stop reason
(`CdStationary` of the 3D stop rule against the 2D `ResidualDrop`), their numbers pass. The medium cones and the level-1 ogive end on the plateau rule after 8000 iterations at 3-4 orders (their coefficients are within the band).

**Sector against half domain** (`sector_vs_half`, GPU f32, level 0 of the Army-Navy Basic Finner at zero incidence, sector `nk` 6 against the half domain with the cell count that replicates it; total drag
coefficient on `pi/4`; cell width = angle of one azimuthal cell, `theta_span / nk`):

|N|Mach|sector (cells, width)|half (cells, width)|sector total (fins / nose)|half total (fins / nose)|d total|
|---|---|---|---|---|---|---|
|4|2|210 x 70 x 6, 7.50 deg|210 x 70 x 22, 8.18 deg|0.48227 (0.37873 / 0.10343), 350 it|0.48947 (0.38631 / 0.10300), 350 it|-1.47 %|
|4|0.8|same|same|0.68853, **8000 it, not converged**|0.73496, 600 it|-6.32 % (sector not settled)|
|3|2|210 x 70 x 6, 10.00 deg|210 x 70 x 17, 10.59 deg|0.34467 (0.24147 / 0.10309), 350 it|0.34616 (0.24330 / 0.10270), 350 it|-0.43 %|
|3|0.8|same|same|0.51824 (0.50205 / 0.01630), 800 it|0.53448 (0.51851 / 0.01589), 600 it|-3.04 %|

The 1 % band is met only by N = 3 at M 2. At M 2 the nose differs by +0.4 % (sector larger) and the fins by -2.0 % (N = 4) and -0.75 % (N = 3); the polygon bias of the revolved cells at these widths
(`1 - sin(dtheta)/dtheta` = 0.3 % at 7.5 deg, 0.5 % at 10 deg) would make the sector smaller, not larger, so it does not explain the nose. [INFERENCE: the fin difference is the discretisation at the fin, a half
cell with a mirror on the symmetry plane against a full layer cell, see `wedge.rs::sector_vs_half`; not isolated.] The N = 4, M 0.8 sector does not settle (iteration cap); at M 0.8 the half domains need 600 iterations.

**Performance** (`bench3`, Euler `Scheme3::Implicit`, M 2, `cfl_max` 10, replay after 20 warm-up iterations, best of 3, RTX 3090, one process per row; us per step, kernel launches / GPU us in brackets; the wedge, sector
and ANF grids are the S1 meshes: `Case3::wedge` on the example table's Mach 3 grid, `Case3::sector` of the Army-Navy Basic Finner with 18 cells over `[0, pi/4]`, the lift half domain):

|grid|cells|f32 block|f32 diagonal|df32 block|df32 diagonal|
|---|---|---|---|---|---|
|wedge 156 x 140 x 1|21 840|357 (22 / 372)|186 (30 / 216), diverges|547 (23 / 567)|382 (31 / 413), diverges|
|sector 210 x 70 x 18|264 600|2 627 (27 / 2 552)|1 559 (38 / 1 542)|3 277 (28 / 3 235)|2 237 (39 / 2 218)|
|ANF half domain 210 x 70 x 24|352 800|3 168 (27 / 3 171)|1 953 (38 / 1 945)|4 040 (28 / 4 033)|2 820 (39 / 2 766)|
|device memory, wedge (MiB)||97|119|187|133|
|device memory, sector (MiB)||969|907|1 134|1 076|
|device memory, ANF (MiB)||1 161|907|1 326|1 076|

The diagonal wedge rows are timed with `--allow-nonfinite` (the step cost does not depend on the values). The Euler wedge table of the example rocket (`gpu_euler_wedge_table_perf`: the ten Mach numbers of the
fixture's `table-euler` on their own grids, each from the free stream, `Block`, `cfl_max` 10, the facade stop rule): df32 (the precision of the 2D table) 6.8 s and 5 900 iterations, f32 8.5 s and 9 850 iterations
(M 0.1 needs 5 150 in f32: the stationarity of `ca` is judged at the round-off of the sums), against 16.6 s and 55 264 iterations of the 2D Euler table (ref2d.json; its M 0.5 and M 0.9 points ran to 10 000 / 13 500
iterations). cd_total against the 2D table: within 0.32 % at M 0.8, 1, 1.1, 1.3, 1.7, 2.3, 3; M 0.1 +0.91 % (df32) / -0.13 % (f32); M 0.9 -2.45 %; M 0.5 -16 % (the 2D point ran to its iteration cap, see G-W1).

Device memory of the sector in f32 with the diagonal scheme against its size (`nk` 6, 18, 36, 72; 147 840 / 443 520 / 887 040 / 1 774 080 cells): 453 / 963 / 1 765 / 3 493 MiB, about 1.96 kB per cell and
0.18 GB fixed: the Euler step alone needs 2.3 kB per cell with the block scheme and df32 3.0 kB, more than the 0.6-1.0 kB of DESIGN3D E, and a 1.7 M-cell half domain is 3.5 GB before any viscous or SST array.
`tools/gpuq/gpumem -- CMD` measures any command the same way.

### The 3D viscous kernels on the GPU (stage S5a: `gpu/viscous3.rs`, `tests/s5a_visc.rs`)

Port of `viscous3.rs` (read its module header: the ghost in the plane of the face, the reflected gradient across symmetry planes, the wall ghost `(rho, -u, -v, -w, p)`, the eddy viscosity of a no-slip face
`2 mu_t,w - mu_t`), expression by expression, kernel level only (not in `GpuSolver3`'s step yet; `GpuSolver3::set_viscous` builds the host data, `LazySolver3::viscous_probe3` runs the kernels on a given padded state):

* `VGeo3`: the host-built face data of the three families (`face_dirs`: `e`, `1/d`, flux mask `d != 0`; `face_cells`: the Green-Gauss mask, the ghost and wall flags of each side, no-slip), uploaded once.
  The wall mask (`face_cells` sides, solid and dead cells) is the update mask of S2, so a dead cell is no flux and no wall of its own; the solver's state arrays are read through the padded record `w` of `assign_ghosts3`.
* `viscous_cells3` (the cell pass: Sutherland `mu = T sqrt(T) (1 + S) / (T + S) / re_c` and the Green-Gauss gradient as per-face contributions `0.5 (phi_c + phi_neighbour) n A` of the three families, the wall
  side being the wall ghost), `viscous_fluxes3` (the face pass: ghost-selected side states and cell data, the reflected gradient `R G R` written into the two symmetry slabs of each family by region assigns, the compact
  face gradient, the flux `[tau.n, u.tau.n + kappa grad T.n]`), `viscous_residual3`, `diffusion_rates3`, `lambda_pass3` and `wall_tractions3`: the viscous force by wall class `[Fx, Fz, Mz]` as masked
  reductions of the same fluxes the mean flow sees (`-sigma Fv.n[1..4] A` per no-slip face, one download of `3 n_classes` numbers), next to the pressure forces.

Tests (`cargo test --release -p bowshock-gpu-cases --test s5a_visc`, through gpuq): the same padded primitive state, solid mask, symmetry sides, `mu_t` and `mu_t,w` into the CPU `viscous3` and the GPU, every output
compared (`max |gpu - cpu| / max |cpu|`) on a skewed box with three solid blocks (one a dead block), the one-cell wedge and the fin sectors N = 4 and 3, laminar and with eddy viscosity, symmetry planes in `k` (and `j`
on the box): f32 at most 7e-6 (the wedge cell gradient; every other quantity 1e-6 or below), df32 at most 4e-13 (cell gradient, flux, spectral radius, diffusion rate, residual, wall forces).

Kernels alone (`bench3 --viscous`, one evaluation of the laminar viscous residual of the current state: cell pass, the three face families, the residual, the face spectral radii; padded state evaluated beforehand;
RTX 3090): wedge 156 x 140 x 1 f32 7 launches / 53 us, df32 7 / 249 us; sector 210 x 70 x 18 (264 600 cells) f32 7 / 238 us, df32 9 / 884 us (the implicit Euler steps of the table above are 186-357 us and
1.6-2.6 ms). Device memory of the process with the viscous face data allocated: wedge 123 / 209 MiB (f32 / df32, step 97 / 187), sector 969 / 1 150 MiB (step 969 / 1 134). One fusion cut, `viscous3.rs:304`
(the face energy flux): `1537 inlined operations > 1536` (f32; df32 adds two `recomputations of an expression cost more than storing it` cuts); harmless here (one extra launch). [INFERENCE: a `cache()` of the
face gradient record removes it; not tried, for S5b to decide with the whole step in view.]

### The 3D SST update on the GPU (stage S5a: `gpu/turbulence3.rs`, `tests/s5a_turb.rs`)

Port of `turbulence3.rs` (written by the `GpuTurb3` agent in the worktree `gpu3d-turb`, merged), kernel level only: `TurbModel3<N>` with `Sst3` (`N` = 2; the lists of scalars are written for `N` as on the CPU), `wall_state_pass3`
(Reichardt inversion with its fixed 8 Newton steps, the Aupoix state with its fixed 16 steps, per-cell `ks`; run on the gathered wall cells only, `take` / `put`: in df32 the transcendental iterations on all cells cost
as much as the rest of the update), `scalar_gradients3`, `blends3`, `face_data3`, `cell_terms3` (four `staged3` groups), `solve_and_update3` (the three scalar tridiagonal line solves of Luma's parts solver, one launch per
direction, no k line when `skip_k`, solid rows identity, a ghost that follows its cell adds nothing to the diagonal) and `turbulence_update3`; the wall distance is computed on the host (`walldist3`) and uploaded.
`LazySolver3::turbulence_probe3` runs the stages on the CPU `TurbIn` and returns every stage; the geometry is a standalone `TGeo3` (`VFace3.noslip` of the viscous geometry gives the wall cells).

Tests (`--test s5a_turb`, through gpuq; a jittered box with a solid block, the same box with symmetry planes, a y+ sweep 0.2-300 with rough cells, the tiny wedge, the tiny sector; each stage against the CPU,
relative to its own largest value, floor 1e-3, gradient normwise): f32 worst: wall state 3.2e-5, gradients 5.8e-6, F1 and X 3.3e-5, face data 9.9e-6, R and d0 7.0e-6, line coefficients 2.3e-7, new `k`, `omega` 8.4e-6, `mu_t` 1.2e-5;
df32 worst: 1.2e-12 (wall state), 1.4e-13, 1.6e-12, 4.9e-13, 4.9e-13, 1.3e-14, **1.1e-5** (`k`, `omega` after the line solves) and **8.1e-6** (`mu_t`): Luma's tridiagonal parts solver takes `f32` arrays only, so `dphi` is
f32-accurate in both precisions and `mu_t` inherits it (it acts as pseudo-time damping; the fixed point `R = 0` is the same). The df32 target 1e-8 is therefore asserted at 5e-5 for those two rows; iterative refinement
is the way to a true df32 solve (not done).

Kernels alone (`turbulence_profile3`, best of 20 after warm-up, `Staging::Fluxes`, RTX 3090, other agents' jobs share the GPU: single runs came out up to 5x slower): wall-state pass / update, wedge-like 156 x 140 x 1:
f32 27 launches 0.09 ms / 16 launches 0.087 ms, df32 27 / 0.24 ms, 17 / 0.164 ms; sector 210 x 70 x 18: f32 27 / 0.14 ms, 17 / 0.83 ms, df32 27 / 0.35 ms, 18 / 1.53 ms. No fusion cuts under `Fluxes`; under
`Staging::Fusion` the update is 2-10x slower (f32 1.8 ms wedge-like, 1.3 ms sector; df32 6.9 ms, 7-12 ms) with the usual `1537 inlined operations > 1536` cuts. Device memory with them allocated is not measured separately.

CPU findings of the port: `scalar_gradients` took the cell as its own face neighbour (`hi as usize ^ 1`: every scalar gradient about zero, the ghosts never entered); fixed in this tree, `phys3d` takes the same
one-line fix. The GPU runs every kernel on all cells and ends each stage with a select to the CPU's solid-cell values; at rest the Newton `0/0` is `u_tau = 0`; `hypot` is `m sqrt(1 + (n/m)^2)` (no `hypot` for `Df32`);
the arguments of F1 and F2 are capped at 1e3 as in the 2D `turbulence.rs`; the probe clamps the wall distance (inf in solid cells) and sanitises solid-cell primitives (the solver's padded array must stay finite there).
**Reading `lowmach5.rs` (S5a item 5; the block path is implemented in S5b, the diagonal one is not and stalls on the CPU too): what the GPU block and diagonal paths needed.** `P = I + kappa a g^T` is rank one in conservative variables
(`kappa = 1/U_r^2 - 1/c^2`, `a = (1, u, v, w, H)`, `g = (gamma - 1)(q^2/2, -u, -v, -w, 1)`, `U_r = clamp(|u|, ur_min, c)`, `kappa = 0` and so `P = I` where `U_r = c`: no branch is needed), so `P x` costs a dot product
and five multiply-adds per cell, never a 25-array block.

* **Block path** (`gpu/implicit3.rs`): (1) `factor3`: the diagonal starts from `P` instead of `Blk5::eye()` (25 products `kappa a_i g_j`, added like the wedge source block of S2); (2) `implicit_step3`, `approx_factor3_parts`: the
  sweeps become `(P - dt J_i) X = dt R`, `(P - dt J_j) dU = P X`, `(P - dt J_k) dU = P dU`, i.e. a `P` application between the three solves; (3) `n_apply3` / the defect residual use `P dU` instead of `dU`
  (`b - (P - dt J_i - dt J_j - dt J_k) dU0`); (4) the face dissipation `abs_jacobian_f5` becomes `abs_jacobian_pc3`: the Roe state, `M = P^-1 A`, three entropy-fixed `|lambda|` (`u_n`, `u' +- c'`), the
  quadratic polynomial in `M` and `P |M|`: three more 5x5 products per face and direction (`Blk5::mm` has the lazy `E::{Z, O, A}` algebra for it; `P` and `P^-1` are `I + rank one`, so two of the three products
  are rank-one updates), with the unchanged shock-sensor blend towards the Rusanov `lambda I`; (5) `pseudo_dt3`: `|u'| + c'` of `precond_speeds` for `|u_n| + c`. `ur_min` is a host constant of the free stream
  (`LowMach::for_freestream`). [INFERENCE: the face-matrix assembly grows by about the cost of `abs_jacobian`; the fusion limits of that kernel are not measured here.]
* **Diagonal path** (`diag_factor3`, `diag_du3`): the per-cell eigensystem `T` of `A` is replaced by `eig_pc5` (closed form: `R`, `L` and `L P^-1`; the first sweep applies
  `L P^-1`, the later ones `L`), the scalar coefficients take `lam_pc5` for the Jacobian eigenvalues and `face_dis_pc5` (`roe_eigs_pc5`, the shock blend kept as the scalar `t lambda`) for the dissipation. The defect pass is
  the block operator with `P`, so the whole block list above is needed for the diagonal scheme too; the diagonal scheme only adds `eig_pc5` and the scalar eigenvalues. The wedge-wide fact of S2 stays: `DiagonalCorrected`
  diverges on the wedge (ISSUES #24), so the wedge starts on the block path, where the low-Mach `P` and the wedge source block both live in the diagonal of the j factor (`factor3`).
* **Luma**: nothing new is needed (`sqrt`, division, the tangent-vector rule `tangents` shared with `diag3`; the GPU `Eig` of `diag_factor3` has the same `cons_to_char` / `char_to_cons` shape); `P` enters the boundary
  treatment nowhere (ghost Jacobian blocks unchanged), as the CPU module says. The explicit low-Mach flux correction of `body3.rs` (`cfg.low_mach`) is independent of it.

### The viscous / SST step on the GPU (stage S5b: `gpu/{rans3,implicit3,body3,turbulence3}.rs`, `tests/s5b.rs`, `examples/{plates_gpu,bench3,setup_cache}.rs`, `GpuRunner3::new_rans`)

`GpuSolver3::from_host` takes a `Solver3` with `set_viscous_with` (laminar or SST) and runs the whole viscous iteration on the device, the order of `solver3/rans.rs`: cell pass (gradients, `mu`), SST wall state, viscous
face fluxes, inviscid fluxes, `res = -(1/V) sum S (F - Fv)` (`rans3.rs: residual3`), the diffusive terms in the pseudo-time step, the implicit step, the SST update with the residual evaluation and the first-stage `dt` (explicit)
or the start-of-step evaluation (implicit). `State3` is ten arrays (`U` x5, residual norm, CFL, `k`, `omega`, `mu_t`; one-element placeholders without SST) so the turbulence state rides the lazy loop. Ported with it:

* **Boundaries**: the no-slip solid side `(rho, -u, -v, -w, p)` of every fluid/solid face (also in the MUSCL stencil), `diag(1, -1, -1, -1, 1)` in the ghost Jacobian; the `i = ni` static-pressure outflow (analytic
  Jacobian); the `j = 0` columns of `Grid3::wall_j0` (the body surface of a grid without solid cells under it) are no-slip in a viscous run (ghost layers and ghost Jacobian by column mask); the `k` mirror ghost of a wedge is
  the host's, and the **swirl projection** of the wedge residual (`res_v, res_w -= (res_v cos - res_w sin)(cos, -sin)`, `th = theta_span / 2`) is applied after the viscous divergence.
* **Implicit step**: `2 lam_f I` on the dissipation of every face (`lambda_pass3`), the diffusive terms in `pseudo_dt3` (`no_dt_k` drops the k term of a wedge), `Block` only: the viscous diagonal path and the diagonal +
  low-Mach path are refused by `from_host` (`Unsupported`; the CPU diagonal + `P` stalls, the diagonal on a wedge diverges: ISSUES #24).
* **Low-Mach preconditioning, block path** (`ImplicitParams::low_mach_precond`): `P` of every cell from the f32 primitives (`p_cells3`, rank one, identity in solid cells), `P` in the diagonal block, between the sweeps
  (`approx_factor3_parts`: two factors on a wedge, `P` between them only), in the defect (`b - (sum_d A_d - (nd - 1) P) dU`; the first defect `N_i (X - dU) + N_j (Y - dU)` is unchanged, `N_d = A_d - P`), `P |P^-1 A|` as the face
  dissipation (`abs_jacobian_pc_f5`: `P^-1 A` and `P` as rank-one updates, the Newton-form polynomial through the entropy-fixed `|u_n|`, `|u' +- c'|`, shock blend towards the plain Rusanov `lambda I`), `|u'| + c'` in
  `pseudo_dt3`. The explicit flux correction (`cfg.low_mach`) was already on the device.
* **Wall treatment** (CPU 8484d6f .. e9ed05a): Reichardt + bump law, `k` cap, law strain at smooth wall cells with the speed **tangential** to the cell's no-slip faces of the law class (`wall_speed_at3`: `|u - (u.e) e|`, largest
  over the faces; the `i` faces count only with `law_i_walls`, a cell with only `i` walls is a plain wall cell: speed 0, no law strain, no `k` cap: `TGeo3.wall_ns/wall_e/wall_counted`, built on the host from `face_dirs`).
* **Forces**: `GpuRunner3::wall_forces` adds the shear of the no-slip faces, reduced on the device (`TractionFace3`: class, direction, face index, sign, area, `x`; `viscous_wall_forces`), to the pressure force of its
  class (`WallForces::shear`, `[all faces, faces up to the base station]` like the pressure; the base wall's shear included as on the CPU); the monitor of the stop rule is the total `ca`; `y_plus_report`
  (host, once per run) fills `PointResult3::y_plus_max` and `cp_base`.
* **Runner**: `GpuRunner3::new(case, prec)` runs a `Case3::viscous` case (the host solver comes from `cache3::shared()` with `set_viscous_with` applied); `run_on` takes `Block` unless the case asks for the
  diagonal solve explicitly (the diagonal default of `Numerics3::implicit_for(true)` blows up on the finned M 3 / 6 degree half domain at `cfl_max` 10 and 5, is stable at 3, and Block is stable; ISSUES #24) and
  feeds the mean-stationary rule (chunks of 10 iterations after `mean_from`).
* **Motor jet** (CPU b71420f, `Solver3::set_jet`): `LazySolver3::new_with_jet` marks the `Part::Jet` cells solid and not wall (`wall` mask, `LGeo3`), the exit-face stencil is `(jet, jet, p0, p1)`
  (`SolidFaces3::{ja, jb}`, `solid_fluxes3`), the implicit factor sees the padded jet state with no wall ghost, no wall Jacobian and no sub-diagonal block, the viscous side of the exit reads the jet
  cell's own data (`VFace3.ghost` false for `SideKind::Jet`; `jet_fix_cell3`: the gradient of the fluid cell behind the exit; `mu(T_jet)` is the cell pass's own; `jet_fix_turb3`: `k`, `omega`,
  `mu_t = rho_jet k / omega` re-stored after every turbulence update, which zeroes solid cells), `set_state` / `set_jet` keep the jet cells at the jet state (a reset, a warm start at another Mach number).

Tests (`tests/s5b.rs`, through gpuq, 10 x 8 x 6 skewed box with a no-slip block and a dead block, 30 iterations from the free stream, `max |dU| / (1 + |U|)` against the f64 CPU): laminar M 0.3 / 2 explicit f32 6.6e-7 / 1.3e-6,
df32 7.6e-12 / 1.5e-11, block f32 3.6e-7 / 4.0e-7, df32 3.3e-9 / 1.0e-8; SST M 0.3 Re 2e3 / 2e5 explicit f32 7.3e-7 / 6.8e-7, df32 1.1e-12 / 8.1e-12, block f32 3.8e-7 / 4.7e-7, df32 4.1e-9 / 3.9e-9, with
`k, omega, mu_t` f32 <= 5.9e-6 and df32 <= 1.1e-7 (explicit) / 9.9e-8 (block); low-Mach block + `P` (M 0.1 / 0.3 laminar, M 0.1 SST) f32 1.0e-6 / 7.6e-7 / 2.0e-6, df32 7.0e-9 / 1.0e-8 / 1.2e-8 (`k, omega, mu_t` 1.2e-5 / 1.9e-7);
the G3 wedge (156 x 122, level 0, M 0.5, the 2D case's block + `P` numerics, wall_j0 body, 20 iterations) laminar f32 1.6e-6, df32 2.2e-8, SST f32 1.6e-6 (`k, omega, mu_t` 1.6e-5), df32 2.9e-8 (5.1e-7). Targets
1e-5 / 3e-7 explicit, 1e-3 / 1e-4 implicit (f32 / df32).

Plates (`plates_gpu --case g1|g2|g2r|yplus:<y+>`, the step-free boundary-wall plate of `plate3`, `Block`, `ImplicitParams::VISCOUS`, df32; `--cpu` runs the CPU on the same case): G1 `cf` +0.65 % / +1.64 % against Blasius at x = 0.4955 /
0.8909 in 1250 iterations (CPU +0.65 / +1.64, 1250); G2 -4.25 % against the correlation at 0.9615 in 703 iterations (CPU -4.25, 709; 2D -4.25, 717); G2r +0.23 % against Mills-Hang (2D +0.20); `y+` sweep at 0.9615
(vs `y+` 0.5): 1: -2.5 %, 5: +4.0 %, 15: -1.1 %, 30: -0.4 %, all inside the 5 % band.
