# 3D physics hand-off (phys3d, branch `cfd-3d-phys`)

Scope: the CPU (f64) RANS integration and kernels in `crates/bowshock-cfd`: `viscous3.rs`, `walldist3.rs`, `turbulence3.rs`, `lowmach5.rs`, `plate3.rs`,
`solver3.rs`, `solver3/rans.rs`, `solver3/jet.rs`, `implicit3.rs`, `implicit3/diag3.rs`, `implicit_params.rs`. The GPU mirror is `gpu/*` (`gpu3d`); the mesher and
engine are `solver3d`'s. Narrative history and more numbers are in `DESIGN3D.md` (sections cited by item number or title). Convention below: **[measured]** = a
number I read from a run in this session (job id, test or commit given); **[not re-measured]** = taken from DESIGN3D / earlier sessions; **[INFERENCE]** = my reading, not
isolated by a measurement.

Units: `rho_inf = c_inf = 1`, lengths in nose diameters, `nu = mu / rho`, Sutherland viscosity (`Viscous::sutherland`).

## 1. The physics as built

### viscous3 (`viscous3.rs`)
Laminar Navier-Stokes terms in Cartesian components (no hoop terms): `mu(T)` Sutherland, `kappa = (mu/Pr + mu_t/Pr_t)/(gamma-1)`, Green-Gauss cell gradients of
`(u, v, w, T)`, face gradient `g_f = gbar + ((phi_R - phi_L)/|d| - gbar.e) e`. Every fluid/solid face is a **no-slip adiabatic wall** (ghost `(rho, -u, -v, -w, p)`).
Ghosts sit at the mirror image of the cell centre **in the plane of the face** (not the face centre); across a symmetry plane the ghost carries the reflected gradient
`R G R` (otherwise an `O(dtheta)` shear times the wedge weight `1/(r dtheta)` is an `O(1)` residual error). Face viscosity = mean of the two sides; across a wall the ghost
eddy viscosity is `2 mu_t,w - mu_t`. Parity with the 2D module on the wedge fixture: `viscous3::parity` tests.

### walldist3 (`walldist3.rs`)
Distance of each fluid cell centroid to the nearest no-slip face: exact point-triangle distance (each quad split along diagonal 0-2), BVH (median split, leaves of 4
triangles), once per grid. Oracle `wall_distance3_brute` used by the tests. Independent of grid topology and axis cells.

### turbulence3 (`turbulence3.rs`): Menter SST 2003 with the automatic all-y+ wall treatment
- SST 2003 as a list of scalar equations `[k, omega]`, line-implicit (three tridiagonal factors), `mu_t = min(rho a1 k / max(a1 omega, S F2), 1e5/re_c)`, production
  limited `P <= 10 beta* rho k omega`, update floors `phi_new >= 0.1 phi`, `k >= 1e-3 k_inf`, `omega >= 1e-3 omega_inf`. Ghosts that follow their cell
  (`ScalarBc::Cell`) add nothing to the diagonal (a difference from the 2D module; reason in the module header, item 1).
- **Smooth wall state** (`wall_state_smooth`): the friction velocity inverts the law of the wall `U+ = Reichardt(y+) (1 + b)` (`law_uplus`; `b` a 4 % Gaussian correction at
  `y+ ~ 6` in `ln y+`, so the law is the resolved SST profile's own) by Newton; `omega_w = hypot(omega_vis, omega_log)` with `omega_vis = 6 nu/(beta1 d^2) * boost`
  (`OMEGA_BOOST` 10, rolled off by `y+ ~ 15`); `k_w = u_tau^2/sqrt(beta*) * y+^4/(y+^4 + 12^4)`; `mu_t,w = clamp(u_tau^2 d/U - nu, 0, NU_T_WALL_MAX nu)` so the discrete
  shear `(mu+mu_t,w)U/d` is the law's `rho u_tau^2`. The cap of `k` in a wall-adjacent cell has an eighth-power weight (`Y_PIN` 4) so a resolved cell (`y+ < 2.5`) is untouched.
  The Esch-Menter blend it replaced was 1.2-6.9 % high in `u_tau^2` in the buffer layer (`wall_state_table` test prints it).
- **Per-class rule** (`RansOpts3::law_i_walls`, default off; `turbulence3::wall_speed`): the automatic treatment applies to the **streamwise walls** (no-slip faces of the `j`
  and `k` families: body surface, fin sides). The `i` family (base plane, blunt fin edges / fin LE-TE faces) gets the plain resolved-wall state, because the log law has no
  meaning in recirculation or at a stagnation face. Evidence: with the law on the base wall the G-V3 base drag was -3.2 % against the 2D solver, plain +0.35 % (source
  comment at `turbulence3.rs:340`) **[not re-measured]**. Consequence seen in session: `i`-face fin edges report `cf ~ 1e-5` by construction (edge friction 0.00002 of the
  total) **[measured]** (`viscous3_fins` `WALL_REPORT`, Draft Mach 1.3).
- The y+ the application reports is that of `j`/`k` faces only; fin tips are excluded (`run3::y_plus_max`; GPU `y_plus_report` since `47c9ccec77`).

### Rough walls (Aupoix 2015 and the rough law beyond 4 ks)
`wall_state(rho, nu, speed, d, ks)` (commit `a0b6040aa5`; GPU mirror `47c9ccec77`, cell-by-cell vs CPU in `s5a_turb`: f32 2.2e-6, df32 1.3e-13 per gpu3d):
1. Aupoix verbatim (`aupoix_wall`: `k_w+`, `omega_w` capped by Menter's smooth `60 nu/(beta1 d^2)`, `u_tau^2 = (nu + nu_t,w)U/d`) for `d <= ks`.
2. Runaway guard: an Aupoix state with `mu_t,w/mu` from 300 (full at 1000) is smoothstep-mixed to the smooth state.
3. Rough law of the wall for `d/ks` from 1 (full from 4; smoothstep in `ln(d/ks)`): `U = u_tau (U+_law(d u_tau/nu) - ln(1 + 0.3 ks+)/kappa)`, Newton 16 iterations
   (`.max(0.5u).min(speed)`; gpu3d fixed a panic of `clamp` at `Re_d < 1`), then the smooth `k_w, omega_w, mu_t,w` at that `u_tau`.
**Why**: Aupoix is a wall-resolved BC. Where the first cell is far above the roughness (`d/ks` 130-260: the first cell above a fin tip is `d = 0.10-0.14` calibre) the map
`u_tau^2 -> (nu + nu_t,w)U/d` has no fixed point (`mu_t,w = k_w/omega_w ~ u_tau^2 d^2`) once `Re_d` exceeds ~`60/(beta1 k_w+)` ~ 240-340: it runs to the cap `1e5 nu`
(`cf` 0.72-0.80, `y+` 1.2-1.9e5) or, with `ks+ < ~3`, stays laminar (`cf` 1e-5). That was the app's fin friction 0.036/0.055/0.099/0.093 and `y+max` 1e5.
**[measured]** CPU Draft example sector, Mach 1.3, ks body = fin = 60 um (7.87e-4 d), 800 its, cfl 10 (`viscous3_fins`, `CASE=rocket KS_BODY=7.87e-4 KS_FIN=7.87e-4 MACH=1.3
MAX_ITER=800 CFL_MAX=10 WALL_REPORT=6`): fin friction 0.04381 (sides 0.03941 + tips 0.00438) before, 0.04011 (sides 0.03957 + tips 0.00052) after; smooth 0.02374; sides 1.69x
smooth (rough plate at this chord 1.6 local Mills-Hang to 2.0 Schlichting mean). Mach 1.0/1.1/1.3 fin 0.0452/0.0422/0.0401; ks 150 um 0.0471. GPU Coarse (job
`phys3d-gate`-era `rough-coarse`, gpu3d, cfl 20 sub 2): fin 0.04446/0.04106/0.03819/0.03711 at Mach 1.0/1.1/1.3/1.7. Body unchanged within 0.4 %.
Wall-state printout of a fin-side cell (CPU Draft M1.3, 500 its): median face `(99,30,1)`: `d/ks` 0.136, `y+` 7.10, `ks+` 52.4, `cf` 0.00414, `k_w/u_tau^2` 1.077, `omega_w nu/u_tau^2` 2.22, `mu_t,w/mu` 0.485;
median tip face `(92,52,0)`: `d` 0.103, `y+` 9438, `ks+` 72.1, `cf` 0.00382, `mu_t,w/mu` 464.
Tests: `turbulence3::tests::rough_wall_state_is_aupoix_verbatim_inside_the_roughness_layer`, `..::a_rough_cell_far_above_the_roughness_takes_the_rough_law_of_the_wall`.

### lowmach5 (`lowmach5.rs`)
Weiss-Smith preconditioning in the 5x5 block of `implicit3` (`P = I + (1/U_r^2 - 1/c^2) a g^T`, `U_r` clamped to `[LOW_MACH_UREF_EPS c, c]` and a Turkel cutoff on
`|u_inf|`), in the time term, between the factors and in the defect; right-hand side untouched so the converged state is that of the plain scheme. Also the closed-form
eigensystem `eig_pc5` for the diagonalised solve (`LineSolve3::DiagonalCorrected`; with preconditioning it stalls on the CPU and is not ported for viscous runs: guard in
`gpu/body3.rs`). 2D evidence quoted in the header: Mach 0.1 14100 -> 1200 iterations **[not re-measured]**.

### Wedge specifics (`Symmetry::Wedge`, single-cell sector of angle `dtheta`)
- **k faces carry pressure only in the inviscid flux**, plus the viscous flux of the mirror ghost (the hoop terms). The pressure of the k faces is the axisymmetric
  radial-momentum source; its Jacobian `wedge_source_block` joins the diagonal of the **j factor** (as 2D `implicit::cell_blocks`).
- **No k line, no k term in `compute_dt`**: two factors (block and diagonal solves), `P` applied between them (a k factor of `I` would be a second `P^-1`).
- **Swirl projection**: the mirror ghost damps a swirl perturbation at `2 mu/(rho (r dtheta/2)^2)`, an explicit rate no CFL survives (swirl of round-off size grew by 500 per
  iteration at the axis cells); so the azimuthal momentum of the residual is projected out (`rans.rs` step 4, `th_m = theta_span/2`).
- **Axis cells** (`Grid3::axis_cells`, degenerate faces on their j side): ghosts from `axis3.rs` (`F(-P) = G^T F(G(-P))`, table in the module header; wedge `-I`, half domain negate `v`);
  axis cells are left out of the residual norm; their time step is capped `WEDGE_AXIS_DT r_c / c` (`solver3.rs:34`; commit `c5aa96e65d`).
- **sub_iters rule** (`Numerics3::from_2d`, `case3.rs`): the wedge takes `sub_iters` 1 only below Mach 1; at level 1 `solver3d`'s G-V4/G-J went non-finite with 1 at Arcas
  fins-off Mach 2.3 (iteration 250) and TP-2005 Mach 1.65, so 3 elsewhere (DESIGN3D "Correction to item 31"). Finned grids use 2 on the GPU table runs and in the CPU rocket harness (its header prints sub_iters 2);
  one is not stable on the finned sector (small finned sector, Mach 0.5, cfl 50: non-finite at 370).

### The jet (`solver3/jet.rs`, DESIGN3D item 20)
A `Part::Jet` cell is solid but not a wall (wall flag cleared, state = jet state, also in the padded array). Inviscid exit stencil `(jet, jet, w[i], w[i+1])`; implicit `i` factor
has no sub-diagonal or boundary Jacobian behind the exit; the exit face is not a no-slip wall (ghost = jet state at the mirror of the fluid cell, `mu(T_jet)`, `rho_jet k/omega`,
gradient of the fluid cell stored every iteration by `jet_viscous_fix`); jet faces are not in the wall-distance set; `k, omega` ghost = the jet's. Gate G-J
(`gate_j_tp2005_jet_cases`): **5 TP-2005 cases still not re-run** (open item).

## 2. Validation numbers (test, command)

CPU commands: `cargo test --release -p bowshock-cfd --lib -- <name>` (`--ignored --nocapture` for ignored ones). Gates through the engine:
`cargo test --release -p bowshock-core --test wedge_gates -- --ignored --nocapture --test-threads=1 <gate>`; fin / lift gates: `BOWSHOCK_WEDGE_BACKEND=gpu BOWSHOCK_FIN_TIER=lift|coarse
cargo test --release -p bowshock-core --test fin_lift_gates -- --ignored --nocapture --exact <gate>` (GPU through `tools/gpuq/gpuq submit`; `tools/release_gates.sh <group>` lists the groups).

| gate | test | number | source |
|---|---|---|---|
| G-V1 laminar plate | `plate3::tests::g1_laminar_plate_matches_blasius` (and `..boundary_wall..`) | `cf` vs Blasius +0.05 % (x 0.4955), +1.32 % (x 0.8909), 1250 its; band 3 % | **[measured]** this session |
| G-V2 y+ sweep | `plate3::tests::g2_yplus_sweep` (ignored) | y+ 0.499 / 0.989 / 5.106 / 14.958 / 29.979: `cf` 2.5979e-3 / 2.5513e-3 / 2.7214e-3 / 2.5907e-3 / 2.5971e-3; vs y+ 0.5: 0 / -1.79 / +4.76 / -0.28 / -0.03 %; vs correlation -4.95 / -6.65 / -0.43 / -5.21 / -4.98 %; band 5 % of the y+ 0.5 value (worst +4.76 % at y+ 5) | **[measured]** this session, full sweep (650/423/338/348/439 its) |
| G-V2 turbulent plate | `plate3::tests::g2_turbulent_plate_matches_the_correlation` (`..boundary_wall..`) | gate `cf` within 5 %, `y+ < 2`, `mu_t,w` max > 50 | passes in nextest (117 passed, cfd crate, `a0b6040aa5`) |
| G2r rough plate | `plate3::tests::g2r_rough_plate_matches_mills_hang` (ignored) | `cf` 5.436e-3 vs Mills-Hang 5.398e-3: **+0.72 %** (y+ 0.516, 297 its) | **[measured]** after the rough-law fix |
| G2r first-cell sweep, Mach 0.2 | `plate3::tests::g2r_yplus_sweep` (ignored; `ROUGH_SWEEP_MACH`, `ROUGH_SWEEP_YP`) | first-cell y+ 0.72 / 7.1 / 20.8 / 44.2: vs Mills-Hang +0.70 / -1.86 / -7.38 / +4.88 %; `ks+` ~250 | **[measured]** (`/tmp/hunt/plate_rough_m0.2.txt`) |
| G-V3 | `wedge_gates::gate_v3_g3_ogive_cylinder_cpu_and_gpu` | CPU wedge, 2150 its: CD 0.209153 vs 0.213002 (-1.81 %, 2 %), friction +0.06 % (3 %), base -3.17 % (4 %), y+max 0.649 | **[not re-measured]** (DESIGN3D item 27); later re-verification item 36: CD 0.213278 (+0.13 %) |
| G-V4 example table, Arcas fins off | `gate_v4_example_table`, `gate_v4_arcas_fins_off` | all ten example points PASS (2 %, 3 % at Mach 0.9-1.1) at item 27 | **[not re-measured]** |
| G-J TP-2005 | `gate_j_tp2005_jet_cases` | not complete: 5 cases outstanding | open |
| G-F1 fin friction | `fin_lift_gates::gate_f1_fin_friction_matches_the_flat_plate` | Mach 0.5, Coarse (y+ 15-25) 0.9194 of the reference, Fine (y+ ~1, 978 k cells) 0.9278; Mach 2 band [0.92, 1.12] (KNOWN item 41: 0.92 is the reference's LE/root layer, not the wall treatment) | gpu3d job, DESIGN3D item 41 **[not re-measured by me]** |
| fin friction, rough | `viscous3_fins` `CASE=rocket` (see Rough walls) | tables above | **[measured]** |
| G-L1 (CN_alpha 13 %, x_cp 0.25 cal), sourced section | `fin_lift_gates::gate_l1_anf_cn_alpha_and_cp`, `BOWSHOCK_FIN_TIER=lift`, GPU f32, both roll phases, controller on | **30 of 32 PASS**; all x_cp PASS; the two FAILs are WT Mach 0.79 CN_alpha 18.43 vs 16.27 (+13.3 %, in-plane +13.8 %). Others (between): 0.69 +11.7 (in-plane +12.2); 1.728 +8.2; 1.99 +6.7; 2.48 +3.2; FF 1.8 +9.7; 2.351 -3.8; 2.665 +3.9 % | **[measured]** gpuq `1-20261117-190526-11075` (0.69, 0.79, 1.728, 1.8; 2017 s remote 3080) and `1-20261117-194712-941` (1.99, 2.48, 2.351, 2.665; 962 s); solver3d section `abe24a6`; tree `15cb5d1cd9` + the harness |
| G-F5 CD0 (8 %), same setup | `fin_lift_gates::gate_f5_anf_cd0` | subsonic vs 0.46: M0.5 0.4963 (+7.9 % PASS) / 0.5027 (+9.3 % FAIL); M0.7 0.5140 / 0.5132 (+11.7 / +11.6 % FAIL). Supersonic KNOWN (item 38): 1.832 0.5300/0.5285 vs 0.585; 2.375 0.4297/0.4280 vs 0.484; 2.718 0.3845/0.3830 vs 0.435; 3.147 0.3387/0.3368 vs 0.373 (-9.2 to -11.9 %) | **[measured]** gpuq `0-20261117-182910-1954` (2396 s, remote 3080) |
| incidence (CPU, half domain, ANF short, Draft/Coarse sector) | `viscous3_fins` `fin_half_incidence_series`, `incidence_rans` | G-F1 sector Mach 2 converged CA 0.34915, Mach 0.5 0.24584; sector settling time ~ 1/CFL, iterations x cfl_max ~ 107 000 at 20/30/50 | **[not re-measured]** (DESIGN3D "Update" sections near item 38 / "viscous half domain at incidence") |
| G-S sector vs half domain | `gate_s_sector_vs_half_on_the_stack_grids` | within 1 % band, item 39 | **[not re-measured]** |
| rough controller table, Draft/Coarse | `finned_table` (`TABLE-RUN`, `TABLE-ROLLBACKS` lines) | -26 to -58 % wall vs fixed cfl 10, CA within 0.5 % | **[measured]** gpuq `0-20261117-154415-25145` (Draft), `0-20261117-154416-28280` (Coarse); DESIGN3D "The CFL controller as a rule, measured" |

Previous G-L1 on the **wrong bevel** (assumed section, 8-column taper, cfl 10) gave CN_alpha +1 to +12 %; G-F5 supersonic was +21 to +27 % there (item 38). Both are superseded by the rows above.

## 3. The CFL limit of the three-factor step

Facts (CPU f64 unless stated; sources in DESIGN3D titles "The example rocket's finned sector...", "What removes the CFL limit...", "The late blow-ups..."):
- The plain approximate factorisation (3 block factors, `sub_iters` 2-3 defect passes) is non-finite after 190-230 iterations at `cfl_max` 50 on the Draft example sector, finned **and finless** (finless Mach 0.5: 196) **[measured]**, so the limit is not in the fin stacks.
- Defect `|b - A du|/|b|` after the factored solve (Draft M0.5): 0.03 at effective CFL 2.7, 0.18 at 7.4, 1.1 at 18, 1.4 at 20; the defect-correction passes diverge from CFL ~25-40 **[measured]**.
- Rule shipped earlier: `VISCOUS_3D_CFL_MAX` 10 (commit `2fafa6353b`); now only the `Case3::without_cfl_control` path (gpu3d `6d009f5`).
- G-F1 sector (Mach 2): settling time ∝ 1/CFL; base drag settles last, algebraically (~`t^-1.4`); converged CA 0.34915; stop-rule bias at cfl 10 +1.48 % (application rule) / +0.82 % (scaled window), residual ~0.8-0.9 % is the drift-rule bias on an algebraic tail.

**Things that do not move it** (each tried, each measured; DESIGN3D has the numbers): MinDirection `dt` (non-finite at iteration 19); scalar Rusanov implicit dissipation (holds at 50 but converges slowly); implicit entropy fix; `implicit_bc` off; `sub_iters` 6; first-order length 400; slow CFL ramp as a rule; the trailing-edge taper and fin geometry as the cause of the CFL limit (the taper is a different cause, section 4); a damped defect correction (weight 0.7: still non-finite ~200 at cfl 50); more passes (diverge faster); `k` lines in fin columns only (finless dies at the same CFL); a reconstruction floor or first order (below); restarting the ramp at the 1st->2nd-order switch; f32 vs df32 vs f64 (shifts the iteration, does not remove it at Mach 3). Laminar equations: also fail.

**DDADI study** (commit `5948e39481`, `ImplicitParams::ddadi: Option<f64>`, default `None`, no run changes): full diagonal `D = P + dt sum Dg_d` in every factor, `D` between factors, correction x omega. Result: **negative**. omega 1 is stable at CFL 100 on the example sector (M0.5 to 1800 its; M0.9, 3.0 to 800), non-finite at 200 (650 its), reaches CA within 0.5 % of 0.4280 in ~1050 its (cfl 100) / ~2200 (cfl 50) against ~5300 (extrapolated) for the plain factors at 20, i.e. the same `sum(CFL x its)` ~ 105-110 k, so **no change to the `t^-1.4` tail**. But on the viscous block (`solver3::rans::tests::ddadi_cfl_scan`, `relaxed_ddadi_converges_where_the_plain_factors_fail`) omega 1 sits on a period-2 limit cycle (residual 1.2e-2 / 7.5e-3 constant, state 3e-3 to 0.12 off) from CFL 3-6; the plain factors with `sub_iters` 10 do the same: an accurate solve of the first-order LHS against the limited second-order residual has a period-2 mode that the plain factors' factorisation error was damping. omega 0.7 converges at CFL 6-50, 0.6 at 6-300, 0.5 at 3-300, but at CFL 100 on the sector CA at 900 its is 0.43102 (1) / 0.44880 (0.7) / 0.45959 (0.6) / 0.47410 (0.5) vs 0.46182 plain cfl 20: at best 1.7x fewer iterations. GPU cost not measured (estimate +10-15 %/step). **Decision: not ported.** Proper cure would be a LHS that matches the residual (limiter-frozen second-order LHS): not prototyped.

**Controller policy** (`run3.rs`, `CflControl`; commits `c764e0700e`, gpu3d `6d009f5`): snapshot every sample chunk; on a failure (residual non-finite or > `ROLLBACK_RESIDUAL` 1e5, non-finite force monitor) restore the last snapshot (the older one if the failure is within `RECENT_RESTORE` 10 iterations of a restore) and set `cfl_max x CFL_BACKOFF` (0.7); `MAX_ROLLBACKS` 12; **hard floor `CFL_FLOOR` 5**: it is `active()` only while `cap x 0.7 >= 5` (from 50: 35, 24.5, 17.15, 12.0, 8.4, 5.88 then stop; from 20: 14, 9.8, 6.86 then stop), a point that fails there fails. The cap only goes down. Stop-rule windows scale by `max(1, cfl_ref/CFL in force)` (`cfl_ref` 20 viscous, 10 Euler). A run without a failure is bit-identical to the old runner. Events are in `PointResult3.cfl_events` (`wasted_iterations()`, `final_cfl_cap()`); the gate lines do not print them. Start caps: engine 50 below Mach 1, 20 above.

**Late Mach 3 / Mach 2.3 failures** **[measured]**: Coarse Mach 3.0 cfl 20, no controller: GPU f32 non-finite at 267, CPU f64 at 297 (`finned_table --cpu --no-control --tier coarse --mach 3.0 --cfl-max 20 --max-iter 400`); Coarse Mach 2.3 f32 at 5587 (gpu3d; not reproduced in f64: 80 min). Draft cfl 20 (gpuq `5-20261117-185159-28836`): Mach 3.0 f32 1277, df32 2433, CPU f64 1900; Mach 2.3 f32 955 but df32 holds 6000 (MeanStationary, CD 0.49717) and f64 holds 2000. cfl 30: all precisions fail in 190-350. **Not** the reconstruction: a floor `f x` cell value in `reconstruct3` (`f` 0.25: 0 hits, bit-identical; 0.7: 12.4 M faces, non-finite 340; 0.9: 400 on CPU; GPU kernel `f` 0.3/0.6: 955/955/956 and 1277/1277/1278; job `1-20261117-194828-19589`); not the 1st->2nd-order switch (first-order 400: same failure; `RAMP_AT_SWITCH=1` same). f64 blow-up cells (Draft M3.0 cfl 20, it 1900): fin-root cells on the body surface at r 0.501 next to `Fin(0)` (volumes 1.4e-9, 4.1e-9); gpu3d's `late-worst` lowest p/p_inf cells (0.03 at Mach 3) are in the first wake columns behind the base, `i` 151-156. **[INFERENCE]** the instability is the plain factors' CFL limit at the smallest, most stretched fin-root cells, lower on Coarse than on Draft; the measurements exclude reconstruction and (Mach 3) precision, they do not isolate the operator. Remedy: the controller (caps settle at 14-24).

## 4. Trailing-edge taper root cause (DESIGN3D "Update: the cause is the mesh behind the flush trailing edge")
The Lift-tier half domain with fins (692 k cells, `Re_c` 7.0e5) was non-finite at ~180 iterations at cfl 20. Cause: the layer cell behind the trailing edge of a flush fin is as wide as the fin (0.074 d) between the first cells of the fin stack (7e-4 d at y+ 60) and narrows to the stack width over `TE_TAPER_COLUMNS` columns; its side faces tilt up to 23 degrees (cosine 0.918 in `k`, 0.934 in `i`) and its volume falls up to 10x per column; the first cells to fail are right behind it at the body radius (`i` 108-110, `j` 28-30). Taper over 2 columns: non-finite at ~135; 4 (old): 150-182; 5, 6, 8, 16 lengthen the life/stabilise (values in the DESIGN3D paragraph); `TE_TAPER_COLUMNS` = 8 (commit `25950ada8f`). `tests/viscous3_fins.rs::corner_geometry` prints the cells. Taper-sensitivity of the Euler tier is `solver3d`'s paragraph in DESIGN3D. This is a mesh-quality failure, separate from the CFL limit of section 3 (the example rocket sector with 8 columns still has the CFL limit).

## 5. Open physics questions, priority order
1. **Implicit scheme.** The three-factor step has a CFL limit (section 3) and its factorisation error is doing useful damping; the controller only manages it. A left-hand side that matches the limited second-order residual (frozen-limiter second-order LHS) with an accurate solve is the untested way to a larger CFL; DDADI is not.
2. **Compressibility correction for supersonic base and fin drag.** G-F5 supersonic reads 9-12 % below the table on the sourced section (KNOWN, item 38) and G-F4 Arcas fin increment is off (item 37); cause open: table provenance, the 2 degree cant of the tested fins, blunt trailing edge, or SST without a compressibility (Sarkar / Wilcox-type) correction in the supersonic base shear layer **[INFERENCE: no run isolates it]**.
3. **Subsonic CN_alpha +12-14 %** (G-L1 Mach 0.69 +11.7/+12.2 %, 0.79 +13.3/+13.8 %) and subsonic G-F5 CD0 +8 to +12 % (base 0.156, fins 0.214 at Mach 0.5). Not tied to a cause **[INFERENCE: base-wake unsteadiness / mean-window bias are candidates]**; the CPU Draft sector settles near CA 0.428 at Mach 0.5 whereas the GPU table window mean is 0.4528 for the same sector (5.7 %, unexplained, config differences suspected).
4. **Transition.** Everything is fully turbulent (SST from the nose, free-stream `k_inf`); no transition model. Affects the nose and fin friction at the Reynolds numbers of the app (`Re_c` 1.7e6).
5. **Second CFL limit near the nose-tube junction at Mach 0.8** (the earlier half-domain / finless Mach 0.8 failures, DESIGN3D "The viscous half domain at incidence..." "What it needs"): this is listed as open in the project notes; I have not re-measured it in this phase and have no new data on it.

## 6. Reproduction notes
- Harness `crates/bowshock-core/tests/viscous3_fins.rs` (`fin_half_incidence_series`, ignored): env `CASE=rocket MACH ALPHA MAX_ITER CFL_MAX EVERY FIRST RAMP_AT_SWITCH DDADI DDADI_RELAX SUB GROWTH PRECOND ROE EPS_FIX IMPLICIT_BC DTMODE KS_BODY KS_FIN WALL_REPORT WORST ROLLBACK FINLESS`.
- `finned_table --tier coarse|draft|draft-safe --mach M [--sub N] [--cfl-max C] [--cfl-ref R] [--first-order N] [--no-control] [--precision f32|df32] [--cpu] [--worst] [--ks-body --ks-fin]`.
- Binaries and logs of this session were in `/tmp/hunt` and `/tmp` (not preserved).
