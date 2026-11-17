# Aerodynamic models and their published sources

Each model in `bowshock-sim` (and the fin drag of `bowshock-core`), the paper it follows and the equation. "Niskanen 2009" is
S. Niskanen, *Development of an Open Source model rocket simulation software*, M.Sc. thesis, Helsinki University of Technology,
2009 (equation and section numbers are those of the thesis). "Barrowman 1967" is J. S. Barrowman, *The Practical Calculation of
the Aerodynamic Characteristics of Slender Finned Vehicles*, M.S. thesis, Catholic University of America, 1967 (with J. and
J. Barrowman, *The Theoretical Prediction of the Center of Pressure*, NARAM-8, 1966). Items marked **own estimate** have no
published source; they are Bowshock's engineering choices and are documented as such in the code.

Reference values from other programs are kept apart, in [openrocket-comparison.md](openrocket-comparison.md).

## Normal force and centre of pressure (`aero.rs`)

| Model | Source |
|---|---|
| Normal force slope of nose, shoulder and boat-tail, `CN_alpha = 2 (A_aft - A_fore)/A_ref` | Barrowman 1967; Niskanen 2009 eq. 3.19 |
| Moment and CP of a body component | Niskanen 2009 eqs. 3.20-3.28 (after Barrowman 1967) |
| Body lift at angle of attack, `CN = K sin^2(alpha)` with `K = 1.1 A_plan/A_ref`, acting at the planform centroid | Niskanen 2009 eqs. 3.26-3.27 (after Galejs) |
| Combination of components, total CP | Niskanen 2009 eq. 3.29 |
| Fin mean aerodynamic chord and its position | Niskanen 2009 eqs. 3.30-3.34 |
| Fin CP: quarter chord subsonic; `(AR beta - 0.67)/(2 AR beta - 1)` supersonic; quintic transonic blend | Niskanen 2009 eqs. 3.35-3.36 (supersonic form after Fleeman, *Tactical Missile Design*) |
| Low-aspect-ratio bridge of the fin CP (cubic Hermite from `AR beta = 0.84` to 1, shape-preserving quintic) | **own estimate** (continuity of the published rule) |
| Fin normal force slope, `CN_alpha` of one fin (Diederich planform form, compressibility `beta`) | Niskanen 2009 eqs. 3.37-3.40 (Diederich, NACA TN 2335 / Rep. 1091) |
| Supersonic fin normal force: Busemann strip pressure `Cp = K1 eta + K2 eta^2 + K3 eta^3` per surface, `K1 = 2/beta`, `K3 = ((g+1) M^8 + (2 g^2 - 7 g - 5) M^6 + 10 (g+1) M^4 - 12 M^2 + 8)/(6 beta^7)`; flat plate (windward `eta = alpha`, leeward `-alpha`) `CN_alpha = A/A_ref * 2 (K1 + K3 alpha^2)`, the `K2` terms cancelling | Barrowman 1967 eq. 3-7 and App. A eq. 2a-2c (NASA/TM-2001-209983; Busemann third-order theory); Niskanen 2009 eqs. 3.41-3.49 print the same `K1`, `K2` but drop the `-12 M^2` of `K3` and use one surface's `Cp` (`A (K1 + K2 alpha + K3 alpha^2)`), so Bowshock does **not** follow eq. 3.44/3.49 there. Checked against the exact oblique-shock / Prandtl-Meyer flat plate in `aero/tests.rs`; Barrowman's `K2` is half the Busemann coefficient `((g+1) M^4 - 4 beta^2)/(2 beta^4)` (own check) and is unused. Not included: the bow-shock `K*` term and Barrowman's fin-tip Mach-cone correction (finite-span loss), so Bowshock fins are 2-D strips; the coefficients are frozen above Mach 4.9 (existing clamp, no source) |
| Fin roll angle to the crossflow, `sin^2` scaling | Niskanen 2009 eqs. 3.50-3.53 |
| Fin-fin interference of 5 or more fins | Niskanen 2009 eq. 3.54 (after Barrowman 1967) |
| Body-fin interference `K_fb = 1 + r/(s + r)` | Niskanen 2009 eqs. 3.55-3.56 (Barrowman 1967) |
| Fin stall limit, 20 degrees | Niskanen 2009 section 3.2.2 |

## Drag (`aero.rs`, `bowshock-core/src/fins.rs`)

| Model | Source |
|---|---|
| Drag coefficient on component reference area scaled to the rocket's | Niskanen 2009 eq. 3.75 |
| Flat-plate skin friction: laminar `1.328/sqrt(Re)`, turbulent `1/(1.5 ln Re - 5.6)^2`, transition at `Re_crit` | Schlichting, *Boundary-Layer Theory*; Niskanen 2009 eqs. 3.77-3.81 |
| Roughness-limited friction `0.032 (eps/L)^0.2`, roughness heights of finishes | Schlichting; Hoerner, *Fluid-Dynamic Drag*, p. 5-3; Niskanen 2009 eq. 3.80, table 3.2 |
| Compressibility corrections of the friction coefficient | Hoerner ch. 2; Niskanen 2009 eqs. 3.82-3.84 |
| Friction drag of the body and fins, thickness factor `1 + 2 t/c` | Hoerner ch. 2; Niskanen 2009 eq. 3.85 |
| Nose cone pressure drag: low-speed `0.8 sin^2(phi)`, tabulated transonic rise for the profile families | Niskanen 2009 eqs. 3.86-3.87, appendix B; NASA TR R-100 (Stoney 1961) p. 16 |
| Supersonic cone and ogive drag, cubic through Mach 1.0 to 1.3 | Niskanen 2009 appendix B.2-B.5 (eqs. B.3-B.6) |
| Extrapolation of the tables to other fineness ratios, subsonic power-law ramp | Niskanen 2009 section 3.4.3 and appendix B.3 |
| Pressure drag of a spherically blunted nose on the nose base area, `CD = CD_sharp (1 - (r_n/R)^2) + CD_cap (r_n/R)^2`: `CD_sharp` the drag of the sharp profile the sphere sits on (its length and fineness), `CD_cap` the drag of a hemispherical cap on its own frontal area `pi r_n^2`; skin friction uses the blunted wetted area, normal force `2 (A_aft - A_fore)/A_ref` and the CP `L - V/A_base` the blunted volume | **own estimate** (area-weighted; no published correlation for a blunt rocket-nose tip was found; to be checked against blunt-body data such as the HB-1 model of AEDC-TDR-64-137) |
| Cap drag above Mach 1, `CD_cap = Cp_max / 2` with the stagnation pressure coefficient behind a normal shock `Cp_max = (2/(gamma M^2))(p02/p1 - 1)` and the Rayleigh pitot ratio `p02/p1 = [(gamma+1)^2 M^2/(4 gamma M^2 - 2 (gamma-1))]^(gamma/(gamma-1)) (1 - gamma + 2 gamma M^2)/(gamma+1)`: modified Newtonian `Cp = Cp_max cos^2(phi)` on the hemisphere, whose axial force `2 Cp_max integral(cos^3 phi sin phi d phi)` over `0..pi/2` is `Cp_max/2` (1.839/2 as `M` tends to infinity); below Mach 1 the table of a hemispherical (elliptical, length = radius) nose, joined to the Newtonian value over Mach 0.95 to 1 by linear interpolation as the stagnation drag is sampled every 0.05 Mach | modified Newtonian theory and the Rayleigh pitot formula: Anderson, *Hypersonic and High-Temperature Gas Dynamics*, 2nd ed., eqs. 3.15-3.19 (NACA Rep. 1135 eq. 100); the use as a cap drag, the area weighting and the join are an **own estimate** |
| Boat-tail pressure drag | Niskanen 2009 eq. 3.88 |
| Stagnation pressure, `0.85 q_stag/q` | Niskanen 2009 eqs. B.1-B.2 (Hoerner pp. 15-2, 16-3) |
| Base drag, `0.12 + 0.13 M^2` / `0.25/M` | Niskanen 2009 eq. 3.94 (Fleeman p. 23); also `bowshock-reference::base_drag` |
| Fin leading-edge drag (stagnation, rounded-edge fit, `cos^2` of sweep), trailing-edge base drag | Niskanen 2009 eqs. 3.89-3.93 (Hoerner) |
| Fin wave drag above Mach 1, double wedge: `cd_w = 4 tau^2 / beta` on the fin planform area, `tau = t/c`, `beta = sqrt(M^2 - 1)` (the slope integral below, with `zeta' = 2` over each half) | Ackeret linear theory: Ackeret 1925, *Über Luftkräfte bei sehr grossen Geschwindigkeiten insbesondere bei ebenen Strömungen*, Helv. Phys. Acta 1; Liepmann and Roshko, *Elements of Gasdynamics* (1957), thin-airfoil theory; `cd_w = (2/beta)(1/c) integral[(dz_u/dx)^2 + (dz_l/dx)^2] dx` (`bowshock-reference/src/fin_wave.rs`, used by `bowshock-core/src/fins.rs` and `bowshock-sim/src/aero/fin.rs`) |
| Fin wave drag above Mach 1, hexagonal (modified double wedge, wedge of chord fraction `f` at each edge, `0 < f <= 0.5`): `cd_w = 2 tau^2 / (f beta)` (equal to the double wedge at `f = 0.5`) | Ackeret linear theory as above; slope `zeta' = 1/f` over a length `f` at each end |
| Fin wave drag above Mach 1, biconvex (circular arc, parabolic in the thin limit): `cd_w = (16/3) tau^2 / beta` | Ackeret linear theory as above; `zeta = 4 xi (1 - xi)`, `zeta'^2` integrates to `16/3` |
| Fin wave drag of a custom section: `cd_w = S tau^2 / beta` with `S` the integral of the squared slope of the normalised section | Ackeret linear theory as above (`section_slope_integral` in `fins.rs`) |
| Near Mach 1 the `1/beta` of the linear theory is floored at `beta^2 = 0.3225` (Mach 1.15); no wave drag below Mach 1; blunt-edged `Square`/`Rounded` and `Airfoil` fins get no thickness wave drag, sharp sections get no blunt leading-edge or trailing-edge drag | **own estimate** (the floor); sections as modelled in the code |
| Fin plate volume of a section (mass) as the area fraction of `t c`: double wedge 1/2, hexagonal `1 - f`, biconvex 2/3, airfoil 0.685 | plane geometry; NACA four-digit area for the airfoil; rounded 0.9 is an own estimate |
| Launch lug: bore pressure drop, Darcy-Weisbach with Swamee-Jain friction factor | Swamee and Jain 1976, J. Hydraul. Div. ASCE 102(HY5); Niskanen 2009 section 3.4.6 |
| Launch lug frontal-ring factor `0.7 (stag + base)` | **own estimate** (the published rule takes 1.0; see `tube_fins.rs`) |
| Rail button: cylinder drag in the boundary layer, `delta = 0.37 x/Re_x^0.2` | Gowen and Perkins, NACA TN 2960 (1953) fig. 7; Schlichting; Niskanen 2009 section 3.4.6 |
| Axial drag at angle of attack (multiplier 1 to 1.3 at 17 degrees, 0 at 90 degrees) | **own estimate** (shape of Niskanen 2009 section 3.4.7) |

## Tube fins (`tube_fins.rs`)

| Model | Source |
|---|---|
| Ring-airfoil normal force `2 AR'/(1 + AR') pi^2 r_in L` | Ribner 1947, J. Aeronautical Sciences 14(9), eq. 5 |
| CP along the tube chord | the fin rule above, Niskanen 2009 eqs. 3.35-3.36 |
| Outer-surface friction, bore pressure drop, interstice drag | Hoerner; Swamee and Jain 1976; Niskanen 2009 eqs. 3.85, 3.94, B.1-B.2 |
| Annulus factor 0.7 | **own estimate** |

## Flight and analyses (`flight.rs`, `analysis.rs`)

| Model | Source |
|---|---|
| Atmosphere | ISA 1976 |
| 3-DOF translational equations of motion, RK4 (`FlightModel::PointMass`) | Niskanen 2009 section 4.2.4 |
| Rigid-body attitude and rotational equations (`FlightModel::RigidBody`): quaternion kinematics, Euler's equations for an axisymmetric body | Zipfel, *Modeling and Simulation of Aerospace Vehicle Dynamics* (2014); Diebel 2006 |
| Normal force at the CP about the moving CG | Barrowman 1967; Niskanen 2009 ch. 3 |
| Pitch/yaw damping `1/2 rho V A_ref sum CNalpha_i (x_i - x_cg)^2` | strip theory of Barrowman 1967; Mandell, Caporaso & Bengen, *Topics in Advanced Model Rocketry* (1973) |
| Body crossflow damping `1/2 rho 1.1 w abs(w) sum A abs(x - x_cg)^3` | Hoerner, *Fluid-Dynamic Drag* (1965) crossflow drag; Zipfel 2014 strip theory |
| Jet damping `mdot (x_exit - x_cg)^2` | Zipfel 2014 (propulsive damping); Mandell et al. 1973 |
| Roll forcing and damping of canted fins (chord-weighted strip theory); positive cant and positive roll rate are clockwise seen from behind (right-handed about the nose direction), the way a fin's roll angle increases; the Inspector limits cant to 15 degrees, the linear range of the fin lift slope | Barrowman 1967; Niskanen 2009 section 3.2.3 |
| Pitch natural frequency `sqrt(q A_ref CNalpha (x_cp - x_cg) / I)` and the roll-pitch resonance flag (the spin rate crossing it, or coming within 15 % of it, above a 2 s pitch period): a Bowshock check; the axisymmetric model carries no asymmetry, so it flags the risk and does not simulate the growth | Zipfel 2014 (short-period frequency); Nielsen, *Missile Aerodynamics* (1960), roll-pitch coupling |
| Step limit `0.3 / (fastest rotational rate)` for RK4 stability | engineering choice (RK4 real-axis stability limit 2.78) |
| Dropped stage flown on its own after a separation (`flight/branch.rs`): one simulation branch per separated stage, starting from the rocket's state at the separation (position, velocity, attitude, body rate), with the stage's own mass, CG, inertias and motor (burning on if it separates before burnout), its own recovery devices and the same integrator and step rules; no separation impulse and no interaction with the stage that left | the approach of OpenRocket's simulator (a separated stage becomes its own simulation branch from the state at the separation); no published equation. The choices above (no separation impulse, no CFD drag table for the stage) are an **own estimate** |
| Aerodynamics of a dropped stage: the Barrowman build-up of the geometry between the joint and the aft end alone (`Vehicle::dropped_stage`; stations from the joint, reference area of the whole rocket); its open forward face is the first segment's full cross-section treated as a flat forward-facing step, stagnation drag `0.85 q_stag/q` on `pi r^2`, and the aft face keeps its base drag (reduced by the exhaust while the motor burns); skin friction on the stage's own length | the drag and normal-force rows above (Niskanen 2009 eqs. 3.85-3.94, B.1-B.2; Barrowman 1967). Using the flat-plate stagnation value for the open face of a tube or interstage coupler is an **own estimate** (the real face leaks into the airframe, so this errs on the high side of the drag) |
| Attitude of a dropped stage: the rigid-body equations above with the stage's own CP, CG and inertia; stable alone it weathercocks, unstable alone it flips and tumbles under its own normal force, the crossflow drag of the turning body (coefficient 1.1) damping the rotation, and a stage flying tail-first uses the coefficients of the mirror angle `pi - alpha`; the point-mass model has no tumble | Zipfel 2014; Hoerner 1965 (crossflow drag); the mirror-angle rule is an **own estimate** shared with the upper stack's descent |
| Fin flutter velocity | Martin, NACA TN 4197 (1958) |
| Mass, CG and moments of inertia of the parts (`bowshock-model/src/mass.rs`) | Niskanen 2009 section 4.2.3; standard solid-body formulas |
| Nose profile families (`bowshock-model/src/geometry.rs`) | Crowell, *The Descriptive Geometry of Nose Cones*, 1996; Niskanen 2009 appendix A |
| Spherically blunted nose tip (`bowshock-reference/src/blunt_nose.rs`, used by the model, the aerodynamics and the CFD body): a sphere of radius `r_n` replaces the tip of the sharp profile `r(x)`, tangent to it where `r r' = x_c - x_t` and `r^2 + (x_t - x_c)^2 = r_n^2`, i.e. `r sqrt(1 + r'^2) = r_n`; closed form for the cone (`x_c = r_n / sin a`, `x_t = x_c - r_n sin a`), bisection for the other families; the nose's stored length is the length as built, sphere apex to base, the sharp profile is longer by `x_c - r_n`; mass, wetted area, volume, the sim's body segment and the CFD wall integrate or draw that one profile; `r_n` up to 0.9 of the base radius and only where a sphere is tangent (an ellipse or a half-power series has a tip radius of curvature it cannot go below; a power series with `n < 1/2` takes none) | Crowell, *The Descriptive Geometry of Nose Cones*, 1996 (spherically blunted cone and ogive); the tangency conditions are derived in the module docs |
