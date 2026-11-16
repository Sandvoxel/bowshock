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
| Supersonic fin pressure `K1, K2, K3` (second-order shock expansion) | Niskanen 2009 eqs. 3.41-3.49 (Pitts, Nielsen and Kaattari, NACA Rep. 1307) |
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
| Boat-tail pressure drag | Niskanen 2009 eq. 3.88 |
| Stagnation pressure, `0.85 q_stag/q` | Niskanen 2009 eqs. B.1-B.2 (Hoerner pp. 15-2, 16-3) |
| Base drag, `0.12 + 0.13 M^2` / `0.25/M` | Niskanen 2009 eq. 3.94 (Fleeman p. 23); also `bowshock-reference::base_drag` |
| Fin leading-edge drag (stagnation, rounded-edge fit, `cos^2` of sweep), trailing-edge base drag | Niskanen 2009 eqs. 3.89-3.93 (Hoerner) |
| Fin wave drag above Mach 1 (double wedge) | Ackeret linear theory (`bowshock-core/src/fins.rs`) |
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
| Fin flutter velocity | Martin, NACA TN 4197 (1958) |
| Mass, CG and moments of inertia of the parts (`bowshock-model/src/mass.rs`) | Niskanen 2009 section 4.2.3; standard solid-body formulas |
| Nose profile families (`bowshock-model/src/geometry.rs`) | Crowell, *The Descriptive Geometry of Nose Cones*, 1996; Niskanen 2009 appendix A |
