# Bowshock vs OpenRocket

How do Bowshock's results compare to OpenRocket's on the same rockets? Measured with OpenRocket 23.09 (the app in
`/Applications/OpenRocket.app`, run headless by `tools/openrocket-harness`), the importer `bowshock_model::ork`, and the
`compare_openrocket` example of `bowshock-sim`. Everything here is reproduced by

```sh
tools/compare-openrocket/run.sh [workdir]     # full tables land in <workdir>/comparison.md (a copy: docs/openrocket-comparison-tables.md)
```

## Method

* **Same rocket.** Each `.ork` is loaded by OpenRocket's own loader (reference) and by Bowshock's importer (`crates/bowshock-model/src/ork.rs`,
  also `bowshock import <file.ork> <out.bowshock>`). The user's designs were copied before use and never modified.
* **Same motors.** The harness exports the thrust curve OpenRocket used for every configuration as RASP `.eng` (with propellant and loaded mass);
  Bowshock flies exactly those curves (bundled curves are not used when an exported one exists).
* **Same conditions.** Static aerodynamics: ISA sea level, alpha 0 and 2 degrees, Mach 0.1-2.0. Flights: each stored OpenRocket simulation's
  rail length, rail angle and site altitude, ISA, **no wind** (a wind-free variant is run in OpenRocket, because its turbulence is random), and again
  with the simulation's **mean wind and no gusts** (a third OpenRocket variant, "wind"), for both Bowshock flight models.
  OpenRocket's own 50 ms time step overshoots the end of the rail, so its rail-exit speed is taken from a 2 ms run (the step changes it by 2 %;
  apogee and peak speed do not change).
* **Rockets.** OpenRocket's examples *A simple model rocket* (ogive nose, 1 stage, 3 fins, parachute), *Dual parachute deployment*
  (high power, single stage, fiberglass), *Two-stage rocket*, *Pods--airframes and winglets* (1- and 2-fin sets at several roll angles, a
  free-form 2-fin wing, fins on a pod), *Pods--powered with recovery deployment* (1-, 2- and 4-fin sets on a booster with motor pods) and
  *Tube fin rocket*; the user's *Spaceport Design (FINAL NEEDS REVIEW)* (two stages, 4+4 fins, rail buttons, 3 configurations) and
  *SpaceportSubscale Rev 4* (two stages, 8 configurations); and a variant of the simple rocket with a conical nose, which isolates the
  nose-cone effect described below.
* Differences are Bowshock relative to OpenRocket. "cal" is calibers of the reference diameter.

## Results

Worst case over the whole sweep (Mach 0.1-2.0, alpha 0 and 2 degrees, all stored flights of the rocket's first flown configuration):

| rocket | worst CP Δ (cal) | worst CNα Δ | worst Cd Δ (M ≤ 2) | apogee Δ (all flights) | max v Δ |
|---|---|---|---|---|---|
| A simple model rocket | -0.020 | -1.0 % | +13.7 % (nose, see 1) | +0.2 .. +3.2 % | up to +2.8 % |
| simple rocket, conical nose | -0.019 | -1.0 % | **-2.3 %** | +2.0 .. +3.2 % | up to +3.0 % |
| Dual parachute deployment | -0.022 | -0.7 % | +11.0 % (nose) | -2.5 .. +0.4 % | ≤ 0.5 % |
| Two-stage rocket | -0.053 | -3.8 % (M 1.2, see 2) | +14.7 % (nose) | +1.3 .. +1.4 % | +1.6 % |
| **Spaceport final** (user) | -0.024 | -0.8 % | +11.7 % (nose) | -1.7 .. +0.5 % | ≤ 0.6 % |
| **Spaceport subscale Rev 4** (user) | -0.043 | -1.6 % | +13.2 % (nose) | -1.6 .. 0.0 % | ≤ 0.6 % |
| Pods, airframes and winglets | +0.027 | -1.7 % (transonic) | +10.1 % (nose) | +1.1 .. +1.5 % | ≤ 1.8 % |
| Pods, powered | +39 (OpenRocket CP at M ≥ 0.55, see fins section) | -1.7 % (transonic) | +8.4 % (nose) | not comparable (pod motors) | |
| Tube fin rocket | +4.4 (OpenRocket 23.09 tube-fin CP, see its section) | 0.0 % | -29.5 % (23.09 tube bore drag) | +1.9 % | +3.0 % |

Mass and CG at liftoff (first configuration; OpenRocket | Bowshock):

| rocket | mass (kg) | CG (m from tip) | ΔCG (cal) |
|---|---|---|---|
| simple model rocket | 0.065 \| 0.064 (-1.1 %) | 0.2480 \| 0.2469 | -0.04 |
| dual parachute | 1.613 \| 1.613 (0.0 %) | 0.9629 \| 0.9640 | +0.02 |
| two-stage | 2.574 \| 2.572 (-0.1 %) | 1.3302 \| 1.3307 | +0.00 |
| Spaceport final | 53.408 \| 53.408 (0.0 %) | 3.1438 \| 3.1442 | +0.00 |
| Spaceport subscale Rev 4 | 9.132 \| 9.176 (+0.5 %) | 1.3416 \| 1.3450 | +0.04 |

Static aerodynamics, alpha 0 (the rockets above, first configuration; CP in m from the nose tip):

| rocket | Mach | CP OR | CP BS | CNα OR | CNα BS | Cd OR | Cd BS |
|---|---|---|---|---|---|---|---|
| simple | 0.3 | 0.3198 | 0.3198 | 11.970 | 11.970 | 0.6339 | 0.6368 (+0.4 %) |
| simple | 1.2 | 0.3327 | 0.3323 | 13.875 | 13.740 | 0.8798 | 0.9875 (+12.3 %) |
| simple, conical nose | 1.0 | 0.3316 | 0.3315 | 13.186 | 13.159 | 1.0068 | 1.0069 (0.0 %) |
| simple, conical nose | 2.0 | 0.3136 | 0.3136 | 8.959 | 8.959 | 0.7412 | 0.7418 (+0.1 %) |
| dual parachute | 0.3 | 1.2047 | 1.2047 | 14.711 | 14.711 | 0.8148 | 0.8183 (+0.4 %) |
| dual parachute | 1.2 | 1.2347 | 1.2336 | 16.230 | 16.113 | 0.9030 | 0.9958 (+10.3 %) |
| two-stage | 0.3 | 1.5423 | 1.5423 | 29.064 | 29.064 | 0.8946 | 0.8972 (+0.3 %) |
| two-stage | 1.2 | 1.5824 | 1.5786 | 34.995 | 33.792 | 1.0695 | 1.2240 (+14.4 %) |
| Spaceport final | 0.3 | 3.5186 | 3.5188 | 18.626 | 18.626 | 0.6377 | 0.6401 (+0.4 %) |
| Spaceport final | 1.0 | 3.5736 | 3.5730 | 20.090 | 20.058 | 0.8119 | 0.9073 (+11.7 %) |
| Spaceport final | 2.0 | 3.3577 | 3.3579 | 13.176 | 13.176 | 0.5849 | 0.6326 (+8.2 %) |
| subscale Rev 4 | 0.3 | 1.7074 | 1.7075 | 16.928 | 16.928 | 0.6842 | 0.6867 (+0.4 %) |
| subscale Rev 4 | 1.2 | 1.7372 | 1.7345 | 18.587 | 18.303 | 0.9348 | 1.0521 (+12.6 %) |

Flights without wind (OpenRocket | Bowshock; the point-mass and rigid-body models fly identically without wind, apart from the descent rate
under a parachute; every simulation of every rocket is in `docs/openrocket-comparison-tables.md`):

| rocket / simulation | apogee (m) | max v (m/s) | rail-exit v (m/s) | stability off rail (cal) | flight time (s) |
|---|---|---|---|---|---|
| simple, sim 1 (A8) | 51.1 \| 52.3 (+2.3 %) | 29.33 \| 29.84 (+1.7 %) | 15.27 \| 15.36 (+0.6 %) | 2.99 \| 3.03 (+1.2 %) | 16.0 \| 15.9 (-0.9 %) |
| dual parachute, sim 1 (H669) | 593.5 \| 592.4 (-0.2 %) | 135.5 \| 135.6 (0.0 %) | 52.94 \| 53.31 (+0.7 %) | 4.47 \| 4.43 (-0.8 %) | 61.8 \| 64.8 (+5.0 %) |
| dual parachute, sim 3 (J570) | 2224.9 \| 2214.5 (-0.5 %) | 388.6 \| 390.6 (+0.5 %) | 53.76 \| 54.01 (+0.5 %) | 3.28 \| 3.25 (-0.7 %) | 129.7 \| 130.3 (+0.5 %) |
| two-stage, sim 1 | 666.6 \| 675.3 (+1.3 %) | 156.7 \| 159.2 (+1.6 %) | 19.64 \| 19.79 (+0.7 %) | 2.07 \| 2.17 (+4.8 %) | 62.6 \| 64.2 (+2.6 %) |
| **Spaceport final**, sim 1 (M1939 + O5500) | 9087.9 \| 8960.1 (-1.4 %) | 442.0 \| 442.8 (+0.2 %) | 36.42 \| 36.48 (+0.2 %) | 2.51 \| 2.50 (-0.5 %) | 371.2 \| 368.2 (-0.8 %) |
| **Spaceport final**, sim 2 (N1000 + O5500) | 10718.5 \| 10776.8 (+0.5 %) | 410.0 \| 412.3 (+0.6 %) | 14.64 \| 14.70 (+0.4 %) | 2.87 \| 2.87 (-0.2 %) | 402.5 \| 403.7 (+0.3 %) |
| **Spaceport final**, sim 4 (M1800 + O5500) | 8045.3 \| 7905.3 (-1.7 %) | 430.1 \| 430.2 (0.0 %) | 23.17 \| 23.18 (+0.1 %) | 2.64 \| 2.62 (-0.6 %) | 338.2 \| 334.9 (-1.0 %) |
| **subscale Rev 4**, sim 1 (J425 + K535) | 2219.9 \| 2219.0 (0.0 %) | 197.2 \| 197.8 (+0.3 %) | 16.94 \| 16.91 (-0.2 %) | 4.90 \| 4.85 (-1.1 %) | 176.8 \| 177.4 (+0.3 %) |
| **subscale Rev 4**, sim 3 (H550 + J425) | 645.6 \| 641.0 (-0.7 %) | 89.5 \| 89.4 (-0.2 %) | 14.21 \| 14.18 (-0.2 %) | 5.74 \| 5.68 (-1.0 %) | 76.5 \| 76.5 (+0.1 %) |

Every apogee and peak speed is within 5 % (largest: simple rocket C6 +3.2 %, user's Spaceport final M1800 configuration -1.7 %). CP and CNα agree
to 0.003 cal / 0.2 % except near Mach 1-1.4. The flight-time differences (up to +5 %, dual parachute sim 1) are in the descent under the
parachute (canopy opening and descent rate), not in the dynamics.

## Flights with wind: point mass before, rigid body after

Until this change Bowshock flew a 3-DOF point mass whose axis was always on the relative wind: no angle of attack, no weathercocking, no
moments. It now flies a rigid body by default (`FlightModel::RigidBody`, pitch, yaw and roll with quaternion attitude; the old model stays as
`FlightModel::PointMass` for regression and costs about the same to run, 5-7 ms per flight in a release build). Equations and their sources are in the module docs of
`crates/bowshock-sim/src/flight.rs` and in `docs/aero-references.md`: the normal force at the CP about the moving CG, aerodynamic pitch/yaw
damping `1/2 rho V A_ref sum CNα_i (x_i - x_cg)^2`, body crossflow damping, jet damping, thrust along the body axis, the rail holding the
attitude, inertias from the parts with the propellant's falling as it burns, fin-cant roll forcing and damping, and the descent under a
parachute flown as a point mass.

The reference is OpenRocket run with the file's **mean wind and its gusts switched off** (the stored simulations all use 2 m/s except
subscale sim 3, 6.7 m/s; OpenRocket's own turbulence is random, so the stored runs with gusts are not reproducible); Bowshock flies the same
wind without gusts, vertical rod. Cells are OpenRocket \| point mass (Δ) \| rigid body (Δ):

| rocket / simulation | apogee (m) | max v (m/s) | flight time (s) |
|---|---|---|---|
| simple, sim 1 (A8), 2 m/s | 50.6 \| 51.7 (+2.2 %) \| 51.9 (+2.7 %) | 29.2 \| 29.8 (+1.8 %) \| 29.7 (+1.7 %) | 15.9 \| 16.1 (+1.3 %) \| 16.1 (+1.2 %) |
| dual parachute, sim 1 (H669), 2 m/s | 592.8 \| 591.8 (-0.2 %) \| 592.3 (-0.1 %) | 135.4 \| 135.5 (+0.1 %) \| 135.5 (+0.1 %) | 63.9 \| 64.8 (+1.5 %) \| 64.9 (+1.6 %) |
| dual parachute, sim 3 (J570), 2 m/s | 2224.1 \| 2211.9 (-0.5 %) \| 2213.8 (-0.5 %) | 388.7 \| 390.5 (+0.5 %) \| 390.5 (+0.5 %) | 129.4 \| 130.3 (+0.7 %) \| 130.3 (+0.7 %) |
| two-stage, sim 1, 2 m/s | 663.6 \| 665.6 (+0.3 %) \| 672.7 (+1.4 %) | 156.8 \| 159.0 (+1.4 %) \| 159.1 (+1.4 %) | 62.1 \| 63.7 (+2.6 %) \| 64.1 (+3.3 %) |
| **Spaceport final**, sim 1 (M1939 + O5500), 2 m/s | 9079.0 \| 8896.4 (-2.0 %) \| 8952.2 (-1.4 %) | 441.9 \| 442.8 (+0.2 %) \| 442.8 (+0.2 %) | 371.7 \| 366.6 (-1.4 %) \| 368.0 (-1.0 %) |
| **Spaceport final**, sim 2 (N1000 + O5500), 2 m/s | 10702.1 \| 10190.3 (-4.8 %) \| 10762.2 (+0.6 %) | 409.9 \| 412.8 (+0.7 %) \| 412.3 (+0.6 %) | 401.6 \| 389.9 (-2.9 %) \| 403.4 (+0.5 %) |
| **Spaceport final**, sim 4 (M1800 + O5500), 2 m/s | 8036.6 \| 7783.5 (-3.2 %) \| 7898.0 (-1.7 %) | 430.0 \| 430.2 (+0.1 %) \| 430.2 (+0.0 %) | 338.7 \| 331.6 (-2.1 %) \| 334.8 (-1.2 %) |
| **subscale Rev 4**, sim 1 (J425 + K535), 2 m/s | 2206.7 \| 2110.3 (-4.4 %) \| 2207.0 (+0.0 %) | 197.2 \| 198.8 (+0.8 %) \| 197.8 (+0.3 %) | 176.3 \| 171.2 (-2.9 %) \| 176.9 (+0.3 %) |
| **subscale Rev 4**, sim 3 (H550 + J425), 6.7 m/s | 635.6 \| 558.4 (-12.2 %) \| 631.9 (-0.6 %) | 89.5 \| 91.1 (+1.8 %) \| 89.4 (-0.2 %) | 75.8 \| 71.0 (-6.3 %) \| 76.2 (+0.5 %) |

| rocket / simulation | stability off rail (cal) | peak AoA, 2 s after rail exit (°) | landing distance (m) |
|---|---|---|---|
| simple, sim 1 (A8), 2 m/s | 2.28 \| 3.03 (+32.5 %) \| 2.28 (-0.2 %) | 7.3 \| 0.0 (-100.0 %) \| 7.4 (+1.0 %) | 19 \| 14 (-25.4 %) \| 19 (-0.9 %) |
| dual parachute, sim 1 (H669), 2 m/s | 3.92 \| 4.43 (+12.9 %) \| 3.83 (-2.3 %) | 2.1 \| 0.0 (-100.0 %) \| 2.1 (+2.1 %) | 97 \| 78 (-19.4 %) \| 101 (+4.6 %) |
| dual parachute, sim 3 (J570), 2 m/s | 2.73 \| 3.25 (+18.9 %) \| 2.66 (-2.6 %) | 2.1 \| 0.0 (-100.0 %) \| 2.1 (+1.2 %) | 175 \| 118 (-32.7 %) \| 182 (+3.9 %) |
| two-stage, sim 1, 2 m/s | 1.72 \| 2.17 (+25.8 %) \| 1.77 (+3.1 %) | 5.6 \| 0.0 (-100.0 %) \| 5.8 (+3.2 %) | 33 \| 30 (-9.4 %) \| 44 (+35.5 %) |
| **Spaceport final**, sim 1 (M1939 + O5500), 2 m/s | 1.94 \| 2.50 (+29.3 %) \| 1.88 (-2.8 %) | 3.0 \| 0.0 (-100.0 %) \| 3.1 (+2.9 %) | 276 \| 541 (+96.4 %) \| 274 (-0.7 %) |
| **Spaceport final**, sim 2 (N1000 + O5500), 2 m/s | 1.67 \| 2.87 (+71.9 %) \| 1.56 (-6.7 %) | 7.4 \| 0.0 (-100.0 %) \| 7.7 (+4.1 %) | 117 \| 3208 (+2645.2 %) \| 118 (+1.3 %) |
| **Spaceport final**, sim 4 (M1800 + O5500), 2 m/s | 1.78 \| 2.62 (+47.0 %) \| 1.71 (-4.4 %) | 4.9 \| 0.0 (-100.0 %) \| 4.9 (+1.4 %) | 237 \| 987 (+316.8 %) \| 244 (+2.9 %) |
| **subscale Rev 4**, sim 1 (J425 + K535), 2 m/s | 3.63 \| 4.85 (+33.7 %) \| 3.50 (-3.4 %) | 6.6 \| 0.0 (-100.0 %) \| 6.7 (+1.9 %) | 19 \| 574 (+2915.2 %) \| 25 (+28.8 %) |
| **subscale Rev 4**, sim 3 (H550 + J425), 6.7 m/s | 4.26 \| 5.68 (+33.3 %) \| 4.14 (-2.7 %) | 8.0 \| 0.0 (-100.0 %) \| 8.0 (+0.3 %) | 29 \| 317 (+1002.6 %) \| 26 (-11.3 %) |

What changed and why:

* **Apogee in wind**: the point mass turns the thrust into the relative wind the instant it leaves the rail, so it tips over early; apogee fell
  by up to 12 % in the rows above (-12.2 % on the subscale sim 3 at 6.7 m/s; -16.8 % on its sim 4; -4.8 % on the Spaceport final N1000). The rigid
  body has to turn, which takes about a pitch period, and agrees with OpenRocket within 2.7 % in every row (worst: the simple rocket
  +2.7 %, of which +2.3 % is already there without wind). Peak speed was never affected (it is reached with the axis on the flight path).
* **Stability off the rail**: OpenRocket evaluates it at the angle of attack the rocket has leaving the rail (the CP moves forward with it: 12-42 % lower than at
  zero angle in these winds). The point mass has none; the rigid body evaluates the margin at its own angle of attack and agrees within
  7 % (worst: Spaceport N1000 on its 5.7 m rod, -6.7 %).
* **Angle of attack** after the rail: within 4.1 % of OpenRocket's peak in all nine rows (the first sample after the rail, `atan(wind / speed)`).
* **Landing**: with the axis free, the distance from the pad is within 5 % of OpenRocket's on the simple rocket, dual parachute and
  Spaceport final rows (-1 .. +4.6 %), -11 % on subscale sim 3, +29 % on subscale sim 1 (25 m against 19 m) and +36 % on the two-stage (44 m
  against 33 m). The point mass was off by up to a factor of 27 (3208 m against 117 m on the Spaceport final's N1000).
* **Flight time**: differences are descent under the parachute, which both models fly as a point mass; unchanged from the calm table.

Remaining gaps. Tail-first flight after apogee without a parachute uses the same drag as nose-first (OpenRocket's own flip is more
detailed); the aerodynamics are axisymmetric (no crossflow azimuth, so 1- and 2-fin sets keep the fixed `CROSSFLOW_ROLL` and a spinning rocket does not
modulate its lift); pod fins and tube fins have no roll damping; staging transients (separation impulse, upper-stage attitude) are not modelled. The
two-stage and subscale sim 1 landing points (large relative, small absolute: 11 m and 6 m) are the rows to look at first. Ground-hit velocity is not
compared in wind: OpenRocket's looks like the vertical component (4.18 m/s with and without wind on the simple rocket) while Bowshock's includes
the drift.

## Differences above 5 %, and why

1. **Cd of ogive-nosed rockets above Mach 0.9: +8 .. +15 %. Cause: OpenRocket's code, not Bowshock.** The whole gap is nose-cone pressure drag.
   OpenRocket's technical documentation (Niskanen 2013, appendix B) says "at supersonic velocities the drag of a tangent ogive is approximately the
   same as the drag of a conical nose cone with the same length and diameter". Its code (`SymmetricComponentCalc.calculateNoseInterpolator`) instead
   takes the sine of the surface angle at 99 % of the length, which is about 0 for a tangent ogive: OpenRocket gives the simple rocket's nose
   Cd_pressure = 0.0012 at Mach 1 and 0.0004 at Mach 2 (the same shape in Bowshock: 0.12 and 0.07). Bowshock follows the documented method. Evidence:
   with the nose changed to conical (the same code path in both programs) Cd agrees to **-2.3 %** worst case over Mach 0.1-2 (table above), and
   subtracting the ogive term from Bowshock's ogive totals reproduces OpenRocket's Cd to 0.5 % (e.g. Spaceport final, Mach 1.2: 0.8683 - 0.1191 = 0.749 vs 0.784 - 0.038 = 0.746).
   (A report of the same behaviour in OpenRocket 24.12 exists as upstream issue 3391, not confirmed by its maintainers.) Not copied.
2. **CNα and CP at Mach 1.0-1.4: -0.7 % .. -3.8 %, CP up to -0.05 cal (below 5 %, listed for completeness).** OpenRocket interpolates the fin CNα between Mach 0.9
   and 1.5 with a quartic whose slope at Mach 0.9 is documented as the Diederich slope *at 0.9*; its code evaluates that slope at the current Mach
   number (`FinSetCalc.calculateFinCNa1`, `subD = 2 * mach * ...`), so its curve is not a single polynomial and reads higher at Mach 1.2. Bowshock
   implements the documented quartic; regression test `transonic_fin_cna_is_a_single_quartic_with_the_slope_taken_at_mach_0_9` (aero.rs).
3. **Motor-exhaust base-drag credit.** Bowshock removes the motor's exit area from the base drag while the motor burns; OpenRocket's static and flight
   drag never do. The columns "BS without the exhaust base-drag credit" of the full tables switch it off: simple rocket C6 apogee +3.2 % becomes +0.7 %;
   but the supersonic Spaceport final goes from -1.4 % to -5.3 % (apogee) and -3.7 % in peak speed, i.e. in the big rocket the ogive term (1., Bowshock
   drags more) and the credit (Bowshock drags less) nearly cancel; each is worth about 5 % of apogee there. Both are physically motivated
   differences, kept.
4. **Rail-exit speed -2 .. -4 % against OpenRocket at its default step**: OpenRocket's 50 ms step overshoots the rail end; against its own 2 ms run
   the difference is within 1 %.

Not a Bowshock-vs-OpenRocket difference: OpenRocket's own results vary a little from run to run on staged rockets (two-stage sim 1 apogee 678.4 and 666.6 m
in two runs of the harness), so differences of 1-2 % on those are within its noise.

## Fin lift: roll angle, fin-fin interference and free-form fins

Three parts of the fin normal force were added to Bowshock's model (`crates/bowshock-sim/src/aero.rs`, from Barrowman 1967 and Niskanen 2009
§3.2.2), each tested against hand values of its formula:

* **Roll angle.** Each fin's normal-force slope is scaled by `sin²(θ - φ)`, `φ` its roll angle and `θ` the crossflow direction. Bowshock's
  flight is axisymmetric in its aerodynamics, so the crossflow is fixed along +Y (`aero::CROSSFLOW_ROLL = 0`, the angle a fin at roll 0 points to; this is also
  the orientation OpenRocket's static analysis uses by default; the rigid-body flight model rolls under fin cant, but the roll angle does not feed back
  into the fin lift). 3 or more evenly spaced fins give `N/2` at any roll, as before; a 1- or 2-fin
  set now gives 0 at roll 0 and its full `1 x` / `2 x` one-fin lift at 90 degrees, instead of the old roll average `N/2`. Fins on a pod stand at
  the pod's angle plus their own.
* **Fin-fin interference.** Fin sets on the same body whose root chords overlap count together; 5, 6, 7, 8 and more fins keep 0.948, 0.913,
  0.854, 0.81 and 0.75 of their lift (`aero::fin_fin_factor`). Each pod copy counts its own fins.
* **Free-form fins** are still flown as the trapezoid of the same area, span and centroid, but the subsonic lift slope now uses the free-form
  outline's span-averaged mid-chord sweep (strips of the real outline) instead of the trapezoid's, and the interference count uses the real root
  chord. A free-form outline with a narrow root strake and a tall tab gives a much less swept trapezoid-equivalent than its real mid-chord line.

The body-fin interference factor stays `K_fb = 1 + r/(s + r)`. OpenRocket's development branch (after 24.12) replaces it with the slender-body
fin-in-body and body-in-fin loads of NACA Report 1307 (Pitts, Nielsen & Kaattari 1957; `(1 + τ)²` in the slender limit, `τ = r/(s + r)`), with
the body load at its own CP. That raises fin-set CNα by roughly `τ`, 25-35 % on these rockets subsonic, so it is **not** adopted while the
reference is the released 23.09; it is a known future change if the reference moves.

Before / after, subsonic (Mach 0.1-0.9, alpha 0 and 2 degrees, first configuration), worst difference:

| rocket | CNα before | CNα after | CP before (cal) | CP after (cal) |
|---|---|---|---|---|
| simple model rocket | 0.0 % | 0.0 % | 0.000 | 0.000 |
| simple, conical nose | 0.0 % | 0.0 % | 0.000 | 0.000 |
| dual parachute | 0.0 % | 0.0 % | 0.001 | 0.001 |
| two-stage | 0.0 % | 0.0 % | 0.000 | 0.000 |
| Spaceport final | 0.0 % | 0.0 % | 0.002 | 0.002 |
| Spaceport subscale Rev 4 | 0.0 % | 0.0 % | 0.001 | 0.001 |
| Pods, airframes and winglets | **-46.8 %** | **0.1 %** | **-0.538** | **+0.017** |
| Pods, powered (Mach ≤ 0.5) | **-26.9 %** | **0.0 %** | **-0.383** | **+0.005** |
| Pods, powered (Mach 0.55-0.9) | -26.9 % | 0.0 % | 5.9 | 6.2 (OpenRocket, below) |
| Tube fin rocket (Mach ≤ 0.5) | 0.0 % | 0.0 % | 0.000 | 0.000 |
| Tube fin rocket (Mach 0.55-0.9) | 0.0 % | 0.0 % | 0.730 | 0.730 (OpenRocket 23.09, see its section) |

The winglet rocket's sets, OpenRocket | Bowshock CNα at Mach 0.3: free-form 2-fin wing at 90° 18.279 | 18.296, vertical stabilizer at 0°
0 | 0, ventral fins at ±150° 0.3108 | 0.3108 each (5 fins on the airframe: 0.948),
horizontal stabilizer (2 fins at 90° on a pod) 2.170 | 2.170. At Mach 2 the wing is 9.787 in both. Flights of the winglet rocket: apogee +1.1 .. +1.5 %,
stability off the rail +1.4 .. +1.7 % (before: -8 .. -11 %).

Remaining gaps, with causes:

* **Pods, powered, CP above Mach 0.5** (+3.8 cal at Mach 0.8, +38 cal at 1.5) is on the reference side: two 1-fin strakes (root 88.9 mm,
  span 6.35 mm, swept past the root: aspect ratio 0.14) get a CP of -19 m at Mach 0.9 and -31 m at Mach 1.2 in OpenRocket's per-component output,
  the low-aspect-ratio pole of the supersonic CP rule `(AR β - 0.67)/(2 AR β - 1)` entering its transonic interpolation. Bowshock bounds the
  fin CP between the quarter chord and the supersonic value (see `fin_cp_fraction`). CNα agrees there. The powered-pods *flights* are not
  comparable: the pod motors, staging and "never" events of that design are modelled differently (warned by the importer).
* **Transonic CNα** -1.7 % (both pod rockets) to -3.8 % (two-stage) is item 2 of the previous section.

Across all nine rockets the subsonic CNα now agrees within 0.1 % and the CP within 0.02 cal, except where the reference's own CP is singular
(the powered pods' strakes above Mach 0.5, 23.09's tube-fin CP above Mach 0.5).

## What the importer maps, and what it cannot

See the module docs of `bowshock_model::ork`. Mass agrees to 0.0 % on the user's Spaceport final and to 0.5 % on the subscale
(component overrides, stage overrides, density differences, fin tabs, capped shoulders are mapped; remaining difference: fin and launch-lug mass
formulas of the model differ by about 1 % from OpenRocket's). Motors are matched by designation and manufacturer; OpenRocket has several database
entries under one designation (the Spaceport final has two different `O5500X-PS`), so the harness exports each configuration's curve and the comparison
loads the configuration's own file. An upper-stage motor reaching a few millimetres past a stage joint (OpenRocket's default overhang) is moved
forward by that amount with a warning so the model's staging rule finds the joint.

Examples run for exploration, not part of the acceptance set (`run.sh` extras; results in `docs` not kept):

* *Clustered motors* (only one of the cluster flies: apogee far off, warned), *Parallel booster staging* (strap-on motor/aerodynamics are
  modelled differently), *Base drag hack* (uses an OpenRocket drag override, which has no equivalent: warned), *Three-stage rocket*
  (after the overhang fix: CP and mass agree, apogee +2.3 %, the remaining Cd gap is the exhaust credit, 3). The two *Pods* examples are now
  part of the set above.
* Secant ogives, non-conical transitions, `upperignition` separation, "never" deployment and cluster configurations are approximated with a warning.

## Tube fin rocket (OpenRocket example)

OpenRocket's *Tube fin rocket* (6 tubes of the body's radius, 76.2 mm long, ogive nose, launch lug) imports as a pod of one open tube
(`RadialKind::Pod`), which the sim flies as tube fins (`crates/bowshock-sim/src/tube_fins.rs`). The importer reads an `auto` tube radius as the
outer radius of the tube it stands on (checked against OpenRocket: 0.01240 m both). Static aerodynamics, α = 0, ISA sea level:

| Mach | CP OR 23.09 (m) | CP OR, upstream fixes (m) | CP BS (m) | ΔCP vs fixed | Cd OR 23.09 | Cd OR, upstream fixes | Cd BS | ΔCd vs fixed | ΔCd vs fixed, nose excluded |
|---|---|---|---|---|---|---|---|---|---|
| 0.3 | 0.4966 | 0.4966 | 0.4966 | 0.00 cal | 1.7811 | 1.7811 | 1.7867 | +0.3 % | +0.4 % |
| 0.8 | 0.4785 | 0.4966 | 0.4966 | 0.00 cal | 1.9104 | 1.9104 | 1.9221 | +0.6 % | |
| 1.0 | 0.4785 | 0.4966 | 0.4966 | 0.00 cal | 1.9860 | 1.9860 | 2.0956 | +5.5 % | +0.4 % |
| 1.2 | 0.4785 | 0.4966 | 0.4966 | 0.00 cal | 2.2060 | 1.9680 | 2.0671 | +5.0 % | +0.4 % |
| 1.5 | 0.4785 | 0.4966 | 0.4966 | 0.00 cal | 1.8244 | 1.8738 | 1.9492 | +4.0 % | |
| 2.0 | 0.3878 | 0.4966 | 0.4966 | 0.00 cal | 1.5839 | 1.7812 | 1.8401 | +3.3 % | +0.4 % |

CNα is 39.847 in both at every Mach number (the ring-airfoil lift has no Mach dependence). Against OpenRocket 23.09 as shipped the CP
differs by +0.73 cal from Mach 0.8 to 1.5 and +4.4 cal at Mach 2, and Cd by -6.3 % (Mach 1.2) to +16.2 % (Mach 2); the sweep's worst Cd
difference, -29.5 %, is at Mach 1.05, where 23.09's Cd spikes to 3.02 (item 3). All of that is three behaviours of OpenRocket 23.09 that
OpenRocket's development branch has since changed:

1. **Tube-fin CP between Mach 0.5 and 2.** 23.09 puts the tube's CP at its *leading edge* over the whole range (its per-component output:
   0.5009 m = the tube's fore end at Mach 0.9-1.2, against the quarter chord 0.5199 m at Mach 0.3), a jump at Mach 0.5 instead of the
   documented interpolation (Niskanen 2013, eq. 3.36). Fixed upstream in commit `c01d03a7` ("Fix tube-fin CP interpolation initialization";
   not in the 24.12 release).
2. **Tube-fin CP at Mach 2.** 23.09 evaluates the empirical supersonic CP `(AR β - 0.67)/(2 AR β - 1)` (Niskanen 2013 eq. 3.35, Fleeman) at
   `AR β = 0.548`, near its pole at 0.5: -1.25 chords, i.e. 95 mm *ahead* of the tubes (per-component output 0.4054 m against a fore end
   at 0.5009 m). Bowshock holds the quarter chord where the formula drops below it (`AR β ≤ 0.84`); upstream changed it the same way in
   `efabea53` ("Regularize low-aspect-ratio fin CP").
3. **Bore drag above Mach 1.** 23.09 divides the Darcy friction factor of open tubes (tube fins and launch lugs) by `β = sqrt(M² - 1)`,
   which diverges at Mach 1: per tube, Cd 0.2620 at Mach 1.2 against 0.2237 without the factor (Bowshock 0.2238), 0.1970 against 0.2286 at
   Mach 2 (Bowshock 0.2286). Removed upstream in `a1d46a88` ("Remove Prandtl-Glauert correction factor"; in the 24.12 release).

The "upstream fixes" columns are OpenRocket 23.09's own per-component numbers with those three behaviours replaced by hand: the tubes' CNα
(37.85 of 39.85) moved to the quarter chord, and the bore term `f L/D (A_bore/A_ref)` multiplied back by β for the six tubes and the lug
(ISA sea level, ν = 1.4607e-5 m²/s, 60 µm roughness, Swamee-Jain f; this reproduces 23.09's per-tube pressure drag to 0.05 % below Mach 1).
Against them Bowshock's CP agrees to within 0.002 cal and its drag per tube to 0.05 % at every Mach number; the whole-rocket Cd differs
by +0.3 to +5.5 %, all of which except 0.4 % is the ogive-nose wave-drag difference of item 1 in the previous section (nose pressure drag,
OpenRocket against Bowshock: 0.0010 vs 0.1028 at Mach 1, 0.0378 vs 0.1293 at Mach 1.2, 0.0003 vs 0.0519 at Mach 2). Without the nose, Cd
agrees within 0.4 % at Mach 0.3, 0.9, 1.0, 1.2 and 2.0 (the Mach numbers with per-component output).

Bowshock's tube-fin model was not changed for this: the gaps are on the reference side. Its low-aspect-ratio CP hold, the 0.7 factor on the
tube's annular front face and the interstice charge are documented, with their reasoning, in `tube_fins.rs`. Raising the annulus factor
to the square-edged-fin value 1.0 would add +5.3 % Cd at Mach 0.3 and +7.9 % at Mach 2 to this rocket, and would not move the CP.

**Flights.** OpenRocket flies this rocket on a D12-7 held by the outer body tube itself (a `motormount` on the `bodytube`). The importer
gives such a body tube a massless inner tube of its bore and length as the mount (with a warning), so the motor sits where OpenRocket
puts it (aft end of the tube plus the 6.35 mm overhang). Liftoff mass 0.076 vs 0.077 kg (+0.2 %), CG 0.4528 vs 0.4529 m; burnout
0.055 kg both. The stored simulation (1 m rod, no wind, ISA):

| metric | OpenRocket | Bowshock | Δ | Bowshock without the exhaust base-drag credit | Δ |
|---|---|---|---|---|---|
| apogee (m) | 282.9 | 288.3 | +1.9 % | 282.3 | -0.2 % |
| max velocity (m/s) | 119.4 | 122.9 | +3.0 % | 119.2 | -0.1 % |
| max Mach | 0.351 | 0.362 | +3.0 % | | |
| time to apogee (s) | 5.92 | 5.95 | +0.5 % | | |
| rail-exit velocity (m/s, OR at 2 ms step) | 18.54 | 18.58 | +0.2 % | | |
| stability off the rail (cal) | 1.883 | 1.865 | -1.0 % | | |
| ground-hit velocity (m/s) | 3.84 | 3.90 | +1.6 % | | |
| flight time (s) | 75.5 | 72.9 | -3.5 % | | |

The flight stays below Mach 0.36, where the tube fins agree (table above); the apogee and speed differences are the exhaust base-drag
credit (item 3 of the previous section), and without it both agree within 0.2 %.
