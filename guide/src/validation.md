# Validation

What has been compared with data and with analytical results, and how far. The numbers below are
copied from the repository's READMEs (`bowshock-validate/README.md`, `bowshock-cfd/README.md`)
as of this writing; they come from the validation programs in
`bowshock-validate` and the tests of the workspace. Where bowshock and the data differ, the table
shows the difference.

## Flat plate and body friction (G1-G3)

Three checks of the viscous terms and the k-omega SST model (names G1-G3 as in `bowshock-cfd`'s
tests):

| Check | Case | Result |
| --- | --- | --- |
| G1 | laminar flat plate | skin friction +0.64 % (x = 0.5) and +1.63 % (x = 0.9) of Blasius (`bowshock-validate/README.md`, "Checks of the viscous terms") |
| G2 | SST flat plate, Re_x = 4.8e6, first-cell y+ = 0.36 | skin friction -4.25 % against the turbulent correlation `0.37 / log10(Re_x)^2.584` (`bowshock-validate/README.md`) |
| G3 | tangent ogive 0.30 x 0.10 m + 0.60 m tube, ISA sea level, Mach 0.5, Coarse | friction drag 0.0900, 0.964 of the flat-plate estimate over the wetted area; base Cp -0.123; total CD 0.213; y+ max 0.65; about 27 s on the CPU (`bowshock-validate/README.md`, "Known limits") |

On the GPU, G3 gives total CD 0.21300 in f32 and in double-float (+0.001 % and -0.000 % against
the CPU), `cd_friction` 0.09003, `cd_base` 0.12257, `cp_base` -0.1226, y+ max 0.648
(`bowshock-cfd/README.md`, "Viscous terms and the SST model").

G3 is a consistency check of friction against a flat-plate estimate, not a comparison with measured
total drag. The Arcas comparison [below](#arcas-sounding-rocket-d1) is.

## Fox base pressures

NASA TM X-2439 (Fox, 1971): sting-mounted nose-cylinder-boattail bodies, subsonic. From
`bowshock-cfd/README.md`, "Validation: NASA TM X-2439", level 1, ogive (config 4):

| M | base Cp CFD (Euler) | data base Cp |
| --- | --- | --- |
| 0.404 | +0.0564 | +0.0691 |
| 0.608 | +0.0643 | +0.0836 |
| 0.831 | +0.0859 | +0.1317 |

The Euler base Cp is below the data in all six configurations (by 0.0116 to 0.0458). The viscous
analysis (RANS) is closer (`README.md`, "Known limits"; `cargo run --release --bin subsonic -- --quick --validate --rans`): RANS base Cp +0.049 to +0.072
against measured +0.069 to +0.132 (Euler: -0.037 to +0.066). The Reynolds numbers per Mach
number in this comparison were interpolated from the range quoted in the report and not
verified.

The wall Cp over the forebody and cylinder agrees with the data to an RMS of 0.017 to 0.091 (ogive
0.017 / 0.023 / 0.060 at M 0.404 / 0.608 / 0.831).

With the RANS equations (`cargo run --release --bin subsonic -- --quick --validate --rans`, quick grids)
the base Cp of the six configurations is +0.049 to +0.072 against measured +0.069 to +0.132 (Euler:
-0.037 to +0.066); the Reynolds numbers per Mach number are interpolated from the range quoted in the
report (`bowshock-validate/README.md`, "Known limits").

## Stoney forebody

Stoney, NACA RM L53K17 (1954), Fig. 5: nose pressure drag of fineness-3 noses at Mach 0.8 to
1.25, free flight. The data were digitised from a poor scan (about +-0.01 in CD) and carry a tare
uncertainty of about +-0.01; the Euler solver at level 1 reproduces the drag rise
(`bowshock-cfd/README.md`, "2. Nose pressure drag vs Mach number"). Von Karman nose:

| M | CD CFD (level 1) | Stoney | CFD - Stoney |
| --- | --- | --- | --- |
| 0.95 | 0.0079 | 0.000 | +0.008 |
| 1.00 | 0.0445 | 0.030 | +0.015 |
| 1.05 | 0.0791 | 0.070 | +0.009 |
| 1.10 | 0.0857 | 0.075 | +0.011 |
| 1.20 | 0.0871 | 0.080 | +0.007 |

The CFD lies systematically 0.007 to 0.016 above the digitised data, at 1 to 1.5 sigma of the
combined uncertainty. For the cone-cylinder (9.46 deg) the CD at Mach below 1.05 does not converge
with the grid (sharp shoulder corner): M 1.0 gives 0.1215 / 0.1301 / 0.1351 at levels 0 / 1 / 2.
That is the reason for `Warning::SharpCornerTransonic`. At supersonic Mach the cone nose converges
to the Taylor-Maccoll value (M 1.2: -3.9 % / -2.0 % / -1.0 % at levels 0 / 1 / 2).

## Arcas sounding rocket (D1)

Half-scale Arcas sounding rocket, configuration 1 (l/d = 18.20): NASA TN D-4013 (J. C. Ferris,
1967, Mach 0.60-1.20, Fig. 11) and TN D-4014 (C. D. Babb & D. E. Fuller, 1967, Mach 1.50-2.96,
Figs. 4-5). Data are figure readings (+/-0.005 on the flat parts, +/-0.02 at Mach 0.90-1.00,
+/-0.012 for D-4014). Compared quantity: axial force without the base (D-4013 `C_A,corr`; D-4014
`C_A - C_A,c`, the sign of the chamber correction inferred), on the body cross-section; bowshock:
`forebody_pressure + friction + fins`. Modelled body: tangent-ogive nose of the same length (the
real nose is up to 7 % slimmer), cylinder, conical boattail; fins as double wedges with the mean
thickness; no fin anchors, no sting. `Resolution::Medium`, viscous at the tunnel conditions
(Re 9.8e6 / m, T0 322 K transonic; 9.84e6 / m, T0 339 K supersonic), GPU, 99 s for all 20 points.

| config | M | data | bowshock | delta | delta / data | gate |
| --- | --- | --- | --- | --- | --- | --- |
| fins off | 0.60 | 0.222 | 0.2334 | +0.0114 | +5.1 % | pass |
| fins off | 0.80 | 0.233 | 0.2380 | +0.0050 | +2.1 % | pass |
| fins off | 0.90 | 0.264 | 0.2607 | -0.0033 | -1.3 % | pass |
| fins off | 0.95 | 0.298 | 0.2793 | -0.0187 | -6.3 % | not gated |
| fins off | 1.00 | 0.425 | 0.3874 | -0.0376 | -8.8 % | not gated |
| fins off | 1.20 | 0.436 | 0.3751 | -0.0609 | -14.0 % | **fail** (12 %) |
| fins off | 1.50 | 0.340 | 0.3262 | -0.0138 | -4.0 % | not gated |
| fins off | 1.80 | 0.322 | 0.2906 | -0.0314 | -9.7 % | not gated |
| fins off | 2.30 | 0.258 | 0.2478 | -0.0102 | -4.0 % | not gated |
| fins off | 2.96 | 0.210 | 0.2090 | -0.0010 | -0.5 % | not gated |
| fins on | 0.60 | 0.300 | 0.2911 | -0.0089 | -3.0 % | pass |
| fins on | 0.80 | 0.332 | 0.2941 | -0.0379 | -11.4 % | pass |
| fins on | 0.90 | 0.420 | 0.3157 | -0.1043 | -24.8 % | **fail** (15 %) |
| fins on | 0.95 | 0.570 | 0.3338 | -0.2362 | -41.4 % | not gated |
| fins on | 1.00 | 0.691 | 0.5214 | -0.1696 | -24.5 % | not gated |
| fins on | 1.20 | 0.600 | 0.4960 | -0.1040 | -17.3 % | **fail** (15 %) |
| fins on | 1.50 | 0.505 | 0.4168 | -0.0882 | -17.5 % | **fail** (15 %) |
| fins on | 1.80 | 0.438 | 0.3681 | -0.0699 | -16.0 % | **fail** (15 %) |
| fins on | 2.30 | 0.349 | 0.3121 | -0.0369 | -10.6 % | pass |
| fins on | 2.96 | 0.273 | 0.2619 | -0.0111 | -4.1 % | pass |

Gates: fins off `|delta| <= max(0.12 data, 0.03)` at M 0.60, 0.80, 0.90, 1.20; fins on
`|delta| <= 0.15 data` at those and the four D-4014 Mach numbers. Every point ended `Stationary`.
**5 of 16 gated points fail.** The fins-off body alone is within 5.1 % at M 0.6-0.9, 1.5, 2.3 and
2.96 (-9.7 % at M 1.8), but 14 % low at M 1.2 (`Resolution::Fine` gives 0.3765, -13.6 %: not a grid effect; the
report attributes part of the short model's higher axial force at M 1.0-1.2 to fin anchors that
were not flush). With fins the totals are low because the empirical fin drag is low (see
[Fins](fins.md#accuracy)). The test is `bowshock/tests/validation.rs::arcas_axial_force`
(`cargo test --release -p bowshock --test validation arcas -- --ignored --nocapture`); it fails
on these gates.

## Motor-on base pressure, NASA TP-2005 (D2)

NASA TP-2005 (1982): strut-mounted ogive-cylinder, D = 6.35 cm, base at 82.55 cm (13 D, flat base),
cold air jets through a 5.08 cm exit; free stream Re 6.56e6 / m, T0 325 K; jet total temperature
305 K. The nose length is not given (fineness 3 assumed). Data: measured base pressure
`pb / p_inf = (pt_j / p_inf) / (pt_j / pb)` from Table II. bowshock: `MotorSpec` with the exit Mach
and the exit pressure from `pt_j / p_inf` (isentropic), `pb / p_inf = 1 + 0.7 M^2 cp_base`.
Gates: `|delta| <= 0.15` and not `Failed` at every point; pb rises from Me 1.69 to Me 1.97.

`Resolution::Medium`, GPU, 178 s for the 11 points; every point ends `Stationary` (an earlier run with the
viscous CFL limit of 50 diverged at 8 of 11 points; the limit is now 20 above Mach 1):

| M_inf | Me | data pb/p_inf | bowshock | delta | stop |
| --- | --- | --- | --- | --- | --- |
| 1.65 | 1.69 | 1.562 | 1.053 | -0.509 | mean stationary, 14000 it |
| 1.65 | 1.97 | 1.625 | 0.974 | -0.651 | mean stationary, 14250 it |
| 1.65 | 2.33 | 1.665 | 0.838 | -0.827 | mean stationary, 14150 it |
| 1.65 | 2.53 | 1.664 | 0.781 | -0.884 | mean stationary, 14000 it |
| 2.00 | 1.69 | 1.342 | 0.912 | -0.429 | CD stationary, 4000 it |
| 2.00 | 1.97 | 1.531 | 0.825 | -0.706 | CD stationary, 4000 it |
| 2.00 | 2.33 | 1.642 | 0.720 | -0.922 | mean stationary, 14000 it |
| 2.00 | 2.53 | 1.641 | 0.680 | -0.961 | CD stationary, 4000 it |
| 2.50 | 1.69 | 0.952 | 0.748 | -0.204 | CD stationary, 4000 it |
| 2.50 | 1.97 | 1.130 | 0.684 | -0.446 | CD stationary, 4000 it |
| 2.50 | 2.53 | 1.335 | 0.588 | -0.748 | CD stationary, 4000 it |

**Every gate fails.** The measured base pressures above p_inf come from plume-induced separation
of the afterbody boundary layer ahead of the strongly under-expanded plumes; the model's base
pressure stays near or below p_inf and falls with exit Mach where the data rise. Open issue:
`bowshock-cfd/ISSUES.md` row 13. The test is `bowshock/tests/validation.rs::tp2005_base_pressure_with_jet`.

The Arcas table above was re-run with the same change: every value is unchanged to the printed digits
(largest change 0.0001).
