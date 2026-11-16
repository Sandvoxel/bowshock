# Fins

Fin drag is an empirical add-on, not part of the flow solution. The method is that of
S. Niskanen, *Development of an Open Source model rocket simulation software* (M.Sc. thesis, 2009,
eqs. 3.78-3.85, 3.89-3.94, B.1-B.2), plus the Ackeret wave drag of a double-wedge section above
Mach 1. The fins are not in the grid, so the body's pressure and base drag are those of the body
alone. **Fin-body interference drag is neglected**.

## FinSpec

One set of identical trapezoidal fins, equally spaced around the body. `roughness` is the equivalent sand-grain roughness of the fin
surface in metres (0 = smooth; `60e-6` is OpenRocket's regular paint): the skin friction is floored at OpenRocket's roughness-limited
value `0.032 (roughness / L)^0.2`, with `L` the length of the body.

```rust
use bowshock_core::{FinProfile, FinSpec};

let fins = FinSpec {
    count: 4,
    root_chord: 0.12,
    tip_chord: 0.06,
    span: 0.08,
    sweep: 0.06,
    thickness: 0.003,
    roughness: 0.0,
    profile: FinProfile::Rounded,
};
fins.validate().unwrap();
```

| Field | Unit | Meaning |
| --- | --- | --- |
| `count` | - | number of fins, 3 to 8 |
| `root_chord` | m | chord at the body surface |
| `tip_chord` | m | chord at the tip; 0 for a triangular fin |
| `span` | m | distance from the body surface to the tip |
| `sweep` | m | axial distance from the root leading edge to the tip leading edge, positive aft |
| `thickness` | m | maximum thickness |
| `profile` | - | edge profile |

| `FinProfile` | Leading edge | Trailing edge |
| --- | --- | --- |
| `Square` | flat | flat (base drag) |
| `Rounded` | rounded | flat, half the base drag |
| `Airfoil` | rounded | tapered (no pressure drag) |
| `DoubleWedge` | sharp (no leading-edge drag), double-wedge section with Ackeret wave drag above Mach 1 | sharp |

### Validation

`FinSpec::validate` (called by `Analysis::run`) returns `Error::InvalidInput` unless:

| Rule | Field in the message |
| --- | --- |
| `count` in 3..=8 | `fins.count` |
| `root_chord`, `span`, `thickness` finite and > 0 | `fins.root_chord`, `fins.span`, `fins.thickness` |
| `tip_chord` finite and >= 0 | `fins.tip_chord` |
| `roughness` finite and >= 0 | `fins.roughness` |
| `sweep` finite and the leading-edge sweep angle `atan(sweep / span)` at most 75 deg in magnitude | `fins.sweep` |
| `thickness / mean aerodynamic chord <= 0.2` | `fins.thickness` |

```rust
use bowshock_core::{FinProfile, FinSpec};

let bad = FinSpec {
    count: 2,
    root_chord: 0.12,
    tip_chord: 0.06,
    span: 0.08,
    sweep: 0.06,
    thickness: 0.003,
    roughness: 0.0,
    profile: FinProfile::Square,
};
let err = bad.validate().unwrap_err();
assert!(err.to_string().starts_with("fins.count"));
```

## Using fins in an analysis

```rust,no_run
use bowshock_core::{
    AftSpec, Analysis, Atmosphere, BodySpec, FinProfile, FinSpec, FlightConditions, NoseShape, NoseSpec,
};

fn main() -> Result<(), bowshock_core::Error> {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }],
    };
    let fins = FinSpec {
        count: 4,
        root_chord: 0.12,
        tip_chord: 0.06,
        span: 0.08,
        sweep: 0.06,
        thickness: 0.003,
        roughness: 0.0,
        profile: FinProfile::Rounded,
    };
    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };

    let polar = Analysis::new(&spec)?.viscous(flight).fins(fins).run(&[0.6, 1.5])?;
    for p in &polar.points {
        let f = p.drag.fins.expect("fins were given");
        println!("M {}: fins friction {:.4} pressure {:.4}, total {:.4}", p.mach, f.friction, f.pressure, p.drag.total);
    }
    Ok(())
}
```

Fins need the free-stream state (for the fin Reynolds number), so `Analysis::viscous` is
required; without it `run` returns `InvalidInput("fins need flight conditions: call Analysis::viscous")`.
Fins also need an analysis built from a dimensional `BodySpec` (`Analysis::new`).

`Drag::fins` is `Some(FinDrag { friction, pressure })`, both on the reference area, and
`Drag::total` includes them:

```text
total = forebody_pressure + friction + base + fins.friction + fins.pressure
```

## Method

Notation: `N` fin count, `c_r`, `c_t` root and tip chord, `t` thickness, `b` span, `M` Mach
number, `A_ref = pi d_ref^2 / 4` the reference area (`d_ref` the [reference diameter](body.md#reference-diameter-and-reference-area)).

```text
A_fin   = b (c_r + c_t) / 2                      planform area of one fin
mac     = 2/3 (c_r + c_t - c_r c_t / (c_r + c_t))   mean aerodynamic chord
Lambda  = atan(sweep / b)                        leading-edge sweep angle
A_wet   = 2 N A_fin                              wetted area
A_front = N t b                                  frontal area
```

### Friction

The Reynolds number uses the mean aerodynamic chord and the free-stream density `rho`, speed of
sound `a` and viscosity `mu` of the [flight conditions](flight.md):

```text
Re   = rho (M a) mac / mu
Cf   = 1.48e-2                          for Re < 1e4
Cf   = 1 / (1.50 ln Re - 5.6)^2         for Re >= 1e4

Cf_c = Cf (1 - 0.1 M^2)                 for M < 1
Cf_c = Cf / (1 + 0.15 M^2)^0.58         for M >= 1

friction = Cf_c (1 + 2 t / mac) A_wet / A_ref
```

The factor `(1 + 2 t / mac)` is the thickness correction.

### Pressure

Stagnation pressure over free-stream dynamic pressure, and the stagnation-point drag:

```text
q_stag/q = 1 + M^2/4 + M^4/40                                      for M < 1
q_stag/q = 1.84 - 0.76/M^2 + 0.166/M^4 + 0.035/M^6                 for M >= 1
CD_stag  = 0.85 q_stag/q
```

Drag of a rounded leading edge, on its frontal area:

```text
LE_round = (1 - M^2)^(-0.417) - 1               for M < 0.9
LE_round = 1 - 1.785 (M - 0.9)                  for 0.9 <= M < 1
LE_round = 1.214 - 0.502/M^2 + 0.1095/M^4       for M >= 1
```

`CD_base` is the base-drag correlation `0.12 + 0.13 M^2` for `M < 1` and `0.25 / M` for `M >= 1`
(Niskanen 2009 eq. 3.94). Per profile, the leading-edge term `LE_perp` (on the plane normal to the
leading edge) and the trailing-edge term `TE`:

| Profile | `LE_perp` | `TE` |
| --- | --- | --- |
| `Square` | `CD_stag` | `CD_base` |
| `Rounded` | `LE_round` | `0.5 CD_base` |
| `Airfoil` | `LE_round` | 0 |
| `DoubleWedge` | 0 | 0 |

```text
pressure = (LE_perp cos^2(Lambda) + TE) A_front / A_ref
         + 4 (t/mac)^2 / sqrt(max(M^2 - 1, 0.3225)) * N A_fin / A_ref     [DoubleWedge and M >= 1 only]
```

The second line is the Ackeret double-wedge wave drag; the square root is floored at `M = 1.15`
(`M^2 - 1 = 0.3225`), so the wave drag does not blow up at Mach 1.

### Custom sections

`FinProfile::Custom { section }` describes your own symmetric section. `section` lists the **upper-surface** points `[x/c, z]` from the
leading edge (`x = 0`) to the trailing edge (`x = 1`). Only the *shape* matters: `z` is normalised by its maximum,
`zeta(x) = z / z_max` (piecewise linear between the points), and the actual maximum thickness is `FinSpec::thickness`. The scale of `z`
is therefore irrelevant, and you can paste the upper surface of a NACA `.dat` file as it is.

`FinSpec::validate` rejects a section (error field `fins.profile.section`) unless it has at least 3 points, all coordinates are finite,
the first `x` is 0 and the last is 1 (within 1e-9), `x` is strictly increasing, `z >= 0` everywhere and the maximum `z` is positive.

Friction is the same as for the other profiles. The pressure drag is derived from the section with `t = thickness` and `mac` the mean
aerodynamic chord:

- blunt leading-edge fraction `b = zeta(0)` and trailing-edge fraction `e = zeta(1)`;
- leading-edge class: with `s_k = (zeta_k - zeta_{k-1}) / (x_k - x_{k-1})` the edge is *rounded* if `s_1 / s_2 >= 1.5` (a sqrt-like
  nose; a wedge gives 1), or if `s_2 = 0` and `s_1 > 0`; a negative `s_2` (a double-wedge peak) is sharp. This is a heuristic on the
  first two segments, so give a few points near the leading edge for a rounded nose (NACA tables do);
- `LE_perp = b * CD_stag + (1 - b) * (LE_round if rounded else 0)` and `TE = e * CD_base`;
- `pressure = (LE_perp cos^2(sweep) + TE) * N t span / A_ref`;
- at `M >= 1` every custom section also gets the Ackeret linear-theory wave drag
  `4 / beta * I * N A_fin / A_ref`, with `I = integral_0^1 (d tau / d x)^2 dx`, `tau(x) = zeta(x) (t / mac) / 2` (half thickness over
  chord), integrated exactly over the linear segments (a blunt leading-edge step is not part of the integral), and
  `beta = sqrt(max(M^2 - 1, 0.3225))`.

A custom double wedge `[[0, 0], [0.5, 1], [1, 0]]` reproduces `FinProfile::DoubleWedge`, and a flat plate `[[0, 1], [0.5, 1], [1, 1]]`
reproduces `FinProfile::Square`. The section only enters through `b`, `e`, the leading-edge class and the wave-drag integral: the
subsonic profile drag of a real airfoil beyond that is outside the empirical method.

```rust
use bowshock_core::{FinProfile, FinSpec};

// NACA 0012 upper surface, generated from the standard thickness formula (any scale of z works)
let xs = [0.0, 0.0025, 0.005, 0.0125, 0.025, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
let section: Vec<[f64; 2]> = xs
    .iter()
    .map(|&x: &f64| [x, 0.6 * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x.powi(3) - 0.1015 * x.powi(4))])
    .collect();

let fins = FinSpec {
    count: 4,
    root_chord: 0.12,
    tip_chord: 0.06,
    span: 0.08,
    sweep: 0.06,
    thickness: 0.003, // the real maximum thickness in metres
    roughness: 0.0,
    profile: FinProfile::Custom { section },
};
fins.validate().unwrap();
```

`FinProfile` and `FinSpec` are `Clone` but not `Copy`; clone a spec to reuse it.

### What is neglected

- Fin-body interference (the body boundary layer on the fin root, the fin's effect on the
  body flow, the fin-body junction).
- Fin flutter, thickness distribution along the chord beyond the profile table, root-fillet and
  tip-vortex drag.
- Anything dependent on angle of attack: the analysis is zero-lift.

## Accuracy

Measured against the half-scale Arcas sounding rocket (NASA TN D-4013 / D-4014, four double-wedge
fins, turbulent; full table in [Validation](validation.md#arcas-sounding-rocket-d1)). The fin drag
increment, fins-on minus fins-off axial force (base excluded, on the body cross-section):

| M | data | bowshock | bowshock / data |
| --- | --- | --- | --- |
| 0.60 | 0.078 | 0.058 | 0.74 |
| 0.80 | 0.099 | 0.056 | 0.57 |
| 0.90 | 0.156 | 0.055 | 0.35 |
| 1.20 | 0.164 | 0.121 | 0.74 |
| 1.50 | 0.165 | 0.091 | 0.55 |
| 1.80 | 0.116 | 0.078 | 0.67 |
| 2.30 | 0.091 | 0.064 | 0.71 |
| 2.96 | 0.063 | 0.053 | 0.84 |

**The model under-predicts fin drag by 16-65 %**, most near Mach 0.9, where the data's transonic
drag rise of the fins is not part of the method. Part of the gap belongs to what is not modelled:
fin-body interference, and the Arcas fin anchors (bumps on the boattail; the report notes they were
not flush on this model). Fin drag above Mach 1 is under-predicted by 16-45 % on the Arcas.
