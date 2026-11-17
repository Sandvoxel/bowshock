# The body

A `BodySpec` is a nose followed by one or more aft sections. All lengths are in metres. The body
ends in a flat base.

```rust
use bowshock_core::{AftSpec, BodySpec, NoseShape, NoseSpec};

let spec = BodySpec {
    nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
    aft: vec![
        AftSpec::Tube { length: 0.60 },
        AftSpec::Transition { length: 0.10, aft_diameter: 0.08 },
    ],
};
assert_eq!(spec.reference_diameter(), 0.10);
assert_eq!(spec.base_diameter(), 0.08);
```

## Nose

`NoseSpec { shape, length, diameter }`: `length` is the axial length of the nose [m], `diameter`
the diameter at the nose base [m]. The tip is on the axis (sharp, or a sphere with `Blunted`). The fineness ratio
`length / diameter` must be in [0.5, 12].

| `NoseShape` | Profile | Parameter range |
| --- | --- | --- |
| `Cone` | straight cone | half-angle `atan(0.5 diameter / length)` at most 30 deg |
| `TangentOgive` | circular-arc tangent ogive | none |
| `Power { n }` | `r/R = (x/L)^n` | `n` in [0.5, 1] |
| `Parabolic { k }` | parabolic series | `k` in [0, 1] (0 cone, 1 tangent parabola) |
| `Haack { c }` | Haack series | `c` in [0, 1/3] (0 Von Karman, 1/3 LV-Haack) |
| `Blunted { base, tip_radius }` | any of the shapes above (`SharpShape`) with its tip replaced by a sphere of radius `tip_radius` [m] tangent to the profile | `tip_radius` at most 0.9 of the base radius, and a sphere that is tangent (a `Power` of `n = 0.5` takes none smaller than its tip's radius of curvature `R^2 / (2 L)`) |

With `Blunted`, `length` is the length of the nose as built, from the sphere's apex to the base: the sharp profile
the sphere sits on is longer by the cut-off. The fineness limits apply to `length`, the cone half-angle limit to the
sharp cone.

## Aft sections

`aft` lists the sections behind the nose, in order; at least one is required.

- `AftSpec::Tube { length }`: constant diameter, `length` [m].
- `AftSpec::Transition { length, aft_diameter }`: cone frustum from the current diameter to
  `aft_diameter` [m], over `length` [m]. A flare if the diameter grows, a boattail if it shrinks.
  The half-angle is at most 30 deg.

## Validation rules

`Analysis::new(&spec)` validates the body and returns `Error::InvalidInput`. The message starts
with the offending field:

| Rule | Field in the message |
| --- | --- |
| `nose.length` and `nose.diameter` finite and > 0 | `nose.length`, `nose.diameter` |
| fineness `nose.length / nose.diameter` in [0.5, 12] | `nose.length` |
| cone half-angle <= 30 deg | `nose.shape` |
| `Power::n`, `Parabolic::k`, `Haack::c` inside their ranges | `nose.shape.n`, `nose.shape.k`, `nose.shape.c` |
| `Blunted::tip_radius` in (0, 0.9 of the base radius] with a tangent sphere on the profile | `nose.shape.tip_radius` |
| `aft` not empty | `aft` |
| every section `length` and `aft_diameter` finite and > 0 | `aft[<index>].length`, `aft[<index>].aft_diameter` |
| transition half-angle <= 30 deg | `aft[<index>]` |
| total length (nose + sections) <= 100 nose diameters | `aft` |

```rust
use bowshock_core::{AftSpec, Analysis, BodySpec, Error, NoseShape, NoseSpec};

// fineness 15 is outside [0.5, 12]
let spec = BodySpec {
    nose: NoseSpec { shape: NoseShape::TangentOgive, length: 1.5, diameter: 0.1 },
    aft: vec![AftSpec::Tube { length: 0.5 }],
};
let err = Analysis::new(&spec).unwrap_err();
assert!(matches!(err, Error::InvalidInput(_)));
assert!(err.to_string().starts_with("nose.length"));
println!("{err}");
```

`Error` implements `Display` (the message above) and `std::error::Error`.

## Reference diameter and reference area

Drag coefficients are referenced to the area of a circle of the reference diameter. By default
that is the largest diameter of the body (`BodySpec::reference_diameter`): the maximum of the
nose diameter and every `aft_diameter`. Override it with `Analysis::reference_diameter(d_m)`
[m], for example to quote coefficients on the body-tube area when a flare is larger:

```rust,no_run
# use bowshock_core::{AftSpec, Analysis, BodySpec, NoseShape, NoseSpec};
# fn main() -> Result<(), bowshock_core::Error> {
# let spec = BodySpec {
#     nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
#     aft: vec![AftSpec::Tube { length: 0.60 }],
# };
let polar = Analysis::new(&spec)?.reference_diameter(0.12).run(&[0.8])?;
# Ok(())
# }
```

The reference diameter must be finite and > 0, otherwise `run` returns `InvalidInput`. Fin drag
uses the same reference area.

## Base diameter

`BodySpec::base_diameter()` is the diameter of the flat base [m]: the `aft_diameter` of the last
transition, or the nose diameter when only tubes follow the nose. A [motor](motor.md) nozzle exit
must fit inside it.
