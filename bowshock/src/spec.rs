//! Dimensional body description (metres) and its conversion to the internal body (lengths in nose diameters).

use crate::body_grid::{BaseBody, Body, Nose, Segment};
use crate::error::Error;

/// Longest accepted body, in nose diameters.
const MAX_LENGTH_DIAMETERS: f64 = 40.0;
/// Steepest accepted nose cone / transition half-angle.
const MAX_HALF_ANGLE: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// Nose profile family.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NoseShape {
    /// Cone (half-angle at most 30 deg).
    Cone,
    /// Circular-arc tangent ogive.
    TangentOgive,
    /// Power series `r/R = (x/L)^n`, `n` in `[0.5, 1]`.
    Power { n: f64 },
    /// Parabolic series with parameter `k` in `[0, 1]` (`0` cone, `1` tangent parabola).
    Parabolic { k: f64 },
    /// Haack series with parameter `c` in `[0, 1/3]` (`0` Von Karman, `1/3` LV-Haack).
    Haack { c: f64 },
}

/// Nose of the body, in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NoseSpec {
    pub shape: NoseShape,
    /// Axial length of the nose.
    pub length: f64,
    /// Diameter at the nose base.
    pub diameter: f64,
}

/// One section behind the nose, in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AftSpec {
    /// Constant-diameter tube.
    Tube { length: f64 },
    /// Cone frustum (flare or boattail) from the current diameter to `aft_diameter`.
    Transition { length: f64, aft_diameter: f64 },
}

/// Axisymmetric body: a nose followed by tubes and transitions, ending in a flat base.
#[derive(Clone, Debug, PartialEq)]
pub struct BodySpec {
    pub nose: NoseSpec,
    pub aft: Vec<AftSpec>,
}

/// Conversion between the internal units and the dimensional body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Scale {
    /// Metres per internal length unit (the nose diameter).
    pub d_nose: f64,
    /// `(d_nose / d_ref)^2`: converts a coefficient referenced to the nose base area to the reference area.
    pub area_ratio: f64,
}

fn invalid<T>(field: &str, why: &str) -> Result<T, Error> {
    Err(Error::InvalidInput(format!("{field}: {why}")))
}

fn positive(field: &str, v: f64) -> Result<(), Error> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        invalid(field, &format!("must be finite and > 0, got {v}"))
    }
}

impl BodySpec {
    /// Largest diameter of the body [m].
    pub fn reference_diameter(&self) -> f64 {
        self.aft.iter().fold(self.nose.diameter, |d, a| match *a {
            AftSpec::Transition { aft_diameter, .. } => d.max(aft_diameter),
            AftSpec::Tube { .. } => d,
        })
    }

    /// Validate the body and convert it to the internal body (lengths divided by the nose diameter) and its [`Scale`].
    pub fn to_internal(&self) -> Result<(BaseBody, Scale), Error> {
        let n = &self.nose;
        positive("nose.length", n.length)?;
        positive("nose.diameter", n.diameter)?;
        let fineness = n.length / n.diameter;
        if !(0.5..=12.0).contains(&fineness) {
            return invalid("nose.length", &format!("fineness length / diameter = {fineness:.3} must be in [0.5, 12]"));
        }
        let nose = match n.shape {
            NoseShape::Cone => {
                let half_angle = (0.5 * n.diameter / n.length).atan();
                if half_angle > MAX_HALF_ANGLE {
                    return invalid("nose.shape", &format!("cone half-angle {:.1} deg exceeds 30 deg", half_angle.to_degrees()));
                }
                Nose::cone_fineness(fineness)
            }
            NoseShape::TangentOgive => Nose::TangentOgive { fineness },
            NoseShape::Power { n } => {
                if !(0.5..=1.0).contains(&n) {
                    return invalid("nose.shape.n", &format!("must be in [0.5, 1], got {n}"));
                }
                Nose::Power { fineness, n }
            }
            NoseShape::Parabolic { k } => {
                if !(0.0..=1.0).contains(&k) {
                    return invalid("nose.shape.k", &format!("must be in [0, 1], got {k}"));
                }
                Nose::Parabolic { fineness, k }
            }
            NoseShape::Haack { c } => {
                if !(0.0..=1.0 / 3.0).contains(&c) {
                    return invalid("nose.shape.c", &format!("must be in [0, 1/3], got {c}"));
                }
                Nose::Haack { fineness, c }
            }
        };
        if self.aft.is_empty() {
            return invalid("aft", "needs at least one section");
        }
        let d = n.diameter;
        let mut segments = Vec::with_capacity(self.aft.len());
        let mut radius = 0.5; // internal radius at the start of the section
        let mut total = n.length;
        for (k, a) in self.aft.iter().enumerate() {
            match *a {
                AftSpec::Tube { length } => {
                    positive(&format!("aft[{k}].length"), length)?;
                    total += length;
                    segments.push(Segment { length: length / d, slope: 0.0 });
                }
                AftSpec::Transition { length, aft_diameter } => {
                    positive(&format!("aft[{k}].length"), length)?;
                    positive(&format!("aft[{k}].aft_diameter"), aft_diameter)?;
                    let (len, r_end) = (length / d, 0.5 * aft_diameter / d);
                    let slope = (r_end - radius) / len;
                    if slope.atan().abs() > MAX_HALF_ANGLE {
                        return invalid(&format!("aft[{k}]"), &format!("transition half-angle {:.1} deg exceeds 30 deg", slope.atan().abs().to_degrees()));
                    }
                    total += length;
                    radius = r_end;
                    segments.push(Segment { length: len, slope });
                }
            }
        }
        if total > MAX_LENGTH_DIAMETERS * d {
            return invalid("aft", &format!("total body length {:.1} nose diameters exceeds {MAX_LENGTH_DIAMETERS}", total / d));
        }
        let d_ref = self.reference_diameter();
        let body = BaseBody { body: Body { nose }, segments, sting_radius: 0.0 };
        Ok((body, Scale { d_nose: d, area_ratio: (d / d_ref).powi(2) }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> BodySpec {
        BodySpec {
            nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.1 },
            aft: vec![AftSpec::Tube { length: 0.6 }, AftSpec::Transition { length: 0.1, aft_diameter: 0.08 }],
        }
    }

    #[test]
    fn conversion_scales_lengths_and_reproduces_the_aft_radius() {
        let (b, s) = spec().to_internal().unwrap();
        assert_eq!(s.d_nose, 0.1);
        assert_eq!(s.area_ratio, 1.0);
        assert!((b.body.nose_length() - 3.0).abs() < 1e-12);
        assert!((b.x_base() - 10.0).abs() < 1e-12);
        assert!((b.base_radius() - 0.4).abs() < 1e-12);
    }
}
