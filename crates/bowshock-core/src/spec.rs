//! Dimensional body description (metres) and its conversion to the internal body (lengths in nose diameters).

use crate::engine::body_grid::{BaseBody, Body, Nose, Segment};
use crate::error::Error;

/// Longest accepted body, in nose diameters.
const MAX_LENGTH_DIAMETERS: f64 = 40.0;
/// Steepest accepted nose cone / transition half-angle.
const MAX_HALF_ANGLE: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// Nose profile family.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum NoseShape {
    /// Cone (half-angle at most 30 deg).
    Cone,
    /// Circular-arc tangent ogive.
    TangentOgive,
    /// Power series `r/R = (x/L)^n`, `n` in `[0.5, 1]`.
    Power {
        /// Exponent.
        n: f64,
    },
    /// Parabolic series with parameter `k` in `[0, 1]` (`0` cone, `1` tangent parabola).
    Parabolic {
        /// Series parameter.
        k: f64,
    },
    /// Haack series with parameter `c` in `[0, 1/3]` (`0` Von Karman, `1/3` LV-Haack).
    Haack {
        /// Series parameter.
        c: f64,
    },
}

/// Nose of the body, in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NoseSpec {
    /// Profile family.
    pub shape: NoseShape,
    /// Axial length of the nose.
    pub length: f64,
    /// Diameter at the nose base.
    pub diameter: f64,
}

/// One section behind the nose, in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AftSpec {
    /// Constant-diameter tube.
    Tube {
        /// Axial length.
        length: f64,
    },
    /// Cone frustum (flare or boattail) from the current diameter to `aft_diameter`.
    Transition {
        /// Axial length.
        length: f64,
        /// Diameter at the aft end.
        aft_diameter: f64,
    },
}

/// Axisymmetric body: a nose followed by tubes and transitions, ending in a flat base.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BodySpec {
    /// The nose.
    pub nose: NoseSpec,
    /// Sections behind the nose, in order, ending in the flat base.
    pub aft: Vec<AftSpec>,
}

/// Conversion between the internal units and the dimensional body.
#[doc(hidden)]
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

/// Edge profile of the fins.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FinProfile {
    /// Flat leading and trailing edges.
    Square,
    /// Rounded leading edge, flat trailing edge.
    Rounded,
    /// Rounded leading edge, tapered trailing edge.
    Airfoil,
    /// Sharp edges, double-wedge section (supersonic wave drag).
    DoubleWedge,
    /// User-defined symmetric section: upper-surface coordinates `[x/c, z]` from the leading edge (`x = 0`) to the trailing edge
    /// (`x = 1`). Only the shape matters: `z` is normalised by its maximum and the maximum thickness is [`FinSpec::thickness`], so
    /// the scale of `z` is irrelevant (a NACA `.dat` upper surface can be pasted as is).
    Custom {
        /// Upper-surface points `[x/c, z]`: at least 3, `x` strictly increasing from 0 to 1, `z >= 0` with a positive maximum.
        section: Vec<[f64; 2]>,
    },
}

/// One set of identical trapezoidal fins, in metres. `sweep` is the axial distance from the root leading edge to the tip leading edge (positive aft).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FinSpec {
    /// Number of fins, equally spaced around the body (3 to 8).
    pub count: u32,
    /// Chord at the body surface \[m\].
    pub root_chord: f64,
    /// Chord at the tip \[m\] (0 for a triangular fin).
    pub tip_chord: f64,
    /// Distance from the body surface to the tip \[m\].
    pub span: f64,
    /// Axial distance from the root leading edge to the tip leading edge \[m\], positive aft.
    pub sweep: f64,
    /// Maximum thickness \[m\].
    pub thickness: f64,
    /// Equivalent sand-grain roughness of the fin surface \[m\]; 0 is smooth. The skin friction is floored at OpenRocket's
    /// roughness-limited value (`60e-6` is OpenRocket's regular paint).
    pub roughness: f64,
    /// Edge profile.
    pub profile: FinProfile,
}

impl FinSpec {
    /// Check the fin set; the error names the offending field as `fins.<field>`.
    pub fn validate(&self) -> Result<(), Error> {
        if !(3..=8).contains(&self.count) {
            return invalid("fins.count", &format!("must be in 3..=8, got {}", self.count));
        }
        positive("fins.root_chord", self.root_chord)?;
        positive("fins.span", self.span)?;
        positive("fins.thickness", self.thickness)?;
        if !(self.tip_chord.is_finite() && self.tip_chord >= 0.0) {
            return invalid("fins.tip_chord", &format!("must be finite and >= 0, got {}", self.tip_chord));
        }
        if !(self.roughness.is_finite() && self.roughness >= 0.0) {
            return invalid("fins.roughness", &format!("must be finite and >= 0, got {}", self.roughness));
        }
        if !(self.sweep.is_finite() && (self.sweep / self.span).atan().abs() <= 75f64.to_radians()) {
            return invalid("fins.sweep", &format!("must be finite with a leading-edge sweep angle of at most 75 degrees, got {}", self.sweep));
        }
        if let FinProfile::Custom { section } = &self.profile {
            validate_section(section)?;
        }
        let mac = crate::fins::mean_aerodynamic_chord(self.root_chord, self.tip_chord);
        if self.thickness / mac > 0.2 {
            return invalid("fins.thickness", &format!("thickness/mean chord must be <= 0.2, got {}", self.thickness / mac));
        }
        Ok(())
    }
}

fn validate_section(s: &[[f64; 2]]) -> Result<(), Error> {
    const F: &str = "fins.profile.section";
    if s.len() < 3 {
        return invalid(F, &format!("needs at least 3 points, got {}", s.len()));
    }
    if s.iter().any(|p| !(p[0].is_finite() && p[1].is_finite())) {
        return invalid(F, "all coordinates must be finite");
    }
    if s[0][0].abs() > 1e-9 {
        return invalid(F, &format!("first x must be 0, got {}", s[0][0]));
    }
    if (s[s.len() - 1][0] - 1.0).abs() > 1e-9 {
        return invalid(F, &format!("last x must be 1, got {}", s[s.len() - 1][0]));
    }
    if s.windows(2).any(|w| w[1][0] <= w[0][0]) {
        return invalid(F, "x must be strictly increasing");
    }
    if s.iter().any(|p| p[1] < 0.0) {
        return invalid(F, "z must be >= 0");
    }
    if s.iter().all(|p| p[1] <= 0.0) {
        return invalid(F, "the maximum z must be > 0");
    }
    Ok(())
}

/// Rocket motor exhausting through the centre of the base, modelled as an air jet (gamma = 1.4) at the nozzle exit.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotorSpec {
    /// Diameter of the nozzle exit \[m\]; at most 0.95 of the base diameter.
    pub exit_diameter: f64,
    /// Mach number of the jet at the exit [-], in `[1, 5]`.
    pub exit_mach: f64,
    /// Static pressure at the exit \[Pa\].
    pub exit_pressure_pa: f64,
    /// Total temperature of the jet \[K\], in `[200, 4000]`.
    pub total_temperature_k: f64,
}

impl MotorSpec {
    /// Check the motor against the base diameter \[m\] of the body; `InvalidInput` names the offending field.
    pub fn validate(&self, base_diameter: f64) -> Result<(), Error> {
        positive("motor.exit_diameter", self.exit_diameter)?;
        if self.exit_diameter > 0.95 * base_diameter {
            return invalid("motor.exit_diameter", &format!("must be <= 0.95 of the base diameter {base_diameter}, got {}", self.exit_diameter));
        }
        if !(1.0..=5.0).contains(&self.exit_mach) {
            return invalid("motor.exit_mach", &format!("must be in [1, 5], got {}", self.exit_mach));
        }
        positive("motor.exit_pressure_pa", self.exit_pressure_pa)?;
        if !(200.0..=4000.0).contains(&self.total_temperature_k) {
            return invalid("motor.total_temperature_k", &format!("must be in [200, 4000] K, got {}", self.total_temperature_k));
        }
        Ok(())
    }
}

impl BodySpec {
    /// Largest diameter of the body \[m\].
    pub fn reference_diameter(&self) -> f64 {
        self.aft.iter().fold(self.nose.diameter, |d, a| match *a {
            AftSpec::Transition { aft_diameter, .. } => d.max(aft_diameter),
            AftSpec::Tube { .. } => d,
        })
    }

    /// Diameter of the flat base \[m\]: the diameter after the last aft section (the nose diameter when only tubes follow the nose).
    pub fn base_diameter(&self) -> f64 {
        self.aft.iter().fold(self.nose.diameter, |d, a| match *a {
            AftSpec::Transition { aft_diameter, .. } => aft_diameter,
            AftSpec::Tube { .. } => d,
        })
    }

    /// Validate the body and convert it to the internal body (lengths divided by the nose diameter) and its [`Scale`].
    #[doc(hidden)]
    pub fn to_internal(&self) -> Result<(BaseBody, Scale), Error> {
        let n = &self.nose;
        positive("nose.length", n.length)?;
        positive("nose.diameter", n.diameter)?;
        let fineness = n.length / n.diameter;
        if !(0.5 * (1.0 - 1e-9)..=12.0 * (1.0 + 1e-9)).contains(&fineness) {
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
