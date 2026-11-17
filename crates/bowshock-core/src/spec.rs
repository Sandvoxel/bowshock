//! Dimensional body description (metres) and its conversion to the internal body (lengths in nose diameters).

use bowshock_reference::blunt_nose::MAX_TIP_FRACTION;

use crate::engine::body_grid::{BaseBody, Body, Family, Nose, Segment};
use crate::error::Error;

/// Longest accepted body, in nose diameters. The cell count along every tube and cone follows its length (a cell is at most one nose
/// diameter long at the coarsest grid level), the domain scales with the body length and no solver array or device limit is reached, so
/// what bounds the length is what has been solved. The two-stage rocket of `tests/long_body.rs` (tangent ogive of fineness 5, a 54 mm
/// booster on a 38.1 mm sustainer) was run with RANS on the coarse grid on the CPU at Mach 0.5 and 2 at 40, 46, 80 and 100 diameters
/// (and the Euler sting grid at 40 and 80): every run reaches a stationary drag, in 4500 to 8000 iterations (at 100 diameters 5500 at
/// Mach 0.5; 15000 at Mach 2, where the drag then wobbles by 0.1 % and the point carries `Warning::OscillatingDrag`); the skin friction
/// matches the flat-plate estimate over the wetted area to 3 %; the forebody pressure drag does not change with the length; doubling the
/// axial cell count changes the drag by 0.2 % at 40 as at 80 diameters. The limit was 40 only because the first version of the
/// validation set it, with no reason recorded. Past 100 diameters nothing has been run.
const MAX_LENGTH_DIAMETERS: f64 = 100.0;
/// Steepest accepted nose cone / transition half-angle.
const MAX_HALF_ANGLE: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// Sharp nose profile family: [`NoseShape`] without its blunted variant, the profile a tip sphere sits on.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SharpShape {
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
    /// A sharp nose with its tip replaced by a sphere tangent to the profile (Crowell, *The Descriptive Geometry of Nose Cones*,
    /// 1996; `bowshock_reference::blunt_nose`). The [`NoseSpec::length`] is then the nose as built, sphere apex to base; the
    /// sharp profile the sphere sits on is longer. At most 0.9 of the base radius, and only where a sphere of that radius is
    /// tangent to the profile.
    Blunted {
        /// The sharp profile.
        base: SharpShape,
        /// Radius of the tip sphere \[m\].
        tip_radius: f64,
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
    /// Sharp edges, hexagonal (modified double wedge) section: a wedge of chordwise length `wedge_fraction` of the chord at the
    /// leading and at the trailing edge with a flat mid-section (supersonic wave drag). `0 < wedge_fraction <= 0.5`; 0.5 is the
    /// double wedge.
    Hexagonal {
        /// Length of each wedge as a fraction of the chord.
        wedge_fraction: f64,
    },
    /// Sharp edges, biconvex (circular-arc) section (supersonic wave drag).
    Biconvex,
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
    /// Distance the root trailing edge lies ahead of the body base \[m\]; 0 is flush with the base. Used by the 3D mesher
    /// to place the fins; the axisymmetric solver ignores it.
    #[cfg_attr(feature = "serde", serde(default))]
    pub root_te_to_base: f64,
    /// Edge profile.
    pub profile: FinProfile,
}

impl FinSpec {
    /// Check the fin set; the error names the offending field as `fins.<field>`.
    pub fn validate(&self) -> Result<(), Error> {
        self.validate_for(crate::analysis::Engine::Legacy)
    }

    /// [`FinSpec::validate`] for `engine`: the legacy engine takes 3 to 8 fins (its empirical fin-drag model and its sting-grid fin layout), the 3D engine 1 to 16
    /// (the sector and the half domain build any count).
    #[doc(hidden)]
    pub fn validate_for(&self, engine: crate::analysis::Engine) -> Result<(), Error> {
        let counts = match engine {
            crate::analysis::Engine::Legacy => 3..=8,
            crate::analysis::Engine::Solver3 => 1..=16,
        };
        if !counts.contains(&self.count) {
            return invalid("fins.count", &format!("must be in {}..={}, got {}", counts.start(), counts.end(), self.count));
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
        if !(self.root_te_to_base.is_finite() && self.root_te_to_base >= 0.0) {
            return invalid("fins.root_te_to_base", &format!("must be finite and >= 0, got {}", self.root_te_to_base));
        }
        if !(self.sweep.is_finite() && (self.sweep / self.span).atan().abs() <= 75f64.to_radians()) {
            return invalid("fins.sweep", &format!("must be finite with a leading-edge sweep angle of at most 75 degrees, got {}", self.sweep));
        }
        match &self.profile {
            FinProfile::Custom { section } => validate_section(section)?,
            FinProfile::Hexagonal { wedge_fraction } if !(*wedge_fraction > 0.0 && *wedge_fraction <= 0.5) => {
                return invalid("fins.profile.wedge_fraction", &format!("must be in (0, 0.5], got {wedge_fraction}"));
            }
            _ => {}
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
        let (base, tip) = match n.shape {
            NoseShape::Cone => (SharpShape::Cone, 0.0),
            NoseShape::TangentOgive => (SharpShape::TangentOgive, 0.0),
            NoseShape::Power { n } => (SharpShape::Power { n }, 0.0),
            NoseShape::Parabolic { k } => (SharpShape::Parabolic { k }, 0.0),
            NoseShape::Haack { c } => (SharpShape::Haack { c }, 0.0),
            NoseShape::Blunted { base, tip_radius } => (base, tip_radius),
        };
        let family = match base {
            SharpShape::Cone => Family::Cone,
            SharpShape::TangentOgive => Family::TangentOgive,
            SharpShape::Power { n } => {
                if !(0.5..=1.0).contains(&n) {
                    return invalid("nose.shape.n", &format!("must be in [0.5, 1], got {n}"));
                }
                Family::Power { n }
            }
            SharpShape::Parabolic { k } => {
                if !(0.0..=1.0).contains(&k) {
                    return invalid("nose.shape.k", &format!("must be in [0, 1], got {k}"));
                }
                Family::Parabolic { k }
            }
            SharpShape::Haack { c } => {
                if !(0.0..=1.0 / 3.0).contains(&c) {
                    return invalid("nose.shape.c", &format!("must be in [0, 1/3], got {c}"));
                }
                Family::Haack { c }
            }
        };
        // The cone's half-angle is that of the sharp cone: with a tip sphere the sharp nose is longer than the one as built.
        let check_cone = |sharp_length: f64| -> Result<(), Error> {
            let half_angle = (0.5 * n.diameter / sharp_length).atan();
            if family == Family::Cone && half_angle > MAX_HALF_ANGLE {
                return invalid("nose.shape", &format!("cone half-angle {:.1} deg exceeds 30 deg", half_angle.to_degrees()));
            }
            Ok(())
        };
        let nose = if tip == 0.0 {
            check_cone(n.length)?;
            family.nose(fineness)
        } else {
            if !(tip.is_finite() && tip > 0.0 && tip <= MAX_TIP_FRACTION * 0.5 * n.diameter * (1.0 + 1e-9)) {
                return invalid("nose.shape.tip_radius", &format!("must be in (0, {MAX_TIP_FRACTION} of the base radius], got {tip}"));
            }
            let Some(blunt) = family.blunted(fineness, tip / n.diameter) else {
                return invalid("nose.shape.tip_radius", "no sphere of that radius is tangent to the nose profile");
            };
            if let Nose::Blunt { tip: sphere, .. } = blunt {
                check_cone(sphere.sharp_length * n.diameter)?;
            }
            blunt
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

    fn blunted(base: SharpShape, tip_radius: f64) -> BodySpec {
        BodySpec { nose: NoseSpec { shape: NoseShape::Blunted { base, tip_radius }, length: 0.3, diameter: 0.1 }, aft: vec![AftSpec::Tube { length: 0.6 }] }
    }

    /// The solved nose keeps the length as built (the sharp nose behind the sphere is longer), starts at a point, ends at the base
    /// radius, and is C1 where the sphere meets the profile.
    #[test]
    fn a_blunted_nose_keeps_its_built_length_and_is_c1() {
        for base in [SharpShape::Cone, SharpShape::TangentOgive, SharpShape::Haack { c: 0.0 }, SharpShape::Power { n: 0.75 }] {
            let (b, s) = blunted(base, 0.02).to_internal().unwrap();
            assert_eq!(s.d_nose, 0.1);
            assert!((b.body.nose_length() - 3.0).abs() < 1e-11, "{base:?}: {}", b.body.nose_length());
            assert!((b.x_base() - 9.0).abs() < 1e-11);
            let Nose::Blunt { tip, .. } = b.body.nose else { panic!("{base:?}: not blunted") };
            assert!(tip.sharp_length > 3.0 && (tip.tip_radius - 0.2).abs() < 1e-12);
            assert_eq!(b.body.r(0.0), 0.0);
            assert!((b.body.r(3.0) - 0.5).abs() < 1e-12);
            // value and slope agree across the tangent point
            let z_t = tip.tangent_z();
            let h = 1e-6;
            let (before, after) = (b.body.r(z_t - h), b.body.r(z_t + h));
            let (slope_before, slope_after) = (b.body.slope(z_t - h), b.body.slope(z_t + h));
            assert!((after - before - 0.5 * (slope_before + slope_after) * 2.0 * h).abs() < 1e-9, "{base:?}");
            assert!((slope_before - slope_after).abs() < 1e-4, "{base:?}: {slope_before} vs {slope_after}");
            // the tip is round: the half-angle there is 90 degrees
            assert!((b.body.tip_half_angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-3);
        }
    }

    /// A blunted cone against the closed form: `r(x) = R (x + cut) / L` behind the tangent point on the sharp cone of length `L`.
    #[test]
    fn a_blunted_cone_follows_the_sharp_cone_behind_the_tangent_point() {
        let (b, _) = blunted(SharpShape::Cone, 0.02).to_internal().unwrap();
        let (built, k, rn) = (3.0_f64, 0.4_f64, 0.2_f64);
        let c = built - rn;
        let sharp = (c + k * (c * c + 0.25 * (1.0 - k * k)).sqrt()) / (1.0 - k * k);
        let cut = rn * ((0.5_f64).hypot(sharp) / 0.5 - 1.0);
        for x in [1.0, 1.5, 2.0, 2.9] {
            assert!((b.body.r(x) - 0.5 * (x + cut) / sharp).abs() < 1e-12, "x {x}");
        }
    }

    #[test]
    fn a_tip_radius_no_sphere_fits_is_refused() {
        // more than 0.9 of the base radius
        let err = blunted(SharpShape::Cone, 0.049).to_internal().unwrap_err().to_string();
        assert!(err.contains("tip_radius"), "{err}");
        assert!(blunted(SharpShape::Cone, f64::NAN).to_internal().is_err());
        assert!(blunted(SharpShape::Cone, -0.01).to_internal().is_err());
        // n < 1/2 is not a family the solver takes; n = 1/2 has a tip radius of curvature R^2/(2 L): a smaller sphere is not tangent
        assert!(blunted(SharpShape::Power { n: 0.4 }, 0.02).to_internal().is_err());
        assert!(blunted(SharpShape::Power { n: 0.5 }, 0.001).to_internal().is_err());
        // zero is the sharp nose, as `Cone` is
        let (zero, _) = blunted(SharpShape::TangentOgive, 0.0).to_internal().unwrap();
        let (sharp, _) =
            BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, ..spec().nose }, aft: vec![AftSpec::Tube { length: 0.6 }] }.to_internal().unwrap();
        assert!(matches!(zero.body.nose, Nose::TangentOgive { .. }) && zero.body.r(1.5) == sharp.body.r(1.5));
        // the cone limit is the sharp cone's half-angle
        let steep = BodySpec {
            nose: NoseSpec { shape: NoseShape::Blunted { base: SharpShape::Cone, tip_radius: 0.0 }, length: 0.05, diameter: 0.1 },
            aft: vec![AftSpec::Tube { length: 0.5 }],
        };
        assert!(steep.to_internal().unwrap_err().to_string().contains("half-angle"));
    }
}
