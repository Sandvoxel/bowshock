//! The outline of a nose cone, with its optional spherical tip.
//!
//! A nose cone is a sharp profile `r(x)` of one of Crowell's families ([`NoseShape`]) or that profile with its tip replaced by a
//! sphere of radius `tip_radius` tangent to it (Crowell, *The Descriptive Geometry of Nose Cones*, 1996, the spherically blunted
//! cone and ogive; the construction and its solve are in [`bowshock_reference::blunt_nose`]). The part's `length` is the length of
//! the nose *as built*, from the sphere's apex to the base, so the length the user types is the length they measure; the sharp
//! profile the sphere sits on is longer by the cut-off and is derived from it.
//!
//! [`NoseProfile`] is the one sampler of that outline: [`PartKind::radius_at`], [`PartKind::profile`] (so the viewport, the
//! Station Lanes silhouette, picking and the printed templates), the mass integration, the aerodynamic body segment and the CFD
//! body all read the outline through it (or through the [`BluntNose`] it holds), never from the families' closed forms directly.

use bowshock_reference::blunt_nose::{self, BluntNose, Key, MAX_TIP_FRACTION};

use crate::geometry::{nose_radius, NoseShape, PartKind};

/// Points on the sphere of a blunted nose's sampled [`profile`](NoseProfile::profile) (segments), on top of the usual 32 along the
/// body of the nose.
const CAP_SEGMENTS: usize = 8;
/// Segments along the nose, from the tangent point to the base (the sharp nose has this many from tip to base).
const BODY_SEGMENTS: usize = 32;

/// The memo key of a nose: the family and its parameter, the built length, the base radius and the tip radius.
fn key(shape: NoseShape, length: f64, base_radius: f64, tip_radius: f64) -> Key {
    let (tag, parameter) = match shape {
        NoseShape::Conical => (0, 0.0),
        NoseShape::TangentOgive => (1, 0.0),
        NoseShape::VonKarman => (2, 0.0),
        NoseShape::Elliptical => (3, 0.0),
        NoseShape::Parabolic => (4, 0.0),
        NoseShape::PowerSeries(n) => (5, n),
        NoseShape::Haack(c) => (6, c),
    };
    [tag, parameter.to_bits(), length.to_bits(), base_radius.to_bits(), tip_radius.to_bits()]
}

/// The tangent sphere of radius `tip_radius` on a nose of `shape` built `length` long with base radius `base_radius`: `None` for a
/// pointed nose (zero or negative radius) and when no sphere of that radius is tangent to the family (see
/// [`PartKind::tip_radius_limits`]). A cone is solved in closed form, every other family numerically.
pub fn blunting(shape: NoseShape, length: f64, base_radius: f64, tip_radius: f64) -> Option<BluntNose> {
    if !(tip_radius > 0.0 && length > 0.0 && base_radius > 0.0) || tip_radius > MAX_TIP_FRACTION * base_radius * (1.0 + 1.0e-9) {
        return None;
    }
    let sharp = |l: f64, x: f64| nose_radius(shape, l, base_radius, x);
    blunt_nose::solve_cached(key(shape, length, base_radius, tip_radius), &sharp, length, tip_radius, (shape == NoseShape::Conical).then_some(base_radius))
}

/// The tip radii a nose accepts, `(smallest positive, largest)`, `None` when none has a tangent sphere.
fn tip_range(shape: NoseShape, length: f64, base_radius: f64) -> Option<(f64, f64)> {
    if !(length > 0.0 && base_radius > 0.0) {
        return None;
    }
    blunt_nose::tip_range(key(shape, length, base_radius, 0.0), base_radius, |rn| blunting(shape, length, base_radius, rn))
}

/// The outline of a nose cone.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NoseProfile {
    shape: NoseShape,
    length: f64,
    base_radius: f64,
    blunt: Option<BluntNose>,
}

impl NoseProfile {
    /// The outline of a nose of `shape`, `length` long (as built), `diameter` across its base, with a spherical tip of radius
    /// `tip_radius` (0 is pointed). A tip radius that has no tangent sphere on the family gives the pointed outline.
    pub fn new(shape: NoseShape, length: f64, diameter: f64, tip_radius: f64) -> NoseProfile {
        let base_radius = diameter / 2.0;
        NoseProfile { shape, length, base_radius, blunt: blunting(shape, length, base_radius, tip_radius) }
    }

    /// The tip sphere, `None` for a pointed nose.
    pub fn blunt(&self) -> Option<&BluntNose> {
        self.blunt.as_ref()
    }

    /// Length of the sharp nose the tip sphere sits on (the built length when pointed).
    pub fn sharp_length(&self) -> f64 {
        self.blunt.map_or(self.length, |b| b.sharp_length)
    }

    /// Outer radius `z` from the apex (clamped to the nose).
    pub fn radius(&self, z: f64) -> f64 {
        if self.length <= 0.0 {
            return self.base_radius;
        }
        let z = z.clamp(0.0, self.length);
        match self.blunt {
            None => nose_radius(self.shape, self.length, self.base_radius, z),
            Some(b) => b.radius(|x| nose_radius(self.shape, b.sharp_length, self.base_radius, x), z).clamp(0.0, self.base_radius),
        }
    }

    /// The outline as `(z, radius)` from the apex to the base. A pointed nose is 33 points evenly along its length; a blunted one
    /// is the sphere in 8 equal steps of angle up to the tangent point and 32 segments evenly along the rest.
    pub fn profile(&self) -> Vec<(f64, f64)> {
        let (length, base) = (self.length, self.base_radius);
        let Some(b) = self.blunt else {
            return (0..=BODY_SEGMENTS)
                .map(|i| match i {
                    0 => (0.0, 0.0),
                    BODY_SEGMENTS => (length, base),
                    _ => {
                        let x = length * i as f64 / BODY_SEGMENTS as f64;
                        (x, nose_radius(self.shape, length, base, x))
                    }
                })
                .collect();
        };
        let z_t = b.tangent_z().min(length);
        let phi_t = b.sphere(b.x_t).atan2(b.x_c - b.x_t);
        let mut points: Vec<(f64, f64)> = (0..CAP_SEGMENTS)
            .map(|j| {
                let phi = phi_t * j as f64 / CAP_SEGMENTS as f64;
                (b.tip_radius * (1.0 - phi.cos()), b.tip_radius * phi.sin())
            })
            .collect();
        for i in 0..BODY_SEGMENTS {
            let z = z_t + (length - z_t) * i as f64 / BODY_SEGMENTS as f64;
            points.push((z, self.radius(z)));
        }
        points.push((length, base));
        points
    }
}

impl PartKind {
    /// The nose's outline (a nose's own `NoseProfile`); `None` for a tube or a transition.
    pub fn nose_profile(&self) -> Option<NoseProfile> {
        match *self {
            PartKind::NoseCone { shape, length, diameter, tip_radius } => Some(NoseProfile::new(shape, length, diameter, tip_radius)),
            _ => None,
        }
    }

    /// Radius of the nose's spherical tip as stored (0 for a pointed nose and for every other kind of part).
    pub fn tip_radius(&self) -> f64 {
        match *self {
            PartKind::NoseCone { tip_radius, .. } => tip_radius,
            _ => 0.0,
        }
    }

    /// This kind with its nose tip radius replaced (limited as [`clamp_tip_radius`](Self::clamp_tip_radius) does); a tube or
    /// a transition is unchanged.
    pub fn with_tip_radius(&self, tip_radius: f64) -> PartKind {
        let mut out = self.clone();
        if let PartKind::NoseCone { tip_radius: t, .. } = &mut out {
            *t = tip_radius;
        }
        out.sanitized_tip()
    }

    /// This kind with a nose tip radius that has no tangent sphere (or is out of range) brought into range, or to 0 when the
    /// family takes none.
    pub(crate) fn sanitized_tip(self) -> PartKind {
        let PartKind::NoseCone { tip_radius, .. } = self else { return self };
        if tip_radius == 0.0 {
            return self;
        }
        let v = self.clamp_tip_radius(tip_radius).map_or(0.0, |(v, _)| v);
        match self {
            PartKind::NoseCone { shape, length, diameter, .. } => PartKind::NoseCone { shape, length, diameter, tip_radius: v },
            other => other,
        }
    }

    /// The tip sphere of a nose whose stored radius has a tangent sphere; `None` for a pointed nose and every other kind of part.
    pub fn nose_blunt(&self) -> Option<BluntNose> {
        match *self {
            PartKind::NoseCone { shape, length, diameter, tip_radius } => blunting(shape, length, diameter / 2.0, tip_radius),
            _ => None,
        }
    }

    /// The tip radii this nose accepts: `(smallest positive, largest)`. The largest is [`MAX_TIP_FRACTION`] (0.9) of the base
    /// radius; the smallest is 0-ish for families that come to a point and the tip's own curvature for the ones whose tip is
    /// already round. `None` for a nose no sphere is tangent to (see [`tip_radius_refusal`](Self::tip_radius_refusal)) and for
    /// other parts.
    pub fn tip_radius_limits(&self) -> Option<(f64, f64)> {
        match *self {
            PartKind::NoseCone { shape, length, diameter, .. } => tip_range(shape, length, diameter / 2.0),
            _ => None,
        }
    }

    /// Why this nose takes no tip radius, `None` when it does (and for other parts). The families whose tip is already round (the
    /// ellipse) or has an infinite slope (power series, Haack, Von Karman) accept a sphere only where one is tangent to the
    /// profile: the sphere's radius cannot be smaller than the radius of curvature the profile's tip already has, and a profile
    /// that curves faster than that at its tip (power series with `n < 1/2`) has no tangent sphere at all.
    pub fn tip_radius_refusal(&self) -> Option<&'static str> {
        let PartKind::NoseCone { shape, .. } = *self else { return None };
        if self.tip_radius_limits().is_some() {
            return None;
        }
        Some(match shape {
            NoseShape::PowerSeries(n) if n < 0.5 => "A power series with n below 1/2 curves faster than any sphere at its tip: none is tangent to it.",
            NoseShape::Elliptical => "This ellipse's own tip is rounder than the largest tip sphere (0.9 of the base radius): none is tangent to it.",
            _ => "No sphere up to 0.9 of the base radius is tangent to this profile.",
        })
    }

    /// `value` limited to the tip radii this nose accepts, with the reason when it was limited. 0 (pointed) is always allowed
    /// where a tangent sphere exists; values between 0 and the smallest tangent sphere go up to it. `None` for a part that is not
    /// a nose.
    pub fn clamp_tip_radius(&self, value: f64) -> Option<(f64, Option<&'static str>)> {
        if !matches!(self, PartKind::NoseCone { .. }) {
            return None;
        }
        if value.is_nan() || value == 0.0 {
            return Some((0.0, None));
        }
        if value < 0.0 {
            return Some((0.0, Some("POINTED TIP")));
        }
        let Some((lo, hi)) = self.tip_radius_limits() else {
            return Some((0.0, Some("NO TANGENT SPHERE")));
        };
        Some(if value < lo * (1.0 - 1.0e-9) {
            (lo, Some("SMALLEST TANGENT SPHERE"))
        } else if value > hi * (1.0 + 1.0e-9) {
            (hi, Some("TIP LIMIT 0.9 × BASE RADIUS"))
        } else {
            (value, None)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Dim, Rocket};

    const R: f64 = 0.05;
    const D: f64 = 2.0 * R;

    fn nose(shape: NoseShape, length: f64, tip_radius: f64) -> PartKind {
        PartKind::NoseCone { shape, length, diameter: D, tip_radius }
    }

    /// The tip sphere of cone: tangent station, centre and built length against the closed form of a sphere on a cone of
    /// half-angle `a`: `x_c = r_n / sin a`, `x_t = x_c - r_n sin a`, built length `L - (x_c - r_n)` (Crowell).
    #[test]
    fn a_blunted_cone_has_the_closed_form_tangent_sphere() {
        let (built, rn) = (0.3, 0.02);
        let b = blunting(NoseShape::Conical, built, R, rn).unwrap();
        // the sharp cone of length L has built length L - r_n (sqrt(R^2 + L^2) / R - 1): solve it independently by bisection
        let built_of = |l: f64| l - rn * (R.hypot(l) / R - 1.0);
        let (mut lo, mut hi) = (built, 10.0 * built);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if built_of(mid) < built {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let l = 0.5 * (lo + hi);
        let sin_a = R / R.hypot(l);
        assert!((b.sharp_length - l).abs() < 1e-12, "{} vs {l}", b.sharp_length);
        assert!((b.x_c - rn / sin_a).abs() < 1e-12);
        assert!((b.x_t - (rn / sin_a - rn * sin_a)).abs() < 1e-12);
        assert!((b.length() - built).abs() < 1e-12);
        assert!((b.apex() - rn * (1.0 / sin_a - 1.0)).abs() < 1e-12, "the cut-off");
        // the numerical solve gives the same sphere
        let numeric = bowshock_reference::blunt_nose::solve(&|l, x| nose_radius(NoseShape::Conical, l, R, x), built, rn).unwrap();
        assert!((numeric.x_t - b.x_t).abs() < 1e-9 && (numeric.x_c - b.x_c).abs() < 1e-9);
    }

    /// The outline is C1 where the sphere meets the profile: same radius and same slope, for the cone, the tangent ogive and the
    /// Von Karman nose.
    #[test]
    fn the_blunted_profile_is_c1_at_the_tangent_point() {
        for (shape, rn) in [(NoseShape::Conical, 0.02), (NoseShape::TangentOgive, 0.02), (NoseShape::VonKarman, 0.015), (NoseShape::Haack(1.0 / 3.0), 0.01)] {
            let b = blunting(shape, 0.3, R, rn).unwrap();
            let sharp = |x: f64| nose_radius(shape, b.sharp_length, R, x);
            assert!((b.sphere(b.x_t) - sharp(b.x_t)).abs() < 1e-9, "{shape:?} value");
            let h = 1e-7;
            let slope = (sharp(b.x_t + h) - sharp(b.x_t - h)) / (2.0 * h);
            assert!((b.sphere_slope(b.x_t) - slope).abs() < 1e-6, "{shape:?} slope {} vs {slope}", b.sphere_slope(b.x_t));
            // and the sampled outline passes through it without a jump
            let p = NoseProfile::new(shape, 0.3, D, rn);
            let z_t = b.tangent_z();
            let (before, after) = (p.radius(z_t - 1e-9), p.radius(z_t + 1e-9));
            assert!((before - after).abs() < 1e-8, "{shape:?}");
            // the nose is as long as asked and ends at the base radius
            assert!((p.radius(0.3) - R).abs() < 1e-12 && p.radius(0.0) == 0.0);
            assert!((b.length() - 0.3).abs() < 1e-11);
        }
    }

    /// Without a tip radius everything is the sharp nose exactly: the same radius at every station and the same 33-point outline,
    /// for every family.
    #[test]
    fn no_tip_radius_is_the_sharp_nose_exactly() {
        for shape in NoseShape::LIST {
            let k = nose(shape, 0.3, 0.0);
            let p = k.profile();
            assert_eq!(p.len(), 33, "{shape:?}");
            for (i, &(z, r)) in p.iter().enumerate() {
                let want = match i {
                    0 => 0.0,
                    32 => R,
                    _ => nose_radius(shape, 0.3, R, 0.3 * i as f64 / 32.0),
                };
                assert_eq!((z, r), (if i == 32 { 0.3 } else { 0.3 * i as f64 / 32.0 }, want), "{shape:?} {i}");
            }
            for i in 0..=40 {
                let z = 0.3 * f64::from(i) / 40.0;
                assert_eq!(k.radius_at(z), nose_radius(shape, 0.3, R, z), "{shape:?} {z}");
            }
            assert_eq!(k.nose_blunt(), None);
        }
    }

    /// The volume of a blunted cone is a frustum plus a spherical cap, integrated through the one profile sampler.
    #[test]
    fn the_volume_of_a_blunted_cone_is_frustum_plus_cap() {
        let (built, rn) = (0.3, 0.02);
        let p = NoseProfile::new(NoseShape::Conical, built, D, rn);
        let n = 200_000;
        let dz = built / f64::from(n);
        let volume: f64 = (0..n).map(|i| std::f64::consts::PI * p.radius((f64::from(i) + 0.5) * dz).powi(2) * dz).sum();
        let b = p.blunt().unwrap();
        let sin_a = R / R.hypot(b.sharp_length);
        let (r_t, h) = (rn * (1.0 - sin_a * sin_a).sqrt(), rn * (1.0 - sin_a));
        let frustum = std::f64::consts::PI * (b.sharp_length - b.x_t) / 3.0 * (R * R + R * r_t + r_t * r_t);
        let cap = std::f64::consts::PI * h * h * (3.0 * rn - h) / 3.0;
        assert!((volume - (frustum + cap)).abs() < 1e-7 * volume, "{volume} vs {}", frustum + cap);
    }

    /// A tip radius no sphere fits is refused with its reason: a power series with `n < 1/2` curves faster than any sphere; an
    /// ellipse takes only spheres at least as round as its own tip.
    #[test]
    fn families_whose_tip_is_already_blunt_take_a_tip_radius_only_where_a_sphere_is_tangent() {
        let power = nose(NoseShape::PowerSeries(0.3), 0.3, 0.0);
        assert!(power.tip_radius_limits().is_none() && power.tip_radius_refusal().is_some());
        assert_eq!(power.clamp_tip_radius(0.01), Some((0.0, Some("NO TANGENT SPHERE"))));
        assert!(nose(NoseShape::PowerSeries(0.75), 0.3, 0.0).tip_radius_refusal().is_none());
        // an ellipse's tip radius of curvature is R^2 / L = 8.3 mm here: smaller spheres are raised to the smallest tangent one
        let ellipse = nose(NoseShape::Elliptical, 0.3, 0.0);
        let (lo, hi) = ellipse.tip_radius_limits().unwrap();
        assert!(lo > 0.0 && lo < R * R / 0.3 * 1.5 && (hi - 0.9 * R).abs() < 1e-12, "{lo} {hi}");
        assert_eq!(ellipse.clamp_tip_radius(0.001), Some((lo, Some("SMALLEST TANGENT SPHERE"))));
        assert_eq!(ellipse.clamp_tip_radius(0.0), Some((0.0, None)));
        // a pointed family takes any sphere up to 0.9 R
        let cone = nose(NoseShape::Conical, 0.3, 0.0);
        assert_eq!(cone.tip_radius_limits(), Some((0.0, 0.9 * R)));
        assert_eq!(cone.clamp_tip_radius(R), Some((0.9 * R, Some("TIP LIMIT 0.9 × BASE RADIUS"))));
        assert_eq!(cone.clamp_tip_radius(-1.0), Some((0.0, Some("POINTED TIP"))));
        assert!(PartKind::BodyTube { length: 0.3, diameter: D }.clamp_tip_radius(0.01).is_none());
    }

    /// The tip radius is a dimension like the others: limited with its reason, and re-limited when the nose changes under it.
    #[test]
    fn the_tip_radius_is_limited_when_edited_and_when_the_nose_changes() {
        let mut r = Rocket::default();
        let id = r.insert(0, nose(NoseShape::TangentOgive, 0.25, 0.0)).unwrap();
        let edit = r.set_dim(id, Dim::TipRadius, 1.0).unwrap();
        assert!((edit.applied - 0.9 * R).abs() < 1e-12 && edit.limited == Some("TIP LIMIT 0.9 × BASE RADIUS"), "{edit:?}");
        // a narrower base lowers the limit and the stored radius follows it
        assert!(r.set_dim(id, Dim::Diameter, 0.04).is_some());
        assert!((r.part(id).unwrap().kind.tip_radius() - 0.9 * 0.02).abs() < 1e-12);
        // the part's length stays the length as built whatever the tip does
        assert_eq!(r.part(id).unwrap().kind.length(), 0.25);
        // a family with no tangent sphere drops it
        assert!(r.set_kind(id, nose(NoseShape::PowerSeries(0.3), 0.25, 0.01)));
        assert_eq!(r.part(id).unwrap().kind.tip_radius(), 0.0);
        assert_eq!(r.set_dim(id, Dim::TipRadius, 0.01).map(|e| (e.applied, e.limited)), Some((0.0, Some("NO TANGENT SPHERE"))));
    }
}
