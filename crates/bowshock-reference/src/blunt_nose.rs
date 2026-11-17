//! Spherically blunted nose tips: the geometry of a sharp nose profile whose tip is replaced by a sphere tangent to the profile
//! (G. M. Crowell, *The Descriptive Geometry of Nose Cones*, 1996, the spherically blunted cone and ogive).
//!
//! Take a sharp nose of length `L` (apex at `x = 0`, base radius `R`) with profile `r(x)`. A sphere of radius `r_n` centred on the
//! axis at `x_c` is tangent to the profile at `x_t` when the profile's normal there passes through the centre, that is
//!
//! * `r(x_t) r'(x_t) = x_c - x_t` (the normal meets the axis at `x_c`), and
//! * `r(x_t)^2 + (x_t - x_c)^2 = r_n^2` (the point is on the sphere).
//!
//! Eliminating `x_c`, the tangent point solves `g(x_t) = r_n` with `g(x) = r(x) sqrt(1 + r'(x)^2)` (the distance from a profile
//! point to the axis along the normal), and then `x_c = x_t + r r'`. Ahead of `x_t` the nose is the sphere, behind it the original
//! profile; the value and the slope are continuous at `x_t` by construction. The new apex is the sphere's front, `x_c - r_n`, so
//! the blunted nose is shorter than the sharp one by `x_c - r_n`.
//!
//! A nose is specified by the length it is *built* to (sphere apex to base): [`solve`] finds the sharp length `L` whose blunting
//! has exactly that length. For a cone everything is closed form ([`cone`]); every other family is solved numerically.

use std::cell::RefCell;
use std::collections::HashMap;

/// The largest tip radius a nose takes, as a fraction of its base radius.
pub const MAX_TIP_FRACTION: f64 = 0.9;

/// Largest relative step the profile's numerical slope uses (the stencil is 4th order, error about `STEP^4`).
const STEP: f64 = 1.0e-3;
/// Samples of `g` the tangent search scans for the first crossing of `r_n`.
const SCAN: usize = 64;
/// The scan stops this fraction short of the base: the slope stencil looks ahead of the station it is taken at.
const BASE_MARGIN: f64 = 4.0 * STEP;
/// Tolerance on the built length the outer solve for the sharp length reaches (relative).
const LENGTH_TOLERANCE: f64 = 1.0e-13;

/// A sharp nose profile blunted by a tangent sphere. All lengths are in the units the profile was given in; stations `x` are
/// measured from the *sharp* apex (use [`BluntNose::apex`] to get to stations from the blunted apex).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BluntNose {
    /// Length of the underlying sharp nose, apex to base.
    pub sharp_length: f64,
    /// Radius of the tip sphere.
    pub tip_radius: f64,
    /// Station of the tangent point.
    pub x_t: f64,
    /// Station of the sphere's centre.
    pub x_c: f64,
}

impl BluntNose {
    /// Station of the blunted nose's front (the sphere's apex): `x_c - r_n`. The nose is `apex()` shorter than the sharp one.
    pub fn apex(&self) -> f64 {
        self.x_c - self.tip_radius
    }

    /// Length of the blunted nose, sphere apex to base.
    pub fn length(&self) -> f64 {
        self.sharp_length - self.apex()
    }

    /// Station of the tangent point measured from the blunted apex.
    pub fn tangent_z(&self) -> f64 {
        self.x_t - self.apex()
    }

    /// Radius of the sphere at sharp station `x` (0 ahead of its apex).
    pub fn sphere(&self, x: f64) -> f64 {
        let d = x - self.x_c;
        (self.tip_radius * self.tip_radius - d * d).max(0.0).sqrt()
    }

    /// Slope `dr/dx` of the sphere at sharp station `x` (infinite at its apex).
    pub fn sphere_slope(&self, x: f64) -> f64 {
        (self.x_c - x) / self.sphere(x)
    }

    /// Radius of the blunted nose at `z` from its apex, given the sharp profile `sharp(x)` of length [`sharp_length`](Self::sharp_length).
    pub fn radius(&self, sharp: impl Fn(f64) -> f64, z: f64) -> f64 {
        if z <= 0.0 {
            return 0.0;
        }
        let x = (z + self.apex()).clamp(self.apex(), self.sharp_length);
        if x <= self.x_t {
            self.sphere(x)
        } else {
            sharp(x)
        }
    }
}

/// `(profile radius, slope)` at `x`, the slope from a 4th-order central difference with a step proportional to `x`.
fn radius_and_slope(profile: &dyn Fn(f64) -> f64, x: f64) -> (f64, f64) {
    let h = STEP * x;
    let slope = (8.0 * (profile(x + h) - profile(x - h)) - (profile(x + 2.0 * h) - profile(x - 2.0 * h))) / (12.0 * h);
    (profile(x), slope)
}

/// Distance from the profile point at `x` to the axis along the profile's normal: `r sqrt(1 + r'^2)`.
fn reach(profile: &dyn Fn(f64) -> f64, x: f64) -> f64 {
    let (r, s) = radius_and_slope(profile, x);
    r * (1.0 + s * s).sqrt()
}

/// The tangent sphere of radius `tip_radius` on the sharp `profile` of length `length`: its tangent station and centre, solved
/// by bisection on `g(x_t) = r_n`. `None` when there is none: the first sample of `g` (at the tip) is already beyond `r_n`
/// (profiles with an infinite or too steep tip slope), `g` never reaches `r_n`, or the sphere would stand ahead of the tip.
pub fn tangency(profile: &dyn Fn(f64) -> f64, length: f64, tip_radius: f64) -> Option<BluntNose> {
    if !(length > 0.0 && tip_radius > 0.0) {
        return None;
    }
    let x_max = length * (1.0 - BASE_MARGIN);
    let station = |i: usize| if i == 0 { 1.0e-9 * length } else { x_max * (i as f64 / SCAN as f64).powi(2) };
    if reach(profile, station(0)) >= tip_radius {
        return None;
    }
    let i = (1..=SCAN).find(|&i| reach(profile, station(i)) >= tip_radius)?;
    let (mut lo, mut hi) = (station(i - 1), station(i));
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if reach(profile, mid) < tip_radius {
            lo = mid;
        } else {
            hi = mid;
        }
        if hi - lo <= 1.0e-15 * length {
            break;
        }
    }
    let x_t = 0.5 * (lo + hi);
    let (r, s) = radius_and_slope(profile, x_t);
    let x_c = x_t + r * s;
    (x_c - tip_radius >= 0.0).then_some(BluntNose { sharp_length: length, tip_radius, x_t, x_c })
}

/// The blunted cone built to `built_length` (sphere apex to base) with base radius `base_radius`, in closed form.
///
/// A cone of half-angle `a` (`sin a = R / sqrt(R^2 + L^2)`) is touched by a sphere at `x_c = r_n / sin a`, `x_t = x_c - r_n sin a`
/// (the tangent point's radius is `r_n cos a`), and the apex is cut by `x_c - r_n = r_n (1/sin a - 1)`. The built length
/// `L_b = L - r_n (sqrt(R^2 + L^2)/R - 1)` solves for the sharp length as
/// `L = (c + k sqrt(c^2 + R^2 (1 - k^2))) / (1 - k^2)` with `c = L_b - r_n` and `k = r_n / R < 1`.
pub fn cone(built_length: f64, base_radius: f64, tip_radius: f64) -> Option<BluntNose> {
    let k = tip_radius / base_radius;
    if !(built_length > 0.0 && base_radius > 0.0 && tip_radius > 0.0 && k < 1.0) {
        return None;
    }
    let c = built_length - tip_radius;
    let sharp = (c + k * (c * c + base_radius * base_radius * (1.0 - k * k)).sqrt()) / (1.0 - k * k);
    let sin_a = base_radius / base_radius.hypot(sharp);
    let x_c = tip_radius / sin_a;
    Some(BluntNose { sharp_length: sharp, tip_radius, x_t: x_c - tip_radius * sin_a, x_c })
}

/// The blunted nose built to `built_length` for the sharp family `sharp(l, x)` (radius at station `x` of a sharp nose of length
/// `l`): the sharp length `l` whose tangent sphere of radius `tip_radius` leaves exactly `built_length` (`l - apex`), found by
/// bisection on `l` (a length at which no tangent sphere exists counts as too short). `None` when there is no such nose.
pub fn solve(sharp: &dyn Fn(f64, f64) -> f64, built_length: f64, tip_radius: f64) -> Option<BluntNose> {
    if !(built_length > 0.0 && tip_radius > 0.0) {
        return None;
    }
    let at = |l: f64| tangency(&|x| sharp(l, x), l, tip_radius);
    let excess = |l: f64| at(l).map_or(f64::NEG_INFINITY, |b| b.length() - built_length);
    let mut hi = 2.0 * built_length;
    let mut doublings = 0;
    while excess(hi) < 0.0 {
        hi *= 2.0;
        doublings += 1;
        if doublings > 60 {
            return None;
        }
    }
    let mut lo = hi / 2.0;
    while hi - lo > LENGTH_TOLERANCE * hi {
        let mid = 0.5 * (lo + hi);
        if excess(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    // At a length where the tangent sphere first exists the built length jumps (a profile with a blunt tip): that is no root.
    at(hi).filter(|b| (b.length() - built_length).abs() <= 1.0e-9 * built_length)
}

/// What a nose is, for the memo tables: a key made of the family tag, its parameter, and the numbers of the solve.
pub type Key = [u64; 5];

thread_local! {
    static BLUNT: RefCell<HashMap<Key, Option<BluntNose>>> = RefCell::new(HashMap::new());
    static RANGE: RefCell<HashMap<Key, Option<(f64, f64)>>> = RefCell::new(HashMap::new());
}

/// Entries a memo table holds before it is emptied.
const MEMO_LIMIT: usize = 512;

fn memo<V: Copy>(table: &'static std::thread::LocalKey<RefCell<HashMap<Key, V>>>, key: Key, compute: impl FnOnce() -> V) -> V {
    if let Some(v) = table.with(|t| t.borrow().get(&key).copied()) {
        return v;
    }
    let v = compute();
    table.with(|t| {
        let mut t = t.borrow_mut();
        if t.len() >= MEMO_LIMIT {
            t.clear();
        }
        t.insert(key, v);
    });
    v
}

/// [`solve`] remembered per `key` (a nose is looked up many times per frame, a solve costs about a millisecond). The key must
/// identify `sharp`, `built_length` and `tip_radius` completely; `cone_base_radius` makes it the closed-form [`cone`] instead.
pub fn solve_cached(key: Key, sharp: &dyn Fn(f64, f64) -> f64, built_length: f64, tip_radius: f64, cone_base_radius: Option<f64>) -> Option<BluntNose> {
    memo(&BLUNT, key, || match cone_base_radius {
        Some(base_radius) => cone(built_length, base_radius, tip_radius),
        None => solve(sharp, built_length, tip_radius),
    })
}

/// The tip radii a nose of base radius `base_radius` accepts, `(smallest, largest)`: the largest is `MAX_TIP_FRACTION *
/// base_radius`; the smallest is 0 for a profile that comes to a point (any sphere is tangent somewhere) and the least tangent
/// sphere for one whose tip is already blunt (an ellipse has none smaller than its tip's radius of curvature). `None` when no
/// positive tip radius has a tangent sphere. `blunted(r_n)` is the solve for tip radius `r_n`; the result is remembered per `key`.
pub fn tip_range(key: Key, base_radius: f64, blunted: impl Fn(f64) -> Option<BluntNose>) -> Option<(f64, f64)> {
    memo(&RANGE, key, || {
        let max = MAX_TIP_FRACTION * base_radius;
        blunted(max)?;
        let floor = 1.0e-4 * base_radius;
        if blunted(floor).is_some() {
            return Some((0.0, max));
        }
        let (mut lo, mut hi) = (floor, max);
        while hi - lo > 1.0e-9 * base_radius {
            let mid = 0.5 * (lo + hi);
            if blunted(mid).is_some() {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        Some((hi, max))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Cone profile of length `l` and base radius `R = 0.05`.
    fn cone_profile(l: f64, x: f64) -> f64 {
        0.05 * (x / l).clamp(0.0, 1.0)
    }

    /// Tangent ogive profile (`rho = (R^2 + L^2) / (2R)`).
    fn ogive_profile(l: f64, x: f64) -> f64 {
        let r = 0.05;
        let rho = (r * r + l * l) / (2.0 * r);
        ((rho * rho - (l - x) * (l - x)).max(0.0)).sqrt() + r - rho
    }

    #[test]
    fn cone_closed_form_matches_the_defining_equations() {
        let (built, r, rn) = (0.3, 0.05, 0.02);
        let b = cone(built, r, rn).unwrap();
        // the built length is what was asked
        assert!((b.length() - built).abs() < 1e-12, "{b:?}");
        // x_c, x_t from the half-angle
        let sin_a = r / r.hypot(b.sharp_length);
        assert!((b.x_c - rn / sin_a).abs() < 1e-12);
        assert!((b.x_t - (b.x_c - rn * sin_a)).abs() < 1e-12);
        // the sphere meets the cone with equal value and slope at x_t
        let tan_a = r / b.sharp_length;
        assert!((b.sphere(b.x_t) - tan_a * b.x_t).abs() < 1e-12);
        assert!((b.sphere_slope(b.x_t) - tan_a).abs() < 1e-12);
    }

    #[test]
    fn numeric_solve_agrees_with_the_cone_closed_form() {
        let exact = cone(0.3, 0.05, 0.02).unwrap();
        let numeric = solve(&cone_profile, 0.3, 0.02).unwrap();
        assert!((numeric.sharp_length - exact.sharp_length).abs() < 1e-9, "{numeric:?} {exact:?}");
        assert!((numeric.x_c - exact.x_c).abs() < 1e-9);
        assert!((numeric.x_t - exact.x_t).abs() < 1e-9);
    }

    #[test]
    fn the_ogive_tangent_sphere_has_equal_value_and_slope() {
        let b = solve(&ogive_profile, 0.25, 0.012).unwrap();
        assert!((b.length() - 0.25).abs() < 1e-11);
        let profile = |x| ogive_profile(b.sharp_length, x);
        let h = 1e-7;
        let slope = (profile(b.x_t + h) - profile(b.x_t - h)) / (2.0 * h);
        assert!((b.sphere(b.x_t) - profile(b.x_t)).abs() < 1e-9);
        assert!((b.sphere_slope(b.x_t) - slope).abs() < 1e-6);
    }

    #[test]
    fn a_blunt_ellipse_tip_needs_a_larger_sphere() {
        let ellipse = |l: f64, x: f64| 0.05 * (1.0 - (1.0 - x / l) * (1.0 - x / l)).max(0.0).sqrt();
        // the ellipse's tip radius of curvature is R^2 / L: 0.05^2 / 0.15 = 16.7 mm; a 5 mm sphere has no tangent point
        assert!(solve(&ellipse, 0.15, 0.005).is_none());
        assert!(solve(&ellipse, 0.15, 0.03).is_some());
        // a half-power series has an infinite tip slope and a tip radius of curvature R^2 / (2 L); n < 1/2 never has a tangent sphere
        let power = |n: f64| move |l: f64, x: f64| 0.05 * (x / l).clamp(0.0, 1.0).powf(n);
        assert!(solve(&power(0.3), 0.15, 0.03).is_none());
    }

    #[test]
    fn tip_range_finds_the_smallest_sphere_of_an_ellipse() {
        let ellipse = |l: f64, x: f64| 0.05 * (1.0 - (1.0 - x / l) * (1.0 - x / l)).max(0.0).sqrt();
        let (lo, hi) = tip_range([1, 0, 0, 0, 0], 0.05, |rn| solve(&ellipse, 0.15, rn)).unwrap();
        assert!((hi - 0.045).abs() < 1e-12);
        assert!(solve(&ellipse, 0.15, lo * 1.001).is_some() && solve(&ellipse, 0.15, lo * 0.99).is_none(), "{lo}");
        // a cone has no smallest
        let (lo, _) = tip_range([2, 0, 0, 0, 0], 0.05, |rn| cone(0.3, 0.05, rn)).unwrap();
        assert_eq!(lo, 0.0);
    }
}
