//! The smallest valid domain of a finned body (S12): from the azimuths of every fin of every fin set, the symmetry of the whole configuration and the domain
//! that exploits it. A fin set is the fins of one geometry at one station; two sets are different parts. Azimuths are in radians from the pitch plane (the plane
//! of the incidence, `theta = 0` towards the leeward side), the body is axisymmetric.
//!
//! * A **rotation** by `2 pi / m` is a symmetry when every set maps onto itself: the largest such `m` is the rotational order.
//! * A **mirror** plane at azimuth `phi` (a line through the axis, `phi` and `phi + pi` are the same plane) is a symmetry when every set maps onto itself under
//!   `theta -> 2 phi - theta`.
//! * Zero incidence: the smallest domain is the sector between two adjacent mirror planes (`pi / m` wide, [`Domain::Sector`]), or, with rotations only, the sector of
//!   one rotation period `2 pi / m` ([`Domain::Periodic`]) whose two end planes are the same plane: a periodic seam.
//! * Incidence in the plane at `phi_inc`: the flow is symmetric only about that plane, so the half domain needs a mirror plane there ([`Domain::Half`]); otherwise the
//!   whole circle ([`Domain::Full`]).

use std::f64::consts::PI;

/// The domain that exploits the symmetry of a configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Domain {
    /// Zero incidence: azimuth `phi0 .. phi0 + pi / n` between two adjacent mirror planes of a configuration with `n`-fold rotation symmetry and mirrors (`n = 1`: the
    /// half domain between a mirror plane and its opposite half).
    Sector { n: u32, phi0: f64 },
    /// Incidence in the mirror plane at `phi0`: the half domain `phi0 .. phi0 + pi`.
    Half { phi0: f64 },
    /// Zero incidence, `n`-fold rotation symmetry and no mirror plane: one period `2 pi / n`, whose two end planes are the same plane.
    Periodic { n: u32 },
    /// Nothing is symmetric (or the incidence plane is not a mirror plane): the whole circle, `2 pi`, a periodic seam in the azimuthal lines.
    Full,
}

impl Domain {
    /// Azimuthal extent of the domain.
    pub fn span(&self) -> f64 {
        match *self {
            Domain::Sector { n, .. } => PI / f64::from(n),
            Domain::Half { .. } => PI,
            Domain::Periodic { n } => 2.0 * PI / f64::from(n),
            Domain::Full => 2.0 * PI,
        }
    }

    /// Does the domain need a periodic seam in the azimuthal (`k`) lines (its two end planes are one plane, not symmetry planes)? What the solver needs for it:
    /// `Solver3`'s implicit `k` factor, `AxisMap` and the viscous gradients treat the two ends as boundaries with ghosts (mirror or axis images); a seam needs cyclic
    /// block-tridiagonal solves along `k` (or a lagged seam coupling in the explicit ghost, slower to converge), the face and gradient stencils across the seam, and an
    /// axis ghost for `n` even / odd that maps a cell to the cell half a period away.
    pub fn needs_periodic_seam(&self) -> bool {
        matches!(self, Domain::Periodic { .. } | Domain::Full)
    }
}

const TOL: f64 = 1e-9;

fn wrap(a: f64) -> f64 {
    let t = a.rem_euclid(2.0 * PI);
    if (2.0 * PI - t) < TOL {
        0.0
    } else {
        t
    }
}

/// Is `b` the same set of azimuths as `a` (as multisets mod `2 pi`)?
fn same_set(a: &[f64], b: &[f64]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let (mut x, mut y): (Vec<f64>, Vec<f64>) = (a.iter().map(|&t| wrap(t)).collect(), b.iter().map(|&t| wrap(t)).collect());
    x.sort_by(f64::total_cmp);
    y.sort_by(f64::total_cmp);
    x.iter().zip(&y).all(|(p, q)| (p - q).abs() < TOL || (p - q).abs() > 2.0 * PI - TOL)
}

/// Largest `m` (up to `max`) such that every set is invariant under the rotation by `2 pi / m`.
pub fn rotational_order(sets: &[Vec<f64>], max: u32) -> u32 {
    (1..=max).rev().find(|&m| sets.iter().all(|s| same_set(s, &s.iter().map(|t| t + 2.0 * PI / f64::from(m)).collect::<Vec<_>>()))).unwrap_or(1)
}

/// The mirror planes of the configuration, as azimuths in `[0, pi)` ascending.
pub fn mirror_planes(sets: &[Vec<f64>]) -> Vec<f64> {
    // a mirror plane passes through a fin or half way between two fins of a set: candidates from the pairs of the first set (every set must have it)
    let Some(first) = sets.first() else { return Vec::new() };
    let mut candidates: Vec<f64> = Vec::new();
    for &a in first {
        for &b in first {
            candidates.push(0.5 * (a + b));
            candidates.push(0.5 * (a + b) + 0.5 * PI);
        }
    }
    let mut planes: Vec<f64> = Vec::new();
    for c in candidates {
        let phi = c.rem_euclid(PI);
        let phi = if PI - phi < TOL { 0.0 } else { phi };
        if planes.iter().any(|&p| (p - phi).abs() < TOL) {
            continue;
        }
        if sets.iter().all(|s| same_set(s, &s.iter().map(|t| 2.0 * phi - t).collect::<Vec<_>>())) {
            planes.push(phi);
        }
    }
    planes.sort_by(f64::total_cmp);
    planes
}

/// The smallest valid domain of the fin sets `sets` (azimuths of each set's fins): at zero incidence (`incidence_plane = None`) the sector of the symmetry group, at
/// incidence in the plane `phi_inc` the half domain when that plane is a mirror plane, else the whole circle. A body without fins is axisymmetric: any plane
/// is a mirror (`Sector { n: 1, phi0: 0 }` stands for the wedge: the caller uses [`crate::engine::grid3::Domain3::Wedge`]).
pub fn choose_domain(sets: &[Vec<f64>], incidence_plane: Option<f64>) -> Domain {
    let sets: Vec<Vec<f64>> = sets.iter().filter(|s| !s.is_empty()).cloned().collect();
    if sets.is_empty() {
        return match incidence_plane {
            Some(phi) => Domain::Half { phi0: phi.rem_euclid(PI) },
            None => Domain::Sector { n: 1, phi0: 0.0 },
        };
    }
    let planes = mirror_planes(&sets);
    match incidence_plane {
        Some(phi) => {
            let phi = phi.rem_euclid(PI);
            if planes.iter().any(|&p| (p - phi).abs() < 1e-7 || (p - phi - PI).abs() < 1e-7) {
                Domain::Half { phi0: phi }
            } else {
                Domain::Full
            }
        }
        None => {
            let m = rotational_order(&sets, 64);
            if planes.is_empty() {
                if m == 1 {
                    Domain::Full
                } else {
                    Domain::Periodic { n: m }
                }
            } else {
                // the planes are `pi / m` apart (a dihedral group `D_m`); the sector starts at the first one
                Domain::Sector { n: m, phi0: planes[0] }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn even(n: u32, offset: f64) -> Vec<f64> {
        (0..n).map(|k| offset + 2.0 * PI * f64::from(k) / f64::from(n)).collect()
    }

    #[test]
    fn evenly_spaced_fins_are_a_dihedral_group() {
        for n in 1..=8u32 {
            let d = choose_domain(&[even(n, 0.0)], None);
            assert!(matches!(d, Domain::Sector { n: m, .. } if m == n), "N = {n}: {d:?}");
            assert!((d.span() - PI / f64::from(n)).abs() < 1e-12);
            assert!(!d.needs_periodic_seam());
        }
    }

    #[test]
    fn the_half_domain_needs_a_mirror_in_the_incidence_plane() {
        // four fins: a fin in the pitch plane, or the pitch plane between two fins: both are mirrors
        assert!(matches!(choose_domain(&[even(4, 0.0)], Some(0.0)), Domain::Half { .. }));
        assert!(matches!(choose_domain(&[even(4, PI / 4.0)], Some(0.0)), Domain::Half { .. }));
        // incidence at 20 degrees to a four-fin set: not a mirror plane: the whole circle
        assert_eq!(choose_domain(&[even(4, 0.0)], Some(20f64.to_radians())), Domain::Full);
        // three fins: a mirror through one fin (and the gap opposite)
        assert!(matches!(choose_domain(&[even(3, 0.0)], Some(0.0)), Domain::Half { .. }));
        assert!(matches!(choose_domain(&[even(3, 0.0)], Some(PI / 3.0)), Domain::Half { .. }), "the plane through the gap and the opposite fin");
        assert_eq!(choose_domain(&[even(3, 0.0)], Some(PI / 6.0)), Domain::Full);
    }

    #[test]
    fn a_pattern_with_rotation_but_no_mirror_is_a_periodic_sector() {
        // three fins at 0, 120 and 240 degrees, each set a pinwheel is not expressible with azimuths alone; two sets of different geometry at an offset are:
        // set A at 0, 90, 180, 270; set B (another station) at 10, 100, 190, 280 degrees: rotation by 90 degrees maps each onto itself, no mirror maps B onto itself
        let a = even(4, 0.0);
        let b = even(4, 10f64.to_radians());
        let d = choose_domain(&[a.clone(), b.clone()], None);
        // the mirrors of A (planes at 0 and 45 degrees) do not preserve B: B has mirrors at 5 and 50 degrees; no common one
        assert_eq!(d, Domain::Periodic { n: 4 });
        assert!(d.needs_periodic_seam() && (d.span() - PI / 2.0).abs() < 1e-12);
        // the incidence in the plane at 0: not a mirror of B
        assert_eq!(choose_domain(&[a, b], Some(0.0)), Domain::Full);
    }

    #[test]
    fn sets_on_different_stations_share_the_common_symmetry() {
        // four fins and three fins: no common rotation, the pitch plane is a common mirror
        let d = choose_domain(&[even(4, 0.0), even(3, 0.0)], None);
        assert!(matches!(d, Domain::Sector { n: 1, .. }), "{d:?}");
        assert!(matches!(choose_domain(&[even(4, 0.0), even(3, 0.0)], Some(0.0)), Domain::Half { .. }));
        // four fins and eight fins: the eight-fin ring has the four-fin rotation and mirrors
        let d = choose_domain(&[even(4, 0.0), even(8, 0.0)], None);
        assert!(matches!(d, Domain::Sector { n: 4, .. }), "{d:?}");
    }

    #[test]
    fn nothing_symmetric_is_the_whole_circle() {
        // three fins at arbitrary angles
        let sets = [vec![0.0, 1.0, 3.5]];
        assert_eq!(choose_domain(&sets, None), Domain::Full);
        assert!(Domain::Full.needs_periodic_seam() && (Domain::Full.span() - 2.0 * PI).abs() < 1e-12);
        // an unfinned body is axisymmetric
        assert!(matches!(choose_domain(&[], Some(0.3)), Domain::Half { .. }));
    }

    #[test]
    fn per_fin_angles_of_one_set_can_keep_a_mirror() {
        // two fins at +-30 degrees about the pitch plane: one mirror, at 0 (and no rotation)
        let sets = [vec![30f64.to_radians(), -30f64.to_radians()]];
        assert_eq!(mirror_planes(&sets), vec![0.0]);
        assert!(matches!(choose_domain(&sets, Some(0.0)), Domain::Half { .. }));
        assert!(matches!(choose_domain(&sets, None), Domain::Sector { n: 1, .. }));
        // the same two fins and their opposites: mirrors at 0 and 90 degrees, rotation by 180 degrees: a quarter domain
        let four = [vec![30f64.to_radians(), -30f64.to_radians(), 150f64.to_radians(), 210f64.to_radians()]];
        let planes = mirror_planes(&four);
        assert_eq!(planes.len(), 2);
        assert!(planes[0].abs() < 1e-12 && (planes[1] - PI / 2.0).abs() < 1e-12, "{planes:?}");
        let d = choose_domain(&four, None);
        assert!(matches!(d, Domain::Sector { n: 2, .. }) && (d.span() - PI / 2.0).abs() < 1e-12, "{d:?}");
    }
}
