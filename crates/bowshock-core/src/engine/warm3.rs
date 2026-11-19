//! Warm starts of the 3D engine across Mach numbers: the neighbouring point's converged field, carried to the new free stream ([`rescale`]), and the order in which the points
//! of a sweep are solved so that every point has a converged neighbour to start from ([`warm_plan`]). The facade's sweep ([`crate::Analysis::warm_start`]) uses both.
//!
//! Non-dimensional state of the solver: free-stream density 1, pressure `1/gamma`, sound speed 1 and velocity `M (cos a, 0, sin a)`, so a change of Mach number at constant
//! incidence is a change of the velocity of the free stream alone, and the field of a neighbouring Mach number is the right shape (shock, wake, boundary layers) at the
//! wrong scale: velocities scale with `M`, the deviations of density and pressure from the free stream (the dynamic pressure) with `M^2`.

use bowshock_cfd::gas3::{cons_to_prim3, freestream3, prim_to_cons3, Cons3};
use bowshock_cfd::grid3::PART_FLUID;

/// How the field of one Mach number is carried to another.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rescale {
    /// Velocities times `M_new / M_old`, density and pressure unchanged (the free stream of the new Mach number is exact; the dynamic pressure of the field is not).
    Velocity,
    /// Velocities times `s = M_new / M_old`, the deviations of density and pressure from the free stream times `s^2` (floored at a tenth of the free stream's value, so a
    /// wake below the free-stream pressure cannot go through zero when `s > 1`): the field of a small perturbation of a uniform flow at the new Mach number.
    Perturbation,
    /// The neighbour's field as it is (the free stream of the new point is only in the boundary conditions): the start of the sweep before the field was rescaled, kept for
    /// the measurements.
    None,
}

/// The floor of the density and pressure of a [`Rescale::Perturbation`] field, as a fraction of the free stream's.
const FLOOR: f64 = 0.1;

/// The conserved state `u` of a field converged at Mach number `from` and incidence `alpha`, carried to Mach number `to` at the same incidence: the fluid cells by `mode`, the
/// other cells the new free stream. `part` is the grid's [`bowshock_cfd::grid3::Grid3::part`].
pub fn rescale(u: &[Cons3], part: &[u8], from: f64, to: f64, alpha: f64, mode: Rescale) -> Vec<Cons3> {
    if mode == Rescale::None {
        return u.to_vec();
    }
    let s = to / from;
    let fs = freestream3(to, alpha);
    let (rho_inf, p_inf) = (fs[0], fs[4]);
    u.iter()
        .zip(part)
        .map(|(c, &p)| {
            if p != PART_FLUID {
                return prim_to_cons3(&fs);
            }
            let w = cons_to_prim3(c);
            let (rho, p) = match mode {
                Rescale::Velocity | Rescale::None => (w[0], w[4]),
                Rescale::Perturbation => {
                    let q = s * s;
                    ((rho_inf + q * (w[0] - rho_inf)).max(FLOOR * rho_inf), (p_inf + q * (w[4] - p_inf)).max(FLOOR * p_inf))
                }
            };
            prim_to_cons3(&[rho, s * w[1], s * w[2], s * w[3], p])
        })
        .collect()
}

/// One point of a warm sweep: the index of the point (of the Mach numbers given to [`warm_plan`]) and the index of the point whose converged field it starts from, `None` for
/// a point that starts from the axisymmetric seed or the free stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WarmStep {
    pub point: usize,
    pub parent: Option<usize>,
}

/// The order of a warm sweep over the points at the Mach numbers `machs` (distinct, any order): the points above `split` from the highest one downward, the others from the lowest
/// one upward (`sub_up`) or from the highest of them downward, every point starting from the previous one of its run of the order and each run's first point from nothing. A
/// point whose Mach number differs from its parent's by more than the factor `max_ratio` (`M_new / M_old` or its inverse) starts from nothing too. The steps come in the order
/// they are solved in, a parent before its children.
pub fn warm_plan(machs: &[f64], split: f64, sub_up: bool, max_ratio: f64) -> Vec<WarmStep> {
    let mut idx: Vec<usize> = (0..machs.len()).collect();
    idx.sort_by(|&a, &b| machs[a].total_cmp(&machs[b]));
    let (sub, sup): (Vec<usize>, Vec<usize>) = idx.into_iter().partition(|&i| machs[i] <= split);
    let mut order: Vec<Vec<usize>> = Vec::new();
    // the points of the sub-split run: ascending from the lowest Mach number, or descending from the highest of them
    order.push(if sub_up { sub } else { sub.into_iter().rev().collect() });
    order.push(sup.into_iter().rev().collect());
    let mut steps = Vec::with_capacity(machs.len());
    for run in order {
        let mut parent: Option<usize> = None;
        for p in run {
            let near = |q: usize| {
                let r = machs[p] / machs[q];
                r.max(1.0 / r) <= max_ratio
            };
            steps.push(WarmStep { point: p, parent: parent.filter(|&q| near(q)) });
            parent = Some(p);
        }
    }
    steps
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_cfd::gas3::GAMMA;

    fn field(mach: f64) -> Vec<Cons3> {
        // a free stream with a stagnation-like cell and a wake-like cell
        let fs = freestream3(mach, 0.0);
        vec![
            prim_to_cons3(&fs),
            prim_to_cons3(&[1.0 + 0.3 * mach * mach, 0.1 * mach, 0.0, 0.0, fs[4] * (1.0 + 0.5 * mach * mach)]),
            prim_to_cons3(&[1.0 - 0.2 * mach * mach, 0.4 * mach, 0.1 * mach, 0.0, fs[4] * (1.0 - 0.2 * mach * mach)]),
        ]
    }

    #[test]
    fn the_free_stream_goes_to_the_free_stream_of_the_new_mach_number() {
        for mode in [Rescale::Velocity, Rescale::Perturbation] {
            let out = rescale(&field(0.8), &[PART_FLUID; 3], 0.8, 2.3, 0.0, mode);
            let fs = prim_to_cons3(&freestream3(2.3, 0.0));
            for k in 0..5 {
                assert!((out[0][k] - fs[k]).abs() < 1e-13, "{mode:?} {k}");
            }
        }
        // at incidence too: both components scale
        let u = vec![prim_to_cons3(&freestream3(0.5, 0.05))];
        let out = rescale(&u, &[PART_FLUID], 0.5, 1.5, 0.05, Rescale::Perturbation);
        let fs = prim_to_cons3(&freestream3(1.5, 0.05));
        for k in 0..5 {
            assert!((out[0][k] - fs[k]).abs() < 1e-13);
        }
    }

    #[test]
    fn a_perturbation_scales_with_the_dynamic_pressure_and_a_velocity_rescale_leaves_the_pressure_alone() {
        let (from, to) = (0.5, 1.0);
        let u = field(from);
        let p = |c: &Cons3| cons_to_prim3(c);
        let per = rescale(&u, &[PART_FLUID; 3], from, to, 0.0, Rescale::Perturbation);
        // the stagnation cell: p' = p_inf + 4 (p - p_inf), velocities double
        let (w0, w1) = (p(&u[1]), p(&per[1]));
        let p_inf = 1.0 / GAMMA;
        assert!((w1[4] - (p_inf + 4.0 * (w0[4] - p_inf))).abs() < 1e-12);
        assert!((w1[0] - (1.0 + 4.0 * (w0[0] - 1.0))).abs() < 1e-12);
        assert!((w1[1] - 2.0 * w0[1]).abs() < 1e-12);
        let vel = rescale(&u, &[PART_FLUID; 3], from, to, 0.0, Rescale::Velocity);
        let (w0, w1) = (p(&u[2]), p(&vel[2]));
        assert!((w1[4] - w0[4]).abs() < 1e-12 && (w1[0] - w0[0]).abs() < 1e-12);
        assert!((w1[1] - 2.0 * w0[1]).abs() < 1e-12 && (w1[2] - 2.0 * w0[2]).abs() < 1e-12);
    }

    #[test]
    fn a_wake_cannot_go_through_zero_pressure_and_solid_cells_take_the_free_stream() {
        // a cell at 5 % of the free-stream pressure, carried from Mach 0.1 to 3 (s^2 = 900)
        let fs = freestream3(0.1, 0.0);
        let u = vec![prim_to_cons3(&[0.5, 0.0, 0.0, 0.0, 0.05 * fs[4]]); 2];
        let out = rescale(&u, &[PART_FLUID, PART_FLUID + 1], 0.1, 3.0, 0.0, Rescale::Perturbation);
        let w = cons_to_prim3(&out[0]);
        assert!(w[0] >= 0.1 && w[4] >= 0.1 / GAMMA - 1e-15, "{w:?}");
        let solid = cons_to_prim3(&out[1]);
        assert!(solid.iter().zip(freestream3(3.0, 0.0)).all(|(a, b)| (a - b).abs() < 1e-13), "{solid:?}");
    }

    #[test]
    fn the_plan_runs_down_from_the_highest_supersonic_point_and_up_from_the_lowest_subsonic_one() {
        let machs = [0.1, 0.5, 0.8, 0.9, 1.0, 1.1, 1.3, 1.7, 2.3, 3.0];
        let plan = warm_plan(&machs, 0.9, true, 10.0);
        assert_eq!(plan.len(), machs.len());
        let order: Vec<f64> = plan.iter().map(|s| machs[s.point]).collect();
        assert_eq!(order, vec![0.1, 0.5, 0.8, 0.9, 3.0, 2.3, 1.7, 1.3, 1.1, 1.0]);
        let parents: Vec<Option<f64>> = plan.iter().map(|s| s.parent.map(|p| machs[p])).collect();
        assert_eq!(parents, vec![None, Some(0.1), Some(0.5), Some(0.8), None, Some(3.0), Some(2.3), Some(1.7), Some(1.3), Some(1.1)]);
        // every parent is solved before its child
        for (k, s) in plan.iter().enumerate() {
            if let Some(p) = s.parent {
                assert!(plan[..k].iter().any(|q| q.point == p));
            }
        }
        // the sub-split run downward, and the ratio limit
        let down = warm_plan(&machs, 0.9, false, 4.0);
        let order: Vec<f64> = down.iter().map(|s| machs[s.point]).collect();
        assert_eq!(&order[..4], &[0.9, 0.8, 0.5, 0.1]);
        assert_eq!(down[3].parent, None, "0.5 to 0.1 is a factor 5");
        assert_eq!(down[2].parent.map(|p| machs[p]), Some(0.8));
        // one run from the top, and one point
        let all = warm_plan(&machs, 0.0, true, 10.0);
        assert_eq!(all[0].point, 9);
        assert_eq!(all.iter().filter(|s| s.parent.is_none()).count(), 1);
        assert_eq!(warm_plan(&[2.0], 0.9, true, 4.0), vec![WarmStep { point: 0, parent: None }]);
    }
}
