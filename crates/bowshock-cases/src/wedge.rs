//! The cone and ogive cases on the single-cell wedge of the 3D engine (`Case3::wedge`): the CPU Euler solver of the product, run on the case's meridian grid
//! revolved by the wedge angle; its final state is a [`Meridian`] (the grid and the conserved state of the meridian plane) that the cases' own post-processing (wall
//! pressure, shock angle, forebody drag) reads. Where the wedge reproduces the frozen 2D results (G-W1 and G-W4 in `tests/wedge_gates.rs`) a validation driver can run
//! on it (`--wedge`); the 2D runners stay until the 2D deletion.

use bowshock_cfd::gas3::GAMMA;
use bowshock_cfd::grid::Grid;
use bowshock_cfd::limiter::Limiter;
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::grid3::{wedge_of_meridian, WEDGE_ANGLE};
use bowshock_core::engine::result::StopReason;

/// What the iteration of a wedge run gives besides the state.
pub struct WedgeRun {
    /// Density residual (L2 of d rho/dt of the 3D solver) per iteration.
    pub residual: Vec<f64>,
    /// Orders between the peak residual of the second-order phase and the last one.
    pub drop_orders: f64,
    pub stop_reason: StopReason,
}

/// The state of a meridian grid: the conserved variables `[rho, rho u, rho v, E]` per cell (`j * ni + i`) and the free stream `[rho, u, v, p]`.
pub struct Meridian {
    pub grid: Grid,
    pub u: Vec<[f64; 4]>,
    pub freestream: [f64; 4],
}

/// Primitive variables `[rho, u, v, p]` of a conserved state of the meridian plane.
pub fn prim(q: &[f64; 4]) -> [f64; 4] {
    let (rho, u, v) = (q[0], q[1] / q[0], q[2] / q[0]);
    [rho, u, v, (GAMMA - 1.0) * (q[3] - 0.5 * rho * (u * u + v * v))]
}

impl Meridian {
    /// Pressure at the cell centre of `(i, j)`.
    pub fn pressure(&self, i: usize, j: usize) -> f64 {
        prim(&self.u[j * self.grid.ni + i])[3]
    }

    /// The state of a 2D solver (the 2D runners of the cases, until the 2D deletion).
    pub fn of_solver(s: &bowshock_cfd::solver::Solver) -> Meridian {
        Meridian { grid: s.grid.clone(), u: s.u.clone(), freestream: s.cfg.freestream }
    }
}

/// Iterates the flow on `g2` (free stream Mach number `mach`) on the wedge until the density residual has dropped `drop_orders` orders from the peak of the second-order
/// phase (`first_order_iters` onward, as `StopRule3`), `max_iter`, or a plateau (the 2D runners' rule); the final state is the [`Meridian`] (the radial momentum from the
/// wedge cell's `y`, `z` momenta).
pub fn run_on_wedge(
    g2: &Grid,
    mach: f64,
    limiter: Limiter,
    first_order_iters: usize,
    max_iter: usize,
    drop_orders: f64,
) -> Result<(Meridian, WedgeRun), String> {
    // the lower boundary is the axis for x < 0 and the body from the tip on
    let n_axis = (0..g2.ni).take_while(|&i| g2.x[i + 1] <= 1e-12).count();
    let grid = wedge_of_meridian(g2, WEDGE_ANGLE, n_axis).map_err(|e| e.to_string())?;
    // the case only sets the numerics of the Euler wedge (its body is where the forces would stop: past every grid)
    let mut case = Case3::wedge(mach, Case3::anf_body(), 0);
    case.numerics.limiter = limiter;
    case.numerics.first_order_iters = first_order_iters;
    let mut s3 = case.solver_on(grid);
    let mut residual = Vec::new();
    let mut rmax: f64 = 0.0;
    let mut reason = StopReason::MaxIterations;
    let from = first_order_iters.max(20);
    for it in 0..max_iter {
        let r = s3.step();
        residual.push(r);
        if it >= from {
            rmax = rmax.max(r);
        }
        if !r.is_finite() {
            reason = StopReason::NonFinite;
            break;
        }
        if it > from && rmax / r >= 10f64.powf(drop_orders) {
            reason = StopReason::ResidualDrop;
            break;
        }
        if it >= 8000 && it % 1000 == 0 {
            let m = |a: usize, b: usize| residual[a..b].iter().map(|x| x.log10()).sum::<f64>() / (b - a) as f64;
            if m(it - 2000, it) - m(it - 4000, it - 2000) > -0.05 {
                reason = StopReason::ResidualPlateau;
                break;
            }
        }
    }
    let (ni, nj) = (g2.ni, g2.nj);
    let g3 = &s3.grid;
    let mut u = vec![[0.0; 4]; ni * nj];
    for j in 0..nj {
        for i in 0..ni {
            let c = g3.cell(i, j, 0);
            let (q, x) = (s3.u[c], g3.xc[c]);
            let r = x[1].hypot(x[2]);
            // the wedge cell's radial direction at its centroid: y = r sin(theta), z = r cos(theta)
            let m_r = if r > 0.0 { (q[2] * x[1] + q[3] * x[2]) / r } else { 0.0 };
            u[j * ni + i] = [q[0], q[1], m_r, q[4]];
        }
    }
    let drop_orders = if rmax > 0.0 { (rmax / residual.last().copied().unwrap_or(rmax)).log10() } else { 0.0 };
    let freestream = [1.0, mach, 0.0, 1.0 / GAMMA];
    Ok((Meridian { grid: g2.clone(), u, freestream }, WedgeRun { residual, drop_orders, stop_reason: reason }))
}

// ---------------------------------------------------------------------------------------------------------------------
// The sting and base-wake cases of the transonic and subsonic families on the wedge: no 2D solver, no 2D `PointResult`.
// ---------------------------------------------------------------------------------------------------------------------

use bowshock_cfd::gas3::{cons_to_prim3, sound_speed3};
use bowshock_cfd::grid3::PART_FLUID;
use bowshock_cfd::solver3::Solver3;
use bowshock_core::engine::body_grid::BaseBody;
use bowshock_core::engine::result::WallPoint;
use bowshock_core::engine::run3::{base_cp, drag_parts_on, run3_on, wall_samples, y_plus_max, Wall};

/// A body with the cylinder continuing as a sting to the outflow plane (no base), as a single-cell wedge on the sting grid at grid `level`: the Euler wedge numerics of
/// the Mach regime ([`bowshock_core::engine::case3::Numerics3::euler_wedge`]).
pub fn sting_case(mach: f64, body: BaseBody, level: u32) -> Case3 {
    let mut c = Case3::new(mach, 0.0, body, None, level);
    c.numerics = bowshock_core::engine::case3::Numerics3::euler_wedge(mach, level);
    c.on_wedge()
}

/// A finite body with a flat base and a wake region (the base-wake grid) as a single-cell wedge at grid `level`.
pub fn base_case(mach: f64, body: BaseBody, level: u32) -> Case3 {
    Case3::wedge(mach, body, level)
}

/// What a run of a sting or base-wake case on the wedge reports: the quantities the transonic and subsonic drivers print and compare with the published data.
/// Drag coefficients refer to the nose base area (`d = 1`); the wall `Cp` is the extrapolated wall value, as `bowshock_core::engine::run3::lift_forces` has it.
#[derive(Clone, Debug)]
pub struct WedgeResult {
    pub mach: f64,
    pub iters: usize,
    pub stop_reason: StopReason,
    pub drop_orders: f64,
    pub cd_nose: f64,
    /// One per aft segment.
    pub cd_aft: Vec<f64>,
    pub cd_base: f64,
    /// Friction and largest `y+` of a viscous case (`0`, `NaN` for Euler).
    pub cd_friction: f64,
    pub y_plus_max: f64,
    /// Mean base `Cp` (area weighted, NaN without a base) and the `Cp` of the base face per radius.
    pub cp_base: f64,
    pub base_wall: Vec<(f64, f64)>,
    /// The body wall in axial order (`s0`, `s1`, `cp_cell` unused: NaN).
    pub wall: Vec<WallPoint>,
    pub cp_min: f64,
    pub x_cp_min: f64,
    /// Largest Mach number of the flow field.
    pub m_max: f64,
    pub min_rho: f64,
    pub min_p: f64,
}

/// Iterates `s` (built from `case`, [`Case3::build_solver`]) to its stop rule ([`run3_on`]; `warm`: from the state it holds) and collects the [`WedgeResult`].
pub fn run_wedge_point(case: &Case3, s: &mut Solver3, warm: bool) -> WedgeResult {
    let r = run3_on(case, s, warm);
    let p = drag_parts_on(case, s);
    let (p_inf, q) = (s.cfg.freestream[4], 0.5 * case.mach * case.mach);
    let mut wall = Vec::new();
    let mut base_wall = Vec::new();
    for w in wall_samples(&s.grid, &s.u) {
        let cp = (w.pw - p_inf) / q;
        let (x, rad) = (w.face.c[0], w.face.c[1].hypot(w.face.c[2]));
        match w.class {
            Wall::Body(_) | Wall::Sting => wall.push(WallPoint { x, r: rad, s0: f64::NAN, s1: f64::NAN, cp, cp_cell: f64::NAN }),
            Wall::Base => base_wall.push((rad, cp)),
            _ => {}
        }
    }
    wall.sort_by(|a, b| a.x.total_cmp(&b.x));
    base_wall.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (cp_min, x_cp_min) = wall.iter().fold((f64::INFINITY, f64::NAN), |(m, xm), w| if w.cp < m { (w.cp, w.x) } else { (m, xm) });
    let (mut m_max, mut min_rho, mut min_p) = (0.0f64, f64::INFINITY, f64::INFINITY);
    for (c, u) in s.u.iter().enumerate() {
        if s.grid.part[c] != PART_FLUID {
            continue;
        }
        let w = cons_to_prim3(u);
        m_max = m_max.max((w[1] * w[1] + w[2] * w[2] + w[3] * w[3]).sqrt() / sound_speed3(&w));
        min_rho = min_rho.min(w[0]);
        min_p = min_p.min(w[4]);
    }
    let friction = p.friction.total;
    WedgeResult {
        mach: case.mach,
        iters: r.iterations,
        stop_reason: r.stop_reason,
        drop_orders: r.drop_orders,
        cd_nose: p.nose,
        cd_aft: p.aft,
        cd_base: p.base,
        cd_friction: friction,
        y_plus_max: if s.rans().is_some() { y_plus_max(s) } else { f64::NAN },
        cp_base: base_cp(case, s),
        base_wall,
        wall,
        cp_min,
        x_cp_min,
        m_max,
        min_rho,
        min_p,
    }
}
