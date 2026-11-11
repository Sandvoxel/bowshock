//! Result types of a solver run and the wall post-processing helpers.

use std::fmt;

/// Why an iteration loop stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason {
    ResidualDrop,
    CdStationary,
    BasePressureStationary,
    ResidualPlateau,
    MaxIterations,
    NonFinite,
}

impl fmt::Display for StopReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.pad(match self {
            StopReason::ResidualDrop => "residual drop reached",
            StopReason::CdStationary => "CD stationary",
            StopReason::BasePressureStationary => "base pressure stationary",
            StopReason::ResidualPlateau => "residual plateau",
            StopReason::MaxIterations => "max iterations",
            StopReason::NonFinite => "non-finite residual",
        })
    }
}

/// One wall face of the body.
#[derive(Clone, Copy, Debug)]
pub struct WallPoint {
    /// Face centre (x/d, r/d).
    pub x: f64,
    pub r: f64,
    /// `s = (r/R)^2` at the two face end nodes (for the base-area-referenced drag integral).
    pub s0: f64,
    pub s1: f64,
    /// Cp from the wall value extrapolated linearly from cells j=0,1, and from the first cell.
    pub cp: f64,
    pub cp_cell: f64,
}

/// One base-face sample: radius range, Cp from the wall-extrapolated and the first-cell pressure.
#[derive(Clone, Copy, Debug)]
pub struct BasePoint {
    pub r0: f64,
    pub r1: f64,
    pub cp: f64,
    pub cp_cell: f64,
}

/// Linear interpolation of the wall `Cp` (extrapolated values) at axial position `x` (None outside the wall range).
pub fn wall_cp_at(wall: &[WallPoint], x: f64) -> Option<f64> {
    for w in wall.windows(2) {
        if x >= w[0].x && x <= w[1].x {
            let t = (x - w[0].x) / (w[1].x - w[0].x);
            return Some(w[0].cp + t * (w[1].cp - w[0].cp));
        }
    }
    None
}

/// Everything a run reports. Drag coefficients are referenced to the maximum cross-section `pi R^2`; the pressure terms of a
/// viscous run add `cd_friction`.
#[derive(Clone, Debug)]
pub struct PointResult {
    pub mach: f64,
    /// Angle of attack [rad]; always 0.0 (the solver is axisymmetric).
    pub alpha: f64,
    pub iters: usize,
    pub residual: Vec<f64>,
    /// Largest residual after iteration 20 (cold start) / of the run (warm start).
    pub residual_peak: f64,
    /// Orders dropped from `residual_peak` to the last residual.
    pub drop_orders: f64,
    pub stop_reason: StopReason,
    /// Wall faces of the nose, cylinder and boattail (and sting extension) in axial order.
    pub wall: Vec<WallPoint>,
    /// Base-face samples (empty without base), `r0 < r1`.
    pub base_wall: Vec<BasePoint>,
    /// Forebody CD from the extrapolated wall Cp and from the first-cell Cp.
    pub cd_nose: f64,
    pub cd_nose_cell: f64,
    /// Pressure drag of each aft segment (`body.segments` order), from the extrapolated wall Cp.
    pub cd_aft: Vec<f64>,
    pub cd_base: f64,
    /// Skin-friction drag of the body surface (`0.0` for the Euler equations).
    pub cd_friction: f64,
    /// `cd_nose + cd_aft[0] + cd_aft[1] + .. + cd_friction + cd_base` (`cd_friction` is 0 for the Euler equations).
    pub cd_total: f64,
    /// Base drag of the empirical free-flight correlation scaled to `pi R^2`; 0 without base, NaN for a sting-mounted base.
    pub cd_base_model: f64,
    /// `cd_nose + cd_aft[0] + cd_aft[1] + .. + cd_friction + cd_base_model`.
    pub cd_total_model: f64,
    /// Area-averaged base Cp (extrapolated wall values) and the first-cell variant; NaN without base.
    pub cp_base: f64,
    pub cp_base_cell: f64,
    /// Mean and range of the area-mean base Cp over the monitor window (NaN range without enough history or without base, and
    /// for a viscous run, whose monitor is the total CD).
    pub cp_base_mean: f64,
    pub cp_base_range: f64,
    /// Max - min of the monitor over the monitor window when it is the forebody CD (sting wake) or the total CD (viscous run);
    /// NaN otherwise or without enough history.
    pub cd_range: f64,
    /// Monitor samples `(iteration, value)` every `MONITOR_EVERY` iterations.
    pub monitor_history: Vec<(usize, f64)>,
    pub cp_min: f64,
    pub x_cp_min: f64,
    pub m_max: f64,
    /// Maximum wall-cell Mach number.
    pub m_wall_max: f64,
    /// Max `|p/p_inf - 1|` over the boundary cells of the inflow, outer and outflow boundaries.
    pub farfield_dp: f64,
    pub min_rho: f64,
    pub min_p: f64,
    /// Largest wall distance `y+` of the first cell over the body surface (NaN for the Euler equations).
    pub y_plus_max: f64,
}
