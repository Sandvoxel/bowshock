//! Flight simulation (x east, y north, z up) with two equations of motion ([`FlightModel`]): a 3-DOF point mass and, by default,
//! a rigid body. Both integrate with RK4 (Niskanen 2009, section 4.2).
//!
//! Model summary:
//! * On the rail the rocket is constrained to the rail axis and does not rotate. Off the rail the point-mass model points the
//!   thrust along the velocity relative to the air (perfect weathercocking); the rigid-body model flies the attitude (below).
//!   Gravity varies with altitude in both. With a launch latitude ([`Launch::latitude`]) the surface gravity is the WGS 84
//!   normal gravity of that latitude (Somigliana's formula, NIMA TR8350.2) and the Coriolis acceleration `-2 Omega x v` of the
//!   rotating Earth acts off the rail, in the local east-north-up frame (a flat Earth otherwise).
//! * **Rigid body.** State: position, velocity, attitude quaternion and body angular rate. Forces in the body frame: thrust along
//!   the axis, the axial drag `q A_ref Cd` (zero-lift build-up times the axial multiplier of [`crate::aero::axial_drag_multiplier`])
//!   and the normal force `q A_ref CNalpha alpha` from [`Aero::normal_force`] against the lateral velocity. The normal force
//!   acts at the centre of pressure, whose lever arm about the current centre of gravity (it moves forward as propellant
//!   burns) gives the pitch/yaw moment. Euler's equations for an axisymmetric body (`I_x` roll, `I_y = I_z` pitch/yaw;
//!   Zipfel 2014, *Modeling and Simulation of Aerospace Vehicle Dynamics*) with damping from the fins and bodies at their
//!   centres of pressure (`1/2 rho V A_ref sum CNalpha_i (x_i - x_cg)^2`, Barrowman 1967; Mandell, Caporaso & Bengen 1973),
//!   the body crossflow drag on the turning body, and the jet damping of the exhaust leaving at the nozzle
//!   (`mdot (x_exit - x_cg)^2`). Fin cant drives a roll moment against the roll damping of the fins. The step is shortened to
//!   keep the fastest pitch dynamics well inside the RK4 stability limit. Inertias come from the parts, with the propellant
//!   inertia falling as it burns.
//! * Drag comes from [`Aero::drag`] at the current Mach/Reynolds number with the motor on/off
//!   (base drag is reduced while any motor burns). Under a deployed recovery device the descent is flown as a point mass and the
//!   drag is `0.5 rho v^2 CdA` summed over deployed devices.
//! * Stages are in firing order (see [`StageMass`]). Each stage's ignition and separation are [`Timing`]s (a trigger plus a
//!   delay: burnout, separation, time from launch, or climbing through an altitude), resolved by [`Timeline`] as the flight
//!   goes: altitude triggers fire the first step the rocket is at or above the altitude while ascending. A separated stage is
//!   dropped; only the remaining upper stack is flown afterwards and the aerodynamics are rebuilt for the truncated vehicle.
//!   Recovery devices on dropped stages are ignored.
//! * Wind is the ground wind below the first level of [`Launch::wind_levels`], interpolated linearly in its east and north
//!   components between levels and held above the top one; turbulence adds an Ornstein-Uhlenbeck gust with standard
//!   deviation `intensity x` the mean wind speed at the rocket's altitude.

mod dynamics;
mod input;
mod result;
mod run;
#[cfg(test)]
mod tests;

pub use input::*;
pub use result::*;
pub(crate) use run::run;
pub use run::simulate;

use crate::aero::{Aero, DragBreakdown};
use crate::atmosphere::{AirState, Atmosphere};
use crate::attitude::{normalize, rate, Quat, Vec3};
use crate::staging::Timeline;

/// Perturbations used by Monte Carlo (all 1.0 = nominal).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Tweaks {
    pub thrust: f64,
    pub cd: f64,
    pub mass: f64,
}

impl Tweaks {
    pub(crate) const NOMINAL: Tweaks = Tweaks { thrust: 1.0, cd: 1.0, mass: 1.0 };
}

/// One aerodynamic model per separation level (0 = full stack, k = lowest k stages dropped).
pub(crate) fn build_aeros(input: &SimInput) -> Vec<Aero> {
    let v = &input.vehicle;
    let n = v.stages.len().max(1);
    (0..n)
        .map(|k| {
            let aero = if k == 0 { Aero::new(v) } else { Aero::new(&v.truncated(v.stages[k].aft_station)) };
            let aero = match input.drag_tables.get(k) {
                Some(Some(table)) => aero.with_table(table.clone()),
                _ => aero,
            };
            match input.lift_tables.get(k) {
                Some(Some(table)) => aero.with_lift(table.clone()),
                _ => aero,
            }
        })
        .collect()
}

#[derive(Clone, Copy)]
struct Raw {
    t: f64,
    pos: [f64; 3],
    vel: [f64; 3],
    nsep: usize,
    d: Diag,
}

/// A deployed recovery device with a finite canopy-inflation time.
///
/// The effective drag area grows as `CdA (tau)^2`, `tau = (t - t_deploy)/t_inflate` clamped to
/// 0..1, the quadratic opening law used for ballistic inflation in Knacke's *Parachute Recovery
/// Systems Design Manual* (1992). The fill time follows the same source's "fill distance"
/// rule of thumb, `t = n D / v` with `n ~ 8` canopy diameters for solid canopies, where `D` is
/// the equivalent canopy diameter for a nominal canopy drag coefficient of 0.75
/// (`CdA = 0.75 pi D^2 / 4`) and `v` is the air speed at deployment, clamped to 0.2..1.5 s so
/// slow deployments at apogee do not take unrealistically long and fast ones are not instant.
#[derive(Clone, Copy)]
struct Chute {
    cd_area: f64,
    t_deploy: f64,
    t_inflate: f64,
}

impl Chute {
    fn new(cd_area: f64, t_deploy: f64, deploy_speed: f64) -> Self {
        let diameter = (4.0 * cd_area / (std::f64::consts::PI * 0.75)).sqrt();
        Self { cd_area, t_deploy, t_inflate: (8.0 * diameter / deploy_speed.max(1.0)).clamp(0.2, 1.5) }
    }

    /// Effective drag area at time `t`.
    fn area_at(&self, t: f64) -> f64 {
        let tau = ((t - self.t_deploy) / self.t_inflate).clamp(0.0, 1.0);
        self.cd_area * tau * tau
    }
}

struct Ctx<'a> {
    input: &'a SimInput,
    launch: &'a Launch,
    tw: Tweaks,
    aeros: &'a [Aero],
    atm: Atmosphere,
    sched: Timeline,
    /// The mean wind profile (see [`Launch::wind_at`]).
    wind: Vec<(f64, [f64; 2])>,
    axis: [f64; 3],
    a_ref: f64,
    /// Gravity at the site's surface (m/s^2): standard, or the normal gravity of the latitude.
    g_surface: f64,
    /// The Earth's rotation in the local east-north-up frame (rad/s), with a latitude.
    omega: Option<[f64; 3]>,
    /// The fin sets under active roll control, per aerodynamic model (see [`Aero::controlled_fin_sets`]).
    controlled: Vec<Vec<(usize, bowshock_model::PartId)>>,
    /// The cant each roll controller holds away from its group's own cant (rad), by group.
    deflection: Vec<(bowshock_model::PartId, f64)>,
    /// The cant of the first roll controller's group (rad), recorded with every sample.
    control_cant: f64,
}

/// Angular rate of the Earth's rotation (rad/s), WGS 84 (NIMA TR8350.2).
pub const EARTH_RATE: f64 = 7.292_115e-5;

/// Normal gravity on the WGS 84 ellipsoid at geodetic latitude `lat` (rad), Somigliana's closed form (NIMA TR8350.2, eq. 4-1):
/// `γ = γe (1 + k sin²φ) / sqrt(1 − e² sin²φ)`.
pub fn normal_gravity(lat: f64) -> f64 {
    let s2 = lat.sin().powi(2);
    9.780_325_335_9 * (1.0 + 0.001_931_852_652_41 * s2) / (1.0 - 0.006_694_379_990_13 * s2).sqrt()
}

/// How a step's equations of motion treat the attitude.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// On the rail: the rocket slides along the axis and does not rotate.
    Rail,
    /// Point mass: the axis follows the relative wind.
    Point,
    /// Rigid body: the attitude is part of the state.
    Rigid,
}

/// The integrated state: position, ground-relative velocity, attitude (body to inertial) and body angular rate.
#[derive(Clone, Copy)]
struct State {
    pos: Vec3,
    vel: Vec3,
    q: Quat,
    w: Vec3,
}

/// Time derivatives of a [`State`].
#[derive(Clone, Copy)]
struct Slope {
    dp: Vec3,
    dv: Vec3,
    dq: Quat,
    dw: Vec3,
}

impl Slope {
    fn of(st: &State, d: &Diag) -> Slope {
        Slope { dp: st.vel, dv: d.acc, dq: rate(st.q, st.w), dw: d.ang_acc }
    }

    /// The RK4 weighted mean `(k1 + 2 k2 + 2 k3 + k4) / 6`.
    fn rk4(k: [&Slope; 4]) -> Slope {
        let mut s = Slope { dp: [0.0; 3], dv: [0.0; 3], dq: [0.0; 4], dw: [0.0; 3] };
        for (j, kj) in k.iter().enumerate() {
            let c = if j == 0 || j == 3 { 1.0 / 6.0 } else { 1.0 / 3.0 };
            for i in 0..3 {
                s.dp[i] += c * kj.dp[i];
                s.dv[i] += c * kj.dv[i];
                s.dw[i] += c * kj.dw[i];
            }
            for i in 0..4 {
                s.dq[i] += c * kj.dq[i];
            }
        }
        s
    }
}

impl State {
    /// The state after `h` seconds at the rates `k`, with the attitude renormalised.
    fn advanced(&self, k: &Slope, h: f64) -> State {
        let mut s = *self;
        for i in 0..3 {
            s.pos[i] += h * k.dp[i];
            s.vel[i] += h * k.dv[i];
            s.w[i] += h * k.dw[i];
        }
        for i in 0..4 {
            s.q[i] += h * k.dq[i];
        }
        s.q = normalize(s.q);
        s
    }
}

/// Mass, balance, inertia and thrust of the flying stack at one instant.
#[derive(Clone, Copy)]
struct Props {
    mass: f64,
    /// Centre of gravity station (m).
    cg: f64,
    thrust: f64,
    burning: bool,
    /// Pitch/yaw moment of inertia about the CG (kg m^2).
    i_long: f64,
    /// Roll moment of inertia (kg m^2).
    i_rot: f64,
    /// Jet damping coefficient `sum mdot (x_exit - x_cg)^2` (kg m^2/s).
    jet: f64,
}

/// The air-relative flow and the mass state at one evaluation.
struct Flow {
    p: Props,
    air: AirState,
    /// Horizontal wind magnitude, gusts included (m/s).
    wind: f64,
    /// Velocity relative to the air (m/s).
    rel: Vec3,
    /// Air speed (m/s).
    s: f64,
    mach: f64,
    re_per_m: f64,
    /// Gravity (m/s^2).
    g: f64,
}

/// Everything one evaluation of the equations of motion produces: the accelerations and the state they were computed from.
#[derive(Clone, Copy)]
struct Diag {
    acc: [f64; 3],
    /// Body-frame angular acceleration (rad/s^2).
    ang_acc: [f64; 3],
    thrust: f64,
    drag: f64,
    mass: f64,
    cg: f64,
    mach: f64,
    air: AirState,
    /// Speed relative to the air (m/s).
    air_speed: f64,
    /// Angle between the rocket axis and the air velocity (rad).
    alpha: f64,
    /// Airframe drag coefficients as flown: scaled by the Monte Carlo factor and faded out as the canopies fill.
    cd: DragBreakdown,
    /// `Cd A` of the deployed devices (m^2).
    open: f64,
    /// Horizontal wind (mean plus gust) magnitude (m/s).
    wind: f64,
    /// Propellant left (kg).
    propellant: f64,
    /// Reynolds number on the stack length.
    reynolds: f64,
    /// Unit vector along the nose direction.
    axis: [f64; 3],
    /// Body angular rate (rad/s); zero unless the rigid body is flying.
    omega: [f64; 3],
    /// Fastest rate of the rotational dynamics (1/s), which limits the RK4 step; zero outside the rigid-body mode.
    stiff: f64,
    /// Natural frequency of the pitch oscillation (rad/s); zero outside the rigid-body mode or without a restoring moment.
    pitch_freq: f64,
    /// Cant of the first actively controlled fin group (rad); 0 without one.
    control_cant: f64,
}
