//! Flight simulation (x east, y north, z up) with two equations of motion ([`FlightModel`]): a 3-DOF point mass and, by default,
//! a rigid body. Both integrate with RK4 (Niskanen 2009, section 4.2).
//!
//! Model summary:
//! * On the rail the rocket is constrained to the rail axis and does not rotate. Off the rail the point-mass model points the
//!   thrust along the velocity relative to the air (perfect weathercocking); the rigid-body model flies the attitude (below).
//!   Gravity varies with altitude in both.
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
//! * Wind is constant with altitude; turbulence adds an Ornstein-Uhlenbeck gust with standard
//!   deviation `intensity x wind_speed`.

use crate::aero::{axial_drag_multiplier, Aero, DragBreakdown, BODY_CROSSFLOW_CD};
use crate::atmosphere::{AirState, Atmosphere};
use crate::drag_table::DragTable;
use crate::rng::Rng;
use crate::staging::{Hits, Timeline};
use crate::vehicle::{Trigger, Vehicle};
use crate::G0;
use std::f64::consts::{FRAC_PI_2, PI};

/// Gust intensity setting (standard deviation as a fraction of the mean wind speed).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Turbulence {
    /// No gusts.
    #[default]
    Off,
    /// 5 % of mean wind.
    Low,
    /// 10 % of mean wind.
    Med,
    /// 20 % of mean wind.
    High,
}

impl Turbulence {
    /// Gust standard deviation as a fraction of mean wind speed.
    pub fn intensity(self) -> f64 {
        match self {
            Turbulence::Off => 0.0,
            Turbulence::Low => 0.05,
            Turbulence::Med => 0.10,
            Turbulence::High => 0.20,
        }
    }
}

/// Launch site and conditions.
#[derive(Clone, Debug, PartialEq)]
pub struct Launch {
    /// Launch rail length (m).
    pub rail_length: f64,
    /// Rail tilt from vertical (rad).
    pub angle_from_vertical: f64,
    /// Azimuth the rail leans towards, clockwise from north (rad).
    pub direction: f64,
    /// Site altitude above sea level (m).
    pub site_altitude: f64,
    /// Ground temperature at the site (K).
    pub ground_temperature: f64,
    /// Mean wind speed (m/s).
    pub wind_speed: f64,
    /// Direction the wind blows *from*, clockwise from north (rad), meteorological convention.
    pub wind_from: f64,
    /// Gust intensity.
    pub turbulence: Turbulence,
    /// Seed for the gust generator.
    pub seed: u64,
}

impl Default for Launch {
    fn default() -> Self {
        Self {
            rail_length: 1.5,
            angle_from_vertical: 0.0,
            direction: 0.0,
            site_altitude: 0.0,
            ground_temperature: 288.15,
            wind_speed: 0.0,
            wind_from: 0.0,
            turbulence: Turbulence::Off,
            seed: 1,
        }
    }
}

/// When a recovery device deploys.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Deploy {
    /// At apogee.
    Apogee,
    /// Descending through this altitude above ground (m).
    Altitude(f64),
    /// At the ejection charge of the motor in the device's stage.
    Ejection,
    /// `delay` seconds after launch (the delay field of the device is the time from launch).
    Launch,
}

/// Parachute or other recovery device.
#[derive(Clone, Debug, PartialEq)]
pub struct RecoveryDevice {
    /// Name used for the deploy event and descent-rate summary.
    pub name: String,
    /// Stage index (firing order) that carries the device.
    pub stage: usize,
    /// Drag coefficient times area (m^2).
    pub cd_area: f64,
    /// Deployment trigger.
    pub deploy: Deploy,
    /// Delay after the trigger (s).
    pub delay: f64,
}

/// Which equations of motion fly the rocket.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FlightModel {
    /// Three degrees of freedom: a point mass whose axis always points into the relative wind (perfect weathercocking, no angle
    /// of attack), Niskanen (2009) section 4.2. Kept for regression and fast sweeps.
    PointMass,
    /// Rigid body with quaternion attitude: pitch, yaw and roll dynamics under the normal force at the centre of pressure
    /// about the moving centre of gravity, aerodynamic, body-crossflow and jet damping, thrust along the body axis, and the
    /// moments of inertia of the parts and of the burning propellant. Wind and gusts produce angle of attack and the rocket
    /// weathercocks, oscillating about the relative wind with the damping of the design. Under a deployed parachute the descent
    /// reverts to the point-mass model.
    #[default]
    RigidBody,
}

/// Everything needed to fly a vehicle.
#[derive(Clone, Debug, PartialEq)]
pub struct SimInput {
    /// The vehicle.
    pub vehicle: Vehicle,
    /// Launch conditions.
    pub launch: Launch,
    /// Recovery devices.
    pub recovery: Vec<RecoveryDevice>,
    /// Fine time step used on the rail and under thrust (s); default 0.001. The rigid-body model shortens it whenever the
    /// pitch dynamics need it.
    pub dt: f64,
    /// Hard stop (s); default 600.
    pub max_time: f64,
    /// CFD drag tables, index `k` for the stack with the lowest `k` stages dropped; missing or `None` flies the build-up.
    pub drag_tables: Vec<Option<DragTable>>,
}

impl SimInput {
    /// Input with default step (1 ms), `max_time` (600 s) and the rigid-body model.
    pub fn new(vehicle: Vehicle, launch: Launch, recovery: Vec<RecoveryDevice>) -> Self {
        Self { vehicle, launch, recovery, dt: 0.001, max_time: 600.0, drag_tables: Vec::new() }
    }
}

/// Flight event kinds.
#[derive(Clone, Debug, PartialEq)]
pub enum EventKind {
    /// A motor ignited (see [`Event::stage`]).
    Ignition,
    /// The rocket left the rail.
    RailExit,
    /// A motor burned out.
    Burnout,
    /// A stage separated (the dropped stage is [`Event::stage`]).
    Separation,
    /// Apogee.
    Apogee,
    /// A recovery device deployed (its name).
    Deploy(String),
    /// Ground contact.
    Landing,
}

/// A timed flight event.
#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    /// Time (s).
    pub t: f64,
    /// What happened.
    pub kind: EventKind,
    /// Altitude above ground (m).
    pub altitude: f64,
    /// Stage involved for ignition/burnout/separation.
    pub stage: Option<usize>,
}

/// One recorded instant of the flight. All values are SI (m, s, kg, N, Pa, K, rad); stations are from the nose tip.
///
/// The flight data set: position and velocity, aerodynamics (Mach, dynamic pressure, Reynolds
/// number, the drag-coefficient build-up), forces, mass and balance, the atmosphere and the recovery device state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sample {
    /// Time (s).
    pub t: f64,
    /// Position (m): x east, y north, z up above the pad.
    pub position: [f64; 3],
    /// Ground-relative velocity (m/s).
    pub velocity: [f64; 3],
    /// Altitude above ground (m).
    pub altitude: f64,
    /// Altitude above sea level (m): the ground altitude of the site plus [`altitude`](Self::altitude).
    pub altitude_msl: f64,
    /// Vertical velocity (m/s).
    pub vertical_velocity: f64,
    /// Horizontal ground speed (m/s).
    pub horizontal_velocity: f64,
    /// Ground speed magnitude (m/s).
    pub speed: f64,
    /// Speed relative to the air, wind and gusts included (m/s).
    pub air_speed: f64,
    /// Distance from the pad measured along the ground (m).
    pub ground_distance: f64,
    /// Flight-path angle above the horizontal (rad): `+pi/2` straight up, `-pi/2` straight down.
    pub flight_path_angle: f64,
    /// Mach number relative to the air.
    pub mach: f64,
    /// Magnitude of the kinematic acceleration (m/s^2).
    pub acceleration: f64,
    /// Signed vertical acceleration (m/s^2).
    pub vertical_acceleration: f64,
    /// Thrust (N).
    pub thrust: f64,
    /// Drag force (N).
    pub drag: f64,
    /// Total airframe drag coefficient (reference-area based): the sum of the parts below, faded out as canopies fill. The
    /// recovery devices' share is [`cd_recovery`](Self::cd_recovery), kept apart because it is hundreds of times larger.
    pub cd: f64,
    /// Airframe skin-friction drag coefficient.
    pub cd_friction: f64,
    /// Airframe pressure (wave) drag coefficient.
    pub cd_pressure: f64,
    /// Airframe base drag coefficient.
    pub cd_base: f64,
    /// Fin drag coefficient.
    pub cd_fins: f64,
    /// Launch lug and rail button drag coefficient.
    pub cd_protuberances: f64,
    /// Share of the drag coefficient carried by deployed recovery devices (`Cd A / A_ref`).
    pub cd_recovery: f64,
    /// Drag area `Cd A` of the deployed recovery devices, canopy inflation included (m^2).
    pub recovery_cd_area: f64,
    /// Total mass (kg).
    pub mass: f64,
    /// Propellant mass remaining in the motors of the stages still flying (kg).
    pub propellant_mass: f64,
    /// Centre of gravity station (m).
    pub cg: f64,
    /// Centre of pressure station at the current Mach and zero angle of attack (m).
    pub cp: f64,
    /// Static margin in calibers at the current Mach, zero angle of attack and current CG.
    pub stability_margin: f64,
    /// Angle between the rocket's axis and the air velocity (rad), 0 flying nose first, `pi` tail first. In the rigid-body model it
    /// is the real angle of attack (wind, gusts and the turn of the flight path make it non-zero). In the point-mass model the
    /// rocket weathercocks perfectly, so it is zero in free flight and non-zero only on the rail (the rail fixes the axis) and
    /// below 1 m/s of air speed.
    pub angle_of_attack: f64,
    /// Horizontal wind speed at the rocket, gusts included (m/s).
    pub wind_speed: f64,
    /// Dynamic pressure `q = 1/2 rho v^2` of the air-relative flow (Pa).
    pub dynamic_pressure: f64,
    /// Air density (kg/m^3).
    pub air_density: f64,
    /// Air temperature (K).
    pub air_temperature: f64,
    /// Static air pressure (Pa).
    pub air_pressure: f64,
    /// Speed of sound (m/s).
    pub speed_of_sound: f64,
    /// Reynolds number based on the length of the flying stack (`rho v L / mu`).
    pub reynolds: f64,
    /// Rate of turn of the rocket's axis, the magnitude of the pitch and yaw rates (rad/s). Zero in the point-mass model, on the
    /// rail and under a parachute.
    pub pitch_rate: f64,
    /// Roll rate about the axis (rad/s), positive in the sense of increasing fin roll angle, the way positive fin cant spins the
    /// rocket. Zero without cant, in the point-mass model, on the rail and under a parachute.
    pub roll_rate: f64,
    /// Elevation of the rocket's axis above the horizontal (rad): `+pi/2` nose straight up, `-pi/2` nose straight down. It lags the
    /// flight-path angle by the angle of attack.
    pub attitude_angle: f64,
}

/// Recorded flight.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Trajectory {
    /// Decimated samples (about 2000), always including the start, end and event instants.
    pub samples: Vec<Sample>,
    /// Events in time order.
    pub events: Vec<Event>,
}

/// Flight summary values.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Summary {
    /// Apogee altitude above ground (m).
    pub apogee: f64,
    /// Time of apogee (s).
    pub time_to_apogee: f64,
    /// Apogee time minus final burnout time (s): the ideal ejection delay.
    pub optimum_delay: f64,
    /// Maximum ground speed before apogee (m/s).
    pub max_velocity: f64,
    /// Maximum Mach number before apogee.
    pub max_mach: f64,
    /// Maximum kinematic acceleration before apogee (in g).
    pub max_acceleration_g: f64,
    /// Speed at rail exit (m/s); 0 if the rocket never left the rail.
    pub rail_exit_velocity: f64,
    /// Static margin at rail exit (cal), at the angle of attack the rocket has there in the rigid-body model (the CP moves forward
    /// with it, so wind lowers it) and at zero in the point-mass model.
    pub stability_off_rail_cal: f64,
    /// `(device name, descent rate m/s)`: vertical speed just before the next device deploys or the ground is hit.
    pub descent_rates: Vec<(String, f64)>,
    /// Speed at ground contact (m/s).
    pub ground_hit_velocity: f64,
    /// Time from ignition to landing (s).
    pub flight_time: f64,
    /// `(distance m, bearing deg clockwise from north)` of the landing point from the pad.
    pub landing: (f64, f64),
    /// Landing point `[east, north]` (m).
    pub landing_position: [f64; 2],
}

/// Result of [`simulate`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlightResult {
    /// Time history and events.
    pub trajectory: Trajectory,
    /// Headline numbers.
    pub summary: Summary,
}

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
            match input.drag_tables.get(k) {
                Some(Some(table)) => aero.with_table(table.clone()),
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
    wind: [f64; 2],
    axis: [f64; 3],
    a_ref: f64,
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
}

impl Ctx<'_> {
    /// Mass, CG station, inertias, thrust and jet damping of the stages still flying at time `t` with `nsep` stages dropped.
    ///
    /// Dry parts carry the inertias of the model (about their own CG). A motor is a thin-walled casing plus the propellant still
    /// in it as a solid cylinder, both as long as the motor and as wide as its diameter, so the propellant's inertia falls
    /// as it burns. Everything is moved to the common CG with the parallel-axis theorem.
    fn mass_props(&self, t: f64, nsep: usize) -> Props {
        let (mut m, mut mx, mut mx2, mut i_long, mut i_rot) = (0.0, 0.0, 0.0, 0.0, 0.0);
        let (mut thrust, mut burning) = (0.0, false);
        // Propellant flow `mdot` and its moments about the nozzle exit stations: sums of mdot, mdot x, mdot x^2.
        let (mut d0, mut d1, mut d2) = (0.0, 0.0, 0.0);
        for (k, st) in self.input.vehicle.stages.iter().enumerate().skip(nsep) {
            let dm = st.dry.mass * self.tw.mass;
            m += dm;
            mx += dm * st.dry.cg;
            mx2 += dm * st.dry.cg * st.dry.cg;
            i_long += st.dry.i_long * self.tw.mass;
            i_rot += st.dry.i_rot * self.tw.mass;
            if let Some(mo) = &st.motor {
                // A motor that has not lit (yet, or ever) is a full dead weight.
                let tau = self.sched.ignition[k].map_or(-1.0, |ign| t - ign);
                let c = &mo.curve;
                let mm = c.mass(tau);
                let xm = mo.cg_station();
                m += mm;
                mx += mm * xm;
                mx2 += mm * xm * xm;
                let (r, l) = (0.5 * c.diameter, c.length);
                let casing = (c.total_mass - c.propellant_mass).max(0.0);
                let propellant = (mm - casing).max(0.0);
                i_long += casing * (0.5 * r * r + l * l / 12.0) + propellant * (0.25 * r * r + l * l / 12.0);
                i_rot += casing * r * r + 0.5 * propellant * r * r;
                let raw = c.thrust(tau);
                if raw * self.tw.thrust > 0.0 {
                    burning = true;
                    thrust += raw * self.tw.thrust;
                }
                let impulse = c.total_impulse();
                if raw > 0.0 && impulse > 0.0 {
                    // Propellant burns in proportion to the impulse delivered (see `ThrustCurve::mass`).
                    let mdot = c.propellant_mass * raw / impulse;
                    let xe = mo.mount_aft_station;
                    d0 += mdot;
                    d1 += mdot * xe;
                    d2 += mdot * xe * xe;
                }
            }
        }
        if m < 1e-3 {
            return Props { mass: 1e-3, cg: 0.0, thrust, burning, i_long: 1e-9, i_rot: 1e-12, jet: 0.0 };
        }
        let cg = mx / m;
        // Parts that carry no inertia data fall back to a uniform rod over the flying stack and a solid cylinder of the reference
        // diameter, so the rotational dynamics are never singular.
        let i_long = {
            let about_cg = i_long + mx2 - m * cg * cg;
            if about_cg > 1e-9 {
                about_cg
            } else {
                let l = self.stack_length(nsep).max(1e-3);
                m * l * l / 12.0
            }
        };
        let i_rot = if i_rot > 1e-12 { i_rot } else { 0.5 * m * (0.5 * self.input.vehicle.reference_diameter).powi(2) }.max(1e-12);
        Props { mass: m, cg, thrust, burning, i_long, i_rot, jet: (d2 - 2.0 * cg * d1 + cg * cg * d0).max(0.0) }
    }

    /// Propellant left in the motors of the stages still flying at time `t`.
    fn propellant(&self, t: f64, nsep: usize) -> f64 {
        let mut left = 0.0;
        for (k, st) in self.input.vehicle.stages.iter().enumerate().skip(nsep) {
            if let Some(mo) = &st.motor {
                let tau = self.sched.ignition[k].map_or(-1.0, |ign| t - ign);
                left += (mo.curve.mass(tau) - (mo.curve.total_mass - mo.curve.propellant_mass)).max(0.0);
            }
        }
        left
    }

    /// Length of the stack that is still flying (m): the vehicle, cut at the aft end of stage `nsep` once stages are dropped.
    fn stack_length(&self, nsep: usize) -> f64 {
        let v = &self.input.vehicle;
        match v.stages.get(nsep) {
            Some(st) if nsep > 0 => v.length().min(st.aft_station),
            _ => v.length(),
        }
    }

    /// Equations of motion at `(t, st)`.
    fn eval(&self, t: f64, st: &State, gust: [f64; 3], nsep: usize, mode: Mode, chutes: &[Chute]) -> Diag {
        let p = self.mass_props(t, nsep);
        let air = self.atm.at_agl(st.pos[2].max(0.0));
        let wind = [self.wind[0] + gust[0], self.wind[1] + gust[1]];
        let rel = [st.vel[0] - wind[0], st.vel[1] - wind[1], st.vel[2] - gust[2]];
        let s = dot(rel, rel).sqrt();
        let flow = Flow {
            p,
            air,
            wind: wind[0].hypot(wind[1]),
            rel,
            s,
            mach: s / air.speed_of_sound,
            re_per_m: s * air.density / air.viscosity,
            g: G0 * (6_371_000.0 / (6_371_000.0 + st.pos[2].max(0.0) + self.launch.site_altitude)).powi(2),
        };
        let d = if mode == Mode::Rigid { self.rigid_eval(st, &flow, nsep) } else { self.point_eval(t, st, &flow, nsep, mode == Mode::Rail, chutes) };
        Diag { propellant: self.propellant(t, nsep), reynolds: flow.re_per_m * self.stack_length(nsep), ..d }
    }

    /// Point mass: the axis follows the relative wind (or the rail), drag opposes the relative velocity.
    fn point_eval(&self, t: f64, st: &State, f: &Flow, nsep: usize, rail: bool, chutes: &[Chute]) -> Diag {
        let (p, s, rel) = (&f.p, f.s, f.rel);
        let dir = if rail || s < 1.0 { self.axis } else { [rel[0] / s, rel[1] / s, rel[2] / s] };
        let q = 0.5 * f.air.density * s * s;
        // Deployed devices: canopy area ramps up (see `Chute`); the airframe's own drag fades out
        // as the canopies fill.
        let full: f64 = chutes.iter().map(|c| c.cd_area).sum();
        let open: f64 = chutes.iter().map(|c| c.area_at(t)).sum();
        let frac = if full > 0.0 { (open / full).min(1.0) } else { 0.0 };
        let mut drag = q * open;
        let mut cd = DragBreakdown::default();
        if frac < 1.0 {
            let b = self.aeros[nsep.min(self.aeros.len() - 1)].drag(f.mach, f.re_per_m, p.burning);
            let w = (1.0 - frac) * self.tw.cd;
            cd = DragBreakdown {
                friction: w * b.friction,
                pressure: w * b.pressure,
                base: w * b.base,
                fins: w * b.fins,
                protuberances: w * b.protuberances,
                total: w * b.total,
            };
            drag += q * cd.total * self.a_ref;
        }
        let rd = if s > 1e-9 { [rel[0] / s, rel[1] / s, rel[2] / s] } else { [0.0; 3] };
        let alpha = if s > 1e-9 { dot(dir, rd).clamp(-1.0, 1.0).acos() } else { 0.0 };
        let mut acc = [0.0; 3];
        for i in 0..3 {
            acc[i] = (p.thrust * dir[i] - drag * rd[i]) / p.mass;
        }
        acc[2] -= f.g;
        if rail {
            let mut au = dot(acc, self.axis);
            if dot(st.vel, self.axis) <= 0.0 && au < 0.0 {
                au = 0.0;
            }
            acc = [au * self.axis[0], au * self.axis[1], au * self.axis[2]];
        }
        Diag {
            acc,
            ang_acc: [0.0; 3],
            thrust: p.thrust,
            drag,
            mass: p.mass,
            cg: p.cg,
            mach: f.mach,
            air: f.air,
            air_speed: s,
            alpha,
            cd,
            open,
            wind: f.wind,
            propellant: 0.0,
            reynolds: 0.0,
            axis: dir,
            omega: [0.0; 3],
            stiff: 0.0,
        }
    }

    /// Rigid body: forces and moments in the body frame (x along the nose, y and z transverse).
    ///
    /// With the lateral air velocity `v_l` and the axial one `u` the angle of attack is `alpha = atan2(|v_l|, u)`; flying tail
    /// first (`alpha > pi/2`) the coefficients are those of the mirror angle `pi - alpha`. The axial force is
    /// `-sign(u) q A_ref Cd k(alpha)` and the normal force `q A_ref CNalpha alpha` opposes `v_l`, acting at the centre of pressure
    /// `x_cp`, which is `x_cg - x_cp` ahead of the centre of gravity (aft when negative): the moment is that lever arm times the
    /// normal force, stabilising when the CP is aft of the CG.
    fn rigid_eval(&self, st: &State, f: &Flow, nsep: usize) -> Diag {
        let aero = &self.aeros[nsep.min(self.aeros.len() - 1)];
        let (p, s) = (&f.p, f.s);
        let vb = rotate_inv(st.q, f.rel);
        let lateral = vb[1].hypot(vb[2]);
        let (alpha, alpha_eff) = if s > 1e-9 {
            let a = lateral.atan2(vb[0]);
            (a, if a > FRAC_PI_2 { PI - a } else { a })
        } else {
            (0.0, 0.0)
        };
        let rho = f.air.density;
        let q = 0.5 * rho * s * s;

        // Axial drag.
        let b = aero.drag(f.mach, f.re_per_m, p.burning);
        let w = self.tw.cd;
        let cd = DragBreakdown {
            friction: w * b.friction,
            pressure: w * b.pressure,
            base: w * b.base,
            fins: w * b.fins,
            protuberances: w * b.protuberances,
            total: w * b.total,
        };
        let axial = -vb[0].signum() * q * cd.total * axial_drag_multiplier(alpha_eff) * self.a_ref;

        // Normal force against the lateral velocity.
        let nf = aero.normal_force(f.mach, alpha_eff);
        let normal = q * self.a_ref * nf.cn_alpha * alpha_eff;
        let (fy, fz) = if lateral > 1e-9 { (-normal * vb[1] / lateral, -normal * vb[2] / lateral) } else { (0.0, 0.0) };

        // Moments about the CG: the normal force at the CP, then damping (fins and bodies at their CPs, crossflow drag on the
        // turning body, the exhaust leaving at the nozzle) and the roll of the canted fins.
        let arm = p.cg - nf.cp;
        let wl = st.w[1].hypot(st.w[2]);
        let linear = 0.5 * rho * s * self.a_ref * nf.damping_sum(p.cg) + p.jet;
        let crossflow = 0.5 * rho * BODY_CROSSFLOW_CD * aero.crossflow_damping(p.cg);
        let damp = linear + crossflow * wl;
        let roll = aero.roll(f.mach);
        // Roll moment in the sense of increasing fin angle (the right-handed body roll rate is its negative).
        let roll_moment = if s > 1e-9 { q * (roll.forcing - roll.damping * st.w[0] / s) } else { 0.0 };
        let (mx, my, mz) = (-roll_moment, -arm * fz - damp * st.w[1], arm * fy - damp * st.w[2]);

        // Euler's equations of an axisymmetric body (I_y = I_z = i_long, I_x = i_rot).
        let (i, ix) = (p.i_long, p.i_rot);
        let ang_acc = [mx / ix, (my + (i - ix) * st.w[2] * st.w[0]) / i, (mz + (ix - i) * st.w[0] * st.w[1]) / i];

        // Translation: body forces into the inertial frame.
        let force = rotate(st.q, [p.thrust + axial, fy, fz]);
        let aero_force = rotate(st.q, [axial, fy, fz]);
        let acc = [force[0] / p.mass, force[1] / p.mass, force[2] / p.mass - f.g];
        let rd = if s > 1e-9 { [f.rel[0] / s, f.rel[1] / s, f.rel[2] / s] } else { [0.0; 3] };

        // The fastest rotational rates (pitch frequency, damping, roll damping) bound the RK4 step.
        let wn = (q * self.a_ref * nf.cn_alpha.abs() * (nf.cp - p.cg).abs() / i).sqrt();
        let stiff = wn.max(damp / i).max(0.5 * rho * s.max(1.0) * roll.damping.abs() / ix);
        Diag {
            acc,
            ang_acc,
            thrust: p.thrust,
            drag: -dot(aero_force, rd),
            mass: p.mass,
            cg: p.cg,
            mach: f.mach,
            air: f.air,
            air_speed: s,
            alpha,
            cd,
            open: 0.0,
            wind: f.wind,
            propellant: 0.0,
            reynolds: 0.0,
            axis: rotate(st.q, [1.0, 0.0, 0.0]),
            omega: st.w,
            stiff,
        }
    }
}

/// Flies `input` and returns the trajectory and summary.
pub fn simulate(input: &SimInput) -> FlightResult {
    let aeros = build_aeros(input);
    run(input, &input.launch, Tweaks::NOMINAL, &aeros)
}

pub(crate) fn run(input: &SimInput, launch: &Launch, tw: Tweaks, aeros: &[Aero]) -> FlightResult {
    let v = &input.vehicle;
    let n = v.stages.len();
    let atm = Atmosphere::new(launch.site_altitude, launch.ground_temperature);
    let th = launch.angle_from_vertical;
    let axis = [th.sin() * launch.direction.sin(), th.sin() * launch.direction.cos(), th.cos()];
    let wind = [-launch.wind_speed * launch.wind_from.sin(), -launch.wind_speed * launch.wind_from.cos()];
    let mut ctx = Ctx { input, launch, tw, aeros, atm, sched: Timeline::resolve(v, &Hits::none(n)), wind, axis, a_ref: v.reference_area().max(1e-12) };

    // Altitude triggers that have fired (the time they fired, before their delay), and which staging events are on record.
    let mut hits = Hits::none(n);
    let (mut done_ignition, mut done_burnout, mut done_separation) = (vec![false; n], vec![false; n], vec![false; n]);
    // Step boundaries beyond the staging timeline (deployments relative to apogee or altitude).
    let mut step_times: Vec<f64> = Vec::new();
    let mut deploy_at: Vec<Option<f64>> = input.recovery.iter().map(|d| (d.deploy == Deploy::Launch).then_some(d.delay)).collect();
    let mut deployed = vec![false; input.recovery.len()];

    let mut rng = Rng::new(launch.seed ^ 0xA5A5_5A5A_1234_5678);
    let gust_sigma = launch.turbulence.intensity() * launch.wind_speed;
    let tau_gust = 1.5;
    let mut gust = [0.0; 3];

    let mut raws: Vec<Raw> = Vec::new();
    let mut events: Vec<Event> = Vec::new();
    let mut event_idx: Vec<usize> = Vec::new();
    let (mut t, mut pos, mut vel) = (0.0_f64, [0.0; 3], [0.0; 3]);
    // Attitude (body to inertial) and body angular rate: the rail axis at rest to begin with.
    let (mut att, mut spin) = (from_axis(axis), [0.0; 3]);
    let rigid_model = input.model == FlightModel::RigidBody;
    let mode_of = |rail: bool, chutes: &[Chute]| {
        if rail {
            Mode::Rail
        } else if rigid_model && chutes.is_empty() {
            Mode::Rigid
        } else {
            Mode::Point
        }
    };
    let mut on_rail = true;
    let mut apogee: Option<(f64, f64)> = None;
    let mut rail_exit_velocity = 0.0;
    let mut stability_off_rail = 0.0;
    let mut rates: Vec<(String, f64)> = Vec::new();
    let mut label: Option<String> = None;
    let mut chutes: Vec<Chute> = Vec::new();
    let mut last_event_t = f64::NEG_INFINITY;
    let mut landed = false;
    let mut ground_hit = 0.0;
    let mut ground_xy = [0.0; 2];
    let mut final_raw: Option<Raw> = None;

    let push_event = |events: &mut Vec<Event>, event_idx: &mut Vec<usize>, raws: &Vec<Raw>, e: Event| {
        event_idx.push(raws.len());
        events.push(e);
    };

    while t < input.max_time {
        let nsep = ctx.sched.separated(t);
        // Staging events that are due; a stage dropped before its burnout never reports one.
        let mut due: Vec<Event> = Vec::new();
        for k in 0..n {
            let separation = ctx.sched.separation[k];
            if v.stages[k].motor.is_some() {
                if let Some(ti) = ctx.sched.ignition[k].filter(|ti| !done_ignition[k] && *ti <= t + 1e-9) {
                    done_ignition[k] = true;
                    due.push(Event { t: ti, kind: EventKind::Ignition, altitude: 0.0, stage: Some(k) });
                }
                if let Some(tb) = ctx.sched.burnout[k].filter(|tb| !done_burnout[k] && *tb <= t + 1e-9) {
                    done_burnout[k] = true;
                    if !separation.is_some_and(|s| s < tb - 1e-9) {
                        due.push(Event { t: tb, kind: EventKind::Burnout, altitude: 0.0, stage: Some(k) });
                    }
                }
            }
            if let Some(ts) = separation.filter(|ts| !done_separation[k] && *ts <= t + 1e-9) {
                done_separation[k] = true;
                due.push(Event { t: ts, kind: EventKind::Separation, altitude: 0.0, stage: Some(k) });
            }
        }
        due.sort_by(|a, b| a.t.total_cmp(&b.t));
        for mut e in due {
            e.t = t;
            e.altitude = pos[2];
            last_event_t = t;
            push_event(&mut events, &mut event_idx, &raws, e);
        }
        // Ejection deployments wait for their stage's charge, whenever the timeline learns of it.
        for (i, d) in input.recovery.iter().enumerate() {
            if d.deploy == Deploy::Ejection && deploy_at[i].is_none() {
                deploy_at[i] = ctx.sched.ejection.get(d.stage).copied().flatten().map(|te| te + d.delay);
            }
        }
        // Devices whose stage has been dropped never deploy.
        for (i, d) in input.recovery.iter().enumerate() {
            if d.stage < nsep {
                deployed[i] = true;
            }
        }
        let mode = mode_of(on_rail, &chutes);
        let d0 = ctx.eval(t, &State { pos, vel, q: att, w: spin }, gust, nsep, mode, &chutes);
        let raw = Raw { t, pos, vel, nsep, d: d0 };
        raws.push(raw);

        let last_burnout = ctx.sched.burnout.iter().flatten().copied().fold(0.0, f64::max);
        if on_rail && t > last_burnout + 0.5 && dot(vel, axis) <= 1e-9 {
            // Never left the pad: the thrust cannot lift the rocket.
            landed = true;
            final_raw = Some(raw);
            push_event(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::Landing, altitude: 0.0, stage: None });
            break;
        }

        // Step size.
        let burning = d0.thrust > 0.0;
        let fine = on_rail || burning || t - last_event_t < 0.05;
        let mut dt = if fine {
            input.dt
        } else if chutes.iter().any(|c| t < c.t_deploy + c.t_inflate) {
            // Step finely through canopy inflation.
            input.dt.max(0.005)
        } else if !chutes.is_empty() {
            input.dt.max(0.02)
        } else {
            input.dt.max(0.01)
        };
        if mode == Mode::Rigid && d0.stiff > 0.0 {
            // Keep the fastest pitch/roll dynamics well inside the RK4 stability limit (about 2.8 / rate).
            dt = dt.min((0.3 / d0.stiff).max(1e-4));
        }
        let mut boundary = f64::INFINITY;
        for bt in ctx.sched.times().chain(step_times.iter().copied()).chain(deploy_at.iter().flatten().copied()) {
            if bt > t + 1e-9 && bt < boundary {
                boundary = bt;
            }
        }
        if boundary - t < dt {
            dt = boundary - t;
        } else if boundary - t < 1.5 * dt {
            dt = 0.5 * (boundary - t);
        }
        if dt < 1e-9 {
            dt = 1e-9;
        }

        // Gust update (held constant over the step).
        if gust_sigma > 0.0 {
            let a = (-dt / tau_gust).exp();
            let b = gust_sigma * (1.0 - a * a).sqrt();
            for (i, gi) in gust.iter_mut().enumerate() {
                let scale = if i == 2 { 0.5 } else { 1.0 };
                *gi = *gi * a + b * scale * rng.normal();
            }
        }

        // RK4 on position, velocity, attitude and body rate.
        let s0 = State { pos, vel, q: att, w: spin };
        let k1 = Slope::of(&s0, &d0);
        let s2 = s0.advanced(&k1, 0.5 * dt);
        let k2 = Slope::of(&s2, &ctx.eval(t + 0.5 * dt, &s2, gust, nsep, mode, &chutes));
        let s3 = s0.advanced(&k2, 0.5 * dt);
        let k3 = Slope::of(&s3, &ctx.eval(t + 0.5 * dt, &s3, gust, nsep, mode, &chutes));
        let s4 = s0.advanced(&k3, dt);
        let k4 = Slope::of(&s4, &ctx.eval(t + dt, &s4, gust, nsep, mode, &chutes));
        let next = s0.advanced(&Slope::rk4([&k1, &k2, &k3, &k4]), dt);
        let (t0, pos0, vel0) = (t, pos, vel);
        t += dt;
        (pos, vel, att, spin) = (next.pos, next.vel, next.q, next.w);

        // Rail exit.
        if on_rail {
            if dot(pos, axis) >= launch.rail_length {
                on_rail = false;
                rail_exit_velocity = dot(vel, vel).sqrt();
                let air = atm.at_agl(pos[2]);
                let rel = [vel[0] - wind[0], vel[1] - wind[1], vel[2]];
                let speed = dot(rel, rel).sqrt();
                let mach = speed / air.speed_of_sound;
                let cg = ctx.mass_props(t, nsep).cg;
                // The rigid body meets the air at the angle between its axis and the relative wind (the CP moves forward with it);
                // the point mass weathercocks perfectly.
                let alpha = if rigid_model && speed > 1e-9 { (dot(axis, rel) / speed).clamp(-1.0, 1.0).acos() } else { 0.0 };
                stability_off_rail = aeros[nsep.min(aeros.len() - 1)].margin_cal(mach, alpha, cg);
                last_event_t = t;
                push_event(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::RailExit, altitude: pos[2], stage: None });
            }
            continue;
        }

        // Altitude triggers of the staging fire the first step the rocket is at or above the altitude on the way up.
        if apogee.is_none() {
            let mut fired = false;
            for k in 0..n {
                if let Some(Trigger::Altitude(h)) = v.stages[k].motor.as_ref().map(|m| m.ignition.trigger) {
                    let armed = k == 0 || ctx.sched.ignition[k - 1].is_some_and(|i| i <= t);
                    if armed && hits.ignition[k].is_none() && pos[2] >= h {
                        hits.ignition[k] = Some(t);
                        fired = true;
                    }
                }
                if let (true, Trigger::Altitude(h)) = (k + 1 < n, v.stages[k].separation.trigger) {
                    let armed = ctx.sched.ignition[k].is_some_and(|i| i <= t);
                    if armed && hits.separation[k].is_none() && pos[2] >= h {
                        hits.separation[k] = Some(t);
                        fired = true;
                    }
                }
            }
            if fired {
                ctx.sched = Timeline::resolve(v, &hits);
            }
        }

        // Apogee.
        if apogee.is_none() && vel[2] <= 0.0 {
            let (ta, za) = if vel0[2] > 0.0 {
                let f = vel0[2] / (vel0[2] - vel[2]);
                (t0 + f * dt, pos0[2] + f * (pos[2] - pos0[2]))
            } else {
                (t, pos[2])
            };
            apogee = Some((ta, za));
            last_event_t = t;
            push_event(&mut events, &mut event_idx, &raws, Event { t: ta, kind: EventKind::Apogee, altitude: za, stage: None });
            for (i, d) in input.recovery.iter().enumerate() {
                if d.deploy == Deploy::Apogee && !deployed[i] {
                    deploy_at[i] = Some(ta + d.delay);
                    if ta + d.delay > t {
                        step_times.push(ta + d.delay);
                    }
                }
            }
        }
        if apogee.is_some() {
            for (i, d) in input.recovery.iter().enumerate() {
                if let Deploy::Altitude(h) = d.deploy {
                    if !deployed[i] && deploy_at[i].is_none() && pos[2] <= h {
                        deploy_at[i] = Some(t + d.delay);
                        if d.delay > 1e-9 {
                            step_times.push(t + d.delay);
                        }
                    }
                }
            }
        }
        for (i, d) in input.recovery.iter().enumerate() {
            if let Some(dt_at) = deploy_at[i] {
                if !deployed[i] && t >= dt_at - 1e-9 {
                    deployed[i] = true;
                    let rel_speed = ((vel[0] - wind[0]).powi(2) + (vel[1] - wind[1]).powi(2) + vel[2].powi(2)).sqrt();
                    chutes.push(Chute::new(d.cd_area, t, rel_speed));
                    // Under a canopy the descent is a point mass: the attitude follows the relative wind from here on.
                    spin = [0.0; 3];
                    if let Some(l) = label.take() {
                        rates.push((l, vel[2].abs()));
                    }
                    label = Some(d.name.clone());
                    last_event_t = t;
                    push_event(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::Deploy(d.name.clone()), altitude: pos[2], stage: None });
                }
            }
        }

        // Landing.
        if pos[2] <= 0.0 {
            let f = if pos0[2] > pos[2] { pos0[2] / (pos0[2] - pos[2]) } else { 1.0 };
            let tl = t0 + f * dt;
            let mut lp = [0.0; 3];
            let mut lv = [0.0; 3];
            for i in 0..3 {
                lp[i] = pos0[i] + f * (pos[i] - pos0[i]);
                lv[i] = vel0[i] + f * (vel[i] - vel0[i]);
            }
            lp[2] = 0.0;
            t = tl;
            pos = lp;
            vel = lv;
            ground_hit = dot(lv, lv).sqrt();
            ground_xy = [lp[0], lp[1]];
            if let Some(l) = label.take() {
                rates.push((l, ground_hit));
            }
            landed = true;
            let d = ctx.eval(t, &State { pos, vel, q: att, w: spin }, gust, nsep, mode_of(false, &chutes), &chutes);
            final_raw = Some(Raw { t, pos, vel, nsep, d: Diag { thrust: 0.0, ..d } });
            push_event(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::Landing, altitude: 0.0, stage: None });
            break;
        }
    }
    if let Some(r) = final_raw {
        raws.push(r);
    } else if !landed {
        // Hit the time limit mid-flight: record the final state.
        let nsep = ctx.sched.separated(t);
        let d = ctx.eval(t, &State { pos, vel, q: att, w: spin }, gust, nsep, mode_of(on_rail, &chutes), &chutes);
        raws.push(Raw { t, pos, vel, nsep, d });
        ground_xy = [pos[0], pos[1]];
        ground_hit = dot(vel, vel).sqrt();
    }

    // Summary over the ascent (up to apogee, or the whole flight if there was none).
    let t_end_ascent = apogee.map_or(f64::INFINITY, |a| a.0);
    let (mut max_v, mut max_m, mut max_a) = (0.0_f64, 0.0_f64, 0.0_f64);
    for r in raws.iter().filter(|r| r.t <= t_end_ascent) {
        max_v = max_v.max(dot(r.vel, r.vel).sqrt());
        max_m = max_m.max(r.d.mach);
        max_a = max_a.max(dot(r.d.acc, r.d.acc).sqrt());
    }
    let final_burn = n.checked_sub(1).and_then(|k| ctx.sched.burnout[k]).unwrap_or(0.0);
    let flight_time = raws.last().map_or(0.0, |r| r.t);
    let bearing = ground_xy[0].atan2(ground_xy[1]).to_degrees().rem_euclid(360.0);
    let summary = Summary {
        apogee: apogee.map_or(0.0, |a| a.1),
        time_to_apogee: apogee.map_or(0.0, |a| a.0),
        optimum_delay: apogee.map_or(0.0, |a| (a.0 - final_burn).max(0.0)),
        max_velocity: max_v,
        max_mach: max_m,
        max_acceleration_g: max_a / G0,
        rail_exit_velocity,
        stability_off_rail_cal: stability_off_rail,
        descent_rates: rates,
        ground_hit_velocity: ground_hit,
        flight_time,
        landing: (ground_xy[0].hypot(ground_xy[1]), bearing),
        landing_position: ground_xy,
    };

    // Decimate.
    let len = raws.len();
    let stride = len.div_ceil(2000).max(1);
    let mut keep = vec![false; len];
    for (i, k) in keep.iter_mut().enumerate() {
        *k = i % stride == 0 || i + 1 == len;
    }
    for &i in &event_idx {
        if i < len {
            keep[i] = true;
        }
    }
    let samples = raws
        .iter()
        .zip(&keep)
        .filter(|(_, &k)| k)
        .map(|(r, _)| {
            let d = &r.d;
            let speed = dot(r.vel, r.vel).sqrt();
            let horizontal = r.vel[0].hypot(r.vel[1]);
            let aero = &aeros[r.nsep.min(aeros.len() - 1)];
            let cp = aero.stability(d.mach, 0.0).cp;
            let recovery = d.open / ctx.a_ref;
            Sample {
                t: r.t,
                position: r.pos,
                velocity: r.vel,
                altitude: r.pos[2],
                altitude_msl: launch.site_altitude + r.pos[2],
                vertical_velocity: r.vel[2],
                horizontal_velocity: horizontal,
                speed,
                air_speed: d.air_speed,
                ground_distance: r.pos[0].hypot(r.pos[1]),
                flight_path_angle: r.vel[2].atan2(horizontal),
                mach: d.mach,
                acceleration: dot(d.acc, d.acc).sqrt(),
                vertical_acceleration: d.acc[2],
                thrust: d.thrust,
                drag: d.drag,
                cd: d.cd.total,
                cd_friction: d.cd.friction,
                cd_pressure: d.cd.pressure,
                cd_base: d.cd.base,
                cd_fins: d.cd.fins,
                cd_protuberances: d.cd.protuberances,
                cd_recovery: recovery,
                recovery_cd_area: d.open,
                mass: d.mass,
                propellant_mass: d.propellant,
                cg: d.cg,
                cp,
                stability_margin: (cp - d.cg) / aero.reference_diameter(),
                angle_of_attack: d.alpha,
                wind_speed: d.wind,
                dynamic_pressure: 0.5 * d.air.density * d.air_speed * d.air_speed,
                air_density: d.air.density,
                air_temperature: d.air.temperature,
                air_pressure: d.air.pressure,
                speed_of_sound: d.air.speed_of_sound,
                reynolds: d.reynolds,
                pitch_rate: d.omega[1].hypot(d.omega[2]),
                roll_rate: -d.omega[0],
                attitude_angle: d.axis[2].atan2(d.axis[0].hypot(d.axis[1])),
            }
        })
        .collect();
    FlightResult { trajectory: Trajectory { samples, events }, summary }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::*;

    /// A 38 mm rocket with four fins and a 0.23 kg motor holding 0.105 kg of propellant.
    fn vehicle() -> Vehicle {
        let r = 0.038;
        Vehicle {
            reference_diameter: 2.0 * r,
            body: vec![
                BodySegment { station: 0.0, length: 0.30, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::TangentOgive), roughness: 30e-6 },
                BodySegment { station: 0.30, length: 0.80, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 30e-6 },
            ],
            tube_fins: vec![],
            fin_sets: vec![FinSet {
                count: 4,
                station: 0.93,
                body_radius: r,
                root_chord: 0.17,
                tip_chord: 0.07,
                span: 0.10,
                sweep: 0.09,
                thickness: 0.003,
                cross_section: FinCrossSection::Rounded,
                cant: 0.0,
                roughness: 30e-6,
                planform: FinPlanform::Trapezoidal,
                angles: Vec::new(),
                parent: 0,
                bodies: 1,
                mid_chord_cos: None,
                root_extent: None,
            }],
            protuberances: vec![],
            stages: vec![StageMass {
                dry: MassProps { mass: 1.07, cg: 0.62, i_long: 0.1, i_rot: 0.0008 },
                fore_station: 0.0,
                aft_station: 1.10,
                motor: Some(StageMotor {
                    curve: ThrustCurve {
                        points: vec![(0.0, 0.0), (0.05, 150.0), (1.0, 120.0), (1.2, 0.0)],
                        propellant_mass: 0.105,
                        total_mass: 0.230,
                        length: 0.205,
                        diameter: 0.038,
                    },
                    mount_aft_station: 1.10,
                    ignition: Timing::LAUNCH,
                    ejection_delay: None,
                }),
                separation: Timing::BURNOUT,
            }],
        }
    }

    fn with_ctx<R>(v: Vehicle, f: impl FnOnce(&Ctx<'_>) -> R) -> R {
        let input = SimInput::new(v, Launch::default(), vec![]);
        let aeros = build_aeros(&input.vehicle);
        let n = input.vehicle.stages.len();
        let ctx = Ctx {
            input: &input,
            launch: &input.launch,
            tw: Tweaks::NOMINAL,
            aeros: &aeros,
            atm: Atmosphere::new(0.0, 288.15),
            sched: Timeline::resolve(&input.vehicle, &Hits::none(n)),
            wind: [0.0; 2],
            axis: [0.0, 0.0, 1.0],
            a_ref: input.vehicle.reference_area(),
        };
        f(&ctx)
    }

    #[test]
    fn inertia_is_the_parallel_axis_sum_and_the_propellant_part_falls_as_it_burns() {
        with_ctx(vehicle(), |ctx| {
            let st = &ctx.input.vehicle.stages[0];
            let (dry, mo) = (st.dry, st.motor.as_ref().unwrap());
            let (r, l, xm) = (0.019, 0.205, mo.cg_station());
            let (casing, prop) = (0.230 - 0.105, 0.105);
            // Before ignition: the full motor is a dead weight.
            let p0 = ctx.mass_props(-5.0, 0);
            let m = dry.mass + 0.230;
            let cg = (dry.mass * dry.cg + 0.230 * xm) / m;
            assert!((p0.mass - m).abs() < 1e-12 && (p0.cg - cg).abs() < 1e-12 && p0.thrust == 0.0 && p0.jet == 0.0);
            let motor_long = casing * (0.5 * r * r + l * l / 12.0) + prop * (0.25 * r * r + l * l / 12.0);
            let hand = dry.i_long + dry.mass * (dry.cg - cg).powi(2) + motor_long + 0.230 * (xm - cg).powi(2);
            assert!((p0.i_long - hand).abs() < 1e-9, "{} vs {hand}", p0.i_long);
            assert!((p0.i_rot - (dry.i_rot + casing * r * r + 0.5 * prop * r * r)).abs() < 1e-12);
            // After burnout: only the casing's roll inertia is left, the propellant's half m r^2 is gone.
            let p1 = ctx.mass_props(10.0, 0);
            assert!((p0.i_rot - p1.i_rot - 0.5 * prop * r * r).abs() < 1e-12, "{} {}", p0.i_rot, p1.i_rot);
            assert!((p0.mass - p1.mass - prop).abs() < 1e-12 && p1.cg < p0.cg && p1.thrust == 0.0);
            // During the burn the jet damping is mdot (x_exit - x_cg)^2 with mdot = thrust x propellant / impulse.
            let (tau, c) = (0.5, &mo.curve);
            let pm = ctx.mass_props(tau, 0);
            let mdot = c.propellant_mass * c.thrust(tau) / c.total_impulse();
            assert!((pm.jet - mdot * (mo.mount_aft_station - pm.cg).powi(2)).abs() < 1e-12, "{} {}", pm.jet, mdot);
            assert!(pm.thrust > 100.0 && pm.i_long < p0.i_long + 1e-3);
        });
    }

    #[test]
    fn a_vehicle_without_inertia_data_gets_a_rod_and_a_cylinder() {
        let mut v = vehicle();
        v.stages[0].dry = MassProps { mass: 1.0, cg: 0.55, i_long: 0.0, i_rot: 0.0 };
        v.stages[0].motor = None;
        with_ctx(v, |ctx| {
            let p = ctx.mass_props(0.0, 0);
            assert!((p.i_long - 1.0 * 1.1 * 1.1 / 12.0).abs() < 1e-12, "{}", p.i_long);
            assert!((p.i_rot - 0.5 * 1.0 * 0.038 * 0.038).abs() < 1e-12);
        });
    }

    /// State of the vehicle flying straight up at `v` m/s with body rate `w`.
    fn flying(v: f64, w: Vec3) -> State {
        State { pos: [0.0, 0.0, 100.0], vel: [0.0, 0.0, v], q: from_axis([0.0, 0.0, 1.0]), w }
    }

    #[test]
    fn pitch_rate_is_damped_by_the_strip_sum_of_the_lifting_components() {
        with_ctx(vehicle(), |ctx| {
            let (v, wy) = (60.0, 0.4);
            let d = ctx.eval(100.0, &flying(v, [0.0, wy, 0.0]), [0.0; 3], 0, Mode::Rigid, &[]);
            let p = ctx.mass_props(100.0, 0);
            // Coasting after burnout, no angle of attack: the only moment is the damping.
            assert_eq!(d.thrust, 0.0);
            let rho = d.air.density;
            let aero = &ctx.aeros[0];
            let st = aero.stability(v / d.air.speed_of_sound, 0.0);
            let strip: f64 = st.components.iter().map(|(_, cna, x)| cna * (x - p.cg).powi(2)).sum();
            let linear = 0.5 * rho * v * ctx.a_ref * strip;
            let crossflow = 0.5 * rho * BODY_CROSSFLOW_CD * aero.crossflow_damping(p.cg) * wy;
            let moment = d.ang_acc[1] * p.i_long;
            assert!((moment + (linear + crossflow) * wy).abs() < 1e-9 * moment.abs(), "{moment} vs {}", -(linear + crossflow) * wy);
            // It opposes the rate, in either sense, and does not couple into the other axes.
            let flipped = ctx.eval(100.0, &flying(v, [0.0, -wy, 0.0]), [0.0; 3], 0, Mode::Rigid, &[]);
            assert!((flipped.ang_acc[1] + d.ang_acc[1]).abs() < 1e-12 && d.ang_acc[1] < 0.0);
            assert!(d.ang_acc[0].abs() < 1e-12 && d.ang_acc[2].abs() < 1e-12);
            // The damping grows with the air speed (a viscous-like term plus the quadratic crossflow).
            let fast = ctx.eval(100.0, &flying(2.0 * v, [0.0, wy, 0.0]), [0.0; 3], 0, Mode::Rigid, &[]);
            assert!(fast.ang_acc[1] < 1.9 * d.ang_acc[1]);
        });
    }

    #[test]
    fn a_stable_rocket_is_pushed_back_towards_the_relative_wind_and_an_unstable_one_away() {
        // Moving up through a 5 m/s crosswind blowing towards -x: the air velocity relative to the rocket is (+5, 0, 50), so the
        // nose (+z) should turn towards +x if the rocket is stable.
        for (cg, stable) in [(0.62, true), (0.98, false)] {
            let mut v = vehicle();
            v.stages[0].dry.cg = cg;
            with_ctx(v, |ctx| {
                let st = State { pos: [0.0, 0.0, 100.0], vel: [0.0, 0.0, 50.0], q: from_axis([0.0, 0.0, 1.0]), w: [0.0; 3] };
                let d = ctx.eval(100.0, &st, [-5.0, 0.0, 0.0], 0, Mode::Rigid, &[]);
                // The body frame of `from_axis` is x_b = +z, y_b = +y, z_b = -x: turning the nose towards +x is a rotation about +y_b.
                let toward_wind = d.ang_acc[1];
                assert_eq!(toward_wind > 0.0, stable, "cg {cg}: {:?}", d.ang_acc);
                // Both feel the same side force towards -x (the normal force opposes the lateral motion).
                assert!(d.acc[0] < 0.0 && d.alpha > 0.05);
            });
        }
    }

    #[test]
    fn the_step_limit_tracks_the_pitch_frequency() {
        with_ctx(vehicle(), |ctx| {
            let d = ctx.eval(100.0, &flying(100.0, [0.0; 3]), [0.0; 3], 0, Mode::Rigid, &[]);
            let p = ctx.mass_props(100.0, 0);
            let nf = ctx.aeros[0].normal_force(d.mach, 0.0);
            let q = 0.5 * d.air.density * 100.0 * 100.0;
            let wn = (q * ctx.a_ref * nf.cn_alpha * (nf.cp - p.cg) / p.i_long).sqrt();
            assert!(d.stiff >= wn - 1e-9 && d.stiff < 4.0 * wn, "{} vs {wn}", d.stiff);
            // wn^2 = q A CNalpha (x_cp - x_cg) / I: tens of rad/s for this rocket at 100 m/s.
            assert!((5.0..100.0).contains(&wn), "{wn}");
        });
    }
}
