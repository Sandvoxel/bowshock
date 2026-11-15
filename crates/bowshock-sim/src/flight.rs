//! 3-DOF point-mass flight simulation (x east, y north, z up) in the style of OpenRocket's
//! "simple" 3-DOF stepper, integrated with RK4.
//!
//! Model summary:
//! * On the rail the rocket is constrained to the rail axis. Off the rail the thrust acts along
//!   the velocity relative to the air (perfect weathercocking), drag opposes it, and gravity
//!   varies with altitude.
//! * Drag comes from [`Aero::drag`] at the current Mach/Reynolds number with the motor on/off
//!   (base drag is reduced while any motor burns). Under a deployed recovery device the drag is
//!   `0.5 rho v^2 CdA` summed over deployed devices.
//! * Stages are in firing order (see [`StageMass`]). The lower stage drops at
//!   `burnout + delay` of the next stage's ignition rule (or at burnout when the upper stage ignites at
//!   launch). Only the remaining upper stack is flown afterwards; the aerodynamics are
//!   rebuilt for the truncated vehicle. Recovery devices on dropped stages are ignored.
//! * Wind is constant with altitude; turbulence adds an Ornstein-Uhlenbeck gust with standard
//!   deviation `intensity x wind_speed`.

use crate::aero::Aero;
use crate::atmosphere::Atmosphere;
use crate::rng::Rng;
use crate::vehicle::{Ignition, Vehicle};
use crate::G0;

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

/// Everything needed to fly a vehicle.
#[derive(Clone, Debug, PartialEq)]
pub struct SimInput {
    /// The vehicle.
    pub vehicle: Vehicle,
    /// Launch conditions.
    pub launch: Launch,
    /// Recovery devices.
    pub recovery: Vec<RecoveryDevice>,
    /// Fine time step used on the rail and under thrust (s); default 0.001.
    pub dt: f64,
    /// Hard stop (s); default 600.
    pub max_time: f64,
}

impl SimInput {
    /// Input with default step (1 ms) and `max_time` (600 s).
    pub fn new(vehicle: Vehicle, launch: Launch, recovery: Vec<RecoveryDevice>) -> Self {
        Self { vehicle, launch, recovery, dt: 0.001, max_time: 600.0 }
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

/// One recorded instant of the flight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Time (s).
    pub t: f64,
    /// Position (m): x east, y north, z up above the pad.
    pub position: [f64; 3],
    /// Ground-relative velocity (m/s).
    pub velocity: [f64; 3],
    /// Altitude above ground (m).
    pub altitude: f64,
    /// Vertical velocity (m/s).
    pub vertical_velocity: f64,
    /// Ground speed magnitude (m/s).
    pub speed: f64,
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
    /// Total mass (kg).
    pub mass: f64,
    /// Static margin in calibers at the current Mach, zero angle of attack and current CG.
    pub stability_margin: f64,
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
    /// Static margin at rail exit (cal).
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

struct Schedule {
    ignition: Vec<f64>,
    burnout: Vec<f64>,
    separation: Vec<f64>,
    ejection: Vec<Option<f64>>,
}

fn schedule(v: &Vehicle) -> Schedule {
    let n = v.stages.len();
    let (mut ign, mut bo) = (vec![0.0; n], vec![0.0; n]);
    for k in 0..n {
        let prev = if k == 0 { 0.0 } else { bo[k - 1] };
        match &v.stages[k].motor {
            Some(m) => {
                ign[k] = match m.ignition {
                    Ignition::Launch => 0.0,
                    Ignition::AfterPreviousBurnout { delay } => prev + delay,
                };
                bo[k] = ign[k] + m.curve.burn_time();
            }
            None => {
                ign[k] = prev;
                bo[k] = prev;
            }
        }
    }
    let mut sep = vec![f64::INFINITY; n];
    for k in 0..n.saturating_sub(1) {
        let delay = match v.stages[k + 1].motor.as_ref().map(|m| m.ignition) {
            Some(Ignition::AfterPreviousBurnout { delay }) => delay,
            _ => 0.0,
        };
        sep[k] = bo[k] + delay;
    }
    let ej = (0..n).map(|k| v.stages[k].motor.as_ref().and_then(|m| m.ejection_delay).map(|d| bo[k] + d)).collect();
    Schedule { ignition: ign, burnout: bo, separation: sep, ejection: ej }
}

/// One aerodynamic model per separation level (0 = full stack, k = lowest k stages dropped).
pub(crate) fn build_aeros(v: &Vehicle) -> Vec<Aero> {
    let n = v.stages.len().max(1);
    (0..n).map(|k| if k == 0 { Aero::new(v) } else { Aero::new(&v.truncated(v.stages[k].aft_station)) }).collect()
}

#[derive(Clone, Copy)]
struct Raw {
    t: f64,
    pos: [f64; 3],
    vel: [f64; 3],
    acc: [f64; 3],
    thrust: f64,
    drag: f64,
    mass: f64,
    cg: f64,
    mach: f64,
    nsep: usize,
}

struct Ctx<'a> {
    input: &'a SimInput,
    launch: &'a Launch,
    tw: Tweaks,
    aeros: &'a [Aero],
    atm: Atmosphere,
    sched: Schedule,
    wind: [f64; 2],
    axis: [f64; 3],
    a_ref: f64,
}

struct Diag {
    acc: [f64; 3],
    thrust: f64,
    drag: f64,
    mass: f64,
    cg: f64,
    mach: f64,
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

impl Ctx<'_> {
    /// Mass, CG station, thrust and whether a motor is burning at time `t` with `nsep` stages dropped.
    fn mass_props(&self, t: f64, nsep: usize) -> (f64, f64, f64, bool) {
        let (mut m, mut mx, mut thrust, mut burning) = (0.0, 0.0, 0.0, false);
        for (k, st) in self.input.vehicle.stages.iter().enumerate().skip(nsep) {
            let dm = st.dry.mass * self.tw.mass;
            m += dm;
            mx += dm * st.dry.cg;
            if let Some(mo) = &st.motor {
                let tau = t - self.sched.ignition[k];
                let mm = mo.curve.mass(tau);
                m += mm;
                mx += mm * mo.cg_station();
                let th = mo.curve.thrust(tau) * self.tw.thrust;
                if th > 0.0 {
                    burning = true;
                    thrust += th;
                }
            }
        }
        if m < 1e-3 {
            return (1e-3, 0.0, thrust, burning);
        }
        (m, mx / m, thrust, burning)
    }

    /// Equations of motion at `(t, pos, vel)`.
    fn eval(&self, t: f64, pos: [f64; 3], vel: [f64; 3], gust: [f64; 3], nsep: usize, rail: bool, chute: f64) -> Diag {
        let (mass, cg, thrust, burning) = self.mass_props(t, nsep);
        let air = self.atm.at_agl(pos[2].max(0.0));
        let rel = [vel[0] - self.wind[0] - gust[0], vel[1] - self.wind[1] - gust[1], vel[2] - gust[2]];
        let s = dot(rel, rel).sqrt();
        let mach = s / air.speed_of_sound;
        let dir = if rail || s < 1.0 { self.axis } else { [rel[0] / s, rel[1] / s, rel[2] / s] };
        let q = 0.5 * air.density * s * s;
        let drag = if chute > 0.0 {
            q * chute
        } else {
            let re = s * air.density / air.viscosity;
            q * self.aeros[nsep.min(self.aeros.len() - 1)].drag(mach, re, burning).total * self.tw.cd * self.a_ref
        };
        let g = G0 * (6_371_000.0 / (6_371_000.0 + pos[2].max(0.0) + self.launch.site_altitude)).powi(2);
        let rd = if s > 1e-9 { [rel[0] / s, rel[1] / s, rel[2] / s] } else { [0.0; 3] };
        let mut acc = [0.0; 3];
        for i in 0..3 {
            acc[i] = (thrust * dir[i] - drag * rd[i]) / mass;
        }
        acc[2] -= g;
        if rail {
            let mut au = dot(acc, self.axis);
            if dot(vel, self.axis) <= 0.0 && au < 0.0 {
                au = 0.0;
            }
            acc = [au * self.axis[0], au * self.axis[1], au * self.axis[2]];
        }
        Diag { acc, thrust, drag, mass, cg, mach }
    }
}

/// Flies `input` and returns the trajectory and summary.
pub fn simulate(input: &SimInput) -> FlightResult {
    let aeros = build_aeros(&input.vehicle);
    run(input, &input.launch, Tweaks::NOMINAL, &aeros)
}

pub(crate) fn run(input: &SimInput, launch: &Launch, tw: Tweaks, aeros: &[Aero]) -> FlightResult {
    let v = &input.vehicle;
    let n = v.stages.len();
    let sched = schedule(v);
    let atm = Atmosphere::new(launch.site_altitude, launch.ground_temperature);
    let th = launch.angle_from_vertical;
    let axis = [th.sin() * launch.direction.sin(), th.sin() * launch.direction.cos(), th.cos()];
    let wind = [-launch.wind_speed * launch.wind_from.sin(), -launch.wind_speed * launch.wind_from.cos()];
    let ctx = Ctx { input, launch, tw, aeros, atm, sched, wind, axis, a_ref: v.reference_area().max(1e-12) };

    // Scheduled events, sorted by time.
    let mut sched_events: Vec<Event> = Vec::new();
    for k in 0..n {
        if v.stages[k].motor.is_some() {
            sched_events.push(Event { t: ctx.sched.ignition[k], kind: EventKind::Ignition, altitude: 0.0, stage: Some(k) });
            sched_events.push(Event { t: ctx.sched.burnout[k], kind: EventKind::Burnout, altitude: 0.0, stage: Some(k) });
        }
        if ctx.sched.separation[k].is_finite() {
            sched_events.push(Event { t: ctx.sched.separation[k], kind: EventKind::Separation, altitude: 0.0, stage: Some(k) });
        }
    }
    sched_events.sort_by(|a, b| a.t.total_cmp(&b.t));
    let mut pending_events = sched_events.into_iter().peekable();
    let mut step_times: Vec<f64> = ctx.sched.ignition.iter().chain(&ctx.sched.burnout).copied().filter(|t| t.is_finite()).collect();
    step_times.extend(ctx.sched.separation.iter().copied().filter(|t| t.is_finite()));
    step_times.extend(ctx.sched.ejection.iter().flatten().copied());

    let mut deploy_at: Vec<Option<f64>> = input
        .recovery
        .iter()
        .map(|d| match d.deploy {
            Deploy::Ejection => ctx.sched.ejection.get(d.stage).copied().flatten().map(|t| t + d.delay),
            _ => None,
        })
        .collect();
    step_times.extend(deploy_at.iter().flatten().copied());
    let mut deployed = vec![false; input.recovery.len()];
    let last_burnout = ctx.sched.burnout.iter().copied().fold(0.0, f64::max);

    let mut rng = Rng::new(launch.seed ^ 0xA5A5_5A5A_1234_5678);
    let gust_sigma = launch.turbulence.intensity() * launch.wind_speed;
    let tau_gust = 1.5;
    let mut gust = [0.0; 3];

    let mut raws: Vec<Raw> = Vec::new();
    let mut events: Vec<Event> = Vec::new();
    let mut event_idx: Vec<usize> = Vec::new();
    let (mut t, mut pos, mut vel) = (0.0_f64, [0.0; 3], [0.0; 3]);
    let mut on_rail = true;
    let mut apogee: Option<(f64, f64)> = None;
    let mut rail_exit_velocity = 0.0;
    let mut stability_off_rail = 0.0;
    let mut rates: Vec<(String, f64)> = Vec::new();
    let mut label: Option<String> = None;
    let mut chute_area = 0.0;
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
        let nsep = (0..n.saturating_sub(1)).filter(|&k| ctx.sched.separation[k] <= t + 1e-9).count();
        while let Some(e) = pending_events.peek() {
            if e.t > t + 1e-9 {
                break;
            }
            let mut e = pending_events.next().expect("peeked");
            e.t = t;
            e.altitude = pos[2];
            last_event_t = t;
            push_event(&mut events, &mut event_idx, &raws, e);
        }
        // Devices whose stage has been dropped never deploy.
        for (i, d) in input.recovery.iter().enumerate() {
            if d.stage < nsep {
                deployed[i] = true;
            }
        }
        let d0 = ctx.eval(t, pos, vel, gust, nsep, on_rail, chute_area);
        let raw = Raw { t, pos, vel, acc: d0.acc, thrust: d0.thrust, drag: d0.drag, mass: d0.mass, cg: d0.cg, mach: d0.mach, nsep };
        raws.push(raw);

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
        } else if chute_area > 0.0 {
            input.dt.max(0.02)
        } else {
            input.dt.max(0.01)
        };
        let mut boundary = f64::INFINITY;
        for &bt in step_times.iter().chain(deploy_at.iter().flatten()) {
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

        // RK4.
        let add = |p: [f64; 3], k: [f64; 3], h: f64| [p[0] + h * k[0], p[1] + h * k[1], p[2] + h * k[2]];
        let (k1v, k1p) = (d0.acc, vel);
        let d2 = ctx.eval(t + 0.5 * dt, add(pos, k1p, 0.5 * dt), add(vel, k1v, 0.5 * dt), gust, nsep, on_rail, chute_area);
        let (k2v, k2p) = (d2.acc, add(vel, k1v, 0.5 * dt));
        let d3 = ctx.eval(t + 0.5 * dt, add(pos, k2p, 0.5 * dt), add(vel, k2v, 0.5 * dt), gust, nsep, on_rail, chute_area);
        let (k3v, k3p) = (d3.acc, add(vel, k2v, 0.5 * dt));
        let d4 = ctx.eval(t + dt, add(pos, k3p, dt), add(vel, k3v, dt), gust, nsep, on_rail, chute_area);
        let (k4v, k4p) = (d4.acc, add(vel, k3v, dt));
        let mut new_pos = pos;
        let mut new_vel = vel;
        for i in 0..3 {
            new_pos[i] += dt / 6.0 * (k1p[i] + 2.0 * k2p[i] + 2.0 * k3p[i] + k4p[i]);
            new_vel[i] += dt / 6.0 * (k1v[i] + 2.0 * k2v[i] + 2.0 * k3v[i] + k4v[i]);
        }
        let (t0, pos0, vel0) = (t, pos, vel);
        t += dt;
        pos = new_pos;
        vel = new_vel;

        // Rail exit.
        if on_rail {
            if dot(pos, axis) >= launch.rail_length {
                on_rail = false;
                rail_exit_velocity = dot(vel, vel).sqrt();
                let air = atm.at_agl(pos[2]);
                let rel = [vel[0] - wind[0], vel[1] - wind[1], vel[2]];
                let mach = dot(rel, rel).sqrt() / air.speed_of_sound;
                let (_, cg, _, _) = ctx.mass_props(t, nsep);
                stability_off_rail = aeros[nsep.min(aeros.len() - 1)].margin_cal(mach, 0.0, cg);
                last_event_t = t;
                push_event(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::RailExit, altitude: pos[2], stage: None });
            }
            continue;
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
                    chute_area += d.cd_area;
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
            let d = ctx.eval(t, pos, vel, gust, nsep, false, chute_area);
            final_raw = Some(Raw { t, pos, vel, acc: d.acc, thrust: 0.0, drag: d.drag, mass: d.mass, cg: d.cg, mach: d.mach, nsep });
            push_event(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::Landing, altitude: 0.0, stage: None });
            break;
        }
    }
    if let Some(r) = final_raw {
        raws.push(r);
    } else if !landed {
        // Hit the time limit mid-flight: record the final state.
        let nsep = (0..n.saturating_sub(1)).filter(|&k| ctx.sched.separation[k] <= t + 1e-9).count();
        let d = ctx.eval(t, pos, vel, gust, nsep, on_rail, chute_area);
        raws.push(Raw { t, pos, vel, acc: d.acc, thrust: d.thrust, drag: d.drag, mass: d.mass, cg: d.cg, mach: d.mach, nsep });
        ground_xy = [pos[0], pos[1]];
        ground_hit = dot(vel, vel).sqrt();
    }

    // Summary over the ascent (up to apogee, or the whole flight if there was none).
    let t_end_ascent = apogee.map_or(f64::INFINITY, |a| a.0);
    let (mut max_v, mut max_m, mut max_a) = (0.0_f64, 0.0_f64, 0.0_f64);
    for r in raws.iter().filter(|r| r.t <= t_end_ascent) {
        max_v = max_v.max(dot(r.vel, r.vel).sqrt());
        max_m = max_m.max(r.mach);
        max_a = max_a.max(dot(r.acc, r.acc).sqrt());
    }
    let final_burn = if n > 0 { ctx.sched.burnout[n - 1] } else { 0.0 };
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
            let speed = dot(r.vel, r.vel).sqrt();
            Sample {
                t: r.t,
                position: r.pos,
                velocity: r.vel,
                altitude: r.pos[2],
                vertical_velocity: r.vel[2],
                speed,
                mach: r.mach,
                acceleration: dot(r.acc, r.acc).sqrt(),
                vertical_acceleration: r.acc[2],
                thrust: r.thrust,
                drag: r.drag,
                mass: r.mass,
                stability_margin: aeros[r.nsep.min(aeros.len() - 1)].margin_cal(r.mach, 0.0, r.cg),
            }
        })
        .collect();
    FlightResult { trajectory: Trajectory { samples, events }, summary }
}
