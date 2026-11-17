//! The dropped stages. At each separation the stage that is let go (stage `k`: everything between the joint and its own aft end,
//! see [`Vehicle::dropped_stage`]) is flown on its own from the state of the rocket at that instant to the ground, after the upper
//! stack's flight is done; OpenRocket likewise makes each separated stage a branch of the simulation that starts from the state at
//! the separation. The upper stack's flight never depends on a branch.
//!
//! * **Start.** Time, position, ground-relative velocity, attitude (body to inertial), body rate and the gust state are the
//!   upper stack's at the separation, so the branch continues the rocket's own motion (no separation impulse is modelled). The
//!   gust process runs on its own random stream.
//! * **Mass.** The stage's own dry mass and inertias and its own motor, as the vehicle builder produced them for the stage
//!   ([`StageMass`](crate::vehicle::StageMass)): a motor that has burned out is its casing, one still burning at the
//!   separation goes on burning for the rest of its curve (thrust along the stage's axis, propellant falling).
//! * **Aerodynamics.** The [`Aero`] build-up of the dropped assembly alone: the body segments, fins and protuberances between the
//!   joint and the aft end, stations measured from the joint, coefficients on the whole vehicle's reference area. There is no
//!   nose: the first segment of the new vehicle has nothing ahead of it, so the build-up treats its whole forward cross-section
//!   as a forward-facing step, a flat face with the **stagnation drag** `0.85 q_stag/q * pi r^2 / A_ref` (Niskanen 2009 eqs.
//!   B.1-B.2; an **own estimate** for an open tube or coupler face, which leaks into the airframe, see `docs/aero-references.md`);
//!   the aft end keeps its **base drag** (reduced by the exhaust while the motor burns). Skin friction uses the stage's own
//!   length and wetted area. Normal force and CP are the Barrowman build-up of that geometry (body lift of the cylinders by the
//!   Galejs term, the fins' lift); CFD drag tables are for the whole stack and are not used.
//! * **Attitude and stability.** With the rigid-body model the same equations of motion fly the stage as the rest of the rocket
//!   (axial drag with its angle-of-attack multiplier, normal force at the CP about the stage's own CG, damping from the fins,
//!   the body's crossflow drag on the turning body and the jet): a stage that is stable alone weathercocks into the relative
//!   wind; one that is not, which is the usual fate of a spent booster whose CP is ahead of its CG, is turned by its own normal
//!   force, flips and **tumbles**, and the crossflow drag of the turning body brakes the rotation, all from the existing
//!   model. Flying tail-first uses the coefficients of the mirror angle `pi - alpha`, the rigid-body model's rule. With the
//!   point-mass model the stage's axis follows the relative wind and has no tumble (that model has none for any body).
//! * **Recovery.** The recovery devices of the stage (the ones [`RecoveryDevice::stage`] assigns to it) deploy on their own
//!   triggers, as they do on the upper stack, with the stage's own apogee for [`Deploy::Apogee`] and its own ejection charge for
//!   [`Deploy::Ejection`]; one that opened before the separation (a device the whole stack was hanging from) carries on from
//!   its state. Under a canopy the descent is a point mass, as everywhere. Devices of other stages are not on this body.
//! * **Step size.** The rules of the upper stack: the fine step while the motor burns and just after an event, the coarser steps
//!   otherwise, and the rigid-body step limit.
//!
//! Not modelled: the separation springs or a stage that is still pushed by the one
//! above it, and fin-set interference with the stage that left.

use super::record;
use super::run::{pointing, tilt};
use super::{
    Branch, BranchSummary, Chute, Ctx, Deploy, Diag, Event, EventKind, FlightModel, Launch, Mode, Raw, RecoveryDevice, SimInput, Slope, State, Trajectory,
    Tweaks,
};
use crate::aero::Aero;
use crate::atmosphere::Atmosphere;
use crate::attitude::{dot, Quat, Vec3};
use crate::rng::Rng;
use crate::staging::Timeline;
use crate::vehicle::Vehicle;
use crate::G0;

/// The upper stack's state at a separation: where the dropped stage starts.
pub(super) struct Handoff {
    /// The stage that separated (firing order).
    pub stage: usize,
    /// Time (s) and the rocket's state then.
    pub t: f64,
    pub pos: Vec3,
    pub vel: Vec3,
    pub att: Quat,
    pub spin: Vec3,
    pub gust: [f64; 3],
    /// The staging timeline as it stood.
    pub sched: Timeline,
    /// The canopies already open that hang from the stage's devices: `(index into the vehicle's recovery list, canopy)`.
    pub chutes: Vec<(usize, Chute)>,
    /// Which recovery devices (by index) had deployed.
    pub deployed: Vec<bool>,
}

fn push(events: &mut Vec<Event>, event_idx: &mut Vec<usize>, raws: &[Raw], e: Event) {
    event_idx.push(raws.len());
    events.push(e);
}

/// Flies the stage of `h` to the ground; `None` for a vehicle that has no such stage.
pub(super) fn fly(parent: &SimInput, launch: &Launch, tw: Tweaks, h: &Handoff) -> Option<Branch> {
    let k = h.stage;
    let vehicle: Vehicle = parent.vehicle.dropped_stage(k)?;
    let joint = parent.vehicle.stages[k + 1].aft_station;
    let devices: Vec<(usize, &RecoveryDevice)> = parent.recovery.iter().enumerate().filter(|(_, d)| d.stage == k).collect();
    let input = SimInput {
        vehicle,
        launch: launch.clone(),
        recovery: Vec::new(),
        dt: parent.dt,
        max_time: parent.max_time,
        model: parent.model,
        drag_tables: Vec::new(),
    };
    let aeros = vec![Aero::new(&input.vehicle)];

    let th = launch.angle_from_vertical;
    let axis = [th.sin() * launch.direction.sin(), th.sin() * launch.direction.cos(), th.cos()];
    let (g_surface, omega) = match launch.latitude {
        Some(lat) => (super::normal_gravity(lat), Some([0.0, super::EARTH_RATE * lat.cos(), super::EARTH_RATE * lat.sin()])),
        None => (G0, None),
    };
    let has_motor = input.vehicle.stages[0].motor.is_some();
    let ctx = Ctx {
        input: &input,
        launch,
        tw,
        aeros: &aeros,
        atm: Atmosphere::new(launch.site_altitude, launch.ground_temperature),
        sched: Timeline { ignition: vec![h.sched.ignition[k]], burnout: vec![h.sched.burnout[k]], separation: vec![None], ejection: vec![h.sched.ejection[k]] },
        wind: launch.wind_profile(),
        axis,
        a_ref: input.vehicle.reference_area().max(1e-12),
        g_surface,
        omega,
    };

    let rigid_model = input.model == FlightModel::RigidBody;
    let mode_of = |chutes: &[Chute]| if rigid_model && chutes.is_empty() { Mode::Rigid } else { Mode::Point };
    let (mut t, mut pos, mut vel, mut att, mut spin, mut gust) = (h.t, h.pos, h.vel, h.att, h.spin, h.gust);
    let mut rng = Rng::new(launch.seed ^ 0xA5A5_5A5A_1234_5678 ^ (k as u64 + 1).wrapping_mul(0x9E37_79B9_7F4A_7C15));
    let intensity = launch.turbulence.intensity();
    let tau_gust = 1.5;

    let mut raws: Vec<Raw> = Vec::new();
    let mut events: Vec<Event> = Vec::new();
    let mut event_idx: Vec<usize> = Vec::new();
    push(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::Separation, altitude: pos[2], stage: Some(k) });

    // The devices on this body: whether each has deployed, when it will, and the canopies already open.
    let mut deployed: Vec<bool> = devices.iter().map(|(i, _)| h.deployed.get(*i).copied().unwrap_or(false)).collect();
    let mut deploy_at: Vec<Option<f64>> =
        devices.iter().zip(&deployed).map(|((_, d), &done)| (!done && d.deploy == Deploy::Launch).then_some(d.delay)).collect();
    let mut step_times: Vec<f64> = Vec::new();
    let mut chutes: Vec<Chute> = h.chutes.iter().map(|(_, c)| *c).collect();
    let mut label: Option<String> = h.chutes.last().map(|(i, _)| parent.recovery[*i].name.clone());
    let mut rates: Vec<(String, f64)> = Vec::new();
    let mut apogee: Option<f64> = None;
    // A burnout that has not happened yet is reported (the upper stack's own record holds the ones before the separation).
    let mut burnout_pending = has_motor && ctx.sched.burnout[0].is_some_and(|tb| tb > t + 1e-9);

    let mut last_event_t = t;
    let mut landed = false;
    let (mut ground_hit, mut ground_xy, mut descent_rate) = (0.0, [0.0; 2], 0.0);
    let mut final_raw: Option<Raw> = None;

    while t < input.max_time {
        if burnout_pending && ctx.sched.burnout[0].is_some_and(|tb| tb <= t + 1e-9) {
            burnout_pending = false;
            last_event_t = t;
            push(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::Burnout, altitude: pos[2], stage: Some(k) });
        }
        // Ejection deployments wait for the stage's own charge.
        for (j, (_, d)) in devices.iter().enumerate() {
            if d.deploy == Deploy::Ejection && !deployed[j] && deploy_at[j].is_none() {
                deploy_at[j] = ctx.sched.ejection[0].map(|te| te + d.delay);
            }
        }
        let mode = mode_of(&chutes);
        let d0 = ctx.eval(t, &State { pos, vel, q: att, w: spin }, gust, 0, mode, &chutes);
        raws.push(Raw { t, pos, vel, nsep: 0, d: d0 });

        // Step size, by the upper stack's rules.
        let fine = d0.thrust > 0.0 || t - last_event_t < 0.05;
        let mut dt = if fine {
            input.dt
        } else if chutes.iter().any(|c| t < c.t_deploy + c.t_inflate) {
            input.dt.max(0.005)
        } else if !chutes.is_empty() {
            input.dt.max(0.02)
        } else {
            input.dt.max(0.01)
        };
        if mode == Mode::Rigid && d0.stiff > 0.0 {
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

        // Gust, held constant over the step.
        let mean = ctx.wind_at(pos[2].max(0.0));
        let gust_sigma = intensity * mean[0].hypot(mean[1]);
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
        let k2 = Slope::of(&s2, &ctx.eval(t + 0.5 * dt, &s2, gust, 0, mode, &chutes));
        let s3 = s0.advanced(&k2, 0.5 * dt);
        let k3 = Slope::of(&s3, &ctx.eval(t + 0.5 * dt, &s3, gust, 0, mode, &chutes));
        let s4 = s0.advanced(&k3, dt);
        let k4 = Slope::of(&s4, &ctx.eval(t + dt, &s4, gust, 0, mode, &chutes));
        let next = s0.advanced(&Slope::rk4([&k1, &k2, &k3, &k4]), dt);
        let (t0, pos0, vel0) = (t, pos, vel);
        t += dt;
        (pos, vel, att, spin) = (next.pos, next.vel, next.q, next.w);

        // The stage's own apogee, for the devices that wait for it.
        if apogee.is_none() && vel[2] <= 0.0 {
            let ta = if vel0[2] > 0.0 { t0 + vel0[2] / (vel0[2] - vel[2]) * dt } else { t };
            apogee = Some(ta);
            for (j, (_, d)) in devices.iter().enumerate() {
                if d.deploy == Deploy::Apogee && !deployed[j] {
                    deploy_at[j] = Some(ta + d.delay);
                    if ta + d.delay > t {
                        step_times.push(ta + d.delay);
                    }
                }
            }
        }
        if apogee.is_some() {
            for (j, (_, d)) in devices.iter().enumerate() {
                if let Deploy::Altitude(height) = d.deploy {
                    if !deployed[j] && deploy_at[j].is_none() && pos[2] <= height {
                        deploy_at[j] = Some(t + d.delay);
                        if d.delay > 1e-9 {
                            step_times.push(t + d.delay);
                        }
                    }
                }
            }
        }
        // Speed and tilt deployments.
        let air_speed = {
            let wind = ctx.wind_at(pos[2].max(0.0));
            ((vel[0] - wind[0]).powi(2) + (vel[1] - wind[1]).powi(2) + vel[2].powi(2)).sqrt()
        };
        let lean = tilt(pointing(false, axis, rigid_model && chutes.is_empty(), att, vel));
        for (j, (_, d)) in devices.iter().enumerate() {
            let met = match d.deploy {
                Deploy::SpeedBelow(limit) => apogee.is_some() && air_speed < limit,
                Deploy::TiltAbove(limit) => lean > limit,
                _ => false,
            };
            if met && !deployed[j] && deploy_at[j].is_none() {
                deploy_at[j] = Some(t + d.delay);
                if d.delay > 1e-9 {
                    step_times.push(t + d.delay);
                }
            }
        }
        for (j, (_, d)) in devices.iter().enumerate() {
            if let Some(at) = deploy_at[j] {
                if !deployed[j] && t >= at - 1e-9 {
                    deployed[j] = true;
                    let wind = ctx.wind_at(pos[2].max(0.0));
                    let rel_speed = ((vel[0] - wind[0]).powi(2) + (vel[1] - wind[1]).powi(2) + vel[2].powi(2)).sqrt();
                    chutes.push(Chute::new(d.cd_area, t, rel_speed));
                    // Under a canopy the descent is a point mass: the attitude follows the relative wind from here on.
                    spin = [0.0; 3];
                    if let Some(l) = label.take() {
                        rates.push((l, vel[2].abs()));
                    }
                    label = Some(d.name.clone());
                    last_event_t = t;
                    push(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::Deploy(d.name.clone()), altitude: pos[2], stage: None });
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
            (pos, vel) = (lp, lv);
            ground_hit = dot(lv, lv).sqrt();
            descent_rate = (-lv[2]).max(0.0);
            ground_xy = [lp[0], lp[1]];
            if let Some(l) = label.take() {
                rates.push((l, ground_hit));
            }
            landed = true;
            let d = ctx.eval(t, &State { pos, vel, q: att, w: spin }, gust, 0, mode_of(&chutes), &chutes);
            final_raw = Some(Raw { t, pos, vel, nsep: 0, d: Diag { thrust: 0.0, ..d } });
            push(&mut events, &mut event_idx, &raws, Event { t, kind: EventKind::Landing, altitude: 0.0, stage: None });
            break;
        }
    }
    if let Some(r) = final_raw {
        raws.push(r);
    } else if !landed {
        // Hit the time limit in the air: record the final state.
        let d = ctx.eval(t, &State { pos, vel, q: att, w: spin }, gust, 0, mode_of(&chutes), &chutes);
        raws.push(Raw { t, pos, vel, nsep: 0, d });
        ground_xy = [pos[0], pos[1]];
        ground_hit = dot(vel, vel).sqrt();
        descent_rate = (-vel[2]).max(0.0);
    }

    let max_velocity = raws.iter().map(|r| dot(r.vel, r.vel).sqrt()).fold(0.0, f64::max);
    let bearing = ground_xy[0].atan2(ground_xy[1]).to_degrees().rem_euclid(360.0);
    let summary = BranchSummary {
        max_velocity,
        descent_rates: rates,
        descent_rate,
        ground_hit_velocity: ground_hit,
        flight_time: raws.last().map_or(t, |r| r.t),
        landing: (ground_xy[0].hypot(ground_xy[1]), bearing),
        landing_position: ground_xy,
    };
    let mut samples = record::samples(&raws, &event_idx, &aeros, ctx.a_ref, launch.site_altitude);
    // The stage's own stations are from the joint; the samples quote them from the nose tip of the whole rocket.
    for s in &mut samples {
        s.cg += joint;
        s.cp += joint;
    }
    let name = if parent.vehicle.stages.len() == 2 { "Booster".to_string() } else { format!("Stage {}", k + 1) };
    Some(Branch { name, stage: k, start: h.t, trajectory: Trajectory { samples, events }, summary })
}
