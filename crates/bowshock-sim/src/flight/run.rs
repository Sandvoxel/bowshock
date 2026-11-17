use super::branch::{self, Handoff};
use super::record;
use super::{
    build_aeros, normal_gravity, Chute, Ctx, Deploy, Diag, Event, EventKind, FlightModel, FlightResult, Launch, Mode, Raw, SimInput, Slope, State, Summary,
    Trajectory, Tweaks, EARTH_RATE,
};
use crate::aero::Aero;
use crate::atmosphere::Atmosphere;
use crate::attitude::{dot, from_axis, rotate, Quat};
use crate::rng::Rng;
use crate::roll::roll_resonance;
use crate::staging::{Hits, Timeline};
use crate::vehicle::Trigger;
use crate::G0;

/// Flies `input` and returns the trajectory and summary of the upper stack, and the dropped stages flown on their own after
/// each separation ([`FlightResult::branches`]).
pub fn simulate(input: &SimInput) -> FlightResult {
    let aeros = build_aeros(input);
    let (mut result, handoffs) = track(input, &input.launch, Tweaks::NOMINAL, &aeros);
    result.branches = handoffs.iter().filter_map(|h| branch::fly(input, &input.launch, Tweaks::NOMINAL, h)).collect();
    result
}

/// The upper stack's flight alone, without the dropped stages (what a Monte Carlo run and the staging timeline need).
pub(crate) fn run(input: &SimInput, launch: &Launch, tw: Tweaks, aeros: &[Aero]) -> FlightResult {
    track(input, launch, tw, aeros).0
}

/// The direction the rocket points (a unit vector): the rail `axis` on the rail; with `rigid` the body axis of attitude `att`;
/// otherwise the flight path of velocity `vel` (the rail axis while it is too slow to have one).
pub(super) fn pointing(on_rail: bool, axis: [f64; 3], rigid: bool, att: Quat, vel: [f64; 3]) -> [f64; 3] {
    let speed = dot(vel, vel).sqrt();
    if on_rail {
        axis
    } else if rigid {
        rotate(att, [1.0, 0.0, 0.0])
    } else if speed > 1e-6 {
        [vel[0] / speed, vel[1] / speed, vel[2] / speed]
    } else {
        axis
    }
}

/// The angle of unit vector `dir` from vertical (rad).
pub(super) fn tilt(dir: [f64; 3]) -> f64 {
    dir[2].clamp(-1.0, 1.0).acos()
}

/// The flight of the upper stack, and the state of the rocket at every separation (the start of each dropped stage's own flight).
fn track(input: &SimInput, launch: &Launch, tw: Tweaks, aeros: &[Aero]) -> (FlightResult, Vec<Handoff>) {
    let v = &input.vehicle;
    let n = v.stages.len();
    let atm = Atmosphere::new(launch.site_altitude, launch.ground_temperature);
    let th = launch.angle_from_vertical;
    let axis = [th.sin() * launch.direction.sin(), th.sin() * launch.direction.cos(), th.cos()];
    let (g_surface, omega) = match launch.latitude {
        Some(lat) => (normal_gravity(lat), Some([0.0, EARTH_RATE * lat.cos(), EARTH_RATE * lat.sin()])),
        None => (G0, None),
    };
    let mut ctx = Ctx {
        input,
        launch,
        tw,
        aeros,
        atm,
        sched: Timeline::resolve(v, &Hits::none(n)),
        wind: launch.wind_profile(),
        axis,
        a_ref: v.reference_area().max(1e-12),
        g_surface,
        omega,
    };

    // Altitude triggers that have fired (the time they fired, before their delay), and which staging events are on record.
    let mut hits = Hits::none(n);
    let (mut done_ignition, mut done_burnout, mut done_separation) = (vec![false; n], vec![false; n], vec![false; n]);
    // Step boundaries beyond the staging timeline (deployments relative to apogee or altitude).
    let mut step_times: Vec<f64> = Vec::new();
    let mut deploy_at: Vec<Option<f64>> = input.recovery.iter().map(|d| (d.deploy == Deploy::Launch).then_some(d.delay)).collect();
    let mut deployed = vec![false; input.recovery.len()];

    let mut rng = Rng::new(launch.seed ^ 0xA5A5_5A5A_1234_5678);
    // Gusts scale with the mean wind where the rocket is (see the gust update below).
    let intensity = launch.turbulence.intensity();
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
    // The recovery device each deployed canopy of `chutes` belongs to, and the state at each separation (see `branch`).
    let mut chute_device: Vec<usize> = Vec::new();
    let mut handoffs: Vec<Handoff> = Vec::new();
    let mut last_event_t = f64::NEG_INFINITY;
    let mut landed = false;
    let mut ground_hit = 0.0;
    let mut ground_xy = [0.0; 2];
    let mut final_raw: Option<Raw> = None;

    let push_event = |events: &mut Vec<Event>, event_idx: &mut Vec<usize>, raws: &Vec<Raw>, e: Event| {
        event_idx.push(raws.len());
        events.push(e);
    };

    // An air start begins off the rail, at its altitude, already moving along the launch direction.
    if let Some((h, v0)) = launch.air_start {
        let (h, v0) = (h.max(0.0), v0.max(0.0));
        pos = [0.0, 0.0, h];
        vel = [axis[0] * v0, axis[1] * v0, axis[2] * v0];
        on_rail = false;
        rail_exit_velocity = v0;
        let air = atm.at_agl(h);
        let wind = ctx.wind_at(h);
        let rel = [vel[0] - wind[0], vel[1] - wind[1], vel[2]];
        let speed = dot(rel, rel).sqrt();
        let alpha = if rigid_model && speed > 1e-9 { (dot(axis, rel) / speed).clamp(-1.0, 1.0).acos() } else { 0.0 };
        stability_off_rail = aeros[0].margin_cal(speed / air.speed_of_sound, alpha, ctx.mass_props(0.0, 0).cg);
        push_event(&mut events, &mut event_idx, &raws, Event { t: 0.0, kind: EventKind::RailExit, altitude: h, stage: None });
    }

    while t < input.max_time {
        let nsep = ctx.sched.separated(t);
        // Where the rocket points now, for the tilt lockouts and tilt deployments: the rail, the body axis of the rigid body, or the
        // flight path of the point mass.
        let facing = pointing(on_rail, axis, rigid_model && chutes.is_empty(), att, vel);
        // Staging events that are due; a stage dropped before its burnout never reports one.
        let mut due: Vec<Event> = Vec::new();
        let mut inhibited = false;
        for k in 0..n {
            let separation = ctx.sched.separation[k];
            if let Some(motor) = &v.stages[k].motor {
                if let Some(ti) = ctx.sched.ignition[k].filter(|ti| !done_ignition[k] && *ti <= t + 1e-9) {
                    done_ignition[k] = true;
                    // A motor whose tilt lockout the rocket leans past never lights (nor does anything that waits for it).
                    if motor.tilt_lockout.is_some_and(|limit| tilt(facing) > limit) {
                        hits.inhibited[k] = true;
                        inhibited = true;
                        due.push(Event { t: ti, kind: EventKind::IgnitionInhibited, altitude: 0.0, stage: Some(k) });
                    } else {
                        due.push(Event { t: ti, kind: EventKind::Ignition, altitude: 0.0, stage: Some(k) });
                    }
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
        if inhibited {
            ctx.sched = Timeline::resolve(v, &hits);
        }
        due.sort_by(|a, b| a.t.total_cmp(&b.t));
        for mut e in due {
            e.t = t;
            e.altitude = pos[2];
            last_event_t = t;
            // The stage that separates is flown on its own from this state (see `branch`).
            if let (EventKind::Separation, Some(stage)) = (&e.kind, e.stage) {
                handoffs.push(Handoff {
                    stage,
                    t,
                    pos,
                    vel,
                    att,
                    spin,
                    gust,
                    sched: ctx.sched.clone(),
                    chutes: chutes.iter().zip(&chute_device).filter(|(_, &i)| input.recovery[i].stage == stage).map(|(c, &i)| (i, *c)).collect(),
                    deployed: deployed.clone(),
                });
            }
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

        // Gust update (held constant over the step): a first-order Gauss-Markov process whose spread is the turbulence
        // intensity times the mean wind at the rocket's altitude.
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
                let wind = ctx.wind_at(pos[2].max(0.0));
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
        // Speed and tilt deployments: descending slower than the speed, or leaning past the tilt off the rail.
        let air_speed = {
            let wind = ctx.wind_at(pos[2].max(0.0));
            ((vel[0] - wind[0]).powi(2) + (vel[1] - wind[1]).powi(2) + vel[2].powi(2)).sqrt()
        };
        let lean = tilt(pointing(on_rail, axis, rigid_model && chutes.is_empty(), att, vel));
        for (i, d) in input.recovery.iter().enumerate() {
            let met = match d.deploy {
                Deploy::SpeedBelow(limit) => apogee.is_some() && air_speed < limit,
                Deploy::TiltAbove(limit) => lean > limit,
                _ => false,
            };
            if met && !deployed[i] && deploy_at[i].is_none() {
                deploy_at[i] = Some(t + d.delay);
                if d.delay > 1e-9 {
                    step_times.push(t + d.delay);
                }
            }
        }
        for (i, d) in input.recovery.iter().enumerate() {
            if let Some(dt_at) = deploy_at[i] {
                if !deployed[i] && t >= dt_at - 1e-9 {
                    deployed[i] = true;
                    let wind = ctx.wind_at(pos[2].max(0.0));
                    let rel_speed = ((vel[0] - wind[0]).powi(2) + (vel[1] - wind[1]).powi(2) + vel[2].powi(2)).sqrt();
                    chutes.push(Chute::new(d.cd_area, t, rel_speed));
                    chute_device.push(i);
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

    // Spin: the roll rate at its peak, at the last burnout and at apogee (the last state at or before the event: a parachute
    // deployed at apogee stops the roll), and where it meets the pitch frequency.
    let roll = |r: &Raw| r.d.omega[0];
    let max_roll_rate = raws.iter().map(roll).fold(0.0, |m: f64, p| if p.abs() > m.abs() { p } else { m });
    let roll_at = |t: f64| raws.iter().rev().find(|r| r.t <= t + 1e-9).or(raws.first()).map_or(0.0, roll);
    let resonance = roll_resonance(raws.iter().map(|r| (r.t, roll(r), r.d.pitch_freq)));
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
        max_roll_rate,
        roll_rate_at_burnout: roll_at(final_burn),
        roll_rate_at_apogee: apogee.map_or(0.0, |a| roll_at(a.0)),
        roll_resonance: resonance,
    };

    // Decimate.
    let samples = record::samples(&raws, &event_idx, aeros, ctx.a_ref, launch.site_altitude);
    (FlightResult { trajectory: Trajectory { samples, events }, summary, branches: Vec::new() }, handoffs)
}
