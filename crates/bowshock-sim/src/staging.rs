//! The staging timeline: when each stage lights, burns out and separates, and when its ejection charge fires, worked out from the
//! [`Timing`](crate::vehicle::Timing)s of a [`Vehicle`].
//!
//! The flight simulation and the builder's sequence list both use [`Timeline::resolve`], so they cannot disagree. A stage's events
//! only depend on stages at or below it, so one pass in firing order is the whole fixed point:
//!
//! * ignition of stage `k`: [`Trigger::Launch`] is `delay`; [`Trigger::Burnout`] is the burnout of stage `k - 1` plus `delay`;
//!   [`Trigger::Separation`] is the separation of stage `k - 1` plus `delay`; [`Trigger::Altitude`] is the time the rocket
//!   climbed through that altitude plus `delay` (known only from a flight);
//! * burnout of stage `k`: its ignition plus the burn time of its motor (a stage without a motor ignites and "burns out" when the
//!   stage below burns out);
//! * separation of stage `k` from the stage above: [`Trigger::Burnout`] (or `Separation`) is the burnout of stage `k` plus `delay`,
//!   `Launch` is `delay` after launch, `Altitude` as for ignition;
//! * ejection of stage `k`: its burnout plus the motor's ejection delay.
//!
//! Times that depend on something unknown (an altitude not yet reached) are `None`, and so is everything that depends on them.

use crate::flight::{simulate, EventKind, SimInput};
use crate::vehicle::{Trigger, Vehicle};

/// Event times of a vehicle's stages, by stage index in firing order (`None`: not known, or never).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Timeline {
    /// When each stage's motor ignites (s from launch).
    pub ignition: Vec<Option<f64>>,
    /// When each stage's motor burns out.
    pub burnout: Vec<Option<f64>>,
    /// When each stage separates from the one above it; the top stage never does.
    pub separation: Vec<Option<f64>>,
    /// When each stage's ejection charge fires.
    pub ejection: Vec<Option<f64>>,
}

/// The times at which the altitude triggers fired (before their delay), by stage index, and the ignitions a tilt lockout held off.
#[derive(Clone, Debug, Default)]
pub(crate) struct Hits {
    /// For ignitions.
    pub ignition: Vec<Option<f64>>,
    /// For separations.
    pub separation: Vec<Option<f64>>,
    /// Stages whose motor never lights: its ignition came while the rocket leaned past the stage's tilt lockout.
    pub inhibited: Vec<bool>,
}

impl Hits {
    /// Nothing has fired yet.
    pub(crate) fn none(stages: usize) -> Hits {
        Hits { ignition: vec![None; stages], separation: vec![None; stages], inhibited: vec![false; stages] }
    }
}

impl Timeline {
    /// The timeline of `vehicle` from its triggers alone: stages waiting for an altitude (and what follows from them) are `None`.
    /// Use [`staging_times`] to have a flight resolve them.
    pub fn of(vehicle: &Vehicle) -> Timeline {
        Timeline::resolve(vehicle, &Hits::none(vehicle.stages.len()))
    }

    pub(crate) fn resolve(v: &Vehicle, hits: &Hits) -> Timeline {
        let n = v.stages.len();
        let mut t = Timeline { ignition: vec![None; n], burnout: vec![None; n], separation: vec![None; n], ejection: vec![None; n] };
        for k in 0..n {
            let stage = &v.stages[k];
            let (below_burnout, below_separation) = if k == 0 { (Some(0.0), Some(0.0)) } else { (t.burnout[k - 1], t.separation[k - 1]) };
            match &stage.motor {
                Some(m) => {
                    let base = match m.ignition.trigger {
                        Trigger::Launch => Some(0.0),
                        Trigger::Burnout => below_burnout,
                        Trigger::Separation => below_separation,
                        Trigger::Altitude(_) => hits.ignition[k],
                    };
                    let inhibited = hits.inhibited.get(k).copied().unwrap_or(false);
                    t.ignition[k] = base.filter(|_| !inhibited).map(|b| b + m.ignition.delay.max(0.0));
                    t.burnout[k] = t.ignition[k].map(|i| i + m.curve.burn_time());
                    t.ejection[k] = m.ejection_delay.and_then(|d| t.burnout[k].map(|b| b + d));
                }
                None => {
                    t.ignition[k] = below_burnout;
                    t.burnout[k] = below_burnout;
                }
            }
            if k + 1 < n {
                let base = match stage.separation.trigger {
                    Trigger::Burnout | Trigger::Separation => t.burnout[k],
                    Trigger::Launch => Some(0.0),
                    Trigger::Altitude(_) => hits.separation[k],
                };
                t.separation[k] = base.map(|b| b + stage.separation.delay.max(0.0));
            }
        }
        t
    }

    /// Every known time, for the integrator's step boundaries.
    pub(crate) fn times(&self) -> impl Iterator<Item = f64> + '_ {
        self.ignition.iter().chain(&self.burnout).chain(&self.separation).chain(&self.ejection).flatten().copied()
    }

    /// How many stages (from the bottom) have separated by time `t`.
    pub(crate) fn separated(&self, t: f64) -> usize {
        self.separation.iter().take_while(|s| s.is_some_and(|s| s <= t + 1e-9)).count()
    }
}

/// Whether any stage waits for an altitude.
pub fn has_altitude_trigger(v: &Vehicle) -> bool {
    v.stages
        .iter()
        .any(|s| matches!(s.separation.trigger, Trigger::Altitude(_)) || s.motor.as_ref().is_some_and(|m| matches!(m.ignition.trigger, Trigger::Altitude(_))))
}

/// The staging timeline of `input`'s vehicle. Without altitude triggers this is [`Timeline::of`]; with them the times come from a
/// coarse flight (5 ms step, no recovery) of up to two minutes, whose events are the same ones [`simulate`] reports.
pub fn staging_times(input: &SimInput) -> Timeline {
    let v = &input.vehicle;
    if !has_altitude_trigger(v) {
        return Timeline::of(v);
    }
    let mut quick = input.clone();
    quick.dt = 0.005;
    quick.max_time = 120.0;
    quick.recovery.clear();
    let flight = simulate(&quick);
    let mut hits = Hits::none(v.stages.len());
    for e in &flight.trajectory.events {
        let Some(k) = e.stage.filter(|k| *k < v.stages.len()) else {
            continue;
        };
        match e.kind {
            EventKind::Ignition => {
                if let Some(m) = &v.stages[k].motor {
                    hits.ignition[k] = Some(e.t - m.ignition.delay.max(0.0));
                }
            }
            EventKind::Separation => hits.separation[k] = Some(e.t - v.stages[k].separation.delay.max(0.0)),
            _ => {}
        }
    }
    Timeline::resolve(v, &hits)
}
