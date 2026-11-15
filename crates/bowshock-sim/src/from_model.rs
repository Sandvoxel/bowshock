//! Bridge from the rocket document ([`bowshock_model::Rocket`]) and the motor database ([`bowshock_motors::MotorDb`]) to the
//! simulation inputs of this crate ([`Vehicle`], [`RecoveryDevice`], [`SimInput`]), plus the live numbers the builder shows
//! (static balance, [`Balance`], and the health list, [`warnings`]).
//!
//! # Mapping
//!
//! * **Body.** Every axial part becomes a [`BodySegment`] at its station, with the roughness of its
//!   [`Finish`](bowshock_model::Finish). The sim knows five nose families (conical, tangent ogive, Von Karman, elliptical and
//!   the K = 0.5 parabola); a model nose of one of the first four maps straight across, and the rest (the model's K = 1
//!   parabola, power series, Haack series with `C != 0`) map to whichever of the five has the least RMS radius error over the
//!   profile for that length and diameter.
//! * **Fins.** A trapezoidal or elliptical fin set maps one to one. A freeform fin becomes the *equivalent trapezoid*: its root
//!   is the edge between the polygon's first and last points, its tip chord follows from the polygon's area and span
//!   (`A = (c_r + c_t) span / 2`; a polygon fuller than a triangle on its root gets its root chord shrunk instead, tip chord
//!   0) and its sweep from the polygon's chordwise centroid. So area, span and centroid carry over exactly and the Barrowman
//!   fin lift and CP stay close. Launch lugs and rail buttons become one [`Protuberance`] per copy.
//! * **Stages.** Model stages ([`Rocket::stages`], stage 1 the aft-most and first to fire) are the sim's `stages` in the same
//!   order (model stage `n` is sim stage `n - 1`). A stage's dry mass is every part (axial, radial, internal) whose
//!   [`Rocket::stage_of`] is that stage, combined with [`MassProps::combine`](bowshock_model::MassProps::combine); the whole
//!   of a part counts toward its stage even when a malformed stage boundary falls inside it (the app flags that, see
//!   [`warnings`]). The motor is the one the **active configuration** puts in the stage's mount; in a cluster only the
//!   *first* (fore-most) mount that holds a motor is flown and [`warnings`] says so.
//! * **Staging.** The configuration's stage events become [`Timing`]s: a stage's separation event is [`StageMass::separation`]
//!   and the ignition event of the stage its motor sits in is [`StageMotor::ignition`] (burnout, separation, time from
//!   launch, altitude, each with its delay; events the configuration does not set take the defaults of
//!   [`StageEvent::default_of`](bowshock_model::StageEvent::default_of)). The separation and the ignition are independent
//!   instants: with the default "ignites on separation, 0 s" the upper stage lights the moment the stage below lets go. The
//!   motor's ejection delay is the slot's delay, `None` for a plugged motor.
//! * **Ballast.** The active configuration's ballast (see [`Configuration::ballast`](bowshock_model::Configuration::ballast))
//!   is dry mass of the stage whose span holds its station, so [`balance`], the flight and the stability margin carry it.
//! * **Recovery.** A parachute or streamer deploys as the active configuration says
//!   ([`Rocket::active_deployment`]), which is the part's own value unless the configuration overrides it.
//! * **Phases.** [`Phase::UpperFrom`] flies the upper stack alone: lower stages, and the geometry aft of their joint, are
//!   dropped (with [`Vehicle::truncated`]), the first remaining motor ignites at launch, and recovery devices are re-indexed.
//!
//! References: Barrowman (1967); Niskanen, *OpenRocket technical documentation* (2013), sections 3.1 and 3.2.

use std::f64::consts::PI;
use std::fmt;

use bowshock_model as model;
use bowshock_model::{DeployEvent, EventKind, InternalKind, MotorMount, MotorRef, PartId, RadialKind, Rocket, Stage, StageIssue, StageTrigger};
use bowshock_motors::{Motor, MotorDb};

use crate::aero::Aero;
use crate::flight::{Deploy, Launch, RecoveryDevice, SimInput};
use crate::vehicle::{
    BodyKind, BodySegment, FinCrossSection, FinPlanform, FinSet, MassProps, NoseShape, Protuberance, StageMass, StageMotor, ThrustCurve, Timing, Trigger,
    Vehicle,
};
use crate::G0;

/// Stability margin (calibers) below which the rocket is flagged.
pub const MIN_MARGIN_CAL: f64 = 1.0;
/// Stability margin (calibers) above which the rocket is flagged as overstable.
pub const MAX_MARGIN_CAL: f64 = 6.0;
/// Thrust-to-weight ratio at liftoff below which the rocket is flagged (the NAR safety-code rule of thumb is 5:1).
pub const MIN_THRUST_TO_WEIGHT: f64 = 5.0;
/// How much larger than a mount's bore a motor may be before it is flagged (m).
const FIT_TOL: f64 = 0.5e-3;
/// Launch lug wall: a tenth of its diameter, at most a millimetre (the same wall the model's mass uses).
const LUG_WALL_MAX: f64 = 0.001;

/// Which part of the stack flies.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Phase {
    /// The whole rocket from the pad.
    #[default]
    Full,
    /// Only the stack from model stage `k` upward (`Stage::index`, from 1) flies, as if lit on the pad. `UpperFrom(1)` (and
    /// `UpperFrom(0)`) is the same as [`Phase::Full`].
    UpperFrom(usize),
}

impl Phase {
    /// The first model stage index (from 1) that flies.
    pub fn first_stage(self) -> usize {
        match self {
            Phase::Full => 1,
            Phase::UpperFrom(k) => k.max(1),
        }
    }
}

/// Why the model could not be turned into a simulation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BridgeError {
    /// The rocket has no airframe parts.
    EmptyRocket,
    /// The phase names a stage the rocket does not have (its index from 1).
    NoSuchStage(usize),
    /// The active configuration names a motor that is not in the database (its designation).
    UnknownMotor(String),
}

impl fmt::Display for BridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BridgeError::EmptyRocket => f.write_str("the rocket has no parts"),
            BridgeError::NoSuchStage(n) => write!(f, "the rocket has no stage {n}"),
            BridgeError::UnknownMotor(d) => write!(f, "motor {d} is not in the motor database"),
        }
    }
}

impl std::error::Error for BridgeError {}

// ---- vehicle ------------------------------------------------------------------------------------------------------------

/// Maps a model nose shape to the nearest of the sim's five (see the module docs).
fn nose_shape(shape: model::NoseShape, length: f64, diameter: f64) -> NoseShape {
    match shape {
        model::NoseShape::Conical => return NoseShape::Conical,
        model::NoseShape::TangentOgive => return NoseShape::TangentOgive,
        model::NoseShape::VonKarman => return NoseShape::VonKarman,
        model::NoseShape::Elliptical => return NoseShape::Elliptical,
        _ => {}
    }
    if length <= 0.0 || diameter <= 0.0 {
        return NoseShape::Conical;
    }
    const SAMPLES: usize = 32;
    let model_nose = model::PartKind::NoseCone { shape, length, diameter };
    let error = |candidate: NoseShape| -> f64 {
        let seg = BodySegment { station: 0.0, length, fore_radius: 0.0, aft_radius: diameter / 2.0, kind: BodyKind::Nose(candidate), roughness: 0.0 };
        (1..SAMPLES)
            .map(|i| {
                let x = length * i as f64 / SAMPLES as f64;
                (seg.radius_at(x) - model_nose.radius_at(x)).powi(2)
            })
            .sum::<f64>()
    };
    [NoseShape::Conical, NoseShape::TangentOgive, NoseShape::VonKarman, NoseShape::Elliptical, NoseShape::Parabolic]
        .into_iter()
        .min_by(|a, b| error(*a).total_cmp(&error(*b)))
        .unwrap_or(NoseShape::Conical)
}

fn body_segment(part: &model::Part, station: f64) -> BodySegment {
    let kind = match part.kind {
        model::PartKind::NoseCone { shape, length, diameter } => BodyKind::Nose(nose_shape(shape, length, diameter)),
        model::PartKind::BodyTube { .. } => BodyKind::Tube,
        model::PartKind::Transition { .. } => BodyKind::Transition,
    };
    BodySegment {
        station,
        length: part.kind.length(),
        fore_radius: part.kind.fore_diameter() / 2.0,
        aft_radius: part.kind.aft_diameter() / 2.0,
        kind,
        roughness: part.finish.roughness(),
    }
}

fn cross_section(section: model::FinSection) -> FinCrossSection {
    match section {
        model::FinSection::Square => FinCrossSection::Square,
        model::FinSection::Rounded => FinCrossSection::Rounded,
        model::FinSection::Airfoil => FinCrossSection::Airfoil,
    }
}

/// The trapezoid with the same area, span and chordwise centroid as a freeform fin, as
/// `(root leading edge offset, root chord, tip chord, span, sweep)` with the offset along the group's `z` axis. `None` for a
/// degenerate polygon (no area, no span or no root).
fn equivalent_trapezoid(kind: &RadialKind) -> Option<(f64, f64, f64, f64, f64)> {
    let RadialKind::FreeformFin { points, .. } = kind else {
        return None;
    };
    let (first, last) = (points.first()?, points.last()?);
    let (area, span, (centroid_z, _)) = (kind.fin_area()?, kind.fin_span()?, kind.fin_centroid()?);
    let root_le = first.0.min(last.0);
    let mut root = (last.0 - first.0).abs();
    if area <= 0.0 || span <= 0.0 || root <= 0.0 {
        return None;
    }
    let mut tip = 2.0 * area / span - root;
    if tip < 0.0 {
        tip = 0.0;
        root = 2.0 * area / span;
    }
    // A trapezoid with its leading edge square to the root (sweep 0): centroid by splitting into a `tip` x `span` rectangle and
    // the triangle (signed) that makes up the rest of the root. Sweeping by `s` shears it, moving the centroid by
    // `s * (centroid height / span)`.
    let z0 = (tip * span * tip / 2.0 + (root - tip) * span / 2.0 * (2.0 * tip + root) / 3.0) / area;
    let centroid_h = span * (root + 2.0 * tip) / (3.0 * (root + tip));
    let sweep = (centroid_z - root_le - z0) * span / centroid_h;
    Some((root_le, root, tip, span, sweep))
}

/// Adds radial group `r` as fin sets and protuberances. Copies with the same effective values (the group's with the copy's own
/// overrides on top) fly as one set, so a group with a canted fin B becomes a set of two and a set of one.
fn add_radial(rocket: &Rocket, r: &model::RadialPart, fins: &mut Vec<FinSet>, prots: &mut Vec<Protuberance>) {
    let (Some(station), Some(base)) = (rocket.radial_station(r), rocket.radial_base_radius(r)) else {
        return;
    };
    for (params, copies) in copy_sets(r) {
        add_copies(r, &params, copies.len() as u32, station, base, fins, prots);
    }
}

/// The copies of group `r` that fly as one set: those with the same effective values, in order of first appearance.
fn copy_sets(r: &model::RadialPart) -> Vec<(model::InstanceParams, Vec<usize>)> {
    let mut sets: Vec<(model::InstanceParams, Vec<usize>)> = Vec::new();
    for index in 0..usize::from(r.count.max(1)) {
        let params = r.effective(index);
        match sets.iter_mut().find(|(known, _)| *known == params) {
            Some((_, copies)) => copies.push(index),
            None => sets.push((params, vec![index])),
        }
    }
    sets
}

/// Which copies of which fin group each entry of [`vehicle`]'s `fin_sets` is, in the same order: `(group id, copy indices)`.
/// Lets a caller give a fin set its own material or report on it by part (every copy with its overrides applied).
pub fn fin_set_sources(rocket: &Rocket) -> Vec<(model::PartId, Vec<usize>)> {
    let mut out = Vec::new();
    for r in rocket.radials() {
        if rocket.radial_station(r).is_none() || rocket.radial_base_radius(r).is_none() {
            continue;
        }
        for (params, copies) in copy_sets(r) {
            let flies = match &params.kind {
                RadialKind::FinSet { .. } | RadialKind::EllipticalFin { .. } => true,
                RadialKind::FreeformFin { .. } => equivalent_trapezoid(&params.kind).is_some(),
                _ => false,
            };
            if flies {
                out.push((r.id, copies));
            }
        }
    }
    out
}

/// `count` copies of the radial part `r` with the values `params`, standing at `station` on a surface of radius `base`.
fn add_copies(
    r: &model::RadialPart,
    params: &model::InstanceParams,
    count: u32,
    station: f64,
    base: f64,
    fins: &mut Vec<FinSet>,
    prots: &mut Vec<Protuberance>,
) {
    let fin = |station: f64, root_chord: f64, tip_chord: f64, span: f64, sweep: f64, thickness: f64, planform: FinPlanform| FinSet {
        count,
        station,
        body_radius: base,
        root_chord,
        tip_chord,
        span,
        sweep,
        thickness,
        cross_section: cross_section(r.cross_section),
        cant: params.cant,
        roughness: r.finish.roughness(),
        planform,
    };
    match &params.kind {
        RadialKind::FinSet { root_chord, tip_chord, span, sweep, thickness } => {
            fins.push(fin(station, *root_chord, *tip_chord, *span, *sweep, *thickness, FinPlanform::Trapezoidal));
        }
        RadialKind::EllipticalFin { root_chord, span, thickness } => {
            fins.push(fin(station, *root_chord, 0.0, *span, 0.0, *thickness, FinPlanform::Elliptical));
        }
        RadialKind::FreeformFin { thickness, .. } => {
            if let Some((le, root, tip, span, sweep)) = equivalent_trapezoid(&params.kind) {
                fins.push(fin(station + le, root, tip, span, sweep, *thickness, FinPlanform::Trapezoidal));
            }
        }
        RadialKind::LaunchLug { length, diameter } => {
            let wall = (diameter / 10.0).min(LUG_WALL_MAX);
            let lug = Protuberance::LaunchLug { station, length: *length, outer_diameter: *diameter, inner_diameter: (diameter - 2.0 * wall).max(0.0) };
            prots.extend(std::iter::repeat_n(lug, count as usize));
        }
        RadialKind::RailButton { height, diameter } => {
            let button = Protuberance::RailButton { station: station + diameter / 2.0, height: *height, diameter: *diameter };
            prots.extend(std::iter::repeat_n(button, count as usize));
        }
    }
}

fn find_motor<'a>(motors: &'a MotorDb, reference: &MotorRef) -> Option<&'a Motor> {
    motors
        .motors
        .iter()
        .find(|m| m.designation.eq_ignore_ascii_case(reference.designation.trim()) && m.manufacturer.eq_ignore_ascii_case(reference.manufacturer.trim()))
        .or_else(|| motors.find(&reference.designation))
}

fn thrust_curve(m: &Motor) -> ThrustCurve {
    let mut points = m.points.clone();
    // RASP tables start at the first thrust sample; the thrust is zero before it.
    if points.first().is_some_and(|p| p.0 > 0.0) {
        points.insert(0, (0.0, 0.0));
    }
    ThrustCurve { points, propellant_mass: m.propellant_mass, total_mass: m.total_mass, length: m.length, diameter: m.diameter }
}

/// The sim's [`Timing`] of the model's staging event of `kind` of `stage` in `config`. A separation that waits for a separation
/// (which the model refuses, but a hand-edited document could say) waits for the burnout instead.
fn timing(config: &model::Configuration, stage: usize, kind: EventKind) -> Timing {
    let e = config.event(stage, kind);
    let trigger = match e.trigger {
        StageTrigger::Burnout => Trigger::Burnout,
        StageTrigger::Separation if kind == EventKind::Ignition => Trigger::Separation,
        StageTrigger::Separation => Trigger::Burnout,
        StageTrigger::TimeFromLaunch => Trigger::Launch,
        StageTrigger::Altitude(h) => Trigger::Altitude(h),
    };
    Timing::new(trigger, e.delay.max(0.0))
}

/// The motor the active configuration puts in `stage`: the first mount (fore to aft) that holds one.
fn stage_motor(
    config: &model::Configuration,
    motors: &MotorDb,
    stage: &Stage,
    mounts: &[MotorMount],
    first_flown: bool,
) -> Result<Option<StageMotor>, BridgeError> {
    for id in &stage.mounts {
        let (Some(slot), Some(mount)) = (config.slot(*id), mounts.iter().find(|m| m.id == *id)) else {
            continue;
        };
        let Some(reference) = &slot.motor else {
            continue;
        };
        let motor = find_motor(motors, reference).ok_or_else(|| BridgeError::UnknownMotor(reference.designation.clone()))?;
        // The stack that flies from the pad lights its first motor at launch, whatever the stage's event says.
        let ignition = if first_flown && stage.index > 1 { Timing::LAUNCH } else { timing(config, stage.index, EventKind::Ignition) };
        return Ok(Some(StageMotor {
            curve: thrust_curve(motor),
            mount_aft_station: mount.motor_aft_station,
            ignition,
            ejection_delay: slot.delay.is_finite().then_some(slot.delay.max(0.0)),
        }));
    }
    Ok(None)
}

fn sim_mass(m: model::MassProps) -> MassProps {
    MassProps { mass: m.mass, cg: m.cg, i_long: m.i_long }
}

/// Builds the simulation [`Vehicle`] of `rocket` flying `phase` with the active configuration's motors.
///
/// Errors: [`BridgeError::EmptyRocket`] without parts, [`BridgeError::NoSuchStage`] when the phase names a missing stage,
/// [`BridgeError::UnknownMotor`] when a configured motor is not in `motors`.
pub fn vehicle(rocket: &Rocket, motors: &MotorDb, phase: Phase) -> Result<Vehicle, BridgeError> {
    if rocket.parts().is_empty() {
        return Err(BridgeError::EmptyRocket);
    }
    let stages = rocket.stages();
    let first = phase.first_stage();
    if first > stages.len() {
        return Err(BridgeError::NoSuchStage(first));
    }

    let stations = rocket.stations();
    let body: Vec<BodySegment> = rocket.parts().iter().zip(&stations).map(|(p, &z)| body_segment(p, z)).collect();
    let (mut fin_sets, mut protuberances) = (Vec::new(), Vec::new());
    for r in rocket.radials() {
        add_radial(rocket, r, &mut fin_sets, &mut protuberances);
    }

    let (config, mounts) = (rocket.active_configuration(), rocket.motor_mounts());
    let mut owned: Vec<(usize, model::MassProps)> = rocket.component_masses().into_iter().filter_map(|(id, m)| Some((rocket.stage_of(id)?, m))).collect();
    if let Some(b) = config.ballast {
        // The stage whose span holds the ballast's station (the nearest end otherwise).
        let holder = stages.iter().find(|s| b.station >= s.fore_station && b.station <= s.aft_station).or_else(|| stages.last());
        owned.extend(holder.map(|s| (s.index, model::MassProps::point(b.mass, b.station))));
    }
    let mut sim_stages = Vec::new();
    for stage in stages.iter().filter(|s| s.index >= first) {
        let mut dry = sim_mass(model::MassProps::combine(owned.iter().filter(|(s, _)| *s == stage.index).map(|(_, m)| *m)));
        if dry.mass <= 0.0 {
            dry.cg = 0.5 * (stage.fore_station + stage.aft_station);
        }
        sim_stages.push(StageMass {
            dry,
            fore_station: stage.fore_station,
            aft_station: stage.aft_station,
            motor: stage_motor(config, motors, stage, &mounts, stage.index == first)?,
            separation: timing(config, stage.index, EventKind::Separation),
        });
    }

    let mut v = Vehicle { reference_diameter: 0.0, body, fin_sets, protuberances, stages: sim_stages };
    if first > 1 {
        v = v.truncated(stages[first - 1].aft_station);
    }
    v.reference_diameter = 2.0 * v.body.iter().map(|b| b.fore_radius.max(b.aft_radius)).fold(0.0, f64::max);
    Ok(v)
}

// ---- recovery -----------------------------------------------------------------------------------------------------------

/// The recovery devices (parachutes and streamers) that fly in `phase`, with stage indices as in [`vehicle`] for the same
/// phase (sim stage 0 is the first stage that flies), deploying as the active configuration says. A parachute's `cd_area` is
/// `cd * pi d^2 / 4`, a streamer's `cd * length * width`; the name is the part's display name. Empty for a phase the rocket
/// does not have.
pub fn recovery(rocket: &Rocket, phase: Phase) -> Vec<RecoveryDevice> {
    let first = phase.first_stage();
    rocket
        .internals()
        .iter()
        .filter_map(|p| {
            let cd_area = match p.kind {
                InternalKind::Parachute { diameter, cd, .. } => cd * PI * diameter * diameter / 4.0,
                InternalKind::Streamer { length, width, cd, .. } => cd * length * width,
                _ => return None,
            };
            let deployment = rocket.active_deployment(p.id)?;
            let stage = rocket.stage_of(p.id)?;
            let deploy = match deployment.event {
                DeployEvent::Apogee => Deploy::Apogee,
                DeployEvent::Altitude(h) => Deploy::Altitude(h),
                DeployEvent::Ejection => Deploy::Ejection,
                DeployEvent::TimeFromLaunch => Deploy::Launch,
            };
            (stage >= first).then(|| RecoveryDevice { name: p.display_name(), stage: stage - first, cd_area, deploy, delay: deployment.delay.max(0.0) })
        })
        .collect()
}

/// The whole simulation input for `phase`: [`vehicle`], [`recovery`] and `launch`, with the default step and time limit.
pub fn sim_input(rocket: &Rocket, motors: &MotorDb, phase: Phase, launch: Launch) -> Result<SimInput, BridgeError> {
    Ok(SimInput::new(vehicle(rocket, motors, phase)?, launch, recovery(rocket, phase)))
}

// ---- balance ------------------------------------------------------------------------------------------------------------

/// Static mass, CG and CP of the vehicle: what the status strip shows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Balance {
    /// Total mass, kg (dry parts plus motors, see [`balance`]).
    pub mass: f64,
    /// Length of the flown stack, m.
    pub length: f64,
    /// Reference diameter (largest body diameter), m.
    pub reference_diameter: f64,
    /// Centre of gravity, station from the nose tip, m.
    pub cg: f64,
    /// Centre of pressure, station from the nose tip, m.
    pub cp: f64,
    /// Static margin `(cp - cg) / reference diameter`, calibers (positive is stable).
    pub margin_cal: f64,
    /// Normal-force slope `CN_alpha` (per radian, on the reference area).
    pub cn_alpha: f64,
    /// Mass of the motors included in `mass` and `cg`, kg.
    pub motor_mass: f64,
}

/// Balance of the vehicle that flies `phase`: the dry mass of its stages combined with their motors, and the Barrowman CP at
/// `mach` (the builder uses 0.3) and angle of attack `alpha` (rad).
///
/// With `with_motor` the motors are the loaded ones (liftoff); without it they are the empty casings
/// (`total_mass - propellant_mass`, burnout). Every flown stage's motor counts, each at the middle of its length behind its
/// mount's aft end.
pub fn balance(rocket: &Rocket, motors: &MotorDb, phase: Phase, mach: f64, alpha: f64, with_motor: bool) -> Result<Balance, BridgeError> {
    let v = vehicle(rocket, motors, phase)?;
    let mut items: Vec<(f64, f64)> = v.stages.iter().map(|s| (s.dry.mass, s.dry.cg)).collect();
    let motor_items: Vec<(f64, f64)> = v
        .stages
        .iter()
        .filter_map(|s| s.motor.as_ref())
        .map(|m| (if with_motor { m.curve.total_mass } else { m.curve.total_mass - m.curve.propellant_mass }, m.cg_station()))
        .collect();
    items.extend(&motor_items);
    let mass: f64 = items.iter().map(|i| i.0).sum();
    let cg = if mass > 0.0 { items.iter().map(|i| i.0 * i.1).sum::<f64>() / mass } else { 0.0 };
    let stability = Aero::new(&v).stability(mach, alpha);
    let d = v.reference_diameter;
    Ok(Balance {
        mass,
        length: v.length(),
        reference_diameter: d,
        cg,
        cp: stability.cp,
        margin_cal: if d > 0.0 { (stability.cp - cg) / d } else { 0.0 },
        cn_alpha: stability.cn_alpha,
        motor_mass: motor_items.iter().map(|i| i.0).sum(),
    })
}

// ---- warnings -----------------------------------------------------------------------------------------------------------

/// How serious a [`Warning`] is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    /// Fine (a reading in range).
    Ok,
    /// Worth a look.
    Warn,
    /// Wrong: it will not fly or will not separate.
    Err,
}

/// One line of the rocket's health list.
#[derive(Clone, Debug, PartialEq)]
pub struct Warning {
    /// How serious it is.
    pub level: Level,
    /// What is wrong and by how much, in plain engineering English ("Stability 0.74 cal, below 1.0 minimum").
    pub message: String,
    /// The part to select when the line is clicked.
    pub part: Option<PartId>,
}

impl Warning {
    fn new(level: Level, message: impl Into<String>, part: Option<PartId>) -> Warning {
        Warning { level, message: message.into(), part }
    }
}

/// `x` with `decimals` places and a typographic minus (U+2212) for negatives.
fn signed(x: f64, decimals: usize) -> String {
    format!("{x:.decimals$}").replace('-', "\u{2212}")
}

fn mm(x: f64) -> String {
    format!("{:.1} mm", x * 1000.0)
}

/// Liftoff thrust-to-weight of the whole stack: the average thrust of every motor lit at launch over the loaded weight.
fn thrust_to_weight(rocket: &Rocket, motors: &MotorDb) -> Option<f64> {
    let v = vehicle(rocket, motors, Phase::Full).ok()?;
    let mass: f64 = v.stages.iter().map(|s| s.dry.mass + s.motor.as_ref().map_or(0.0, |m| m.curve.total_mass)).sum();
    let thrust: f64 = v
        .stages
        .iter()
        .filter_map(|s| s.motor.as_ref())
        .filter(|m| m.ignition.trigger == Trigger::Launch && m.ignition.delay <= 0.0 && m.curve.burn_time() > 0.0)
        .map(|m| m.curve.total_impulse() / m.curve.burn_time())
        .sum();
    (thrust > 0.0 && mass > 0.0).then(|| thrust / (mass * G0))
}

/// The health list of the rocket's active configuration, worst first within each topic: stability (judged from `balance`,
/// normally the liftoff balance of the phase on show), missing nose, motors (none, unknown, wider or longer than the mount,
/// clusters of which only the first flies, stages left empty), staging (no joint behind the upper motor), recovery (none, or
/// waiting on an ejection charge the motor does not have) and thrust-to-weight.
///
/// Lines of level [`Level::Ok`] are readings in range, kept so the list is never empty for a sound rocket.
pub fn warnings(rocket: &Rocket, motors: &MotorDb, balance: &Balance) -> Vec<Warning> {
    let mut out = Vec::new();
    if rocket.parts().is_empty() {
        out.push(Warning::new(Level::Err, "Rocket has no parts", None));
        return out;
    }

    // Stability.
    if balance.reference_diameter > 0.0 {
        let m = balance.margin_cal;
        out.push(if m < 0.0 {
            Warning::new(Level::Err, format!("Unstable · {} cal (min {MIN_MARGIN_CAL:.1})", signed(m, 2)), None)
        } else if m < MIN_MARGIN_CAL {
            Warning::new(Level::Warn, format!("Marginal · {m:.2} cal (min {MIN_MARGIN_CAL:.1})"), None)
        } else if m > MAX_MARGIN_CAL {
            Warning::new(Level::Warn, format!("Over-stable · {m:.1} cal (max {MAX_MARGIN_CAL:.1})"), None)
        } else {
            Warning::new(Level::Ok, format!("Stable · {m:.2} cal (min {MIN_MARGIN_CAL:.1})"), None)
        });
    }
    if !matches!(rocket.parts().first().map(|p| &p.kind), Some(model::PartKind::NoseCone { .. })) {
        out.push(Warning::new(Level::Warn, "No nose cone", None));
    }

    // Motors.
    let (config, mounts, stages) = (rocket.active_configuration(), rocket.motor_mounts(), rocket.stages());
    if mounts.is_empty() {
        out.push(Warning::new(Level::Warn, "No motor mount in the airframe", None));
    }
    if config.motors.iter().all(|s| s.motor.is_none()) {
        out.push(Warning::new(Level::Warn, "No motor in the current configuration", None));
    } else {
        for stage in &stages {
            let loaded: Vec<&MotorMount> = stage
                .mounts
                .iter()
                .filter(|id| config.slot(**id).is_some_and(|s| s.motor.is_some()))
                .filter_map(|id| mounts.iter().find(|m| m.id == *id))
                .collect();
            if loaded.is_empty() {
                if let Some(&mount) = stage.mounts.first() {
                    out.push(Warning::new(Level::Warn, format!("No motor in {}", stage.name), Some(mount)));
                }
            } else if loaded.len() > 1 {
                out.push(Warning::new(
                    Level::Warn,
                    format!("{}: {} motors in a cluster, only the first is simulated", stage.name, loaded.len()),
                    Some(loaded[0].id),
                ));
            }
        }
    }
    for slot in &config.motors {
        let (Some(reference), Some(mount)) = (&slot.motor, mounts.iter().find(|m| m.id == slot.mount)) else {
            continue;
        };
        let Some(motor) = find_motor(motors, reference) else {
            out.push(Warning::new(Level::Err, format!("Motor {} is not in the motor database", reference.designation), Some(mount.id)));
            continue;
        };
        if motor.diameter > mount.inner_diameter + FIT_TOL {
            out.push(Warning::new(Level::Err, format!("Motor too wide · {} (bore {:.1})", mm(motor.diameter), mount.inner_diameter * 1000.0), Some(mount.id)));
        }
        if motor.length > mount.length + mount.overhang + FIT_TOL {
            out.push(Warning::new(
                Level::Warn,
                format!("Motor too long · {} (mount {:.1})", mm(motor.length), (mount.length + mount.overhang).max(0.0) * 1000.0),
                Some(mount.id),
            ));
        }
    }

    // Staging.
    for (_, issue) in rocket.stage_issues() {
        let StageIssue::NoJoint { tube, station } = issue;
        let name = rocket.part(tube).map_or_else(|| "tube".to_string(), |p| p.display_name());
        out.push(Warning::new(Level::Err, format!("No joint aft of the upper motor: split {name} at {}", mm(station)), Some(tube)));
    }

    // Recovery.
    let devices: Vec<&model::InternalPart> =
        rocket.internals().iter().filter(|p| matches!(p.kind, InternalKind::Parachute { .. } | InternalKind::Streamer { .. })).collect();
    if devices.is_empty() {
        out.push(Warning::new(Level::Warn, "No recovery device", None));
    }
    for p in devices {
        if rocket.active_deployment(p.id).map(|d| d.event) != Some(DeployEvent::Ejection) {
            continue;
        }
        let ejects = rocket
            .stage_of(p.id)
            .and_then(|s| stages.iter().find(|st| st.index == s))
            .is_some_and(|st| st.mounts.iter().filter_map(|id| config.slot(*id)).any(|slot| slot.motor.is_some() && !slot.is_plugged()));
        if !ejects {
            out.push(Warning::new(Level::Warn, format!("{} deploys on ejection but its stage has no ejection charge", p.display_name()), Some(p.id)));
        }
    }

    // Thrust to weight.
    if let Some(twr) = thrust_to_weight(rocket, motors) {
        if twr < MIN_THRUST_TO_WEIGHT {
            out.push(Warning::new(Level::Warn, format!("Thrust/weight · {twr:.1} (min {MIN_THRUST_TO_WEIGHT:.1})"), None));
        }
    }
    out
}
