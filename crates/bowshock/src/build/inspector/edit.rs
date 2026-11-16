//! What the inspector's fields do to the rocket. Every function here is a pure edit of a `&mut Rocket` (the update step wraps
//! them in `Document::edit` / `edit_merging`), so the behaviour is testable without a window.
//!
//! Geometry goes through `bowshock_model::Rocket::set_dim` (DirectEdit's limits and neighbour following), so the field and
//! the handle in the viewport can never disagree. The fields that are not a plain dimension (sweep angle, an auto diameter, the
//! parachute's numbers, mass overrides) are the [`Field`] variants beside [`Field::Dim`].

use bowshock_model::{DeployEvent, Deployment, Dim, EventKind, FinTab, InternalKind, MassRole, PartId, Rocket, StageEvent, StageTrigger};

/// Most a shroud-line count may be.
const MAX_LINES: f64 = 64.0;
/// Altitude a parachute is given when its trigger is switched to an altitude (500 ft, a typical main deployment).
pub const DEFAULT_DEPLOY_ALTITUDE: f64 = 152.4;
/// Altitude a staging event is given when its trigger is switched to an altitude (100 m).
pub const DEFAULT_STAGE_ALTITUDE: f64 = 100.0;
/// Smallest diameter, length or width an edited recovery or mass part may be given, metres.
const MIN_SIZE: f64 = 0.001;
/// Largest sweep angle a typed value may give, so `tan` stays finite.
const MAX_SWEEP_ANGLE: f64 = 85.0 * std::f64::consts::PI / 180.0;

/// One numeric field of the inspector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Field {
    /// A dimension of the part, edited with `Rocket::set_dim`. `Dim::Position` is the offset from the chosen anchor.
    Dim(Dim),
    /// A trapezoidal fin's sweep angle: derived from the sweep length and the span, and typing it sets the sweep.
    SweepAngle,
    /// An axial part's inner diameter: derived from the diameter and the wall, and typing it sets the wall.
    BoreDiameter,
    /// An internal part's diameter that may follow its parent (`AUTO`): a ring's outer, a bulkhead's,
    /// a coupler's, a packed bundle's or a mass component's. Typing makes it explicit.
    AutoDiameter,
    /// A parachute's canopy diameter.
    CanopyDiameter,
    /// A parachute's or streamer's drag coefficient.
    Cd,
    /// A parachute's shroud-line count.
    LineCount,
    /// A parachute's shroud-line length.
    LineLength,
    /// A streamer's length.
    StreamerLength,
    /// A streamer's width.
    StreamerWidth,
    /// A shock cord's deployed length.
    CordLength,
    /// A mass component's mass (the part's mass itself, not an override).
    MassValue,
    /// The altitude a recovery device waits for, in the active configuration.
    DeployAltitude,
    /// The air speed below which a recovery device deploys on descent, in the active configuration.
    DeploySpeed,
    /// The tilt from vertical past which a recovery device deploys, in the active configuration.
    DeployTilt,
    /// An ignition's tilt lockout of stage `n` (`Stage::index`), in the active configuration.
    TiltLockout(usize),
    /// The delay after a recovery device's trigger, in the active configuration.
    DeployDelay,
    /// The mass override.
    MassOverride,
    /// The drag coefficient override (on the rocket's reference area).
    CdOverride,
    /// The CG override, metres behind the part's fore face.
    CgOverride,
    /// A fin group's own shear modulus for the flutter estimate, in GPa (a plain number: the document keeps it in Pa).
    ShearModulus,
    /// A fin group's through-the-wall tab: how deep it reaches below the root.
    TabHeight,
    /// A fin group's tab length along the root.
    TabLength,
    /// A fin group's tab position: its fore edge from the root's leading edge.
    TabPosition,
    /// A fin group's roll control: the roll rate it holds, in °/s (a plain number: the document keeps rad/s).
    RollTarget,
    /// A fin group's roll control: when it switches on (s after launch).
    RollStart,
    /// A fin group's roll control: the proportional gain, ×1000 (a plain number: the document keeps rad per rad/s).
    RollKp,
    /// A fin group's roll control: the integral gain.
    RollKi,
    /// A fin group's roll control: the largest cant.
    RollMaxCant,
    /// A fin group's roll control: the fastest cant slew, in °/s (a plain number: the document keeps rad/s).
    RollMaxRate,
    /// The active configuration's ejection delay of a motor mount.
    EjectionDelay,
    /// The active configuration's delay after the trigger of a staging event: the separation of stage `n` or the ignition of
    /// stage `n` (`Stage::index`).
    StageDelay(usize, EventKind),
    /// The altitude of the active configuration's altitude trigger for a staging event.
    StageAltitude(usize, EventKind),
}

impl Field {
    /// The label of the undo step an edit of this field makes.
    pub fn undo_label(self) -> String {
        let what = match self {
            Field::Dim(d) => d.label().to_lowercase(),
            Field::SweepAngle => "sweep angle".into(),
            Field::BoreDiameter => "inner diameter".into(),
            Field::AutoDiameter => "diameter".into(),
            Field::CanopyDiameter => "canopy diameter".into(),
            Field::Cd => "drag coefficient".into(),
            Field::LineCount => "line count".into(),
            Field::LineLength => "line length".into(),
            Field::StreamerLength => "streamer length".into(),
            Field::StreamerWidth => "streamer width".into(),
            Field::CordLength => "cord length".into(),
            Field::MassValue | Field::MassOverride => "mass".into(),
            Field::CdOverride => "drag".into(),
            Field::DeployAltitude => "deploy altitude".into(),
            Field::DeployDelay => "deploy delay".into(),
            Field::DeploySpeed => "deploy speed".into(),
            Field::DeployTilt => "deploy tilt".into(),
            Field::TiltLockout(_) => "tilt lockout".into(),
            Field::CgOverride => "CG".into(),
            Field::ShearModulus => "shear modulus".into(),
            Field::TabHeight => "tab height".into(),
            Field::TabLength => "tab length".into(),
            Field::TabPosition => "tab position".into(),
            Field::RollTarget | Field::RollStart | Field::RollKp | Field::RollKi | Field::RollMaxCant | Field::RollMaxRate => "roll control".into(),
            Field::EjectionDelay => "ejection delay".into(),
            Field::StageDelay(_, EventKind::Separation) => "separation delay".into(),
            Field::StageDelay(_, EventKind::Ignition) => "ignition delay".into(),
            Field::StageAltitude(_, EventKind::Separation) => "separation altitude".into(),
            Field::StageAltitude(_, EventKind::Ignition) => "ignition altitude".into(),
        };
        format!("Edit {what}")
    }

    /// The widget id of the field, so a request to focus a dimension finds it. `Span` and `Height` are one field.
    pub fn key(self) -> String {
        match self {
            Field::Dim(Dim::Span | Dim::Height) => "insp:Height".into(),
            Field::Dim(d) => format!("insp:{d:?}"),
            Field::AutoDiameter => "insp:Diameter".into(),
            other => format!("insp:{other:?}"),
        }
    }
}

/// The widget id key of the field that edits `dim` on a part (see [`Field::key`]). `Diameter` is the auto diameter of an
/// internal part too, so the key is the same.
pub fn focus_key(dim: Dim) -> String {
    Field::Dim(dim).key()
}

/// What an edit did.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Applied {
    /// The rocket was edited (the field exists on this part and the value was usable).
    pub done: bool,
    /// Why the value was limited, if it was ("MINIMUM LENGTH").
    pub limited: Option<&'static str>,
}

impl Applied {
    fn yes(limited: Option<&'static str>) -> Applied {
        Applied { done: true, limited }
    }

    fn no() -> Applied {
        Applied::default()
    }

    fn flag(done: bool) -> Applied {
        Applied { done, limited: None }
    }
}

/// Applies `value` (SI) to `field` of part `id`, editing the group's own value of a radial part's field.
pub fn apply(r: &mut Rocket, id: PartId, field: Field, value: f64) -> Applied {
    apply_copy(r, id, usize::MAX, field, value)
}

/// Applies `value` (SI) to `field` of part `id` as seen from copy `copy` of a radial group: a field that copy has unlinked
/// (*Unlink for this fin*) sets the copy's own value, every other field the group's. (`usize::MAX` is no copy.)
pub fn apply_copy(r: &mut Rocket, id: PartId, copy: usize, field: Field, value: f64) -> Applied {
    if !value.is_finite() {
        return Applied::no();
    }
    match field {
        Field::Dim(dim) => match r.set_instance_dim(id, copy, dim, value) {
            Some(edit) => Applied::yes(edit.limited),
            None => Applied::no(),
        },
        Field::SweepAngle => {
            let Some(span) = r.instance_dim(id, copy, Dim::Span) else {
                return Applied::no();
            };
            let angle = value.clamp(-MAX_SWEEP_ANGLE, MAX_SWEEP_ANGLE);
            let limited = (angle != value).then_some("SWEEP ANGLE LIMIT 85°");
            match r.set_instance_dim(id, copy, Dim::Sweep, span * angle.tan()) {
                Some(edit) => Applied::yes(limited.or(edit.limited)),
                None => Applied::no(),
            }
        }
        Field::BoreDiameter => {
            let outer = match r.part(id) {
                Some(p) => Some(p.kind.fore_diameter().max(p.kind.aft_diameter())),
                None => r.dim(id, Dim::Diameter),
            };
            let Some(outer) = outer else {
                return Applied::no();
            };
            match r.set_dim(id, Dim::Wall, (outer - value) / 2.0) {
                Some(edit) => Applied::yes(edit.limited),
                None => Applied::no(),
            }
        }
        Field::AutoDiameter => Applied::flag(set_auto_diameter(r, id, Some(value))),
        Field::MassOverride => Applied::flag(r.set_mass_override(id, Some(value))),
        Field::CdOverride => Applied::flag(r.set_cd_override(id, Some(value))),
        Field::CgOverride => Applied::flag(r.set_cg_override(id, Some(value))),
        Field::ShearModulus => Applied::flag(r.set_fin_shear_modulus(id, Some(value * 1.0e9))),
        Field::TabHeight | Field::TabLength | Field::TabPosition => {
            let Some(tab) = r.radial(id).and_then(|fin| fin.tab) else {
                return Applied::no();
            };
            let v = value.max(0.0);
            let tab = match field {
                Field::TabHeight => FinTab { height: v, ..tab },
                Field::TabLength => FinTab { length: v, ..tab },
                _ => FinTab { position: v, ..tab },
            };
            Applied::flag(r.set_fin_tab(id, Some(tab)))
        }
        Field::RollTarget | Field::RollStart | Field::RollKp | Field::RollKi | Field::RollMaxCant | Field::RollMaxRate => {
            let Some(mut rc) = r.radial(id).and_then(|fin| fin.roll_control) else {
                return Applied::no();
            };
            match field {
                Field::RollTarget => rc.target_rate = value.to_radians(),
                Field::RollStart => rc.start = value,
                Field::RollKp => rc.kp = value / 1000.0,
                Field::RollKi => rc.ki = value,
                Field::RollMaxCant => rc.max_cant = value,
                _ => rc.max_rate = value.to_radians(),
            }
            Applied::flag(r.set_roll_control(id, Some(rc)))
        }
        Field::EjectionDelay => {
            let config = r.active_config_id();
            Applied::flag(r.set_motor_delay(config, id, value.max(0.0)).is_ok())
        }
        Field::StageDelay(stage, kind) => Applied::flag(edit_stage_event(r, stage, kind, |e| e.delay = value.max(0.0))),
        Field::StageAltitude(stage, kind) => Applied::flag(edit_stage_event(r, stage, kind, |e| e.trigger = StageTrigger::Altitude(value.max(0.0)))),
        Field::DeployAltitude => Applied::flag(edit_deployment(r, id, |d| d.event = DeployEvent::Altitude(value.max(0.0)))),
        Field::DeployDelay => Applied::flag(edit_deployment(r, id, |d| d.delay = value.max(0.0))),
        Field::DeploySpeed => Applied::flag(edit_deployment(r, id, |d| d.event = DeployEvent::SpeedBelow(value.max(0.0)))),
        Field::DeployTilt => Applied::flag(edit_deployment(r, id, |d| d.event = DeployEvent::TiltAbove(value.clamp(0.0, std::f64::consts::PI)))),
        Field::TiltLockout(stage) => {
            Applied::flag(edit_stage_event(r, stage, EventKind::Ignition, |e| e.tilt_lockout = Some(value.clamp(0.0, std::f64::consts::PI))))
        }
        other => Applied::flag(edit_internal(r, id, other, value)),
    }
}

/// The drag coefficient the build-up gives part `id` (summed over its copies; a pod's is everything on it) at Mach 0.3 at sea
/// level, zero angle of attack, motor off: the value an override starts from. `None` when the rocket cannot be built for the
/// sim (a motor missing from `motors`) or the part has no aerodynamics.
pub fn computed_cd(r: &Rocket, motors: &bowshock_motors::MotorDb, id: PartId) -> Option<f64> {
    use bowshock_sim::from_model::{vehicle, Phase};
    const MACH: f64 = 0.3;
    let mut ids = vec![id];
    if let Some(body) = r.radial(id).and_then(|p| p.kind.pod_body()) {
        ids.extend(body.parts().iter().map(|p| p.id).chain(body.radials().iter().map(|p| p.id)));
    }
    let v = vehicle(r, motors, Phase::Full).ok()?;
    let air = bowshock_sim::Atmosphere::standard().at_msl(0.0);
    let re = MACH * air.speed_of_sound * air.density / air.viscosity;
    let comps = bowshock_sim::Aero::new(&v).components(MACH, re, false, 0.0);
    let mine: Vec<f64> = comps.iter().filter(|c| c.part.is_some_and(|p| ids.contains(&p))).map(|c| c.drag.total).collect();
    (!mine.is_empty()).then(|| mine.iter().sum())
}

/// The recovery and mass fields: they change the internal part's kind.
fn edit_internal(r: &mut Rocket, id: PartId, field: Field, value: f64) -> bool {
    let Some(mut kind) = r.internal(id).map(|p| p.kind.clone()) else {
        return false;
    };
    let size = value.max(MIN_SIZE);
    let changed = match (&mut kind, field) {
        (InternalKind::Parachute { diameter, .. }, Field::CanopyDiameter) => {
            *diameter = size;
            true
        }
        (InternalKind::Parachute { cd, .. } | InternalKind::Streamer { cd, .. }, Field::Cd) => {
            *cd = value.clamp(0.05, 3.0);
            true
        }
        (InternalKind::Parachute { line_count, .. }, Field::LineCount) => {
            *line_count = value.round().clamp(0.0, MAX_LINES) as u32;
            true
        }
        (InternalKind::Parachute { line_length, .. }, Field::LineLength) => {
            *line_length = value.max(0.0);
            true
        }
        (InternalKind::Streamer { length, .. }, Field::StreamerLength) => {
            *length = size;
            true
        }
        (InternalKind::Streamer { width, .. }, Field::StreamerWidth) => {
            *width = size;
            true
        }
        (InternalKind::ShockCord { length, .. }, Field::CordLength) => {
            *length = size;
            true
        }
        (InternalKind::MassComponent { mass, .. }, Field::MassValue) => {
            *mass = value.max(0.0);
            true
        }
        _ => false,
    };
    changed && r.set_internal_kind(id, kind)
}

/// Sets (or with `None` resets to automatic) the diameter that may follow the parent: a ring's outer, a
/// bulkhead's, a coupler's, a parachute's packed bundle's, a mass component's. Returns whether the part has one.
pub fn set_auto_diameter(r: &mut Rocket, id: PartId, value: Option<f64>) -> bool {
    let Some(mut kind) = r.internal(id).map(|p| p.kind.clone()) else {
        return false;
    };
    let value = value.map(|v| v.max(MIN_SIZE));
    match &mut kind {
        InternalKind::CenteringRing { outer_diameter: d, .. }
        | InternalKind::Bulkhead { diameter: d, .. }
        | InternalKind::Coupler { outer_diameter: d, .. }
        | InternalKind::Parachute { packed_diameter: d, .. }
        | InternalKind::MassComponent { diameter: d, .. } => *d = value,
        _ => return false,
    }
    r.set_internal_kind(id, kind)
}

/// Makes a centering ring's hole follow the tube it sits on again.
pub fn reset_inner_diameter(r: &mut Rocket, id: PartId) -> bool {
    let Some(mut kind) = r.internal(id).map(|p| p.kind.clone()) else {
        return false;
    };
    match &mut kind {
        InternalKind::CenteringRing { inner_diameter, .. } => *inner_diameter = None,
        _ => return false,
    }
    r.set_internal_kind(id, kind)
}

/// Which trigger a recovery device waits for, without the altitude.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trigger {
    /// At apogee.
    Apogee,
    /// On descent through an altitude.
    Altitude,
    /// With the motor's ejection charge.
    Ejection,
    /// After launch (the delay is the time from launch).
    Launch,
    /// On descent once the air speed falls below a value.
    SpeedBelow,
    /// Once the rocket tilts past an angle from vertical.
    TiltAbove,
}

/// Air speed a parachute is given when its trigger is switched to a speed (20 m/s, a typical drogue-to-main hand-off).
pub const DEFAULT_DEPLOY_SPEED: f64 = 20.0;
/// Tilt a parachute is given when its trigger is switched to a tilt (30°).
pub const DEFAULT_DEPLOY_TILT: f64 = 30.0 * std::f64::consts::PI / 180.0;

impl Trigger {
    /// Every trigger, in menu order.
    pub const ALL: [Trigger; 6] = [Trigger::Ejection, Trigger::Apogee, Trigger::Altitude, Trigger::Launch, Trigger::SpeedBelow, Trigger::TiltAbove];

    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            Trigger::Apogee => "Apogee",
            Trigger::Altitude => "Altitude",
            Trigger::Ejection => "Motor ejection",
            Trigger::Launch => "Time from launch",
            Trigger::SpeedBelow => "Below speed…",
            Trigger::TiltAbove => "Tilt above…",
        }
    }

    /// The trigger of `event`.
    pub fn of(event: DeployEvent) -> Trigger {
        match event {
            DeployEvent::Apogee => Trigger::Apogee,
            DeployEvent::Altitude(_) => Trigger::Altitude,
            DeployEvent::Ejection => Trigger::Ejection,
            DeployEvent::TimeFromLaunch => Trigger::Launch,
            DeployEvent::SpeedBelow(_) => Trigger::SpeedBelow,
            DeployEvent::TiltAbove(_) => Trigger::TiltAbove,
        }
    }
}

/// Applies `change` to the deployment of recovery device `id` in the active configuration (starting from what it has now: the
/// configuration's own, else the part's) and stores the result with the configuration.
fn edit_deployment(r: &mut Rocket, id: PartId, change: impl FnOnce(&mut Deployment)) -> bool {
    let Some(mut deploy) = r.active_deployment(id) else {
        return false;
    };
    change(&mut deploy);
    let config = r.active_config_id();
    r.set_deployment(config, id, deploy).is_ok()
}

/// Applies `change` to the active configuration's staging event `kind` of `stage` (starting from what it has now, the default
/// when it says nothing) and stores the result.
fn edit_stage_event(r: &mut Rocket, stage: usize, kind: EventKind, change: impl FnOnce(&mut StageEvent)) -> bool {
    let config = r.active_config_id();
    let mut event = r.active_configuration().event(stage, kind);
    change(&mut event);
    r.set_stage_event(config, event).is_ok()
}

/// Switches a recovery device's trigger in the active configuration, keeping its delay (an altitude trigger starts at
/// [`DEFAULT_DEPLOY_ALTITUDE`], or the altitude it already had).
pub fn set_trigger(r: &mut Rocket, id: PartId, trigger: Trigger) -> bool {
    edit_deployment(r, id, |deploy| {
        deploy.event = match (trigger, deploy.event) {
            (Trigger::Apogee, _) => DeployEvent::Apogee,
            (Trigger::Ejection, _) => DeployEvent::Ejection,
            (Trigger::Launch, _) => DeployEvent::TimeFromLaunch,
            (Trigger::Altitude, DeployEvent::Altitude(h)) => DeployEvent::Altitude(h),
            (Trigger::Altitude, _) => DeployEvent::Altitude(DEFAULT_DEPLOY_ALTITUDE),
            (Trigger::SpeedBelow, DeployEvent::SpeedBelow(v)) => DeployEvent::SpeedBelow(v),
            (Trigger::SpeedBelow, _) => DeployEvent::SpeedBelow(DEFAULT_DEPLOY_SPEED),
            (Trigger::TiltAbove, DeployEvent::TiltAbove(a)) => DeployEvent::TiltAbove(a),
            (Trigger::TiltAbove, _) => DeployEvent::TiltAbove(DEFAULT_DEPLOY_TILT),
        }
    })
}

/// Gives a mass component another role; a ballast weight becomes lead and anything else aluminium unless the user picked
/// another stock (the role does not change a material that is neither of those defaults).
pub fn set_role(r: &mut Rocket, id: PartId, role: MassRole) -> bool {
    let Some(part) = r.internal(id) else {
        return false;
    };
    let mut kind = part.kind.clone();
    let old_default = kind.default_material();
    let was_default = part.material == old_default;
    match &mut kind {
        InternalKind::MassComponent { role: current, .. } => *current = role,
        _ => return false,
    }
    let new_default = kind.default_material();
    let done = r.set_internal_kind(id, kind);
    if done && was_default {
        r.set_material(id, new_default);
    }
    done
}

/// Turns an inner tube into (or out of) a motor mount.
pub fn set_motor_mount(r: &mut Rocket, id: PartId, mount: bool) -> bool {
    let Some(mut kind) = r.internal(id).map(|p| p.kind.clone()) else {
        return false;
    };
    match &mut kind {
        InternalKind::InnerTube { motor_mount, .. } => *motor_mount = mount,
        _ => return false,
    }
    r.set_internal_kind(id, kind)
}

/// What a staging event waits for, as its select offers it (the altitude is a field of its own).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StageOn {
    /// The burnout of the stage (a separation) or of the stage below (an ignition).
    Burnout,
    /// The separation of the stage below (an ignition only).
    Separation,
    /// Launch: the delay is the time from launch.
    Launch,
    /// Climbing through an altitude.
    Altitude,
}

impl StageOn {
    /// The choice that `trigger` is.
    pub fn of(trigger: StageTrigger) -> StageOn {
        match trigger {
            StageTrigger::Burnout => StageOn::Burnout,
            StageTrigger::Separation => StageOn::Separation,
            StageTrigger::TimeFromLaunch => StageOn::Launch,
            StageTrigger::Altitude(_) => StageOn::Altitude,
        }
    }
}

/// Switches what the active configuration's staging event `kind` of `stage` waits for, keeping its delay (and an altitude it
/// already had; a new one starts at [`DEFAULT_STAGE_ALTITUDE`]). A separation cannot wait for a separation.
pub fn set_stage_on(r: &mut Rocket, stage: usize, kind: EventKind, on: StageOn) -> bool {
    edit_stage_event(r, stage, kind, |e| {
        e.trigger = match (on, e.trigger) {
            (StageOn::Burnout, _) => StageTrigger::Burnout,
            (StageOn::Separation, _) => StageTrigger::Separation,
            (StageOn::Launch, _) => StageTrigger::TimeFromLaunch,
            (StageOn::Altitude, StageTrigger::Altitude(h)) => StageTrigger::Altitude(h),
            (StageOn::Altitude, _) => StageTrigger::Altitude(DEFAULT_STAGE_ALTITUDE),
        }
    })
}

/// The tilt lockout an ignition is given when it is switched on (20°, a common electronics default).
pub const DEFAULT_TILT_LOCKOUT: f64 = 20.0 * std::f64::consts::PI / 180.0;

/// Sets (or with `None` removes) the tilt lockout of stage `stage`'s ignition in the active configuration (see
/// [`StageEvent::tilt_lockout`]).
pub fn set_tilt_lockout(r: &mut Rocket, stage: usize, lockout: Option<f64>) -> bool {
    edit_stage_event(r, stage, EventKind::Ignition, |e| e.tilt_lockout = lockout)
}

/// Plugs (no ejection charge) or unplugs the motor in `mount` of the active configuration. Unplugging gives it a 0 s delay.
pub fn set_plugged(r: &mut Rocket, mount: PartId, plugged: bool) -> bool {
    let config = r.active_config_id();
    let delay = if plugged { f64::INFINITY } else { 0.0 };
    r.set_motor_delay(config, mount, delay).is_ok()
}

/// Gives a parachute another shroud-line material.
pub fn set_line_material(r: &mut Rocket, id: PartId, material: bowshock_model::MaterialId) -> bool {
    let Some(mut kind) = r.internal(id).map(|p| p.kind.clone()) else {
        return false;
    };
    match &mut kind {
        InternalKind::Parachute { line_material, .. } if material.kind() == bowshock_model::MaterialKind::Line => *line_material = material,
        _ => return false,
    }
    r.set_internal_kind(id, kind)
}

/// Changes the number of copies of a radial group.
pub fn set_count(r: &mut Rocket, id: PartId, count: u8) -> bool {
    match r.radial(id).map(|p| p.kind.clone()) {
        Some(kind) => r.set_radial(id, kind, count),
        None => false,
    }
}

/// Gives a nose cone another profile family (parametric ones start at their conventional parameter).
pub fn set_nose_shape(r: &mut Rocket, id: PartId, shape: bowshock_model::NoseShape) -> bool {
    match r.part(id).map(|p| p.kind.clone()) {
        Some(bowshock_model::PartKind::NoseCone { length, diameter, .. }) => r.set_kind(id, bowshock_model::PartKind::NoseCone { shape, length, diameter }),
        _ => false,
    }
}

/// Gives a fin another planform (see `RadialKind::with_planform`).
pub fn set_planform(r: &mut Rocket, id: PartId, planform: bowshock_model::FinPlanform) -> bool {
    let Some(radial) = r.radial(id) else {
        return false;
    };
    let (kind, count) = (radial.kind.with_planform(planform), radial.count);
    r.set_radial(id, kind, count)
}

/// A copy of part `id`, placed right after it (axial) or in the same place (radial, internal) with the same values, name,
/// material, finish and overrides. Returns the new part's id. A nose cone cannot be duplicated.
pub fn duplicate(r: &mut Rocket, id: PartId) -> Option<PartId> {
    if let Some(part) = r.part(id).cloned() {
        let new = match r.pod_part_index(id) {
            Some((pod, index)) => r.insert_in_pod(pod, index + 1, part.kind.clone()).ok()?,
            None => {
                let index = r.index_of(id)?;
                r.insert(index + 1, part.kind.clone()).ok()?
            }
        };
        r.set_name(new, part.name.clone());
        r.set_material(new, part.material);
        r.set_finish(new, part.finish);
        r.set_kind(new, part.kind.clone());
        r.set_thickness(new, part.thickness);
        r.set_mass_override(new, part.mass_override);
        r.set_cg_override(new, part.cg_override);
        return Some(new);
    }
    if let Some(radial) = r.radial(id).cloned() {
        let new = r.attach_radial(radial.parent, radial.offset, radial.angle, radial.count, radial.kind.clone()).ok()?;
        r.set_name(new, radial.name.clone());
        r.set_material(new, radial.material);
        r.set_finish(new, radial.finish);
        r.set_fin_section(new, radial.cross_section);
        r.set_cant(new, radial.cant);
        r.set_overrides(new, radial.overrides.clone());
        r.set_mass_override(new, radial.mass_override);
        r.set_cg_override(new, radial.cg_override);
        return Some(new);
    }
    let part = r.internal(id).cloned()?;
    // A second motor mount is a second stage: the copy is an ordinary inner tube until the user makes it one.
    let mut kind = part.kind.clone();
    if let InternalKind::InnerTube { motor_mount, .. } = &mut kind {
        *motor_mount = false;
    }
    let new = r.attach_internal(part.parent, part.offset, kind).ok()?;
    r.set_name(new, part.name.clone());
    r.set_material(new, part.material);
    r.set_mass_override(new, part.mass_override);
    r.set_cg_override(new, part.cg_override);
    Some(new)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::Anchor;
    use bowshock_model::{Deployment, FinPlanform, MaterialId, NoseShape, PartKind, RadialKind};

    /// Nose, one 600 mm tube with a 3-fin set at its aft end, a motor mount, a ring, a parachute and a ballast weight.
    fn rocket() -> (Rocket, PartId, PartId, [PartId; 4]) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: 0.0762 }).unwrap();
        let fins = r
            .attach_radial(tube, 0.48, 0.0, 3, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.00318 })
            .unwrap();
        let mount = r
            .attach_internal(tube, 0.3, InternalKind::InnerTube { outer_diameter: 0.042, thickness: 0.002, length: 0.3, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let ring = r.attach_internal(tube, 0.3, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 }).unwrap();
        let chute = r
            .attach_internal(
                tube,
                0.05,
                InternalKind::Parachute {
                    diameter: 0.9,
                    cd: 0.8,
                    line_count: 6,
                    line_length: 0.9,
                    line_material: MaterialId::BRAIDED_NYLON_2MM,
                    packed_length: 0.2,
                    packed_diameter: None,
                    deploy: Deployment { event: DeployEvent::Ejection, delay: 0.0 },
                },
            )
            .unwrap();
        let ballast = r.attach_internal(tube, 0.02, InternalKind::MassComponent { mass: 0.1, length: 0.04, diameter: None, role: MassRole::Ballast }).unwrap();
        (r, tube, fins, [mount, ring, chute, ballast])
    }

    #[test]
    fn dimension_fields_go_through_set_dim_and_report_the_limit() {
        let (mut r, tube, fins, _) = rocket();
        assert_eq!(apply(&mut r, fins, Field::Dim(Dim::RootChord), 0.1), Applied { done: true, limited: None });
        assert_eq!(r.dim(fins, Dim::RootChord), Some(0.1));
        let hit = apply(&mut r, tube, Field::Dim(Dim::Length), 0.0001);
        assert!(hit.done && hit.limited == Some("MINIMUM LENGTH"));
        assert_eq!(apply(&mut r, tube, Field::Dim(Dim::TipChord), 0.01), Applied::default());
        assert_eq!(apply(&mut r, tube, Field::Dim(Dim::Length), f64::NAN), Applied::default());
    }

    #[test]
    fn typing_the_sweep_angle_drives_the_sweep_length() {
        let (mut r, _, fins, _) = rocket();
        // 38.2 degrees over a 76.2 mm span is a 60 mm sweep: the kit's example.
        let span = r.dim(fins, Dim::Span).unwrap();
        assert!(apply(&mut r, fins, Field::SweepAngle, 45f64.to_radians()).done);
        assert!((r.dim(fins, Dim::Sweep).unwrap() - span).abs() < 1e-12);
        assert!((r.dim(fins, Dim::Sweep).unwrap().atan2(span).to_degrees() - 45.0).abs() < 1e-9);
        assert_eq!(apply(&mut r, fins, Field::SweepAngle, 89f64.to_radians()).limited, Some("SWEEP ANGLE LIMIT 85°"));
    }

    #[test]
    fn typing_the_inner_diameter_drives_the_wall() {
        let (mut r, tube, fins, _) = rocket();
        assert!(apply(&mut r, tube, Field::BoreDiameter, 0.0702).done);
        assert!((r.part(tube).unwrap().thickness - 0.003).abs() < 1e-12);
        assert!(!apply(&mut r, fins, Field::BoreDiameter, 0.05).done);
    }

    #[test]
    fn auto_diameters_resolve_until_typed_and_come_back_on_reset() {
        let (mut r, _, _, [_, ring, chute, ballast]) = rocket();
        let bore = 0.0762 - 2.0 * 0.0016;
        assert!(r.resolved(ring).unwrap().outer_auto);
        assert!((r.resolved(ring).unwrap().outer_diameter - bore).abs() < 1e-9);
        assert!(apply(&mut r, ring, Field::AutoDiameter, 0.05).done);
        let res = r.resolved(ring).unwrap();
        assert!(!res.outer_auto && (res.outer_diameter - 0.05).abs() < 1e-12);
        assert!(set_auto_diameter(&mut r, ring, None));
        assert!(r.resolved(ring).unwrap().outer_auto);
        assert!(apply(&mut r, chute, Field::AutoDiameter, 0.05).done && !r.resolved(chute).unwrap().outer_auto);
        assert!(apply(&mut r, ballast, Field::AutoDiameter, 0.03).done);
        let nose = r.ids()[0];
        assert!(!set_auto_diameter(&mut r, nose, Some(0.03)));
        assert!(apply(&mut r, ring, Field::Dim(Dim::InnerDiameter), 0.03).done);
        assert!(reset_inner_diameter(&mut r, ring) && r.resolved(ring).unwrap().inner_auto);
    }

    #[test]
    fn recovery_fields_edit_the_parachute() {
        let (mut r, _, _, [_, _, chute, ballast]) = rocket();
        assert!(apply(&mut r, chute, Field::CanopyDiameter, 1.2).done);
        assert!(apply(&mut r, chute, Field::Cd, 99.0).done);
        assert!(apply(&mut r, chute, Field::LineCount, 8.4).done);
        assert!(apply(&mut r, chute, Field::DeployDelay, 1.5).done);
        let InternalKind::Parachute { diameter, cd, line_count, deploy, .. } = r.internal(chute).unwrap().kind else { unreachable!() };
        assert_eq!((diameter, cd, line_count), (1.2, 3.0, 8));
        // The deployment is stored with the active configuration; the part keeps its own value as the default.
        assert_eq!(deploy, Deployment { event: DeployEvent::Ejection, delay: 0.0 });
        assert_eq!(r.active_deployment(chute), Some(Deployment { event: DeployEvent::Ejection, delay: 1.5 }));
        let other = r.add_configuration("Other");
        assert_eq!(r.deployment(other, chute), Some(Deployment { event: DeployEvent::Ejection, delay: 0.0 }), "other configurations keep the default");
        let active = |r: &Rocket| r.active_deployment(chute).unwrap().event;
        assert!(set_trigger(&mut r, chute, Trigger::Altitude));
        assert_eq!(active(&r), DeployEvent::Altitude(DEFAULT_DEPLOY_ALTITUDE));
        assert!(apply(&mut r, chute, Field::DeployAltitude, 300.0).done);
        // Switching to the same trigger keeps the altitude.
        assert!(set_trigger(&mut r, chute, Trigger::Altitude));
        assert_eq!(active(&r), DeployEvent::Altitude(300.0));
        assert!(set_trigger(&mut r, chute, Trigger::Launch));
        assert_eq!(Trigger::of(active(&r)), Trigger::Launch);
        assert!(set_trigger(&mut r, chute, Trigger::Apogee));
        assert_eq!(Trigger::of(active(&r)), Trigger::Apogee);
        // The parachute's fields do nothing on other parts.
        assert!(!apply(&mut r, ballast, Field::CanopyDiameter, 1.0).done && !set_trigger(&mut r, ballast, Trigger::Apogee));
    }

    #[test]
    fn stage_events_edit_the_active_configuration() {
        let (mut r, _, _, _) = rocket();
        let first = r.active_config_id();
        let second = r.add_configuration("Other");
        assert!(apply(&mut r, 0, Field::StageDelay(1, EventKind::Separation), 0.75).done);
        assert!(set_stage_on(&mut r, 1, EventKind::Separation, StageOn::Altitude));
        assert!(apply(&mut r, 0, Field::StageAltitude(1, EventKind::Separation), 250.0).done);
        let e = r.configuration(first).unwrap().event(1, EventKind::Separation);
        assert_eq!((e.trigger, e.delay), (StageTrigger::Altitude(250.0), 0.75));
        assert_eq!(r.configuration(second).unwrap().event(1, EventKind::Separation), StageEvent::default_of(1, EventKind::Separation));
        // Same choice again keeps the altitude; a separation cannot wait for a separation, an ignition can.
        assert!(set_stage_on(&mut r, 1, EventKind::Separation, StageOn::Altitude));
        assert_eq!(r.active_configuration().event(1, EventKind::Separation).trigger, StageTrigger::Altitude(250.0));
        assert!(!set_stage_on(&mut r, 1, EventKind::Separation, StageOn::Separation));
        assert!(set_stage_on(&mut r, 2, EventKind::Ignition, StageOn::Launch));
        assert_eq!(r.active_configuration().event(2, EventKind::Ignition).trigger, StageTrigger::TimeFromLaunch);
        assert!(apply(&mut r, 0, Field::StageDelay(2, EventKind::Ignition), -3.0).done);
        assert_eq!(r.active_configuration().event(2, EventKind::Ignition).delay, 0.0, "a negative delay is 0");
    }

    #[test]
    fn mass_component_role_changes_the_stock_unless_the_user_chose_one() {
        let (mut r, _, _, [_, _, _, ballast]) = rocket();
        assert_eq!(r.internal(ballast).unwrap().material, MaterialId::LEAD);
        assert!(set_role(&mut r, ballast, MassRole::Altimeter));
        assert_eq!(r.internal(ballast).unwrap().material, MaterialId::ALUMINUM);
        r.set_material(ballast, MaterialId::BRASS);
        assert!(set_role(&mut r, ballast, MassRole::Ballast));
        assert_eq!(r.internal(ballast).unwrap().material, MaterialId::BRASS);
        assert!(apply(&mut r, ballast, Field::MassValue, 0.25).done);
        assert_eq!(r.component_mass(ballast).unwrap().mass, 0.25);
    }

    #[test]
    fn overrides_set_and_clear() {
        let (mut r, tube, ..) = rocket();
        let computed = r.component_mass(tube).unwrap().mass;
        assert!(apply(&mut r, tube, Field::MassOverride, 0.2).done);
        assert_eq!(r.component_mass(tube).unwrap().mass, 0.2);
        assert!(r.set_mass_override(tube, None));
        assert_eq!(r.component_mass(tube).unwrap().mass, computed);
        assert!(apply(&mut r, tube, Field::CgOverride, 0.1).done);
        assert!(r.part(tube).unwrap().cg_override == Some(0.1));
    }

    #[test]
    fn anchors_convert_both_ways() {
        // A 120 mm fin flush with the aft end of a 393.4 mm tube sits at 273.4: offset 0 from the bottom.
        let (offset, extent, length) = (0.2734, 0.12, 0.3934);
        assert!(Anchor::Bottom.shown(offset, extent, length).abs() < 1e-12);
        assert_eq!(Anchor::Top.shown(offset, extent, length), offset);
        for anchor in Anchor::ALL {
            let shown = anchor.shown(offset, extent, length);
            assert!((anchor.offset(shown, extent, length) - offset).abs() < 1e-12);
        }
        // Centred: 136.7 from the fore face puts the middle of the fin at the middle of the tube.
        assert!(Anchor::Middle.shown((length - extent) / 2.0, extent, length).abs() < 1e-12);
        // Positive is aft (the .ork convention): moving the part aft raises every anchor's value.
        assert!(Anchor::Bottom.shown(offset + 0.01, extent, length) > 0.0);
        assert_eq!(Anchor::Bottom.label("Booster"), "Bottom of Booster");
    }

    #[test]
    fn count_planform_and_shape_changes() {
        let (mut r, tube, fins, _) = rocket();
        assert!(set_count(&mut r, fins, 4) && r.radial(fins).unwrap().count == 4);
        assert!(!set_count(&mut r, fins, 0));
        assert!(set_planform(&mut r, fins, FinPlanform::Elliptical));
        assert_eq!(r.radial(fins).unwrap().kind.fin_planform(), Some(FinPlanform::Elliptical));
        assert!(matches!(r.radial(fins).unwrap().kind, RadialKind::EllipticalFin { .. }));
        let nose = r.parts()[0].id;
        assert!(set_nose_shape(&mut r, nose, NoseShape::Haack(1.0 / 3.0)));
        assert_eq!(r.dim(nose, Dim::Shape), Some(1.0 / 3.0));
        assert!(!set_nose_shape(&mut r, tube, NoseShape::Conical));
    }

    #[test]
    fn duplicates_carry_values_and_never_a_second_motor_mount() {
        let (mut r, tube, fins, [mount, _, chute, _]) = rocket();
        r.set_name(tube, Some("Recovery bay".into()));
        r.set_material(tube, MaterialId::FIBERGLASS);
        let copy = duplicate(&mut r, tube).unwrap();
        assert_eq!(r.index_of(copy), Some(r.index_of(tube).unwrap() + 1));
        let (a, b) = (r.part(tube).unwrap(), r.part(copy).unwrap());
        assert_eq!((a.kind.clone(), a.name.clone(), a.material, a.thickness), (b.kind.clone(), b.name.clone(), b.material, b.thickness));
        let fin_copy = duplicate(&mut r, fins).unwrap();
        assert_eq!((r.radial(fin_copy).unwrap().count, r.radial(fin_copy).unwrap().kind.clone()), (3, r.radial(fins).unwrap().kind.clone()));
        let mount_copy = duplicate(&mut r, mount).unwrap();
        assert!(!r.internal(mount_copy).unwrap().kind.is_motor_mount() && r.internal(mount).unwrap().kind.is_motor_mount());
        assert!(duplicate(&mut r, chute).is_some());
        assert_eq!(duplicate(&mut r, 9999), None);
        let nose = r.parts()[0].id;
        assert_eq!(duplicate(&mut r, nose), None);
    }

    #[test]
    fn field_keys_are_stable_and_height_is_one_field() {
        assert_eq!(Field::Dim(Dim::Span).key(), Field::Dim(Dim::Height).key());
        assert_eq!(focus_key(Dim::Span), "insp:Height");
        assert_eq!(Field::AutoDiameter.key(), focus_key(Dim::Diameter));
        assert_ne!(Field::Dim(Dim::Wall).key(), Field::Dim(Dim::Thickness).key());
        assert_eq!(Field::Dim(Dim::RootChord).undo_label(), "Edit root chord");
    }
}
