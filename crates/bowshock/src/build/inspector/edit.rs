//! What the inspector's fields do to the rocket. Every function here is a pure edit of a `&mut Rocket` (the update step wraps
//! them in `Document::edit` / `edit_merging`), so the behaviour is testable without a window.
//!
//! Geometry goes through `bowshock_model::Rocket::set_dim` (DirectEdit's limits and neighbour following), so the field and
//! the handle in the viewport can never disagree. The fields that are not a plain dimension (sweep angle, an auto diameter, the
//! parachute's numbers, mass overrides) are the [`Field`] variants beside [`Field::Dim`].

use bowshock_model::{DeployEvent, Dim, InternalKind, MassRole, PartId, Rocket};

/// Most a shroud-line count may be.
const MAX_LINES: f64 = 64.0;
/// Altitude a parachute is given when its trigger is switched to an altitude (500 ft, a typical main deployment).
pub const DEFAULT_DEPLOY_ALTITUDE: f64 = 152.4;
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
    /// An internal part's diameter that may follow its parent (`AUTO`): a ring's outer, an engine block's outer, a bulkhead's,
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
    /// The altitude a recovery device waits for.
    DeployAltitude,
    /// The delay after a recovery device's trigger.
    DeployDelay,
    /// The mass override.
    MassOverride,
    /// The CG override, metres behind the part's fore face.
    CgOverride,
    /// The active configuration's ejection delay of a motor mount.
    EjectionDelay,
    /// The active configuration's ignition delay of a motor mount (stages above the first).
    IgnitionDelay,
    /// The active configuration's separation delay of stage `n` (`Stage::index`).
    SeparationDelay(usize),
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
            Field::DeployAltitude => "deploy altitude".into(),
            Field::DeployDelay => "deploy delay".into(),
            Field::CgOverride => "CG".into(),
            Field::EjectionDelay => "ejection delay".into(),
            Field::IgnitionDelay => "ignition delay".into(),
            Field::SeparationDelay(_) => "separation delay".into(),
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

/// Applies `value` (SI) to `field` of part `id`.
pub fn apply(r: &mut Rocket, id: PartId, field: Field, value: f64) -> Applied {
    if !value.is_finite() {
        return Applied::no();
    }
    match field {
        Field::Dim(dim) => match r.set_dim(id, dim, value) {
            Some(edit) => Applied::yes(edit.limited),
            None => Applied::no(),
        },
        Field::SweepAngle => {
            let Some(span) = r.dim(id, Dim::Span) else {
                return Applied::no();
            };
            let angle = value.clamp(-MAX_SWEEP_ANGLE, MAX_SWEEP_ANGLE);
            let limited = (angle != value).then_some("SWEEP ANGLE LIMIT 85°");
            match r.set_dim(id, Dim::Sweep, span * angle.tan()) {
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
        Field::CgOverride => Applied::flag(r.set_cg_override(id, Some(value))),
        Field::EjectionDelay => {
            let config = r.active_config_id();
            Applied::flag(r.set_motor_delay(config, id, value.max(0.0)).is_ok())
        }
        Field::IgnitionDelay => {
            let config = r.active_config_id();
            Applied::flag(r.set_ignition(config, id, bowshock_model::Ignition::StageSeparation { delay: value.max(0.0) }).is_ok())
        }
        Field::SeparationDelay(stage) => {
            let config = r.active_config_id();
            Applied::flag(r.set_stage_separation(config, stage, value.max(0.0)).is_ok())
        }
        other => Applied::flag(edit_internal(r, id, other, value)),
    }
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
        (InternalKind::Parachute { deploy, .. } | InternalKind::Streamer { deploy, .. }, Field::DeployAltitude) => {
            deploy.event = DeployEvent::Altitude(value.max(0.0));
            true
        }
        (InternalKind::Parachute { deploy, .. } | InternalKind::Streamer { deploy, .. }, Field::DeployDelay) => {
            deploy.delay = value.max(0.0);
            true
        }
        _ => false,
    };
    changed && r.set_internal_kind(id, kind)
}

/// Sets (or with `None` resets to automatic) the diameter that may follow the parent: a ring's or engine block's outer, a
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
        | InternalKind::EngineBlock { outer: d, .. }
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
}

impl Trigger {
    /// Every trigger, in menu order.
    pub const ALL: [Trigger; 3] = [Trigger::Ejection, Trigger::Apogee, Trigger::Altitude];

    /// Display name.
    pub fn name(self) -> &'static str {
        match self {
            Trigger::Apogee => "Apogee",
            Trigger::Altitude => "Altitude",
            Trigger::Ejection => "Motor ejection",
        }
    }

    /// The trigger of `event`.
    pub fn of(event: DeployEvent) -> Trigger {
        match event {
            DeployEvent::Apogee => Trigger::Apogee,
            DeployEvent::Altitude(_) => Trigger::Altitude,
            DeployEvent::Ejection => Trigger::Ejection,
        }
    }
}

/// The deployment of a parachute or streamer.
#[cfg(test)]
fn deployment(kind: &InternalKind) -> Option<bowshock_model::Deployment> {
    match kind {
        InternalKind::Parachute { deploy, .. } | InternalKind::Streamer { deploy, .. } => Some(*deploy),
        _ => None,
    }
}

/// Switches a recovery device's trigger, keeping its delay (an altitude trigger starts at [`DEFAULT_DEPLOY_ALTITUDE`], or the
/// altitude it already had).
pub fn set_trigger(r: &mut Rocket, id: PartId, trigger: Trigger) -> bool {
    let Some(mut kind) = r.internal(id).map(|p| p.kind.clone()) else {
        return false;
    };
    match &mut kind {
        InternalKind::Parachute { deploy, .. } | InternalKind::Streamer { deploy, .. } => {
            deploy.event = match (trigger, deploy.event) {
                (Trigger::Apogee, _) => DeployEvent::Apogee,
                (Trigger::Ejection, _) => DeployEvent::Ejection,
                (Trigger::Altitude, DeployEvent::Altitude(h)) => DeployEvent::Altitude(h),
                (Trigger::Altitude, _) => DeployEvent::Altitude(DEFAULT_DEPLOY_ALTITUDE),
            }
        }
        _ => return false,
    }
    r.set_internal_kind(id, kind)
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

/// When a motor lights, as the ignition select offers it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ignite {
    /// On the pad.
    Launch,
    /// After the stage below separates.
    Separation,
}

/// Sets when the motor in `mount` lights in the active configuration. Moving to "Separation" keeps a delay it already had.
pub fn set_ignite(r: &mut Rocket, mount: PartId, on: Ignite) -> bool {
    let config = r.active_config_id();
    let ignition = match (on, r.active_configuration().slot(mount).map(|s| s.ignition)) {
        (Ignite::Launch, _) => bowshock_model::Ignition::Launch,
        (Ignite::Separation, Some(existing @ bowshock_model::Ignition::StageSeparation { .. })) => existing,
        (Ignite::Separation, _) => bowshock_model::Ignition::StageSeparation { delay: 0.0 },
    };
    r.set_ignition(config, mount, ignition).is_ok()
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

/// Where a radial or internal part's offset is measured from: the position section's *Relative to*.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Anchor {
    /// The part's fore edge from the parent's fore face.
    Top,
    /// The part's middle from the parent's middle.
    Middle,
    /// The part's aft edge from the parent's aft face (a fin flush with the tube's end has offset 0).
    Bottom,
}

impl Anchor {
    /// Every anchor, in menu order.
    pub const ALL: [Anchor; 3] = [Anchor::Top, Anchor::Middle, Anchor::Bottom];

    /// The menu label for a parent named `parent`.
    pub fn label(self, parent: &str) -> String {
        match self {
            Anchor::Top => format!("Top of {parent}"),
            Anchor::Middle => format!("Middle of {parent}"),
            Anchor::Bottom => format!("Bottom of {parent}"),
        }
    }

    /// The displayed offset (positive aft, as in OpenRocket) of a part whose fore edge is `offset` from the parent's fore
    /// face, `extent` long, in a parent `parent_length` long.
    pub fn shown(self, offset: f64, extent: f64, parent_length: f64) -> f64 {
        match self {
            Anchor::Top => offset,
            Anchor::Middle => offset + extent / 2.0 - parent_length / 2.0,
            Anchor::Bottom => offset + extent - parent_length,
        }
    }

    /// The inverse of [`shown`](Self::shown): the offset from the parent's fore face for a displayed offset.
    pub fn offset(self, shown: f64, extent: f64, parent_length: f64) -> f64 {
        match self {
            Anchor::Top => shown,
            Anchor::Middle => shown + parent_length / 2.0 - extent / 2.0,
            Anchor::Bottom => shown + parent_length - extent,
        }
    }
}

/// A copy of part `id`, placed right after it (axial) or in the same place (radial, internal) with the same values, name,
/// material, finish and overrides. Returns the new part's id. A nose cone cannot be duplicated.
pub fn duplicate(r: &mut Rocket, id: PartId) -> Option<PartId> {
    if let Some(part) = r.part(id).cloned() {
        let index = r.index_of(id)?;
        let new = r.insert(index + 1, part.kind.clone()).ok()?;
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

/// Shear modulus of a material in Pa: the approximation the flutter chip uses. Handbook figures for the isotropic
/// equivalent: wood and plywood from the USDA *Wood Handbook* (G about 1/16 to 1/12 of E along the grain, fins being
/// cross-grain laminates), plastics from supplier data sheets (G = E / 2(1 + nu), nu 0.35), metals and composites from MatWeb.
/// It is a first estimate (a laminate's real stiffness depends on its layup), enough to tell a 1.2x from a 6x margin.
pub fn shear_modulus(material: bowshock_model::MaterialId) -> f64 {
    use bowshock_model::MaterialId as M;
    match material {
        M::BALSA => 0.2e9,
        M::BASSWOOD => 0.6e9,
        M::BIRCH_PLYWOOD | M::AIRCRAFT_PLYWOOD => 1.0e9,
        M::CARDBOARD => 0.3e9,
        M::KRAFT_PHENOLIC => 1.5e9,
        M::BLUE_TUBE => 2.0e9,
        M::FIBERGLASS => 4.0e9,
        M::CARBON_FIBER => 5.0e9,
        M::ALUMINUM => 26.0e9,
        M::STEEL => 79.0e9,
        M::BRASS => 39.0e9,
        M::LEAD => 5.6e9,
        M::PLA => 1.3e9,
        M::ABS => 0.8e9,
        M::POLYCARBONATE => 0.85e9,
        M::POLYSTYRENE => 1.3e9,
        _ => 1.0e9,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
        assert_eq!(deploy, Deployment { event: DeployEvent::Ejection, delay: 1.5 });
        assert!(set_trigger(&mut r, chute, Trigger::Altitude));
        assert_eq!(deployment(&r.internal(chute).unwrap().kind).unwrap().event, DeployEvent::Altitude(DEFAULT_DEPLOY_ALTITUDE));
        assert!(apply(&mut r, chute, Field::DeployAltitude, 300.0).done);
        // Switching away and back keeps nothing but the delay; switching to the same trigger keeps the altitude.
        assert!(set_trigger(&mut r, chute, Trigger::Altitude));
        assert_eq!(deployment(&r.internal(chute).unwrap().kind).unwrap().event, DeployEvent::Altitude(300.0));
        assert!(set_trigger(&mut r, chute, Trigger::Apogee));
        assert_eq!(Trigger::of(deployment(&r.internal(chute).unwrap().kind).unwrap().event), Trigger::Apogee);
        // The parachute's fields do nothing on other parts.
        assert!(!apply(&mut r, ballast, Field::CanopyDiameter, 1.0).done && !set_trigger(&mut r, ballast, Trigger::Apogee));
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
        // Positive is aft (OpenRocket): moving the part aft raises every anchor's value.
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
    fn shear_moduli_order_the_materials_sensibly() {
        assert!(shear_modulus(MaterialId::BALSA) < shear_modulus(MaterialId::BIRCH_PLYWOOD));
        assert!(shear_modulus(MaterialId::BIRCH_PLYWOOD) < shear_modulus(MaterialId::FIBERGLASS));
        assert!(shear_modulus(MaterialId::FIBERGLASS) < shear_modulus(MaterialId::ALUMINUM));
        // Every database material has a positive modulus.
        assert!(bowshock_model::materials().iter().all(|m| shear_modulus(m.id) > 0.0));
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
