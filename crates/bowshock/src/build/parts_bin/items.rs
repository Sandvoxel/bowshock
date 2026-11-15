//! What each parts-bin category offers, whether a row is available for the current selection, and where a click puts it.
//!
//! Everything here is a function of the rocket and the selection, so it is unit tested without a window.

use bowshock_model::{DeployEvent, Deployment, InternalKind, MassRole, MaterialId, MotorMount, NoseShape, Part, PartId, PartKind, RadialKind, Rocket, System};

use super::Category;
use crate::app::{drag, Ctx, Selection};

/// A row of the bin: its category and its position in that category's full (unfiltered) list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RowId {
    /// The category the row belongs to (search shows rows of every category).
    pub category: Category,
    /// Index into [`items`] of that category.
    pub index: usize,
}

impl RowId {
    /// Row `index` of `category`.
    pub fn new(category: Category, index: usize) -> Self {
        RowId { category, index }
    }
}

/// Something a row can add.
#[derive(Debug, Clone)]
pub enum Item {
    /// A nose cone, tube or transition (an axial part).
    Axial(&'static str, PartKind),
    /// Fins, a lug or a rail button.
    Radial(&'static str, RadialKind),
    /// Something that lives inside an axial part.
    Internal(&'static str, InternalKind),
    /// A motor, by index into the motor database; a click puts it in the active configuration.
    Motor(usize),
    /// A saved assembly, by index into the bin's preset list.
    Preset(usize),
}

/// The airframe parts offered, with their default sizes (metres). The names are the bin's: the model calls any contracting
/// transition a boattail, but the bin offers a plain Transition that happens to contract too.
pub fn axial_catalogue() -> Vec<(&'static str, PartKind)> {
    let nose = |shape| PartKind::NoseCone { shape, length: 0.2286, diameter: 0.0762 };
    vec![
        ("Nose cone", nose(NoseShape::TangentOgive)),
        ("Conical nose", nose(NoseShape::Conical)),
        ("Elliptical nose", nose(NoseShape::Elliptical)),
        ("Von Kármán nose", nose(NoseShape::VonKarman)),
        ("Parabolic nose", nose(NoseShape::Parabolic)),
        ("Body tube", PartKind::BodyTube { length: 0.600, diameter: 0.0762 }),
        ("Transition", PartKind::Transition { length: 0.0762, fore_diameter: 0.0762, aft_diameter: 0.054 }),
        ("Boattail", PartKind::Transition { length: 0.050, fore_diameter: 0.0762, aft_diameter: 0.060 }),
        ("Payload bay", PartKind::BodyTube { length: 0.250, diameter: 0.0762 }),
    ]
}

/// The airframe list in the bin's order: the axial catalogue with the tube coupler (an internal part) after the transition.
pub fn airframe_items() -> Vec<Item> {
    let mut items: Vec<Item> = axial_catalogue().into_iter().map(|(n, k)| Item::Axial(n, k)).collect();
    let at = items.iter().position(|i| matches!(i, Item::Axial("Boattail", _))).unwrap_or(items.len());
    items.insert(at, Item::Internal("Tube coupler", coupler()));
    items
}

/// The radial parts offered ("Fins & guides"), with their default sizes (metres).
pub fn radial_catalogue() -> Vec<(&'static str, RadialKind)> {
    vec![
        ("Trapezoidal fins", RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.00318 }),
        ("Elliptical fins", RadialKind::EllipticalFin { root_chord: 0.12, span: 0.0762, thickness: 0.00318 }),
        ("Freeform fins", RadialKind::FreeformFin { points: vec![(0.0, 0.0), (0.03, 0.0762), (0.07, 0.0762), (0.09, 0.03), (0.12, 0.0)], thickness: 0.00318 }),
        ("Launch lug", RadialKind::LaunchLug { length: 0.0508, diameter: 0.0095 }),
        ("Rail button", RadialKind::RailButton { height: 0.008, diameter: 0.011 }),
    ]
}

fn coupler() -> InternalKind {
    InternalKind::Coupler { outer_diameter: None, thickness: 0.0016, length: 0.1524 }
}

/// The structural internals ("Internal"), default sizes in metres. Diameters that are `None` follow the part they go in.
pub fn structure_catalogue() -> Vec<(&'static str, InternalKind)> {
    vec![
        ("Motor mount", InternalKind::InnerTube { outer_diameter: 0.0413, thickness: 0.0016, length: 0.2, motor_mount: true, overhang: 0.0 }),
        ("Inner tube", InternalKind::InnerTube { outer_diameter: 0.0413, thickness: 0.0016, length: 0.15, motor_mount: false, overhang: 0.0 }),
        ("Centering ring", InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.00635 }),
        ("Bulkhead", InternalKind::Bulkhead { diameter: None, thickness: 0.00635 }),
        ("Engine block", InternalKind::EngineBlock { outer: None, inner: 0.0381, thickness: 0.00635 }),
        ("Coupler", coupler()),
    ]
}

/// The recovery devices, default sizes in metres.
pub fn recovery_catalogue() -> Vec<(&'static str, InternalKind)> {
    let chute = |diameter: f64, cd: f64, lines: u32, packed: f64, event: DeployEvent| InternalKind::Parachute {
        diameter,
        cd,
        line_count: lines,
        line_length: diameter,
        line_material: MaterialId::BRAIDED_NYLON_2MM,
        packed_length: packed,
        packed_diameter: None,
        deploy: Deployment { event, delay: 0.0 },
    };
    vec![
        ("Parachute", chute(0.4572, 0.8, 8, 0.12, DeployEvent::Altitude(244.0))),
        ("Drogue", chute(0.3048, 0.75, 6, 0.09, DeployEvent::Apogee)),
        ("Streamer", InternalKind::Streamer { length: 0.762, width: 0.0508, cd: 0.6, deploy: Deployment { event: DeployEvent::Ejection, delay: 0.0 } }),
        ("Shock cord", InternalKind::ShockCord { length: 3.0, packed_length: 0.08 }),
    ]
}

/// The mass components, default sizes in metres and kilograms.
pub fn mass_catalogue() -> Vec<(&'static str, InternalKind)> {
    let mass = |mass: f64, length: f64, role: MassRole| InternalKind::MassComponent { mass, length, diameter: None, role };
    vec![
        ("Altimeter", mass(0.030, 0.060, MassRole::Altimeter)),
        ("Battery", mass(0.045, 0.050, MassRole::Battery)),
        ("Tracker", mass(0.050, 0.070, MassRole::Tracker)),
        ("Payload", mass(0.250, 0.120, MassRole::Payload)),
        ("Ballast", mass(0.100, 0.030, MassRole::Ballast)),
    ]
}

/// Every internal, recovery and mass part the bin offers, with default sizes (for menus that add a part elsewhere, such as
/// Station Lanes).
#[allow(dead_code)] // For the lanes' "Add here" menu.
pub fn internal_catalogue() -> Vec<(&'static str, InternalKind)> {
    structure_catalogue().into_iter().chain(recovery_catalogue()).chain(mass_catalogue()).collect()
}

/// `kind` with its diameters scaled so the largest end is `diameter` (lengths and shape unchanged); `None` leaves it alone.
pub fn sized(kind: &PartKind, diameter: Option<f64>) -> PartKind {
    let Some(d) = diameter else {
        return kind.clone();
    };
    match *kind {
        PartKind::NoseCone { shape, length, .. } => PartKind::NoseCone { shape, length, diameter: d },
        PartKind::BodyTube { length, .. } => PartKind::BodyTube { length, diameter: d },
        PartKind::Transition { length, fore_diameter, aft_diameter } => {
            let k = d / fore_diameter.max(aft_diameter);
            PartKind::Transition { length, fore_diameter: fore_diameter * k, aft_diameter: aft_diameter * k }
        }
    }
}

/// A motor mount or inner tube too wide for `host_inner` is narrowed to 80 % of it (keeping its wall); other kinds are unchanged.
pub fn fitted(kind: &InternalKind, host_inner: f64) -> InternalKind {
    match *kind {
        InternalKind::InnerTube { outer_diameter, thickness, length, motor_mount, overhang } if outer_diameter > host_inner * 0.95 && host_inner > 0.0 => {
            InternalKind::InnerTube { outer_diameter: host_inner * 0.8, thickness, length, motor_mount, overhang }
        }
        InternalKind::EngineBlock { outer, inner, thickness } if inner > host_inner * 0.8 && host_inner > 0.0 => {
            InternalKind::EngineBlock { outer, inner: host_inner * 0.6, thickness }
        }
        _ => kind.clone(),
    }
}

/// Where a part of this kind goes when it is not placed by hand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Zone {
    /// The aft-most tube (motor mount, rings, bulkheads).
    Aft,
    /// The forward-most tube (recovery gear and avionics).
    Forward,
}

/// The zone an internal part of this kind goes in.
pub fn zone_of(kind: &InternalKind) -> Zone {
    match kind.system() {
        System::Recovery | System::Avionics => Zone::Forward,
        _ => Zone::Aft,
    }
}

fn holds_internals(part: &Part) -> bool {
    !matches!(part.kind, PartKind::NoseCone { .. })
}

/// The axial part a new internal or radial part goes into: the selected part (or the part a selected radial or internal part
/// sits on), else the forward-most or aft-most body tube (any non-nose part if there is no tube). Never a nose cone.
pub fn host_for(rocket: &Rocket, selection: Selection, zone: Zone) -> Option<PartId> {
    let chosen = selection
        .part()
        .and_then(|id| if rocket.part(id).is_some() { Some(id) } else { rocket.parent_of(id) })
        .filter(|id| rocket.part(*id).is_some_and(holds_internals));
    chosen.or_else(|| {
        let tubes: Vec<&Part> = rocket.parts().iter().filter(|p| matches!(p.kind, PartKind::BodyTube { .. })).collect();
        let pool: Vec<&Part> = if tubes.is_empty() { rocket.parts().iter().filter(|p| holds_internals(p)).collect() } else { tubes };
        match zone {
            Zone::Aft => pool.last(),
            Zone::Forward => pool.first(),
        }
        .map(|p| p.id)
    })
}

/// The bore of axial part `host` at its middle, metres.
pub fn host_inner(rocket: &Rocket, host: PartId) -> Option<f64> {
    let part = rocket.part(host)?;
    Some(part.inner_diameter_at(part.kind.length() / 2.0))
}

/// Where a click puts an internal part of `kind` in `host`: aft flush for tubes, rings and blocks (rings stack fore of each
/// other, around a motor mount if there is one), the fore end for bulkheads and couplers, and recovery gear and avionics stacked
/// aft of whatever recovery gear and avionics are already in the tube.
pub fn default_offset(rocket: &Rocket, host: PartId, kind: &InternalKind) -> f64 {
    let Some(part) = rocket.part(host) else {
        return 0.0;
    };
    let (len, ext) = (part.kind.length(), kind.extent());
    let aft = (len - ext).max(0.0);
    let mount = rocket.internals_of(host).filter(|p| p.kind.is_motor_mount()).max_by(|a, b| a.offset.total_cmp(&b.offset));
    let taken = |at: f64| {
        rocket
            .internals_of(host)
            .any(|p| std::mem::discriminant(&p.kind) == std::mem::discriminant(kind) && p.offset < at + ext - 1e-6 && at < p.offset + p.kind.extent() - 1e-6)
    };
    let first_free = |candidates: &[f64]| candidates.iter().copied().find(|&c| !taken(c));
    match kind {
        InternalKind::InnerTube { .. } => aft,
        InternalKind::EngineBlock { .. } => first_free(&[mount.map_or(aft, |m| m.offset), aft]).unwrap_or(aft),
        InternalKind::CenteringRing { .. } => {
            let around = mount.map(|m| vec![m.offset + m.kind.extent() - ext, m.offset]).unwrap_or_else(|| vec![aft]);
            first_free(&around).unwrap_or_else(|| {
                // Stack fore of the fore-most ring.
                let fore = rocket.internals_of(host).filter(|p| matches!(p.kind, InternalKind::CenteringRing { .. })).map(|p| p.offset).fold(aft, f64::min);
                (fore - ext).max(0.0)
            })
        }
        InternalKind::Bulkhead { .. } | InternalKind::Coupler { .. } => first_free(&[0.0, aft]).unwrap_or(0.0),
        _ => rocket.internals_of(host).filter(|p| matches!(zone_of(&p.kind), Zone::Forward)).map(|p| p.offset + p.kind.extent()).fold(0.0, f64::max),
    }
}

/// Adds internal part `kind` by a click: into the host for its zone, at its default offset, then selects it. Returns its id.
pub fn attach_internal(ctx: &mut Ctx, name: &str, kind: InternalKind) -> Option<PartId> {
    let rocket = ctx.doc.rocket();
    let host = host_for(rocket, ctx.selection, zone_of(&kind))?;
    let kind = fitted(&kind, host_inner(rocket, host)?);
    let offset = default_offset(rocket, host, &kind);
    let id = ctx.doc.edit(&format!("Add {}", name.to_lowercase()), |r| r.attach_internal(host, offset, kind).ok())?;
    ctx.selection = Selection::Part(id);
    Some(id)
}

/// Adds radial part `kind` by a click, with the bin's symmetry count: onto the selected part (or the aft-most body tube), fins
/// flush with its aft end and lugs and buttons at its middle, then selects it. Returns its id.
pub fn attach_radial(ctx: &mut Ctx, name: &str, kind: RadialKind) -> Option<PartId> {
    let rocket = ctx.doc.rocket();
    let host = host_for(rocket, ctx.selection, Zone::Aft).or_else(|| rocket.parts().last().map(|p| p.id))?;
    let len = rocket.part(host)?.kind.length();
    let offset = if kind.is_fin() { len - kind.extent() } else { (len - kind.extent()) / 2.0 };
    let count = ctx.drag.symmetry;
    let id = ctx.doc.edit(&format!("Add {}", name.to_lowercase()), |r| r.attach_radial(host, offset, 0.0, count, kind).ok())?;
    ctx.selection = Selection::Part(id);
    Some(id)
}

/// The attach node a click on an axial row uses: right after the selected part when it is valid, else the last valid node.
pub fn click_node(rocket: &Rocket, selection: Selection, kind: &PartKind) -> Option<usize> {
    let nodes = rocket.nodes_for(kind);
    let after = selection.part().and_then(|id| rocket.index_of(id).or_else(|| rocket.parent_of(id).and_then(|p| rocket.index_of(p)))).map(|i| i + 1);
    after.filter(|n| nodes.iter().any(|(i, _)| i == n)).or_else(|| nodes.last().map(|(i, _)| *i))
}

/// Adds axial part `kind` by a click and selects it. Returns its id.
pub fn attach_axial(ctx: &mut Ctx, kind: PartKind) -> Option<PartId> {
    let node = click_node(ctx.doc.rocket(), ctx.selection, &kind)?;
    let before = ctx.doc.rocket().ids();
    if !drag::place(ctx, node, kind) {
        return None;
    }
    let id = ctx.doc.rocket().ids().into_iter().find(|id| !before.contains(id))?;
    ctx.selection = Selection::Part(id);
    Some(id)
}

/// The mount motors go in: the selected motor mount (or the one in the selected part), else the first one.
pub fn motor_mount(rocket: &Rocket, selection: Selection) -> Option<MotorMount> {
    let mounts = rocket.motor_mounts();
    let chosen = selection.part().and_then(|id| mounts.iter().find(|m| m.id == id || m.parent == id));
    chosen.or_else(|| mounts.first()).copied()
}

/// Indices into `motors` of the motors that fit `mount`, smallest total impulse first.
pub fn fitting_motors(motors: &bowshock_motors::MotorDb, mount: &MotorMount) -> Vec<usize> {
    let max_length = mount.length + mount.overhang;
    let mut found: Vec<usize> = (0..motors.motors.len())
        .filter(|&i| {
            let m = &motors.motors[i];
            (m.diameter - mount.inner_diameter).abs() <= 0.0005 + 1e-9 && m.length <= max_length + 1e-9
        })
        .collect();
    found.sort_by(|&a, &b| motors.motors[a].total_impulse().total_cmp(&motors.motors[b].total_impulse()));
    found
}

/// Puts motor `index` of the database in `mount` in the active configuration (first available delay) as one undo step.
pub fn set_motor(ctx: &mut Ctx, mount: PartId, index: usize) -> bool {
    let Some(motor) = ctx.physics.motors().motors.get(index) else {
        return false;
    };
    let (reference, delay) =
        (bowshock_model::MotorRef::new(motor.designation.clone(), motor.manufacturer.clone()), motor.delays.first().copied().unwrap_or(0.0));
    let name = motor.designation.clone();
    let config = ctx.doc.rocket().active_config_id();
    let ok =
        ctx.doc.edit(&format!("Set motor {name}"), |r| r.set_motor(config, mount, Some(reference)).is_ok() && r.set_motor_delay(config, mount, delay).is_ok());
    if ok {
        ctx.notify(format!("{name} in the active configuration"));
    }
    ok
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rocket() -> (Rocket, PartId, PartId) {
        let mut r = Rocket::default();
        let nose = r.insert(0, sized(&axial_catalogue()[0].1, None)).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: 0.0762 }).unwrap();
        let _ = nose;
        (r, tube, tube)
    }

    #[test]
    fn internals_go_in_the_aft_tube_recovery_forward_and_never_in_a_nose() {
        let (mut r, tube, _) = rocket();
        let aft = r.insert(2, PartKind::BodyTube { length: 0.4, diameter: 0.0762 }).unwrap();
        let mount = &structure_catalogue()[0].1;
        let chute = &recovery_catalogue()[0].1;
        assert_eq!(host_for(&r, Selection::None, zone_of(mount)), Some(aft));
        assert_eq!(host_for(&r, Selection::None, zone_of(chute)), Some(tube));
        assert_eq!(host_for(&r, Selection::Part(tube), zone_of(mount)), Some(tube));
        // A nose cone is never the host: the fallback is used.
        let nose = r.parts()[0].id;
        assert_eq!(host_for(&r, Selection::Part(nose), zone_of(mount)), Some(aft));
        // Only a nose cone: nowhere to go.
        let mut only = Rocket::default();
        only.insert(0, axial_catalogue()[0].1.clone()).unwrap();
        assert_eq!(host_for(&only, Selection::None, Zone::Aft), None);
    }

    #[test]
    fn default_offsets_are_aft_flush_stacked_rings_and_forward_stacked_recovery() {
        let (mut r, tube, _) = rocket();
        let mount = structure_catalogue()[0].1.clone();
        let ring = structure_catalogue()[2].1.clone();
        let block = structure_catalogue()[4].1.clone();
        assert!((default_offset(&r, tube, &mount) - 0.4).abs() < 1e-9);
        r.attach_internal(tube, 0.4, mount).unwrap();
        // Rings sit at the mount's aft end, then its fore end, then stack fore of that.
        let aft_ring = default_offset(&r, tube, &ring);
        assert!((aft_ring - (0.6 - 0.00635)).abs() < 1e-9);
        r.attach_internal(tube, aft_ring, ring.clone()).unwrap();
        assert!((default_offset(&r, tube, &ring) - 0.4).abs() < 1e-9);
        // An engine block sits at the mount's fore end.
        assert!((default_offset(&r, tube, &block) - 0.4).abs() < 1e-9);
        // Recovery gear stacks from the fore end.
        let chute = recovery_catalogue()[0].1.clone();
        let cord = recovery_catalogue()[3].1.clone();
        assert_eq!(default_offset(&r, tube, &chute), 0.0);
        r.attach_internal(tube, 0.0, chute).unwrap();
        assert!((default_offset(&r, tube, &cord) - 0.12).abs() < 1e-9);
    }

    #[test]
    fn click_node_follows_the_selection() {
        let (r, tube, _) = rocket();
        let kind = PartKind::BodyTube { length: 0.3, diameter: 0.0762 };
        // Nothing selected: the last valid node; the nose selected: right behind it.
        assert_eq!(click_node(&r, Selection::None, &kind), Some(2));
        assert_eq!(click_node(&r, Selection::Part(r.parts()[0].id), &kind), Some(1));
        assert_eq!(click_node(&r, Selection::Part(tube), &kind), Some(2));
    }

    #[test]
    fn a_mount_too_wide_for_its_tube_is_narrowed() {
        let mount = structure_catalogue()[0].1.clone();
        let InternalKind::InnerTube { outer_diameter, .. } = fitted(&mount, 0.03) else { panic!("an inner tube") };
        assert!((outer_diameter - 0.024).abs() < 1e-9);
        assert_eq!(fitted(&mount, 0.0762), mount);
    }

    #[test]
    fn motors_fit_the_mount_bore() {
        let db = bowshock_motors::MotorDb::bundled();
        let (mut r, tube, _) = rocket();
        r.attach_internal(tube, 0.3, structure_catalogue()[0].1.clone()).unwrap();
        let mount = motor_mount(&r, Selection::None).unwrap();
        let found = fitting_motors(&db, &mount);
        assert!(found.iter().all(|&i| (db.motors[i].diameter - 0.0381).abs() < 0.001));
        assert!(found.windows(2).all(|w| db.motors[w[0]].total_impulse() <= db.motors[w[1]].total_impulse()));
    }

    #[test]
    fn the_airframe_list_has_the_coupler_before_the_boattail() {
        let items = airframe_items();
        let names: Vec<&str> = items
            .iter()
            .map(|i| match i {
                Item::Axial(n, _) | Item::Internal(n, _) => *n,
                _ => "",
            })
            .collect();
        let coupler = names.iter().position(|n| *n == "Tube coupler").unwrap();
        assert_eq!(names[coupler + 1], "Boattail");
        assert_eq!(names.last(), Some(&"Payload bay"));
    }
}
