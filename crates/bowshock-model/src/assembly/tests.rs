use std::collections::BTreeSet;
use super::*;
use crate::config::MotorRef;
use crate::expr::{Expr, FieldRef, Op};
use crate::geometry::{NoseShape, PartKind, RadialKind, RadialPart};
use crate::handles::Dim;
use crate::internal::{DeployEvent, Deployment, InternalKind, MassRole};
use crate::material::MaterialId;
use crate::{FormatError, Rocket};

const D: f64 = 0.0762;

fn tube(length: f64, diameter: f64) -> PartKind {
    PartKind::BodyTube { length, diameter }
}

fn fin() -> RadialKind {
    RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: 0.05, thickness: 0.003 }
}

fn mount() -> InternalKind {
    InternalKind::InnerTube { outer_diameter: 0.054, thickness: 0.0016, length: 0.2, motor_mount: true, overhang: 0.0 }
}

/// A body tube of its own diameter at `index` (an ordinary insert would match the neighbour's).
fn add_tube(r: &mut Rocket, index: usize, length: f64, diameter: f64) -> PartId {
    let id = r.insert(index, tube(length, diameter)).unwrap();
    assert!(r.set_kind(id, tube(length, diameter)));
    id
}

fn altimeter(diameter: Option<f64>) -> InternalKind {
    InternalKind::MassComponent { mass: 0.05, length: 0.08, diameter, role: MassRole::Altimeter }
}

fn ring(outer: Option<f64>) -> InternalKind {
    InternalKind::CenteringRing { outer_diameter: outer, inner_diameter: None, thickness: 0.006 }
}

/// Everything a test needs to find again.
struct Rig {
    r: Rocket,
    nose: PartId,
    upper: PartId,
    bay: PartId,
    booster: PartId,
    /// In `bay`: bulkhead (auto), avionics (auto), wall ring (given diameter = the bay's bore), parachute (given packed diameter).
    bulkhead: PartId,
    avionics: PartId,
    wall_ring: PartId,
    chute: PartId,
    lug: PartId,
    fins: PartId,
    mount: PartId,
}

fn rig() -> Rig {
    let mut r = Rocket::default();
    let nose = r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: D, tip_radius: 0.0 }).unwrap();
    let upper = r.insert(1, tube(0.5, D)).unwrap();
    let bay = r.insert(2, tube(0.3, D)).unwrap();
    let booster = r.insert(3, tube(0.6, D)).unwrap();
    let bore = r.inner_diameter_at(bay, 0.1).unwrap();
    let bulkhead = r.attach_internal(bay, 0.0, InternalKind::Bulkhead { diameter: None, thickness: 0.006 }).unwrap();
    let avionics = r.attach_internal(bay, 0.05, altimeter(None)).unwrap();
    let wall_ring = r.attach_internal(bay, 0.2, ring(Some(bore))).unwrap();
    let chute = r
        .attach_internal(
            bay,
            0.14,
            InternalKind::Parachute {
                diameter: 0.5,
                cd: 0.8,
                line_count: 6,
                line_length: 0.5,
                line_material: MaterialId::KEVLAR_LINE,
                packed_length: 0.05,
                packed_diameter: Some(0.05),
                deploy: Deployment { event: DeployEvent::Ejection, delay: 0.0 },
            },
        )
        .unwrap();
    let lug = r.attach_radial(bay, 0.05, 1.0, 1, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap();
    let fins = r.attach_radial(booster, 0.45, 0.3, 3, fin()).unwrap();
    let mount = r.attach_internal(booster, 0.3, mount()).unwrap();
    r.set_name(avionics, Some("Altimeter".to_string()));
    Rig { r, nose, upper, bay, booster, bulkhead, avionics, wall_ring, chute, lug, fins, mount }
}

/// The document without its assemblies: what must not change when one is made.
fn flat(r: &Rocket) -> Rocket {
    Rocket { assemblies: Vec::new(), instances: Vec::new(), next_id: 0, ..r.clone() }
}

fn bits(r: &Rocket) -> (Vec<u64>, u64, u64) {
    let stations = r.stations().iter().map(|s| s.to_bits()).collect();
    let dry = r.dry_mass();
    (stations, dry.mass.to_bits(), dry.cg.to_bits())
}

// ---- group ----------------------------------------------------------------------------------------------------------------

#[test]
fn grouping_each_kind_leaves_geometry_mass_and_stations_bit_identical() {
    let rig = rig();
    // A stack: the bay and what it carries (selecting the tube alone closes over its lug and internals).
    let mut r = rig.r.clone();
    let before = r.clone();
    let id = r.group(&[rig.bay], "Avionics bay").unwrap();
    assert_eq!(flat(&r), flat(&before));
    assert_eq!(bits(&r), bits(&before));
    let mut members = r.members(id);
    members.sort_unstable();
    let mut expect = vec![rig.bay, rig.lug, rig.bulkhead, rig.avionics, rig.wall_ring, rig.chute];
    expect.sort_unstable();
    assert_eq!(members, expect);
    assert!(r.is_member(rig.chute) && !r.is_member(rig.booster) && !r.is_member(rig.fins));
    assert_eq!(r.assemblies().len(), 1);
    let def = &r.assemblies()[0];
    assert_eq!((def.version, def.name.as_str(), def.kind()), (1, "Avionics bay", FragmentKind::Stack));
    assert_eq!(def.identity.as_str().len(), 32);
    // Local ids are the canonical order from 0: the tube, its lug, its internals.
    assert_eq!(def.fragment.ids(), (0..6).collect::<Vec<_>>());
    assert_eq!(def.fragment.next_id(), 6);
    assert_eq!(r.instance(id).unwrap().members.len(), 6);
    assert!(def.fragment.check().is_ok());

    // Internal parts of one tube.
    let mut r = rig.r.clone();
    let before = r.clone();
    let id = r.group(&[rig.avionics, rig.chute, rig.wall_ring], "Payload").unwrap();
    assert_eq!(flat(&r), flat(&before));
    assert_eq!(bits(&r), bits(&before));
    let def = &r.assemblies()[0];
    assert_eq!(def.kind(), FragmentKind::Internal);
    assert!(def.fragment.internals().iter().all(|i| i.parent == Fragment::HOST));
    assert_eq!(def.fragment.origin(), 0.05);
    let host = def.fragment.host().unwrap();
    assert!((host.length - 0.3).abs() < 1e-12 && (host.outer_diameter - D).abs() < 1e-12);
    assert_eq!(r.members(id).len(), 3);
    assert!(!r.is_member(rig.bulkhead));

    // Radial parts on one body.
    let mut r = rig.r.clone();
    let before = r.clone();
    r.group(&[rig.fins], "Fin can").unwrap();
    assert_eq!(flat(&r), flat(&before));
    assert_eq!(bits(&r), bits(&before));
    assert_eq!(r.assemblies()[0].kind(), FragmentKind::Surface);
    assert_eq!(r.assemblies()[0].fragment.radials()[0].parent, Fragment::HOST);
}

#[test]
fn a_run_of_airframe_parts_is_grouped_with_everything_they_carry() {
    let rig = rig();
    let mut r = rig.r.clone();
    let id = r.group(&[rig.bay, rig.booster], "Lower stage").unwrap();
    let members: BTreeSet<PartId> = r.members(id).into_iter().collect();
    for p in [rig.bay, rig.booster, rig.lug, rig.fins, rig.mount, rig.bulkhead, rig.chute] {
        assert!(members.contains(&p), "{p}");
    }
    assert!(!members.contains(&rig.upper) && !members.contains(&rig.nose));
    assert_eq!(r.placement(id), Some(Placement::Stack { pod: None, index: 2 }));
}

#[test]
fn what_cannot_be_an_assembly_is_refused_with_the_reason() {
    let rig = rig();
    let mut r = rig.r.clone();
    let before = r.clone();
    // Airframe parts that are not one run.
    assert_eq!(r.group(&[rig.upper, rig.booster], "x"), Err(AssemblyError::NotContiguous { gap: rig.bay }));
    // Internal parts in different hosts, no airframe part chosen.
    assert_eq!(r.group(&[rig.avionics, rig.mount], "x"), Err(AssemblyError::DifferentHosts));
    // Radial and internal parts together with no airframe part.
    assert_eq!(r.group(&[rig.lug, rig.avionics], "x"), Err(AssemblyError::MixedClasses));
    // An internal part chosen with airframe parts that do not hold it.
    assert_eq!(r.group(&[rig.upper, rig.avionics], "x"), Err(AssemblyError::NotCarried(rig.avionics)));
    assert_eq!(r.group(&[], "x"), Err(AssemblyError::Empty));
    assert_eq!(r.group(&[999], "x"), Err(AssemblyError::NoSuchPart(999)));
    assert_eq!(r, before, "a refusal changes nothing");
    for err in [AssemblyError::NotContiguous { gap: 3 }, AssemblyError::DifferentHosts, AssemblyError::MixedClasses] {
        assert!(!err.to_string().is_empty());
    }

    // A part belongs to one instance only; assemblies do not nest.
    let first = r.group(&[rig.avionics], "Altimeter").unwrap();
    assert_eq!(r.group(&[rig.avionics, rig.chute], "x"), Err(AssemblyError::AlreadyMember { part: rig.avionics, instance: first }));
    assert!(matches!(r.group(&[rig.bay], "x"), Err(AssemblyError::AlreadyMember { instance, .. }) if instance == first));
}

// ---- place, move, update --------------------------------------------------------------------------------------------------

/// The bay as an assembly, placed twice more: `(rocket, identity, [first, second, third] instances)`.
fn bay_thrice() -> (Rig, AssemblyId, [InstanceId; 3]) {
    let mut rig = rig();
    let first = rig.r.group(&[rig.bay], "Avionics bay").unwrap();
    let identity = rig.r.assemblies()[0].identity.clone();
    let end = rig.r.parts().len();
    let second = rig.r.place(&identity, Placement::Stack { pod: None, index: end }).unwrap();
    let third = rig.r.place(&identity, Placement::Stack { pod: None, index: 2 }).unwrap();
    (rig, identity, [first, second.instance, third.instance])
}

#[test]
fn a_stack_assembly_is_placed_with_its_own_geometry_and_new_ids() {
    let (rig, identity, [first, second, third]) = bay_thrice();
    let r = &rig.r;
    // 4 original parts + 2 bays; the third went to index 2, ahead of the first (which moved back to 3).
    assert_eq!(r.parts().len(), 6);
    assert_eq!(r.placement(first), Some(Placement::Stack { pod: None, index: 3 }));
    assert_eq!(r.placement(third), Some(Placement::Stack { pod: None, index: 2 }));
    assert_eq!(r.placement(second), Some(Placement::Stack { pod: None, index: 5 }));
    // New ids for every member; each instance has all six members and the parts are ordinary.
    let all: BTreeSet<PartId> = [first, second, third].into_iter().flat_map(|i| r.members(i)).collect();
    assert_eq!(all.len(), 18);
    for &i in &[second, third] {
        let members = r.members(i);
        assert_eq!(members.len(), 6);
        let tube = members.iter().copied().find(|&m| r.part(m).is_some()).unwrap();
        assert_eq!(r.internals_of(tube).count(), 4);
        assert_eq!(r.radials().iter().filter(|g| g.parent == tube).count(), 1);
        // The copy equals the original member for member (names and values kept).
        let altimeter = members.iter().copied().find(|&m| r.internal(m).is_some_and(|p| p.name.as_deref() == Some("Altimeter"))).unwrap();
        assert_eq!(r.internal(altimeter).unwrap().parent, tube);
    }
    assert_eq!(r.instances_of(&identity).count(), 3);
    // Stack rules: a bay is not a nose, so index 0 under a nose is refused, as is an index past the end.
    let mut work = rig.r.clone();
    assert_eq!(work.place(&identity, Placement::Stack { pod: None, index: 0 }), Err(AssemblyError::Insert(InsertError::SecondNose)));
    assert_eq!(work.place(&identity, Placement::Stack { pod: None, index: 99 }), Err(AssemblyError::Insert(InsertError::IndexOutOfRange)));
    assert_eq!(work.place(&identity, Placement::Internal { host: rig.upper, offset: 0.0 }), Err(AssemblyError::WrongTarget { expected: FragmentKind::Stack }));
    assert_eq!(work, rig.r, "a refused placement changes nothing");
    assert_eq!(rig.r.assembly_nodes(&identity, None, None).iter().map(|n| n.0).collect::<Vec<_>>(), (1..=6).collect::<Vec<_>>());
    assert_eq!(rig.r.assembly_nodes(&identity, None, Some(third)).iter().map(|n| n.0).collect::<Vec<_>>(), (1..=5).collect::<Vec<_>>());
}

#[test]
fn updating_an_assembly_changes_every_instance_and_nothing_else() {
    let (rig, identity, [first, second, third]) = bay_thrice();
    let mut r = rig.r.clone();
    // A motor on the booster's mount must survive (the mount is not part of the bay).
    let config = r.active_config_id();
    r.set_motor(config, rig.mount, Some(MotorRef::new("H128W", "Aerotech"))).unwrap();
    let before = r.clone();
    let placements: Vec<Option<Placement>> = [first, second, third].iter().map(|&i| r.placement(i)).collect();
    let old_members: Vec<Vec<PartId>> = [first, second, third].iter().map(|&i| r.members(i)).collect();

    // Edit the definition the way the editor does: open it as a document, change it, read it back.
    let def = r.assembly(&identity).unwrap().clone();
    let mut editor = def.to_rocket();
    let tube_id = editor.parts()[0].id;
    assert!(editor.set_dim(tube_id, Dim::Length, 0.4).is_some());
    let removed_local = editor.internals().iter().find(|p| matches!(p.kind, InternalKind::Bulkhead { .. })).unwrap().id;
    editor.remove_internal(removed_local).unwrap();
    let added_local = editor.attach_internal(tube_id, 0.3, altimeter(None)).unwrap();
    assert!(added_local >= def.fragment.next_id(), "the editor never reuses a local id");
    let newer = AssemblyDef::from_rocket(identity.clone(), def.version + 1, "Avionics bay", &editor).unwrap();

    // Not newer: refused.
    assert_eq!(r.update_assembly(def.clone()), Err(AssemblyError::NotNewer { stored: 1, offered: 1 }));
    let report = r.update_assembly(newer.clone()).unwrap();
    assert_eq!(report.len(), 3);

    for (k, &i) in [first, second, third].iter().enumerate() {
        // The placement is kept, every instance has the new tube length and the new content.
        assert_eq!(r.placement(i), placements[k], "instance {k}");
        assert_eq!(r.instance(i).unwrap().version, 2);
        let members = r.members(i);
        let tube = members.iter().copied().find(|&m| r.part(m).is_some()).unwrap();
        assert_eq!(r.part(tube).unwrap().kind.length(), 0.4);
        assert_eq!(r.internals_of(tube).count(), 4, "one bulkhead gone, one altimeter new");
        assert!(!r.internals_of(tube).any(|p| matches!(p.kind, InternalKind::Bulkhead { .. })));
        // Surviving members keep their ids; the removed one is gone; the new one has a new id.
        let update = report.iter().find(|u| u.instance == i).unwrap();
        assert_eq!(update.removed.len(), 1);
        assert_eq!(update.added.len(), 1);
        assert_eq!(update.kept.len(), 5);
        assert!(update.kept.iter().all(|id| old_members[k].contains(id)));
        assert!(old_members[k].contains(&update.removed[0]) && !members.contains(&update.removed[0]));
        assert!(!old_members[k].contains(&update.added[0]) && members.contains(&update.added[0]));
        assert!(r.kind_of(update.removed[0]).is_none());
    }
    // The original bay's tube kept its id (it is the instance made in place).
    assert_eq!(r.part(rig.bay).unwrap().kind.length(), 0.4);
    assert!(r.internal(rig.avionics).is_some() && r.internal(rig.bulkhead).is_none());
    assert_eq!(r.assembly(&identity), Some(&newer));

    // Nothing else changed: the other parts, the motor, the rest of the document.
    for id in [rig.nose, rig.upper, rig.booster] {
        assert_eq!(r.part(id), before.part(id));
    }
    assert_eq!(r.radial(rig.fins), before.radial(rig.fins));
    assert_eq!(r.internal(rig.mount), before.internal(rig.mount));
    assert_eq!(r.configurations(), before.configurations());
    assert!(r.configuration(config).unwrap().slot(rig.mount).unwrap().motor.is_some());
    assert_eq!(r.parts().len(), before.parts().len());
    assert_eq!(r.flights(), before.flights());
    // Lengthening three bays by 10 cm each lengthened the rocket by 30 cm.
    assert!((r.length() - before.length() - 0.3).abs() < 1e-12);
    // One definition, the same three instances, ids still unique.
    assert_eq!(r.instances().len(), 3);
    let ids = r.ids();
    assert_eq!(ids.len(), ids.iter().copied().collect::<BTreeSet<_>>().len());
    let saved = Rocket::from_bytes(&r.to_bytes()).unwrap();
    assert_eq!(saved, r);

    // Update does not change a definition's kind: a surface assembly cannot become a stack.
    let mut surface = rig.r.clone();
    surface.group(&[rig.fins], "Fin can").unwrap();
    let mut stack_def = r.assembly(&identity).unwrap().clone();
    stack_def.identity = surface.assemblies().last().unwrap().identity.clone();
    stack_def.version = 2;
    let err = surface.update_assembly(stack_def).unwrap_err();
    assert_eq!(err, AssemblyError::KindChanged { from: FragmentKind::Surface, to: FragmentKind::Stack });
}

#[test]
fn updates_that_do_not_fit_leave_the_document_alone() {
    let (rig, identity, _) = bay_thrice();
    let mut r = rig.r.clone();
    let mut editor = r.assembly(&identity).unwrap().to_rocket();
    // A nose cone in the middle of the stack is refused by the stack rules.
    let tube_id = editor.parts()[0].id;
    editor.remove(tube_id);
    let nose = editor.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.1, diameter: D, tip_radius: 0.0 }).unwrap();
    let def = AssemblyDef::from_rocket(identity.clone(), 2, "Bay", &editor).unwrap();
    assert!(editor.part(nose).is_some());
    let before = r.clone();
    assert!(matches!(r.update_assembly(def), Err(AssemblyError::Insert(_))));
    assert_eq!(r, before);
}

#[test]
fn an_internal_assembly_follows_the_walls_of_a_wider_tube_and_reports_what_does_not_fit() {
    let mut rig = rig();
    rig.r.group(&[rig.avionics, rig.chute, rig.wall_ring, rig.bulkhead], "Payload").unwrap();
    let identity = rig.r.assemblies()[0].identity.clone();

    // A wider host: 98 mm.
    let wide = add_tube(&mut rig.r, 4, 0.4, 0.098);
    let bore = rig.r.inner_diameter_at(wide, 0.1).unwrap();
    let placed = rig.r.place(&identity, Placement::Internal { host: wide, offset: 0.1 }).unwrap();
    let members = rig.r.members(placed.instance);
    assert_eq!(members.len(), 4);
    let by_kind = |f: &dyn Fn(&InternalKind) -> bool| members.iter().copied().find(|&m| rig.r.internal(m).is_some_and(|p| f(&p.kind))).unwrap();
    let ring_id = by_kind(&|k| matches!(k, InternalKind::CenteringRing { .. }));
    let chute_id = by_kind(&|k| matches!(k, InternalKind::Parachute { .. }));
    let bulkhead_id = by_kind(&|k| matches!(k, InternalKind::Bulkhead { .. }));
    let avionics_id = by_kind(&|k| matches!(k, InternalKind::MassComponent { .. }));
    // The ring touched the old wall: it was resized to the new bore. The auto bulkhead resolves to it by itself.
    assert_eq!(placed.fit, Fit { resized: vec![ring_id], unfit: vec![] });
    let InternalKind::CenteringRing { outer_diameter: Some(d), .. } = rig.r.internal(ring_id).unwrap().kind else {
        panic!("ring lost its diameter");
    };
    assert!((d - bore).abs() < 1e-12, "{d} vs {bore}");
    assert_eq!(rig.r.resolved(bulkhead_id).unwrap().outer_diameter, bore);
    assert!(rig.r.resolved(bulkhead_id).unwrap().outer_auto);
    // The chute kept its own size (50 mm) and fits.
    assert_eq!(rig.r.internal(chute_id).unwrap().kind.given_outer_diameter(), Some(0.05));
    // Relative positions are kept: the avionics sat 5 cm before the ring group's ... the whole fragment moved by +0.1 - 0.0.
    let origin = rig.r.internal(bulkhead_id).unwrap().offset;
    assert!((origin - 0.1).abs() < 1e-12, "the bulkhead (fore-most member) is at the station asked for: {origin}");
    assert!((rig.r.internal(avionics_id).unwrap().offset - (0.1 + 0.05)).abs() < 1e-12);
    assert!((rig.r.internal(ring_id).unwrap().offset - (0.1 + 0.2)).abs() < 1e-12);

    // A narrower host: 54 mm. The ring that touched the wall follows it down, the 50 mm chute still fits, a 60 mm part does not.
    let mut rig2 = rig_with_wide_member();
    let narrow = add_tube(&mut rig2.r, 4, 0.4, 0.054);
    let identity = rig2.r.assemblies()[0].identity.clone();
    let placed = rig2.r.place(&identity, Placement::Internal { host: narrow, offset: 0.0 }).unwrap();
    let wide_member =
        rig2.r.members(placed.instance).into_iter().find(|&m| rig2.r.internal(m).is_some_and(|p| p.kind.given_outer_diameter() == Some(0.06))).unwrap();
    assert!(placed.fit.unfit.contains(&wide_member));
    assert!(placed.fit.resized.len() == 1);
    assert!(rig2.r.internal(wide_member).is_some(), "placed anyway, with its own size");

    // A host that is too short for the fragment is refused outright.
    let short = rig2.r.insert(5, tube(0.1, D)).unwrap();
    assert!(matches!(rig2.r.place(&identity, Placement::Internal { host: short, offset: 0.0 }), Err(AssemblyError::TooLong { .. })));
    assert_eq!(rig2.r.place(&identity, Placement::Internal { host: 999, offset: 0.0 }), Err(AssemblyError::NoSuchHost(999)));
}

/// An internal assembly with a wall-touching ring and a 60 mm tube that fits only a wide host.
fn rig_with_wide_member() -> Rig {
    let mut rig = rig();
    let wide = rig
        .r
        .attach_internal(rig.bay, 0.1, InternalKind::InnerTube { outer_diameter: 0.06, thickness: 0.001, length: 0.05, motor_mount: false, overhang: 0.0 })
        .unwrap();
    rig.r.group(&[rig.wall_ring, wide, rig.avionics], "Core").unwrap();
    rig
}

#[test]
fn internal_assemblies_move_between_tubes_with_the_same_fit_rules() {
    let mut rig = rig();
    let id = rig.r.group(&[rig.avionics, rig.chute, rig.wall_ring], "Payload").unwrap();
    let before_offsets: Vec<f64> = rig.r.members(id).iter().map(|&m| rig.r.internal(m).unwrap().offset).collect();
    // Slide along the bay: all members move by the same amount.
    let fit = rig.r.move_instance(id, Placement::Internal { host: rig.bay, offset: 0.0 }).unwrap();
    assert_eq!(fit, Fit::default());
    let after: Vec<f64> = rig.r.members(id).iter().map(|&m| rig.r.internal(m).unwrap().offset).collect();
    for (a, b) in after.iter().zip(&before_offsets) {
        assert!((a - (b - 0.05)).abs() < 1e-12);
    }
    // To another tube (narrower): the wall ring follows; ids kept.
    let ids_before = rig.r.members(id);
    let narrow = add_tube(&mut rig.r, 4, 0.5, 0.054);
    let fit = rig.r.move_instance(id, Placement::Internal { host: narrow, offset: 0.1 }).unwrap();
    assert_eq!(fit.resized, vec![rig.wall_ring]);
    assert_eq!(rig.r.members(id), ids_before);
    assert_eq!(rig.r.internal(rig.avionics).unwrap().parent, narrow);
    // Not onto a host that is not an airframe part, nor with the wrong kind of target.
    assert_eq!(rig.r.move_instance(id, Placement::Internal { host: 999, offset: 0.0 }), Err(AssemblyError::NoSuchHost(999)));
    assert!(matches!(rig.r.move_instance(id, Placement::Surface { host: narrow, offset: 0.0, angle: 0.0 }), Err(AssemblyError::WrongTarget { .. })));
    assert_eq!(rig.r.placement(id), Some(Placement::Internal { host: narrow, offset: 0.1 }));
}

#[test]
fn a_stack_assembly_moves_as_one_keeping_what_it_carries() {
    let (rig, _, [first, second, third]) = bay_thrice();
    let mut r = rig.r.clone();
    let before_members: Vec<Vec<PartId>> = [first, second, third].iter().map(|&i| r.members(i)).collect();
    let motor_config = r.active_config_id();
    r.set_motor(motor_config, rig.mount, Some(MotorRef::new("H128W", "Aerotech"))).unwrap();
    // Move the third (index 2) to the end: counting the stack without it.
    let without = r.parts().len() - 1;
    r.move_instance(third, Placement::Stack { pod: None, index: without }).unwrap();
    assert_eq!(r.placement(third), Some(Placement::Stack { pod: None, index: without }));
    assert_eq!(r.members(third), before_members[2], "same ids");
    assert_eq!(r.parts().last().unwrap().id, before_members[2].iter().copied().find(|&m| r.part(m).is_some()).unwrap());
    // What it carries went with it.
    let tube = r.parts().last().unwrap().id;
    assert_eq!(r.internals_of(tube).count(), 4);
    assert!(r.configuration(motor_config).unwrap().slot(rig.mount).is_some(), "motors stay");
    // The rules of the stack still apply: not above the nose.
    let before = r.clone();
    assert_eq!(r.move_instance(first, Placement::Stack { pod: None, index: 0 }), Err(AssemblyError::Insert(InsertError::SecondNose)));
    assert_eq!(r, before);
    // Every id is still unique, and the document survives a save.
    assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap(), r);
}

#[test]
fn surface_assemblies_place_and_move_with_their_relative_angles_and_offsets() {
    let mut rig = rig();
    // A fin can: the fin group and a lug on the booster (lug first, so the first radial part sets the angle).
    let lug = rig.r.attach_radial(rig.booster, 0.2, 1.0, 1, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap();
    let id = rig.r.group(&[lug, rig.fins], "Aft kit").unwrap();
    let identity = rig.r.assemblies()[0].identity.clone();
    let def = rig.r.assemblies()[0].clone();
    assert_eq!(def.fragment.radials().len(), 2);
    assert_eq!(def.fragment.origin(), 0.2);
    // Place on the bay at 0.05 with the first part at 2.0 rad.
    let placed = rig.r.place(&identity, Placement::Surface { host: rig.bay, offset: 0.05, angle: 2.0 }).unwrap();
    let members = rig.r.members(placed.instance);
    let on_bay: Vec<&RadialPart> = rig.r.radials().iter().filter(|g| members.contains(&g.id)).collect();
    assert_eq!(on_bay.len(), 2);
    let lug2 = on_bay.iter().find(|g| matches!(g.kind, RadialKind::LaunchLug { .. })).unwrap();
    let fins2 = on_bay.iter().find(|g| g.kind.is_fin()).unwrap();
    // The fin group (a quarter of its 0.12 chord must stay on the 0.3 m bay) cannot reach past it: the origin was limited to 0.02
    // so every member stays on the host, and the members keep their distance from it.
    // The first radial part of the fragment in document order (the fin group; the lug was made later) sets the angle.
    assert!((lug2.offset - 0.02).abs() < 1e-9 && (fins2.angle - 2.0).abs() < 1e-12, "{}", lug2.offset);
    assert!((fins2.offset - (0.02 + 0.25)).abs() < 1e-9, "{}", fins2.offset);
    assert!(fins2.offset + fins2.kind.min_attached() <= 0.3 + 1e-12);
    // The fins' angle relative to the lug (0.3 vs 1.0 rad) is kept.
    let rel = (fins2.angle - lug2.angle).rem_euclid(std::f64::consts::TAU);
    let original = (rig.r.radial(rig.fins).unwrap().angle - rig.r.radial(lug).unwrap().angle).rem_euclid(std::f64::consts::TAU);
    assert!((rel - original).abs() < 1e-9);

    // Move the original instance one tube forward: same rules as move_radial.
    rig.r.move_instance(id, Placement::Surface { host: rig.upper, offset: 0.0, angle: 0.5 }).unwrap();
    assert_eq!(rig.r.radial(lug).unwrap().parent, rig.upper);
    assert_eq!(rig.r.radial(rig.fins).unwrap().parent, rig.upper);
    assert!((rig.r.radial(rig.fins).unwrap().offset - 0.25).abs() < 1e-12);
    assert!((rig.r.radial(rig.fins).unwrap().angle - 0.5).abs() < 1e-12);
    assert!((rig.r.radial(lug).unwrap().angle - (0.5 + 0.7)).abs() < 1e-12, "the lug keeps its angle relative to the fins");
    assert!(matches!(rig.r.move_instance(id, Placement::Surface { host: 999, offset: 0.0, angle: 0.0 }), Err(AssemblyError::NoSuchHost(999))));
    // A host too short for the fragment (0.25 m from the first part's origin to the fin's aft quarter) is refused.
    let tiny = rig.r.insert(4, tube(0.1, D)).unwrap();
    assert!(matches!(rig.r.place(&identity, Placement::Surface { host: tiny, offset: 0.0, angle: 0.0 }), Err(AssemblyError::TooLong { .. })));
}

#[test]
fn a_pod_set_is_a_surface_assembly_with_its_body_and_pods_do_not_nest() {
    let mut rig = rig();
    let pod = rig.r.attach_radial(rig.booster, 0.1, 0.0, 2, rig.r.default_pod()).unwrap();
    let body: Vec<PartId> = rig.r.radial(pod).unwrap().kind.pod_body().unwrap().ids().collect();
    let tube_in_pod = rig.r.pod_of(body[0]).map(|p| p.id);
    assert_eq!(tube_in_pod, Some(pod));
    let before = rig.r.clone();
    // Choosing a part inside the pod with its pod set chosen too: the set comes with its body.
    let id = rig.r.group(&[pod, body[0]], "Strap-ons").unwrap();
    assert_eq!(flat(&rig.r), flat(&before));
    assert_eq!(bits(&rig.r), bits(&before));
    let members: BTreeSet<PartId> = rig.r.members(id).into_iter().collect();
    assert!(body.iter().all(|b| members.contains(b)) && members.contains(&pod));
    assert!(rig.r.is_member(body[0]), "the parts inside a member pod are members");
    // Place a second set on the upper tube: new ids for the pod and everything in it.
    let identity = rig.r.assemblies()[0].identity.clone();
    let placed = rig.r.place(&identity, Placement::Surface { host: rig.upper, offset: 0.0, angle: 0.0 }).unwrap();
    assert_eq!(rig.r.pods().count(), 2);
    let second: BTreeSet<PartId> = rig.r.members(placed.instance).into_iter().collect();
    assert_eq!(second.len(), members.len());
    assert!(second.is_disjoint(&members));
    assert_eq!(rig.r.ids().len(), rig.r.ids().iter().copied().collect::<BTreeSet<_>>().len());
    // Pods do not nest: not onto a pod's own part.
    let inside = rig.r.pod_of(*second.iter().find(|&&m| rig.r.part(m).is_some()).unwrap()).unwrap().kind.pod_body().unwrap().parts()[1].id;
    assert_eq!(rig.r.place(&identity, Placement::Surface { host: inside, offset: 0.0, angle: 0.0 }), Err(AssemblyError::PodInPod));
    // Updating re-makes the bodies in place with their ids.
    let mut editor = rig.r.assembly(&identity).unwrap().to_rocket();
    let body_tube = editor.pods().next().unwrap().kind.pod_body().unwrap().parts()[1].id;
    editor.attach_internal(body_tube, 0.05, altimeter(None)).unwrap();
    let def = AssemblyDef::from_rocket(identity.clone(), 2, "Strap-ons", &editor).unwrap();
    let report = rig.r.update_assembly(def).unwrap();
    assert_eq!(report.len(), 2);
    assert!(report.iter().all(|u| u.added.len() == 1 && u.removed.is_empty()));
    assert!(rig.r.is_member(pod) && rig.r.pods().count() == 2);
}

#[test]
fn a_stack_goes_into_a_pods_stack() {
    let mut rig = rig();
    let pod = rig.r.attach_radial(rig.booster, 0.1, 0.0, 2, rig.r.default_pod()).unwrap();
    let tube_in_pod = rig.r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
    let seed = rig.r.insert(4, tube(0.1, 0.05)).unwrap();
    rig.r.attach_internal(seed, 0.02, altimeter(None)).unwrap();
    let id = rig.r.group(&[seed], "Short bay").unwrap();
    let identity = rig.r.assemblies()[0].identity.clone();
    // Into the pod's stack behind its body tube (index 2); counted in the pod's own stack.
    let nodes = rig.r.assembly_nodes(&identity, Some(pod), None);
    assert_eq!(nodes.last().map(|n| n.0), Some(2));
    let placed = rig.r.place(&identity, Placement::Stack { pod: Some(pod), index: 2 }).unwrap();
    assert_eq!(rig.r.pod_of(rig.r.members(placed.instance)[0]).map(|p| p.id), Some(pod));
    assert_eq!(rig.r.placement(placed.instance), Some(Placement::Stack { pod: Some(pod), index: 2 }));
    assert_eq!(rig.r.radial(pod).unwrap().kind.pod_body().unwrap().parts().len(), 3);
    assert!(rig.r.part(tube_in_pod).is_some());
    // A stack does not move between the rocket and a pod.
    assert_eq!(rig.r.move_instance(placed.instance, Placement::Stack { pod: None, index: 2 }), Err(AssemblyError::AcrossPods));
    assert_eq!(rig.r.move_instance(id, Placement::Stack { pod: Some(pod), index: 0 }), Err(AssemblyError::AcrossPods));
    // Within the pod it moves.
    rig.r.move_instance(placed.instance, Placement::Stack { pod: Some(pod), index: 1 }).unwrap();
    assert_eq!(rig.r.placement(placed.instance), Some(Placement::Stack { pod: Some(pod), index: 1 }));
    // Updating re-makes the bay in the pod's stack and in the rocket's, each where it is.
    let mut editor = rig.r.assembly(&identity).unwrap().to_rocket();
    let editor_tube = editor.parts()[0].id;
    editor.set_dim(editor_tube, Dim::Length, 0.2).unwrap();
    let def = AssemblyDef::from_rocket(identity.clone(), 2, "Short bay", &editor).unwrap();
    rig.r.update_assembly(def).unwrap();
    assert_eq!(rig.r.placement(placed.instance), Some(Placement::Stack { pod: Some(pod), index: 1 }));
    assert_eq!(rig.r.placement(id), Some(Placement::Stack { pod: None, index: 4 }));
    assert_eq!(rig.r.radial(pod).unwrap().kind.pod_body().unwrap().parts().len(), 3);
    let member_tube = rig.r.members(placed.instance).into_iter().find(|&m| rig.r.part(m).is_some()).unwrap();
    assert_eq!(rig.r.part(member_tube).unwrap().kind.length(), 0.2);
}

#[test]
fn an_assembly_with_a_nose_travels_to_another_document() {
    let rig = rig();
    let mut r = rig.r.clone();
    r.group(&[rig.nose, rig.upper], "Upper stage").unwrap();
    let def = AssemblyDef::from_bytes(&r.assemblies()[0].to_bytes()).unwrap();
    let identity = def.identity.clone();

    // Into an empty document: the nose goes first.
    let mut other = Rocket::default();
    other.add_assembly(def).unwrap();
    assert_eq!(other.assembly_nodes(&identity, None, None), vec![(0, 0.0)]);
    let first = other.place(&identity, Placement::Stack { pod: None, index: 0 }).unwrap();
    assert_eq!(other.parts().len(), 2);
    assert!(matches!(other.parts()[0].kind, PartKind::NoseCone { .. }));
    // A second nose is refused, in the middle or at the front; the document is untouched by the refusal.
    let before = other.clone();
    assert_eq!(other.place(&identity, Placement::Stack { pod: None, index: 0 }), Err(AssemblyError::Insert(InsertError::SecondNose)));
    assert_eq!(other.place(&identity, Placement::Stack { pod: None, index: 2 }), Err(AssemblyError::Insert(InsertError::NoseNotFirst)));
    assert_eq!(other, before);
    assert!(other.assembly_nodes(&identity, None, None).is_empty());
    // Moving it is moving the whole stack; the nose stays first, so the only index is 0 (without it, the stack is empty).
    assert_eq!(other.assembly_nodes(&identity, None, Some(first.instance)), vec![(0, 0.0)]);
    other.move_instance(first.instance, Placement::Stack { pod: None, index: 0 }).unwrap();
    assert_eq!(other, before);
}

// ---- formulas ---------------------------------------------------------------------------------------------------------------

#[test]
fn formulas_inside_a_fragment_survive_and_bind_to_the_new_instance() {
    let mut rig = rig();
    let r = &mut rig.r;
    // Inside the bay: the lug's diameter follows half the bay's length; and one that refers outside (the booster): frozen.
    let inside = Expr::Bin(Box::new(Expr::Num(0.1)), Op::Mul, Box::new(Expr::Ref(FieldRef::new(rig.bay, Dim::Length))));
    r.set_formula(rig.lug, Dim::Length, inside).unwrap();
    r.set_formula(rig.bay, Dim::Diameter, Expr::Ref(FieldRef::new(rig.upper, Dim::Diameter))).unwrap();
    r.set_formula(rig.avionics, Dim::Length, Expr::Num(0.08)).unwrap();
    assert_eq!(r.formulas().len(), 3);
    let diameter_before = r.part(rig.bay).unwrap().kind.aft_diameter();

    let id = r.group(&[rig.bay], "Bay").unwrap();
    // The formula that refers to the upper tube is gone from the document; the field kept its value.
    assert!(r.formula(rig.bay, Dim::Diameter).is_none());
    assert_eq!(r.part(rig.bay).unwrap().kind.aft_diameter(), diameter_before);
    // The inside formula (and the constant one) stays and is in the definition over local ids.
    assert!(r.formula(rig.lug, Dim::Length).is_some());
    let def = r.assemblies()[0].clone();
    assert_eq!(def.fragment.formulas().len(), 2);
    assert!(def.fragment.formulas().iter().all(|f| f.expr.refs().iter().all(|x| def.fragment.ids().contains(&x.part))));
    assert_eq!(r.instance(id).unwrap().members.len(), 6);

    // Place a second instance: its formulas bind to its own ids, and it follows its own bay.
    let end = r.parts().len();
    let placed = r.place(&def.identity, Placement::Stack { pod: None, index: end }).unwrap();
    let members = r.members(placed.instance);
    let tube2 = members.iter().copied().find(|&m| r.part(m).is_some()).unwrap();
    let lug2 = r.radials().iter().find(|g| g.parent == tube2).unwrap().id;
    let f = r.formula(lug2, Dim::Length).unwrap();
    assert_eq!(f.expr.refs(), vec![FieldRef::new(tube2, Dim::Length)]);
    assert!(members.contains(&lug2));
    // Lengthen the second bay by its own dimension: only its lug follows.
    r.set_dim(tube2, Dim::Length, 0.5).unwrap();
    r.apply_formulas();
    assert!((r.dim(lug2, Dim::Length).unwrap() - 0.05).abs() < 1e-12);
    assert!((r.dim(rig.lug, Dim::Length).unwrap() - 0.03).abs() < 1e-12);

    // Updating re-binds: the formula is re-made over the (kept) ids; no stale copies.
    let mut newer = def.clone();
    newer.version = 2;
    r.update_assembly(newer).unwrap();
    assert_eq!(r.formulas().len(), 4);
    assert!(r.formula(lug2, Dim::Length).is_some());
}

// ---- ending instances -----------------------------------------------------------------------------------------------------

#[test]
fn detach_delete_and_remove_follow_their_rules() {
    let (rig, identity, [first, second, third]) = bay_thrice();
    let mut r = rig.r.clone();
    let before_parts = r.parts().to_vec();

    // Removing a definition is refused while instances exist.
    assert_eq!(r.remove_assembly(&identity), Err(AssemblyError::InUse { instances: 3 }));
    assert_eq!(r.remove_assembly(&AssemblyId::random()).map_err(|e| matches!(e, AssemblyError::NoSuchAssembly(_))), Err(true));

    // Detach: ordinary parts, nothing changes in the rocket, the definition stays.
    let members = r.members(first);
    let doc = flat(&r);
    let freed = r.detach(first).unwrap();
    assert_eq!(freed.len(), 6);
    assert!(members.iter().all(|&m| !r.is_member(m)));
    assert_eq!(flat(&r), doc);
    assert_eq!(r.parts(), before_parts);
    assert_eq!(r.assemblies().len(), 1);
    assert_eq!(r.detach(first), Err(AssemblyError::NoSuchInstance(first)));

    // Delete: the members go, by the rules of deleting the parts.
    let second_members = r.members(second);
    r.delete_instance(second).unwrap();
    assert!(second_members.iter().all(|&m| r.kind_of(m).is_none()));
    assert_eq!(r.parts().len(), before_parts.len() - 1);
    assert_eq!(r.instances().len(), 1);

    // ungroup: the last instance goes, the definition goes with it unless it is wanted on the shelf.
    let kept = {
        let mut k = r.clone();
        k.ungroup(third, true).unwrap();
        k
    };
    assert_eq!(kept.assemblies().len(), 1);
    assert!(kept.instances().is_empty());
    r.ungroup(third, false).unwrap();
    assert!(r.assemblies().is_empty() && r.instances().is_empty());
    // A definition with no instance is removable, and a duplicate is a new assembly.
    let mut r = kept;
    let copy = r.duplicate_assembly(&identity, "Bay B").unwrap();
    assert_ne!(copy, identity);
    let (a, b) = (r.assembly(&identity).unwrap().clone(), r.assembly(&copy).unwrap().clone());
    assert_eq!((b.version, b.name.as_str()), (1, "Bay B"));
    assert_eq!(a.fragment, b.fragment);
    r.remove_assembly(&identity).unwrap();
    assert_eq!(r.assemblies().len(), 1);
}

#[test]
fn deleting_one_member_the_ordinary_way_detaches_the_instance_first() {
    let (rig, _, [first, second, third]) = bay_thrice();
    let mut r = rig.r.clone();
    // A member internal part.
    let chute = r.members(second).into_iter().find(|&m| matches!(r.internal(m).map(|p| &p.kind), Some(InternalKind::Parachute { .. }))).unwrap();
    assert!(r.remove_internal(chute).is_some());
    assert!(r.instance(second).is_none() && r.instance(first).is_some());
    // The other members of it stay as ordinary parts.
    assert!(r.members(first).len() == 6);

    // A member airframe part: its whole instance goes ordinary, then the part and what is on it are deleted.
    let tube = r.members(third).into_iter().find(|&m| r.part(m).is_some()).unwrap();
    assert!(r.remove(tube).is_some());
    assert!(r.instance(third).is_none());

    // Removing a part that a member stands on (a host tube) detaches too.
    let mut rig2 = rig_internal_instance();
    let instance = rig2.r.instances()[0].id;
    assert!(rig2.r.remove(rig2.bay).is_some());
    assert!(rig2.r.instance(instance).is_none());
    assert!(rig2.r.instances().is_empty());

    // Splitting a tube that holds a member detaches; a fin group of an instance split into copies too.
    let mut rig3 = self::rig();
    rig3.r.group(&[rig3.fins], "Fins").unwrap();
    assert!(rig3.r.detach_radial(rig3.fins, 0).is_ok());
    assert!(rig3.r.instances().is_empty());
    let mut rig4 = rig_internal_instance();
    rig4.r.split_tube(rig4.bay, 0.1 + rig4.r.part_station(rig4.bay).unwrap()).unwrap();
    assert!(rig4.r.instances().is_empty());
}

fn rig_internal_instance() -> Rig {
    let mut rig = rig();
    rig.r.group(&[rig.avionics, rig.chute], "Payload").unwrap();
    rig
}

#[test]
fn reconcile_drops_an_instance_that_lost_a_member_by_other_means() {
    let mut rig = rig_internal_instance();
    let instance = rig.r.instances()[0].id;
    // A part taken out and never put back (a carried part dropped on the bin).
    let taken = rig.r.take_internals(rig.bay);
    assert!(!taken.is_empty());
    assert_eq!(rig.r.members(instance).len(), 0, "members that are not there are not listed");
    assert_eq!(rig.r.reconcile_instances(), vec![instance]);
    assert!(rig.r.instances().is_empty());
    // Put back first: nothing to reconcile.
    let mut rig = rig_internal_instance();
    let taken = rig.r.take_internals(rig.bay);
    rig.r.restore_internals(taken);
    assert!(rig.r.reconcile_instances().is_empty());
    assert_eq!(rig.r.instances().len(), 1);
}

// ---- the definition as a document ------------------------------------------------------------------------------------------

#[test]
fn a_definition_opened_as_a_document_reads_back_unchanged_for_each_kind() {
    let rig = rig();
    let cases: [(&[PartId], FragmentKind); 4] = [
        (&[rig.bay], FragmentKind::Stack),
        (&[rig.bay, rig.booster], FragmentKind::Stack),
        (&[rig.avionics, rig.wall_ring, rig.chute], FragmentKind::Internal),
        (&[rig.fins], FragmentKind::Surface),
    ];
    for (ids, kind) in cases {
        let mut r = rig.r.clone();
        r.group(ids, "Thing").unwrap();
        let def = r.assemblies()[0].clone();
        assert_eq!(def.kind(), kind);
        let editor = def.to_rocket();
        assert_eq!(editor.stand_in().is_some(), kind != FragmentKind::Stack);
        assert_eq!(editor.parts().len(), def.fragment.parts().len() + usize::from(kind != FragmentKind::Stack));
        let back = AssemblyDef::from_rocket(def.identity.clone(), def.version, &def.name, &editor).unwrap();
        assert_eq!(back, def, "{kind:?}");
        // And through the editor's own file format.
        let saved = Rocket::from_bytes(&editor.to_bytes()).unwrap();
        assert_eq!(AssemblyDef::from_rocket(def.identity.clone(), def.version, &def.name, &saved).unwrap(), def);
    }

    // The stand-in is stripped, edits to the host become the recorded host, a formula that refers to the host is dropped.
    let mut r = rig.r.clone();
    r.group(&[rig.fins], "Fins").unwrap();
    let def = r.assemblies()[0].clone();
    let mut editor = def.to_rocket();
    let host = editor.stand_in().unwrap();
    let fins = editor.radials()[0].id;
    editor.set_formula(fins, Dim::Span, Expr::Ref(FieldRef::new(host, Dim::Diameter))).unwrap();
    editor.set_dim(host, Dim::Diameter, 0.1).unwrap();
    editor.apply_formulas();
    let edited = AssemblyDef::from_rocket(def.identity.clone(), 2, "Fins", &editor).unwrap();
    assert!((edited.fragment.host().unwrap().outer_diameter - 0.1).abs() < 1e-12);
    assert!(edited.fragment.parts().is_empty() && edited.fragment.formulas().is_empty());
    assert_eq!(edited.fragment.radials()[0].kind.fin_span(), Some(0.1), "the field kept the value the formula gave it");

    // An editor document with nothing in it, or radial and internal parts with no airframe part, is not an assembly.
    assert_eq!(AssemblyDef::from_rocket(AssemblyId::random(), 1, "x", &Rocket::default()), Err(AssemblyError::Empty));
    let mut mixed = def.to_rocket();
    mixed.attach_internal(mixed.stand_in().unwrap(), 0.05, altimeter(None)).unwrap();
    assert_eq!(AssemblyDef::from_rocket(AssemblyId::random(), 1, "x", &mixed), Err(AssemblyError::MixedClasses));
    // Motor assignments in the editor are not read.
    let mut stack = rig.r.clone();
    stack.group(&[rig.booster], "Booster").unwrap();
    let def = stack.assemblies()[0].clone();
    let mut editor = def.to_rocket();
    let mount = editor.internals().iter().find(|p| p.kind.is_motor_mount()).unwrap().id;
    let config = editor.active_config_id();
    editor.set_motor(config, mount, Some(MotorRef::new("H128W", "Aerotech"))).unwrap();
    assert_eq!(AssemblyDef::from_rocket(def.identity.clone(), def.version, &def.name, &editor).unwrap(), def);
}

// ---- files ------------------------------------------------------------------------------------------------------------------

#[test]
fn a_definition_round_trips_as_a_standalone_file() {
    let rig = rig();
    for ids in [vec![rig.bay], vec![rig.avionics, rig.chute], vec![rig.fins]] {
        let mut r = rig.r.clone();
        r.group(&ids, "Thing").unwrap();
        let def = r.assemblies()[0].clone();
        let bytes = def.to_bytes();
        assert_eq!(&bytes[..8], ASSEMBLY_MAGIC);
        assert_eq!(u32::from(u16::from_le_bytes([bytes[8], bytes[9]])), ASSEMBLY_FORMAT_VERSION);
        assert_eq!(AssemblyDef::from_bytes(&bytes).unwrap(), def);
        // A rocket file is not an assembly file and the other way round.
        assert!(matches!(Rocket::from_bytes(&bytes), Err(FormatError::NotBowshock)));
        assert!(matches!(AssemblyDef::from_bytes(&Rocket::default().to_bytes()), Err(FormatError::NotBowshock)));
        // Newer versions, damage and inconsistency are refused.
        let mut newer = bytes.clone();
        newer[8..10].copy_from_slice(&(ASSEMBLY_FORMAT_VERSION as u16 + 1).to_le_bytes());
        assert!(matches!(AssemblyDef::from_bytes(&newer), Err(FormatError::UnsupportedVersion(_))));
        assert!(matches!(AssemblyDef::from_bytes(&bytes[..bytes.len() / 2]), Err(FormatError::Corrupt(_))));
        let mut broken = def.clone();
        broken.fragment.next_id = 0;
        assert!(matches!(AssemblyDef::from_bytes(&broken.to_bytes()), Err(FormatError::Invalid(_))));
    }
    // The identity is text of 32 hex digits and is checked on the way in.
    let id = AssemblyId::random();
    assert_eq!(AssemblyId::parse(id.as_str()), Some(id.clone()));
    assert_eq!(AssemblyId::parse(&id.as_str().to_uppercase()), Some(id.clone()));
    assert_eq!(AssemblyId::parse("nope"), None);
    assert_ne!(id, AssemblyId::random());
    let json = serde_json::to_string(&id).unwrap();
    assert_eq!(serde_json::from_str::<AssemblyId>(&json).unwrap(), id);
    assert!(serde_json::from_str::<AssemblyId>("\"12\"").is_err());
}

#[test]
fn a_document_with_assemblies_round_trips_and_older_files_have_none() {
    let (rig, _, _) = bay_thrice();
    let mut r = rig.r;
    r.group(&[rig.fins], "Fins").unwrap();
    // A shelf definition with no instance too.
    let mut other = Rocket::default();
    other.insert(0, tube(0.2, D)).unwrap();
    let t = other.parts()[0].id;
    other.attach_internal(t, 0.0, altimeter(None)).unwrap();
    other.group(&[t], "Spare bay").unwrap();
    let spare = other.assemblies()[0].clone();
    r.add_assembly(spare.clone()).unwrap();
    assert_eq!(r.add_assembly(spare.clone()), Err(AssemblyError::Duplicate(spare.identity.clone())));
    assert_eq!(r.assemblies().len(), 3);
    let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
    assert_eq!(back, r);
    assert_eq!(back.instances(), r.instances());
    assert_eq!(back.assemblies(), r.assemblies());
    // The loaded document keeps working: new ids do not collide with instance ids.
    let mut back = back;
    let identity = back.assemblies()[0].identity.clone();
    let end = back.parts().len();
    let placed = back.place(&identity, Placement::Stack { pod: None, index: end }).unwrap();
    assert!(!back.ids().contains(&placed.instance) && back.instances().iter().filter(|i| i.id == placed.instance).count() == 1);
    // A document without any writes no assembly keys.
    let plain = crate::format::cbor_text(&rig_internal_instance().r.clone());
    assert!(plain.contains("instances"));
    assert!(!crate::format::cbor_text(&self::rig().r).contains("instances"));
    assert!(!crate::format::cbor_text(&self::rig().r).contains("assemblies"));
}

#[test]
fn loading_drops_a_damaged_instance_or_definition_and_keeps_the_parts() {
    let (rig, identity, [first, second, _]) = bay_thrice();
    let parts_before = rig.r.parts().len();
    // An instance whose member is gone.
    let mut r = rig.r.clone();
    r.instances.iter_mut().find(|i| i.id == second).unwrap().members.insert(0, 987_654);
    let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
    assert!(back.instance(second).is_none() && back.instance(first).is_some());
    assert_eq!(back.parts().len(), parts_before);
    // A member shared by two instances: the later one is dropped.
    let mut r = rig.r.clone();
    let shared = r.instance(first).unwrap().members.clone();
    r.instances.iter_mut().find(|i| i.id == second).unwrap().members = shared;
    let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
    assert_eq!(back.instances().len(), 2);
    // A definition that is not well-formed goes, with the instances that used it.
    let mut r = rig.r.clone();
    r.assemblies[0].fragment.next_id = 0;
    let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
    assert!(back.assemblies().is_empty() && back.instances().is_empty());
    assert_eq!(back.parts().len(), parts_before);
    assert!(back.assembly(&identity).is_none());
    // A stand-in flag on a part that is not there is cleared.
    let mut r = rig.r.clone();
    r.stand_in = Some(31_337);
    assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap().stand_in(), None);
}

#[test]
fn a_stand_in_document_carries_its_flag_through_a_save() {
    let mut rig = rig();
    rig.r.group(&[rig.fins], "Fins").unwrap();
    let editor = rig.r.assemblies()[0].to_rocket();
    let host = editor.stand_in().unwrap();
    let back = Rocket::from_bytes(&editor.to_bytes()).unwrap();
    assert_eq!(back.stand_in(), Some(host));
    assert_eq!(back, editor);
}

#[test]
fn a_fragment_check_names_what_is_wrong() {
    let mut rig = rig();
    rig.r.group(&[rig.bay], "Bay").unwrap();
    let good = rig.r.assemblies()[0].fragment.clone();
    assert!(good.check().is_ok());
    let invalid = |f: &Fragment| matches!(f.check(), Err(AssemblyError::Invalid(_)));

    let mut f = good.clone();
    f.internals[0].parent = 99;
    assert!(invalid(&f));
    let mut f = good.clone();
    f.host = Some(Host { outer_diameter: D, thickness: 0.001, length: 0.3 });
    assert!(invalid(&f), "a stack has no host");
    let mut f = good.clone();
    f.parts.clear();
    f.radials.clear();
    assert!(matches!(f.check(), Err(AssemblyError::MixedClasses) | Err(AssemblyError::Invalid(_))), "{:?}", f.check());
    let mut f = good.clone();
    f.internals.clear();
    f.parts.clear();
    f.radials[0].parent = 7;
    assert!(invalid(&f), "an internal or surface assembly needs its host");
    let mut f = good;
    f.formulas.push(crate::Formula { target: FieldRef::new(0, Dim::Length), expr: Expr::Ref(FieldRef::new(500, Dim::Length)), names: Default::default() });
    assert!(invalid(&f));
}
