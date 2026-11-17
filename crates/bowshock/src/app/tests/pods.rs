use super::*;

#[test]
fn a_click_attaches_radial_parts_to_the_aftmost_tube_with_the_symmetry_count() {
    let (mut app, [_, tube, _]) = app();
    app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(4)));
    app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 0))));
    app.update(Message::Released);
    let [fins] = app.ctx.doc.rocket().radials() else { panic!("one radial part") };
    assert_eq!((fins.parent, fins.count, fins.angle), (tube, 4, 0.0));
    // Flush with the aft end of the tube (0.5 long, 0.12 root chord).
    assert!((fins.offset - (0.5 - 0.12)).abs() < 1e-9);
    // With nothing to attach to, a click does nothing.
    let mut empty = App::new();
    empty.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
    empty.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 1))));
    empty.update(Message::Released);
    assert!(empty.ctx.doc.rocket().radials().is_empty());
}

#[test]
fn dragging_a_radial_part_onto_the_surface_creates_a_group_with_the_live_count() {
    let (mut app, [_, tube, _]) = app();
    app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(3)));
    // Press the lug row and move beyond the slop: the drag starts with the footer's count.
    app.update(Message::Cursor(Point::new(100.0, 100.0)));
    app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 3))));
    app.update(Message::Cursor(Point::new(160.0, 100.0)));
    assert!(app.ctx.drag.radial.as_ref().is_some_and(|d| d.count == 3 && d.row == Some(3)));
    // X cycles the count (and the footer's, for the next one); Shift+X goes back.
    app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(true)));
    assert_eq!((app.ctx.drag.radial.as_ref().unwrap().count, app.ctx.drag.symmetry), (4, 4));
    app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(false)));
    app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(false)));
    assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 2);
    vp(&mut app, Vp::Surface(Some(target(tube, 0.1, 1.0)), false));
    assert!(app.ctx.drag.radial.as_ref().is_some_and(|d| d.target == Some(target(tube, 0.1, 1.0))));
    vp(&mut app, Vp::DropRadial);
    let [lug] = app.ctx.doc.rocket().radials() else { panic!("one radial part") };
    assert_eq!((lug.parent, lug.count, lug.offset, lug.angle), (tube, 2, 0.1, 1.0));
    assert!(app.ctx.drag.radial.is_none());

    // No target: cancelled. Escape: cancelled.
    let start = |app: &mut App| {
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 4))));
        app.update(Message::Cursor(Point::new(220.0, 100.0)));
    };
    start(&mut app);
    vp(&mut app, Vp::DropRadial);
    start(&mut app);
    vp(&mut app, Vp::Surface(Some(target(tube, 0.1, 1.0)), false));
    app.update(Message::Cancel);
    assert_eq!(app.ctx.doc.rocket().radials().len(), 1);
}

#[test]
fn a_radial_group_can_be_picked_up_moved_and_put_back_and_deleted() {
    let (mut app, [_, tube, transition]) = app();
    let id = app.ctx.doc.edit("Lug", |r| r.attach_radial(tube, 0.0, 0.0, 3, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap());
    let before = app.ctx.doc.rocket().radials().to_vec();
    vp(&mut app, Vp::Grab(Pick::Radial { id, instance: 1 }));
    assert!(app.ctx.doc.rocket().radials().is_empty() && app.ctx.selection == Selection::Part(id));
    // Cancelled, or dropped with no target: back as it was.
    app.update(Message::Cancel);
    assert_eq!(app.ctx.doc.rocket().radials(), before.as_slice());
    vp(&mut app, Vp::Grab(Pick::Radial { id, instance: 1 }));
    vp(&mut app, Vp::DropRadial);
    assert_eq!(app.ctx.doc.rocket().radials(), before.as_slice());
    // Dropped on another part: moved there, same id, same count.
    vp(&mut app, Vp::Grab(Pick::Radial { id, instance: 1 }));
    vp(&mut app, Vp::Surface(Some(target(transition, 0.0, 2.0)), false));
    vp(&mut app, Vp::DropRadial);
    let moved = app.ctx.doc.rocket().radial(id).unwrap();
    assert_eq!((moved.parent, moved.count, moved.angle), (transition, 3, 2.0));
    // Deleting the selected group removes all of it; its ids are not reused.
    app.update(Message::Delete);
    assert!(app.ctx.doc.rocket().radials().is_empty() && app.ctx.selection == Selection::None);
    assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
}

#[test]
fn an_ordinary_tube_dragged_onto_a_tubes_side_becomes_a_pod_and_a_node_still_stacks() {
    let (mut app, [_, tube, _]) = app();
    let before = app.ctx.doc.rocket().clone();
    let row = parts_bin::axial_catalogue().iter().position(|(n, _)| *n == "Body tube").unwrap();
    let start = |app: &mut App| {
        app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Airframe)));
        app.update(Message::Cursor(Point::new(100.0, 100.0)));
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, row))));
        app.update(Message::Cursor(Point::new(160.0, 100.0)));
    };
    // On a tube's side: a pod set of the bin's symmetry count whose body is exactly the part dropped.
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(3)));
    start(&mut app);
    assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.count == 3 && d.surface.is_none()));
    vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 1.0))));
    assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.surface.is_some() && d.snap.is_none()));
    // X cycles the copies while it hovers there.
    app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(false)));
    assert_eq!(app.ctx.drag.axial.as_ref().unwrap().count, 2);
    vp(&mut app, Vp::DropAxial);
    let pod = app.ctx.doc.rocket().pods().next().expect("a pod set").clone();
    let body = pod.kind.pod_body().unwrap();
    assert_eq!((pod.count, pod.parent, pod.offset, pod.angle), (2, tube, 0.1, 1.0));
    assert_eq!(body.parts().len(), 1);
    assert!(matches!(body.parts()[0].kind, bowshock_model::PartKind::BodyTube { .. }));
    assert_eq!(app.ctx.doc.rocket().parts().len(), 3, "the core is unchanged");
    assert_eq!(app.ctx.selection, Selection::Part(body.parts()[0].id));
    // Undo is one step.
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket().ids(), before.ids());
    // A node still stacks, and moving to a node leaves the surface mode.
    start(&mut app);
    vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 1.0))));
    vp(&mut app, Vp::SnapNode(Some(1)));
    assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.surface.is_none() && d.snap == Some(1)));
    vp(&mut app, Vp::DropAxial);
    assert_eq!((app.ctx.doc.rocket().parts().len(), app.ctx.doc.rocket().pods().count()), (4, 0));
}

#[test]
fn moving_an_existing_tube_onto_a_tubes_side_makes_a_pod_and_back_onto_a_node_stacks_it() {
    let (mut app, [_, tube, transition]) = app();
    let fins = app.ctx.doc.edit("Fins", |r| {
        r.attach_radial(transition, 0.0, 0.0, 3, RadialKind::FinSet { root_chord: 0.03, tip_chord: 0.01, span: 0.02, sweep: 0.01, thickness: 0.002 }).unwrap()
    });
    let before = app.ctx.doc.rocket().clone();
    // Esc cancels exactly.
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 2.0))));
    app.update(Message::Cancel);
    assert_eq!(app.ctx.doc.rocket(), &before);
    // Dropped on a tube's side: a pod of that one part (ids and the fins on it kept), as one undo step.
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 2.0))));
    vp(&mut app, Vp::DropAxial);
    let rocket = app.ctx.doc.rocket();
    let pod = rocket.pod_of(transition).expect("the transition is a pod's part now");
    assert_eq!((rocket.parts().len(), pod.kind.pod_body().unwrap().parts().len(), pod.angle), (2, 1, 2.0));
    assert_eq!(rocket.parent_of(fins), Some(transition));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    app.update(Message::Redo);
    // Back onto a node: the part stacks into the core again and the pod is gone.
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.origin.is_some()) && app.ctx.doc.rocket().pods().next().is_none());
    vp(&mut app, Vp::SnapNode(Some(2)));
    vp(&mut app, Vp::DropAxial);
    let rocket = app.ctx.doc.rocket();
    assert_eq!((rocket.parts().len(), rocket.pods().count(), rocket.index_of(transition)), (3, 0, Some(2)));
    assert_eq!(rocket.parent_of(fins), Some(transition));
}

#[test]
fn a_click_add_lands_on_the_selected_pod_tube_and_a_pod_mount_takes_a_motor() {
    let (mut app, [_, tube, _]) = app();
    let kind = app.ctx.doc.rocket().default_pod();
    let pod = app.ctx.doc.edit("Pod", |r| r.attach_radial(tube, 0.0, 0.0, 2, kind).unwrap());
    let pod_tube = app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
    app.ctx.selection = Selection::Part(pod_tube);
    app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
    app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 0))));
    app.update(Message::Released);
    let fins = app.ctx.doc.rocket().pod_of(pod_tube).and_then(|p| p.kind.pod_body()).unwrap().radials()[0].id;
    assert_eq!(app.ctx.doc.rocket().parent_of(fins), Some(pod_tube));
    // A motor mount in the pod is a mount of the rocket (one motor per copy) and takes a motor from the bin.
    let mount = app.ctx.doc.edit("Mount", |r| {
        r.attach_internal(
            pod_tube,
            0.05,
            bowshock_model::InternalKind::InnerTube { outer_diameter: 0.0229, thickness: 0.0005, length: 0.12, motor_mount: true, overhang: 0.0 },
        )
        .unwrap()
    });
    let m = app.ctx.doc.rocket().motor_mounts().into_iter().find(|m| m.id == mount).expect("the pod's mount is a mount of the rocket");
    assert_eq!((m.pod, m.copies), (Some(pod), 2));
    let config = app.ctx.doc.rocket().active_config_id();
    app.ctx.doc.edit("Motor", |r| {
        r.set_motor(config, mount, Some(bowshock_model::MotorRef::new("x", "y"))).unwrap();
    });
    assert!(app.ctx.doc.rocket().active_configuration().slot(mount).is_some_and(|s| s.motor.is_some()));
    // Pod mounts are not stages.
    assert!(app.ctx.doc.rocket().stages().iter().all(|s| !s.mounts.contains(&mount)));
}

#[test]
fn the_nose_rows_stay_enabled_while_any_body_has_a_free_fore_end() {
    let (mut app, [_, tube, _]) = app();
    // The core has its nose and there is no other body: every nose row says why it is dim.
    assert!(nose_reasons(&app).iter().all(|r| r.as_deref().is_some_and(|r| r.starts_with("Every body has a nose"))), "{:?}", nose_reasons(&app));
    // A bare tube on the outside has a free fore end: every nose variant is available again.
    let (pod, _) = bare_pod(&mut app, tube);
    assert!(nose_reasons(&app).iter().all(Option::is_none), "{:?}", nose_reasons(&app));
    // Once that pod has a nose too, no body has a free fore end.
    assert!(app
        .ctx
        .doc
        .edit("Nose", |r| r.insert_in_pod(pod, 0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.05, diameter: 0.03, tip_radius: 0.0 }).is_ok()));
    assert!(nose_reasons(&app).iter().all(Option::is_some));
    // The tail cone goes to the aft node of any body, the pod's included.
    let boattail = parts_bin::RowId::new(parts_bin::Category::Airframe, 7);
    assert!(app.parts_bin.row_unavailable(boattail, &app.ctx).is_none());
}

#[test]
fn a_nose_dragged_onto_a_pod_tubes_fore_node_goes_on_it_and_the_core_keeps_its_own() {
    let (mut app, [_, tube, _]) = app();
    let (pod, pod_tube) = bare_pod(&mut app, tube);
    let before = app.ctx.doc.rocket().clone();
    app.update(Message::Cursor(Point::new(100.0, 100.0)));
    app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, 0))));
    app.update(Message::Cursor(Point::new(160.0, 100.0)));
    assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| matches!(d.kind, PartKind::NoseCone { shape: NoseShape::TangentOgive, .. })), "the nose row drags");
    vp(&mut app, Vp::SnapPodNode(Some((pod, 0))));
    vp(&mut app, Vp::DropAxial);
    let kinds = pod_kinds(&app, pod);
    assert_eq!(kinds.len(), 2);
    // It is the pod's first part, sized to the tube behind it (Ø30), and the core is unchanged.
    assert!(matches!(kinds[0], PartKind::NoseCone { shape: NoseShape::TangentOgive, diameter, .. } if (diameter - 0.03).abs() < 1e-9), "{kinds:?}");
    assert_eq!(app.ctx.doc.rocket().parts(), before.parts());
    assert_eq!(app.ctx.doc.rocket().pod_part_index(pod_tube), Some((pod, 1)));
    // One undo step takes it off again.
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
}

#[test]
fn a_clicked_nose_goes_on_the_selected_pod_and_else_on_the_first_pod_with_a_free_fore_end() {
    let (mut app, [_, tube, _]) = app();
    let (pod, pod_tube) = bare_pod(&mut app, tube);
    // (Different rows, so the second click is not taken for a double click on the first.)
    let click = |app: &mut App, row: usize| {
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, row))));
        app.update(Message::Released);
    };
    // A pod's tube selected: the nose goes on that pod.
    app.ctx.selection = Selection::Part(pod_tube);
    click(&mut app, 0);
    assert!(matches!(pod_kinds(&app, pod)[0], PartKind::NoseCone { .. }));
    assert_eq!(app.ctx.doc.rocket().parts().len(), 3, "the core is unchanged");
    // A second pod, with the core's tube selected: the core has its nose, so it goes to the pod.
    let (pod2, _) = bare_pod(&mut app, tube);
    app.ctx.selection = Selection::Part(tube);
    click(&mut app, 1);
    assert!(matches!(pod_kinds(&app, pod2)[0], PartKind::NoseCone { shape: NoseShape::Conical, .. }), "{:?}", pod_kinds(&app, pod2));
    assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
}

#[test]
fn a_pod_is_dragged_in_filled_with_fins_extended_moved_and_undone_as_one_body() {
    let (mut app, [_, tube, _]) = app();
    let before = app.ctx.doc.rocket().clone();
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(2)));
    app.update(Message::Cursor(Point::new(100.0, 100.0)));
    // The pod row is the last of the Airframe list.
    let row = parts_bin::airframe_items().len() - 1;
    app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, row))));
    app.update(Message::Cursor(Point::new(160.0, 100.0)));
    assert!(app.ctx.drag.radial.as_ref().is_some_and(|d| d.kind.is_pod() && d.count == 2));
    vp(&mut app, Vp::Surface(Some(target(tube, 0.05, 0.0)), false));
    vp(&mut app, Vp::DropRadial);
    let pod = app.ctx.doc.rocket().pods().next().expect("a pod was placed").id;
    let body = app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().clone();
    assert_eq!((app.ctx.doc.rocket().radial(pod).unwrap().count, body.parts().len()), (2, 2));
    let pod_tube = body.parts()[1].id;
    // Fins land on the pod's tube; a part of the pod joins at a node of its stack.
    app.ctx.doc.begin_drag("Add fins");
    app.ctx.drag.radial = Some(crate::viewport::surface::RadialDrag {
        row: None,
        kind: RadialKind::FinSet { root_chord: 0.05, tip_chord: 0.02, span: 0.03, sweep: 0.02, thickness: 0.002 },
        count: 3,
        origin: None,
        target: Some(target(pod_tube, 0.05, 0.0)),
        detached: false,
    });
    vp(&mut app, Vp::DropRadial);
    let fins = app.ctx.doc.rocket().pod_of(pod_tube).and_then(|p| p.kind.pod_body()).unwrap().radials()[0].id;
    assert_eq!(app.ctx.doc.rocket().parent_of(fins), Some(pod_tube));
    app.ctx.doc.begin_drag("Add tube");
    app.ctx.drag.axial = Some(crate::viewport::assembly::Drag {
        row: None,
        origin: None,
        name: "Body tube",
        kind: bowshock_model::PartKind::BodyTube { length: 0.1, diameter: 0.0381 },
        snap: None,
        pod_snap: Some((pod, 2)),
        surface: None,
        count: 2,
    });
    vp(&mut app, Vp::DropAxial);
    assert_eq!(app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().parts().len(), 3);
    // Picking up any part of the pod carries the whole body, fins and all; cancelled, everything is back.
    let snapshot = app.ctx.doc.rocket().clone();
    vp(&mut app, Vp::Grab(Pick::Axial(pod_tube)));
    assert!(app.ctx.doc.rocket().pods().next().is_none() && app.ctx.drag.radial.as_ref().is_some_and(|d| d.origin.is_some()));
    app.update(Message::Cancel);
    assert_eq!(app.ctx.doc.rocket(), &snapshot);
    // Dropped elsewhere it moves with every part, ids unchanged.
    vp(&mut app, Vp::Grab(Pick::Axial(pod_tube)));
    vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 1.5)), false));
    vp(&mut app, Vp::DropRadial);
    let moved = app.ctx.doc.rocket().radial(pod).unwrap();
    assert_eq!((moved.offset, moved.angle), (0.2, 1.5));
    assert!(app.ctx.doc.rocket().radial(fins).is_some() && app.ctx.doc.rocket().part(pod_tube).is_some());
    // Four steps above (the pod, its fins, the tube, the move) undo back to the rocket we started with.
    for _ in 0..4 {
        app.update(Message::Undo);
    }
    assert_eq!(app.ctx.doc.rocket().ids(), before.ids());
}

#[test]
fn moving_or_deleting_an_axial_part_takes_its_radial_parts_along() {
    let (mut app, [nose, tube, transition]) = app();
    let id = app.ctx.doc.edit("Lug", |r| r.attach_radial(tube, 0.3, 0.5, 2, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap());
    // Grab the tube: its radial part rides along (out of the rocket) and returns with it, offset unchanged.
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    assert!(app.ctx.doc.rocket().radials().is_empty());
    app.update(Message::Cancel);
    assert_eq!(app.ctx.doc.rocket().radial(id).map(|r| (r.parent, r.offset)), Some((tube, 0.3)));
    // Move the transition forward of the tube: the tube's radial part is unaffected.
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    vp(&mut app, Vp::SnapNode(Some(1)));
    vp(&mut app, Vp::DropAxial);
    assert_eq!(app.ctx.doc.rocket().index_of(tube), Some(2));
    assert_eq!(app.ctx.doc.rocket().radial(id).map(|r| r.parent), Some(tube));
    // Resizing the parent keeps the radial part on it.
    vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag face")));
    vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.2)));
    vp(&mut app, Vp::Edit(Edit::End));
    assert!((app.ctx.doc.rocket().radial(id).unwrap().offset - 0.15).abs() < 1e-9);
    // Deleting the tube deletes its radial part.
    vp(&mut app, Vp::Select(Some(Pick::Axial(tube))));
    app.update(Message::Delete);
    assert!(app.ctx.doc.rocket().radials().is_empty() && app.ctx.doc.rocket().index_of(nose) == Some(0));
}

#[test]
fn resizing_a_fin_set_changes_the_group() {
    let (mut app, [_, tube, _]) = app();
    let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 };
    let id = app.ctx.doc.edit("Fins", |r| r.attach_radial(tube, 0.0, 0.0, 4, fins).unwrap());
    vp(&mut app, Vp::Edit(Edit::Begin(id, "Drag face")));
    vp(&mut app, Vp::Edit(Edit::Dim(id, Dim::Span, 0.09)));
    vp(&mut app, Vp::Edit(Edit::End));
    let radial = app.ctx.doc.rocket().radial(id).unwrap();
    assert_eq!((radial.kind.fin_span(), radial.count), (Some(0.09), 4));
}
