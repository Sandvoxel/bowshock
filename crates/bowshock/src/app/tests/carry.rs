use super::*;

#[test]
fn a_fin_set_pulled_off_the_body_is_removed_on_empty_space_or_lands_on_another_tube() {
    let (mut app, [_, tube, transition]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let (before, was) = (app.ctx.doc.rocket().clone(), committed(&app));
    // Pulled free of the body over empty space: the preview is the rocket without them, released they are removed (one
    // undo step, with a notice); the status strip gets the CP of the finless rocket while it is carried.
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    assert_eq!(app.ctx.drag.landing(), None);
    vp(&mut app, Vp::Surface(None, true));
    assert_eq!(app.ctx.drag.landing(), Some(drag::Landing::Removed));
    assert!(app.ctx.shown().now.is_some_and(|b| (b.cp - was.cp).abs() > 1e-6), "the preview is the rocket without its fins");
    vp(&mut app, Vp::DropRadial);
    assert!(app.ctx.doc.rocket().radial(fins).is_none() && app.ctx.selection == Selection::None);
    assert!(app.ctx.notice.as_ref().is_some_and(|(text, _)| text.starts_with("Removed Fin set")), "{:?}", app.ctx.notice);
    assert_eq!(app.ctx.doc.undo_label(), Some("Remove fin set"));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);

    // Esc after pulling free puts them back exactly, with no notice.
    app.ctx.notice = None;
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    vp(&mut app, Vp::Surface(None, true));
    app.update(Message::Cancel);
    assert_eq!(app.ctx.doc.rocket(), &before);
    assert!(app.ctx.notice.is_none());

    // Back within reach of the body (not detached any more) with no target: it goes back as it was, not removed.
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    vp(&mut app, Vp::Surface(None, true));
    vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 0.0)), false));
    vp(&mut app, Vp::DropRadial);
    assert_eq!(app.ctx.doc.rocket().radial(fins).map(|r| (r.parent, r.offset)), Some((tube, 0.2)));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);

    // Pulled free and dropped on another tube (a part of the core, or of a pod): it lands there, one undo step.
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    vp(&mut app, Vp::Surface(None, true));
    vp(&mut app, Vp::Surface(Some(target(transition, 0.0, 2.0)), true));
    vp(&mut app, Vp::DropRadial);
    assert_eq!(app.ctx.doc.rocket().radial(fins).map(|r| (r.parent, r.angle, r.count)), Some((transition, 2.0, 3)));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    let (_, pod_tube) = bare_pod(&mut app, tube);
    let with_pod = app.ctx.doc.rocket().clone();
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    vp(&mut app, Vp::Surface(None, true));
    vp(&mut app, Vp::Surface(Some(target(pod_tube, 0.05, 1.0)), true));
    vp(&mut app, Vp::DropRadial);
    assert_eq!(app.ctx.doc.rocket().parent_of(fins), Some(pod_tube));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &with_pod);
}

#[test]
fn a_motor_mount_carried_from_the_nose_into_the_body_keeps_its_station_and_is_one_undo_step() {
    let (mut app, ids) = app();
    let [nose, tube, _] = ids;
    let mount = mount_in_the_nose(&mut app, ids);
    let (before, was) = (app.ctx.doc.rocket().clone(), committed(&app));
    assert_eq!(before.internal(mount).map(|p| p.parent), Some(nose));
    // Pressed on and dragged: the part is selected and one undo step opens; the tube around it is not touched.
    vp(&mut app, Vp::Edit(Edit::Carry(mount)));
    assert_eq!(app.ctx.selection, Selection::Part(mount));
    assert!(app.ctx.drag.axial.is_none() && app.ctx.drag.radial.is_none() && app.ctx.doc.dragging());
    // Over the tube (station 0.2 to 0.7), 0.35 further aft: the station is kept, the offset is from the tube's fore face.
    vp(&mut app, Vp::Edit(Edit::Place(mount, tube, 0.18)));
    let moved = app.ctx.doc.rocket().internal(mount).unwrap().clone();
    assert_eq!((moved.parent, moved.offset), (tube, 0.18));
    assert!((app.ctx.doc.rocket().part_station(mount).unwrap() - 0.38).abs() < 1e-12);
    // A motor mount is placed from the bottom of its body part, wherever it goes.
    assert_eq!(moved.anchor, Some(bowshock_model::Anchor::Bottom));
    // The CG moves live, before the release.
    assert!((committed(&app).cg - was.cg).abs() > 1e-6 && app.ctx.changing().cg);
    vp(&mut app, Vp::Edit(Edit::End));
    assert_eq!(app.ctx.doc.undo_label(), Some("Move motor mount"));
    assert_eq!(app.ctx.doc.rocket().internal(mount).map(|p| p.parent), Some(tube));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    assert!((committed(&app).cg - was.cg).abs() < 1e-12);
    // Escape puts it back exactly.
    vp(&mut app, Vp::Edit(Edit::Carry(mount)));
    vp(&mut app, Vp::Edit(Edit::Place(mount, tube, 0.18)));
    vp(&mut app, Vp::Edit(Edit::Cancel));
    assert_eq!(app.ctx.doc.rocket(), &before);
}

#[test]
fn an_internal_part_dragged_from_the_bin_over_the_viewport_opens_the_airframe_and_lands_inside_the_body_part() {
    use crate::viewport::InsideView;
    let (mut app, [_, tube, _]) = app();
    let (before, was) = (app.ctx.doc.rocket().clone(), committed(&app));
    let ring = bowshock_model::InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
    let mount = bowshock_model::InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.002, length: 0.15, motor_mount: true, overhang: 0.0 };
    let target = |offset| crate::app::drag::InternalTarget { parent: tube, offset };
    assert_eq!(app.viewport.inside(), InsideView::Solid);
    // A centering ring: the view goes to X-ray, the preview moves the CG, the drop inserts it (anchored to the nearer end).
    drag::start_internal(&mut app.ctx, Some(0), "Centering ring", ring);
    app.after_update();
    assert_eq!(app.viewport.inside(), InsideView::XRay);
    vp(&mut app, Vp::InternalTarget(Some(target(0.05))));
    assert!(app.ctx.shown().now.is_some_and(|b| (b.cg - was.cg).abs() > 1e-9 && b.mass > was.mass));
    vp(&mut app, Vp::DropInternal);
    let [placed] = app.ctx.doc.rocket().internals() else { panic!("one internal part") };
    assert_eq!((placed.parent, placed.offset, placed.anchor), (tube, 0.05, Some(bowshock_model::Anchor::Top)));
    assert_eq!(app.viewport.inside(), InsideView::Solid, "the view it was before");
    assert_eq!(app.ctx.selection, Selection::Part(placed.id));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    // A motor mount is anchored to the bottom.
    drag::start_internal(&mut app.ctx, Some(1), "Motor mount", mount);
    app.after_update();
    vp(&mut app, Vp::InternalTarget(Some(target(0.3))));
    vp(&mut app, Vp::DropInternal);
    assert_eq!(app.ctx.doc.rocket().internals()[0].anchor, Some(bowshock_model::Anchor::Bottom));
    app.update(Message::Undo);
    // Esc adds nothing and puts the view back to what it was (here the cutaway).
    vp(&mut app, Vp::InsideView(InsideView::Cutaway));
    drag::start_internal(&mut app.ctx, Some(0), "Bulkhead", bowshock_model::InternalKind::Bulkhead { diameter: None, thickness: 0.006 });
    app.after_update();
    vp(&mut app, Vp::InternalTarget(Some(target(0.1))));
    app.update(Message::Cancel);
    assert!(app.ctx.doc.rocket().internals().is_empty() && app.viewport.inside() == InsideView::Cutaway);
}

#[test]
fn a_fin_set_pulled_free_takes_the_symmetry_selection_as_its_count_when_it_lands_again() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let before = app.ctx.doc.rocket().clone();
    assert_eq!(before.radial(fins).unwrap().count, 3);
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    vp(&mut app, Vp::Surface(None, true));
    // X (or the wheel) steps the count and makes it the Symmetry selection; so does the footer.
    app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(true)));
    assert_eq!((app.ctx.drag.radial.as_ref().unwrap().count, app.ctx.drag.symmetry), (4, 4));
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(6)));
    assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 6);
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(4)));
    // Back onto the tube: the group lands with ×4, in one undo step.
    vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 0.0)), true));
    vp(&mut app, Vp::DropRadial);
    let landed = app.ctx.doc.rocket().radial(fins).unwrap();
    assert_eq!((landed.count, landed.offset, landed.parent), (4, 0.2, tube));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    assert_eq!(app.ctx.doc.rocket().radial(fins).unwrap().count, 3);
    // Still stuck (sliding): the footer does not recount a picked-up group.
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(8)));
    assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 3);
    app.update(Message::Cancel);
}

#[test]
fn pulling_a_fin_set_free_hands_it_the_symmetry_selection_without_pressing_anything() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(4)));
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    // Still stuck: the group keeps its own count.
    assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 3);
    // Fully pulled free: it takes the selection at once, and lands with it.
    vp(&mut app, Vp::Surface(None, true));
    assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 4);
    vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 0.0)), true));
    vp(&mut app, Vp::DropRadial);
    assert_eq!(app.ctx.doc.rocket().radial(fins).unwrap().count, 4);
}

#[test]
fn the_symmetry_footer_and_x_recount_a_selected_fin_set_in_place() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let before = app.ctx.doc.rocket().radial(fins).unwrap().clone();
    app.ctx.selection = Selection::Part(fins);
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(4)));
    let after = app.ctx.doc.rocket().radial(fins).unwrap().clone();
    assert_eq!((after.count, after.offset, after.parent), (4, before.offset, before.parent));
    app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(true)));
    assert_eq!(app.ctx.doc.rocket().radial(fins).unwrap().count, 6);
    app.update(Message::Undo);
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket().radial(fins).unwrap().count, 3);
}

#[test]
fn the_part_menu_opens_on_a_right_click_renames_and_deletes_in_one_undo_step() {
    use part_menu::Action;
    let (mut app, [_, tube, transition]) = app();
    app.ctx.cursor = Some(Point::new(300.0, 200.0));
    // The viewport: the right-click selects the part under it, then the menu opens at the cursor.
    app.update(Message::Viewport(Vp::PartMenu(Pick::Axial(tube))));
    assert_eq!(app.ctx.selection, Selection::Part(tube));
    app.update(Message::OpenPartMenu(tube));
    assert_eq!(app.ctx.part_menu, Some(part_menu::Open { part: tube, at: Point::new(300.0, 200.0) }));
    // Escape closes it.
    app.update(Message::Cancel);
    assert!(app.ctx.part_menu.is_none());
    // The viewport: a right-click on another part moves the selection to it.
    app.update(Message::Viewport(Vp::PartMenu(Pick::Axial(transition))));
    assert_eq!(app.ctx.selection, Selection::Part(transition));
    app.update(Message::OpenPartMenu(transition));
    assert_eq!(app.ctx.part_menu.map(|m| m.part), Some(transition));
    // Rename starts the inspector's inline rename (the F2 path): the commit renames the part.
    app.update(Message::PartMenu(Action::Rename));
    assert!(app.ctx.part_menu.is_none());
    app.update(Message::Inspector(inspector::Message::RenameInput("Adapter".to_string())));
    app.update(Message::Inspector(inspector::Message::RenameCommit));
    assert_eq!(app.ctx.doc.rocket().kind_of(transition).unwrap().name(), "Adapter");
    // Delete removes the part as one undo step.
    let before = app.ctx.doc.rocket().clone();
    app.update(Message::OpenPartMenu(transition));
    app.update(Message::PartMenu(Action::Delete));
    assert!(app.ctx.doc.rocket().kind_of(transition).is_none());
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
}

#[test]
fn the_scale_overlay_applies_one_undo_step() {
    let (mut app, [_, tube, _]) = app();
    let before = app.ctx.doc.rocket().clone();
    let length = before.length();
    app.update(Message::Scale(scale::Message::OpenWhole));
    assert!(app.ctx.scale.is_some());
    app.update(Message::Scale(scale::Message::Factor("2".into())));
    app.update(Message::Scale(scale::Message::Apply));
    assert!(app.ctx.scale.is_none());
    assert!((app.ctx.doc.rocket().length() - 2.0 * length).abs() < 1e-9);
    assert_eq!(app.ctx.doc.undo_label(), Some("Scale ×2.000"));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before, "one undo step restores the rocket");
    // An out-of-range factor cannot be applied; the parts form scales only the selection.
    app.update(Message::Scale(scale::Message::OpenWhole));
    app.update(Message::Scale(scale::Message::Factor("500".into())));
    app.update(Message::Scale(scale::Message::Apply));
    assert!(app.ctx.scale.is_some());
    app.update(Message::Cancel);
    assert!(app.ctx.scale.is_none());
    // Enter applies the open overlay; anything unrelated is blocked while it is open.
    let before = app.ctx.doc.rocket().clone();
    app.update(Message::Scale(scale::Message::OpenWhole));
    app.update(Message::Scale(scale::Message::Factor("2".into())));
    app.ctx.selection = Selection::Part(tube);
    app.update(Message::Delete);
    assert!(app.ctx.scale.is_some());
    assert_eq!(app.ctx.doc.rocket(), &before, "Delete is blocked while the scale overlay is open");
    app.update(Message::Key(KeyPress { key: keyboard::Key::Named(keyboard::key::Named::Enter), modifiers: keyboard::Modifiers::empty(), captured: false }));
    assert!(app.ctx.scale.is_none());
    assert!((app.ctx.doc.rocket().length() - 2.0 * length).abs() < 1e-9);
    app.update(Message::Undo);
    app.ctx.selection = Selection::Part(tube);
    app.update(Message::Scale(scale::Message::OpenParts));
    app.update(Message::Scale(scale::Message::Factor("2".into())));
    app.update(Message::Scale(scale::Message::Apply));
    assert!((app.ctx.doc.rocket().length() - (length + 0.5)).abs() < 1e-9);
}

#[test]
fn a_chosen_unit_set_is_kept_in_the_settings_file_and_not_in_the_rocket() {
    use crate::units::{Family, Quantity};
    let file = std::env::temp_dir().join(format!("bowshock_app_units_{}.json", std::process::id()));
    let _ = std::fs::remove_file(&file);
    let (mut app, _) = app();
    app.ctx.settings_file = Some(file.clone());
    let rocket_before = app.ctx.doc.rocket().clone();

    let mixed = UnitSystem::MM_G_S.with(Family::Length, "in").with(Family::Mass, "kg");
    app.update(Message::Units(mixed));
    assert_eq!(app.ctx.units.label(), "in · kg · s");
    // Lengths read in inches and masses in kilograms through the same set.
    assert_eq!(ui::with_unit(0.0762, Quantity::Length, app.ctx.units), "3.00 in");
    assert_eq!(ui::with_unit(1.284, Quantity::Mass, app.ctx.units), "1.284 kg");
    // The next launch reads the choice back; the document is untouched (not dirty from a display preference).
    assert_eq!(settings::Settings::load(Some(&file)).units, mixed);
    assert_eq!(app.ctx.doc.rocket(), &rocket_before);

    // A preset replaces it, and so does a project-wide chip choice (cleared when the set changes).
    crate::units::set_unit_override(Quantity::Length, &Quantity::Length.units()[4]);
    app.update(Message::Units(UnitSystem::M_KG_S));
    assert_eq!(Quantity::Length.unit_for(app.ctx.units).symbol, "m");
    assert_eq!(settings::Settings::load(Some(&file)).units, UnitSystem::M_KG_S);
    let _ = std::fs::remove_file(&file);
}
