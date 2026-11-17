//! Placing a carried part: the free ghost, snapping by the part's ends, click to place, and the parts bin as a way to delete.

use super::*;
use crate::app::Mode;
use crate::viewport::assembly::{self, End};
use crate::viewport::scene::GHOST;
use crate::viewport::{LAYER_INTERNAL, LAYER_RADIAL, LAYER_SHELL};
use bowshock_model::InternalKind;

const TUBE: PartKind = PartKind::BodyTube { length: 0.3, diameter: 0.1 };

fn tube_row(app: &mut App) {
    drag::start_axial(&mut app.ctx, 5, "Body tube", TUBE);
    app.after_update();
}

fn ghosted(app: &App, layer: f32) -> Vec<[f32; 3]> {
    app.viewport.mesh().vertices.iter().filter(|v| v.state == GHOST && v.layer == layer).map(|v| v.position).collect()
}

fn click_mode(app: &mut App) {
    app.ctx.set_click_to_place(true);
}

/// A bulkhead in `tube`, 50 mm behind its fore face.
fn bulkhead(app: &mut App, tube: PartId) -> PartId {
    let id = app.ctx.doc.edit("Bulkhead", |r| r.attach_internal(tube, 0.05, InternalKind::Bulkhead { diameter: None, thickness: 0.006 }).unwrap());
    app.after_update();
    id
}

fn over_bin(app: &mut App, over: bool) {
    app.update(Message::PartsBin(parts_bin::Message::Hover(over)));
}

#[test]
fn every_carried_part_from_the_bin_has_a_ghost_in_the_air_before_it_snaps() {
    let (mut app, _) = fitted();
    let (parts_before, min, max) = (app.viewport.mesh().parts.len(), app.viewport.mesh().min, app.viewport.mesh().max);
    assert!(ghosted(&app, LAYER_SHELL).is_empty());

    // An axial part: its own ghost at the cursor, sized as it is, outside the parts, the bounds and picking.
    tube_row(&mut app);
    assert!(ghosted(&app, LAYER_SHELL).is_empty(), "no ghost without a cursor over the viewport");
    app.update(Message::Cursor(Point::new(450.0, 300.0)));
    let here = ghosted(&app, LAYER_SHELL);
    assert!(!here.is_empty(), "the part is in hand");
    let mesh = app.viewport.mesh();
    assert_eq!((mesh.parts.len(), mesh.min, mesh.max), (parts_before, min, max));
    app.update(Message::Cursor(Point::new(300.0, 200.0)));
    assert_ne!(ghosted(&app, LAYER_SHELL), here, "it follows the cursor");
    // Off the viewport it is gone; snapped to a node it is the snapped ghost instead.
    app.update(Message::Cursor(Point::new(2000.0, 300.0)));
    assert!(ghosted(&app, LAYER_SHELL).is_empty());
    app.update(Message::Cursor(Point::new(450.0, 300.0)));
    vp(&mut app, Vp::SnapNode(Some(3)));
    app.after_update();
    assert_ne!(ghosted(&app, LAYER_SHELL), here);
    assert_eq!(app.viewport.mesh().parts.len(), parts_before, "the snapped ghost is not listed either");
    app.update(Message::Cancel);

    // A radial group: the copies stand round an axis at the cursor, without the tube they are drawn on.
    let fins = RadialKind::FinSet { root_chord: 0.08, tip_chord: 0.04, span: 0.06, sweep: 0.03, thickness: 0.003 };
    drag::start_radial(&mut app.ctx, 0, fins);
    app.update(Message::Cursor(Point::new(450.0, 300.0)));
    assert!(!ghosted(&app, LAYER_RADIAL).is_empty());
    assert!(ghosted(&app, LAYER_SHELL).is_empty());
    // Over a valid surface the group is on the surface (still a ghost), not also in the air.
    let on_surface = ghosted(&app, LAYER_RADIAL).len();
    let tube = app.ctx.doc.rocket().parts()[1].id;
    vp(&mut app, Vp::Surface(Some(target(tube, 0.1, 0.0)), false));
    app.after_update();
    assert_eq!(ghosted(&app, LAYER_RADIAL).len(), on_surface);
    app.update(Message::Cancel);

    // An internal part: itself, in the air, auto-sized.
    let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
    drag::start_internal(&mut app.ctx, Some(0), "Centering ring", ring);
    app.update(Message::Cursor(Point::new(450.0, 300.0)));
    assert!(!ghosted(&app, LAYER_INTERNAL).is_empty());
    assert!(ghosted(&app, LAYER_SHELL).is_empty());
}

#[test]
fn the_ghost_in_the_air_is_the_free_pose_the_snapping_works_from() {
    let (mut app, _) = fitted();
    tube_row(&mut app);
    let cursor = Point::new(500.0, 250.0);
    app.update(Message::Cursor(cursor));
    let drag = app.ctx.drag.axial.clone().unwrap();
    assert_eq!(drag.hold, 0.15, "a part from the bin is held by its middle");
    let pose = assembly::free_pose(&app.viewport.camera(), iced::Size::new(900.0, 560.0), cursor, drag.kind.length(), drag.hold).unwrap();
    let shell = ghosted(&app, LAYER_SHELL);
    let (fore, aft) = shell.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| (lo.min(p[2]), hi.max(p[2])));
    // The ghost's faces are inset a hair from the ends: the same pose to a fraction of a millimetre.
    assert!((fore - pose.fore.z).abs() < 5e-4 && (aft - pose.aft.z).abs() < 5e-4, "drawn {fore}..{aft}, pose {}..{}", pose.fore.z, pose.aft.z);
    assert!(pose.end(End::Aft).z > pose.end(End::Fore).z);
}

#[test]
fn a_part_is_held_where_it_was_pressed() {
    let (mut app, [_, tube, _]) = app();
    // Pressed 0.1 m behind the tube's fore face: that is where it is held once it is picked up.
    vp(&mut app, Vp::Held(tube, 0.1));
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    assert_eq!(app.ctx.drag.axial.as_ref().map(|d| d.hold), Some(0.1));
    app.update(Message::Cancel);
    // A press on another part, or none, holds it by its middle.
    let nose = app.ctx.doc.rocket().parts()[0].id;
    vp(&mut app, Vp::Held(nose, 0.1));
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    assert_eq!(app.ctx.drag.axial.as_ref().map(|d| d.hold), Some(0.25));
    app.update(Message::Cancel);
    // A press beyond the part is limited to it.
    vp(&mut app, Vp::Held(tube, 9.0));
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    assert_eq!(app.ctx.drag.axial.as_ref().map(|d| d.hold), Some(0.5));
    app.update(Message::Cancel);
}

#[test]
fn with_drag_and_drop_the_release_places_the_part_and_a_press_does_nothing() {
    let (mut app, _) = app();
    assert!(!app.ctx.click_to_place, "drag and drop is the default");
    tube_row(&mut app);
    vp(&mut app, Vp::SnapNode(Some(3)));
    vp(&mut app, Vp::PlaceHere);
    assert!(app.ctx.drag.axial.is_some() && app.ctx.doc.rocket().parts().len() == 3);
    vp(&mut app, Vp::DropAxial);
    assert!(app.ctx.drag.axial.is_none() && app.ctx.doc.rocket().parts().len() == 4);
    assert_eq!(app.ctx.doc.undo_label(), Some("Add body tube"));
    // Released over nothing that takes it (the inspector, say), it goes back.
    tube_row(&mut app);
    vp(&mut app, Vp::SnapNode(Some(3)));
    app.update(Message::Released);
    assert!(app.ctx.drag.axial.is_none() && app.ctx.doc.rocket().parts().len() == 4);
}

#[test]
fn click_to_place_keeps_the_part_in_hand_on_release_and_a_press_with_a_snap_places_it_in_one_undo_step() {
    let (mut app, _) = app();
    click_mode(&mut app);
    let before = app.ctx.doc.rocket().clone();
    tube_row(&mut app);
    vp(&mut app, Vp::SnapNode(Some(3)));
    // Released over the viewport, and over anything else: still in hand, nothing placed.
    vp(&mut app, Vp::DropAxial);
    app.update(Message::Released);
    assert!(app.ctx.drag.axial.is_some() && app.ctx.doc.rocket() == &before);
    // A press places it where it snaps: one undo step.
    vp(&mut app, Vp::PlaceHere);
    assert!(app.ctx.drag.axial.is_none());
    assert_eq!(app.ctx.doc.rocket().parts().len(), 4);
    assert_eq!(app.ctx.doc.undo_label(), Some("Add body tube"));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    assert_ne!(app.ctx.doc.undo_label(), Some("Add body tube"), "one step, not two");
}

#[test]
fn click_to_place_with_no_snap_does_nothing_and_escape_puts_a_picked_up_part_back_exactly() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let before = app.ctx.doc.rocket().clone();
    click_mode(&mut app);
    tube_row(&mut app);
    vp(&mut app, Vp::PlaceHere);
    assert!(app.ctx.drag.axial.is_some() && app.ctx.doc.rocket() == &before, "no target: the press does nothing and the part stays in hand");
    app.update(Message::Cancel);
    assert!(app.ctx.drag.axial.is_none() && app.ctx.doc.rocket() == &before);

    // A part picked up off the rocket: released it stays in hand; Esc puts it back exactly, fins and all.
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    vp(&mut app, Vp::SnapNode(Some(1)));
    vp(&mut app, Vp::DropAxial);
    assert!(app.ctx.drag.axial.is_some() && app.ctx.doc.rocket().part(tube).is_none());
    app.update(Message::Cancel);
    assert_eq!(app.ctx.doc.rocket(), &before);
    assert!(app.ctx.doc.rocket().radial(fins).is_some());

    // Radial and internal parts follow the same rules.
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    vp(&mut app, Vp::DropRadial);
    vp(&mut app, Vp::PlaceHere);
    assert!(app.ctx.drag.radial.is_some(), "no target");
    vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 0.0)), false));
    vp(&mut app, Vp::DropRadial);
    assert!(app.ctx.drag.radial.is_some(), "released: still in hand");
    vp(&mut app, Vp::PlaceHere);
    assert!(app.ctx.drag.radial.is_none() && app.ctx.doc.rocket().radial(fins).map(|r| r.offset) == Some(0.2));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    // A group pulled free over empty space is removed by a release in drag and drop; by a click it is not (the bin deletes).
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    vp(&mut app, Vp::Surface(None, true));
    vp(&mut app, Vp::PlaceHere);
    assert!(app.ctx.drag.radial.is_some() && app.ctx.doc.rocket().radial(fins).is_none());
    app.update(Message::Cancel);

    let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
    drag::start_internal(&mut app.ctx, Some(0), "Centering ring", ring);
    vp(&mut app, Vp::PlaceHere);
    assert!(app.ctx.drag.internal.is_some());
    vp(&mut app, Vp::InternalTarget(Some(crate::app::drag::InternalTarget { parent: tube, offset: 0.05 })));
    vp(&mut app, Vp::DropInternal);
    assert!(app.ctx.drag.internal.is_some());
    vp(&mut app, Vp::PlaceHere);
    assert!(app.ctx.drag.internal.is_none() && app.ctx.doc.rocket().internals().len() == 1);
}

#[test]
fn a_part_in_hand_is_put_back_by_leaving_build_and_by_switching_the_setting() {
    let (mut app, _) = app();
    click_mode(&mut app);
    let before = app.ctx.doc.rocket().clone();
    tube_row(&mut app);
    app.update(Message::Mode(Mode::Simulate));
    assert!(app.ctx.drag.axial.is_none() && app.ctx.doc.rocket() == &before && !app.ctx.doc.dragging());
    app.update(Message::Mode(Mode::Build));
    tube_row(&mut app);
    app.ctx.set_click_to_place(false);
    assert!(app.ctx.drag.axial.is_none() && !app.ctx.doc.dragging());
}

#[test]
fn the_hint_names_the_click_and_the_escape() {
    let (mut app, _) = app();
    click_mode(&mut app);
    tube_row(&mut app);
    assert_eq!(app.viewport.hints(&app.ctx), vec![("", "Click to place · Esc to put back")]);
    app.update(Message::Cancel);
    assert!(app.viewport.hints(&app.ctx).is_empty());
}

#[test]
fn a_click_on_a_bin_row_picks_the_part_up_and_a_click_on_the_bin_puts_it_back() {
    let (mut app, _) = app();
    click_mode(&mut app);
    let before = app.ctx.doc.rocket().clone();
    let row = parts_bin::RowId::new(parts_bin::Category::Airframe, 5);
    app.update(Message::PartsBin(parts_bin::Message::RowPress(row)));
    app.update(Message::Released);
    assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.row == Some(5) && d.origin.is_none()));
    app.update(Message::Released);
    assert!(app.ctx.drag.axial.is_some(), "the second report of the same release changes nothing");
    assert_eq!(app.ctx.doc.rocket(), &before);
    // A press on the bin discards it: nothing changed, no undo step.
    over_bin(&mut app, true);
    let undo = app.ctx.doc.undo_label().map(str::to_string);
    app.update(Message::PartsBin(parts_bin::Message::RowPress(row)));
    app.update(Message::Released);
    assert!(app.ctx.drag.axial.is_none() && app.ctx.doc.rocket() == &before);
    assert_eq!(app.ctx.doc.undo_label().map(str::to_string), undo);
    // With drag and drop a click still adds the part at once.
    app.ctx.set_click_to_place(false);
    app.update(Message::PartsBin(parts_bin::Message::RowPress(row)));
    app.update(Message::Released);
    assert_eq!(app.ctx.doc.rocket().parts().len(), before.parts().len() + 1);
}

#[test]
fn a_part_from_the_bin_dropped_back_on_the_bin_changes_nothing() {
    let (mut app, _) = app();
    let before = app.ctx.doc.rocket().clone();
    let undo = app.ctx.doc.undo_label().map(str::to_string);
    tube_row(&mut app);
    over_bin(&mut app, true);
    app.update(Message::Released);
    assert!(app.ctx.drag.axial.is_none() && app.ctx.doc.rocket() == &before);
    assert_eq!(app.ctx.doc.undo_label().map(str::to_string), undo, "no undo step");
    assert!(!app.ctx.doc.dragging());
    // The same for radial and internal rows.
    drag::start_radial(&mut app.ctx, 0, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 });
    app.update(Message::Released);
    assert!(app.ctx.drag.radial.is_none() && app.ctx.doc.rocket() == &before);
    drag::start_internal(&mut app.ctx, Some(0), "Bulkhead", InternalKind::Bulkhead { diameter: None, thickness: 0.006 });
    app.update(Message::Released);
    assert!(app.ctx.drag.internal.is_none() && app.ctx.doc.rocket() == &before);
    assert_eq!(app.ctx.doc.undo_label().map(str::to_string), undo);
}

#[test]
fn a_picked_up_finned_tube_dropped_on_the_bin_is_deleted_with_all_it_carries_in_one_undo_step() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let bulkhead = bulkhead(&mut app, tube);
    let before = app.ctx.doc.rocket().clone();
    let last_step = app.ctx.doc.undo_label().map(str::to_string);
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    assert!(app.ctx.drag.axial.is_some());
    // The cue while it is over the bin.
    over_bin(&mut app, true);
    app.update(Message::Released);
    let rocket = app.ctx.doc.rocket();
    assert!(rocket.part(tube).is_none() && rocket.radial(fins).is_none() && rocket.internal(bulkhead).is_none());
    assert_eq!(rocket.parts().len(), 2);
    assert!(app.ctx.drag.axial.is_none() && app.ctx.selection == Selection::None && !app.ctx.doc.dragging());
    assert_eq!(app.ctx.doc.undo_label(), Some("Delete part"), "the label Delete uses");
    // One Undo restores the rocket exactly, attachments included; and it was one step.
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    assert_eq!(app.ctx.doc.undo_label().map(str::to_string), last_step);

    // What Delete does is the same: the same label, the same rocket.
    vp(&mut app, Vp::Select(Some(Pick::Axial(tube))));
    app.update(Message::Delete);
    let by_delete = app.ctx.doc.rocket().clone();
    assert_eq!(app.ctx.doc.undo_label(), Some("Delete part"));
    app.update(Message::Undo);
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    app.update(Message::Released);
    assert_eq!(app.ctx.doc.rocket(), &by_delete);
}

#[test]
fn a_picked_up_fin_set_or_internal_part_dropped_on_the_bin_is_deleted_too() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let mount = bulkhead(&mut app, tube);
    let before = app.ctx.doc.rocket().clone();
    over_bin(&mut app, true);
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    app.update(Message::Released);
    assert!(app.ctx.doc.rocket().radial(fins).is_none() && app.ctx.selection == Selection::None);
    assert_eq!(app.ctx.doc.undo_label(), Some("Delete part"));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    // An internal part is carried along the rocket where it stands; let go over the bin it is deleted, not placed.
    over_bin(&mut app, false);
    vp(&mut app, Vp::Edit(Edit::Carry(mount)));
    vp(&mut app, Vp::Edit(Edit::Place(mount, tube, 0.2)));
    over_bin(&mut app, true);
    vp(&mut app, Vp::Edit(Edit::End));
    assert!(app.ctx.doc.rocket().internal(mount).is_none() && !app.ctx.doc.dragging() && !app.ctx.drag.active());
    assert_eq!(app.ctx.doc.undo_label(), Some("Delete part"));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before, "back where it was, not where it was carried to");
    // Let go over the viewport it is placed as before.
    over_bin(&mut app, false);
    vp(&mut app, Vp::Edit(Edit::Carry(mount)));
    vp(&mut app, Vp::Edit(Edit::Place(mount, tube, 0.2)));
    vp(&mut app, Vp::Edit(Edit::End));
    assert_eq!(app.ctx.doc.rocket().internal(mount).map(|p| p.offset), Some(0.2));
    assert_eq!(app.ctx.doc.undo_label(), Some("Move bulkhead"));
}

#[test]
fn with_click_to_place_a_click_on_the_bin_deletes_a_picked_up_part() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let mount = bulkhead(&mut app, tube);
    let before = app.ctx.doc.rocket().clone();
    click_mode(&mut app);
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    over_bin(&mut app, true);
    // The release does nothing: it is in hand with the cue showing. The click deletes it.
    app.update(Message::Released);
    assert!(app.ctx.drag.axial.is_some());
    app.update(Message::PartsBin(parts_bin::Message::BinPress));
    let rocket = app.ctx.doc.rocket();
    assert!(rocket.part(tube).is_none() && rocket.radial(fins).is_none() && rocket.internal(mount).is_none());
    assert_eq!(app.ctx.doc.undo_label(), Some("Delete part"));
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    // A click on a row of the bin does the same.
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, 0))));
    assert!(app.ctx.doc.rocket().radial(fins).is_none() && app.ctx.drag.radial.is_none());
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
    // An internal part carried along the rocket is in hand all the same: the click on the bin deletes it.
    over_bin(&mut app, false);
    vp(&mut app, Vp::Edit(Edit::Carry(mount)));
    // The overlay sends nothing on the release (see its own test): the part is in hand.
    assert!(app.ctx.drag.lifted.is_some());
    over_bin(&mut app, true);
    app.update(Message::PartsBin(parts_bin::Message::BinPress));
    assert!(app.ctx.doc.rocket().internal(mount).is_none() && app.ctx.drag.lifted.is_none());
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket(), &before);
}

#[test]
fn the_bin_shows_what_letting_go_there_does() {
    let (mut app, [_, tube, _]) = app();
    // Nothing in hand: no cue.
    over_bin(&mut app, true);
    assert_eq!(app.parts_bin.cue_words(&app.ctx), None);
    // From the bin: put back (a neutral ground). Picked up off the rocket: delete (the err soft ground).
    tube_row(&mut app);
    assert_eq!(app.parts_bin.cue_words(&app.ctx), Some((parts_bin::DROP_TO_PUT_BACK, false)));
    over_bin(&mut app, false);
    assert_eq!(app.parts_bin.cue_words(&app.ctx), None, "only over the bin");
    app.update(Message::Cancel);
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    over_bin(&mut app, true);
    assert_eq!(app.parts_bin.cue_words(&app.ctx), Some((parts_bin::DROP_TO_DELETE, true)));
    assert_eq!(parts_bin::DROP_TO_DELETE, "Drop to delete");
    assert_eq!(parts_bin::DROP_TO_PUT_BACK, "Drop to put back");
}

#[test]
fn the_snap_is_decided_again_when_the_view_changes_under_a_cursor_that_has_not_moved() {
    use crate::viewport::Projection;
    let (mut app, _) = fitted();
    let size = iced::Size::new(900.0, 560.0);
    // Where the side view puts a tube's fore end on the tail node (0.8 m): the cursor is half the tube (0.05 m) aft of it.
    land(&mut app, Vp::Projection(Projection::Profile));
    let side = app.viewport.camera();
    let tail = side.project(glam::Vec3::new(0.0, 0.0, 0.8), size);
    let cursor = Point::new(tail.x + 0.05 * side.scale, tail.y);
    land(&mut app, Vp::Projection(Projection::ThreeD));
    click_mode(&mut app);
    drag::start_axial(&mut app.ctx, 5, "Body tube", PartKind::BodyTube { length: 0.1, diameter: 0.1 });
    app.after_update();
    app.update(Message::Cursor(cursor));
    assert_eq!(app.ctx.drag.axial.as_ref().map(|d| d.snap), Some(None), "no node is caught in the 3D view");
    // Nothing moves but the camera (no cursor event, no message from the overlay): the part is now held by its end at the tail.
    land(&mut app, Vp::Projection(Projection::Profile));
    app.sync();
    assert_eq!(app.ctx.drag.axial.as_ref().map(|d| d.snap), Some(Some(3)));
    // And back: the view changes again, the node is let go.
    land(&mut app, Vp::Projection(Projection::ThreeD));
    app.sync();
    assert_eq!(app.ctx.drag.axial.as_ref().map(|d| d.snap), Some(None));
}
