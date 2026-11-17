use super::*;

#[test]
fn select_and_delete() {
    let (mut app, [_, tube, _]) = app();
    vp(&mut app, Vp::Select(Some(Pick::Axial(tube))));
    assert_eq!(app.ctx.selection, Selection::Part(tube));
    app.update(Message::Delete);
    assert!(app.ctx.doc.rocket().part(tube).is_none() && app.ctx.selection == Selection::None);
    assert_eq!(app.ctx.doc.rocket().parts().len(), 2);
    // Clicking empty space deselects.
    vp(&mut app, Vp::Select(None));
    app.update(Message::Delete);
    assert_eq!(app.ctx.doc.rocket().parts().len(), 2);
    // The delete is one undo step, and the selection is dropped when undo removes what it pointed at.
    app.update(Message::Undo);
    assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
}

#[test]
fn grabbing_a_part_moves_it_and_cancelling_puts_it_back() {
    let (mut app, [nose, tube, transition]) = app();
    let before = parts(&app);
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    assert!(app.ctx.doc.rocket().part(tube).is_none() && app.ctx.drag.axial.is_some() && app.ctx.selection == Selection::Part(tube));
    app.update(Message::Cancel);
    assert_eq!(parts(&app), before);

    // No snap target on release: back where it was.
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    vp(&mut app, Vp::DropAxial);
    assert_eq!(parts(&app), before);

    // Dropped on a node: moved, keeping its id, and matched to its new neighbours.
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    vp(&mut app, Vp::SnapNode(Some(1)));
    assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.snap == Some(1)));
    vp(&mut app, Vp::DropAxial);
    let rocket = app.ctx.doc.rocket();
    assert_eq!(rocket.index_of(transition), Some(1));
    assert_eq!(rocket.index_of(nose), Some(0));
    assert_eq!(rocket.part(transition).unwrap().kind.fore_diameter(), 0.1);
    assert_eq!(rocket.part(transition).unwrap().kind.aft_diameter(), 0.1);
    assert!(app.ctx.drag.axial.is_none());
}

#[test]
fn a_drop_flashes_the_part_once_and_a_cancel_does_not() {
    let (mut app, [_, tube, transition]) = app();
    assert!(app.ctx.flash.is_none());
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    app.update(Message::Cancel);
    assert!(app.ctx.flash.is_none());
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    assert!(app.ctx.flash.is_none(), "nothing flashes while the part is carried");
    vp(&mut app, Vp::SnapNode(Some(1)));
    vp(&mut app, Vp::DropAxial);
    let (id, since) = app.ctx.flash.expect("the dropped part flashes");
    assert_eq!(id, transition);
    // The flash outlasts the 120 ms settle, runs out after 400 ms, and is cleared by the frame tick that sees it end.
    let levels = app.ctx.flash_levels().expect("flash running");
    assert_eq!(levels.0, transition);
    app.update(Message::Frame(since + crate::viewport::flash::SETTLE_TIME));
    assert!(app.ctx.flash.is_some());
    app.update(Message::Frame(since + crate::viewport::flash::FLASH_TIME));
    assert!(app.ctx.flash.is_none());
}

#[test]
fn only_a_committed_inspector_edit_flashes_the_part() {
    use crate::build::inspector::{Field, Message as Insp};
    let (mut app, [_, tube, _]) = app();
    vp(&mut app, Vp::Select(Some(Pick::Axial(tube))));
    let field = Field::Dim(bowshock_model::Dim::Length);
    app.update(Message::Inspector(Insp::Edit(field, 0.4, true)));
    assert!(app.ctx.flash.is_none(), "a scrub tick does not flash");
    app.update(Message::Inspector(Insp::Edit(field, 0.42, false)));
    assert_eq!(app.ctx.flash.map(|(id, _)| id), Some(tube));
}

#[test]
fn tab_folds_the_side_columns_in_build_but_not_while_a_field_captures_it() {
    let (mut app, _) = app();
    let tab = |captured| KeyPress { key: keyboard::Key::Named(keyboard::key::Named::Tab), modifiers: keyboard::Modifiers::empty(), captured };
    app.update(Message::Key(tab(true)));
    assert!(!app.layout.sides_hidden());
    app.update(Message::Key(tab(false)));
    assert!(app.layout.sides_hidden());
    app.update(Message::Key(tab(false)));
    assert!(!app.layout.sides_hidden());
}

#[test]
fn moving_a_part_keeps_its_material_and_undo_is_one_step() {
    let (mut app, [_, tube, transition]) = app();
    app.ctx.doc.edit("Name", |r| r.set_name(tube, Some("Payload".to_string())));
    app.ctx.doc.edit("Wall", |r| r.set_thickness(tube, 0.004));
    let before = parts(&app);
    vp(&mut app, Vp::Grab(Pick::Axial(tube)));
    vp(&mut app, Vp::SnapNode(Some(2)));
    vp(&mut app, Vp::DropAxial);
    let rocket = app.ctx.doc.rocket();
    assert_eq!(rocket.index_of(tube), Some(2));
    let moved = rocket.part(tube).unwrap();
    assert_eq!((moved.name.as_deref(), moved.thickness), (Some("Payload"), 0.004));
    assert_eq!(rocket.index_of(transition), Some(1));
    // The whole move is one undo step.
    app.update(Message::Undo);
    assert_eq!(parts(&app), before);
}

#[test]
fn resizing_changes_only_that_part_and_a_resize_drag_is_one_undo_step() {
    let (mut app, [_, tube, transition]) = app();
    let before = parts(&app);
    vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag face")));
    for length in [0.55, 0.6, 0.7] {
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, length)));
    }
    vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Diameter, 0.12)));
    vp(&mut app, Vp::Edit(Edit::End));
    let kind = PartKind::BodyTube { length: 0.7, diameter: 0.12 };
    assert_eq!(app.ctx.doc.rocket().part(tube).unwrap().kind, kind);
    // The transition behind it follows the tube's new diameter at their joint.
    assert_eq!(app.ctx.doc.rocket().part(transition).unwrap().kind.fore_diameter(), 0.12);
    // Cancelled, a drag puts everything back.
    vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag face")));
    vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.3)));
    vp(&mut app, Vp::Edit(Edit::Cancel));
    assert_eq!(app.ctx.doc.rocket().part(tube).unwrap().kind, kind);
    app.update(Message::Undo);
    assert_eq!(parts(&app), before);
}

#[test]
fn undo_and_redo_of_a_fore_end_drag_keep_the_aft_end_still_on_screen() {
    let (mut app, [_, tube, _]) = app();
    let start = app.viewport.camera().target.z;
    let before = app.ctx.doc.rocket().dim(tube, Dim::Length).unwrap();
    vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag fore face")));
    for length in [before + 0.05, before + 0.2] {
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::ForeLength, length)));
    }
    vp(&mut app, Vp::Edit(Edit::End));
    let grown = app.viewport.camera().target.z;
    let delta = (app.ctx.doc.rocket().dim(tube, Dim::Length).unwrap() - before) as f32;
    assert!(delta > 0.1 && (grown - start - delta).abs() < 1e-5, "the camera follows the aft end");
    app.update(Message::Undo);
    assert!((app.viewport.camera().target.z - start).abs() < 1e-5, "undo moves the camera back");
    app.update(Message::Redo);
    assert!((app.viewport.camera().target.z - grown).abs() < 1e-5, "redo moves it forward again");
}
