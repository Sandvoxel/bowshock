use super::*;

#[test]
fn the_wheel_mid_radial_drag_steps_the_count_and_does_not_zoom() {
    use iced::widget::canvas::Program;
    let (mut app, _) = app();
    app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
    app.update(Message::PartsBin(parts_bin::Message::Symmetry(3)));
    app.update(Message::Cursor(Point::new(100.0, 100.0)));
    app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 3))));
    app.update(Message::Cursor(Point::new(160.0, 100.0)));
    assert_eq!(app.ctx.drag.radial.as_ref().map(|d| d.count), Some(3));

    // The overlay that owns the viewport during the drag turns a wheel notch into a count step and captures the event, so
    // the camera's own wheel handler (zoom) never sees it.
    let camera = app.viewport.camera();
    let drag = app.ctx.drag.radial.clone().unwrap();
    let overlay = crate::viewport::surface::Overlay {
        rocket: app.ctx.doc.rocket(),
        drag: &drag,
        camera,
        snap: 0,
        snap_step: 0.001,
        cursor: None,
        margin: None,
        units: Default::default(),
        click: false,
    };
    let bounds = iced::Rectangle::new(Point::ORIGIN, iced::Size::new(800.0, 600.0));
    let cursor = mouse::Cursor::Available(Point::new(400.0, 300.0));
    let wheel = |y: f32| iced::Event::Mouse(mouse::Event::WheelScrolled { delta: mouse::ScrollDelta::Lines { x: 0.0, y } });
    let mut state = crate::viewport::surface::State::default();
    let (up, _, status) = overlay.update(&mut state, &wheel(1.0), bounds, cursor).expect("the overlay handles the wheel").into_inner();
    assert!(matches!(up, Some(Vp::CopiesStep(true))), "a notch up is the next count, not a zoom: {up:?}");
    assert_eq!(status, iced::event::Status::Captured);
    let (down, _, _) = overlay.update(&mut state, &wheel(-1.0), bounds, cursor).unwrap().into_inner();
    assert!(matches!(down, Some(Vp::CopiesStep(false))));

    // The viewport applies the step to the group in the air (and to the bin's count) and leaves the camera alone.
    vp(&mut app, up.unwrap());
    assert_eq!((app.ctx.drag.radial.as_ref().unwrap().count, app.ctx.drag.symmetry), (4, 4));
    vp(&mut app, down.unwrap());
    vp(&mut app, Vp::CopiesStep(false));
    assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 2);
    assert_eq!(app.viewport.camera(), camera, "the wheel did not zoom");
}

#[test]
fn readouts_that_a_handle_drag_changes_are_marked_until_the_release() {
    let (mut app, [_, tube, _]) = app();
    assert_eq!(app.ctx.changing(), live::Changed::default());
    vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag face")));
    assert_eq!(app.ctx.changing(), live::Changed::default(), "nothing has changed yet");
    vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.7)));
    let c = app.ctx.changing();
    assert!(c.mass && c.length && c.cg && c.cp, "a longer tube moves mass, length, CG and CP: {c:?}");
    assert!(!c.diameter, "the diameter did not move");
    // Dragged back to the start, the readout is regular again even though the drag goes on.
    vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.5)));
    assert_eq!(app.ctx.changing(), live::Changed::default());
    vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.6)));
    assert!(app.ctx.changing().length);
    app.update(Message::Released);
    vp(&mut app, Vp::Edit(Edit::End));
    assert_eq!(app.ctx.changing(), live::Changed::default(), "back to regular on release");
}

#[test]
fn moving_a_fin_set_previews_the_balance_before_the_release_and_the_release_commits_it() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let was = committed(&app);
    let health = app.ctx.physics.warnings().to_vec();
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    // Picked up with no target yet: the committed balance, nothing to compare, and the health of the rocket before the
    // drag (never that of the rocket with the fins out of it).
    assert_eq!(app.ctx.shown().now, Some(was));
    assert_eq!(app.ctx.shown().committed, None);
    assert_eq!(app.ctx.physics.warnings(), health.as_slice());
    vp(&mut app, Vp::Surface(Some(target(tube, 0.35, 0.0)), false));
    let aft = app.ctx.shown();
    assert_eq!(aft.committed, Some(was));
    let aft_now = aft.now.expect("a preview");
    assert!(aft_now.cp > was.cp, "fins moved aft move the CP aft: {} vs {}", aft_now.cp, was.cp);
    assert!(app.ctx.changing().cp, "the CP readout goes semibold");
    // Further forward than the start: the CP goes the other way.
    vp(&mut app, Vp::Surface(Some(target(tube, 0.0, 0.0)), false));
    assert!(app.ctx.shown().now.unwrap().cp < was.cp);
    // No valid target any more: the committed balance again.
    vp(&mut app, Vp::Surface(None, false));
    assert_eq!((app.ctx.shown().now, app.ctx.shown().committed), (Some(was), None));
    // The gauge follows what is shown.
    vp(&mut app, Vp::Surface(Some(target(tube, 0.35, 0.0)), false));
    assert!((f64::from(app.ctx.physics.gauge().target()) - aft_now.margin_cal).abs() < 1e-5);
    let previewed_health = app.ctx.physics.warnings().to_vec();
    // Release: the committed balance and health are the previewed ones.
    vp(&mut app, Vp::DropRadial);
    assert_eq!(app.ctx.physics.warnings(), previewed_health.as_slice());
    let after = committed(&app);
    assert!((after.cp - aft_now.cp).abs() < 1e-12 && (after.cg - aft_now.cg).abs() < 1e-12 && (after.mass - aft_now.mass).abs() < 1e-12);
    assert_eq!(app.ctx.shown(), live::Shown { now: Some(after), committed: None });
    assert_eq!(app.ctx.changing(), live::Changed::default());
    assert!((f64::from(app.ctx.physics.gauge().target()) - after.margin_cal).abs() < 1e-5);
}

#[test]
fn cancelling_a_carry_restores_the_committed_balance_everywhere() {
    let (mut app, [_, tube, _]) = app();
    let fins = finned(&mut app, tube, 0.1);
    let was = committed(&app);
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
    vp(&mut app, Vp::Surface(Some(target(tube, 0.35, 0.0)), false));
    assert_ne!(app.ctx.shown().now, Some(was));
    app.update(Message::Cancel);
    assert_eq!(committed(&app), was);
    assert_eq!(app.ctx.shown(), live::Shown { now: Some(was), committed: None });
    assert!((f64::from(app.ctx.physics.gauge().target()) - was.margin_cal).abs() < 1e-5);
}

#[test]
fn a_tube_dragged_in_from_the_bin_previews_its_node_and_shifts_the_cg() {
    let (mut app, [_, _, _]) = app();
    let was = committed(&app);
    let row = parts_bin::axial_catalogue().iter().position(|(n, _)| *n == "Body tube").unwrap();
    app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Airframe)));
    app.update(Message::Cursor(Point::new(100.0, 100.0)));
    app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, row))));
    app.update(Message::Cursor(Point::new(160.0, 100.0)));
    assert!(app.ctx.drag.axial.is_some());
    // No node: the committed balance.
    assert_eq!(app.ctx.shown().now, Some(was));
    // Between the nose and the tube: heavier up front, longer, the CG moves.
    vp(&mut app, Vp::SnapNode(Some(1)));
    let fore = app.ctx.shown();
    let fore_now = fore.now.expect("a preview");
    assert_eq!(fore.committed, Some(was));
    assert!(fore_now.mass > was.mass && fore_now.length > was.length && (fore_now.cg - was.cg).abs() > 1e-6);
    assert!(app.ctx.changing().mass && app.ctx.changing().cg);
    // The aft node gives a different CG than the fore node.
    vp(&mut app, Vp::SnapNode(Some(3)));
    let aft_now = app.ctx.shown().now.unwrap();
    assert!((aft_now.cg - fore_now.cg).abs() > 1e-6);
    // A pod on the tube's side previews too.
    let tube = app.ctx.doc.rocket().parts()[1].id;
    vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 0.0))));
    assert!(app.ctx.shown().now.is_some_and(|b| b.mass > was.mass));
    // Release at the fore node: committed equals the preview.
    vp(&mut app, Vp::SnapNode(Some(1)));
    vp(&mut app, Vp::DropAxial);
    let after = committed(&app);
    assert!((after.cg - fore_now.cg).abs() < 1e-12 && (after.cp - fore_now.cp).abs() < 1e-12 && (after.mass - fore_now.mass).abs() < 1e-12);
}

#[test]
fn a_picked_up_part_and_a_radial_pod_preview_where_they_would_land() {
    let (mut app, [_, tube, transition]) = app();
    let was = committed(&app);
    // An axial part moved to another node keeps its mass: only the CG and CP move.
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    vp(&mut app, Vp::SnapNode(Some(1)));
    let moved = app.ctx.shown().now.expect("a preview");
    assert!((moved.cg - was.cg).abs() > 1e-6, "the CG moves when the transition does");
    app.update(Message::Cancel);
    assert_eq!(committed(&app), was);
    vp(&mut app, Vp::Grab(Pick::Axial(transition)));
    vp(&mut app, Vp::SnapNode(Some(1)));
    vp(&mut app, Vp::DropAxial);
    assert!((committed(&app).cg - moved.cg).abs() < 1e-12 && (committed(&app).mass - moved.mass).abs() < 1e-12);
    app.update(Message::Undo);
    assert_eq!(committed(&app), was);
    // A pod's parts are carried with the pod: dragging it along the tube previews its CG.
    let pod = app.ctx.doc.edit("Pod", |r| {
        let body = bowshock_model::PodBody::single(bowshock_model::Part::new(0, PartKind::BodyTube { length: 0.2, diameter: 0.03 }), Vec::new(), Vec::new());
        r.attach_radial(tube, 0.0, 0.0, 2, RadialKind::Pod { gap: 0.0, body }).unwrap()
    });
    app.after_update();
    let with_pod = committed(&app);
    vp(&mut app, Vp::Grab(Pick::Radial { id: pod, instance: 0 }));
    vp(&mut app, Vp::Surface(Some(target(tube, 0.3, 0.0)), false));
    let shifted = app.ctx.shown().now.expect("a preview");
    assert!((shifted.mass - with_pod.mass).abs() < 1e-9 && shifted.cg > with_pod.cg);
    vp(&mut app, Vp::DropRadial);
    assert!((committed(&app).cg - shifted.cg).abs() < 1e-12);
}

/// The per-message cost of a preview on the two-stage example, release build: `cargo test --release -p bowshock
/// the_preview_costs -- --nocapture`.
#[test]
fn the_preview_costs_a_few_milliseconds_at_most() {
    let mut app = App::new();
    let rocket = bowshock_model::Rocket::from_bytes(include_bytes!("../../../assets/examples/two_stage.bowshock")).unwrap();
    app.ctx.doc.replace(rocket, None);
    app.ctx.loaded();
    app.after_update();
    let fins = app.ctx.doc.rocket().radials().iter().find(|r| matches!(r.kind, RadialKind::FinSet { .. })).expect("the example has fins").clone();
    vp(&mut app, Vp::Grab(Pick::Radial { id: fins.id, instance: 0 }));
    let mut total = std::time::Duration::ZERO;
    let steps = 40;
    for i in 0..steps {
        vp(&mut app, Vp::Surface(Some(target(fins.parent, 0.002 * f64::from(i), 0.0)), false));
        total += app.ctx.preview.cost;
        assert!(app.ctx.shown().committed.is_some(), "step {i} has a preview that differs from the committed balance");
    }
    let each = total / steps;
    eprintln!("preview per update (two_stage, {}): {each:?}", if cfg!(debug_assertions) { "debug" } else { "release" });
    assert!(app.ctx.shown().now.is_some());
    assert!(each < std::time::Duration::from_millis(200), "{each:?}");
    app.update(Message::Cancel);
}
