use super::*;

#[test]
fn an_edit_that_pushes_the_rocket_out_of_an_untouched_view_refits_it() {
    let (mut app, [nose, ..]) = fitted();
    assert!(!app.viewport.out_of_view());
    let was = app.viewport.camera().scale;
    lengthen_nose(&mut app, nose);
    // The refit is the usual animated one: it lands with the whole rocket in the view, zoomed out from before.
    app.viewport.tick(std::time::Instant::now() + std::time::Duration::from_secs(5));
    assert!(!app.viewport.out_of_view(), "the longer rocket is framed");
    assert!(app.viewport.camera().scale < was, "zoomed out to take it in");
    assert!(app.viewport.hints(&app.ctx).is_empty(), "nothing to hint: it was refitted for the user");
}

#[test]
fn after_the_user_zoomed_an_edit_leaves_the_view_alone_offers_fit_and_zooming_out_still_works() {
    let (mut app, [nose, ..]) = fitted();
    // Zoom out a little by hand (all of the rocket still shows), then grow the rocket beyond the edge.
    land(&mut app, Vp::Zoom { factor: 0.9, offset: iced::Vector::new(-100.0, 0.0) });
    assert!(!app.viewport.out_of_view());
    let hand = app.viewport.camera();
    lengthen_nose(&mut app, nose);
    app.viewport.tick(std::time::Instant::now() + std::time::Duration::from_secs(5));
    assert_eq!(app.viewport.camera(), hand, "the view is the user's, not refitted under them");
    assert!(app.viewport.out_of_view());
    assert!(app.viewport.hints(&app.ctx).contains(&("F", "fit")), "the hint bar offers fit");

    // Zoom out about the nose end, as the wheel does: it is not limited by the old size of the rocket.
    let mut notches = 0;
    while app.viewport.out_of_view() {
        let before = app.viewport.camera().scale;
        land(&mut app, Vp::Zoom { factor: 1.0 / 1.2, offset: iced::Vector::new(-300.0, 0.0) });
        assert!(app.viewport.camera().scale < before, "each notch zooms out");
        notches += 1;
        assert!(notches < 30, "never came into view");
    }
    assert!(app.viewport.hints(&app.ctx).is_empty(), "the hint goes once the whole rocket is in view");

    // Fit (the `F` key) frames it again whatever was done.
    land(&mut app, Vp::Fit(iced::Size::new(900.0, 560.0)));
    assert!(!app.viewport.out_of_view());
}

#[test]
fn a_pinch_over_the_viewport_zooms_about_the_cursor_and_two_finger_scroll_does_not() {
    use crate::widgets::pinch::Phase as GesturePhase;
    let (app, _) = app();
    let mut sim = iced_test::Simulator::with_size(iced::Settings::default(), iced::Size::new(1280.0, 800.0), app.view());
    sim.point_at(Point::new(640.0, 400.0));
    // The viewport's first event only reports its size.
    let _ = sim.simulate([iced::Event::Mouse(mouse::Event::CursorMoved { position: Point::new(640.0, 400.0) })]);
    let _ = sim.drain().count();
    let pinch = |delta| iced::Event::Mouse(mouse::Event::PinchGesture { delta, phase: GesturePhase::Moved });
    let _ = sim.simulate([pinch(0.25), pinch(-0.25)]);
    let zooms: Vec<f32> = sim
        .drain()
        .filter_map(|m| match m {
            Message::Viewport(Vp::Zoom { factor, .. }) => Some(factor),
            _ => None,
        })
        .collect();
    assert_eq!(zooms.len(), 2, "one zoom per pinch event: {zooms:?}");
    assert!(zooms[0] > 1.0 && zooms[1] < 1.0, "spreading zooms in, closing zooms out: {zooms:?}");
    assert!((zooms[0] * zooms[1] - 1.0).abs() < 1e-6);

    // Two-finger scroll (pixel deltas, no modifier) still orbits.
    let _ = sim.scroll(mouse::ScrollDelta::Pixels { x: 6.0, y: 3.0 });
    let moves: Vec<Message> = sim.drain().collect();
    assert!(moves.iter().any(|m| matches!(m, Message::Viewport(Vp::Orbit(_)))), "{moves:?}");
    assert!(!moves.iter().any(|m| matches!(m, Message::Viewport(Vp::Zoom { .. }))));
}
