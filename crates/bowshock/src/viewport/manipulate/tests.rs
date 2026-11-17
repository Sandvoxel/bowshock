use super::super::handles::{self, Context};
use super::super::markers::{self, BalanceMarks};
use super::super::pick::{self, Pick};
use super::super::{Camera, InsideView, Message};
use super::{visible, Drag, Edit, Hand, Hover, Live, Overlay, State, Visible};
use crate::units::UnitSystem;
use bowshock_model::{Dim, PartId, Rocket};
use glam::Vec3;
use iced::widget::canvas::Event;
use iced::{keyboard, mouse, Point, Rectangle, Size, Vector};

#[cfg(test)]
mod edit_tests {
    use super::*;
    use bowshock_model::{InternalKind, NoseShape, PartKind, RadialKind};
    use std::f32::consts::FRAC_PI_2;

    const SIZE: Size = Size::new(900.0, 600.0);

    fn side() -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.4), yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 800.0, ortho_blend: 1.0 }
    }

    fn rocket() -> (Rocket, PartId, PartId, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::Haack(1.0 / 3.0), length: 0.2, diameter: 0.1, tip_radius: 0.0 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let fin =
            r.attach_radial(tube, 0.4, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.06, sweep: 0.04, thickness: 0.003 }).unwrap();
        (r, tube, fin, 0)
    }

    fn vis(r: &Rocket, id: PartId, all: bool, inside: InsideView) -> Visible {
        let cam = side();
        let b = match r.index_of(id) {
            Some(i) => pick::screen_bounds(&cam, SIZE, r.stations()[i], &r.parts()[i].kind),
            None => {
                let rad = r.radial(id).unwrap();
                pick::radial_bounds(&cam, SIZE, &rad.kind, pick::placement(r, rad, 0).unwrap())
            }
        };
        visible(&Context { rocket: r, camera: &cam, size: SIZE, instance: 0, bounds: b, cube: false, avoid: Vec::new() }, id, all, inside)
    }

    fn dims(v: &Visible) -> Vec<Dim> {
        v.placed.iter().map(|p| p.spec.dim).collect()
    }

    #[test]
    fn selecting_shows_primary_handles_and_alt_shows_the_rest() {
        let (r, tube, fin, _) = rocket();
        let d = dims(&vis(&r, tube, false, InsideView::Solid));
        assert!(d.contains(&Dim::Length) && d.contains(&Dim::Diameter) && !d.contains(&Dim::Wall));
        let d = dims(&vis(&r, fin, false, InsideView::Solid));
        assert!(d.contains(&Dim::Span) && d.contains(&Dim::RootChord) && !d.contains(&Dim::Thickness));
        let v = vis(&r, fin, true, InsideView::Solid);
        let d = dims(&v);
        assert!(d.contains(&Dim::Thickness) && d.contains(&Dim::Cant) && d.contains(&Dim::Angle));
        assert!(v.inset.is_some());
    }

    #[test]
    fn internal_dimensions_show_in_cutaway() {
        let (mut r, tube, _, _) = rocket();
        let ring = r.attach_internal(tube, 0.1, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.02 }).unwrap();
        assert!(!dims(&vis(&r, tube, true, InsideView::Solid)).contains(&Dim::Wall));
        assert!(dims(&vis(&r, tube, true, InsideView::Cutaway)).contains(&Dim::Wall));
        // A selected internal part always has its primary handles.
        assert!(dims(&vis(&r, tube, false, InsideView::Solid)).iter().all(|d| *d != Dim::InnerDiameter));
        let _ = ring;
    }

    #[test]
    fn a_tiny_part_has_no_handles_and_crowded_ones_hide() {
        let (mut r, tube, _, _) = rocket();
        let cam = side();
        let small = Rectangle::new(Point::new(300.0, 200.0), Size::new(10.0, 10.0));
        let cx = Context { rocket: &r, camera: &cam, size: SIZE, instance: 0, bounds: small, cube: false, avoid: Vec::new() };
        assert!(visible(&cx, tube, true, InsideView::Cutaway).placed.is_empty());
        // A 20 mm long tube is 16 px wide: its wall handle lands within 16 px of its diameter handle and hides, with the hint.
        r.set_dim(tube, Dim::Length, 0.02).unwrap();
        let v = vis(&r, tube, true, InsideView::Cutaway);
        assert!(v.crowded && !dims(&v).contains(&Dim::Wall) && dims(&v).contains(&Dim::Length));
    }

    #[test]
    fn typed_values_parse_in_the_display_unit() {
        let (r, ..) = rocket();
        let o = Overlay {
            rocket: &r,
            camera: side(),
            selected: None,
            also: Vec::new(),
            instance: 0,
            hover: Hover::None,
            cursor: None,
            snap_step: 0.001,
            inside: InsideView::Solid,
            profile: false,
            units: UnitSystem::default(),
            drive_modifiers: keyboard::Modifiers::empty(),
            pod: None,
            balance: None,
            hand: Hand::default(),
        };
        let v = o.parse(Dim::Length, "50").unwrap();
        assert!((v - 0.05).abs() < 1e-12);
        let a = o.parse(Dim::Cant, "15").unwrap();
        assert!((a.to_degrees() - 15.0).abs() < 1e-9);
        assert!(o.parse(Dim::Length, "abc").is_none());
        assert_eq!(o.delta(Dim::Length, 0.058), "+58.00");
    }

    #[test]
    fn dragging_a_handle_edits_one_dimension() {
        let (r, tube, ..) = rocket();
        let o = Overlay {
            rocket: &r,
            camera: side(),
            selected: Some(tube),
            also: Vec::new(),
            instance: 0,
            hover: Hover::None,
            cursor: None,
            snap_step: 0.001,
            inside: InsideView::Solid,
            profile: false,
            units: UnitSystem::default(),
            drive_modifiers: keyboard::Modifiers::empty(),
            pod: None,
            balance: None,
            hand: Hand::default(),
        };
        let v = vis(&r, tube, false, InsideView::Solid);
        let h = v.placed.iter().find(|p| p.spec.dim == Dim::Length).copied().unwrap();
        let mut drag = Drag {
            id: tube,
            placed: h,
            base: 0.5,
            base_vertex: (0.0, 0.0),
            from: h.at,
            targets: r.snap_targets(tube, Dim::Length),
            typed: String::new(),
            last: None,
            live: Live::default(),
        };
        // 80 px at 800 px/m is 100 mm: the length goes from 500 to 600.
        let Some(Edit::Dim(id, Dim::Length, value)) = o.drag_to(&mut drag, h.at + Vector::new(80.0, 20.0), keyboard::Modifiers::empty()) else {
            panic!("a length edit");
        };
        assert_eq!(id, tube);
        assert!((value - 0.6).abs() < 1e-9, "{value}");
        // The same position again changes nothing.
        assert!(o.drag_to(&mut drag, h.at + Vector::new(80.0, 20.0), keyboard::Modifiers::empty()).is_none());
        // Beyond the limit the value stops with a reason.
        o.drag_to(&mut drag, h.at + Vector::new(-4000.0, 0.0), keyboard::Modifiers::empty());
        assert_eq!(drag.live.limit, Some("MINIMUM LENGTH"));
        // With Shift a 12 in stock length catches within 8 px.
        let stock = 12.0 * 0.0254;
        let near = h.at + Vector::new(((stock - 0.5) * 800.0) as f32 + 3.0, 0.0);
        o.drag_to(&mut drag, near, keyboard::Modifiers::SHIFT);
        assert!(drag.live.caught.is_some());
        assert!((drag.live.value - stock).abs() < 1e-9);
    }
}

#[cfg(test)]
mod label_tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind, RadialKind};

    /// The example rocket's airframe (76.2 mm, nose + two tubes) with its three-fin set and a rail button.
    fn example() -> (Rocket, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762, tip_radius: 0.0 }).unwrap();
        r.insert(1, PartKind::BodyTube { length: 0.52, diameter: 0.0762 }).unwrap();
        let booster = r.insert(2, PartKind::BodyTube { length: 0.3934, diameter: 0.0762 }).unwrap();
        let fins = r
            .attach_radial(booster, 0.0, 0.0, 3, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.00318 })
            .unwrap();
        (r, fins)
    }

    fn overlay(r: &Rocket, fins: PartId, camera: Camera, profile: bool) -> Overlay<'_> {
        Overlay {
            rocket: r,
            camera,
            selected: Some(fins),
            also: Vec::new(),
            instance: 0,
            hover: Hover::None,
            cursor: None,
            snap_step: 0.001,
            inside: InsideView::Solid,
            profile,
            units: UnitSystem::default(),
            drive_modifiers: keyboard::Modifiers::empty(),
            pod: None,
            balance: Some(BalanceMarks { cg: 0.7035, cp: 0.8517, margin: 1.94, show: (true, true), staged: false }),
            hand: Hand::default(),
        }
    }

    fn camera(profile: bool, size: Size) -> Camera {
        let mut camera = if profile {
            Camera { yaw: -std::f32::consts::FRAC_PI_2, pitch: 0.0, ortho_blend: 1.0, ..Camera::default() }
        } else {
            crate::viewport::home(Camera::default())
        };
        camera.fit(Vec3::new(-0.04, -0.04, 0.0), Vec3::new(0.04, 0.04, 1.142), size);
        camera
    }

    fn disjoint(rects: &[Rectangle]) {
        for (i, a) in rects.iter().enumerate() {
            for b in &rects[i + 1..] {
                assert!(!a.intersects(b), "{a:?} overlaps {b:?}");
            }
        }
    }

    #[test]
    fn no_two_labels_of_a_selected_fin_set_overlap_and_the_inset_stays_clear() {
        let (r, fins) = example();
        // A viewport at 1280x800 and at 1000x700 (window minus the parts bin and the inspector).
        for size in [Size::new(704.0, 580.0), Size::new(404.0, 480.0)] {
            // 2D side (every handle), 3D home with Alt (every handle), 3D home on plain select (primary handles).
            for (profile, all) in [(true, true), (false, true), (false, false)] {
                let o = overlay(&r, fins, camera(profile, size), profile);
                let modifiers = if all && !profile { keyboard::Modifiers::ALT } else { keyboard::Modifiers::empty() };
                let selection = o.selection();
                let tag = o.tag_text(fins, &selection);
                let b = o.bounds_of_selection(size).expect("the fin set is on screen");
                let vis = o.visible(size, modifiers);
                let plan = o.plan(fins, &tag, b, size, &vis, all, modifiers.alt());
                let case = format!("{size:?} profile={profile} all={all}");

                let mut rects = vec![plan.tag];
                rects.extend(plan.side.iter().map(|l| l.rect));
                disjoint(&rects);
                // The CG / CP value labels and the margin label (the CP label sits over the fins here) are kept clear too.
                let marks = o.balance.expect("the fixture has balance markers");
                for m in markers::balance_label_rects(&r, &o.camera, size, &marks, o.units) {
                    for r in &rects {
                        assert!(!r.intersects(&m), "{case}: {r:?} over a marker label {m:?}");
                    }
                }
                if all {
                    assert!(!plan.side.is_empty() || !plan.hidden.is_empty(), "{case}: handle labels were planned");
                }
                if let Some(ins) = &vis.inset {
                    // In a viewport this narrow the long linked-group tag (215 px) has no free spot beside a 170 px inset: the tag
                    // is never dropped, so it may overlap there; the handle labels still must not.
                    let skip_tag = usize::from(size.width < 500.0);
                    for r in &rects[skip_tag..] {
                        assert!(!r.intersects(&ins.panel), "{case}: {r:?} over the inset {:?}", ins.panel);
                    }
                    // The inset keeps off the fin set itself.
                    assert!(!ins.panel.intersects(&b), "{case}: the inset covers the part");
                    let inner: Vec<Rectangle> = plan.inset.iter().map(|l| l.rect).collect();
                    disjoint(&inner);
                    for l in &inner {
                        assert!(l.x >= ins.panel.x && l.x + l.width <= ins.panel.x + ins.panel.width, "{case}: inset label outside its panel");
                    }
                }
            }
        }
    }

    #[test]
    fn inset_labels_stay_off_the_picture_and_a_tube_keeps_clear_of_the_marker_labels() {
        let (r, fins) = example();
        let tube = r.parts()[2].id;
        for size in [Size::new(704.0, 580.0), Size::new(404.0, 480.0)] {
            for profile in [true, false] {
                // The fin set: the inset's labels sit in the bands round the picture, never over it.
                let o = overlay(&r, fins, camera(profile, size), profile);
                let b = o.bounds_of_selection(size).unwrap();
                let vis = o.visible(size, keyboard::Modifiers::empty());
                let plan = o.plan(fins, "FIN SET", b, size, &vis, true, false);
                if let Some(ins) = &vis.inset {
                    let radius = ins.base + ins.reach.max(handles::CANT_LEVER) + 6.0;
                    let picture = Rectangle::new(ins.centre - Vector::new(radius, radius), Size::new(2.0 * radius, 2.0 * radius));
                    for l in &plan.inset {
                        assert!(!l.rect.intersects(&picture), "{size:?} profile={profile}: {:?} over the inset picture", l.text);
                    }
                }
                // Without Alt only the primary handles are labelled.
                assert!(plan.side.iter().all(|l| vis.placed[l.handle].spec.primary));
                // A tube: its tag and labels stay off the CG / CP / margin labels.
                let o = overlay(&r, tube, camera(profile, size), profile);
                let b = o.bounds_of_selection(size).unwrap();
                let vis = o.visible(size, keyboard::Modifiers::empty());
                let tag = o.tag_text(tube, &o.selection());
                let plan = o.plan(tube, &tag, b, size, &vis, true, false);
                let mut rects = vec![plan.tag];
                rects.extend(plan.side.iter().map(|l| l.rect));
                disjoint(&rects);
                for m in markers::balance_label_rects(&r, &o.camera, size, &o.balance.unwrap(), o.units) {
                    for rect in &rects {
                        assert!(!rect.intersects(&m), "{size:?} profile={profile}: {rect:?} over a marker label {m:?}");
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod carry_tests {
    use super::*;
    use bowshock_model::{InternalKind, NoseShape, PartKind};
    use iced::widget::canvas::Program;
    use std::f32::consts::FRAC_PI_2;

    const SIZE: Size = Size::new(900.0, 600.0);
    const BOUNDS: Rectangle = Rectangle { x: 0.0, y: 0.0, width: 900.0, height: 600.0 };

    fn camera() -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.55), yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 800.0, ortho_blend: 1.0 }
    }

    /// Nose (0 to 0.2), a recovery bay (to 0.7), a booster (to 1.1) with a Ø40 mount in it at stations 0.8 to 1.0.
    fn rocket() -> (Rocket, [PartId; 3], PartId) {
        let mut r = Rocket::default();
        let nose = r.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.2, diameter: 0.1, tip_radius: 0.0 }).unwrap();
        let bay = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let booster = r.insert(2, PartKind::BodyTube { length: 0.4, diameter: 0.1 }).unwrap();
        let mount = r
            .attach_internal(booster, 0.1, InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.002, length: 0.2, motor_mount: true, overhang: 0.0 })
            .unwrap();
        (r, [nose, bay, booster], mount)
    }

    fn overlay(r: &Rocket, inside: InsideView) -> Overlay<'_> {
        Overlay {
            rocket: r,
            camera: camera(),
            selected: None,
            also: Vec::new(),
            instance: 0,
            hover: Hover::None,
            cursor: None,
            snap_step: 0.001,
            inside,
            profile: true,
            units: UnitSystem::default(),
            drive_modifiers: keyboard::Modifiers::empty(),
            pod: None,
            balance: None,
            hand: Hand::default(),
        }
    }

    fn at(z: f32, y: f32) -> Point {
        camera().project(Vec3::new(0.0, y, z), SIZE)
    }

    /// What the overlay publishes for `event` with the cursor at `cursor`.
    fn send(o: &Overlay<'_>, state: &mut State, event: Event, cursor: Point) -> Option<Message> {
        o.update(state, &event, BOUNDS, mouse::Cursor::Available(cursor)).and_then(|action| action.into_inner().0)
    }

    fn press(o: &Overlay<'_>, state: &mut State, cursor: Point) {
        send(o, state, Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)), cursor);
    }

    fn moved(o: &Overlay<'_>, state: &mut State, cursor: Point) -> Option<Message> {
        send(o, state, Event::Mouse(mouse::Event::CursorMoved { position: cursor }), cursor)
    }

    #[test]
    fn pressing_and_dragging_an_internal_part_in_the_cutaway_carries_it_and_never_picks_the_tube_up() {
        let (r, [nose, bay, booster], mount) = rocket();
        // Solid: the shell hides the mount, so the drag picks the booster up (the radial attach flow), as it always did.
        let (o, mut state) = (overlay(&r, InsideView::Solid), State::default());
        press(&o, &mut state, at(0.9, 0.01));
        assert!(matches!(moved(&o, &mut state, at(0.88, 0.01)), Some(Message::Grab(Pick::Axial(id))) if id == booster));

        // Cutaway: the same press is on the mount. Dragging carries it in place; nothing is picked up.
        let (o, mut state) = (overlay(&r, InsideView::Cutaway), State::default());
        press(&o, &mut state, at(0.9, 0.01));
        assert!(matches!(moved(&o, &mut state, at(0.88, 0.01)), Some(Message::Edit(Edit::Carry(id))) if id == mount));
        assert!(state.carry.is_some());
        // Dragged 0.4 m fore (past the booster's end) over the recovery bay: it goes into the bay, keeping its station (0.8 to
        // 0.4 is 0.2 into the bay).
        let place = moved(&o, &mut state, at(0.5, 0.01));
        assert!(matches!(place, Some(Message::Edit(Edit::Place(m, p, offset))) if m == mount && p == bay && (offset - 0.2).abs() < 1e-6), "{place:?}");
        // The same place again sends nothing.
        assert!(moved(&o, &mut state, at(0.5, 0.01)).is_none());
        // Over the nose cone a Ø40 mount does not fit: nothing is sent and the tag says so.
        assert!(moved(&o, &mut state, at(0.1, 0.01)).is_none());
        assert!(state.carry.as_ref().and_then(|c| c.landing).is_some_and(|l| l.parent == nose && !l.fits));
        // Back on the bay it is placed again; the release ends the edit (one undo step).
        assert!(matches!(moved(&o, &mut state, at(0.55, 0.01)), Some(Message::Edit(Edit::Place(_, p, _))) if p == bay));
        let end = send(&o, &mut state, Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)), at(0.55, 0.01));
        assert!(matches!(end, Some(Message::Edit(Edit::End))) && state.carry.is_none());
    }

    #[test]
    fn a_click_on_an_internal_part_selects_it_and_the_selected_one_can_be_picked_in_solid() {
        let (r, _, mount) = rocket();
        let (o, mut state) = (overlay(&r, InsideView::XRay), State::default());
        press(&o, &mut state, at(0.9, 0.01));
        let click = send(&o, &mut state, Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)), at(0.9, 0.01));
        assert!(matches!(click, Some(Message::Select(Some(Pick::Internal(id)))) if id == mount));
        // Selected in Solid (the shell is ghosted around it): a drag on it carries it.
        let mut o = overlay(&r, InsideView::Solid);
        o.selected = Some(mount);
        let mut state = State::default();
        press(&o, &mut state, at(0.9, 0.01));
        assert!(matches!(moved(&o, &mut state, at(0.88, 0.01)), Some(Message::Edit(Edit::Carry(id))) if id == mount));
    }
    #[test]
    fn a_right_click_in_place_on_a_part_asks_for_its_menu_and_a_right_drag_does_not() {
        let (r, [_, _, booster], mount) = rocket();
        let right = |b: fn(mouse::Button) -> mouse::Event| Event::Mouse(b(mouse::Button::Right));
        // Solid: the shell hides the mount, so the booster tube is the part under the cursor.
        let (o, mut state) = (overlay(&r, InsideView::Solid), State::default());
        send(&o, &mut state, right(mouse::Event::ButtonPressed), at(0.9, 0.01));
        let menu = send(&o, &mut state, right(mouse::Event::ButtonReleased), at(0.9, 0.01));
        assert!(matches!(menu, Some(Message::PartMenu(Pick::Axial(id))) if id == booster));
        // Cutaway: the same click is on the inner tube.
        let (o, mut state) = (overlay(&r, InsideView::Cutaway), State::default());
        send(&o, &mut state, right(mouse::Event::ButtonPressed), at(0.9, 0.01));
        let menu = send(&o, &mut state, right(mouse::Event::ButtonReleased), at(0.9, 0.01));
        assert!(matches!(menu, Some(Message::PartMenu(Pick::Internal(id))) if id == mount));
        // Dragged (an orbit): no menu.
        let (o, mut state) = (overlay(&r, InsideView::Solid), State::default());
        send(&o, &mut state, right(mouse::Event::ButtonPressed), at(0.9, 0.01));
        moved(&o, &mut state, at(0.8, 0.01));
        assert!(send(&o, &mut state, right(mouse::Event::ButtonReleased), at(0.8, 0.01)).is_none());
    }

    #[test]
    fn clicking_to_place_a_carried_internal_part_survives_the_release_and_ends_with_a_press() {
        let (r, [_, bay, _], mount) = rocket();
        let mut o = overlay(&r, InsideView::Cutaway);
        o.hand = Hand { click: true, lifted: None };
        let mut state = State::default();
        press(&o, &mut state, at(0.9, 0.01));
        assert!(matches!(moved(&o, &mut state, at(0.88, 0.01)), Some(Message::Edit(Edit::Carry(id))) if id == mount));
        // The app has the part in hand (the overlay is rebuilt with it). Released: nothing is sent and the part stays in hand,
        // following the cursor with no button down.
        o.hand.lifted = Some(mount);
        let release = send(&o, &mut state, Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)), at(0.88, 0.01));
        assert!(release.is_none() && state.carry.is_some());
        let place = moved(&o, &mut state, at(0.5, 0.01));
        assert!(matches!(place, Some(Message::Edit(Edit::Place(m, p, _))) if m == mount && p == bay), "{place:?}");
        // A press puts it down: one edit ends; the release after it is nothing.
        let end = send(&o, &mut state, Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)), at(0.5, 0.01));
        assert!(matches!(end, Some(Message::Edit(Edit::End))) && state.carry.is_none());
        assert!(send(&o, &mut state, Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)), at(0.5, 0.01)).is_none());

        // A carry the app has put back (Esc, the parts bin) is forgotten by the overlay, so it never draws or places a stale one.
        press(&o, &mut state, at(0.9, 0.01));
        moved(&o, &mut state, at(0.88, 0.01));
        assert!(state.carry.is_some());
        o.hand.lifted = None;
        moved(&o, &mut state, at(0.8, 0.01));
        assert!(state.carry.is_some(), "not seen in hand yet: kept (the app has not caught up)");
        o.hand.lifted = Some(mount);
        moved(&o, &mut state, at(0.7, 0.01));
        o.hand.lifted = None;
        moved(&o, &mut state, at(0.6, 0.01));
        assert!(state.carry.is_none());
    }

    #[test]
    fn pressing_an_airframe_part_reports_where_it_was_pressed() {
        let (r, [_, bay, _], _) = rocket();
        let (o, mut state) = (overlay(&r, InsideView::Solid), State::default());
        // The bay runs 0.2 to 0.7: pressed at 0.35 it is 0.15 behind the fore face.
        let held = send(&o, &mut state, Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)), at(0.35, 0.01));
        assert!(matches!(held, Some(Message::Held(id, hold)) if id == bay && (hold - 0.15).abs() < 1e-4), "{held:?}");
    }
}
