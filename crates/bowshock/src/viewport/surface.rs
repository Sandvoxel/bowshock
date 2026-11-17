//! The overlay for dragging a radial part (fin set, lug, rail button) over the rocket. These have no attach nodes: they snap to
//! the surface under the cursor, at an offset along the part and an angle round it. This canvas works out that target from the
//! cursor, reports it when it changes and the drop on release (or on a click, when parts are placed by clicking), and marks the
//! target with a ring, a leader and a tag. The ghost copies themselves are drawn in 3D by the model mesh: on the surface, or
//! held in the air at the cursor when there is no surface under it.
//!
//! A group picked up off the rocket slides along the rocket by default, keeping its angle and following the cursor along the
//! axis wherever the cursor is (onto other parts as it passes them). Pulled [`DETACH_PX`] clear of the rocket's surface it comes
//! off: it follows the cursor as a free ghost and lands on the surface of any tube under it (the core's or a pod's); released
//! over empty space it is removed (one undo step, with a notice), and back within reach it slides again. Holding Alt makes it
//! follow the surface under the cursor (angle included) like a new part from the bin. The station of the part snaps to the view
//! toolbar's grid step, or 5 mm with Ctrl/Cmd.

use bowshock_model::{PartId, RadialKind, RadialPart, Rocket};
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke};
use iced::{keyboard, mouse, Point, Rectangle, Renderer, Size, Theme};

use super::assembly::{margin_suffix, no_entry, tag, tag_with};
use super::pick::{self, SurfaceTarget};
use super::pods::PodView;
use super::radial;
use super::{Camera, Message};
use crate::app::ui::{len, len_unit};
use crate::theme;
use crate::units::UnitSystem;

/// Diameter of the ring marking the target.
const RING: f32 = 7.0;

/// How far (px) past where it was grabbed the cursor must get from the rocket's surface before a group picked up off the rocket
/// is pulled free of it (ViewportOverlay: it stops sliding and follows the cursor).
pub const DETACH_PX: f32 = 40.0;

/// A radial part being dragged: from the parts bin, or a group picked up off the rocket.
#[derive(Debug, Clone)]
pub struct RadialDrag {
    /// The row of the parts bin it came from, if it came from there.
    pub row: Option<usize>,
    pub kind: RadialKind,
    /// Copies; `X` changes it mid-drag.
    pub count: u8,
    /// The group as it was, when picked up from the rocket (it is out of the rocket while dragged, and goes back as it was if
    /// the drag is cancelled or ends with no target).
    pub origin: Option<RadialPart>,
    /// Where it would land now.
    pub target: Option<SurfaceTarget>,
    /// A group picked up from the rocket has been pulled free of it (see [`DETACH_PX`]): released with no target it is removed
    /// instead of going back.
    pub detached: bool,
}

pub struct Overlay<'a> {
    pub rocket: &'a Rocket,
    pub drag: &'a RadialDrag,
    pub camera: Camera,
    /// Angle snap in degrees, 0 for none.
    pub snap: u8,
    /// The view toolbar's grid step, metres (Ctrl/Cmd gives 5 mm).
    pub snap_step: f64,
    /// The cursor in window coordinates (tracked by the app, so it also draws without a live cursor).
    pub cursor: Option<Point>,
    /// The margin of the rocket as it would be if released now, calibers, for the tag; `None` with no valid target.
    pub margin: Option<f64>,
    /// The display unit set the tag's stations are written in.
    pub units: UnitSystem,
    /// Parts are placed by clicking (Preferences > Editor): a group pulled free of the rocket is not removed by releasing.
    pub click: bool,
}

#[derive(Debug, Default)]
pub struct State {
    modifiers: keyboard::Modifiers,
    /// Trackpad scroll not yet worth a step, in steps.
    wheel: f32,
    /// How far (px) the cursor was from the rocket's surface when the drag was first seen: the part is pulled free when the
    /// cursor gets [`DETACH_PX`] further than that.
    grab: Option<f32>,
    /// The part has been pulled free and not yet put back within reach: a surface under the cursor, on any tube, takes it.
    freed: bool,
}

impl Overlay<'_> {
    /// Where the part would land with the cursor at `at` (viewport px), and whether it has been pulled free of the rocket it
    /// was picked up from. `free` (Alt) follows the surface under the cursor.
    ///
    /// A group picked up off the rocket slides along it until the cursor is [`DETACH_PX`] away from the rocket's surface; then
    /// it is free: it follows the surface under the cursor (any tube, the core's or a pod's), and over empty space it has no
    /// target (releasing there removes it). Back within reach of the rocket with no surface under the cursor, it slides again.
    fn choose(&self, state: &mut State, at: Point, size: Size) -> (Option<SurfaceTarget>, bool) {
        let (target, detached) = self.choose_free(state, at, size);
        // Shift snaps to the part's ends: the group's fore edge flush with the tube's fore end, or its aft edge flush with the aft
        // end (fins down to the bottom of the tube), whichever is nearer.
        let target = match target {
            Some(t) if state.modifiers.shift() => Some(pick::flush(self.rocket, &self.drag.kind, t)),
            other => other,
        };
        (target, detached)
    }

    fn choose_free(&self, state: &mut State, at: Point, size: Size) -> (Option<SurfaceTarget>, bool) {
        let (rocket, camera, kind) = (self.rocket, &self.camera, &self.drag.kind);
        let modifiers = state.modifiers;
        let (free, step) = (modifiers.alt(), pick::step(modifiers, self.snap_step));
        let surface = || pick::surface_target(rocket, camera, at, size, kind, pick::Snapping { angle: self.snap, station: step });
        let Some(origin) = &self.drag.origin else {
            return (surface(), false);
        };
        let distance = pick::stack_distance(rocket, camera, at, size, origin.parent);
        let grab = *state.grab.get_or_insert(distance);
        let pulled = distance - grab > DETACH_PX;
        state.freed |= pulled;
        if state.freed {
            if let Some(target) = surface() {
                return (Some(target), true);
            }
            if pulled {
                return (None, true);
            }
            state.freed = false;
        }
        // Picked up off the rocket: in 3D, over its own tube, it follows the cursor round and along the surface (so a placed group
        // can be rotated about the tube as well as slid; side-on in 2D the angle is ambiguous, so there it only slides).
        // Elsewhere it slides along the tube keeping the angle (the one it had, or the last target's).
        if let Some(target) = (!camera.is_ortho()).then(surface).flatten().filter(|t| t.parent == origin.parent) {
            return (Some(target), false);
        }
        let angle = self.drag.target.map_or(origin.angle, |t| t.angle);
        let slide = || pick::slide_from(rocket, camera, at, size, kind, angle, step, origin.parent);
        (if free { surface().or_else(slide) } else { slide() }, false)
    }

    fn parent_name(&self, parent: PartId) -> &'static str {
        self.rocket.part(parent).map_or("", |p| p.kind.name())
    }
}

impl canvas::Program<Message> for Overlay<'_> {
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        if let Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) = event {
            state.modifiers = *modifiers;
            return None;
        }
        if let Event::Mouse(mouse::Event::WheelScrolled { delta }) = event {
            // Mid-drag the wheel steps the number of copies rather than zooming the camera (one step per notch, or per 40 px of
            // trackpad scroll).
            cursor.position_over(bounds)?;
            let steps = match *delta {
                mouse::ScrollDelta::Lines { y, .. } => y,
                mouse::ScrollDelta::Pixels { y, .. } => {
                    state.wheel += y / 40.0;
                    let whole = state.wheel.trunc();
                    state.wheel -= whole;
                    whole
                }
            };
            return Some(if steps == 0.0 { Action::capture() } else { Action::publish(Message::CopiesStep(steps > 0.0)).and_capture() });
        }
        // A release over the viewport drops and a press places a part held with no button down (the app decides what each does:
        // see `drag::end_carry`); elsewhere the app decides.
        if let Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) = event {
            return cursor.position_over(bounds).map(|_| Action::publish(Message::DropRadial).and_capture());
        }
        if let Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) = event {
            return cursor.position_over(bounds).map(|_| Action::publish(Message::PlaceHere).and_capture());
        }
        let (target, detached) = cursor.position_in(bounds).map_or((None, false), |p| self.choose(state, p, bounds.size()));
        ((target, detached) != (self.drag.target, self.drag.detached)).then(|| Action::publish(Message::Surface(target, detached)))
    }

    fn draw(&self, _: &State, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let cursor =
            self.cursor.map(|c| Point::new(c.x - bounds.x, c.y - bounds.y)).filter(|c| (0.0..=size.width).contains(&c.x) && (0.0..=size.height).contains(&c.y));
        let Some(cursor) = cursor else {
            return vec![frame.into_geometry()];
        };

        match self.drag.target {
            Some(target) => {
                let extent = self.drag.kind.extent();
                let radius = self.rocket.surface_radius(target.parent, target.offset + extent / 2.0).unwrap_or(0.0);
                let (u, _) = radial::dirs(target.angle as f32);
                // On a part of a pod: marked on the copy the cursor is on, through that copy's own frame.
                let (station, at) = match PodView::of_part(self.rocket, target.parent, &self.camera) {
                    Some(view) => {
                        let copy = target.copy.min(view.cameras.len().saturating_sub(1));
                        let local = view.sub.index_of(target.parent).map_or(0.0, |i| view.sub.stations()[i]);
                        let z = local + target.offset + extent / 2.0;
                        (f64::from(view.poses[copy].z0) + local, view.cameras[copy].project(u * radius as f32 + Vec3::Z * z as f32, size))
                    }
                    None => {
                        let station = self.rocket.index_of(target.parent).map_or(0.0, |i| self.rocket.stations()[i]);
                        (station, self.camera.project(u * radius as f32 + Vec3::Z * (station + target.offset + extent / 2.0) as f32, size))
                    }
                };
                frame.fill(&Path::circle(at, RING * 1.5), t.ok.scale_alpha(0.18));
                frame.stroke(&Path::circle(at, RING / 2.0), Stroke::default().with_color(t.ok).with_width(1.5));
                let dashed = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(t.ok).with_width(1.0) };
                frame.stroke(&Path::line(cursor, at), dashed);
                let times = if self.drag.count > 1 { format!(" ×{}", self.drag.count) } else { String::new() };
                let label = format!(
                    "{}{times} → {} @ {} {}, {:.0}°",
                    self.drag.kind.name(),
                    self.parent_name(target.parent),
                    len(station + target.offset, self.units),
                    len_unit(self.units),
                    target.angle.to_degrees().round() % 360.0
                );
                tag_with(&mut frame, cursor, 0.0, &label, margin_suffix(self.rocket, t, self.margin), t.ink, t.line_200, t.bg_200);
            }
            // Pulled free of the rocket over empty space: the group is drawn in the air by the model mesh; releasing removes it.
            None if self.drag.detached && !self.click => {
                let times = if self.drag.count > 1 { format!(" ×{}", self.drag.count) } else { String::new() };
                no_entry(&mut frame, cursor + iced::Vector::new(14.0, 14.0), t.err);
                tag(&mut frame, cursor, 18.0, &format!("{}{times} · release to remove", self.drag.kind.name()), t.ink, t.line_200, t.bg_200);
            }
            None => {
                no_entry(&mut frame, cursor + iced::Vector::new(14.0, 14.0), t.err);
                tag(&mut frame, cursor, 18.0, "No surface under cursor", t.ink, t.line_200, t.bg_200);
            }
        }
        vec![frame.into_geometry()]
    }

    fn mouse_interaction(&self, _: &State, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        match (cursor.is_over(bounds), self.drag.target) {
            (false, _) => mouse::Interaction::default(),
            (true, Some(_)) => mouse::Interaction::Grabbing,
            (true, None) if self.drag.detached => mouse::Interaction::Grabbing,
            (true, None) => mouse::Interaction::NotAllowed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind};
    use std::f32::consts::FRAC_PI_2;

    const SIZE: Size = Size::new(800.0, 600.0);
    const FINS: RadialKind = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.06, sweep: 0.04, thickness: 0.003 };

    /// Side-on (the nose points left, +Z runs right) at 1000 px/m.
    fn camera() -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.5), yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 1000.0, ortho_blend: 1.0 }
    }

    /// A nose and a tube (Ø0.1), with the fin set picked up off the tube (it is out of the rocket, as while dragged).
    fn picked_up() -> (Rocket, PartId, RadialDrag) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.2, diameter: 0.1, tip_radius: 0.0 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let id = r.attach_radial(tube, 0.3, 0.0, 3, FINS).unwrap();
        let origin = r.remove_radial(id).unwrap();
        let drag = RadialDrag { row: None, kind: origin.kind.clone(), count: 3, origin: Some(origin), target: None, detached: false };
        (r, tube, drag)
    }

    fn overlay<'a>(r: &'a Rocket, drag: &'a RadialDrag) -> Overlay<'a> {
        Overlay { rocket: r, drag, camera: camera(), snap: 0, snap_step: 0.001, cursor: None, margin: None, units: UnitSystem::default(), click: false }
    }

    fn at(z: f32, y: f32) -> Point {
        camera().project(Vec3::new(0.0, y, z), SIZE)
    }

    #[test]
    fn a_group_slides_until_the_cursor_is_pulled_40_px_clear_then_follows_the_cursor_and_sticks_again_when_it_comes_back() {
        let (r, tube, drag) = picked_up();
        let (o, mut state) = (overlay(&r, &drag), State::default());
        // Grabbed on the fin, 30 px off the tube's surface: it slides along the tube under the cursor.
        let (slide, detached) = o.choose(&mut state, at(0.5, 0.08), SIZE);
        assert!(!detached && slide.is_some_and(|t| t.parent == tube && (t.angle - 0.0).abs() < 1e-9), "{slide:?}");
        // 20 px further out (50 px off): still attached, and the slide follows the cursor along the axis.
        let (slide, detached) = o.choose(&mut state, at(0.4, 0.10), SIZE);
        assert!(!detached && slide.is_some_and(|t| t.parent == tube));
        // 40 px past where it was grabbed (70 px off the surface): free, with no target over empty space.
        let (free, detached) = o.choose(&mut state, at(0.4, 0.121), SIZE);
        assert!(detached && free.is_none(), "{free:?}");
        assert!(state.freed);
        // Back within reach with no surface under the cursor: it sticks again.
        let (stuck, detached) = o.choose(&mut state, at(0.4, 0.09), SIZE);
        assert!(!detached && stuck.is_some_and(|t| t.parent == tube) && !state.freed);
    }

    #[test]
    fn once_free_a_surface_under_the_cursor_takes_the_part_whichever_tube_it_is() {
        let (r, tube, drag) = picked_up();
        let (o, mut state) = (overlay(&r, &drag), State::default());
        o.choose(&mut state, at(0.5, 0.08), SIZE);
        assert!(o.choose(&mut state, at(0.4, 0.14), SIZE).1);
        // Over the body again: the surface under the cursor (angle from where it is hit), still a detached drag.
        let (target, detached) = o.choose(&mut state, at(0.45, 0.0), SIZE);
        let target = target.expect("the tube's surface");
        assert!(detached && target.parent == tube);
        assert!((target.angle.to_degrees() - 270.0).abs() < 1e-3, "{}", target.angle.to_degrees());
    }

    #[test]
    fn a_part_from_the_bin_has_no_rocket_to_be_pulled_from() {
        let (r, _, mut drag) = picked_up();
        drag.origin = None;
        let (o, mut state) = (overlay(&r, &drag), State::default());
        assert_eq!(o.choose(&mut state, at(0.4, 0.14), SIZE), (None, false));
        assert!(o.choose(&mut state, at(0.45, 0.0), SIZE).0.is_some());
    }
}
