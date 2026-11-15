//! The overlay for dragging a radial part (fin set, lug, rail button) over the rocket. These have no attach nodes: they snap to
//! the surface under the cursor, at an offset along the part and an angle round it. This canvas works out that target from the
//! cursor, reports it when it changes and the drop on release, and marks the target with a ring, a leader and a tag. The ghost
//! copies themselves are drawn in 3D by the model mesh.
//!
//! A group picked up off the rocket slides along the rocket by default, keeping its angle and following the cursor along the
//! axis wherever the cursor is (onto other parts as it passes them); holding Alt makes it follow the surface under the
//! cursor (angle included) like a new part from the bin. The station of the part snaps to the view toolbar's grid step, or
//! 5 mm with Ctrl/Cmd.

use bowshock_model::{PartId, RadialKind, RadialPart, Rocket};
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke};
use iced::{keyboard, mouse, Point, Rectangle, Renderer, Size, Theme};

use super::assembly::{no_entry, tag};
use super::pick::{self, SurfaceTarget};
use super::pods::PodView;
use super::radial;
use super::{Camera, Message};
use crate::app::ui::mm;
use crate::theme;

/// Diameter of the ring marking the target.
const RING: f32 = 7.0;

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
}

#[derive(Debug, Default)]
pub struct State {
    modifiers: keyboard::Modifiers,
    /// Trackpad scroll not yet worth a step, in steps.
    wheel: f32,
}

impl Overlay<'_> {
    /// Where the part would land with the cursor at `at` (viewport px). `free` (Alt) follows the surface under the cursor.
    fn target_at(&self, at: Point, size: Size, modifiers: keyboard::Modifiers) -> Option<SurfaceTarget> {
        let (rocket, camera, kind) = (self.rocket, &self.camera, &self.drag.kind);
        let (free, step) = (modifiers.alt(), pick::step(modifiers, self.snap_step));
        let surface = || pick::surface_target(rocket, camera, at, size, kind, pick::Snapping { angle: self.snap, station: step });
        let Some(origin) = &self.drag.origin else {
            return surface();
        };
        // Picked up off the rocket: slide along it, keeping the angle (the one it had, or the last target's).
        let angle = self.drag.target.map_or(origin.angle, |t| t.angle);
        let slide = || pick::slide_from(rocket, camera, at, size, kind, angle, step, origin.parent);
        if free {
            surface().or_else(slide)
        } else {
            slide()
        }
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
        if let Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) = event {
            // Releasing over the viewport drops (or, with no target, cancels); elsewhere the app cancels.
            return cursor.position_over(bounds).map(|_| Action::publish(Message::DropRadial).and_capture());
        }
        let target = cursor.position_in(bounds).and_then(|p| self.target_at(p, bounds.size(), state.modifiers));
        (target != self.drag.target).then(|| Action::publish(Message::Surface(target)))
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
                    "{}{times} → {} @ {} mm, {:.0}°",
                    self.drag.kind.name(),
                    self.parent_name(target.parent),
                    mm(station + target.offset),
                    target.angle.to_degrees().round() % 360.0
                );
                tag(&mut frame, cursor, 0.0, &label, t.ink, t.line_200, t.bg_200);
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
            (true, None) => mouse::Interaction::NotAllowed,
        }
    }
}
