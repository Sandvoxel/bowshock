//! The overlay for dragging an internal part (ring, bulkhead, chute, motor mount, ...) in from the parts bin over the rocket. The
//! view is in X-ray while it is carried (see `View::sync`), so the ghost shows inside the airframe: a dashed outline of the part
//! at the body part under the cursor, centred on the cursor's station, with a tag saying where it would land. A body part whose
//! bore the part does not fit gets a red ⊘ and no target. The target is reported when it changes, the drop on release.

use bowshock_model::Rocket;
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke};
use iced::{keyboard, mouse, Point, Rectangle, Renderer, Size, Theme, Vector};

use super::assembly::{margin_suffix, no_entry, tag, tag_with};
use super::carry::{centred_landing, Landing};
use super::{handles, pick, Camera, Message};
use crate::app::drag::InternalDrag;
use crate::app::ui::mm;
use crate::theme;

pub struct Overlay<'a> {
    pub rocket: &'a Rocket,
    pub drag: &'a InternalDrag,
    pub camera: Camera,
    /// The view toolbar's grid step, metres (Ctrl/Cmd gives 5 mm).
    pub snap_step: f64,
    /// The cursor in window coordinates (tracked by the app, so it also draws without a live cursor).
    pub cursor: Option<Point>,
    /// The margin of the rocket as it would be if released now, calibers, for the tag; `None` with no valid target.
    pub margin: Option<f64>,
}

#[derive(Debug, Default)]
pub struct State {
    modifiers: keyboard::Modifiers,
}

impl Overlay<'_> {
    /// Where the part lands with the cursor at `at` (viewport px).
    fn landing_at(&self, at: Point, size: Size, modifiers: keyboard::Modifiers) -> Option<Landing> {
        let z = pick::axis_station(&self.camera, at, size)?;
        centred_landing(self.rocket, &self.drag.kind, z, pick::step(modifiers, self.snap_step))
    }
}

impl canvas::Program<Message> for Overlay<'_> {
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        if let Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) = event {
            state.modifiers = *modifiers;
            return None;
        }
        // Over the lanes the lanes set the target; only a cursor over the viewport is judged here.
        let at = cursor.position_in(bounds)?;
        if let Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) = event {
            return Some(Action::publish(Message::DropInternal).and_capture());
        }
        let target = self
            .landing_at(at, bounds.size(), state.modifiers)
            .filter(|l| l.fits)
            .map(|l| crate::app::drag::InternalTarget { parent: l.parent, offset: l.offset });
        (target != self.drag.target).then(|| Action::publish(Message::InternalTarget(target)))
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
        let Some(landing) = self.landing_at(cursor, size, keyboard::Modifiers::empty()) else {
            return vec![frame.into_geometry()];
        };
        let rocket = self.rocket;
        let parent_station = rocket.part_station(landing.parent).unwrap_or(0.0);
        let fore = parent_station + landing.offset;
        let extent = self.drag.kind.extent();
        let bore = rocket.inner_diameter_at(landing.parent, landing.offset + extent / 2.0).unwrap_or(0.0);
        let radius = (self.drag.kind.given_outer_diameter().unwrap_or(0.8 * bore) / 2.0) as f32;
        let up = handles::radial_up(&self.camera);
        let corner = |z: f64, s: f32| self.camera.project(Vec3::Z * z as f32 + up * radius * s, size);
        let ghost = Path::new(|p| {
            p.move_to(corner(fore, 1.0));
            p.line_to(corner(fore + extent, 1.0));
            p.line_to(corner(fore + extent, -1.0));
            p.line_to(corner(fore, -1.0));
            p.close();
        });
        let ink = if landing.fits { t.accent } else { t.err };
        frame.fill(&ghost, ink.scale_alpha(0.18));
        frame.stroke(&ghost, Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(ink).with_width(1.0) });
        let parent = rocket.part(landing.parent).map_or_else(String::new, |p| p.display_name());
        if landing.fits {
            let label = format!("{} → {parent} @ {} mm", self.drag.name, mm(fore));
            tag_with(&mut frame, cursor, 0.0, &label, margin_suffix(rocket, t, self.margin), t.ink, t.line_200, t.bg_200);
        } else {
            no_entry(&mut frame, cursor + Vector::new(14.0, 14.0), t.err);
            tag(&mut frame, cursor, 18.0, &format!("{} does not fit the {parent}", self.drag.name), t.ink, t.line_200, t.bg_200);
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
