//! The drag overlay: while a part is dragged in from the parts bin this canvas shows the valid attach nodes on the rocket's
//! centreline, picks the one nearest the cursor as the snap target, and reports snap changes and the drop (design/components/
//! ViewportOverlay). The ghost part itself is drawn in 3D by the model mesh.

use bowshock_model::{InternalPart, Part, PartKind, RadialPart, Rocket};
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::{alignment, mouse, Color, Point, Rectangle, Renderer, Size, Theme};

use super::{Camera, Message};
use crate::app::ui::mm;
use crate::theme::{self, MONO};
use crate::tokens::TEXT_LABEL;

/// A node within this many px of the cursor is the snap target.
const SNAP_RANGE: f32 = 40.0;
/// Diameter of a node ring.
const RING: f32 = 7.0;
/// Width of one MONO glyph at `TEXT_LABEL`, as a fraction of the font size (IBM Plex Mono advances 0.6 em).
const GLYPH_EM: f32 = 0.6;
const TAG_PAD: f32 = 6.0;

/// A part being dragged: what, and the attach node it would drop onto.
#[derive(Debug, Clone)]
pub struct Drag {
    /// The row of the parts bin it came from, if it came from there.
    pub row: Option<usize>,
    /// Where it was picked up from, if it is a part of the rocket being moved.
    pub origin: Option<Origin>,
    /// The bin's name for it ("Transition", "Boattail"), used in the cursor tag.
    pub name: &'static str,
    pub kind: PartKind,
    pub snap: Option<usize>,
}

/// A part of the rocket picked up for moving: it is out of the rocket while dragged, and goes back as it was if the drag is
/// cancelled or ends with no snap target.
#[derive(Debug, Clone)]
pub struct Origin {
    /// The part exactly as it was (material, finish, overrides), before any diameter matching.
    pub part: Part,
    /// The radial parts it carries: taken off while it is dragged, put back on it where it lands.
    pub radials: Vec<RadialPart>,
    /// The internal parts it carries, likewise.
    pub internals: Vec<InternalPart>,
}

pub struct Overlay<'a> {
    pub rocket: &'a Rocket,
    pub drag: &'a Drag,
    pub camera: Camera,
    /// The cursor in window coordinates (tracked by the app, so it also draws without a live cursor).
    pub cursor: Option<Point>,
}

impl Overlay<'_> {
    /// Valid nodes: index, station (m) and screen position.
    fn nodes(&self, size: Size) -> Vec<(usize, f64, Point)> {
        self.rocket
            .nodes_for(&self.drag.kind)
            .into_iter()
            .map(|(index, station)| (index, station, self.camera.project(Vec3::new(0.0, 0.0, station as f32), size)))
            .collect()
    }

    /// The node nearest `at` within [`SNAP_RANGE`].
    fn nearest(&self, at: Point, size: Size) -> Option<usize> {
        self.nodes(size)
            .into_iter()
            .map(|(index, _, p)| (index, p.distance(at)))
            .filter(|&(_, d)| d <= SNAP_RANGE)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(index, _)| index)
    }

    /// What releasing here would do, as a tag.
    fn tag(&self, snap: usize, station: f64) -> String {
        let name = self.drag.name;
        let parts = self.rocket.parts().len();
        if snap == 0 {
            format!("{name} → snap fore")
        } else if snap == parts {
            format!("{name} → snap aft")
        } else {
            format!("{name} → insert at {} mm", mm(station))
        }
    }
}

impl canvas::Program<Message> for Overlay<'_> {
    type State = ();

    fn update(&self, _: &mut (), event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        if let Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) = event {
            // Releasing over the viewport drops (or, with no snap target, cancels); elsewhere the app cancels.
            return cursor.position_over(bounds).map(|_| Action::publish(Message::DropAxial).and_capture());
        }
        let snap = cursor.position_in(bounds).and_then(|p| self.nearest(p, bounds.size()));
        (snap != self.drag.snap).then(|| Action::publish(Message::SnapNode(snap)))
    }

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let nodes = self.nodes(size);

        // Parts after the ghost have moved aft by its length: draw their nodes where those parts now are. Snapping keeps
        // to the settled positions, so the target does not slide under the cursor.
        let ghost = self.drag.snap.map(|snap| (snap, self.rocket.matched(snap, &self.drag.kind).length()));
        for &(index, station, at) in &nodes {
            let at = match ghost {
                Some((snap, len)) if index > snap => self.camera.project(Vec3::new(0.0, 0.0, (station + len) as f32), size),
                _ => at,
            };
            let target = self.drag.snap == Some(index);
            if target {
                frame.fill(&Path::circle(at, RING * 1.5), t.ok.scale_alpha(0.18));
            }
            let color = if target { t.ok } else { t.ok.scale_alpha(0.5) };
            frame.stroke(&Path::circle(at, RING / 2.0), Stroke::default().with_color(color).with_width(1.5));
        }

        // Cursor feedback only while the cursor is over the viewport.
        let local = self.cursor.map(|c| Point::new(c.x - bounds.x, c.y - bounds.y));
        let Some(cursor) = local.filter(|c| (0.0..=size.width).contains(&c.x) && (0.0..=size.height).contains(&c.y)) else {
            return vec![frame.into_geometry()];
        };

        let target = self.drag.snap.and_then(|snap| nodes.iter().find(|n| n.0 == snap));
        let label = match target {
            Some(&(index, station, at)) => {
                let leader = Path::line(cursor, at);
                let dashed = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(t.ok).with_width(1.0) };
                frame.stroke(&leader, dashed);
                self.tag(index, station)
            }
            None => {
                no_entry(&mut frame, cursor + iced::Vector::new(14.0, 14.0), t.err);
                "No attach node in range".to_string()
            }
        };
        // The tag sits down-right of the cursor (after the no-entry glyph when there is one).
        let glyph = if target.is_some() { 0.0 } else { 18.0 };
        tag(&mut frame, cursor, glyph, &label, t.ink, t.line_200, t.bg_200);
        vec![frame.into_geometry()]
    }

    fn mouse_interaction(&self, _: &(), bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        match (cursor.is_over(bounds), self.drag.snap) {
            (false, _) => mouse::Interaction::default(),
            (true, Some(_)) => mouse::Interaction::Grabbing,
            (true, None) => mouse::Interaction::NotAllowed,
        }
    }
}

/// A framed square tag with `label` in MONO, down-right of `cursor` (`indent` px further right), flipped to stay inside the frame.
pub fn tag(frame: &mut Frame, cursor: Point, indent: f32, label: &str, ink: Color, edge: Color, bg: Color) {
    let size = frame.size();
    let (w, h) = (label.chars().count() as f32 * TEXT_LABEL * GLYPH_EM + 2.0 * TAG_PAD, TEXT_LABEL + 2.0 * TAG_PAD - 2.0);
    let mut origin = cursor + iced::Vector::new(14.0 + indent, 14.0);
    if origin.x + w > size.width {
        origin.x = cursor.x - 14.0 - w;
    }
    if origin.y + h > size.height {
        origin.y = cursor.y - 14.0 - h;
    }
    frame.fill_rectangle(origin, Size::new(w, h), bg);
    frame.stroke_rectangle(origin, Size::new(w, h), Stroke::default().with_color(edge).with_width(1.0));
    frame.fill_text(Text {
        content: label.to_string(),
        position: Point::new(origin.x + TAG_PAD, origin.y + h / 2.0),
        color: ink,
        size: TEXT_LABEL.into(),
        font: MONO,
        align_y: alignment::Vertical::Center,
        ..Text::default()
    });
}

/// A ⊘ glyph with its top-left at `at`: a ring crossed by a slash, in `color`.
pub fn no_entry(frame: &mut Frame, at: Point, color: Color) {
    let r = 6.0;
    let centre = Point::new(at.x + r, at.y + r);
    let stroke = Stroke::default().with_color(color).with_width(1.5);
    frame.stroke(&Path::circle(centre, r), stroke);
    let d = r * std::f32::consts::FRAC_1_SQRT_2;
    frame.stroke(&Path::line(Point::new(centre.x - d, centre.y + d), Point::new(centre.x + d, centre.y - d)), stroke);
}
