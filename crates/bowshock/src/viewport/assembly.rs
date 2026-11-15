//! The drag overlay: while a part is dragged in from the parts bin this canvas shows the valid attach nodes on the rocket's
//! centreline, picks the one nearest the cursor as the snap target, and reports snap changes and the drop (design/components/
//! ViewportOverlay). The ghost part itself is drawn in 3D by the model mesh.

use bowshock_model::{InternalPart, Part, PartId, PartKind, RadialPart, Rocket};
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::{alignment, mouse, Color, Point, Rectangle, Renderer, Size, Theme};

use super::pods::PodView;
use super::{Camera, Message};
use crate::app::ui::mm;
use crate::theme::{self, MONO};
use crate::tokens::TEXT_LABEL;

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
    /// The node of a pod's stack it would drop onto instead: `(pod, node index)`. At most one of `snap` and `pod_snap` is set.
    pub pod_snap: Option<(PartId, usize)>,
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
    /// Whether the part carries a pod (a part of the core that has pods on it cannot go into a pod: pods do not nest).
    fn carries_pod(&self) -> bool {
        self.drag.origin.as_ref().is_some_and(|o| o.radials.iter().any(|r| r.kind.is_pod()))
    }

    /// Valid nodes of the core and of every pod's stack, with their screen positions (a pod's on each of its copies).
    fn nodes(&self, size: Size) -> Vec<Node> {
        let core = self.rocket.nodes_for(&self.drag.kind).into_iter().map(|(index, station)| Node {
            snap: Snap::Core(index),
            station,
            at: self.camera.project(Vec3::new(0.0, 0.0, station as f32), size),
        });
        let mut nodes: Vec<Node> = core.collect();
        if !self.carries_pod() {
            for pod in self.rocket.pods() {
                let Some(view) = PodView::of_pod(self.rocket, pod, &self.camera) else { continue };
                for (index, station) in self.rocket.nodes_for_pod(pod.id, &self.drag.kind) {
                    for (pose, camera) in view.poses.iter().zip(&view.cameras) {
                        let at = camera.project(Vec3::Z * (station as f32 - pose.z0), size);
                        nodes.push(Node { snap: Snap::Pod(pod.id, index), station, at });
                    }
                }
            }
        }
        nodes
    }

    /// The node nearest `at`: a part dropped from the bin always attaches to the nearest node, no key needed.
    fn nearest(&self, at: Point, size: Size) -> Option<Snap> {
        self.nodes(size).into_iter().map(|n| (n.snap, n.at.distance(at))).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(snap, _)| snap)
    }

    /// Where the drag is snapped now.
    fn current(&self) -> Option<Snap> {
        self.drag.snap.map(Snap::Core).or_else(|| self.drag.pod_snap.map(|(pod, index)| Snap::Pod(pod, index)))
    }

    /// What releasing here would do, as a tag.
    fn tag(&self, snap: Snap, station: f64) -> String {
        let name = self.drag.name;
        match snap {
            Snap::Core(snap) => {
                let parts = self.rocket.parts().len();
                if snap == 0 {
                    format!("{name} → snap fore")
                } else if snap == parts {
                    format!("{name} → snap aft")
                } else {
                    format!("{name} → insert at {} mm", mm(station))
                }
            }
            Snap::Pod(pod, index) => {
                let copies = self.rocket.radial(pod).map_or(1, |p| p.count);
                let times = if copies > 1 { format!(" ×{copies}") } else { String::new() };
                let parts = self.rocket.radial(pod).and_then(|p| p.kind.pod_body()).map_or(0, |b| b.parts().len());
                let place = if index == 0 {
                    "fore".to_string()
                } else if index == parts {
                    "aft".to_string()
                } else {
                    format!("at {} mm", mm(station))
                };
                format!("{name} → pod{times}, {place}")
            }
        }
    }
}

/// Where a carried part would attach: a node of the core's stack or of a pod's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Snap {
    /// Node `index` of the core.
    Core(usize),
    /// Node `index` of pod `pod`'s stack.
    Pod(PartId, usize),
}

/// An attach node on screen.
struct Node {
    snap: Snap,
    station: f64,
    at: Point,
}

impl canvas::Program<Message> for Overlay<'_> {
    type State = ();

    fn update(&self, _: &mut (), event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        if let Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) = event {
            // Releasing over the viewport drops (or, with no snap target, cancels); elsewhere the app cancels.
            return cursor.position_over(bounds).map(|_| Action::publish(Message::DropAxial).and_capture());
        }
        let snap = cursor.position_in(bounds).and_then(|p| self.nearest(p, bounds.size()));
        (snap != self.current()).then(|| {
            Action::publish(match snap {
                Some(Snap::Pod(pod, index)) => Message::SnapPodNode(Some((pod, index))),
                Some(Snap::Core(index)) => Message::SnapNode(Some(index)),
                None => Message::SnapNode(None),
            })
        })
    }

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let nodes = self.nodes(size);

        // Parts after the ghost have moved aft by its length: draw their nodes where those parts now are. Snapping keeps
        // to the settled positions, so the target does not slide under the cursor.
        let ghost = self.drag.snap.map(|snap| (snap, self.rocket.matched(snap, &self.drag.kind).length()));
        let current = self.current();
        for node in &nodes {
            let at = match (ghost, node.snap) {
                (Some((snap, len)), Snap::Core(index)) if index > snap => self.camera.project(Vec3::new(0.0, 0.0, (node.station + len) as f32), size),
                _ => node.at,
            };
            let target = current == Some(node.snap);
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

        let target = current.and_then(|snap| nodes.iter().find(|n| n.snap == snap));
        let label = match target {
            Some(node) => {
                let leader = Path::line(cursor, node.at);
                let dashed = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(t.ok).with_width(1.0) };
                frame.stroke(&leader, dashed);
                self.tag(node.snap, node.station)
            }
            None => {
                no_entry(&mut frame, cursor + iced::Vector::new(14.0, 14.0), t.err);
                "No attach node".to_string()
            }
        };
        // The tag sits down-right of the cursor (after the no-entry glyph when there is one).
        let glyph = if target.is_some() { 0.0 } else { 18.0 };
        tag(&mut frame, cursor, glyph, &label, t.ink, t.line_200, t.bg_200);
        vec![frame.into_geometry()]
    }

    fn mouse_interaction(&self, _: &(), bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        match (cursor.is_over(bounds), self.current()) {
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
