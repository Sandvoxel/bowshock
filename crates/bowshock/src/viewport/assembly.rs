//! The drag overlay: while a part is dragged in from the parts bin or picked up it shows the valid attach nodes on the
//! rocket's centreline and, over a tube's side, the radial surface target (design/components/ViewportOverlay). A node within
//! [`NODE_PX`] of the cursor wins (the part stacks); otherwise the tube under the cursor does: the part becomes a pod, `count`
//! copies standing on the tube's surface (KSP-style radial attachment of an ordinary part); else the nearest node. The ghost part
//! itself is drawn in 3D by the model mesh.

use bowshock_model::{InternalPart, Part, PartId, PartKind, PodBody, RadialKind, RadialPart, Rocket};
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::{alignment, mouse, Color, Point, Rectangle, Renderer, Size, Theme};

use super::pick::{self, SurfaceTarget};
use super::pods::PodView;
use super::radial;
use super::{Camera, Message};
use crate::app::ui::mm;
use crate::theme::{self, MONO};
use crate::tokens::TEXT_LABEL;

/// How close (px) the cursor must be to an attach node for the part to stack there rather than attach to the tube's side.
pub const NODE_PX: f32 = 26.0;
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
    /// The tube side it would attach to radially instead (the part becomes a pod): at most one of `snap`, `pod_snap` and
    /// `surface` is set.
    pub surface: Option<SurfaceTarget>,
    /// Copies of the pod it would become on a tube's side (the bin's symmetry count, or the old pod's).
    pub count: u8,
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
    /// Angle snap for the radial target, degrees (0 for none).
    pub snap_angle: u8,
    /// The view toolbar's grid step, metres.
    pub snap_step: f64,
    /// The margin of the rocket as it would be if released now, calibers: appended to the cursor tag in its stability band's
    /// colour. `None` shows nothing (no valid target).
    pub margin: Option<f64>,
}

/// The pod body the dragged part would be on a tube's side: that one part with what stands on and in it, ids kept for a part
/// of the rocket, placeholder ids for a new one.
pub fn body_of(drag: &Drag) -> PodBody {
    match &drag.origin {
        Some(o) => PodBody::single(Part { kind: drag.kind.clone(), ..o.part.clone() }, o.radials.clone(), o.internals.clone()),
        None => PodBody::single(Part::new(0, drag.kind.clone()), Vec::new(), Vec::new()),
    }
}

/// What the cursor chooses.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Choice {
    Node(Snap),
    Surface(SurfaceTarget),
}

/// Interaction state: trackpad scroll not yet worth a step.
#[derive(Debug, Default)]
pub struct State {
    wheel: f32,
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

    /// What the cursor at `at` chooses: a node within [`NODE_PX`], else the tube side under it (a part that carries a pod cannot
    /// become one), else the nearest node.
    fn choice(&self, at: Point, size: Size) -> Option<Choice> {
        let nearest = self.nodes(size).into_iter().map(|n| (n.snap, n.at.distance(at))).min_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((snap, _)) = nearest.filter(|n| n.1 <= NODE_PX) {
            return Some(Choice::Node(snap));
        }
        if !self.carries_pod() {
            let kind = RadialKind::Pod { gap: 0.0, body: body_of(self.drag) };
            let snapping = pick::Snapping { angle: self.snap_angle, station: Some(self.snap_step) };
            if let Some(target) = pick::surface_target(self.rocket, &self.camera, at, size, &kind, snapping) {
                return Some(Choice::Surface(target));
            }
        }
        // Away from every node and any tube's side there is no target ("no node"); an empty rocket has only its first node.
        nearest.filter(|_| self.rocket.parts().is_empty()).map(|(snap, _)| Choice::Node(snap))
    }

    /// Where the drag is snapped now.
    fn current(&self) -> Option<Choice> {
        self.drag
            .surface
            .map(Choice::Surface)
            .or_else(|| self.drag.snap.map(|i| Choice::Node(Snap::Core(i))))
            .or_else(|| self.drag.pod_snap.map(|(pod, index)| Choice::Node(Snap::Pod(pod, index))))
    }

    /// The node the drag is snapped to, if it stacks.
    fn current_node(&self) -> Option<Snap> {
        match self.current() {
            Some(Choice::Node(snap)) => Some(snap),
            _ => None,
        }
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
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        if let Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) = event {
            // Releasing over the viewport drops (or, with no snap target, cancels); elsewhere the app cancels.
            return cursor.position_over(bounds).map(|_| Action::publish(Message::DropAxial).and_capture());
        }
        if let (Event::Mouse(mouse::Event::WheelScrolled { delta }), true) = (event, self.drag.surface.is_some()) {
            // Over a tube's side the wheel steps the number of copies rather than zooming.
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
        let choice = cursor.position_in(bounds).and_then(|p| self.choice(p, bounds.size()));
        (choice != self.current()).then(|| {
            Action::publish(match choice {
                Some(Choice::Node(Snap::Pod(pod, index))) => Message::SnapPodNode(Some((pod, index))),
                Some(Choice::Node(Snap::Core(index))) => Message::SnapNode(Some(index)),
                Some(Choice::Surface(target)) => Message::AxialSurface(Some(target)),
                None => Message::SnapNode(None),
            })
        })
    }

    fn draw(&self, _: &State, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let nodes = self.nodes(size);

        // Parts after the ghost have moved aft by its length: draw their nodes where those parts now are. Snapping keeps
        // to the settled positions, so the target does not slide under the cursor.
        let ghost = self.drag.snap.map(|snap| (snap, self.rocket.matched(snap, &self.drag.kind).length()));
        let current = self.current_node();
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

        if let Some(target) = self.drag.surface {
            // Radial: a ring where the part would stand on the tube, a leader to it and what releasing does.
            let extent = self.drag.kind.length();
            let station = self.rocket.index_of(target.parent).map_or(0.0, |i| self.rocket.stations()[i]);
            let radius = self.rocket.surface_radius(target.parent, target.offset + extent / 2.0).unwrap_or(0.0);
            let (u, _) = radial::dirs(target.angle as f32);
            let at = self.camera.project(u * radius as f32 + Vec3::Z * (station + target.offset + extent / 2.0) as f32, size);
            frame.fill(&Path::circle(at, RING * 1.5), t.ok.scale_alpha(0.18));
            frame.stroke(&Path::circle(at, RING / 2.0), Stroke::default().with_color(t.ok).with_width(1.5));
            let dashed = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(t.ok).with_width(1.0) };
            frame.stroke(&Path::line(cursor, at), dashed);
            let times = if self.drag.count > 1 { format!(" ×{}", self.drag.count) } else { String::new() };
            let parent = self.rocket.part(target.parent).map_or("", |p| p.kind.name());
            let label = format!(
                "{}{times} → {parent} @ {} mm, {:.0}° · radial",
                self.drag.name,
                mm(station + target.offset),
                target.angle.to_degrees().round() % 360.0
            );
            tag_with(&mut frame, cursor, 0.0, &label, margin_suffix(self.rocket, t, self.margin), t.ink, t.line_200, t.bg_200);
            return vec![frame.into_geometry()];
        }

        let target = current.and_then(|snap| nodes.iter().find(|n| n.snap == snap));
        let label = match target {
            Some(node) => {
                // A dashed accent outline round the ghost where the view is square-on (the 3D view shows its edge instead).
                if let (Snap::Core(index), true) = (node.snap, self.camera.is_ortho()) {
                    let b = pick::screen_bounds(&self.camera, size, node.station, &self.rocket.matched(index, &self.drag.kind));
                    let outline = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(t.accent).with_width(1.0) };
                    frame.stroke_rectangle(b.position(), b.size(), outline);
                }
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
        let suffix = margin_suffix(self.rocket, t, self.margin.filter(|_| target.is_some()));
        tag_with(&mut frame, cursor, glyph, &label, suffix, t.ink, t.line_200, t.bg_200);
        vec![frame.into_geometry()]
    }

    fn mouse_interaction(&self, _: &State, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        match (cursor.is_over(bounds), self.current()) {
            (false, _) => mouse::Interaction::default(),
            (true, Some(_)) => mouse::Interaction::Grabbing,
            (true, None) => mouse::Interaction::NotAllowed,
        }
    }
}

/// A framed square tag with `label` in MONO, down-right of `cursor` (`indent` px further right), flipped to stay inside the frame.
pub fn tag(frame: &mut Frame, cursor: Point, indent: f32, label: &str, ink: Color, edge: Color, bg: Color) {
    tag_with(frame, cursor, indent, label, None, ink, edge, bg);
}

/// The text the tag appends for the previewed margin: ` · 2.07 cal` in the colour of the margin's stability band.
pub fn margin_suffix(rocket: &Rocket, t: &crate::tokens::Tokens, margin: Option<f64>) -> Option<(String, Color)> {
    margin.map(|m| (format!(" · {m:.2} cal"), super::markers::margin_color(t, rocket.stability_bands(), m)))
}

/// [`tag`] with `suffix` after the label, in its own colour (the previewed margin).
#[allow(clippy::too_many_arguments)]
pub fn tag_with(frame: &mut Frame, cursor: Point, indent: f32, label: &str, suffix: Option<(String, Color)>, ink: Color, edge: Color, bg: Color) {
    let size = frame.size();
    let chars = label.chars().count() + suffix.as_ref().map_or(0, |(s, _)| s.chars().count());
    let (w, h) = (chars as f32 * TEXT_LABEL * GLYPH_EM + 2.0 * TAG_PAD, TEXT_LABEL + 2.0 * TAG_PAD - 2.0);
    let mut origin = cursor + iced::Vector::new(14.0 + indent, 14.0);
    if origin.x + w > size.width {
        origin.x = cursor.x - 14.0 - w;
    }
    if origin.y + h > size.height {
        origin.y = cursor.y - 14.0 - h;
    }
    frame.fill_rectangle(origin, Size::new(w, h), bg);
    frame.stroke_rectangle(origin, Size::new(w, h), Stroke::default().with_color(edge).with_width(1.0));
    let text = |content: String, x: f32, color: Color| Text {
        content,
        position: Point::new(x, origin.y + h / 2.0),
        color,
        size: TEXT_LABEL.into(),
        font: MONO,
        align_y: alignment::Vertical::Center,
        ..Text::default()
    };
    frame.fill_text(text(label.to_string(), origin.x + TAG_PAD, ink));
    if let Some((suffix, color)) = suffix {
        frame.fill_text(text(suffix, origin.x + TAG_PAD + label.chars().count() as f32 * TEXT_LABEL * GLYPH_EM, color));
    }
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
