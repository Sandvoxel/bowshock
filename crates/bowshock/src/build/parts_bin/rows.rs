//! One ruled row of the bin: silhouette, name, size line in mono (design/components/PartsBin), and the silhouettes themselves.

use bowshock_model::{InternalKind, MassRole, PartKind, RadialKind};
use iced::widget::canvas::{self, Frame, Geometry, LineDash, Path, Stroke};
use iced::widget::{canvas as canvas_widget, column, container, mouse_area, row, text, tooltip, Space};
use iced::{mouse, Background, Center, Element, Fill, Point, Rectangle, Renderer, Theme, Widget};

use super::items::RowId;
use super::presets::Preset;
use super::Message;
use crate::theme::{self, MONO};
use crate::tokens::{PART_ROW_H, SPACE_1, SPACE_2, TEXT_BODY, TEXT_LABEL};

/// Size of a row's silhouette canvas, in logical px.
const SILHOUETTE: (f32, f32) = (40.0, 24.0);
/// Space the drawn profile occupies inside it: full width less a margin, and the largest radius mapped to this height.
const SILHOUETTE_FIT: (f32, f32) = (36.0, 18.0);
/// Opacity of an unavailable row.
const UNAVAILABLE: f32 = 0.45;

/// How a row is drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RowState {
    /// At rest.
    Idle,
    /// Under the cursor.
    Hovered,
    /// Being dragged into the viewport.
    Dragging,
    /// The motor the active configuration uses.
    Current,
    /// Nothing may attach it right now: dimmed, and neither draggable nor clickable.
    Unavailable,
}

/// Everything a row shows.
#[derive(Debug, Clone)]
pub struct RowView {
    /// Which row it is (reported by its messages).
    pub id: RowId,
    /// The part's name.
    pub name: String,
    /// The default size line (mono).
    pub size: String,
    /// The silhouette.
    pub sil: Sil,
    /// How it is drawn.
    pub state: RowState,
    /// Shown in a tooltip when `state` is `Unavailable`.
    pub reason: String,
    /// A right click deletes this preset (index into the bin's presets).
    pub deletable: Option<usize>,
}

/// A row's silhouette in canvas coordinates: filled polygons, solid lines and dashed outlines.
#[derive(Debug, Clone, Default)]
pub struct Sil {
    fills: Vec<Vec<Point>>,
    lines: Vec<Vec<Point>>,
    dashed: Vec<Vec<Point>>,
}

impl Sil {
    /// One filled polygon.
    pub fn fill(outline: Vec<Point>) -> Sil {
        Sil { fills: vec![outline], ..Sil::default() }
    }
}

/// Maps unit-box points `(x, y)` (both 0..1, y up) into the silhouette box.
fn unit(points: &[(f32, f32)]) -> Vec<Point> {
    let x0 = (SILHOUETTE.0 - SILHOUETTE_FIT.0) / 2.0;
    let bottom = (SILHOUETTE.1 + SILHOUETTE_FIT.1) / 2.0;
    points.iter().map(|&(x, y)| Point::new(x0 + x * SILHOUETTE_FIT.0, bottom - y * SILHOUETTE_FIT.1)).collect()
}

fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<(f32, f32)> {
    vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
}

/// Maps points `(x, y)` in a `width` x `height` box (y up) into the silhouette box with `y = 0` on canvas row `baseline`.
fn fit(points: &[(f64, f64)], width: f64, height: f64, baseline: f32) -> Vec<Point> {
    let (w, h) = (f64::from(SILHOUETTE_FIT.0), f64::from(SILHOUETTE_FIT.1));
    let x0 = (SILHOUETTE.0 - SILHOUETTE_FIT.0) / 2.0;
    points.iter().map(|&(x, y)| Point::new(x0 + (x / width * w) as f32, baseline - (y / height * h) as f32)).collect()
}

/// The silhouette of an axial part: its profile mirrored about the axis. Length fills the width and the largest diameter the
/// height, so a long tube reads as a bar rather than a hairline.
pub fn axial_outline(kind: &PartKind) -> Vec<Point> {
    let profile = kind.profile();
    let length = profile.last().map_or(0.0, |p| p.0);
    let radius = profile.iter().fold(0.0f64, |m, p| m.max(p.1));
    if length <= 0.0 || radius <= 0.0 {
        return Vec::new();
    }
    let top = profile.iter().map(|&(z, r)| (z, r));
    let bottom = profile.iter().rev().map(|&(z, r)| (z, -r));
    // Height spans -radius..radius over the fit height, centred on the canvas.
    let all: Vec<(f64, f64)> = top.chain(bottom).map(|(z, r)| (z, r + radius)).collect();
    fit(&all, length, 2.0 * radius, (SILHOUETTE.1 + SILHOUETTE_FIT.1) / 2.0)
}

/// The silhouette of a radial part: a fin in profile, a lug as a tube seen from the side, a rail button as a stem and head.
pub fn radial_outline(kind: &RadialKind) -> Vec<Point> {
    let base = (SILHOUETTE.1 + SILHOUETTE_FIT.1) / 2.0;
    match *kind {
        RadialKind::FinSet { tip_chord, span, sweep, root_chord, .. } => {
            let extent = root_chord.max(sweep + tip_chord);
            fit(&[(0.0, 0.0), (sweep, span), (sweep + tip_chord, span), (root_chord, 0.0)], extent, span, base)
        }
        RadialKind::LaunchLug { .. } => fit(&[(0.0, 0.0), (0.0, 0.4), (1.0, 0.4), (1.0, 0.0)], 1.0, 1.0, base - SILHOUETTE_FIT.1 * 0.3),
        RadialKind::RailButton { .. } => {
            fit(&[(0.0, 0.0), (0.0, 0.7), (0.15, 0.7), (0.15, 1.0), (0.85, 1.0), (0.85, 0.7), (1.0, 0.7), (1.0, 0.0)], 1.0, 1.0, base)
        }
        RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => {
            let (extent, span) = (kind.extent().max(1e-9), kind.fin_span().unwrap_or(0.0).max(1e-9));
            fit(&kind.planform().unwrap_or_default(), extent, span, base)
        }
    }
}

/// A canopy (half ellipse) with shroud lines to a point below it.
fn canopy(width: f32) -> Sil {
    let (c, r) = (0.5, width / 2.0);
    let dome: Vec<(f32, f32)> = (0..=12)
        .map(|i| {
            let a = std::f32::consts::PI * i as f32 / 12.0;
            (c + r * a.cos(), 0.35 + 0.65 * a.sin())
        })
        .collect();
    let lines = [(c - r, 0.35), (c - r / 2.0, 0.35), (c + r / 2.0, 0.35), (c + r, 0.35)].iter().map(|&p| unit(&[p, (c, 0.0)])).collect();
    Sil { fills: vec![unit(&dome)], lines, dashed: vec![] }
}

/// The silhouette of an internal part.
pub fn internal_outline(kind: &InternalKind) -> Sil {
    match kind {
        InternalKind::InnerTube { .. } => Sil { fills: vec![unit(&rect(0.0, 0.2, 1.0, 0.8))], lines: vec![unit(&[(0.0, 0.5), (1.0, 0.5)])], dashed: vec![] },
        InternalKind::CenteringRing { .. } => Sil { fills: vec![unit(&rect(0.4, 0.0, 0.6, 0.3)), unit(&rect(0.4, 0.7, 0.6, 1.0))], ..Sil::default() },
        InternalKind::Bulkhead { .. } => Sil::fill(unit(&rect(0.42, 0.0, 0.58, 1.0))),
        InternalKind::Coupler { .. } => Sil {
            fills: vec![unit(&rect(0.15, 0.3, 0.85, 0.7))],
            lines: vec![],
            dashed: vec![unit(&[(0.0, 0.1), (1.0, 0.1), (1.0, 0.9), (0.0, 0.9), (0.0, 0.1)])],
        },
        InternalKind::EngineBlock { .. } => {
            Sil { fills: vec![unit(&rect(0.4, 0.0, 0.6, 0.3)), unit(&rect(0.4, 0.7, 0.6, 1.0))], lines: vec![unit(&[(0.0, 0.5), (1.0, 0.5)])], dashed: vec![] }
        }
        InternalKind::Parachute { diameter, .. } => canopy(if *diameter < 0.35 { 0.6 } else { 1.0 }),
        InternalKind::Streamer { .. } => {
            Sil::fill(unit(&[(0.0, 0.55), (0.25, 0.7), (0.5, 0.55), (0.75, 0.7), (1.0, 0.55), (1.0, 0.3), (0.75, 0.45), (0.5, 0.3), (0.25, 0.45), (0.0, 0.3)]))
        }
        InternalKind::ShockCord { .. } => Sil {
            lines: vec![unit(&[(0.0, 0.5), (0.15, 0.85), (0.3, 0.15), (0.45, 0.85), (0.6, 0.15), (0.75, 0.85), (0.9, 0.15), (1.0, 0.5)])],
            ..Sil::default()
        },
        InternalKind::MassComponent { role, .. } => match role {
            MassRole::Altimeter => Sil { fills: vec![unit(&rect(0.05, 0.25, 0.95, 0.75))], lines: vec![unit(&[(0.2, 0.5), (0.8, 0.5)])], dashed: vec![] },
            MassRole::Battery => Sil::fill(unit(&[(0.0, 0.2), (0.9, 0.2), (0.9, 0.4), (1.0, 0.4), (1.0, 0.6), (0.9, 0.6), (0.9, 0.8), (0.0, 0.8)])),
            MassRole::Tracker => Sil { fills: vec![unit(&rect(0.1, 0.2, 0.9, 0.65))], lines: vec![unit(&[(0.5, 0.65), (0.5, 1.0)])], dashed: vec![] },
            MassRole::Payload => Sil::fill(unit(&[(0.0, 0.15), (0.8, 0.15), (1.0, 0.5), (0.8, 0.85), (0.0, 0.85)])),
            MassRole::Ballast | MassRole::Generic => Sil::fill(unit(&[(0.15, 0.1), (0.85, 0.1), (1.0, 0.7), (0.0, 0.7)])),
        },
    }
}

/// The silhouette of a motor: case and nozzle.
pub fn motor_outline() -> Sil {
    Sil { fills: vec![unit(&rect(0.0, 0.25, 0.8, 0.75)), unit(&[(0.8, 0.4), (1.0, 0.15), (1.0, 0.85), (0.8, 0.6)])], ..Sil::default() }
}

/// The silhouette of a preset: a fin can, a mount with rings, or a stack of parts.
pub fn preset_outline(preset: &Preset) -> Sil {
    if preset.has_fins() {
        let mut fills = vec![unit(&rect(0.15, 0.3, 0.85, 0.7))];
        fills.push(unit(&[(0.45, 0.3), (0.85, 0.3), (0.95, 0.0), (0.6, 0.0)]));
        fills.push(unit(&[(0.45, 0.7), (0.85, 0.7), (0.95, 1.0), (0.6, 1.0)]));
        Sil { fills, ..Sil::default() }
    } else if preset.has_mount() {
        Sil { fills: vec![unit(&rect(0.15, 0.35, 0.85, 0.65)), unit(&rect(0.2, 0.0, 0.3, 1.0)), unit(&rect(0.7, 0.0, 0.8, 1.0))], ..Sil::default() }
    } else {
        Sil { fills: vec![unit(&rect(0.1, 0.55, 0.9, 0.85)), unit(&rect(0.1, 0.15, 0.9, 0.45))], ..Sil::default() }
    }
}

/// A silhouette, filled `part-body`.
struct Silhouette {
    sil: Sil,
    alpha: f32,
}

fn polygon(points: &[Point], close: bool) -> Option<Path> {
    let (first, rest) = points.split_first()?;
    Some(Path::new(|b| {
        b.move_to(*first);
        rest.iter().for_each(|p| b.line_to(*p));
        if close {
            b.close();
        }
    }))
}

impl<Message> canvas::Program<Message> for Silhouette {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let mut frame = Frame::new(renderer, bounds.size());
        for fill in &self.sil.fills {
            if let Some(path) = polygon(fill, true) {
                frame.fill(&path, t.part_body.scale_alpha(self.alpha));
            }
        }
        let solid = Stroke::default().with_color(t.part_body.scale_alpha(self.alpha)).with_width(1.25);
        for line in &self.sil.lines {
            if let Some(path) = polygon(line, false) {
                frame.stroke(&path, solid);
            }
        }
        let dash = [2.0, 2.0];
        let dashed =
            Stroke { line_dash: LineDash { segments: &dash, offset: 0 }, ..Stroke::default().with_color(t.ink_muted.scale_alpha(self.alpha)).with_width(1.0) };
        for line in &self.sil.dashed {
            if let Some(path) = polygon(line, false) {
                frame.stroke(&path, dashed);
            }
        }
        vec![frame.into_geometry()]
    }
}

/// One ruled row: silhouette, name, size line. A press starts a drag or click; an unavailable row shows its reason in a tooltip.
pub fn entry<'a>(view: RowView) -> Element<'a, Message> {
    let RowView { id, name, size, sil, state, reason, deletable } = view;
    let alpha = if state == RowState::Unavailable { UNAVAILABLE } else { 1.0 };
    let name = text(name).size(TEXT_BODY).style(move |theme: &Theme| text::Style { color: Some(theme::tokens(theme).ink.scale_alpha(alpha)), selection: None });
    let size = text(size)
        .font(MONO)
        .size(TEXT_LABEL)
        .style(move |theme: &Theme| text::Style { color: Some(theme::tokens(theme).ink_muted.scale_alpha(alpha)), selection: None });
    let silhouette = canvas_widget(Silhouette { sil, alpha }).width(SILHOUETTE.0).height(SILHOUETTE.1);
    let content = row![silhouette, column![name, size].spacing(2)].spacing(SPACE_2).align_y(Center);

    let marked = matches!(state, RowState::Dragging | RowState::Current);
    let edge = container(Space::new()).width(2).height(Fill).style(move |theme: &Theme| container::Style {
        background: Some(Background::Color(if marked { theme::tokens(theme).accent } else { iced::Color::TRANSPARENT })),
        ..container::Style::default()
    });
    let body =
        container(row![edge, container(content).padding([0.0, SPACE_2]).center_y(Fill)]).width(Fill).height(PART_ROW_H - 1.0).style(move |theme: &Theme| {
            let t = theme::tokens(theme);
            let bg = match state {
                RowState::Dragging => Some(t.accent_soft),
                RowState::Current => Some(t.bg_300),
                RowState::Hovered => Some(t.bg_400),
                RowState::Idle | RowState::Unavailable => None,
            };
            container::Style { background: bg.map(Background::Color), ..container::Style::default() }
        });
    let rule = container(Space::new())
        .width(Fill)
        .height(1)
        .style(|theme: &Theme| container::Style { background: Some(Background::Color(theme::tokens(theme).line_100)), ..container::Style::default() });
    let ruled = column![body, rule];

    if state == RowState::Unavailable {
        let reason = container(text(reason).size(TEXT_LABEL)).padding(SPACE_1).style(theme::overlay);
        return tooltip(ruled, reason).position(tooltip::Position::Right).boxed();
    }
    let area = mouse_area(ruled)
        .on_press(Message::RowPress(id))
        .on_enter(Message::RowHover(Some(id)))
        .on_exit(Message::RowHover(None))
        .interaction(mouse::Interaction::Grab);
    match deletable {
        Some(index) => area.on_right_press(Message::DeletePreset(index)).boxed(),
        None => area.boxed(),
    }
}
