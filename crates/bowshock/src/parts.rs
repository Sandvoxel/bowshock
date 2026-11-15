//! The parts bin's airframe list: the catalogue of default parts, their ruled rows (silhouette, name, default size) and how
//! a default is sized to the rocket (design/components/PartsBin).

use bowshock_model::{NoseShape, PartKind, RadialKind};
use iced::widget::canvas::{self, Frame, Geometry, Path};
use iced::widget::{canvas as canvas_widget, column, container, mouse_area, row, text, tooltip, Space};
use iced::{mouse, Background, Center, Element, Fill, Point, Rectangle, Renderer, Theme, Widget};

use crate::theme::{self, MONO};
use crate::tokens::{PART_ROW_H, SPACE_1, SPACE_2, TEXT_BODY, TEXT_LABEL};
use crate::Message;

/// Size of a row's silhouette canvas, in logical px.
const SILHOUETTE: (f32, f32) = (40.0, 24.0);
/// Space the drawn profile occupies inside it: full width less a margin, and the largest radius mapped to this height.
const SILHOUETTE_FIT: (f32, f32) = (36.0, 18.0);
/// Opacity of an unavailable row.
const UNAVAILABLE: f32 = 0.45;

/// The airframe parts offered, with their default sizes (metres). The names are the bin's: the model calls any contracting
/// transition a boattail, but the bin offers a plain Transition that happens to contract too.
pub fn catalogue() -> [(&'static str, PartKind); 4] {
    [
        ("Nose cone", PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }),
        ("Body tube", PartKind::BodyTube { length: 0.600, diameter: 0.0762 }),
        ("Transition", PartKind::Transition { length: 0.0762, fore_diameter: 0.0762, aft_diameter: 0.054 }),
        ("Boattail", PartKind::Transition { length: 0.050, fore_diameter: 0.0762, aft_diameter: 0.060 }),
    ]
}

/// The radial parts offered ("Fins & guides"), with their default sizes (metres).
pub fn radial_catalogue() -> [(&'static str, RadialKind); 3] {
    [
        ("Fin set", RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.00318 }),
        ("Launch lug", RadialKind::LaunchLug { length: 0.0508, diameter: 0.0095 }),
        ("Rail button", RadialKind::RailButton { height: 0.008, diameter: 0.011 }),
    ]
}

/// The size line of a radial row: a fin set's root and tip chord and span, else `Ø9.5 × 50.8`.
pub fn radial_size_label(kind: &RadialKind) -> String {
    match kind {
        RadialKind::FinSet { root_chord, tip_chord, span, .. } => format!("{} → {} × {}", mm(*root_chord), mm(*tip_chord), mm(*span)),
        RadialKind::LaunchLug { length, diameter } => format!("Ø{} × {}", mm(*diameter), mm(*length)),
        RadialKind::RailButton { height, diameter } => format!("Ø{} × {}", mm(*diameter), mm(*height)),
        RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => format!("{} × {}", mm(kind.extent()), mm(kind.fin_span().unwrap_or(0.0))),
    }
}

/// `kind` with its diameters scaled so the largest end is `diameter` (lengths and shape unchanged); `None` leaves it alone.
pub fn sized(kind: &PartKind, diameter: Option<f64>) -> PartKind {
    let Some(d) = diameter else {
        return kind.clone();
    };
    match *kind {
        PartKind::NoseCone { shape, length, .. } => PartKind::NoseCone { shape, length, diameter: d },
        PartKind::BodyTube { length, .. } => PartKind::BodyTube { length, diameter: d },
        PartKind::Transition { length, fore_diameter, aft_diameter } => {
            let k = d / fore_diameter.max(aft_diameter);
            PartKind::Transition { length, fore_diameter: fore_diameter * k, aft_diameter: aft_diameter * k }
        }
    }
}

/// Metres as millimetres to one decimal, as shown everywhere (`76.2`).
pub fn mm(metres: f64) -> String {
    format!("{:.1}", metres * 1000.0)
}

/// The default size line of a row: `Ø76.2 × 600.0`, or `Ø76.2 → Ø54.0 × 76.2` for a transition.
pub fn size_label(kind: &PartKind) -> String {
    match kind {
        PartKind::Transition { length, fore_diameter, aft_diameter } => format!("Ø{} → Ø{} × {}", mm(*fore_diameter), mm(*aft_diameter), mm(*length)),
        _ => format!("Ø{} × {}", mm(kind.fore_diameter().max(kind.aft_diameter())), mm(kind.length())),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RowState {
    Idle,
    Hovered,
    /// Being dragged into the viewport.
    Dragging,
    /// Nothing may attach it right now: dimmed, and neither draggable nor clickable.
    Unavailable,
}

/// One ruled row of the list. `index` is the position in its catalogue, reported by the row's messages. `outline` is the
/// silhouette (see [`axial_outline`], [`radial_outline`]); an unavailable row shows `reason` in a tooltip.
pub fn entry<'a>(index: usize, name: &'static str, size: String, outline: Vec<Point>, state: RowState, reason: &'static str) -> Element<'a, Message> {
    let alpha = if state == RowState::Unavailable { UNAVAILABLE } else { 1.0 };
    let name = text(name).size(TEXT_BODY).style(move |theme: &Theme| text::Style { color: Some(theme::tokens(theme).ink.scale_alpha(alpha)), selection: None });
    let size = text(size)
        .font(MONO)
        .size(TEXT_LABEL)
        .style(move |theme: &Theme| text::Style { color: Some(theme::tokens(theme).ink_muted.scale_alpha(alpha)), selection: None });
    let silhouette = canvas_widget(Silhouette { outline, alpha }).width(SILHOUETTE.0).height(SILHOUETTE.1);
    let content = row![silhouette, column![name, size].spacing(2)].spacing(SPACE_2).align_y(Center);

    let dragging = state == RowState::Dragging;
    let edge = container(Space::new()).width(2).height(Fill).style(move |theme: &Theme| container::Style {
        background: Some(Background::Color(if dragging { theme::tokens(theme).accent } else { iced::Color::TRANSPARENT })),
        ..container::Style::default()
    });
    let body =
        container(row![edge, container(content).padding([0.0, SPACE_2]).center_y(Fill)]).width(Fill).height(PART_ROW_H - 1.0).style(move |theme: &Theme| {
            let t = theme::tokens(theme);
            let bg = match state {
                RowState::Dragging => Some(t.accent_soft),
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
    mouse_area(ruled)
        .on_press(Message::RowPress(index))
        .on_enter(Message::RowHover(Some(index)))
        .on_exit(Message::RowHover(None))
        .interaction(mouse::Interaction::Grab)
        .boxed()
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
    let top: Vec<(f64, f64)> = profile.iter().map(|&(z, r)| (z, r)).collect();
    let bottom: Vec<(f64, f64)> = profile.iter().rev().map(|&(z, r)| (z, -r)).collect();
    // Height spans -radius..radius over the fit height, centred on the canvas.
    let all: Vec<(f64, f64)> = top.into_iter().chain(bottom).map(|(z, r)| (z, r + radius)).collect();
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

/// A silhouette polygon, filled `part-body`.
struct Silhouette {
    outline: Vec<Point>,
    alpha: f32,
}

impl<Message> canvas::Program<Message> for Silhouette {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let mut frame = Frame::new(renderer, bounds.size());
        if let Some((first, rest)) = self.outline.split_first() {
            let path = Path::new(|b| {
                b.move_to(*first);
                rest.iter().for_each(|p| b.line_to(*p));
                b.close();
            });
            frame.fill(&path, theme::tokens(theme).part_body.scale_alpha(self.alpha));
        }
        vec![frame.into_geometry()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_follow_the_reference_diameter() {
        let [(_, nose), (_, tube), (_, transition), _] = catalogue();
        assert_eq!(size_label(&tube), "Ø76.2 × 600.0");
        assert_eq!(size_label(&transition), "Ø76.2 → Ø54.0 × 76.2");
        assert_eq!(sized(&nose, Some(0.1)).fore_diameter().max(sized(&nose, Some(0.1)).aft_diameter()), 0.1);
        // A transition keeps its ratio and its length.
        let big = sized(&transition, Some(0.1524));
        assert_eq!(size_label(&big), "Ø152.4 → Ø108.0 × 76.2");
        assert_eq!(sized(&tube, None), tube);
    }
}
