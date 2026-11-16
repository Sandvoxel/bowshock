use super::super::handles::{self, Placed};
use super::{BRACKET, BRACKET_GAP, CIRCLE, GLYPH_EM, SQUARE};
use crate::theme::MONO;
use crate::tokens::{Tokens, TEXT_LABEL};
use bowshock_model::Dim;
use iced::widget::canvas::{Frame, LineDash, Path, Stroke, Text};
use iced::{alignment, Color, Point, Rectangle, Size, Vector};

pub(super) fn bounds(points: &[Point]) -> Rectangle {
    let (mut min, mut max) = (Point::new(f32::MAX, f32::MAX), Point::new(f32::MIN, f32::MIN));
    for p in points {
        (min.x, min.y, max.x, max.y) = (min.x.min(p.x), min.y.min(p.y), max.x.max(p.x), max.y.max(p.y));
    }
    Rectangle::new(min, Size::new(max.x - min.x, max.y - min.y))
}

/// Corner brackets round `b`, solid or dashed.
pub(super) fn brackets(frame: &mut Frame, b: Rectangle, color: Color, dashed: bool) {
    let (x0, y0, x1, y1) = (b.x - BRACKET_GAP, b.y - BRACKET_GAP, b.x + b.width + BRACKET_GAP, b.y + b.height + BRACKET_GAP);
    if dashed {
        let outline = Path::rectangle(Point::new(x0, y0), Size::new(x1 - x0, y1 - y0));
        let stroke = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(color).with_width(1.0) };
        frame.stroke(&outline, stroke);
        return;
    }
    let corners = Path::new(|p| {
        for (x, y, sx, sy) in [(x0, y0, 1.0, 1.0), (x1, y0, -1.0, 1.0), (x1, y1, -1.0, -1.0), (x0, y1, 1.0, -1.0)] {
            p.move_to(Point::new(x + sx * BRACKET, y));
            p.line_to(Point::new(x, y));
            p.line_to(Point::new(x, y + sy * BRACKET));
        }
    });
    frame.stroke(&corners, Stroke::default().with_color(color).with_width(1.5));
}

pub(super) fn text(frame: &mut Frame, content: impl Into<String>, at: Point, color: Color, size: f32, ax: alignment::Horizontal, ay: alignment::Vertical) {
    frame.fill_text(Text { content: content.into(), position: at, color, size: size.into(), font: MONO, align_x: ax.into(), align_y: ay, ..Text::default() });
}

/// The floating view toolbar's extent in the viewport (logical px): handle labels stay clear of it.
pub(super) const TOOLBAR_X: f32 = 600.0;
pub(super) const TOOLBAR_Y: f32 = 76.0;
/// Height of the viewport's bottom row (legend, key hints, scale bar) that labels stay out of, and the free band of that row:
/// the legend (and its gap) at the left, the scale bar (and its gap) at the right.
pub(super) const BOTTOM_STRIP: f32 = 44.0;
pub(super) const BAR_LEFT: f32 = 16.0 + 226.0 + 12.0;
pub(super) const BAR_RIGHT: f32 = 16.0 + 120.0 + 12.0;
/// The ViewCube's distance from the viewport's top-right corner.
pub(super) const CUBE_MARGIN: f32 = 16.0;
/// Side of the square a handle occupies for labels to keep off.
pub(super) const HANDLE_BLOCK: f32 = 14.0;

/// What the key-hint bar calls the thing being dragged (`Drag tip`, `Drag face`, `Drag edge`...).
pub(super) fn drag_verb(dim: Dim, nose: bool) -> &'static str {
    match dim {
        Dim::Length if nose => "Drag tip",
        Dim::Length => "Drag aft face",
        Dim::ForeLength => "Drag fore face",
        Dim::Diameter | Dim::ForeDiameter | Dim::Span | Dim::Height | Dim::TipChord | Dim::RootChord | Dim::Sweep | Dim::Gap => "Drag edge",
        Dim::Wall | Dim::Thickness | Dim::InnerDiameter => "Drag wall",
        Dim::Position => "Drag part",
        Dim::Cant | Dim::Angle => "Drag angle",
        Dim::Shape => "Drag shape",
        Dim::Vertex(_) => "Drag vertex",
        _ => "Drag handle",
    }
}

/// A label with a backing fill so it reads over the model.
pub(super) fn label(frame: &mut Frame, content: &str, at: Point, color: Color, back: Color) {
    let w = content.chars().count() as f32 * TEXT_LABEL * GLYPH_EM;
    let size = frame.size();
    let x = at.x.min(size.width - w - 4.0).max(4.0);
    let y = at.y.clamp(TEXT_LABEL, size.height - 4.0);
    frame.fill_rectangle(Point::new(x - 2.0, y - TEXT_LABEL - 1.0), Size::new(w + 4.0, TEXT_LABEL + 3.0), back.scale_alpha(0.92));
    text(frame, content, Point::new(x, y), color, TEXT_LABEL, alignment::Horizontal::Left, alignment::Vertical::Bottom);
}

/// Where the selection's text goes (see [`Overlay::plan`]).
pub(super) struct Plan {
    /// The selection tag.
    pub(super) tag: Rectangle,
    /// Labels beside the handles in the picture.
    pub(super) side: Vec<Labelled>,
    /// Labels beside the handles in the aft-view inset (inside its panel).
    pub(super) inset: Vec<Labelled>,
    /// Names of the labels that found no free spot.
    pub(super) hidden: Vec<String>,
}

/// A label placed beside one handle.
pub(super) struct Labelled {
    /// Index into `Visible::placed`.
    pub(super) handle: usize,
    pub(super) text: String,
    pub(super) rect: Rectangle,
}

impl Plan {
    /// The note for the key-hint spot when handles were left out: some hidden for crowding (`crowded`) or labels dropped
    /// for want of room. `None` when nothing was left out.
    pub(super) fn note(&self, crowded: bool) -> Option<String> {
        if !crowded && self.hidden.is_empty() {
            return None;
        }
        let mut note = "Zoom in for more handles".to_string();
        if !self.hidden.is_empty() {
            note.push_str(&format!(" · labels hidden: {}", self.hidden.join(", ").to_lowercase()));
        }
        Some(note)
    }
}

/// The size of the backing rectangle of a label (`label` and `label_in` draw the same one).
pub(super) fn label_size(content: &str) -> Size {
    Size::new(content.chars().count() as f32 * TEXT_LABEL * GLYPH_EM + 4.0, TEXT_LABEL + 3.0)
}

/// A label in the rectangle `rect` (as placed by [`labels::layout`]) in the kit's tag style (opaque panel, hairline frame), so it
/// reads over the model.
pub(super) fn label_in(frame: &mut Frame, t: &Tokens, content: &str, rect: Rectangle, color: Color) {
    frame.fill_rectangle(rect.position(), rect.size(), t.bg_200);
    frame.stroke_rectangle(
        Point::new(rect.x + 0.5, rect.y + 0.5),
        Size::new(rect.width - 1.0, rect.height - 1.0),
        Stroke::default().with_color(t.line_200).with_width(1.0),
    );
    text(frame, content, Point::new(rect.x + 2.0, rect.y + rect.height - 2.0), color, TEXT_LABEL, alignment::Horizontal::Left, alignment::Vertical::Bottom);
}

/// A note in the viewport's bottom row, framed like the key-hint bar and centred in the free band between the CG/CP legend and the
/// scale bar: the handles left out (`Zoom in for more handles`) or the way to the hidden aft view of a fin set. The words after a
/// ` · ` are dropped, then the note shortens to `shortest`, until it fits.
pub(super) fn draw_note(frame: &mut Frame, t: &Tokens, size: Size, note: &str, shortest: &str) {
    let (left, right) = (BAR_LEFT, size.width - BAR_RIGHT);
    let short = note.split(" · ").next().unwrap_or(note);
    let fits = |s: &str| s.chars().count() as f32 * (TEXT_LABEL + 1.0) * 0.55 + 24.0 <= right - left;
    let Some(shown) = [note, short, shortest].into_iter().find(|s| fits(s)) else {
        return;
    };
    let w = shown.chars().count() as f32 * (TEXT_LABEL + 1.0) * 0.55 + 24.0;
    let (x, y) = ((left + right - w) / 2.0, size.height - 16.0 - 26.0);
    frame.fill_rectangle(Point::new(x, y), Size::new(w, 26.0), t.bg_200);
    frame.stroke_rectangle(Point::new(x, y), Size::new(w, 26.0), Stroke::default().with_color(t.line_200).with_width(1.0));
    text(frame, shown, Point::new(x + w / 2.0, y + 13.0), t.ink_muted, TEXT_LABEL + 1.0, alignment::Horizontal::Center, alignment::Vertical::Center);
}

pub(super) fn dashed(frame: &mut Frame, a: Point, b: Point, color: Color, dotted: bool) {
    let segments: &[f32] = if dotted { &[1.5, 3.0] } else { &[4.0, 3.0] };
    frame.stroke(&Path::line(a, b), Stroke { line_dash: LineDash { segments, offset: 0 }, ..Stroke::default().with_color(color).with_width(1.0) });
}

pub(super) fn handle_shape(frame: &mut Frame, t: &Tokens, p: &Placed, active: bool, hovered: bool) {
    let stroke = Stroke::default().with_color(t.accent).with_width(1.5);
    let fill = if active {
        t.accent
    } else if hovered {
        t.accent.scale_alpha(0.35)
    } else {
        t.bg_000
    };
    if handles::is_circle(p.spec.kind) {
        let path = Path::circle(p.at, CIRCLE / 2.0);
        frame.fill(&path, fill);
        frame.stroke(&path, stroke);
    } else {
        let corner = Point::new(p.at.x - SQUARE / 2.0, p.at.y - SQUARE / 2.0);
        frame.fill_rectangle(corner, Size::new(SQUARE, SQUARE), fill);
        frame.stroke_rectangle(corner, Size::new(SQUARE, SQUARE), stroke);
    }
}

/// An arrowhead at `tip` pointing along unit `dir`.
pub(super) fn arrow(frame: &mut Frame, from: Point, dir: Vector, len: f32, color: Color) {
    let tip = from + dir * len;
    let side = Vector::new(-dir.y, dir.x);
    frame.stroke(&Path::line(from, tip), Stroke::default().with_color(color).with_width(1.5));
    let head = Path::new(|p| {
        p.move_to(tip);
        p.line_to(tip - dir * 7.0 + side * 3.5);
        p.line_to(tip - dir * 7.0 - side * 3.5);
        p.close();
    });
    frame.fill(&head, color);
}
