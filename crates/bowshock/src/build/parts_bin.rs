//! The parts bin: the rail of categories, the airframe and fins & guides lists (the catalogue of default parts, their ruled
//! rows: silhouette, name, default size), the search, "size to selection" and the symmetry footer (design/components/PartsBin,
//! Symmetry).
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`] and [`State::on_key`] with the shared [`Ctx`]. A press on a row that moves past
//! [`DRAG_SLOP`] starts a drag through `crate::app::drag`; one that does not is a click that attaches the part aft.

use bowshock_model::{NoseShape, PartKind, RadialKind};
use iced::widget::canvas::{self, Frame, Geometry, Path};
use iced::widget::{button, canvas as canvas_widget, checkbox, column, container, mouse_area, row, text, text_input, tooltip, Space};
use iced::{keyboard, mouse, Background, Center, Element, Fill, Point, Rectangle, Renderer, Task, Theme, Widget};

use crate::app::ui::{label, mm, radial_size_label, size_label};
use crate::app::{self, drag, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{PART_ROW_H, RAIL_W, SPACE_1, SPACE_2, TEXT_BODY, TEXT_LABEL};
use crate::widgets::segmented::segmented;

/// Size of a row's silhouette canvas, in logical px.
const SILHOUETTE: (f32, f32) = (40.0, 24.0);
/// Space the drawn profile occupies inside it: full width less a margin, and the largest radius mapped to this height.
const SILHOUETTE_FIT: (f32, f32) = (36.0, 18.0);
/// Opacity of an unavailable row.
const UNAVAILABLE: f32 = 0.45;
/// Movement in px after pressing a row beyond which the press is a drag rather than a click.
const DRAG_SLOP: f32 = 4.0;

/// Symmetry counts for radial parts (design/components/Symmetry).
pub const SYMMETRY: [(u8, &str); 6] = [(1, "1"), (2, "2"), (3, "3"), (4, "4"), (6, "6"), (8, "8")];

/// Angle snap choices for radial parts, in degrees.
pub const ANGLE_SNAP: [(u8, &str); 3] = [(0, "Off"), (15, "15°"), (45, "45°")];

/// The rail's categories.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Category {
    /// Nose, tubes, transitions.
    Airframe,
    /// Fin sets, lugs, rail buttons.
    Fins,
    /// Inner tubes, rings, bulkheads.
    Internal,
    /// Parachutes, streamers, shock cord.
    Recovery,
    /// Avionics, payload, ballast.
    Mass,
    /// Motors.
    Motors,
    /// Saved assemblies.
    Presets,
}

impl Category {
    /// Every category, in rail order.
    pub const ALL: [Category; 7] =
        [Category::Airframe, Category::Fins, Category::Internal, Category::Recovery, Category::Mass, Category::Motors, Category::Presets];

    /// The uppercase header label.
    pub fn label(self) -> &'static str {
        match self {
            Category::Airframe => "AIRFRAME",
            Category::Fins => "FINS & GUIDES",
            Category::Internal => "INTERNAL",
            Category::Recovery => "RECOVERY",
            Category::Mass => "MASS",
            Category::Motors => "MOTORS",
            Category::Presets => "PRESETS",
        }
    }
}

/// The parts bin's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// A rail category was picked.
    Category(Category),
    /// Symmetry count for new radial parts.
    Symmetry(u8),
    /// Angle snap for radial parts, degrees (0 for none).
    AngleSnap(u8),
    /// The search text changed.
    Search(String),
    /// "Size to selection" toggled.
    SizeToSelection(bool),
    /// Left button pressed on a row (index into the category's catalogue).
    RowPress(usize),
    /// The cursor entered or left a row.
    RowHover(Option<usize>),
    /// The cursor moved (sent by the app for every move, so a press can turn into a drag).
    CursorMoved,
    /// Left button released anywhere nothing took it: a press that never became a drag is a click.
    Released,
    /// Escape: forget the press.
    Cancel,
    /// `X` / `Shift+X`: the next (or previous) symmetry count.
    CycleSymmetry(bool),
}

/// A pressed row that has not yet become a click or a drag.
#[derive(Debug)]
struct Press {
    row: usize,
    origin: Option<Point>,
}

/// The parts bin's state.
pub struct State {
    category: Category,
    search: String,
    size_to_selection: bool,
    /// The row under the cursor.
    row_hover: Option<usize>,
    press: Option<Press>,
}

impl State {
    /// The bin on the airframe list.
    pub fn new() -> Self {
        State { category: Category::Airframe, search: String::new(), size_to_selection: true, row_hover: None, press: None }
    }

    /// The reference diameter new parts are sized to, when "size to selection" is on.
    fn sizing(&self, ctx: &Ctx) -> Option<f64> {
        self.size_to_selection.then(|| ctx.doc.rocket().reference_diameter()).flatten()
    }

    /// The part a catalogue row would add right now.
    fn row_kind(&self, row: usize, ctx: &Ctx) -> PartKind {
        sized(&catalogue()[row].1, self.sizing(ctx))
    }

    /// Starts dragging row `row` of the current category, if the part can be placed.
    fn start_drag(&mut self, row: usize, ctx: &mut Ctx) {
        match self.category {
            Category::Airframe => {
                let kind = self.row_kind(row, ctx);
                if !ctx.doc.rocket().nodes_for(&kind).is_empty() {
                    drag::start_axial(ctx, row, catalogue()[row].0, kind);
                }
            }
            Category::Fins if !ctx.doc.rocket().parts().is_empty() => drag::start_radial(ctx, row, radial_catalogue()[row].1.clone()),
            _ => {}
        }
    }

    /// Handles one of the bin's messages.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Category(c) => self.category = c,
            Message::Symmetry(n) => ctx.drag.symmetry = n,
            Message::AngleSnap(degrees) => ctx.drag.angle_snap = degrees,
            Message::Search(s) => self.search = s,
            Message::SizeToSelection(on) => self.size_to_selection = on,
            Message::RowPress(row) => self.press = Some(Press { row, origin: ctx.cursor }),
            Message::RowHover(row) => self.row_hover = row,
            Message::CursorMoved => {
                let at = ctx.cursor;
                let started = self.press.as_ref().filter(|p| matches!((p.origin, at), (Some(o), Some(at)) if o.distance(at) > DRAG_SLOP)).map(|p| p.row);
                if let (Some(row), false) = (started, ctx.drag.active()) {
                    self.press = None;
                    self.start_drag(row, ctx);
                }
            }
            Message::Released => {
                if let Some(press) = self.press.take() {
                    // No drag started: a click attaches aft, at the last node the part may take.
                    match self.category {
                        Category::Airframe => {
                            let kind = self.row_kind(press.row, ctx);
                            if let Some(&(node, _)) = ctx.doc.rocket().nodes_for(&kind).last() {
                                drag::place(ctx, node, kind);
                            }
                        }
                        Category::Fins => drag::attach_radial_aft(ctx, radial_catalogue()[press.row].1.clone()),
                        _ => {}
                    }
                }
            }
            Message::Cancel => self.press = None,
            Message::CycleSymmetry(forward) => {
                let step = |n: u8| {
                    let at = SYMMETRY.iter().position(|(c, _)| *c == n).unwrap_or(0);
                    SYMMETRY[(at + if forward { 1 } else { SYMMETRY.len() - 1 }) % SYMMETRY.len()].0
                };
                match ctx.drag.radial.as_mut() {
                    Some(drag) => {
                        drag.count = step(drag.count);
                        // A group dragged from the bin also sets the count for the next one; a picked-up group keeps its own.
                        if drag.origin.is_none() {
                            ctx.drag.symmetry = drag.count;
                        }
                    }
                    None => ctx.drag.symmetry = step(ctx.drag.symmetry),
                }
            }
        }
        Task::none()
    }

    /// `X` / `Shift+X` cycle the symmetry count. Only called for keys no widget handled.
    pub fn on_key(&self, key: &keyboard::Key, modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        match key {
            keyboard::Key::Character(c) if (modifiers.is_empty() || modifiers == keyboard::Modifiers::SHIFT) && c.eq_ignore_ascii_case("x") => {
                Some(Message::CycleSymmetry(!modifiers.shift()))
            }
            _ => None,
        }
    }

    /// The bin: rail, header, the list of the category, symmetry footer.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let rocket = ctx.doc.rocket();
        let rail = column(Category::ALL.iter().map(|&c| {
            let active = c == self.category;
            let glyph = text(&c.label()[..1]).font(MONO).size(TEXT_BODY);
            row![
                container(Space::new()).width(2).height(RAIL_W).style(theme::underline(active)),
                button(container(glyph).center(Fill)).width(Fill).height(RAIL_W).style(theme::segment(active)).on_press(Message::Category(c)),
            ]
            .boxed()
        }))
        .width(RAIL_W);

        let symmetry = column![
            label("SYMMETRY"),
            segmented(&SYMMETRY, ctx.drag.symmetry, Message::Symmetry),
            label("ANGLE SNAP"),
            segmented(&ANGLE_SNAP, ctx.drag.angle_snap, Message::AngleSnap),
        ]
        .spacing(SPACE_2);

        let rows: Vec<Element<'_, Message>> = match self.category {
            Category::Airframe => {
                let needle = self.search.to_lowercase();
                catalogue()
                    .iter()
                    .enumerate()
                    .map(|(index, (name, _))| (index, *name, self.row_kind(index, ctx)))
                    .filter(|(_, name, _)| name.to_lowercase().contains(&needle))
                    .map(|(index, name, kind)| {
                        let state = if rocket.nodes_for(&kind).is_empty() {
                            RowState::Unavailable
                        } else if ctx.drag.axial.as_ref().is_some_and(|d| d.row == Some(index)) {
                            RowState::Dragging
                        } else if self.row_hover == Some(index) && ctx.drag.axial.is_none() {
                            RowState::Hovered
                        } else {
                            RowState::Idle
                        };
                        entry(index, name, size_label(&kind), axial_outline(&kind), state, "A rocket has one nose cone, at the fore end")
                    })
                    .collect()
            }
            Category::Fins => {
                let needle = self.search.to_lowercase();
                let attachable = !rocket.parts().is_empty();
                radial_catalogue()
                    .iter()
                    .enumerate()
                    .filter(|(_, (name, _))| name.to_lowercase().contains(&needle))
                    .map(|(index, (name, kind))| {
                        let state = if !attachable {
                            RowState::Unavailable
                        } else if ctx.drag.radial.as_ref().is_some_and(|d| d.row == Some(index)) {
                            RowState::Dragging
                        } else if self.row_hover == Some(index) && ctx.drag.radial.is_none() {
                            RowState::Hovered
                        } else {
                            RowState::Idle
                        };
                        entry(index, name, radial_size_label(kind), radial_outline(kind), state, "Needs an airframe part to attach to")
                    })
                    .collect()
            }
            // Internals, recovery, mass, motors and presets are not in the bin yet.
            _ => Vec::new(),
        };

        let search = text_input("Search parts", &self.search).on_input(Message::Search).size(TEXT_BODY).padding([4.0, SPACE_2]).style(theme::input);
        let sizing = row![
            checkbox(self.size_to_selection)
                .label("Size to selection")
                .size(12)
                .text_size(TEXT_LABEL)
                .spacing(6)
                .style(theme::check)
                .on_toggle(Message::SizeToSelection),
            text(rocket.reference_diameter().map(|d| format!("Ø{}", mm(d))).unwrap_or_default()).font(MONO).size(TEXT_LABEL),
        ]
        .spacing(SPACE_1)
        .align_y(iced::Center);

        let header = container(column![label(self.category.label()), search, sizing].spacing(SPACE_2)).padding(theme::PAD_PANEL);
        let list = column![header, column(rows), Space::new().height(Fill), container(symmetry).padding(theme::PAD_PANEL),].width(Fill);

        container(row![rail, list]).height(Fill).style(theme::panel).boxed()
    }
}

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
