//! The manipulation overlay: direct handling of the parts already in the viewport (design/README.md section 7). It hover-
//! picks a part under the cursor, selects on click, picks a part up when it is pressed and dragged, and drags the resize
//! handles of the selected part. It draws the selection (corner brackets, a mono tag, handles), the hover label, and the
//! live value of a resize. Everything it decides is reported as a message; the app owns the rocket.
//!
//! A radial part is a group of linked copies: clicking any copy selects the group, with solid brackets on the clicked copy
//! and dashed ones on its siblings.

use bowshock_model::{PartId, PartKind, RadialKind, RadialPart, Rocket};
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::{alignment, keyboard, mouse, Color, Point, Rectangle, Renderer, Size, Theme};

use super::assembly::tag;
use super::pick::{self, Handle, Pick};
use super::radial::Placement;
use super::Camera;
use crate::parts::{mm, radial_size_label, size_label};
use crate::theme::{self, MONO};
use crate::tokens::TEXT_LABEL;
use crate::Message;

/// Movement in px after pressing a part beyond which it is picked up rather than clicked.
const SLOP: f32 = 4.0;
/// Side of a handle square.
const HANDLE: f32 = 8.0;
/// A handle is grabbed within this many px (per axis) of its centre.
const HANDLE_REACH: f32 = 7.0;
/// Corner bracket leg length, and the gap between the part's bounds and its brackets.
const BRACKET: f32 = 8.0;
const BRACKET_GAP: f32 = 4.0;

/// What is under the cursor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Hover {
    #[default]
    None,
    /// An axial part, or a radial group, by id.
    Part(PartId),
    Handle(Handle),
}

impl Hover {
    /// The part to draw hovered.
    pub fn part(self) -> Option<PartId> {
        match self {
            Hover::Part(id) => Some(id),
            _ => None,
        }
    }

    /// The cursor to show: a grab hand over a part, a resize arrow over a handle.
    pub fn interaction(self) -> mouse::Interaction {
        match self {
            Hover::None => mouse::Interaction::default(),
            Hover::Part(_) => mouse::Interaction::Grab,
            Hover::Handle(Handle::Length | Handle::RootChord | Handle::Sweep | Handle::TipChord) => mouse::Interaction::ResizingHorizontally,
            Hover::Handle(Handle::AftDiameter | Handle::ForeDiameter | Handle::Span) => mouse::Interaction::ResizingVertically,
        }
    }
}

pub struct Overlay<'a> {
    pub rocket: &'a Rocket,
    pub camera: Camera,
    pub selected: Option<PartId>,
    /// Which copy of a selected radial group was clicked.
    pub instance: usize,
    pub hover: Hover,
    /// The cursor in window coordinates (tracked by the app, so it also draws without a live cursor).
    pub cursor: Option<Point>,
}

/// A press on a part (or on empty space) that has not yet become a click or a pick-up.
#[derive(Debug)]
pub struct Press {
    at: Point,
    target: Option<Pick>,
}

/// The part as it was when a handle drag began; every move is applied to this, so snapping never accumulates.
#[derive(Debug, Clone)]
enum Base {
    Axial(PartKind),
    Radial(RadialKind, f32),
}

/// A handle being dragged.
#[derive(Debug)]
pub struct Resize {
    id: PartId,
    handle: Handle,
    base: Base,
    from: Point,
}

#[derive(Debug, Default)]
pub struct State {
    press: Option<Press>,
    resize: Option<Resize>,
    modifiers: keyboard::Modifiers,
}

/// The selected part.
enum Selection<'a> {
    Axial { id: PartId, z0: f64, kind: &'a PartKind },
    Radial { radial: &'a RadialPart, at: Placement },
}

impl Overlay<'_> {
    fn selection(&self) -> Option<Selection<'_>> {
        let id = self.selected?;
        if let Some(index) = self.rocket.index_of(id) {
            return Some(Selection::Axial { id, z0: self.rocket.stations()[index], kind: &self.rocket.parts()[index].kind });
        }
        let radial = self.rocket.radial(id)?;
        let instance = self.instance.min(radial.angles().len().saturating_sub(1));
        Some(Selection::Radial { radial, at: pick::placement(self.rocket, radial, instance)? })
    }

    fn handle_list(&self, size: Size) -> Vec<(Handle, Point)> {
        match self.selection() {
            Some(Selection::Axial { z0, kind, .. }) => pick::handles(&self.camera, size, z0, kind),
            Some(Selection::Radial { radial, at }) => pick::radial_handles(&self.camera, size, &radial.kind, at),
            None => Vec::new(),
        }
    }

    fn handle_at(&self, at: Point, size: Size) -> Option<Handle> {
        self.handle_list(size)
            .into_iter()
            .filter(|(_, p)| (p.x - at.x).abs() <= HANDLE_REACH && (p.y - at.y).abs() <= HANDLE_REACH)
            .min_by(|a, b| a.1.distance(at).total_cmp(&b.1.distance(at)))
            .map(|(handle, _)| handle)
    }

    fn hover_at(&self, at: Point, size: Size) -> Hover {
        if let Some(handle) = self.handle_at(at, size) {
            return Hover::Handle(handle);
        }
        pick::pick(self.rocket, &self.camera, at, size).map_or(Hover::None, |p| Hover::Part(p.id()))
    }

    /// The part a handle drag starts from.
    fn base(&self) -> Option<(PartId, Base)> {
        match self.selection()? {
            Selection::Axial { id, kind, .. } => Some((id, Base::Axial(kind.clone()))),
            Selection::Radial { radial, at } => Some((radial.id, Base::Radial(radial.kind.clone(), at.angle))),
        }
    }
}

/// The message that applies `base` after a drag to `at`.
fn resize_message(camera: &Camera, size: Size, resize: &Resize, at: Point, step: Option<f64>) -> Message {
    match &resize.base {
        Base::Axial(base) => Message::Resize(resize.id, pick::resized(camera, size, resize.handle, base, resize.from, at, step)),
        Base::Radial(base, angle) => Message::ResizeRadial(resize.id, pick::resized_radial(camera, size, resize.handle, base, *angle, resize.from, at, step)),
    }
}

/// Corner brackets round `b`, solid or dashed.
fn brackets(frame: &mut Frame, b: Rectangle, color: Color, dashed: bool) {
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

impl canvas::Program<Message> for Overlay<'_> {
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        let size = bounds.size();
        match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) => {
                state.modifiers = *modifiers;
                None
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key: keyboard::Key::Named(keyboard::key::Named::Escape), .. }) => {
                // Escape abandons a resize, putting the part back as it was.
                let resize = state.resize.take()?;
                let message = match resize.base {
                    Base::Axial(kind) => Message::Resize(resize.id, kind),
                    Base::Radial(kind, _) => Message::ResizeRadial(resize.id, kind),
                };
                Some(Action::publish(message).and_capture())
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let at = cursor.position_in(bounds)?;
                match (self.handle_at(at, size), self.base()) {
                    (Some(handle), Some((id, base))) => state.resize = Some(Resize { id, handle, base, from: at }),
                    _ => state.press = Some(Press { at, target: pick::pick(self.rocket, &self.camera, at, size) }),
                }
                Some(Action::capture())
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                let at = cursor.position().map(|p| Point::new(p.x - bounds.x, p.y - bounds.y))?;
                if let Some(resize) = &state.resize {
                    let message = resize_message(&self.camera, size, resize, at, pick::step(state.modifiers));
                    let changed = match &message {
                        Message::Resize(id, kind) => self.rocket.part(*id).is_some_and(|p| &p.kind != kind),
                        Message::ResizeRadial(id, kind) => self.rocket.radial(*id).is_some_and(|r| &r.kind != kind),
                        _ => false,
                    };
                    return changed.then(|| Action::publish(message).and_capture());
                }
                if let Some(press) = &state.press {
                    if at.distance(press.at) <= SLOP {
                        return None;
                    }
                    // Beyond the slop it is a drag: on a part that picks it up; on empty space it is not a click either.
                    let target = press.target;
                    state.press = None;
                    return target.map(|pick| Action::publish(Message::Grab(pick)).and_capture());
                }
                let hover = cursor.position_in(bounds).map_or(Hover::None, |p| self.hover_at(p, size));
                (hover != self.hover).then(|| Action::publish(Message::Hover(hover)))
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                if state.resize.take().is_some() {
                    return Some(Action::capture());
                }
                let press = state.press.take()?;
                Some(Action::publish(Message::Select(press.target)).and_capture())
            }
            _ => None,
        }
    }

    fn draw(&self, state: &State, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let cursor =
            self.cursor.map(|c| Point::new(c.x - bounds.x, c.y - bounds.y)).filter(|c| (0.0..=size.width).contains(&c.x) && (0.0..=size.height).contains(&c.y));

        // The selection: brackets (solid on the part, or on the clicked copy of a group and dashed on its siblings) with a tag.
        let selection = self.selection();
        let (tag_bounds, tag_text, id) = match &selection {
            Some(Selection::Axial { id, z0, kind }) => {
                let b = pick::screen_bounds(&self.camera, size, *z0, kind);
                brackets(&mut frame, b, t.accent, false);
                (Some(b), kind.name().to_uppercase(), *id)
            }
            Some(Selection::Radial { radial, at }) => {
                for (i, _) in radial.angles().iter().enumerate().filter(|(i, _)| *i != self.instance.min(radial.angles().len() - 1)) {
                    if let Some(sibling) = pick::placement(self.rocket, radial, i) {
                        brackets(&mut frame, pick::radial_bounds(&self.camera, size, &radial.kind, sibling), t.accent.scale_alpha(0.7), true);
                    }
                }
                let b = pick::radial_bounds(&self.camera, size, &radial.kind, *at);
                brackets(&mut frame, b, t.accent, false);
                let text =
                    if radial.count > 1 { format!("{} ×{}", radial.kind.name().to_uppercase(), radial.count) } else { radial.kind.name().to_uppercase() };
                (Some(b), text, radial.id)
            }
            None => (None, String::new(), 0),
        };
        if let Some(b) = tag_bounds {
            frame.fill_text(Text {
                content: tag_text,
                position: Point::new(b.x - BRACKET_GAP, b.y - BRACKET_GAP - 4.0),
                color: t.accent,
                size: TEXT_LABEL.into(),
                font: MONO,
                align_y: alignment::Vertical::Bottom,
                ..Text::default()
            });
            for (handle, at) in self.handle_list(size) {
                let live = state.resize.as_ref().is_some_and(|r| r.id == id && r.handle == handle);
                let fill = if live || self.hover == Hover::Handle(handle) { t.ink } else { t.accent };
                let corner = Point::new(at.x - HANDLE / 2.0, at.y - HANDLE / 2.0);
                frame.fill_rectangle(corner, Size::new(HANDLE, HANDLE), fill);
                frame.stroke_rectangle(corner, Size::new(HANDLE, HANDLE), Stroke::default().with_color(t.bg_000).with_width(1.0));
            }
        }

        if let (Some(resize), Some(cursor)) = (&state.resize, cursor) {
            let current = match (&self.rocket.part(resize.id), self.rocket.radial(resize.id), resize.handle) {
                (Some(part), _, Handle::Length) => Some(("LENGTH", part.kind.length())),
                (Some(part), _, Handle::AftDiameter) => {
                    Some((if matches!(part.kind, PartKind::Transition { .. }) { "AFT DIAMETER" } else { "DIAMETER" }, part.kind.aft_diameter()))
                }
                (Some(part), _, Handle::ForeDiameter) => Some(("FORE DIAMETER", part.kind.fore_diameter())),
                (_, Some(RadialPart { kind: RadialKind::FinSet { span, .. }, .. }), Handle::Span) => Some(("SPAN", *span)),
                (_, Some(RadialPart { kind: RadialKind::FinSet { root_chord, .. }, .. }), Handle::RootChord) => Some(("ROOT CHORD", *root_chord)),
                (_, Some(RadialPart { kind: RadialKind::FinSet { tip_chord, .. }, .. }), Handle::TipChord) => Some(("TIP CHORD", *tip_chord)),
                (_, Some(RadialPart { kind: RadialKind::FinSet { sweep, .. }, .. }), Handle::Sweep) => Some(("SWEEP", *sweep)),
                _ => None,
            };
            if let Some((name, value)) = current {
                tag(&mut frame, cursor, 0.0, &format!("{name} {} mm", mm(value)), t.accent, t.accent, t.bg_200);
            }
        } else if let (Hover::Part(id), Some(cursor)) = (self.hover, cursor) {
            let label = match (self.rocket.part(id), self.rocket.radial(id)) {
                (Some(part), _) => Some(format!("{} · {}", part.kind.name(), size_label(&part.kind))),
                (_, Some(radial)) => {
                    let times = if radial.count > 1 { format!(" ×{}", radial.count) } else { String::new() };
                    Some(format!("{}{times} · {}", radial.kind.name(), radial_size_label(&radial.kind)))
                }
                _ => None,
            };
            if let Some(label) = label {
                tag(&mut frame, cursor, 0.0, &label, t.ink, t.line_200, t.bg_200);
            }
        }
        vec![frame.into_geometry()]
    }

    // No `mouse_interaction`: a layer that reports a cursor hides the mouse from the layers below it (see
    // `scene::Scene::interaction`), and the model underneath needs the middle button and the wheel.
}
