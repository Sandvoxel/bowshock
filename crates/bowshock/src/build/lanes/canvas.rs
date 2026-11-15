//! The Station Lanes canvas: draws the ruler, lanes, bars, balance marks and drag feedback, hit-tests with the cursor and turns
//! mouse input into the lanes' [`Message`]s. All geometry comes from [`geometry`](super::geometry); all data from the
//! [`Scene`]. Nothing here edits the document.

use std::time::{Duration, Instant};

use bowshock_model::{PartClass, PartId, Rocket, System};
use iced::advanced::text::Alignment;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::{alignment, keyboard, mouse, Color, Pixels, Point, Rectangle, Renderer, Size, Theme};

use super::edit::{self, End, SNAP_PX};
use super::geometry::{self, Axis, Hit, LaneBox, Zone, Zoom, FOOTER_H, LABEL_W, RULER_H};
use super::pods::{self, PodLayout};
use super::scene::{describe, Bar, BarKey, Scene, MASS_BIN};
use super::Message;
use crate::app::{ui, Selection};
use crate::theme::{self, MONO, SANS};
use crate::tokens::Tokens;
use crate::units::{Quantity, UnitSystem};
use crate::viewport::Camera;

const TEXT_BAR: f32 = 11.5;
pub(super) const TEXT_SMALL: f32 = 10.5;
const MONO_EM: f32 = 0.6;
/// Movement before a press becomes a drag, px.
const DRAG_SLOP: f32 = 3.0;
/// Two presses this close in time are a double click.
const DOUBLE_CLICK: Duration = Duration::from_millis(400);
/// Tallest mass bar in the ruler, px.
const MASS_H: f32 = 16.0;
/// Width of the collapse caret's hit area at the right end of the ruler.
const CARET_W: f32 = 24.0;

/// A part being carried over the lanes (from the parts bin, or an airframe part picked up in the lanes): the ghost bar.
#[derive(Clone, Debug, PartialEq)]
pub struct Carry {
    /// What it is called (`Centering ring`).
    pub name: String,
    /// The lane it belongs in: the ghost is drawn there whatever lane the cursor is over.
    pub system: System,
    /// Length along the axis, metres.
    pub extent: f64,
    /// Station of its fore edge where it would land, if it would.
    pub landing: Option<f64>,
}

/// What the cursor is over.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Over {
    Bar(Hit),
    Separation(usize),
    CgLabel,
    CgTick(usize),
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum Mode {
    #[default]
    Idle,
    /// Button down on a bar, not moved far enough to be a drag.
    Pressed { hit: Hit, at: Point, grab: f64 },
    /// Dragging a part along the length.
    Moving { id: PartId, grab: f64 },
    /// Dragging one end of a part.
    Resizing { id: PartId, end: End },
    /// Middle button: panning.
    Panning { last: f32 },
}

/// The canvas's own interaction state.
#[derive(Debug)]
pub struct Interaction {
    mode: Mode,
    over: Option<Over>,
    cursor: Option<Point>,
    modifiers: keyboard::Modifiers,
    last_click: Option<(Instant, Option<BarKey>)>,
}

impl Default for Interaction {
    /// A drive script holds modifiers through `BOWSHOCK_DRIVE_MODIFIERS` (see [`crate::viewport::manipulate::drive_modifiers`]).
    fn default() -> Self {
        Interaction { mode: Mode::default(), over: None, cursor: None, modifiers: crate::viewport::manipulate::drive_modifiers(), last_click: None }
    }
}

impl Interaction {
    /// A drag of the lanes' own is in progress.
    fn dragging(&self) -> bool {
        matches!(self.mode, Mode::Moving { .. } | Mode::Resizing { .. })
    }
}

/// What the canvas draws and reacts to. Built by the lanes' `view` from the document each frame.
pub struct Lanes<'a> {
    /// The bars and marks.
    pub scene: Scene,
    /// The document (for the drag tag's container and the hover tag).
    pub rocket: &'a Rocket,
    /// The user's zoom and pan.
    pub zoom: Zoom,
    /// The viewport's camera when the lanes follow it.
    pub lock: Option<Camera>,
    /// The display unit system.
    pub units: UnitSystem,
    /// What is selected (a separation, or the primary part).
    pub selection: Selection,
    /// Every selected part (several in a multi-selection): each bar of one is bracketed.
    pub parts: Vec<PartId>,
    /// The part just dropped or edited and how strong its flash still is (0 to 1).
    pub flash: Option<(PartId, f32)>,
    /// Whether the CG and the CP are drawn (the status strip's cells toggle them); the margin needs both.
    pub show: (bool, bool),
    /// A part carried over the lanes.
    pub carry: Option<Carry>,
    /// Mass shading pinned on with `M`.
    pub pinned: bool,
    /// Only the ruler.
    pub collapsed: bool,
    /// Lane height (28 px at full size).
    pub lane_h: f32,
}

/// What drawing one bar needs besides the bar.
struct Paint<'a> {
    t: &'a Tokens,
    axis: &'a Axis,
    rows: &'a [LaneBox],
    /// The part being dragged, if any.
    moving: Option<PartId>,
}

/// `c` as it is drawn for a part that has dropped away (30 %).
fn faded(c: Color, dropped: bool) -> Color {
    if dropped {
        c.scale_alpha(0.3)
    } else {
        c
    }
}

fn lane_colors(t: &Tokens, system: System) -> (Color, Color) {
    match system {
        System::Structure => (t.lane_structure, t.part_body),
        System::Aero => (t.lane_aero, t.sys_aero),
        System::Propulsion => (t.lane_propulsion, t.sys_propulsion),
        System::Recovery => (t.lane_recovery, t.sys_recovery),
        System::Avionics => (t.lane_avionics, t.sys_avionics),
    }
}

fn lane_label(system: System) -> &'static str {
    match system {
        System::Structure => "STRUCTURE",
        System::Aero => "AERO",
        System::Propulsion => "PROPULSION",
        System::Recovery => "RECOVERY",
        System::Avionics => "AVIONICS",
    }
}

pub(super) fn mono_w(chars: usize, size: f32) -> f32 {
    chars as f32 * size * MONO_EM
}

/// The width of `text` set in `font` at `size`, as the renderer lays it out.
fn text_w(text: &str, size: f32, font: iced::Font) -> f32 {
    use iced::advanced::text::{self as typeset, Paragraph as _};
    let paragraph = <Renderer as typeset::Renderer>::Paragraph::with_text(typeset::Text {
        content: text,
        bounds: Size::INFINITE,
        size: Pixels(size),
        line_height: typeset::LineHeight::default(),
        font,
        align_x: Alignment::Left,
        align_y: alignment::Vertical::Top,
        shaping: typeset::Shaping::default(),
        wrapping: typeset::Wrapping::None,
        ellipsis: typeset::Ellipsis::None,
        hint_factor: None,
    });
    paragraph.min_width()
}

fn sans_w(text: &str, size: f32) -> f32 {
    text_w(text, size, SANS)
}

#[allow(clippy::too_many_arguments)]
fn put(frame: &mut Frame, content: &str, at: Point, color: Color, size: f32, font: iced::Font, ax: Alignment, ay: alignment::Vertical) {
    frame.fill_text(Text { content: content.to_string(), position: at, color, size: Pixels(size), font, align_x: ax, align_y: ay, ..Text::default() });
}

/// A filled framed tag with its text centred vertically on `y`, its left edge at `x` (or right edge with `right`).
fn tag(frame: &mut Frame, x: f32, y: f32, text: &str, fg: Color, bg: Color, edge: Option<Color>, right: bool) -> Rectangle {
    let (w, h) = (mono_w(text.chars().count(), TEXT_SMALL) + 8.0, 15.0);
    let left = if right { x - w } else { x };
    let left = left.clamp(0.0, (frame.width() - w).max(0.0));
    let rect = Rectangle { x: left, y: y - h / 2.0, width: w, height: h };
    frame.fill_rectangle(rect.position(), rect.size(), bg);
    if let Some(edge) = edge {
        frame.stroke_rectangle(Point::new(rect.x + 0.5, rect.y + 0.5), Size::new(w - 1.0, h - 1.0), Stroke::default().with_color(edge).with_width(1.0));
    }
    put(frame, text, Point::new(left + 4.0, y), fg, TEXT_SMALL, MONO, Alignment::Left, alignment::Vertical::Center);
    rect
}

fn dotted<'a>(color: Color) -> Stroke<'a> {
    Stroke { line_dash: LineDash { segments: &[1.0, 2.0], offset: 0 }, ..Stroke::default().with_color(color).with_width(1.0) }
}

fn dashed<'a>(color: Color) -> Stroke<'a> {
    Stroke { line_dash: LineDash { segments: &[3.0, 2.0], offset: 0 }, ..Stroke::default().with_color(color).with_width(1.0) }
}

fn vline(frame: &mut Frame, x: f32, y0: f32, y1: f32, stroke: Stroke) {
    let x = x.floor() + 0.5;
    frame.stroke(&Path::line(Point::new(x, y0), Point::new(x, y1)), stroke);
}

/// Outline of `r` inset by half a pixel, dashed or solid.
fn outline(frame: &mut Frame, r: Rectangle, stroke: Stroke) {
    frame.stroke(&Path::rectangle(Point::new(r.x + 0.5, r.y + 0.5), Size::new((r.width - 1.0).max(1.0), (r.height - 1.0).max(1.0))), stroke);
}

/// The link glyph of a symmetry group: two interlocked links, 9 px wide.
fn link(frame: &mut Frame, at: Point, color: Color) {
    let stroke = Stroke::default().with_color(color).with_width(1.25);
    for (dx, dy) in [(0.0, 3.0), (3.5, 0.0)] {
        frame.stroke_rectangle(Point::new(at.x + dx + 0.5, at.y + dy + 0.5), Size::new(5.0, 3.0), stroke);
    }
}

impl Lanes<'_> {
    /// The axis in force for a canvas `width` px wide.
    pub fn axis(&self, width: f32) -> Axis {
        self.lock.and_then(|c| Axis::locked(&c, width)).unwrap_or_else(|| self.zoom.apply(Axis::fit(width, self.scene.length)))
    }

    /// The pod brackets and their tags for this axis (which may add a row to a lane for a tag with nowhere else to go).
    fn pod_layout(&self, axis: &Axis, width: f32) -> PodLayout<'_> {
        if self.collapsed {
            return PodLayout { lanes: Vec::new(), pods: Vec::new() };
        }
        pods::layout(&self.scene, axis, width, &|name| sans_w(name, TEXT_BAR))
    }

    fn rows(&self, pods: &PodLayout, height: f32) -> Vec<LaneBox> {
        if self.collapsed {
            Vec::new()
        } else {
            geometry::rows(&pods.lanes, height, self.lane_h)
        }
    }

    /// Whether the mass shading is on: while something is dragged, on the CG label, or pinned.
    fn shading(&self, state: &Interaction) -> bool {
        self.pinned || self.carry.is_some() || state.dragging() || state.over == Some(Over::CgLabel)
    }

    /// Where the CG label sits, for hover.
    fn cg_label_rect(&self, axis: &Axis, height: f32) -> Option<Rectangle> {
        let m = self.scene.marks?;
        let (cg, cp) = (axis.px(m.cg), axis.px(m.cp));
        let w = mono_w(format!("CG {}", ui::bare(m.cg, Quantity::Length, self.units)).chars().count(), TEXT_SMALL) + 8.0;
        let x = if cg <= cp { cg - 3.0 - w } else { cg + 3.0 };
        Some(Rectangle { x, y: height - 12.0, width: w, height: 12.0 })
    }

    fn separation_at(&self, axis: &Axis, p: Point, height: f32) -> Option<usize> {
        if p.x < LABEL_W || p.y > height - FOOTER_H {
            return None;
        }
        self.scene.seps.iter().find(|s| (axis.px(s.station) - p.x).abs() <= 4.0).map(|s| s.stage)
    }

    fn over(&self, axis: &Axis, rows: &[LaneBox], p: Point, height: f32) -> Option<Over> {
        if let Some(rect) = self.cg_label_rect(axis, height) {
            if rect.contains(p) {
                return Some(Over::CgLabel);
            }
        }
        if p.y >= height - 20.0 {
            if let Some(i) = self.scene.other_cgs.iter().position(|o| (axis.px(o.cg) - p.x).abs() <= 3.0) {
                return Some(Over::CgTick(i));
            }
        }
        if let Some(hit) = geometry::hit(&self.scene, rows, axis, p) {
            return Some(Over::Bar(hit));
        }
        self.separation_at(axis, p, height).map(Over::Separation)
    }

    fn local(cursor: mouse::Cursor, bounds: Rectangle) -> Option<Point> {
        cursor.position().map(|p| Point::new(p.x - bounds.x, p.y - bounds.y))
    }

    /// The station of the fore edge a bar being dragged gets: the cursor minus where on the bar it was grabbed, snapped to faces
    /// with Shift.
    fn start_for(&self, key: Option<BarKey>, station: f64, grab: f64, axis: &Axis, shift: bool) -> f64 {
        let start = station - grab;
        let Some(bar) = key.and_then(|k| self.scene.bar(k)) else { return start };
        if !shift {
            return start;
        }
        let tolerance = f64::from(SNAP_PX / axis.ppm.abs().max(1e-3));
        edit::snap_span(start, bar.extent(), &self.scene.faces(Some(bar.key)), tolerance)
    }

    fn zoom_message(&self, bounds: Rectangle, next: Axis) -> Message {
        let fit = Axis::fit(bounds.width, self.scene.length);
        Message::SetZoom { zoom: Zoom::between(fit, next), camera: self.lock }
    }
}

impl canvas::Program<Message> for Lanes<'_> {
    type State = Interaction;

    fn update(&self, state: &mut Interaction, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        let axis = self.axis(bounds.width);
        let pods = self.pod_layout(&axis, bounds.width);
        let rows = self.rows(&pods, bounds.height);
        let local = Self::local(cursor, bounds);
        match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(m)) => {
                state.modifiers = *m;
                None
            }
            Event::Mouse(mouse::Event::CursorLeft) => {
                state.cursor = None;
                (state.over.take().is_some()).then(Action::request_redraw)
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                let p = local?;
                state.cursor = Some(p);
                let shift = state.modifiers.shift();
                match state.mode {
                    Mode::Idle => {
                        let over = if p.x >= 0.0 && p.y >= 0.0 && p.x <= bounds.width && p.y <= bounds.height {
                            self.over(&axis, &rows, p, bounds.height)
                        } else {
                            None
                        };
                        if self.carry.is_some() {
                            return self.carry_at(state, &axis, &rows, p, bounds);
                        }
                        let changed = over != state.over;
                        state.over = over;
                        changed.then(Action::request_redraw)
                    }
                    Mode::Pressed { hit, at, grab } => {
                        if p.distance(at) < DRAG_SLOP {
                            return None;
                        }
                        let bar = self.scene.bar(hit.key)?;
                        let id = bar.part();
                        match hit.zone {
                            Zone::Body if bar.edit.movable => {
                                state.mode = Mode::Moving { id, grab };
                                if bar.class == Some(PartClass::Axial) && bar.pod.is_none() {
                                    return Some(Action::publish(Message::GrabAxial(id)));
                                }
                                let start = self.start_for(Some(hit.key), axis.station(p.x), grab, &axis, shift);
                                Some(Action::publish(Message::MoveTo { id, start }).and_capture())
                            }
                            Zone::Aft | Zone::Fore => {
                                let end = if hit.zone == Zone::Fore { End::Fore } else { End::Aft };
                                state.mode = Mode::Resizing { id, end };
                                let station = self.edge_station(hit.key, axis.station(p.x), &axis, shift);
                                Some(Action::publish(Message::ResizeTo { id, end, station }).and_capture())
                            }
                            Zone::Body => None,
                        }
                    }
                    Mode::Moving { id, grab } => {
                        let station = axis.station(p.x);
                        if let Some(carry) = &self.carry {
                            let start = self.carry_start(station, carry.extent, grab, &axis, shift);
                            return Some(Action::publish(Message::CarryAt(start)).and_capture());
                        }
                        let start = self.start_for(Some(BarKey::Part(id)), station, grab, &axis, shift);
                        Some(Action::publish(Message::MoveTo { id, start }).and_capture())
                    }
                    Mode::Resizing { id, end } => {
                        let station = self.edge_station(BarKey::Part(id), axis.station(p.x), &axis, shift);
                        Some(Action::publish(Message::ResizeTo { id, end, station }).and_capture())
                    }
                    Mode::Panning { last } => {
                        state.mode = Mode::Panning { last: p.x };
                        let next = axis.panned(p.x - last);
                        Some(Action::publish(self.zoom_message(bounds, next)).and_capture())
                    }
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(button)) => {
                let p = local.filter(|p| p.x >= 0.0 && p.y >= 0.0 && p.x <= bounds.width && p.y <= bounds.height)?;
                if self.carry.is_some() {
                    return None;
                }
                match button {
                    mouse::Button::Left => self.press(state, &axis, &rows, p, bounds.width, bounds.height),
                    mouse::Button::Right => {
                        let hit = geometry::hit(&self.scene, &rows, &axis, p);
                        match (hit, geometry::lane_at(&rows, p.y)) {
                            (Some(h), _) => Some(Action::publish(Message::Select(Some(h.key))).and_capture()),
                            (None, Some(lane)) if p.x >= LABEL_W => Some(
                                Action::publish(Message::Menu { at: p, system: lane.lane.system, station: axis.station(p.x), width: bounds.width })
                                    .and_capture(),
                            ),
                            _ => None,
                        }
                    }
                    mouse::Button::Middle => {
                        state.mode = Mode::Panning { last: p.x };
                        Some(Action::capture())
                    }
                    _ => None,
                }
            }
            Event::Mouse(mouse::Event::ButtonReleased(button)) => match (button, state.mode) {
                (mouse::Button::Left, Mode::Moving { id, .. }) => {
                    state.mode = Mode::Idle;
                    // A free fin released on a group's bar asks to rejoin it; anywhere else the move just ends.
                    let group = local.and_then(|p| self.rejoin_target(&axis, &rows, p, id));
                    let message = group.map_or(Message::EditEnd, |group| Message::DropOnGroup { part: id, group });
                    Some(Action::publish(message).and_capture())
                }
                (mouse::Button::Left, Mode::Resizing { .. }) => {
                    state.mode = Mode::Idle;
                    Some(Action::publish(Message::EditEnd).and_capture())
                }
                (mouse::Button::Left, Mode::Pressed { hit, .. }) => {
                    state.mode = Mode::Idle;
                    matches!(hit.key, BarKey::Motor { config, .. } if self.rocket.active_config_id() == config)
                        .then(|| Action::publish(Message::OpenConfigMenu).and_capture())
                }
                (mouse::Button::Left, Mode::Idle) if self.carry.is_some() => {
                    // A part from the bin released over the lanes lands (or, with no landing, goes back).
                    local
                        .filter(|p| p.x >= 0.0 && p.y >= 0.0 && p.x <= bounds.width && p.y <= bounds.height)
                        .map(|_| Action::publish(Message::EditEnd).and_capture())
                }
                (mouse::Button::Middle, Mode::Panning { .. }) => {
                    state.mode = Mode::Idle;
                    Some(Action::capture())
                }
                _ => None,
            },
            Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let p = local.filter(|p| p.x >= 0.0 && p.y >= 0.0 && p.x <= bounds.width && p.y <= bounds.height)?;
                let y = match delta {
                    mouse::ScrollDelta::Lines { y, .. } => *y,
                    mouse::ScrollDelta::Pixels { y, .. } => *y / 40.0,
                };
                if y == 0.0 {
                    return None;
                }
                let k = 1.15f32.powf(y);
                let fit = Axis::fit(bounds.width, self.scene.length);
                // Zoom the axis in force about the cursor; the lanes do not zoom out past the whole rocket unless they are
                // following a camera that has.
                let min = fit.ppm.abs() * if self.lock.is_some() { 0.05 } else { 1.0 };
                let next = axis.zoomed(k, p.x, min, fit.ppm.abs() * 400.0);
                Some(Action::publish(self.zoom_message(bounds, next)).and_capture())
            }
            _ => None,
        }
    }

    fn draw(&self, state: &Interaction, renderer: &Renderer, theme: &Theme, bounds: Rectangle, cursor: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        // Text is drawn over all the shapes of its geometry layer, so what must cover text (short bars over a long bar's
        // label, the lane-name column over bars that scrolled under it) is a layer of its own.
        let (mut base, mut thin, mut over, mut column) =
            (Frame::new(renderer, size), Frame::new(renderer, size), Frame::new(renderer, size), Frame::new(renderer, size));
        let axis = self.axis(size.width);
        let pods = self.pod_layout(&axis, size.width);
        let rows = self.rows(&pods, size.height);
        let unit = *Quantity::Length.unit_for(self.units);
        let busy = self.shading(state);

        // Background: the panel, the ruler strip and the footer strip.
        base.fill_rectangle(Point::ORIGIN, size, t.bg_200);
        base.fill_rectangle(Point::ORIGIN, Size::new(size.width, RULER_H), t.bg_100);
        if !self.collapsed {
            base.fill_rectangle(Point::new(0.0, size.height - FOOTER_H), Size::new(size.width, FOOTER_H), t.bg_100);
        }

        self.draw_ruler(&mut base, t, &axis, &unit, size, busy);
        if !self.collapsed {
            self.draw_lanes(&mut base, &mut thin, &mut over, t, &axis, &rows, &pods, state, size);
        }

        // The collapse caret at the right end of the ruler: down while the lanes are open, up when only the ruler shows.
        let (cx, cy) = (size.width - CARET_W / 2.0, RULER_H / 2.0);
        let tip = if self.collapsed { -2.0 } else { 2.0 };
        let caret = Path::new(|b| {
            b.move_to(Point::new(cx - 4.0, cy - tip));
            b.line_to(Point::new(cx + 4.0, cy - tip));
            b.line_to(Point::new(cx, cy + tip));
            b.close();
        });
        // The kit's panel has no caret at rest: it appears while the cursor is over the lanes (or the lanes are collapsed).
        if self.collapsed || cursor.is_over(bounds) {
            column.fill(&caret, t.ink_faint);
        }

        // The lane-name column sits over everything that scrolled under it.
        column.fill_rectangle(Point::ORIGIN, Size::new(LABEL_W, size.height), t.bg_100);
        column.fill_rectangle(Point::new(LABEL_W - 1.0, 0.0), Size::new(1.0, size.height), t.line_100);
        column.fill_rectangle(Point::new(0.0, RULER_H - 1.0), Size::new(size.width, 1.0), t.line_100);
        let heading = if busy { "TIP · MASS".to_string() } else { format!("{} FROM TIP", unit.symbol.to_uppercase()) };
        put(&mut column, &heading, Point::new(8.0, RULER_H / 2.0), t.ink_faint, 11.0, MONO, Alignment::Left, alignment::Vertical::Center);
        for lane in &rows {
            let (_, edge) = lane_colors(t, lane.lane.system);
            let mid = lane.y + lane.h / 2.0;
            column.fill_rectangle(Point::new(8.0, mid - 3.0), Size::new(6.0, 6.0), edge);
            put(&mut column, lane_label(lane.lane.system), Point::new(20.0, mid), t.ink_faint, 11.0, MONO, Alignment::Left, alignment::Vertical::Center);
            column.fill_rectangle(Point::new(0.0, lane.y + lane.h - 1.0), Size::new(size.width, 1.0), t.line_100);
        }
        vec![base.into_geometry(), thin.into_geometry(), over.into_geometry(), column.into_geometry()]
    }

    fn mouse_interaction(&self, state: &Interaction, _bounds: Rectangle, _cursor: mouse::Cursor) -> mouse::Interaction {
        match (state.mode, state.over) {
            (Mode::Moving { .. }, _) => mouse::Interaction::Grabbing,
            (Mode::Resizing { .. }, _) => mouse::Interaction::ResizingHorizontally,
            (Mode::Panning { .. }, _) => mouse::Interaction::Grabbing,
            _ if self.carry.is_some() => mouse::Interaction::Grabbing,
            (_, Some(Over::Bar(Hit { zone: Zone::Fore | Zone::Aft, .. }))) => mouse::Interaction::ResizingHorizontally,
            (_, Some(Over::Bar(_))) => mouse::Interaction::Grab,
            (_, Some(Over::Separation(_) | Over::CgLabel)) => mouse::Interaction::Pointer,
            _ => mouse::Interaction::default(),
        }
    }
}

impl Lanes<'_> {
    /// The station an end being dragged lands on: the cursor, snapped to faces with Shift.
    fn edge_station(&self, key: BarKey, station: f64, axis: &Axis, shift: bool) -> f64 {
        if !shift {
            return station;
        }
        let tolerance = f64::from(SNAP_PX / axis.ppm.abs().max(1e-3));
        edit::snap_edge(station, &self.scene.faces(Some(key)), tolerance)
    }

    /// A carried part over the lanes: where it would land with the cursor here.
    fn carry_at(&self, state: &mut Interaction, axis: &Axis, rows: &[LaneBox], p: Point, bounds: Rectangle) -> Option<Action<Message>> {
        let inside = p.x >= LABEL_W && p.x <= bounds.width && geometry::lane_at(rows, p.y).is_some();
        let carry = self.carry.as_ref()?;
        let start = self.carry_start(axis.station(p.x), carry.extent, carry.extent / 2.0, axis, state.modifiers.shift());
        inside.then(|| Action::publish(Message::CarryAt(start)))
    }

    /// Where a carried part's fore edge is with the cursor at `station` having grabbed it `grab` from its fore edge; Shift
    /// snaps either of its faces to a face of the rocket.
    fn carry_start(&self, station: f64, extent: f64, grab: f64, axis: &Axis, shift: bool) -> f64 {
        let start = station - grab;
        if !shift {
            return start;
        }
        let tolerance = f64::from(SNAP_PX / axis.ppm.abs().max(1e-3));
        edit::snap_span(start, extent, &self.scene.faces(None), tolerance)
    }

    /// A left press: selects what is under the cursor and starts a possible drag.
    fn press(&self, state: &mut Interaction, axis: &Axis, rows: &[LaneBox], p: Point, bounds_width: f32, height: f32) -> Option<Action<Message>> {
        let now = Instant::now();
        let double = |key: Option<BarKey>| state.last_click.is_some_and(|(at, k)| k == key && now.duration_since(at) <= DOUBLE_CLICK);
        if p.y < RULER_H && p.x >= bounds_width - CARET_W {
            return Some(Action::publish(Message::ToggleCollapsed).and_capture());
        }
        if p.y < RULER_H {
            let was = double(None);
            state.last_click = Some((now, None));
            return was.then(|| Action::publish(Message::ResetView).and_capture());
        }
        if p.x < LABEL_W {
            return None;
        }
        match self.over(axis, rows, p, height) {
            Some(Over::Bar(hit)) => {
                // Shift-click or Cmd/Ctrl-click adds the part to the selection (or removes it): no drag starts.
                if state.modifiers.shift() || state.modifiers.command() || state.modifiers.control() {
                    state.last_click = None;
                    return Some(Action::publish(Message::Toggle(hit.key)).and_capture());
                }
                let was = double(Some(hit.key));
                state.last_click = Some((now, Some(hit.key)));
                let bar = self.scene.bar(hit.key)?;
                let grab = axis.station(p.x) - bar.start;
                state.mode = Mode::Pressed { hit, at: p, grab };
                let message = if was { Message::Focus(bar.part()) } else { Message::Select(Some(hit.key)) };
                Some(Action::publish(message).and_capture())
            }
            Some(Over::Separation(stage)) => {
                state.last_click = None;
                Some(Action::publish(Message::SelectSeparation(stage)).and_capture())
            }
            _ => {
                state.last_click = None;
                Some(Action::publish(Message::Select(None)).and_capture())
            }
        }
    }

    fn draw_ruler(&self, f: &mut Frame, t: &Tokens, axis: &Axis, unit: &crate::units::Unit, size: Size, busy: bool) {
        // Mass shading: a faint bar chart per 50 mm behind the ticks.
        if busy && !self.scene.mass.is_empty() {
            let max = self.scene.mass.iter().copied().fold(0.0f32, f32::max).max(1e-6);
            for (i, &m) in self.scene.mass.iter().enumerate() {
                let (a, b) = (axis.px(i as f64 * MASS_BIN), axis.px((i + 1) as f64 * MASS_BIN));
                let (left, right) = (a.min(b), a.max(b));
                let h = (m / max * MASS_H).max(if m > 0.0 { 1.0 } else { 0.0 });
                f.fill_rectangle(Point::new(left + 1.0, RULER_H - 1.0 - h), Size::new((right - left - 1.0).max(1.0), h), t.ink_muted.scale_alpha(0.28));
            }
        }
        // Stage names either side of each separation; ruler numbers they would run into are left out.
        let stage_names: Vec<(f32, f32, Alignment, String, bool)> = self
            .scene
            .seps
            .iter()
            .flat_map(|sep| {
                let x = axis.px(sep.station);
                let selected = self.selection == Selection::Separation(sep.stage);
                let lower = if sep.lower_dropped { format!("{} · DROPPED", sep.lower.to_uppercase()) } else { sep.lower.to_uppercase() };
                let flip = axis.ppm < 0.0;
                let (fx, ax) = if flip { (x + 4.0, x - 4.0) } else { (x - 4.0, x + 4.0) };
                let (fore_side, aft_side) = if flip { (Alignment::Left, Alignment::Right) } else { (Alignment::Right, Alignment::Left) };
                [(fx, fore_side, sep.upper.to_uppercase(), selected), (ax, aft_side, lower, selected)]
                    .map(|(at, side, text, selected)| (at, mono_w(text.chars().count(), TEXT_SMALL), side, text, selected))
            })
            .collect();
        let covered = |from: f32, to: f32| {
            to > size.width - CARET_W
                || stage_names.iter().any(|(at, w, side, _, _)| {
                    let (a, b) = if matches!(side, Alignment::Right) { (at - w, *at) } else { (*at, at + w) };
                    from < b + 4.0 && to > a - 4.0
                })
        };
        for tick in geometry::ticks(axis, LABEL_W, size.width, unit) {
            let x = tick.x.floor();
            f.fill_rectangle(Point::new(x, RULER_H - 1.0 - tick.height), Size::new(1.0, tick.height), t.ink_faint);
            if let Some(label) = &tick.label {
                let w = mono_w(label.chars().count(), TEXT_SMALL);
                if !covered(x + 4.0, x + 4.0 + w) {
                    put(f, label, Point::new(x + 4.0, 9.0), t.ink_muted, TEXT_SMALL, MONO, Alignment::Left, alignment::Vertical::Center);
                }
            }
        }
        for (at, _, side, text, selected) in stage_names {
            put(f, &text, Point::new(at, 9.0), if selected { t.accent } else { t.ink_muted }, TEXT_SMALL, MONO, side, alignment::Vertical::Center);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_lanes(
        &self,
        base: &mut Frame,
        thin: &mut Frame,
        over: &mut Frame,
        t: &Tokens,
        axis: &Axis,
        rows: &[LaneBox],
        pods: &PodLayout,
        state: &Interaction,
        size: Size,
    ) {
        let foot = size.height - FOOTER_H;
        let f = &mut *base;

        // Faint dotted rules between sub-rows.
        for lane in rows {
            for sub in 1..lane.lane.subs {
                let y = lane.y + (4.0 + geometry::SUB_PITCH * sub as f32) * lane.k - 2.0;
                f.stroke(&Path::line(Point::new(LABEL_W, y.floor() + 0.5), Point::new(size.width, y.floor() + 0.5)), dotted(t.line_100));
            }
        }

        // Separation lines run through every lane (and the ruler); one can be selected but not dragged.
        for sep in &self.scene.seps {
            let selected = self.selection == Selection::Separation(sep.stage);
            let color = if selected { t.accent } else { t.line_200 };
            vline(f, axis.px(sep.station), 0.0, foot, dashed(color));
        }

        // Harness lines, then bars: other configurations' motors, regular bars, then short ones on top.
        for h in &self.scene.harness {
            if let Some(lane) = rows.iter().find(|l| l.lane.system == System::Recovery) {
                let (y, bar_h) = lane.row(h.sub);
                let (a, b) = (axis.px(h.from), axis.px(h.to));
                let y = (y + bar_h / 2.0).floor() + 0.5;
                f.stroke(&Path::line(Point::new(a, y), Point::new(b, y)), dotted(t.sys_recovery));
            }
        }
        self.draw_pod_lines(f, t, rows, pods);
        let paint = Paint { t, axis, rows, moving: if let Mode::Moving { id, .. } | Mode::Resizing { id, .. } = state.mode { Some(id) } else { None } };
        for i in geometry::paint_order(&self.scene) {
            let bar = &self.scene.bars[i];
            if let Some(rect) = geometry::bar_rect(bar, rows, axis) {
                let target: &mut Frame = if bar.thin { &mut *thin } else { &mut *f };
                self.draw_bar(target, &paint, bar, rect);
            }
        }

        // Drag feedback.
        let f = over;
        self.draw_pod_labels(f, t, rows, pods);
        let moving = match state.mode {
            Mode::Moving { id, .. } | Mode::Resizing { id, .. } => self.scene.part_bar(id),
            _ => None,
        };
        if let Some(bar) = moving {
            if matches!(state.mode, Mode::Moving { .. }) && bar.class == Some(PartClass::Internal) {
                if let Some((a, b)) = bar.container {
                    for wall in [a, b] {
                        vline(f, axis.px(wall), RULER_H, foot, dashed(t.ink_muted));
                    }
                }
            }
            if let Some(rect) = geometry::bar_rect(bar, rows, axis) {
                let x = rect.x;
                vline(f, x, RULER_H, rect.y, Stroke::default().with_color(t.accent).with_width(1.0));
                self.draw_move_tag(f, t, bar, axis, x);
            }
        }
        if let Some(carry) = &self.carry {
            self.draw_ghost(f, t, carry, axis, rows, state);
        }

        // CG and CP run through every lane as dotted lines; their stations are labelled in the footer.
        if let Some(marks) = self.scene.marks {
            let (cg_x, cp_x) = (axis.px(marks.cg), axis.px(marks.cp));
            let (show_cg, show_cp) = self.show;
            let both = show_cg && show_cp;
            if show_cg {
                vline(f, cg_x, RULER_H, size.height - 12.0, dotted(t.cg));
            }
            if show_cp {
                vline(f, cp_x, RULER_H, size.height - 12.0, dotted(t.cp));
            }
            let bands = self.rocket.stability_bands();
            let color = if marks.margin >= bands.stable {
                t.ok
            } else if marks.margin >= bands.marginal {
                t.warn
            } else {
                t.err
            };
            let (left, right) = (cg_x.min(cp_x), cg_x.max(cp_x));
            if both {
                f.fill_rectangle(Point::new(left, size.height - 15.0), Size::new((right - left).max(1.0), 3.0), color);
            }
            let ink = |c: Color, x: f32, text: &str, align: Alignment, f: &mut Frame| {
                let w = mono_w(text.chars().count(), TEXT_SMALL) + 8.0;
                let left = match align {
                    Alignment::Right => x - w,
                    Alignment::Center => x - w / 2.0,
                    _ => x,
                };
                f.fill_rectangle(Point::new(left, size.height - 12.0), Size::new(w, 12.0), t.bg_100);
                let tx = match align {
                    Alignment::Right => x - 4.0,
                    Alignment::Center => x,
                    _ => x + 4.0,
                };
                put(f, text, Point::new(tx, size.height - 6.0), c, TEXT_SMALL, MONO, align, alignment::Vertical::Center);
            };
            let cg_text = format!("CG {}", ui::bare(marks.cg, Quantity::Length, self.units));
            let cp_text = format!("CP {}", ui::bare(marks.cp, Quantity::Length, self.units));
            let margin_text = format!("{:.2} cal", marks.margin);
            let (cg_side, cp_side) = if cg_x <= cp_x { (Alignment::Right, Alignment::Left) } else { (Alignment::Left, Alignment::Right) };
            let (cg_at, cp_at) = if cg_x <= cp_x { (cg_x - 3.0, cp_x + 3.0) } else { (cg_x + 3.0, cp_x - 3.0) };
            if show_cg {
                ink(t.cg, cg_at, &cg_text, cg_side, f);
            }
            if show_cp {
                ink(t.cp, cp_at, &cp_text, cp_side, f);
            }
            // The margin sits between the lines; when they are too close for it, it goes beyond the outer label instead.
            let margin_w = mono_w(margin_text.chars().count(), TEXT_SMALL) + 8.0;
            if both {
                if right - left >= margin_w + 6.0 {
                    ink(color, (left + right) / 2.0, &margin_text, Alignment::Center, f);
                } else {
                    let outer_w = mono_w(cp_text.chars().count().max(cg_text.chars().count()), TEXT_SMALL) + 8.0;
                    if cg_x <= cp_x {
                        ink(color, cp_x + 3.0 + outer_w + 2.0, &margin_text, Alignment::Left, f);
                    } else {
                        ink(color, cg_x + 3.0 + outer_w + 2.0, &margin_text, Alignment::Left, f);
                    }
                }
            }
        }
        for other in &self.scene.other_cgs {
            vline(f, axis.px(other.cg), size.height - 18.0, size.height - 9.0, Stroke::default().with_color(t.cg.scale_alpha(0.6)).with_width(1.0));
        }

        // Hover: short parts are named (with their station) because their label cannot fit.
        if let (Some(over), Some(p), false) = (state.over, state.cursor, state.dragging() || self.carry.is_some()) {
            let text = match over {
                Over::Bar(hit) => {
                    self.scene.bar(hit.key).filter(|b| b.thin || b.alt || b.dashed || !self.label_fits(b, axis)).map(|b| describe(b, self.rocket, self.units))
                }
                Over::CgTick(i) => self.scene.other_cgs.get(i).map(|o| format!("{} · CG {}", o.name, ui::bare(o.cg, Quantity::Length, self.units))),
                Over::Separation(stage) => self
                    .scene
                    .seps
                    .iter()
                    .find(|s| s.stage == stage)
                    .map(|s| format!("{} | {} · {}", s.upper, s.lower, ui::bare(s.station, Quantity::Length, self.units))),
                Over::CgLabel => None,
            };
            if let Some(text) = text {
                tag(f, p.x + 10.0, (p.y - 14.0).max(RULER_H + 8.0), &text, t.ink, t.bg_300, Some(t.line_200), false);
            }
        }

        // A free part carried onto a group's bar: the bar it would rejoin is outlined and named.
        if let (Mode::Moving { id, .. }, Some(p)) = (state.mode, state.cursor) {
            let target = self.rejoin_target(axis, rows, p, id).and_then(|g| self.scene.bars.iter().find(|b| b.key == BarKey::Part(g)));
            if let Some((bar, rect)) = target.and_then(|b| geometry::bar_rect(b, rows, axis).map(|r| (b, r))) {
                outline(
                    f,
                    Rectangle { x: rect.x - 2.0, y: rect.y - 2.0, width: rect.width + 4.0, height: rect.height + 4.0 },
                    Stroke::default().with_color(t.accent).with_width(2.0),
                );
                tag(f, p.x + 10.0, (p.y - 14.0).max(RULER_H + 8.0), &format!("Release to rejoin {}", bar.name), t.ink, t.bg_300, Some(t.accent), false);
            }
        }
    }

    fn draw_bar(&self, f: &mut Frame, paint: &Paint, bar: &Bar, rect: Rectangle) {
        let Paint { t, axis, rows, moving } = *paint;
        let dim = |c: Color| faded(c, bar.dropped);
        let (fill, edge) = lane_colors(t, bar.system);
        let selected = self.parts.contains(&bar.part()) && !bar.alt;
        let dragging = moving == Some(bar.part()) && !bar.alt;
        let edge_color = dim(if selected {
            t.accent
        } else if bar.alt {
            edge.scale_alpha(0.55)
        } else {
            edge
        });
        if dragging {
            // A part being moved: the dashed accent outline over a faint accent wash (the kit's drag style).
            f.fill_rectangle(rect.position(), rect.size(), t.bg_200);
            f.fill_rectangle(rect.position(), rect.size(), t.accent_soft);
            outline(f, rect, dashed(t.accent));
        } else if bar.alt {
            // Another configuration's motor: a dashed outline with no fill. Where a vertical edge would cross the label of a
            // bar of the active configuration, that stretch of the edge is left out so the text stays legible.
            let mut cuts: Vec<(f32, f32)> = Vec::new();
            for o in self.scene.bars.iter().filter(|o| !o.alt && o.system == bar.system && o.sub == bar.sub) {
                if let Some(r) = geometry::bar_rect(o, rows, axis) {
                    let dim_w = if o.dim.is_empty() { 0.0 } else { text_w(&o.dim, TEXT_SMALL, MONO) + 6.0 };
                    let x0 = r.x.max(LABEL_W) + 5.0;
                    cuts.push((x0 - 2.0, (x0 + sans_w(&o.name, TEXT_BAR) + dim_w + 2.0).min(r.x + r.width)));
                }
            }
            let stroke = dashed(edge_color);
            let mid = rect.y + rect.height / 2.0;
            let (top, bottom) = (rect.y + 0.5, rect.y + rect.height - 0.5);
            f.stroke(&Path::line(Point::new(rect.x, top), Point::new(rect.x + rect.width, top)), stroke);
            f.stroke(&Path::line(Point::new(rect.x, bottom), Point::new(rect.x + rect.width, bottom)), stroke);
            for x in [rect.x + 0.5, rect.x + rect.width - 0.5] {
                if cuts.iter().any(|&(a, b)| x >= a && x <= b) {
                    f.stroke(&Path::line(Point::new(x, top), Point::new(x, mid - 8.0)), stroke);
                    f.stroke(&Path::line(Point::new(x, mid + 8.0), Point::new(x, bottom)), stroke);
                } else {
                    f.stroke(&Path::line(Point::new(x, top), Point::new(x, bottom)), stroke);
                }
            }
        } else if bar.dashed {
            // A shock cord: a dashed outline with no fill.
            outline(f, rect, dashed(edge_color));
        } else {
            f.fill_rectangle(rect.position(), rect.size(), dim(fill));
            outline(f, rect, Stroke::default().with_color(edge_color).with_width(1.0));
        }
        if selected {
            // Corner brackets, never a fill.
            let (x0, x1, y0, y1) = (rect.x - 4.0, rect.x + rect.width + 4.0, rect.y - 4.0, rect.y + rect.height + 4.0);
            let s = Stroke::default().with_color(t.accent).with_width(2.0);
            for (x, dx) in [(x0 + 1.0, 6.0), (x1 - 1.0, -6.0)] {
                let p = Path::new(|b| {
                    b.move_to(Point::new(x + dx, y0 + 1.0));
                    b.line_to(Point::new(x, y0 + 1.0));
                    b.line_to(Point::new(x, y1 - 1.0));
                    b.line_to(Point::new(x + dx, y1 - 1.0));
                });
                f.stroke(&p, s);
            }
        }
        if let Some((_, level)) = self.flash.filter(|&(id, _)| bar.part() == id && !bar.alt) {
            // A part just dropped or edited: its bar flashes once, an accent wash and outline fading with the viewport's.
            f.fill_rectangle(rect.position(), rect.size(), t.accent_soft.scale_alpha(level));
            outline(f, rect, Stroke::default().with_color(t.accent.scale_alpha(level)).with_width(2.0));
        }
        if bar.warn {
            f.fill_rectangle(Point::new(rect.x + rect.width + 3.0, rect.y + rect.height / 2.0 - 3.0), Size::new(6.0, 6.0), dim(t.warn));
        }
        if bar.alt {
            return;
        }

        // Labels: inside when they fit (starting at the visible left edge, past any short block on top of the bar), else the
        // name beside a narrow bar when there is room, else nothing (hovering names it).
        let (ink, faint) = (dim(t.ink), dim(t.ink_faint));
        let mid = rect.y + rect.height / 2.0;
        let right = rect.x + rect.width;
        let name_w = sans_w(&bar.name, TEXT_BAR);
        let link_w = if bar.count > 1 { 13.0 } else { 0.0 };
        let need_name = link_w + name_w;
        let dim_w = if bar.dim.is_empty() { 0.0 } else { text_w(&bar.dim, TEXT_SMALL, MONO) + 6.0 };
        let mut x = rect.x.max(LABEL_W) + 5.0;
        if !bar.thin {
            for _ in 0..4 {
                let covering = self
                    .scene
                    .bars
                    .iter()
                    .filter(|o| o.thin && !o.alt && o.system == bar.system && o.sub == bar.sub)
                    .map(|o| axis.span(o.start, o.end))
                    .find(|&(a, b)| a < x + need_name + 2.0 && b > x - 2.0);
                match covering {
                    Some((_, b)) => x = b + 4.0,
                    None => break,
                }
            }
        }
        let avail = right - x;
        // A group's link glyph is the first thing to go when the bar is too short for it.
        let link_w = if bar.count > 1 && avail >= link_w + name_w + 4.0 { link_w } else { 0.0 };
        let need_name = link_w + name_w;
        if !bar.dashed && avail >= need_name - 1.0 {
            let mut tx = x;
            if link_w > 0.0 {
                link(f, Point::new(tx, mid - 3.5), if selected { dim(t.accent) } else { ink });
                tx += link_w;
            }
            // A group's per-copy overrides are listed in mono after its name (`B: cant`), in the `warn` colour.
            let ovr_w = if bar.overrides.is_empty() { 0.0 } else { text_w(&bar.overrides, TEXT_SMALL, MONO) + 6.0 };
            let ovr_w = if avail >= need_name + ovr_w + 2.0 { ovr_w } else { 0.0 };
            f.with_clip(Rectangle { x: x - 1.0, y: rect.y, width: avail, height: rect.height }, |f| {
                put(f, &bar.name, Point::new(tx, mid), ink, TEXT_BAR, SANS, Alignment::Left, alignment::Vertical::Center);
                if ovr_w > 0.0 {
                    put(f, &bar.overrides, Point::new(tx + name_w + 6.0, mid), dim(t.warn), TEXT_SMALL, MONO, Alignment::Left, alignment::Vertical::Center);
                }
                let at = tx + name_w + ovr_w;
                if dim_w > 0.0 && avail >= need_name + ovr_w + dim_w + 2.0 && !self.thin_over(bar, axis, at + 4.0, at + 6.0 + dim_w) {
                    put(f, &bar.dim, Point::new(at + 6.0, mid), faint, TEXT_SMALL, MONO, Alignment::Left, alignment::Vertical::Center);
                }
            });
            if ovr_w == 0.0 && !bar.overrides.is_empty() {
                // Too long for the bar: the list goes into the open space ahead of it, ending at its fore edge.
                let w = mono_w(bar.overrides.chars().count(), TEXT_SMALL) + 8.0;
                let left = rect.x - 2.0 - w;
                let blocked = self.scene.bars.iter().any(|o| {
                    o.key != bar.key && !o.alt && o.system == bar.system && o.sub == bar.sub && {
                        let (a, b) = axis.span(o.start, o.end);
                        a < rect.x && b > left - 2.0
                    }
                });
                if !blocked && left > LABEL_W {
                    tag(f, rect.x - 2.0, mid, &bar.overrides, dim(t.warn), t.bg_200, Some(dim(t.warn)), true);
                }
            }
        } else if rect.width < need_name + 8.0 {
            let Some(lane) = rows.iter().find(|l| l.lane.system == bar.system) else { return };
            let label = if bar.count > 1 {
                format!("{} ×{}", bar.name.split(" ×").next().unwrap_or(&bar.name).to_lowercase(), bar.count)
            } else {
                bar.name.to_lowercase()
            };
            let label = if bar.overrides.is_empty() { label } else { format!("{label} {}", bar.overrides) };
            let (lx, w) = (right + 4.0, mono_w(label.chars().count(), TEXT_SMALL));
            // Room: nothing else on this sub-row is under the label, and the block is not itself on top of a longer bar.
            let (row_y, _) = lane.row(bar.sub);
            // Blocked when the label's box (any sub-row it touches, not just its own) would run into another bar of the lane.
            let label_box = Rectangle { x: lx - 1.0, y: row_y + rect.height / 2.0 - 7.0, width: w + 6.0 + if bar.warn { 9.0 } else { 0.0 }, height: 14.0 };
            let blocked = self.scene.bars.iter().any(|o| {
                o.key != bar.key
                    && !o.alt
                    && o.system == bar.system
                    && geometry::bar_rect(o, rows, axis).is_some_and(|r| r.intersects(&label_box) && !(o.thin && r.x < rect.x))
            });
            if !blocked && lx + w < f.width() {
                put(
                    f,
                    &label,
                    Point::new(lx + if bar.warn { 9.0 } else { 0.0 }, row_y + rect.height / 2.0),
                    dim(t.ink_muted),
                    TEXT_SMALL,
                    MONO,
                    Alignment::Left,
                    alignment::Vertical::Center,
                );
            }
        }
    }

    /// The bracket that ties a pod's parts in one lane together: a thin line over the top of their bars with a tick down at each
    /// end, `accent` while the pod is selected.
    fn draw_pod_lines(&self, f: &mut Frame, t: &Tokens, rows: &[LaneBox], pods: &PodLayout) {
        for (s, _) in &pods.pods {
            let Some(lane) = rows.iter().find(|l| l.lane.system == s.system) else { continue };
            let row_y = lane.row(s.row).0;
            let color = faded(if self.parts.contains(&s.id) { t.accent } else { t.ink_muted }, s.dropped);
            let y = (row_y - 2.0).floor() + 0.5;
            let stroke = Stroke::default().with_color(color).with_width(1.0);
            f.stroke(&Path::line(Point::new(s.x0.max(LABEL_W), y), Point::new(s.x1, y)), stroke);
            for x in [s.x0, s.x1] {
                if x >= LABEL_W {
                    vline(f, x - 0.5, y, row_y + 2.0, stroke);
                }
            }
        }
    }

    /// The group tag of each pod bracket (the link glyph and `Pod ×2`), where [`pods::layout`] put it: beside the bracket's ends,
    /// or along it, or in another row of the lane; never left out. Hovering a part names its pod as well.
    fn draw_pod_labels(&self, f: &mut Frame, t: &Tokens, rows: &[LaneBox], pods: &PodLayout) {
        for (s, tag) in &pods.pods {
            let Some(lane) = rows.iter().find(|l| l.lane.system == s.system) else { continue };
            let (row_y, row_h) = lane.row(tag.row);
            let mid = row_y + row_h / 2.0;
            let ink = faded(if self.parts.contains(&s.id) { t.accent } else { t.ink_muted }, s.dropped);
            let rect = Rectangle { x: tag.x, y: mid - pods::TAG_H / 2.0, width: tag.w, height: pods::TAG_H };
            f.fill_rectangle(rect.position(), rect.size(), t.bg_200);
            outline(f, rect, Stroke::default().with_color(faded(t.line_200, s.dropped)).with_width(1.0));
            link(f, Point::new(tag.x + 4.0, mid - 3.5), ink);
            put(f, s.label, Point::new(tag.x + 17.0, mid), ink, TEXT_SMALL, MONO, Alignment::Left, alignment::Vertical::Center);
        }
    }

    /// The radial group a free part `part` dragged to `p` would rejoin: the bar of another radial part of the same family under
    /// the cursor (the dragged bar itself follows the cursor, so it is skipped).
    fn rejoin_target(&self, axis: &Axis, rows: &[LaneBox], p: Point, part: PartId) -> Option<PartId> {
        self.scene.bars.iter().filter(|b| !b.alt).find_map(|b| {
            let BarKey::Part(group) = b.key else { return None };
            let candidate = group != part && b.class == Some(PartClass::Radial) && self.rocket.can_rejoin(part, group);
            (candidate && geometry::bar_rect(b, rows, axis)?.contains(p)).then_some(group)
        })
    }

    /// Whether a short block sits over `bar`'s row between pixels `from` and `to` (a label there would be run through).
    fn thin_over(&self, bar: &Bar, axis: &Axis, from: f32, to: f32) -> bool {
        self.scene.bars.iter().filter(|o| o.thin && !o.alt && o.key != bar.key && o.system == bar.system && o.sub == bar.sub).any(|o| {
            let (a, b) = axis.span(o.start, o.end);
            a < to && b > from
        })
    }

    /// Whether `bar`'s name can be read inside it at the current scale.
    fn label_fits(&self, bar: &Bar, axis: &Axis) -> bool {
        let (a, b) = axis.span(bar.start, bar.end);
        b - a >= sans_w(&bar.name, TEXT_BAR) + 8.0
    }

    /// The accent tag of a part being moved, on the ruler: where it is from the tip, and from the nearest face of what it sits in.
    fn draw_move_tag(&self, f: &mut Frame, t: &Tokens, bar: &Bar, axis: &Axis, x: f32) {
        let len = |m: f64| ui::bare(m, Quantity::Length, self.units);
        let mut text = format!("{} from tip", len(bar.start));
        if let Some((fore, aft)) = bar.container {
            let (to_fore, to_aft) = (bar.start - fore, aft - bar.end);
            if bar.fin {
                text = format!("{text} · {} from aft", len(to_aft));
            } else if to_fore <= to_aft {
                text = format!("{text} · {} from {} front", len(to_fore), bar.container_word);
            } else {
                text = format!("{text} · {} from {} aft", len(to_aft), bar.container_word);
            }
        }
        let _ = axis;
        tag(f, x, RULER_H / 2.0 - 1.0, &text, t.accent_ink, t.accent, None, false);
    }

    /// The dashed accent ghost of a carried part in its own lane, with its station tag on the ruler.
    fn draw_ghost(&self, f: &mut Frame, t: &Tokens, carry: &Carry, axis: &Axis, rows: &[LaneBox], state: &Interaction) {
        let Some(lane) = rows.iter().find(|l| l.lane.system == carry.system) else { return };
        let start = match (carry.landing, state.cursor) {
            (Some(s), _) => s,
            (None, Some(p)) => axis.station(p.x) - carry.extent / 2.0,
            (None, None) => return,
        };
        let label = format!("+ {}", carry.name);
        let (y, h) = lane.row(0);
        let (x0, x1) = axis.span(start, start + carry.extent);
        // Wider than true scale when it must be, so its name fits inside (a ring is 6 mm).
        let x1 = x1.max(x0 + sans_w(&label, TEXT_BAR) + 12.0);
        let rect = Rectangle { x: x0, y, width: x1 - x0, height: h };
        f.fill_rectangle(rect.position(), rect.size(), t.bg_200);
        f.fill_rectangle(rect.position(), rect.size(), t.accent_soft);
        outline(f, rect, dashed(t.accent));
        put(f, &label, Point::new(rect.x + 6.0, rect.y + rect.height / 2.0), t.ink, TEXT_BAR, SANS, Alignment::Left, alignment::Vertical::Center);
        let text = ui::bare(start, Quantity::Length, self.units);
        let busy = carry.landing.is_some();
        tag(f, x0, RULER_H / 2.0 - 1.0, &text, if busy { t.accent_ink } else { t.ink_faint }, if busy { t.accent } else { t.bg_300 }, None, false);
    }
}
