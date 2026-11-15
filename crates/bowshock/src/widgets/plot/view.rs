//! The canvas that draws and drives a [`PlotGroup`]: panels, events strip, selection band, synced crosshair.

use std::time::{Duration, Instant};

use iced::advanced::text::Alignment;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, LineJoin, Path, Stroke, Text};
use iced::{alignment, keyboard, mouse, Color, Element, Fill, Font, Pixels, Point, Rectangle, Renderer, Size, Theme, Widget};

use super::ticks::{decimals_for, even_range, even_ticks, fmt_fixed, nice_step, nice_ticks};
use super::{window_points, LabelAt, LineStyle, Panel, PanelAction, PlotGroup, PlotMessage, PlotState, Tag, YTicks};
use crate::theme::{self, MONO, SANS};
use crate::tokens::{Tokens, TEXT_LABEL};

/// Left gutter for y tick labels.
const GUTTER: f32 = 54.0;
/// Clear space right of the plots (besides the group's label margin).
const PAD_R: f32 = 12.0;
/// Row under each plot for x tick labels.
const TICK_H: f32 = 18.0;
/// Gap above each plot (room for the first y tick label).
const PANEL_TOP: f32 = 6.0;
const STRIP_TIMELINE_H: f32 = 40.0;
const STRIP_MINI_H: f32 = 26.0;
const STRIP_GAP: f32 = 4.0;
/// Reach of a band's edge handle.
const EDGE_GRAB: f32 = 5.0;
/// Shift-drag snaps to an event within this many px.
const SNAP_PX: f32 = 6.0;
/// A press that moves less than this is a click, not a drag.
const DRAG_SLOP: f32 = 3.0;
const DOUBLE_CLICK: Duration = Duration::from_millis(350);
/// Advance of a MONO glyph, in em (IBM Plex Mono is 600/1000).
const GLYPH_EM: f32 = 0.6;
const TAG_H: f32 = 20.0;
const TAG_PAD: f32 = 6.0;
const FONT_TICK: f32 = TEXT_LABEL;
const FONT_SMALL: f32 = 10.0;

/// The linked-plot canvas for one [`PlotGroup`], publishing [`PlotMessage`]s for the app to feed into
/// [`PlotState::update`].
pub struct PlotView<'a> {
    group: &'a PlotGroup,
    state: &'a PlotState,
    pinned: bool,
}

impl<'a> PlotView<'a> {
    /// A view of `group` in the zoom/hover/band state `state`.
    pub fn new(group: &'a PlotGroup, state: &'a PlotState) -> Self {
        PlotView { group, state, pinned: false }
    }

    /// Keeps every editable panel's controls (`+ series`, move, remove) visible, as while a source picker is open; otherwise
    /// they show only under the pointer.
    pub fn pinned(mut self, pinned: bool) -> Self {
        self.pinned = pinned;
        self
    }

    /// The canvas as an element filling its space; `.map` it into the app's message.
    pub fn element(self) -> Element<'a, PlotMessage> {
        canvas::Canvas::new(self).width(Fill).height(Fill).boxed()
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq)]
enum StripKind {
    /// Events with phase bars (Simulate), always shown.
    Timeline,
    /// Minimap of the first series, shown while zoomed.
    Mini,
}

/// Where everything sits, computed from the canvas size and the group; shared by `draw` and `update`.
struct Layout {
    x0: f32,
    x1: f32,
    window: (f64, f64),
    full: (f64, f64),
    strip: Option<(Rectangle, StripKind)>,
    plots: Vec<Rectangle>,
    top: f32,
    bottom: f32,
}

impl Layout {
    fn new(size: Size, group: &PlotGroup, state: &PlotState) -> Layout {
        let x0 = GUTTER;
        let x1 = (size.width - group.right_margin - PAD_R).max(x0 + 1.0);
        // Zoomed in, the overview (ALL: a mini curve with the window boxed) replaces the events strip, in the same height so the
        // plots do not jump (PlotZoom).
        let strip = if state.is_zoomed() {
            let h = if group.timeline { STRIP_TIMELINE_H } else { STRIP_MINI_H };
            Some((Rectangle::new(Point::new(x0, 0.0), Size::new(x1 - x0, h)), StripKind::Mini))
        } else if group.timeline {
            Some((Rectangle::new(Point::new(x0, 0.0), Size::new(x1 - x0, STRIP_TIMELINE_H)), StripKind::Timeline))
        } else {
            None
        };
        let top = strip.map_or(0.0, |(r, _)| r.y + r.height + STRIP_GAP);
        let bottom = (size.height - 2.0).max(top);
        let total: f32 = group.panels.iter().map(|p| p.weight.max(0.01)).sum();
        let mut y = top;
        let plots = group
            .panels
            .iter()
            .map(|p| {
                let slot = (bottom - top) * p.weight.max(0.01) / total.max(0.01);
                let plot = Rectangle::new(Point::new(x0, y + PANEL_TOP), Size::new(x1 - x0, (slot - PANEL_TOP - TICK_H).max(8.0)));
                y += slot;
                plot
            })
            .collect();
        Layout { x0, x1, window: state.window(), full: state.full(), strip, plots, top, bottom }
    }

    fn width(&self) -> f32 {
        (self.x1 - self.x0).max(1.0)
    }

    /// Canvas x of data x.
    fn px(&self, x: f64) -> f32 {
        self.x0 + ((x - self.window.0) / (self.window.1 - self.window.0)) as f32 * self.width()
    }

    /// Data x at canvas x.
    fn x_at(&self, px: f32) -> f64 {
        self.window.0 + ((px - self.x0) / self.width()) as f64 * (self.window.1 - self.window.0)
    }

    /// Canvas x of data x on the overview strip, which always spans the full range.
    fn strip_px(&self, x: f64) -> f32 {
        self.x0 + ((x - self.full.0) / (self.full.1 - self.full.0)) as f32 * self.width()
    }

    fn strip_x_at(&self, px: f32) -> f64 {
        self.full.0 + ((px - self.x0) / self.width()) as f64 * (self.full.1 - self.full.0)
    }

    fn in_plots(&self, p: Point) -> bool {
        !self.plots.is_empty() && p.x >= self.x0 && p.x <= self.x1 && p.y >= self.top && p.y <= self.bottom
    }

    fn in_strip(&self, p: Point) -> bool {
        self.strip.is_some_and(|(r, _)| r.contains(p))
    }

    fn plots_span(&self) -> Option<(f32, f32)> {
        let first = self.plots.first()?;
        let last = self.plots.last()?;
        Some((first.y, last.y + last.height))
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Interaction
// ---------------------------------------------------------------------------------------------------------------------

/// Canvas-local interaction state: the gesture in progress. Everything that outlives a gesture lives in [`PlotState`].
#[derive(Default)]
pub struct Interaction {
    modifiers: keyboard::Modifiers,
    space: bool,
    hovering: bool,
    /// The last pointer position over the canvas.
    pointer: Option<Point>,
    drag: Option<Drag>,
    last_click: Option<(Instant, Point)>,
    /// A Cmd/Ctrl + scroll zoom is in progress (its level closes when the modifier is released).
    pinching: bool,
}

#[derive(Clone, Copy)]
enum Drag {
    /// Drawing a new band from `anchor`; `from` is the press x in px.
    Band { anchor: f64, from: f32, moved: bool },
    /// Moving one edge of the band; `fixed` is the other.
    Edge { fixed: f64 },
    /// Sliding the whole band; `grab` is the press offset from its start.
    Slide { grab: f64, len: f64 },
    /// Panning x: the window and the press x px at the start.
    Pan { window: (f64, f64), from: f32 },
    /// Dragging the overview window: `grab` is the press offset from the window's start.
    Strip { grab: f64 },
}

impl Interaction {
    fn shaping_band(&self) -> bool {
        matches!(self.drag, Some(Drag::Band { moved: true, .. } | Drag::Edge { .. } | Drag::Slide { .. }))
    }
}

impl PlotView<'_> {
    /// `x` snapped to the nearest event within [`SNAP_PX`] when Shift is held.
    fn snap(&self, layout: &Layout, shift: bool, x: f64) -> f64 {
        if !shift {
            return x;
        }
        let px = layout.px(x);
        self.group
            .events
            .iter()
            .filter(|e| (layout.px(e.x) - px).abs() <= SNAP_PX)
            .min_by(|a, b| (layout.px(a.x) - px).abs().total_cmp(&(layout.px(b.x) - px).abs()))
            .map_or(x, |e| e.x)
    }

    fn clamp_x(&self, layout: &Layout, x: f64) -> f64 {
        x.clamp(layout.window.0, layout.window.1)
    }

    fn press(&self, st: &mut Interaction, layout: &Layout, p: Point) -> Option<Action<PlotMessage>> {
        let window = layout.window;
        if let Some((i, chip)) = self.chip_at(layout, p) {
            st.last_click = None;
            return Some(if chip.enabled { Action::publish(PlotMessage::Panel(i, chip.action, chip.rect)).and_capture() } else { Action::capture() });
        }
        if layout.in_strip(p) && self.state.is_zoomed() {
            let x = layout.strip_x_at(p.x);
            if x >= window.0 && x <= window.1 {
                st.drag = Some(Drag::Strip { grab: x - window.0 });
                return Some(Action::capture());
            }
            let half = (window.1 - window.0) / 2.0;
            st.drag = Some(Drag::Strip { grab: half });
            return Some(Action::publish(PlotMessage::Window((x - half, x + half))).and_capture());
        }
        if !layout.in_plots(p) {
            return None;
        }
        let now = Instant::now();
        if let Some((then, at)) = st.last_click {
            if now.duration_since(then) < DOUBLE_CLICK && at.distance(p) < 6.0 {
                st.last_click = None;
                st.drag = None;
                return Some(Action::publish(PlotMessage::Reset).and_capture());
            }
        }
        st.last_click = Some((now, p));
        if st.space {
            st.drag = Some(Drag::Pan { window, from: p.x });
            return Some(Action::capture());
        }
        let x = self.clamp_x(layout, layout.x_at(p.x));
        st.drag = Some(match self.state.band() {
            Some((a, b)) if (p.x - layout.px(a)).abs() <= EDGE_GRAB => Drag::Edge { fixed: b },
            Some((a, b)) if (p.x - layout.px(b)).abs() <= EDGE_GRAB => Drag::Edge { fixed: a },
            Some((a, b)) if x >= a && x <= b => Drag::Slide { grab: x - a, len: b - a },
            _ => Drag::Band { anchor: x, from: p.x, moved: false },
        });
        Some(Action::capture())
    }

    fn moved(&self, st: &mut Interaction, layout: &Layout, p: Option<Point>) -> Option<Action<PlotMessage>> {
        let shift = st.modifiers.shift();
        if let Some(drag) = st.drag {
            let p = p?;
            let band = |a: f64, b: f64| Some(Action::publish(PlotMessage::Band(Some((a, b)))).and_capture());
            return match drag {
                Drag::Band { anchor, from, moved } => {
                    if !moved && (p.x - from).abs() < DRAG_SLOP {
                        return None;
                    }
                    st.drag = Some(Drag::Band { anchor, from, moved: true });
                    let x = self.snap(layout, shift, self.clamp_x(layout, layout.x_at(p.x)));
                    band(anchor, x)
                }
                Drag::Edge { fixed } => {
                    let x = self.snap(layout, shift, self.clamp_x(layout, layout.x_at(p.x)));
                    band(fixed, x)
                }
                Drag::Slide { grab, len } => {
                    let lo = (layout.x_at(p.x) - grab).clamp(layout.window.0, (layout.window.1 - len).max(layout.window.0));
                    band(lo, lo + len)
                }
                Drag::Pan { window, from } => {
                    let dx = f64::from(from - p.x) * (window.1 - window.0) / f64::from(layout.width());
                    Some(Action::publish(PlotMessage::Window((window.0 + dx, window.1 + dx))).and_capture())
                }
                Drag::Strip { grab } => {
                    let lo = layout.strip_x_at(p.x) - grab;
                    let width = layout.window.1 - layout.window.0;
                    Some(Action::publish(PlotMessage::Window((lo, lo + width))).and_capture())
                }
            };
        }
        match p.filter(|p| layout.in_plots(*p)) {
            Some(p) => {
                st.hovering = true;
                let x = layout.x_at(p.x);
                let unchanged = self.state.hover_x().is_some_and(|h| (h - x).abs() < 1e-9 * (layout.window.1 - layout.window.0));
                (!unchanged).then(|| Action::publish(PlotMessage::Hover(Some(x))))
            }
            None => std::mem::take(&mut st.hovering).then(|| Action::publish(PlotMessage::Hover(None))),
        }
    }
}

impl canvas::Program<PlotMessage> for PlotView<'_> {
    type State = Interaction;

    fn update(&self, st: &mut Interaction, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<PlotMessage>> {
        let layout = Layout::new(bounds.size(), self.group, self.state);
        let local = cursor.position().map(|p| Point::new(p.x - bounds.x, p.y - bounds.y));
        let over = cursor.is_over(bounds);
        match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(m)) => {
                st.modifiers = *m;
                (st.pinching && !m.command() && !m.control()).then(|| {
                    st.pinching = false;
                    Action::publish(PlotMessage::PinchEnd)
                })
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key: keyboard::Key::Named(named), modifiers, .. }) => {
                st.modifiers = *modifiers;
                match named {
                    keyboard::key::Named::Space if over => {
                        st.space = true;
                        None
                    }
                    keyboard::key::Named::Enter if over && self.state.can_zoom_in() => Some(Action::publish(PlotMessage::ZoomIn).and_capture()),
                    // Shift+Esc is the PlotZoom "full range" and works wherever the cursor is; plain Esc steps back under the cursor.
                    keyboard::key::Named::Escape if (over || modifiers.shift()) && self.state.can_back() => {
                        let message = if modifiers.shift() { PlotMessage::Reset } else { PlotMessage::Back };
                        Some(Action::publish(message).and_capture())
                    }
                    _ => None,
                }
            }
            Event::Keyboard(keyboard::Event::KeyReleased { key: keyboard::Key::Named(keyboard::key::Named::Space), .. }) => {
                st.space = false;
                None
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => self.press(st, &layout, local?),
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Middle)) => {
                let p = local.filter(|p| layout.in_plots(*p))?;
                st.drag = Some(Drag::Pan { window: layout.window, from: p.x });
                Some(Action::capture())
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left | mouse::Button::Middle)) => {
                let drag = st.drag.take()?;
                match drag {
                    // A click that never became a drag clears the band.
                    Drag::Band { moved: false, .. } if self.state.band().is_some() => Some(Action::publish(PlotMessage::Band(None)).and_capture()),
                    _ => Some(Action::capture()),
                }
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                st.pointer = local;
                self.moved(st, &layout, local)
            }
            Event::Mouse(mouse::Event::CursorLeft) => {
                st.pointer = None;
                if st.drag.is_none() {
                    std::mem::take(&mut st.hovering).then(|| Action::publish(PlotMessage::Hover(None)))
                } else {
                    None
                }
            }
            Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let p = local.filter(|p| layout.in_plots(*p))?;
                let lines = match delta {
                    mouse::ScrollDelta::Lines { y, .. } => *y,
                    mouse::ScrollDelta::Pixels { y, .. } => *y / 40.0,
                };
                if lines == 0.0 {
                    return None;
                }
                let (w0, w1) = layout.window;
                let span = (w1 - w0) * 0.85f64.powf(f64::from(lines));
                if span >= layout.full.1 - layout.full.0 && !self.state.is_zoomed() {
                    return Some(Action::capture());
                }
                let frac = f64::from((p.x - layout.x0) / layout.width());
                let x = layout.x_at(p.x);
                let lo = x - frac * span;
                // Cmd/Ctrl + scroll is the pinch stand-in: it adds a level of its own, closed when the modifier is released.
                if st.modifiers.command() || st.modifiers.control() {
                    st.pinching = true;
                    return Some(Action::publish(PlotMessage::Pinch((lo, lo + span))).and_capture());
                }
                Some(Action::publish(PlotMessage::Window((lo, lo + span))).and_capture())
            }
            _ => None,
        }
    }

    fn mouse_interaction(&self, st: &Interaction, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        match st.drag {
            Some(Drag::Pan { .. } | Drag::Strip { .. }) => return mouse::Interaction::Grabbing,
            Some(Drag::Edge { .. }) => return mouse::Interaction::ResizingHorizontally,
            Some(Drag::Slide { .. }) => return mouse::Interaction::Grabbing,
            _ => {}
        }
        let Some(p) = cursor.position_in(bounds) else { return mouse::Interaction::default() };
        let layout = Layout::new(bounds.size(), self.group, self.state);
        if layout.in_strip(p) && self.state.is_zoomed() {
            return mouse::Interaction::Grab;
        }
        if self.chip_at(&layout, p).is_some_and(|(_, chip)| chip.enabled) {
            return mouse::Interaction::Pointer;
        }
        if !layout.in_plots(p) {
            return mouse::Interaction::default();
        }
        if st.space {
            return mouse::Interaction::Grab;
        }
        match self.state.band() {
            Some((a, b)) if (p.x - layout.px(a)).abs() <= EDGE_GRAB || (p.x - layout.px(b)).abs() <= EDGE_GRAB => mouse::Interaction::ResizingHorizontally,
            Some((a, b)) if (layout.x_at(p.x)) >= a && layout.x_at(p.x) <= b => mouse::Interaction::Grab,
            _ => mouse::Interaction::Crosshair,
        }
    }

    fn draw(&self, st: &Interaction, renderer: &Renderer, theme: &Theme, bounds: Rectangle, cursor: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let mut frame = Frame::new(renderer, bounds.size());
        let layout = Layout::new(bounds.size(), self.group, self.state);
        let ranges: Vec<(f64, f64)> = self
            .group
            .panels
            .iter()
            .map(|p| {
                let (lo, hi) = p.y_range(layout.window.0, layout.window.1);
                match self.group.y_ticks {
                    YTicks::Even(n) => even_range(lo, hi, n),
                    YTicks::Nice => (lo, hi),
                }
            })
            .collect();

        // The strip sits on `bg-100`, the plots on `bg-200`, flush with the panel: no frame around the group.
        frame.fill_rectangle(Point::ORIGIN, bounds.size(), t.bg_200);
        if layout.top > 0.0 {
            frame.fill_rectangle(Point::ORIGIN, Size::new(bounds.width, layout.top), t.bg_100);
        }
        if let Some((rect, kind)) = layout.strip {
            self.draw_strip(&mut frame, t, &layout, rect, kind);
        }
        for ((panel, plot), range) in self.group.panels.iter().zip(&layout.plots).zip(&ranges) {
            self.draw_panel(&mut frame, t, &layout, panel, *plot, *range);
        }
        // The pointer is also tracked from events, so a hosted canvas without a live cursor (the headless driver) still shows hover.
        let hot = cursor.position_in(bounds).or(st.pointer);
        for (panel, plot) in self.group.panels.iter().zip(&layout.plots) {
            self.draw_chrome(&mut frame, t, panel, *plot, hot);
        }
        if let Some(band) = self.state.band() {
            self.draw_band(&mut frame, t, &layout, band);
        }
        if let (Some(x), false) = (self.state.hover_x(), st.shaping_band()) {
            if x >= layout.window.0 && x <= layout.window.1 {
                self.draw_hover(&mut frame, t, &layout, &ranges, x);
            }
        }
        vec![frame.into_geometry()]
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------------------------------------------------

/// A coordinate on a pixel centre, so 1 px lines are crisp at scale 1.
fn crisp(v: f32) -> f32 {
    v.floor() + 0.5
}

fn put(frame: &mut Frame, content: &str, at: Point, color: Color, size: f32, ax: Alignment, ay: alignment::Vertical, font: Font) {
    frame.fill_text(Text { content: content.to_string(), position: at, color, size: Pixels(size), font, align_x: ax, align_y: ay, ..Text::default() });
}

fn stroke<'a>(color: Color, width: f32, style: LineStyle) -> Stroke<'a> {
    let s = Stroke::default().with_color(color).with_width(width).with_line_join(LineJoin::Round);
    match style {
        LineStyle::Solid => s,
        LineStyle::Dashed => Stroke { line_dash: LineDash { segments: &[6.0, 4.0], offset: 0 }, ..s },
        LineStyle::Dotted => Stroke { line_dash: LineDash { segments: &[1.5, 3.0], offset: 0 }, ..s },
        LineStyle::DashDot => Stroke { line_dash: LineDash { segments: &[6.0, 3.0, 1.5, 3.0], offset: 0 }, ..s },
    }
}

fn line(frame: &mut Frame, a: Point, b: Point, color: Color, width: f32, style: LineStyle) {
    frame.stroke(&Path::line(a, b), stroke(color, width, style));
}

/// An event label to place on the strip: the x of its tick, its text width, and whether it is the accented (apogee) event.
#[derive(Clone, Copy)]
struct LabelWant {
    px: f32,
    width: f32,
    accent: bool,
}

/// Places each label in row 0 (above the bar) or 1 (below), to the right of its tick or, failing that, to its left: the
/// accented one first (row 0, always), then the rest in order, alternating rows. `None` for a label with no free spot. Returns
/// `(start x, row)` per label, within `[x0, x1]` and at least 6 px from its neighbours in the row.
fn place_event_labels(wants: &[LabelWant], x0: f32, x1: f32) -> Vec<Option<(f32, usize)>> {
    let mut taken: [Vec<(f32, f32)>; 2] = [Vec::new(), Vec::new()];
    let mut out = vec![None; wants.len()];
    let free = |row: &Vec<(f32, f32)>, a: f32, b: f32| a >= x0 && b <= x1 && row.iter().all(|(s, e)| a >= e + 6.0 || b + 6.0 <= *s);
    let order = (0..wants.len()).filter(|i| wants[*i].accent).chain((0..wants.len()).filter(|i| !wants[*i].accent));
    for (n, i) in order.enumerate() {
        let w = wants[i];
        let rows = if w.accent { [0, 1] } else { [n % 2, (n + 1) % 2] };
        let sides = [(w.px + 3.0).min(x1 - w.width), (w.px - 3.0 - w.width).max(x0)];
        let spot = rows.into_iter().flat_map(|r| sides.into_iter().map(move |s| (r, s))).find(|(r, s)| free(&taken[*r], *s, *s + w.width));
        if let Some((r, s)) = spot {
            taken[r].push((s, s + w.width));
            out[i] = Some((s, r));
        }
    }
    out
}

fn mono_width(chars: usize, size: f32) -> f32 {
    chars as f32 * size * GLYPH_EM
}

/// Thins `points` to at most four samples (first, min, max, last) per pixel column so long series stay cheap and keep
/// their peaks. Series already sparser than that pass through.
fn decimate(points: Vec<(f64, f64)>, w0: f64, w1: f64, columns: usize) -> Vec<(f64, f64)> {
    if points.len() <= columns.max(1) * 4 {
        return points;
    }
    let scale = columns as f64 / (w1 - w0);
    let col = |x: f64| ((x - w0) * scale).floor().clamp(0.0, columns as f64 - 1.0) as usize;
    let mut out = Vec::with_capacity(columns * 4);
    let mut i = 0;
    while i < points.len() {
        let c = col(points[i].0);
        let mut j = i;
        while j < points.len() && col(points[j].0) == c {
            j += 1;
        }
        let run = &points[i..j];
        let min = run.iter().copied().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap_or(run[0]);
        let max = run.iter().copied().max_by(|a, b| a.1.total_cmp(&b.1)).unwrap_or(run[0]);
        let mut pick = [run[0], min, max, run[run.len() - 1]];
        pick.sort_by(|a, b| a.0.total_cmp(&b.0));
        for p in pick {
            if out.last() != Some(&p) {
                out.push(p);
            }
        }
        i = j;
    }
    out
}

impl PlotView<'_> {
    fn draw_strip(&self, f: &mut Frame, t: &Tokens, layout: &Layout, rect: Rectangle, kind: StripKind) {
        let (ink_faint, left) = (t.ink_faint, Alignment::Left);
        let zoomed = self.state.is_zoomed();
        match kind {
            StripKind::Timeline => {
                put(f, "EVENTS", Point::new(6.0, 19.0), ink_faint, FONT_SMALL, left, alignment::Vertical::Center, MONO);
                let bar = (rect.y + 17.0, 4.0);
                f.fill_rectangle(Point::new(rect.x, bar.0), Size::new(rect.width, bar.1), t.line_200);
                for ph in &self.group.phases {
                    let (a, b) = (layout.strip_px(ph.from).max(rect.x), layout.strip_px(ph.to).min(rect.x + rect.width));
                    if b > a {
                        f.fill_rectangle(Point::new(a, bar.0), Size::new(b - a, bar.1), ph.color.resolve(t));
                    }
                }
                // Event ticks; labels sit above and below the bar, the accented event (apogee) always shown and the others
                // shifted to the other row or the other side of their tick to avoid it.
                let mut order: Vec<_> = self.group.events.iter().collect();
                order.sort_by(|a, b| a.x.total_cmp(&b.x));
                let labels: Vec<String> = order.iter().map(|e| format!("{} {}", e.label.to_uppercase(), fmt_fixed(e.x, 1))).collect();
                let wants: Vec<LabelWant> = order
                    .iter()
                    .zip(&labels)
                    .map(|(e, label)| LabelWant { px: crisp(layout.strip_px(e.x)), width: mono_width(label.chars().count(), FONT_SMALL), accent: e.accent })
                    .collect();
                let placed = place_event_labels(&wants, rect.x, rect.x + rect.width);
                for ((e, label), (want, at)) in order.iter().zip(&labels).zip(wants.iter().zip(placed)) {
                    let color = if e.accent { t.accent } else { t.ink_muted };
                    line(f, Point::new(want.px, rect.y + 13.0), Point::new(want.px, rect.y + 27.0), color, 1.0, LineStyle::Solid);
                    let Some((start, row)) = at else { continue };
                    let (y, ay) = if row == 0 { (rect.y + 11.0, alignment::Vertical::Bottom) } else { (rect.y + 29.0, alignment::Vertical::Top) };
                    put(f, label, Point::new(start, y), if e.accent { t.accent } else { ink_faint }, FONT_SMALL, left, ay, MONO);
                }
            }
            StripKind::Mini => {
                put(f, "ALL", Point::new(6.0, rect.y + rect.height / 2.0), ink_faint, FONT_SMALL, left, alignment::Vertical::Center, MONO);
                f.fill_rectangle(rect.position(), rect.size(), t.bg_100);
                if let Some(s) = self.group.panels.iter().flat_map(|p| &p.series).find(|s| s.fill.is_none() && s.points.len() > 1) {
                    let (lo, hi) = s.points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p.1), hi.max(p.1)));
                    let span = (hi - lo).max(1e-12);
                    let pts = decimate(s.points.clone(), layout.full.0, layout.full.1, rect.width as usize);
                    let path = Path::new(|b| {
                        for (i, p) in pts.iter().enumerate() {
                            let at = Point::new(layout.strip_px(p.0), rect.y + rect.height - 3.0 - ((p.1 - lo) / span) as f32 * (rect.height - 6.0));
                            if i == 0 {
                                b.move_to(at);
                            } else {
                                b.line_to(at);
                            }
                        }
                    });
                    f.stroke(&path, Stroke::default().with_color(t.ink_faint).with_width(1.0));
                }
            }
        }
        if zoomed {
            // The window framed in accent, the rest dimmed.
            let (a, b) = (layout.strip_px(layout.window.0).max(rect.x), layout.strip_px(layout.window.1).min(rect.x + rect.width));
            let dim = Color { a: 0.55, ..t.bg_100 };
            if a > rect.x {
                f.fill_rectangle(rect.position(), Size::new(a - rect.x, rect.height), dim);
            }
            if b < rect.x + rect.width {
                f.fill_rectangle(Point::new(b, rect.y), Size::new(rect.x + rect.width - b, rect.height), dim);
            }
            let w = (b - a).max(3.0);
            f.stroke_rectangle(Point::new(a, rect.y + 0.75), Size::new(w, rect.height - 1.5), Stroke::default().with_color(t.accent).with_width(1.5));
        }
    }

    fn draw_panel(&self, f: &mut Frame, t: &Tokens, layout: &Layout, panel: &Panel, plot: Rectangle, (ylo, yhi): (f64, f64)) {
        let (w0, w1) = layout.window;
        let bottom = plot.y + plot.height;
        let right = plot.x + plot.width;
        let ymap = |y: f64| bottom - ((y - ylo) / (yhi - ylo)) as f32 * plot.height;
        let pt = |x: f64, y: f64| Point::new(layout.px(x), ymap(y));
        let center = Alignment::Center;

        // Grid and tick labels.
        match self.group.y_ticks {
            YTicks::Nice => {
                let y_target = ((plot.height / 30.0) as usize).clamp(2, 8);
                let ydec = decimals_for(nice_step(yhi - ylo, y_target));
                for v in nice_ticks(ylo, yhi, y_target) {
                    let y = crisp(ymap(v));
                    line(f, Point::new(plot.x, y), Point::new(right, y), t.line_100, 1.0, LineStyle::Solid);
                    put(f, &fmt_fixed(v, ydec), Point::new(plot.x - 6.0, y), t.ink_faint, FONT_TICK, Alignment::Right, alignment::Vertical::Center, MONO);
                }
            }
            YTicks::Even(n) => {
                let (ticks, ydec) = even_ticks(ylo, yhi, n);
                for v in ticks {
                    let y = crisp(ymap(v));
                    line(f, Point::new(plot.x, y), Point::new(right, y), t.line_100, 1.0, LineStyle::Solid);
                    // Left-aligned in the gutter, nudged inside the panel at the very top and bottom.
                    let at = y.clamp(plot.y + 6.0, bottom - 6.0);
                    put(f, &fmt_fixed(v, ydec), Point::new(6.0, at), t.ink_faint, FONT_TICK, Alignment::Left, alignment::Vertical::Center, MONO);
                }
            }
        }
        let x_target = ((plot.width / 80.0) as usize).clamp(2, 12);
        let xdec = decimals_for(nice_step(w1 - w0, x_target));
        for v in nice_ticks(w0, w1, x_target) {
            let x = crisp(layout.px(v));
            line(f, Point::new(x, bottom), Point::new(x, bottom + 3.0), t.line_200, 1.0, LineStyle::Solid);
            put(f, &fmt_fixed(v, xdec), Point::new(x, bottom + 4.0), t.ink_faint, FONT_TICK, center, alignment::Vertical::Top, MONO);
        }
        if ylo < 0.0 && yhi > 0.0 {
            let y = crisp(ymap(0.0));
            line(f, Point::new(plot.x, y), Point::new(right, y), t.line_200, 1.0, LineStyle::Solid);
        }

        // The range beyond the flown envelope: a subtle muted band, labelled at the top of the first panel.
        if let Some((from, label)) = &self.group.beyond {
            if *from < w1 {
                let xa = layout.px(from.max(w0)).max(plot.x);
                let band = if t.bg_200.r > 0.5 { 0.07 } else { 0.16 };
                f.fill_rectangle(Point::new(xa, plot.y), Size::new((right - xa).max(0.0), plot.height), Color { a: band, ..t.ink_muted });
                if self.group.panels.first().is_some_and(|p| std::ptr::eq(p, panel)) {
                    put(
                        f,
                        &label.to_uppercase(),
                        Point::new(xa + 6.0, bottom - 3.0),
                        t.ink_muted,
                        FONT_SMALL,
                        Alignment::Left,
                        alignment::Vertical::Bottom,
                        MONO,
                    );
                }
            }
        }

        // Stacked area fills, bottom layer first, each edged so neighbours stay separate.
        if panel.is_stacked() {
            let xs = panel.stack_xs(w0, w1);
            for (i, run) in panel.stack_layers(&xs) {
                let s = &panel.series[i];
                // On the light surface the fills are pastel (about 90 % over white), as the kit draws them.
                let pastel = if t.bg_200.r > 0.5 { 0.9 } else { 1.0 };
                let alpha = s.fill.map_or(1.0, |fill| fill.alpha) * pastel;
                if run.len() < 2 {
                    continue;
                }
                let shape = Path::new(|b| {
                    b.move_to(pt(run[0].0, run[0].2));
                    for &(x, _, hi) in &run[1..] {
                        b.line_to(pt(x, hi));
                    }
                    for &(x, lo, _) in run.iter().rev() {
                        b.line_to(pt(x, lo));
                    }
                    b.close();
                });
                f.fill(&shape, Color { a: alpha, ..s.color.resolve(t) });
                let edge = Path::new(|b| {
                    for (k, &(x, _, hi)) in run.iter().enumerate() {
                        if k == 0 {
                            b.move_to(pt(x, hi));
                        } else {
                            b.line_to(pt(x, hi));
                        }
                    }
                });
                f.stroke(&edge, Stroke::default().with_color(t.bg_200).with_width(1.0));
            }
        }

        // The beyond-flight band over stacked fills (which would hide a band drawn under them): the fills fade there.
        if let (Some((from, _)), true) = (&self.group.beyond, panel.is_stacked()) {
            if *from < w1 {
                let xa = layout.px(from.max(w0)).max(plot.x);
                f.fill_rectangle(Point::new(xa, plot.y), Size::new((right - xa).max(0.0), plot.height), Color { a: 0.28, ..t.bg_200 });
                line(f, Point::new(crisp(xa), plot.y), Point::new(crisp(xa), bottom), t.ink_faint, 1.0, LineStyle::Dotted);
            }
        }

        // Event lines, dotted, across every panel.
        for e in self.group.events.iter().filter(|e| e.x >= w0 && e.x <= w1) {
            let x = crisp(layout.px(e.x));
            line(f, Point::new(x, plot.y), Point::new(x, bottom), t.ink_faint, 1.0, LineStyle::Dotted);
            if self.group.event_tags {
                put(f, &e.label.to_uppercase(), Point::new(x + 4.0, bottom - 3.0), t.ink_faint, FONT_SMALL, Alignment::Left, alignment::Vertical::Bottom, MONO);
            }
        }

        // Reference lines with right-aligned labels.
        for l in &panel.hlines {
            let color = l.color.resolve(t);
            let y = crisp(ymap(l.y));
            line(f, Point::new(plot.x, y), Point::new(right, y), color, 1.0, l.style);
            put(f, &l.label.to_uppercase(), Point::new(right - 6.0, y - 3.0), color, FONT_SMALL, Alignment::Right, alignment::Vertical::Bottom, MONO);
        }

        // Lines.
        let columns = plot.width as usize;
        for s in panel.series.iter().filter(|s| s.fill.is_none()) {
            let pts = decimate(window_points(&s.points, w0, w1), w0, w1, columns);
            if pts.len() < 2 {
                continue;
            }
            let path = Path::new(|b| {
                for (k, p) in pts.iter().enumerate() {
                    if k == 0 {
                        b.move_to(pt(p.0, p.1));
                    } else {
                        b.line_to(pt(p.0, p.1));
                    }
                }
            });
            f.stroke(&path, stroke(s.color.resolve(t), 1.5, s.style));
        }

        // Title, top left inside the plot, on a clear inset (drawn over the curves) so no line or grid strikes through it. An
        // editable panel draws its title as a button after the lines (`draw_chrome`).
        if panel.editor.is_none() {
            let title = panel_title(panel);
            let inset = Size::new(mono_width(title.chars().count(), FONT_TICK) + 8.0 + 2.0, FONT_TICK + 6.0);
            // From the plot's edge, so no sliver of a line (a dashed reference line at the title's height) shows left of it.
            f.fill_rectangle(Point::new(plot.x, plot.y + 1.0), inset, t.bg_200);
            put(f, &title, Point::new(plot.x + 6.0, plot.y + 3.0), t.ink_faint, FONT_TICK, Alignment::Left, alignment::Vertical::Top, MONO);
        }

        // Direct labels: margin labels (de-overlapped) and inside labels.
        let mut margin: Vec<(f32, String, Color)> = Vec::new();
        let xs = panel.stack_xs(w0, w1);
        let layers = panel.stack_layers(&xs);
        for (i, s) in panel.series.iter().enumerate() {
            let Some(at) = s.label else { continue };
            let x_end = s.points.last().map_or(w1, |p| p.0.min(w1));
            let y_end = if s.fill.is_some() {
                layers.iter().find(|(k, _)| *k == i).and_then(|(_, run)| run.last()).map(|&(_, lo, hi)| (lo + hi) / 2.0)
            } else {
                s.value_at(x_end)
            };
            let Some(y_end) = y_end else { continue };
            match at {
                LabelAt::Margin => margin.push((ymap(y_end), s.label_text.clone().unwrap_or_else(|| s.name.clone()), s.color.resolve(t))),
                LabelAt::End => {
                    let text = s.label_text.as_deref().unwrap_or(&s.name);
                    let w = mono_width(text.chars().count(), FONT_SMALL);
                    let (px, py) = (layout.px(x_end), ymap(y_end));
                    let (x, ax) = if px + 6.0 + w <= right { (px + 6.0, Alignment::Left) } else { (px - 6.0, Alignment::Right) };
                    let y = if py - 12.0 < plot.y + 22.0 { (py + 12.0).min(bottom - 4.0) } else { py - 8.0 };
                    put(f, text, Point::new(x, y), t.ink_muted, FONT_SMALL, ax, alignment::Vertical::Center, MONO);
                }
                LabelAt::Inside => {
                    let x_label = (w1 - (w1 - w0) * 40.0 / f64::from(plot.width)).max(w0);
                    let y = s.value_at(x_label.min(x_end)).unwrap_or(y_end);
                    put(
                        f,
                        s.label_text.as_deref().unwrap_or(&s.name),
                        Point::new(right - 6.0, ymap(y) - 5.0),
                        t.ink_muted,
                        FONT_SMALL,
                        Alignment::Right,
                        alignment::Vertical::Bottom,
                        MONO,
                    );
                }
            }
        }
        margin.sort_by(|a, b| a.0.total_cmp(&b.0));
        for k in 1..margin.len() {
            let floor = margin[k - 1].0 + 14.0;
            if margin[k].0 < floor {
                margin[k].0 = floor;
            }
        }
        for (y, name, color) in margin {
            f.fill_rectangle(Point::new(right + 6.0, y - 4.0), Size::new(8.0, 8.0), color);
            put(f, &name, Point::new(right + 18.0, y), t.ink_muted, FONT_TICK, Alignment::Left, alignment::Vertical::Center, SANS);
        }

        // Key of the line styles, in the title row after the title, on a backing so no line strikes through it.
        if !panel.key.is_empty() {
            let tr = title_rect(panel, plot);
            let y = tr.y + tr.height / 2.0;
            let total: f32 = panel.key.iter().map(|(_, w)| 16.0 + 4.0 + mono_width(w.chars().count(), FONT_SMALL) + 12.0).sum();
            let mut kx = tr.x + tr.width + 14.0;
            if kx + total <= right {
                f.fill_rectangle(Point::new(kx - 4.0, y - 8.0), Size::new(total, 16.0), t.bg_200);
                for (style, word) in &panel.key {
                    line(f, Point::new(kx, y), Point::new(kx + 16.0, y), t.ink_muted, 1.5, *style);
                    put(f, word, Point::new(kx + 20.0, y), t.ink_muted, FONT_SMALL, Alignment::Left, alignment::Vertical::Center, MONO);
                    kx += 16.0 + 4.0 + mono_width(word.chars().count(), FONT_SMALL) + 12.0;
                }
            }
        }
    }

    fn draw_band(&self, f: &mut Frame, t: &Tokens, layout: &Layout, (a, b): (f64, f64)) {
        let Some((top, bottom)) = layout.plots_span() else { return };
        let (xa, xb) = (layout.px(a).max(layout.x0), layout.px(b).min(layout.x1));
        if xb <= xa {
            return;
        }
        f.fill_rectangle(Point::new(xa, top), Size::new(xb - xa, bottom - top), t.accent_soft);
        for x in [xa, xb] {
            line(f, Point::new(x, top), Point::new(x, bottom), t.accent, 1.5, LineStyle::Solid);
        }
        // A grab handle on each edge, per panel.
        for plot in &layout.plots {
            let cy = plot.y + plot.height / 2.0;
            for x in [xa, xb] {
                let origin = Point::new(x - 3.0, cy - 8.0);
                f.fill_rectangle(origin, Size::new(6.0, 16.0), t.bg_100);
                f.stroke_rectangle(origin, Size::new(6.0, 16.0), Stroke::default().with_color(t.accent).with_width(1.5));
            }
        }
    }

    fn draw_hover(&self, f: &mut Frame, t: &Tokens, layout: &Layout, ranges: &[(f64, f64)], hx: f64) {
        let cx = crisp(layout.px(hx));
        for ((panel, plot), &(ylo, yhi)) in self.group.panels.iter().zip(&layout.plots).zip(ranges) {
            let bottom = plot.y + plot.height;
            let ymap = |y: f64| bottom - ((y - ylo) / (yhi - ylo)) as f32 * plot.height;
            // The crosshair stops short of the panel title when it passes through it.
            let tr = title_rect(panel, *plot);
            let top = if cx >= tr.x && cx <= tr.x + tr.width { tr.y + tr.height } else { plot.y };
            line(f, Point::new(cx, top), Point::new(cx, bottom), Color { a: 0.75, ..t.ink_muted }, 1.0, LineStyle::Solid);

            // Value precision follows the plot's own scale.
            let step = nice_step(yhi - ylo, 4);
            let span = yhi - ylo;
            let floor = if span < 1.0 {
                3
            } else if span < 10.0 {
                2
            } else {
                0
            };
            let dec = (decimals_for(step) + 1).max(floor);

            // One dot per line (and for the stacked total), at the snapped value.
            let mut rows: Vec<(usize, f64)> = Vec::new();
            for (i, s) in panel.series.iter().enumerate().filter(|(_, s)| s.fill.is_none()) {
                if let Some(y) = s.value_at(hx) {
                    rows.push((i, y));
                }
            }
            let total = panel.stack_total_at(hx);
            for &(i, y) in &rows {
                let at = Point::new(cx, ymap(y));
                f.fill(&Path::circle(at, 5.0), t.bg_200);
                f.fill(&Path::circle(at, 3.5), panel.series[i].color.resolve(t));
            }
            if let Some(y) = total {
                let at = Point::new(cx, ymap(y));
                f.fill(&Path::circle(at, 5.0), t.bg_200);
                f.fill(&Path::circle(at, 3.5), t.ink);
            }

            let xs = self.group.x.format(hx);
            match panel.tag {
                Tag::Off => {}
                Tag::Only(i) => {
                    if let Some(&(_, y)) = rows.iter().find(|(k, _)| *k == i) {
                        let text = format!("{} {}{}", panel.series[i].name, fmt_fixed(y, dec), unit_suffix(panel));
                        draw_tag(f, t, *plot, title_rect(panel, *plot), Point::new(cx, ymap(y)), &text);
                    }
                }
                Tag::Auto => match (rows.len(), total) {
                    (1, None) => {
                        let y = rows[0].1;
                        let text = format!("{}{} @ {xs}", fmt_fixed(y, dec), unit_suffix(panel));
                        draw_tag(f, t, *plot, title_rect(panel, *plot), Point::new(cx, ymap(y)), &text);
                    }
                    (0, Some(y)) => {
                        let text = format!("{}{} @ {xs}", fmt_fixed(y, dec), unit_suffix(panel));
                        draw_tag(f, t, *plot, title_rect(panel, *plot), Point::new(cx, ymap(y)), &text);
                    }
                    (n, _) if n > 1 => draw_tooltip(f, t, *plot, cx, &xs, panel, &rows, dec),
                    _ => {}
                },
            }
        }
    }
}

/// ` m/s`: the unit written after a value in a hover tag. A panel's `unit` is used when it is a single word; a
/// description (`mm from tip`, `stacked by source`) needs an explicit [`Panel::value_unit`].
fn unit_suffix(panel: &Panel) -> String {
    let unit = match &panel.value_unit {
        Some(u) => u.as_str(),
        None if panel.unit.contains(' ') => "",
        None => panel.unit.as_str(),
    };
    if unit.is_empty() {
        String::new()
    } else {
        format!(" {unit}")
    }
}

/// Where a tag `w` wide goes for a value drawn at `at`: above-right by default; never over `avoid` (the panel's title), so it
/// tries below, then the other side of the point, then below that.
fn tag_position(plot: Rectangle, avoid: Rectangle, at: Point, w: f32) -> Point {
    let right = plot.x + plot.width;
    let near = at.x + 10.0;
    let far = at.x - 10.0 - w;
    let (x_first, x_second) = if near + w > right { (far, near) } else { (near, far) };
    let above = at.y - TAG_H - 8.0;
    let below = at.y + 8.0;
    let (y_first, y_second) = if above < plot.y { (below, above) } else { (above, below) };
    let place = |x: f32, y: f32| (x, y.min(plot.y + plot.height - TAG_H).max(plot.y));
    let candidates = [place(x_first, y_first), place(x_first, y_second), place(x_second, y_first), place(x_second, y_second)];
    let inside = |x: f32| x >= plot.x && x + w <= right;
    let clear = |x: f32, y: f32| inside(x) && !Rectangle::new(Point::new(x, y), Size::new(w, TAG_H)).intersects(&avoid);
    // The point itself may sit under the title: then the tag goes just below the title, beside the point.
    let under = place(x_first.clamp(plot.x, (right - w).max(plot.x)), avoid.y + avoid.height + 2.0);
    let (x, y) = candidates.iter().copied().find(|&(x, y)| clear(x, y)).unwrap_or(under);
    Point::new(x.round(), y.round())
}

fn draw_tag(f: &mut Frame, t: &Tokens, plot: Rectangle, avoid: Rectangle, at: Point, text: &str) {
    let w = mono_width(text.chars().count(), FONT_TICK) + 2.0 * TAG_PAD;
    let origin = tag_position(plot, avoid, at, w);
    f.fill_rectangle(origin, Size::new(w, TAG_H), t.bg_200);
    f.stroke_rectangle(Point::new(origin.x + 0.5, origin.y + 0.5), Size::new(w - 1.0, TAG_H - 1.0), Stroke::default().with_color(t.line_200).with_width(1.0));
    put(f, text, Point::new(origin.x + TAG_PAD, origin.y + TAG_H / 2.0), t.ink, FONT_TICK, Alignment::Left, alignment::Vertical::Center, MONO);
}

/// The compare tooltip: the x on top, then one row per line (colour square, name, value) in legend order.
fn draw_tooltip(f: &mut Frame, t: &Tokens, plot: Rectangle, cx: f32, xs: &str, panel: &Panel, rows: &[(usize, f64)], dec: usize) {
    let name_w = rows.iter().map(|(i, _)| panel.series[*i].name.chars().count()).max().unwrap_or(0);
    let values: Vec<String> = rows.iter().map(|(_, y)| format!("{}{}", fmt_fixed(*y, dec), unit_suffix(panel))).collect();
    let value_w = values.iter().map(|v| v.chars().count()).max().unwrap_or(0);
    let row_h = 16.0;
    let w = 2.0 * TAG_PAD + 14.0 + mono_width(name_w + 2 + value_w, FONT_TICK).max(mono_width(xs.chars().count(), FONT_SMALL));
    let h = 2.0 * 5.0 + row_h * (rows.len() as f32 + 1.0);
    // A tooltip taller than its plot would cover the next one: the dots alone say where the lines are.
    let top = if panel.editor.is_some() { CHIP_H + 4.0 } else { 6.0 };
    if h + top + 4.0 > plot.height {
        return;
    }
    let right = plot.x + plot.width;
    let mut x = if cx + 12.0 + w > right { cx - 12.0 - w } else { cx + 12.0 };
    let mut y = plot.y + top;
    // Never over the panel's title: try the other side of the crosshair, else go below the title.
    let avoid = title_rect(panel, plot);
    if Rectangle::new(Point::new(x, y), Size::new(w, h)).intersects(&avoid) {
        let flipped = if x > cx { cx - 12.0 - w } else { cx + 12.0 };
        if flipped >= plot.x && flipped + w <= right && !Rectangle::new(Point::new(flipped, y), Size::new(w, h)).intersects(&avoid) {
            x = flipped;
        } else {
            y = avoid.y + avoid.height + 2.0;
            if y + h + 4.0 > plot.y + plot.height {
                return;
            }
        }
    }
    let origin = Point::new(x.round(), y.round());
    f.fill_rectangle(origin, Size::new(w, h), t.bg_200);
    f.stroke_rectangle(Point::new(origin.x + 0.5, origin.y + 0.5), Size::new(w - 1.0, h - 1.0), Stroke::default().with_color(t.line_200).with_width(1.0));
    let left = Alignment::Left;
    let mid = alignment::Vertical::Center;
    put(f, xs, Point::new(origin.x + TAG_PAD, origin.y + 5.0 + row_h / 2.0), t.ink_faint, FONT_SMALL, left, mid, MONO);
    for (k, ((i, _), value)) in rows.iter().zip(&values).enumerate() {
        let y = origin.y + 5.0 + row_h * (k as f32 + 1.5);
        let s = &panel.series[*i];
        line(f, Point::new(origin.x + TAG_PAD, y), Point::new(origin.x + TAG_PAD + 10.0, y), s.color.resolve(t), 2.0, s.style);
        put(f, &s.name, Point::new(origin.x + TAG_PAD + 14.0, y), t.ink, FONT_TICK, left, mid, MONO);
        put(f, value, Point::new(origin.x + w - TAG_PAD, y), t.ink, FONT_TICK, Alignment::Right, mid, MONO);
    }
}

/// Height of a panel chip.
const CHIP_H: f32 = FONT_TICK + 7.0;
/// Width of a square icon chip.
const ICON_W: f32 = 20.0;
/// Width of the chevron after an editable panel's title.
const CHEVRON_W: f32 = 14.0;

/// Where `panel`'s title sits in `plot` (the inset behind the text, or an editable panel's title chip): the rectangle hover tags
/// keep clear of.
fn title_rect(panel: &Panel, plot: Rectangle) -> Rectangle {
    let text_w = mono_width(panel_title(panel).chars().count(), FONT_TICK);
    if panel.editor.is_some() {
        Rectangle::new(Point::new(plot.x + 2.0, plot.y), Size::new(text_w + 8.0 + CHEVRON_W, CHIP_H))
    } else {
        Rectangle::new(Point::new(plot.x, plot.y + 1.0), Size::new(text_w + 10.0, FONT_TICK + 6.0))
    }
}

/// `ALTITUDE · m`: a panel's title with its unit.
fn panel_title(panel: &Panel) -> String {
    if panel.unit.is_empty() {
        panel.title.to_uppercase()
    } else {
        format!("{} · {}", panel.title.to_uppercase(), panel.unit)
    }
}

/// One clickable chip of an editable panel.
#[derive(Clone, Copy)]
struct Chip {
    action: PanelAction,
    rect: Rectangle,
    enabled: bool,
}

/// The chips of an editable panel drawn in `plot`: the title (left) and `+ series`, up, down, remove (right, in that order).
fn chips(plot: Rectangle, panel: &Panel) -> Vec<Chip> {
    let Some(editor) = &panel.editor else { return Vec::new() };
    let chip = |action, x: f32, w: f32, enabled| Chip { action, rect: Rectangle::new(Point::new(x, plot.y + 1.0), Size::new(w, CHIP_H)), enabled };
    let title_w = mono_width(panel_title(panel).chars().count(), FONT_TICK) + 8.0 + CHEVRON_W;
    let add_w = mono_width("+ series".len(), FONT_TICK) + 14.0;
    let right = plot.x + plot.width - 4.0;
    let remove = right - ICON_W;
    let down = remove - 2.0 - ICON_W;
    let up = down - 2.0 - ICON_W;
    let add = up - 6.0 - add_w;
    vec![
        chip(PanelAction::Pick, plot.x + 2.0, title_w, true),
        chip(PanelAction::AddSeries, add, add_w, editor.can_add),
        chip(PanelAction::MoveUp, up, ICON_W, editor.can_up),
        chip(PanelAction::MoveDown, down, ICON_W, editor.can_down),
        chip(PanelAction::Remove, remove, ICON_W, editor.can_remove),
    ]
}

impl PlotView<'_> {
    /// The editable panel chip under `p`, with its panel's index.
    fn chip_at(&self, layout: &Layout, p: Point) -> Option<(usize, Chip)> {
        self.group
            .panels
            .iter()
            .zip(&layout.plots)
            .enumerate()
            .find_map(|(i, (panel, plot))| chips(*plot, panel).into_iter().find(|c| c.rect.contains(p)).map(|c| (i, c)))
    }

    /// An editable panel's title button, legend and controls, over the lines.
    fn draw_chrome(&self, f: &mut Frame, t: &Tokens, panel: &Panel, plot: Rectangle, hot: Option<Point>) {
        let Some(editor) = &panel.editor else { return };
        let all = chips(plot, panel);
        let mid = alignment::Vertical::Center;
        let mut limit = plot.x + plot.width;
        // The title is always a button; `+ series`, up, down and remove show only while the pointer is over this panel (or a
        // picker is open).
        let controls = self.pinned || hot.is_some_and(|p| plot.contains(p));
        for chip in &all {
            if chip.action == PanelAction::AddSeries {
                limit = chip.rect.x;
            }
            if chip.action != PanelAction::Pick && !controls {
                continue;
            }
            let over = chip.enabled && hot.is_some_and(|p| chip.rect.contains(p));
            let r = chip.rect;
            let color = match (chip.enabled, over) {
                (false, _) => Color { a: 0.45, ..t.ink_faint },
                (true, true) => t.ink,
                (true, false) => t.ink_muted,
            };
            f.fill_rectangle(r.position(), r.size(), if over { t.bg_300 } else { t.bg_200 });
            let framed = chip.action != PanelAction::Pick || over;
            if framed {
                let edge = if over { t.line_200 } else { t.line_100 };
                f.stroke_rectangle(
                    Point::new(r.x + 0.5, r.y + 0.5),
                    Size::new(r.width - 1.0, r.height - 1.0),
                    Stroke::default().with_color(edge).with_width(1.0),
                );
            }
            let (cx, cy) = (r.x + r.width / 2.0, r.y + r.height / 2.0);
            let stroke = Stroke::default().with_color(color).with_width(1.5);
            let chevron = |f: &mut Frame, x: f32, y: f32, down: bool| {
                let d = if down { 2.0 } else { -2.0 };
                f.stroke(
                    &Path::new(|b| {
                        b.move_to(Point::new(x - 3.5, y - d));
                        b.line_to(Point::new(x, y + d));
                        b.line_to(Point::new(x + 3.5, y - d));
                    }),
                    stroke,
                );
            };
            match chip.action {
                PanelAction::Pick => {
                    put(f, &panel_title(panel), Point::new(r.x + 4.0, cy), color, FONT_TICK, Alignment::Left, mid, MONO);
                    chevron(f, r.x + r.width - CHEVRON_W / 2.0 - 1.0, cy, true);
                    if over {
                        // Affordance: an underline under the title and a tooltip below it.
                        let text_w = mono_width(panel_title(panel).chars().count(), FONT_TICK);
                        line(
                            f,
                            Point::new(r.x + 4.0, r.y + r.height - 2.5),
                            Point::new(r.x + 4.0 + text_w, r.y + r.height - 2.5),
                            t.ink,
                            1.0,
                            LineStyle::Solid,
                        );
                        let tip = "Choose what this plot shows";
                        let w = mono_width(tip.chars().count(), FONT_SMALL) + 2.0 * TAG_PAD;
                        let origin = Point::new(r.x.round(), (r.y + r.height + 4.0).round());
                        f.fill_rectangle(origin, Size::new(w, 20.0), t.bg_200);
                        f.stroke_rectangle(
                            Point::new(origin.x + 0.5, origin.y + 0.5),
                            Size::new(w - 1.0, 19.0),
                            Stroke::default().with_color(t.line_200).with_width(1.0),
                        );
                        put(f, tip, Point::new(origin.x + TAG_PAD, origin.y + 10.0), t.ink, FONT_SMALL, Alignment::Left, mid, MONO);
                    }
                }
                PanelAction::AddSeries => put(f, "+ series", Point::new(cx, cy), color, FONT_TICK, Alignment::Center, mid, MONO),
                PanelAction::MoveUp => chevron(f, cx, cy, false),
                PanelAction::MoveDown => chevron(f, cx, cy, true),
                PanelAction::Remove => {
                    f.stroke(&Path::line(Point::new(cx - 3.5, cy - 3.5), Point::new(cx + 3.5, cy + 3.5)), stroke);
                    f.stroke(&Path::line(Point::new(cx - 3.5, cy + 3.5), Point::new(cx + 3.5, cy - 3.5)), stroke);
                }
            }
        }

        // The legend: the line style of each source and its name, after the title; what does not fit wraps under the title (as many
        // rows as the plot has room for), so every source is named.
        if editor.legend.len() > 1 {
            let first = all[0].rect.x + all[0].rect.width + 12.0;
            let start = plot.x + 8.0;
            let rows = if plot.height >= 4.0 * CHIP_H { 2 } else { 1 };
            let (mut x, mut row) = (first, 0);
            for item in &editor.legend {
                let w = 20.0 + mono_width(item.text.chars().count(), FONT_SMALL);
                let right = if row == 0 { limit - 8.0 } else { plot.x + plot.width - 8.0 };
                if x + w > right {
                    if row + 1 >= rows {
                        break;
                    }
                    row += 1;
                    x = start;
                }
                let top = plot.y + 1.0 + row as f32 * (CHIP_H + 1.0);
                let y = top + CHIP_H / 2.0;
                f.fill_rectangle(Point::new(x - 3.0, top), Size::new(w + 6.0, CHIP_H), t.bg_200);
                line(f, Point::new(x, y), Point::new(x + 16.0, y), item.color.resolve(t), 2.0, item.style);
                put(f, &item.text, Point::new(x + 20.0, y), t.ink_muted, FONT_SMALL, Alignment::Left, mid, MONO);
                x += w + 12.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn apogee_label_is_always_placed_and_labels_never_overlap() {
        // Rail exit, burnout and apogee within a few pixels of each other, the apogee (accent) in the middle.
        let wants = [
            LabelWant { px: 10.0, width: 80.0, accent: false },
            LabelWant { px: 18.0, width: 70.0, accent: false },
            LabelWant { px: 26.0, width: 70.0, accent: true },
            LabelWant { px: 400.0, width: 90.0, accent: false },
        ];
        let placed = place_event_labels(&wants, 0.0, 500.0);
        assert!(placed[2].is_some(), "the apogee label must always be drawn");
        let rects: Vec<(usize, f32, f32)> = placed.iter().zip(&wants).filter_map(|(p, w)| p.map(|(s, r)| (r, s, s + w.width))).collect();
        for (i, a) in rects.iter().enumerate() {
            assert!(a.1 >= 0.0 && a.2 <= 500.0, "{a:?} leaves the strip");
            for b in &rects[i + 1..] {
                assert!(a.0 != b.0 || a.2 <= b.1 || b.2 <= a.1, "{a:?} overlaps {b:?}");
            }
        }
    }

    use super::*;
    use crate::widgets::plot::{Axis, EventMark, Series};

    fn group() -> PlotGroup {
        let pts: Vec<_> = (0..=100).map(|i| (i as f64, i as f64)).collect();
        let mut g = PlotGroup::new(Axis::time())
            .panel(Panel::new("A", "m").with(Series::new("a", pts.clone())))
            .panel(Panel::new("B", "m").with(Series::new("b", pts)));
        g.events.push(EventMark::new(50.0, "Apogee"));
        g
    }

    #[test]
    fn hover_tags_keep_clear_of_the_panel_title() {
        let plot = Rectangle::new(Point::new(54.0, 100.0), Size::new(600.0, 160.0));
        let panel = Panel::new("Static margin", "cal");
        let avoid = title_rect(&panel, plot);
        let w = 120.0;
        // A point high in the panel near the left edge: above-right would sit on the title.
        for at in [Point::new(70.0, 125.0), Point::new(100.0, 118.0), Point::new(200.0, 110.0), Point::new(60.0, 101.0)] {
            let o = tag_position(plot, avoid, at, w);
            let tag = Rectangle::new(o, Size::new(w, TAG_H));
            assert!(!tag.intersects(&avoid), "tag {tag:?} over title {avoid:?} for {at:?}");
            assert!(o.y >= plot.y && o.y + TAG_H <= plot.y + plot.height && o.x >= plot.x && o.x + w <= plot.x + plot.width);
        }
        // Away from the title the tag is above-right of the point, as before.
        let o = tag_position(plot, avoid, Point::new(300.0, 200.0), w);
        assert_eq!((o.x, o.y), (310.0, 172.0));
    }

    #[test]
    fn x_mapping_round_trips_and_follows_the_window() {
        let g = group();
        let mut s = PlotState::new((0.0, 100.0));
        let size = Size::new(800.0, 400.0);
        let l = Layout::new(size, &g, &s);
        assert!((l.px(0.0) - GUTTER).abs() < 1e-4);
        assert!((l.px(100.0) - (800.0 - PAD_R)).abs() < 1e-3);
        assert!((l.x_at(l.px(37.5)) - 37.5).abs() < 1e-4);
        s.update(PlotMessage::Band(Some((20.0, 30.0))));
        s.update(PlotMessage::ZoomIn);
        let l = Layout::new(size, &g, &s);
        assert!((l.px(20.0) - GUTTER).abs() < 1e-4);
        assert!((l.px(30.0) - (800.0 - PAD_R)).abs() < 1e-3);
        // The strip keeps mapping the full range.
        assert!((l.strip_px(0.0) - GUTTER).abs() < 1e-4);
        assert!((l.strip_x_at(l.strip_px(70.0)) - 70.0).abs() < 1e-4);
    }

    #[test]
    fn panels_split_the_height_by_weight_and_stay_inside() {
        let mut g = group();
        g.panels[1].weight = 3.0;
        let l = Layout::new(Size::new(800.0, 400.0), &g, &PlotState::new((0.0, 100.0)));
        assert_eq!(l.plots.len(), 2);
        assert!(l.plots[1].height > 2.0 * l.plots[0].height);
        let last = l.plots[1];
        assert!(last.y + last.height + TICK_H <= 400.0);
        assert!(l.plots[0].y + l.plots[0].height < l.plots[1].y);
    }

    #[test]
    fn the_strip_shows_for_timelines_and_when_zoomed() {
        let mut g = group();
        let mut s = PlotState::new((0.0, 100.0));
        assert!(Layout::new(Size::new(800.0, 400.0), &g, &s).strip.is_none());
        s.update(PlotMessage::Window((10.0, 20.0)));
        let l = Layout::new(Size::new(800.0, 400.0), &g, &s);
        assert!(matches!(l.strip, Some((_, StripKind::Mini))));
        assert!(l.plots[0].y > STRIP_MINI_H);
        g.timeline = true;
        let l = Layout::new(Size::new(800.0, 400.0), &g, &PlotState::new((0.0, 100.0)));
        assert!(matches!(l.strip, Some((_, StripKind::Timeline))));
    }

    #[test]
    fn shift_snaps_to_events_within_six_pixels() {
        let g = group();
        let s = PlotState::new((0.0, 100.0));
        let v = PlotView::new(&g, &s);
        let l = Layout::new(Size::new(800.0, 400.0), &g, &s);
        let near = l.x_at(l.px(50.0) + 5.0);
        let far = l.x_at(l.px(50.0) + 9.0);
        assert_eq!(v.snap(&l, true, near), 50.0);
        assert_eq!(v.snap(&l, true, far), far);
        assert_eq!(v.snap(&l, false, near), near);
    }

    #[test]
    fn decimation_keeps_peaks_and_passes_sparse_series() {
        let sparse: Vec<_> = (0..10).map(|i| (i as f64, 0.0)).collect();
        assert_eq!(decimate(sparse.clone(), 0.0, 9.0, 100), sparse);
        // 10 000 samples with one spike into 50 columns: far fewer points, the spike survives.
        let mut dense: Vec<_> = (0..10_000).map(|i| (i as f64, 0.0)).collect();
        dense[5_003].1 = 99.0;
        let d = decimate(dense, 0.0, 9_999.0, 50);
        assert!(d.len() <= 50 * 4);
        assert!(d.iter().any(|p| p.1 == 99.0));
        assert!(d.windows(2).all(|w| w[0].0 <= w[1].0));
    }

    #[test]
    fn unit_suffix_skips_descriptions() {
        assert_eq!(unit_suffix(&Panel::new("A", "m/s")), " m/s");
        assert_eq!(unit_suffix(&Panel::new("A", "")), "");
        assert_eq!(unit_suffix(&Panel::new("A", "mm from tip")), "");
        assert_eq!(unit_suffix(&Panel::new("A", "mm from tip").value_unit("mm")), " mm");
    }
}
