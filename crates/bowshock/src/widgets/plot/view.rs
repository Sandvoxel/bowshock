//! The canvas that draws and drives a [`PlotGroup`]: panels, events strip, selection band, synced crosshair.

use std::time::{Duration, Instant};

use iced::advanced::text::Alignment;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, LineJoin, Path, Stroke, Text};
use iced::{alignment, keyboard, mouse, Color, Element, Fill, Font, Pixels, Point, Rectangle, Renderer, Size, Theme, Widget};

use super::ticks::{decimals_for, fmt_fixed, nice_step, nice_ticks};
use super::{window_points, LabelAt, LineStyle, Panel, PlotGroup, PlotMessage, PlotState, Tag};
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
}

impl<'a> PlotView<'a> {
    /// A view of `group` in the zoom/hover/band state `state`.
    pub fn new(group: &'a PlotGroup, state: &'a PlotState) -> Self {
        PlotView { group, state }
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
        let strip = if group.timeline {
            Some((Rectangle::new(Point::new(x0, 0.0), Size::new(x1 - x0, STRIP_TIMELINE_H)), StripKind::Timeline))
        } else if state.is_zoomed() {
            Some((Rectangle::new(Point::new(x0, 0.0), Size::new(x1 - x0, STRIP_MINI_H)), StripKind::Mini))
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
    drag: Option<Drag>,
    last_click: Option<(Instant, Point)>,
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
                None
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key: keyboard::Key::Named(named), modifiers, .. }) => {
                st.modifiers = *modifiers;
                match named {
                    keyboard::key::Named::Space if over => {
                        st.space = true;
                        None
                    }
                    keyboard::key::Named::Enter if over && self.state.can_zoom_in() => Some(Action::publish(PlotMessage::ZoomIn).and_capture()),
                    keyboard::key::Named::Escape if over && self.state.can_back() => {
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
            Event::Mouse(mouse::Event::CursorMoved { .. }) => self.moved(st, &layout, local),
            Event::Mouse(mouse::Event::CursorLeft) => {
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

    fn draw(&self, st: &Interaction, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _cursor: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let mut frame = Frame::new(renderer, bounds.size());
        let layout = Layout::new(bounds.size(), self.group, self.state);
        let ranges: Vec<(f64, f64)> = self.group.panels.iter().map(|p| p.y_range(layout.window.0, layout.window.1)).collect();

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
    }
}

fn line(frame: &mut Frame, a: Point, b: Point, color: Color, width: f32, style: LineStyle) {
    frame.stroke(&Path::line(a, b), stroke(color, width, style));
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
                // Event ticks and labels alternate above and below the bar, skipping a label that would collide.
                let mut row_end = [f32::NEG_INFINITY; 2];
                let mut order: Vec<_> = self.group.events.iter().collect();
                order.sort_by(|a, b| a.x.total_cmp(&b.x));
                for (n, e) in order.into_iter().enumerate() {
                    let px = crisp(layout.strip_px(e.x));
                    let color = if e.accent { t.accent } else { t.ink_muted };
                    line(f, Point::new(px, rect.y + 13.0), Point::new(px, rect.y + 27.0), color, 1.0, LineStyle::Solid);
                    let label = format!("{} {}", e.label.to_uppercase(), fmt_fixed(e.x, 1));
                    let w = mono_width(label.chars().count(), FONT_SMALL);
                    let start = (px + 3.0).min(rect.x + rect.width - w);
                    let row = [n % 2, (n + 1) % 2].into_iter().find(|r| start > row_end[*r] + 6.0);
                    let Some(row) = row else { continue };
                    row_end[row] = start + w;
                    let (y, ay) = if row == 0 { (rect.y + 11.0, alignment::Vertical::Bottom) } else { (rect.y + 29.0, alignment::Vertical::Top) };
                    put(f, &label, Point::new(start, y), if e.accent { t.accent } else { ink_faint }, FONT_SMALL, left, ay, MONO);
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
                    f.stroke(&path, Stroke::default().with_color(t.line_200).with_width(1.0));
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
        let y_target = ((plot.height / 30.0) as usize).clamp(2, 8);
        let ydec = decimals_for(nice_step(yhi - ylo, y_target));
        for v in nice_ticks(ylo, yhi, y_target) {
            let y = crisp(ymap(v));
            line(f, Point::new(plot.x, y), Point::new(right, y), t.line_100, 1.0, LineStyle::Solid);
            put(f, &fmt_fixed(v, ydec), Point::new(plot.x - 6.0, y), t.ink_faint, FONT_TICK, Alignment::Right, alignment::Vertical::Center, MONO);
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

        // Stacked area fills, bottom layer first, each edged so neighbours stay separate.
        if panel.is_stacked() {
            let xs = panel.stack_xs(w0, w1);
            for (i, run) in panel.stack_layers(&xs) {
                let s = &panel.series[i];
                let alpha = s.fill.map_or(1.0, |fill| fill.alpha);
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

        // Title, top left inside the plot, on a clear inset so grid and event lines do not strike through it.
        let title = if panel.unit.is_empty() { panel.title.to_uppercase() } else { format!("{} · {}", panel.title.to_uppercase(), panel.unit) };
        let inset = Size::new(mono_width(title.chars().count(), FONT_TICK) + 8.0, FONT_TICK + 6.0);
        f.fill_rectangle(Point::new(plot.x + 2.0, plot.y + 1.0), inset, t.bg_200);
        put(f, &title, Point::new(plot.x + 6.0, plot.y + 3.0), t.ink_faint, FONT_TICK, Alignment::Left, alignment::Vertical::Top, MONO);

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
            f.stroke(&path, stroke(s.color.resolve(t), 2.0, s.style));
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
            line(f, Point::new(cx, plot.y), Point::new(cx, bottom), Color { a: 0.75, ..t.ink_muted }, 1.0, LineStyle::Solid);

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
                        draw_tag(f, t, *plot, Point::new(cx, ymap(y)), &text);
                    }
                }
                Tag::Auto => match (rows.len(), total) {
                    (1, None) => {
                        let y = rows[0].1;
                        let text = format!("{}{} @ {xs}", fmt_fixed(y, dec), unit_suffix(panel));
                        draw_tag(f, t, *plot, Point::new(cx, ymap(y)), &text);
                    }
                    (0, Some(y)) => {
                        let text = format!("{}{} @ {xs}", fmt_fixed(y, dec), unit_suffix(panel));
                        draw_tag(f, t, *plot, Point::new(cx, ymap(y)), &text);
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

/// A framed value tag above-right of `at` (below when there is no room above, left when it would leave the plot).
fn draw_tag(f: &mut Frame, t: &Tokens, plot: Rectangle, at: Point, text: &str) {
    let w = mono_width(text.chars().count(), FONT_TICK) + 2.0 * TAG_PAD;
    let right = plot.x + plot.width;
    let x = if at.x + 10.0 + w > right { at.x - 10.0 - w } else { at.x + 10.0 };
    let mut y = at.y - TAG_H - 8.0;
    if y < plot.y {
        y = at.y + 8.0;
    }
    let y = y.min(plot.y + plot.height - TAG_H).max(plot.y);
    let origin = Point::new(x.round(), y.round());
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
    let right = plot.x + plot.width;
    let x = if cx + 12.0 + w > right { cx - 12.0 - w } else { cx + 12.0 };
    let origin = Point::new(x.round(), (plot.y + 6.0).round());
    f.fill_rectangle(origin, Size::new(w, h), t.bg_200);
    f.stroke_rectangle(Point::new(origin.x + 0.5, origin.y + 0.5), Size::new(w - 1.0, h - 1.0), Stroke::default().with_color(t.line_200).with_width(1.0));
    let left = Alignment::Left;
    let mid = alignment::Vertical::Center;
    put(f, xs, Point::new(origin.x + TAG_PAD, origin.y + 5.0 + row_h / 2.0), t.ink_faint, FONT_SMALL, left, mid, MONO);
    for (k, ((i, _), value)) in rows.iter().zip(&values).enumerate() {
        let y = origin.y + 5.0 + row_h * (k as f32 + 1.5);
        let s = &panel.series[*i];
        f.fill_rectangle(Point::new(origin.x + TAG_PAD, y - 4.0), Size::new(8.0, 8.0), s.color.resolve(t));
        put(f, &s.name, Point::new(origin.x + TAG_PAD + 14.0, y), t.ink, FONT_TICK, left, mid, MONO);
        put(f, value, Point::new(origin.x + w - TAG_PAD, y), t.ink, FONT_TICK, Alignment::Right, mid, MONO);
    }
}

#[cfg(test)]
mod tests {
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
