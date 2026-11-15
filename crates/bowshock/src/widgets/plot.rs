//! Linked time/Mach plots: the data model, zoom state and chrome for design/components/PlotZoom.
//!
//! A [`PlotGroup`] is a stack of [`Panel`]s that share one x axis. The app keeps one [`PlotState`] per group (the
//! PlotZoom breadcrumb stack, the selection band and the hover x), draws it with [`PlotView`] and routes the
//! [`PlotMessage`]s it publishes into [`PlotState::update`]:
//!
//! ```ignore
//! // view
//! column![plot::header(&self.group, &self.plot), PlotView::new(&self.group, &self.plot).element().map(Msg::Plot)]
//! // update
//! Msg::Plot(m) => self.plot.update(m),
//! ```
//!
//! Interaction (PlotZoom): drag along x to select a full-height band, `Enter` zooms to it (y re-fits to the data in view),
//! `Esc` steps back (band first, then one level), `Shift+Esc` or a double-click returns to the full range. The wheel zooms
//! around the cursor, middle-drag or `Space`+drag pans, and the overview strip pans or recentres the window.

#![allow(dead_code)] // The screens that compose these widgets arrive after the widgets do.

use iced::widget::{button, container, row, text, Space};
use iced::{Border, Color, Element, Length, Rectangle, Theme, Widget};

use crate::theme::{self, MONO};
use crate::tokens::{self, Tokens};

pub mod demo;
mod ticks;
mod view;

pub use view::PlotView;

/// Smallest window or band, as a fraction of the full range. Narrower selections are ignored.
const MIN_SPAN_FRAC: f64 = 1e-6;
/// Headroom added above (and below) the data when y re-fits to the window, before the range is pushed out to a nice tick.
const HEADROOM: f64 = 0.04;

// ---------------------------------------------------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------------------------------------------------

/// A theme-independent colour role. Resolved against the active [`Tokens`] when drawn, so data never holds colours.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeriesColor {
    /// `ink`: the single series of a plot.
    Ink,
    /// `ink-muted`.
    Muted,
    /// `accent`: the thing being acted on.
    Accent,
    /// Categorical chart slot 1–4 (`series-1…4`), in fixed order; anything else clamps into range.
    Series(u8),
    /// Centre of gravity.
    Cg,
    /// Centre of pressure.
    Cp,
    /// `ok`: passing / stable.
    Ok,
    /// `warn`: marginal.
    Warn,
    /// `err`.
    Err,
    /// System colours, for flight phases.
    Propulsion,
    /// Recovery system colour.
    Recovery,
    /// Avionics system colour.
    Avionics,
    /// Aero system colour.
    Aero,
}

impl SeriesColor {
    /// The colour of this role in `t`.
    pub fn resolve(self, t: &Tokens) -> Color {
        match self {
            SeriesColor::Ink => t.ink,
            SeriesColor::Muted => t.ink_muted,
            SeriesColor::Accent => t.accent,
            SeriesColor::Series(0 | 1) => t.series_1,
            SeriesColor::Series(2) => t.series_2,
            SeriesColor::Series(3) => t.series_3,
            SeriesColor::Series(_) => t.series_4,
            SeriesColor::Cg => t.cg,
            SeriesColor::Cp => t.cp,
            SeriesColor::Ok => t.ok,
            SeriesColor::Warn => t.warn,
            SeriesColor::Err => t.err,
            SeriesColor::Propulsion => t.sys_propulsion,
            SeriesColor::Recovery => t.sys_recovery,
            SeriesColor::Avionics => t.sys_avionics,
            SeriesColor::Aero => t.sys_aero,
        }
    }
}

/// How a line is stroked.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LineStyle {
    /// A continuous line.
    #[default]
    Solid,
    /// Long dashes: thresholds, the second CG.
    Dashed,
    /// Dots: event lines.
    Dotted,
    /// Alternating dashes and dots: a fourth line in one plot.
    DashDot,
}

/// Marks a series as one layer of a stacked area. Fill series of a panel stack in order, bottom first, sharing the x
/// samples of the first one; their `points` are the layer's own value, not the running total.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StackedFill {
    /// Fill opacity, 0–1.
    pub alpha: f32,
}

impl Default for StackedFill {
    fn default() -> Self {
        StackedFill { alpha: 1.0 }
    }
}

/// Where a series' direct label goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LabelAt {
    /// In the right margin ([`PlotGroup::right_margin`]) at the series' value at the right edge, with a colour square.
    Margin,
    /// Inside the plot, right-aligned above the line.
    Inside,
}

/// One line (or one stacked layer) of a panel. `points` are `(x, y)` in SI units, sorted by ascending x.
#[derive(Debug, Clone)]
pub struct Series {
    /// Name used in labels and in the hover tooltip.
    pub name: String,
    /// `(x, y)` samples, ascending in x.
    pub points: Vec<(f64, f64)>,
    /// Colour role.
    pub color: SeriesColor,
    /// Stroke style.
    pub style: LineStyle,
    /// Stacked area fill; see [`StackedFill`].
    pub fill: Option<StackedFill>,
    /// Direct label, if any.
    pub label: Option<LabelAt>,
    /// Text of the direct label when it should differ from `name` (`Total` for a series that reads `Cd` in the tag).
    pub label_text: Option<String>,
}

impl Series {
    /// A solid `ink` line.
    pub fn new(name: impl Into<String>, points: Vec<(f64, f64)>) -> Self {
        Series { name: name.into(), points, color: SeriesColor::Ink, style: LineStyle::Solid, fill: None, label: None, label_text: None }
    }

    /// Sets the colour role.
    pub fn color(mut self, color: SeriesColor) -> Self {
        self.color = color;
        self
    }

    /// Sets the stroke style.
    pub fn style(mut self, style: LineStyle) -> Self {
        self.style = style;
        self
    }

    /// Makes this series a layer of the panel's stacked area.
    pub fn filled(mut self) -> Self {
        self.fill = Some(StackedFill::default());
        self
    }

    /// Labels the series in the right margin.
    pub fn label_margin(mut self) -> Self {
        self.label = Some(LabelAt::Margin);
        self
    }

    /// Labels the series inside the plot.
    pub fn label_inside(mut self) -> Self {
        self.label = Some(LabelAt::Inside);
        self
    }

    /// Labels the series with `text` (instead of its name) at `at`.
    pub fn label_as(mut self, text: impl Into<String>, at: LabelAt) -> Self {
        self.label = Some(at);
        self.label_text = Some(text.into());
        self
    }

    /// The linearly interpolated value at `x`; `None` outside the data (so a flight that has landed has no dot).
    pub fn value_at(&self, x: f64) -> Option<f64> {
        let p = &self.points;
        let (first, last) = (p.first()?, p.last()?);
        if !x.is_finite() || x < first.0 || x > last.0 {
            return None;
        }
        let i = p.partition_point(|q| q.0 <= x);
        if i >= p.len() {
            return Some(last.1);
        }
        let (a, b) = (p[i - 1], p[i]);
        if b.0 == a.0 {
            Some(b.1)
        } else {
            Some(a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0))
        }
    }
}

/// A horizontal reference line with a right-aligned label (`MIN 1.0`, `OVER-STABLE 3.0`).
#[derive(Debug, Clone)]
pub struct HLine {
    /// The y value.
    pub y: f64,
    /// Label, drawn upper-case.
    pub label: String,
    /// Colour role (`ok`, `warn`).
    pub color: SeriesColor,
    /// Stroke style.
    pub style: LineStyle,
}

impl HLine {
    /// A dashed reference line.
    pub fn dashed(y: f64, label: impl Into<String>, color: SeriesColor) -> Self {
        HLine { y, label: label.into(), color, style: LineStyle::Dashed }
    }
}

/// What the hover tag of a panel shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Tag {
    /// One framed `value unit @ x` tag for a single line; a tooltip listing every line's value for several.
    #[default]
    Auto,
    /// One framed tag `name value unit` for the line at this index; the others only get their dot.
    Only(usize),
    /// Dots only.
    Off,
}

/// What a click on a panel's chrome asks for (see [`PanelEditor`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PanelAction {
    /// The title was clicked: pick the panel's primary data source.
    Pick,
    /// `+ series`: overlay another source.
    AddSeries,
    /// Move the panel up one place.
    MoveUp,
    /// Move the panel down one place.
    MoveDown,
    /// Remove the panel.
    Remove,
}

/// One legend entry of an editable panel: a short stroke in the line's style and the source's name.
#[derive(Debug, Clone)]
pub struct LegendItem {
    /// Stroke style of the line.
    pub style: LineStyle,
    /// Colour role of the stroke.
    pub color: SeriesColor,
    /// Source name.
    pub text: String,
}

/// Makes a panel editable: its title becomes a picker button (a chevron after it), a row of small controls (`+ series`, move
/// up, move down, remove) sits at the top right inside the plot, and a legend of its sources follows the title. Clicks arrive
/// as [`PlotMessage::Panel`] with the clicked chip's rectangle in the canvas, so the app can open a menu there.
#[derive(Debug, Clone, Default)]
pub struct PanelEditor {
    /// `+ series` is available.
    pub can_add: bool,
    /// The panel can move up.
    pub can_up: bool,
    /// The panel can move down.
    pub can_down: bool,
    /// The panel can be removed.
    pub can_remove: bool,
    /// Names of the panel's sources with their line styles (drawn only when there are two or more).
    pub legend: Vec<LegendItem>,
}

/// One plot of a [`PlotGroup`]: a title, its series and its reference lines. y is never shared between panels.
#[derive(Debug, Clone)]
pub struct Panel {
    /// Title, drawn upper-case at the top left inside the plot (`ALTITUDE · m`).
    pub title: String,
    /// Unit text after the title.
    pub unit: String,
    /// Lines and stacked layers.
    pub series: Vec<Series>,
    /// Horizontal reference lines (included in the y fit).
    pub hlines: Vec<HLine>,
    /// Share of the group's height relative to the other panels.
    pub weight: f32,
    /// Hover tag behaviour.
    pub tag: Tag,
    /// Anchor y = 0 at the bottom: no headroom below zero-based data. Implied by stacked fills.
    pub zero_base: bool,
    /// Unit written after values in hover tags, when `unit` is a description (`mm from tip` → `mm`). Defaults to `unit`
    /// when that is a single word.
    pub value_unit: Option<String>,
    /// Editing chrome (picker title, controls, legend), for panels the user composes.
    pub editor: Option<PanelEditor>,
}

impl Panel {
    /// An empty panel titled `title` with y in `unit`.
    pub fn new(title: impl Into<String>, unit: impl Into<String>) -> Self {
        Panel {
            title: title.into(),
            unit: unit.into(),
            series: Vec::new(),
            hlines: Vec::new(),
            weight: 1.0,
            tag: Tag::Auto,
            zero_base: false,
            value_unit: None,
            editor: None,
        }
    }

    /// Adds a series.
    pub fn with(mut self, series: Series) -> Self {
        self.series.push(series);
        self
    }

    /// Adds a reference line.
    pub fn hline(mut self, line: HLine) -> Self {
        self.hlines.push(line);
        self
    }

    /// Sets the height weight.
    pub fn weight(mut self, weight: f32) -> Self {
        self.weight = weight;
        self
    }

    /// Sets the hover tag behaviour.
    pub fn tag(mut self, tag: Tag) -> Self {
        self.tag = tag;
        self
    }

    /// Anchors y = 0 at the bottom of the plot.
    pub fn zero_based(mut self) -> Self {
        self.zero_base = true;
        self
    }

    /// Sets the unit written after hover-tag values.
    pub fn value_unit(mut self, unit: impl Into<String>) -> Self {
        self.value_unit = Some(unit.into());
        self
    }

    /// Makes the panel editable (see [`PanelEditor`]).
    pub fn editable(mut self, editor: PanelEditor) -> Self {
        self.editor = Some(editor);
        self
    }

    /// Whether any series is a stacked layer.
    pub fn is_stacked(&self) -> bool {
        self.series.iter().any(|s| s.fill.is_some())
    }

    /// The x samples the stacked layers share: those of the first layer within the window, plus the window's ends.
    pub(crate) fn stack_xs(&self, w0: f64, w1: f64) -> Vec<f64> {
        let Some(first) = self.series.iter().find(|s| s.fill.is_some()) else { return Vec::new() };
        window_points(&first.points, w0, w1).into_iter().map(|p| p.0).collect()
    }

    /// Each stacked layer at `xs`: `(series index, [(x, lower, upper)])`, bottom layer first. A layer with no data at `x`
    /// counts as zero.
    pub(crate) fn stack_layers(&self, xs: &[f64]) -> Vec<(usize, Vec<(f64, f64, f64)>)> {
        let mut base = vec![0.0; xs.len()];
        let mut layers = Vec::new();
        for (i, s) in self.series.iter().enumerate().filter(|(_, s)| s.fill.is_some()) {
            let mut run = Vec::with_capacity(xs.len());
            for (k, &x) in xs.iter().enumerate() {
                let top = base[k] + s.value_at(x).unwrap_or(0.0);
                run.push((x, base[k], top));
                base[k] = top;
            }
            layers.push((i, run));
        }
        layers
    }

    /// The running total of the stacked layers at `x`.
    pub fn stack_total_at(&self, x: f64) -> Option<f64> {
        let mut any = false;
        let mut sum = 0.0;
        for v in self.series.iter().filter(|s| s.fill.is_some()).filter_map(|s| s.value_at(x)) {
            any = true;
            sum += v;
        }
        any.then_some(sum)
    }

    /// The min and max of everything drawn in `[w0, w1]` (lines, stacked totals, reference lines).
    pub fn y_extent(&self, w0: f64, w1: f64) -> Option<(f64, f64)> {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let mut take = |y: f64| {
            if y.is_finite() {
                lo = lo.min(y);
                hi = hi.max(y);
            }
        };
        for s in self.series.iter().filter(|s| s.fill.is_none()) {
            for p in window_points(&s.points, w0, w1) {
                take(p.1);
            }
        }
        if self.is_stacked() {
            let xs = self.stack_xs(w0, w1);
            for (_, run) in self.stack_layers(&xs) {
                for (_, a, b) in run {
                    take(a);
                    take(b);
                }
            }
        }
        for l in &self.hlines {
            take(l.y);
        }
        (lo <= hi).then_some((lo, hi))
    }

    /// The y range to draw for the window `[w0, w1]`: the data in view plus 4 % headroom, then pushed out to the next "nice" tick
    /// above the top (and below the bottom), so the range always contains the data, the peak never sits on the edge and the top
    /// tick label is at or above the highest value. y re-fits when x zooms (PlotZoom). Zero-based panels keep y = 0 at the bottom.
    pub fn y_range(&self, w0: f64, w1: f64) -> (f64, f64) {
        let Some((mut lo, mut hi)) = self.y_extent(w0, w1) else { return (0.0, 1.0) };
        let anchored = self.zero_base || self.is_stacked();
        if anchored && lo >= 0.0 {
            lo = 0.0;
        }
        if hi - lo < 1e-12 * hi.abs().max(1.0) {
            let e = if hi == 0.0 { 1.0 } else { hi.abs() * 0.1 };
            hi += e;
            if !(anchored && lo == 0.0) {
                lo -= e;
            }
        }
        let floor_at_zero = anchored && lo == 0.0;
        let pad = HEADROOM * (hi - lo);
        hi += pad;
        if !floor_at_zero {
            lo -= pad;
        }
        let step = ticks::nice_step(hi - lo, 5);
        hi = (hi / step - 1e-9).ceil() * step;
        if !floor_at_zero {
            lo = (lo / step + 1e-9).floor() * step;
        }
        (lo, hi)
    }
}

/// An event on the x axis: a dotted vertical line across every panel and a tag on the events strip.
#[derive(Debug, Clone)]
pub struct EventMark {
    /// Position on the x axis.
    pub x: f64,
    /// Name (`Burnout`); drawn upper-case.
    pub label: String,
    /// Highlight in `accent` on the strip (apogee).
    pub accent: bool,
}

impl EventMark {
    /// An event at `x`.
    pub fn new(x: f64, label: impl Into<String>) -> Self {
        EventMark { x, label: label.into(), accent: false }
    }

    /// Highlights the event in `accent`.
    pub fn accent(mut self) -> Self {
        self.accent = true;
        self
    }
}

/// A flight phase bar on the events strip (propulsion while burning, recovery under canopy).
#[derive(Debug, Clone, Copy)]
pub struct Phase {
    /// Start on the x axis.
    pub from: f64,
    /// End on the x axis.
    pub to: f64,
    /// Bar colour role.
    pub color: SeriesColor,
}

/// The shared x axis: how its values and ranges are written.
#[derive(Debug, Clone)]
pub struct Axis {
    /// Name used in breadcrumbs when the unit is empty (`Mach 0.10 – 3.00`).
    pub label: String,
    /// Unit suffix (`s`); empty for dimensionless axes.
    pub unit: String,
    /// Prefix of single readouts (`M ` → `M 1.10`).
    pub prefix: String,
    /// Decimal places of readouts.
    pub decimals: usize,
}

impl Axis {
    /// Time in seconds: `4.20 s`, `0 – 96 s`.
    pub fn time() -> Self {
        Axis { label: "Time".into(), unit: "s".into(), prefix: String::new(), decimals: 2 }
    }

    /// Mach number: `M 1.10`, `Mach 0.10 – 3.00`.
    pub fn mach() -> Self {
        Axis { label: "Mach".into(), unit: String::new(), prefix: "M ".into(), decimals: 2 }
    }

    /// `x` as a readout: `4.20 s`, `M 1.10`.
    pub fn format(&self, x: f64) -> String {
        let n = format!("{}{}", self.prefix, ticks::fmt_fixed(x, self.decimals));
        if self.unit.is_empty() {
            n
        } else {
            format!("{n} {}", self.unit)
        }
    }

    /// A width as a readout: `2.70 s` (no prefix, since a width is not a position).
    pub fn format_width(&self, w: f64) -> String {
        let n = ticks::fmt_fixed(w, self.decimals);
        if self.unit.is_empty() {
            n
        } else {
            format!("{n} {}", self.unit)
        }
    }

    /// `lo` and `hi` as a range at the span's natural precision (whole numbers for wide ranges, 2 places for ordinary
    /// ones, more when zoomed far in), with one place when a wide range has fractional ends: `0 – 96 s`,
    /// `2.10 – 4.80 s`, `0.0 – 60.7 s`, `Mach 0.10 – 3.00`.
    pub fn format_range(&self, lo: f64, hi: f64) -> String {
        let mut d = ticks::range_decimals(hi - lo);
        if d == 0 && (lo.fract().abs() > 1e-9 || hi.fract().abs() > 1e-9) {
            d = 1;
        }
        let body = format!("{} – {}", ticks::fmt_fixed(lo, d), ticks::fmt_fixed(hi, d));
        if !self.unit.is_empty() {
            format!("{body} {}", self.unit)
        } else if !self.label.is_empty() {
            format!("{} {body}", self.label)
        } else {
            body
        }
    }
}

/// How a group chooses its y ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum YTicks {
    /// 1-2-5 multiples inside the range, labels right-aligned against the plot (the time plots).
    #[default]
    Nice,
    /// This many evenly spaced ticks from the bottom to the top of each panel, labels left-aligned in the gutter (the
    /// AnalyzeScreen's: `0.8 1.4 2.0 2.6 3.2`).
    Even(usize),
}

/// Panels that share one x axis, with the events drawn across them.
#[derive(Debug, Clone)]
pub struct PlotGroup {
    /// The shared x axis.
    pub x: Axis,
    /// Plots, top to bottom.
    pub panels: Vec<Panel>,
    /// Vertical event lines (the reference flight's only).
    pub events: Vec<EventMark>,
    /// Phase bars of the events strip.
    pub phases: Vec<Phase>,
    /// Show the events strip (Simulate) above the plots, always. Otherwise a minimap appears only while zoomed.
    pub timeline: bool,
    /// Draw each event's name next to its line on the plots.
    pub event_tags: bool,
    /// Pixels kept clear at the right of every plot for direct labels, so the x axes stay aligned.
    pub right_margin: f32,
    /// The full x range; `None` takes the extent of the data.
    pub x_range: Option<(f64, f64)>,
    /// How the y ticks of every panel are chosen.
    pub y_ticks: YTicks,
    /// Shade the x range from this value to the end of the axis as lying outside the flown envelope, labelled with the text
    /// (`beyond flight · M 0.53`) at the top of the first panel.
    pub beyond: Option<(f64, String)>,
}

impl PlotGroup {
    /// An empty group on `x`.
    pub fn new(x: Axis) -> Self {
        PlotGroup {
            x,
            panels: Vec::new(),
            events: Vec::new(),
            phases: Vec::new(),
            timeline: false,
            event_tags: false,
            right_margin: 0.0,
            x_range: None,
            y_ticks: YTicks::Nice,
            beyond: None,
        }
    }

    /// Adds a panel below the others.
    pub fn panel(mut self, panel: Panel) -> Self {
        self.panels.push(panel);
        self
    }

    /// The full x range: [`PlotGroup::x_range`] or the extent of every series.
    pub fn x_extent(&self) -> (f64, f64) {
        if let Some(r) = self.x_range {
            return r;
        }
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for s in self.panels.iter().flat_map(|p| &p.series) {
            if let (Some(a), Some(b)) = (s.points.first(), s.points.last()) {
                lo = lo.min(a.0);
                hi = hi.max(b.0);
            }
        }
        if lo < hi {
            (lo, hi)
        } else {
            (0.0, 1.0)
        }
    }

    /// The interpolated value of series `series` of panel `panel` at `x` (for "values at cursor" panels).
    pub fn value_at(&self, panel: usize, series: usize, x: f64) -> Option<f64> {
        self.panels.get(panel)?.series.get(series)?.value_at(x)
    }
}

/// The samples of `points` within `[w0, w1]`, plus the interpolated values at the window's ends when they fall inside
/// the data, so a line enters and leaves the plot exactly at its edges.
pub(crate) fn window_points(points: &[(f64, f64)], w0: f64, w1: f64) -> Vec<(f64, f64)> {
    let (Some(first), Some(last)) = (points.first(), points.last()) else { return Vec::new() };
    if w1 < first.0 || w0 > last.0 {
        return Vec::new();
    }
    let a = points.partition_point(|p| p.0 < w0);
    let b = points.partition_point(|p| p.0 <= w1);
    let inner = &points[a..b];
    let at = |x: f64| -> Option<(f64, f64)> {
        let i = points.partition_point(|p| p.0 <= x);
        if i == 0 || i >= points.len() {
            return None;
        }
        let (p, q) = (points[i - 1], points[i]);
        let y = if q.0 == p.0 { q.1 } else { p.1 + (q.1 - p.1) * (x - p.0) / (q.0 - p.0) };
        Some((x, y))
    };
    let mut out = Vec::with_capacity(inner.len() + 2);
    if inner.first().is_none_or(|p| p.0 > w0) {
        out.extend(at(w0));
    }
    out.extend_from_slice(inner);
    if inner.last().is_none_or(|p| p.0 < w1) {
        out.extend(at(w1));
    }
    out
}

// ---------------------------------------------------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------------------------------------------------

/// What a plot group tells the app.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlotMessage {
    /// The cursor is over x (or left the plots): moves the synced crosshair.
    Hover(Option<f64>),
    /// The selection band changed while dragging; `None` clears it.
    Band(Option<(f64, f64)>),
    /// Enter: zoom to the band and push a level.
    ZoomIn,
    /// Esc: clear the band, else pop one level.
    Back,
    /// Shift+Esc or double-click: back to the full range.
    Reset,
    /// A breadcrumb was clicked: jump to that level (0 = full range).
    Jump(usize),
    /// Wheel / pan: set the current window without adding a stack level (at the full range this pushes one).
    Window((f64, f64)),
    /// Cmd/Ctrl + scroll (iced delivers no pinch): zoom x about the cursor. The first event of a gesture pushes a level; the
    /// rest edit it, and [`PlotMessage::PinchEnd`] (the modifier released) closes it.
    Pinch((f64, f64)),
    /// The pinch gesture ended: a level equal to the one below (or the full range) is dropped again.
    PinchEnd,
    /// A click on an editable panel's chrome: the panel's index, what was clicked and the chip's rectangle in the plot canvas.
    Panel(usize, PanelAction, Rectangle),
}

/// The PlotZoom state of one linked group: the stack of x ranges (level 0 is the full range), the selection band and
/// the hover x. Each view keeps its own.
#[derive(Debug, Clone, PartialEq)]
pub struct PlotState {
    full: (f64, f64),
    stack: Vec<(f64, f64)>,
    band: Option<(f64, f64)>,
    hover: Option<f64>,
    /// A Cmd/Ctrl + scroll zoom is in progress: its level is on top of the stack until the modifier is released.
    pinching: bool,
}

impl PlotState {
    /// A state showing the whole of `full`.
    pub fn new(full: (f64, f64)) -> Self {
        PlotState { full, stack: vec![full], band: None, hover: None, pinching: false }
    }

    /// A state for `group`, showing all of it.
    pub fn for_group(group: &PlotGroup) -> Self {
        Self::new(group.x_extent())
    }

    /// New data arrived with a different full range (a re-run): returns to the full view and drops the band.
    pub fn set_full(&mut self, full: (f64, f64)) {
        if full != self.full {
            *self = PlotState { hover: self.hover, ..PlotState::new(full) };
        }
    }

    /// The full x range.
    pub fn full(&self) -> (f64, f64) {
        self.full
    }

    /// The x window the plots show (top of the stack).
    pub fn window(&self) -> (f64, f64) {
        *self.stack.last().unwrap_or(&self.full)
    }

    /// The zoom levels, widest first; the breadcrumb.
    pub fn levels(&self) -> &[(f64, f64)] {
        &self.stack
    }

    /// Number of levels (1 = full range).
    pub fn depth(&self) -> usize {
        self.stack.len()
    }

    /// Whether the window is narrower than the full range.
    pub fn is_zoomed(&self) -> bool {
        self.stack.len() > 1
    }

    /// The selection band, if any.
    pub fn band(&self) -> Option<(f64, f64)> {
        self.band
    }

    /// The x under the (possibly external) crosshair, for "values at cursor" panels and linked views.
    pub fn hover_x(&self) -> Option<f64> {
        self.hover
    }

    /// Moves the crosshair, e.g. when a number in the results panel is clicked ("every number moves the crosshair").
    pub fn set_hover(&mut self, x: Option<f64>) {
        self.hover = x.filter(|x| x.is_finite());
    }

    fn min_span(&self) -> f64 {
        (self.full.1 - self.full.0).abs() * MIN_SPAN_FRAC
    }

    /// Applies a message from the plots, the header or the keyboard.
    pub fn update(&mut self, message: PlotMessage) {
        match message {
            PlotMessage::Hover(x) => self.set_hover(x),
            PlotMessage::Band(band) => {
                let (w0, w1) = self.window();
                self.band = band.map(|(a, b)| (a.min(b).max(w0), a.max(b).min(w1))).filter(|(a, b)| b - a > self.min_span());
            }
            PlotMessage::ZoomIn => {
                if let Some(band) = self.band.take() {
                    self.stack.push(band);
                }
            }
            PlotMessage::Back => {
                if self.band.take().is_none() && self.stack.len() > 1 {
                    self.stack.pop();
                }
            }
            PlotMessage::Reset => {
                self.band = None;
                self.stack.truncate(1);
            }
            PlotMessage::Jump(level) => {
                self.band = None;
                self.stack.truncate(level.saturating_add(1).clamp(1, self.stack.len()));
            }
            PlotMessage::Window(w) => {
                let w = self.clamp_window(w);
                if (w.1 - w.0) >= (self.full.1 - self.full.0) - self.min_span() {
                    self.stack.truncate(1);
                } else if self.stack.len() == 1 {
                    self.stack.push(w);
                } else if let Some(top) = self.stack.last_mut() {
                    *top = w;
                }
                self.band = self.band.and_then(|(a, b)| {
                    let (w0, w1) = self.window();
                    let (a, b) = (a.max(w0), b.min(w1));
                    (b > a).then_some((a, b))
                });
            }
            PlotMessage::Pinch(w) => {
                let w = self.clamp_window(w);
                if !self.pinching {
                    self.pinching = true;
                    let current = self.window();
                    self.stack.push(current);
                }
                if let Some(top) = self.stack.last_mut() {
                    *top = w;
                }
                self.band = self.band.and_then(|(a, b)| {
                    let (w0, w1) = self.window();
                    let (a, b) = (a.max(w0), b.min(w1));
                    (b > a).then_some((a, b))
                });
            }
            PlotMessage::PinchEnd => {
                if std::mem::take(&mut self.pinching) {
                    let eps = self.min_span();
                    let (top, below) = (self.window(), self.stack.len().checked_sub(2).and_then(|i| self.stack.get(i)).copied());
                    let full = (top.1 - top.0) >= (self.full.1 - self.full.0) - eps;
                    if full {
                        self.stack.truncate(1);
                    } else if below.is_some_and(|b| (b.0 - top.0).abs() <= eps && (b.1 - top.1).abs() <= eps) {
                        self.stack.pop();
                    }
                }
            }
            // Panel chrome edits the layout, not the view.
            PlotMessage::Panel(..) => {}
        }
    }

    /// `w` sorted, no narrower than the minimum span, and shifted (not squashed) to lie inside the full range.
    pub fn clamp_window(&self, w: (f64, f64)) -> (f64, f64) {
        let (f0, f1) = self.full;
        let span = (w.1 - w.0).abs().clamp(self.min_span(), f1 - f0);
        let lo = w.0.min(w.1).clamp(f0, f1 - span);
        (lo, lo + span)
    }

    /// Whether Enter would zoom now.
    pub fn can_zoom_in(&self) -> bool {
        self.band.is_some()
    }

    /// Whether Esc would do anything now.
    pub fn can_back(&self) -> bool {
        self.band.is_some() || self.stack.len() > 1
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Header: breadcrumb + hints
// ---------------------------------------------------------------------------------------------------------------------

/// The centre column behind a plot group: `bg-100`, no frame (the plots sit flush on it, as in the kit).
pub fn surface(theme: &Theme) -> container::Style {
    let t = theme::tokens(theme);
    container::Style { background: Some(t.bg_100.into()), text_color: Some(t.ink), ..container::Style::default() }
}

fn mono<'a, M: 'a>(content: impl text::IntoFragment<'a>, size: f32, color: impl Fn(&Tokens) -> Color + 'a) -> Element<'a, M> {
    text(content).font(MONO).size(size).style(move |theme: &Theme| text::Style { color: Some(color(theme::tokens(theme))), ..text::Style::default() }).boxed()
}

fn framed(color: fn(&Tokens) -> Color, background: fn(&Tokens) -> Option<Color>) -> impl Fn(&Theme) -> container::Style {
    move |theme| {
        let t = theme::tokens(theme);
        container::Style {
            background: background(t).map(Into::into),
            border: Border { color: color(t), width: 1.0, radius: 0.0.into() },
            ..container::Style::default()
        }
    }
}

fn keycap<'a, M: 'a>(label: &'a str) -> Element<'a, M> {
    container(mono(label, 10.0, |t| t.ink_muted)).padding([1, 4]).style(framed(|t| t.line_200, |t| Some(t.bg_300))).boxed()
}

fn crumb<'a>(label: String, level: usize, current: bool) -> Element<'a, PlotMessage> {
    button(mono(label, tokens::TEXT_LABEL, move |t| if current { t.ink } else { t.ink_muted }))
        .padding([2, 8])
        .on_press(PlotMessage::Jump(level))
        .style(move |theme, status| {
            let t = theme::tokens(theme);
            let hot = matches!(status, button::Status::Hovered | button::Status::Pressed);
            button::Style {
                background: Some(if hot { t.bg_400 } else { t.bg_100 }.into()),
                text_color: if current { t.ink } else { t.ink_muted },
                border: Border { color: if current { t.accent } else { t.line_200 }, width: 1.0, radius: 0.0.into() },
                ..button::Style::default()
            }
        })
        .boxed()
}

/// The group's header row: the zoom breadcrumb on the left (the current level framed `accent`, every crumb clickable),
/// and on the right the hover x with the key hints, or, while a band is showing, its range and width in `accent`
/// (`2.10 – 4.80 s · Δ 2.70 s`).
pub fn header<'a>(group: &PlotGroup, state: &PlotState) -> Element<'a, PlotMessage> {
    let (crumbs, hint) = header_parts(group, state, false);
    row![crumbs, Space::new().width(Length::Fill), hint].spacing(tokens::SPACE_3).align_y(iced::Center).boxed()
}

/// The two halves of [`header`]: the zoom breadcrumb and the hints, for a screen that puts its own controls between them
/// (Simulate's plot presets). `compact` shortens the idle hint to its key caps.
pub fn header_parts<'a>(group: &PlotGroup, state: &PlotState, compact: bool) -> (Element<'a, PlotMessage>, Element<'a, PlotMessage>) {
    let last = state.depth() - 1;
    let mut crumbs: Vec<Element<'a, PlotMessage>> = Vec::new();
    for (i, (lo, hi)) in state.levels().iter().enumerate() {
        if i > 0 {
            crumbs.push(mono("›", tokens::TEXT_LABEL, |t| t.ink_faint));
        }
        crumbs.push(crumb(group.x.format_range(*lo, *hi), i, i == last));
    }

    let mut hint: Vec<Element<'a, PlotMessage>> = Vec::new();
    if let Some((a, b)) = state.band() {
        let range = format!("{} · Δ {}", group.x.format_range(a, b), group.x.format_width(b - a));
        hint.push(mono(range, tokens::TEXT_LABEL, |t| t.accent));
        hint.push(keycap("Enter"));
        hint.push(mono("zoom", tokens::TEXT_LABEL, |t| t.ink_muted));
        hint.push(keycap("Esc"));
        hint.push(mono("clear", tokens::TEXT_LABEL, |t| t.ink_muted));
    } else {
        // Compact (Simulate's shared row): only the key caps; the hover value is in the plots' tags.
        if !compact {
            if let Some(x) = state.hover_x() {
                hint.push(mono(format!("hover {}", group.x.format(x)), tokens::TEXT_LABEL, |t| t.ink_muted));
            }
            hint.push(mono("drag x to select ·", tokens::TEXT_LABEL, |t| t.ink_faint));
        }
        // Compact and zoomed: the breadcrumb is long, so only the way back.
        if !(compact && state.depth() > 1) {
            hint.push(keycap("Enter"));
            hint.push(mono("zoom ·", tokens::TEXT_LABEL, |t| t.ink_faint));
        }
        hint.push(keycap("Esc"));
        hint.push(mono("back", tokens::TEXT_LABEL, |t| t.ink_faint));
    }

    let crumbs = row(crumbs).spacing(tokens::SPACE_2).align_y(iced::Center).boxed();
    let hint = row(hint).spacing(tokens::SPACE_2).align_y(iced::Center).boxed();
    (crumbs, hint)
}

/// A legend row: a filled colour square and the name, per item, in text colours (never the series colour).
pub fn legend<'a, M: 'a>(items: &[(SeriesColor, String)]) -> Element<'a, M> {
    let entries: Vec<Element<'a, M>> = items
        .iter()
        .map(|(color, name)| {
            let color = *color;
            let swatch = container(Space::new())
                .width(8)
                .height(8)
                .style(move |theme| container::Style { background: Some(color.resolve(theme::tokens(theme)).into()), ..container::Style::default() });
            row![swatch, text(name.clone()).size(tokens::TEXT_BODY - 1.0).font(theme::SANS)].spacing(tokens::SPACE_1 + 2.0).align_y(iced::Center).boxed()
        })
        .collect();
    row(entries).spacing(tokens::SPACE_4).align_y(iced::Center).boxed()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ramp() -> Series {
        Series::new("ramp", (0..=10).map(|i| (i as f64, 10.0 * i as f64)).collect())
    }

    #[test]
    fn interpolates_between_samples() {
        let s = ramp();
        assert_eq!(s.value_at(0.0), Some(0.0));
        assert_eq!(s.value_at(10.0), Some(100.0));
        assert_eq!(s.value_at(2.5), Some(25.0));
        assert_eq!(s.value_at(9.99), Some(99.9));
    }

    #[test]
    fn no_value_outside_the_data() {
        let s = ramp();
        assert_eq!(s.value_at(-0.1), None);
        assert_eq!(s.value_at(10.1), None);
        assert_eq!(s.value_at(f64::NAN), None);
        assert_eq!(Series::new("empty", vec![]).value_at(1.0), None);
    }

    #[test]
    fn a_step_takes_the_later_value() {
        let s = Series::new("step", vec![(0.0, 0.0), (1.0, 1.0), (1.0, 5.0), (2.0, 5.0)]);
        assert_eq!(s.value_at(1.0), Some(5.0));
        assert_eq!(s.value_at(1.5), Some(5.0));
        assert_eq!(s.value_at(0.5), Some(0.5));
    }

    #[test]
    fn group_value_at_addresses_panel_and_series() {
        let g = PlotGroup::new(Axis::time()).panel(Panel::new("A", "m").with(ramp())).panel(Panel::new("B", "m").with(ramp().color(SeriesColor::Cg)));
        assert_eq!(g.value_at(1, 0, 4.0), Some(40.0));
        assert_eq!(g.value_at(2, 0, 4.0), None);
        assert_eq!(g.value_at(0, 1, 4.0), None);
    }

    #[test]
    fn window_points_clip_exactly_at_the_edges() {
        let s = ramp();
        let w = window_points(&s.points, 2.5, 4.5);
        assert_eq!(w, vec![(2.5, 25.0), (3.0, 30.0), (4.0, 40.0), (4.5, 45.0)]);
        // Edges on samples are not duplicated.
        assert_eq!(window_points(&s.points, 2.0, 4.0), vec![(2.0, 20.0), (3.0, 30.0), (4.0, 40.0)]);
        // A window beyond the data has nothing; one that only overlaps keeps the data's end.
        assert!(window_points(&s.points, 11.0, 12.0).is_empty());
        assert_eq!(window_points(&s.points, 9.5, 20.0), vec![(9.5, 95.0), (10.0, 100.0)]);
    }

    #[test]
    fn y_refits_to_the_window_with_headroom() {
        let p = Panel::new("Alt", "m").with(ramp());
        // 0 – 100 plus 4 % headroom, pushed out to the next 20 above and below: -20 – 120.
        let (lo, hi) = p.y_range(0.0, 10.0);
        assert_eq!((lo, hi), (-20.0, 120.0));
        // Zoomed to 20 – 40 the range re-fits to it (15 – 45), still containing the data with room to spare.
        let (lo, hi) = p.y_range(2.0, 4.0);
        assert_eq!((lo, hi), (15.0, 45.0));
        // Zero-based panels have no headroom below zero.
        let (lo, hi) = Panel::new("Alt", "m").with(ramp()).zero_based().y_range(0.0, 10.0);
        assert_eq!((lo, hi), (0.0, 120.0));
    }

    #[test]
    fn the_y_range_always_contains_the_data_and_ends_on_a_nice_tick() {
        // Awkward maxima: the peak of a 619 m flight, a Mach-sized 0.871, a 1236 m flight, a top just under a tick, and one on it.
        for max in [619.0, 0.871, 1236.0, 599.0, 600.0, 7.3e-4, 98_765.0] {
            let line = Series::new("s", (0..=20).map(|i| (f64::from(i), max * (1.0 - ((f64::from(i) - 10.0) / 10.0).powi(2)))).collect());
            for zero_based in [true, false] {
                let mut p = Panel::new("P", "u").with(line.clone());
                p.zero_base = zero_based;
                let (lo, hi) = p.y_range(0.0, 20.0);
                let (dmin, dmax) = p.y_extent(0.0, 20.0).unwrap();
                assert!(lo <= dmin && hi >= dmax * 1.04 - 1e-12, "{max}: [{lo}, {hi}] vs data [{dmin}, {dmax}]");
                if zero_based {
                    assert_eq!(lo, 0.0, "{max}");
                }
                // The top has at most two significant digits (a 1-2-5 step multiple), and is not far above the data.
                let digits = hi / 10f64.powf(hi.log10().floor() - 1.0);
                assert!((digits - digits.round()).abs() < 1e-6, "{max}: {hi} is not a round number");
                assert!(hi - dmax * 1.04 < 0.3 * (hi - lo), "{max}: {hi} leaves too much room above {dmax}");
                // The evenly spaced ticks of the Analyze plots span the data too (first at the bottom, last at the top).
                let (even, _) = ticks::even_ticks(lo, hi, 5);
                assert!(even.first().is_some_and(|v| *v <= dmin) && even.last().is_some_and(|v| *v >= dmax), "{max}: {even:?}");
                // So do the 1-2-5 ticks: one at or above the maximum would need the range to reach a tick, and the range does.
                assert!(ticks::nice_ticks(lo, hi, 5).last().is_some_and(|v| *v >= dmax * 0.7), "{max}");
            }
        }
        // The 619 m case of the Simulate altitude plot: 0 – 700, the peak well inside.
        let alt = Series::new("alt", vec![(0.0, 0.0), (10.0, 619.0), (20.0, 0.0)]);
        assert_eq!(Panel::new("Altitude", "m").with(alt).zero_based().y_range(0.0, 20.0), (0.0, 700.0));
    }

    #[test]
    fn reference_lines_widen_the_fit() {
        let p = Panel::new("Margin", "cal")
            .with(Series::new("m", vec![(0.0, 1.4), (1.0, 2.5)]))
            .hline(HLine::dashed(1.0, "min 1.0", SeriesColor::Ok))
            .hline(HLine::dashed(3.0, "over-stable 3.0", SeriesColor::Warn));
        let (lo, hi) = p.y_range(0.0, 1.0);
        assert!(lo < 1.0 && hi > 3.0);
    }

    #[test]
    fn a_flat_series_still_has_a_range() {
        let p = Panel::new("Flat", "").with(Series::new("c", vec![(0.0, 5.0), (1.0, 5.0)]));
        let (lo, hi) = p.y_range(0.0, 1.0);
        assert!(lo < 5.0 && hi > 5.0);
        let (lo, hi) = Panel::new("Empty", "").y_range(0.0, 1.0);
        assert_eq!((lo, hi), (0.0, 1.0));
    }

    fn stacked() -> Panel {
        let xs = [0.0, 1.0, 2.0];
        Panel::new("Cd", "")
            .with(Series::new("skin", xs.iter().map(|&x| (x, 0.2)).collect()).color(SeriesColor::Series(1)).filled())
            .with(Series::new("base", xs.iter().map(|&x| (x, 0.1 + 0.1 * x)).collect()).color(SeriesColor::Series(3)).filled())
    }

    #[test]
    fn stacked_layers_accumulate_bottom_first() {
        let p = stacked();
        let xs = p.stack_xs(0.0, 2.0);
        let layers = p.stack_layers(&xs);
        assert_eq!(layers.len(), 2);
        assert_eq!(layers[0].1[1], (1.0, 0.0, 0.2));
        let (_, lo, hi) = layers[1].1[2];
        assert!((lo - 0.2).abs() < 1e-12 && (hi - 0.5).abs() < 1e-12);
        assert!((p.stack_total_at(1.5).unwrap() - 0.45).abs() < 1e-12);
        // Stacked panels are zero-based with headroom only above the total.
        let (lo, hi) = p.y_range(0.0, 2.0);
        assert_eq!(lo, 0.0);
        assert!((0.5 * 1.04..=0.6 + 1e-9).contains(&hi), "{hi}");
    }

    #[test]
    fn zoom_stack_push_pop_reset() {
        let mut s = PlotState::new((0.0, 96.0));
        assert_eq!(s.window(), (0.0, 96.0));
        assert_eq!(s.depth(), 1);
        // Enter with no band does nothing.
        s.update(PlotMessage::ZoomIn);
        assert_eq!(s.depth(), 1);

        s.update(PlotMessage::Band(Some((30.0, 0.0))));
        assert_eq!(s.band(), Some((0.0, 30.0)));
        s.update(PlotMessage::ZoomIn);
        assert_eq!(s.window(), (0.0, 30.0));
        assert_eq!(s.band(), None, "the band clears so another can be dragged straight away");

        s.update(PlotMessage::Band(Some((2.1, 4.8))));
        s.update(PlotMessage::ZoomIn);
        assert_eq!(s.levels(), &[(0.0, 96.0), (0.0, 30.0), (2.1, 4.8)]);

        // Esc steps back one level at a time.
        s.update(PlotMessage::Back);
        assert_eq!(s.window(), (0.0, 30.0));
        s.update(PlotMessage::Back);
        assert_eq!(s.window(), (0.0, 96.0));
        s.update(PlotMessage::Back);
        assert_eq!(s.depth(), 1);

        // Reset (Shift+Esc / double-click) goes straight to the top.
        s.update(PlotMessage::Band(Some((1.0, 5.0))));
        s.update(PlotMessage::ZoomIn);
        s.update(PlotMessage::Band(Some((2.0, 3.0))));
        s.update(PlotMessage::ZoomIn);
        s.update(PlotMessage::Reset);
        assert_eq!((s.depth(), s.band()), (1, None));
    }

    #[test]
    fn esc_clears_the_band_before_popping() {
        let mut s = PlotState::new((0.0, 24.0));
        s.update(PlotMessage::Band(Some((2.0, 10.0))));
        s.update(PlotMessage::ZoomIn);
        s.update(PlotMessage::Band(Some((3.0, 4.0))));
        s.update(PlotMessage::Back);
        assert_eq!((s.depth(), s.band()), (2, None));
        s.update(PlotMessage::Back);
        assert_eq!(s.depth(), 1);
    }

    #[test]
    fn bands_stay_inside_the_window_and_tiny_ones_vanish() {
        let mut s = PlotState::new((0.0, 10.0));
        s.update(PlotMessage::Band(Some((-5.0, 4.0))));
        assert_eq!(s.band(), Some((0.0, 4.0)));
        s.update(PlotMessage::Band(Some((20.0, 30.0))));
        assert_eq!(s.band(), None);
        s.update(PlotMessage::Band(Some((5.0, 5.0))));
        assert_eq!(s.band(), None);
    }

    #[test]
    fn jump_goes_to_a_breadcrumb_level() {
        let mut s = PlotState::new((0.0, 24.0));
        for band in [(2.0, 20.0), (4.0, 10.0), (5.0, 6.0)] {
            s.update(PlotMessage::Band(Some(band)));
            s.update(PlotMessage::ZoomIn);
        }
        assert_eq!(s.depth(), 4);
        s.update(PlotMessage::Jump(1));
        assert_eq!(s.levels(), &[(0.0, 24.0), (2.0, 20.0)]);
        s.update(PlotMessage::Jump(9));
        assert_eq!(s.depth(), 2, "a stale crumb index is ignored");
        s.update(PlotMessage::Jump(0));
        assert_eq!(s.depth(), 1);
    }

    #[test]
    fn wheel_and_pan_modify_the_current_level() {
        let mut s = PlotState::new((0.0, 100.0));
        // From the full range a wheel zoom pushes a level, so Esc returns to full.
        s.update(PlotMessage::Window((10.0, 60.0)));
        assert_eq!(s.levels(), &[(0.0, 100.0), (10.0, 60.0)]);
        // Further wheel/pan edits that level without growing the stack.
        s.update(PlotMessage::Window((20.0, 70.0)));
        assert_eq!(s.levels(), &[(0.0, 100.0), (20.0, 70.0)]);
        // Panning past the end slides the window back inside, keeping its width.
        s.update(PlotMessage::Window((80.0, 130.0)));
        assert_eq!(s.window(), (50.0, 100.0));
        // Zooming all the way out returns to level 0.
        s.update(PlotMessage::Window((-10.0, 120.0)));
        assert_eq!(s.depth(), 1);
    }

    #[test]
    fn pinch_pushes_one_level_on_release() {
        let mut s = PlotState::new((0.0, 100.0));
        s.update(PlotMessage::Window((10.0, 60.0)));
        // A pinch (Cmd/Ctrl + scroll) edits a fresh level on top of the current one, however many events it takes ...
        s.update(PlotMessage::Pinch((20.0, 50.0)));
        s.update(PlotMessage::Pinch((25.0, 45.0)));
        assert_eq!(s.levels(), &[(0.0, 100.0), (10.0, 60.0), (25.0, 45.0)]);
        s.update(PlotMessage::PinchEnd);
        // ... and Esc pops exactly that one.
        s.update(PlotMessage::Back);
        assert_eq!(s.levels(), &[(0.0, 100.0), (10.0, 60.0)]);
        // A pinch that ends where it started adds nothing; one that zooms out to the full range returns to level 0.
        s.update(PlotMessage::Pinch((10.0, 60.0)));
        s.update(PlotMessage::PinchEnd);
        assert_eq!(s.depth(), 2);
        s.update(PlotMessage::Pinch((0.0, 100.0)));
        s.update(PlotMessage::PinchEnd);
        assert_eq!(s.depth(), 1);
    }

    #[test]
    fn reset_returns_to_the_full_range_from_any_depth() {
        let mut s = PlotState::new((0.0, 100.0));
        s.update(PlotMessage::Band(Some((10.0, 60.0))));
        s.update(PlotMessage::ZoomIn);
        s.update(PlotMessage::Band(Some((20.0, 30.0))));
        s.update(PlotMessage::ZoomIn);
        s.update(PlotMessage::Band(Some((22.0, 24.0))));
        s.update(PlotMessage::Reset);
        assert_eq!((s.depth(), s.band(), s.window()), (1, None, (0.0, 100.0)));
    }

    #[test]
    fn set_full_resets_only_when_the_range_changes() {
        let mut s = PlotState::new((0.0, 10.0));
        s.update(PlotMessage::Band(Some((1.0, 2.0))));
        s.update(PlotMessage::ZoomIn);
        s.set_hover(Some(1.5));
        s.set_full((0.0, 10.0));
        assert_eq!(s.depth(), 2);
        s.set_full((0.0, 20.0));
        assert_eq!((s.depth(), s.window(), s.hover_x()), (1, (0.0, 20.0), Some(1.5)));
    }

    #[test]
    fn hover_is_reported_and_cleared() {
        let mut s = PlotState::new((0.0, 10.0));
        s.update(PlotMessage::Hover(Some(4.2)));
        assert_eq!(s.hover_x(), Some(4.2));
        s.update(PlotMessage::Hover(Some(f64::NAN)));
        assert_eq!(s.hover_x(), None);
    }

    #[test]
    fn axis_formats_match_the_design() {
        let t = Axis::time();
        assert_eq!(t.format(4.2), "4.20 s");
        assert_eq!(t.format_range(0.0, 96.0), "0 – 96 s");
        assert_eq!(t.format_range(2.1, 4.8), "2.10 – 4.80 s");
        assert_eq!(t.format_range(3.0, 3.4), "3.00 – 3.40 s");
        assert_eq!(t.format_range(0.0, 60.7), "0.0 – 60.7 s");
        assert_eq!(t.format_width(2.7), "2.70 s");
        let m = Axis::mach();
        assert_eq!(m.format(1.1), "M 1.10");
        assert_eq!(m.format_range(0.1, 3.0), "Mach 0.10 – 3.00");
    }

    #[test]
    fn extent_comes_from_the_data_unless_overridden() {
        let mut g = PlotGroup::new(Axis::time())
            .panel(Panel::new("A", "m").with(ramp()))
            .panel(Panel::new("B", "m").with(Series::new("b", vec![(-1.0, 0.0), (12.0, 1.0)])));
        assert_eq!(g.x_extent(), (-1.0, 12.0));
        g.x_range = Some((0.0, 96.0));
        assert_eq!(g.x_extent(), (0.0, 96.0));
        assert_eq!(PlotGroup::new(Axis::time()).x_extent(), (0.0, 1.0));
    }

    #[test]
    fn series_colours_clamp_into_the_four_slots() {
        let t = &tokens::DARK;
        assert_eq!(SeriesColor::Series(1).resolve(t), t.series_1);
        assert_eq!(SeriesColor::Series(4).resolve(t), t.series_4);
        assert_eq!(SeriesColor::Series(9).resolve(t), t.series_4);
        assert_eq!(SeriesColor::Series(0).resolve(t), t.series_1);
    }
}
