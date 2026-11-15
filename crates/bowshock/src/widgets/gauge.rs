//! StabilityGauge (design/components/StabilityGauge): the static margin in calibers, read at a glance. [`full`] is the
//! Analyze header card; [`compact`] is the status strip's band-and-tick. Both draw `err` below the marginal threshold,
//! `warn` up to the stable threshold and `ok` above it (the thresholds are user settings, see [`Bands`]), with a 2 px `ink`
//! needle that springs to each new value in about 120 ms ([`GaugeState`]). Above the scale's end the needle pins right and
//! the state word becomes "Over-stable".
//!
//! The app owns one [`GaugeState`], calls [`GaugeState::set`] whenever the margin changes and, while
//! [`GaugeState::moving`] is true, subscribes to `iced::window::frames()` and calls [`GaugeState::tick`]:
//!
//! ```ignore
//! // update: Message::Frame(now) => self.gauge.tick(now),
//! // subscription: self.gauge.moving().then(|| iced::window::frames().map(Message::Frame))
//! // view: gauge::full(&self.gauge, &Bands::default(), Some("146.3 mm".into()))
//! ```

use std::time::Instant;

use iced::widget::canvas::{self, Frame, Geometry, Path, Stroke};
use iced::widget::{column, container, row, text, Space};
use iced::{alignment, mouse, Element, Fill, Pixels, Point, Rectangle, Renderer, Size, Theme, Widget};

use super::glyph::Tone;
use crate::theme::{self, tokens, MONO};
use crate::tokens::{SPACE_1, SPACE_3, TEXT_LABEL};

/// Margin thresholds and scale length, in calibers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bands {
    /// Below this the rocket is unstable (`err`).
    pub marginal: f32,
    /// From `marginal` up to this the margin is marginal (`warn`); above it is stable (`ok`).
    pub stable: f32,
    /// The scale's right end; a margin beyond it is over-stable.
    pub max: f32,
}

impl Default for Bands {
    /// The kit's defaults: 0.5 and 1.0 on a 0 to 3 cal scale.
    fn default() -> Self {
        Self { marginal: 0.5, stable: 1.0, max: 3.0 }
    }
}

impl From<bowshock_model::StabilityBands> for Bands {
    /// The document's thresholds as the gauge's (`over` is the end of the scale).
    fn from(b: bowshock_model::StabilityBands) -> Self {
        Self { marginal: b.marginal as f32, stable: b.stable as f32, max: b.over as f32 }
    }
}

/// How a margin reads against [`Bands`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stability {
    /// Below the marginal threshold.
    Unstable,
    /// Between the thresholds.
    Marginal,
    /// Above the stable threshold.
    Stable,
    /// Above the scale: weathercocking risk.
    OverStable,
}

impl Stability {
    /// The state word, always shown next to the value so colour is never the only signal.
    pub fn word(self) -> &'static str {
        match self {
            Stability::Unstable => "UNSTABLE",
            Stability::Marginal => "MARGINAL",
            Stability::Stable => "STABLE",
            Stability::OverStable => "OVER-STABLE",
        }
    }

    /// The tone of the state word.
    pub fn tone(self) -> Tone {
        match self {
            Stability::Unstable => Tone::Err,
            Stability::Marginal | Stability::OverStable => Tone::Warn,
            Stability::Stable => Tone::Ok,
        }
    }
}

impl Bands {
    /// Classifies `margin` (calibers).
    pub fn classify(&self, margin: f32) -> Stability {
        if margin > self.max {
            Stability::OverStable
        } else if margin >= self.stable {
            Stability::Stable
        } else if margin >= self.marginal {
            Stability::Marginal
        } else {
            Stability::Unstable
        }
    }
}

/// The animated needle: a damped spring (natural frequency 48 rad/s, damping ratio 0.8) that settles in about 120 ms.
#[derive(Debug, Clone, Copy)]
pub struct GaugeState {
    value: f32,
    velocity: f32,
    target: f32,
    last: Option<Instant>,
}

const OMEGA: f32 = 48.0;
const ZETA: f32 = 0.8;

impl GaugeState {
    /// A gauge resting at `margin`.
    pub fn new(margin: f32) -> Self {
        Self { value: margin, velocity: 0.0, target: margin, last: None }
    }

    /// Sets the margin the needle moves to. Cheap to call on every edit.
    pub fn set(&mut self, margin: f32) {
        if (margin - self.target).abs() > f32::EPSILON {
            self.target = margin;
        }
    }

    /// Jumps to `margin` without animating (first show, switching configuration).
    pub fn snap(&mut self, margin: f32) {
        *self = Self::new(margin);
    }

    /// The margin the needle is moving to; this is what the value text shows.
    pub fn target(&self) -> f32 {
        self.target
    }

    /// Where the needle is drawn now.
    pub fn value(&self) -> f32 {
        self.value
    }

    /// Whether the needle still needs frames. Subscribe to `window::frames()` only while this is true.
    pub fn moving(&self) -> bool {
        (self.value - self.target).abs() > 1e-3 || self.velocity.abs() > 1e-2
    }

    /// Advances the spring to `now` (the instant of a `window::frames()` event).
    pub fn tick(&mut self, now: Instant) {
        let dt = self.last.map_or(1.0 / 60.0, |last| now.saturating_duration_since(last).as_secs_f32()).min(0.05);
        self.last = Some(now);
        self.step(dt);
        if !self.moving() {
            self.value = self.target;
            self.velocity = 0.0;
            self.last = None;
        }
    }

    /// Integrates the spring by `dt` seconds in sub-steps of at most 4 ms.
    pub fn step(&mut self, dt: f32) {
        let steps = (dt / 0.004).ceil().max(1.0);
        let h = dt / steps;
        for _ in 0..steps as u32 {
            let a = -2.0 * ZETA * OMEGA * self.velocity - OMEGA * OMEGA * (self.value - self.target);
            self.velocity += a * h;
            self.value += self.velocity * h;
        }
    }
}

/// The full gauge card: `STATIC MARGIN` and the state word, the headline value in the 20 px readout with `cal`, an optional
/// margin in length units (`"146.3 mm"`) on the right, and the ruled scale.
pub fn full<'a, M: 'a>(state: &GaugeState, bands: &Bands, length: Option<String>) -> Element<'a, M> {
    let stability = bands.classify(state.target);
    let word = text(stability.word()).font(MONO).size(TEXT_LABEL).style(match stability.tone() {
        Tone::Err => theme::colored(|t| t.err),
        Tone::Ok => theme::colored(|t| t.ok),
        _ => theme::colored(|t| t.warn),
    });
    let header = row![text("STATIC MARGIN").font(MONO).size(TEXT_LABEL).style(theme::faint), Space::new().width(Fill), word];
    let mut readout = row![text(format!("{:.2}", state.target)).font(MONO).size(20).style(theme::ink), text("cal").font(MONO).size(12).style(theme::muted),]
        .spacing(SPACE_1 + 2.0)
        .align_y(alignment::Vertical::Bottom);
    readout = readout.push(Space::new().width(Fill).boxed());
    if let Some(length) = length {
        readout = readout.push(text(length).font(MONO).size(12).style(theme::muted).boxed());
    }
    let scale = canvas::Canvas::new(Scale { value: state.value, bands: *bands, compact: false }).width(Fill).height(FULL_H);
    container(column![header, readout, scale].spacing(SPACE_1 + 2.0)).padding(SPACE_3).style(|t: &Theme| theme::panel(t)).boxed()
}

/// The status strip's compact gauge: a 60 px band strip with the needle and no ticks or labels.
pub fn compact<'a, M: 'a>(state: &GaugeState, bands: &Bands) -> Element<'a, M> {
    canvas::Canvas::new(Scale { value: state.value, bands: *bands, compact: true }).width(COMPACT_W).height(COMPACT_H).boxed()
}

/// Width of the compact gauge.
pub const COMPACT_W: f32 = 60.0;
/// Height of the compact gauge.
pub const COMPACT_H: f32 = 14.0;
const FULL_H: f32 = 36.0;

struct Scale {
    value: f32,
    bands: Bands,
    compact: bool,
}

impl<M> canvas::Program<M> for Scale {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = tokens(theme);
        let mut frame = Frame::new(renderer, bounds.size());
        let inset = if self.compact { 1.0 } else { 6.0 };
        let (x0, x1) = (inset, bounds.width - inset);
        let x_of = |v: f32| x0 + (x1 - x0) * (v / self.bands.max).clamp(0.0, 1.0);

        // Band strip.
        let (strip_y, strip_h) = if self.compact { (4.0, 6.0) } else { (6.0, 3.0) };
        let bands = [(0.0, self.bands.marginal, t.err), (self.bands.marginal, self.bands.stable, t.warn), (self.bands.stable, self.bands.max, t.ok)];
        for (from, to, color) in bands {
            frame.fill_rectangle(Point::new(x_of(from), strip_y), Size::new(x_of(to) - x_of(from), strip_h), color);
        }

        if !self.compact {
            // Ticks every 0.1 cal, longer every 1.0, labelled at whole calibers.
            let tick = Stroke::default().with_color(t.line_200).with_width(1.0);
            let steps = (self.bands.max * 10.0).round() as i32;
            for i in 0..=steps {
                let v = i as f32 / 10.0;
                let major = i % 10 == 0;
                let x = x_of(v).floor() + 0.5;
                frame.stroke(&Path::line(Point::new(x, 13.0), Point::new(x, if major { 19.0 } else { 16.0 })), tick);
                if major {
                    frame.fill_text(canvas::Text {
                        content: format!("{}", i / 10),
                        position: Point::new(x, 21.0),
                        color: t.ink_faint,
                        size: Pixels(TEXT_LABEL),
                        font: MONO,
                        align_x: iced::widget::text::Alignment::Center,
                        align_y: alignment::Vertical::Top,
                        ..canvas::Text::default()
                    });
                }
            }
        }

        // Needle: 2 px ink, taller than the strip; pinned to the ends outside the scale.
        let x = x_of(self.value).clamp(x0 + 1.0, x1 - 1.0);
        let (top, bottom) = if self.compact { (1.0, bounds.height - 1.0) } else { (2.0, 19.0) };
        frame.stroke(&Path::line(Point::new(x, top), Point::new(x, bottom)), Stroke::default().with_color(t.ink).with_width(2.0));

        vec![frame.into_geometry()]
    }
}
