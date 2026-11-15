//! GroundTrack (design/components/GroundTrack): the flight seen from above, north up, equal scale. Pad at the centre,
//! range rings, field radius, wind, one track per flight with its apogee ring and landing square, and a landing label
//! with distance and bearing from the pad. Linked to the time plots: the app passes the plots' hover time in, and a
//! hover over a track publishes the time under it.
//!
//! ```ignore
//! GroundTrackView::new(&self.track, self.plot.hover_x()).element().map(Msg::Ground)
//! // update
//! Msg::Ground(GroundMessage::Hover(t)) => self.plot.set_hover(t),
//! Msg::Ground(GroundMessage::Select(i)) => self.select_flight(i),
//! ```

#![allow(dead_code)] // The screens that compose these widgets arrive after the widgets do.

use std::time::{Duration, Instant};

use iced::advanced::text::Alignment;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineCap, LineDash, Path, Stroke, Text};
use iced::{alignment, mouse, Color, Element, Fill, Pixels, Point, Rectangle, Renderer, Size, Theme, Vector, Widget};

use super::plot::SeriesColor;
use crate::theme::{self, MONO};
use crate::tokens::{Tokens, TEXT_LABEL};

const GLYPH_EM: f32 = 0.6;
const DOUBLE_CLICK: Duration = Duration::from_millis(350);
/// Hover reach of a track, in px.
const HIT_PX: f32 = 8.0;
/// Click reach of a landing marker, in px.
const LANDING_HIT_PX: f32 = 10.0;
/// Rings are drawn at least this many px apart.
const RING_MIN_PX: f32 = 60.0;
/// 95 % of a 2-D normal lies inside the ellipse whose squared Mahalanobis radius is the χ² quantile with 2 dof, 5.991.
const CHI2_95_2DOF: f64 = 5.991;

/// One sample of a track: time since ignition (s), metres east and north of the pad, altitude (m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackPoint {
    /// Time since ignition, s.
    pub t: f64,
    /// Metres east of the pad.
    pub east: f64,
    /// Metres north of the pad.
    pub north: f64,
    /// Altitude above the pad, m.
    pub alt: f64,
}

impl TrackPoint {
    /// Distance from the pad over the ground, m.
    pub fn range(&self) -> f64 {
        self.east.hypot(self.north)
    }
}

/// A 95 % dispersion ellipse: centre, semi-axes and the major axis' angle (radians, counter-clockwise from east), metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ellipse {
    /// Centre, (east, north) m.
    pub centre: (f64, f64),
    /// Semi-major axis, m.
    pub semi_major: f64,
    /// Semi-minor axis, m.
    pub semi_minor: f64,
    /// Angle of the major axis, radians counter-clockwise from east.
    pub angle: f64,
}

impl Ellipse {
    /// The point at parameter `a` (radians) around the ellipse.
    pub fn at(&self, a: f64) -> (f64, f64) {
        let (x, y) = (self.semi_major * a.cos(), self.semi_minor * a.sin());
        let (s, c) = self.angle.sin_cos();
        (self.centre.0 + x * c - y * s, self.centre.1 + x * s + y * c)
    }
}

/// The ellipse that contains 95 % of normally distributed `points` (east, north): centred on their mean, axes from the
/// eigen-decomposition of the sample covariance scaled by the χ² (2 dof) 95 % quantile. `None` for fewer than 3 points.
pub fn confidence_ellipse(points: &[(f64, f64)]) -> Option<Ellipse> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let (mx, my) = points.iter().fold((0.0, 0.0), |(x, y), p| (x + p.0, y + p.1));
    let (mx, my) = (mx / n, my / n);
    let (mut a, mut b, mut c) = (0.0, 0.0, 0.0);
    for p in points {
        let (dx, dy) = (p.0 - mx, p.1 - my);
        a += dx * dx;
        b += dx * dy;
        c += dy * dy;
    }
    let (a, b, c) = (a / (n - 1.0), b / (n - 1.0), c / (n - 1.0));
    let mid = (a + c) / 2.0;
    let rad = (((a - c) / 2.0).powi(2) + b * b).sqrt();
    let (l1, l2) = (mid + rad, (mid - rad).max(0.0));
    Some(Ellipse { centre: (mx, my), semi_major: (CHI2_95_2DOF * l1).sqrt(), semi_minor: (CHI2_95_2DOF * l2).sqrt(), angle: 0.5 * (2.0 * b).atan2(a - c) })
}

/// Compass bearing in degrees (0 = north, clockwise) of the offset (`east`, `north`).
pub fn bearing_deg(east: f64, north: f64) -> f64 {
    east.atan2(north).to_degrees().rem_euclid(360.0)
}

/// `337 m · 067°`: distance and bearing from the pad.
pub fn landing_label(east: f64, north: f64) -> String {
    format!("{:.0} m · {:03.0}°", east.hypot(north), bearing_deg(east, north).rem_euclid(360.0).round() % 360.0)
}

/// The smallest 1-2-5 × 10ⁿ spacing (m) that keeps rings at least `min_px` apart at `px_per_m`.
pub fn ring_spacing(px_per_m: f32, min_px: f32) -> f64 {
    if !(px_per_m.is_finite() && px_per_m > 0.0) {
        return 100.0;
    }
    let want = f64::from(min_px / px_per_m);
    let mag = 10f64.powf(want.log10().floor());
    [1.0, 2.0, 5.0, 10.0].into_iter().map(|m| m * mag).find(|s| *s >= want).unwrap_or(10.0 * mag)
}

/// One flight's path.
#[derive(Debug, Clone)]
pub struct Track {
    /// Flight name.
    pub name: String,
    /// Colour role: `Ink` for a lone flight (its apogee ring then draws in `accent`), `Series(1..4)` when compared.
    pub color: SeriesColor,
    /// Samples in time order; the last is the landing.
    pub points: Vec<TrackPoint>,
    /// Index of the apogee sample.
    pub apogee: Option<usize>,
    /// Dispersion runs' landings (east, north), drawn as dots at 45 % opacity.
    pub scatter: Vec<(f64, f64)>,
    /// Dashed 95 % ellipse around the scatter.
    pub ellipse: Option<Ellipse>,
}

impl Track {
    /// A track with no dispersion.
    pub fn new(name: impl Into<String>, color: SeriesColor, points: Vec<TrackPoint>) -> Self {
        let apogee = points.iter().enumerate().max_by(|a, b| a.1.alt.total_cmp(&b.1.alt)).map(|(i, _)| i);
        Track { name: name.into(), color, points, apogee, scatter: Vec::new(), ellipse: None }
    }

    /// Adds dispersion landings and their 95 % ellipse.
    pub fn with_scatter(mut self, scatter: Vec<(f64, f64)>) -> Self {
        self.ellipse = confidence_ellipse(&scatter);
        self.scatter = scatter;
        self
    }

    /// The landing sample.
    pub fn landing(&self) -> Option<&TrackPoint> {
        self.points.last()
    }

    /// The interpolated sample at time `t`; `None` before the first or after the last.
    pub fn at_time(&self, t: f64) -> Option<TrackPoint> {
        let p = &self.points;
        let (first, last) = (p.first()?, p.last()?);
        if !t.is_finite() || t < first.t || t > last.t {
            return None;
        }
        let i = p.partition_point(|q| q.t <= t);
        if i >= p.len() {
            return Some(*last);
        }
        Some(lerp(&p[i - 1], &p[i], t))
    }
}

fn lerp(a: &TrackPoint, b: &TrackPoint, t: f64) -> TrackPoint {
    let k = if b.t > a.t { ((t - a.t) / (b.t - a.t)).clamp(0.0, 1.0) } else { 1.0 };
    TrackPoint { t: a.t + (b.t - a.t) * k, east: a.east + (b.east - a.east) * k, north: a.north + (b.north - a.north) * k, alt: a.alt + (b.alt - a.alt) * k }
}

/// The wind: its speed and the compass direction it blows *from*.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wind {
    /// Speed, m/s.
    pub speed: f64,
    /// Direction it blows from, degrees clockwise from north.
    pub from_deg: f64,
}

impl Wind {
    /// Unit vector (east, north) the wind blows toward.
    pub fn toward(&self) -> (f64, f64) {
        let a = (self.from_deg + 180.0).to_radians();
        (a.sin(), a.cos())
    }
}

/// Everything a [`GroundTrackView`] draws.
#[derive(Debug, Clone, Default)]
pub struct GroundTrack {
    /// One track per flight.
    pub tracks: Vec<Track>,
    /// Wind readout and arrow, top left.
    pub wind: Option<Wind>,
    /// Field radius from the pad, m (dashed `warn` circle).
    pub field_radius: Option<f64>,
}

impl GroundTrack {
    /// The ground bounds (min east, min north, max east, max north) of the pad and everything drawn, m.
    pub fn extent(&self) -> (f64, f64, f64, f64) {
        let mut b = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let mut take = |e: f64, n: f64| {
            b = (b.0.min(e), b.1.min(n), b.2.max(e), b.3.max(n));
        };
        for t in &self.tracks {
            for p in &t.points {
                take(p.east, p.north);
            }
            for s in &t.scatter {
                take(s.0, s.1);
            }
            if let Some(e) = &t.ellipse {
                for k in 0..32 {
                    let p = e.at(k as f64 / 32.0 * std::f64::consts::TAU);
                    take(p.0, p.1);
                }
            }
        }
        if b.2 - b.0 < 1.0 && b.3 - b.1 < 1.0 {
            let r = self.field_radius.unwrap_or(150.0);
            b = (-r, -r, r, r);
        }
        b
    }

    /// The nearest track within `reach` px of `at`, as (track index, interpolated sample).
    fn nearest(&self, xf: &Xf, at: Point, reach: f32) -> Option<(usize, TrackPoint)> {
        let mut best: Option<(f32, usize, TrackPoint)> = None;
        for (ti, track) in self.tracks.iter().enumerate() {
            for w in track.points.windows(2) {
                let (a, b) = (xf.to_screen(w[0].east, w[0].north), xf.to_screen(w[1].east, w[1].north));
                let ab = b - a;
                let len2 = ab.x * ab.x + ab.y * ab.y;
                let k = if len2 > 0.0 { (((at.x - a.x) * ab.x + (at.y - a.y) * ab.y) / len2).clamp(0.0, 1.0) } else { 0.0 };
                let d = at.distance(Point::new(a.x + ab.x * k, a.y + ab.y * k));
                if d <= reach && best.as_ref().is_none_or(|(bd, ..)| d < *bd) {
                    let t = w[0].t + (w[1].t - w[0].t) * f64::from(k);
                    best = Some((d, ti, lerp(&w[0], &w[1], t)));
                }
            }
        }
        best.map(|(_, i, p)| (i, p))
    }
}

/// What a ground track tells the app.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GroundMessage {
    /// The cursor is over a track at this flight time (move the time plots' crosshair), or left it.
    Hover(Option<f64>),
    /// A landing marker was clicked: select that flight (index into [`GroundTrack::tracks`]).
    Select(usize),
}

/// The ground track canvas.
pub struct GroundTrackView<'a> {
    data: &'a GroundTrack,
    hover_time: Option<f64>,
}

impl<'a> GroundTrackView<'a> {
    /// A view of `data`, with a dot on each track at `hover_time` (the time plots' crosshair), if any.
    pub fn new(data: &'a GroundTrack, hover_time: Option<f64>) -> Self {
        GroundTrackView { data, hover_time }
    }

    /// The canvas filling its space; `.map` it into the app's message.
    pub fn element(self) -> Element<'a, GroundMessage> {
        canvas::Canvas::new(self).width(Fill).height(Fill).boxed()
    }
}

/// Pan, zoom and gesture state, local to the canvas (independent of the time-axis zoom stack).
pub struct View {
    zoom: f32,
    pan: Vector,
    drag: Option<(Point, Vector)>,
    last_click: Option<(Instant, Point)>,
    over_track: bool,
}

impl Default for View {
    fn default() -> Self {
        View { zoom: 1.0, pan: Vector::new(0.0, 0.0), drag: None, last_click: None, over_track: false }
    }
}

/// Metres ↔ pixels for the current size, pan and zoom.
struct Xf {
    /// Screen position of the world centre.
    origin: Point,
    /// World centre (east, north).
    centre: (f64, f64),
    /// Pixels per metre.
    scale: f32,
}

impl Xf {
    /// The fit that shows the whole extent with a margin, then the view's zoom and pan on top.
    fn new(data: &GroundTrack, size: Size, view: &View) -> Xf {
        let (e0, n0, e1, n1) = data.extent();
        let (w, h) = (((e1 - e0) as f32).max(40.0), ((n1 - n0) as f32).max(40.0));
        let fit = (size.width / w).min(size.height / h) * 0.72;
        Xf {
            origin: Point::new(size.width / 2.0 + view.pan.x, size.height / 2.0 + view.pan.y),
            centre: ((e0 + e1) / 2.0, (n0 + n1) / 2.0),
            scale: fit * view.zoom,
        }
    }

    fn to_screen(&self, east: f64, north: f64) -> Point {
        Point::new(self.origin.x + (east - self.centre.0) as f32 * self.scale, self.origin.y - (north - self.centre.1) as f32 * self.scale)
    }

    fn pad(&self) -> Point {
        self.to_screen(0.0, 0.0)
    }
}

impl canvas::Program<GroundMessage> for GroundTrackView<'_> {
    type State = View;

    fn update(&self, st: &mut View, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<GroundMessage>> {
        let local = cursor.position().map(|p| Point::new(p.x - bounds.x, p.y - bounds.y));
        let xf = Xf::new(self.data, bounds.size(), st);
        match event {
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let p = cursor.position_in(bounds)?;
                for (i, track) in self.data.tracks.iter().enumerate() {
                    if let Some(l) = track.landing() {
                        if xf.to_screen(l.east, l.north).distance(p) <= LANDING_HIT_PX {
                            return Some(Action::publish(GroundMessage::Select(i)).and_capture());
                        }
                    }
                }
                let now = Instant::now();
                if st.last_click.is_some_and(|(then, at)| now.duration_since(then) < DOUBLE_CLICK && at.distance(p) < 6.0) {
                    // Double-click fits all tracks.
                    st.last_click = None;
                    st.zoom = 1.0;
                    st.pan = Vector::new(0.0, 0.0);
                    return Some(Action::request_redraw().and_capture());
                }
                st.last_click = Some((now, p));
                st.drag = Some((p, st.pan));
                Some(Action::capture())
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => st.drag.take().map(|_| Action::capture()),
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                if let Some((from, pan0)) = st.drag {
                    let p = local?;
                    st.pan = pan0 + (p - from);
                    return Some(Action::request_redraw().and_capture());
                }
                match local.filter(|p| bounds.contains(Point::new(p.x + bounds.x, p.y + bounds.y))).and_then(|p| self.data.nearest(&xf, p, HIT_PX)) {
                    Some((_, point)) => {
                        st.over_track = true;
                        let same = self.hover_time.is_some_and(|t| (t - point.t).abs() < 1e-6);
                        (!same).then(|| Action::publish(GroundMessage::Hover(Some(point.t))))
                    }
                    None => std::mem::take(&mut st.over_track).then(|| Action::publish(GroundMessage::Hover(None))),
                }
            }
            Event::Mouse(mouse::Event::CursorLeft) if st.over_track => {
                st.over_track = false;
                Some(Action::publish(GroundMessage::Hover(None)))
            }
            Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let p = cursor.position_in(bounds)?;
                let lines = match delta {
                    mouse::ScrollDelta::Lines { y, .. } => *y,
                    mouse::ScrollDelta::Pixels { y, .. } => *y / 40.0,
                };
                // Zoom around the cursor: the world point under it stays put.
                let zoom = (st.zoom * 1.15f32.powf(lines)).clamp(0.2, 400.0);
                let k = zoom / st.zoom;
                let centre = Point::new(bounds.width / 2.0, bounds.height / 2.0);
                let under = Vector::new(p.x - centre.x - st.pan.x, p.y - centre.y - st.pan.y);
                st.pan = Vector::new(p.x - centre.x - under.x * k, p.y - centre.y - under.y * k);
                st.zoom = zoom;
                Some(Action::request_redraw().and_capture())
            }
            _ => None,
        }
    }

    fn mouse_interaction(&self, st: &View, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        if st.drag.is_some() {
            return mouse::Interaction::Grabbing;
        }
        let Some(p) = cursor.position_in(bounds) else { return mouse::Interaction::default() };
        let xf = Xf::new(self.data, bounds.size(), st);
        let on_landing = self.data.tracks.iter().filter_map(|t| t.landing()).any(|l| xf.to_screen(l.east, l.north).distance(p) <= LANDING_HIT_PX);
        if on_landing {
            mouse::Interaction::Pointer
        } else if self.data.nearest(&xf, p, HIT_PX).is_some() {
            mouse::Interaction::Crosshair
        } else {
            mouse::Interaction::Grab
        }
    }

    fn draw(&self, st: &View, renderer: &Renderer, theme: &Theme, bounds: Rectangle, cursor: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut f = Frame::new(renderer, size);
        f.fill_rectangle(Point::ORIGIN, size, t.bg_200);
        let xf = Xf::new(self.data, size, st);
        let pad = xf.pad();
        let colour = |c: SeriesColor| c.resolve(t);

        // Axes through the pad and range rings, labelled on the south side.
        let faint = |f: &mut Frame, a: Point, b: Point| f.stroke(&Path::line(a, b), Stroke::default().with_color(t.line_100).with_width(1.0));
        faint(&mut f, Point::new(0.0, pad.y.floor() + 0.5), Point::new(size.width, pad.y.floor() + 0.5));
        faint(&mut f, Point::new(pad.x.floor() + 0.5, 0.0), Point::new(pad.x.floor() + 0.5, size.height));
        let spacing = ring_spacing(xf.scale, RING_MIN_PX);
        let farthest = [(0.0, 0.0), (size.width, 0.0), (0.0, size.height), (size.width, size.height)]
            .iter()
            .map(|c| pad.distance(Point::new(c.0, c.1)))
            .fold(0.0f32, f32::max);
        let rings = ((f64::from(farthest / xf.scale) / spacing).ceil() as usize).min(60);
        for k in 1..=rings {
            let r = (spacing * k as f64) as f32 * xf.scale;
            f.stroke(&Path::circle(pad, r), Stroke::default().with_color(t.line_100).with_width(1.0));
            let at = Point::new(pad.x, pad.y + r + 3.0);
            if at.y < size.height - 12.0 && at.y > 0.0 {
                put(&mut f, &format!("{} m", fmt_m(spacing * k as f64)), at, t.ink_faint, 10.0, Alignment::Center, alignment::Vertical::Top);
            }
        }

        // Field radius.
        if let Some(radius) = self.data.field_radius {
            let r = radius as f32 * xf.scale;
            f.stroke(
                &Path::circle(pad, r),
                Stroke { line_dash: LineDash { segments: &[6.0, 4.0], offset: 0 }, ..Stroke::default().with_color(t.warn).with_width(1.0) },
            );
            let label = format!("FIELD {} m", fmt_m(radius));
            let w = text_w(&label, 10.0);
            let anchors = [-45.0f32, 45.0, -135.0, 135.0, -90.0, 90.0, 0.0, 180.0];
            let spot = anchors
                .iter()
                .map(|a| a.to_radians())
                .map(|a| Point::new(pad.x + r * a.cos(), pad.y + r * a.sin()))
                .find(|p| p.x > 8.0 && p.x + w + 8.0 < size.width && p.y > 20.0 && p.y < size.height - 20.0 && !(p.x < 160.0 && p.y < 70.0));
            if let Some(p) = spot {
                put(&mut f, &label, Point::new(p.x + 5.0, p.y - 4.0), t.warn, 10.0, Alignment::Left, alignment::Vertical::Bottom);
            }
        }

        // Dispersion: dots and the 95 % ellipse.
        for track in &self.data.tracks {
            let c = colour(track.color);
            for s in &track.scatter {
                f.fill(&Path::circle(xf.to_screen(s.0, s.1), 1.8), Color { a: 0.45, ..c });
            }
            if let Some(e) = &track.ellipse {
                let path = Path::new(|b| {
                    for k in 0..=64 {
                        let p = e.at(k as f64 / 64.0 * std::f64::consts::TAU);
                        let at = xf.to_screen(p.0, p.1);
                        if k == 0 {
                            b.move_to(at);
                        } else {
                            b.line_to(at);
                        }
                    }
                });
                f.stroke(&path, Stroke { line_dash: LineDash { segments: &[5.0, 3.0], offset: 0 }, ..Stroke::default().with_color(c).with_width(1.0) });
            }
        }

        // Pad.
        for (a, b) in [((-6.0, 0.0), (6.0, 0.0)), ((0.0, -6.0), (0.0, 6.0))] {
            f.stroke(
                &Path::line(Point::new(pad.x + a.0, pad.y + a.1), Point::new(pad.x + b.0, pad.y + b.1)),
                Stroke::default().with_color(t.ink).with_width(1.5),
            );
        }
        put(&mut f, "PAD", Point::new(pad.x - 3.0, pad.y - 7.0), t.ink, 10.0, Alignment::Right, alignment::Vertical::Bottom);

        // Tracks, thinned to about one vertex per 2 px.
        for track in &self.data.tracks {
            let c = colour(track.color);
            let mut last: Option<Point> = None;
            let path = Path::new(|b| {
                for (i, p) in track.points.iter().enumerate() {
                    let at = xf.to_screen(p.east, p.north);
                    let end = i + 1 == track.points.len();
                    if last.is_some_and(|l| l.distance(at) < 2.0) && !end {
                        continue;
                    }
                    if last.is_none() {
                        b.move_to(at);
                    } else {
                        b.line_to(at);
                    }
                    last = Some(at);
                }
            });
            f.stroke(&path, Stroke::default().with_color(c).with_width(2.0).with_line_cap(LineCap::Round));
        }
        // Landing labels go on the first side of their marker that touches no track, no other label and not "PAD".
        let label_h = TEXT_LABEL + 2.0;
        let pad_w = text_w("PAD", 10.0);
        let mut taken = vec![
            Rectangle::new(Point::new(pad.x - 3.0 - pad_w - 2.0, pad.y - 7.0 - 12.0), Size::new(pad_w + 4.0, 14.0)),
            Rectangle::new(Point::new(pad.x - 7.0, pad.y - 7.0), Size::new(14.0, 14.0)),
        ];
        let mut samples: Vec<Point> = Vec::new();
        for track in &self.data.tracks {
            let screen: Vec<Point> = track.points.iter().map(|p| xf.to_screen(p.east, p.north)).collect();
            for pair in screen.windows(2) {
                let n = (pair[0].distance(pair[1]) / 2.0).ceil().max(1.0) as usize;
                samples.extend((0..n).map(|k| {
                    let u = k as f32 / n as f32;
                    Point::new(pair[0].x + (pair[1].x - pair[0].x) * u, pair[0].y + (pair[1].y - pair[0].y) * u)
                }));
            }
            samples.extend(screen.last().copied());
        }
        let mut placed: Vec<(String, Rectangle)> = Vec::new();
        for track in &self.data.tracks {
            let Some(l) = track.landing() else { continue };
            let at = xf.to_screen(l.east, l.north);
            let label = landing_label(l.east, l.north);
            let w = text_w(&label, TEXT_LABEL);
            let corners = [
                (at.x + 9.0, at.y - 9.0 - label_h),
                (at.x - 9.0 - w, at.y - 9.0 - label_h),
                (at.x + 9.0, at.y + 9.0),
                (at.x - 9.0 - w, at.y + 9.0),
                (at.x + 9.0, at.y - label_h / 2.0),
                (at.x - 9.0 - w, at.y - label_h / 2.0),
                (at.x - w / 2.0, at.y + 9.0),
                (at.x - w / 2.0, at.y - 9.0 - label_h),
                (at.x - 9.0 - w, at.y - 9.0 - label_h * 2.0 - 2.0),
                (at.x - w / 2.0, at.y - 9.0 - label_h * 2.0 - 2.0),
                (at.x - 9.0 - w, at.y + 9.0 + label_h + 2.0),
            ];
            let fits = |r: &Rectangle| {
                let grown = Rectangle::new(Point::new(r.x - 2.0, r.y - 2.0), Size::new(r.width + 4.0, r.height + 4.0));
                r.x >= 0.0
                    && r.y >= 0.0
                    && r.x + r.width <= size.width
                    && r.y + r.height <= size.height
                    && !samples.iter().any(|p| grown.contains(*p))
                    && !taken.iter().any(|o| o.intersects(&grown))
            };
            let rects: Vec<Rectangle> =
                corners.iter().map(|&(x, y)| Rectangle::new(Point::new(x.min(size.width - w - 2.0).max(2.0), y), Size::new(w, label_h))).collect();
            let rect = rects.iter().find(|r| fits(r)).copied().unwrap_or(rects[0]);
            taken.push(rect);
            placed.push((label, rect));
        }
        for (label, rect) in &placed {
            put(&mut f, label, Point::new(rect.x, rect.y + 1.0), t.ink_muted, TEXT_LABEL, Alignment::Left, alignment::Vertical::Top);
        }
        for track in &self.data.tracks {
            let c = colour(track.color);
            // Apogee: a hollow ring with a 2 px surface ring around it.
            if let Some(a) = track.apogee.and_then(|i| track.points.get(i)) {
                let at = xf.to_screen(a.east, a.north);
                let ring = if track.color == SeriesColor::Ink { t.accent } else { c };
                f.fill(&Path::circle(at, 6.0), t.bg_200);
                f.fill(&Path::circle(at, 4.0), ring);
                f.fill(&Path::circle(at, 2.0), t.bg_200);
            }
            // Landing: a filled square with a surface ring, and the label.
            if let Some(l) = track.landing() {
                let at = xf.to_screen(l.east, l.north);
                f.fill_rectangle(Point::new(at.x - 5.5, at.y - 5.5), Size::new(11.0, 11.0), t.bg_200);
                f.fill_rectangle(Point::new(at.x - 3.5, at.y - 3.5), Size::new(7.0, 7.0), c);
            }
        }

        // The time plots' crosshair: a dot on each track at that moment.
        if let Some(time) = self.hover_time {
            for track in &self.data.tracks {
                if let Some(p) = track.at_time(time) {
                    let at = xf.to_screen(p.east, p.north);
                    f.fill(&Path::circle(at, 5.5), t.bg_200);
                    f.fill(&Path::circle(at, 3.5), colour(track.color));
                }
            }
        }

        corner_marks(&mut f, t, size, self.data.wind, xf.scale);

        // Hovering a track: a tooltip with time, altitude and distance from the pad.
        if let Some(p) = cursor.position_in(bounds) {
            if st.drag.is_none() {
                if let Some((_, point)) = self.data.nearest(&xf, p, HIT_PX) {
                    let text = format!("{:.2} s · {:.0} m up · {:.0} m out", point.t, point.alt, point.range());
                    let at = xf.to_screen(point.east, point.north);
                    tooltip(&mut f, t, size, at, &text);
                }
            }
        }
        vec![f.into_geometry()]
    }
}

fn text_w(s: &str, size: f32) -> f32 {
    s.chars().count() as f32 * size * GLYPH_EM
}

/// Metres without trailing zeros: `100`, `2500`, `0.5`.
fn fmt_m(m: f64) -> String {
    if (m - m.round()).abs() < 1e-9 {
        format!("{m:.0}")
    } else {
        format!("{m}")
    }
}

fn put(f: &mut Frame, content: &str, at: Point, color: Color, size: f32, ax: Alignment, ay: alignment::Vertical) {
    f.fill_text(Text { content: content.to_string(), position: at, color, size: Pixels(size), font: MONO, align_x: ax, align_y: ay, ..Text::default() });
}

fn tooltip(f: &mut Frame, t: &Tokens, size: Size, at: Point, text: &str) {
    let w = text_w(text, TEXT_LABEL) + 12.0;
    let h = 20.0;
    let x = if at.x + 12.0 + w > size.width { at.x - 12.0 - w } else { at.x + 12.0 };
    let y = (at.y - h - 8.0).max(2.0);
    let origin = Point::new(x.round().max(2.0), y.round());
    f.fill_rectangle(origin, Size::new(w, h), t.bg_200);
    f.stroke_rectangle(Point::new(origin.x + 0.5, origin.y + 0.5), Size::new(w - 1.0, h - 1.0), Stroke::default().with_color(t.line_200).with_width(1.0));
    put(f, text, Point::new(origin.x + 6.0, origin.y + h / 2.0), t.ink, TEXT_LABEL, Alignment::Left, alignment::Vertical::Center);
}

/// Wind readout (top left), north arrow (top right) and scale bar (bottom right).
fn corner_marks(f: &mut Frame, t: &Tokens, size: Size, wind: Option<Wind>, px_per_m: f32) {
    if let Some(w) = wind {
        put(f, "WIND", Point::new(10.0, 8.0), t.ink_faint, 10.0, Alignment::Left, alignment::Vertical::Top);
        put(
            f,
            &format!("{:.1} m/s · {:03.0}°", w.speed, w.from_deg.rem_euclid(360.0)),
            Point::new(10.0, 21.0),
            t.ink,
            TEXT_LABEL,
            Alignment::Left,
            alignment::Vertical::Top,
        );
        // An arrow pointing the way the wind blows.
        let (e, n) = w.toward();
        let (dx, dy) = (e as f32, -(n as f32));
        let mid = Point::new(28.0, 50.0);
        let (tail, head) = (Point::new(mid.x - dx * 14.0, mid.y - dy * 14.0), Point::new(mid.x + dx * 14.0, mid.y + dy * 14.0));
        let stroke = || Stroke::default().with_color(t.ink_muted).with_width(1.5);
        f.stroke(&Path::line(tail, head), stroke());
        for side in [-1.0f32, 1.0] {
            let (c, s) = (0.866f32, 0.5f32 * side);
            let (bx, by) = (-dx * c + dy * s, -dx * s - dy * c);
            f.stroke(&Path::line(head, Point::new(head.x + bx * 6.0, head.y + by * 6.0)), stroke());
        }
    }
    // North arrow.
    let n = Point::new(size.width - 18.0, 26.0);
    f.stroke(&Path::line(Point::new(n.x, n.y + 10.0), Point::new(n.x, n.y - 10.0)), Stroke::default().with_color(t.ink_muted).with_width(1.5));
    f.fill(
        &Path::new(|b| {
            b.move_to(Point::new(n.x, n.y - 12.0));
            b.line_to(Point::new(n.x - 4.0, n.y - 4.0));
            b.line_to(Point::new(n.x + 4.0, n.y - 4.0));
            b.close();
        }),
        t.ink_muted,
    );
    put(f, "N", Point::new(n.x, n.y + 13.0), t.ink_faint, 10.0, Alignment::Center, alignment::Vertical::Top);

    // Scale bar: the longest 1-2-5 length that fits in 50–120 px.
    let metres = ring_spacing(px_per_m, 50.0);
    let px = metres as f32 * px_per_m;
    if px <= 120.0 * 1.25 {
        let (x1, y) = (size.width - 12.0, size.height - 14.0);
        let x0 = x1 - px;
        let s = || Stroke::default().with_color(t.ink_muted).with_width(1.0);
        f.stroke(&Path::line(Point::new(x0, y), Point::new(x1, y)), s());
        for x in [x0, x1] {
            f.stroke(&Path::line(Point::new(x, y - 3.0), Point::new(x, y + 3.0)), s());
        }
        put(f, &format!("{} m", fmt_m(metres)), Point::new(x1, y - 6.0), t.ink_faint, 10.0, Alignment::Right, alignment::Vertical::Bottom);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(t: f64, e: f64, n: f64, alt: f64) -> TrackPoint {
        TrackPoint { t, east: e, north: n, alt }
    }

    fn track() -> Track {
        Track::new(
            "Nominal",
            SeriesColor::Series(1),
            vec![pt(0.0, 0.0, 0.0, 0.0), pt(10.0, -20.0, -8.0, 600.0), pt(20.0, 40.0, 16.0, 300.0), pt(30.0, 96.0, 38.0, 0.0)],
        )
    }

    #[test]
    fn bearings_are_compass_bearings() {
        assert_eq!(bearing_deg(0.0, 100.0), 0.0);
        assert_eq!(bearing_deg(100.0, 0.0), 90.0);
        assert_eq!(bearing_deg(0.0, -100.0), 180.0);
        assert_eq!(bearing_deg(-100.0, 0.0), 270.0);
        // 104 m at 068° (SimulateScreen): east = 104 sin 68°, north = 104 cos 68°.
        let (e, n) = (104.0 * 68f64.to_radians().sin(), 104.0 * 68f64.to_radians().cos());
        assert_eq!(landing_label(e, n), "104 m · 068°");
    }

    #[test]
    fn landing_label_wraps_to_three_digits() {
        assert_eq!(landing_label(0.0, 50.0), "50 m · 000°");
        assert_eq!(landing_label(-0.001, 50.0), "50 m · 000°");
    }

    #[test]
    fn wind_blows_toward_the_opposite_of_where_it_comes_from() {
        let w = Wind { speed: 2.2, from_deg: 250.0 };
        let (e, n) = w.toward();
        assert!((bearing_deg(e, n) - 70.0).abs() < 1e-9);
        let (e, n) = Wind { speed: 1.0, from_deg: 0.0 }.toward();
        assert!(e.abs() < 1e-9 && (n + 1.0).abs() < 1e-9, "a north wind blows south");
    }

    #[test]
    fn rings_switch_spacing_with_the_scale() {
        // 0.6 px/m: 100 m is 60 px. Zoomed in to 1.2 px/m: 50 m is 60 px. Zoomed out to 0.12: 500 m.
        assert_eq!(ring_spacing(0.6, 60.0), 100.0);
        assert_eq!(ring_spacing(1.2, 60.0), 50.0);
        assert_eq!(ring_spacing(0.12, 60.0), 500.0);
        assert_eq!(ring_spacing(0.0, 60.0), 100.0);
    }

    #[test]
    fn interpolates_the_position_at_a_time() {
        let t = track();
        let p = t.at_time(15.0).unwrap();
        assert_eq!((p.east, p.north, p.alt), (10.0, 4.0, 450.0));
        assert_eq!(t.at_time(0.0).unwrap().east, 0.0);
        assert_eq!(t.at_time(30.0).unwrap().east, 96.0);
        assert!(t.at_time(-1.0).is_none() && t.at_time(31.0).is_none());
    }

    #[test]
    fn apogee_is_the_highest_sample() {
        assert_eq!(track().apogee, Some(1));
        assert_eq!(track().landing().unwrap().east, 96.0);
    }

    #[test]
    fn the_95_percent_ellipse_of_axis_aligned_scatter() {
        // A symmetric cross of samples: variance 2 along x, 0.5 along y (n - 1 = 5).
        let pts = [(2.0, 0.0), (-2.0, 0.0), (0.0, 1.0), (0.0, -1.0), (0.0, 0.0), (0.0, 0.0)];
        let e = confidence_ellipse(&pts).unwrap();
        let (vx, vy) = (8.0 / 5.0, 2.0 / 5.0);
        assert!((e.semi_major - (CHI2_95_2DOF * vx).sqrt()).abs() < 1e-9, "{e:?}");
        assert!((e.semi_minor - (CHI2_95_2DOF * vy).sqrt()).abs() < 1e-9);
        assert!(e.angle.abs() < 1e-9);
        assert!(e.centre.0.abs() < 1e-12 && e.centre.1.abs() < 1e-12);
    }

    #[test]
    fn the_ellipse_follows_a_rotated_scatter() {
        // Points on y = x: the major axis is at 45°.
        let pts: Vec<_> = (-5..=5).map(|i| (f64::from(i), f64::from(i))).collect();
        let e = confidence_ellipse(&pts).unwrap();
        assert!((e.angle - std::f64::consts::FRAC_PI_4).abs() < 1e-9);
        assert!(e.semi_minor < 1e-6);
        // Its parametric points lie on the line.
        let p = e.at(0.0);
        assert!((p.0 - p.1).abs() < 1e-9);
        assert!(confidence_ellipse(&pts[..2]).is_none());
    }

    #[test]
    fn the_95_percent_chi_square_quantile_covers_95_percent_of_a_ring_of_normals() {
        // For N(0, I) in 2-D, P(r² ≤ q) = 1 − exp(−q/2); at q = 5.991 that is 0.95.
        assert!((1.0 - (-CHI2_95_2DOF / 2.0).exp() - 0.95).abs() < 1e-4);
    }

    #[test]
    fn extent_covers_pad_and_tracks_and_has_a_default() {
        let g = GroundTrack { tracks: vec![track()], wind: None, field_radius: Some(300.0) };
        assert_eq!(g.extent(), (-20.0, -8.0, 96.0, 38.0));
        let empty = GroundTrack { field_radius: Some(300.0), ..GroundTrack::default() };
        assert_eq!(empty.extent(), (-300.0, -300.0, 300.0, 300.0));
    }

    #[test]
    fn north_is_up_and_scale_is_equal() {
        let g = GroundTrack { tracks: vec![track()], wind: None, field_radius: None };
        let xf = Xf::new(&g, Size::new(400.0, 300.0), &View::default());
        let (a, b) = (xf.to_screen(0.0, 0.0), xf.to_screen(0.0, 100.0));
        assert!(b.y < a.y && (b.x - a.x).abs() < 1e-4, "north is up the screen");
        let c = xf.to_screen(100.0, 0.0);
        assert!(((c.x - a.x) - (a.y - b.y)).abs() < 1e-4, "equal scale in both axes");
    }

    #[test]
    fn nearest_finds_the_track_under_the_cursor_with_its_time() {
        let g = GroundTrack { tracks: vec![track()], wind: None, field_radius: None };
        let xf = Xf::new(&g, Size::new(400.0, 300.0), &View::default());
        let at = xf.to_screen(10.0, 4.0);
        let (i, p) = g.nearest(&xf, Point::new(at.x + 2.0, at.y + 1.0), HIT_PX).unwrap();
        assert_eq!(i, 0);
        assert!((p.t - 15.0).abs() < 0.5, "{p:?}");
        assert!(g.nearest(&xf, Point::new(at.x, at.y - 80.0), HIT_PX).is_none());
    }
}
