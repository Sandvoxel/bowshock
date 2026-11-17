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

use crate::units::{Quantity, Unit, UnitSystem};

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

/// `337 m · 067°`: distance (in `dist`, at its decimals) and bearing from the pad.
pub fn landing_label(east: f64, north: f64, dist: &Unit) -> String {
    format!("{:.*} {} · {:03.0}°", dist.decimals, dist.from_si(east.hypot(north)), dist.symbol, bearing_deg(east, north).rem_euclid(360.0).round() % 360.0)
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
    pub fn new(color: SeriesColor, points: Vec<TrackPoint>) -> Self {
        let apogee = points.iter().enumerate().max_by(|a, b| a.1.alt.total_cmp(&b.1.alt)).map(|(i, _)| i);
        Track { color, points, apogee, scatter: Vec::new(), ellipse: None }
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
    /// The display unit set: distances (rings, labels, tooltip) are in its altitude unit, the wind speed in its speed unit.
    pub units: UnitSystem,
}

impl GroundTrack {
    /// The unit distances are written in.
    fn dist(&self) -> Unit {
        *Quantity::Altitude.unit_for(self.units)
    }

    /// The unit the wind speed is written in.
    fn speed(&self) -> Unit {
        *Quantity::Velocity.unit_for(self.units)
    }

    /// `FIELD 300 m`, the key of the field circle.
    fn field_label(&self) -> Option<String> {
        let dist = self.dist();
        self.field_radius.map(|r| format!("FIELD {} {}", fmt_m(dist.from_si(r)), dist.symbol))
    }

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
            Event::Mouse(mouse::Event::WheelScrolled { .. } | mouse::Event::PinchGesture { .. }) => {
                let p = cursor.position_in(bounds)?;
                // The wheel steps by notch (or by pixels of trackpad scroll); a pinch zooms by the gesture's own amount.
                let step = match event {
                    Event::Mouse(mouse::Event::PinchGesture { delta, .. }) => super::pinch::factor(*delta),
                    Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                        let lines = match delta {
                            mouse::ScrollDelta::Lines { y, .. } => *y,
                            mouse::ScrollDelta::Pixels { y, .. } => *y / 40.0,
                        };
                        1.15f32.powf(lines)
                    }
                    _ => return None,
                };
                // Zoom around the cursor: the world point under it stays put.
                let zoom = (st.zoom * step).clamp(0.2, 400.0);
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

        // Axes through the pad and range rings, labelled along the clearest cardinal spoke (south first).
        let faint = |f: &mut Frame, a: Point, b: Point| f.stroke(&Path::line(a, b), Stroke::default().with_color(t.line_100).with_width(1.0));
        faint(&mut f, Point::new(0.0, pad.y.floor() + 0.5), Point::new(size.width, pad.y.floor() + 0.5));
        faint(&mut f, Point::new(pad.x.floor() + 0.5, 0.0), Point::new(pad.x.floor() + 0.5, size.height));
        let dist = self.data.dist();
        let spacing = ring_spacing(xf.scale * dist.to_si as f32, RING_MIN_PX);
        let farthest = [(0.0, 0.0), (size.width, 0.0), (0.0, size.height), (size.width, size.height)]
            .iter()
            .map(|c| pad.distance(Point::new(c.0, c.1)))
            .fold(0.0f32, f32::max);
        let rings = ((f64::from(farthest / xf.scale) / (spacing * dist.to_si)).ceil() as usize).min(60);
        let mut ring_items: Vec<(f32, String)> = Vec::with_capacity(rings);
        for k in 1..=rings {
            let r = (spacing * dist.to_si * k as f64) as f32 * xf.scale;
            f.stroke(&Path::circle(pad, r), Stroke::default().with_color(t.line_100).with_width(1.0));
            ring_items.push((r, format!("{} {}", fmt_m(spacing * k as f64), dist.symbol)));
        }

        // Field radius: the dashed circle; its name is in the bottom-left corner (`corner_marks`), clear of everything else.
        if let Some(radius) = self.data.field_radius {
            let r = radius as f32 * xf.scale;
            f.stroke(
                &Path::circle(pad, r),
                Stroke { line_dash: LineDash { segments: &[6.0, 4.0], offset: 0 }, ..Stroke::default().with_color(t.warn).with_width(1.0) },
            );
        }

        let label_h = TEXT_LABEL + 2.0;
        let pad_w = text_w("PAD", 10.0);
        let mut reserved = corner_rects(size, self.data.wind, self.data.field_label().as_deref(), self.data.speed().symbol);
        reserved.push(Rectangle::new(Point::new(pad.x - 3.0 - pad_w - 2.0, pad.y - 7.0 - 12.0), Size::new(pad_w + 4.0, 14.0)));
        reserved.push(Rectangle::new(Point::new(pad.x - 7.0, pad.y - 7.0), Size::new(14.0, 14.0)));
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
        let mut ring_obstacles = samples.clone();
        for track in &self.data.tracks {
            if let Some(l) = track.landing() {
                ring_obstacles.push(xf.to_screen(l.east, l.north));
            }
            ring_obstacles.extend(track.scatter.iter().map(|&(e, n)| xf.to_screen(e, n)));
            if let Some(e) = &track.ellipse {
                ring_obstacles.extend((0..32).map(|k| {
                    let p = e.at(k as f64 / 32.0 * std::f64::consts::TAU);
                    xf.to_screen(p.0, p.1)
                }));
            }
        }
        let label_bounds = Size::new((size.width - 10.0).max(0.0), size.height);
        let (_, ring_labels) = place_ring_labels(label_bounds, pad, &ring_items, &ring_obstacles, &reserved);
        for (rect, label) in &ring_labels {
            f.fill_rectangle(rect.position(), rect.size(), Color { a: 0.85, ..t.bg_200 });
            put(&mut f, label, Point::new(rect.x + 2.0, rect.y + 1.0), t.ink_faint, 10.0, Alignment::Left, alignment::Vertical::Top);
            reserved.push(*rect);
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
        // Landing labels: each on the first free spot around its marker (see `place_landing_labels`), joined to the marker by a
        // leader line when it had to move away.
        let landings: Vec<(Point, String, Color)> = self
            .data
            .tracks
            .iter()
            .filter_map(|track| {
                let l = track.landing()?;
                Some((xf.to_screen(l.east, l.north), landing_label(l.east, l.north, &self.data.dist()), colour(track.color)))
            })
            .collect();
        let items: Vec<(Point, f32)> = landings.iter().map(|(at, label, _)| (*at, text_w(label, TEXT_LABEL))).collect();
        // A margin at the right edge, where the results column's scrollbar overlays the map.
        let rects = place_landing_labels(label_bounds, &items, label_h, &samples, &reserved);
        for ((at, label, c), rect) in landings.iter().zip(&rects) {
            let near = Point::new(at.x.clamp(rect.x, rect.x + rect.width), at.y.clamp(rect.y, rect.y + rect.height));
            if near.distance(*at) > 14.0 {
                f.stroke(&Path::line(*at, near), Stroke::default().with_color(Color { a: 0.7, ..*c }).with_width(1.0));
            }
            // On a backing, so a ring or the field circle never strikes through the text.
            f.fill_rectangle(rect.position(), rect.size(), Color { a: 0.85, ..t.bg_200 });
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

        let field = self.data.field_label();
        corner_marks(&mut f, t, size, self.data.wind, field.as_deref(), xf.scale, &self.data.dist(), &self.data.speed());

        // Hovering a track: a tooltip with time, altitude and distance from the pad.
        if let Some(p) = cursor.position_in(bounds) {
            if st.drag.is_none() {
                if let Some((_, point)) = self.data.nearest(&xf, p, HIT_PX) {
                    let (dist, decimals) = (self.data.dist(), self.data.dist().decimals);
                    let text = format!(
                        "{:.2} s · {:.*} {sym} up · {:.*} {sym} out",
                        point.t,
                        decimals,
                        dist.from_si(point.alt),
                        decimals,
                        dist.from_si(point.range()),
                        sym = dist.symbol
                    );
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

/// The corners the marks of `corner_marks` own: the wind block (top left), the north arrow (top right), the scale bar (bottom
/// right) and the field-radius key (bottom left). Landing labels keep out of them.
fn corner_rects(size: Size, wind: Option<Wind>, field: Option<&str>, speed_symbol: &str) -> Vec<Rectangle> {
    let mut out = vec![
        Rectangle::new(Point::new(size.width - 36.0, 0.0), Size::new(36.0, 52.0)),
        Rectangle::new(Point::new(size.width - 140.0, size.height - 30.0), Size::new(140.0, 30.0)),
    ];
    if wind.is_some() {
        let w = text_w(&format!("0.0 {speed_symbol} · 000°"), TEXT_LABEL).max(60.0) + 20.0;
        out.push(Rectangle::new(Point::ORIGIN, Size::new(w, 70.0)));
    }
    if let Some(label) = field {
        out.push(Rectangle::new(Point::new(0.0, size.height - 24.0), Size::new(text_w(label, 10.0) + 38.0, 24.0)));
    }
    out
}

/// Places a label `(marker, width)` per landing, `height` tall, as rectangles inside `size`: on the first of the positions around
/// its marker that touches no track `samples`, no `reserved` rectangle and no earlier label; failing that, in a column of rows
/// beside the marker where only the reserved rectangles and the other labels are avoided.
fn place_landing_labels(size: Size, items: &[(Point, f32)], height: f32, samples: &[Point], reserved: &[Rectangle]) -> Vec<Rectangle> {
    let mut taken: Vec<Rectangle> = reserved.to_vec();
    // Every landing marker is an obstacle too, so no label covers another flight's square.
    taken.extend(items.iter().map(|(at, _)| Rectangle::new(Point::new(at.x - 6.0, at.y - 6.0), Size::new(12.0, 12.0))));
    let mut out: Vec<Rectangle> = Vec::new();
    for &(at, w) in items {
        let clamp =
            |x: f32, y: f32| Rectangle::new(Point::new(x.min(size.width - w - 2.0).max(2.0), y.min(size.height - height - 2.0).max(2.0)), Size::new(w, height));
        let near = [
            (at.x + 12.0, at.y - 12.0 - height),
            (at.x - 12.0 - w, at.y - 12.0 - height),
            (at.x + 12.0, at.y + 12.0),
            (at.x - 12.0 - w, at.y + 12.0),
            (at.x + 12.0, at.y - height / 2.0),
            (at.x - 12.0 - w, at.y - height / 2.0),
            (at.x - w / 2.0, at.y + 12.0),
            (at.x - w / 2.0, at.y - 12.0 - height),
            (at.x - 12.0 - w, at.y - 12.0 - height * 2.0 - 6.0),
            (at.x - w / 2.0, at.y - 12.0 - height * 2.0 - 6.0),
            (at.x - 12.0 - w, at.y + 12.0 + height + 6.0),
        ];
        let rows = (0..10).flat_map(|k| {
            let dy = k as f32 * (height + 6.0);
            [
                (at.x + 16.0, at.y + 12.0 + dy),
                (at.x - 16.0 - w, at.y + 12.0 + dy),
                (at.x + 16.0, at.y - 12.0 - height - dy),
                (at.x - 16.0 - w, at.y - 12.0 - height - dy),
            ]
        });
        let clear = |r: &Rectangle, avoid_tracks: bool| {
            let grown = Rectangle::new(Point::new(r.x - 5.0, r.y - 5.0), Size::new(r.width + 10.0, r.height + 10.0));
            r.x >= 0.0
                && r.y >= 0.0
                && r.x + r.width <= size.width
                && r.y + r.height <= size.height
                && !(avoid_tracks && samples.iter().any(|p| grown.contains(*p)))
                && !taken.iter().any(|o| o.intersects(&grown))
        };
        let exact = |x: f32, y: f32| Rectangle::new(Point::new(x, y), Size::new(w, height));
        // Positions that fit as they are come first; a clamped position (pushed in from an edge, so over its own marker's side)
        // is the fallback.
        let rect = near
            .iter()
            .map(|&(x, y)| exact(x, y))
            .find(|r| clear(r, true))
            .or_else(|| near.iter().map(|&(x, y)| clamp(x, y)).find(|r| clear(r, true)))
            .or_else(|| near.iter().map(|&(x, y)| exact(x, y)).find(|r| clear(r, false)))
            .or_else(|| rows.clone().map(|(x, y)| exact(x, y)).find(|r| clear(r, false)))
            .or_else(|| rows.map(|(x, y)| clamp(x, y)).find(|r| clear(r, false)))
            .unwrap_or_else(|| clamp(near[0].0, near[0].1));
        taken.push(rect);
        out.push(rect);
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Spoke {
    South,
    East,
    West,
    North,
}

/// Places the range-ring labels `(radius_px, text)` along the clearest cardinal spoke from `pad` (`South` first, matching the
/// design kit, then `East`, `West`, `North` when a track or dispersion cloud runs south).
fn place_ring_labels(size: Size, pad: Point, rings: &[(f32, String)], obstacles: &[Point], reserved: &[Rectangle]) -> (Spoke, Vec<(Rectangle, String)>) {
    const SPOKES: [Spoke; 4] = [Spoke::South, Spoke::East, Spoke::West, Spoke::North];
    let h = 12.0;
    let candidate = |spoke: Spoke| -> Vec<(Rectangle, String)> {
        rings
            .iter()
            .filter_map(|(r, label)| {
                let w = text_w(label, 10.0) + 4.0;
                let origin = match spoke {
                    Spoke::South => Point::new(pad.x + 3.0, pad.y + r + 2.0),
                    Spoke::East => Point::new(pad.x + r + 3.0, pad.y + 3.0),
                    Spoke::West => Point::new(pad.x - r - 3.0 - w, pad.y + 3.0),
                    Spoke::North => Point::new(pad.x + 3.0, pad.y - r - 2.0 - h),
                };
                let rect = Rectangle::new(origin, Size::new(w, h));
                let inside = rect.x >= 2.0 && rect.y >= 2.0 && rect.x + rect.width <= size.width - 2.0 && rect.y + rect.height <= size.height - 2.0;
                (inside && !reserved.iter().any(|c| c.intersects(&rect))).then(|| (rect, label.clone()))
            })
            .collect()
    };
    SPOKES
        .into_iter()
        .enumerate()
        .map(|(order, spoke)| {
            let placed = candidate(spoke);
            let mut hit_labels = 0usize;
            let mut hit_points = 0usize;
            for (rect, _) in &placed {
                let grown = Rectangle::new(Point::new(rect.x - 4.0, rect.y - 4.0), Size::new(rect.width + 8.0, rect.height + 8.0));
                let hits = obstacles.iter().filter(|p| grown.contains(**p)).count();
                if hits > 0 {
                    hit_labels += 1;
                    hit_points += hits;
                }
            }
            ((placed.is_empty(), hit_labels, hit_points, order), (spoke, placed))
        })
        .min_by_key(|(score, _)| *score)
        .map_or((Spoke::South, Vec::new()), |(_, best)| best)
}

/// Wind readout (top left), north arrow (top right), field-radius key (bottom left) and scale bar (bottom right).
fn corner_marks(f: &mut Frame, t: &Tokens, size: Size, wind: Option<Wind>, field: Option<&str>, px_per_m: f32, dist: &Unit, speed: &Unit) {
    // Only the text corners get a backing, so a ring or the field circle never strikes through them.
    for r in corner_rects(size, wind, field, speed.symbol).into_iter().skip(2) {
        f.fill_rectangle(r.position(), r.size(), Color { a: 0.85, ..t.bg_200 });
    }
    if let Some(label) = field {
        let y = size.height - 12.0;
        f.stroke(
            &Path::line(Point::new(10.0, y), Point::new(30.0, y)),
            Stroke { line_dash: LineDash { segments: &[6.0, 4.0], offset: 0 }, ..Stroke::default().with_color(t.warn).with_width(1.0) },
        );
        put(f, label, Point::new(36.0, y), t.warn, 10.0, Alignment::Left, alignment::Vertical::Center);
    }
    if let Some(w) = wind {
        put(f, "WIND", Point::new(10.0, 8.0), t.ink_faint, 10.0, Alignment::Left, alignment::Vertical::Top);
        put(
            f,
            &format!("{:.1} {} · {:03.0}°", speed.from_si(w.speed), speed.symbol, w.from_deg.rem_euclid(360.0)),
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
    let px_per_unit = px_per_m * dist.to_si as f32;
    let spacing = ring_spacing(px_per_unit, 50.0);
    let px = spacing as f32 * px_per_unit;
    if px <= 120.0 * 1.25 {
        let (x1, y) = (size.width - 12.0, size.height - 14.0);
        let x0 = x1 - px;
        let s = || Stroke::default().with_color(t.ink_muted).with_width(1.0);
        f.stroke(&Path::line(Point::new(x0, y), Point::new(x1, y)), s());
        for x in [x0, x1] {
            f.stroke(&Path::line(Point::new(x, y - 3.0), Point::new(x, y + 3.0)), s());
        }
        put(f, &format!("{} {}", fmt_m(spacing), dist.symbol), Point::new(x1, y - 6.0), t.ink_faint, 10.0, Alignment::Right, alignment::Vertical::Bottom);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn landing_labels_never_intersect_each_other_or_the_corner_marks() {
        let size = Size::new(340.0, 300.0);
        let wind = Some(Wind { speed: 2.2, from_deg: 250.0 });
        let reserved = corner_rects(size, wind, Some("FIELD 300 m"), "m/s");
        // Landing points almost on top of each other, one near the north arrow, one near the wind block.
        let items = [
            (Point::new(200.0, 140.0), 90.0),
            (Point::new(204.0, 142.0), 90.0),
            (Point::new(206.0, 138.0), 90.0),
            (Point::new(330.0, 30.0), 90.0),
            (Point::new(20.0, 30.0), 90.0),
        ];
        let rects = place_landing_labels(size, &items, 13.0, &[], &reserved);
        assert_eq!(rects.len(), items.len());
        for (i, a) in rects.iter().enumerate() {
            assert!(a.x >= 0.0 && a.y >= 0.0 && a.x + a.width <= size.width && a.y + a.height <= size.height, "{a:?} leaves the plot");
            for r in &reserved {
                assert!(!a.intersects(r), "label {a:?} on a corner mark {r:?}");
            }
            for b in &rects[i + 1..] {
                assert!(!a.intersects(b), "labels {a:?} and {b:?} intersect");
            }
        }
    }

    #[test]
    fn two_near_identical_landings_at_the_right_edge_get_separate_labels_clear_of_the_north_arrow() {
        // Nominal (190 m) and Windy (195 m) land almost on the same spot, close to the right edge and below the north arrow.
        let size = Size::new(340.0, 232.0);
        let reserved = corner_rects(size, Some(Wind { speed: 2.2, from_deg: 250.0 }), Some("FIELD 300 m"), "m/s");
        let markers = [Point::new(300.0, 100.0), Point::new(303.0, 98.0)];
        let items: Vec<(Point, f32)> = markers.iter().map(|m| (*m, 90.0)).collect();
        let rects = place_landing_labels(Size::new(size.width - 10.0, size.height), &items, 13.0, &[], &reserved);
        assert!(!rects[0].intersects(&rects[1]));
        for r in &rects {
            assert!(reserved.iter().all(|c| !r.intersects(c)), "{r:?} on a corner mark");
            assert!(markers.iter().all(|m| !r.contains(*m)), "{r:?} covers a marker");
        }
    }

    fn pt(t: f64, e: f64, n: f64, alt: f64) -> TrackPoint {
        TrackPoint { t, east: e, north: n, alt }
    }

    fn track() -> Track {
        Track::new(SeriesColor::Series(1), vec![pt(0.0, 0.0, 0.0, 0.0), pt(10.0, -20.0, -8.0, 600.0), pt(20.0, 40.0, 16.0, 300.0), pt(30.0, 96.0, 38.0, 0.0)])
    }

    #[test]
    fn bearings_are_compass_bearings() {
        assert_eq!(bearing_deg(0.0, 100.0), 0.0);
        assert_eq!(bearing_deg(100.0, 0.0), 90.0);
        assert_eq!(bearing_deg(0.0, -100.0), 180.0);
        assert_eq!(bearing_deg(-100.0, 0.0), 270.0);
        // 104 m at 068° (SimulateScreen): east = 104 sin 68°, north = 104 cos 68°.
        let (e, n) = (104.0 * 68f64.to_radians().sin(), 104.0 * 68f64.to_radians().cos());
        assert_eq!(landing_label(e, n, &Quantity::Altitude.units()[0]), "104 m · 068°");
        // In feet (the unit set's distance unit) the same landing reads 341 ft.
        assert_eq!(landing_label(e, n, &Quantity::Altitude.units()[1]), "341 ft · 068°");
    }

    #[test]
    fn landing_label_wraps_to_three_digits() {
        assert_eq!(landing_label(0.0, 50.0, &Quantity::Altitude.units()[0]), "50 m · 000°");
        assert_eq!(landing_label(-0.001, 50.0, &Quantity::Altitude.units()[0]), "50 m · 000°");
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
        let g = GroundTrack { tracks: vec![track()], wind: None, field_radius: Some(300.0), units: UnitSystem::default() };
        assert_eq!(g.extent(), (-20.0, -8.0, 96.0, 38.0));
        let empty = GroundTrack { field_radius: Some(300.0), ..GroundTrack::default() };
        assert_eq!(empty.extent(), (-300.0, -300.0, 300.0, 300.0));
    }

    #[test]
    fn north_is_up_and_scale_is_equal() {
        let g = GroundTrack { tracks: vec![track()], wind: None, field_radius: None, units: UnitSystem::default() };
        let xf = Xf::new(&g, Size::new(400.0, 300.0), &View::default());
        let (a, b) = (xf.to_screen(0.0, 0.0), xf.to_screen(0.0, 100.0));
        assert!(b.y < a.y && (b.x - a.x).abs() < 1e-4, "north is up the screen");
        let c = xf.to_screen(100.0, 0.0);
        assert!(((c.x - a.x) - (a.y - b.y)).abs() < 1e-4, "equal scale in both axes");
    }

    #[test]
    fn nearest_finds_the_track_under_the_cursor_with_its_time() {
        let g = GroundTrack { tracks: vec![track()], wind: None, field_radius: None, units: UnitSystem::default() };
        let xf = Xf::new(&g, Size::new(400.0, 300.0), &View::default());
        let at = xf.to_screen(10.0, 4.0);
        let (i, p) = g.nearest(&xf, Point::new(at.x + 2.0, at.y + 1.0), HIT_PX).unwrap();
        assert_eq!(i, 0);
        assert!((p.t - 15.0).abs() < 0.5, "{p:?}");
        assert!(g.nearest(&xf, Point::new(at.x, at.y - 80.0), HIT_PX).is_none());
    }

    #[test]
    fn ring_labels_default_to_south_and_switch_spokes_when_drift_runs_south() {
        let size = Size::new(680.0, 240.0);
        let pad = Point::new(340.0, 80.0);
        let rings = vec![(60.0, "200 m".to_string()), (120.0, "400 m".to_string()), (180.0, "600 m".to_string())];
        let reserved = corner_rects(size, Some(Wind { speed: 3.0, from_deg: 0.0 }), Some("FIELD 300 m"), "m/s");

        // With an east-northeast track, south is clear and sits to the right of the vertical axis line.
        let ene: Vec<Point> = (1..=30).map(|k| Point::new(pad.x + k as f32 * 4.0, pad.y - k as f32 * 1.5)).collect();
        let (spoke, placed) = place_ring_labels(size, pad, &rings, &ene, &reserved);
        assert_eq!(spoke, Spoke::South);
        assert!(!placed.is_empty());
        assert!(placed.iter().all(|(r, _)| r.x > pad.x), "south labels stay clear of the vertical axis hairline");

        // A north wind (from 0°) blows due south along x = pad.x: ring labels move off the 180° drift vector onto the east spoke.
        let due_south: Vec<Point> = (1..=70).map(|k| Point::new(pad.x, pad.y + k as f32 * 2.0)).collect();
        let (spoke, placed) = place_ring_labels(size, pad, &rings, &due_south, &reserved);
        assert_eq!(spoke, Spoke::East);
        assert_eq!(placed.len(), 3);
        for (rect, _) in &placed {
            assert!(due_south.iter().all(|p| !rect.contains(*p)), "{rect:?} overlaps south drift");
        }
    }
}
