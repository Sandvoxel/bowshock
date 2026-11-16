//! Turns finished runs into what the centre and right columns draw: the linked plots of the layout (any data sources, one or
//! several flights overlaid) with the reference flight's events (SimulateScreen, SimulateCompare) and the ground tracks
//! (GroundTrack).

use bowshock_model::{PlotLayout, PlotX};
use bowshock_sim::{EventKind, FlightResult, Sample};

use super::channels::{self, Channel};
use super::flight::{Run, Spec};
use crate::units::UnitSystem;
use crate::widgets::ground_track::{GroundTrack, Track, TrackPoint, Wind};
use crate::widgets::plot::{Axis, EventMark, HLine, LabelAt, LegendItem, LineStyle, Panel, PanelEditor, Phase, PlotGroup, Series, SeriesColor};

/// The line style of the first, second, third and fourth source of a plot: the flight is the colour, the source is the style.
pub const STYLES: [LineStyle; 4] = [LineStyle::Solid, LineStyle::Dashed, LineStyle::Dotted, LineStyle::DashDot];

/// Width kept clear at the right of every plot for the direct labels of a comparison (SimulateCompare).
const LABEL_MARGIN: f32 = 96.0;

/// A flight on show: its name, colour role and run.
#[derive(Debug, Clone, Copy)]
pub struct Entry<'a> {
    /// The flight's name.
    pub name: &'a str,
    /// `Ink` for a lone flight, `Series(1..=4)` in a comparison.
    pub color: SeriesColor,
    /// Its run.
    pub run: &'a Run,
    /// Landings of a dispersion study to draw as dots with their 95 % ellipse, if the flight has a current one.
    pub scatter: Option<&'a [(f64, f64)]>,
}

/// The event marks of `result`, named as the kit's timeline: `Rail exit`, `Burnout`, `Apogee` (accent), the deployments by
/// the recovery device's name, `Landing`. The first ignition (t = 0) is not marked; later stages' ignitions and separations are.
pub fn events(result: &FlightResult) -> Vec<EventMark> {
    let stages = result.trajectory.events.iter().filter(|e| e.kind == EventKind::Burnout).count();
    let mut marks = Vec::new();
    for e in &result.trajectory.events {
        let numbered = |name: &str| match e.stage {
            Some(s) if stages > 1 => format!("{name} {}", s + 1),
            _ => name.to_string(),
        };
        let mark = match &e.kind {
            EventKind::Ignition if e.t <= 1e-9 => continue,
            EventKind::Ignition => EventMark::new(e.t, numbered("Ignition")),
            EventKind::RailExit => EventMark::new(e.t, "Rail exit"),
            EventKind::Burnout => EventMark::new(e.t, numbered("Burnout")),
            EventKind::Separation => EventMark::new(e.t, numbered("Separation")),
            EventKind::Apogee => EventMark::new(e.t, "Apogee").accent(),
            EventKind::Deploy(name) => EventMark::new(e.t, name.clone()),
            EventKind::Landing => EventMark::new(e.t, "Landing"),
        };
        marks.push(mark);
    }
    marks
}

/// The phase bars of the events strip: propulsion from the first ignition to the last burnout, recovery from the first
/// deployment to landing.
pub fn phases(result: &FlightResult) -> Vec<Phase> {
    let ev = &result.trajectory.events;
    let burnout = ev.iter().filter(|e| e.kind == EventKind::Burnout).map(|e| e.t).fold(f64::NAN, f64::max);
    let deploy = ev.iter().filter(|e| matches!(e.kind, EventKind::Deploy(_))).map(|e| e.t).fold(f64::NAN, f64::min);
    let landing = ev.iter().find(|e| e.kind == EventKind::Landing).map_or(result.summary.flight_time, |e| e.t);
    let mut bars = Vec::new();
    if burnout.is_finite() {
        bars.push(Phase { from: 0.0, to: burnout, color: SeriesColor::Propulsion });
    }
    if deploy.is_finite() {
        bars.push(Phase { from: deploy, to: landing, color: SeriesColor::Recovery });
    }
    bars
}

/// The channel an x choice reads (`None` for time, which is every sample's own `t`).
pub fn x_channel(x: PlotX) -> Option<&'static Channel> {
    match x {
        PlotX::Time => None,
        PlotX::Altitude => channels::find("altitude"),
        PlotX::Mach => channels::find("mach"),
        PlotX::Distance => channels::find("ground_distance"),
    }
}

/// The selector's word for an x choice (short: it shares the header row with the zoom readout and the presets).
pub fn x_label(x: PlotX) -> &'static str {
    match x {
        PlotX::Time => "Time",
        PlotX::Altitude => "Alt",
        PlotX::Mach => "Mach",
        PlotX::Distance => "Dist",
    }
}

/// The shared x axis for `x` in the unit family `imperial` (time in s, Mach bare, lengths in m or ft).
fn x_axis(x: PlotX, imperial: bool) -> Axis {
    match x_channel(x) {
        None => Axis::time(),
        Some(_) if x == PlotX::Mach => Axis::mach(),
        Some(c) => {
            let label = if x == PlotX::Altitude { "Altitude" } else { "Distance" };
            Axis { label: label.into(), unit: c.unit(imperial).symbol.to_string(), prefix: String::new(), decimals: 0 }
        }
    }
}

/// One stretch of a curve plotted against something other than time, where x only grows or only shrinks: `points` ascending in
/// x (a shrinking stretch is reversed), the flight phase it is flown in, and whether that is after apogee.
#[derive(Debug, Clone, PartialEq)]
pub struct Stretch {
    /// `(x, y)`, ascending in x.
    pub points: Vec<(f64, f64)>,
    /// The phase, unique within the curve: `boost`, `coast`, `ascent` (a piece spanning burnout), `descent`, numbered `coast 2`,
    /// `descent 3`... when the curve turns back more than once in one phase.
    pub phase: String,
    /// Flown after apogee (drawn dashed).
    pub descent: bool,
}

/// Cuts the parametric curve `(t, x, y)` (in time order) where x turns back, so each piece is a function of x the plots can
/// draw, interpolate and hover. Neighbouring pieces share the turning sample, so the curve stays one unbroken path. Each piece is
/// named by its phase from the event times `burnout` and `apogee` (s): descent when most of it is after apogee, boost or coast
/// when nearly all of it is before or after burnout, else ascent.
pub fn monotone_runs(curve: &[(f64, f64, f64)], burnout: f64, apogee: f64) -> Vec<Stretch> {
    let mut runs: Vec<Stretch> = Vec::new();
    let mut start = 0;
    let mut dir = 0.0f64;
    let close = |from: usize, to: usize, dir: f64, runs: &mut Vec<Stretch>| {
        let mut points: Vec<(f64, f64)> = curve[from..=to].iter().map(|&(_, x, y)| (x, y)).collect();
        if dir < 0.0 {
            points.reverse();
        }
        let (t0, t1) = (curve[from].0, curve[to].0);
        // Judged at its middle, so the piece that turns at the apogee sample is not split on which side the event fell.
        let descent = (t0 + t1) / 2.0 > apogee;
        // A piece flown almost wholly (90 %) before or after burnout is boost or coast: peak Mach lags burnout by a few samples.
        let span = (t1 - t0).max(f64::MIN_POSITIVE);
        // Burnout 0 means no burnout event: the ascent is not split.
        let base = if descent {
            "descent"
        } else if burnout <= 0.0 {
            "ascent"
        } else if burnout - t0 >= 0.9 * span {
            "boost"
        } else if t1 - burnout >= 0.9 * span {
            "coast"
        } else {
            "ascent"
        };
        let n = runs.iter().filter(|r| r.phase == base || r.phase.strip_prefix(base).is_some_and(|s| s.starts_with(' '))).count();
        let phase = if n == 0 { base.to_string() } else { format!("{base} {}", n + 1) };
        runs.push(Stretch { points, phase, descent });
    };
    for i in 1..curve.len() {
        let dx = curve[i].1 - curve[i - 1].1;
        if dx == 0.0 || !dx.is_finite() {
            continue;
        }
        if dir != 0.0 && dx.signum() != dir {
            close(start, i - 1, dir, &mut runs);
            start = i - 1;
        }
        dir = dx.signum();
    }
    if !curve.is_empty() {
        close(start, curve.len() - 1, dir, &mut runs);
    }
    runs
}

/// `samples`' value of `x` (display unit) at time `t`, interpolated; time itself for [`PlotX::Time`].
fn x_at(samples: &[Sample], x: Option<(&Channel, &crate::units::Unit)>, t: f64) -> f64 {
    let Some((c, unit)) = x else { return t };
    let i = samples.partition_point(|s| s.t <= t);
    let value = |s: &Sample| unit.from_si(c.si(s));
    match (i.checked_sub(1).and_then(|j| samples.get(j)), samples.get(i)) {
        (Some(a), Some(b)) if b.t > a.t => value(a) + (value(b) - value(a)) * (t - a.t) / (b.t - a.t),
        (Some(a), _) => value(a),
        (None, Some(b)) => value(b),
        (None, None) => 0.0,
    }
}

/// How many times the sampled curve `xs` passes `v` (a sample on `v` counts with the stretch that leaves it).
fn crossings(xs: &[f64], v: f64) -> usize {
    xs.windows(2).filter(|w| w[0].min(w[1]) <= v && v < w[0].max(w[1])).count()
}

/// The header key of a plot drawn in phase pieces: the solid line is the powered/coasting ascent (named as the curve's pieces
/// are: `boost`, `coast`, `ascent`), the dashed one the `descent`. `None` when no piece is flown after apogee.
fn phase_key(panel: &Panel) -> Option<Vec<(LineStyle, String)>> {
    if !panel.series.iter().any(|s| s.style == LineStyle::Dashed && s.name.contains("· descent")) {
        return None;
    }
    let mut ascent: Vec<&str> = Vec::new();
    for word in ["boost", "coast", "ascent"] {
        let tag = format!("· {word}");
        if panel.series.iter().any(|s| s.name.contains(&tag)) {
            ascent.push(word);
        }
    }
    Some(vec![(LineStyle::Solid, ascent.join("/")), (LineStyle::Dashed, "descent".into())])
}

/// The linked plots of `layout` for `entries` in the unit family of `system`. One entry is a lone flight (one framed value tag per
/// plot); several are a comparison (the flight is the line's colour, altitude-like plots label each flight at the right end, one
/// tooltip lists every line). A plot with several sources draws each in its own line style and names them in a legend. The
/// events, phases and timeline belong to the entry at `reference`. A plot with the stability margin gets the document's minimum
/// margin `min_margin` (calibers) as a dashed reference line. `None` without entries.
///
/// Against anything but time ([`PlotLayout::x`]) a curve may double back (altitude rises and falls): it is drawn as the pieces
/// of [`monotone_runs`], each named by its phase (`· boost`, `· coast`, `· descent`...; no two alike, and the hover lists only
/// the pieces whose x range holds the cursor), the descent dashed (a solid source), and
/// the events sit where the reference flight's x was at their time (no phase bars or timeline: they are spans of time).
pub fn group(entries: &[Entry], reference: usize, system: UnitSystem, layout: &PlotLayout, min_margin: f64) -> Option<PlotGroup> {
    let reference = entries.get(reference).or(entries.first())?;
    let compare = entries.len() > 1;
    let imperial = super::format::imperial(system);
    let count = layout.panels.len();
    let x_source = x_channel(layout.x);
    let x_unit = x_source.map(|c| c.unit(imperial));
    let x_of = x_source.zip(x_unit.as_ref());

    let mut group = PlotGroup::new(x_axis(layout.x, imperial));
    for (index, def) in layout.panels.iter().enumerate() {
        let mut sources: Vec<&Channel> = def.sources.iter().filter_map(|key| channels::find(key)).collect();
        // A source equal to the x quantity is a straight line against itself: it is dropped, and a panel left empty by that is
        // not drawn rather than standing in some other quantity.
        if let Some(xc) = x_source {
            sources.retain(|c| c.key != xc.key);
        }
        let Some(primary) = sources.first() else { continue };
        let multi = sources.len() > 1;
        let unit = primary.unit(imperial);
        let title = if multi { primary.dim.title() } else { primary.label };
        let mut panel = Panel::new(title, unit.symbol).editable(PanelEditor {
            can_add: sources.len() < bowshock_model::MAX_PANEL_SOURCES,
            can_up: index > 0,
            can_down: index + 1 < count,
            can_remove: count > 1,
            legend: sources
                .iter()
                .zip(STYLES)
                .map(|(c, style)| LegendItem { style, color: if compare { SeriesColor::Muted } else { c.color }, text: c.label.to_string() })
                .collect(),
        });
        if primary.key == "altitude" {
            panel = panel.weight(1.2);
        }
        if sources.iter().all(|c| c.zero_based) {
            panel = panel.zero_based();
        }
        // The document's minimum static margin, dashed `ok` like Analyze's `MIN 1.0`.
        if sources.iter().any(|c| c.key == "stability_margin") {
            panel = panel.hline(HLine::dashed(min_margin, format!("Min {min_margin:.1}"), SeriesColor::Ok));
        }
        for e in entries {
            let samples = &e.run.result.trajectory.samples;
            let events = &e.run.result.trajectory.events;
            let apogee = events.iter().find(|ev| ev.kind == EventKind::Apogee).map_or(f64::INFINITY, |ev| ev.t);
            let burnout = events.iter().filter(|ev| ev.kind == EventKind::Burnout).map(|ev| ev.t).fold(0.0, f64::max);
            for (c, style) in sources.iter().zip(STYLES) {
                let name = match (compare, multi) {
                    (false, false) => c.label.to_string(),
                    (true, false) => e.name.to_string(),
                    (false, true) => c.label.to_string(),
                    (true, true) => format!("{} · {}", e.name, c.label),
                };
                let color = if compare { e.color } else { c.color };
                // A comparison names each flight at the right end of the first plot's first source.
                let labelled = compare && index == 0 && std::ptr::eq(*c, sources[0]);
                let Some((xc, xu)) = x_of else {
                    let series = Series::new(name, c.series(samples, &unit)).color(color).style(style);
                    panel = panel.with(if labelled { series.label_as(e.name.to_string(), LabelAt::Margin) } else { series });
                    continue;
                };
                let curve: Vec<(f64, f64, f64)> = samples.iter().map(|s| (s.t, xu.from_si(xc.si(s)), unit.from_si(c.si(s)))).collect();
                let runs = monotone_runs(&curve, burnout, apogee);
                let first_source = std::ptr::eq(*c, sources[0]);
                // Each phase is named once, on its widest piece (a parachute's wobble in Mach makes many short ones).
                let base = |p: &str| p.split(' ').next().unwrap_or("").to_string();
                let span = |pts: &[(f64, f64)]| pts.last().map_or(0.0, |l| (l.0 - pts[0].0).abs());
                let widest: Vec<Option<usize>> = runs
                    .iter()
                    .map(|r| {
                        let b = base(&r.phase);
                        runs.iter()
                            .enumerate()
                            .filter(|(_, o)| base(&o.phase) == b)
                            .max_by(|x, y| span(&x.1.points).total_cmp(&span(&y.1.points)))
                            .map(|(k, _)| k)
                    })
                    .collect();
                for (i, run) in runs.into_iter().enumerate() {
                    let style = if run.descent && style == LineStyle::Solid { LineStyle::Dashed } else { style };
                    let name = format!("{name} · {}", run.phase);
                    let label = (widest[i] == Some(i)).then(|| base(&run.phase));
                    let series = Series::new(name, run.points).color(color).style(style);
                    panel = panel.with(if labelled && i == 0 {
                        series.label_as(e.name.to_string(), LabelAt::Margin)
                    } else if let (false, true, Some(text)) = (compare, first_source, label) {
                        series.label_as(text, LabelAt::End)
                    } else {
                        series
                    });
                }
            }
        }
        if x_of.is_some() && !compare && sources.len() == 1 {
            if let Some(key) = phase_key(&panel) {
                panel = panel.key(key);
            }
        }
        group = group.panel(panel);
    }

    let samples = &reference.run.result.trajectory.samples;
    group.events = events(&reference.run.result);
    if compare || x_of.is_some() {
        group.events.retain(|m| m.label != "Landing");
    }
    if compare {
        group.right_margin = LABEL_MARGIN;
    }
    if x_of.is_some() {
        // An event is only marked where its x is reached once: a value the curve passes on the way up and again on the way down
        // (or twice in one coast) has no single place on the axis.
        let xs: Vec<f64> = samples.iter().map(|s| x_at(samples, x_of, s.t)).collect();
        group.events = std::mem::take(&mut group.events)
            .into_iter()
            .filter_map(|mut m| {
                m.x = x_at(samples, x_of, m.x);
                (crossings(&xs, m.x) <= 1).then_some(m)
            })
            .collect();
        return Some(group);
    }
    group.phases = phases(&reference.run.result);
    group.timeline = true;
    let end = entries.iter().map(|e| e.run.result.trajectory.samples.last().map_or(0.0, |s| s.t)).fold(0.0, f64::max);
    group.x_range = (end > 0.0).then_some((0.0, end));
    Some(group)
}

/// The wind of `spec` as the ground track draws it.
pub fn wind(spec: &Spec) -> Wind {
    Wind { speed: spec.wind_speed, from_deg: spec.wind_from.to_degrees().rem_euclid(360.0) }
}

/// The ground tracks of `entries` (metres from the pad), north up, with the reference flight's wind and field radius.
pub fn ground(entries: &[Entry], reference: usize) -> GroundTrack {
    let Some(reference) = entries.get(reference).or(entries.first()) else { return GroundTrack::default() };
    let tracks = entries
        .iter()
        .map(|e| {
            let points =
                e.run.result.trajectory.samples.iter().map(|s| TrackPoint { t: s.t, east: s.position[0], north: s.position[1], alt: s.altitude }).collect();
            let track = Track::new(e.name, e.color, points);
            match e.scatter {
                Some(landings) => track.with_scatter(landings.to_vec()),
                None => track,
            }
        })
        .collect();
    GroundTrack { tracks, wind: Some(wind(&reference.run.spec)), field_radius: Some(reference.run.spec.field_radius) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::PlotPanel;
    use bowshock_sim::{Event, Sample, Summary, Trajectory, G0};
    use std::sync::Arc;
    use std::time::Duration;

    use crate::widgets::plot::Tag;

    fn sample(t: f64, alt: f64, vz: f64) -> Sample {
        Sample {
            t,
            position: [t, 0.5 * t, alt],
            velocity: [0.0, 0.0, vz],
            altitude: alt,
            vertical_velocity: vz,
            speed: vz.abs(),
            mach: 0.1,
            acceleration: 1.0,
            vertical_acceleration: -G0,
            mass: 1.0,
            stability_margin: 2.0,
            cg: 0.7,
            cp: 0.85,
            thrust: 50.0 * t,
            drag: 2.0 * t,
            ..Sample::default()
        }
    }

    fn run() -> Run {
        let event = |t, kind| Event { t, kind, altitude: 0.0, stage: Some(0) };
        let result = FlightResult {
            trajectory: Trajectory {
                samples: vec![sample(0.0, 0.0, 0.0), sample(5.0, 300.0, 0.0), sample(10.0, 0.0, -20.0)],
                events: vec![
                    event(0.0, EventKind::Ignition),
                    event(0.3, EventKind::RailExit),
                    event(1.6, EventKind::Burnout),
                    event(5.0, EventKind::Apogee),
                    event(6.0, EventKind::Deploy("Parachute".into())),
                    event(10.0, EventKind::Landing),
                ],
            },
            summary: Summary { flight_time: 10.0, ..Summary::default() },
        };
        Run {
            spec: Spec::default(),
            revision: 0,
            result: Arc::new(result),
            took: Duration::ZERO,
            dt: 0.001,
            delay: 4.4,
            times: super::super::flight::Times::default(),
        }
    }

    #[test]
    fn events_follow_the_timeline_order_and_apogee_is_the_accent() {
        let run = run();
        let marks = events(&run.result);
        let names: Vec<&str> = marks.iter().map(|m| m.label.as_str()).collect();
        assert_eq!(names, ["Rail exit", "Burnout", "Apogee", "Parachute", "Landing"]);
        assert!(marks.iter().all(|m| m.accent == (m.label == "Apogee")));
        let bars = phases(&run.result);
        assert_eq!((bars[0].from, bars[0].to), (0.0, 1.6));
        assert_eq!((bars[1].from, bars[1].to), (6.0, 10.0));
    }

    #[test]
    fn a_curve_that_doubles_back_in_x_is_cut_into_monotone_pieces() {
        // Altitude up to 300 m at t = 5 and back down: the ascent and the reversed descent, sharing the apogee sample.
        let curve = [(0.0, 0.0, 0.0), (2.5, 150.0, 40.0), (5.0, 300.0, 0.0), (7.5, 150.0, -20.0), (10.0, 0.0, -20.0)];
        let runs = monotone_runs(&curve, 1.0, 5.0);
        assert_eq!(runs.len(), 2);
        let phase = |s: &str| s.to_string();
        assert_eq!(runs[0], Stretch { points: vec![(0.0, 0.0), (150.0, 40.0), (300.0, 0.0)], phase: phase("ascent"), descent: false });
        assert_eq!(runs[1], Stretch { points: vec![(0.0, -20.0), (150.0, -20.0), (300.0, 0.0)], phase: phase("descent"), descent: true });
        // A monotone x (time-like) is one piece; a held x does not cut.
        let flat = [(0.0, 0.0, 1.0), (1.0, 1.0, 2.0), (2.0, 1.0, 3.0), (3.0, 2.0, 4.0)];
        assert_eq!(monotone_runs(&flat, 0.0, 10.0).len(), 1);
        assert!(monotone_runs(&[], 0.0, 0.0).is_empty());
    }

    #[test]
    fn the_key_uses_the_phase_names_of_the_curve_pieces() {
        // Burnout known: the solid line is boost/coast, as the hover rows and the end labels name it.
        let curve = [(0.0, 0.0, 0.0), (1.0, 0.5, 1.0), (3.0, 0.2, 2.0), (5.0, 0.01, 3.0), (6.0, 0.1, 4.0)];
        let panel_of = |burnout: f64| {
            monotone_runs(&curve, burnout, 5.0).into_iter().fold(Panel::new("Altitude", "m"), |p, r| {
                let style = if r.descent { LineStyle::Dashed } else { LineStyle::Solid };
                p.with(Series::new(format!("Altitude · {}", r.phase), r.points).style(style))
            })
        };
        let key = phase_key(&panel_of(1.0)).unwrap();
        assert_eq!(key.iter().map(|(_, w)| w.as_str()).collect::<Vec<_>>(), ["boost/coast", "descent"]);
        // No burnout event: the ascent is one piece, named ascent everywhere.
        assert_eq!(monotone_runs(&curve, 0.0, 5.0)[0].phase, "ascent");
        let key = phase_key(&panel_of(0.0)).unwrap();
        assert_eq!(key.iter().map(|(_, w)| w.as_str()).collect::<Vec<_>>(), ["ascent", "descent"]);
    }

    #[test]
    fn mach_pieces_are_named_by_phase_and_never_twice() {
        // Mach up in boost, down in coast to apogee, then up, down and up again under the chute: the repeats are numbered.
        let curve = [(0.0, 0.0, 0.0), (1.0, 0.5, 1.0), (3.0, 0.2, 2.0), (5.0, 0.01, 3.0), (6.0, 0.1, 4.0), (8.0, 0.05, 5.0), (9.0, 0.06, 6.0)];
        let runs = monotone_runs(&curve, 1.0, 5.0);
        let names: Vec<&str> = runs.iter().map(|r| r.phase.as_str()).collect();
        assert_eq!(names, ["boost", "coast", "descent", "descent 2", "descent 3"]);
        assert!(runs.iter().all(|r| r.descent == r.phase.starts_with("descent")));
        // Only pieces whose x range holds the cursor have a value there (the hover lists those).
        let series: Vec<Series> = runs.into_iter().map(|r| Series::new(r.phase, r.points)).collect();
        let at = |x: f64| series.iter().filter(|s| s.value_at(x).is_some()).map(|s| s.name.as_str()).collect::<Vec<_>>();
        assert_eq!(at(0.3), ["boost", "coast"]);
        assert_eq!(at(0.08), ["boost", "coast", "descent", "descent 2"]);
    }

    #[test]
    fn against_altitude_the_descent_is_dashed_and_events_sit_at_their_altitude() {
        let run = run();
        let entries = [Entry { name: "Nominal", color: SeriesColor::Ink, run: &run, scatter: None }];
        let layout = PlotLayout { x: PlotX::Altitude, ..layout(&[&["vertical_velocity"]]) };
        let g = group(&entries, 0, UnitSystem::MKgS, &layout, 1.0).unwrap();
        assert_eq!(g.x.label, "Altitude");
        assert_eq!(g.x.unit, "m");
        let series = &g.panels[0].series;
        assert_eq!(series.len(), 2);
        assert_eq!((series[0].style, series[1].style), (LineStyle::Solid, LineStyle::Dashed));
        assert_eq!(series[1].name, "Vertical velocity · descent");
        assert_eq!(g.x_extent(), (0.0, 300.0));
        let apogee = g.events.iter().find(|m| m.label == "Apogee").unwrap();
        // The pieces are named in place at their ends and a compact key names the line styles.
        assert_eq!(
            series.iter().map(|s| (s.label, s.label_text.as_deref())).collect::<Vec<_>>(),
            [(Some(LabelAt::End), Some("ascent")), (Some(LabelAt::End), Some("descent"))]
        );
        assert_eq!(g.panels[0].key.len(), 2);
        assert!((apogee.x - 300.0).abs() < 1e-9);
        assert!(g.events.iter().all(|m| m.label != "Landing") && g.phases.is_empty() && !g.timeline);
        // In imperial the axis is in feet.
        let g = group(&entries, 0, UnitSystem::FtLbS, &layout, 1.0).unwrap();
        assert_eq!(g.x.unit, "ft");
        assert!((g.x_extent().1 - 300.0 / 0.3048).abs() < 1e-6);
    }

    #[test]
    fn a_panel_whose_source_is_the_x_quantity_is_not_drawn_and_ambiguous_events_are_not_marked() {
        let run = run();
        let entries = [Entry { name: "Nominal", color: SeriesColor::Ink, run: &run, scatter: None }];
        let layout = PlotLayout { x: PlotX::Altitude, ..layout(&[&["altitude"], &["vertical_velocity"]]) };
        let g = group(&entries, 0, UnitSystem::MKgS, &layout, 1.0).unwrap();
        assert_eq!(g.panels.len(), 1);
        assert_eq!(g.panels[0].title, "Vertical velocity");
        // Burnout (t = 1.6, 96 m on the way up) is passed again on the way down: no single place on the axis.
        assert!(g.events.iter().all(|m| m.label != "Burnout"));
        assert!(g.events.iter().any(|m| m.label == "Apogee"));
        assert_eq!(crossings(&[0.0, 300.0, 0.0], 100.0), 2);
        assert_eq!(crossings(&[0.0, 1.0, 2.0], 1.0), 1);
    }

    #[test]
    fn a_lone_flight_is_ink_and_a_comparison_labels_the_first_plot_and_keeps_the_reference_events() {
        let r = run();
        let one = [Entry { name: "Nominal", color: SeriesColor::Ink, run: &r, scatter: None }];
        let g = group(&one, 0, UnitSystem::MmGS, &PlotLayout::default(), 1.0).unwrap();
        assert_eq!(g.panels.len(), 3);
        assert_eq!(g.panels[0].unit, "m");
        assert!(g.panels[0].series[0].label.is_none() && g.right_margin == 0.0);
        assert!(g.panels[0].series[0].color == SeriesColor::Ink && g.panels[0].zero_base);
        assert_eq!(g.x_extent(), (0.0, 10.0));
        // Acceleration is in g: -g reads -1.
        assert!((g.panels[2].series[0].points[1].1 + 1.0).abs() < 1e-12);
        // Every panel is editable: its title picks, and only the ends cannot move further.
        let editors: Vec<_> = g.panels.iter().map(|p| p.editor.clone().unwrap()).collect();
        assert!(!editors[0].can_up && editors[0].can_down && editors[1].can_up && editors[1].can_down && editors[2].can_up && !editors[2].can_down);
        assert!(editors.iter().all(|e| e.can_add && e.can_remove));

        let two = [one[0], Entry { name: "Windy", color: SeriesColor::Series(2), run: &r, scatter: None }];
        let g = group(&two, 1, UnitSystem::MmGS, &PlotLayout::default(), 1.0).unwrap();
        assert!(g.panels[0].series.iter().all(|s| s.label.is_some()));
        assert!(g.panels[1].series.iter().all(|s| s.label.is_none()));
        assert!(g.right_margin > 0.0 && g.panels[1].tag == Tag::Auto);
        assert!(g.events.iter().all(|m| m.label != "Landing"));
        assert_eq!(g.panels[0].series[1].color, SeriesColor::Series(2));
    }

    #[test]
    fn imperial_systems_plot_feet() {
        let r = run();
        let e = [Entry { name: "n", color: SeriesColor::Ink, run: &r, scatter: None }];
        let g = group(&e, 0, UnitSystem::FtLbS, &PlotLayout::default(), 1.0).unwrap();
        assert_eq!((g.panels[0].unit.as_str(), g.panels[1].unit.as_str()), ("ft", "ft/s"));
        assert!((g.panels[0].series[0].points[1].1 - 300.0 / 0.3048).abs() < 1e-9);
    }

    fn layout(panels: &[&[&str]]) -> PlotLayout {
        PlotLayout { panels: panels.iter().map(|p| PlotPanel::new(p)).collect(), ..PlotLayout::default() }
    }

    #[test]
    fn a_panel_draws_any_source_in_its_unit_with_one_line_style_per_source() {
        let r = run();
        let e = [Entry { name: "Nominal", color: SeriesColor::Ink, run: &r, scatter: None }];
        let g = group(&e, 0, UnitSystem::MmGS, &layout(&[&["thrust", "drag"], &["cg_station", "cp_station"], &["mach"]]), 1.0).unwrap();
        // Force: both lines in N, the first solid and the second dashed, named by source and listed in the legend.
        let force = &g.panels[0];
        assert_eq!((force.title.as_str(), force.unit.as_str()), ("Force", "N"));
        assert_eq!(
            force.series.iter().map(|s| (s.name.as_str(), s.style)).collect::<Vec<_>>(),
            [("Thrust", LineStyle::Solid), ("Drag force", LineStyle::Dashed)]
        );
        assert_eq!(force.series[0].points[2], (10.0, 500.0));
        assert_eq!(force.series[1].points[2], (10.0, 20.0));
        assert_eq!(force.editor.as_ref().unwrap().legend.len(), 2);
        // Stations are millimetres and keep the CG and CP colours of the kit; the panel is anchored at zero.
        let balance = &g.panels[1];
        assert_eq!(balance.unit, "mm");
        assert!((balance.series[0].points[0].1 - 700.0).abs() < 1e-9 && (balance.series[1].points[0].1 - 850.0).abs() < 1e-9);
        assert_eq!((balance.series[0].color, balance.series[1].color), (SeriesColor::Cg, SeriesColor::Cp));
        // Mach has no unit text.
        assert_eq!((g.panels[2].title.as_str(), g.panels[2].unit.as_str()), ("Mach number", ""));
        // The same layout in imperial: stations in inches, forces in lbf.
        let g = group(&e, 0, UnitSystem::InOzS, &layout(&[&["thrust"], &["cg_station"]]), 1.0).unwrap();
        assert_eq!((g.panels[0].unit.as_str(), g.panels[1].unit.as_str()), ("lbf", "in"));
        assert!((g.panels[1].series[0].points[0].1 - 0.7 / 0.0254).abs() < 1e-9);
    }

    #[test]
    fn overlaid_flights_and_several_sources_combine_colour_by_flight_and_style_by_source() {
        let r = run();
        let a = Entry { name: "Nominal", color: SeriesColor::Series(1), run: &r, scatter: None };
        let b = Entry { name: "Windy", color: SeriesColor::Series(2), run: &r, scatter: None };
        let g = group(&[a, b], 0, UnitSystem::MmGS, &layout(&[&["thrust", "drag"]]), 1.0).unwrap();
        let lines: Vec<_> = g.panels[0].series.iter().map(|s| (s.name.as_str(), s.color, s.style)).collect();
        assert_eq!(
            lines,
            [
                ("Nominal · Thrust", SeriesColor::Series(1), LineStyle::Solid),
                ("Nominal · Drag force", SeriesColor::Series(1), LineStyle::Dashed),
                ("Windy · Thrust", SeriesColor::Series(2), LineStyle::Solid),
                ("Windy · Drag force", SeriesColor::Series(2), LineStyle::Dashed),
            ]
        );
        // Only the first source of each flight is labelled at the right end; the legend of the sources is neutral.
        assert_eq!(g.panels[0].series.iter().filter(|s| s.label.is_some()).count(), 2);
        assert!(g.panels[0].editor.as_ref().unwrap().legend.iter().all(|l| l.color == SeriesColor::Muted));
        // One source per panel and two flights: the lines are named by flight.
        let g = group(&[a, b], 0, UnitSystem::MmGS, &layout(&[&["mach"]]), 1.0).unwrap();
        assert_eq!(g.panels[0].series.iter().map(|s| s.name.as_str()).collect::<Vec<_>>(), ["Nominal", "Windy"]);
    }

    #[test]
    fn ground_tracks_start_at_the_pad_and_carry_the_wind_and_field() {
        let r = run();
        let e = [Entry { name: "n", color: SeriesColor::Ink, run: &r, scatter: None }];
        let g = ground(&e, 0);
        assert_eq!(g.tracks.len(), 1);
        assert_eq!((g.tracks[0].points[0].east, g.tracks[0].points[0].north), (0.0, 0.0));
        assert_eq!(g.field_radius, Some(300.0));
        assert!((g.wind.unwrap().from_deg - 250.0).abs() < 1e-9);
        assert!(group(&[], 0, UnitSystem::MmGS, &PlotLayout::default(), 1.0).is_none());
        assert!(ground(&[], 0).tracks.is_empty());
    }
}
