//! Turns finished runs into what the centre and right columns draw: the linked altitude / vertical velocity / acceleration
//! plots with the reference flight's events (SimulateScreen, SimulateCompare) and the ground tracks (GroundTrack).

use bowshock_sim::{EventKind, FlightResult, G0};

use super::flight::{Run, Spec};
use crate::units::{Quantity, UnitSystem};
use crate::widgets::ground_track::{GroundTrack, Track, TrackPoint, Wind};
use crate::widgets::plot::{Axis, EventMark, Panel, Phase, PlotGroup, Series, SeriesColor, Tag};

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

/// The linked plots for `entries` in the unit family of `system`. One entry is a lone flight (`ink`, one framed value tag per
/// plot); several are a comparison (series colours, altitude labelled at the right end, one tooltip listing every flight).
/// The events, phases and timeline belong to the entry at `reference`. `None` without entries.
pub fn group(entries: &[Entry], reference: usize, system: UnitSystem) -> Option<PlotGroup> {
    let reference = entries.get(reference).or(entries.first())?;
    let compare = entries.len() > 1;
    let alt = system_unit(Quantity::Altitude, system);
    let vel = system_unit(Quantity::Velocity, system);

    let mut altitude = Panel::new("Altitude", alt.symbol).weight(1.2).zero_based();
    let mut velocity = Panel::new("Vertical velocity", vel.symbol);
    let mut accel = Panel::new("Acceleration", "g");
    if compare {
        velocity = velocity.tag(Tag::Off);
        accel = accel.tag(Tag::Off);
    }
    for e in entries {
        let samples = &e.run.result.trajectory.samples;
        let series = |name: &str, f: &dyn Fn(&bowshock_sim::Sample) -> f64| Series::new(name, samples.iter().map(|s| (s.t, f(s))).collect()).color(e.color);
        let a = series(e.name, &|s| alt.from_si(s.altitude));
        altitude = altitude.with(if compare { a.label_margin() } else { a });
        velocity = velocity.with(series(e.name, &|s| vel.from_si(s.vertical_velocity)));
        accel = accel.with(series(e.name, &|s| s.vertical_acceleration / G0));
    }

    let mut group = PlotGroup::new(Axis::time()).panel(altitude).panel(velocity).panel(accel);
    group.events = events(&reference.run.result);
    if compare {
        group.events.retain(|m| m.label != "Landing");
        group.right_margin = LABEL_MARGIN;
    }
    group.phases = phases(&reference.run.result);
    group.timeline = true;
    let end = entries.iter().map(|e| e.run.result.trajectory.samples.last().map_or(0.0, |s| s.t)).fold(0.0, f64::max);
    group.x_range = (end > 0.0).then_some((0.0, end));
    Some(group)
}

fn system_unit(quantity: Quantity, system: UnitSystem) -> crate::units::Unit {
    *quantity.unit_for(super::format::plot_system(system))
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
    use bowshock_sim::{Event, Sample, Summary, Trajectory};
    use std::sync::Arc;
    use std::time::Duration;

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
            thrust: 0.0,
            drag: 0.0,
            mass: 1.0,
            stability_margin: 2.0,
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
    fn a_lone_flight_is_ink_and_a_comparison_labels_altitude_and_keeps_the_reference_events() {
        let r = run();
        let one = [Entry { name: "Nominal", color: SeriesColor::Ink, run: &r, scatter: None }];
        let g = group(&one, 0, UnitSystem::MmGS).unwrap();
        assert_eq!(g.panels.len(), 3);
        assert_eq!(g.panels[0].unit, "m");
        assert!(g.panels[0].series[0].label.is_none() && g.right_margin == 0.0);
        assert_eq!(g.x_extent(), (0.0, 10.0));
        // Acceleration is in g: -g reads -1.
        assert!((g.panels[2].series[0].points[1].1 + 1.0).abs() < 1e-12);

        let two = [one[0], Entry { name: "Windy", color: SeriesColor::Series(2), run: &r, scatter: None }];
        let g = group(&two, 1, UnitSystem::MmGS).unwrap();
        assert!(g.panels[0].series.iter().all(|s| s.label.is_some()));
        assert!(g.right_margin > 0.0 && g.panels[1].tag == Tag::Off);
        assert!(g.events.iter().all(|m| m.label != "Landing"));
        assert_eq!(g.panels[0].series[1].color, SeriesColor::Series(2));
    }

    #[test]
    fn imperial_systems_plot_feet() {
        let r = run();
        let e = [Entry { name: "n", color: SeriesColor::Ink, run: &r, scatter: None }];
        let g = group(&e, 0, UnitSystem::FtLbS).unwrap();
        assert_eq!((g.panels[0].unit.as_str(), g.panels[1].unit.as_str()), ("ft", "ft/s"));
        assert!((g.panels[0].series[0].points[1].1 - 300.0 / 0.3048).abs() < 1e-9);
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
        assert!(group(&[], 0, UnitSystem::MmGS).is_none());
        assert!(ground(&[], 0).tracks.is_empty());
    }
}
