//! Turns finished runs into what the centre and right columns draw: the linked plots of the layout (any data sources, one or
//! several flights overlaid) with the reference flight's events (SimulateScreen, SimulateCompare) and the ground tracks
//! (GroundTrack).

use bowshock_model::PlotLayout;
use bowshock_sim::{EventKind, FlightResult};

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

/// The linked plots of `layout` for `entries` in the unit family of `system`. One entry is a lone flight (one framed value tag per
/// plot); several are a comparison (the flight is the line's colour, altitude-like plots label each flight at the right end, one
/// tooltip lists every line). A plot with several sources draws each in its own line style and names them in a legend. The
/// events, phases and timeline belong to the entry at `reference`. A plot with the stability margin gets the document's minimum
/// margin `min_margin` (calibers) as a dashed reference line. `None` without entries.
pub fn group(entries: &[Entry], reference: usize, system: UnitSystem, layout: &PlotLayout, min_margin: f64) -> Option<PlotGroup> {
    let reference = entries.get(reference).or(entries.first())?;
    let compare = entries.len() > 1;
    let imperial = super::format::imperial(system);
    let count = layout.panels.len();

    let mut group = PlotGroup::new(Axis::time());
    for (index, def) in layout.panels.iter().enumerate() {
        let sources: Vec<&Channel> = def.sources.iter().filter_map(|key| channels::find(key)).collect();
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
            for (c, style) in sources.iter().zip(STYLES) {
                let name = match (compare, multi) {
                    (false, false) => c.label.to_string(),
                    (true, false) => e.name.to_string(),
                    (false, true) => c.label.to_string(),
                    (true, true) => format!("{} · {}", e.name, c.label),
                };
                let color = if compare { e.color } else { c.color };
                let series = Series::new(name, c.series(samples, &unit)).color(color).style(style);
                // A comparison names each flight at the right end of the first plot's first source.
                let labelled = compare && index == 0 && std::ptr::eq(*c, sources[0]);
                panel = panel.with(if labelled { series.label_as(e.name.to_string(), LabelAt::Margin) } else { series });
            }
        }
        group = group.panel(panel);
    }

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
        PlotLayout { panels: panels.iter().map(|p| PlotPanel::new(p)).collect() }
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
