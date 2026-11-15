//! Plot data of the Analyze screen: turns an [`Output`] into the linked [`PlotGroup`] of its analysis (and the ground track
//! and apogee histogram of the Monte Carlo). All y values are converted to the display unit here, so everything below the
//! view stays SI; [`Scale`] converts the plots' x back.

use bowshock_model::{ConfigId, StabilityBands};
use bowshock_sim::{AlphaPoint, MachPoint};

use super::compute::{Analysis, Data, Output, Run, FLUTTER_MIN_MARGIN};
use super::overlay::{Field, Overlay};
use super::readout::{config_color, drag_parts, other_visible, DRAG_PARTS};
use crate::units::{Quantity, Unit, UnitSystem};
use crate::widgets::ground_track::{GroundTrack, Track, TrackPoint, Wind};
use crate::widgets::plot::{Axis, HLine, LabelAt, LineStyle, Panel, PlotGroup, Series, SeriesColor, Tag, YTicks};

/// Width kept free at the right of the plots for direct labels.
const MARGIN_PX: f32 = 100.0;
/// Bins of the apogee histogram.
const BINS: usize = 12;

/// How the plots' x values relate to SI.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scale {
    /// SI per plot unit (1 for Mach, π/180 for degrees, the altitude unit for flutter).
    pub x_si: f64,
}

impl Scale {
    /// The plots' x as SI.
    pub fn to_si(self, x: f64) -> f64 {
        x * self.x_si
    }
}

/// Everything the centre column draws.
#[derive(Debug, Clone)]
pub struct Built {
    /// The linked plots (for the Monte Carlo: the apogee histogram).
    pub group: PlotGroup,
    /// The landing scatter, for the Monte Carlo.
    pub ground: Option<GroundTrack>,
    /// Colour keys of the configurations when more than one is drawn.
    pub legend: Vec<(SeriesColor, String)>,
    /// x conversion.
    pub scale: Scale,
}

fn unit(q: Quantity, units: UnitSystem) -> &'static Unit {
    q.unit_for(units)
}

/// The subtitle note of a panel the CFD table method takes from the Extended method (the solver has no angle of attack).
const CFD_ELSEWHERE: &str = "Extended (CFD is zero-lift)";

/// Builds the plots of `out` for `units`, with the comparison `overlay` drawn dashed where it has the quantity. `active` is
/// the document's active configuration: the Cd panels stack its drag by source whatever else is drawn.
pub fn build(out: &Output, units: UnitSystem, overlay: Option<&Overlay>, active: ConfigId) -> Built {
    let legend = if out.runs.len() > 1 { out.runs.iter().enumerate().map(|(i, r)| (config_color(i), r.name.clone())).collect() } else { Vec::new() };
    let (mut group, ground, scale) = match out.analysis {
        Analysis::Aerodynamics => (aero(out, units, overlay, active), None, Scale { x_si: 1.0 }),
        Analysis::Drag => (drag(out, overlay, active), None, Scale { x_si: 1.0 }),
        Analysis::Stability => (stability(out, units, overlay), None, Scale { x_si: std::f64::consts::PI / 180.0 }),
        Analysis::Flutter => (flutter(out, units), None, Scale { x_si: unit(Quantity::Altitude, units).to_si }),
        Analysis::Dispersion => {
            let (hist, ground) = dispersion(out, units);
            (hist, Some(ground), Scale { x_si: unit(Quantity::Altitude, units).to_si })
        }
    };
    group.y_ticks = YTicks::Even(5);
    // One right gutter in every analysis, so switching analyses does not change the plot width.
    group.right_margin = MARGIN_PX;
    // The CFD table is zero-lift drag only: the drag panels are the solver's, every other panel is the Extended method's, and
    // they say so.
    if out.cfd.is_some() {
        for panel in &mut group.panels {
            if panel.title.starts_with("Drag") || panel.title.starts_with("Share of drag") {
                panel.unit = format!("{} · CFD", panel.unit);
                continue;
            }
            if panel.value_unit.is_none() && !panel.unit.is_empty() && !panel.unit.contains(' ') {
                panel.value_unit = Some(panel.unit.clone());
            }
            panel.unit = if panel.unit.is_empty() { CFD_ELSEWHERE.to_string() } else { format!("{} · {CFD_ELSEWHERE}", panel.unit) };
        }
    }
    // Mach beyond what the nominal flight reaches is shaded (the worst case there is out of the envelope).
    if matches!(out.analysis, Analysis::Aerodynamics | Analysis::Drag) {
        let flown = mach_runs(out).iter().filter_map(|(_, r, _)| r.max_mach).fold(f64::NEG_INFINITY, f64::max);
        if flown.is_finite() && flown < out.conditions.mach.1 {
            group.beyond = Some((flown, format!("beyond flight · M {flown:.2}")));
        }
    }
    Built { group, ground, legend, scale }
}

fn mach_runs(out: &Output) -> Vec<(usize, &Run, &[MachPoint])> {
    out.runs.iter().enumerate().filter_map(|(i, r)| if let Data::Mach(p) = &r.data { Some((i, r, p.as_slice())) } else { None }).collect()
}

fn curve(points: &[MachPoint], f: impl Fn(&MachPoint) -> f64) -> Vec<(f64, f64)> {
    points.iter().map(|p| (p.mach, f(p))).collect()
}

fn margin_panel(series: Vec<Series>, bands: StabilityBands) -> Panel {
    let mut panel = Panel::new("Static margin", "cal")
        .hline(HLine::dashed(bands.over, format!("Over-stable {:.1}", bands.over), SeriesColor::Warn))
        .hline(HLine::dashed(bands.stable, format!("Min {:.1}", bands.stable), SeriesColor::Ok));
    panel.series = series;
    panel
}

/// The stacked Cd layers of one run: skin friction, pressure, base, fins, and "Other" only when launch lugs and rail buttons
/// are over 5 % of the drag (otherwise they are part of Pressure; see [`other_visible`]).
fn stacked(points: &[MachPoint]) -> Vec<Series> {
    let show_other = other_visible(points);
    let mut layers: Vec<Series> = DRAG_PARTS
        .iter()
        .enumerate()
        .map(|(i, (name, color))| {
            Series::new(*name, points.iter().map(|p| (p.mach, drag_parts(&p.drag, show_other)[i])).collect()).color(*color).filled().label_margin()
        })
        .collect();
    if show_other {
        layers.push(Series::new("Other", curve(points, |p| p.drag.protuberances)).color(SeriesColor::Muted).filled().label_margin());
    }
    layers
}

/// Adds the comparison's `field` to `panel` as a dashed series in its own colour, labelled with its source; `fx` / `fy` map
/// the overlay's SI x and y to the plot's.
fn overlaid(
    panel: Panel,
    overlay: Option<&Overlay>,
    analysis: Analysis,
    field: Field,
    single: bool,
    fx: impl Fn(f64) -> f64,
    fy: impl Fn(f64) -> f64,
) -> Panel {
    let Some(o) = overlay else { return panel };
    let points: Vec<(f64, f64)> = o.points(analysis, field).into_iter().map(|(x, y)| (fx(x), fy(y))).collect();
    if points.is_empty() {
        return panel;
    }
    let color = if single { SeriesColor::Series(2) } else { SeriesColor::Muted };
    panel.with(Series::new(o.label.clone(), points).color(color).style(LineStyle::Dashed).label_margin())
}

/// The Cd panel of the Mach analyses: always the stacked areas by source of the active configuration `active` (an index into
/// `runs`) with its total as an `ink` line on top, whatever else is drawn. Every other configuration is its total Cd as a dashed
/// line in its own colour (the one of its legend entry and of its margin and CP lines), labelled in the margin; they are added
/// over the stack and never replace it.
fn cd_panel(runs: &[(usize, &Run, &[MachPoint])], active: usize) -> Panel {
    let Some((_, run, points)) = runs.get(active) else { return Panel::new("Drag coefficient Cd", "") };
    let subtitle = if runs.len() == 1 { "stacked by source".to_string() } else { format!("stacked by source · {}", run.name) };
    let mut panel = Panel::new("Drag coefficient Cd", subtitle).weight(1.4);
    let layers = stacked(points);
    panel.tag = Tag::Only(layers.len());
    panel.series = layers;
    panel.series.push(Series::new("Cd", curve(points, |p| p.drag.total)).label_as("Total", LabelAt::Inside));
    for (i, run, points) in runs.iter().enumerate().filter_map(|(k, r)| (k != active).then_some(r)) {
        panel.series.push(Series::new(run.name.clone(), curve(points, |p| p.drag.total)).color(config_color(*i)).style(LineStyle::Dashed).label_margin());
    }
    panel
}

/// Where the active configuration's run is in `runs` (the first when it is not among them).
fn active_index(runs: &[(usize, &Run, &[MachPoint])], active: ConfigId) -> usize {
    runs.iter().position(|(_, r, _)| r.id == active).unwrap_or(0)
}

fn aero(out: &Output, units: UnitSystem, overlay: Option<&Overlay>, active: ConfigId) -> PlotGroup {
    let runs = mach_runs(out);
    let active = active_index(&runs, active);
    let len = unit(Quantity::Length, units);
    let to = |m: f64| m / len.to_si;
    let single = runs.len() == 1;

    let margin: Vec<Series> = if single {
        runs.iter().map(|(_, _, p)| Series::new("Margin", curve(p, |p| p.margin_cal))).collect()
    } else {
        runs.iter().map(|(i, r, p)| Series::new(r.name.clone(), curve(p, |p| p.margin_cal)).color(config_color(*i)).label_margin()).collect()
    };

    let mut cpcg = Panel::new("CP and CG", format!("{} from tip", len.symbol)).value_unit(len.symbol).tag(Tag::Only(0));
    if let Some((_, first, p)) = runs.get(active) {
        cpcg.series.push(Series::new("CP", curve(p, |p| to(p.cp))).color(SeriesColor::Cp).label_inside());
        let flat = |cg: f64| p.iter().map(|q| (q.mach, to(cg))).collect::<Vec<_>>();
        if single {
            cpcg.series.push(Series::new("CG liftoff", flat(first.cg_liftoff)).color(SeriesColor::Cg).label_inside());
            cpcg.series.push(Series::new("CG burnout", flat(first.cg_burnout)).color(SeriesColor::Cg).style(LineStyle::Dashed).label_inside());
        } else {
            let burnout = out.conditions.burnout;
            for (i, run, q) in &runs {
                let pts = q.iter().map(|m| (m.mach, to(run.cg(burnout)))).collect::<Vec<_>>();
                cpcg.series.push(Series::new(format!("CG {}", run.name), pts).color(config_color(*i)).label_inside());
            }
        }
    }

    let id = |x: f64| x;
    let a = Analysis::Aerodynamics;
    let margin_panel = overlaid(margin_panel(margin, out.bands), overlay, a, Field::Margin, single, id, id);
    let cpcg = overlaid(cpcg, overlay, a, Field::Cp, single, id, to);
    let cd = overlaid(cd_panel(&runs, active), overlay, a, Field::Cd, single, id, id);
    let mut group = PlotGroup::new(Axis::mach()).panel(margin_panel).panel(cpcg).panel(cd);
    group.right_margin = MARGIN_PX;
    group.x_range = Some(out.conditions.mach);
    group
}

fn drag(out: &Output, overlay: Option<&Overlay>, active: ConfigId) -> PlotGroup {
    let runs = mach_runs(out);
    let active = active_index(&runs, active);
    // The shares are the active configuration's, like the stack above them: the other configurations only add their totals.
    let mut shares = Panel::new("Share of drag", "% of total").value_unit("%").zero_based();
    if let Some((_, _, points)) = runs.get(active) {
        let show_other = other_visible(points);
        let part = |i: usize| move |p: &MachPoint| 100.0 * drag_parts(&p.drag, show_other)[i] / p.drag.total.max(1e-12);
        for (i, (name, color)) in DRAG_PARTS.iter().enumerate() {
            shares.series.push(Series::new(*name, curve(points, part(i))).color(*color).label_margin());
        }
        if show_other {
            shares
                .series
                .push(Series::new("Other", curve(points, |p| 100.0 * p.drag.protuberances / p.drag.total.max(1e-12))).color(SeriesColor::Muted).label_margin());
        }
    }
    let id = |x: f64| x;
    let cd = overlaid(cd_panel(&runs, active).weight(1.6), overlay, Analysis::Drag, Field::Cd, runs.len() == 1, id, id);
    let mut group = PlotGroup::new(Axis::mach()).panel(cd).panel(shares);
    group.right_margin = MARGIN_PX;
    group.x_range = Some(out.conditions.mach);
    group
}

fn stability(out: &Output, units: UnitSystem, overlay: Option<&Overlay>) -> PlotGroup {
    let len = unit(Quantity::Length, units);
    let to = |m: f64| m / len.to_si;
    let single = out.runs.len() == 1;
    let alpha_runs: Vec<(usize, &Run, &[AlphaPoint])> =
        out.runs.iter().enumerate().filter_map(|(i, r)| if let Data::Alpha(p) = &r.data { Some((i, r, p.as_slice())) } else { None }).collect();
    let deg = |p: &AlphaPoint, y: f64| (p.alpha.to_degrees(), y);

    let mut margin = Vec::new();
    let mut cn = Panel::new("Normal force CN", "");
    let mut cpcg = Panel::new("CP and CG", format!("{} from tip", len.symbol)).value_unit(len.symbol).tag(Tag::Only(0));
    for (i, run, points) in &alpha_runs {
        let color = config_color(*i);
        let cg = to(run.cg(out.conditions.burnout));
        let m: Vec<_> = points.iter().map(|p| deg(p, p.margin_cal)).collect();
        let cn_pts: Vec<_> = points.iter().map(|p| deg(p, p.cn)).collect();
        if single {
            margin.push(Series::new("Margin", m));
            cn = cn.with(Series::new("CN", cn_pts));
        } else {
            margin.push(Series::new(run.name.clone(), m).color(color).label_margin());
            cn = cn.with(Series::new(run.name.clone(), cn_pts).color(color).label_margin());
        }
        if *i == 0 {
            cpcg = cpcg.with(Series::new("CP", points.iter().map(|p| deg(p, to(p.cp))).collect()).color(SeriesColor::Cp).label_inside());
        }
        let name = if single { "CG".to_string() } else { format!("CG {}", run.name) };
        cpcg = cpcg.with(Series::new(name, points.iter().map(|p| deg(p, cg)).collect()).color(if single { SeriesColor::Cg } else { color }).label_inside());
    }
    let axis = Axis { label: "AoA".into(), unit: "°".into(), prefix: "α ".into(), decimals: 1 };
    let a = Analysis::Stability;
    let (deg, id) = (f64::to_degrees, |y: f64| y);
    let margin_panel = overlaid(margin_panel(margin, out.bands), overlay, a, Field::Margin, single, deg, id);
    let cpcg = overlaid(cpcg, overlay, a, Field::Cp, single, deg, to);
    let cn = overlaid(cn, overlay, a, Field::Cn, single, deg, id);
    let mut group = PlotGroup::new(axis).panel(margin_panel).panel(cpcg).panel(cn);
    group.right_margin = MARGIN_PX;
    group.x_range = Some((0.0, out.conditions.alpha_max.to_degrees()));
    group
}

fn flutter(out: &Output, units: UnitSystem) -> PlotGroup {
    let alt = unit(Quantity::Altitude, units);
    let vel = unit(Quantity::Velocity, units);
    let single = out.runs.len() == 1;
    let mut velocity = Panel::new("Flutter velocity", vel.symbol).value_unit(vel.symbol);
    let mut margin = Panel::new("Flutter margin", "× max velocity").value_unit("×").hline(HLine::dashed(
        FLUTTER_MIN_MARGIN,
        format!("Min {FLUTTER_MIN_MARGIN:.1}"),
        SeriesColor::Ok,
    ));
    let mut top: f64 = 0.0;
    for (i, run) in out.runs.iter().enumerate() {
        let Data::Flutter(Some(curve)) = &run.data else { continue };
        let color = if single { SeriesColor::Ink } else { config_color(i) };
        let label = |s: Series| if single { s } else { s.label_margin() };
        velocity =
            velocity.with(label(Series::new(run.name.clone(), curve.velocity.iter().map(|&(h, v)| (h / alt.to_si, v / vel.to_si)).collect()).color(color)));
        margin =
            margin.with(label(Series::new(run.name.clone(), curve.velocity.iter().map(|&(h, _)| (h / alt.to_si, curve.margin_at(h))).collect()).color(color)));
        velocity = velocity.hline(HLine::dashed(
            curve.max_velocity / vel.to_si,
            if single { "Max velocity".to_string() } else { format!("Max {}", run.name) },
            SeriesColor::Warn,
        ));
        top = top.max(curve.velocity.last().map_or(0.0, |p| p.0));
    }
    let axis = Axis { label: "Altitude".into(), unit: alt.symbol.into(), prefix: String::new(), decimals: 0 };
    let mut group = PlotGroup::new(axis).panel(velocity).panel(margin);
    group.right_margin = if single { 0.0 } else { MARGIN_PX };
    if top > 0.0 {
        group.x_range = Some((0.0, top / alt.to_si));
    }
    group
}

fn dispersion(out: &Output, units: UnitSystem) -> (PlotGroup, GroundTrack) {
    let len = unit(Quantity::Altitude, units);
    let single = out.runs.len() == 1;
    let mut tracks = Vec::new();
    let all: Vec<f64> =
        out.runs.iter().filter_map(|r| if let Data::Dispersion(Some(d)) = &r.data { Some(d.apogees.iter().copied()) } else { None }).flatten().collect();
    let (lo, hi) = all.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &a| (lo.min(a), hi.max(a)));
    let width = if hi > lo { (hi - lo) / BINS as f64 } else { 1.0 };

    let mut hist = Panel::new("Apogee", "runs per bin").value_unit("runs").zero_based();
    for (i, run) in out.runs.iter().enumerate() {
        let Data::Dispersion(Some(d)) = &run.data else { continue };
        let color = config_color(i);
        let points = d.nominal.iter().map(|&(t, east, north, alt)| TrackPoint { t, east, north, alt }).collect();
        tracks.push(Track::new(run.name.clone(), color, points).with_scatter(d.landings.iter().map(|l| (l[0], l[1])).collect()));

        // A frequency polygon: the count of each bin at its centre, closed to zero one bin beyond either end.
        let mut counts = [0usize; BINS];
        for &a in &d.apogees {
            counts[(((a - lo) / width) as usize).min(BINS - 1)] += 1;
        }
        let mut pts = vec![((lo - 0.5 * width) / len.to_si, 0.0)];
        pts.extend(counts.iter().enumerate().map(|(k, &c)| ((lo + (k as f64 + 0.5) * width) / len.to_si, c as f64)));
        pts.push(((lo + (BINS as f64 + 0.5) * width) / len.to_si, 0.0));
        let series = Series::new(run.name.clone(), pts).color(color);
        hist = hist.with(if single { series.filled() } else { series.label_margin() });
    }
    let axis = Axis { label: "Apogee".into(), unit: len.symbol.into(), prefix: String::new(), decimals: 0 };
    let mut group = PlotGroup::new(axis).panel(hist);
    group.right_margin = if single { 0.0 } else { MARGIN_PX };
    let wind = (out.conditions.wind > 0.0).then_some(Wind { speed: out.conditions.wind, from_deg: 0.0 });
    (group, GroundTrack { tracks, wind, field_radius: None })
}

#[cfg(test)]
mod tests {
    use super::super::compute::{compute, jobs, Conditions, Method, Request};
    use super::super::overlay::Sample;
    use super::*;
    use bowshock_model::Rocket;
    use bowshock_motors::MotorDb;
    use bowshock_sim::from_model::Phase;

    const NAMES: [&str; 4] = ["Alpha", "Bravo", "Charlie", "Delta"];

    /// The example with `configs` configurations named [`NAMES`] (copies of the first), the first active.
    fn example(configs: usize) -> Rocket {
        let mut rocket = Rocket::from_json(include_str!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        for _ in 1..configs {
            rocket.duplicate_configuration(rocket.active_config_id());
        }
        let ids: Vec<_> = rocket.configurations().iter().map(|c| c.id).collect();
        for (id, name) in ids.into_iter().zip(NAMES) {
            rocket.rename_configuration(id, name);
        }
        rocket
    }

    /// The plots of `analysis` for `rocket` (Extended method), the way the screen builds them.
    fn plots_of(rocket: &Rocket, analysis: Analysis, overlay: Option<&Overlay>) -> Built {
        let (jobs, _) = jobs(rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        let bands = rocket.stability_bands();
        let out = compute(&Request { analysis, method: Method::Extended, conditions: Conditions { runs: 8, ..Conditions::default() }, jobs, bands, cfd: None });
        build(&out, UnitSystem::MmGS, overlay, rocket.active_config_id())
    }

    fn built(analysis: Analysis, configs: usize) -> Built {
        plots_of(&example(configs), analysis, None)
    }

    /// An overlay of a made-up Cd (what an imported OpenRocket / RASAero file gives).
    fn imported_overlay() -> Overlay {
        let s = |x: f64, cd: f64| Sample { x, margin: Some(2.0), cp: Some(0.8), cd: Some(cd), cn_alpha: None, cn: None };
        Overlay { label: "OpenRocket".into(), mach: vec![s(0.1, 0.55), s(1.0, 0.7), s(3.0, 0.5)], alpha: Vec::new() }
    }

    /// The kit's stack: skin friction, pressure, base and fins as filled layers that add up to the total line on top, none empty.
    fn assert_stack(panel: &Panel) {
        let layers: Vec<&Series> = panel.series.iter().filter(|s| s.fill.is_some()).collect();
        let names: Vec<&str> = layers.iter().map(|s| s.name.as_str()).collect();
        assert!(names.len() >= 4 && names[..4] == ["Skin friction", "Pressure", "Base", "Fins"], "stacked layers: {names:?}");
        assert!(panel.is_stacked());
        for layer in &layers {
            assert!(layer.points.iter().any(|p| p.1 > 0.0), "{} is empty", layer.name);
        }
        let total = panel.series.iter().find(|s| s.name == "Cd").expect("the total line");
        assert_eq!(panel.series.iter().position(|s| s.name == "Cd"), Some(layers.len()), "the total follows the layers");
        for (k, &(mach, cd)) in total.points.iter().enumerate() {
            let sum: f64 = layers.iter().map(|s| s.points[k].1).sum();
            assert!((sum - cd).abs() < 1e-9, "the layers add up to the total at M {mach}: {sum} vs {cd}");
        }
    }

    /// The Cd panel of `b` (first of the Drag breakdown, last of Aerodynamics).
    fn cd_panel_of(b: &Built) -> &Panel {
        b.group.panels.iter().find(|p| p.title == "Drag coefficient Cd").expect("a Cd panel")
    }

    /// The lines drawn over the stack (everything that is neither a layer nor the total).
    fn lines_over(panel: &Panel) -> Vec<&Series> {
        panel.series.iter().filter(|s| s.fill.is_none() && s.name != "Cd").collect()
    }

    #[test]
    fn aerodynamics_has_the_three_kit_panels_with_threshold_lines() {
        let b = built(Analysis::Aerodynamics, 1);
        let titles: Vec<&str> = b.group.panels.iter().map(|p| p.title.as_str()).collect();
        assert_eq!(titles, ["Static margin", "CP and CG", "Drag coefficient Cd"]);
        assert_eq!(b.group.panels[0].hlines.len(), 2);
        assert_stack(&b.group.panels[2]);
        assert!(lines_over(&b.group.panels[2]).is_empty());
        assert!(b.legend.is_empty());
    }

    #[test]
    fn several_configurations_get_a_series_each_and_a_legend_and_keep_the_stack() {
        for configs in [2, 3] {
            let b = built(Analysis::Aerodynamics, configs);
            assert_eq!(b.legend.len(), configs);
            assert_eq!(b.group.panels[0].series.len(), configs);
            let cd = cd_panel_of(&b);
            assert_stack(cd);
            // The other configurations are totals on top, dashed, one each; the active one's total is the stack's line.
            let others = lines_over(cd);
            assert_eq!(others.len(), configs - 1, "{configs} configurations");
            assert!(others.iter().all(|s| s.style == LineStyle::Dashed && s.label == Some(LabelAt::Margin)));
        }
    }

    #[test]
    fn the_stack_is_the_active_configurations_and_the_others_are_lines() {
        let mut rocket = example(3);
        let bravo = rocket.configurations()[1].id;
        rocket.select_configuration(bravo);
        for analysis in [Analysis::Aerodynamics, Analysis::Drag] {
            let b = plots_of(&rocket, analysis, None);
            let cd = cd_panel_of(&b);
            assert_stack(cd);
            assert!(cd.unit.contains("Bravo"), "{analysis:?}: {}", cd.unit);
            let others: Vec<&str> = lines_over(cd).iter().map(|s| s.name.as_str()).collect();
            assert_eq!(others, ["Alpha", "Charlie"], "{analysis:?}");
            // Each keeps the colour of its legend entry (the configuration's place in the document, not in this panel).
            let colors: Vec<SeriesColor> = lines_over(cd).iter().map(|s| s.color).collect();
            assert_eq!(colors, [config_color(0), config_color(2)]);
        }
    }

    #[test]
    fn the_drag_breakdown_stacks_the_active_configuration_whatever_the_count() {
        for configs in [1, 3] {
            let b = built(Analysis::Drag, configs);
            let titles: Vec<&str> = b.group.panels.iter().map(|p| p.title.as_str()).collect();
            assert_eq!(titles, ["Drag coefficient Cd", "Share of drag"], "{configs} configurations");
            assert_stack(&b.group.panels[0]);
            // The shares are the active configuration's four sources, never one line per configuration.
            let shares: Vec<&str> = b.group.panels[1].series.iter().map(|s| s.name.as_str()).collect();
            assert!(shares.len() >= 4 && shares[..4] == ["Skin friction", "Pressure", "Base", "Fins"], "{shares:?}");
        }
    }

    #[test]
    fn a_comparison_is_a_dashed_line_over_the_stack_never_instead_of_it() {
        let overlay = imported_overlay();
        for (configs, analysis) in [(1, Analysis::Aerodynamics), (1, Analysis::Drag), (3, Analysis::Aerodynamics), (3, Analysis::Drag)] {
            let b = plots_of(&example(configs), analysis, Some(&overlay));
            let cd = cd_panel_of(&b);
            assert_stack(cd);
            let theirs = lines_over(cd)
                .into_iter()
                .find(|s| s.name == "OpenRocket")
                .unwrap_or_else(|| panic!("{configs} configurations, {analysis:?}: the comparison is drawn"));
            assert_eq!(theirs.style, LineStyle::Dashed);
            assert_eq!(theirs.points.len(), 3);
            assert_eq!(lines_over(cd).len(), configs, "the other configurations' totals and the comparison");
        }
    }

    #[test]
    fn an_imported_openrocket_design_has_the_stack_with_one_configuration_and_with_several() {
        let mut rocket = bowshock_model::ork::import(include_bytes!("../../../bowshock-model/tests/data/mini_two_stage.ork")).unwrap().rocket;
        // The fixture's first configuration flies a motor the bundled database lacks, so only the second can be analysed.
        let flyable = rocket.configurations()[1].id;
        rocket.select_configuration(flyable);
        let check = |rocket: &Rocket, flying: usize| {
            for analysis in [Analysis::Aerodynamics, Analysis::Drag] {
                let b = plots_of(rocket, analysis, None);
                let cd = cd_panel_of(&b);
                assert_stack(cd);
                assert_eq!(lines_over(cd).len(), flying - 1, "{analysis:?}");
            }
        };
        check(&rocket, 1);
        // A design with several motor set-ups: copies of the flyable one.
        for _ in 0..2 {
            rocket.duplicate_configuration(flyable);
        }
        check(&rocket, 3);
    }

    #[test]
    fn other_analyses_build() {
        for a in [Analysis::Drag, Analysis::Stability, Analysis::Flutter, Analysis::Dispersion] {
            let b = built(a, 1);
            assert!(b.group.panels.iter().all(|p| !p.series.is_empty()), "{a:?}");
        }
        assert!(built(Analysis::Dispersion, 1).ground.is_some());
    }
}
