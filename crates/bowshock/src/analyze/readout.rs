//! The right panel's "values at cursor" as data: sections of rows built from an [`Output`], the cursor position and the
//! unit system, so the numbers can be tested without a UI. `view.rs` draws them.

use bowshock_model::ConfigId;
use bowshock_sim::MachPoint;

use super::compute::{alpha_at, interp, mach_at, Analysis, Data, Output, Run, FLUTTER_MIN_MARGIN, MIN_MARGIN, OVER_STABLE};
use crate::app::ui;
use crate::units::{Quantity, UnitSystem};
use crate::widgets::chip::Severity;
use crate::widgets::gauge::{Bands, Stability};
use crate::widgets::plot::SeriesColor;

/// What sits left of a row's value.
#[derive(Debug, Clone, PartialEq)]
pub enum Tag {
    /// A chip (`✓ M 0.1`).
    Chip(Severity, String),
    /// Muted mono text (`M 1.08`).
    Plain(String),
}

/// One `label … value unit` row.
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    /// The label.
    pub label: String,
    /// The value, formatted.
    pub value: String,
    /// The unit after the value (may be empty).
    pub unit: String,
    /// A colour square before the label (series colour keys).
    pub swatch: Option<SeriesColor>,
    /// A chip or note between label and value.
    pub tag: Option<Tag>,
    /// The headline row of a section (20 px readout).
    pub big: bool,
}

impl Row {
    fn new(label: impl Into<String>, value: impl Into<String>, unit: impl Into<String>) -> Self {
        Row { label: label.into(), value: value.into(), unit: unit.into(), swatch: None, tag: None, big: false }
    }

    fn big(mut self) -> Self {
        self.big = true;
        self
    }

    fn swatch(mut self, color: SeriesColor) -> Self {
        self.swatch = Some(color);
        self
    }

    fn tag(mut self, tag: Tag) -> Self {
        self.tag = Some(tag);
        self
    }
}

/// A numbered section of rows (`01 STABILITY`).
#[derive(Debug, Clone, PartialEq)]
pub struct Section {
    /// The title (shown upper-case).
    pub title: String,
    /// Its rows.
    pub rows: Vec<Row>,
}

/// The whole right panel.
#[derive(Debug, Clone, PartialEq)]
pub struct Readout {
    /// The cursor position in `accent` next to the heading (`M 1.10`).
    pub cursor: String,
    /// The sections.
    pub sections: Vec<Section>,
}

/// The colour of the `i`-th configuration (the four categorical slots, in order).
pub fn config_color(i: usize) -> SeriesColor {
    SeriesColor::Series(i.min(3) as u8 + 1)
}

/// The colour keys of a drag breakdown, in stacking order.
pub const DRAG_PARTS: [(&str, SeriesColor); 4] =
    [("Skin friction", SeriesColor::Series(1)), ("Pressure", SeriesColor::Series(2)), ("Base", SeriesColor::Series(3)), ("Fins", SeriesColor::Series(4))];

fn severity(margin: f64) -> Severity {
    match (Bands { marginal: 0.5, stable: MIN_MARGIN as f32, max: OVER_STABLE as f32 }).classify(margin as f32) {
        Stability::Unstable => Severity::Err,
        Stability::Marginal | Stability::OverStable => Severity::Warn,
        Stability::Stable => Severity::Ok,
    }
}

fn length(m: f64, units: UnitSystem) -> (String, String) {
    let (v, u) = ui::readout(m, Quantity::Length, units);
    (v, u.to_string())
}

fn cal(v: f64) -> String {
    format!("{v:.2}")
}

fn cd(v: f64) -> String {
    format!("{v:.3}")
}

/// The run to show values of: the active configuration's, else the first.
pub fn active_run(out: &Output, active: ConfigId) -> Option<&Run> {
    out.runs.iter().find(|r| r.id == active).or_else(|| out.runs.first())
}

/// The default cursor when nothing is hovered: where the drag peaks (Mach), zero angle, the altitude of maximum velocity.
pub fn default_cursor(out: &Output, active: ConfigId) -> f64 {
    let Some(run) = active_run(out, active) else { return 0.0 };
    match &run.data {
        Data::Mach(p) => p.iter().max_by(|a, b| a.drag.total.total_cmp(&b.drag.total)).map_or(0.0, |p| p.mach),
        Data::Alpha(_) => 0.0,
        Data::Flutter(c) => c.as_ref().map_or(0.0, |c| c.max_velocity_altitude),
        Data::Dispersion(_) => 0.0,
    }
}

/// The static margin (cal) and its length (m) at the cursor of the active run, for the gauge; `None` for analyses without one.
pub fn margin_at(out: &Output, x_si: Option<f64>, active: ConfigId) -> Option<(f64, f64)> {
    let run = active_run(out, active)?;
    let x = x_si.unwrap_or_else(|| default_cursor(out, active));
    let cal = match &run.data {
        Data::Mach(p) => mach_at(p, x)?.margin_cal,
        Data::Alpha(p) => alpha_at(p, x)?.margin_cal,
        _ => return None,
    };
    Some((cal, cal * run.reference_diameter))
}

/// Builds the right panel for `out` at `x_si` (Mach, radians of angle of attack, or metres of altitude by analysis; `None` =
/// the default cursor), for configuration `active`, in `units`.
pub fn readout(out: &Output, x_si: Option<f64>, active: ConfigId, units: UnitSystem) -> Readout {
    let Some(run) = active_run(out, active) else { return Readout { cursor: String::new(), sections: Vec::new() } };
    let x = x_si.unwrap_or_else(|| default_cursor(out, active));
    match &run.data {
        Data::Mach(points) => mach(out, run, points, x, units),
        Data::Alpha(_) => alpha(out, run, x, units),
        Data::Flutter(curve) => flutter(run, curve.as_ref(), x, units),
        Data::Dispersion(d) => dispersion(run, d.as_ref(), units),
    }
}

fn stability_rows(run: &Run, p: &MachPointLike, units: UnitSystem, out: &Output) -> Vec<Row> {
    let (cp, u) = length(p.cp, units);
    let (cg_l, _) = length(run.cg_liftoff, units);
    let (cg_b, _) = length(run.cg_burnout, units);
    let mut rows = vec![Row::new("Static margin", cal(p.margin), "cal").big()];
    if out.runs.len() > 1 {
        for (i, r) in out.runs.iter().enumerate() {
            if let Some(v) = per_run_margin(out, r, p.x) {
                rows.push(Row::new(r.name.clone(), cal(v), "cal").swatch(config_color(i)));
            }
        }
    }
    rows.push(Row::new("CP", cp, u.clone()));
    rows.push(Row::new("CG liftoff", cg_l, u.clone()));
    rows.push(Row::new("CG burnout", cg_b, u));
    rows.push(Row::new("CNα", format!("{:.2}", p.cn_alpha), "/rad"));
    rows
}

struct MachPointLike {
    x: f64,
    margin: f64,
    cp: f64,
    cn_alpha: f64,
}

fn per_run_margin(out: &Output, run: &Run, x: f64) -> Option<f64> {
    let _ = out;
    match &run.data {
        Data::Mach(p) => Some(mach_at(p, x)?.margin_cal),
        Data::Alpha(p) => Some(alpha_at(p, x)?.margin_cal),
        _ => None,
    }
}

fn mach(out: &Output, run: &Run, points: &[MachPoint], x: f64, units: UnitSystem) -> Readout {
    let Some(at) = mach_at(points, x) else { return Readout { cursor: String::new(), sections: Vec::new() } };
    let like = MachPointLike { x, margin: at.margin_cal, cp: at.cp, cn_alpha: at.cn_alpha };
    let stability = Section { title: "Stability".into(), rows: stability_rows(run, &like, units, out) };

    let d = at.drag;
    let mut drag = vec![Row::new("Total Cd", cd(d.total), "").big()];
    for ((name, color), v) in DRAG_PARTS.iter().zip([d.friction, d.pressure, d.base, d.fins]) {
        drag.push(Row::new(*name, cd(v), "").swatch(*color));
    }
    if points.iter().any(|p| p.drag.protuberances > 1e-9) {
        drag.push(Row::new("Other", cd(d.protuberances), "").swatch(SeriesColor::Muted));
    }

    let by_margin = |a: &&MachPoint, b: &&MachPoint| a.margin_cal.total_cmp(&b.margin_cal);
    let (lo, hi) = (points.iter().min_by(by_margin), points.iter().max_by(by_margin));
    let peak = points.iter().max_by(|a, b| a.drag.total.total_cmp(&b.drag.total));
    let mut worst = Vec::new();
    if let Some(p) = lo {
        worst.push(Row::new("Min margin", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(p.margin_cal), format!("M {:.1}", p.mach))));
    }
    if let Some(p) = hi {
        worst.push(Row::new("Max margin", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(p.margin_cal), format!("M {:.1}", p.mach))));
    }
    if let Some(p) = peak {
        worst.push(Row::new("Peak Cd", cd(p.drag.total), "").tag(Tag::Plain(format!("M {:.2}", p.mach))));
    }
    let (m0, m1) = (points.first().map_or(0.0, |p| p.mach), points.last().map_or(0.0, |p| p.mach));
    let drag = Section { title: "Drag".into(), rows: drag };
    let worst = Section { title: format!("Worst case {m0:.1} – {m1:.1}"), rows: worst };
    // The drag breakdown analysis reads drag first; the others read stability first.
    let sections = if out.analysis == Analysis::Drag { vec![drag, stability, worst] } else { vec![stability, drag, worst] };
    Readout { cursor: format!("M {x:.2}"), sections }
}

fn alpha(out: &Output, run: &Run, alpha_rad: f64, units: UnitSystem) -> Readout {
    let Data::Alpha(points) = &run.data else { unreachable!("called for an angle sweep") };
    let Some(at) = alpha_at(points, alpha_rad) else { return Readout { cursor: String::new(), sections: Vec::new() } };
    let like = MachPointLike { x: alpha_rad, margin: at.margin_cal, cp: at.cp, cn_alpha: at.cn_alpha };
    let stability = Section { title: "Stability".into(), rows: stability_rows(run, &like, units, out) };
    let force = Section {
        title: "Force".into(),
        rows: vec![Row::new("Normal force CN", format!("{:.3}", at.cn), "").big(), Row::new("Total Cd", cd(at.drag.total), "")],
    };
    let by_margin = |a: &&bowshock_sim::AlphaPoint, b: &&bowshock_sim::AlphaPoint| a.margin_cal.total_cmp(&b.margin_cal);
    let mut worst = Vec::new();
    if let Some(p) = points.iter().min_by(by_margin) {
        worst.push(Row::new("Min margin", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(p.margin_cal), format!("{:.1}°", p.alpha.to_degrees()))));
    }
    if let Some(p) = points.iter().max_by(by_margin) {
        worst.push(Row::new("Max margin", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(p.margin_cal), format!("{:.1}°", p.alpha.to_degrees()))));
    }
    if let Some(p) = points.iter().max_by(|a, b| a.cn.total_cmp(&b.cn)) {
        worst.push(Row::new("Peak CN", format!("{:.3}", p.cn), "").tag(Tag::Plain(format!("{:.1}°", p.alpha.to_degrees()))));
    }
    let top = points.last().map_or(0.0, |p| p.alpha.to_degrees());
    Readout {
        cursor: format!("α {:.1}°", alpha_rad.to_degrees()),
        sections: vec![stability, force, Section { title: format!("Worst case 0 – {top:.0}°"), rows: worst }],
    }
}

fn flutter(run: &Run, curve: Option<&super::compute::FlutterCurve>, h: f64, units: UnitSystem) -> Readout {
    let alt = |m: f64| {
        let (v, u) = ui::readout(m, Quantity::Altitude, units);
        (v, u.to_string())
    };
    let vel = |v: f64| {
        let (n, u) = ui::readout(v, Quantity::Velocity, units);
        (n, u.to_string())
    };
    let Some(curve) = curve else {
        return Readout { cursor: String::new(), sections: vec![Section { title: "Flutter".into(), rows: vec![Row::new("No fins", "—", "")] }] };
    };
    let (ha, hu) = alt(h);
    let (vf, vu) = vel(interp(&curve.velocity, h));
    let (vmax, _) = vel(curve.max_velocity);
    let (at_alt, au) = alt(curve.max_velocity_altitude);
    let (apogee, apu) = alt(curve.apogee);
    let margin = curve.margin_at(curve.max_velocity_altitude);
    let sev = if margin >= FLUTTER_MIN_MARGIN {
        Severity::Ok
    } else if margin >= 1.0 {
        Severity::Warn
    } else {
        Severity::Err
    };
    let local = curve.margin_at(h);
    Readout {
        cursor: format!("{ha} {hu}"),
        sections: vec![
            Section {
                title: format!("Flutter · {}", run.name),
                rows: vec![
                    Row::new("Flutter velocity", vf, vu.clone()).big(),
                    Row::new("Margin here", if local.is_finite() { format!("{local:.2}") } else { "—".into() }, "× Vmax"),
                    Row::new("Max velocity", vmax, vu),
                    Row::new("at altitude", at_alt, au),
                ],
            },
            Section {
                title: "Worst case".into(),
                rows: vec![
                    Row::new("Margin at Vmax", if margin.is_finite() { format!("{margin:.2}") } else { "—".into() }, "× Vmax")
                        .tag(Tag::Chip(sev, format!("min {FLUTTER_MIN_MARGIN}"))),
                    Row::new("Apogee", apogee, apu),
                ],
            },
        ],
    }
}

fn dispersion(run: &Run, d: Option<&super::compute::Scatter>, units: UnitSystem) -> Readout {
    let Some(d) = d else {
        return Readout { cursor: String::new(), sections: vec![Section { title: "Dispersion".into(), rows: vec![Row::new("No motor", "—", "")] }] };
    };
    let len = |m: f64| {
        let (v, u) = ui::readout(m, Quantity::Altitude, units);
        (v, u.to_string())
    };
    let [e, n] = d.stats.landing_mean;
    let dist = e.hypot(n);
    let bearing = n.atan2(e).to_degrees();
    let bearing = (90.0 - bearing).rem_euclid(360.0);
    let (dv, du) = len(dist);
    let (cep, cu) = len(d.stats.cep50);
    let (major, mu) = len(d.stats.ellipse_major);
    let (minor, _) = len(d.stats.ellipse_minor);
    let (am, au) = len(d.stats.apogee_mean);
    let (asd, _) = len(d.stats.apogee_std);
    let (amin, _) = len(d.stats.apogee_min);
    let (amax, _) = len(d.stats.apogee_max);
    let (sde, _) = len(d.stats.landing_std[0]);
    let (sdn, _) = len(d.stats.landing_std[1]);
    Readout {
        cursor: format!("{} runs", d.landings.len()),
        sections: vec![
            Section {
                title: format!("Landing · {}", run.name),
                rows: vec![
                    Row::new("Mean distance", dv, du).big(),
                    Row::new("Bearing", format!("{bearing:.0}"), "°"),
                    Row::new("CEP 50 %", cep, cu),
                    Row::new("Ellipse major", major, mu.clone()),
                    Row::new("Ellipse minor", minor, mu),
                    Row::new("σ east / north", format!("{sde} / {sdn}"), ""),
                ],
            },
            Section {
                title: "Apogee".into(),
                rows: vec![
                    Row::new("Mean", am, au.clone()).big(),
                    Row::new("σ", asd, au.clone()),
                    Row::new("Min", amin, au.clone()),
                    Row::new("Max", amax, au),
                ],
            },
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::super::compute::{compute, jobs, Conditions, Method, Request};
    use super::*;
    use bowshock_model::Rocket;
    use bowshock_motors::MotorDb;
    use bowshock_sim::from_model::Phase;

    fn out(analysis: Analysis) -> (Output, ConfigId) {
        let rocket = Rocket::from_json(include_str!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        let (jobs, _) = jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        (
            compute(&Request { analysis, method: Method::Extended, conditions: Conditions { runs: 10, ..Conditions::default() }, jobs }),
            rocket.active_config_id(),
        )
    }

    #[test]
    fn aerodynamics_readout_has_the_kit_sections() {
        let (o, id) = out(Analysis::Aerodynamics);
        let r = readout(&o, Some(1.1), id, UnitSystem::MmGS);
        assert_eq!(r.cursor, "M 1.10");
        let titles: Vec<&str> = r.sections.iter().map(|s| s.title.as_str()).collect();
        assert_eq!(titles, ["Stability", "Drag", "Worst case 0.1 – 3.0"]);
        let margin: f64 = r.sections[0].rows[0].value.parse().unwrap();
        assert_eq!(Some(margin), margin_at(&o, Some(1.1), id).map(|m| (m.0 * 100.0).round() / 100.0));
        // The four drag parts sum to the total (to the displayed precision).
        let rows = &r.sections[1].rows;
        let sum: f64 = rows[1..].iter().map(|r| r.value.parse::<f64>().unwrap()).sum();
        let total: f64 = rows[0].value.parse().unwrap();
        assert!((sum - total).abs() < 0.0035, "{sum} vs {total}");
        assert!(r.sections[2].rows.iter().any(|row| matches!(&row.tag, Some(Tag::Chip(..)))));
    }

    #[test]
    fn default_cursor_is_the_drag_peak() {
        let (o, id) = out(Analysis::Aerodynamics);
        let x = default_cursor(&o, id);
        assert!(x > 0.8 && x < 1.6);
        assert_eq!(readout(&o, None, id, UnitSystem::MmGS).cursor, format!("M {x:.2}"));
    }

    #[test]
    fn other_analyses_have_readouts() {
        for a in [Analysis::Stability, Analysis::Flutter, Analysis::Dispersion] {
            let (o, id) = out(a);
            let r = readout(&o, None, id, UnitSystem::MmGS);
            assert!(!r.sections.is_empty() && r.sections.iter().all(|s| !s.rows.is_empty()), "{a:?}");
        }
    }
}
