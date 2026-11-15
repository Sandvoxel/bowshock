//! The right panel's "values at cursor" as data: sections of rows built from an [`Output`], the cursor position and the
//! unit system, so the numbers can be tested without a UI. `view.rs` draws them.

use bowshock_model::{ConfigId, StabilityBands};
use bowshock_sim::MachPoint;

use super::compute::{alpha_at, interp, mach_at, Analysis, Data, Output, Run, FLUTTER_MIN_MARGIN};
use super::overlay::{Field, Overlay};
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
    /// A one-sentence explanation, shown as the row's tooltip.
    pub help: Option<&'static str>,
    /// The row's share of the section's total (`44%`), muted before the value (drag sources).
    pub share: Option<String>,
    /// A secondary row: label and value in muted ink (the full-range extreme beside the in-flight worst case).
    pub muted: bool,
}

impl Row {
    fn new(label: impl Into<String>, value: impl Into<String>, unit: impl Into<String>) -> Self {
        let label = label.into();
        let help = help(&label);
        Row { label, value: value.into(), unit: unit.into(), swatch: None, tag: None, big: false, help, share: None, muted: false }
    }

    fn share(mut self, of_total: f64) -> Self {
        self.share = Some(format!("{:.0}%", 100.0 * of_total));
        self
    }

    fn muted(mut self) -> Self {
        self.muted = true;
        self
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

/// One plain sentence for a row label: what the quantity is and what moves it. Shown as the row's tooltip and, for the drag
/// sources, under the Drag breakdown plot. `None` for labels that need no explanation (names of configurations, units).
pub fn help(label: &str) -> Option<&'static str> {
    Some(match label {
        "Skin friction" => "Air rubbing along every outer surface. Grows with wetted area and a rougher finish.",
        "Pressure" => "Drag from the nose and transitions pushing air aside; rises sharply near Mach 1 as shocks form. Includes lugs and rail buttons.",
        "Base" => "Low-pressure wake behind the blunt aft end. Smaller while the motor burns (the exhaust fills the wake).",
        "Fins" => "The fins' own skin friction plus leading- and trailing-edge pressure drag.",
        "Other" => "Launch lugs and rail buttons sticking out into the airflow.",
        "Total Cd" => "Total drag coefficient: drag force divided by dynamic pressure and reference area. Lower is better.",
        "Static margin" => {
            "Distance from the centre of gravity back to the centre of pressure, in body diameters (calibers). Above about 1 the rocket flies straight."
        }
        "CP" => "Centre of pressure: where the air's side force acts, measured from the nose tip. It must sit behind the CG.",
        "CG liftoff" => "Centre of gravity from the nose tip with the motors loaded.",
        "CG burnout" => "Centre of gravity from the nose tip once the motors have burned out.",
        "CNα" => "How much side force the rocket makes per radian of angle of attack; higher = stronger restoring push.",
        "Min margin" => "The least stable point of the sweep, and the Mach (or angle) where it happens. The chip's colour is the stability band: green stable, amber marginal, red unstable or over-stable.",
        "full range" => "The least stable point of the whole sweep. Part of it lies beyond the Mach this configuration actually reaches in flight, so it is shown for reference only.",
        "Max margin" => "The most stable point of the sweep. Above about 3 calibers the rocket may weathercock into the wind.",
        "Peak Cd" => "The highest drag coefficient of the sweep, usually just above Mach 1 where the transonic drag rise peaks.",
        "Normal force CN" => "Side-force coefficient at this angle of attack (CNα times the angle, plus body lift).",
        "Peak CN" => "The largest side-force coefficient of the sweep, at the largest angle.",
        "Flutter velocity" => "Speed at which the fins would start to flutter at this altitude (NACA TN 4197).",
        "Margin here" | "Margin at Vmax" => "Flutter velocity divided by the rocket's speed. Keep it above 1.5.",
        "Max velocity" => "The highest speed of the configuration's nominal flight.",
        "CEP 50 %" => "Radius of the circle around the mean landing point that holds half of the landings.",
        "Ellipse major" => "Long axis of the ellipse that holds 95 % of the landings.",
        "Ellipse minor" => "Short axis of the ellipse that holds 95 % of the landings.",
        _ => return None,
    })
}

fn severity(bands: StabilityBands, margin: f64) -> Severity {
    match Bands::from(bands).classify(margin as f32) {
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

/// The Mach number the cursor starts at: the maximum Mach of the configuration's nominal flight when it has one inside the
/// sweep, else Mach 0.3; zero angle of attack; the altitude of maximum velocity.
pub fn default_cursor(out: &Output, active: ConfigId) -> f64 {
    let Some(run) = active_run(out, active) else { return 0.0 };
    match &run.data {
        Data::Mach(p) => {
            let (lo, hi) = (p.first().map_or(0.0, |p| p.mach), p.last().map_or(0.0, |p| p.mach));
            run.max_mach.unwrap_or(super::compute::BALANCE_MACH).clamp(lo, hi.max(lo))
        }
        Data::Alpha(_) => 0.0,
        Data::Flutter(c) => c.as_ref().map_or(0.0, |c| c.max_velocity_altitude),
        Data::Dispersion(_) => 0.0,
    }
}

/// Whether the drag breakdown shows protuberances (launch lugs, rail buttons) as their own `Other` layer: only when they are
/// more than 5 % of the drag summed over the sweep. Below that they are folded into Pressure, as the kit's four-part
/// breakdown has it.
pub fn other_visible(points: &[MachPoint]) -> bool {
    let (other, total) = points.iter().fold((0.0, 0.0), |(o, t), p| (o + p.drag.protuberances, t + p.drag.total));
    other > 0.05 * total
}

/// Skin friction, pressure, base and fins of `d`, with the protuberances added to pressure unless `show_other`.
pub fn drag_parts(d: &bowshock_sim::DragBreakdown, show_other: bool) -> [f64; 4] {
    [d.friction, if show_other { d.pressure } else { d.pressure + d.protuberances }, d.base, d.fins]
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

/// The "Compare" section: the comparison's value of each quantity it has at the cursor, with the difference to ours
/// (`theirs − ours`) next to it. `None` for analyses without an overlay or when nothing is shared.
pub fn compare_section(out: &Output, overlay: &Overlay, x_si: Option<f64>, active: ConfigId, units: UnitSystem) -> Option<Section> {
    let run = active_run(out, active)?;
    let x = x_si.unwrap_or_else(|| default_cursor(out, active));
    let (margin, cp, cd_total, cn) = match &run.data {
        Data::Mach(p) => {
            let a = mach_at(p, x)?;
            (a.margin_cal, a.cp, a.drag.total, None)
        }
        Data::Alpha(p) => {
            let a = alpha_at(p, x)?;
            (a.margin_cal, a.cp, a.drag.total, Some(a.cn))
        }
        _ => return None,
    };
    let an = out.analysis;
    let sign = |d: f64| if d < 0.0 { '−' } else { '+' };
    let color = SeriesColor::Series(2);
    let mut rows = Vec::new();
    if let Some(t) = overlay.at(an, Field::Margin, x) {
        let d = t - margin;
        rows.push(Row::new("Margin", cal(t), "cal").swatch(color).tag(Tag::Plain(format!("Δ {}{:.2} · ours {}", sign(d), d.abs(), cal(margin)))));
    }
    if let Some(t) = overlay.at(an, Field::Cp, x) {
        let d = t - cp;
        let (v, u) = length(t, units);
        let (dv, _) = length(d.abs(), units);
        let (mine, _) = length(cp, units);
        rows.push(Row::new("CP", v, u).swatch(color).tag(Tag::Plain(format!("Δ {}{dv} · ours {mine}", sign(d)))));
    }
    if let Some(t) = overlay.at(an, Field::Cd, x) {
        let d = t - cd_total;
        rows.push(Row::new("Cd", cd(t), "").swatch(color).tag(Tag::Plain(format!("Δ {}{:.3} · ours {}", sign(d), d.abs(), cd(cd_total)))));
    }
    if let (Some(t), Some(mine)) = (overlay.at(an, Field::Cn, x), cn) {
        let d = t - mine;
        rows.push(Row::new("CN", format!("{t:.3}"), "").swatch(color).tag(Tag::Plain(format!("Δ {}{:.3} · ours {mine:.3}", sign(d), d.abs()))));
    }
    (!rows.is_empty()).then(|| Section { title: format!("Compare · {}", overlay.label), rows })
}

fn stability_rows(run: &Run, p: &MachPointLike, units: UnitSystem, out: &Output) -> Vec<Row> {
    let (cp, u) = length(p.cp, units);
    let burnout = out.conditions.burnout;
    let (cg, _) = length(run.cg(burnout), units);
    let mut rows = vec![Row::new("Static margin", cal(p.margin), "cal").big()];
    if out.runs.len() > 1 {
        for (i, r) in out.runs.iter().enumerate() {
            if let Some(v) = per_run_margin(r, p.x) {
                rows.push(Row::new(r.name.clone(), cal(v), "cal").swatch(config_color(i)));
            }
        }
    }
    rows.push(Row::new("CP", cp, u.clone()));
    // The kit shows one CG: the one the margin uses (the Conditions' "CG for margin").
    rows.push(Row::new(if burnout { "CG burnout" } else { "CG liftoff" }, cg, u));
    rows.push(Row::new("CNα", format!("{:.2}", p.cn_alpha), "/rad"));
    rows
}

struct MachPointLike {
    x: f64,
    margin: f64,
    cp: f64,
    cn_alpha: f64,
}

fn per_run_margin(run: &Run, x: f64) -> Option<f64> {
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
    let show_other = other_visible(points);
    let mut drag = vec![Row::new("Total Cd", cd(d.total), "").big()];
    for ((name, color), v) in DRAG_PARTS.iter().zip(drag_parts(&d, show_other)) {
        drag.push(Row::new(*name, cd(v), "").swatch(*color).share(v / d.total.max(1e-12)));
    }
    if show_other {
        drag.push(Row::new("Other", cd(d.protuberances), "").swatch(SeriesColor::Muted).share(d.protuberances / d.total.max(1e-12)));
    }

    let by_margin = |a: &&MachPoint, b: &&MachPoint| a.margin_cal.total_cmp(&b.margin_cal);
    let (m0, m1) = (points.first().map_or(0.0, |p| p.mach), points.last().map_or(0.0, |p| p.mach));
    // The worst case is judged where the rocket actually flies: up to the nominal flight's maximum Mach, when it has one inside
    // the sweep. The full-range extreme follows as a secondary muted row.
    let envelope = run.max_mach.filter(|m| *m > m0 && *m < m1);
    let flown: Vec<&MachPoint> = points.iter().filter(|p| envelope.is_none_or(|m| p.mach <= m + 1e-9)).collect();
    let (lo, hi) = (flown.iter().copied().min_by(by_margin), flown.iter().copied().max_by(by_margin));
    let peak = flown.iter().copied().max_by(|a, b| a.drag.total.total_cmp(&b.drag.total));
    let mut worst = Vec::new();
    if let Some(p) = lo {
        worst.push(Row::new("Min margin", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(out.bands, p.margin_cal), format!("M {:.1}", p.mach))));
    }
    if let Some(p) = hi {
        worst.push(Row::new("Max margin", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(out.bands, p.margin_cal), format!("M {:.1}", p.mach))));
    }
    if let Some(p) = peak {
        worst.push(Row::new("Peak Cd", cd(p.drag.total), "").tag(Tag::Plain(format!("M {:.2}", p.mach))));
    }
    let title = match envelope {
        Some(m) => {
            let first = flown.first().map_or(m0, |p| p.mach);
            if let Some(p) = points.iter().min_by(by_margin) {
                if lo.is_some_and(|l| (l.mach - p.mach).abs() > 1e-9) {
                    worst.push(
                        Row::new("full range", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(out.bands, p.margin_cal), format!("M {:.1}", p.mach))).muted(),
                    );
                }
            }
            format!("Worst case · in flight {first:.1} – {m:.2}")
        }
        None => format!("Worst case {m0:.1} – {m1:.1}"),
    };
    let drag = Section { title: "Drag".into(), rows: drag };
    let worst = Section { title, rows: worst };
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
        worst.push(Row::new("Min margin", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(out.bands, p.margin_cal), format!("{:.1}°", p.alpha.to_degrees()))));
    }
    if let Some(p) = points.iter().max_by(by_margin) {
        worst.push(Row::new("Max margin", cal(p.margin_cal), "cal").tag(Tag::Chip(severity(out.bands, p.margin_cal), format!("{:.1}°", p.alpha.to_degrees()))));
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
            compute(&Request {
                analysis,
                method: Method::Extended,
                conditions: Conditions { runs: 10, ..Conditions::default() },
                jobs,
                bands: rocket.stability_bands(),
                cfd: None,
            }),
            rocket.active_config_id(),
        )
    }

    #[test]
    fn aerodynamics_readout_has_the_kit_sections() {
        let (o, id) = out(Analysis::Aerodynamics);
        let r = readout(&o, Some(1.1), id, UnitSystem::MmGS);
        assert_eq!(r.cursor, "M 1.10");
        let titles: Vec<&str> = r.sections.iter().map(|s| s.title.as_str()).collect();
        let flight_mach = active_run(&o, id).unwrap().max_mach.expect("the example flies");
        assert_eq!(titles, ["Stability", "Drag", &format!("Worst case · in flight 0.1 – {flight_mach:.2}")]);
        let margin: f64 = r.sections[0].rows[0].value.parse().unwrap();
        let at = mach_at(
            match &active_run(&o, id).unwrap().data {
                Data::Mach(p) => p,
                _ => unreachable!(),
            },
            1.1,
        )
        .unwrap();
        assert_eq!(margin, (at.margin_cal * 100.0).round() / 100.0);
        // The four drag parts sum to the total (to the displayed precision).
        let rows = &r.sections[1].rows;
        let sum: f64 = rows[1..].iter().map(|r| r.value.parse::<f64>().unwrap()).sum();
        let total: f64 = rows[0].value.parse().unwrap();
        assert!((sum - total).abs() < 0.0035, "{sum} vs {total}");
        assert!(r.sections[2].rows.iter().any(|row| matches!(&row.tag, Some(Tag::Chip(..)))));
        // The worst case is judged within the flight envelope; the full-range minimum follows as a secondary muted row.
        let worst = &r.sections[2];
        let flight_mach = active_run(&o, id).unwrap().max_mach.expect("the example flies");
        assert_eq!(worst.title, format!("Worst case · in flight 0.1 – {flight_mach:.2}"));
        let full = worst.rows.last().unwrap();
        assert!(full.muted && full.label == "full range");
        let min_in_flight: f64 = worst.rows[0].value.parse().unwrap();
        let min_full: f64 = full.value.parse().unwrap();
        assert!(min_full <= min_in_flight, "{min_full} vs {min_in_flight}");
    }

    #[test]
    fn the_cursor_starts_at_the_flights_maximum_mach() {
        let (o, id) = out(Analysis::Aerodynamics);
        let run = active_run(&o, id).unwrap();
        let flight_mach = run.max_mach.expect("the example flies");
        assert!(flight_mach > 0.2 && flight_mach < 1.0, "{flight_mach}");
        let x = default_cursor(&o, id);
        assert_eq!(x, flight_mach);
        assert_eq!(readout(&o, None, id, UnitSystem::MmGS).cursor, format!("M {x:.2}"));
    }

    #[test]
    fn protuberances_fold_into_pressure_unless_they_are_a_twentieth_of_the_drag() {
        let p = |prot: f64| bowshock_sim::MachPoint {
            mach: 1.0,
            margin_cal: 1.0,
            cp: 0.5,
            cn_alpha: 10.0,
            drag: bowshock_sim::DragBreakdown { friction: 0.2, pressure: 0.2, base: 0.1, fins: 0.1, protuberances: prot, total: 0.6 + prot },
        };
        assert!(!other_visible(&[p(0.02)]) && other_visible(&[p(0.06)]));
        let d = p(0.02).drag;
        assert_eq!(drag_parts(&d, false), [0.2, 0.22, 0.1, 0.1]);
        assert_eq!(drag_parts(&d, true), [0.2, 0.2, 0.1, 0.1]);
    }

    #[test]
    fn every_readout_row_explains_itself() {
        for analysis in [Analysis::Aerodynamics, Analysis::Drag, Analysis::Stability, Analysis::Flutter, Analysis::Dispersion] {
            let (o, id) = out(analysis);
            for section in readout(&o, None, id, UnitSystem::MmGS).sections {
                for row in section.rows {
                    // Rows that are plain measurements (altitude, apogee, spreads, means) need no sentence; the rest do.
                    let plain = ["Apogee", "at altitude", "Mean distance", "Bearing", "σ east / north", "Mean", "σ", "Min", "Max"];
                    assert!(row.help.is_some() || plain.contains(&row.label.as_str()), "{analysis:?}: `{}` has no explanation", row.label);
                }
            }
        }
    }

    #[test]
    fn drag_rows_carry_their_share_and_the_four_sources_have_the_kit_sentences() {
        let (o, id) = out(Analysis::Aerodynamics);
        let r = readout(&o, Some(1.1), id, UnitSystem::MmGS);
        let drag = &r.sections[1];
        let shares: f64 = drag.rows[1..].iter().map(|row| row.share.as_ref().unwrap().trim_end_matches('%').parse::<f64>().unwrap()).sum();
        assert!((shares - 100.0).abs() <= 2.0, "shares of the four sources sum to the total: {shares}");
        for (name, _) in DRAG_PARTS {
            assert!(help(name).unwrap().len() > 30, "{name}");
        }
        assert!(help("Pressure").unwrap().contains("Mach 1") && help("Base").unwrap().contains("exhaust"));
    }

    #[test]
    fn comparing_a_result_with_itself_has_zero_differences() {
        let (o, id) = out(Analysis::Aerodynamics);
        let overlay = Overlay::from_output(&o, id, "Same").unwrap();
        let s = compare_section(&o, &overlay, Some(1.1), id, UnitSystem::MmGS).unwrap();
        assert_eq!(s.title, "Compare · Same");
        let labels: Vec<&str> = s.rows.iter().map(|r| r.label.as_str()).collect();
        assert_eq!(labels, ["Margin", "CP", "Cd"]);
        assert!(s.rows.iter().all(|r| matches!(&r.tag, Some(Tag::Plain(t)) if t.starts_with("Δ +0") || t.starts_with("Δ −0"))));
        // Outside the comparison's data there is nothing to compare.
        assert!(compare_section(&o, &overlay, Some(9.0), id, UnitSystem::MmGS).is_none());
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
