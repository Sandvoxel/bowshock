//! The right panel's "values at cursor" as data: sections of rows built from an [`Output`], the cursor position and the
//! unit system, so the numbers can be tested without a UI. `view.rs` draws them.

use bowshock_model::{ConfigId, StabilityBands};
use bowshock_sim::flutter::FlutterWarning;
use bowshock_sim::MachPoint;

use super::compute::{alpha_at, mach_at, Analysis, Data, Output, Run, FLUTTER_MIN_MARGIN};
use super::flutter::FlutterAnalysis;
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

/// One plain sentence for a row label: what the quantity is and what moves it, shown as the row's tooltip (the drag sources'
/// sentences live here and nowhere else). `None` for labels that need no explanation (names of configurations, units).
pub fn help(label: &str) -> Option<&'static str> {
    let label = label.strip_suffix(super::cfd::LIFT_LABEL).unwrap_or(label);
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
        "Lowest margin" => {
            "Flutter speed divided by the rocket's speed, at the worst moment of the flight up to apogee. 1.0 is the estimated flutter onset; keep it above 1.25."
        }
        "Margin here" => "Flutter speed divided by the rocket's speed at the cursor.",
        "Flutter speed V_f" => "The speed at which the estimate says the fin starts to flutter, in the air at that moment (it rises as the air thins).",
        "Rocket speed V" => "The rocket's speed through the air.",
        "Aspect ratio AR" => "Span squared over the area of one fin panel. Slender fins flutter at lower speeds.",
        "Taper ratio λ" => "Tip chord over root chord.",
        "Thickness t" => "The mean thickness of the fin section (a rounded or airfoil edge removes some of the plate's thickness).",
        "Thickness ratio t/c" => "Thickness over root chord: flutter speed grows with its 1.5 power, so thickness is the strongest lever.",
        "Centroid offset ε" => "How far the panel's centre of area lies behind the root quarter chord, as a fraction of the root chord. Swept fins have a larger one and flutter sooner.",
        "D_N" => "Martin's denominator constant 24 ε κ p₀ / π.",
        "Sound speed a" => "Speed of sound in the air at that altitude.",
        "Pressure p/p₀" => "Static pressure over sea-level standard pressure; thin air raises the flutter speed.",
        "Shear modulus G" => "Stiffness of the fin stock in shear. Flutter speed grows with its square root.",
        "G from" => "Where the shear modulus comes from: the material's tabulated value, or the value set on the fin.",
        "Stock" => "The fin material the shear modulus is taken from (an imported material takes the table entry of its family, e.g. plywood for 'Aircraft plywood').",
        "Estimate" => "Martin (NACA TN 4197) bending–torsion flutter estimate for a thin flat panel, in the form Bennett gives (2023).",
        "Accuracy" => "An empirical fit, good for preliminary design: it does not model the layup, the root fixity or the tip; test a marginal fin.",
        "Root" => "Martin's panels are clamped at the root, which is how a fin glued along its root, filleted or through-the-wall is built; a tab adds no credit to the estimate.",
        "at time" => "When in the flight (seconds after liftoff) the margin is lowest.",
        "Mach" => "The rocket's Mach number at that moment.",
        "Fin set" => "The fin set that decides the margin: the one with the lowest flutter speed at the worst moment.",
        "Fin sets" => "The rocket has more than one fin set (copies with their own overrides count); the worst is shown.",
        "+10 % thickness" => "Flutter speed grows with the 1.5 power of thickness: a fin 10 % thicker flutters at about 15 % higher speed.",
        "+10 % shear modulus" => "Flutter speed grows with the square root of the shear modulus: 10 % stiffer stock gives about 5 % more speed.",
        l if l.starts_with("Thickness for") => "The fin thickness that brings the lowest margin up to the minimum, everything else unchanged.",
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
        Data::Flutter(f) => f.as_ref().map_or(0.0, |f| f.trace.worst().t),
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
        Data::Flutter(f) => flutter(run, f.as_ref(), x, units),
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
    let lift = if out.lift_3d() { super::cfd::LIFT_LABEL } else { "" };
    let (cg, _) = length(run.cg(burnout), units);
    let mut rows = vec![Row::new(format!("Static margin{lift}"), cal(p.margin), "cal").big()];
    if out.runs.len() > 1 {
        for (i, r) in out.runs.iter().enumerate() {
            if let Some(v) = per_run_margin(r, p.x) {
                rows.push(Row::new(r.name.clone(), cal(v), "cal").swatch(config_color(i)));
            }
        }
    }
    rows.push(Row::new(format!("CP{lift}"), cp, u.clone()));
    // The kit shows one CG: the one the margin uses (the Conditions' "CG for margin").
    rows.push(Row::new(if burnout { "CG burnout" } else { "CG liftoff" }, cg, u));
    rows.push(Row::new(format!("CNα{lift}"), format!("{:.2}", p.cn_alpha), "/rad"));
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

/// The stability section's title: under the CFD table it says where the numbers come from (Extended, the drag solver being
/// not solved, or the 3D solve when a fresh one feeds them).
fn stability_title(out: &Output) -> String {
    if out.lift_3d() {
        format!("Stability · {}", super::cfd::PANEL_FROM_3D)
    } else if out.cfd.is_some() {
        "Stability · Extended".into()
    } else {
        "Stability".into()
    }
}

fn mach(out: &Output, run: &Run, points: &[MachPoint], x: f64, units: UnitSystem) -> Readout {
    let Some(at) = mach_at(points, x) else { return Readout { cursor: String::new(), sections: Vec::new() } };
    let like = MachPointLike { x, margin: at.margin_cal, cp: at.cp, cn_alpha: at.cn_alpha };
    let stability = Section { title: stability_title(out), rows: stability_rows(run, &like, units, out) };

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
    let drag = Section { title: if out.cfd.is_some() { "Drag · CFD".into() } else { "Drag".into() }, rows: drag };
    let worst = Section { title, rows: worst };
    // The drag breakdown analysis reads drag first; the others read stability first.
    let sections = if out.analysis == Analysis::Drag { vec![drag, stability, worst] } else { vec![stability, drag, worst] };
    Readout { cursor: format!("M {x:.2}"), sections }
}

fn alpha(out: &Output, run: &Run, alpha_rad: f64, units: UnitSystem) -> Readout {
    let Data::Alpha(points) = &run.data else { unreachable!("called for an angle sweep") };
    let Some(at) = alpha_at(points, alpha_rad) else { return Readout { cursor: String::new(), sections: Vec::new() } };
    let like = MachPointLike { x: alpha_rad, margin: at.margin_cal, cp: at.cp, cn_alpha: at.cn_alpha };
    let stability = Section { title: stability_title(out), rows: stability_rows(run, &like, units, out) };
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

fn flutter(run: &Run, analysis: Option<&FlutterAnalysis>, t: f64, units: UnitSystem) -> Readout {
    let Some(f) = analysis else {
        return Readout { cursor: String::new(), sections: vec![Section { title: "Flutter".into(), rows: vec![Row::new("No fins or no motor", "—", "")] }] };
    };
    let vel = |v: f64| ui::readout(v, Quantity::Velocity, units);
    let worst = f.trace.worst();
    let margin = worst.margin();
    let severity = if margin >= FLUTTER_MIN_MARGIN {
        Severity::Ok
    } else if margin >= 1.0 {
        Severity::Warn
    } else {
        Severity::Err
    };
    let (v_worst, vu) = vel(worst.speed);
    let (vf_worst, _) = vel(worst.flutter_velocity);
    let (alt, au) = ui::readout(worst.altitude, Quantity::Altitude, units);
    let length_unit = Quantity::Length.unit_for(units).symbol;
    let mut sections = vec![Section {
        title: format!("Flutter · {}", run.name),
        rows: vec![
            Row::new("Lowest margin", format!("{margin:.2}"), "×").big().tag(Tag::Chip(severity, format!("min {FLUTTER_MIN_MARGIN:.2}"))),
            Row::new("at time", format!("{:.1}", worst.t), "s"),
            Row::new("at altitude", alt, au),
            Row::new("Mach", format!("{:.2}", worst.mach), ""),
            Row::new("Rocket speed V", v_worst, vu),
            Row::new("Flutter speed V_f", vf_worst, vu),
        ],
    }];
    if let Some((vf, v)) = f.at_time(t) {
        let local = if v > 1.0 { format!("{:.2}", vf / v) } else { "—".into() };
        sections.push(Section {
            title: "At cursor".into(),
            rows: vec![Row::new("Rocket speed V", vel(v).0, vu), Row::new("Flutter speed V_f", vel(vf).0, vu), Row::new("Margin here", local, "×")],
        });
    }
    if let Some(i) = f.inputs() {
        let gov = f.governing();
        let (dn, dn_unit) = ui::readout(i.denominator_constant, Quantity::Pressure, units);
        let mut rows = vec![
            Row::new("Fin set", gov.name.clone(), ""),
            Row::new("Aspect ratio AR", format!("{:.2}", i.aspect_ratio), ""),
            Row::new("Taper ratio λ", format!("{:.2}", i.taper_ratio), ""),
            Row::new("Thickness t", ui::bare(i.thickness, Quantity::Length, units), length_unit),
            Row::new("Thickness ratio t/c", format!("{:.4}", i.thickness_ratio), ""),
            Row::new("Centroid offset ε", format!("{:.3}", i.epsilon), ""),
            Row::new("D_N", dn, dn_unit),
            Row::new("Sound speed a", vel(i.speed_of_sound).0, vu),
            Row::new("Pressure p/p₀", format!("{:.3}", i.pressure_ratio), ""),
            Row::new("Shear modulus G", format!("{:.2}", i.shear_modulus / 1e9), if gov.tip_to_tip { "GPa · tip-to-tip ×2" } else { "GPa" }),
            Row::new("Stock", gov.g_source.split_once(" · ").map_or("set on the fin".to_string(), |(stock, _)| stock.to_string()), "").muted(),
            Row::new("G from", gov.g_source.split_once(" · ").map_or(gov.g_source.clone(), |(_, source)| source.to_string()), "").muted(),
        ];
        if f.stocks.len() > 1 {
            rows.insert(1, Row::new("Fin sets", f.stocks.len().to_string(), "the worst is shown").muted());
        }
        sections.push(Section { title: "Inputs at the lowest margin".into(), rows });
        // What would move the margin: V_f ∝ t^1.5 and ∝ √G, so the thickness that reaches the target is closed-form.
        let mut hints = vec![
            Row::new("+10 % thickness", format!("{:+.0}", (1.1f64.powf(1.5) - 1.0) * 100.0), "% V_f"),
            Row::new("+10 % shear modulus", format!("{:+.1}", (1.1f64.sqrt() - 1.0) * 100.0), "% V_f"),
        ];
        if margin.is_finite() && margin > 0.0 {
            let needed = i.thickness * (FLUTTER_MIN_MARGIN / margin).powf(2.0 / 3.0);
            let tag = if margin >= FLUTTER_MIN_MARGIN { Tag::Plain("met".into()) } else { Tag::Chip(Severity::Warn, "thicker".into()) };
            hints.push(Row::new(format!("Thickness for {FLUTTER_MIN_MARGIN:.2}×"), ui::bare(needed, Quantity::Length, units), length_unit).tag(tag));
        }
        sections.push(Section { title: "What moves it".into(), rows: hints });
    }
    sections.push(Section {
        title: "Method".into(),
        rows: vec![
            Row::new("Estimate", "NACA TN 4197", ""),
            Row::new("Accuracy", "preliminary design", ""),
            Row::new("Root", if f.governing().tabbed { "clamped · tab" } else { "clamped" }, "").muted(),
        ],
    });
    let warnings = f.warnings();
    if !warnings.is_empty() {
        let rows = warnings
            .iter()
            .map(|w| {
                let (label, value) = match w {
                    FlutterWarning::BeyondMach(m) => ("Beyond Mach 1.5", format!("reaches M {m:.2}")),
                    FlutterWarning::AspectRatio(ar) => ("Aspect ratio", format!("{ar:.2}")),
                    FlutterWarning::ThicknessRatio(tc) => ("Thickness ratio", format!("{tc:.3}")),
                    FlutterWarning::CentroidForward(e) => ("Centroid ahead of ¼ chord", format!("ε {e:.2}")),
                    FlutterWarning::CentroidAft(e) => ("Centroid behind the root", format!("ε {e:.2}")),
                    FlutterWarning::Degenerate => ("Fin panel", "unusable".to_string()),
                    FlutterWarning::NoShearModulus => ("Shear modulus G", "not set".to_string()),
                };
                let chip = match w {
                    FlutterWarning::Degenerate | FlutterWarning::NoShearModulus => Tag::Chip(Severity::Err, "no estimate".into()),
                    _ => Tag::Chip(Severity::Warn, "limit".into()),
                };
                let mut row = Row::new(label, value, "").tag(chip);
                row.help = Some(match w {
                    FlutterWarning::BeyondMach(_) => "Martin's flutter data do not reach past Mach 1.5; above it the estimate is an extrapolation.",
                    FlutterWarning::AspectRatio(_) => "The aspect ratio is outside the range the thin-panel estimate was fitted to; treat the result as rough.",
                    FlutterWarning::ThicknessRatio(_) => {
                        "The thickness ratio is outside the range the thin-panel estimate was fitted to; treat the result as rough."
                    }
                    FlutterWarning::CentroidForward(_) => {
                        "With the area centroid at or ahead of the root quarter chord the formula has no finite answer; ε is held at a small value."
                    }
                    FlutterWarning::CentroidAft(_) => {
                        "The area centroid lies behind the root's trailing edge: the panel is swept far more than the trapezoids the formula is meant for, so its D_N is an extrapolation."
                    }
                    FlutterWarning::Degenerate => "The fin has a zero or non-finite chord, span, area or thickness, so there is no panel to estimate; the speed shown is 0.",
                    FlutterWarning::NoShearModulus => "The fin stock has no shear modulus (zero or not a number); set the fin's own value in the inspector.",
                });
                row
            })
            .collect();
        sections.push(Section { title: "Limits of the estimate".into(), rows });
    }
    Readout { cursor: format!("t {t:.1} s"), sections }
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
        let rocket = Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        let (jobs, _) = jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        (
            compute(&Request {
                analysis,
                method: Method::Extended,
                conditions: Conditions { runs: 10, ..Conditions::default() },
                jobs,
                bands: rocket.stability_bands(),
                cfd: None,
                drag_tables: Vec::new(),
            }),
            rocket.active_config_id(),
        )
    }

    #[test]
    fn aerodynamics_readout_has_the_kit_sections() {
        let (o, id) = out(Analysis::Aerodynamics);
        let r = readout(&o, Some(1.1), id, UnitSystem::MM_G_S);
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
        assert_eq!(readout(&o, None, id, UnitSystem::MM_G_S).cursor, format!("M {x:.2}"));
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
            for section in readout(&o, None, id, UnitSystem::MM_G_S).sections {
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
        let r = readout(&o, Some(1.1), id, UnitSystem::MM_G_S);
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
        let s = compare_section(&o, &overlay, Some(1.1), id, UnitSystem::MM_G_S).unwrap();
        assert_eq!(s.title, "Compare · Same");
        let labels: Vec<&str> = s.rows.iter().map(|r| r.label.as_str()).collect();
        assert_eq!(labels, ["Margin", "CP", "Cd"]);
        assert!(s.rows.iter().all(|r| matches!(&r.tag, Some(Tag::Plain(t)) if t.starts_with("Δ +0") || t.starts_with("Δ −0"))));
        // Outside the comparison's data there is nothing to compare.
        assert!(compare_section(&o, &overlay, Some(9.0), id, UnitSystem::MM_G_S).is_none());
    }

    #[test]
    fn other_analyses_have_readouts() {
        for a in [Analysis::Stability, Analysis::Flutter, Analysis::Dispersion] {
            let (o, id) = out(a);
            let r = readout(&o, None, id, UnitSystem::MM_G_S);
            assert!(!r.sections.is_empty() && r.sections.iter().all(|s| !s.rows.is_empty()), "{a:?}");
        }
    }
}
