//! The parts list and the design report: a side view with the CG and CP, the summary and configuration tables, the flights table,
//! the health warnings and the stability page ([`stability`]: CG, CP and margin against Mach, the 3D solve's when there is one),
//! all in the user's unit set.
//!
//! What needs the app's context (the balance, the health list, the flights' conditions, the stored CFD tables and the motor
//! database) is gathered on the UI thread by [`gather`]; the flights are then prepared and flown, and the stability page built,
//! on the worker that builds the file ([`Flown::fly`]).

use std::sync::Arc;

use bowshock_model::{InternalKind, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{self, Balance, Level, Phase, Warning};
use bowshock_sim::Summary;

use super::pdf::{Page, PageSize, Pdf};
use super::stability;
use super::templates::MARGIN;
use crate::app::{ui, Ctx};
use crate::simulate::flight::{prepare_rocket, Spec};
use crate::units::{Quantity, UnitSystem};

/// Row height of a table, mm.
const ROW: f64 = 5.0;
/// Text size of table cells, pt.
const CELL: f64 = 8.0;

/// A configuration's row of the design report.
#[derive(Clone, Debug)]
pub struct ConfigRow {
    /// Display name.
    pub name: String,
    /// The motors it carries, as "designation, designation".
    pub motors: String,
    /// Liftoff mass, kg, or why it could not be worked out.
    pub liftoff: Result<f64, String>,
}

/// What the report needs from the app, gathered on the UI thread.
pub struct Inputs {
    /// Display units.
    pub units: UnitSystem,
    /// Mass, CG and CP of the active configuration at liftoff.
    pub balance: Result<Balance, String>,
    /// One row per configuration.
    pub configs: Vec<ConfigRow>,
    /// The health list.
    pub warnings: Vec<Warning>,
    /// Every document flight: its name and conditions (empty without the design report).
    pub flights: Vec<(String, Spec)>,
    /// The motor database the flights fly with.
    pub motors: Arc<MotorDb>,
    /// What the stability page is made from (`None` without the design report).
    pub stability: Option<stability::Inputs>,
}

/// Gathers [`Inputs`]. The flights are neither prepared nor flown here.
pub fn gather(ctx: &Ctx, with_flights: bool) -> Inputs {
    let rocket = ctx.doc.rocket();
    let motors = ctx.physics.motors();
    let balance = from_model::balance(rocket, motors, Phase::Full, 0.3, 0.0, true).map_err(|e| format!("{e:?}"));
    let warnings = balance.as_ref().map(|b| from_model::warnings(rocket, motors, b)).unwrap_or_default();
    let configs = rocket
        .configurations()
        .iter()
        .map(|c| {
            let mut copy = rocket.clone();
            let liftoff = if copy.select_configuration(c.id) {
                from_model::balance(&copy, motors, Phase::Full, 0.3, 0.0, true).map(|b| b.mass).map_err(|e| format!("{e:?}"))
            } else {
                Err("missing".to_string())
            };
            let motors_text = c.motors.iter().filter_map(|s| s.motor.as_ref().map(|m| m.designation.clone())).collect::<Vec<_>>().join(", ");
            ConfigRow { name: c.display_name(), motors: if motors_text.is_empty() { "No motor".to_string() } else { motors_text }, liftoff }
        })
        .collect();
    let flights = if with_flights { rocket.flights().iter().map(|def| (def.name.clone(), Spec::of(def))).collect() } else { Vec::new() };
    let motors = Arc::new(if with_flights { motors.clone() } else { MotorDb::default() });
    let stability = with_flights.then(|| stability_inputs(ctx));
    Inputs { units: ctx.units, balance, configs, warnings, flights, motors, stability }
}

/// What the stability page needs of the session: the stored CFD drag table of the whole stack when it is the rocket's now (it is
/// named in the provenance) and whether its 3D stability solve is fresh, which is the solve the flights fly on.
fn stability_inputs(ctx: &Ctx) -> stability::Inputs {
    let rocket = ctx.doc.rocket();
    stability::Inputs {
        table: ctx.physics.stored_table(rocket, 1).ok().cloned(),
        lift_fresh: ctx.physics.lift_table(rocket, 1).is_ok(),
        date: stability::date_utc(crate::analyze::cfd::now()),
    }
}

/// What the worker makes of [`Inputs`] and the document: the flights' outcomes in document order, and the stability page.
pub struct Flown {
    /// Each flight's name and summary, or why it could not be flown.
    pub flights: Vec<(String, Result<Summary, String>)>,
    /// The stability page, `None` when [`Inputs::stability`] is (the report was not asked for), or why it could not be made.
    pub stability: Option<Result<stability::Stability, String>>,
}

impl Flown {
    /// Prepares (with [`prepare_rocket`]) and flies every flight of `inputs` with `rocket`, and builds the stability page: the
    /// slow part, for the worker.
    pub fn fly(rocket: &Rocket, inputs: &Inputs) -> Flown {
        let fly = |spec: &Spec| prepare_rocket(rocket.clone(), &inputs.motors, spec).map(|(input, _)| bowshock_sim::simulate(&input).summary);
        Flown {
            flights: inputs.flights.iter().map(|(name, spec)| (name.clone(), fly(spec))).collect(),
            stability: inputs.stability.as_ref().map(|s| stability::compute(rocket, &inputs.motors, s)),
        }
    }
}

/// Cell alignment.
#[derive(Clone, Copy, PartialEq)]
enum Align {
    Left,
    Right,
}

/// Pages that flow downwards: a cursor, a new page when something does not fit.
struct Flow<'a> {
    pdf: &'a mut Pdf,
    size: PageSize,
    y: f64,
}

impl<'a> Flow<'a> {
    fn new(pdf: &'a mut Pdf, size: PageSize) -> Flow<'a> {
        pdf.page(size);
        Flow { pdf, size, y: MARGIN }
    }

    fn width(&self) -> f64 {
        self.size.mm().0 - 2.0 * MARGIN
    }

    fn page(&mut self) -> &mut Page {
        self.pdf.current().expect("a page exists")
    }

    fn new_page(&mut self) {
        self.pdf.page(self.size);
        self.y = MARGIN;
    }

    /// Starts a new page unless `h` mm fit under the cursor.
    fn need(&mut self, h: f64) {
        if self.y + h > self.size.mm().1 - MARGIN {
            self.new_page();
        }
    }

    fn heading(&mut self, s: &str) {
        self.need(14.0);
        self.y += 8.0;
        let y = self.y;
        self.page().text((MARGIN, y), 11.0, true, s);
        self.y += 3.0;
    }

    fn paragraph(&mut self, size: f64, bold: bool, s: &str) {
        let width = self.width();
        for line in wrap(s, size, bold, width) {
            self.need(size * 0.5);
            self.y += size * 0.45;
            let y = self.y;
            self.page().text((MARGIN, y), size, bold, &line);
        }
        self.y += 1.0;
    }

    /// A table with a shaded header that repeats on each page.
    fn table(&mut self, cols: &[(f64, &str, Align)], rows: &[Vec<String>], bold_last: bool) {
        let header = |flow: &mut Flow<'_>| {
            flow.need(2.0 * ROW);
            let y = flow.y;
            let total: f64 = cols.iter().map(|c| c.0).sum();
            let page = flow.page();
            page.fill_rect((MARGIN, y), (total, ROW), 0.88);
            let mut x = MARGIN;
            for (w, name, align) in cols {
                cell(page, x, y, *w, name, true, *align);
                x += w;
            }
            flow.y += ROW;
        };
        header(self);
        for (i, row) in rows.iter().enumerate() {
            if self.y + ROW > self.size.mm().1 - MARGIN {
                self.new_page();
                header(self);
            }
            let (y, last) = (self.y, bold_last && i + 1 == rows.len());
            let total: f64 = cols.iter().map(|c| c.0).sum();
            let page = self.page();
            let mut x = MARGIN;
            for ((w, _, align), text) in cols.iter().zip(row) {
                cell(page, x, y, *w, text, last, *align);
                x += w;
            }
            page.gray(0.7);
            page.line((MARGIN, y + ROW), (MARGIN + total, y + ROW), 0.1);
            page.gray(0.0);
            self.y += ROW;
        }
    }
}

/// Text `s` in the cell at `(x, top)` of width `w`, cut with "..." if it is too wide.
fn cell(page: &mut Page, x: f64, top: f64, w: f64, s: &str, bold: bool, align: Align) {
    let pad = 1.2;
    let text = fit(s, CELL, bold, w - 2.0 * pad);
    let tw = Page::text_width(CELL, bold, &text);
    let at = match align {
        Align::Left => x + pad,
        Align::Right => x + w - pad - tw,
    };
    page.text((at, top + ROW - 1.5), CELL, bold, &text);
}

/// `s` shortened to fit `width` mm at `size` pt.
fn fit(s: &str, size: f64, bold: bool, width: f64) -> String {
    if Page::text_width(size, bold, s) <= width {
        return s.to_string();
    }
    let mut out: String = s.to_string();
    while !out.is_empty() && Page::text_width(size, bold, &format!("{out}...")) > width {
        out.pop();
    }
    format!("{out}...")
}

/// `s` broken into lines of at most `width` mm.
fn wrap(s: &str, size: f64, bold: bool, width: f64) -> Vec<String> {
    let mut lines = Vec::new();
    let mut line = String::new();
    for word in s.split_whitespace() {
        let trial = if line.is_empty() { word.to_string() } else { format!("{line} {word}") };
        if !line.is_empty() && Page::text_width(size, bold, &trial) > width {
            lines.push(std::mem::replace(&mut line, word.to_string()));
        } else {
            line = trial;
        }
    }
    if !line.is_empty() {
        lines.push(line);
    }
    lines
}

fn len(v: f64, u: UnitSystem) -> String {
    ui::with_unit(v, Quantity::Length, u)
}

fn mass(v: f64, u: UnitSystem) -> String {
    ui::with_unit(v, Quantity::Mass, u)
}

/// The rows of the parts list: part, kind, material, size, total mass, count; the last row is the total.
pub fn parts_rows(rocket: &Rocket, u: UnitSystem) -> Vec<Vec<String>> {
    let unit = ui::len_unit(u);
    let mass_of = |id| rocket.component_mass(id).map_or_else(|| "—".to_string(), |m| mass(m.mass, u));
    let mut rows = Vec::new();
    for p in rocket.parts() {
        rows.push(vec![
            p.display_name(),
            p.kind.name().to_string(),
            p.material.name().to_string(),
            format!("{} {unit}", ui::size_label(&p.kind, u)),
            mass_of(p.id),
            "1".to_string(),
        ]);
    }
    for r in rocket.radials() {
        let material = if r.kind.is_pod() { "—".to_string() } else { r.material.name().to_string() };
        rows.push(vec![
            r.display_name(),
            r.kind.name().to_string(),
            material,
            format!("{} {unit}", ui::radial_size_label(&r.kind, u)),
            mass_of(r.id),
            r.count.to_string(),
        ]);
    }
    for i in rocket.internals() {
        let Some(res) = rocket.resolved(i.id) else { continue };
        let size = match &i.kind {
            InternalKind::Parachute { diameter, .. } => format!("Ø{} {unit} canopy", ui::len(*diameter, u)),
            InternalKind::Streamer { length, width, .. } => format!("{} × {} {unit}", ui::len(*length, u), ui::len(*width, u)),
            InternalKind::ShockCord { length, .. } => format!("{} {unit} cord", ui::len(*length, u)),
            _ if res.inner_diameter > 0.0 => {
                format!("Ø{} / {} × {} {unit}", ui::len(res.outer_diameter, u), ui::len(res.inner_diameter, u), ui::len(res.length, u))
            }
            _ => format!("Ø{} × {} {unit}", ui::len(res.outer_diameter, u), ui::len(res.length, u)),
        };
        let material = if i.kind.material_kind().is_some() { i.material.name().to_string() } else { "—".to_string() };
        rows.push(vec![i.display_name(), i.kind.name().to_string(), material, size, mass_of(i.id), "1".to_string()]);
    }
    rows.push(vec!["Total (no motors)".to_string(), String::new(), String::new(), String::new(), mass(rocket.dry_mass().mass, u), String::new()]);
    rows
}

/// The parts list, starting on a new page.
pub fn parts_list(pdf: &mut Pdf, rocket: &Rocket, u: UnitSystem, size: PageSize) {
    let mut flow = Flow::new(pdf, size);
    let name = if rocket.name.trim().is_empty() { "Untitled rocket" } else { rocket.name.trim() };
    flow.y += 5.0;
    let y = flow.y;
    flow.page().text((MARGIN, y), 14.0, true, &format!("{name}: parts list"));
    flow.y += 4.0;
    let w = flow.width();
    let rest = w - 40.0 - 32.0 - 52.0 - 22.0 - 10.0;
    let cols = [
        (40.0, "Part", Align::Left),
        (32.0, "Kind", Align::Left),
        (rest.max(20.0), "Material", Align::Left),
        (52.0, "Size", Align::Left),
        (22.0, "Mass", Align::Right),
        (10.0, "Qty", Align::Right),
    ];
    let scale = w / cols.iter().map(|c| c.0).sum::<f64>();
    let cols: Vec<(f64, &str, Align)> = cols.iter().map(|&(cw, n, a)| (cw * scale, n, a)).collect();
    flow.table(&cols, &parts_rows(rocket, u), true);
}

/// The side view of the rocket in the box `origin`, `size` (mm), with the CG and CP marked when known.
fn side_view(page: &mut Page, rocket: &Rocket, balance: Option<&Balance>, u: UnitSystem, origin: (f64, f64), size: (f64, f64)) {
    let stations = rocket.stations();
    let mut reach = rocket.length();
    let mut rmax = rocket.reference_diameter().unwrap_or(0.0) / 2.0;
    for r in rocket.radials().iter().filter(|r| r.kind.is_fin()) {
        if let (Some(z), Some(base)) = (rocket.radial_station(r), rocket.radial_base_radius(r)) {
            reach = reach.max(z + r.kind.extent());
            rmax = rmax.max(base + r.kind.fin_span().unwrap_or(0.0));
        }
    }
    if reach <= 0.0 || rmax <= 0.0 {
        return;
    }
    // Leave room under the body for the CG and CP labels.
    let label_h = 11.0;
    let scale = (size.0 / reach).min((size.1 - label_h) / (2.0 * rmax));
    let cy = origin.1 + (size.1 - label_h) / 2.0;
    let x = |z: f64| origin.0 + z * scale;
    for (i, p) in rocket.parts().iter().enumerate() {
        let profile = p.kind.profile();
        let mut outline: Vec<(f64, f64)> = profile.iter().map(|&(z, r)| (x(stations[i] + z), cy - r * scale)).collect();
        outline.extend(profile.iter().rev().map(|&(z, r)| (x(stations[i] + z), cy + r * scale)));
        page.path(&outline, true, 0.25, None);
    }
    for r in rocket.radials() {
        let (Some(z0), Some(base)) = (rocket.radial_station(r), rocket.radial_base_radius(r)) else { continue };
        if let Some(plan) = r.kind.planform() {
            for sign in [-1.0, 1.0] {
                let pts: Vec<(f64, f64)> = plan.iter().map(|&(z, h)| (x(z0 + z), cy + sign * (base + h) * scale)).collect();
                page.path(&pts, true, 0.25, None);
            }
        } else if let bowshock_model::RadialKind::LaunchLug { length, diameter } = r.kind {
            let (a, b) = (x(z0), x(z0 + length));
            page.path(
                &[(a, cy - base * scale), (b, cy - base * scale), (b, cy - (base + diameter) * scale), (a, cy - (base + diameter) * scale)],
                true,
                0.2,
                None,
            );
        } else if let bowshock_model::RadialKind::RailButton { height, diameter } = r.kind {
            let (a, b) = (x(z0), x(z0 + diameter));
            page.path(&[(a, cy - base * scale), (b, cy - base * scale), (b, cy - (base + height) * scale), (a, cy - (base + height) * scale)], true, 0.2, None);
        }
    }
    if let Some(b) = balance {
        let label_y = cy + rmax * scale + 4.0;
        // CG: a ringed cross; CP: a filled disc. Their labels sit under the body, on separate lines.
        let (gx, px) = (x(b.cg), x(b.cp));
        page.rgb(0.78, 0.12, 0.12);
        page.circle((gx, cy), 2.4, 0.3);
        page.line((gx - 2.4, cy), (gx + 2.4, cy), 0.3);
        page.line((gx, cy - 2.4), (gx, cy + 2.4), 0.3);
        page.text((gx - 6.0, label_y), 7.0, true, &format!("CG {}", len(b.cg, u)));
        page.rgb(0.1, 0.3, 0.8);
        page.disc((px, cy), 1.8);
        page.text((px - 6.0, label_y + 3.6), 7.0, true, &format!("CP {}", len(b.cp, u)));
        page.gray(0.0);
    }
}

/// Pages of the design report: the rocket, then the flights and the health list.
pub fn design_report(pdf: &mut Pdf, rocket: &Rocket, inputs: &Inputs, flown: Option<&Flown>, size: PageSize) {
    let u = inputs.units;
    let mut flow = Flow::new(pdf, size);
    let name = if rocket.name.trim().is_empty() { "Untitled rocket" } else { rocket.name.trim() };
    flow.y += 7.0;
    let y = flow.y;
    flow.page().text((MARGIN, y), 18.0, true, name);
    if !rocket.designer.trim().is_empty() {
        flow.y += 6.0;
        let y = flow.y;
        flow.page().text((MARGIN, y), 10.0, false, &format!("Designed by {}", rocket.designer.trim()));
    }
    flow.y += 5.0;
    let (w, top) = (flow.width(), flow.y);
    let box_h = 62.0;
    side_view(flow.page(), rocket, inputs.balance.as_ref().ok(), u, (MARGIN, top), (w, box_h));
    flow.y += box_h + 2.0;
    if let Err(e) = &inputs.balance {
        flow.paragraph(8.0, false, &format!("CG and CP unavailable: {e}"));
    }

    flow.heading("Summary");
    let row = |k: &str, v: String| vec![k.to_string(), v];
    let mut summary = vec![row("Length", len(rocket.length(), u)), row("Largest diameter", rocket.reference_diameter().map_or("—".to_string(), |d| len(d, u)))];
    if let Ok(b) = &inputs.balance {
        summary.push(row("Liftoff mass (active configuration)", mass(b.mass, u)));
        summary.push(row("CG, from the nose tip", len(b.cg, u)));
        summary.push(row("CP, from the nose tip (Mach 0.3)", len(b.cp, u)));
        summary.push(row("Static margin", format!("{:.2} cal", b.margin_cal)));
    }
    flow.table(&[(70.0, "Quantity", Align::Left), (45.0, "Value", Align::Right)], &summary, false);

    flow.heading("Configurations");
    let configs: Vec<Vec<String>> = inputs
        .configs
        .iter()
        .map(|c| vec![c.name.clone(), c.motors.clone(), c.liftoff.as_ref().map_or_else(|e| format!("— ({e})"), |m| mass(*m, u))])
        .collect();
    flow.table(&[(55.0, "Configuration", Align::Left), (85.0, "Motors", Align::Left), (35.0, "Liftoff mass", Align::Right)], &configs, false);

    flow.new_page();
    flow.y += 5.0;
    let y = flow.y;
    flow.page().text((MARGIN, y), 14.0, true, &format!("{name}: flights"));
    flow.y += 2.0;
    let mut errors = Vec::new();
    let flights: Vec<Vec<String>> = match flown {
        Some(Flown { flights: list, .. }) => list
            .iter()
            .map(|(flight_name, outcome)| match outcome {
                Ok(s) => vec![
                    flight_name.clone(),
                    ui::with_unit(s.apogee, Quantity::Altitude, u),
                    ui::with_unit(s.max_velocity, Quantity::Velocity, u),
                    format!("{:.2}", s.max_mach),
                    ui::with_unit(s.rail_exit_velocity, Quantity::Velocity, u),
                    format!("{:.2} cal", s.stability_off_rail_cal),
                    format!("{:.1} s", s.optimum_delay),
                    ui::with_unit(s.ground_hit_velocity, Quantity::Velocity, u),
                    format!("{} @ {:.0}°", ui::with_unit(s.landing.0, Quantity::Altitude, u), s.landing.1),
                ],
                Err(e) => {
                    errors.push(format!("{flight_name}: not flown ({e})"));
                    let mut r = vec![flight_name.clone()];
                    r.extend((0..8).map(|_| "—".to_string()));
                    r
                }
            })
            .collect(),
        None => Vec::new(),
    };
    let cols = [
        (24.0, "Flight", Align::Left),
        (18.0, "Apogee", Align::Right),
        (18.0, "Max vel.", Align::Right),
        (17.0, "Max Mach", Align::Right),
        (17.0, "Rail exit", Align::Right),
        (17.0, "Off-rail", Align::Right),
        (18.0, "Opt. delay", Align::Right),
        (19.0, "Ground hit", Align::Right),
        (34.0, "Landing", Align::Right),
    ];
    let scale = (flow.width() / cols.iter().map(|c| c.0).sum::<f64>()).min(1.0);
    let cols: Vec<(f64, &str, Align)> = cols.iter().map(|&(cw, n, a)| (cw * scale, n, a)).collect();
    flow.table(&cols, &flights, false);
    for e in &errors {
        flow.paragraph(8.0, false, e);
    }

    flow.heading("Health");
    for w in &inputs.warnings {
        let tag = match w.level {
            Level::Ok => "OK",
            Level::Warn => "WARN",
            Level::Err => "ERROR",
        };
        flow.paragraph(8.5, w.level == Level::Err, &format!("[{tag}] {}", w.message));
    }
    if inputs.warnings.is_empty() {
        flow.paragraph(8.5, false, "No health information (the rocket could not be balanced).");
    }
    match flown.and_then(|f| f.stability.as_ref()) {
        Some(Ok(st)) => stability_page(&mut flow, name, st, u),
        Some(Err(why)) => flow.paragraph(8.5, false, &format!("Stability page not made: {why}")),
        None => {}
    }
}

/// The stability page: the CG, the margin against the guidance, the Mach table and where the numbers come from.
fn stability_page(flow: &mut Flow<'_>, name: &str, st: &stability::Stability, u: UnitSystem) {
    flow.new_page();
    flow.y += 5.0;
    let y = flow.y;
    flow.page().text((MARGIN, y), 14.0, true, &format!("{name}: stability"));
    flow.y += 2.0;
    for fact in st.facts(u) {
        flow.paragraph(8.5, false, &fact);
    }

    flow.heading("Centre of gravity (calculated)");
    let unit = ui::len_unit(u);
    let cg_header = format!("CG from the nose tip ({unit})");
    flow.table(&[(60.0, "Condition", Align::Left), (55.0, &cg_header, Align::Right), (45.0, "CG (cal from the tip)", Align::Right)], &st.cg_rows(u), false);

    flow.heading("Summary");
    flow.paragraph(8.0, false, &stability::guidance_note());
    for line in &st.summary {
        flow.paragraph(8.5, line.contains("BELOW"), &format!("- {line}"));
    }

    flow.heading("Provenance");
    for line in &st.provenance {
        flow.paragraph(8.0, false, &format!("- {line}"));
    }

    flow.heading("How measured");
    flow.paragraph(8.0, false, stability::HOW_MEASURED);

    // The table (a row per 0.1 Mach) takes a page of its own, so it is never split over two.
    flow.new_page();
    flow.y += 5.0;
    let y = flow.y;
    flow.page().text((MARGIN, y), 14.0, true, &format!("{name}: stability against Mach"));
    flow.y += 2.0;
    flow.paragraph(8.0, false, &st.table_note());
    let (header, rows) = st.table(u);
    let first = 15.0;
    let rest = (flow.width() - first) / (header.len() - 1) as f64;
    let cols: Vec<(f64, &str, Align)> =
        header.iter().enumerate().map(|(i, h)| (if i == 0 { first } else { rest }, h.as_str(), if i == 0 { Align::Left } else { Align::Right })).collect();
    flow.table(&cols, &rows, false);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example() -> Rocket {
        Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).expect("the example loads")
    }

    #[test]
    fn the_parts_list_has_every_part_and_a_total() {
        let rocket = example();
        let rows = parts_rows(&rocket, UnitSystem::MM_G_S);
        let n = rocket.parts().len() + rocket.radials().len() + rocket.internals().len();
        assert_eq!(rows.len(), n + 1);
        assert!(rows.iter().all(|r| r.len() == 6));
        assert!(rows.last().unwrap()[0].starts_with("Total"));
        assert!(rows.iter().any(|r| r[1] == "Fin set" && r[5].parse::<u32>().is_ok_and(|n| n >= 3)));
    }

    #[test]
    fn text_is_fitted_and_wrapped() {
        let narrow = fit("A very long part name indeed", 8.0, false, 20.0);
        assert!(narrow.ends_with("...") && Page::text_width(8.0, false, &narrow) <= 20.0);
        assert_eq!(fit("short", 8.0, false, 20.0), "short");
        let lines = wrap("one two three four five six seven eight nine ten", 8.0, false, 25.0);
        assert!(lines.len() > 1 && lines.iter().all(|l| Page::text_width(8.0, false, l) <= 25.0));
    }

    #[test]
    fn the_report_without_flights_still_lays_out_two_pages() {
        let rocket = example();
        let inputs = Inputs {
            units: UnitSystem::MM_G_S,
            balance: Err("no motor".into()),
            configs: Vec::new(),
            warnings: Vec::new(),
            flights: Vec::new(),
            motors: Default::default(),
            stability: None,
        };
        let mut pdf = Pdf::new("t");
        design_report(&mut pdf, &rocket, &inputs, None, PageSize::A4);
        assert_eq!(pdf.page_count(), 2);
        let mut pdf = Pdf::new("t");
        parts_list(&mut pdf, &rocket, UnitSystem::IN_OZ_S, PageSize::Letter);
        assert!(pdf.page_count() >= 1);
    }

    #[test]
    fn the_stability_pages_follow_the_health_list_when_they_could_be_made() {
        let rocket = example();
        let inputs = Inputs {
            units: UnitSystem::MM_G_S,
            balance: Err("none".into()),
            configs: Vec::new(),
            warnings: Vec::new(),
            flights: Vec::new(),
            motors: Arc::new(MotorDb::bundled()),
            stability: Some(stability::Inputs { table: None, lift_fresh: false, date: "2026-11-15".into() }),
        };
        let flown = Flown::fly(&rocket, &inputs);
        assert!(flown.stability.as_ref().is_some_and(Result::is_ok));
        let mut pdf = Pdf::new("t");
        design_report(&mut pdf, &rocket, &inputs, Some(&flown), PageSize::A4);
        assert_eq!(pdf.page_count(), 4, "the report's two pages, the stability page and its Mach table");
        let text = String::from_utf8_lossy(&pdf.to_bytes()).into_owned();
        assert!(text.contains("stability") && text.contains("Centre of gravity") && text.contains("stability against Mach"));
        // A failed page is said, not dropped silently.
        let failed = Flown { flights: Vec::new(), stability: Some(Err("no sweep".into())) };
        let mut pdf = Pdf::new("t");
        design_report(&mut pdf, &rocket, &inputs, Some(&failed), PageSize::A4);
        assert_eq!(pdf.page_count(), 2);
        assert!(String::from_utf8_lossy(&pdf.to_bytes()).contains("Stability page not made: no sweep"));
    }

    #[test]
    fn the_flights_are_prepared_and_flown_off_the_document() {
        let rocket = example();
        let flights = rocket.flights().iter().map(|def| (def.name.clone(), Spec::of(def))).collect();
        let inputs = Inputs {
            units: UnitSystem::MM_G_S,
            balance: Err("unused".into()),
            configs: Vec::new(),
            warnings: Vec::new(),
            flights,
            motors: Arc::new(MotorDb::bundled()),
            stability: None,
        };
        let Flown { flights: list, stability } = Flown::fly(&rocket, &inputs);
        assert!(stability.is_none(), "no stability inputs, no page");
        assert_eq!(list.len(), rocket.flights().len());
        assert!(list.iter().all(|(_, s)| s.as_ref().is_ok_and(|s| s.apogee > 100.0)), "{:?}", list.iter().map(|l| &l.1).collect::<Vec<_>>());
    }
}
