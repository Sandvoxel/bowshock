//! The stability page of the design report: what a flyer brings to the range safety officer for Tripoli Safety Code 7-1.2 ("the
//! flier shall document the location of the center of pressure and be able to demonstrate the center of gravity") and 7-1.4
//! (thrust-to-weight typically at least 5:1, 3:1 with the RSO's approval).
//!
//! It is made from what Analyze already computes (the Mach sweep of the active configuration; the stored 3D stability solve when
//! the stack's flights fly on it, see [`Physics::lift_table`](crate::app::physics::Physics::lift_table)) and from the document; it
//! runs no solve. [`build`] is pure, so the tests can feed it a hand-built sweep; [`compute`] builds the sweeps from the document
//! on the worker that builds the PDF. [`Stability::cg_rows`] and [`Stability::table`] give the rows in the user's units;
//! `report::design_report` lays them out.

use std::fmt::Write;

use bowshock_model::{CfdLiftTable, CfdQuality, CfdTable, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::analysis::MachPoint;
use bowshock_sim::from_model::Phase;
use bowshock_sim::Trigger;

use crate::analyze::cfd;
use crate::analyze::compute::{self, ConfigJob};
use crate::units::{self, Quantity, UnitSystem};

/// Spacing of the report's Mach rows.
const ROW_STEP: f64 = 0.1;
/// Thrust-to-weight typically asked for (Tripoli Safety Code 7-1.4).
const TW_TYPICAL: f64 = 5.0;
/// The lowest thrust-to-weight an RSO may approve (Tripoli Safety Code 7-1.4).
const TW_MINIMUM: f64 = 3.0;
/// What the 3D solve was checked against; stated as measured by the validation of the solver.
const VALIDATION: &str = "3D Euler; validated on the Army-Navy Basic Finner: CNalpha within -7 % to +6 %, CP within 0.15 cal of wind-tunnel data at Mach 0.69-2.48; transonic CNalpha 13-19 % low";
/// Standard gravity (m/s^2).
const G0: f64 = 9.80665;

/// What the page needs from the app, gathered on the UI thread.
#[derive(Clone, Debug)]
pub struct Inputs {
    /// The stored CFD table of the whole stack when it is the rocket's now (its drag provenance), else `None`.
    pub table: Option<CfdTable>,
    /// Whether the table's 3D stability solve is fresh: the one the flights fly on.
    pub lift_fresh: bool,
    /// The date of the report (`2026-11-15`, UTC).
    pub date: String,
}

/// One stability value pair of a source.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stab {
    /// CP from the nose tip (m).
    pub cp: f64,
    /// Normal force slope per radian on the reference area.
    pub cn_alpha: f64,
}

/// One row of the Mach table.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Row {
    /// The Mach number.
    pub mach: f64,
    /// A Mach number the 3D solve was run at.
    pub node: bool,
    /// The stability the report is made of: the 3D solve's when there is one, else Extended's.
    pub used: Stab,
    /// Extended's, shown beside the 3D solve's.
    pub ext: Option<Stab>,
}

/// The page.
#[derive(Clone, Debug)]
pub struct Stability {
    /// The configuration the page is about.
    pub configuration: String,
    /// The date of the report.
    pub date: String,
    /// The reference diameter (m): one caliber.
    pub reference_diameter: f64,
    /// CG at liftoff, motors loaded (m from the tip).
    pub cg_liftoff: f64,
    /// CG at burnout (m from the tip).
    pub cg_burnout: f64,
    /// Whether the 3D solve is in the report (Extended is then alongside).
    pub three_d: bool,
    /// The angle of attack of Extended's values (rad).
    pub alpha: f64,
    /// The Mach sweep's ends and the top of the flight's Mach range (where the minimum is looked for).
    pub mach: (f64, f64, f64),
    /// The Mach table.
    pub rows: Vec<Row>,
    /// The summary lines: the minimum margins against the guidance and the thrust-to-weight.
    pub summary: Vec<String>,
    /// The provenance lines.
    pub provenance: Vec<String>,
}

/// What [`build`] is made of.
pub struct Source<'a> {
    /// The configuration's display name.
    pub configuration: &'a str,
    /// The date of the report.
    pub date: &'a str,
    /// The reference diameter (m).
    pub reference_diameter: f64,
    /// CG at liftoff and at burnout (m from the tip).
    pub cg: (f64, f64),
    /// The maximum Mach number of the configuration's nominal flight, `None` when it has none.
    pub max_mach: Option<f64>,
    /// The Mach sweep with Extended's stability, at `alpha` (rad).
    pub extended: &'a [MachPoint],
    /// The angle of attack of `extended` (rad).
    pub alpha: f64,
    /// The same sweep with the 3D solve's stability, with the solve; `None` when the report is Extended's alone.
    pub lift: Option<(&'a [MachPoint], &'a CfdLiftTable)>,
    /// The CFD drag table of the stack when it is the rocket's now.
    pub table: Option<&'a CfdTable>,
    /// Thrust-to-weight at liftoff, `None` when the configuration has no motor.
    pub thrust_to_weight: Option<f64>,
}

/// The margin (cal) asked for at `mach` (RASAero II guidance, the Simulate screen's [`required_margin_cal`]).
pub use crate::simulate::flight::required_margin_cal as guidance;

/// Whether `margin` (cal) falls short of the guidance at `mach`, judged on the two decimals the report prints.
pub fn below_guidance(margin: f64, mach: f64) -> bool {
    (margin * 100.0).round() < (guidance(mach) * 100.0).round()
}

/// Thrust-to-weight at liftoff: the average thrust of every stage that lights at launch over the weight of the whole loaded
/// stack. `None` without a motor.
pub fn thrust_to_weight(job: &ConfigJob) -> Option<f64> {
    let stages = &job.vehicle.stages;
    let mass: f64 = stages.iter().map(|s| s.dry.mass + s.motor.as_ref().map_or(0.0, |m| m.curve.total_mass)).sum();
    let thrust: f64 = stages
        .iter()
        .enumerate()
        .filter_map(|(i, s)| s.motor.as_ref().map(|m| (i, m)))
        .filter(|(i, m)| *i == 0 || (m.ignition.trigger == Trigger::Launch && m.ignition.delay <= 0.0))
        .filter(|(_, m)| m.curve.burn_time() > 0.0)
        .map(|(_, m)| m.curve.total_impulse() / m.curve.burn_time())
        .sum();
    (thrust > 0.0 && mass > 0.0).then_some(thrust / (mass * G0))
}

/// `2026-11-15` for a count of seconds since the Unix epoch (UTC).
pub fn date_utc(secs: u64) -> String {
    let z = (secs / 86_400) as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}-{month:02}-{day:02}")
}

/// A duration as `42 s` or `3.5 min`.
fn duration(seconds: f64) -> String {
    if seconds < 90.0 {
        format!("{seconds:.0} s")
    } else {
        format!("{:.1} min", seconds / 60.0)
    }
}

fn at(points: &[MachPoint], mach: f64) -> Option<Stab> {
    compute::mach_at(points, mach).map(|p| Stab { cp: p.cp, cn_alpha: p.cn_alpha })
}

/// The Mach numbers of the rows: every [`ROW_STEP`] inside `[lo, hi]` and every solved node, ascending.
fn row_machs(lo: f64, hi: f64, nodes: &[f64]) -> Vec<(f64, bool)> {
    let mut out: Vec<(f64, bool)> = Vec::new();
    let first = (lo / ROW_STEP - 1e-9).ceil() as i64;
    let last = (hi / ROW_STEP + 1e-9).floor() as i64;
    for k in first..=last {
        out.push((k as f64 * ROW_STEP, false));
    }
    for &n in nodes.iter().filter(|n| **n >= lo - 1e-9 && **n <= hi + 1e-9) {
        match out.iter_mut().find(|(m, _)| (m - n).abs() < 1e-6) {
            Some(row) => row.1 = true,
            None => out.push((n, true)),
        }
    }
    out.sort_by(|a, b| a.0.total_cmp(&b.0));
    out
}

/// The smallest margin of a source at one CG over the flight's Mach range, and the worst shortfall against the guidance.
struct Worst {
    margin: f64,
    mach: f64,
    /// `(Mach, margin, guidance)` of the point furthest below its guidance, when any is below.
    short: Option<(f64, f64, f64)>,
}

fn worst(points: &[MachPoint], cg: f64, d_ref: f64, hi: f64) -> Option<Worst> {
    let mut out: Option<Worst> = None;
    let mut deficit = 0.0;
    for p in points.iter().filter(|p| p.mach <= hi + 1e-9) {
        let margin = (p.cp - cg) / d_ref;
        let w = out.get_or_insert(Worst { margin, mach: p.mach, short: None });
        if margin < w.margin {
            w.margin = margin;
            w.mach = p.mach;
        }
        if below_guidance(margin, p.mach) && guidance(p.mach) - margin > deficit {
            deficit = guidance(p.mach) - margin;
            w.short = Some((p.mach, margin, guidance(p.mach)));
        }
    }
    out
}

fn margin_cell(margin: f64, mach: f64, flagged: bool) -> String {
    if flagged && below_guidance(margin, mach) {
        format!("{margin:.2} LOW")
    } else {
        format!("{margin:.2}")
    }
}

/// The page for `src`.
pub fn build(src: &Source) -> Result<Stability, String> {
    let ext_points = src.extended;
    if ext_points.is_empty() {
        return Err("the stability page needs the Mach sweep".to_string());
    }
    let used_points = src.lift.map_or(ext_points, |(points, _)| points);
    let three_d = src.lift.is_some();
    let d = src.reference_diameter;
    let (cg_lo, cg_bo) = src.cg;
    let (lo, last) = (used_points[0].mach, used_points[used_points.len() - 1].mach);
    let hi = src.max_mach.map_or(last, |m| m.clamp(lo, last));

    let mut nodes: Vec<f64> = src.lift.map_or_else(Vec::new, |(_, l)| l.points.iter().map(|p| p.mach).collect());
    nodes.sort_by(f64::total_cmp);
    nodes.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let rows: Vec<Row> = row_machs(lo, last, &nodes)
        .into_iter()
        .filter_map(|(mach, node)| Some(Row { mach, node, used: at(used_points, mach)?, ext: three_d.then(|| at(ext_points, mach)).flatten() }))
        .collect();

    let mut summary = Vec::new();
    let mut sources: Vec<(&str, &[MachPoint])> = vec![(if three_d { "3D" } else { "Extended" }, used_points)];
    if three_d {
        sources.push(("Extended", ext_points));
    }
    for (name, pts) in sources {
        for (cg_name, cg) in [("liftoff CG", cg_lo), ("burnout CG", cg_bo)] {
            let Some(wo) = worst(pts, cg, d, hi) else { continue };
            let flag = match wo.short {
                None => "meets the guidance over the flight Mach range".to_string(),
                Some((m, margin, need)) => format!("BELOW GUIDANCE: {margin:.2} cal at Mach {m:.2} where {need:.1} cal is asked for"),
            };
            summary.push(format!("{name}, {cg_name}: minimum margin {:.2} cal at Mach {:.2}; {flag}", wo.margin, wo.mach));
        }
    }
    if let Some(tw) = src.thrust_to_weight {
        let flag = if tw < TW_MINIMUM {
            format!("BELOW the {TW_MINIMUM:.0}:1 an RSO may approve")
        } else if tw < TW_TYPICAL {
            format!("below the typical {TW_TYPICAL:.0}:1; needs RSO approval down to {TW_MINIMUM:.0}:1")
        } else {
            format!("at or above the typical {TW_TYPICAL:.0}:1")
        };
        summary.push(format!("Thrust-to-weight at liftoff: {tw:.1}:1 (average thrust over loaded weight); {flag} (Tripoli Safety Code 7-1.4)"));
    }

    let mut provenance = Vec::new();
    provenance.push(format!(
        "Method: {}.",
        if three_d {
            "CFD table; margin, CP and CNalpha from the 3D Euler solve, Extended shown for comparison"
        } else {
            "Extended; margin, CP and CNalpha from the Barrowman build-up with body lift, no 3D solve"
        }
    ));
    if let Some(t) = src.table {
        provenance.push(format!(
            "Drag: 2D axisymmetric RANS, {} grid, {} Mach nodes, {} ({}), solved {}.",
            format!("{:?}", t.grid).to_lowercase(),
            t.points.len(),
            t.backend,
            duration(t.solve_seconds),
            date_utc(t.solved_at)
        ));
    }
    if let Some((_, l)) = src.lift {
        let mut machs: Vec<f64> = l.points.iter().map(|p| p.mach).collect();
        machs.sort_by(f64::total_cmp);
        machs.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        let mut alphas: Vec<f64> = l.points.iter().map(|p| p.alpha_deg).collect();
        alphas.sort_by(f64::total_cmp);
        alphas.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        let list = |v: &[f64], dec: usize| v.iter().map(|x| format!("{x:.dec$}")).collect::<Vec<_>>().join(", ");
        provenance.push(format!(
            "Stability: {} Euler points at Mach {} and {} degrees, {} ({}), solved {}; between nodes the values are interpolated in Mach.",
            l.points.len(),
            list(&machs, 2),
            list(&alphas, 0),
            l.backend,
            duration(l.solve_seconds),
            date_utc(l.solved_at)
        ));
        let weak = l.points.iter().filter(|p| p.quality != CfdQuality::Converged).count();
        if weak > 0 {
            provenance.push(format!("{weak} of {} points did not meet the full convergence test (stationary or stalled).", l.points.len()));
        }
        provenance.extend(l.notes.iter().cloned());
        provenance.push(format!("Validation: {VALIDATION}."));
    } else {
        provenance.push("Run Solve stability (3D) in Analyze with the CFD table method to add a 3D Euler solve to this report.".to_string());
    }
    if let Some(t) = src.table {
        provenance.extend(t.notes.iter().cloned());
    }

    Ok(Stability {
        configuration: src.configuration.to_string(),
        date: src.date.to_string(),
        reference_diameter: d,
        cg_liftoff: cg_lo,
        cg_burnout: cg_bo,
        three_d,
        alpha: src.alpha,
        mach: (lo, last, hi),
        rows,
        summary,
        provenance,
    })
}

/// Builds the page for the active configuration of `rocket`: the Extended sweep, and the 3D one when the solve is fresh. Runs the
/// (cheap) build-up sweeps and the configuration's nominal flight (for the top of its Mach range); meant for a worker thread.
pub fn compute(rocket: &Rocket, motors: &MotorDb, inputs: &Inputs) -> Result<Stability, String> {
    let (jobs, _) = compute::jobs(rocket, motors, Phase::Full)?;
    let active = rocket.active_config_id();
    let at = jobs.iter().position(|j| j.id == active).unwrap_or(0);
    let job = jobs.into_iter().nth(at).ok_or("no configuration to report on")?;
    let tw = thrust_to_weight(&job);
    let conditions = compute::Conditions::default();
    let output = compute::compute(&compute::Request {
        analysis: crate::analyze::Analysis::Aerodynamics,
        method: compute::Method::Extended,
        conditions,
        jobs: vec![job.clone()],
        bands: rocket.stability_bands(),
        cfd: None,
        drag_tables: Vec::new(),
    });
    let run = crate::analyze::readout::active_run(&output, job.id).ok_or("no configuration to report on")?;
    let compute::Data::Mach(extended) = &run.data else { return Err("the stability page needs the Mach sweep".to_string()) };
    let lift = inputs.table.as_ref().and_then(|t| t.lift.as_ref()).filter(|l| inputs.lift_fresh && cfd::lift_usable(l));
    let lift_points = lift.map(|l| {
        let ext_at_2 = compute::extended_at_2(&job.vehicle, job.cg_liftoff, conditions.altitude);
        (cfd::lift_mach_points(l, extended.clone(), job.cg_liftoff, run.reference_diameter, &ext_at_2), l)
    });
    build(&Source {
        configuration: &run.name,
        date: &inputs.date,
        reference_diameter: run.reference_diameter,
        cg: (run.cg_liftoff, run.cg_burnout),
        max_mach: run.max_mach,
        extended,
        alpha: conditions.alpha,
        lift: lift_points.as_ref().map(|(points, l)| (points.as_slice(), *l)),
        table: inputs.table.as_ref(),
        thrust_to_weight: tw,
    })
}

impl Stability {
    /// The facts above the tables: the configuration, the date, the caliber, the source of the numbers and the Mach ranges.
    pub fn facts(&self, u: UnitSystem) -> Vec<String> {
        let (lo, last, hi) = self.mach;
        let source = if self.three_d { "3D Euler solve (CFD (3D)), with Extended alongside" } else { "Extended (Barrowman build-up with body lift)" };
        vec![
            format!("Configuration: {}", self.configuration),
            format!("Date: {} (UTC)", self.date),
            format!("Reference diameter (1 cal): {}", units::with_unit(self.reference_diameter, Quantity::Length, u)),
            format!("Stability numbers from: {source}"),
            format!("Mach sweep: {lo:.2} to {last:.2}; flight Mach range used for the minimum: {lo:.2} to {hi:.2}"),
        ]
    }

    /// The CG table: `(condition, CG from the nose tip in the user's length unit, CG in calibers from the tip)`.
    pub fn cg_rows(&self, u: UnitSystem) -> Vec<Vec<String>> {
        [("Liftoff, motors loaded", self.cg_liftoff), ("Burnout", self.cg_burnout)]
            .iter()
            .map(|(name, cg)| vec![name.to_string(), units::len(*cg, u), format!("{:.2}", cg / self.reference_diameter)])
            .collect()
    }

    /// The Mach table: its header and its rows, the CP in the user's length unit. `LOW` marks a margin below the guidance at flight
    /// Mach numbers (rows above the flight's top Mach number are not flagged).
    pub fn table(&self, u: UnitSystem) -> (Vec<String>, Vec<Vec<String>>) {
        let unit = units::len_unit(u);
        let (d, hi) = (self.reference_diameter, self.mach.2);
        let mut header = vec!["Mach".to_string()];
        let sets: &[&str] = if self.three_d { &["3D ", "Ext "] } else { &[""] };
        for set in sets {
            header.extend([format!("{set}CP ({unit})"), format!("{set}LO"), format!("{set}BO"), format!("{set}CNa")]);
        }
        let rows = self
            .rows
            .iter()
            .map(|row| {
                let flagged = row.mach <= hi + 1e-9;
                let cells = |st: Stab| {
                    [
                        units::len(st.cp, u),
                        margin_cell((st.cp - self.cg_liftoff) / d, row.mach, flagged),
                        margin_cell((st.cp - self.cg_burnout) / d, row.mach, flagged),
                        format!("{:.2}", st.cn_alpha),
                    ]
                };
                let mut line = vec![format!("{:.2}{}", row.mach, if row.node { "*" } else { "" })];
                line.extend(cells(row.used));
                if self.three_d {
                    line.extend(row.ext.map_or_else(|| std::array::from_fn::<String, 4, _>(|_| "-".to_string()), cells));
                }
                line
            })
            .collect();
        (header, rows)
    }

    /// The text above the Mach table.
    pub fn table_note(&self) -> String {
        let alpha = self.alpha.to_degrees();
        let mut s = String::new();
        if self.three_d {
            let _ = write!(s, "CP and CNa (CNalpha, per radian on the reference area): 3D from the solves at 2 degrees, Extended at {alpha:.1} degrees. ");
        } else {
            let _ = write!(s, "Extended at {alpha:.1} degrees. CNa is CNalpha per radian on the reference area. ");
        }
        let _ = write!(
            s,
            "LO and BO are the margin in calibers at the liftoff and burnout CG; LOW marks a margin below the guidance at flight Mach numbers (rows above Mach {:.2} are outside the simulated flight and not flagged).",
            self.mach.2
        );
        if self.three_d {
            s.push_str(" * marks a Mach number the 3D solve was run at.");
        }
        s
    }
}

/// The margin guidance, as the page states it.
pub fn guidance_note() -> String {
    format!(
        "Margin = (CP - CG) / reference diameter. Guidance: at least {:.1} cal below Mach {:.1}, at least {:.1} cal from Mach {:.1} up (RASAero guidance).",
        guidance(0.0),
        crate::simulate::flight::TRANSONIC_MACH,
        guidance(10.0),
        crate::simulate::flight::TRANSONIC_MACH
    )
}

/// How the numbers were obtained, for the end of the page.
pub const HOW_MEASURED: &str = "The centre of pressure and the CG in this report are calculated from the rocket's model, not measured on the built rocket. The CG comes from the component masses and positions in the document (with the motor loaded and burned out); the CP is where the normal force acts at a small angle of attack, taken from the source named above. Weigh and balance the built rocket and compare its CG with the figures above; this page does not replace that check.";

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{CfdGrid, CfdLiftPoint};
    use bowshock_sim::aero::DragBreakdown;

    const D: f64 = 0.1;
    const CG_LIFTOFF: f64 = 1.0;
    const CG_BURNOUT: f64 = 0.9;

    fn point(mach: f64, margin_liftoff: f64) -> MachPoint {
        MachPoint { mach, margin_cal: margin_liftoff, cp: CG_LIFTOFF + margin_liftoff * D, cn_alpha: 12.0, drag: DragBreakdown::default() }
    }

    fn sweep(margin: impl Fn(f64) -> f64) -> Vec<MachPoint> {
        (0..=29).map(|i| 0.1 + 0.1 * i as f64).map(|m| point(m, margin(m))).collect()
    }

    fn lift_table() -> CfdLiftTable {
        let p = |mach: f64, alpha_deg: f64| CfdLiftPoint { mach, alpha_deg, cn: 0.4, x_cp: 1.2, quality: CfdQuality::Converged };
        CfdLiftTable {
            key: 1,
            solved_at: 86_400,
            solve_seconds: 240.0,
            backend: "GPU (Luma)".into(),
            points: vec![p(0.5, 2.0), p(0.5, 6.0), p(0.95, 2.0), p(0.95, 6.0), p(1.7, 2.0), p(1.7, 6.0)],
            notes: Vec::new(),
        }
    }

    fn build_with(extended: &[MachPoint], lift: Option<(&[MachPoint], &CfdLiftTable)>, tw: Option<f64>) -> Stability {
        build(&Source {
            configuration: "Test config",
            date: "2026-11-15",
            reference_diameter: D,
            cg: (CG_LIFTOFF, CG_BURNOUT),
            max_mach: Some(2.0),
            extended,
            alpha: 0.0,
            lift,
            table: None,
            thrust_to_weight: tw,
        })
        .unwrap()
    }

    fn extended_only(margin: impl Fn(f64) -> f64, tw: Option<f64>) -> Stability {
        build_with(&sweep(margin), None, tw)
    }

    fn row<'a>(rows: &'a [Vec<String>], mach: &str) -> &'a Vec<String> {
        rows.iter().find(|r| r[0] == mach).unwrap_or_else(|| panic!("no row {mach}"))
    }

    #[test]
    fn rows_cover_every_lift_node_with_both_sources() {
        let ext = sweep(|_| 2.5);
        let used = sweep(|_| 3.0);
        let lift = lift_table();
        let st = build_with(&ext, Some((&used, &lift)), None);
        let (header, rows) = st.table(UnitSystem::MM_G_S);
        for node in ["0.50*", "0.95*", "1.70*"] {
            assert!(rows.iter().any(|r| r[0] == node), "{node} missing");
        }
        assert!(rows.iter().any(|r| r[0] == "1.00") && !rows.iter().any(|r| r[0] == "1.00*"));
        assert_eq!(header[1], "3D CP (mm)");
        assert_eq!(header.len(), 9);
        assert!(header.iter().any(|h| h == "Ext CNa"));
        // The 3D margin (3.0 cal) and Extended's (2.5 cal) sit side by side on a row.
        let r = row(&rows, "0.95*");
        assert_eq!((r[2].as_str(), r[6].as_str()), ("3.00", "2.50"), "{r:?}");
        assert!(st.provenance.iter().any(|l| l.starts_with("Validation: 3D Euler; validated on the Army-Navy Basic Finner")));
        assert!(st.provenance[0].contains("3D Euler solve") && st.table_note().contains("* marks a Mach number"));
    }

    #[test]
    fn without_a_lift_table_only_extended_is_reported() {
        let st = extended_only(|_| 2.5, None);
        let (header, rows) = st.table(UnitSystem::MM_G_S);
        assert_eq!(header, ["Mach", "CP (mm)", "LO", "BO", "CNa"]);
        assert!(!st.three_d && !st.table_note().contains('*'));
        assert!(!st.provenance.iter().any(|l| l.starts_with("Validation:")));
        assert!(st.provenance.iter().any(|l| l.contains("Run Solve stability (3D)")));
        assert!(rows.iter().any(|r| r[0] == "0.10") && rows.iter().any(|r| r[0] == "3.00"));
        assert!(rows.iter().all(|r| r.len() == 5));
    }

    #[test]
    fn the_cp_is_in_the_users_unit() {
        let st = extended_only(|_| 2.5, None);
        // CP 1.25 m: 1250.0 mm, 49.2 in.
        assert_eq!(row(&st.table(UnitSystem::MM_G_S).1, "1.00")[1], "1250.0");
        assert_eq!(st.table(UnitSystem::IN_OZ_S).0[1], "CP (in)");
        let cg = st.cg_rows(UnitSystem::MM_G_S);
        assert_eq!((cg[0][1].as_str(), cg[0][2].as_str()), ("1000.0", "10.00"), "{cg:?}");
        assert_eq!(cg[1][1], "900.0");
    }

    #[test]
    fn flags_fire_at_the_thresholds() {
        // 1.00 cal below Mach 0.8 passes, 0.99 fails; 2.00 from Mach 0.8 passes, 1.99 fails.
        assert!(!below_guidance(1.0, 0.5) && below_guidance(0.99, 0.5));
        assert!(!below_guidance(2.0, 0.8) && below_guidance(1.99, 0.8));
        assert!(!below_guidance(1.5, 0.79) && below_guidance(1.5, 0.8));
        // 1.9 cal everywhere: fine below 0.8, short from 0.8 up (flight Mach range ends at 2.0).
        let st = extended_only(|_| 1.9, None);
        assert!(st.summary.iter().any(|l| l.contains("BELOW GUIDANCE: 1.90 cal at Mach 0.80 where 2.0 cal is asked for")), "{:?}", st.summary);
        let rows = st.table(UnitSystem::MM_G_S).1;
        assert!(!row(&rows, "0.70")[2].contains("LOW") && row(&rows, "0.80")[2].contains("1.90 LOW") && row(&rows, "1.90")[2].contains("LOW"));
        // Above the flight's maximum Mach the rows are not flagged.
        assert!(!row(&rows, "2.50")[2].contains("LOW"));
        // Plenty of margin: no flag.
        let ok = extended_only(|_| 2.4, None);
        assert!(!ok.summary.iter().any(|l| l.contains("BELOW GUIDANCE")) && !ok.table(UnitSystem::MM_G_S).1.iter().flatten().any(|c| c.contains("LOW")));
        // The minimum is named with its Mach number, at both CGs: the burnout CG (0.1 m forward of the liftoff one, one caliber)
        // adds one caliber of margin.
        let dip = extended_only(|m| if (m - 1.1).abs() < 1e-9 { 1.4 } else { 2.5 }, None);
        assert!(dip.summary.iter().any(|l| l.contains("minimum margin 1.40 cal at Mach 1.10")), "{:?}", dip.summary);
        assert!(dip.summary.iter().any(|l| l.contains("burnout CG: minimum margin 2.40 cal at Mach 1.10")), "{:?}", dip.summary);
    }

    #[test]
    fn thrust_to_weight_is_flagged_and_omitted_without_a_motor() {
        let line = |tw| extended_only(|_| 2.5, tw).summary.into_iter().find(|l| l.contains("Thrust-to-weight"));
        assert_eq!(line(None), None);
        assert!(line(Some(6.2)).unwrap().contains("6.2:1") && line(Some(6.2)).unwrap().contains("at or above the typical 5:1"));
        assert!(line(Some(4.0)).unwrap().contains("below the typical 5:1; needs RSO approval"));
        assert!(line(Some(2.5)).unwrap().contains("BELOW the 3:1"));
        // The thrust-to-weight of a document's configuration: the example flies on a motor, a stack without stages has none.
        let rocket = Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        let mut job = compute::jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap().0.remove(0);
        let tw = thrust_to_weight(&job).expect("the example has a motor");
        assert!((1.0..60.0).contains(&tw), "{tw}");
        job.vehicle.stages.clear();
        assert_eq!(thrust_to_weight(&job), None);
    }

    #[test]
    fn the_page_is_made_from_the_example_and_names_its_drag_table() {
        let rocket = Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        let table = CfdTable {
            key: 1,
            geometry: 1,
            stage: 1,
            solved_at: 86_400,
            solve_seconds: 600.0,
            backend: "GPU (Luma)".into(),
            fallback: None,
            altitude: 120.0,
            notes: vec!["a note".into()],
            points: Vec::new(),
            grid: CfdGrid::Coarse,
            lift: None,
        };
        let inputs = Inputs { table: Some(table), lift_fresh: false, date: "2026-11-15".into() };
        let st = compute(&rocket, &MotorDb::bundled(), &inputs).unwrap();
        assert!(!st.three_d && st.rows.len() >= 30 && st.mach.2 > 0.0 && st.mach.2 <= st.mach.1);
        assert!(st.cg_burnout < st.cg_liftoff + 1.0 && st.reference_diameter > 0.0);
        assert!(st.summary.iter().any(|l| l.contains("Thrust-to-weight")), "{:?}", st.summary);
        assert!(st.provenance.iter().any(|l| l.starts_with("Drag: 2D axisymmetric RANS, coarse grid, 0 Mach nodes, GPU (Luma) (10.0 min), solved 1970-01-02")));
        assert!(st.provenance.iter().any(|l| l == "a note"));
    }

    #[test]
    fn a_fresh_3d_solve_is_in_the_page_and_a_stale_one_is_not() {
        let rocket = Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        let table = CfdTable {
            key: 1,
            geometry: 1,
            stage: 1,
            solved_at: 86_400,
            solve_seconds: 600.0,
            backend: "GPU (Luma)".into(),
            fallback: None,
            altitude: 120.0,
            notes: Vec::new(),
            points: Vec::new(),
            grid: CfdGrid::Coarse,
            lift: Some(lift_table()),
        };
        let page = |lift_fresh| compute(&rocket, &MotorDb::bundled(), &Inputs { table: Some(table.clone()), lift_fresh, date: "2026-11-15".into() }).unwrap();
        let fresh = page(true);
        assert!(fresh.three_d && fresh.rows.iter().all(|r| r.ext.is_some()) && fresh.rows.iter().filter(|r| r.node).count() == 3);
        // The solve's CP (1.2 m at every node, held above the last) is what the 3D columns carry.
        let last = fresh.rows.last().unwrap();
        assert!((last.used.cp - 1.2).abs() < 1e-9 && (last.used.cn_alpha - 0.4 / 2f64.to_radians()).abs() < 1e-9, "{last:?}");
        assert!(fresh.provenance.iter().any(|l| l.starts_with("Validation:")));
        let stale = page(false);
        assert!(!stale.three_d && stale.rows.iter().all(|r| r.ext.is_none() && !r.node));
    }

    #[test]
    fn dates_are_utc_calendar_dates() {
        assert_eq!(date_utc(0), "1970-01-01");
        assert_eq!(date_utc(951_782_400), "2000-02-29");
        assert_eq!(date_utc(1_794_700_800), "2026-11-15");
    }
}
