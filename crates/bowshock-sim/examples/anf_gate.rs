//! Stability gate: how far is the Extended (Barrowman build-up) CP and CNα from the Army-Navy Basic Finner data?
//!
//! ```text
//! cargo run --release -p bowshock-sim --example anf_gate [-- <openrocket comparison.md>]
//! ```
//!
//! Builds the ANF directly (geometry from [`bowshock_reference::anf`]), evaluates [`bowshock_sim::vs_mach`] at each data Mach
//! (α = 2°, sea level) and prints/writes `out/gate/anf_gate.md`: one table for the wind tunnel, one for free flight, the
//! OpenRocket comparison summary (from `tools/compare-openrocket/run.sh`, default `/tmp/bowshock-openrocket/comparison.md`)
//! and the checkpoint decision. Gate: Extended within [`CP_TOL_CAL`] cal of the data x_cp and [`CNA_TOL`] of the data CNα at
//! every data Mach.

use std::fmt::Write as _;
use std::path::PathBuf;

use bowshock_reference::anf::{self, AnfPoint};
use bowshock_sim::vehicle::{BodyKind, BodySegment, FinCrossSection, FinPlanform, FinSet, NoseShape, Vehicle};
use bowshock_sim::{vs_mach, Atmosphere};

const CP_TOL_CAL: f64 = 0.15;
const CNA_TOL: f64 = 0.08;
/// One caliber in metres (Dupuis's model); Extended is scale-free apart from Reynolds number.
const CAL: f64 = 0.03;

fn anf_vehicle() -> Vehicle {
    let r = 0.5 * CAL;
    let nose = anf::NOSE_LENGTH_CAL * CAL;
    let length = anf::LENGTH_CAL * CAL;
    Vehicle {
        reference_diameter: CAL,
        body: vec![
            BodySegment { station: 0.0, length: nose, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::Conical), roughness: 0.0 },
            BodySegment { station: nose, length: length - nose, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 0.0 },
        ],
        tube_fins: vec![],
        fin_sets: vec![FinSet {
            count: anf::FIN_COUNT,
            // Root leading edge from the tip: the fins are flush with the base.
            station: length - anf::FIN_CHORD_CAL * CAL,
            body_radius: r,
            root_chord: anf::FIN_CHORD_CAL * CAL,
            tip_chord: anf::FIN_CHORD_CAL * CAL,
            span: anf::FIN_SPAN_CAL * CAL,
            sweep: 0.0,
            thickness: anf::FIN_THICKNESS_CAL * CAL,
            cross_section: FinCrossSection::Square,
            cant: 0.0,
            roughness: 0.0,
            planform: FinPlanform::Trapezoidal,
            angles: Vec::new(),
            parent: 0,
            bodies: 1,
            mid_chord_cos: None,
            root_extent: None,
        }],
        protuberances: vec![],
        stages: vec![],
        labels: Default::default(),
    }
}

struct Row {
    p: AnfPoint,
    cna: f64,
    cp_cal: f64,
    dcna: f64,
    dcp: f64,
}

impl Row {
    fn misses(&self) -> bool {
        self.dcp.abs() > CP_TOL_CAL || self.dcna.abs() > CNA_TOL * 100.0
    }
}

fn rows(v: &Vehicle, data: &[AnfPoint]) -> Vec<Row> {
    let cg = anf::CG_CAL * CAL;
    let atm = Atmosphere::standard();
    data.iter()
        .map(|&p| {
            let pt = &vs_mach(v, cg, (p.mach, p.mach), 1, 2f64.to_radians(), &atm, 0.0)[0];
            let cp_cal = pt.cp / CAL;
            Row { p, cna: pt.cn_alpha, cp_cal, dcna: 100.0 * (pt.cn_alpha / p.cn_alpha - 1.0), dcp: cp_cal - p.x_cp_cal }
        })
        .collect()
}

fn table(out: &mut String, title: &str, rows: &[Row]) {
    writeln!(out, "\n### {title}\n").unwrap();
    writeln!(out, "| Mach | data CNα | Ext CNα | Δ % | data x_cp (cal) | Ext x_cp | Δ cal | gate |").unwrap();
    writeln!(out, "|---|---|---|---|---|---|---|---|").unwrap();
    for r in rows {
        writeln!(
            out,
            "| {:.3} | {:.2} | {:.2} | {:+.1} | {:.2} | {:.2} | {:+.2} | {} |",
            r.p.mach,
            r.p.cn_alpha,
            r.cna,
            r.dcna,
            r.p.x_cp_cal,
            r.cp_cal,
            r.dcp,
            if r.misses() { "MISS" } else { "ok" }
        )
        .unwrap();
    }
}

/// Mach ranges of consecutive missing points: `0.69–0.79 (cp, cna)`.
fn miss_ranges(label: &str, rows: &[Row]) -> Vec<String> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < rows.len() {
        if !rows[i].misses() {
            i += 1;
            continue;
        }
        let mut j = i;
        while j + 1 < rows.len() && rows[j + 1].misses() {
            j += 1;
        }
        let run = &rows[i..=j];
        let cp = run.iter().filter(|r| r.dcp.abs() > CP_TOL_CAL).count();
        let cn = run.iter().filter(|r| r.dcna.abs() > CNA_TOL * 100.0).count();
        let worst_cp = run.iter().map(|r| r.dcp.abs()).fold(0.0, f64::max);
        let worst_cn = run.iter().map(|r| r.dcna.abs()).fold(0.0, f64::max);
        out.push(format!(
            "{label}: M {:.2}–{:.2} ({} pts; x_cp misses {cp}, worst {worst_cp:.2} cal; CNα misses {cn}, worst {worst_cn:.1} %)",
            run[0].p.mach,
            run[run.len() - 1].p.mach,
            run.len()
        ));
        i = j + 1;
    }
    out
}

fn main() {
    let v = anf_vehicle();
    let wt = rows(&v, anf::WIND_TUNNEL);
    let ff = rows(&v, anf::FREE_FLIGHT);

    let mut md = String::new();
    writeln!(md, "# ANF stability gate: Extended vs Basic Finner data\n").unwrap();
    writeln!(md, "Extended = `bowshock_sim::vs_mach` (Barrowman build-up), α = 2°, ISA sea level, CG {} cal, reference area π d²/4, CNα per rad.", anf::CG_CAL)
        .unwrap();
    writeln!(md, "Data digitised from Yayla 2021 Fig. 3.10/4.4 (Dupuis WT and FF), ±3 % on CNα; data x_cp = x_cg − Cmα/CNα, about ±0.1 cal. See `crates/bowshock-reference/src/anf.rs`.").unwrap();
    writeln!(md, "Gate: |Δ x_cp| ≤ {CP_TOL_CAL} cal and |Δ CNα| ≤ {:.0} % at every data Mach.", CNA_TOL * 100.0).unwrap();
    table(&mut md, "Wind tunnel", &wt);
    table(&mut md, "Free flight", &ff);

    let mean = |r: &[Row]| r.iter().map(|x| x.dcp.abs()).sum::<f64>() / r.len() as f64;
    writeln!(md, "\nMean |Δ x_cp|: wind tunnel {:.2} cal, free flight {:.2} cal (Extended's baseline for the G2 comparison).", mean(&wt), mean(&ff)).unwrap();

    let or_path = std::env::args().nth(1).map(PathBuf::from).unwrap_or_else(|| PathBuf::from("/tmp/bowshock-openrocket/comparison.md"));
    writeln!(md, "\n## OpenRocket comparison (Bowshock Extended vs OpenRocket 23.09, `tools/compare-openrocket/run.sh`)\n").unwrap();
    match std::fs::read_to_string(&or_path) {
        Ok(text) => match text.find("## Summary") {
            Some(i) => {
                writeln!(md, "Source: `{}` (rows are Bowshock minus OpenRocket, worst over the Mach sweep).\n", or_path.display()).unwrap();
                md.push_str(text[i..].trim_start_matches("## Summary").trim());
                md.push('\n');
            }
            None => writeln!(md, "`{}` has no Summary section.", or_path.display()).unwrap(),
        },
        Err(e) => {
            writeln!(md, "Not available: could not read `{}` ({e}); run `OPENROCKET_JAR=<jar> tools/compare-openrocket/run.sh` first.", or_path.display())
                .unwrap()
        }
    }

    let mut misses = miss_ranges("wind tunnel", &wt);
    misses.extend(miss_ranges("free flight", &ff));
    writeln!(md, "\n## Checkpoint decision\n").unwrap();
    if misses.is_empty() {
        writeln!(md, "**PASS**: Extended is within {CP_TOL_CAL} cal x_cp and {:.0} % CNα at every data Mach (wind tunnel and free flight). Stop after B1 and present this table to the user before starting C3.", CNA_TOL * 100.0).unwrap();
    } else {
        writeln!(md, "**FAIL**: Extended misses the {CP_TOL_CAL} cal / {:.0} % thresholds in these Mach ranges; continue with the 3D solver and have G2 report 3D error against exactly these:\n", CNA_TOL * 100.0).unwrap();
        for m in &misses {
            writeln!(md, "- {m}").unwrap();
        }
    }

    print!("{md}");
    std::fs::create_dir_all("out/gate").expect("create out/gate");
    std::fs::write("out/gate/anf_gate.md", &md).expect("write out/gate/anf_gate.md");
}
