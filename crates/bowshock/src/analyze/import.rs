//! Readers for the aerodynamic CSV files of other tools, so Analyze can overlay them on its own curves (`Compare ▾`).
//!
//! Columns are found by header name, never by position, and unit suffixes in headers are honoured. The names this reads come
//! from the tools' own documentation:
//!
//! * **OpenRocket**: its simulation export ("Plot / Export → Export data") writes one row per time step with a header such as
//!   `# Time (s),Mach number (),Angle of attack (°),Drag coefficient (CD) (),CP location (cm),Stability margin calibers ()`; the
//!   field names are those of OpenRocket's `FlightDataType`s, as defined in
//!   `openrocket/core/src/main/resources/l10n/messages.properties` (`Time`, `Mach number`, `Drag coefficient (CD)`,
//!   `CP location`, `CG location`, `Stability margin calibers`, `Angle of attack`; the `(CD)` is part of the name, the unit
//!   group is appended after it and may be `()`) and the user guide, *Advanced flight simulation*. The header row is written
//!   as a `#` comment line, which is read as the header when it is the one that
//!   names the columns. The drag / stability *lookup table* files of the same guide (`Mach,AoA,Cd` and `Mach,AoA,Cn,Cm,Cp`;
//!   "names are case-insensitive, spaces and underscores are ignored", `#` lines are comments) are read too.
//! * **RASAero II**: *Aero Plots → File → Export → To CSV File* (RASAero II Users Manual, 2019, p. 76) writes `Mach`,
//!   `Alpha`, `CN`, `CN Potential`, `CN Viscous`, `CP`, `CNalpha (0 to 4 deg)` and `CP (0 to 4 deg)`; the drag export adds
//!   `CD Power-Off` and `CD Power-On`. RASAero measures lengths in inches from the nose (Users Manual p. 13, p. 114), so a
//!   CP without a unit in its header is inches there and metres for OpenRocket.
//!
//! A delimiter of `,`, `;` or tab is detected from the header; with `;` or tab a comma is a decimal comma (`0,25`).

use super::overlay::{Overlay, Sample};

/// Whose file this is; sets the unit of a CP column that names none.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// An OpenRocket export.
    OpenRocket,
    /// A RASAero II export.
    RasAero,
}

impl Source {
    /// The label drawn on the overlaid series.
    pub fn label(self) -> &'static str {
        match self {
            Source::OpenRocket => "OpenRocket",
            Source::RasAero => "RASAero",
        }
    }

    /// Metres per unit of a CP column without a unit in its header.
    fn default_length(self) -> f64 {
        match self {
            Source::OpenRocket => 1.0,
            Source::RasAero => 0.0254,
        }
    }
}

/// One row of an import, in SI (angles in radians, CP in metres from the nose).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Row {
    /// Mach number.
    pub mach: Option<f64>,
    /// Angle of attack (rad).
    pub alpha: Option<f64>,
    /// Drag coefficient.
    pub cd: Option<f64>,
    /// Centre of pressure from the nose (m).
    pub cp: Option<f64>,
    /// Static margin (calibers).
    pub margin: Option<f64>,
    /// Normal force slope (/rad).
    pub cn_alpha: Option<f64>,
    /// Normal force coefficient.
    pub cn: Option<f64>,
    /// Pitching-moment coefficient (the lookup table's `Cm`).
    pub cm: Option<f64>,
}

/// A parsed file.
#[derive(Debug, Clone, PartialEq)]
pub struct Imported {
    /// Whose file it is.
    pub source: Source,
    /// The rows in file order.
    pub rows: Vec<Row>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Col {
    Mach,
    Alpha,
    Cd,
    Cp,
    Margin,
    CnAlpha,
    Cn,
    Cm,
}

/// Which column a header names, with a rank (lower wins when several columns name the same thing).
fn classify(key: &str) -> Option<(Col, u8)> {
    Some(match key {
        "mach" | "machnumber" => (Col::Mach, 0),
        "alpha" | "aoa" | "angleofattack" => (Col::Alpha, 0),
        "cd" | "dragcoefficient" | "dragcoefficientcd" | "totaldragcoefficient" => (Col::Cd, 0),
        "cdpoweroff" => (Col::Cd, 1),
        "cdpoweron" => (Col::Cd, 2),
        "cp" | "cplocation" | "centerofpressure" | "centreofpressure" => (Col::Cp, 0),
        "stabilitymargincalibers" | "stabilitymargin" | "staticmargin" | "margin" => (Col::Margin, 0),
        "cnalpha" | "cna" => (Col::CnAlpha, 0),
        "cnalpha0to4deg" => (Col::CnAlpha, 1),
        "cn" | "normalforcecoefficient" => (Col::Cn, 0),
        "cm" | "pitchingmomentcoefficient" => (Col::Cm, 0),
        _ => return None,
    })
}

/// Metres per unit for a length unit token.
fn length_unit(u: &str) -> Option<f64> {
    Some(match u {
        "m" => 1.0,
        "cm" => 0.01,
        "mm" => 0.001,
        "in" | "\"" | "inch" | "inches" => 0.0254,
        "ft" | "feet" => 0.3048,
        _ => return None,
    })
}

/// Whether a parenthesised header suffix is a unit (so `CP (cm)` loses it but `CP (0 to 4 deg)` keeps it).
fn is_unit(u: &str) -> bool {
    length_unit(u).is_some() || matches!(u, "" | "deg" | "°" | "rad" | "cal" | "calibers" | "-" | "1" | "/rad" | "/deg" | "/°")
}

/// A header cell as (lower-case name with only letters and digits, unit if it had one).
fn split_header(cell: &str) -> (String, Option<String>) {
    let cell = cell.trim().trim_matches('"').trim_start_matches('#').trim();
    let mut name = cell;
    let mut unit = None;
    if let Some(end) = cell.rfind([')', ']']) {
        if end + 1 == cell.len() {
            if let Some(start) = cell.rfind(['(', '[']) {
                let inner = cell[start + 1..end].trim().to_lowercase();
                if is_unit(&inner) {
                    name = cell[..start].trim();
                    unit = Some(inner);
                }
            }
        }
    }
    (name.chars().filter(|c| c.is_alphanumeric()).collect::<String>().to_lowercase(), unit)
}

fn delimiter(line: &str) -> char {
    [';', '\t', ','].into_iter().max_by_key(|d| line.matches(*d).count()).filter(|d| line.contains(*d)).unwrap_or(',')
}

fn cells(line: &str, delim: char) -> Vec<&str> {
    line.split(delim).map(|c| c.trim().trim_matches('"')).collect()
}

fn number(cell: &str, delim: char) -> Option<f64> {
    let v = if delim == ',' { cell.parse::<f64>().ok() } else { cell.replace(',', ".").parse::<f64>().ok() };
    v.filter(|v| v.is_finite())
}

struct Header {
    delim: char,
    /// For each column index: what it is and the factor to SI.
    cols: Vec<Option<(Col, f64)>>,
}

fn read_header(line: &str, source: Source) -> Header {
    let delim = delimiter(line.trim_start_matches('#'));
    let parsed: Vec<_> = cells(line.trim_start_matches('#'), delim).into_iter().map(split_header).collect();
    // Keep only the best-ranked column of each kind.
    let mut best: Vec<(Col, u8, usize)> = Vec::new();
    for (i, (key, _)) in parsed.iter().enumerate() {
        if let Some((col, rank)) = classify(key) {
            match best.iter_mut().find(|b| b.0 == col) {
                Some(b) if rank < b.1 => *b = (col, rank, i),
                Some(_) => {}
                None => best.push((col, rank, i)),
            }
        }
    }
    let mut cols = vec![None; parsed.len()];
    for (col, _, i) in best {
        let unit = parsed[i].1.as_deref();
        let factor = match col {
            Col::Cp => unit.and_then(length_unit).unwrap_or(source.default_length()),
            Col::Alpha => {
                if unit == Some("rad") {
                    1.0
                } else {
                    std::f64::consts::PI / 180.0
                }
            }
            Col::CnAlpha if unit == Some("/deg") => 180.0 / std::f64::consts::PI,
            _ => 1.0,
        };
        cols[i] = Some((col, factor));
    }
    Header { delim, cols }
}

fn names_a_curve(h: &Header) -> bool {
    h.cols.iter().flatten().any(|(c, _)| matches!(c, Col::Mach | Col::Alpha))
}

/// Reads `text` as an export of `source`. Errors say what is missing.
pub fn parse(text: &str, source: Source) -> Result<Imported, String> {
    let lines: Vec<&str> = text.lines().map(str::trim).filter(|l| !l.is_empty()).collect();
    let body: Vec<&str> = lines.iter().copied().filter(|l| !l.starts_with('#')).collect();
    let comments: Vec<&str> = lines.iter().copied().filter(|l| l.starts_with('#')).collect();
    let first = body.first().ok_or("the file has no data")?;
    let mut header = read_header(first, source);
    let mut data = &body[1..];
    if !names_a_curve(&header) {
        // OpenRocket writes the column names as a `#` comment line: the last comment that names a Mach or angle column.
        let alt = comments.iter().rev().map(|c| read_header(c, source)).find(names_a_curve);
        match alt {
            Some(h) => {
                header = h;
                data = &body[..];
            }
            None => return Err("no Mach or angle-of-attack column found in the header".into()),
        }
    }
    let mut rows = Vec::new();
    for line in data {
        let mut row = Row::default();
        for (cell, spec) in cells(line, header.delim).into_iter().zip(&header.cols) {
            let Some((col, factor)) = spec else { continue };
            let Some(v) = number(cell, header.delim) else { continue };
            let v = v * factor;
            match col {
                Col::Mach => row.mach = Some(v),
                Col::Alpha => row.alpha = Some(v),
                Col::Cd => row.cd = Some(v),
                Col::Cp => row.cp = Some(v),
                Col::Margin => row.margin = Some(v),
                Col::CnAlpha => row.cn_alpha = Some(v),
                Col::Cn => row.cn = Some(v),
                Col::Cm => row.cm = Some(v),
            }
        }
        if row.mach.is_some() || row.alpha.is_some() {
            rows.push(row);
        }
    }
    if rows.is_empty() {
        return Err("no numeric rows under the header".into());
    }
    if rows.iter().all(|r| r.cd.is_none() && r.cp.is_none() && r.margin.is_none() && r.cn.is_none() && r.cn_alpha.is_none() && r.cm.is_none()) {
        return Err("no drag, CP, margin or normal-force column found".into());
    }
    Ok(Imported { source, rows })
}

/// The rows of `rows` whose `key` is the value nearest `target` (rounded to 1e-3); all rows when that leaves fewer than three
/// (a simulation export, where nothing repeats).
fn nearest_group(rows: Vec<Row>, key: impl Fn(&Row) -> Option<f64>, target: f64) -> Vec<Row> {
    let q = |v: f64| (v * 1e3).round() as i64;
    let Some(best) = rows.iter().filter_map(&key).min_by(|a, b| (a - target).abs().total_cmp(&(b - target).abs())) else { return rows };
    let group: Vec<Row> = rows.iter().copied().filter(|r| key(r).is_some_and(|v| q(v) == q(best))).collect();
    if group.len() >= 3 {
        group
    } else {
        rows
    }
}

impl Imported {
    /// The overlay curves: against Mach at the angle nearest `alpha` (rad), against angle at the Mach nearest `mach`.
    pub fn overlay(&self, alpha: f64, mach: f64) -> Overlay {
        let sample = |x: f64, r: &Row| Sample { x, margin: r.margin, cp: r.cp, cd: r.cd, cn_alpha: r.cn_alpha, cn: r.cn };
        let with_mach: Vec<Row> = self.rows.iter().copied().filter(|r| r.mach.is_some()).collect();
        let by_mach = if with_mach.iter().any(|r| r.alpha.is_some()) { nearest_group(with_mach, |r| r.alpha, alpha) } else { with_mach };
        let mut mach_curve: Vec<Sample> = by_mach.iter().filter_map(|r| Some(sample(r.mach?, r))).collect();
        mach_curve.sort_by(|a, b| a.x.total_cmp(&b.x));

        let with_alpha: Vec<Row> = self.rows.iter().copied().filter(|r| r.alpha.is_some()).collect();
        let by_alpha = if with_alpha.iter().any(|r| r.mach.is_some()) { nearest_group(with_alpha, |r| r.mach, mach) } else { with_alpha };
        let mut alpha_curve: Vec<Sample> = by_alpha.iter().filter_map(|r| Some(sample(r.alpha?, r))).collect();
        alpha_curve.sort_by(|a, b| a.x.total_cmp(&b.x));
        // A file at one angle (a zero-angle simulation) has no angle curve.
        if alpha_curve.first().zip(alpha_curve.last()).is_none_or(|(a, b)| b.x - a.x < 1e-9) {
            alpha_curve.clear();
        }
        Overlay { label: self.source.label().to_string(), mach: mach_curve, alpha: alpha_curve }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OPENROCKET: &str = include_str!("../../assets/test-data/openrocket-sim.csv");
    const RASAERO: &str = include_str!("../../assets/test-data/rasaero-aero.csv");
    const LOOKUP: &str = include_str!("../../assets/test-data/openrocket-lookup-semicolon.csv");

    #[test]
    fn openrocket_header_is_a_comment_and_cp_is_in_centimetres() {
        let f = parse(OPENROCKET, Source::OpenRocket).unwrap();
        assert_eq!(f.rows.len(), 9);
        let r = &f.rows[2];
        assert_eq!(r.mach, Some(0.5));
        assert!((r.cp.unwrap() - 0.851).abs() < 1e-9, "{:?}", r.cp); // 85.1 cm
        assert_eq!(r.margin, Some(1.85));
        assert_eq!(r.cd, Some(0.545));
        assert!((r.alpha.unwrap() - 0.0).abs() < 1e-12);
    }

    #[test]
    fn rasaero_cp_is_inches_and_power_off_drag_wins() {
        let f = parse(RASAERO, Source::RasAero).unwrap();
        let r = f.rows.iter().find(|r| r.mach == Some(0.5) && (r.alpha.unwrap().to_degrees() - 2.0).abs() < 1e-9).unwrap();
        assert!((r.cp.unwrap() - 34.3 * 0.0254).abs() < 1e-9);
        assert_eq!(r.cd, Some(0.5640)); // Power-Off, not the Power-On 0.49
        assert_eq!(r.cn, Some(0.1031));
        // `CNalpha (0 to 4 deg)` is the secant, kept only as a fallback; the header names it.
        assert!(r.cn_alpha.is_some());
    }

    #[test]
    fn decimal_commas_with_semicolons_and_units() {
        let f = parse(LOOKUP, Source::OpenRocket).unwrap();
        assert_eq!(f.rows.len(), 4);
        assert_eq!(f.rows[1].cd, Some(0.55));
        assert!((f.rows[1].cp.unwrap() - 0.265).abs() < 1e-9); // 265 mm
        assert!((f.rows[3].alpha.unwrap().to_degrees() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn unit_suffix_is_only_stripped_when_it_is_a_unit() {
        assert_eq!(split_header("CP location (cm)"), ("cplocation".into(), Some("cm".into())));
        assert_eq!(split_header("CP (0 to 4 deg)"), ("cp0to4deg".into(), None));
        assert_eq!(split_header("# Mach number ()"), ("machnumber".into(), Some(String::new())));
        assert_eq!(split_header("Angle of attack (°)"), ("angleofattack".into(), Some("°".into())));
    }

    #[test]
    fn openrocket_drag_header_keeps_its_cd_and_accepts_a_trailing_unit() {
        let with_unit = split_header("Drag coefficient (CD) ()");
        assert_eq!(with_unit, ("dragcoefficientcd".into(), Some(String::new())));
        assert_eq!(split_header("Drag coefficient (CD)"), ("dragcoefficientcd".into(), None));
        for header in ["Mach number,Drag coefficient (CD) ()", "Mach number,Drag coefficient (CD)"] {
            let f = parse(&format!("{header}\n0.5,0.55\n0.6,0.58\n"), Source::OpenRocket).unwrap();
            assert_eq!(f.rows[1].cd, Some(0.58), "{header}");
        }
    }

    #[test]
    fn overlay_picks_the_alpha_group_and_mach_group() {
        let f = parse(RASAERO, Source::RasAero).unwrap();
        let o = f.overlay(0.0, 0.5);
        assert!(o.mach.windows(2).all(|w| w[0].x <= w[1].x));
        assert!(o.mach.len() >= 3);
        // The angle sweep is the Mach 0.5 rows only.
        assert!(o.alpha.len() >= 3);
        let o2 = f.overlay(2f64.to_radians(), 0.5);
        assert_eq!(o.mach.len(), o2.mach.len());
        assert_ne!(o.mach[2].cn, o2.mach[2].cn);
    }

    #[test]
    fn rejects_files_without_columns() {
        assert!(parse("", Source::RasAero).is_err());
        assert!(parse("a,b\n1,2\n", Source::RasAero).is_err());
        assert!(parse("Mach,Foo\n0.1,2\n", Source::RasAero).is_err());
    }
}
