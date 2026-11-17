//! What the screens say about what the CFD table is made of, in one place.
//!
//! One solve gives the drag table (zero incidence) and, unless it is left out, the normal force and centre of pressure at two angles
//! of attack. What each part comes from is the engine's business and changes when the engine does: **this file is the only place
//! that names it**. Today the zero-incidence drag is the axisymmetric solver's and the angles are a 3D Euler solve; the fins' drag in
//! the table is the empirical model's. When the engine swap makes one 3D viscous solver do all of it, the strings that say otherwise
//! are the ones to change here (each constant below says what it claims); nothing else in the app describes the solver.

use bowshock_model::{CfdLiftTable, CfdTable};

/// The CFD method's tooltip: what the table is made of.
pub const METHOD_TIP: &str = "Drag solved by CFD on a Mach table (the zero-incidence solver on the GPU, with the fins' empirical drag); margin, CP and CNα from a 3D Euler solve at two angles of attack, or from Extended when the angles are left out";

/// Claims: the drag parts and where the fins' drag comes from, for the provenance of a table with no angles.
pub const DRAG_FROM: &str = "Drag from the solver";

/// Claim: what the solver cannot take (the reasons a stack has no table): bodies beside the axis.
pub const NO_PODS: &str = "pods and strap-ons are bodies beside the axis; the solver takes one axisymmetric stack";
/// The same for tube fins.
pub const NO_TUBE_FINS: &str = "tube fins (open pod tubes) are bodies beside the axis; the solver takes one axisymmetric stack";

/// Why the angles are unsupported for a Mach range (the stability nodes stop at Mach 3), `Status::Unsupported`'s reason.
pub fn angles_range(mach: (f64, f64)) -> String {
    format!("the 3D solve covers Mach up to 3; the Mach range is {:.2} to {:.2}", mach.0, mach.1)
}

/// A note on an angle that did not converge (it is left out of the table).
pub fn angle_left_out(mach: f64, alpha_deg: f64) -> String {
    format!("3D point Mach {mach:.2}, {alpha_deg:.0}° did not converge and is left out.")
}

/// Why a stability part has too few Mach numbers to be a table.
pub const ANGLES_FEW: &str = "the 3D solver did not converge at enough Mach numbers";
/// Why it has no point at all.
pub const ANGLES_NONE: &str = "the 3D solver returned no usable point";

/// The note of a solve whose 3D angles could not be reduced to a table: why they are left out (the drag table stands).
pub fn stability_left_out(why: &str) -> String {
    format!("3D angles left out: {why}.")
}

/// Claim: what the lugs and rail buttons come from (the solver takes no lugs or buttons).
pub const APPENDAGES_NOTE: &str = "Lugs and rail buttons are not in the solved geometry: their drag comes from the build-up.";

/// The subtitle note of a plot panel the CFD table method takes from the Extended method (the solver gives the drag at zero
/// incidence and, when it was solved, the normal force and CP at two angles: nothing else).
pub const PANEL_FROM_EXTENDED: &str = "Extended (not solved by CFD)";
/// The subtitle note of a stability panel fed by a fresh 3D solve of the angles.
pub const PANEL_FROM_3D: &str = "CFD (3D)";
/// Appended to the margin, CP and `CNα` labels when they come from the 3D solve.
pub const LIFT_LABEL: &str = " · CFD (3D)";

/// The Conditions panel's row for the option to leave the angles out.
pub const ANGLES_LABEL: &str = "3D angles";
/// Its tooltip.
pub const ANGLES_TIP: &str =
    "Also solve the normal force and CP in 3D at 2° and 6° (up to 8 Mach numbers, minutes on a fast GPU); off: the drag table alone, a quick table";
/// The hint under the angle of attack field for the CFD method, by whether the angles are in the solve.
pub fn alpha_hint(angles: bool) -> &'static str {
    if angles {
        "Drag solved at α = 0; margin, CP and CNα from the 3D solve at 2° and 6°"
    } else {
        "Solved at α = 0; Extended adds the change with angle"
    }
}

/// What a solve in progress is called, by what is running now.
pub fn solving_title(angles_running: bool) -> &'static str {
    if angles_running {
        "SOLVING · CFD TABLE · 3D NORMAL FORCE AND CP"
    } else {
        "SOLVING · CFD TABLE · DRAG AT ZERO INCIDENCE"
    }
}

/// What a stopped solve is called.
pub const STOPPED_TITLE: &str = "STOPPED · CFD TABLE";

/// The line under the headline of a solve: where it runs, the grid and what is solved.
pub fn place(device: &str, grid: &str, altitude: &str, angles_running: bool) -> String {
    if angles_running {
        format!("{device} · coarse grid · 3D Euler at angles of attack")
    } else {
        format!("{device} · {grid} · altitude {altitude}")
    }
}

/// The solve's outcome line: `Solved 10 Mach points and 3D stability at 8 Mach numbers in 2 min 14 s on Apple M1 Max`.
pub fn solved_what(points: usize, stability_machs: usize) -> String {
    match stability_machs {
        0 => format!("{points} Mach points"),
        n => format!("{points} Mach points and 3D stability at {n} Mach numbers"),
    }
}

/// The toast's short form of the same.
pub fn solved_brief(points: usize, angles: bool) -> String {
    if angles {
        format!("{points} points and 3D stability")
    } else {
        format!("{points} points")
    }
}

/// What a table's points are, for the provenance line: `10 Mach points · zero incidence` (and the angles it has).
pub fn table_points(nodes: usize, angles: Option<&CfdLiftTable>) -> String {
    match angles {
        Some(l) => format!("{nodes} Mach points at zero incidence · {} Mach numbers at {} in 3D", distinct_machs(l), angle_list(l)),
        None => format!("{nodes} Mach points at zero incidence"),
    }
}

/// `2° and 6°`: the angles a stability solve has points at.
fn angle_list(l: &CfdLiftTable) -> String {
    let mut a: Vec<f64> = Vec::new();
    for p in &l.points {
        if a.iter().all(|x| (x - p.alpha_deg).abs() > 1e-9) {
            a.push(p.alpha_deg);
        }
    }
    a.sort_by(f64::total_cmp);
    a.iter().map(|x| format!("{x:.0}°")).collect::<Vec<_>>().join(" and ")
}

/// The provenance of the stability part of a fresh table (what the angles are and where they were solved is the line above it).
pub fn stability_from_3d() -> String {
    format!("{DRAG_FROM}; margin, CP and CNα from the 3D Euler solve.")
}

/// The provenance when the margin, CP and CNα are Extended's.
pub fn stability_from_extended() -> String {
    format!("{DRAG_FROM}; margin, CP and CNα from Extended.")
}

/// Why a table is stale, in one line, by what differs: the drag part and the angle part of the solve are one table, one verdict.
pub fn stale_reason(drag_stale: bool, angles_missing: bool, angles_stale: bool) -> String {
    match (drag_stale, angles_missing, angles_stale) {
        (false, true, _) => "Solved without the 3D angles; Re-solve to add them.".to_string(),
        (false, false, true) => "The 3D angles were solved for other geometry or Mach numbers; Re-solve for a table that matches.".to_string(),
        _ => "The geometry or the conditions changed since; Re-solve for a table that matches.".to_string(),
    }
}

/// How far a cancelled solve got: `4 of 10 Mach points`, or with the angles `10 of 10 Mach points and 4 of 16 3D runs`.
pub fn progress_text(drag: (usize, usize), angles: (usize, usize)) -> String {
    match angles {
        (_, 0) => format!("{} of {} Mach points", drag.0, drag.1),
        (done, of) => format!("{} of {} Mach points and {done} of {of} 3D runs", drag.0, drag.1),
    }
}

/// The short form of [`progress_text`], for the toast: `4 of 10 points`, `10 points, 8 of 16 3D runs`.
pub fn progress_brief(drag: (usize, usize), angles: (usize, usize)) -> String {
    if angles.1 == 0 || drag.0 < drag.1 {
        format!("{} of {} points", drag.0, drag.1)
    } else {
        format!("{} points, {} of {} 3D runs", drag.0, angles.0, angles.1)
    }
}

/// The line under the provenance of a table that stopped early (what is in it is the `Stopped early` note).
pub const PARTIAL_ACTION: &str = "Re-solve for the rest.";

/// The line of a solve stopped early: how much of each part is in.
pub fn stopped_early(solved: usize, of: usize, stability: Option<(usize, usize)>) -> String {
    match stability {
        Some((s, n)) => format!("Stopped early: {solved} of {of} Mach points and {s} of {n} 3D Mach numbers solved."),
        None => format!("Stopped early: {solved} of {of} Mach points solved."),
    }
}

/// A note on the stability part of a stopped solve that had a Mach number with only one of its angles.
pub fn half_angle_note(mach: f64) -> String {
    format!("3D Mach {mach:.2} had only one of its two angles solved and is left out.")
}

/// The label of the Keep button of a stopped solve: `Keep 5 solved points`, with the 3D Mach numbers it also has.
pub fn keep_label(table: &CfdTable) -> String {
    let n = table.points.len();
    let base = format!("Keep {n} solved points");
    match &table.lift {
        Some(l) => format!("Keep {n} points · 3D at {} Mach", distinct_machs(l)),
        None => base,
    }
}

/// The toast of a kept table: `Kept 5 solved points, Mach 0.10 to 1.30`, and with the angles `Kept 5 points, 3D at 3 Mach numbers`.
pub fn kept_text(table: &CfdTable) -> String {
    let n = table.points.len();
    match &table.lift {
        Some(l) => format!("Kept {n} points, 3D at {} Mach numbers", distinct_machs(l)),
        None => {
            let (lo, hi) = (table.points[0].mach, table.points[n - 1].mach);
            format!("Kept {n} solved points, Mach {lo:.2} to {hi:.2}")
        }
    }
}

/// How many Mach numbers a stability table has points at.
pub fn distinct_machs(l: &CfdLiftTable) -> usize {
    let mut v: Vec<f64> = Vec::new();
    for p in &l.points {
        if v.iter().all(|x| (x - p.mach).abs() > 1e-9) {
            v.push(p.mach);
        }
    }
    v.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{CfdLiftPoint, CfdQuality};

    fn lift(machs: &[f64]) -> CfdLiftTable {
        let points = machs
            .iter()
            .flat_map(|&m| [2.0, 6.0].map(|a| CfdLiftPoint { mach: m, alpha_deg: a, cn: 0.1, x_cp: 1.0, cd: None, cm: None, quality: CfdQuality::Converged }))
            .collect();
        CfdLiftTable { key: 1, solved_at: 0, solve_seconds: 1.0, backend: "GPU (Luma)".to_string(), points, notes: Vec::new() }
    }

    #[test]
    fn the_texts_count_what_a_solve_has() {
        assert_eq!(solved_what(10, 0), "10 Mach points");
        assert_eq!(solved_what(10, 8), "10 Mach points and 3D stability at 8 Mach numbers");
        assert_eq!(solved_brief(10, true), "10 points and 3D stability");
        assert_eq!(table_points(10, None), "10 Mach points at zero incidence");
        assert_eq!(table_points(10, Some(&lift(&[0.5, 0.8]))), "10 Mach points at zero incidence · 2 Mach numbers at 2° and 6° in 3D");
        assert_eq!(progress_text((4, 10), (0, 0)), "4 of 10 Mach points");
        assert_eq!(progress_text((10, 10), (8, 16)), "10 of 10 Mach points and 8 of 16 3D runs");
        assert_eq!(progress_brief((4, 10), (0, 16)), "4 of 10 points");
        assert_eq!(progress_brief((10, 10), (8, 16)), "10 points, 8 of 16 3D runs");
        assert_eq!(stopped_early(10, 10, Some((4, 8))), "Stopped early: 10 of 10 Mach points and 4 of 8 3D Mach numbers solved.");
        assert_eq!(distinct_machs(&lift(&[0.5, 0.8, 1.1])), 3);
    }

    #[test]
    fn one_verdict_has_one_reason_whichever_part_is_stale() {
        assert!(stale_reason(true, false, false).starts_with("The geometry or the conditions changed"));
        assert!(stale_reason(false, true, false).starts_with("Solved without the 3D angles"));
        assert!(stale_reason(false, false, true).starts_with("The 3D angles were solved for other"));
    }

    /// What the solver is made of is said in this file and nowhere else: a screen string that names the method (axisymmetric, zero-lift, Euler)
    /// anywhere else in the app is a place the engine swap would leave saying the old thing.
    #[test]
    fn no_other_screen_text_says_what_the_solver_is_made_of() {
        const WORDS: [&str; 8] = ["axisymmetric", "zero-lift", "zero lift", "euler", "3d stability", "3d solve", "3d point", "3d angles"];
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for e in std::fs::read_dir(dir).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    if p.file_name().is_some_and(|n| n != "tests") {
                        walk(&p, out);
                    }
                } else if p.extension().is_some_and(|x| x == "rs") {
                    out.push(p);
                }
            }
        }
        let mut files = Vec::new();
        walk(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src"), &mut files);
        let mut found = Vec::new();
        for f in files.iter().filter(|f| !f.ends_with("wording.rs") && !f.ends_with("solve_demo.rs")) {
            let text = std::fs::read_to_string(f).unwrap();
            // the part of the file before its test module
            let code = text.split("#[cfg(test)]").next().unwrap();
            for (n, line) in code.lines().enumerate() {
                let t = line.trim_start();
                if t.starts_with("//") || !t.contains('"') {
                    continue;
                }
                let lower = line.to_lowercase();
                if let Some(w) = WORDS.iter().find(|w| lower.contains(*w)) {
                    found.push(format!("{}:{} says `{w}`: {t}", f.display(), n + 1));
                }
            }
        }
        assert!(found.is_empty(), "screen text about the solver outside cfd/wording.rs:\n{}", found.join("\n"));
    }
}
