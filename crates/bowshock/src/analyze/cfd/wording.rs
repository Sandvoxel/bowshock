//! What the screens say about what the CFD table is made of, in one place.
//!
//! One solve gives the drag table (zero incidence) and, unless it is left out, the normal force and centre of pressure at an angle
//! of attack. What each part comes from is the engine's business and changes when the engine does: **this file is the only place
//! that names it**. Today the zero-incidence drag is the axisymmetric solver's and the angles are a 3D Euler solve; the fins' drag in
//! the table is the empirical model's. When the engine swap makes one 3D viscous solver do all of it, the strings that say otherwise
//! are the ones to change here (each constant below says what it claims); nothing else in the app describes the solver.

use bowshock_core as solver;
use bowshock_model::{CfdLiftTable, CfdTable};

/// Whether the 3D engine (`BOWSHOCK_ENGINE=3d`, [`solver::Engine::Solver3`]) is the one the screens describe. The strings that name the solver are [`Words`]; the
/// free functions of this file speak for the engine in use, so the app needs no other place that knows there are two.
fn solver3() -> bool {
    solver::Engine::from_env() == solver::Engine::Solver3
}

/// The strings that name what the CFD table is made of, for one engine: `Words(false)` is the legacy pair (the axisymmetric drag solver and the sting-grid 3D Euler
/// solve at angles of attack, the fins' drag from the empirical model), `Words(true)` the 3D engine (one solver on the base-wake grid for both: the fins and the
/// base are solved). At the engine swap `Words(false)` and the branches that serve it are deleted.
#[derive(Clone, Copy)]
pub struct Words(pub bool);

impl Words {
    /// The CFD method's tooltip: what the table is made of.
    pub fn method_tip(self) -> &'static str {
        if self.0 {
            "Drag, margin, CP and CNα solved by CFD on a Mach table (the 3D solver on the GPU: the fins and the base are solved); margin and CP from Extended when the angles are left out"
        } else {
            "Drag solved by CFD on a Mach table (the zero-incidence solver on the GPU, with the fins' empirical drag); margin, CP and CNα from a 3D Euler solve at an angle of attack, or from Extended when the angles are left out"
        }
    }

    /// Claim: what the solver cannot take (the reasons a stack has no table): bodies beside the axis.
    pub fn no_pods(self) -> &'static str {
        if self.0 {
            "pods and strap-ons are bodies beside the axis; the solver takes one body of revolution with its fins"
        } else {
            "pods and strap-ons are bodies beside the axis; the solver takes one axisymmetric stack"
        }
    }

    /// The same for tube fins.
    pub fn no_tube_fins(self) -> &'static str {
        if self.0 {
            "tube fins (open pod tubes) are bodies beside the axis; the solver takes one body of revolution with its fins"
        } else {
            "tube fins (open pod tubes) are bodies beside the axis; the solver takes one axisymmetric stack"
        }
    }

    /// Why the angles are unsupported for a Mach range (the stability nodes stop at Mach 3), `Status::Unsupported`'s reason.
    pub fn angles_range(self, mach: (f64, f64)) -> String {
        let what = if self.0 { "the solve at angles of attack" } else { "the 3D solve" };
        format!("{what} covers Mach up to 3; the Mach range is {:.2} to {:.2}", mach.0, mach.1)
    }

    /// A note on an angle that did not converge (it is left out of the table).
    pub fn angle_left_out(self, mach: f64, alpha_deg: f64) -> String {
        if self.0 {
            format!("Mach {mach:.2} at {alpha_deg:.0}° did not converge and is left out.")
        } else {
            format!("3D point Mach {mach:.2}, {alpha_deg:.0}° did not converge and is left out.")
        }
    }

    /// Why a stability part has too few Mach numbers to be a table.
    pub fn angles_few(self) -> &'static str {
        if self.0 {
            "the solver did not converge at enough Mach numbers at the angles"
        } else {
            "the 3D solver did not converge at enough Mach numbers"
        }
    }

    /// Why it has no point at all.
    pub fn angles_none(self) -> &'static str {
        if self.0 {
            "the solver returned no usable point at the angles"
        } else {
            "the 3D solver returned no usable point"
        }
    }

    /// The note of a solve whose angles could not be reduced to a table: why they are left out (the drag table stands).
    pub fn stability_left_out(self, why: &str) -> String {
        format!("{} left out: {why}.", if self.0 { "Angles" } else { "3D angles" })
    }

    /// The tooltip of the angles option.
    pub fn angles_tip(self) -> &'static str {
        if self.0 {
            "Also solve the normal force and CP at 2° (up to 8 Mach numbers more); off: the drag table alone, a quick table"
        } else {
            "Also solve the normal force and CP in 3D at 2° (up to 8 Mach numbers, minutes on a fast GPU); off: the drag table alone, a quick table"
        }
    }

    /// The hint under the angle of attack field for the CFD method, by whether the angles are in the solve.
    pub fn alpha_hint(self, angles: bool) -> &'static str {
        match (angles, self.0) {
            (true, true) => "Drag solved at α = 0; margin, CP and CNα from the solve at 2°",
            (true, false) => "Drag solved at α = 0; margin, CP and CNα from the 3D solve at 2°",
            (false, _) => "Solved at α = 0; Extended adds the change with angle",
        }
    }

    /// What a solve in progress is called, by what is running now.
    pub fn solving_title(self, angles_running: bool) -> &'static str {
        match (angles_running, self.0) {
            (true, true) => "SOLVING · CFD TABLE · NORMAL FORCE AND CP AT ANGLE OF ATTACK",
            (true, false) => "SOLVING · CFD TABLE · 3D NORMAL FORCE AND CP",
            (false, _) => "SOLVING · CFD TABLE · DRAG AT ZERO INCIDENCE",
        }
    }

    /// The line under the headline of a solve: where it runs, the grid and what is solved. The altitude sets the Reynolds number of an engine that solves the
    /// viscous terms; the legacy angles are Euler, at no altitude.
    pub fn place(self, device: &str, grid: &str, altitude: &str, angles_running: bool) -> String {
        match (angles_running, self.0) {
            (true, false) => format!("{device} · coarse grid · 3D Euler at angles of attack"),
            _ => format!("{device} · {grid} · altitude {altitude}"),
        }
    }

    /// The solve's outcome line: `Solved 10 Mach points and 3D stability at 8 Mach numbers in 2 min 14 s on Apple M1 Max`.
    pub fn solved_what(self, points: usize, stability_machs: usize) -> String {
        match (stability_machs, self.0) {
            (0, _) => format!("{points} Mach points"),
            (n, true) => format!("{points} Mach points and the angles at {n} Mach numbers"),
            (n, false) => format!("{points} Mach points and 3D stability at {n} Mach numbers"),
        }
    }

    /// The toast's short form of the same.
    pub fn solved_brief(self, points: usize, angles: bool) -> String {
        match (angles, self.0) {
            (false, _) => format!("{points} points"),
            (true, true) => format!("{points} points and the angles"),
            (true, false) => format!("{points} points and 3D stability"),
        }
    }

    /// What a table's points are, for the provenance line: `10 Mach points at zero incidence` (and the angles it has).
    pub fn table_points(self, nodes: usize, angles: Option<&CfdLiftTable>) -> String {
        match (angles, self.0) {
            (Some(l), true) => format!("{nodes} Mach points at zero incidence · {} Mach numbers at {}", distinct_machs(l), angle_list(l)),
            (Some(l), false) => format!("{nodes} Mach points at zero incidence · {} Mach numbers at {} in 3D", distinct_machs(l), angle_list(l)),
            (None, _) => format!("{nodes} Mach points at zero incidence"),
        }
    }

    /// The provenance of the stability part of a fresh table (what the angles are and where they were solved is the line above it).
    pub fn stability_from_3d(self) -> String {
        if self.0 {
            "Drag, margin, CP and CNα from the 3D solver (the fins and the base are solved).".to_string()
        } else {
            format!("{DRAG_FROM}; margin, CP and CNα from the 3D Euler solve.")
        }
    }

    /// The provenance when the margin, CP and CNα are Extended's.
    pub fn stability_from_extended(self) -> String {
        if self.0 {
            "Drag from the 3D solver (the fins and the base are solved); margin, CP and CNα from Extended.".to_string()
        } else {
            format!("{DRAG_FROM}; margin, CP and CNα from Extended.")
        }
    }

    /// Why a table is stale, in one line, by what differs: the drag part and the angle part of the solve are one table, one verdict.
    pub fn stale_reason(self, drag_stale: bool, angles_missing: bool, angles_stale: bool) -> String {
        let angles = if self.0 { "angles" } else { "3D angles" };
        match (drag_stale, angles_missing, angles_stale) {
            (false, true, _) => format!("Solved without the {angles}; Re-solve to add them."),
            (false, false, true) => format!("The {angles} were solved for other geometry or Mach numbers; Re-solve for a table that matches."),
            _ => "The geometry or the conditions changed since; Re-solve for a table that matches.".to_string(),
        }
    }

    /// How far a cancelled solve got: `4 of 10 Mach points`, or with the angles `10 of 10 Mach points and 4 of 16 3D runs`.
    pub fn progress_text(self, drag: (usize, usize), angles: (usize, usize)) -> String {
        match angles {
            (_, 0) => format!("{} of {} Mach points", drag.0, drag.1),
            (done, of) => format!("{} of {} Mach points and {done} of {of} {} runs", drag.0, drag.1, if self.0 { "angle" } else { "3D" }),
        }
    }

    /// The short form of [`Words::progress_text`], for the toast: `4 of 10 points`, `10 points, 8 of 16 3D runs`.
    pub fn progress_brief(self, drag: (usize, usize), angles: (usize, usize)) -> String {
        if angles.1 == 0 || drag.0 < drag.1 {
            format!("{} of {} points", drag.0, drag.1)
        } else {
            format!("{} points, {} of {} {} runs", drag.0, angles.0, angles.1, if self.0 { "angle" } else { "3D" })
        }
    }

    /// The line of a solve stopped early: how much of each part is in.
    pub fn stopped_early(self, solved: usize, of: usize, stability: Option<(usize, usize)>) -> String {
        match stability {
            Some((s, n)) => format!("Stopped early: {solved} of {of} Mach points and {s} of {n} {}Mach numbers solved.", if self.0 { "angle " } else { "3D " }),
            None => format!("Stopped early: {solved} of {of} Mach points solved."),
        }
    }

    /// A note on the stability part of a stopped solve that had a Mach number with only one of its angles.
    pub fn half_angle_note(self, mach: f64) -> String {
        format!("{}Mach {mach:.2} had only one of its two angles solved and is left out.", if self.0 { "" } else { "3D " })
    }

    /// The label of the Keep button of a stopped solve: `Keep 5 solved points`, with the Mach numbers of the angles it also has.
    pub fn keep_label(self, table: &CfdTable) -> String {
        let n = table.points.len();
        match &table.lift {
            Some(l) => format!("Keep {n} points · {} at {} Mach", if self.0 { "angles" } else { "3D" }, distinct_machs(l)),
            None => format!("Keep {n} solved points"),
        }
    }

    /// The toast of a kept table: `Kept 5 solved points, Mach 0.10 to 1.30`, and with the angles `Kept 5 points, 3D at 3 Mach numbers`.
    pub fn kept_text(self, table: &CfdTable) -> String {
        let n = table.points.len();
        match &table.lift {
            Some(l) => format!("Kept {n} points, {} at {} Mach numbers", if self.0 { "angles" } else { "3D" }, distinct_machs(l)),
            None => {
                let (lo, hi) = (table.points[0].mach, table.points[n - 1].mach);
                format!("Kept {n} solved points, Mach {lo:.2} to {hi:.2}")
            }
        }
    }
}

/// The CFD method's tooltip ([`Words::method_tip`]) for the engine in use; the free functions below are all `Words(engine in use)`.
pub fn method_tip() -> &'static str {
    Words(solver3()).method_tip()
}
/// See [`Words::no_pods`].
pub fn no_pods() -> &'static str {
    Words(solver3()).no_pods()
}
/// See [`Words::no_tube_fins`].
pub fn no_tube_fins() -> &'static str {
    Words(solver3()).no_tube_fins()
}
/// See [`Words::angles_range`].
pub fn angles_range(mach: (f64, f64)) -> String {
    Words(solver3()).angles_range(mach)
}
/// See [`Words::angle_left_out`].
pub fn angle_left_out(mach: f64, alpha_deg: f64) -> String {
    Words(solver3()).angle_left_out(mach, alpha_deg)
}
/// See [`Words::angles_few`].
pub fn angles_few() -> &'static str {
    Words(solver3()).angles_few()
}
/// See [`Words::angles_none`].
pub fn angles_none() -> &'static str {
    Words(solver3()).angles_none()
}
/// See [`Words::stability_left_out`].
pub fn stability_left_out(why: &str) -> String {
    Words(solver3()).stability_left_out(why)
}
/// See [`Words::angles_tip`].
pub fn angles_tip() -> &'static str {
    Words(solver3()).angles_tip()
}
/// See [`Words::alpha_hint`].
pub fn alpha_hint(angles: bool) -> &'static str {
    Words(solver3()).alpha_hint(angles)
}
/// See [`Words::solving_title`].
pub fn solving_title(angles_running: bool) -> &'static str {
    Words(solver3()).solving_title(angles_running)
}
/// See [`Words::place`].
pub fn place(device: &str, grid: &str, altitude: &str, angles_running: bool) -> String {
    Words(solver3()).place(device, grid, altitude, angles_running)
}
/// See [`Words::solved_what`].
pub fn solved_what(points: usize, stability_machs: usize) -> String {
    Words(solver3()).solved_what(points, stability_machs)
}
/// See [`Words::solved_brief`].
pub fn solved_brief(points: usize, angles: bool) -> String {
    Words(solver3()).solved_brief(points, angles)
}
/// See [`Words::table_points`].
pub fn table_points(nodes: usize, angles: Option<&CfdLiftTable>) -> String {
    Words(solver3()).table_points(nodes, angles)
}
/// See [`Words::stability_from_3d`].
pub fn stability_from_3d() -> String {
    Words(solver3()).stability_from_3d()
}
/// See [`Words::stability_from_extended`].
pub fn stability_from_extended() -> String {
    Words(solver3()).stability_from_extended()
}
/// See [`Words::stale_reason`].
pub fn stale_reason(drag_stale: bool, angles_missing: bool, angles_stale: bool) -> String {
    Words(solver3()).stale_reason(drag_stale, angles_missing, angles_stale)
}
/// See [`Words::progress_text`].
pub fn progress_text(drag: (usize, usize), angles: (usize, usize)) -> String {
    Words(solver3()).progress_text(drag, angles)
}
/// See [`Words::progress_brief`].
pub fn progress_brief(drag: (usize, usize), angles: (usize, usize)) -> String {
    Words(solver3()).progress_brief(drag, angles)
}
/// See [`Words::stopped_early`].
pub fn stopped_early(solved: usize, of: usize, stability: Option<(usize, usize)>) -> String {
    Words(solver3()).stopped_early(solved, of, stability)
}
/// See [`Words::half_angle_note`].
pub fn half_angle_note(mach: f64) -> String {
    Words(solver3()).half_angle_note(mach)
}
/// See [`Words::keep_label`].
pub fn keep_label(table: &CfdTable) -> String {
    Words(solver3()).keep_label(table)
}
/// See [`Words::kept_text`].
pub fn kept_text(table: &CfdTable) -> String {
    Words(solver3()).kept_text(table)
}

/// Claims: the drag parts and where the fins' drag comes from, for the provenance of a table with no angles.
pub const DRAG_FROM: &str = "Drag from the solver";

/// The tooltip of the Quick button (the 3D solver's draft grid).
pub const DRAFT_TOOLTIP: &str =
    "Re-solve on the draft grid: wall functions and about a quarter of the cells, much quicker, Cd within about 1 % at Mach 2 and less certain below Mach 0.8";

/// The toast of a draft-first solve that stored its table: `brief` is the solve's own (`10 points and the angles in 1 min 2 s`).
pub fn draft_ready(brief: &str) -> String {
    format!("Draft table: {brief} · refining on the coarse grid")
}

/// The toast of a draft solve that failed: the coarse solve goes ahead.
pub fn draft_failed(why: &str) -> String {
    format!("Draft solve stopped: {why}; solving on the coarse grid")
}

/// The toast of a refinement that was cancelled: the draft table stays.
pub fn refine_cancelled() -> &'static str {
    "Refinement cancelled; the draft table stays"
}

/// The line under a draft table while the coarse solve that replaces it runs (the stability is still Extended's: the draft solve takes the drag alone).
pub const REFINING_NOTE: &str =
    "A draft table, a first look (wall functions, about a quarter of the cells); the coarse solve now running replaces it and adds the 3D angles.";

/// The note for a point that converged only after the solver took the run back to an earlier state and restarted it at a lower CFL limit (the 3D solver's controller).
pub fn restart_note(mach: f64, r: &bowshock_model::CfdRestart) -> String {
    let times = if r.rollbacks == 1 { "after a restart".to_string() } else { format!("after {} restarts", r.rollbacks) };
    format!("Mach {mach:.2} converged {times} at a lower CFL limit ({:.1} to {:.1}): the run first went unstable, so the point is from different numerics than the others.", r.cfl_from, r.cfl_to)
}

/// The short form for the point list of the solve screen.
pub fn restart_tag(r: &bowshock_model::CfdRestart) -> String {
    format!("restarted at CFL {:.1}", r.cfl_to)
}

/// What a stopped solve is called.
pub const STOPPED_TITLE: &str = "STOPPED · CFD TABLE";

/// The line under the provenance of a table that stopped early (what is in it is the `Stopped early` note).
pub const PARTIAL_ACTION: &str = "Re-solve for the rest.";

/// Claim: what the lugs and rail buttons come from (the solver takes no lugs or buttons).
pub const APPENDAGES_NOTE: &str = "Lugs and rail buttons are not in the solved geometry: their drag comes from the build-up.";

/// The subtitle note of a plot panel the CFD table method takes from the Extended method (the solver gives the drag at zero
/// incidence and, when it was solved, the normal force and CP at the angle: nothing else).
pub const PANEL_FROM_EXTENDED: &str = "Extended (not solved by CFD)";
/// The subtitle note of a stability panel fed by a fresh solve of the angles.
pub const PANEL_FROM_3D: &str = "CFD (3D)";
/// Appended to the margin, CP and `CNα` labels when they come from the solve of the angles.
pub const LIFT_LABEL: &str = " · CFD (3D)";

/// The Conditions panel's row for the option to leave the angles out.
pub const ANGLES_LABEL: &str = "3D angles";

/// `2°`: the angles a stability solve has points at.
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
        let points =
            machs.iter().map(|&m| CfdLiftPoint { mach: m, alpha_deg: 2.0, cn: 0.1, x_cp: 1.0, cd: None, cm: None, quality: CfdQuality::Converged }).collect();
        CfdLiftTable { key: 1, solved_at: 0, solve_seconds: 1.0, backend: "GPU (Luma)".to_string(), points, notes: Vec::new() }
    }

    #[test]
    fn the_texts_count_what_a_solve_has() {
        let (old, new) = (Words(false), Words(true));
        assert_eq!(old.solved_what(10, 0), "10 Mach points");
        assert_eq!(old.solved_what(10, 8), "10 Mach points and 3D stability at 8 Mach numbers");
        assert_eq!(new.solved_what(10, 8), "10 Mach points and the angles at 8 Mach numbers");
        assert_eq!(old.solved_brief(10, true), "10 points and 3D stability");
        assert_eq!(new.solved_brief(10, true), "10 points and the angles");
        assert_eq!(old.table_points(10, None), "10 Mach points at zero incidence");
        assert_eq!(old.table_points(10, Some(&lift(&[0.5, 0.8]))), "10 Mach points at zero incidence · 2 Mach numbers at 2° in 3D");
        assert_eq!(new.table_points(10, Some(&lift(&[0.5, 0.8]))), "10 Mach points at zero incidence · 2 Mach numbers at 2°");
        assert_eq!(old.progress_text((4, 10), (0, 0)), "4 of 10 Mach points");
        assert_eq!(old.progress_text((10, 10), (8, 16)), "10 of 10 Mach points and 8 of 16 3D runs");
        assert_eq!(new.progress_text((10, 10), (8, 16)), "10 of 10 Mach points and 8 of 16 angle runs");
        assert_eq!(old.progress_brief((4, 10), (0, 16)), "4 of 10 points");
        assert_eq!(old.progress_brief((10, 10), (8, 16)), "10 points, 8 of 16 3D runs");
        assert_eq!(old.stopped_early(10, 10, Some((4, 8))), "Stopped early: 10 of 10 Mach points and 4 of 8 3D Mach numbers solved.");
        assert_eq!(new.stopped_early(10, 10, Some((4, 8))), "Stopped early: 10 of 10 Mach points and 4 of 8 angle Mach numbers solved.");
        assert_eq!(distinct_machs(&lift(&[0.5, 0.8, 1.1])), 3);
    }

    #[test]
    fn one_verdict_has_one_reason_whichever_part_is_stale() {
        for w in [Words(false), Words(true)] {
            assert!(w.stale_reason(true, false, false).starts_with("The geometry or the conditions changed"));
            assert!(w.stale_reason(false, true, false).starts_with("Solved without the "));
            assert!(w.stale_reason(false, false, true).contains("were solved for other"));
        }
        assert!(Words(false).stale_reason(false, true, false).contains("3D angles"));
        assert!(!Words(true).stale_reason(false, true, false).contains("3D"));
    }

    #[test]
    fn the_3d_engine_names_one_solver_for_everything() {
        let w = Words(true);
        for s in [w.method_tip(), w.angles_tip(), w.alpha_hint(true), w.solving_title(true), &w.stability_from_3d(), &w.stability_from_extended(), w.no_pods()]
        {
            let lower = s.to_lowercase();
            assert!(!lower.contains("axisymmetric") && !lower.contains("euler") && !lower.contains("empirical"), "{s}");
        }
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
