//! The CFD results as OpenRocket aerodynamic lookup tables (*Simulation options → Aerodynamic data → Configure…*; OpenRocket user guide, *Advanced flight
//! simulation*, "Aerodynamic lookup tables"): a drag table `Mach,AoA,Cd` and a stability table `Mach,AoA,Cn,Cm,Cp`. Headers are case-insensitive, `#` lines are
//! comments, values are interpolated in Mach and angle and clamped outside the table. The files are read back by [`super::import`], which is the round-trip test.
//!
//! What the tables hold:
//!
//! * **Drag**: the solved Cd of every Mach number at zero angle of attack (the whole table, including the build-up method's lugs and rail buttons) on the vehicle's
//!   reference area, as the CFD table stores it. OpenRocket then flies this Cd at every angle (no AoA column).
//! * **Stability**: the normal force `Cn` and the centre of pressure `Cp` (metres from the nose tip) of the 3D solve at the angle it was solved at, preceded by the
//!   zero-angle row (`Cn` 0, `Cm` 0, the same `Cp`: the centre of pressure of the normal force is the limit at zero angle). `Cm` is the pitching moment about the nose
//!   tip on the reference area and diameter, `-Cn Cp / d` (the solver's definition, nose-up positive). OpenRocket clamps outside the table: beyond the largest
//!   solved angle the table holds its edge values.
//!
//! The comment lines of a file say what the numbers are (solver, grid, altitude) and, for the 3D engine, the uncertainty of the table.

use bowshock_model::{CfdLiftTable, CfdTable};

/// Every file of an export: the drag table and, when the table has a stability solve, the stability table.
pub struct Export {
    /// `Mach,AoA,Cd`.
    pub drag: String,
    /// `Mach,AoA,Cn,Cm,Cp`; `None` when the table has no usable 3D solve (or one stored without its moments).
    pub stability: Option<String>,
}

fn comments(table: &CfdTable, what: &str) -> String {
    let mut out =
        format!("# Bowshock CFD {what} table: {} grid, {}, altitude {:.0} m\n", format!("{:?}", table.grid).to_lowercase(), table.backend, table.altitude);
    if table.bands {
        out.push_str(
            "# CFD solve: the drag reads 3-7 % high against free flight (fully turbulent smooth model), up to 20 % high subsonically with a boattail or a friction-dominated body,\n",
        );
        out.push_str("# and up to 17 % low from Mach 0.95 to 1.2 (fin drag rise); the centre of pressure is within 0.3 caliber and CNalpha reads up to 21 % high subsonically.\n");
        out.push_str("# Lugs and rail buttons are not meshed.\n");
    }
    out
}

/// The OpenRocket tables of `table`.
pub fn export(table: &CfdTable) -> Export {
    let mut drag = comments(table, "drag");
    drag.push_str("Mach,AoA,Cd\n");
    for p in &table.points {
        drag.push_str(&format!("{},0,{}\n", p.mach, p.cd));
    }
    Export { drag, stability: table.lift.as_ref().and_then(|l| stability(table, l)) }
}

fn stability(table: &CfdTable, lift: &CfdLiftTable) -> Option<String> {
    // a point stored without its moment (a table older than the moment) has none to write: the file would not be the solve's, so there is none
    if lift.points.is_empty() || lift.points.iter().any(|p| p.cm.is_none()) {
        return None;
    }
    let mut out = comments(table, "stability");
    out.push_str("# Cm is about the nose tip (-Cn Cp / d); the 0 degree row is Cn 0 with the centre of pressure of the solved angle; Cp is in metres from the nose tip.\n");
    out.push_str("Mach,AoA,Cn,Cm,Cp\n");
    let mut machs: Vec<f64> = lift.points.iter().map(|p| p.mach).collect();
    machs.dedup();
    for m in machs {
        let at: Vec<_> = lift.points.iter().filter(|p| p.mach == m).collect();
        let first = at.iter().min_by(|a, b| a.alpha_deg.total_cmp(&b.alpha_deg))?;
        if first.alpha_deg > 0.0 {
            out.push_str(&format!("{m},0,0,0,{}\n", first.x_cp));
        }
        for p in at {
            out.push_str(&format!("{m},{},{},{},{}\n", p.alpha_deg, p.cn, p.cm?, p.x_cp));
        }
    }
    Some(out)
}

/// The two file names of an export written at `path` (`name.csv` gives `name-drag.csv` and `name-stability.csv`).
pub fn file_names(path: &std::path::Path) -> (std::path::PathBuf, std::path::PathBuf) {
    let stem = path.file_stem().map_or_else(|| "cfd".to_string(), |s| s.to_string_lossy().into_owned());
    let dir = path.parent().map(std::path::Path::to_path_buf).unwrap_or_default();
    (dir.join(format!("{stem}-drag.csv")), dir.join(format!("{stem}-stability.csv")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analyze::import::{parse, Source};
    use bowshock_model::{CfdGrid, CfdLiftPoint, CfdPoint, CfdQuality};

    fn table() -> CfdTable {
        let p = |mach: f64, cd: f64| CfdPoint {
            mach,
            cd,
            friction: cd * 0.2,
            pressure: cd * 0.2,
            base: cd * 0.2,
            fins: cd * 0.3,
            other: cd * 0.1,
            interference: 0.0,
            quality: CfdQuality::Converged,
            restart: None,
        };
        CfdTable {
            key: 1,
            geometry: 1,
            stage: 1,
            solved_at: 1,
            solve_seconds: 1.0,
            bands: false,
            backend: "GPU (Luma)".into(),
            fallback: None,
            altitude: 120.0,
            notes: Vec::new(),
            points: vec![p(0.5, 0.4123456789), p(1.0, 0.8), p(2.0, 0.6)],
            grid: CfdGrid::Coarse,
            lift: None,
            boattail: false,
            power_on: None,
        }
    }

    fn lift_table(key: u64) -> CfdLiftTable {
        let p = |mach: f64, cn: f64, x_cp: f64| CfdLiftPoint {
            mach,
            alpha_deg: 2.0,
            cn,
            x_cp,
            cd: None,
            cm: Some(-cn * x_cp / 0.1),
            quality: CfdQuality::Converged,
        };
        CfdLiftTable {
            key,
            solved_at: 1,
            solve_seconds: 60.0,
            bands: true,
            backend: "GPU (Luma)".into(),
            points: vec![p(0.8, 0.1234567, 1.0), p(2.0, 0.2, 1.4)],
            notes: Vec::new(),
        }
    }

    #[test]
    fn the_drag_table_round_trips_through_the_importer() {
        let t = table();
        let file = export(&t).drag;
        assert!(file.lines().any(|l| l == "Mach,AoA,Cd"));
        let back = parse(&file, Source::OpenRocket).unwrap();
        assert_eq!(back.rows.len(), t.points.len());
        for (row, p) in back.rows.iter().zip(&t.points) {
            assert_eq!(row.mach, Some(p.mach));
            assert_eq!(row.alpha, Some(0.0));
            assert_eq!(row.cd, Some(p.cd));
        }
    }

    #[test]
    fn the_stability_table_has_the_zero_angle_row_and_round_trips() {
        let t = CfdTable { lift: Some(lift_table(1)), bands: true, ..table() };
        let e = export(&t);
        let file = e.stability.expect("the table has a 3D solve");
        assert!(file.lines().any(|l| l == "Mach,AoA,Cn,Cm,Cp"));
        assert!(file.contains("not meshed"));
        let back = parse(&file, Source::OpenRocket).unwrap();
        let lift = t.lift.as_ref().unwrap();
        // every solved point comes back, preceded by one zero-angle row per Mach number
        let solved: Vec<_> = back.rows.iter().filter(|r| r.alpha.is_some_and(|a| a > 0.0)).collect();
        assert_eq!(solved.len(), lift.points.len());
        for (row, p) in solved.iter().zip(&lift.points) {
            assert_eq!(row.mach, Some(p.mach));
            assert!((row.alpha.unwrap().to_degrees() - p.alpha_deg).abs() < 1e-9);
            assert_eq!(row.cn, Some(p.cn));
            assert_eq!(row.cp, Some(p.x_cp));
            assert_eq!(row.cm, p.cm);
        }
        let zero: Vec<_> = back.rows.iter().filter(|r| r.alpha == Some(0.0)).collect();
        assert!(!zero.is_empty() && zero.iter().all(|r| r.cn == Some(0.0) && r.cm == Some(0.0) && r.cp.is_some()));
    }

    #[test]
    fn a_table_without_a_3d_solve_has_no_stability_file() {
        assert!(export(&table()).stability.is_none());
    }

    #[test]
    fn the_file_names_follow_the_chosen_one() {
        let (d, s) = file_names(std::path::Path::new("/tmp/x/rocket.csv"));
        assert_eq!(d, std::path::Path::new("/tmp/x/rocket-drag.csv"));
        assert_eq!(s, std::path::Path::new("/tmp/x/rocket-stability.csv"));
    }
}
