//! The live numbers: what the status strip, the config menu and the motor picker show, computed by `bowshock-sim` from the
//! document and the motor database. One [`Physics`] lives in the app context; [`Physics::sync`] recomputes it when the
//! document's revision or the phase changed, so views only ever read the cache.
//!
//! The numbers are the static ones of [`from_model::balance`]: Barrowman CP at Mach 0.3 and zero angle of attack, mass and CG
//! of the dry parts plus the loaded motors at liftoff (and the burned-out casings, [`Physics::burnout`]).

use std::ffi::OsString;
use std::path::{Path, PathBuf};

use bowshock_model::{ConfigId, MotorMount, MotorRef, PartId, Rocket, Stage};
use bowshock_motors::{Motor, MotorDb};
use bowshock_sim::from_model::{self, Balance, BridgeError, Level, Phase, Warning};

use crate::widgets::gauge::GaugeState;

/// Mach number of the builder's balance readout.
pub const BALANCE_MACH: f64 = 0.3;

/// Where the user's own `.eng` files live: `$BOWSHOCK_MOTORS_DIR`, else `~/.config/bowshock/motors/`.
pub fn user_motors_dir() -> Option<PathBuf> {
    user_dir_from(std::env::var_os("BOWSHOCK_MOTORS_DIR"), std::env::var_os("HOME"))
}

fn user_dir_from(over: Option<OsString>, home: Option<OsString>) -> Option<PathBuf> {
    over.filter(|s| !s.is_empty()).map(PathBuf::from).or_else(|| home.map(|h| PathBuf::from(h).join(".config").join("bowshock").join("motors")))
}

/// Whether `db` already holds a motor with this designation and manufacturer.
fn has_motor(db: &MotorDb, m: &Motor) -> bool {
    db.motors.iter().any(|o| o.designation == m.designation && o.manufacturer == m.manufacturer)
}

/// Parses `.eng` text and appends the motors `db` does not hold yet; returns `(added, skipped duplicates)`. A parse error comes
/// back as a notice with the file and line: `Custom.eng · line 12: bad header (expected 7 fields)`.
pub fn import_eng(db: &mut MotorDb, name: &str, text: &str) -> Result<(usize, usize), String> {
    let parsed = bowshock_motors::parse_eng(text).map_err(|e| format!("{name} · line {}: {}", e.line, e.message))?;
    let total = parsed.len();
    let mut added = 0;
    for motor in parsed {
        if !has_motor(db, &motor) {
            db.motors.push(motor);
            added += 1;
        }
    }
    Ok((added, total - added))
}

/// Loads every `.eng` in `dir` (sorted by name) into `db`; files that fail to parse are skipped with a note on stderr.
fn load_dir(db: &mut MotorDb, dir: &Path) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    let mut files: Vec<PathBuf> =
        entries.filter_map(Result::ok).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("eng"))).collect();
    files.sort();
    for file in files {
        let name = file.file_name().map_or_else(String::new, |n| n.to_string_lossy().into_owned());
        let result = std::fs::read_to_string(&file).map_err(|e| format!("{name}: {e}")).and_then(|text| import_eng(db, &name, &text));
        if let Err(e) = result {
            eprintln!("motor file {e}");
        }
    }
}

/// Imports `paths` into `db`, copying each good file into `dir` so it is loaded at the next start; returns the status notice.
fn import_files(db: &mut MotorDb, dir: Option<&Path>, paths: &[PathBuf]) -> String {
    let mut notes = Vec::new();
    for path in paths {
        let name = path.file_name().map_or_else(|| path.display().to_string(), |n| n.to_string_lossy().into_owned());
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) => {
                notes.push(format!("{name}: cannot read ({e})"));
                continue;
            }
        };
        match import_eng(db, &name, &text) {
            Err(e) => notes.push(e),
            Ok((added, skipped)) => {
                if let Some(dir) = dir {
                    if let Err(e) = std::fs::create_dir_all(dir).and_then(|()| std::fs::write(dir.join(&name), &text)) {
                        notes.push(format!("{name}: loaded, but not kept for next time ({e})"));
                    }
                }
                let plural = if added == 1 { "motor" } else { "motors" };
                notes.push(if skipped == 0 {
                    format!("Loaded {added} {plural} from {name}")
                } else {
                    format!("Loaded {added} {plural} from {name} ({skipped} already known)")
                });
            }
        }
    }
    notes.join(" · ")
}

/// Cached physics of the open document. See the module docs.
pub struct Physics {
    motors: MotorDb,
    /// How many of `motors` are bundled; the ones after are the user's.
    bundled: usize,
    /// What is flying: the whole stack or only the upper stages. A view setting, not part of the document.
    phase: Phase,
    /// The document revision and phase the cache was computed for.
    computed_for: Option<(u64, Phase)>,
    liftoff: Result<Balance, BridgeError>,
    burnout: Result<Balance, BridgeError>,
    warnings: Vec<Warning>,
    /// The health list of the rocket as a carried part would leave it (see [`show_warnings`](Self::show_warnings)), shown in
    /// place of `warnings` while the carry lasts.
    previewed: Option<Vec<Warning>>,
    stages: Vec<Stage>,
    /// Margin of every configuration (whole stack, liftoff), in the document's order.
    config_margins: Vec<(ConfigId, Option<f64>)>,
    gauge: GaugeState,
}

impl Default for Physics {
    fn default() -> Self {
        Physics::new()
    }
}

impl Physics {
    /// An empty cache with the bundled motor database.
    pub fn new() -> Self {
        let (motors, bundled) = Physics::initial_motors();
        Physics {
            motors,
            bundled,
            phase: Phase::Full,
            computed_for: None,
            liftoff: Err(BridgeError::EmptyRocket),
            burnout: Err(BridgeError::EmptyRocket),
            warnings: Vec::new(),
            previewed: None,
            stages: Vec::new(),
            config_margins: Vec::new(),
            gauge: GaugeState::new(0.0),
        }
    }

    /// The bundled motors plus the user's directory (not in tests, which must not depend on the machine), and the bundled count.
    fn initial_motors() -> (MotorDb, usize) {
        let mut db = MotorDb::bundled();
        let bundled = db.len();
        if !cfg!(test) {
            if let Some(dir) = user_motors_dir() {
                load_dir(&mut db, &dir);
            }
        }
        (db, bundled)
    }

    /// Whether motor `index` came from the user's files rather than the bundled set.
    pub fn is_user_motor(&self, index: usize) -> bool {
        index >= self.bundled
    }

    /// Loads the user's `.eng` files (from the file dialog), keeps copies in [`user_motors_dir`], and returns the notice.
    pub fn load_user_motors(&mut self, paths: &[PathBuf]) -> String {
        import_files(&mut self.motors, user_motors_dir().as_deref(), paths)
    }

    /// The motor database.
    pub fn motors(&self) -> &MotorDb {
        &self.motors
    }

    /// The phase on show.
    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// Chooses the phase on show (the next [`sync`](Self::sync) recomputes).
    pub fn set_phase(&mut self, phase: Phase) {
        self.phase = phase;
    }

    /// Recomputes if `revision` or the phase changed since the last time. Returns whether it did.
    pub fn sync(&mut self, rocket: &Rocket, revision: u64) -> bool {
        if self.computed_for == Some((revision, self.phase)) {
            return false;
        }
        // A phase that no longer exists (stages were removed) falls back to the whole stack.
        let stages = rocket.stages();
        if self.phase.first_stage() > stages.len().max(1) {
            self.phase = Phase::Full;
        }
        self.compute(rocket, stages);
        self.computed_for = Some((revision, self.phase));
        true
    }

    fn compute(&mut self, rocket: &Rocket, stages: Vec<Stage>) {
        self.liftoff = from_model::balance(rocket, &self.motors, self.phase, BALANCE_MACH, 0.0, true);
        self.burnout = from_model::balance(rocket, &self.motors, self.phase, BALANCE_MACH, 0.0, false);
        self.warnings = self.warnings_for(rocket, self.liftoff.as_ref().ok());
        self.stages = stages;
        self.config_margins = rocket.configurations().iter().map(|c| (c.id, self.margin_of(rocket, c.id))).collect();
        if let Ok(balance) = &self.liftoff {
            self.gauge.set(balance.margin_cal as f32);
        }
    }

    /// The margin the whole stack would have in configuration `id` at liftoff.
    fn margin_of(&self, rocket: &Rocket, id: ConfigId) -> Option<f64> {
        let mut copy = rocket.clone();
        copy.select_configuration(id);
        from_model::balance(&copy, &self.motors, Phase::Full, BALANCE_MACH, 0.0, true).ok().map(|b| b.margin_cal)
    }

    /// Mass, length, CG, CP and margin of the phase on show at liftoff; `None` for a rocket without parts.
    pub fn balance(&self) -> Option<&Balance> {
        self.liftoff.as_ref().ok()
    }

    /// The liftoff balance of the phase on show for another rocket (a preview of an edit that has not been made); `None` when it
    /// has no balance.
    pub fn balance_of(&self, rocket: &Rocket) -> Option<Balance> {
        from_model::balance(rocket, &self.motors, self.phase, BALANCE_MACH, 0.0, true).ok()
    }

    /// Points the gauge at `margin` (a previewed one), or at the committed margin again when it is `None`.
    pub fn show_margin(&mut self, margin: Option<f64>) {
        if let Some(margin) = margin.or_else(|| self.balance().map(|b| b.margin_cal)) {
            self.gauge.set(margin as f32);
        }
    }

    /// The health list of `rocket` with its liftoff `balance` (`None`: it has none, judged as weightless).
    pub fn warnings_for(&self, rocket: &Rocket, balance: Option<&Balance>) -> Vec<Warning> {
        let none = Balance { mass: 0.0, length: 0.0, reference_diameter: 0.0, cg: 0.0, cp: 0.0, margin_cal: 0.0, cn_alpha: 0.0, motor_mass: 0.0 };
        from_model::warnings(rocket, &self.motors, balance.unwrap_or(&none))
    }

    /// Shows `warnings` (the health of a previewed rocket) in place of the document's, or the document's again when `None`.
    pub fn show_warnings(&mut self, warnings: Option<Vec<Warning>>) {
        self.previewed = warnings;
    }

    /// The same with the motors burned out (casings only).
    #[allow(dead_code)] // For Analyze and the lanes (CG at burnout).
    pub fn burnout(&self) -> Option<&Balance> {
        self.burnout.as_ref().ok()
    }

    /// Why there is no balance (an empty rocket, a motor that is not in the database).
    #[allow(dead_code)]
    pub fn error(&self) -> Option<&BridgeError> {
        self.liftoff.as_ref().err()
    }

    /// The health list of the active configuration, worst first within each topic, including `Ok` readings.
    #[allow(dead_code)]
    pub fn warnings(&self) -> &[Warning] {
        self.previewed.as_deref().unwrap_or(&self.warnings)
    }

    /// The warnings that are not mere `Ok` readings: what the status strip's chip counts.
    pub fn issues(&self) -> impl Iterator<Item = &Warning> {
        self.warnings().iter().filter(|w| w.level != Level::Ok)
    }

    /// The worst level among [`issues`](Self::issues), `Ok` when there are none.
    pub fn worst(&self) -> Level {
        self.issues().map(|w| w.level).max().unwrap_or(Level::Ok)
    }

    /// The stages of the document, firing order (stage 1 first).
    pub fn stages(&self) -> &[Stage] {
        &self.stages
    }

    /// Whether the rocket has more than one stage (the Phase switch appears).
    pub fn is_staged(&self) -> bool {
        self.stages.len() > 1
    }

    /// The margin of configuration `id` (whole stack at liftoff) for the config menu, `None` when it cannot be computed.
    pub fn config_margin(&self, id: ConfigId) -> Option<f64> {
        self.config_margins.iter().find(|(c, _)| *c == id).and_then(|(_, m)| *m)
    }

    /// The margin the whole stack would have at liftoff with `motor` in `mount` of the active configuration (every other
    /// mount as configured). The motor picker's "With this motor" column.
    pub fn margin_with_motor(&self, rocket: &Rocket, mount: PartId, motor: &Motor) -> Option<f64> {
        let mut copy = rocket.clone();
        let config = copy.active_config_id();
        copy.set_motor(config, mount, Some(MotorRef::new(motor.designation.clone(), motor.manufacturer.clone()))).ok()?;
        from_model::balance(&copy, &self.motors, Phase::Full, BALANCE_MACH, 0.0, true).ok().map(|b| b.margin_cal)
    }

    /// The motor mounts of `rocket` that belong to model stage `stage` (from 1), fore to aft.
    pub fn mounts_of_stage(&self, rocket: &Rocket, stage: usize) -> Vec<MotorMount> {
        let Some(stage) = self.stages.iter().find(|s| s.index == stage) else {
            return Vec::new();
        };
        rocket.motor_mounts().into_iter().filter(|m| stage.mounts.contains(&m.id)).collect()
    }

    /// The margin gauge the status strip draws; animated, see [`GaugeState`].
    pub fn gauge(&self) -> &GaugeState {
        &self.gauge
    }

    /// Advances the gauge animation.
    pub fn tick(&mut self, now: std::time::Instant) {
        self.gauge.tick(now);
    }

    /// Whether the gauge needs frame ticks.
    pub fn animating(&self) -> bool {
        self.gauge.moving()
    }

    /// Jumps the gauge to the current margin without animating (after loading a document).
    pub fn snap_gauge(&mut self) {
        if let Ok(balance) = &self.liftoff {
            self.gauge.snap(balance.margin_cal as f32);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind, RadialKind};

    fn finned() -> Rocket {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.9, diameter: 0.0762 }).unwrap();
        r.attach_radial(tube, 0.78, 0.0, 3, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.003 }).unwrap();
        r
    }

    #[test]
    fn sync_recomputes_only_when_the_revision_or_phase_changes() {
        let mut physics = Physics::new();
        let rocket = finned();
        assert!(physics.sync(&Rocket::default(), 0));
        assert!(physics.balance().is_none() && physics.error() == Some(&BridgeError::EmptyRocket));
        assert!(physics.sync(&rocket, 1));
        assert!(!physics.sync(&rocket, 1));
        let balance = *physics.balance().unwrap();
        // A finned rocket: CP aft of CG, positive margin, length and diameter from the geometry.
        assert!(balance.cp > balance.cg && balance.margin_cal > 0.0);
        assert!((balance.length - 1.1286).abs() < 1e-9 && (balance.reference_diameter - 0.0762).abs() < 1e-9);
        // Burnout has no more mass than liftoff.
        assert!(physics.burnout().unwrap().mass <= balance.mass);
        assert!(physics.sync(&rocket, 2));
        // The only configuration's margin is the strip's.
        assert_eq!(physics.config_margin(rocket.active_config_id()), Some(balance.margin_cal));
        assert!(!physics.is_staged());
    }

    #[test]
    fn issues_are_the_warnings_that_are_not_ok_readings() {
        let mut physics = Physics::new();
        let mut rocket = finned();
        // The bare airframe is far over the default 3 cal scale; this test is about Ok readings, so widen the document's band.
        rocket.set_stability_bands(bowshock_model::StabilityBands { over: 20.0, ..Default::default() });
        physics.sync(&rocket, 1);
        // No motor and no recovery: warnings, but no errors; the stability reading is Ok and is not counted.
        assert!(physics.warnings().iter().any(|w| w.level == Level::Ok));
        assert!(physics.issues().count() >= 2 && physics.issues().all(|w| w.level != Level::Ok));
        assert_eq!(physics.worst(), Level::Warn);
    }

    const CUSTOM: &str = "; my own motor\nZ99 54 300 0 0.2 0.35 XX\n   0.1 100\n   1.0 80\n   2.0 0\n";

    fn temp_dir(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("bowshock-motors-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn user_dir_prefers_the_env_override() {
        assert_eq!(user_dir_from(Some("/x/motors".into()), Some("/home/u".into())), Some(PathBuf::from("/x/motors")));
        assert_eq!(user_dir_from(None, Some("/home/u".into())), Some(PathBuf::from("/home/u/.config/bowshock/motors")));
        assert_eq!(user_dir_from(Some("".into()), None), None);
    }

    #[test]
    fn imported_files_are_kept_and_reloaded_from_the_user_dir() {
        let dir = temp_dir("keep");
        let source = dir.join("Custom.eng");
        std::fs::write(&source, CUSTOM).unwrap();
        let store = dir.join("store");
        let mut db = MotorDb::bundled();
        let bundled = db.len();
        let notice = import_files(&mut db, Some(&store), &[source]);
        assert_eq!(notice, "Loaded 1 motor from Custom.eng");
        assert_eq!(db.len(), bundled + 1);
        assert!(store.join("Custom.eng").is_file());
        let mut next = MotorDb::bundled();
        load_dir(&mut next, &store);
        assert_eq!(next.len(), bundled + 1);
        assert_eq!(next.motors.last().unwrap().designation, "Z99");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn duplicates_are_skipped_and_reported() {
        let mut db = MotorDb::bundled();
        let before = db.len();
        assert_eq!(import_eng(&mut db, "a.eng", CUSTOM), Ok((1, 0)));
        assert_eq!(import_eng(&mut db, "a.eng", CUSTOM), Ok((0, 1)));
        let h128w = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../bowshock-motors/data/aerotech_h128w.eng")).unwrap();
        assert_eq!(import_eng(&mut db, "h.eng", &h128w), Ok((0, 1)));
        assert_eq!(db.len(), before + 1);
    }

    #[test]
    fn a_parse_error_names_the_file_and_line() {
        let mut db = MotorDb::bundled();
        let before = db.len();
        let e = import_eng(&mut db, "Broken.eng", "; c\nZ1 54 300\n 0.1 5\n").unwrap_err();
        assert!(e.starts_with("Broken.eng · line 2: "), "{e}");
        assert_eq!(db.len(), before);
    }
}
