//! The live numbers: what the status strip, the config menu and the motor picker show, computed by `bowshock-sim` from the
//! document and the motor database. One [`Physics`] lives in the app context; [`Physics::sync`] recomputes it when the
//! document's revision or the phase changed, so views only ever read the cache.
//!
//! The numbers are the static ones of [`from_model::balance`]: Barrowman CP at Mach 0.3 and zero angle of attack, mass and CG
//! of the dry parts plus the loaded motors at liftoff (and the burned-out casings, [`Physics::burnout`]).

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use bowshock_model::{ConfigId, MotorMount, PartId, Rocket, Stage};
use bowshock_motors::{Motor, MotorDb, FILE_ID_PREFIX};
use bowshock_sim::from_model::{self, Balance, BridgeError, Level, Phase, Warning};

use crate::widgets::gauge::GaugeState;

/// Mach number of the builder's balance readout.
pub const BALANCE_MACH: f64 = 0.3;

/// Where the user's own motors live: the `.eng` and `.rse` files they loaded, the thrust curves stored in imported OpenRocket files (kept as `.eng`) and the ThrustCurve.org downloads (`<motorId>.motor`).
/// `$BOWSHOCK_MOTORS_DIR`, else `~/.config/bowshock/motors/`.
pub fn user_motors_dir() -> Option<PathBuf> {
    super::env::default_motors_dir()
}

/// Where a motor in the database came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorOrigin {
    /// The catalogue bundled with the app.
    Catalogue,
    /// Downloaded from ThrustCurve.org (and cached in [`user_motors_dir`]).
    Downloaded,
    /// One of the user's own `.eng` files.
    File,
}

/// Whether `db` already holds this motor: the same id, or the same curve data under another id (a user's `.eng` copy of a
/// catalogue motor).
fn has_motor(db: &MotorDb, m: &Motor) -> bool {
    db.motors.iter().any(|o| o.id == m.id || o.same_data(m))
}

/// Parses `.eng` text and appends the motors `db` does not hold yet; returns `(added, skipped duplicates)`. A parse error comes
/// back as a notice with the file and line: `Custom.eng · line 12: bad header (expected 7 fields)`. A second curve with the
/// designation of a known motor is added: it is another motor.
pub fn import_eng(db: &mut MotorDb, name: &str, text: &str) -> Result<(usize, usize), String> {
    let parsed = bowshock_motors::parse_eng(text).map_err(|e| format!("{name} · line {}: {}", e.line, e.message))?;
    Ok(add_new(db, parsed))
}

/// Whether `name` is a RockSim engine file (`.rse`); anything else is read as RASP.
fn is_rse(name: &str) -> bool {
    Path::new(name).extension().is_some_and(|x| x.eq_ignore_ascii_case("rse"))
}

/// Parses `.rse` text and appends the motors `db` does not hold yet, like [`import_eng`]; the error is `Name.rse · <why>`.
pub fn import_rse(db: &mut MotorDb, name: &str, text: &str) -> Result<(usize, usize), String> {
    let parsed = bowshock_motors::parse_rse(text).map_err(|e| format!("{name} · {e}"))?;
    Ok(add_new(db, parsed))
}

/// Appends the motors of `parsed` that `db` does not hold yet; returns `(added, skipped)`.
fn add_new(db: &mut MotorDb, parsed: Vec<Motor>) -> (usize, usize) {
    let total = parsed.len();
    let mut added = 0;
    for motor in parsed {
        if !has_motor(db, &motor) {
            db.upsert(motor);
            added += 1;
        }
    }
    (added, total - added)
}

/// [`import_rse`] for a name ending in `.rse`, else [`import_eng`].
fn import_text(db: &mut MotorDb, name: &str, text: &str) -> Result<(usize, usize), String> {
    if is_rse(name) {
        import_rse(db, name, text)
    } else {
        import_eng(db, name, text)
    }
}

/// The RASP `.eng` text of `motor` (full precision, so reading it back gives the same [`Motor::id`]).
fn eng_text(motor: &Motor) -> String {
    let mut out = String::new();
    for line in &motor.comments {
        out.push_str(&format!("; {line}\n"));
    }
    let delays = if motor.delays.is_empty() { "P".to_string() } else { motor.delays.iter().map(ToString::to_string).collect::<Vec<_>>().join("-") };
    out.push_str(&format!(
        "{} {} {} {} {} {} {}\n",
        motor.designation,
        motor.diameter * 1000.0,
        motor.length * 1000.0,
        delays,
        motor.propellant_mass,
        motor.total_mass,
        motor.manufacturer
    ));
    for (t, f) in &motor.points {
        out.push_str(&format!("{t} {f}\n"));
    }
    out
}

/// Writes `motor` into the user motors directory `dir` as `<designation>-<hash>.eng`, where the next start loads it.
fn keep_as_eng(dir: &Path, motor: &Motor) -> Result<(), String> {
    if motor.designation.split_whitespace().count() != 1 || motor.designation.contains(['/', '\\', ':']) {
        return Err(format!("`{}` cannot be a file name", motor.designation));
    }
    let name = format!("{}-{}.eng", motor.designation, motor.id.trim_start_matches(FILE_ID_PREFIX));
    std::fs::create_dir_all(dir).and_then(|()| std::fs::write(dir.join(name), eng_text(motor))).map_err(|e| e.to_string())
}

/// Loads the user motors directory into `db`: the cached downloads (`*.motor`, which replace the catalogue's copy of the same
/// motor) and every `.eng` and `.rse` (sorted by name). Files that cannot be read are skipped with a note on stderr. Returns the ids of
/// the cached downloads.
fn load_dir(db: &mut MotorDb, dir: &Path) -> HashSet<String> {
    let (cached, failed) = bowshock_motors::load_cached(dir);
    for (path, e) in failed {
        eprintln!("cached motor {}: {e}", path.display());
    }
    let downloaded = cached.iter().map(|m| m.id.clone()).collect();
    for motor in cached {
        db.upsert(motor);
    }
    let Ok(entries) = std::fs::read_dir(dir) else {
        return downloaded;
    };
    let mut files: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("eng") || x.eq_ignore_ascii_case("rse")))
        .collect();
    files.sort();
    for file in files {
        let name = file.file_name().map_or_else(String::new, |n| n.to_string_lossy().into_owned());
        let result = std::fs::read_to_string(&file).map_err(|e| format!("{name}: {e}")).and_then(|text| import_text(db, &name, &text));
        if let Err(e) = result {
            eprintln!("motor file {e}");
        }
    }
    downloaded
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
        match import_text(db, &name, &text) {
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
    /// Where the user's motors are kept (`None`: nowhere, as in tests).
    motors_dir: Option<PathBuf>,
    /// Ids of the motors downloaded from ThrustCurve.org (cached or fetched this session).
    downloaded: HashSet<String>,
    /// What is flying: the whole stack or only the upper stages. A view setting, not part of the document.
    phase: Phase,
    /// The document revision and phase the cache was computed for.
    computed_for: Option<(u64, Phase)>,
    /// The CFD plan of the stack from each stage (index `k` is model stage `k + 1`).
    cfd_plans: Vec<Result<crate::analyze::cfd::Plan, String>>,
    /// The units `cfd_plans` was made in.
    cfd_units: Option<crate::units::UnitSystem>,
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
        Physics::with_motors_dir(None)
    }

    /// The cache with the user's motors from `env.motors_dir` added to the bundled database.
    pub fn from_env(env: &super::env::Env) -> Self {
        Physics::with_motors_dir(env.motors_dir.clone())
    }

    fn with_motors_dir(motors_dir: Option<PathBuf>) -> Self {
        let (motors, downloaded) = Physics::initial_motors(motors_dir.as_deref());
        Physics {
            motors,
            motors_dir,
            downloaded,
            phase: Phase::Full,
            computed_for: None,
            cfd_plans: Vec::new(),
            cfd_units: None,
            liftoff: Err(BridgeError::EmptyRocket),
            burnout: Err(BridgeError::EmptyRocket),
            warnings: Vec::new(),
            previewed: None,
            stages: Vec::new(),
            config_margins: Vec::new(),
            gauge: GaugeState::new(0.0),
        }
    }

    /// The bundled catalogue plus the user's directory, and the ids of the cached downloads. The catalogue decodes in a few
    /// milliseconds, so this runs at startup.
    fn initial_motors(dir: Option<&Path>) -> (MotorDb, HashSet<String>) {
        let mut db = MotorDb::bundled();
        let downloaded = dir.map(|dir| load_dir(&mut db, dir)).unwrap_or_default();
        (db, downloaded)
    }

    /// Where motor `index` came from.
    pub fn motor_origin(&self, index: usize) -> MotorOrigin {
        let id = self.motors.motors.get(index).map_or("", |m| m.id.as_str());
        if id.starts_with(FILE_ID_PREFIX) {
            MotorOrigin::File
        } else if self.downloaded.contains(id) {
            MotorOrigin::Downloaded
        } else {
            MotorOrigin::Catalogue
        }
    }

    /// Loads the user's `.eng` and `.rse` files (from the file dialog), keeps copies in [`user_motors_dir`], and returns the notice.
    pub fn load_user_motors(&mut self, paths: &[PathBuf]) -> String {
        import_files(&mut self.motors, self.motors_dir.as_deref(), paths)
    }

    /// Adds a motor downloaded from ThrustCurve.org (replacing the catalogue's copy of it, if any) and caches it in
    /// [`user_motors_dir`] (when it has one) so it loads at the next start. Returns its index and, when the cache could not be
    /// written, why.
    pub fn add_downloaded(&mut self, motor: Motor) -> (usize, Option<String>) {
        let kept = self.motors_dir.as_deref().map(|dir| bowshock_motors::save_cached(dir, &motor, &bowshock_motors::today()));
        let note = match kept {
            Some(Err(e)) => Some(format!("{}: downloaded, but not kept for next time ({e})", motor.designation)),
            _ => None,
        };
        self.downloaded.insert(motor.id.clone());
        // A motor that was missing changes the balance and the warnings of the document already computed.
        self.computed_for = None;
        (self.motors.upsert(motor), note)
    }

    /// Adds a motor made from a thrust curve stored in an OpenRocket file (see [`Motor::renamed`]) and, when there is a motors
    /// directory, keeps it there as an `.eng` file so the saved document finds it by id at the next start (the cache of
    /// downloads is for ThrustCurve.org motors only). It is a file motor: [`MotorOrigin::File`], the picker's USER tag.
    /// Returns why it could not be kept, if so.
    pub fn add_embedded(&mut self, motor: Motor) -> Option<String> {
        let note = self
            .motors_dir
            .as_deref()
            .and_then(|dir| keep_as_eng(dir, &motor).err())
            .map(|e| format!("{}: thrust curve not kept for next time ({e})", motor.designation));
        self.computed_for = None;
        self.motors.upsert(motor);
        note
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

    /// Recomputes if `revision`, the phase or the units changed since the last time. Returns whether it did.
    pub fn sync(&mut self, rocket: &Rocket, revision: u64, units: crate::units::UnitSystem) -> bool {
        let geom_same = self.computed_for == Some((revision, self.phase));
        if geom_same && self.cfd_units == Some(units) {
            return false;
        }
        if !geom_same {
            // A phase that no longer exists (stages were removed) falls back to the whole stack.
            let stages = rocket.stages();
            if self.phase.first_stage() > stages.len().max(1) {
                self.phase = Phase::Full;
            }
            self.compute(rocket, stages);
            self.computed_for = Some((revision, self.phase));
        }
        self.cfd_plans = (1..=self.stages.len().max(1))
            .map(|stage| {
                let phase = if stage <= 1 { Phase::Full } else { Phase::UpperFrom(stage) };
                from_model::vehicle(rocket, &self.motors, phase).map_err(|e| e.to_string()).and_then(|v| crate::analyze::cfd::plan(rocket, &v, units))
            })
            .collect();
        self.cfd_units = Some(units);
        true
    }

    /// The CFD plan of the stack from model stage `stage` (1 is the whole rocket), cached per revision and units.
    pub fn cfd_plan(&self, stage: usize) -> Option<&Result<crate::analyze::cfd::Plan, String>> {
        self.cfd_plans.get(stage.checked_sub(1)?)
    }

    /// The stored CFD drag table the stack from `stage` flies on, or why it flies on the build-up instead.
    pub fn drag_table(&self, rocket: &Rocket, stage: usize) -> Result<bowshock_sim::DragTable, String> {
        let Some(table) = rocket.cfd_table(stage as u32) else {
            return Err("No CFD table for this rocket yet: solve one in Analyze > CFD table. Flying on Extended.".to_string());
        };
        match self.cfd_plan(stage) {
            Some(Ok(plan)) if plan.geometry == table.geometry => Ok(bowshock_sim::DragTable::from_cfd(table)),
            Some(Ok(_)) | None => Err("CFD table is stale (the rocket changed): flying on Extended.".to_string()),
            Some(Err(why)) => Err(format!("CFD cannot model this rocket ({why}): flying on Extended.")),
        }
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
    #[cfg(test)]
    pub fn burnout(&self) -> Option<&Balance> {
        self.burnout.as_ref().ok()
    }

    /// Why there is no balance (an empty rocket, a motor that is not in the database).
    #[cfg(test)]
    pub fn error(&self) -> Option<&BridgeError> {
        self.liftoff.as_ref().err()
    }

    /// The health list of the active configuration, worst first within each topic, including `Ok` readings.
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
        copy.set_motor(config, mount, Some(from_model::motor_ref(motor))).ok()?;
        from_model::balance(&copy, &self.motors, Phase::Full, BALANCE_MACH, 0.0, true).ok().map(|b| b.margin_cal)
    }

    /// [`margin_with_motor`](Self::margin_with_motor) for many motors (indices into the database) at once, as the picker's
    /// table needs it for a catalogue of a thousand motors. CP does not depend on the motor, so the rocket is balanced once
    /// with `mount` empty and each motor's mass is added at its CG station, exactly as the bridge places it. When emptying the
    /// mount would let another core mount of its stage fly instead (the bridge flies a stage's first loaded core mount), it
    /// falls back to the full computation per motor.
    pub fn margins_with_motors(&self, rocket: &Rocket, mount: &MotorMount, indices: &[usize]) -> Vec<Option<f64>> {
        let config = rocket.active_configuration();
        let other_core_loaded = mount.pod.is_none()
            && self.stages.iter().find(|s| s.mounts.contains(&mount.id)).is_some_and(|s| {
                let mounts = rocket.motor_mounts();
                s.mounts
                    .iter()
                    .filter(|&&id| id != mount.id)
                    .any(|&id| mounts.iter().any(|m| m.id == id && m.pod.is_none()) && config.slot(id).is_some_and(|slot| slot.motor.is_some()))
            });
        let empty = (!other_core_loaded)
            .then(|| {
                let mut copy = rocket.clone();
                let id = copy.active_config_id();
                copy.set_motor(id, mount.id, None).ok()?;
                from_model::balance(&copy, &self.motors, Phase::Full, BALANCE_MACH, 0.0, true).ok()
            })
            .flatten();
        indices
            .iter()
            .map(|&i| {
                let motor = self.motors.motors.get(i)?;
                let Some(b) = &empty else {
                    return self.margin_with_motor(rocket, mount.id, motor);
                };
                let added = f64::from(mount.copies.max(1)) * motor.total_mass;
                let mass = b.mass + added;
                let cg = if mass > 0.0 { (b.mass * b.cg + added * (mount.motor_aft_station - 0.5 * motor.length)) / mass } else { 0.0 };
                Some(if b.reference_diameter > 0.0 { (b.cp - cg) / b.reference_diameter } else { 0.0 })
            })
            .collect()
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
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762, tip_radius: 0.0 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.9, diameter: 0.0762 }).unwrap();
        r.attach_radial(tube, 0.78, 0.0, 3, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.003 }).unwrap();
        r
    }

    #[test]
    fn sync_recomputes_only_when_the_revision_or_phase_changes() {
        let mut physics = Physics::new();
        let rocket = finned();
        assert!(physics.sync(&Rocket::default(), 0, crate::units::UnitSystem::MM_G_S));
        assert!(physics.balance().is_none() && physics.error() == Some(&BridgeError::EmptyRocket));
        assert!(physics.sync(&rocket, 1, crate::units::UnitSystem::MM_G_S));
        assert!(!physics.sync(&rocket, 1, crate::units::UnitSystem::MM_G_S));
        let balance = *physics.balance().unwrap();
        // A finned rocket: CP aft of CG, positive margin, length and diameter from the geometry.
        assert!(balance.cp > balance.cg && balance.margin_cal > 0.0);
        assert!((balance.length - 1.1286).abs() < 1e-9 && (balance.reference_diameter - 0.0762).abs() < 1e-9);
        // Burnout has no more mass than liftoff.
        assert!(physics.burnout().unwrap().mass <= balance.mass);
        assert!(physics.sync(&rocket, 2, crate::units::UnitSystem::MM_G_S));
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
        physics.sync(&rocket, 1, crate::units::UnitSystem::MM_G_S);
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
    fn cached_downloads_replace_the_catalogue_copy_and_load_first() {
        let store = temp_dir("cache");
        let mut db = MotorDb::bundled();
        let n = db.len();
        // A newer download of the catalogue's H128W, and a motor the catalogue does not have.
        let mut h = db.get("tc:5f4294d20002310000000083").unwrap().clone();
        h.thrustcurve.as_mut().unwrap().availability = "OOP".into();
        let mut new = h.clone();
        new.id = "tc:000000000000000000000001".into();
        new.designation = "H999X".into();
        bowshock_motors::save_cached(&store, &h, "2026-11-15").unwrap();
        bowshock_motors::save_cached(&store, &new, "2026-11-15").unwrap();
        let downloaded = load_dir(&mut db, &store);
        assert_eq!(downloaded.len(), 2);
        assert_eq!(db.len(), n + 1, "the cached H128W is not listed twice");
        assert!(db.get(&h.id).unwrap().thrustcurve.as_ref().unwrap().out_of_production());
        assert_eq!(db.find("H999X").unwrap().id, new.id);
        let _ = std::fs::remove_dir_all(&store);
    }

    #[test]
    fn downloaded_motors_are_tagged_and_flown() {
        let mut physics = Physics::new();
        let mut new = physics.motors().find("H128W").unwrap().clone();
        new.id = "tc:000000000000000000000002".into();
        new.designation = "H128W-NEW".into();
        let (index, note) = physics.add_downloaded(new);
        assert_eq!(note, None);
        assert_eq!(physics.motor_origin(index), MotorOrigin::Downloaded);
        assert_eq!(physics.motor_origin(physics.motors().position("tc:5f4294d20002310000000083").unwrap()), MotorOrigin::Catalogue);
        let mut db = MotorDb::default();
        import_eng(&mut db, "c.eng", CUSTOM).unwrap();
        physics.motors.upsert(db.motors.remove(0));
        assert_eq!(physics.motor_origin(physics.motors().len() - 1), MotorOrigin::File);
    }

    const RSE: &str = "<engine-database><engine-list><engine mfg=\"AeroTech\" code=\"I32N\" dia=\"38\" len=\"356\" initWt=\"647\" propWt=\"380.5\" delays=\"1000\">\
        <comments>from a post</comments><data><eng-data t=\"0\" f=\"0\" m=\"647\"/><eng-data t=\"0.055\" f=\"93.099\" m=\"645\"/><eng-data t=\"20.192\" f=\"0\" m=\"266.5\"/></data></engine></engine-list></engine-database>";

    #[test]
    fn rse_files_load_like_eng_files_and_are_kept_for_the_next_start() {
        let dir = temp_dir("rse");
        let source = dir.join("Curve.rse");
        std::fs::write(&source, RSE).unwrap();
        let store = dir.join("store");
        let mut db = MotorDb::bundled();
        let bundled = db.len();
        assert_eq!(import_files(&mut db, Some(&store), std::slice::from_ref(&source)), "Loaded 1 motor from Curve.rse");
        assert_eq!(db.len(), bundled + 1);
        assert_eq!(import_files(&mut db, None, &[source]), "Loaded 0 motors from Curve.rse (1 already known)");
        assert!(store.join("Curve.rse").is_file());
        let mut next = MotorDb::bundled();
        load_dir(&mut next, &store);
        assert_eq!(next.motors.last().unwrap().designation, "I32N");
        let bad = dir.join("Bad.rse");
        std::fs::write(&bad, "<engine code=\"Q1\" dia=\"1\" len=\"1\" initWt=\"1\" propWt=\"1\"><data/></engine>").unwrap();
        let notice = import_files(&mut db, None, &[bad]);
        assert!(notice.starts_with("Bad.rse · ") && notice.contains("Q1"), "{notice}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn embedded_curves_are_user_motors_kept_as_eng_files_that_keep_their_id() {
        let store = temp_dir("embedded");
        let env = crate::app::env::Env { motors_dir: Some(store.clone()), ..crate::app::env::Env::for_tests() };
        let mut physics = Physics::from_env(&env);
        let curve = bowshock_motors::parse_rse(RSE).unwrap().remove(0);
        let j33n = curve.renamed("J33N", "AeroTech");
        let j34n = curve.renamed("J34N", "Kuhns");
        assert_ne!(j33n.id, j34n.id, "one curve under two names is two motors");
        assert!(j33n.id.starts_with(FILE_ID_PREFIX));
        for m in [&j33n, &j34n] {
            assert_eq!(physics.add_embedded(m.clone()), None);
            let index = physics.motors().position(&m.id).unwrap();
            assert_eq!(physics.motor_origin(index), MotorOrigin::File);
        }
        // The next session finds both by id, with the same data.
        let next = Physics::from_env(&env);
        for m in [&j33n, &j34n] {
            let found = next.motors().get(&m.id).unwrap_or_else(|| panic!("{} was not kept", m.designation));
            assert_eq!((found.designation.as_str(), found.manufacturer.as_str()), (m.designation.as_str(), m.manufacturer.as_str()));
            assert_eq!(found.points, m.points);
            assert_eq!(found.delays, m.delays);
            assert!((found.total_mass - m.total_mass).abs() < 1e-9 && (found.propellant_mass - m.propellant_mass).abs() < 1e-9);
        }
        // A name that cannot be a file name is reported, and the motor still flies this session.
        let odd = curve.renamed("J 33/N", "AeroTech");
        assert!(physics.add_embedded(odd.clone()).is_some());
        assert!(physics.motors().get(&odd.id).is_some());
        let _ = std::fs::remove_dir_all(&store);
    }

    #[test]
    fn batch_margins_match_the_full_computation() {
        let mut physics = Physics::new();
        let mut rocket = finned();
        let tube = rocket.parts().iter().find(|p| matches!(p.kind, bowshock_model::PartKind::BodyTube { .. })).map(|p| p.id).unwrap();
        rocket
            .attach_internal(
                tube,
                0.6,
                bowshock_model::InternalKind::InnerTube { outer_diameter: 0.031, thickness: 0.001, length: 0.3, motor_mount: true, overhang: 0.0 },
            )
            .unwrap();
        physics.sync(&rocket, 1, crate::units::UnitSystem::MM_G_S);
        let mount = physics.mounts_of_stage(&rocket, 1).remove(0);
        let indices: Vec<usize> = (0..physics.motors().len()).filter(|&i| (physics.motors().motors[i].diameter - 0.029).abs() < 1e-3).take(40).collect();
        assert!(indices.len() >= 20);
        let fast = physics.margins_with_motors(&rocket, &mount, &indices);
        for (&i, fast) in indices.iter().zip(fast) {
            let full = physics.margin_with_motor(&rocket, mount.id, &physics.motors().motors[i]);
            assert!((fast.unwrap() - full.unwrap()).abs() < 1e-9, "{}: {fast:?} vs {full:?}", physics.motors().motors[i].designation);
        }
    }

    #[test]
    fn duplicates_are_skipped_and_reported() {
        let mut db = MotorDb::bundled();
        let before = db.len();
        assert_eq!(import_eng(&mut db, "a.eng", CUSTOM), Ok((1, 0)));
        assert_eq!(import_eng(&mut db, "a.eng", CUSTOM), Ok((0, 1)));
        // ThrustCurve's H128W certification file is the catalogue's H128W.
        let h128w = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../bowshock-motors/tests/fixtures/aerotech_h128w.eng")).unwrap();
        assert_eq!(import_eng(&mut db, "h.eng", &h128w), Ok((0, 1)));
        // A different curve under the same designation and maker is another motor, not a duplicate.
        let other = h128w.replace("102.423", "99.0");
        assert_eq!(import_eng(&mut db, "h2.eng", &other), Ok((1, 0)));
        assert_eq!(db.motors.iter().filter(|m| m.designation == "H128W" && m.manufacturer == "AeroTech").count(), 2);
        assert_eq!(db.len(), before + 2);
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
