//! App settings kept per user, not per document: they live in the user config directory next to the recent-files list and the
//! user motors (`~/.config/bowshock/settings.json`, or `$BOWSHOCK_SETTINGS_FILE`), so they follow the person to every rocket.
//!
//! Today that is the display unit set (Preferences > Units), the theme, where CFD solves run ([`Compute`]) and which
//! device the local ones use ([`LocalSolver`]); Preferences (`crate::prefs`) edits all of them. A missing, unreadable or malformed
//! file is the defaults; unknown keys and unit symbols are ignored, so a newer or older build never refuses to start over a settings
//! file.

use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::units::UnitSystem;

/// What is stored on disk: the unit symbol chosen for each [`crate::units::Family`], by its key.
#[derive(Debug, Default, Serialize, Deserialize)]
struct Stored {
    #[serde(default)]
    units: BTreeMap<String, String>,
    /// Kept as loose JSON so a value an older or newer build wrote cannot cost the unit set.
    #[serde(default)]
    compute: Option<serde_json::Value>,
    /// `"dark"` or `"light"` (loose, like `compute`).
    #[serde(default)]
    theme: Option<serde_json::Value>,
    /// `"auto"` or `"cpu"` (loose, like `compute`).
    #[serde(default)]
    solver: Option<serde_json::Value>,
}

/// Which device the solves that run on this computer use.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum LocalSolver {
    /// The GPU when there is one, else the CPU.
    #[default]
    Auto,
    /// The CPU only, even when a GPU is present.
    Cpu,
}

fn yes() -> bool {
    true
}

/// A computer the solves can run on besides this one: its GPU over SSH (`bowshock-remote`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Machine {
    /// `user@host` or an `ssh` alias.
    pub target: String,
    /// The SSH port when it is not 22.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub port: Option<u16>,
    /// Switched on: used by the solves. A machine that is off is never contacted.
    #[serde(default = "yes")]
    pub enabled: bool,
    /// A name for the list; the target when there is none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

impl Machine {
    /// A machine from the text of the target field: `user@host`, or `user@host:port` for an SSH port other than 22. Switched on.
    pub fn parse(text: &str) -> Machine {
        let text = text.trim();
        if let Some((target, port)) = text.rsplit_once(':') {
            if let Ok(port) = port.parse::<u16>() {
                return Machine { target: target.trim().to_string(), port: Some(port), enabled: true, label: None };
            }
        }
        Machine { target: text.to_string(), port: None, enabled: true, label: None }
    }

    /// What names the machine in the settings: `user@host`, or `user@host:port`.
    pub fn key(&self) -> String {
        match self.port {
            Some(p) => format!("{}:{p}", self.target),
            None => self.target.clone(),
        }
    }

    /// The label, or the target.
    pub fn name(&self) -> String {
        self.label.as_deref().map(str::trim).filter(|l| !l.is_empty()).map_or_else(|| self.key(), str::to_string)
    }

    /// The SSH host, when a target is typed in.
    pub fn host(&self) -> Option<bowshock_remote::RemoteHost> {
        (!self.target.trim().is_empty()).then(|| bowshock_remote::RemoteHost { target: self.target.trim().to_string(), port: self.port })
    }
}

/// One computer a solve can run on.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MachineRef {
    /// This computer.
    This,
    /// The machine with this [`Machine::key`].
    Machine(String),
}

/// How the solves use the machines that are switched on.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Mode {
    /// All on one machine.
    Pick(MachineRef),
    /// The Mach numbers are shared out over every switched-on machine, each taking the next when it is free.
    Share,
}

/// Where a solve runs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Where {
    /// This computer (its GPU, else its CPU).
    This,
    /// A machine over SSH.
    Remote {
        /// What the screen calls it.
        name: String,
        /// How to reach it.
        host: bowshock_remote::RemoteHost,
    },
}

impl Where {
    /// What the screen calls it.
    pub fn name(&self) -> &str {
        match self {
            Where::This => "This computer",
            Where::Remote { name, .. } => name,
        }
    }
}

/// Where the CFD solves run: this computer and the machines added in Preferences, each switched on or off, used one at a time or
/// all together.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Compute {
    /// The machines besides this computer.
    #[serde(default)]
    pub machines: Vec<Machine>,
    /// Whether this computer takes part.
    #[serde(default = "yes")]
    pub use_this_computer: bool,
    /// One machine, or all that are on.
    pub mode: Mode,
}

impl Default for Compute {
    fn default() -> Self {
        Compute { machines: Vec::new(), use_this_computer: true, mode: Mode::Pick(MachineRef::This) }
    }
}

impl Compute {
    /// Reads the `compute` of a settings file, this build's form or the one before it (`{"mode":"remote","target":…}`: one switched-on
    /// machine, picked; `{"mode":"local"}`: this computer). `None` for anything else.
    pub fn from_json(value: serde_json::Value) -> Option<Compute> {
        if value.get("machines").is_some() {
            return serde_json::from_value(value).ok();
        }
        match value.get("mode")?.as_str()? {
            "local" => Some(Compute::default()),
            "remote" => {
                let target = value.get("target")?.as_str()?.to_string();
                let port = value.get("port").and_then(|p| p.as_u64()).and_then(|p| u16::try_from(p).ok());
                let machine = Machine { target, port, enabled: true, label: None };
                Some(Compute { mode: Mode::Pick(MachineRef::Machine(machine.key())), machines: vec![machine], ..Compute::default() })
            }
            _ => None,
        }
    }

    /// The machine with this key.
    pub fn machine(&self, key: &str) -> Option<&Machine> {
        self.machines.iter().find(|m| m.key() == key)
    }

    /// Where `r` runs, if it can be used now: this computer when it takes part, a machine that is on with a target.
    fn place(&self, r: &MachineRef) -> Option<Where> {
        match r {
            MachineRef::This => self.use_this_computer.then_some(Where::This),
            MachineRef::Machine(key) => {
                let m = self.machine(key).filter(|m| m.enabled)?;
                Some(Where::Remote { name: m.name(), host: m.host()? })
            }
        }
    }

    /// Everything that can be picked: this computer when it takes part, then the machines that are on.
    pub fn pickable(&self) -> Vec<(MachineRef, String)> {
        let this = self.use_this_computer.then(|| (MachineRef::This, "This computer".to_string()));
        let machines = self.machines.iter().filter(|m| m.enabled && m.host().is_some()).map(|m| (MachineRef::Machine(m.key()), m.name()));
        this.into_iter().chain(machines).collect()
    }

    /// Keeps the settings consistent after an edit: the picked machine is one that can be used (else the first that can, this
    /// computer first), and no two machines share a key.
    pub fn normalise(&mut self) {
        let mut seen = Vec::new();
        self.machines.retain(|m| {
            let key = m.key();
            let fresh = !m.target.trim().is_empty() && !seen.contains(&key);
            seen.push(key);
            fresh
        });
        if let Mode::Pick(r) = &self.mode {
            if self.place(r).is_none() {
                if let Some((first, _)) = self.pickable().into_iter().next() {
                    self.mode = Mode::Pick(first);
                }
            }
        }
    }

    /// Where a solve runs now: the picked machine, or every one that is on. An error for the text of the solve screen when there is
    /// none.
    pub fn places(&self) -> Result<Vec<Where>, String> {
        match &self.mode {
            Mode::Pick(r) => self
                .place(r)
                .map(|w| vec![w])
                .ok_or_else(|| "The machine picked in Preferences > Compute is switched off; switch it on or pick another.".to_string()),
            Mode::Share => {
                let places: Vec<Where> = self.pickable().iter().filter_map(|(r, _)| self.place(r)).collect();
                if places.is_empty() {
                    Err("No machine is switched on in Preferences > Compute.".to_string())
                } else {
                    Ok(places)
                }
            }
        }
    }

    /// The one-line `Runs on:` text: the picked machine (with what its last test found, `devices` by key), or the machines shared.
    pub fn runs_on(&self, devices: &std::collections::BTreeMap<String, String>) -> String {
        match &self.mode {
            Mode::Pick(MachineRef::This) => "This computer".to_string(),
            Mode::Pick(MachineRef::Machine(key)) => match (devices.get(key), self.machine(key)) {
                (Some(device), _) => device.clone(),
                (None, Some(m)) => m.name(),
                (None, None) => key.clone(),
            },
            Mode::Share => match self.places() {
                Ok(p) => format!("Shared: {} machine{}", p.len(), if p.len() == 1 { "" } else { "s" }),
                Err(_) => "Shared: no machine is on".to_string(),
            },
        }
    }
}

/// The user's settings.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Settings {
    /// The display unit set.
    pub units: UnitSystem,
    /// Where the CFD solves run.
    pub compute: Compute,
    /// Dark theme (else light).
    pub dark: bool,
    /// The device local solves use.
    pub local_solver: LocalSolver,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { units: UnitSystem::default(), compute: Compute::default(), dark: true, local_solver: LocalSolver::default() }
    }
}

impl Settings {
    /// The settings stored in `file`; the defaults when there is no file or it cannot be read.
    pub fn load(file: Option<&Path>) -> Settings {
        let stored: Stored = file.and_then(|f| std::fs::read_to_string(f).ok()).and_then(|text| serde_json::from_str(&text).ok()).unwrap_or_default();
        let mut compute = stored.compute.and_then(Compute::from_json).unwrap_or_default();
        compute.normalise();
        let dark = stored.theme.as_ref().and_then(|v| v.as_str()) != Some("light");
        let local_solver = if stored.solver.as_ref().and_then(|v| v.as_str()) == Some("cpu") { LocalSolver::Cpu } else { LocalSolver::Auto };
        Settings { units: UnitSystem::from_symbols(stored.units.iter().map(|(k, v)| (k.as_str(), v.as_str()))), compute, dark, local_solver }
    }

    /// Writes the settings to `file` (creating its directory); `None` is a session with nowhere to keep them.
    pub fn save(&self, file: Option<&Path>) -> std::io::Result<()> {
        let Some(file) = file else { return Ok(()) };
        if let Some(dir) = file.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let stored = Stored {
            units: self.units.symbols().into_iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
            compute: serde_json::to_value(&self.compute).ok(),
            theme: Some(if self.dark { "dark" } else { "light" }.into()),
            solver: Some(if self.local_solver == LocalSolver::Cpu { "cpu" } else { "auto" }.into()),
        };
        let text = serde_json::to_string_pretty(&stored).map_err(std::io::Error::other)?;
        std::fs::write(file, text)
    }
}

impl super::Ctx {
    /// The settings as the context holds them now.
    pub fn settings(&self) -> Settings {
        Settings { units: self.units, compute: self.compute.clone(), dark: self.dark, local_solver: self.local_solver }
    }

    /// Loads `settings` into the context (at start-up).
    pub(super) fn apply_settings(&mut self, settings: Settings) {
        self.units = settings.units;
        self.compute = settings.compute;
        self.dark = settings.dark;
        self.local_solver = settings.local_solver;
    }

    /// Writes every user setting to the settings file; a failure is a status message, never a stop.
    pub fn save_settings(&mut self) {
        if let Err(e) = self.settings().save(self.settings_file.as_deref()) {
            self.notify(format!("Could not save settings: {e}"));
        }
    }

    /// Switches to the unit set `units` (Preferences) and keeps it.
    pub fn set_units(&mut self, units: UnitSystem) {
        crate::units::clear_unit_overrides();
        self.units = units;
        self.save_settings();
    }

    /// Switches between the dark and the light theme (Preferences) and keeps the choice.
    pub fn set_dark(&mut self, dark: bool) {
        self.dark = dark;
        self.save_settings();
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::units::Family;

    fn temp(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("bowshock_settings_{}_{name}", std::process::id()))
    }

    #[test]
    fn the_unit_set_survives_a_restart() {
        let file = temp("round.json");
        let units = UnitSystem::MM_G_S.with(Family::Length, "in").with(Family::Mass, "kg").with(Family::Pressure, "psi");
        Settings { units, ..Settings::default() }.save(Some(&file)).unwrap();
        let loaded = Settings::load(Some(&file));
        assert_eq!(loaded.units, units);
        assert_eq!(loaded.units.label(), "in · kg · s");
        assert_eq!(loaded.compute, Compute::default(), "compute is this computer unless chosen");
        let _ = std::fs::remove_file(&file);
    }

    fn two_machines() -> Compute {
        let mut c = Compute {
            machines: vec![Machine::parse("sandvoxel@192.168.1.88"), Machine { label: Some("Laptop".into()), enabled: false, ..Machine::parse("me@box:2222") }],
            ..Compute::default()
        };
        c.mode = Mode::Share;
        c
    }

    #[test]
    fn the_compute_choice_survives_a_restart() {
        let file = temp("compute.json");
        let compute = two_machines();
        Settings { compute: compute.clone(), ..Settings::default() }.save(Some(&file)).unwrap();
        assert_eq!(Settings::load(Some(&file)).compute, compute);
        let picked = Compute { mode: Mode::Pick(MachineRef::Machine("me@box:2222".into())), ..Compute::default() };
        assert_eq!(picked.mode, Mode::Pick(MachineRef::Machine("me@box:2222".into())));
        Settings::default().save(Some(&file)).unwrap();
        assert_eq!(Settings::load(Some(&file)).compute, Compute::default());
        // A value the build does not know leaves the units alone and compute at its default.
        std::fs::write(&file, r#"{"units":{"length":"in"},"compute":{"mode":"cloud"}}"#).unwrap();
        let loaded = Settings::load(Some(&file));
        assert_eq!(loaded.compute, Compute::default());
        assert_eq!(loaded.units, UnitSystem::MM_G_S.with(Family::Length, "in"));
        let _ = std::fs::remove_file(&file);
    }

    #[test]
    fn the_single_machine_settings_of_the_build_before_are_read_losslessly() {
        let file = temp("migrate.json");
        std::fs::write(&file, r#"{"units":{"length":"in"},"compute":{"mode":"remote","target":"sandvoxel@192.168.1.88","port":2222}}"#).unwrap();
        let loaded = Settings::load(Some(&file));
        let machine = Machine { target: "sandvoxel@192.168.1.88".into(), port: Some(2222), enabled: true, label: None };
        assert_eq!(loaded.compute.machines, vec![machine]);
        assert_eq!(loaded.compute.mode, Mode::Pick(MachineRef::Machine("sandvoxel@192.168.1.88:2222".into())));
        assert!(loaded.compute.use_this_computer);
        assert_eq!(loaded.compute.places().unwrap().iter().map(|w| w.name().to_string()).collect::<Vec<_>>(), ["sandvoxel@192.168.1.88:2222"]);
        assert_eq!(loaded.units, UnitSystem::MM_G_S.with(Family::Length, "in"));
        std::fs::write(&file, r#"{"compute":{"mode":"remote","target":"me@box"}}"#).unwrap();
        assert_eq!(Settings::load(Some(&file)).compute.machines[0].port, None);
        std::fs::write(&file, r#"{"compute":{"mode":"local"}}"#).unwrap();
        assert_eq!(Settings::load(Some(&file)).compute, Compute::default());
        // Saved again it is the new form and reads back the same.
        let again = Settings { compute: loaded.compute.clone(), ..Settings::default() };
        again.save(Some(&file)).unwrap();
        assert!(std::fs::read_to_string(&file).unwrap().contains("\"machines\""));
        assert_eq!(Settings::load(Some(&file)).compute, loaded.compute);
        let _ = std::fs::remove_file(&file);
    }

    #[test]
    fn the_target_field_reads_a_port_and_a_machine_is_named_by_its_label() {
        assert_eq!(Machine::parse(" me@box ").key(), "me@box");
        assert_eq!(Machine::parse("me@box:2200").port, Some(2200));
        assert_eq!(Machine::parse("me@box:2200").key(), "me@box:2200");
        assert_eq!(Machine::parse("me@box:2200").name(), "me@box:2200");
        assert_eq!(Machine { label: Some("Laptop".into()), ..Machine::parse("me@box") }.name(), "Laptop");
        assert_eq!(Machine::parse("").host(), None, "an empty target is no host");
    }

    #[test]
    fn the_places_of_a_solve_are_the_picked_machine_or_every_one_that_is_on() {
        let mut c = two_machines();
        let names = |c: &Compute| c.places().map(|p| p.iter().map(|w| w.name().to_string()).collect::<Vec<_>>());
        // Share: this computer and the machine that is on; the one that is off is not a place.
        assert_eq!(names(&c).unwrap(), ["This computer", "sandvoxel@192.168.1.88"]);
        assert_eq!(c.runs_on(&Default::default()), "Shared: 2 machines");
        c.use_this_computer = false;
        assert_eq!(names(&c).unwrap(), ["sandvoxel@192.168.1.88"]);
        c.machines[0].enabled = false;
        assert!(names(&c).unwrap_err().contains("No machine is switched on"));
        // Pick: the picked one; switching it off moves the pick to one that can be used.
        let mut c = two_machines();
        c.mode = Mode::Pick(MachineRef::Machine("sandvoxel@192.168.1.88".into()));
        assert_eq!(names(&c).unwrap(), ["sandvoxel@192.168.1.88"]);
        let devices = std::collections::BTreeMap::from([("sandvoxel@192.168.1.88".to_string(), "RTX 3080 on omarchy".to_string())]);
        assert_eq!(c.runs_on(&devices), "RTX 3080 on omarchy");
        c.machines[0].enabled = false;
        assert!(names(&c).is_err(), "before normalising the pick is off");
        c.normalise();
        assert_eq!(c.mode, Mode::Pick(MachineRef::This));
        assert_eq!(names(&c).unwrap(), ["This computer"]);
        assert_eq!(c.pickable().len(), 1);
        // Duplicates and empty targets are dropped.
        c.machines = vec![Machine::parse("a@b"), Machine::parse("a@b"), Machine::parse("  ")];
        c.normalise();
        assert_eq!(c.machines.len(), 1);
    }

    #[test]
    fn a_missing_or_damaged_file_is_the_defaults() {
        assert_eq!(Settings::load(None), Settings::default());
        assert_eq!(Settings::load(Some(&temp("missing.json"))), Settings::default());
        let file = temp("damaged.json");
        std::fs::write(&file, "{ not json").unwrap();
        assert_eq!(Settings::load(Some(&file)), Settings::default());
        // Unknown keys and symbols are skipped; what is known still applies.
        std::fs::write(&file, r#"{"units":{"length":"in","mass":"stone","colour":"red"},"theme":"dark"}"#).unwrap();
        let units = Settings::load(Some(&file)).units;
        assert_eq!(units, UnitSystem::MM_G_S.with(Family::Length, "in"));
        let _ = std::fs::remove_file(&file);
    }

    #[test]
    fn saving_creates_the_directory() {
        let dir = temp("dir");
        let file = dir.join("nested").join("settings.json");
        Settings::default().save(Some(&file)).unwrap();
        assert!(file.exists());
        let _ = std::fs::remove_dir_all(&dir);
    }
    #[test]
    fn the_theme_and_the_local_solver_survive_a_restart() {
        let file = temp("theme.json");
        assert!(Settings::default().dark, "dark is the default");
        assert_eq!(Settings::default().local_solver, LocalSolver::Auto);
        Settings { dark: false, local_solver: LocalSolver::Cpu, ..Settings::default() }.save(Some(&file)).unwrap();
        let loaded = Settings::load(Some(&file));
        assert!(!loaded.dark);
        assert_eq!(loaded.local_solver, LocalSolver::Cpu);
        assert!(std::fs::read_to_string(&file).unwrap().contains(r#""theme": "light""#));
        Settings::default().save(Some(&file)).unwrap();
        let loaded = Settings::load(Some(&file));
        assert!(loaded.dark && loaded.local_solver == LocalSolver::Auto);
        // A value this build does not know is the default and costs nothing else.
        std::fs::write(&file, r#"{"units":{"length":"in"},"theme":7,"solver":"quantum"}"#).unwrap();
        let loaded = Settings::load(Some(&file));
        assert!(loaded.dark && loaded.local_solver == LocalSolver::Auto);
        assert_eq!(loaded.units, UnitSystem::MM_G_S.with(Family::Length, "in"));
        // A file from before these settings existed is dark with Auto.
        std::fs::write(&file, r#"{"units":{"length":"in"}}"#).unwrap();
        assert!(Settings::load(Some(&file)).dark);
        let _ = std::fs::remove_file(&file);
    }
}
