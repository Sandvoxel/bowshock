//! App settings kept per user, not per document: they live in the user config directory next to the recent-files list and the
//! user motors (`~/.config/bowshock/settings.json`, or `$BOWSHOCK_SETTINGS_FILE`), so they follow the person to every rocket.
//!
//! Today that is the display unit set (top bar select and its custom panel), the theme, where CFD solves run ([`Compute`]) and which
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

/// Where the CFD solves run.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "lowercase")]
pub enum Compute {
    /// On this computer (its GPU, else its CPU).
    #[default]
    Local,
    /// On another computer's GPU over SSH (`bowshock-remote`): `target` is `user@host` or an `ssh` alias.
    Remote {
        target: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        port: Option<u16>,
    },
}

impl Compute {
    /// Remote compute from the text of the target field: `user@host`, or `user@host:port` for an SSH port other than 22.
    pub fn remote(text: &str) -> Compute {
        let text = text.trim();
        if let Some((target, port)) = text.rsplit_once(':') {
            if let Ok(port) = port.parse::<u16>() {
                return Compute::Remote { target: target.trim().to_string(), port: Some(port) };
            }
        }
        Compute::Remote { target: text.to_string(), port: None }
    }

    /// The target field's text for this setting (empty for Local).
    pub fn text(&self) -> String {
        match self {
            Compute::Local => String::new(),
            Compute::Remote { target, port: None } => target.clone(),
            Compute::Remote { target, port: Some(p) } => format!("{target}:{p}"),
        }
    }

    /// The SSH host of a remote setting with a target typed in.
    pub fn host(&self) -> Option<bowshock_remote::RemoteHost> {
        match self {
            Compute::Remote { target, port } if !target.trim().is_empty() => {
                Some(bowshock_remote::RemoteHost { target: target.trim().to_string(), port: *port })
            }
            _ => None,
        }
    }

    /// Whether this is remote compute.
    pub fn is_remote(&self) -> bool {
        matches!(self, Compute::Remote { .. })
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
        let compute = stored.compute.and_then(|v| serde_json::from_value(v).ok()).unwrap_or_default();
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

    /// Switches to the unit set `units` (top bar select, Preferences) and keeps it.
    pub fn set_units(&mut self, units: UnitSystem) {
        crate::units::clear_unit_overrides();
        self.units = units;
        self.save_settings();
    }

    /// Switches between the dark and the light theme (top bar toggle, Preferences) and keeps the choice.
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
        assert_eq!(loaded.compute, Compute::Local, "compute is local unless chosen");
        let _ = std::fs::remove_file(&file);
    }

    #[test]
    fn the_compute_choice_survives_a_restart() {
        let file = temp("compute.json");
        let remote = Compute::Remote { target: "sandvoxel@192.168.1.88".to_string(), port: Some(2222) };
        Settings { compute: remote.clone(), ..Settings::default() }.save(Some(&file)).unwrap();
        assert_eq!(Settings::load(Some(&file)).compute, remote);
        Settings::default().save(Some(&file)).unwrap();
        assert_eq!(Settings::load(Some(&file)).compute, Compute::Local);
        // A value the build does not know leaves the units alone and compute local.
        std::fs::write(&file, r#"{"units":{"length":"in"},"compute":{"mode":"cloud"}}"#).unwrap();
        let loaded = Settings::load(Some(&file));
        assert_eq!(loaded.compute, Compute::Local);
        assert_eq!(loaded.units, UnitSystem::MM_G_S.with(Family::Length, "in"));
        let _ = std::fs::remove_file(&file);
    }

    #[test]
    fn the_target_field_reads_a_port() {
        assert_eq!(Compute::remote(" me@box "), Compute::Remote { target: "me@box".into(), port: None });
        assert_eq!(Compute::remote("me@box:2200"), Compute::Remote { target: "me@box".into(), port: Some(2200) });
        assert_eq!(Compute::remote("me@box:2200").text(), "me@box:2200");
        assert_eq!(Compute::remote("").host(), None, "an empty target is no host");
        assert_eq!(Compute::Local.host(), None);
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
