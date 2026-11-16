//! App settings kept per user, not per document: they live in the user config directory next to the recent-files list and the
//! user motors (`~/.config/bowshock/settings.json`, or `$BOWSHOCK_SETTINGS_FILE`), so they follow the person to every rocket.
//!
//! Today that is the display unit set (top bar select and its custom panel). A missing, unreadable or malformed file is the
//! defaults; unknown keys and unit symbols are ignored, so a newer or older build never refuses to start over a settings file.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::units::UnitSystem;

/// What is stored on disk: the unit symbol chosen for each [`crate::units::Family`], by its key.
#[derive(Debug, Default, Serialize, Deserialize)]
struct Stored {
    #[serde(default)]
    units: BTreeMap<String, String>,
}

/// The user's settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Settings {
    /// The display unit set.
    pub units: UnitSystem,
}

impl Settings {
    /// Where the file lives: `$BOWSHOCK_SETTINGS_FILE`, else `~/.config/bowshock/settings.json`.
    pub fn default_file() -> Option<PathBuf> {
        let over = std::env::var_os("BOWSHOCK_SETTINGS_FILE").filter(|s| !s.is_empty()).map(PathBuf::from);
        over.or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config").join("bowshock").join("settings.json")))
    }

    /// The settings stored in `file`; the defaults when there is no file or it cannot be read.
    pub fn load(file: Option<&Path>) -> Settings {
        let stored: Stored = file.and_then(|f| std::fs::read_to_string(f).ok()).and_then(|text| serde_json::from_str(&text).ok()).unwrap_or_default();
        Settings { units: UnitSystem::from_symbols(stored.units.iter().map(|(k, v)| (k.as_str(), v.as_str()))) }
    }

    /// Writes the settings to `file` (creating its directory); `None` is a session with nowhere to keep them.
    pub fn save(&self, file: Option<&Path>) -> std::io::Result<()> {
        let Some(file) = file else { return Ok(()) };
        if let Some(dir) = file.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let stored = Stored { units: self.units.symbols().into_iter().map(|(k, v)| (k.to_string(), v.to_string())).collect() };
        let text = serde_json::to_string_pretty(&stored).map_err(std::io::Error::other)?;
        std::fs::write(file, text)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::Family;

    fn temp(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("bowshock_settings_{}_{name}", std::process::id()))
    }

    #[test]
    fn the_unit_set_survives_a_restart() {
        let file = temp("round.json");
        let units = UnitSystem::MM_G_S.with(Family::Length, "in").with(Family::Mass, "kg").with(Family::Pressure, "psi");
        Settings { units }.save(Some(&file)).unwrap();
        let loaded = Settings::load(Some(&file));
        assert_eq!(loaded.units, units);
        assert_eq!(loaded.units.label(), "in · kg · s");
        let _ = std::fs::remove_file(&file);
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
}
