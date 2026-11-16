//! Runtime environment and animation config: every `BOWSHOCK_*` variable and `$HOME` default is read once into an [`Env`] that
//! lives on `Ctx`, so screens never touch the process environment and unit tests never see the host's.

use std::ffi::OsString;
use std::path::PathBuf;

use iced::keyboard::Modifiers;

#[derive(Debug, Clone)]
pub struct Env {
    /// Camera and flash animations run (off in a headless `bowshock drive`, which delivers no frame ticks).
    pub animate: bool,
    /// `BOWSHOCK_FREEZE_FLASH`: hold the edit flash at its first frame so a screenshot can show it.
    pub freeze_flash: bool,
    /// `BOWSHOCK_OPEN`: a document to open at start-up.
    pub open_path: Option<PathBuf>,
    /// `BOWSHOCK_DRIVE_TAB`: press Tab once at start-up.
    pub drive_tab: bool,
    /// `BOWSHOCK_DRIVE_MODIFIERS`: modifiers a drive script holds.
    pub drive_modifiers: Modifiers,
    /// `BOWSHOCK_EXPORT_FILE`: answers the save dialog of an export or print.
    pub export_file: Option<PathBuf>,
    /// `BOWSHOCK_COMPARE_FILE`: a CSV loaded as the Analyze comparison at start-up.
    pub compare_file: Option<String>,
    /// `BOWSHOCK_CFD_DEMO`: scripted CFD demo names for the solve view.
    pub cfd_demo: Vec<String>,
    /// `BOWSHOCK_EXAMPLE=two_stage`: the two-stage example.
    pub example_two_stage: bool,
    pub recent_file: Option<PathBuf>,
    pub settings_file: Option<PathBuf>,
    pub motors_dir: Option<PathBuf>,
    pub presets_dir: Option<PathBuf>,
    /// `BOWSHOCK_THRUSTCURVE_API`: base URL of the ThrustCurve.org client.
    pub thrustcurve_api: Option<String>,
    /// `BOWSHOCK_CAPTURE_PNG`: where the 4K capture is written.
    #[cfg(test)]
    pub capture_png: Option<PathBuf>,
}

/// `shift`, `alt`, `ctrl` joined by `+`; anything else is ignored.
pub fn parse_drive_modifiers(raw: &str) -> Modifiers {
    let mut modifiers = Modifiers::empty();
    for name in raw.split('+') {
        modifiers |= match name.trim() {
            "shift" => Modifiers::SHIFT,
            "alt" => Modifiers::ALT,
            "ctrl" => Modifiers::CTRL,
            _ => Modifiers::empty(),
        };
    }
    modifiers
}

fn var(name: &str) -> Option<OsString> {
    std::env::var_os(name).filter(|s| !s.is_empty())
}

fn config_file(name: &str) -> Option<PathBuf> {
    std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config").join("bowshock").join(name))
}

/// Where the user's own motors live: `$BOWSHOCK_MOTORS_DIR`, else `~/.config/bowshock/motors/`.
pub fn default_motors_dir() -> Option<PathBuf> {
    var("BOWSHOCK_MOTORS_DIR").map(PathBuf::from).or_else(|| config_file("motors"))
}

impl Env {
    pub fn from_process(animate: bool) -> Self {
        Env {
            animate,
            freeze_flash: std::env::var_os("BOWSHOCK_FREEZE_FLASH").is_some(),
            open_path: var("BOWSHOCK_OPEN").map(PathBuf::from),
            drive_tab: std::env::var_os("BOWSHOCK_DRIVE_TAB").is_some(),
            drive_modifiers: parse_drive_modifiers(&std::env::var("BOWSHOCK_DRIVE_MODIFIERS").unwrap_or_default()),
            export_file: var("BOWSHOCK_EXPORT_FILE").map(PathBuf::from),
            compare_file: var("BOWSHOCK_COMPARE_FILE").map(|p| p.to_string_lossy().into_owned()),
            cfd_demo: std::env::var("BOWSHOCK_CFD_DEMO")
                .map(|v| v.split(',').map(str::trim).filter(|s| !s.is_empty()).map(String::from).collect())
                .unwrap_or_default(),
            example_two_stage: std::env::var("BOWSHOCK_EXAMPLE").is_ok_and(|v| v == "two_stage"),
            recent_file: var("BOWSHOCK_RECENT_FILE").map(PathBuf::from).or_else(|| config_file("recent.txt")),
            settings_file: var("BOWSHOCK_SETTINGS_FILE").map(PathBuf::from).or_else(|| config_file("settings.json")),
            motors_dir: default_motors_dir(),
            presets_dir: var("BOWSHOCK_PRESETS_DIR").map(PathBuf::from).or_else(|| config_file("presets")),
            thrustcurve_api: std::env::var("BOWSHOCK_THRUSTCURVE_API").ok().map(|v| v.trim().to_string()).filter(|v| !v.is_empty()),
            #[cfg(test)]
            capture_png: var("BOWSHOCK_CAPTURE_PNG").map(PathBuf::from),
        }
    }

    /// Animations on, nothing read from the host: tests never touch `~/.config/bowshock` or inherit a stray variable.
    pub fn for_tests() -> Self {
        Env {
            animate: true,
            freeze_flash: false,
            open_path: None,
            drive_tab: false,
            drive_modifiers: Modifiers::empty(),
            export_file: None,
            compare_file: None,
            cfd_demo: Vec::new(),
            example_two_stage: false,
            recent_file: None,
            settings_file: None,
            motors_dir: None,
            presets_dir: None,
            thrustcurve_api: None,
            #[cfg(test)]
            capture_png: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_env_reads_nothing_from_the_host() {
        let env = Env::for_tests();
        assert!(env.animate && !env.freeze_flash && !env.drive_tab && !env.example_two_stage);
        assert!(env.drive_modifiers.is_empty() && env.cfd_demo.is_empty());
        assert!(env.open_path.is_none() && env.export_file.is_none() && env.compare_file.is_none());
        assert!(env.recent_file.is_none() && env.settings_file.is_none() && env.motors_dir.is_none() && env.presets_dir.is_none());
        assert!(env.thrustcurve_api.is_none() && env.capture_png.is_none());
    }

    #[test]
    fn drive_modifiers_join_on_plus() {
        assert_eq!(parse_drive_modifiers("shift+alt"), Modifiers::SHIFT | Modifiers::ALT);
        assert_eq!(parse_drive_modifiers(" ctrl "), Modifiers::CTRL);
        assert!(parse_drive_modifiers("").is_empty());
        assert!(parse_drive_modifiers("meta+nonsense").is_empty());
    }
}
