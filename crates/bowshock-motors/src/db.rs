//! [`MotorDb`]: a searchable list of motors, with a bundled set of common hobby motors.

use crate::motor::Motor;
use crate::rasp::{parse_eng, ParseError};
use std::fmt;
use std::path::Path;

/// Motors whose outer diameter is within this of a mount's inner diameter are considered to fit, m.
const FIT_TOLERANCE: f64 = 0.0005;

/// Real thrust curves from ThrustCurve.org (see the attribution comments in each file).
const BUNDLED: &[(&str, &str)] = &[
    ("estes_a8.eng", include_str!("../data/estes_a8.eng")),
    ("estes_b6.eng", include_str!("../data/estes_b6.eng")),
    ("estes_c6.eng", include_str!("../data/estes_c6.eng")),
    ("estes_d12.eng", include_str!("../data/estes_d12.eng")),
    ("estes_e9.eng", include_str!("../data/estes_e9.eng")),
    ("estes_e12.eng", include_str!("../data/estes_e12.eng")),
    ("aerotech_f39t.eng", include_str!("../data/aerotech_f39t.eng")),
    ("aerotech_f67w.eng", include_str!("../data/aerotech_f67w.eng")),
    ("aerotech_g40w.eng", include_str!("../data/aerotech_g40w.eng")),
    ("aerotech_g80t.eng", include_str!("../data/aerotech_g80t.eng")),
    ("aerotech_h128w.eng", include_str!("../data/aerotech_h128w.eng")),
    ("aerotech_h238t.eng", include_str!("../data/aerotech_h238t.eng")),
    ("aerotech_i218r.eng", include_str!("../data/aerotech_i218r.eng")),
    ("aerotech_i284w.eng", include_str!("../data/aerotech_i284w.eng")),
    ("aerotech_j350w.eng", include_str!("../data/aerotech_j350w.eng")),
    ("aerotech_j420r.eng", include_str!("../data/aerotech_j420r.eng")),
    ("cesaroni_h133.eng", include_str!("../data/cesaroni_h133.eng")),
    ("cesaroni_i216.eng", include_str!("../data/cesaroni_i216.eng")),
    ("cesaroni_j285.eng", include_str!("../data/cesaroni_j285.eng")),
];

/// Error loading motors from a file.
#[derive(Debug)]
pub enum MotorError {
    /// The file could not be read.
    Io(std::io::Error),
    /// The file contents are not valid RASP `.eng`.
    Parse(ParseError),
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MotorError::Io(e) => write!(f, "cannot read motor file: {e}"),
            MotorError::Parse(e) => write!(f, "invalid motor file: {e}"),
        }
    }
}

impl std::error::Error for MotorError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            MotorError::Io(e) => Some(e),
            MotorError::Parse(e) => Some(e),
        }
    }
}

impl From<std::io::Error> for MotorError {
    fn from(e: std::io::Error) -> Self {
        MotorError::Io(e)
    }
}

impl From<ParseError> for MotorError {
    fn from(e: ParseError) -> Self {
        MotorError::Parse(e)
    }
}

/// A collection of motors.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MotorDb {
    /// All motors, in insertion order.
    pub motors: Vec<Motor>,
}

impl MotorDb {
    /// The motors bundled with the crate (Estes A8–E12, AeroTech F39–J420, Cesaroni H133/I216/J285).
    ///
    /// # Panics
    /// Never in practice: every bundled file is checked by the crate's tests.
    pub fn bundled() -> Self {
        let mut db = MotorDb::default();
        for (name, text) in BUNDLED {
            db.add_eng_str(text).unwrap_or_else(|e| panic!("bundled motor file {name} is invalid: {e}"));
        }
        db
    }

    /// Parses `.eng` text and appends its motors; returns how many were added.
    pub fn add_eng_str(&mut self, src: &str) -> Result<usize, ParseError> {
        let parsed = parse_eng(src)?;
        let n = parsed.len();
        self.motors.extend(parsed);
        Ok(n)
    }

    /// Reads an `.eng` file and appends its motors; returns how many were added.
    pub fn load_eng_file(&mut self, path: impl AsRef<Path>) -> Result<usize, MotorError> {
        let text = std::fs::read_to_string(path)?;
        Ok(self.add_eng_str(&text)?)
    }

    /// Number of motors.
    pub fn len(&self) -> usize {
        self.motors.len()
    }

    /// `true` when the database holds no motors.
    pub fn is_empty(&self) -> bool {
        self.motors.is_empty()
    }

    /// First motor named `designation`, ignoring case and any delay suffix (see [`Motor::matches`]):
    /// `H128W` and `h128w-14a` both find the AeroTech H128W.
    pub fn find(&self, designation: &str) -> Option<&Motor> {
        self.motors.iter().find(|m| m.designation.eq_ignore_ascii_case(designation.trim())).or_else(|| self.motors.iter().find(|m| m.matches(designation)))
    }

    /// Motors that fit a mount of inner diameter `diameter` (m): motor diameter within 0.5 mm, and no longer than
    /// `max_length` (m) when given.
    pub fn fitting(&self, diameter: f64, max_length: Option<f64>) -> impl Iterator<Item = &Motor> {
        self.motors.iter().filter(move |m| (m.diameter - diameter).abs() <= FIT_TOLERANCE + 1e-9 && max_length.is_none_or(|l| m.length <= l + 1e-9))
    }

    /// Motors of impulse class `class` (`'A'`..`'O'`, case-insensitive).
    pub fn by_class(&self, class: char) -> impl Iterator<Item = &Motor> {
        let class = class.to_ascii_uppercase();
        self.motors.iter().filter(move |m| m.impulse_class() == class)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// (designation, published total impulse N·s, diameter mm) from the ThrustCurve.org motor listings.
    const PUBLISHED: &[(&str, f64, f64)] = &[
        ("A8", 2.5, 18.0),
        ("B6", 4.33, 18.0),
        ("C6", 8.82, 18.0),
        ("D12", 16.84, 24.0),
        ("E12", 27.24, 24.0),
        ("E9", 27.87, 24.0),
        ("F39T", 49.66, 24.0),
        ("F67W", 61.1, 29.0),
        ("G40W", 97.14, 29.0),
        ("G80T", 136.6, 29.0),
        ("H128W", 172.9, 29.0),
        ("H238T", 165.5, 29.0),
        ("I218R", 319.63, 38.0),
        ("I284W", 607.3, 38.0),
        ("J350W", 700.0, 38.0),
        ("J420R", 658.0, 38.0),
        ("H133", 163.3, 29.0),
        ("I216", 636.1, 38.0),
        ("J285", 648.0, 38.0),
    ];

    #[test]
    fn bundled_files_parse_and_are_well_formed() {
        for (name, text) in BUNDLED {
            let motors = parse_eng(text).unwrap_or_else(|e| panic!("{name}: {e}"));
            assert!(!motors.is_empty(), "{name}");
            for m in motors {
                assert!(m.points.len() >= 3, "{name}");
                assert!(m.points.windows(2).all(|w| w[1].0 >= w[0].0), "{name}: time not monotonic");
                assert!(m.points.last().unwrap().1.abs() < 0.5, "{name}: does not end at ~0 thrust");
                assert!(m.total_impulse() > 0.0 && m.burn_time() > 0.0 && m.max_thrust() >= m.average_thrust(), "{name}");
                assert!(m.total_mass > m.propellant_mass && m.propellant_mass > 0.0, "{name}");
                assert!(!m.comments.is_empty(), "{name}: attribution comment missing");
                assert!(!m.approximated, "{name}");
            }
        }
        assert_eq!(MotorDb::bundled().len(), BUNDLED.len());
    }

    #[test]
    fn bundled_motors_match_published_impulse() {
        let db = MotorDb::bundled();
        for &(name, published, diameter_mm) in PUBLISHED {
            let m = db.find(name).unwrap_or_else(|| panic!("{name} not bundled"));
            let got = m.total_impulse();
            // The nominal listing values differ slightly from the integrated certification curves (A8: 2.32 vs 2.5 N·s).
            assert!((got - published).abs() / published < 0.10, "{name}: {got} N·s vs published {published}");
            assert!((m.diameter * 1000.0 - diameter_mm).abs() < 1e-6, "{name}");
        }
    }

    #[test]
    fn h128w_is_about_176_ns() {
        let db = MotorDb::bundled();
        let h = db.find("H128W").unwrap();
        assert!((h.total_impulse() - 176.0).abs() / 176.0 < 0.05, "{}", h.total_impulse());
        assert_eq!(h.impulse_class(), 'H');
        assert_eq!(h.manufacturer, "AeroTech");
        assert!((h.burn_time() - 1.293).abs() < 1e-9);
        assert!((h.average_thrust() - h.total_impulse() / h.burn_time()).abs() < 1e-9);
        assert!(h.thrust(0.5) > 150.0);
        assert!(h.mass(0.0) > h.mass(5.0) && (h.mass(5.0) - (0.2016 - 0.09408)).abs() < 1e-9);
    }

    #[test]
    fn find_is_case_insensitive_and_ignores_delay() {
        let db = MotorDb::bundled();
        assert_eq!(db.find("h128w-14a").unwrap().designation, "H128W");
        assert_eq!(db.find("H133-14A").unwrap().manufacturer, "Cesaroni");
        assert_eq!(db.find("B6-4").unwrap().designation, "B6-0");
        assert_eq!(db.find("j350w").unwrap().common_name, "J350W");
        assert!(db.find("Z999").is_none());
    }

    #[test]
    fn fitting_and_class_queries() {
        let db = MotorDb::bundled();
        let fit29: Vec<_> = db.fitting(0.029, None).map(|m| m.common_name.as_str()).collect();
        assert!(fit29.contains(&"H128W") && fit29.contains(&"G80T") && fit29.contains(&"H133") && !fit29.contains(&"J350W"));
        assert!(db.fitting(0.0292, None).any(|m| m.common_name == "H128W"), "within 0.5 mm fits");
        assert!(!db.fitting(0.0296, None).any(|m| m.common_name == "H128W"));
        assert!(db.fitting(0.029, Some(0.2)).all(|m| m.length <= 0.2));
        assert!(db.fitting(0.029, Some(0.2)).any(|m| m.common_name == "H128W"));
        assert!(!db.fitting(0.029, Some(0.15)).any(|m| m.common_name == "H128W"));
        let h: Vec<_> = db.by_class('h').map(|m| m.common_name.as_str()).collect();
        assert!(h.contains(&"H128W") && h.contains(&"H238T"));
    }

    #[test]
    fn load_eng_file_roundtrip() {
        let path = std::env::temp_dir().join("bowshock_motors_test.eng");
        std::fs::write(&path, "T1 18 70 3 .002 .01 Test\n0.1 3\n0.5 3\n0.6 0\n").unwrap();
        let mut db = MotorDb::default();
        assert_eq!(db.load_eng_file(&path).unwrap(), 1);
        assert_eq!(db.find("t1-3").unwrap().manufacturer, "Test");
        std::fs::remove_file(&path).unwrap();
        assert!(matches!(db.load_eng_file(&path), Err(MotorError::Io(_))));
        assert!(db.add_eng_str("junk").is_err());
    }
}
