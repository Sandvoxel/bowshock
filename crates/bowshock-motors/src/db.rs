//! [`MotorDb`]: a searchable list of motors, with the bundled ThrustCurve.org catalogue.

use crate::motor::Motor;
use crate::rasp::{parse_eng, ParseError};
use crate::rse::{parse_rse, RseError};
use crate::store::MotorFile;
use std::fmt;
use std::path::Path;
use std::sync::LazyLock;

/// Motors whose outer diameter is within this of a mount's inner diameter are considered to fit, m.
const FIT_TOLERANCE: f64 = 0.0005;

/// The catalogue as compiled in: a [`MotorFile`] written by the `tc_catalog` generator.
static CATALOGUE_BYTES: &[u8] = include_bytes!("../data/thrustcurve.bin");

/// The catalogue, decoded on first use.
static CATALOGUE: LazyLock<MotorFile> =
    LazyLock::new(|| MotorFile::from_bytes(CATALOGUE_BYTES).unwrap_or_else(|e| panic!("the bundled motor catalogue is invalid: {e}")));

/// Where the bundled catalogue came from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CatalogueInfo {
    /// The data source, `ThrustCurve.org`.
    pub source: String,
    /// The credit line of the data.
    pub attribution: String,
    /// When the snapshot was taken, `YYYY-MM-DD`.
    pub snapshot: String,
    /// How many motors it holds.
    pub motors: usize,
}

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

/// Whether two motors share a name: the same designation and the same manufacturer, each ignoring case and surrounding space.
/// The rule [`MotorDb::resolve`] looks a saved reference up by, and the one a search result must meet to stand for a reference.
pub fn same_motor_name(designation: &str, manufacturer: &str, other_designation: &str, other_manufacturer: &str) -> bool {
    designation.trim().eq_ignore_ascii_case(other_designation.trim()) && manufacturer.trim().eq_ignore_ascii_case(other_manufacturer.trim())
}

/// Whether two motors share a name once a delay or option suffix is dropped from each designation (the text from the first
/// `-`): ThrustCurve.org lists the AeroTech `K455NW` as `K455NW-17`, and a design file names it without the delay.
pub fn same_motor_base_name(designation: &str, manufacturer: &str, other_designation: &str, other_manufacturer: &str) -> bool {
    let base = |d: &str| d.trim().split('-').next().unwrap_or_default().to_ascii_uppercase();
    base(designation) == base(other_designation) && manufacturer.trim().eq_ignore_ascii_case(other_manufacturer.trim())
}

/// A collection of motors.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MotorDb {
    /// All motors, in insertion order. Ids are unique when motors are added through [`upsert`](Self::upsert) or the loaders.
    pub motors: Vec<Motor>,
}

impl MotorDb {
    /// The bundled catalogue: every ThrustCurve.org motor with a data file at the snapshot date ([`catalogue_info`]).
    /// Decoded once per process (the first call pays for it); later calls copy the decoded list.
    ///
    /// [`catalogue_info`]: Self::catalogue_info
    ///
    /// # Panics
    /// Never in practice: the catalogue is checked by the crate's tests.
    pub fn bundled() -> Self {
        MotorDb { motors: CATALOGUE.motors.clone() }
    }

    /// Source, attribution and snapshot date of [`bundled`](Self::bundled).
    pub fn catalogue_info() -> CatalogueInfo {
        CatalogueInfo {
            source: CATALOGUE.source.clone(),
            attribution: CATALOGUE.attribution.clone(),
            snapshot: CATALOGUE.snapshot.clone(),
            motors: CATALOGUE.motors.len(),
        }
    }

    /// Adds `motor`, or replaces the motor with the same [`Motor::id`] in place (a user's or cached copy of a catalogue motor
    /// overrides it rather than listing it twice). Returns its index.
    pub fn upsert(&mut self, motor: Motor) -> usize {
        match self.position(&motor.id) {
            Some(i) => {
                self.motors[i] = motor;
                i
            }
            None => {
                self.motors.push(motor);
                self.motors.len() - 1
            }
        }
    }

    /// Index of the motor with id `id`.
    pub fn position(&self, id: &str) -> Option<usize> {
        self.motors.iter().position(|m| m.id == id)
    }

    /// The motor with id `id`.
    pub fn get(&self, id: &str) -> Option<&Motor> {
        self.motors.iter().find(|m| m.id == id)
    }

    /// Index of the motor a saved reference means: by `id` when it is set and known, else the first motor with this
    /// designation and manufacturer (both case-insensitive), else the first [`find`](Self::find) of the designation.
    pub fn resolve_position(&self, id: &str, designation: &str, manufacturer: &str) -> Option<usize> {
        let (designation, manufacturer) = (designation.trim(), manufacturer.trim());
        (!id.is_empty()).then(|| self.position(id)).flatten().or_else(|| {
            self.motors
                .iter()
                .position(|m| same_motor_name(&m.designation, &m.manufacturer, designation, manufacturer))
                .or_else(|| self.find_position(designation))
        })
    }

    /// The motor a saved reference means (see [`resolve_position`](Self::resolve_position)).
    pub fn resolve(&self, id: &str, designation: &str, manufacturer: &str) -> Option<&Motor> {
        self.resolve_position(id, designation, manufacturer).map(|i| &self.motors[i])
    }

    /// Parses `.eng` text and adds its motors ([`upsert`](Self::upsert): the same curve read twice is one motor); returns how
    /// many were parsed.
    pub fn add_eng_str(&mut self, src: &str) -> Result<usize, ParseError> {
        let parsed = parse_eng(src)?;
        let n = parsed.len();
        for motor in parsed {
            self.upsert(motor);
        }
        Ok(n)
    }

    /// Parses `.rse` (RockSim) text and adds its motors, like [`add_eng_str`](Self::add_eng_str); returns how many were parsed.
    pub fn add_rse_str(&mut self, src: &str) -> Result<usize, RseError> {
        let parsed = parse_rse(src)?;
        let n = parsed.len();
        for motor in parsed {
            self.upsert(motor);
        }
        Ok(n)
    }

    /// Reads an `.eng` file and adds its motors (see [`add_eng_str`](Self::add_eng_str)); returns how many were parsed.
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
    /// `H128W` and `h128w-14a` both find the AeroTech H128W. Several motors can share a designation; [`get`](Self::get) by
    /// id names one exactly.
    pub fn find(&self, designation: &str) -> Option<&Motor> {
        self.find_position(designation).map(|i| &self.motors[i])
    }

    /// Index of [`find`](Self::find)'s motor.
    pub fn find_position(&self, designation: &str) -> Option<usize> {
        self.motors
            .iter()
            .position(|m| m.designation.eq_ignore_ascii_case(designation.trim()))
            .or_else(|| self.motors.iter().position(|m| m.matches(designation)))
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

    /// (designation, maker, published total impulse N·s, diameter mm) from the ThrustCurve.org motor listings: the motors the
    /// app's examples and tests fly.
    const PUBLISHED: &[(&str, &str, f64, f64)] = &[
        ("A8", "Estes", 2.5, 18.0),
        ("B6", "Estes", 4.33, 18.0),
        ("C6", "Estes", 8.82, 18.0),
        ("D12", "Estes", 16.84, 24.0),
        ("E12", "Estes", 27.24, 24.0),
        ("E9", "Estes", 27.87, 24.0),
        ("F39T", "AeroTech", 49.66, 24.0),
        ("F67W", "AeroTech", 61.1, 29.0),
        ("G40W", "AeroTech", 97.14, 29.0),
        ("G80T", "AeroTech", 136.6, 29.0),
        ("H128W", "AeroTech", 172.9, 29.0),
        ("H238T", "AeroTech", 165.5, 29.0),
        ("I218R", "AeroTech", 319.63, 38.0),
        ("I284W", "AeroTech", 607.3, 38.0),
        ("J350W", "AeroTech", 700.0, 38.0),
        ("J420R", "AeroTech", 658.0, 38.0),
        ("H133", "Cesaroni", 163.3, 29.0),
        ("I216", "Cesaroni", 636.1, 38.0),
        ("J285", "Cesaroni", 648.0, 38.0),
    ];

    #[test]
    fn catalogue_decodes_and_is_well_formed() {
        let start = std::time::Instant::now();
        let file = MotorFile::from_bytes(CATALOGUE_BYTES).unwrap();
        eprintln!("catalogue: {} motors, {} bytes, decoded in {:?}", file.motors.len(), CATALOGUE_BYTES.len(), start.elapsed());
        assert_eq!(file.source, "ThrustCurve.org");
        assert!(file.attribution.contains("ThrustCurve.org"));
        assert_eq!(file.snapshot.len(), 10);
        assert!(file.motors.len() > 1000, "{}", file.motors.len());
        assert_eq!(MotorDb::catalogue_info().motors, file.motors.len());
        let mut ids = std::collections::HashSet::new();
        let mut near_listing = 0;
        for m in &file.motors {
            let name = format!("{} {} ({})", m.manufacturer, m.designation, m.id);
            assert!(ids.insert(m.id.clone()), "{name}: duplicate id");
            let info = m.thrustcurve.as_ref().unwrap_or_else(|| panic!("{name}: no metadata"));
            assert_eq!(m.thrustcurve_id(), Some(info.motor_id.as_str()), "{name}");
            assert!(!m.designation.is_empty() && !m.manufacturer.is_empty(), "{name}");
            assert!(m.points.len() >= 2, "{name}");
            assert!(m.points.windows(2).all(|w| w[1].0 >= w[0].0), "{name}: time not monotonic");
            assert!(m.points.iter().all(|p| p.0 >= 0.0 && p.1 >= 0.0 && p.0.is_finite() && p.1.is_finite()), "{name}");
            assert!(m.total_impulse() > 0.0 && m.burn_time() > 0.0, "{name}");
            assert!(m.diameter > 0.0 && m.length > 0.0 && m.total_mass >= m.propellant_mass, "{name}");
            if info.listed_impulse > 0.0 && (m.total_impulse() - info.listed_impulse).abs() / info.listed_impulse < 0.15 {
                near_listing += 1;
            }
        }
        // The integrated curves agree with ThrustCurve's listed total impulse (a handful of old or user-traced files do not).
        assert!(near_listing as f64 > 0.95 * file.motors.len() as f64, "{near_listing} of {}", file.motors.len());
    }

    #[test]
    fn bundled_motors_match_published_impulse() {
        let db = MotorDb::bundled();
        for &(name, maker, published, diameter_mm) in PUBLISHED {
            let m = db.motors.iter().find(|m| m.manufacturer == maker && m.matches(name)).unwrap_or_else(|| panic!("{maker} {name} not bundled"));
            let got = m.total_impulse();
            // The nominal listing values differ slightly from the integrated certification curves.
            assert!((got - published).abs() / published < 0.15, "{name}: {got} N·s vs published {published}");
            assert!((m.diameter * 1000.0 - diameter_mm).abs() < 1e-3, "{name}");
        }
    }

    #[test]
    fn h128w_is_about_176_ns() {
        let db = MotorDb::bundled();
        let h = db.find("H128W").unwrap();
        assert_eq!(h.id, "tc:5f4294d20002310000000083");
        assert!((h.total_impulse() - 176.0).abs() / 176.0 < 0.05, "{}", h.total_impulse());
        assert_eq!(h.impulse_class(), 'H');
        assert_eq!(h.manufacturer, "AeroTech");
        assert!((h.burn_time() - 1.293).abs() < 1e-6, "{}", h.burn_time());
        assert!((h.average_thrust() - h.total_impulse() / h.burn_time()).abs() < 1e-9);
        assert!(h.thrust(0.5) > 150.0);
        assert!(h.mass(0.0) > h.mass(5.0) && (h.mass(5.0) - (0.2016 - 0.09408)).abs() < 1e-6);
        let info = h.thrustcurve.as_ref().unwrap();
        assert_eq!((info.data_source.as_str(), info.availability.as_str()), ("cert", "regular"));
    }

    #[test]
    fn a_file_of_a_catalogue_curve_is_the_same_data() {
        // The H128W certification file as ThrustCurve.org serves it, read as a user's .eng file.
        let file = parse_eng(include_str!("../tests/fixtures/aerotech_h128w.eng")).unwrap().remove(0);
        let db = MotorDb::bundled();
        let catalogue = db.get("tc:5f4294d20002310000000083").unwrap();
        assert_ne!(file.id, catalogue.id);
        assert!(catalogue.same_data(&file), "the f32 catalogue curve and the text file hash alike");
    }

    #[test]
    fn find_is_case_insensitive_and_ignores_delay() {
        let db = MotorDb::bundled();
        assert_eq!(db.find("h128w-14a").unwrap().designation, "H128W");
        assert_eq!(db.find("H133-14A").unwrap().manufacturer, "Cesaroni");
        assert_eq!(db.find("B6-4").unwrap().common_name, "B6");
        assert_eq!(db.find("j350w").unwrap().common_name, "J350W");
        assert!(db.find("Z999").is_none());
    }

    /// A user's own curve of the AeroTech O5500X-PS beside the catalogue's: same designation and maker, different data.
    fn second_o5500() -> Motor {
        let db = MotorDb::bundled();
        let catalogue = db.find("O5500X-PS").expect("the catalogue has the O5500X-PS");
        let mut other = catalogue.clone();
        other.thrustcurve = None;
        other.points.iter_mut().for_each(|p| p.1 *= 0.9);
        other.id = other.file_id();
        other
    }

    #[test]
    fn same_designation_different_curves_are_two_motors() {
        let mut db = MotorDb::bundled();
        let n = db.len();
        let catalogue_id = db.find("O5500X-PS").unwrap().id.clone();
        let other = second_o5500();
        let i = db.upsert(other.clone());
        assert_eq!(db.len(), n + 1);
        assert_ne!(other.id, catalogue_id);
        // By id each reference finds its own curve; by name alone the first (the catalogue's) is found.
        assert_eq!(db.resolve(&other.id, "O5500X-PS", "AeroTech").unwrap().id, other.id);
        assert_eq!(db.resolve(&catalogue_id, "O5500X-PS", "AeroTech").unwrap().id, catalogue_id);
        assert_eq!(db.resolve("", "O5500X-PS", "AeroTech").unwrap().id, catalogue_id);
        assert_eq!(db.resolve("tc:gone", "O5500X-PS", "aerotech").unwrap().id, catalogue_id, "an unknown id falls back to the name");
        assert_eq!(db.resolve_position(&other.id, "", ""), Some(i));
        let (a, b) = (db.get(&catalogue_id).unwrap(), db.get(&other.id).unwrap());
        assert!((b.total_impulse() / a.total_impulse() - 0.9).abs() < 1e-9);
        // Adding the same curve again replaces it in place; so does a cached copy of a catalogue motor.
        assert_eq!(db.upsert(other), i);
        let mut cached = db.get(&catalogue_id).unwrap().clone();
        cached.thrustcurve.as_mut().unwrap().availability = "OOP".into();
        let at = db.position(&catalogue_id).unwrap();
        assert_eq!(db.upsert(cached), at);
        assert_eq!(db.len(), n + 1);
        assert!(db.motors[at].thrustcurve.as_ref().unwrap().out_of_production());
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
        // Every motor of the 29 mm list fits.
        assert!(db.fitting(0.029, None).all(|m| (m.diameter - 0.029).abs() <= FIT_TOLERANCE + 1e-9));
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
        // The same file read twice is one motor (its id is the curve's hash).
        db.add_eng_str("T1 18 70 3 .002 .01 Test\n0.1 3\n0.5 3\n0.6 0\n").unwrap();
        assert_eq!(db.len(), 1);
        assert!(db.motors[0].id.starts_with(crate::FILE_ID_PREFIX));
    }
}
