//! The binary motor file: the bundled catalogue (`data/thrustcurve.bin`) and each cached download are a [`MotorFile`].
//!
//! # Layout
//!
//! | bytes  | content |
//! |--------|---------|
//! | 0..8   | the magic `BSMOTORS` |
//! | 8..10  | the format version, `u16` little-endian ([`MOTOR_FILE_VERSION`]) |
//! | 10..   | the file body as CBOR (RFC 8949), then compressed as a raw deflate stream |
//!
//! The body is a map `{ s: source, a: attribution, d: snapshot date, m: [record, ...] }`. Each motor record holds its id,
//! designation, manufacturer and ThrustCurve metadata as text, and every number as packed little-endian `f32` in CBOR byte
//! strings: `g` = diameter m, length m, propellant kg, total kg; `y` = delays s; `t` / `f` = the curve's times s and thrusts
//! N (parallel arrays). No data-file text is stored, and loading parses no text: it inflates, walks the CBOR and copies the
//! byte strings into numbers. Values come back as the `f32` nearest the original (about 7 significant digits).
//!
//! The same container style as the `.bowshock` document. A file with another magic or version, or a damaged stream, is
//! refused with a [`StoreError`].

use std::fmt;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use flate2::read::DeflateDecoder;
use flate2::write::DeflateEncoder;
use flate2::Compression;
use serde::{Deserialize, Serialize};

use crate::motor::{common_name, Motor, ThrustCurveInfo, THRUSTCURVE_ID_PREFIX};

/// The first eight bytes of every motor file.
pub const MOTOR_FILE_MAGIC: &[u8; 8] = b"BSMOTORS";
/// The version [`MotorFile::to_bytes`] writes and [`MotorFile::from_bytes`] reads.
pub const MOTOR_FILE_VERSION: u16 = 1;
/// Extension of a cached download in the user motors directory: `<motorId>.motor`.
pub const CACHE_EXTENSION: &str = "motor";
/// The attribution every ThrustCurve-derived file carries.
pub const THRUSTCURVE_ATTRIBUTION: &str =
    "Motor data from ThrustCurve.org (https://www.thrustcurve.org), compiled by John Coker from certification and manufacturer data.";

const HEADER_LEN: usize = MOTOR_FILE_MAGIC.len() + 2;
/// The most a load will inflate (a guard against a decompression bomb; the full catalogue is a few MB).
const MAX_INFLATED: u64 = 256 << 20;

/// A set of motors with where they came from.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MotorFile {
    /// Where the data came from, e.g. `ThrustCurve.org`.
    pub source: String,
    /// Credit line to show with the data.
    pub attribution: String,
    /// When the data was taken from the source, `YYYY-MM-DD`.
    pub snapshot: String,
    /// The motors. [`Motor::comments`] are not stored; the file's attribution stands for them.
    pub motors: Vec<Motor>,
}

/// The stored body (see the module docs).
#[derive(Serialize, Deserialize)]
struct Body {
    #[serde(rename = "s")]
    source: String,
    #[serde(rename = "a")]
    attribution: String,
    #[serde(rename = "d")]
    snapshot: String,
    #[serde(rename = "m")]
    motors: Vec<Record>,
}

/// One stored motor.
#[derive(Serialize, Deserialize)]
struct Record {
    #[serde(rename = "i")]
    id: String,
    #[serde(rename = "n")]
    designation: String,
    #[serde(rename = "k")]
    manufacturer: String,
    /// Diameter m, length m, propellant mass kg, total mass kg.
    #[serde(rename = "g", with = "serde_bytes")]
    geometry: Vec<u8>,
    #[serde(rename = "y", with = "serde_bytes")]
    delays: Vec<u8>,
    #[serde(rename = "t", with = "serde_bytes")]
    time: Vec<u8>,
    #[serde(rename = "f", with = "serde_bytes")]
    thrust: Vec<u8>,
    #[serde(rename = "x", default, skip_serializing_if = "std::ops::Not::not")]
    approximated: bool,
    #[serde(rename = "c", default, skip_serializing_if = "Option::is_none")]
    thrustcurve: Option<TcRecord>,
}

/// [`ThrustCurveInfo`] as stored; the motorId is the record's id without its `tc:` prefix.
#[derive(Serialize, Deserialize)]
struct TcRecord {
    #[serde(rename = "s")]
    simfile_id: String,
    #[serde(rename = "m")]
    manufacturer_name: String,
    #[serde(rename = "n")]
    common_name: String,
    #[serde(rename = "t")]
    motor_type: String,
    #[serde(rename = "o")]
    cert_org: String,
    #[serde(rename = "a")]
    availability: String,
    #[serde(rename = "d")]
    data_source: String,
    #[serde(rename = "c")]
    case_info: String,
    #[serde(rename = "p")]
    propellant: String,
    #[serde(rename = "u")]
    updated_on: String,
    /// Listed total impulse N·s, average thrust N, burn time s.
    #[serde(rename = "l", with = "serde_bytes")]
    listed: Vec<u8>,
}

/// Packs numbers as little-endian `f32`.
fn pack(values: impl IntoIterator<Item = f64>) -> Vec<u8> {
    values.into_iter().flat_map(|v| (v as f32).to_le_bytes()).collect()
}

/// Unpacks little-endian `f32`s; `None` when the length is not a multiple of four.
fn unpack(bytes: &[u8]) -> Option<Vec<f64>> {
    let (words, rest) = bytes.as_chunks::<4>();
    rest.is_empty().then(|| words.iter().map(|&w| f64::from(f32::from_le_bytes(w))).collect())
}

impl Record {
    fn of(m: &Motor) -> Record {
        Record {
            id: m.id.clone(),
            designation: m.designation.clone(),
            manufacturer: m.manufacturer.clone(),
            geometry: pack([m.diameter, m.length, m.propellant_mass, m.total_mass]),
            delays: pack(m.delays.iter().copied()),
            time: pack(m.points.iter().map(|p| p.0)),
            thrust: pack(m.points.iter().map(|p| p.1)),
            approximated: m.approximated,
            thrustcurve: m.thrustcurve.as_ref().map(|i| TcRecord {
                simfile_id: i.simfile_id.clone(),
                manufacturer_name: i.manufacturer_name.clone(),
                common_name: i.common_name.clone(),
                motor_type: i.motor_type.clone(),
                cert_org: i.cert_org.clone(),
                availability: i.availability.clone(),
                data_source: i.data_source.clone(),
                case_info: i.case_info.clone(),
                propellant: i.propellant.clone(),
                updated_on: i.updated_on.clone(),
                listed: pack([i.listed_impulse, i.listed_average_thrust, i.listed_burn_time]),
            }),
        }
    }

    fn into_motor(self) -> Result<Motor, StoreError> {
        let bad = |what: &str| StoreError::Corrupt(format!("motor {}: bad {what}", self.id));
        let geometry = unpack(&self.geometry).filter(|g| g.len() == 4).ok_or_else(|| bad("geometry"))?;
        let delays = unpack(&self.delays).ok_or_else(|| bad("delays"))?;
        let time = unpack(&self.time).ok_or_else(|| bad("curve"))?;
        let thrust = unpack(&self.thrust).filter(|f| f.len() == time.len()).ok_or_else(|| bad("curve"))?;
        let thrustcurve = match self.thrustcurve {
            None => None,
            Some(r) => {
                let listed = unpack(&r.listed).filter(|l| l.len() == 3).ok_or_else(|| bad("listing"))?;
                Some(ThrustCurveInfo {
                    motor_id: self.id.strip_prefix(THRUSTCURVE_ID_PREFIX).unwrap_or_default().to_string(),
                    simfile_id: r.simfile_id,
                    manufacturer_name: r.manufacturer_name,
                    common_name: r.common_name,
                    motor_type: r.motor_type,
                    cert_org: r.cert_org,
                    availability: r.availability,
                    data_source: r.data_source,
                    case_info: r.case_info,
                    propellant: r.propellant,
                    listed_impulse: listed[0],
                    listed_average_thrust: listed[1],
                    listed_burn_time: listed[2],
                    updated_on: r.updated_on,
                })
            }
        };
        Ok(Motor {
            common_name: common_name(&self.designation),
            id: self.id,
            thrustcurve,
            designation: self.designation,
            manufacturer: self.manufacturer,
            diameter: geometry[0],
            length: geometry[1],
            delays,
            propellant_mass: geometry[2],
            total_mass: geometry[3],
            points: time.into_iter().zip(thrust).collect(),
            approximated: self.approximated,
            comments: Vec::new(),
        })
    }
}

/// Why a motor file could not be read or written.
#[derive(Debug)]
pub enum StoreError {
    /// Reading or writing the file failed.
    Io(std::io::Error),
    /// The bytes do not start with [`MOTOR_FILE_MAGIC`].
    NotMotorFile,
    /// A motor file of a version this build does not read.
    UnsupportedVersion(u16),
    /// The compressed stream or the CBOR inside it is damaged.
    Corrupt(String),
}

impl fmt::Display for StoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StoreError::Io(e) => write!(f, "motor file: {e}"),
            StoreError::NotMotorFile => write!(f, "not a Bowshock motor file"),
            StoreError::UnsupportedVersion(v) => write!(f, "motor file version {v} (this build reads {MOTOR_FILE_VERSION})"),
            StoreError::Corrupt(why) => write!(f, "damaged motor file: {why}"),
        }
    }
}

impl std::error::Error for StoreError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            StoreError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for StoreError {
    fn from(e: std::io::Error) -> Self {
        StoreError::Io(e)
    }
}

impl MotorFile {
    /// Encodes the file (see the module docs for the layout).
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(HEADER_LEN + 1024);
        out.extend_from_slice(MOTOR_FILE_MAGIC);
        out.extend_from_slice(&MOTOR_FILE_VERSION.to_le_bytes());
        let body = Body {
            source: self.source.clone(),
            attribution: self.attribution.clone(),
            snapshot: self.snapshot.clone(),
            motors: self.motors.iter().map(Record::of).collect(),
        };
        let mut encoder = DeflateEncoder::new(out, Compression::best());
        ciborium::into_writer(&body, &mut encoder).expect("writing CBOR into memory cannot fail");
        encoder.finish().expect("deflating into memory cannot fail")
    }

    /// Decodes a file written by [`to_bytes`](Self::to_bytes).
    pub fn from_bytes(bytes: &[u8]) -> Result<MotorFile, StoreError> {
        if bytes.len() < HEADER_LEN || &bytes[..MOTOR_FILE_MAGIC.len()] != MOTOR_FILE_MAGIC {
            return Err(StoreError::NotMotorFile);
        }
        let version = u16::from_le_bytes([bytes[8], bytes[9]]);
        if version != MOTOR_FILE_VERSION {
            return Err(StoreError::UnsupportedVersion(version));
        }
        let mut cbor = Vec::new();
        DeflateDecoder::new(&bytes[HEADER_LEN..]).take(MAX_INFLATED).read_to_end(&mut cbor).map_err(|e| StoreError::Corrupt(e.to_string()))?;
        let body: Body = ciborium::from_reader(cbor.as_slice()).map_err(|e| StoreError::Corrupt(e.to_string()))?;
        Ok(MotorFile {
            source: body.source,
            attribution: body.attribution,
            snapshot: body.snapshot,
            motors: body.motors.into_iter().map(Record::into_motor).collect::<Result<_, _>>()?,
        })
    }

    /// Reads a motor file from disk.
    pub fn read(path: impl AsRef<Path>) -> Result<MotorFile, StoreError> {
        MotorFile::from_bytes(&std::fs::read(path)?)
    }

    /// Writes the file to `path` through a temporary sibling, so a reader never sees half a file.
    pub fn write(&self, path: impl AsRef<Path>) -> Result<(), StoreError> {
        let path = path.as_ref();
        let tmp = path.with_extension("tmp");
        {
            let mut file = std::fs::File::create(&tmp)?;
            file.write_all(&self.to_bytes())?;
            file.sync_all()?;
        }
        std::fs::rename(&tmp, path)?;
        Ok(())
    }
}

/// Where a downloaded motor is cached in the user motors directory `dir`: `<dir>/<motorId>.motor`. `None` for a motor that
/// did not come from ThrustCurve.org (or whose id would not make a safe file name).
pub fn cache_path(dir: &Path, motor: &Motor) -> Option<PathBuf> {
    let id = motor.thrustcurve_id()?;
    (!id.is_empty() && id.chars().all(|c| c.is_ascii_alphanumeric())).then(|| dir.join(format!("{id}.{CACHE_EXTENSION}")))
}

/// Saves a downloaded motor (with its ThrustCurve metadata) into the user motors directory `dir`, creating it if needed.
/// Returns the file written.
pub fn save_cached(dir: &Path, motor: &Motor, snapshot: &str) -> Result<PathBuf, StoreError> {
    let path = cache_path(dir, motor).ok_or_else(|| StoreError::Corrupt(format!("motor {} has no ThrustCurve id to cache it by", motor.designation)))?;
    std::fs::create_dir_all(dir)?;
    let file =
        MotorFile { source: "ThrustCurve.org".into(), attribution: THRUSTCURVE_ATTRIBUTION.into(), snapshot: snapshot.into(), motors: vec![motor.clone()] };
    file.write(&path)?;
    Ok(path)
}

/// Every cached download in `dir` (files `*.motor`, sorted by name). Files that cannot be read are reported with their path.
pub fn load_cached(dir: &Path) -> (Vec<Motor>, Vec<(PathBuf, StoreError)>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return (Vec::new(), Vec::new());
    };
    let mut files: Vec<PathBuf> =
        entries.filter_map(Result::ok).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case(CACHE_EXTENSION))).collect();
    files.sort();
    let (mut motors, mut failed) = (Vec::new(), Vec::new());
    for path in files {
        match MotorFile::read(&path) {
            Ok(file) => motors.extend(file.motors),
            Err(e) => failed.push((path, e)),
        }
    }
    (motors, failed)
}

/// Today's date (UTC) as `YYYY-MM-DD`, for [`MotorFile::snapshot`].
pub fn today() -> String {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    civil_date(secs / 86_400)
}

/// The proleptic Gregorian date `days` after 1970-01-01 (H. Hinnant's `civil_from_days`).
fn civil_date(days: u64) -> String {
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}-{m:02}-{d:02}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::ThrustCurveInfo;

    fn downloaded() -> Motor {
        let mut m = crate::parse_eng("; provided by ThrustCurve.org\nH128W 29 194 14 0.09408 0.2016 AT\n0.024 102.4\n0.5 180\n1.29 0\n").unwrap().remove(0);
        m.id = "tc:5f4294d20002310000000083".into();
        m.thrustcurve = Some(ThrustCurveInfo {
            motor_id: "5f4294d20002310000000083".into(),
            availability: "regular".into(),
            cert_org: "Tripoli Rocketry Association, Inc.".into(),
            listed_impulse: 172.9,
            ..Default::default()
        });
        m
    }

    /// `m` as a load gives it back: numbers rounded to `f32`, comments dropped.
    fn stored(m: &Motor) -> Motor {
        let r = |v: f64| f64::from(v as f32);
        let mut s = m.clone();
        (s.diameter, s.length, s.propellant_mass, s.total_mass) = (r(m.diameter), r(m.length), r(m.propellant_mass), r(m.total_mass));
        s.delays = m.delays.iter().map(|&d| r(d)).collect();
        s.points = m.points.iter().map(|&(t, f)| (r(t), r(f))).collect();
        s.comments.clear();
        if let Some(i) = &mut s.thrustcurve {
            (i.listed_impulse, i.listed_average_thrust, i.listed_burn_time) = (r(i.listed_impulse), r(i.listed_average_thrust), r(i.listed_burn_time));
        }
        s
    }

    #[test]
    fn container_round_trips_and_refuses_damage() {
        let file = MotorFile { source: "ThrustCurve.org".into(), attribution: "x".into(), snapshot: "2026-11-15".into(), motors: vec![downloaded()] };
        let bytes = file.to_bytes();
        assert_eq!(&bytes[..8], MOTOR_FILE_MAGIC);
        let back = MotorFile::from_bytes(&bytes).unwrap();
        assert_eq!((back.source.as_str(), back.snapshot.as_str()), ("ThrustCurve.org", "2026-11-15"));
        assert_eq!(back.motors, vec![stored(&file.motors[0])]);
        assert!((back.motors[0].diameter - 0.029).abs() < 1e-9 && (back.motors[0].total_impulse() - file.motors[0].total_impulse()).abs() < 1e-4);
        // Stored numbers are packed f32, so a second round trip is exact.
        assert_eq!(MotorFile::from_bytes(&back.to_bytes()).unwrap(), back);
        // No data-file text is stored: the comments (with the attribution line) are not in the inflated body.
        let mut cbor = Vec::new();
        DeflateDecoder::new(&bytes[HEADER_LEN..]).read_to_end(&mut cbor).unwrap();
        let body = String::from_utf8_lossy(&cbor);
        assert!(!body.contains("provided by") && !body.contains("0.024") && !body.contains("102.4"));
        assert!(matches!(MotorFile::from_bytes(b"BOWSHOCK\x01\x00"), Err(StoreError::NotMotorFile)));
        let mut newer = bytes.clone();
        newer[8] = 9;
        assert!(matches!(MotorFile::from_bytes(&newer), Err(StoreError::UnsupportedVersion(9))));
        assert!(matches!(MotorFile::from_bytes(&bytes[..bytes.len() / 2]), Err(StoreError::Corrupt(_))));
    }

    #[test]
    fn cache_round_trip_keeps_metadata() {
        let dir = std::env::temp_dir().join(format!("bowshock-motor-cache-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let motor = downloaded();
        let path = save_cached(&dir, &motor, "2026-11-15").unwrap();
        assert_eq!(path, dir.join("5f4294d20002310000000083.motor"));
        // A damaged neighbour is reported, not fatal.
        std::fs::write(dir.join("broken.motor"), b"junk").unwrap();
        let (motors, failed) = load_cached(&dir);
        assert_eq!(motors, vec![stored(&motor)]);
        assert_eq!(motors[0].thrustcurve.as_ref().unwrap().cert_org, "Tripoli Rocketry Association, Inc.");
        assert_eq!(failed.len(), 1);
        // Saving again overwrites in place.
        save_cached(&dir, &motor, "2026-11-16").unwrap();
        assert_eq!(load_cached(&dir).0.len(), 1);
        // A file motor has no ThrustCurve id to cache by.
        assert!(save_cached(&dir, &crate::parse_eng("A1 18 70 3 .001 .01 E\n0.1 2\n").unwrap()[0], "").is_err());
        let _ = std::fs::remove_dir_all(&dir);
        assert!(load_cached(&dir).0.is_empty());
    }

    #[test]
    fn civil_dates() {
        assert_eq!(civil_date(0), "1970-01-01");
        assert_eq!(civil_date(11_016), "2000-02-29");
        assert_eq!(civil_date(20_772), "2026-11-15");
        assert_eq!(today().len(), 10);
    }
}
