//! Model-rocket motor thrust curves for Bowshock.
//!
//! * [`parse_eng`] reads RASP `.eng` files (the format used by ThrustCurve.org and most rocketry simulators).
//! * [`parse_rse`] reads RockSim `.rse` files (the format OpenRocket stores user-defined thrust curves in, inside an `.ork`).
//! * [`Motor`] is one motor with thrust / mass queries; everything is SI (m, kg, s, N). [`Motor::id`] identifies it: two
//!   curves with the same designation are two motors.
//! * [`MotorDb`] is a searchable collection. [`MotorDb::bundled`] is the motor catalogue compiled into the binary.
//! * [`thrustcurve`] talks to the ThrustCurve.org API (search, download; the HTTP client needs the `thrustcurve` feature).
//! * [`MotorFile`] is the binary container of the catalogue and of cached downloads.
//!
//! # Data source and attribution
//!
//! The bundled catalogue (`data/thrustcurve.bin`) is a snapshot of every motor with a data file on
//! [ThrustCurve.org](https://www.thrustcurve.org), the hobby-rocket motor database maintained by John Coker, built from
//! certification-test and manufacturer data. Each motor keeps its data file's attribution comments and its ThrustCurve
//! motorId; apps showing the data credit it with [`thrustcurve::CREDIT`]. The snapshot date is [`MotorDb::catalogue_info`].
//!
//! # Regenerating the catalogue
//!
//! ```text
//! cargo run --release -p bowshock-motors --features thrustcurve --bin tc_catalog -- crates/bowshock-motors/data/thrustcurve.bin
//! ```
//!
//! The generator lists every motor (one search per impulse class and per manufacturer, any availability), downloads the data
//! files in batches of 20 motorIds, keeps the best file of each motor (RASP certification data first), and writes the file.
//! It stays at one request per ~1.1 s, so a full run takes a few minutes.

mod db;
mod motor;
mod rasp;
mod rse;
mod store;
pub mod thrustcurve;

pub use db::{same_motor_base_name, same_motor_name, CatalogueInfo, MotorDb, MotorError};
pub use motor::{Motor, ThrustCurveInfo, FILE_ID_PREFIX, THRUSTCURVE_ID_PREFIX};
pub use rasp::{parse_eng, ParseError};
pub use rse::{parse_rse, RseError, PLUGGED_DELAY};
pub use store::{
    cache_path, load_cached, save_cached, today, MotorFile, StoreError, CACHE_EXTENSION, MOTOR_FILE_MAGIC, MOTOR_FILE_VERSION, THRUSTCURVE_ATTRIBUTION,
};
