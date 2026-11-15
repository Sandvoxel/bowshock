//! Model-rocket motor thrust curves for Bowshock.
//!
//! * [`parse_eng`] reads RASP `.eng` files (the format used by ThrustCurve.org, OpenRocket, RockSim, ...).
//! * [`Motor`] is one parsed motor with thrust / mass queries; everything is SI (m, kg, s, N).
//! * [`MotorDb`] is a searchable collection, including a set of common motors bundled into the binary
//!   ([`MotorDb::bundled`]); the data files in `data/` retain their original attribution comments.

mod db;
mod motor;
mod rasp;

pub use db::{MotorDb, MotorError};
pub use motor::Motor;
pub use rasp::{parse_eng, ParseError};
