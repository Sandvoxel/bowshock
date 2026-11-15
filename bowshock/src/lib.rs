//! `bowshock`: zero-lift drag of axisymmetric rocket bodies versus Mach number. Describe the body with [`BodySpec`], configure
//! an [`Analysis`] (resolution, backend, viscous flight conditions) and run it to get a [`Polar`].

#![warn(missing_docs)]

pub mod analysis;
pub mod atmosphere;
#[doc(hidden)]
pub mod engine;
pub mod error;
pub mod spec;

pub use analysis::{Analysis, Backend, BackendUsed, Drag, PointDetail, Polar, PolarPoint, Quality, Resolution, WallCp, Warning};
pub use atmosphere::{AirState, Atmosphere, FlightConditions};
pub use engine::case::Wake;
pub use engine::result::StopReason;
pub use error::Error;
pub use spec::{AftSpec, BodySpec, NoseShape, NoseSpec};
