//! `bowshock-core`: zero-lift drag of axisymmetric rockets versus Mach number (0.1 to 3) from CFD: the axisymmetric Euler or
//! Reynolds-averaged Navier-Stokes equations (k-omega SST) on the GPU (Luma) or the CPU, plus empirical fin drag and a motor-on
//! base flow with the exhaust jet.
//!
//! Describe the body with [`BodySpec`], optionally add [`FinSpec`] fins and a [`MotorSpec`] motor, give the flight conditions
//! ([`FlightConditions`]) and run an [`Analysis`] over Mach numbers to get a [`Polar`]. The guide is at
//! <https://sandvoxel.github.io/bowshock/>.
//!
//! ```no_run
//! use bowshock_core::{AftSpec, Analysis, Atmosphere, BodySpec, FinProfile, FinSpec, FlightConditions, MotorSpec, NoseShape, NoseSpec};
//!
//! # fn main() -> Result<(), bowshock_core::Error> {
//! let body = BodySpec {
//!     nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
//!     aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
//! };
//! let fins = FinSpec { count: 4, root_chord: 0.12, tip_chord: 0.06, span: 0.08, sweep: 0.06, thickness: 0.003, roughness: 0.0, root_te_to_base: 0.0, profile: FinProfile::Rounded };
//! let motor = MotorSpec { exit_diameter: 0.05, exit_mach: 2.5, exit_pressure_pa: 101_325.0, total_temperature_k: 1500.0 };
//! let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };
//!
//! let polar = Analysis::new(&body)?.fins(fins).viscous(flight).motor(motor).run(&[0.5, 0.9, 1.5])?;
//! for p in &polar.points {
//!     println!("M {:.2}: CD {:.4} (base {:.4}), {:?}", p.mach, p.drag.total, p.drag.base, p.quality);
//! }
//! # Ok(())
//! # }
//! ```

#![warn(missing_docs)]

pub mod analysis;
pub mod atmosphere;
mod device;
#[doc(hidden)]
pub mod engine;
pub mod error;
pub mod field;
pub mod field3;
mod fins;
pub mod progress;
pub mod spec;

#[doc(hidden)]
pub use analysis::{take_facade_timings, warm_runs, Engine, FacadeTiming};
pub use analysis::{
    Analysis, AnalysisSpec, Backend, BackendUsed, Condition, Drag, FinDrag, Forces, PointDetail, Polar, PolarPoint, Quality, Resolution, Rollback, WallCp,
    Warning,
};
pub use atmosphere::{AirState, Atmosphere, FlightConditions};
#[cfg(feature = "gpu")]
#[doc(hidden)]
pub use bowshock_cfd::gpu::Precision;
pub use device::gpu_name;
pub use engine::meshing::Wake;
pub use engine::result::StopReason;
pub use error::Error;
pub use field::Field;
pub use field3::{Field3, Part, Symmetry, Tile, WallFace};
pub use progress::{Progress, ProgressSink};
#[doc(hidden)]
pub use spec::FinOutline;
pub use spec::{AftSpec, BodySpec, FinProfile, FinSpec, MotorSpec, NoseShape, NoseSpec, SharpShape};

/// Compiles and runs the code blocks of the user guide (`guide/src/*.md`) as doctests.
#[cfg(doctest)]
mod guide {
    #[doc = include_str!("../../../guide/src/introduction.md")]
    pub mod introduction {}
    #[doc = include_str!("../../../guide/src/getting-started.md")]
    pub mod getting_started {}
    #[doc = include_str!("../../../guide/src/body.md")]
    pub mod body {}
    #[doc = include_str!("../../../guide/src/fins.md")]
    pub mod fins {}
    #[doc = include_str!("../../../guide/src/flight.md")]
    pub mod flight {}
    #[doc = include_str!("../../../guide/src/sweeps.md")]
    pub mod sweeps {}
    #[doc = include_str!("../../../guide/src/motor.md")]
    pub mod motor {}
    #[doc = include_str!("../../../guide/src/results.md")]
    pub mod results {}
    #[doc = include_str!("../../../guide/src/validation.md")]
    pub mod validation {}
    #[doc = include_str!("../../../guide/src/performance.md")]
    pub mod performance {}
    #[doc = include_str!("../../../guide/src/faq.md")]
    pub mod faq {}
    #[doc = include_str!("../../../README.md")]
    pub mod readme {}
}
