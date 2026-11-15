//! Rocket aerodynamics and flight simulation in the style of OpenRocket / RASAero.
//!
//! * [`atmosphere`]: ISA 1976 with site altitude and ground-temperature offset.
//! * [`aero`]: Barrowman centre of pressure / normal force and an OpenRocket-style drag build-up.
//! * [`flight`]: 3-DOF point-mass flight simulation (rail, thrust, staging, recovery).
//! * [`analysis`]: sweeps vs Mach and angle of attack, fin flutter, Monte Carlo dispersion.
//!
//! Everything is SI (metres, kilograms, seconds, radians, pascals). Axial stations are measured
//! from the nose tip, increasing aft. The crate has no dependencies and does not know about the
//! application's document model; the app maps its model into [`Vehicle`] and [`SimInput`].
//!
//! References: Barrowman, J. S. (1967) "The Practical Calculation of the Aerodynamic
//! Characteristics of Slender Finned Vehicles"; Niskanen, S. (2013) "OpenRocket technical
//! documentation"; NACA TN 4197 (Martin, 1958) for fin flutter.

pub mod aero;
pub mod analysis;
pub mod atmosphere;
pub mod flight;
pub mod rng;
pub mod vehicle;

pub use aero::{Aero, DragBreakdown, Stability};
pub use analysis::{fin_flutter, monte_carlo, vs_alpha, vs_mach, AlphaPoint, Dispersion, FlutterResult, MachPoint, MonteCarlo, MonteCarloRun, MonteCarloStats};
pub use atmosphere::{AirState, Atmosphere};
pub use flight::{simulate, Deploy, Event, EventKind, FlightResult, Launch, RecoveryDevice, Sample, SimInput, Summary, Trajectory, Turbulence};
pub use vehicle::{
    BodyKind, BodySegment, FinCrossSection, FinPlanform, FinSet, Ignition, MassProps, NoseShape, Protuberance, StageMass, StageMotor, ThrustCurve, Vehicle,
};

/// Standard gravity (m/s^2).
pub const G0: f64 = 9.806_65;
