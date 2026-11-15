//! Rocket aerodynamics and flight simulation in the style of OpenRocket / RASAero.
//!
//! * [`atmosphere`]: ISA 1976 with site altitude and ground-temperature offset.
//! * [`aero`]: Barrowman centre of pressure / normal force and an OpenRocket-style drag build-up.
//! * [`flight`]: 3-DOF point-mass flight simulation (rail, thrust, staging, recovery).
//! * [`analysis`]: sweeps vs Mach and angle of attack, fin flutter, Monte Carlo dispersion.
//!
//! Everything is SI (metres, kilograms, seconds, radians, pascals). Axial stations are measured
//! from the nose tip, increasing aft. The core modules do not know about the application's
//! document model; [`from_model`] is the one bridge that maps a `bowshock-model` rocket and a
//! `bowshock-motors` database into [`Vehicle`] and [`SimInput`].
//!
//! References: Barrowman, J. S. (1967) "The Practical Calculation of the Aerodynamic
//! Characteristics of Slender Finned Vehicles"; Niskanen, S. (2013) "OpenRocket technical
//! documentation"; NACA TN 4197 (Martin, 1958) for fin flutter.

pub mod aero;
pub mod analysis;
pub mod atmosphere;
pub mod flight;
pub mod from_model;
pub mod rng;
pub mod staging;
pub mod vehicle;

pub use aero::{Aero, DragBreakdown, Stability};
pub use analysis::{fin_flutter, monte_carlo, vs_alpha, vs_mach, AlphaPoint, Dispersion, FlutterResult, MachPoint, MonteCarlo, MonteCarloRun, MonteCarloStats};
pub use atmosphere::{AirState, Atmosphere};
pub use flight::{simulate, Deploy, Event, EventKind, FlightResult, Launch, RecoveryDevice, Sample, SimInput, Summary, Trajectory, Turbulence};
pub use staging::{has_altitude_trigger, staging_times, Timeline};
pub use vehicle::{
    BodyKind, BodySegment, FinCrossSection, FinPlanform, FinSet, MassProps, NoseShape, Protuberance, StageMass, StageMotor, ThrustCurve, Timing, Trigger,
    Vehicle,
};

/// Standard gravity (m/s^2).
pub const G0: f64 = 9.806_65;
