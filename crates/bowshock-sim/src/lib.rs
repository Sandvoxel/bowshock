//! Rocket aerodynamics (Barrowman build-up) and 3-DOF flight simulation.
//!
//! * [`atmosphere`]: ISA 1976 with site altitude and ground-temperature offset.
//! * [`aero`]: Barrowman centre of pressure / normal force and a component drag build-up (Niskanen 2009, section 3.4).
//! * [`flight`]: 3-DOF point-mass flight simulation (rail, thrust, staging, recovery).
//! * [`analysis`]: sweeps vs Mach and angle of attack, fin flutter, Monte Carlo dispersion.
//! * [`tube_fins`]: tube fin aerodynamics (open tubes on the outside of the body).
//!
//! Everything is SI (metres, kilograms, seconds, radians, pascals). Axial stations are measured
//! from the nose tip, increasing aft. The core modules do not know about the application's
//! document model; [`from_model`] is the one bridge that maps a `bowshock-model` rocket and a
//! `bowshock-motors` database into [`Vehicle`] and [`SimInput`].
//!
//! References: Barrowman, J. S. (1967) "The Practical Calculation of the Aerodynamic
//! Characteristics of Slender Finned Vehicles"; Niskanen, S. (2009) "Development of an Open Source
//! model rocket simulation software", Helsinki University of Technology; NACA TN 4197 (Martin, 1958) for fin flutter.

pub mod aero;
pub mod analysis;
pub mod atmosphere;
pub mod flight;
pub mod from_model;
pub mod rng;
pub mod staging;
pub mod tube_fins;
pub mod vehicle;

pub use aero::{Aero, DragBreakdown, Stability};
pub use analysis::{fin_flutter, monte_carlo, vs_alpha, vs_mach, AlphaPoint, Dispersion, FlutterResult, MachPoint, MonteCarlo, MonteCarloRun, MonteCarloStats};
pub use atmosphere::{AirState, Atmosphere};
pub use flight::{simulate, Deploy, Event, EventKind, FlightResult, Launch, RecoveryDevice, Sample, SimInput, Summary, Trajectory, Turbulence};
pub use staging::{has_altitude_trigger, staging_times, Timeline};
pub use vehicle::{
    BodyKind, BodySegment, FinCrossSection, FinPlanform, FinSet, MassProps, NoseShape, Protuberance, StageMass, StageMotor, ThrustCurve, Timing, Trigger,
    TubeFinSet, Vehicle,
};

/// Standard gravity (m/s^2).
pub const G0: f64 = 9.806_65;
