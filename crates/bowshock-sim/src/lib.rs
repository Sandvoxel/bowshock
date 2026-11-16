//! Rocket aerodynamics (Barrowman build-up) and flight simulation (point mass or rigid body).
//!
//! * [`atmosphere`]: ISA 1976 with site altitude and ground-temperature offset.
//! * [`aero`]: Barrowman centre of pressure / normal force and a component drag build-up (Niskanen 2009, section 3.4).
//! * [`flight`]: flight simulation (rail, thrust, staging, recovery): a 3-DOF point mass or, by default, a rigid body with
//!   pitch/yaw/roll dynamics.
//! * [`analysis`]: sweeps vs Mach and angle of attack, Monte Carlo dispersion.
//! * [`flutter`]: fin flutter along a flight (Martin, NACA TN 4197, 1958, in the form of Bennett 2023).
//! * [`tube_fins`]: tube fin aerodynamics (open tubes on the outside of the body).
//!
//! Everything is SI (metres, kilograms, seconds, radians, pascals). Axial stations are measured
//! from the nose tip, increasing aft. The core modules do not know about the application's
//! document model; [`from_model`] is the one bridge that maps a `bowshock-model` rocket and a
//! `bowshock-motors` database into [`Vehicle`] and [`SimInput`].
//!
//! References: Barrowman, J. S. (1967) "The Practical Calculation of the Aerodynamic
//! Characteristics of Slender Finned Vehicles"; Niskanen, S. (2009) "Development of an Open Source
//! model rocket simulation software", Helsinki University of Technology; Martin, NACA TN 4197 (1958) and Bennett (2023) "Fin
//! Flutter Analysis Revisited (Again)" for fin flutter; Zipfel (2014) "Modeling and Simulation of Aerospace Vehicle Dynamics"
//! for the rigid-body equations.

pub mod aero;
pub mod analysis;
pub mod atmosphere;
mod attitude;
pub mod drag_table;
pub mod flight;
pub mod flutter;
pub mod from_model;
pub mod rng;
pub mod staging;
pub mod tube_fins;
pub mod vehicle;

pub use aero::{Aero, DragBreakdown, NormalForce, RollAero, Stability};
pub use analysis::{monte_carlo, vs_alpha, vs_mach, AlphaPoint, Dispersion, MachPoint, MonteCarlo, MonteCarloRun, MonteCarloStats};
pub use atmosphere::{AirState, Atmosphere};
pub use drag_table::{DragTable, DragTablePoint};
pub use flight::{simulate, Deploy, Event, EventKind, FlightModel, FlightResult, Launch, RecoveryDevice, Sample, SimInput, Summary, Trajectory, Turbulence};
pub use flutter::{flutter_velocity, FlutterFin, FlutterGeometry, FlutterInputs, FlutterPoint, FlutterTrace, FlutterWarning};
pub use staging::{has_altitude_trigger, staging_times, Timeline};
pub use vehicle::{
    BodyKind, BodySegment, FinCrossSection, FinPlanform, FinSet, MassProps, NoseShape, Protuberance, StageMass, StageMotor, ThrustCurve, Timing, Trigger,
    TubeFinSet, Vehicle,
};

/// Standard gravity (m/s^2).
pub const G0: f64 = 9.806_65;
