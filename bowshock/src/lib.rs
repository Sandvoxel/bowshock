//! `bowshock`: rocket aerodynamics on top of the bowshock axisymmetric flow solver: body geometry and grids, cases,
//! numerics and the CPU runner with its post-processing.

pub mod body_grid;
pub mod case;
pub mod error;
pub mod numerics;
pub mod result;
pub mod run;
