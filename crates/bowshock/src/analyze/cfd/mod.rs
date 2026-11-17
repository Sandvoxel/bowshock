//! The `CFD table` method: the bridge from the document to `bowshock-core`'s solver, the solve itself (off the UI thread, one
//! Mach number at a time; the solver's progress callback gives an [`Event::Iteration`] about ten times a second and a set cancel
//! flag ends the point in flight within one report, see [`solve`]), the hashes that say whether a stored table still describes the
//! rocket, and the table's interpolation onto the Mach grid the plots use. What the screen makes of the events is in [`super::solve`]
//! (the state) and `solve_view` (the drawing).
//!
//! # What the solver gives
//!
//! `bowshock-core` solves the axisymmetric Reynolds-averaged Navier-Stokes equations (k-omega SST, wall-resolved) on the GPU
//! (Luma; Metal on a Mac) or the CPU for **one axisymmetric body at zero angle of attack** from Mach 0.1 to 3, and adds an
//! empirical drag for one set of 3 to 8 trapezoidal fins. It returns the zero-lift drag with its parts (nose and aft-body
//! pressure, skin friction, base, fins). It has **no angle of attack, no normal force and no centre of pressure**, and no
//! pods, lugs or rail buttons. So the table holds drag only; the Analyze screen takes margin, CP and `CNα` from the Extended
//! method and adds Extended's angle-of-attack change of the drag on top of the solved zero-lift drag.
//!
//! # What does not map
//!
//! The mapping ([`plan`]) says what it could not solve as the document has it, in [`Plan::notes`], and the screen prints
//! them under the provenance line: elliptical noses (solved as a half-power profile), freeform and elliptical fins (equivalent
//! trapezoids), fin sets beyond the first, lugs and rail buttons (their drag comes from the build-up method, in the table's
//! `other` column). Geometry the solver cannot take at all (pods, steps in diameter, no nose cone, a nose outside the solver's
//! range) is an `Err` with the reason.
//!
//! # References
//!
//! `bowshock-core` and its guide (`guide/src/performance.md` for timings); Menter (1994) for the k-omega SST model it solves.

mod fields;
mod fresh;
mod interp;
mod plan;
pub mod remote;
mod run;
mod share;
#[cfg(test)]
mod tests;

pub use fields::*;
pub use fresh::*;
pub use interp::*;
pub use plan::*;
pub use run::*;

/// The example rocket's plan, for the tests of the modules that play scripted solves.
#[cfg(test)]
pub(in crate::analyze) fn test_plan() -> Plan {
    let (rocket, vehicle) = tests::example();
    plan(&rocket, &vehicle, crate::units::UnitSystem::default()).expect("the example is an axisymmetric stack")
}
