//! The `CFD table` method: the bridge from the document to `bowshock-core`'s solver, the solve itself (off the UI thread, one
//! Mach number at a time; the solver's progress callback gives an [`Event::Iteration`] about ten times a second and a set cancel
//! flag ends the point in flight within one report, see [`solve`]), the hashes that say whether a stored table still describes the
//! rocket, and the table's interpolation onto the Mach grid the plots use. What the screen makes of the events is in [`super::solve`]
//! (the state) and `solve_view` (the drawing).
//!
//! # What a solve gives
//!
//! One solve, one job: the drag at zero incidence at each Mach node of the table and, unless the Conditions leave them out, the
//! normal force and centre of pressure at two angles of attack at the sparser stability nodes ([`lift_nodes`]). Both go through the
//! one `bowshock_core::Analysis::run` (a node of each kind has its own [`Analysis`](bowshock_core::Analysis): the drag nodes are the
//! viscous axisymmetric solve, the angle nodes the coarse 3D one), the result is one [`CfdTable`](bowshock_model::CfdTable) whose
//! `lift` part is the angles, and the screen has one action, one progress screen, one cancel and one staleness verdict for it. The
//! Analyze screen takes what the solve did not give (the margin, CP and `CNα` of a table without angles, the drag change with the
//! angle of attack) from the Extended method. **What each part is made of is the engine's business and changes with it; the app
//! says so in [`wording`] and nowhere else.**
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
mod wording;

pub use fields::*;
pub use fresh::*;
pub use interp::*;
pub use plan::*;
pub use run::*;
pub use wording::*;

/// The example rocket's plan, for the tests of the modules that play scripted solves.
#[cfg(test)]
pub(in crate::analyze) fn test_plan() -> Plan {
    let (rocket, vehicle) = tests::example();
    plan(&rocket, &vehicle, crate::units::UnitSystem::default()).expect("the example is an axisymmetric stack")
}
