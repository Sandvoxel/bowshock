//! Solver plumbing used by the validation crates; not part of the stable API, may change in any release.
#![allow(missing_docs)]
pub mod body_grid;
pub mod case;
#[cfg(feature = "gpu")]
pub mod gpu;
pub mod numerics;
pub mod result;
pub mod run;
