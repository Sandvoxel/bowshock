//! Solver plumbing used by the validation crates; not part of the stable API, may change in any release.
#![allow(missing_docs)]
pub mod body_grid;
pub mod cache3;
pub mod case;
pub mod case3;
pub mod field3;
#[cfg(feature = "gpu")]
pub mod gpu;
#[cfg(feature = "gpu")]
pub mod gpu3;
pub mod grid3;
pub mod numerics;
pub mod result;
pub mod run;
pub mod run3;
