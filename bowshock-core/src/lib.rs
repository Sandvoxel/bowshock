//! `bowshock-core`: CPU reference solver for 2D axisymmetric compressible Euler flow.
//!
//! Every kernel (`flux::face_flux`, `flux::hllc`, `flux::reconstruct`, the per-face / per-cell loops in
//! `solver`) is a plain function of its inputs with no hidden state so that it can be
//! ported one-to-one to WGSL and validated against this crate.

pub mod bc;
pub mod flux;
pub mod gas;
pub mod grid;
pub mod implicit;
pub mod multigrid;
pub mod solver;
