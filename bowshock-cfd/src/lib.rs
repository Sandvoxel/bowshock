//! `bowshock-cfd`: CPU reference solver for 2D axisymmetric compressible Euler flow.
//!
//! Every kernel (`flux::face_flux`, `flux::hllc`, `flux::reconstruct`, the per-face / per-cell loops in
//! `solver`) is a plain function of its inputs with no hidden state so that it can be
//! ported one-to-one to WGSL and validated against this crate.

pub mod bc;
pub mod body_grid;
pub mod cone;
pub mod flux;
pub mod gas;
pub mod grid;
pub mod ogive;
pub mod shock_expansion;
pub mod solver;
pub mod taylor_maccoll;
pub mod transonic;
