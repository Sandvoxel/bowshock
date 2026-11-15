//! `bowshock-cfd`: axisymmetric compressible Euler / RANS (Menter SST k-omega) finite-volume solver.
//!
//! * The CPU solver (always built, f64, `rayon`): [`gas`], [`flux`] (limiters, MUSCL, HLLC), [`grid`], [`bc`], [`solver`] (SSP-RK3 and
//!   the line-implicit scheme of [`implicit`], FAS [`multigrid`]), [`viscous`] and [`turbulence`]. Every kernel is a plain function of
//!   its inputs with no hidden state, so it ports one-to-one to array operations.
//! * The GPU solver (cargo feature `gpu`, on by default): `bowshock_cfd::gpu`, the same solver written in plain
//!   [Luma](https://github.com/flipnoteman/Luma) array ops (f32 or double-float), validated against the CPU solver.

pub mod bc;
pub mod flux;
pub mod gas;
pub mod grid;
pub mod implicit;
pub mod multigrid;
pub mod solver;
pub mod turbulence;
pub mod viscous;

#[cfg(feature = "gpu")]
pub mod gpu;
