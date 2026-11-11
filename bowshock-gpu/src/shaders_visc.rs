//! WGSL of the viscous terms and the Menter SST k-omega model on the GPU, and the host-side data they need: a kernel-for-kernel
//! translation of `bowshock_core::viscous` and `bowshock_core::turbulence` (read their module documentation for the formulas) for
//! the line-implicit scheme in the guarded formulation (`Mode::Guarded` + `Scheme::Implicit`).
//!
//! The mean flow keeps its double-float residual of [`crate::Precision::Extended`]; everything in this module is plain f32 on the
//! high part of the state (the viscous fluxes, the gradients, the turbulence equations and the left-hand side only steer or
//! perturb the iteration at the level of f32 round-off). Per iteration, in the order of the launches
//! ([`crate::GpuSolver`] `record_implicit`):
//!
//! | kernel | launch | mirrors | reads / writes |
//! |---|---|---|---|
//! | `VGRAD` | `[ni, nj]` | `viscous::cell_pass` | `u` -> `grad` (`u_x, u_r, v_x, v_r` / `T_x, T_r`), `mu` (`turb.w`) |
//! | `VFLUX` | `[ni + 1, 2 nj + 1]` | `viscous::face_pass` | `u`, `grad`, `turb` -> `fvi`, `fvj` |
//! | residual | `[ni, nj]` | `Solver::eval_residual` (3.) | `shaders_impl::residual_store(viscous)`: `F - Fv`, `t_tt`, diffusive `dt` |
//! | assemble | `[nj, ni]`, `[ni, nj]` | `implicit::assemble` | `shaders_impl::assemble_i/j(viscous)`: no-slip wall ghost and Jacobian, `2 lam I` |
//! | `TURB_GRAD` | `[ni, nj]` | `turbulence::scalar_gradients` + `blends` | `u`, `turb` -> `tgb` (`k_x, k_r, omega_x, omega_r` / `F1, X`) |
//! | `TURB_COEF` | `[ni, nj]` | `turbulence::face_pass` + `cell_terms` | `fi`, `fj`, `tgb`, ... -> `tres` and the diagonal blocks of the two line systems |
//! | line solves | `[nj]`, `[ni]` | `solve_lines` | `shaders_impl::line_i/line_j` on the turbulence blocks (4x4 blocks that are diagonal) |
//! | `TURB_UPDATE` | `[ni, nj]` | `turbulence::update` (clipped update, `mu_t`) | `tdu`, `u`, `grad` -> `turb` |
//!
//! The turbulence kernels run before the mean-flow update of the iteration: like the CPU routine (which works from `Solver::w`, the
//! primitives of the start of the iteration) they see the state `u` of the start of the iteration. The CPU's two per-cell passes of
//! `turbulence.rs` (scalar gradients, blending functions) are one launch here (`TURB_GRAD`), its two per-face / per-cell passes
//! (face data, cell terms) another (`TURB_COEF`, which evaluates the data of the four faces of a cell from the stored gradients).
//!
//! Storage (`Array<[f32; 4]>`): the per-face geometry `(e_x, e_r, 1/|d|, r_m)` of the compact face gradient
//! (`vgi`, `vgj`, computed in f64 from the grid with the ghost-mirror rule of `viscous::stencil`), the per-cell `(1/A, r_c, d_wall, 0)`
//! (`vcell`), the state `turb = (k, omega, mu_t, mu)` of every cell, `grad` (two vec4 per cell) and the viscous face fluxes `fvi`, `fvj`.

use crate::{Result, Viscous};
use bowshock_core::grid::Grid;
use bowshock_core::solver::{Block, Config, Turbulence, PRANDTL, PRANDTL_T};
use bowshock_core::turbulence as sst;
use luma::{Array, Context, Kernel};

/// WGSL constants of the viscous / turbulence kernels, taken from `bowshock-core`.
pub(crate) fn consts() -> String {
    let c = |name: &str, x: f64| format!("const {name}: f32 = {x:e};\n");
    [
        c("PRANDTL", PRANDTL),
        c("PRANDTL_T", PRANDTL_T),
        c("A1", sst::A1),
        c("BETA_STAR", sst::BETA_STAR),
        c("SIGMA_K1", sst::SIGMA_K1),
        c("SIGMA_K2", sst::SIGMA_K2),
        c("SIGMA_W1", sst::SIGMA_W1),
        c("SIGMA_W2", sst::SIGMA_W2),
        c("BETA1", sst::BETA1),
        c("BETA2", sst::BETA2),
        c("GAMMA1", sst::GAMMA1),
        c("GAMMA2", sst::GAMMA2),
    ]
    .concat()
}

/// Prelude of the kernels that need the 4x4 block library: `COMMON` + `IMPL_LIB` + the constants.
fn with_blocks(src: &str) -> String {
    format!("{}\n{}\n{}", crate::shaders_impl::common_impl(), consts(), src)
}

/// Prelude of the other kernels: `COMMON` + the constants.
fn with_common(src: &str) -> String {
    format!("{}\n{}\n{}", crate::shaders::common(), consts(), src)
}

/// Faces and their two sides (`viscous::iface_sides`, `viscous::jface_sides`), `mu(T)`. Needs the bindings `u`, `iface`, `jface`.
///
/// A side is the primitive state of a cell, or of a ghost: the BC ghost across a domain boundary, the reflected / reversed state
/// across a wall of the solid block (the block ghost of the j direction, the base wall of the i direction); `cell` is the fluid cell the
/// side belongs to (for a ghost the adjacent one) and `valid` is false for a face inside the solid block.
const SIDES_LIB: &str = r#"
struct Side { w: vec4<f32>, cell: u32, ghost: bool }
struct Sides { l: Side, r: Side, noslip: bool, valid: bool }

fn fs_state() -> vec4<f32> { return vec4<f32>(p.fs_rho, p.fs_u, p.fs_v, p.fs_p); }
fn cell_prim(i: u32, j: u32) -> vec4<f32> { return cons_to_prim(u[j * p.ni + i]); }
fn temperature(w: vec4<f32>) -> f32 { return GAMMA * w.w / w.x; }
// Sutherland viscosity (`Viscous::mu`), free-stream temperature 1, `p.mu_inf = 1 / re_c`, `p.suth` the Sutherland constant.
fn visc_mu(t: f32) -> f32 { return t * sqrt(t) * (1.0 + p.suth) / (t + p.suth) * p.mu_inf; }

fn iface_sides(i: u32, j: u32) -> Sides {
    let ni = p.ni;
    var s: Sides;
    s.valid = true;
    if (p.blk_j > 0u && j < p.blk_j) {
        if (j < p.blk_s || i < p.blk_i) { s.valid = false; return s; }
        if (i == p.blk_i) {
            // base wall: the left side is the wall ghost of the cell behind it
            let c = j * ni + i;
            let f = iface[j * (ni + 1u) + i];
            let wr = cell_prim(i, j);
            s.noslip = noslip_iface(i, j);
            s.l = Side(wall_ghost(wr, f.x, f.y, s.noslip), c, true);
            s.r = Side(wr, c, false);
            return s;
        }
    }
    if (i == 0u) {
        let c = j * ni;
        let f = iface[j * (ni + 1u)];
        let e = cell_prim(0u, j);
        s.l = Side(ghost_state(p.bc0, fs_state(), e, e, f, -1.0), c, true);
        s.r = Side(e, c, false);
    } else if (i == ni) {
        let c = j * ni + ni - 1u;
        let f = iface[j * (ni + 1u) + ni];
        let e = cell_prim(ni - 1u, j);
        s.l = Side(e, c, false);
        s.r = Side(ghost_state(p.bc1, fs_state(), e, e, f, 1.0), c, true);
    } else {
        s.l = Side(cell_prim(i - 1u, j), j * ni + i - 1u, false);
        s.r = Side(cell_prim(i, j), j * ni + i, false);
    }
    return s;
}

fn jface_sides(i: u32, j: u32) -> Sides {
    let ni = p.ni;
    let nj = p.nj;
    var s: Sides;
    s.valid = true;
    if (p.blk_j > 0u) {
        let j0 = col_start(i);
        if (j < j0) { s.valid = false; return s; }
        if (j == j0 && j0 > 0u) {
            // body or sting surface: the solid cell below holds the block ghost
            let c = j * ni + i;
            let f = jface[c];
            let wr = cell_prim(i, j);
            s.noslip = noslip_jface(i, j);
            s.l = Side(wall_ghost(wr, f.x, f.y, s.noslip), c, true);
            s.r = Side(wr, c, false);
            return s;
        }
    }
    if (j == 0u) {
        let f = jface[i];
        let e = cell_prim(i, 0u);
        s.noslip = noslip_jface(i, 0u);
        var g = ghost_state(p.bc2, fs_state(), e, e, f, -1.0);
        if (s.noslip) { g = wall_ghost(e, f.x, f.y, true); }
        s.l = Side(g, i, true);
        s.r = Side(e, i, false);
    } else if (j == nj) {
        let c = (nj - 1u) * ni + i;
        let f = jface[nj * ni + i];
        let e = cell_prim(i, nj - 1u);
        s.l = Side(e, c, false);
        s.r = Side(ghost_state(p.bc3, fs_state(), e, e, f, 1.0), c, true);
    } else {
        s.l = Side(cell_prim(i, j - 1u), (j - 1u) * ni + i, false);
        s.r = Side(cell_prim(i, j), j * ni + i, false);
    }
    return s;
}
"#;

/// Cell viscosity and Green-Gauss gradient (`viscous::cell_pass`). Problem size `[ni, nj]`. `vcell` = `(1/A, r_c, d_wall, 0)`; `grad` holds two
/// vec4 per cell: `(u_x, u_r, v_x, v_r)` and `(T_x, T_r, 0, 0)`; `turb.w` receives `mu(T)`. Solid cells get zeros.
const VGRAD: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> vcell: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read_write> grad: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read_write> turb: array<vec4<f32>>;
@group(1) @binding(6) var<uniform> p: Params;

//@SIDES

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    let i = gid.x;
    let j = gid.y;
    let c = j * p.ni + i;
    if (is_solid(i, j)) {
        grad[2u * c] = vec4<f32>(0.0);
        grad[2u * c + 1u] = vec4<f32>(0.0);
        turb[c].w = 0.0;
        return;
    }
    let wc = cell_prim(i, j);
    turb[c].w = visc_mu(temperature(wc));
    let pc = vec3<f32>(wc.y, wc.z, temperature(wc));
    let fl = iface[j * (p.ni + 1u) + i];
    let fr = iface[j * (p.ni + 1u) + i + 1u];
    let fb = jface[c];
    let ft = jface[c + p.ni];
    let nl = iface_sides(i, j).l.w;
    let nr = iface_sides(i + 1u, j).r.w;
    let nb = jface_sides(i, j).l.w;
    let nt = jface_sides(i, j + 1u).r.w;
    // phi_f = (phi_c + phi_neighbour) / 2, the faces in the order right, left, top, bottom; components (u, v, T)
    var gx = vec3<f32>(0.0);
    var gr = vec3<f32>(0.0);
    var pf = 0.5 * (pc + vec3<f32>(nr.y, nr.z, temperature(nr)));
    gx += 1.0 * pf * fr.x * fr.z;
    gr += 1.0 * pf * fr.y * fr.z;
    pf = 0.5 * (pc + vec3<f32>(nl.y, nl.z, temperature(nl)));
    gx += -1.0 * pf * fl.x * fl.z;
    gr += -1.0 * pf * fl.y * fl.z;
    pf = 0.5 * (pc + vec3<f32>(nt.y, nt.z, temperature(nt)));
    gx += 1.0 * pf * ft.x * ft.z;
    gr += 1.0 * pf * ft.y * ft.z;
    pf = 0.5 * (pc + vec3<f32>(nb.y, nb.z, temperature(nb)));
    gx += -1.0 * pf * fb.x * fb.z;
    gr += -1.0 * pf * fb.y * fb.z;
    let inv_a = vcell[c].x;
    gx = gx * inv_a;
    gr = gr * inv_a;
    grad[2u * c] = vec4<f32>(gx.x, gr.x, gx.y, gr.y);
    grad[2u * c + 1u] = vec4<f32>(gx.z, gr.z, 0.0, 0.0);
}
"#;

/// Viscous face fluxes `Fv.n` of every i-face (`fvi`, `ni + 1` per row) and j-face (`fvj`) (`viscous::face_pass`, `viscous_flux`): the face gradient
/// `g_f = gbar + ((phi_R - phi_L) / |d| - gbar.e) e`, face viscosities (a ghost side with the sign of `mu_t` flipped across a no-slip face), the
/// stress tensor and the heat flux. Problem size `[ni + 1, 2 nj + 1]` (the layout of the flux kernel): row `y < nj` is i-face row `y`, row
/// `y >= nj` is j-face row `y - nj`. `vgi`, `vgj` = `(e_x, e_r, 1/|d|, r_m)` per face.
const VFLUX: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> vgi: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> vgj: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read> grad: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read> turb: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read_write> fvi: array<vec4<f32>>;
@group(1) @binding(8) var<storage, read_write> fvj: array<vec4<f32>>;
@group(1) @binding(9) var<uniform> p: Params;

//@SIDES

// Face viscosity and eddy viscosity (`viscous::face_mu`).
fn face_mu(sd: Sides) -> vec2<f32> {
    var ml = turb[sd.l.cell].z;
    var mr = turb[sd.r.cell].z;
    if (sd.noslip && sd.l.ghost) { ml = -ml; }
    if (sd.noslip && sd.r.ghost) { mr = -mr; }
    return vec2<f32>(0.5 * (turb[sd.l.cell].w + turb[sd.r.cell].w), 0.5 * (ml + mr));
}

fn visc_flux(sd: Sides, f: vec4<f32>, vg: vec4<f32>) -> vec4<f32> {
    if (!sd.valid || vg.w == 0.0) { return vec4<f32>(0.0); }
    let ex = vg.x;
    let er = vg.y;
    let gl0 = grad[2u * sd.l.cell];
    let gl1 = grad[2u * sd.l.cell + 1u];
    let gr0 = grad[2u * sd.r.cell];
    let gr1 = grad[2u * sd.r.cell + 1u];
    let pl = vec3<f32>(sd.l.w.y, sd.l.w.z, temperature(sd.l.w));
    let pr = vec3<f32>(sd.r.w.y, sd.r.w.z, temperature(sd.r.w));
    // components (u, v, T): x and r derivatives of the two cells, their mean and the corrected face gradient
    let gbx = 0.5 * (vec3<f32>(gl0.x, gl0.z, gl1.x) + vec3<f32>(gr0.x, gr0.z, gr1.x));
    let gbr = 0.5 * (vec3<f32>(gl0.y, gl0.w, gl1.y) + vec3<f32>(gr0.y, gr0.w, gr1.y));
    let corr = (pr - pl) * vg.z - (gbx * ex + gbr * er);
    let gfx = gbx + corr * ex;
    let gfr = gbr + corr * er;
    let mm = face_mu(sd);
    let uf = 0.5 * (sd.l.w.y + sd.r.w.y);
    let vf = 0.5 * (sd.l.w.z + sd.r.w.z);
    let dv = gfx.x + gfr.y + vf / vg.w;
    let mue = mm.x + mm.y;
    let txx = mue * (2.0 * gfx.x - 2.0 / 3.0 * dv);
    let trr = mue * (2.0 * gfr.y - 2.0 / 3.0 * dv);
    let txr = mue * (gfr.x + gfx.y);
    let kappa = (mm.x / PRANDTL + mm.y / PRANDTL_T) / (GAMMA - 1.0);
    return vec4<f32>(
        0.0,
        txx * f.x + txr * f.y,
        txr * f.x + trr * f.y,
        (uf * txx + vf * txr + kappa * gfx.z) * f.x + (uf * txr + vf * trr + kappa * gfr.z) * f.y);
}

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.y < p.nj) {
        if (gid.x > p.ni) { return; }
        let k = gid.y * (p.ni + 1u) + gid.x;
        fvi[k] = visc_flux(iface_sides(gid.x, gid.y), iface[k], vgi[k]);
    } else {
        let jr = gid.y - p.nj;
        if (gid.x >= p.ni || jr > p.nj) { return; }
        let k = jr * p.ni + gid.x;
        fvj[k] = visc_flux(jface_sides(gid.x, jr), jface[k], vgj[k]);
    }
}
"#;

/// Scalar states `(k, omega)` of the two sides of a face (`turbulence::Ctx::kw`): the cell values, the free-stream values across a far-field
/// boundary (`Bc::Freestream | Characteristic | PressureOutflow`), the wall values `k = -k_c`, `omega = 2 omega_w - omega_c`,
/// `omega_w = 60 nu_c / (beta1 d_c^2)` across a no-slip wall. Needs `u`, `turb` (`mu` in `.w`), `vcell` and `SIDES_LIB`.
const KW_LIB: &str = r#"
const KIND_CELL: u32 = 0u;
const KIND_INF: u32 = 1u;
const KIND_WALL: u32 = 2u;

// Origin of the ghost state of a boundary condition code (0 free stream, 1 zero gradient, 2 slip wall, 3 characteristic, 4 pressure outflow).
fn bc_kind(bc: u32) -> u32 {
    if (bc == 1u || bc == 2u) { return KIND_CELL; }
    return KIND_INF;
}

fn kw_of(s: Side, kind: u32) -> vec2<f32> {
    let t = turb[s.cell];
    if (!s.ghost || kind == KIND_CELL) { return t.xy; }
    if (kind == KIND_INF) { return vec2<f32>(p.k_inf, p.om_inf); }
    let nu = t.w / u[s.cell].x;
    let d = vcell[s.cell].z;
    return vec2<f32>(-t.x, 2.0 * (60.0 * nu / (BETA1 * d * d)) - t.y);
}

struct FaceKW { sd: Sides, kl: vec2<f32>, kr: vec2<f32> }

fn iface_kw(i: u32, j: u32) -> FaceKW {
    var o: FaceKW;
    o.sd = iface_sides(i, j);
    if (!o.sd.valid) { return o; }
    var kind = KIND_CELL;
    if (i == 0u) { kind = bc_kind(p.bc0); }
    else if (i == p.ni) { kind = bc_kind(p.bc1); }
    else if (o.sd.l.ghost && o.sd.noslip) { kind = KIND_WALL; }
    o.kl = kw_of(o.sd.l, kind);
    o.kr = kw_of(o.sd.r, kind);
    return o;
}

fn jface_kw(i: u32, j: u32) -> FaceKW {
    var o: FaceKW;
    o.sd = jface_sides(i, j);
    if (!o.sd.valid) { return o; }
    var kind = KIND_CELL;
    if (o.sd.l.ghost) {
        if (o.sd.noslip) { kind = KIND_WALL; }
        else if (j == 0u) { kind = bc_kind(p.bc2); }
    } else if (o.sd.r.ghost) {
        kind = bc_kind(p.bc3);
    }
    o.kl = kw_of(o.sd.l, kind);
    o.kr = kw_of(o.sd.r, kind);
    return o;
}
"#;

/// Scalar gradients `(k_x, k_r, omega_x, omega_r)` (Green-Gauss, `turbulence::scalar_gradients`), `F1` and the cross-diffusion `X` (`turbulence::blends`)
/// of every fluid cell: `tgb` holds `(k_x, k_r, omega_x, omega_r)` and `(F1, X, 0, 0)` per cell. Problem size `[ni, nj]`.
const TURB_GRAD: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> vcell: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> turb: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read_write> tgb: array<vec4<f32>>;
@group(1) @binding(6) var<uniform> p: Params;

//@SIDES
//@KW

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    let i = gid.x;
    let j = gid.y;
    let c = j * p.ni + i;
    if (is_solid(i, j)) {
        tgb[2u * c] = vec4<f32>(0.0);
        tgb[2u * c + 1u] = vec4<f32>(0.0);
        return;
    }
    let t = turb[c];
    let pc = t.xy;
    let fl = iface[j * (p.ni + 1u) + i];
    let fr = iface[j * (p.ni + 1u) + i + 1u];
    let fb = jface[c];
    let ft = jface[c + p.ni];
    let nr = iface_kw(i + 1u, j).kr;
    let nl = iface_kw(i, j).kl;
    let nt = jface_kw(i, j + 1u).kr;
    let nb = jface_kw(i, j).kl;
    // phi_f = (phi_c + phi_neighbour) / 2 per scalar, the faces in the order right, left, top, bottom
    var g = vec4<f32>(0.0);
    var pf = 0.5 * (pc + nr);
    g += 1.0 * vec4<f32>(pf.x * fr.x, pf.x * fr.y, pf.y * fr.x, pf.y * fr.y) * fr.z;
    pf = 0.5 * (pc + nl);
    g += -1.0 * vec4<f32>(pf.x * fl.x, pf.x * fl.y, pf.y * fl.x, pf.y * fl.y) * fl.z;
    pf = 0.5 * (pc + nt);
    g += 1.0 * vec4<f32>(pf.x * ft.x, pf.x * ft.y, pf.y * ft.x, pf.y * ft.y) * ft.z;
    pf = 0.5 * (pc + nb);
    g += -1.0 * vec4<f32>(pf.x * fb.x, pf.x * fb.y, pf.y * fb.x, pf.y * fb.y) * fb.z;
    g = g * vcell[c].x;
    // F1 and X
    let rho = u[c].x;
    let nu = t.w / rho;
    let d = vcell[c].z;
    let x = 2.0 * rho * SIGMA_W2 * (g.x * g.z + g.y * g.w) / pc.y;
    let cd = max(x, 1e-20);
    let ta = blend_args(nu, pc.x, pc.y, d);
    let t3 = 4.0 * rho * SIGMA_W2 * pc.x / (cd * d * d);
    let a = min(max(ta.x, ta.y), t3);
    let a2 = a * a;
    tgb[2u * c] = g;
    tgb[2u * c + 1u] = vec4<f32>(tanh_pos(a2 * a2), x, 0.0, 0.0);
}
"#;

/// Coefficients of the two scalar line systems of every fluid cell: the residual `R = rho d(k, omega)/dt` (convection, diffusion, production,
/// destruction, cross-diffusion), `diag0` and the line coefficients (`turbulence::face_pass` + `cell_terms` + the assembly in `update`). The face data
/// (mass flow of the stored inviscid flux, diffusive conductance `G` and flux of the corrected scalar gradient, the two side states) are
/// evaluated here for the four faces of the cell. Output: `tres = (V R_k / dt, V R_w / dt, 0, 0)` (the line kernel multiplies by `dt`) and the 4x4 blocks that
/// are diagonal (entries 0, 5 the `k` and `omega` coefficients, 10, 15 = 1 on `d`) of the i factor (`blo_i`, `bd_i`, `bup_i`, stored at `i nj + j`) and
/// the j factor (`_j`, stored at `j ni + i`, scaled by `1 / diag0`). Problem size `[ni, nj]`. `FST`: 1 (f32 `fi`, `fj`) or 2 (double-float:
/// `(hi, lo)` per face).
const TURB_COEF: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> cell: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> vcell: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read> fi: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read> fj: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read> dt: array<f32>;
@group(1) @binding(8) var<storage, read> turb: array<vec4<f32>>;
@group(1) @binding(9) var<storage, read> grad: array<vec4<f32>>;
@group(1) @binding(10) var<storage, read> tgb: array<vec4<f32>>;
@group(1) @binding(11) var<storage, read> vgi: array<vec4<f32>>;
@group(1) @binding(12) var<storage, read> vgj: array<vec4<f32>>;
@group(1) @binding(13) var<storage, read_write> tres: array<vec4<f32>>;
@group(1) @binding(14) var<storage, read_write> blo_i: array<vec4<f32>>;
@group(1) @binding(15) var<storage, read_write> bd_i: array<vec4<f32>>;
@group(1) @binding(16) var<storage, read_write> bup_i: array<vec4<f32>>;
@group(1) @binding(17) var<storage, read_write> blo_j: array<vec4<f32>>;
@group(1) @binding(18) var<storage, read_write> bd_j: array<vec4<f32>>;
@group(1) @binding(19) var<storage, read_write> bup_j: array<vec4<f32>>;
@group(1) @binding(20) var<uniform> p: Params;

//@SIDES
//@KW
//@ACC
//@FST

// Face viscosity and eddy viscosity (`viscous::face_mu`).
fn face_mu(sd: Sides) -> vec2<f32> {
    var ml = turb[sd.l.cell].z;
    var mr = turb[sd.r.cell].z;
    if (sd.noslip && sd.l.ghost) { ml = -ml; }
    if (sd.noslip && sd.r.ghost) { mr = -mr; }
    return vec2<f32>(0.5 * (turb[sd.l.cell].w + turb[sd.r.cell].w), 0.5 * (ml + mr));
}

// Diffusion and convection data of one face (`turbulence::TFace`): mass flow `m` in the positive direction, conductance `g` and diffusive flux
// `flux` per scalar, the two side states `l` (lower index) and `r`.
struct TF { m: f32, g: vec2<f32>, flux: vec2<f32>, l: vec2<f32>, r: vec2<f32> }

// Diffusive flux and conductance `(flux, G)` of one scalar `e` (`turbulence::tface`): `sigma` blended from the `F1` of the two sides, the compact
// gradient `gbar + ((phi_R - phi_L) / |d| - gbar.e) e` projected on the face normal.
fn scalar_face(s1: f32, s2: f32, f1l: f32, f1r: f32, mm: vec2<f32>, gl: vec2<f32>, gr: vec2<f32>, kl: f32, kr: f32, vg: vec4<f32>, f: vec4<f32>) -> vec2<f32> {
    let sigma = 0.5 * (f1l * s1 + (1.0 - f1l) * s2 + f1r * s1 + (1.0 - f1r) * s2);
    let coef = mm.x + sigma * mm.y;
    let gbx = 0.5 * (gl.x + gr.x);
    let gbr = 0.5 * (gl.y + gr.y);
    let corr = (kr - kl) * vg.z - (gbx * vg.x + gbr * vg.y);
    let gn = (gbx + corr * vg.x) * f.x + (gbr + corr * vg.y) * f.y;
    return vec2<f32>(coef * gn * f.w, coef * f.w * vg.z);
}

fn tface(fk: FaceKW, f: vec4<f32>, vg: vec4<f32>, m: f32) -> TF {
    var o: TF;
    o.m = m;
    o.l = fk.kl;
    o.r = fk.kr;
    if (vg.w == 0.0) { return o; }
    let gl = tgb[2u * fk.sd.l.cell];
    let gr = tgb[2u * fk.sd.r.cell];
    let f1l = tgb[2u * fk.sd.l.cell + 1u].x;
    let f1r = tgb[2u * fk.sd.r.cell + 1u].x;
    let mm = face_mu(fk.sd);
    let sk = scalar_face(SIGMA_K1, SIGMA_K2, f1l, f1r, mm, gl.xy, gr.xy, fk.kl.x, fk.kr.x, vg, f);
    let sw = scalar_face(SIGMA_W1, SIGMA_W2, f1l, f1r, mm, gl.zw, gr.zw, fk.kl.y, fk.kr.y, vg, f);
    o.flux = vec2<f32>(sk.x, sw.x);
    o.g = vec2<f32>(sk.y, sw.y);
    return o;
}

fn tf_i(i: u32, j: u32) -> TF {
    let k = j * (p.ni + 1u) + i;
    let f = iface[k];
    return tface(iface_kw(i, j), f, vgi[k], fi[FST * k].x * f.w);
}

fn tf_j(i: u32, j: u32) -> TF {
    let k = j * p.ni + i;
    let f = jface[k];
    return tface(jface_kw(i, j), f, vgj[k], fj[FST * k].x * f.w);
}

// One line direction of a cell: `rate` = -convection + diffusion, the off-diagonal coefficients (0 at the end of a line) and the diagonal part.
struct Dir { rate: vec2<f32>, lo: vec2<f32>, up: vec2<f32>, dd: vec2<f32> }

fn dir_terms(lo_f: TF, up_f: TF, phi: vec2<f32>, at_lo: bool, at_hi: bool) -> Dir {
    let a_lo = max(lo_f.m, 0.0);
    let a_up = max(-up_f.m, 0.0);
    var o: Dir;
    o.rate = a_lo * (lo_f.l - phi) + a_up * (up_f.r - phi) + up_f.flux - lo_f.flux;
    o.lo = select(-(a_lo + lo_f.g), vec2<f32>(0.0), at_lo);
    o.up = select(-(a_up + up_f.g), vec2<f32>(0.0), at_hi);
    o.dd = a_lo + a_up + lo_f.g + up_f.g;
    return o;
}

fn diag2(v: vec2<f32>) -> M4 {
    return m_rows(vec4<f32>(v.x, 0.0, 0.0, 0.0), vec4<f32>(0.0, v.y, 0.0, 0.0), vec4<f32>(0.0, 0.0, 1.0, 0.0), vec4<f32>(0.0, 0.0, 0.0, 1.0));
}
fn offdiag2(v: vec2<f32>) -> M4 {
    return m_rows(vec4<f32>(v.x, 0.0, 0.0, 0.0), vec4<f32>(0.0, v.y, 0.0, 0.0), vec4<f32>(0.0), vec4<f32>(0.0));
}

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    let i = gid.x;
    let j = gid.y;
    let c = j * p.ni + i;
    let mi = i * p.nj + j;
    if (is_solid(i, j)) {
        st_blo_i(mi, m_zero());
        st_bd_i(mi, m_eye());
        st_bup_i(mi, m_zero());
        st_blo_j(c, m_zero());
        st_bd_j(c, m_eye());
        st_bup_j(c, m_zero());
        tres[c] = vec4<f32>(0.0);
        return;
    }
    let vol = cell[c].y;
    let rho = u[c].x;
    let t = turb[c];
    let phi = t.xy;
    let bl = tgb[2u * c + 1u];
    let v_c = u[c].z / rho;
    let s2 = strain2(grad[2u * c], v_c, vcell[c].y);
    let mut_c = t.z;
    let prod = min(mut_c * s2, 10.0 * BETA_STAR * rho * phi.x * phi.y);
    let beta = bl.x * BETA1 + (1.0 - bl.x) * BETA2;
    let gamma = bl.x * GAMMA1 + (1.0 - bl.x) * GAMMA2;
    let sources = vec2<f32>(
        prod - BETA_STAR * rho * phi.y * phi.x,
        gamma * rho * prod / max(mut_c, 1e-30) - beta * rho * phi.y * phi.y + (1.0 - bl.x) * bl.y);
    let dest = vec2<f32>(BETA_STAR * phi.y, 2.0 * beta * phi.y);
    let di = dir_terms(tf_i(i, j), tf_i(i + 1u, j), phi, i == row_start(j), i == p.ni - 1u);
    let dj = dir_terms(tf_j(i, j), tf_j(i, j + 1u), phi, j == col_start(i), j == p.nj - 1u);
    var rate = di.rate;
    rate = rate + dj.rate;
    let r = rate / vol + sources;
    let d0 = rho * vol / dt[c] + rho * vol * dest;
    tres[c] = vec4<f32>(vol * r / dt[c], 0.0, 0.0);
    st_blo_i(mi, offdiag2(di.lo));
    st_bup_i(mi, offdiag2(di.up));
    st_bd_i(mi, diag2(d0 + di.dd));
    st_blo_j(c, offdiag2(dj.lo / d0));
    st_bup_j(c, offdiag2(dj.up / d0));
    st_bd_j(c, diag2(1.0 + dj.dd / d0));
}
"#;

/// Clipped update of `k` and `omega` from the solution `tdu` of the line systems and the new eddy viscosity (`turbulence::update`, last kernels):
/// `phi_new = max(phi + dphi, 0.1 phi)`, then `k >= 1e-3 k_inf`, `omega >= 1e-3 omega_inf`; `mu_t = min(rho a1 k / max(a1 omega, S F2), 1e5 / re_c)`.
/// `turb.w` (`mu`) is kept. Problem size `[ni, nj]`.
const TURB_UPDATE: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> vcell: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> grad: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> tdu: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read_write> turb: array<vec4<f32>>;
@group(1) @binding(5) var<uniform> p: Params;

//@MATH

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    let i = gid.x;
    let j = gid.y;
    let c = j * p.ni + i;
    if (is_solid(i, j)) { return; }
    let t = turb[c];
    let dphi = tdu[c];
    let k = max(max(t.x + dphi.x, 0.1 * t.x), 1e-3 * p.k_inf);
    let om = max(max(t.y + dphi.y, 0.1 * t.y), 1e-3 * p.om_inf);
    let rho = u[c].x;
    let nu = t.w / rho;
    let strain = sqrt(max(strain2(grad[2u * c], u[c].z / rho, vcell[c].y), 0.0));
    let ta = blend_args(nu, k, om, vcell[c].z);
    let b = max(2.0 * ta.x, ta.y);
    let f2 = tanh_pos(b * b);
    let mu_t = min(rho * A1 * k / max(A1 * om, strain * f2), 1e5 * p.mu_inf);
    turb[c] = vec4<f32>(k, om, mu_t, t.w);
}
"#;

/// `strain2`, `blend_args` and `tanh_pos` (`turbulence::strain2`, `blend_args`), needing no face data; also part of every kernel that uses [`KW_LIB`].
const KW_MATH: &str = r#"
// Strain `S^2` of a cell from its gradient `(u_x, u_r, v_x, v_r)`, radial velocity and radius (`turbulence::strain2`).
fn strain2(g: vec4<f32>, v: f32, r: f32) -> f32 {
    let vr = v / r;
    let dv = g.x + g.w + vr;
    let shear = g.y + g.z;
    return 2.0 * (g.x * g.x + g.w * g.w + vr * vr) + shear * shear - 2.0 / 3.0 * dv * dv;
}

// `(sqrt(k) / (beta* omega d), 500 nu / (d^2 omega))` (`turbulence::blend_args`).
fn blend_args(nu: f32, k: f32, om: f32, d: f32) -> vec2<f32> {
    return vec2<f32>(sqrt(k) / (BETA_STAR * om * d), 500.0 * nu / (d * d * om));
}

// tanh of a non-negative argument; tanh(20) = 1 in f32, and the clamp keeps the exponentials finite.
fn tanh_pos(x: f32) -> f32 { return tanh(min(x, 20.0)); }
"#;

/// The kernels of the viscous terms and the SST model (built by [`GpuSolver::set_viscous`](crate::GpuSolver::set_viscous)).
pub(crate) struct ViscKernels {
    pub vgrad: Kernel,
    pub vflux: Kernel,
    /// SST only.
    pub sst: Option<SstKernels>,
}

/// The turbulence kernels (the line solves reuse `line_i` / `line_j` of the mean flow).
pub(crate) struct SstKernels {
    pub grad: Kernel,
    pub coef: Kernel,
    pub update: Kernel,
}

fn build(ctx: &Context, src: &str, label: &str) -> Result<Kernel> {
    Ok(ctx.kernel(src).label(label).build()?)
}

impl ViscKernels {
    /// Compiles the kernels; `sst`: with the turbulence model; `ext`: the face fluxes `fi`, `fj` are double-float pairs.
    pub(crate) fn build(ctx: &Context, sst: bool, ext: bool) -> Result<ViscKernels> {
        let kw = format!("{KW_MATH}\n{KW_LIB}");
        let fill = |s: &str| s.replace("//@SIDES", SIDES_LIB).replace("//@KW", &kw).replace("//@MATH", KW_MATH);
        let sst = if sst {
            let blocks = [("blo_i", true), ("bd_i", true), ("bup_i", true), ("blo_j", true), ("bd_j", true), ("bup_j", true)];
            let coef = TURB_COEF
                .replace("//@FST", if ext { "const FST: u32 = 2u;" } else { "const FST: u32 = 1u;" })
                .replace("//@ACC", &crate::shaders_impl::acc(&blocks));
            Some(SstKernels {
                grad: build(ctx, &with_common(&fill(TURB_GRAD)), "turb_grad")?,
                coef: build(ctx, &with_blocks(&fill(&coef)), "turb_coef")?,
                update: build(ctx, &with_common(&fill(TURB_UPDATE)), "turb_update")?,
            })
        } else {
            None
        };
        Ok(ViscKernels { vgrad: build(ctx, &with_common(&fill(VGRAD)), "vgrad")?, vflux: build(ctx, &with_common(&fill(VFLUX)), "vflux")?, sst })
    }
}

/// Work arrays of the SST model.
pub(crate) struct SstState {
    /// `(k_x, k_r, omega_x, omega_r)` and `(F1, X, 0, 0)` per cell.
    pub tgb: Array<[f32; 4]>,
    /// Right-hand side `(V R_k / dt, V R_w / dt, 0, 0)`, the intermediate and the final solution of the line systems.
    pub tres: Array<[f32; 4]>,
    pub tx: Array<[f32; 4]>,
    pub tdu: Array<[f32; 4]>,
    /// Blocks of the i and j factors of the two scalar systems (4 vec4 per cell, as `crate::Imp`).
    pub lo_i: Array<[f32; 4]>,
    pub d_i: Array<[f32; 4]>,
    pub up_i: Array<[f32; 4]>,
    pub lo_j: Array<[f32; 4]>,
    pub d_j: Array<[f32; 4]>,
    pub up_j: Array<[f32; 4]>,
}

/// State and geometry of the viscous terms on the GPU.
pub(crate) struct ViscState {
    pub v: Viscous,
    /// Number of cells.
    pub n: usize,
    /// Per i-face / j-face `(e_x, e_r, 1/|d|, r_m)`, per cell `(1/A, r_c, d_wall, 0)`.
    pub vgi: Array<[f32; 4]>,
    pub vgj: Array<[f32; 4]>,
    pub vcell: Array<[f32; 4]>,
    /// Two vec4 per cell: `(u_x, u_r, v_x, v_r)` and `(T_x, T_r, 0, 0)`.
    pub grad: Array<[f32; 4]>,
    /// `(k, omega, mu_t, mu)` per cell.
    pub turb: Array<[f32; 4]>,
    pub fvi: Array<[f32; 4]>,
    pub fvj: Array<[f32; 4]>,
    pub sst: Option<SstState>,
}

/// Distance and direction `(e_x, e_r, 1/|d|, r_m)` between the two sides of a face centred at `(xf, rf)` (`viscous::stencil`): a ghost sits at the mirror
/// image of the centre of its cell in the face centre.
fn stencil(grid: &Grid, (xf, rf): (f64, f64), l: (usize, bool), r: (usize, bool), rm: f64) -> [f32; 4] {
    let pos = |(c, ghost): (usize, bool)| {
        let (x, y) = (grid.xc[c], grid.rc[c]);
        if ghost {
            (2.0 * xf - x, 2.0 * rf - y)
        } else {
            (x, y)
        }
    };
    let ((xl, rl), (xr, rr)) = (pos(l), pos(r));
    let (dx, dr) = (xr - xl, rr - rl);
    let dd = dx.hypot(dr);
    [(dx / dd) as f32, (dr / dd) as f32, (1.0 / dd) as f32, rm as f32]
}

impl ViscState {
    /// Uploads the geometry (computed in f64 from `grid` with the sides of `viscous::iface_sides` / `jface_sides`) and allocates the state: the
    /// turbulence state at the free stream, `mu_t = rho k_inf / omega_inf`. `block` must be the body block of the solver.
    pub(crate) fn new(ctx: &Context, grid: &Grid, cfg: &Config, block: Option<Block>, v: Viscous) -> Result<ViscState> {
        let (ni, nj) = (grid.ni, grid.nj);
        let n = ni * nj;
        let mut vgi = vec![[0.0f32; 4]; (ni + 1) * nj];
        for j in 0..nj {
            for i in 0..=ni {
                let sides = match block {
                    Some(b) if j < b.j_end && (j < b.j_sting || i < b.i_end) => None,
                    Some(b) if j < b.j_end && i == b.i_end => Some(((j * ni + i, true), (j * ni + i, false))),
                    _ if i == 0 => Some(((j * ni, true), (j * ni, false))),
                    _ if i == ni => Some(((j * ni + ni - 1, false), (j * ni + ni - 1, true))),
                    _ => Some(((j * ni + i - 1, false), (j * ni + i, false))),
                };
                if let Some((l, r)) = sides {
                    let (a, b) = (grid.node(i, j), grid.node(i, j + 1));
                    vgi[j * (ni + 1) + i] = stencil(grid, (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1)), l, r, grid.iface[j * (ni + 1) + i].rm);
                }
            }
        }
        let mut vgj = vec![[0.0f32; 4]; ni * (nj + 1)];
        for j in 0..=nj {
            for i in 0..ni {
                let j0 = block.map_or(0, |b| b.col_start(i));
                let sides = match block {
                    Some(_) if j < j0 => None,
                    Some(_) if j == j0 && j0 > 0 => Some(((j * ni + i, true), (j * ni + i, false))),
                    _ if j == 0 => Some(((i, true), (i, false))),
                    _ if j == nj => Some((((nj - 1) * ni + i, false), ((nj - 1) * ni + i, true))),
                    _ => Some((((j - 1) * ni + i, false), (j * ni + i, false))),
                };
                if let Some((l, r)) = sides {
                    let (a, b) = (grid.node(i, j), grid.node(i + 1, j));
                    vgj[j * ni + i] = stencil(grid, (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1)), l, r, grid.jface[j * ni + i].rm);
                }
            }
        }
        let sst_on = v.turbulence == Turbulence::Sst;
        let dwall = if sst_on { sst::wall_distances(grid, v, &cfg.bc, block) } else { Vec::new() };
        let vcell: Vec<[f32; 4]> =
            (0..n).map(|c| [(1.0 / grid.area[c]) as f32, grid.rc[c] as f32, dwall.get(c).map_or(0.0, |d| d.min(1e15) as f32), 0.0]).collect();
        let arr = |x: &[[f32; 4]]| Array::from_slice_on(ctx, x, x.len());
        let sst_state = if sst_on {
            let m = || Array::<[f32; 4]>::zeros_on(ctx, 4 * n);
            Some(SstState {
                tgb: Array::zeros_on(ctx, 2 * n)?,
                tres: Array::zeros_on(ctx, n)?,
                tx: Array::zeros_on(ctx, n)?,
                tdu: Array::zeros_on(ctx, n)?,
                lo_i: m()?,
                d_i: m()?,
                up_i: m()?,
                lo_j: m()?,
                d_j: m()?,
                up_j: m()?,
            })
        } else {
            None
        };
        let mut s = ViscState {
            v,
            n,
            vgi: arr(&vgi)?,
            vgj: arr(&vgj)?,
            vcell: arr(&vcell)?,
            grad: Array::zeros_on(ctx, 2 * n)?,
            turb: Array::zeros_on(ctx, n)?,
            fvi: Array::zeros_on(ctx, (ni + 1) * nj)?,
            fvj: Array::zeros_on(ctx, ni * (nj + 1))?,
            sst: sst_state,
        };
        s.reset_turbulence(cfg.freestream[0])?;
        Ok(s)
    }

    /// Turbulence state back to the free stream (`turbulence::init`): `k_inf`, `omega_inf` and `mu_t = rho k_inf / omega_inf` (solid cells carry the
    /// same values, which are never read); all zero for a laminar run.
    pub(crate) fn reset_turbulence(&mut self, rho: f64) -> Result<()> {
        let t = if self.v.turbulence == Turbulence::Sst {
            let (k, om) = (sst::K_INF, sst::omega_inf(self.v.re_c));
            vec![[k as f32, om as f32, (rho * k / om) as f32, 0.0]; self.n]
        } else {
            vec![[0.0f32; 4]; self.n]
        };
        self.turb.write(&t)?;
        Ok(())
    }
}
