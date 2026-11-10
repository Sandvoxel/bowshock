//! WGSL of the line-implicit scheme (`bowshock_cfd::implicit`): approximate-factorization pseudo-time stepping
//!
//! ```text
//!   (I - dt J_i) X = dt R      block-tridiagonal along every i line (one workgroup per line)
//!   (I - dt J_j) dU = X        block-tridiagonal along every j line
//!   defect correction passes against the unfactored operator, then U += alpha dU (positivity damping)
//! ```
//!
//! `R` is the residual of the *second-order* scheme evaluated by the same flux kernels as the explicit solver (in the
//! double-float arithmetic of [`crate::Precision::Extended`]); the left-hand side (first-order flux Jacobians with a Roe-type
//! dissipation matrix) only steers the iteration and is evaluated in plain f32, so the fixed point of the iteration (`R = 0`) is
//! that of the explicit scheme. All kernels are all-cells-parallel except the line solves, which use one workgroup per line
//! (`nj` i lines, `ni` j lines) and a parallel cyclic reduction along it (4x4 blocks, see [`line`]).
//!
//! Storage (per direction, `Array<[f32; 4]>` read as `array<mat4x4<f32>>`, one block per cell): `lo`, `d`, `up` of `I - dt J`. The
//! i-direction blocks are stored transposed (`i * nj + j`); the vector arrays `x`, `du`, `res` use the cell index `j * ni + i`.

/// Matrix library, flux Jacobians, Roe-type dissipation matrix, ghost-state Jacobians and the per-cell blocks of `I - dt J`.
/// A 4x4 block is a `mat4x4<f32>` whose element `(row r, column c)` is `m[c][r]` (the CPU `M4` is row-major `m[4 r + c]`).
/// Appended to `shaders::common()`.
const IMPL_LIB: &str = r#"
alias M4 = mat4x4<f32>;

fn m_zero() -> M4 { return M4(vec4<f32>(0.0), vec4<f32>(0.0), vec4<f32>(0.0), vec4<f32>(0.0)); }
fn m_eye() -> M4 {
    return M4(vec4<f32>(1.0, 0.0, 0.0, 0.0), vec4<f32>(0.0, 1.0, 0.0, 0.0), vec4<f32>(0.0, 0.0, 1.0, 0.0), vec4<f32>(0.0, 0.0, 0.0, 1.0));
}
// Matrix from its rows.
fn m_rows(a: vec4<f32>, b: vec4<f32>, c: vec4<f32>, d: vec4<f32>) -> M4 { return transpose(M4(a, b, c, d)); }

// Inverse by the Laplace expansion in 2x2 minors (no pivoting; the pivot blocks `I - dt J ...` are diagonally dominant).
fn inv4(m: M4) -> M4 {
    let m00 = m[0][0]; let m01 = m[1][0]; let m02 = m[2][0]; let m03 = m[3][0];
    let m10 = m[0][1]; let m11 = m[1][1]; let m12 = m[2][1]; let m13 = m[3][1];
    let m20 = m[0][2]; let m21 = m[1][2]; let m22 = m[2][2]; let m23 = m[3][2];
    let m30 = m[0][3]; let m31 = m[1][3]; let m32 = m[2][3]; let m33 = m[3][3];
    let s0 = m00 * m11 - m10 * m01;
    let s1 = m00 * m12 - m10 * m02;
    let s2 = m00 * m13 - m10 * m03;
    let s3 = m01 * m12 - m11 * m02;
    let s4 = m01 * m13 - m11 * m03;
    let s5 = m02 * m13 - m12 * m03;
    let c5 = m22 * m33 - m32 * m23;
    let c4 = m21 * m33 - m31 * m23;
    let c3 = m21 * m32 - m31 * m22;
    let c2 = m20 * m33 - m30 * m23;
    let c1 = m20 * m32 - m30 * m22;
    let c0 = m20 * m31 - m30 * m21;
    let id = 1.0 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);
    return m_rows(
        vec4<f32>(m11 * c5 - m12 * c4 + m13 * c3, -m01 * c5 + m02 * c4 - m03 * c3, m31 * s5 - m32 * s4 + m33 * s3, -m21 * s5 + m22 * s4 - m23 * s3) * id,
        vec4<f32>(-m10 * c5 + m12 * c2 - m13 * c1, m00 * c5 - m02 * c2 + m03 * c1, -m30 * s5 + m32 * s2 - m33 * s1, m20 * s5 - m22 * s2 + m23 * s1) * id,
        vec4<f32>(m10 * c4 - m11 * c2 + m13 * c0, -m00 * c4 + m01 * c2 - m03 * c0, m30 * s4 - m31 * s2 + m33 * s0, -m20 * s4 + m21 * s2 - m23 * s0) * id,
        vec4<f32>(-m10 * c3 + m11 * c1 - m12 * c0, m00 * c3 - m01 * c1 + m02 * c0, -m30 * s3 + m31 * s1 - m32 * s0, m20 * s3 - m21 * s1 + m22 * s0) * id);
}

fn prim_to_cons(w: vec4<f32>) -> vec4<f32> {
    return vec4<f32>(w.x, w.x * w.y, w.x * w.z, w.w / (GAMMA - 1.0) + 0.5 * w.x * (w.y * w.y + w.z * w.z));
}

// A = d(F.n)/dU for the primitive state w (bowshock-cfd `implicit::flux_jacobian`).
fn flux_jacobian(w: vec4<f32>, nx: f32, nr: f32) -> M4 {
    let rho = w.x; let u = w.y; let v = w.z; let pr = w.w;
    let g1 = GAMMA - 1.0;
    let q2 = u * u + v * v;
    let th = u * nx + v * nr;
    let phi = 0.5 * g1 * q2;
    let h = GAMMA * pr / (g1 * rho) + 0.5 * q2;
    return m_rows(
        vec4<f32>(0.0, nx, nr, 0.0),
        vec4<f32>(phi * nx - u * th, th + (2.0 - GAMMA) * u * nx, u * nr - g1 * v * nx, g1 * nx),
        vec4<f32>(phi * nr - v * th, v * nx - g1 * u * nr, th + (2.0 - GAMMA) * v * nr, g1 * nr),
        vec4<f32>(th * (phi - h), h * nx - g1 * u * th, h * nr - g1 * v * th, GAMMA * th));
}

fn fix_eig(l: f32, eps: f32) -> f32 {
    let al = abs(l);
    if (al >= eps) { return al; }
    return 0.5 * (l * l / eps + eps);
}

// |A| of the Roe-averaged state as the quadratic polynomial through |lambda| at the three eigenvalues, with the Harten-type
// entropy fix `eps_fix c` (bowshock-cfd `implicit::abs_jacobian`).
fn abs_jacobian(wl: vec4<f32>, wr: vec4<f32>, nx: f32, nr: f32, eps_fix: f32) -> M4 {
    let sl = sqrt(wl.x);
    let sr = sqrt(wr.x);
    let inv = 1.0 / (sl + sr);
    let u = (sl * wl.y + sr * wr.y) * inv;
    let v = (sl * wl.z + sr * wr.z) * inv;
    let g1 = GAMMA - 1.0;
    let hl = GAMMA * wl.w / (g1 * wl.x) + 0.5 * (wl.y * wl.y + wl.z * wl.z);
    let hr = GAMMA * wr.w / (g1 * wr.x) + 0.5 * (wr.y * wr.y + wr.z * wr.z);
    let h = (sl * hl + sr * hr) * inv;
    let c = sqrt(max(g1 * (h - 0.5 * (u * u + v * v)), 1e-30));
    let a = u * nx + v * nr;
    let eps = eps_fix * c;
    let e1 = fix_eig(a, eps);
    let e2 = fix_eig(a + c, eps);
    let e3 = fix_eig(a - c, eps);
    let d1 = (e2 - e1) / c;
    let d2 = (e2 + e3 - 2.0 * e1) / (2.0 * c * c);
    let wa = vec4<f32>(1.0, u, v, g1 / GAMMA * (h - 0.5 * (u * u + v * v)));
    let am = flux_jacobian(wa, nx, nr);
    let b1 = am - m_eye() * a;
    let b2 = am - m_eye() * (a + c);
    return b1 * d1 + (b1 * b2) * d2 + m_eye() * e1;
}

fn spectral(w: vec4<f32>, nx: f32, nr: f32) -> f32 {
    return abs(w.y * nx + w.z * nr) + sqrt(GAMMA * w.w / w.x);
}

// Reflection matrix of the conserved variables about the wall normal.
fn reflection(nx: f32, nr: f32) -> M4 {
    return m_rows(
        vec4<f32>(1.0, 0.0, 0.0, 0.0),
        vec4<f32>(0.0, 1.0 - 2.0 * nx * nx, -2.0 * nx * nr, 0.0),
        vec4<f32>(0.0, -2.0 * nx * nr, 1.0 - 2.0 * nr * nr, 0.0),
        vec4<f32>(0.0, 0.0, 0.0, 1.0));
}

// T = dU_ghost / dU_interior of the first ghost layer (exact for the free stream, zero gradient and slip wall; central finite
// difference, relative step 1e-3, for the characteristic and pressure-outflow BCs: only the left-hand side depends on it).
fn ghost_jac(bc: u32, fs: vec4<f32>, edge: vec4<f32>, f: vec4<f32>, outward: f32) -> M4 {
    if (bc == 0u) { return m_zero(); }
    if (bc == 1u) { return m_eye(); }
    if (bc == 2u) { return reflection(f.x, f.y); }
    let u0 = prim_to_cons(edge);
    var t = m_zero();
    for (var q: i32 = 0; q < 4; q++) {
        let h = 1e-3 * (1.0 + abs(u0[q]));
        var up = u0;
        up[q] = up[q] + h;
        var um = u0;
        um[q] = um[q] - h;
        let gp = prim_to_cons(far_ghost(bc, fs, cons_to_prim(up), f, outward));
        let gm = prim_to_cons(far_ghost(bc, fs, cons_to_prim(um), f, outward));
        t[q] = (gp - gm) / (2.0 * h);
    }
    return t;
}

struct Blk { lo: M4, d: M4, up: M4 }

// Blocks of `I - dt J` of a cell (bowshock-cfd `implicit::cell_blocks`): `wk` the cell, `wm` / `wp` the lower / upper neighbour or
// ghost state, `fl` / `fr` the lower / upper face (nx, nr, len, weight), `src = A / V` for the axisymmetric pressure source (j lines).
fn cell_blocks(wk: vec4<f32>, wm: vec4<f32>, wp: vec4<f32>, fl: vec4<f32>, fr: vec4<f32>, vol: f32, dt: f32, src: f32) -> Blk {
    let lines = (p.iflags & 1u) != 0u;
    let roe = (p.iflags & 4u) != 0u;
    let s = 0.5 * dt / vol;
    let wl = fl.w;
    let wr = fr.w;
    var dis_l = m_zero();
    var dis_r = m_zero();
    if (roe) {
        dis_l = abs_jacobian(wm, wk, fl.x, fl.y, p.eps_fix);
        dis_r = abs_jacobian(wk, wp, fr.x, fr.y, p.eps_fix);
        // same shock sensor as the face flux: Rusanov-like dissipation inside shocks, Roe-type elsewhere
        let tl = shock_weight(wm.w, wk.w);
        let tr = shock_weight(wk.w, wp.w);
        if (tl > 0.0 || tr > 0.0) {
            let lam_l = max(spectral(wk, fl.x, fl.y), spectral(wm, fl.x, fl.y));
            let lam_r = max(spectral(wk, fr.x, fr.y), spectral(wp, fr.x, fr.y));
            dis_l = dis_l * (1.0 - tl) + m_eye() * (tl * lam_l);
            dis_r = dis_r * (1.0 - tr) + m_eye() * (tr * lam_r);
        }
    } else {
        dis_l = m_eye() * max(spectral(wk, fl.x, fl.y), spectral(wm, fl.x, fl.y));
        dis_r = m_eye() * max(spectral(wk, fr.x, fr.y), spectral(wp, fr.x, fr.y));
    }
    var d = m_eye() + flux_jacobian(wk, fr.x, fr.y) * (s * wr) - flux_jacobian(wk, fl.x, fl.y) * (s * wl)
        + dis_r * (s * wr) + dis_l * (s * wl);
    if (src != 0.0) {
        // -dt dS/dU of S = (0, 0, p A / V, 0): row 2 of the block
        let c = -dt * src * (GAMMA - 1.0);
        d[0].z += c * 0.5 * (wk.y * wk.y + wk.z * wk.z);
        d[1].z += -c * wk.y;
        d[2].z += -c * wk.z;
        d[3].z += c;
    }
    if (!lines) { return Blk(m_zero(), d, m_zero()); }
    let lo = (flux_jacobian(wm, fl.x, fl.y) + dis_l) * (-s * wl);
    let up = (flux_jacobian(wp, fr.x, fr.y) - dis_r) * (s * wr);
    return Blk(lo, d, up);
}
"#;

/// WGSL prelude of the f32 implicit kernels: `COMMON` + [`IMPL_LIB`].
pub fn common_impl() -> String {
    format!("{}\n{}", crate::shaders::common(), IMPL_LIB)
}

/// WGSL prelude of the extended-precision residual kernel (the implicit LHS kernels use [`common_impl`]).
fn common_impl_ext() -> String {
    crate::shaders_ext::common_ext()
}

/// Cell residual in f32 from the f32 face fluxes, stored in `res` (stage-independent, nothing is updated) together with the
/// pseudo-time step `dt`, `rho_res` and the per-workgroup partial sums of `rho_res^2` (residual history). Solid cells of a block
/// get `res = dt = 0`. `dt = icfl A / (L_i + L_j)` (`DtMode::Sum`) or `icfl A / (2 min(L_i, L_j))` (`iflags` bit 8), `L_d` = half
/// the sum of `(|u.n| + c) len` over the two faces of direction `d`. Problem size `[ni, nj]`.
pub const RESIDUAL_STORE: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> fi: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> fj: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read> cell: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read_write> res: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read_write> dt: array<f32>;
@group(1) @binding(8) var<storage, read_write> rho_res: array<f32>;
@group(1) @binding(9) var<storage, read_write> partial: array<f32>;
@group(1) @binding(10) var<uniform> p: Params;

var<workgroup> sh: array<f32, 128>;

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(local_invocation_index) li: u32,
        @builtin(workgroup_id) wid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    var r2: f32 = 0.0;
    if (gid.x < p.ni && gid.y < p.nj) {
        let i = gid.x;
        let j = gid.y;
        let c = j * p.ni + i;
        if (is_solid(i, j)) {
            res[c] = vec4<f32>(0.0);
            dt[c] = 0.0;
            rho_res[c] = 0.0;
        } else {
            let wc = cons_to_prim(u[c]);
            let fl = iface[j * (p.ni + 1u) + i];
            let fr = iface[j * (p.ni + 1u) + i + 1u];
            let fb = jface[j * p.ni + i];
            let ft = jface[(j + 1u) * p.ni + i];
            let gl = fi[j * (p.ni + 1u) + i];
            let gr = fi[j * (p.ni + 1u) + i + 1u];
            let gb = fj[j * p.ni + i];
            let gt = fj[(j + 1u) * p.ni + i];
            let geo = cell[c];
            let inv_v = 1.0 / geo.y;
            var rs = -(gr * fr.w - gl * fl.w + gt * ft.w - gb * fb.w) * inv_v;
            rs.z = rs.z + wc.w * geo.x * inv_v; // axisymmetric pressure source p*A/V
            res[c] = rs;
            rho_res[c] = rs.x;
            r2 = rs.x * rs.x;
            let cs = sqrt(GAMMA * wc.w / wc.x);
            let lam_l = (abs(wc.y * fl.x + wc.z * fl.y) + cs) * fl.z;
            let lam_r = (abs(wc.y * fr.x + wc.z * fr.y) + cs) * fr.z;
            let lam_b = (abs(wc.y * fb.x + wc.z * fb.y) + cs) * fb.z;
            let lam_t = (abs(wc.y * ft.x + wc.z * ft.y) + cs) * ft.z;
            let l_i = 0.5 * (lam_l + lam_r);
            let l_j = 0.5 * (lam_b + lam_t);
            var dtc = p.icfl * geo.x / (l_i + l_j);
            if ((p.iflags & 8u) != 0u) { dtc = p.icfl * geo.x / (2.0 * min(l_i, l_j)); }
            dt[c] = dtc;
        }
    }
    sh[li] = r2;
    workgroupBarrier();
    for (var stride: u32 = 64u; stride > 0u; stride = stride >> 1u) {
        if (li < stride) { sh[li] += sh[li + stride]; }
        workgroupBarrier();
    }
    if (li == 0u) { partial[wid.y * nwg.x + wid.x] = sh[0]; }
}
"#;

/// As [`RESIDUAL_STORE`] with the double-float arithmetic of the extended precision: the fluxes `fi`, `fj` are `(hi, lo)` vec4 pairs,
/// the state `u` + `ulo`, the cell sum is formed in df and collapsed to the f32 `res`. Appended to `common_ext()`.
pub const RESIDUAL_STORE_EXT: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> fi: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> fj: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read> cell: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read_write> res: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read_write> dt: array<f32>;
@group(1) @binding(8) var<storage, read_write> rho_res: array<f32>;
@group(1) @binding(9) var<storage, read_write> partial: array<f32>;
@group(1) @binding(10) var<storage, read> ulo: array<vec4<f32>>;
@group(1) @binding(11) var<uniform> p: Params;

var<workgroup> sh: array<f32, 128>;

// -(gr wr - gl wl + gt wt - gb wb) for one component (`pa`: the source term added before the division by V).
fn dsum(gl: D, wl: f32, gr: D, wr: f32, gb: D, wb: f32, gt: D, wt: f32, pa: D) -> D {
    return dadd(dsub(dsub(dmulf(gl, wl), dmulf(gr, wr)), dsub(dmulf(gt, wt), dmulf(gb, wb))), pa);
}

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(local_invocation_index) li: u32,
        @builtin(workgroup_id) wid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    var r2: f32 = 0.0;
    if (gid.x < p.ni && gid.y < p.nj) {
        let i = gid.x;
        let j = gid.y;
        let c = j * p.ni + i;
        if (is_solid(i, j)) {
            res[c] = vec4<f32>(0.0);
            dt[c] = 0.0;
            rho_res[c] = 0.0;
        } else {
            let wc = dcons_to_prim(u[c], ulo[c]);
            let fl = iface[j * (p.ni + 1u) + i];
            let fr = iface[j * (p.ni + 1u) + i + 1u];
            let fb = jface[j * p.ni + i];
            let ft = jface[(j + 1u) * p.ni + i];
            let gl = c_from(fi[2u * (j * (p.ni + 1u) + i)], fi[2u * (j * (p.ni + 1u) + i) + 1u]);
            let gr = c_from(fi[2u * (j * (p.ni + 1u) + i + 1u)], fi[2u * (j * (p.ni + 1u) + i + 1u) + 1u]);
            let gb = c_from(fj[2u * (j * p.ni + i)], fj[2u * (j * p.ni + i) + 1u]);
            let gt = c_from(fj[2u * ((j + 1u) * p.ni + i)], fj[2u * ((j + 1u) * p.ni + i) + 1u]);
            let geo = cell[c];
            let inv_v = 1.0 / geo.y;
            let gc = dgeo_corr(wc, cell[c].zwxy);
            let s = vec4<f32>(
                dsum(gl.a, fl.w, gr.a, fr.w, gb.a, fb.w, gt.a, ft.w, gc.a).h,
                dsum(gl.b, fl.w, gr.b, fr.w, gb.b, fb.w, gt.b, ft.w, gc.b).h,
                dsum(gl.c, fl.w, gr.c, fr.w, gb.c, fb.w, gt.c, ft.w, dadd(dmulf(wc.p, geo.x), gc.c)).h,
                dsum(gl.d, fl.w, gr.d, fr.w, gb.d, fb.w, gt.d, ft.w, gc.d).h);
            let rs = s * inv_v;
            res[c] = rs;
            rho_res[c] = rs.x;
            r2 = rs.x * rs.x;
            let cs = sqrt(GAMMA * wc.p.h / wc.r.h);
            let lam_l = (abs(wc.u.h * fl.x + wc.v.h * fl.y) + cs) * fl.z;
            let lam_r = (abs(wc.u.h * fr.x + wc.v.h * fr.y) + cs) * fr.z;
            let lam_b = (abs(wc.u.h * fb.x + wc.v.h * fb.y) + cs) * fb.z;
            let lam_t = (abs(wc.u.h * ft.x + wc.v.h * ft.y) + cs) * ft.z;
            let l_i = 0.5 * (lam_l + lam_r);
            let l_j = 0.5 * (lam_b + lam_t);
            var dtc = p.icfl * geo.x / (l_i + l_j);
            if ((p.iflags & 8u) != 0u) { dtc = p.icfl * geo.x / (2.0 * min(l_i, l_j)); }
            dt[c] = dtc;
        }
    }
    sh[li] = r2;
    workgroupBarrier();
    for (var stride: u32 = 64u; stride > 0u; stride = stride >> 1u) {
        if (li < stride) { sh[li] += sh[li + stride]; }
        workgroupBarrier();
    }
    if (li == 0u) { partial[wid.y * nwg.x + wid.x] = sh[0]; }
}
"#;

/// Assembly of the blocks `lo`, `d`, `up` of `I - dt J_dir` for every cell (bowshock-cfd `implicit::assemble`). `dir_i`: lines along
/// i, blocks stored at `i * nj + j`, launched as `[nj, ni]` (`gid.x` = j); otherwise lines along j, blocks at `j * ni + i`, launched
/// as `[ni, nj]`. Neighbour states are the f32 primitives of the high state; at the ends of a line the ghost state of the BC (or the
/// mirror image across a wall of the solid block) enters the pivot block through `T = dU_ghost / dU` (`lo`/`up` of that end = 0).
/// WGSL load (and, if `rw`, store) functions `ld_<name>(i)`, `st_<name>(i, m)` of the 4x4 blocks stored as four consecutive column
/// vec4 at `4 i ..` of the `array<vec4<f32>>` binding `name` (Luma arrays hold vec4 at most).
fn acc(names: &[(&str, bool)]) -> String {
    let mut s = String::new();
    for (n, rw) in names {
        s += &format!("fn ld_{n}(i: u32) -> M4 {{ return M4({n}[4u * i], {n}[4u * i + 1u], {n}[4u * i + 2u], {n}[4u * i + 3u]); }}\n");
        if *rw {
            s += &format!("fn st_{n}(i: u32, m: M4) {{ {n}[4u * i] = m[0]; {n}[4u * i + 1u] = m[1]; {n}[4u * i + 2u] = m[2]; {n}[4u * i + 3u] = m[3]; }}\n");
        }
    }
    s
}

fn assemble(dir_i: bool) -> String {
    ASSEMBLE_TEMPLATE
        .replace("//@ACC", &acc(&[("blo", true), ("bd", true), ("bup", true)]))
        .replace("//@DIR", if dir_i { "const DIR_I: bool = true;" } else { "const DIR_I: bool = false;" })
}

const ASSEMBLE_TEMPLATE: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> cell: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> dt: array<f32>;
@group(1) @binding(5) var<storage, read_write> blo: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read_write> bd: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read_write> bup: array<vec4<f32>>;
//@ACC
@group(1) @binding(8) var<uniform> p: Params;

//@DIR

fn cell_w(i: u32, j: u32) -> vec4<f32> { return cons_to_prim(u[j * p.ni + i]); }

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    var i = gid.x;
    var j = gid.y;
    if (DIR_I) { i = gid.y; j = gid.x; }
    if (i >= p.ni || j >= p.nj) { return; }
    let c = j * p.ni + i;
    var mi = c;
    if (DIR_I) { mi = i * p.nj + j; }
    if (is_solid(i, j)) {
        st_blo(mi, m_zero());
        st_bd(mi, m_eye());
        st_bup(mi, m_zero());
        return;
    }
    let fs = vec4<f32>(p.fs_rho, p.fs_u, p.fs_v, p.fs_p);
    let implicit_bc = (p.iflags & 2u) != 0u;
    let wk = cell_w(i, j);
    let geo = cell[c];
    var fl: vec4<f32>;
    var fr: vec4<f32>;
    var k: u32;
    var kn: u32;
    var k0: u32;
    var src = 0.0;
    var bc_lo: u32;
    var bc_hi: u32;
    var wm = wk;
    var wp = wk;
    if (DIR_I) {
        k = i; kn = p.ni; k0 = row_start(j);
        fl = iface[j * (p.ni + 1u) + i];
        fr = iface[j * (p.ni + 1u) + i + 1u];
        bc_lo = p.bc0; bc_hi = p.bc1;
        if (k > k0) { wm = cell_w(i - 1u, j); }
        if (k + 1u < kn) { wp = cell_w(i + 1u, j); }
    } else {
        k = j; kn = p.nj; k0 = col_start(i);
        src = geo.x / geo.y;
        fl = jface[c];
        fr = jface[c + p.ni];
        bc_lo = p.bc2; bc_hi = p.bc3;
        if (k > k0) { wm = cell_w(i, j - 1u); }
        if (k + 1u < kn) { wp = cell_w(i, j + 1u); }
    }
    let at_lo = (k == k0);
    let at_hi = (k + 1u == kn);
    var t_lo = m_zero();
    var t_hi = m_zero();
    if (at_lo) {
        if (k0 > 0u) {
            // solid body below / behind the cell: slip wall with the face normal
            wm = reflect_wall(wk, fl.x, fl.y);
            t_lo = reflection(fl.x, fl.y);
        } else {
            wm = ghost_state(bc_lo, fs, wk, wk, fl, -1.0);
            if (implicit_bc) { t_lo = ghost_jac(bc_lo, fs, wk, fl, -1.0); }
        }
    }
    if (at_hi) {
        wp = ghost_state(bc_hi, fs, wk, wk, fr, 1.0);
        if (implicit_bc) { t_hi = ghost_jac(bc_hi, fs, wk, fr, 1.0); }
    }
    var b = cell_blocks(wk, wm, wp, fl, fr, geo.y, dt[c], src);
    if (at_lo) { b.d = b.d + b.lo * t_lo; b.lo = m_zero(); }
    if (at_hi) { b.d = b.d + b.up * t_hi; b.up = m_zero(); }
    st_blo(mi, b.lo);
    st_bd(mi, b.d);
    st_bup(mi, b.up);
}
"#;

/// Block-tridiagonal solves along all lines of one direction, one workgroup per line (`nj` i lines, `ni` j lines), by cyclic
/// reduction on the rows `k0 .. n` of the line (`k0` the first fluid cell, `m = n - k0` rows, `r = k - k0`). The rows
/// `(lo, d, up | rhs)` are normalised to `x_r + L x_{r-1} + U x_{r+1} = f`. Forward level `h` = 1, 2, 4, ...: the rows `r = 2 h q`
/// eliminate their neighbours `r -+ h` (rows that are not modified at this level, so the update is in place and one barrier per
/// level suffices), which leaves couplings at distance `2 h`; the row 0 then holds its solution `x_0 = f_0`. Backward, `h` halving:
/// the rows `h (2 q + 1)` follow from `x_r = f_r - L_r x_{r-h} - U_r x_{r+h}` with their own, never overwritten, `L`, `U`, `f`.
/// A thread handles several rows of a level in a loop if the line is longer than the workgroup.
///
/// The rows live in workgroup memory (`L`, `U`, `f` as 9 vec4 per row, component-wise); if that exceeds [`LINE_SHARED_BYTES`] only the
/// rows that are a multiple of `ks` do, the others (touched in the first and last `log2 ks` levels only) in the global scratch `sm`.
/// `dir_i`: the right-hand side of pass `p.aux == 0` is `res * dt`, later passes read `x` (written by `APPLY`); the solution is written
/// back to `x`. j lines: the right-hand side is `x` (the result of the i lines), the solution is written to `du` (pass 0) or added to
/// it (`aux > 0`, defect correction).
fn line(dir_i: bool, n: usize) -> String {
    let ks = line_row_stride(n);
    LINE_TEMPLATE
        .replace("//@ACC", &acc(&[("blo", false), ("bd", false), ("bup", false)]))
        .replace("//@DIR", if dir_i { "const DIR_I: bool = true;" } else { "const DIR_I: bool = false;" })
        .replace("//@CONSTS", &format!("const WG: u32 = {}u;\nconst KS: u32 = {ks}u;\nconst NSH: u32 = {}u;", line_threads(n), n.div_ceil(ks)))
}

/// Threads per line of a line solve of length `n`: one per row up to 256, a multiple of the warp size.
fn line_threads(n: usize) -> u32 {
    let rows = n.div_ceil(256);
    (n.div_ceil(rows).div_ceil(32) * 32) as u32
}

/// Workgroup memory the rows of one line may take (wgpu's default limit); 9 vec4 per row.
const LINE_SHARED_BYTES: usize = 16384;

/// Power of two `ks`: the rows that are a multiple of `ks` live in workgroup memory, the others in global memory.
fn line_row_stride(n: usize) -> usize {
    let mut ks = 1;
    while n.div_ceil(ks) * 144 > LINE_SHARED_BYTES {
        ks *= 2;
    }
    ks
}

const LINE_TEMPLATE: &str = r#"
@group(1) @binding(0) var<storage, read> blo: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> bd: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> bup: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read_write> sm: array<vec4<f32>>;
//@ACC
@group(1) @binding(4) var<storage, read_write> x: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read_write> du: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read> res: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read> dt: array<f32>;
@group(1) @binding(8) var<uniform> p: Params;

//@DIR
//@CONSTS

var<workgroup> shs: array<vec4<f32>, 9u * NSH>;
var<private> row_base: u32;

fn cidx(l: u32, k: u32) -> u32 {
    if (DIR_I) { return l * p.ni + k; }
    return k * p.ni + l;
}
fn midx(l: u32, k: u32) -> u32 {
    if (DIR_I) { return k * p.nj + l; }
    return k * p.ni + l;
}

// Component `e` .. `e + 3` of the row `r`: `e` = 0 the columns of `L`, 4 those of `U`, 8 the vector `f` (single vec4). The global
// scratch is component-wise (index `e ni nj + sc`, `sc = l n + k` the row) so that neighbouring rows are neighbouring vec4.
fn in_shared(r: u32) -> bool { return (r & (KS - 1u)) == 0u; }
fn ld_v(e: u32, r: u32) -> vec4<f32> {
    if (in_shared(r)) { return shs[e * NSH + r / KS]; }
    return sm[e * p.ni * p.nj + row_base + r];
}
fn st_v(e: u32, r: u32, v: vec4<f32>) {
    if (in_shared(r)) { shs[e * NSH + r / KS] = v; return; }
    sm[e * p.ni * p.nj + row_base + r] = v;
}
fn ld_m(e: u32, r: u32) -> M4 {
    if (in_shared(r)) {
        let o = e * NSH + r / KS;
        return M4(shs[o], shs[o + NSH], shs[o + 2u * NSH], shs[o + 3u * NSH]);
    }
    let c = p.ni * p.nj;
    let o = e * c + row_base + r;
    return M4(sm[o], sm[o + c], sm[o + 2u * c], sm[o + 3u * c]);
}
fn st_m(e: u32, r: u32, m: M4) {
    if (in_shared(r)) {
        let o = e * NSH + r / KS;
        shs[o] = m[0];
        shs[o + NSH] = m[1];
        shs[o + 2u * NSH] = m[2];
        shs[o + 3u * NSH] = m[3];
        return;
    }
    let c = p.ni * p.nj;
    let o = e * c + row_base + r;
    sm[o] = m[0];
    sm[o + c] = m[1];
    sm[o + 2u * c] = m[2];
    sm[o + 3u * c] = m[3];
}

// Barrier after rows were written; the global scratch needs the storage barrier, the rows in workgroup memory do not.
fn sync(global_rows: bool) {
    if (global_rows) { storageBarrier(); }
    workgroupBarrier();
}

@compute @workgroup_size(WG)
fn main(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let l = wid.x;
    var k0: u32;
    var n: u32;
    if (DIR_I) {
        k0 = row_start(l);
        n = p.ni;
    } else {
        k0 = col_start(l);
        n = p.nj;
    }
    if (k0 >= n) { return; }
    let m = n - k0;
    row_base = l * n + k0;
    for (var r: u32 = li; r < m; r += WG) {
        let k = k0 + r;
        let mi = midx(l, k);
        let ci = cidx(l, k);
        var rhs = x[ci];
        if (DIR_I && p.aux == 0u) { rhs = res[ci] * dt[ci]; }
        let di = inv4(ld_bd(mi));
        var lm = m_zero();
        var um = m_zero();
        if (r > 0u) { lm = di * ld_blo(mi); }
        if (r + 1u < m) { um = di * ld_bup(mi); }
        st_m(0u, r, lm);
        st_m(4u, r, um);
        st_v(8u, r, di * rhs);
    }
    sync(KS > 1u);
    var h = 1u;
    for (; h < m; h = h << 1u) {
        for (var r: u32 = 2u * h * li; r < m; r += 2u * h * WG) {
            let lk = ld_m(0u, r);
            let uk = ld_m(4u, r);
            var fk = ld_v(8u, r);
            var d = m_eye();
            var ln = m_zero();
            var un = m_zero();
            if (r > 0u) {
                d = d - lk * ld_m(4u, r - h);
                ln = (lk * ld_m(0u, r - h)) * -1.0;
                fk = fk - lk * ld_v(8u, r - h);
            }
            if (r + h < m) {
                d = d - uk * ld_m(0u, r + h);
                un = (uk * ld_m(4u, r + h)) * -1.0;
                fk = fk - uk * ld_v(8u, r + h);
            }
            let di = inv4(d);
            st_m(0u, r, di * ln);
            st_m(4u, r, di * un);
            st_v(8u, r, di * fk);
        }
        sync(2u * h < KS);
    }
    while (h > 1u) {
        h = h >> 1u;
        for (var r: u32 = h + 2u * h * li; r < m; r += 2u * h * WG) {
            var xk = ld_v(8u, r) - ld_m(0u, r) * ld_v(8u, r - h);
            if (r + h < m) { xk = xk - ld_m(4u, r) * ld_v(8u, r + h); }
            st_v(8u, r, xk);
        }
        sync(h < KS);
    }
    for (var r: u32 = li; r < m; r += WG) {
        let ci = cidx(l, k0 + r);
        let xk = ld_v(8u, r);
        if (DIR_I) {
            x[ci] = xk;
        } else if (p.aux == 0u) {
            du[ci] = xk;
        } else {
            du[ci] = du[ci] + xk;
        }
    }
}
"#;

/// Defect `x = dt res - (A_i du + A_j du - du)` with the stored (unfactored) blocks of both directions (bowshock-cfd `implicit::step`,
/// `apply_factor`), the right-hand side of a defect-correction pass; 0 in solid cells. Problem size `[ni, nj]`.
pub fn apply() -> String {
    APPLY_TEMPLATE.replace(
        "//@ACC",
        &acc(&[("lo_i", false), ("d_i", false), ("up_i", false), ("lo_j", false), ("d_j", false), ("up_j", false)]),
    )
}

const APPLY_TEMPLATE: &str = r#"
@group(1) @binding(0) var<storage, read> lo_i: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> d_i: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> up_i: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> lo_j: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> d_j: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read> up_j: array<vec4<f32>>;
//@ACC
@group(1) @binding(6) var<storage, read> du: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read> res: array<vec4<f32>>;
@group(1) @binding(8) var<storage, read> dt: array<f32>;
@group(1) @binding(9) var<storage, read_write> x: array<vec4<f32>>;
@group(1) @binding(10) var<uniform> p: Params;

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    let i = gid.x;
    let j = gid.y;
    let c = j * p.ni + i;
    if (is_solid(i, j)) { x[c] = vec4<f32>(0.0); return; }
    let mi = i * p.nj + j;
    let dc = du[c];
    var ai = ld_d_i(mi) * dc;
    if (i > row_start(j)) { ai = ai + ld_lo_i(mi) * du[c - 1u]; }
    if (i + 1u < p.ni) { ai = ai + ld_up_i(mi) * du[c + 1u]; }
    var aj = ld_d_j(c) * dc;
    if (j > col_start(i)) { aj = aj + ld_lo_j(c) * du[c - p.ni]; }
    if (j + 1u < p.nj) { aj = aj + ld_up_j(c) * du[c + p.ni]; }
    x[c] = res[c] * dt[c] - (ai + aj - dc);
}
"#;

/// `U += alpha dU` with the positivity damping of `implicit::step` (halving `alpha` up to 7 times until density and pressure are
/// positive and finite); in the compensated / extended precision the increment is added with the error-free `df_add`. Solid
/// cells are left alone. Problem size `[ni, nj]`.
pub fn update(comp: bool) -> String {
    let (bind, body) = if comp {
        (
            "@group(1) @binding(2) var<storage, read_write> ulo: array<vec4<f32>>;\n@group(1) @binding(3) var<uniform> p: Params;",
            "let ql = ulo[c];\n\
             for (var t: i32 = 0; t < 8; t++) {\n\
                 let r = df_add(q, ql, alpha * dq);\n\
                 let wq = cons_to_prim(r.hi);\n\
                 if (wq.x > 0.0 && wq.w > 0.0 && all(abs(r.hi) < vec4<f32>(1e30))) { u[c] = r.hi; ulo[c] = r.lo; return; }\n\
                 alpha = alpha * 0.5;\n\
             }",
        )
    } else {
        (
            "@group(1) @binding(2) var<uniform> p: Params;",
            "for (var t: i32 = 0; t < 8; t++) {\n\
                 let r = q + alpha * dq;\n\
                 let wq = cons_to_prim(r);\n\
                 if (wq.x > 0.0 && wq.w > 0.0 && all(abs(r) < vec4<f32>(1e30))) { u[c] = r; return; }\n\
                 alpha = alpha * 0.5;\n\
             }",
        )
    };
    UPDATE_TEMPLATE.replace("//@BINDINGS", bind).replace("//@BODY", body)
}

const UPDATE_TEMPLATE: &str = r#"
@group(1) @binding(0) var<storage, read_write> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> du: array<vec4<f32>>;
//@BINDINGS

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    if (is_solid(gid.x, gid.y)) { return; }
    let c = gid.y * p.ni + gid.x;
    let q = u[c];
    let dq = du[c];
    var alpha: f32 = 1.0;
    //@BODY
}
"#;

pub fn assemble_i() -> String {
    assemble(true)
}
pub fn assemble_j() -> String {
    assemble(false)
}

/// i lines (length `ni`) / j lines (length `nj`).
pub fn line_i(ni: usize) -> String {
    line(true, ni)
}
pub fn line_j(nj: usize) -> String {
    line(false, nj)
}

/// Kernel sources with their preludes: `(common + source)` for the f32 kernels and the extended residual.
pub fn with_common(src: &str) -> String {
    format!("{}\n{}", common_impl(), src)
}

pub fn residual_store_ext() -> String {
    format!("{}\n{}", common_impl_ext(), RESIDUAL_STORE_EXT)
}
