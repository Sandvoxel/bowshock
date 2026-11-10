//! WGSL sources. `COMMON` holds direct translations of bowshock-cfd's pure functions
//! (`gas.rs`, `flux.rs`); every kernel is `COMMON` + its own bindings + one entry point `main`
//! (Luma reflects *all* module-level bindings, so each kernel needs its own source string).
//!
//! State layouts (same as bowshock-cfd): conservative `(rho, rho u, rho v, E)`, primitive `(rho, u, v, p)`.
//! Index conventions (same as bowshock-cfd): cell `(i, j)` at `j*ni + i`, i-face `(i, j)` (between cells
//! `i-1` and `i`) at `j*(ni+1) + i`, j-face at `j*ni + i`, primitive array with 2 ghost layers at
//! `(j+2)*(ni+4) + (i+2)`.

/// Uniform block shared by every kernel (128 bytes, matches `crate::Params`). `fs_lo_*`: low (df) parts of the free stream.
/// `limiter`: 0 = first order, 1 = minmod, 2 = van Albada. `bc*`: 0 = free stream, 1 = extrapolate,
/// 2 = slip wall, 3 = characteristic far field, 4 = static-pressure outflow. `stage`: 1..3 (SSP-RK3 stage). `slot`:
/// residual-history slot. `aux`: number of partial sums read by `HIST_FINAL` (line solves: pass index). `ra`, `rb`: RK stage update
/// `u = ra (u + dt res) + rb u0`. `low_mach`: 1 = Thornber low-Mach correction of the reconstructed face states.
/// `blk_i, blk_j, blk_s`: solid body block (`bowshock_cfd::solver::Block`; all 0 = no block). `icfl`, `eps_fix`, `iflags`:
/// implicit scheme (pseudo-time CFL, entropy fix of the Roe-type dissipation, bit 1 lines, 2 implicit BCs, 4 Roe-type
/// dissipation, 8 `DtMode::MinDirection`).
const COMMON_TEMPLATE: &str = r#"
struct Params {
    ni: u32, nj: u32, limiter: u32, stage: u32,
    cfl: f32, fs_rho: f32, fs_u: f32, fs_v: f32,
    fs_p: f32, bc0: u32, bc1: u32, bc2: u32,
    bc3: u32, slot: u32, aux: u32, zero: u32,
    ra: f32, rb: f32, low_mach: u32, iflags: u32,
    fs_lo_rho: f32, fs_lo_u: f32, fs_lo_v: f32, fs_lo_p: f32,
    blk_i: u32, blk_j: u32, blk_s: u32, ipad0: u32,
    icfl: f32, eps_fix: f32, ipad1: u32, ipad2: u32,
}

// Solid body block (all-zero parameters: no block). Cell (i, j) is solid if i < blk_i && j < blk_j (body) or
// i >= blk_i && j < blk_s (sting).
fn is_solid(i: u32, j: u32) -> bool {
    if (i < p.blk_i) { return j < p.blk_j; }
    return j < p.blk_s;
}
// First fluid row of column i.
fn col_start(i: u32) -> u32 {
    if (i < p.blk_i) { return p.blk_j; }
    return p.blk_s;
}
// First fluid cell of row j (`ni` if the whole row is solid).
fn row_start(j: u32) -> u32 {
    if (j < p.blk_s) { return p.ni; }
    if (j < p.blk_j) { return p.blk_i; }
    return 0u;
}

const NG: i32 = 2;
const GAMMA: f32 = 1.4;

fn pidx(ni: u32, i: i32, j: i32) -> u32 {
    return u32((j + NG) * (i32(ni) + 2 * NG) + (i + NG));
}

fn cons_to_prim(q: vec4<f32>) -> vec4<f32> {
    let rho = q.x;
    let u = q.y / rho;
    let v = q.z / rho;
    let p = (GAMMA - 1.0) * (q.w - 0.5 * rho * (u * u + v * v));
    return vec4<f32>(rho, u, v, p);
}

fn sound_speed(w: vec4<f32>) -> f32 {
    return sqrt(GAMMA * w.w / w.x);
}

// Physical Euler flux F.n projected on the unit normal (nx, nr).
fn physical_flux(w: vec4<f32>, nx: f32, nr: f32) -> vec4<f32> {
    let rho = w.x; let u = w.y; let v = w.z; let p = w.w;
    let un = u * nx + v * nr;
    let e = p / (GAMMA - 1.0) + 0.5 * rho * (u * u + v * v);
    return vec4<f32>(rho * un, rho * u * un + p * nx, rho * v * un + p * nr, (e + p) * un);
}

// Velocity reflected about the wall normal (nx, nr).
fn reflect_wall(w: vec4<f32>, nx: f32, nr: f32) -> vec4<f32> {
    let un = w.y * nx + w.z * nr;
    return vec4<f32>(w.x, w.y - 2.0 * un * nx, w.z - 2.0 * un * nr, w.w);
}

// Far-field ghost states (bowshock-cfd `bc.rs`); `w` is the first interior cell, `(nx, nr)` the OUTWARD unit normal.
// Characteristic (Riemann-invariant) far field: the branch is selected by the interior normal Mach number; supersonic
// inflow gives the free stream, supersonic outflow the interior state, in between the 1-D Riemann invariants fix the
// normal velocity and the sound speed, and entropy + tangential velocity are taken from the interior (outflow) or the
// free stream (inflow). The CPU evaluates `rho = (c_b^2 / (gamma s))^(1/(gamma-1))` with `s = p_ref / rho_ref^gamma`;
// since `c_ref^2 = gamma p_ref / rho_ref` this is `rho_ref (c_b / c_ref)^(2/(gamma-1))`, an integer power (5) for
// gamma = 1.4 (checked when the source is generated, `characteristic_exponent`), so no `pow` is needed.
fn characteristic_ghost(fs: vec4<f32>, w: vec4<f32>, nx: f32, nr: f32) -> vec4<f32> {
    let ci = sound_speed(w);
    let uni = w.y * nx + w.z * nr;
    let m = uni / ci;
    if (m <= -1.0) { return fs; }
    if (m >= 1.0) { return w; }
    let ce = sound_speed(fs);
    let une = fs.y * nx + fs.z * nr;
    let r_out = uni + 2.0 * ci / (GAMMA - 1.0);
    let r_in = une - 2.0 * ce / (GAMMA - 1.0);
    let un_b = 0.5 * (r_out + r_in);
    let c_b = max(0.25 * (GAMMA - 1.0) * (r_out - r_in), 1e-8);
    var rf = fs; var un_ref = une; var c_ref = ce;
    if (un_b >= 0.0) { rf = w; un_ref = uni; c_ref = ci; }
    let q = c_b / c_ref;
    let q2 = q * q;
    let rho = rf.x * (q2 * q2 * q);
    let dn = un_b - un_ref;
    return vec4<f32>(rho, rf.y + dn * nx, rf.z + dn * nr, rho * c_b * c_b / GAMMA);
}

// Fixed static pressure `p_back` for subsonic faces, extrapolation for supersonic ones.
fn pressure_outflow_ghost(p_back: f32, w: vec4<f32>, nx: f32, nr: f32) -> vec4<f32> {
    if ((w.y * nx + w.z * nr) / sound_speed(w) >= 1.0) { return w; }
    return vec4<f32>(w.x, w.y, w.z, p_back);
}

// Far-field ghost of boundary condition `bc` (3 characteristic, 4 static-pressure outflow) from the first interior cell.
fn far_ghost(bc: u32, fs: vec4<f32>, edge: vec4<f32>, f: vec4<f32>, outward: f32) -> vec4<f32> {
    if (bc == 3u) { return characteristic_ghost(fs, edge, outward * f.x, outward * f.y); }
    return pressure_outflow_ghost(fs.w, edge, outward * f.x, outward * f.y);
}

// Ghost state of boundary condition `bc` (0 free stream, 1 zero-gradient, 2 slip wall, 3 characteristic far field,
// 4 static-pressure outflow); `f` is the boundary face (nx, nr, len, w), `outward` = +1 if (nx, nr) points out of the
// domain (imax, jmax) and -1 if it points into it (imin, jmin).
fn ghost_state(bc: u32, fs: vec4<f32>, edge: vec4<f32>, mirror: vec4<f32>, f: vec4<f32>, outward: f32) -> vec4<f32> {
    if (bc == 0u) { return fs; }
    if (bc == 1u) { return edge; }
    if (bc == 2u) { return reflect_wall(mirror, f.x, f.y); }
    return far_ghost(bc, fs, edge, f, outward);
}

const VA_EPS: f32 = 1e-14;

// a = q_i - q_{i-1}, b = q_{i+1} - q_i. Face values are q_i -+ phi/2.
fn limited_slope(a: f32, b: f32, lim: u32) -> f32 {
    if (lim == 0u) { return 0.0; }
    if (a * b <= 0.0) { return 0.0; }
    if (lim == 1u) {
        if (abs(a) < abs(b)) { return a; }
        return b;
    }
    return (a * (b * b + VA_EPS) + b * (a * a + VA_EPS)) / (a * a + b * b + 2.0 * VA_EPS);
}

fn limited_slope4(a: vec4<f32>, b: vec4<f32>, lim: u32) -> vec4<f32> {
    return vec4<f32>(
        limited_slope(a.x, b.x, lim), limited_slope(a.y, b.y, lim),
        limited_slope(a.z, b.z, lim), limited_slope(a.w, b.w, lim));
}

struct LeftRight { l: vec4<f32>, r: vec4<f32> }

// MUSCL reconstruction at the face between cells m1 (left) and p0 (right); m2, p1 are the outer
// neighbours. First-order fallback where the reconstructed rho or p is not positive.
fn reconstruct(m2: vec4<f32>, m1: vec4<f32>, p0: vec4<f32>, p1: vec4<f32>, lim: u32) -> LeftRight {
    let phi_l = limited_slope4(m1 - m2, p0 - m1, lim);
    let phi_r = limited_slope4(p0 - m1, p1 - p0, lim);
    var l = m1 + 0.5 * phi_l;
    var r = p0 - 0.5 * phi_r;
    if (l.x <= 0.0 || l.w <= 0.0) { l = m1; }
    if (r.x <= 0.0 || r.w <= 0.0) { r = p0; }
    return LeftRight(l, r);
}

// HLLC flux (Toro ch. 10) with Einfeldt/Roe-averaged wave speeds, per unit face length.
fn hllc(wl: vec4<f32>, wr: vec4<f32>, nx: f32, nr: f32) -> vec4<f32> {
    let rl = wl.x; let ul = wl.y; let vl = wl.z; let pl = wl.w;
    let rr = wr.x; let ur = wr.y; let vr = wr.z; let pr = wr.w;
    let unl = ul * nx + vl * nr;
    let unr = ur * nx + vr * nr;
    let cl = sqrt(GAMMA * pl / rl);
    let cr = sqrt(GAMMA * pr / rr);
    let el = pl / (GAMMA - 1.0) + 0.5 * rl * (ul * ul + vl * vl);
    let er = pr / (GAMMA - 1.0) + 0.5 * rr * (ur * ur + vr * vr);

    // Roe averages
    let sql = sqrt(rl);
    let sqr = sqrt(rr);
    let inv = 1.0 / (sql + sqr);
    let uh = (sql * ul + sqr * ur) * inv;
    let vh = (sql * vl + sqr * vr) * inv;
    let hl = (el + pl) / rl;
    let hr = (er + pr) / rr;
    let hh = (sql * hl + sqr * hr) * inv;
    let ch2 = (GAMMA - 1.0) * (hh - 0.5 * (uh * uh + vh * vh));
    let ch = sqrt(max(ch2, 1e-30));
    let unh = uh * nx + vh * nr;

    let sl = min(unl - cl, unh - ch);
    let sr = max(unr + cr, unh + ch);

    if (sl >= 0.0) { return physical_flux(wl, nx, nr); }
    if (sr <= 0.0) { return physical_flux(wr, nx, nr); }

    let sm = (pr - pl + rl * unl * (sl - unl) - rr * unr * (sr - unr))
        / (rl * (sl - unl) - rr * (sr - unr));

    if (sm >= 0.0) {
        let f = physical_flux(wl, nx, nr);
        let fac = rl * (sl - unl) / (sl - sm);
        let us = vec4<f32>(
            fac,
            fac * (ul + (sm - unl) * nx),
            fac * (vl + (sm - unl) * nr),
            fac * (el / rl + (sm - unl) * (sm + pl / (rl * (sl - unl)))));
        let q = vec4<f32>(rl, rl * ul, rl * vl, el);
        return f + sl * (us - q);
    } else {
        let f = physical_flux(wr, nx, nr);
        let fac = rr * (sr - unr) / (sr - sm);
        let us = vec4<f32>(
            fac,
            fac * (ur + (sm - unr) * nx),
            fac * (vr + (sm - unr) * nr),
            fac * (er / rr + (sm - unr) * (sm + pr / (rr * (sr - unr)))));
        let q = vec4<f32>(rr, rr * ur, rr * vr, er);
        return f + sr * (us - q);
    }
}

// Rusanov flux 0.5 (F_L + F_R) - 0.5 smax (U_R - U_L), smax = max(|un_L| + c_L, |un_R| + c_R).
fn rusanov(wl: vec4<f32>, wr: vec4<f32>, nx: f32, nr: f32) -> vec4<f32> {
    let unl = wl.y * nx + wl.z * nr;
    let unr = wr.y * nx + wr.z * nr;
    let smax = max(abs(unl) + sqrt(GAMMA * wl.w / wl.x), abs(unr) + sqrt(GAMMA * wr.w / wr.x));
    let fl = physical_flux(wl, nx, nr);
    let fr = physical_flux(wr, nx, nr);
    let ql = vec4<f32>(wl.x, wl.x * wl.y, wl.x * wl.z, wl.w / (GAMMA - 1.0) + 0.5 * wl.x * (wl.y * wl.y + wl.z * wl.z));
    let qr = vec4<f32>(wr.x, wr.x * wr.y, wr.x * wr.z, wr.w / (GAMMA - 1.0) + 0.5 * wr.x * (wr.y * wr.y + wr.z * wr.z));
    return 0.5 * (fl + fr) - 0.5 * smax * (qr - ql);
}

// Weight of the Rusanov flux: smoothstep of the relative cell-to-cell pressure jump.
fn shock_weight(pl: f32, pr: f32) -> f32 {
    let s = abs(pr - pl) / min(pl, pr);
    let t = clamp((s - SHOCK_SENSOR_LO) / (SHOCK_SENSOR_HI - SHOCK_SENSOR_LO), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// Low-Mach correction of the reconstructed states (bowshock-cfd `flux::low_mach_states`, Thornber et al. 2008): the jump of the
// face-normal velocity is scaled by z = clamp(max(M_L, M_R), LOW_MACH_FLOOR, 1); applied when `p.low_mach != 0`.
fn low_mach_states(l: vec4<f32>, r: vec4<f32>, nx: f32, nr: f32) -> LeftRight {
    let unl = l.y * nx + l.z * nr;
    let unr = r.y * nx + r.z * nr;
    let ml = sqrt(l.y * l.y + l.z * l.z) / sqrt(GAMMA * l.w / l.x);
    let mr = sqrt(r.y * r.y + r.z * r.z) / sqrt(GAMMA * r.w / r.x);
    let z = clamp(max(ml, mr), LOW_MACH_FLOOR, 1.0);
    if (z >= 1.0) { return LeftRight(l, r); }
    let avg = 0.5 * (unl + unr);
    let dl = avg + z * (unl - avg) - unl;
    let dr = avg + z * (unr - avg) - unr;
    return LeftRight(vec4<f32>(l.x, l.y + dl * nx, l.z + dl * nr, l.w), vec4<f32>(r.x, r.y + dr * nx, r.z + dr * nr, r.w));
}

fn reconstruct_lm(m2: vec4<f32>, m1: vec4<f32>, p0: vec4<f32>, p1: vec4<f32>, lim: u32, nx: f32, nr: f32) -> LeftRight {
    let lr = reconstruct(m2, m1, p0, p1, lim);
    if (p.low_mach != 0u) { return low_mach_states(lr.l, lr.r, nx, nr); }
    return lr;
}

// Complete face kernel (bowshock-cfd `flux::face_flux_lm`): MUSCL reconstruction (+ optional low-Mach correction) + HLLC blended towards
// Rusanov inside shocks; the sensor uses the two cell-centred pressures adjacent to the face.
fn face_flux(m2: vec4<f32>, m1: vec4<f32>, p0: vec4<f32>, p1: vec4<f32>, lim: u32, nx: f32, nr: f32) -> vec4<f32> {
    let lr = reconstruct_lm(m2, m1, p0, p1, lim, nx, nr);
    let c = hllc(lr.l, lr.r, nx, nr);
    let t = shock_weight(m1.w, p0.w);
    if (t == 0.0) { return c; }
    let f = rusanov(lr.l, lr.r, nx, nr);
    return (1.0 - t) * c + t * f;
}

// ---- compensated (double-float) state: value = hi + lo, |lo| <= ulp(hi)/2 ----
struct DF { hi: vec4<f32>, lo: vec4<f32> }

// Identity that the compiler cannot see through (`p.zero` is a runtime 0). The GPU compiler reassociates float
// arithmetic (`(a + b) - a` becomes `b`), which silently destroys error-free transformations (two-sum); routing the
// sum `s = a + b` (and `s - a`) and product operands through an integer OR keeps them intact (it also prevents
// a product feeding the sum from being fused into an fma, which would count its rounding error twice).
fn opq(x: f32) -> f32 { return bitcast<f32>(bitcast<u32>(x) | p.zero); }
fn opq4(x: vec4<f32>) -> vec4<f32> { return bitcast<vec4<f32>>(bitcast<vec4<u32>>(x) | vec4<u32>(p.zero)); }

// (hi + lo) + inc for a small increment `inc`: two-sum of hi and inc, lo folded into the error term,
// renormalised.
fn df_add(hi: vec4<f32>, lo: vec4<f32>, inc_in: vec4<f32>) -> DF {
    let inc = opq4(inc_in); // a product `dt * res` must not be contracted into the sum (fma) either
    let s = opq4(hi + inc);
    let bb = opq4(s - hi);
    let t1 = opq4(s - bb);
    let e = (opq4(hi - t1) + opq4(inc - bb)) + lo;
    let h = opq4(s + e);
    return DF(h, e - opq4(h - s));
}
"#;

/// Exponent `2 / (gamma - 1)` of `c_b / c_ref` in the density of the characteristic far-field ghost state. The WGSL
/// `characteristic_ghost` evaluates it as the integer power 5.
pub fn characteristic_exponent() -> f64 {
    2.0 / (bowshock_cfd::gas::GAMMA - 1.0)
}

/// Common WGSL prelude of every kernel: the `Params` block and the direct translations of bowshock-cfd's
/// `gas.rs` / `flux.rs` / `bc.rs` functions. The shock-sensor thresholds are taken from `bowshock_cfd::flux`.
pub fn common() -> String {
    assert!((characteristic_exponent() - 5.0).abs() < 1e-12, "characteristic_ghost assumes gamma = 1.4 (integer power 5)");
    format!(
        "const SHOCK_SENSOR_LO: f32 = {:e};\nconst SHOCK_SENSOR_HI: f32 = {:e};\nconst LOW_MACH_FLOOR: f32 = {:e};\n{}",
        bowshock_cfd::flux::SHOCK_SENSOR_LO,
        bowshock_cfd::flux::SHOCK_SENSOR_HI,
        bowshock_cfd::flux::LOW_MACH_FLOOR,
        COMMON_TEMPLATE
    )
}

/// Pressure of the two first cells behind the base face (`i = blk_i`, `blk_i + 1`) of every row `blk_s + y`, `y < blk_j - blk_s`
/// (`bowshock_cfd::subsonic::base_state`); with `comp` the low energy word is added. Problem size `[blk_j - blk_s]`.
pub fn base_pressure(comp: bool) -> String {
    let (bind, out_bind, e0, e1) = if comp {
        (
            "@group(1) @binding(1) var<storage, read> ulo: array<vec4<f32>>;\n",
            "@group(1) @binding(2) var<storage, read_write> out: array<vec4<f32>>;\n@group(1) @binding(3) var<uniform> p: Params;",
            "ulo[c0].w",
            "ulo[c0 + 1u].w",
        )
    } else {
        ("", "@group(1) @binding(1) var<storage, read_write> out: array<vec4<f32>>;\n@group(1) @binding(2) var<uniform> p: Params;", "0.0", "0.0")
    };
    format!(
        "@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;\n{bind}{out_bind}
fn pres(q: vec4<f32>, elo: f32) -> f32 {{
    let ke = 0.5 * (q.y * q.y + q.z * q.z) / q.x;
    return (GAMMA - 1.0) * ((q.w - ke) + elo);
}}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {{
    let y = gid.x;
    if (y >= p.blk_j - p.blk_s) {{ return; }}
    let c0 = (p.blk_s + y) * p.ni + p.blk_i;
    out[y] = vec4<f32>(pres(u[c0], {e0}), pres(u[c0 + 1u], {e1}), 0.0, 0.0);
}}
"
    )
}

/// cons -> prim for every interior cell (bowshock-cfd `eval_residual` step 1).
/// Problem size `[ni, nj]`.
pub const PRIM: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read_write> w: array<vec4<f32>>;
@group(1) @binding(2) var<uniform> p: Params;

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    w[pidx(p.ni, i32(gid.x), i32(gid.y))] = cons_to_prim(u[gid.y * p.ni + gid.x]);
}
"#;

/// Ghost-layer fill (bowshock-cfd `fill_ghosts`). Problem size `[nj + ni]`: thread `t < nj` fills both
/// i-sides of row `t`, thread `t >= nj` both j-sides of column `t - nj`. 2 ghost layers per side,
/// no corner ghosts. Face arrays are `(nx, nr, len, w)` per face.
pub const GHOST: &str = r#"
@group(1) @binding(0) var<storage, read_write> w: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let t = gid.x;
    let fs = vec4<f32>(p.fs_rho, p.fs_u, p.fs_v, p.fs_p);
    let ni = i32(p.ni);
    let nj = i32(p.nj);
    if (t < p.nj) {
        let j = i32(t);
        for (var k: i32 = 1; k <= NG; k++) {
            var f = iface[t * (p.ni + 1u)];
            var edge = w[pidx(p.ni, 0, j)];
            var mir = w[pidx(p.ni, k - 1, j)];
            w[pidx(p.ni, -k, j)] = ghost_state(p.bc0, fs, edge, mir, f, -1.0);
            f = iface[t * (p.ni + 1u) + p.ni];
            edge = w[pidx(p.ni, ni - 1, j)];
            mir = w[pidx(p.ni, ni - k, j)];
            w[pidx(p.ni, ni + k - 1, j)] = ghost_state(p.bc1, fs, edge, mir, f, 1.0);
        }
    } else if (t < p.nj + p.ni) {
        let iu = t - p.nj;
        let i = i32(iu);
        for (var k: i32 = 1; k <= NG; k++) {
            var f = jface[iu];
            var edge = w[pidx(p.ni, i, 0)];
            var mir = w[pidx(p.ni, i, k - 1)];
            w[pidx(p.ni, i, -k)] = ghost_state(p.bc2, fs, edge, mir, f, -1.0);
            f = jface[p.nj * p.ni + iu];
            edge = w[pidx(p.ni, i, nj - 1)];
            mir = w[pidx(p.ni, i, nj - k)];
            w[pidx(p.ni, i, nj + k - 1)] = ghost_state(p.bc3, fs, edge, mir, f, 1.0);
        }
    }
}
"#;

/// Face flux (MUSCL + HLLC/Rusanov blend) through every i-face. Problem size `[ni + 1, nj]`.
pub const FLUX_I: &str = r#"
@group(1) @binding(0) var<storage, read> w: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read_write> fi: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x > p.ni || gid.y >= p.nj) { return; }
    let i = i32(gid.x);
    let j = i32(gid.y);
    let k = gid.y * (p.ni + 1u) + gid.x;
    let f = iface[k];
    fi[k] = face_flux(
        w[pidx(p.ni, i - 2, j)], w[pidx(p.ni, i - 1, j)],
        w[pidx(p.ni, i, j)], w[pidx(p.ni, i + 1, j)], p.limiter, f.x, f.y);
}
"#;

/// Face flux (MUSCL + HLLC/Rusanov blend) through every j-face. Problem size `[ni, nj + 1]`.
pub const FLUX_J: &str = r#"
@group(1) @binding(0) var<storage, read> w: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read_write> fj: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y > p.nj) { return; }
    let i = i32(gid.x);
    let j = i32(gid.y);
    let k = gid.y * p.ni + gid.x;
    let f = jface[k];
    fj[k] = face_flux(
        w[pidx(p.ni, i, j - 2)], w[pidx(p.ni, i, j - 1)],
        w[pidx(p.ni, i, j)], w[pidx(p.ni, i, j + 1)], p.limiter, f.x, f.y);
}
"#;

/// Cell residual `dU/dt = -(1/V) sum_f w_f F_f + (0, 0, p A / V, 0)` and (stage 1 only) the local
/// time step and the density-residual array. `cell` is `(A, V, x_c, r_c)`. Problem size `[ni, nj]`.
pub const RESIDUAL: &str = r#"
@group(1) @binding(0) var<storage, read> w: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> fi: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> fj: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read> cell: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read_write> res: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read_write> dt: array<f32>;
@group(1) @binding(8) var<storage, read_write> rho_res: array<f32>;
@group(1) @binding(9) var<uniform> p: Params;

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    let i = gid.x;
    let j = gid.y;
    let c = j * p.ni + i;
    let fl = iface[j * (p.ni + 1u) + i];
    let fr = iface[j * (p.ni + 1u) + i + 1u];
    let fb = jface[j * p.ni + i];
    let ft = jface[(j + 1u) * p.ni + i];
    let gl = fi[j * (p.ni + 1u) + i];
    let gr = fi[j * (p.ni + 1u) + i + 1u];
    let gb = fj[j * p.ni + i];
    let gt = fj[(j + 1u) * p.ni + i];
    let wc = w[pidx(p.ni, i32(i), i32(j))];
    let geo = cell[c];
    let inv_v = 1.0 / geo.y;
    var out = -(gr * fr.w - gl * fl.w + gt * ft.w - gb * fb.w) * inv_v;
    out.z = out.z + wc.w * geo.x * inv_v; // axisymmetric pressure source p*A/V
    res[c] = out;
    if (p.stage == 1u) {
        rho_res[c] = out.x;
        let cs = sound_speed(wc);
        let lam_l = (abs(wc.y * fl.x + wc.z * fl.y) + cs) * fl.z;
        let lam_r = (abs(wc.y * fr.x + wc.z * fr.y) + cs) * fr.z;
        let lam_b = (abs(wc.y * fb.x + wc.z * fb.y) + cs) * fb.z;
        let lam_t = (abs(wc.y * ft.x + wc.z * ft.y) + cs) * ft.z;
        let sum = lam_l + lam_r + lam_b + lam_t;
        dt[c] = p.cfl * geo.x / (0.5 * sum);
    }
}
"#;

/// SSP-RK3 stage update with local time step. Problem size `[ni, nj]`. `comp`: compensated (hi + lo) state.
/// stage 1: `u0 = u; u += dt res`. stages 2, 3: `u = ra (u + dt res) + rb u0` (bowshock-cfd `Solver::stage`).
/// Compensated: `u` / `u0` are `hi + lo` pairs and the stage is evaluated in increment form
/// `u = u0 + ra ((u - u0) + dt res)` (`ra + rb = 1`), so that only small numbers go through f32 arithmetic and the
/// sum with `u0` is a double-float addition.
pub fn update(comp: bool) -> String {
    let (bind, s1, s2) = if comp {
        (
            "@group(1) @binding(4) var<storage, read_write> ulo: array<vec4<f32>>;\n\
             @group(1) @binding(5) var<storage, read_write> u0lo: array<vec4<f32>>;\n\
             @group(1) @binding(6) var<uniform> p: Params;",
            "let qh = u[c]; let ql = ulo[c];\n\
             u0[c] = qh; u0lo[c] = ql;\n\
             let r = df_add(qh, ql, dt[c] * res[c]);\n\
             u[c] = r.hi; ulo[c] = r.lo;",
            "let d = (u[c] - u0[c]) + (ulo[c] - u0lo[c]);\n\
             let r = df_add(u0[c], u0lo[c], p.ra * (d + dt[c] * res[c]));\n\
             u[c] = r.hi; ulo[c] = r.lo;",
        )
    } else {
        (
            "@group(1) @binding(4) var<uniform> p: Params;",
            "let q = u[c];\n u0[c] = q;\n u[c] = q + dt[c] * res[c];",
            "u[c] = p.ra * (u[c] + dt[c] * res[c]) + p.rb * u0[c];",
        )
    };
    UPDATE_TEMPLATE.replace("//@BINDINGS", bind).replace("//@STAGE1", s1).replace("//@STAGE2", s2)
}

const UPDATE_TEMPLATE: &str = r#"
@group(1) @binding(0) var<storage, read_write> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read_write> u0: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> res: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> dt: array<f32>;
//@BINDINGS

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    let c = gid.y * p.ni + gid.x;
    if (p.stage == 1u) {
        //@STAGE1
    } else {
        //@STAGE2
    }
}
"#;

/// Residual history, stage A (direct mode): `partial[g] = sum of rho_res^2` over a strided slice of
/// the cells. Launched with 128 workgroups of 256 threads (problem size `[128 * 256]`).
pub const HIST_PARTIAL: &str = r#"
@group(1) @binding(0) var<storage, read_write> partial: array<f32>;
@group(1) @binding(1) var<storage, read> rho_res: array<f32>;
@group(1) @binding(2) var<uniform> p: Params;

var<workgroup> sh: array<f32, 256>;

@compute @workgroup_size(256)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(local_invocation_id) lid: vec3<u32>,
        @builtin(workgroup_id) wid: vec3<u32>) {
    let n = p.ni * p.nj;
    var s: f32 = 0.0;
    for (var c: u32 = gid.x; c < n; c += 128u * 256u) {
        let r = rho_res[c];
        s += r * r;
    }
    sh[lid.x] = s;
    workgroupBarrier();
    for (var stride: u32 = 128u; stride > 0u; stride = stride >> 1u) {
        if (lid.x < stride) { sh[lid.x] += sh[lid.x + stride]; }
        workgroupBarrier();
    }
    if (lid.x == 0u) { partial[wid.x] = sh[0]; }
}
"#;

/// Residual history, stage B: `hist[slot] = sum_{g < aux} partial[g]` (one workgroup). The host
/// divides by N and takes the square root.
pub const HIST_FINAL: &str = r#"
@group(1) @binding(0) var<storage, read_write> hist: array<f32>;
@group(1) @binding(1) var<storage, read> partial: array<f32>;
@group(1) @binding(2) var<uniform> p: Params;

var<workgroup> sh: array<f32, 256>;

@compute @workgroup_size(256)
fn main(@builtin(local_invocation_id) lid: vec3<u32>) {
    var s: f32 = 0.0;
    for (var g: u32 = lid.x; g < p.aux; g += 256u) {
        s += partial[g];
    }
    sh[lid.x] = s;
    workgroupBarrier();
    for (var stride: u32 = 128u; stride > 0u; stride = stride >> 1u) {
        if (lid.x < stride) { sh[lid.x] += sh[lid.x + stride]; }
        workgroupBarrier();
    }
    if (lid.x == 0u) { hist[p.slot] = sh[0]; }
}
"#;

/// Guarded (ghost-free) formulation, kernel 1 of 2 per RK stage: face flux (MUSCL + HLLC/Rusanov blend) through every
/// i-face and j-face. The primitive states of the stencil are computed from `u` on the fly and the
/// ghost states of boundary faces are evaluated lazily from the interior cells (same BC logic as
/// `GHOST`). Problem size `[ni + 1, 2 nj + 1]`: row `y < nj` is i-face row `y`, row `y >= nj` is
/// j-face row `y - nj`.
pub const FLUX_GUARDED: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read_write> fi: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read_write> fj: array<vec4<f32>>;
@group(1) @binding(5) var<uniform> p: Params;

fn fs_state() -> vec4<f32> { return vec4<f32>(p.fs_rho, p.fs_u, p.fs_v, p.fs_p); }

fn cell_prim(i: i32, j: i32) -> vec4<f32> {
    return cons_to_prim(u[u32(j) * p.ni + u32(i)]);
}

// Cell (i, j) with i in [-2, ni + 1] and j interior: interior state or lazily evaluated ghost.
fn prim_i(i: i32, j: i32) -> vec4<f32> {
    let ni = i32(p.ni);
    if (i < 0) {
        let k = -i;
        let f = iface[u32(j) * (p.ni + 1u)];
        if (p.bc0 == 0u) { return fs_state(); }
        if (p.bc0 == 1u) { return cell_prim(0, j); }
        if (p.bc0 == 2u) { return reflect_wall(cell_prim(k - 1, j), f.x, f.y); }
        return far_ghost(p.bc0, fs_state(), cell_prim(0, j), f, -1.0);
    }
    if (i >= ni) {
        let k = i - ni + 1;
        let f = iface[u32(j) * (p.ni + 1u) + p.ni];
        if (p.bc1 == 0u) { return fs_state(); }
        if (p.bc1 == 1u) { return cell_prim(ni - 1, j); }
        if (p.bc1 == 2u) { return reflect_wall(cell_prim(ni - k, j), f.x, f.y); }
        return far_ghost(p.bc1, fs_state(), cell_prim(ni - 1, j), f, 1.0);
    }
    return cell_prim(i, j);
}

// Cell (i, j) with j in [-2, nj + 1] and i interior. Cells below the body / sting surface of a solid block (j < col_start(i),
// `bowshock_cfd::solver::Solver::fill_block_ghosts`): mirror image of the fluid cells above the surface.
fn prim_j(i: i32, j: i32) -> vec4<f32> {
    let nj = i32(p.nj);
    let j0 = i32(col_start(u32(i)));
    if (j < j0 && j0 > 0) {
        let f = jface[u32(j0) * p.ni + u32(i)];
        return reflect_wall(cell_prim(i, 2 * j0 - 1 - j), f.x, f.y);
    }
    if (j < 0) {
        let k = -j;
        let f = jface[u32(i)];
        if (p.bc2 == 0u) { return fs_state(); }
        if (p.bc2 == 1u) { return cell_prim(i, 0); }
        if (p.bc2 == 2u) { return reflect_wall(cell_prim(i, k - 1), f.x, f.y); }
        return far_ghost(p.bc2, fs_state(), cell_prim(i, 0), f, -1.0);
    }
    if (j >= nj) {
        let k = j - nj + 1;
        let f = jface[p.nj * p.ni + u32(i)];
        if (p.bc3 == 0u) { return fs_state(); }
        if (p.bc3 == 1u) { return cell_prim(i, nj - 1); }
        if (p.bc3 == 2u) { return reflect_wall(cell_prim(i, nj - k), f.x, f.y); }
        return far_ghost(p.bc3, fs_state(), cell_prim(i, nj - 1), f, 1.0);
    }
    return cell_prim(i, j);
}

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.y < p.nj) {
        if (gid.x > p.ni) { return; }
        if (gid.y < p.blk_j) {
            if (gid.y < p.blk_s || gid.x < p.blk_i) { fi[gid.y * (p.ni + 1u) + gid.x] = vec4<f32>(0.0); return; } // inside the body / sting
            if (gid.x <= p.blk_i + 1u) {
                // base wall (i = blk_i) and the face next to it: the solid stencil cells are the mirror image of the fluid cells
                // across the base about the base-wall normal
                let i = i32(gid.x);
                let j = i32(gid.y);
                let fb = iface[gid.y * (p.ni + 1u) + p.blk_i];
                let f = iface[gid.y * (p.ni + 1u) + gid.x];
                let p0 = prim_i(i, j);
                let p1 = prim_i(i + 1, j);
                var m1 = prim_i(i - 1, j);
                var m2 = reflect_wall(m1, fb.x, fb.y);
                if (gid.x == p.blk_i) {
                    m1 = reflect_wall(p0, fb.x, fb.y);
                    m2 = reflect_wall(p1, fb.x, fb.y);
                }
                fi[gid.y * (p.ni + 1u) + gid.x] = face_flux(m2, m1, p0, p1, p.limiter, f.x, f.y);
                return;
            }
        }
        let i = i32(gid.x);
        let j = i32(gid.y);
        let k = gid.y * (p.ni + 1u) + gid.x;
        let f = iface[k];
        fi[k] = face_flux(prim_i(i - 2, j), prim_i(i - 1, j), prim_i(i, j), prim_i(i + 1, j), p.limiter, f.x, f.y);
    } else {
        let jr = gid.y - p.nj;
        if (gid.x >= p.ni || jr > p.nj) { return; }
        if (jr < col_start(gid.x)) { fj[jr * p.ni + gid.x] = vec4<f32>(0.0); return; } // inside the body / sting
        let i = i32(gid.x);
        let j = i32(jr);
        let k = jr * p.ni + gid.x;
        let f = jface[k];
        fj[k] = face_flux(prim_j(i, j - 2), prim_j(i, j - 1), prim_j(i, j), prim_j(i, j + 1), p.limiter, f.x, f.y);
    }
}
"#;

/// Guarded formulation, kernel 2 of 2 per RK stage: cell residual (as `RESIDUAL`) fused with the
/// SSP-RK3 stage update (as `UPDATE`); the residual is not stored. In stage 1 it also writes the
/// local time step, `rho_res` and the per-workgroup sum of `rho_res^2` into `partial` (consumed
/// by `HIST_FINAL`). Problem size `[ni, nj]`.
pub fn residual_update(comp: bool) -> String {
    let (bind, s1, s2) = if comp {
        (
            "@group(1) @binding(10) var<storage, read_write> ulo: array<vec4<f32>>;\n\
             @group(1) @binding(11) var<storage, read_write> u0lo: array<vec4<f32>>;\n\
             @group(1) @binding(12) var<uniform> p: Params;",
            "let ql = ulo[c];\n\
             u0[c] = q; u0lo[c] = ql;\n\
             let r = df_add(q, ql, dtc * res);\n\
             u[c] = r.hi; ulo[c] = r.lo;",
            "let d = (q - u0[c]) + (ulo[c] - u0lo[c]);\n\
             let r = df_add(u0[c], u0lo[c], p.ra * (d + dt[c] * res));\n\
             u[c] = r.hi; ulo[c] = r.lo;",
        )
    } else {
        (
            "@group(1) @binding(10) var<uniform> p: Params;",
            "u0[c] = q;\n u[c] = q + dtc * res;",
            "u[c] = p.ra * (q + dt[c] * res) + p.rb * u0[c];",
        )
    };
    RESIDUAL_UPDATE_TEMPLATE.replace("//@BINDINGS", bind).replace("//@STAGE1", s1).replace("//@STAGE2", s2)
}

const RESIDUAL_UPDATE_TEMPLATE: &str = r#"
@group(1) @binding(0) var<storage, read_write> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read_write> u0: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> fi: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> fj: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(6) var<storage, read> cell: array<vec4<f32>>;
@group(1) @binding(7) var<storage, read_write> dt: array<f32>;
@group(1) @binding(8) var<storage, read_write> rho_res: array<f32>;
@group(1) @binding(9) var<storage, read_write> partial: array<f32>;
//@BINDINGS

var<workgroup> sh: array<f32, 128>;

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(local_invocation_index) li: u32,
        @builtin(workgroup_id) wid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    var r2: f32 = 0.0;
    if (gid.x < p.ni && gid.y < p.nj && !is_solid(gid.x, gid.y)) {
        let i = gid.x;
        let j = gid.y;
        let c = j * p.ni + i;
        let q = u[c];
        let wc = cons_to_prim(q);
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
        var res = -(gr * fr.w - gl * fl.w + gt * ft.w - gb * fb.w) * inv_v;
        res.z = res.z + wc.w * geo.x * inv_v; // axisymmetric pressure source p*A/V
        if (p.stage == 1u) {
            rho_res[c] = res.x;
            r2 = res.x * res.x;
            let cs = sound_speed(wc);
            let lam_l = (abs(wc.y * fl.x + wc.z * fl.y) + cs) * fl.z;
            let lam_r = (abs(wc.y * fr.x + wc.z * fr.y) + cs) * fr.z;
            let lam_b = (abs(wc.y * fb.x + wc.z * fb.y) + cs) * fb.z;
            let lam_t = (abs(wc.y * ft.x + wc.z * ft.y) + cs) * ft.z;
            let sum = lam_l + lam_r + lam_b + lam_t;
            let dtc = p.cfl * geo.x / (0.5 * sum);
            dt[c] = dtc;
            //@STAGE1
        } else {
            //@STAGE2
        }
    }
    if (p.stage == 1u) {
        sh[li] = r2;
        workgroupBarrier();
        for (var stride: u32 = 64u; stride > 0u; stride = stride >> 1u) {
            if (li < stride) { sh[li] += sh[li + stride]; }
            workgroupBarrier();
        }
        if (li == 0u) { partial[wid.y * nwg.x + wid.x] = sh[0]; }
    }
}
"#;

/// Test kernel: MUSCL reconstruction, HLLC and the complete face flux for `p.ni` independent samples.
/// `states` holds `(m2, m1, p0, p1)` per sample, `normals` the face normal; `out` holds
/// `(left, right, hllc(left, right), face_flux)`.
pub const FACE_TEST: &str = r#"
@group(1) @binding(0) var<storage, read> states: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> normals: array<vec2<f32>>;
@group(1) @binding(2) var<storage, read_write> out: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let s = gid.x;
    if (s >= p.ni) { return; }
    let m2 = states[4u * s]; let m1 = states[4u * s + 1u]; let p0 = states[4u * s + 2u]; let p1 = states[4u * s + 3u];
    let n = normals[s];
    let lr = reconstruct_lm(m2, m1, p0, p1, p.limiter, n.x, n.y);
    out[4u * s] = lr.l;
    out[4u * s + 1u] = lr.r;
    out[4u * s + 2u] = hllc(lr.l, lr.r, n.x, n.y);
    out[4u * s + 3u] = face_flux(m2, m1, p0, p1, p.limiter, n.x, n.y);
}
"#;
