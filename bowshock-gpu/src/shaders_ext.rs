//! Extended-precision (double-float, "df") WGSL for [`crate::Precision::Extended`].
//!
//! A df number is an unevaluated sum `h + l` of two f32 with `|l| <= ulp(h)/2` (about 48 significant bits). The face
//! flux (`face_flux` of bowshock-cfd: MUSCL reconstruction, HLLC, Rusanov blend, shock sensor), the primitive variables of
//! the stencil and the cell residual sum are evaluated in df, from a double-float state (`u` + `ulo`), so that the
//! residual is accurate to ~1e-12 relative instead of the ~1e-7 of f32 arithmetic and f32-rounded states. Only
//! geometry (face normals, weights, cell areas/volumes: f32 inputs), the time step and the final residual that enters
//! the increment `dt * res` stay f32.
//!
//! The functions mirror `COMMON` in [`crate::shaders`] one to one; names carry a `d` prefix.

/// Error-free df arithmetic plus the df translation of the face kernels. Appended to `shaders::common()`.
const DD_TEMPLATE: &str = r#"
struct D { h: f32, l: f32 }
struct W { r: D, u: D, v: D, p: D }
struct C { a: D, b: D, c: D, d: D }
struct LR { l: W, r: W }

fn dc(x: f32) -> D { return D(x, 0.0); }
fn qts(a: f32, b: f32) -> D { let s = opq(a + b); return D(s, b - opq(s - a)); }
fn ts(a: f32, b: f32) -> D {
    let s = opq(a + b);
    let bb = opq(s - a);
    let t1 = opq(s - bb);
    return D(s, opq(a - t1) + opq(b - bb));
}
fn tp(a: f32, b: f32) -> D { let p = opq(a * b); return D(p, fma(a, b, -p)); }
fn dneg(a: D) -> D { return D(-a.h, -a.l); }
fn dadd(a: D, b: D) -> D {
    let s = ts(a.h, b.h);
    return qts(s.h, s.l + (a.l + b.l));
}
fn dsub(a: D, b: D) -> D { return dadd(a, dneg(b)); }
fn dmul(a: D, b: D) -> D { let p = tp(a.h, b.h); return qts(p.h, p.l + (a.h * b.l + a.l * b.h)); }
fn dmulf(a: D, b: f32) -> D { let p = tp(a.h, b); return qts(p.h, p.l + a.l * b); }
fn ddiv(a: D, b: D) -> D {
    let q1 = a.h / b.h;
    let r1 = dsub(a, dmulf(b, q1));
    return qts(q1, r1.h / b.h);
}
fn dsqrt(a: D) -> D {
    if (a.h <= 0.0) { return dc(0.0); }
    let s = sqrt(a.h);
    return qts(s, dsub(a, tp(s, s)).h / (2.0 * s));
}
fn dlt(a: D, b: D) -> bool { return a.h < b.h || (a.h == b.h && a.l < b.l); }
fn dmin(a: D, b: D) -> D { if (dlt(b, a)) { return b; } return a; }
fn dmax(a: D, b: D) -> D { if (dlt(a, b)) { return b; } return a; }
fn dabs(a: D) -> D { if (a.h < 0.0) { return dneg(a); } return a; }

// Constants rounded from f64 (see `shaders_ext::common_ext`).
fn d_gamma() -> D { return D(GAMMA_H, GAMMA_L); }
fn d_gm1() -> D { return D(GM1_H, GM1_L); }
fn d_igm1() -> D { return D(IGM1_H, IGM1_L); }
fn d_va_eps() -> D { return D(VAEPS_H, VAEPS_L); }
fn d_sens_lo() -> D { return D(SLO_H, SLO_L); }
fn d_sens_inv() -> D { return D(SINV_H, SINV_L); }

fn w_from_f32(w: vec4<f32>) -> W { return W(dc(w.x), dc(w.y), dc(w.z), dc(w.w)); }

// cons -> prim of a df conserved state (hi, lo).
fn dcons_to_prim(qh: vec4<f32>, ql: vec4<f32>) -> W {
    let r = D(qh.x, ql.x);
    let u = ddiv(D(qh.y, ql.y), r);
    let v = ddiv(D(qh.z, ql.z), r);
    let k = dmulf(dmul(r, dadd(dmul(u, u), dmul(v, v))), 0.5);
    return W(r, u, v, dmul(d_gm1(), dsub(D(qh.w, ql.w), k)));
}

fn dun(w: W, nx: f32, nr: f32) -> D { return dadd(dmulf(w.u, nx), dmulf(w.v, nr)); }
fn dsound(w: W) -> D { return dsqrt(ddiv(dmul(d_gamma(), w.p), w.r)); }
fn dener(w: W) -> D { return dadd(dmul(w.p, d_igm1()), dmulf(dmul(w.r, dadd(dmul(w.u, w.u), dmul(w.v, w.v))), 0.5)); }
fn dcons(w: W, e: D) -> C { return C(w.r, dmul(w.r, w.u), dmul(w.r, w.v), e); }

fn dphys(w: W, nx: f32, nr: f32) -> C {
    let un = dun(w, nx, nr);
    let rho_un = dmul(w.r, un);
    return C(rho_un, dadd(dmul(w.u, rho_un), dmulf(w.p, nx)), dadd(dmul(w.v, rho_un), dmulf(w.p, nr)),
             dmul(dadd(dener(w), w.p), un));
}

fn dreflect_wall(w: W, nx: f32, nr: f32) -> W {
    let un = dun(w, nx, nr);
    return W(w.r, dsub(w.u, dmulf(un, 2.0 * nx)), dsub(w.v, dmulf(un, 2.0 * nr)), w.p);
}

// df translations of `characteristic_ghost`, `pressure_outflow_ghost`, `far_ghost` of `COMMON` (bowshock-cfd `bc.rs`);
// `w` is the first interior cell, `(nx, nr)` the outward unit normal. `rho = rho_ref (c_b / c_ref)^5` (see COMMON).
fn dcharacteristic_ghost(fs: W, w: W, nx: f32, nr: f32) -> W {
    let ci = dsound(w);
    let uni = dun(w, nx, nr);
    let m = ddiv(uni, ci);
    if (!dlt(dc(-1.0), m)) { return fs; }
    if (!dlt(m, dc(1.0))) { return w; }
    let ce = dsound(fs);
    let une = dun(fs, nx, nr);
    let k2 = dmulf(d_igm1(), 2.0);
    let r_out = dadd(uni, dmul(ci, k2));
    let r_in = dsub(une, dmul(ce, k2));
    let un_b = dmulf(dadd(r_out, r_in), 0.5);
    let c_b = dmax(dmulf(dmul(d_gm1(), dsub(r_out, r_in)), 0.25), dc(1e-8));
    var rf = fs; var un_ref = une; var c_ref = ce;
    if (!dlt(un_b, dc(0.0))) { rf = w; un_ref = uni; c_ref = ci; }
    let q = ddiv(c_b, c_ref);
    let q2 = dmul(q, q);
    let rho = dmul(rf.r, dmul(dmul(q2, q2), q));
    let dn = dsub(un_b, un_ref);
    return W(rho, dadd(rf.u, dmulf(dn, nx)), dadd(rf.v, dmulf(dn, nr)), ddiv(dmul(dmul(rho, c_b), c_b), d_gamma()));
}

fn dpressure_outflow_ghost(p_back: D, w: W, nx: f32, nr: f32) -> W {
    if (!dlt(ddiv(dun(w, nx, nr), dsound(w)), dc(1.0))) { return w; }
    return W(w.r, w.u, w.v, p_back);
}

fn dfar_ghost(bc: u32, fs: W, edge: W, f: vec4<f32>, outward: f32) -> W {
    if (bc == 3u) { return dcharacteristic_ghost(fs, edge, outward * f.x, outward * f.y); }
    return dpressure_outflow_ghost(fs.p, edge, outward * f.x, outward * f.y);
}

// a = q_i - q_{i-1}, b = q_{i+1} - q_i (bowshock-cfd `limited_slope`).
fn dslope(a: D, b: D, lim: u32) -> D {
    if (lim == 0u) { return dc(0.0); }
    if (!((a.h > 0.0 && b.h > 0.0) || (a.h < 0.0 && b.h < 0.0))) { return dc(0.0); }
    if (lim == 1u) {
        if (dlt(dabs(a), dabs(b))) { return a; }
        return b;
    }
    let e = d_va_eps();
    let a2 = dmul(a, a);
    let b2 = dmul(b, b);
    let num = dadd(dmul(a, dadd(b2, e)), dmul(b, dadd(a2, e)));
    let den = dadd(dadd(a2, b2), dmulf(e, 2.0));
    return ddiv(num, den);
}

fn dleft(m2: D, m1: D, p0: D, lim: u32) -> D {
    return dadd(m1, dmulf(dslope(dsub(m1, m2), dsub(p0, m1), lim), 0.5));
}
fn dright(m1: D, p0: D, p1: D, lim: u32) -> D {
    return dsub(p0, dmulf(dslope(dsub(p0, m1), dsub(p1, p0), lim), 0.5));
}

fn dreconstruct(m2: W, m1: W, p0: W, p1: W, lim: u32) -> LR {
    var l = W(dleft(m2.r, m1.r, p0.r, lim), dleft(m2.u, m1.u, p0.u, lim), dleft(m2.v, m1.v, p0.v, lim), dleft(m2.p, m1.p, p0.p, lim));
    var r = W(dright(m1.r, p0.r, p1.r, lim), dright(m1.u, p0.u, p1.u, lim), dright(m1.v, p0.v, p1.v, lim), dright(m1.p, p0.p, p1.p, lim));
    if (l.r.h <= 0.0 || l.p.h <= 0.0) { l = m1; }
    if (r.r.h <= 0.0 || r.p.h <= 0.0) { r = p0; }
    return LR(l, r);
}

// f + s (u* - q) of the HLLC star state on one side (`w`, `un`, `e` of that side, `s` its wave speed).
fn dstar_flux(w: W, un: D, e: D, s: D, sm: D, nx: f32, nr: f32) -> C {
    let f = dphys(w, nx, nr);
    let sun = dsub(s, un);
    let sms = dsub(sm, un);
    let fac = ddiv(dmul(w.r, sun), dsub(s, sm));
    let us_e = dmul(fac, dadd(ddiv(e, w.r), dmul(sms, dadd(sm, ddiv(w.p, dmul(w.r, sun))))));
    let q = dcons(w, e);
    return C(
        dadd(f.a, dmul(s, dsub(fac, q.a))),
        dadd(f.b, dmul(s, dsub(dmul(fac, dadd(w.u, dmulf(sms, nx))), q.b))),
        dadd(f.c, dmul(s, dsub(dmul(fac, dadd(w.v, dmulf(sms, nr))), q.c))),
        dadd(f.d, dmul(s, dsub(us_e, q.d))));
}

fn dhllc(wl: W, wr: W, nx: f32, nr: f32) -> C {
    let unl = dun(wl, nx, nr);
    let unr = dun(wr, nx, nr);
    let cl = dsound(wl);
    let cr = dsound(wr);
    let el = dener(wl);
    let er = dener(wr);
    let sql = dsqrt(wl.r);
    let sqr = dsqrt(wr.r);
    let inv = ddiv(dc(1.0), dadd(sql, sqr));
    let uh = dmul(dadd(dmul(sql, wl.u), dmul(sqr, wr.u)), inv);
    let vh = dmul(dadd(dmul(sql, wl.v), dmul(sqr, wr.v)), inv);
    let hl = ddiv(dadd(el, wl.p), wl.r);
    let hr = ddiv(dadd(er, wr.p), wr.r);
    let hh = dmul(dadd(dmul(sql, hl), dmul(sqr, hr)), inv);
    let ch2 = dmul(d_gm1(), dsub(hh, dmulf(dadd(dmul(uh, uh), dmul(vh, vh)), 0.5)));
    var ch = dc(1e-15);
    if (ch2.h > 1e-30) { ch = dsqrt(ch2); }
    let unh = dadd(dmulf(uh, nx), dmulf(vh, nr));
    let sl = dmin(dsub(unl, cl), dsub(unh, ch));
    let sr = dmax(dadd(unr, cr), dadd(unh, ch));
    if (sl.h >= 0.0) { return dphys(wl, nx, nr); }
    if (sr.h <= 0.0) { return dphys(wr, nx, nr); }
    let sl_un = dsub(sl, unl);
    let sr_un = dsub(sr, unr);
    let num = dsub(dadd(dsub(wr.p, wl.p), dmul(dmul(wl.r, unl), sl_un)), dmul(dmul(wr.r, unr), sr_un));
    let sm = ddiv(num, dsub(dmul(wl.r, sl_un), dmul(wr.r, sr_un)));
    if (sm.h >= 0.0) { return dstar_flux(wl, unl, el, sl, sm, nx, nr); }
    return dstar_flux(wr, unr, er, sr, sm, nx, nr);
}

fn drusanov(wl: W, wr: W, nx: f32, nr: f32) -> C {
    let smax = dmax(dadd(dabs(dun(wl, nx, nr)), dsound(wl)), dadd(dabs(dun(wr, nx, nr)), dsound(wr)));
    let fl = dphys(wl, nx, nr);
    let fr = dphys(wr, nx, nr);
    let ql = dcons(wl, dener(wl));
    let qr = dcons(wr, dener(wr));
    return C(
        dsub(dmulf(dadd(fl.a, fr.a), 0.5), dmulf(dmul(smax, dsub(qr.a, ql.a)), 0.5)),
        dsub(dmulf(dadd(fl.b, fr.b), 0.5), dmulf(dmul(smax, dsub(qr.b, ql.b)), 0.5)),
        dsub(dmulf(dadd(fl.c, fr.c), 0.5), dmulf(dmul(smax, dsub(qr.c, ql.c)), 0.5)),
        dsub(dmulf(dadd(fl.d, fr.d), 0.5), dmulf(dmul(smax, dsub(qr.d, ql.d)), 0.5)));
}

fn dshock_weight(pl: D, pr: D) -> D {
    let s = ddiv(dabs(dsub(pr, pl)), dmin(pl, pr));
    let t = dmul(dsub(s, d_sens_lo()), d_sens_inv());
    if (t.h <= 0.0) { return dc(0.0); }
    if (t.h > 1.0 || (t.h == 1.0 && t.l >= 0.0)) { return dc(1.0); }
    return dmul(dmul(t, t), dsub(dc(3.0), dmulf(t, 2.0)));
}

fn dface_flux(m2: W, m1: W, p0: W, p1: W, lim: u32, nx: f32, nr: f32) -> C {
    let lr = dreconstruct(m2, m1, p0, p1, lim);
    let c = dhllc(lr.l, lr.r, nx, nr);
    let t = dshock_weight(m1.p, p0.p);
    if (t.h == 0.0) { return c; }
    let f = drusanov(lr.l, lr.r, nx, nr);
    let omt = dsub(dc(1.0), t);
    return C(
        dadd(dmul(omt, c.a), dmul(t, f.a)),
        dadd(dmul(omt, c.b), dmul(t, f.b)),
        dadd(dmul(omt, c.c), dmul(t, f.c)),
        dadd(dmul(omt, c.d), dmul(t, f.d)));
}

fn w_from_hl(h: vec4<f32>, l: vec4<f32>) -> W { return W(D(h.x, l.x), D(h.y, l.y), D(h.z, l.z), D(h.w, l.w)); }
fn c_from(h: vec4<f32>, l: vec4<f32>) -> C { return C(D(h.x, l.x), D(h.y, l.y), D(h.z, l.z), D(h.w, l.w)); }

fn c_hi(c: C) -> vec4<f32> { return vec4<f32>(c.a.h, c.b.h, c.c.h, c.d.h); }
fn c_lo(c: C) -> vec4<f32> { return vec4<f32>(c.a.l, c.b.l, c.c.l, c.d.l); }
fn w_hi(w: W) -> vec4<f32> { return vec4<f32>(w.r.h, w.u.h, w.v.h, w.p.h); }
fn w_lo(w: W) -> vec4<f32> { return vec4<f32>(w.r.l, w.u.l, w.v.l, w.p.l); }
"#;

fn split(x: f64) -> (f32, f32) {
    let h = x as f32;
    (h, (x - h as f64) as f32)
}

/// `COMMON` + the df library with its constants (all rounded from f64, hi/lo split).
pub fn common_ext() -> String {
    let g = bowshock_cfd::gas::GAMMA;
    let c = |name: &str, x: f64| {
        let (h, l) = split(x);
        format!("const {name}_H: f32 = {h:e};\nconst {name}_L: f32 = {l:e};\n")
    };
    let consts = [
        c("GAMMA", g),
        c("GM1", g - 1.0),
        c("IGM1", 1.0 / (g - 1.0)),
        c("VAEPS", 1e-14),
        c("SLO", bowshock_cfd::flux::SHOCK_SENSOR_LO),
        c("SINV", 1.0 / (bowshock_cfd::flux::SHOCK_SENSOR_HI - bowshock_cfd::flux::SHOCK_SENSOR_LO)),
    ]
    .concat();
    format!("{}\n{}\n{}", crate::shaders::common(), consts, DD_TEMPLATE)
}

/// Face flux of the guarded formulation in df, kernel 1 of 2 per RK stage (as `shaders::FLUX_GUARDED`): the primitive
/// states of the stencil are computed from the df state on the fly, ghost states lazily. `fi` / `fj` hold two
/// vec4 per face: `(hi, lo)` at `2 k` and `2 k + 1`. Problem size `[ni + 1, 2 nj + 1]`.
pub const FLUX_EXT: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> ulo: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(3) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(4) var<storage, read_write> fi: array<vec4<f32>>;
@group(1) @binding(5) var<storage, read_write> fj: array<vec4<f32>>;
@group(1) @binding(6) var<uniform> p: Params;

fn fs_state() -> W {
    return W(D(p.fs_rho, p.fs_lo_rho), D(p.fs_u, p.fs_lo_u), D(p.fs_v, p.fs_lo_v), D(p.fs_p, p.fs_lo_p));
}

fn cell_prim(i: i32, j: i32) -> W {
    let c = u32(j) * p.ni + u32(i);
    return dcons_to_prim(u[c], ulo[c]);
}

fn prim_i(i: i32, j: i32) -> W {
    let ni = i32(p.ni);
    if (i < 0) {
        let k = -i;
        let f = iface[u32(j) * (p.ni + 1u)];
        if (p.bc0 == 0u) { return fs_state(); }
        if (p.bc0 == 1u) { return cell_prim(0, j); }
        if (p.bc0 == 2u) { return dreflect_wall(cell_prim(k - 1, j), f.x, f.y); }
        return dfar_ghost(p.bc0, fs_state(), cell_prim(0, j), f, -1.0);
    }
    if (i >= ni) {
        let k = i - ni + 1;
        let f = iface[u32(j) * (p.ni + 1u) + p.ni];
        if (p.bc1 == 0u) { return fs_state(); }
        if (p.bc1 == 1u) { return cell_prim(ni - 1, j); }
        if (p.bc1 == 2u) { return dreflect_wall(cell_prim(ni - k, j), f.x, f.y); }
        return dfar_ghost(p.bc1, fs_state(), cell_prim(ni - 1, j), f, 1.0);
    }
    return cell_prim(i, j);
}

fn prim_j(i: i32, j: i32) -> W {
    let nj = i32(p.nj);
    if (j < 0) {
        let k = -j;
        let f = jface[u32(i)];
        if (p.bc2 == 0u) { return fs_state(); }
        if (p.bc2 == 1u) { return cell_prim(i, 0); }
        if (p.bc2 == 2u) { return dreflect_wall(cell_prim(i, k - 1), f.x, f.y); }
        return dfar_ghost(p.bc2, fs_state(), cell_prim(i, 0), f, -1.0);
    }
    if (j >= nj) {
        let k = j - nj + 1;
        let f = jface[p.nj * p.ni + u32(i)];
        if (p.bc3 == 0u) { return fs_state(); }
        if (p.bc3 == 1u) { return cell_prim(i, nj - 1); }
        if (p.bc3 == 2u) { return dreflect_wall(cell_prim(i, nj - k), f.x, f.y); }
        return dfar_ghost(p.bc3, fs_state(), cell_prim(i, nj - 1), f, 1.0);
    }
    return cell_prim(i, j);
}

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.y < p.nj) {
        if (gid.x > p.ni) { return; }
        let i = i32(gid.x);
        let j = i32(gid.y);
        let k = gid.y * (p.ni + 1u) + gid.x;
        let f = iface[k];
        let c = dface_flux(prim_i(i - 2, j), prim_i(i - 1, j), prim_i(i, j), prim_i(i + 1, j), p.limiter, f.x, f.y);
        fi[2u * k] = c_hi(c);
        fi[2u * k + 1u] = c_lo(c);
    } else {
        let jr = gid.y - p.nj;
        if (gid.x >= p.ni || jr > p.nj) { return; }
        let i = i32(gid.x);
        let j = i32(jr);
        let k = jr * p.ni + gid.x;
        let f = jface[k];
        let c = dface_flux(prim_j(i, j - 2), prim_j(i, j - 1), prim_j(i, j), prim_j(i, j + 1), p.limiter, f.x, f.y);
        fj[2u * k] = c_hi(c);
        fj[2u * k + 1u] = c_lo(c);
    }
}
"#;

/// Kernel 2 of 2 per RK stage in df: residual `-(sum_f w_f F_f) / V + (0, 0, p A / V, 0)` summed in df, collapsed to
/// f32 (`res`), then the compensated increment-form update (see `shaders::update`). Stage 1 also writes `dt`, `rho_res`
/// and the per-workgroup sum of `rho_res^2`. Problem size `[ni, nj]`.
pub const RESIDUAL_UPDATE_EXT: &str = r#"
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
@group(1) @binding(10) var<storage, read_write> ulo: array<vec4<f32>>;
@group(1) @binding(11) var<storage, read_write> u0lo: array<vec4<f32>>;
@group(1) @binding(12) var<uniform> p: Params;

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
        let q = u[c];
        let ql = ulo[c];
        let wc = dcons_to_prim(q, ql);
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
        let z = dc(0.0);
        let s = vec4<f32>(
            dsum(gl.a, fl.w, gr.a, fr.w, gb.a, fb.w, gt.a, ft.w, z).h,
            dsum(gl.b, fl.w, gr.b, fr.w, gb.b, fb.w, gt.b, ft.w, z).h,
            dsum(gl.c, fl.w, gr.c, fr.w, gb.c, fb.w, gt.c, ft.w, dmulf(wc.p, geo.x)).h,
            dsum(gl.d, fl.w, gr.d, fr.w, gb.d, fb.w, gt.d, ft.w, z).h);
        let res = s * inv_v;
        if (p.stage == 1u) {
            rho_res[c] = res.x;
            r2 = res.x * res.x;
            let cs = sqrt(GAMMA * wc.p.h / wc.r.h);
            let lam_l = (abs(wc.u.h * fl.x + wc.v.h * fl.y) + cs) * fl.z;
            let lam_r = (abs(wc.u.h * fr.x + wc.v.h * fr.y) + cs) * fr.z;
            let lam_b = (abs(wc.u.h * fb.x + wc.v.h * fb.y) + cs) * fb.z;
            let lam_t = (abs(wc.u.h * ft.x + wc.v.h * ft.y) + cs) * ft.z;
            let sum = lam_l + lam_r + lam_b + lam_t;
            let dtc = p.cfl * geo.x / (0.5 * sum);
            dt[c] = dtc;
            u0[c] = q; u0lo[c] = ql;
            let r = df_add(q, ql, dtc * res);
            u[c] = r.hi; ulo[c] = r.lo;
        } else {
            let d = (q - u0[c]) + (ql - u0lo[c]);
            let r = df_add(u0[c], u0lo[c], p.ra * (d + dt[c] * res));
            u[c] = r.hi; ulo[c] = r.lo;
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

/// Test kernel: df MUSCL reconstruction, HLLC and `face_flux` for `p.ni` independent samples (as `shaders::FACE_TEST`);
/// `out` holds `(left, right, hllc, face_flux)` high parts at `8 s ..` and the low parts at `8 s + 4 ..`.
pub const FACE_TEST_EXT: &str = r#"
@group(1) @binding(0) var<storage, read> states: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> normals: array<vec2<f32>>;
@group(1) @binding(2) var<storage, read_write> out: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let s = gid.x;
    if (s >= p.ni) { return; }
    let m2 = w_from_f32(states[4u * s]); let m1 = w_from_f32(states[4u * s + 1u]);
    let p0 = w_from_f32(states[4u * s + 2u]); let p1 = w_from_f32(states[4u * s + 3u]);
    let lr = dreconstruct(m2, m1, p0, p1, p.limiter);
    let n = normals[s];
    let h = dhllc(lr.l, lr.r, n.x, n.y);
    let f = dface_flux(m2, m1, p0, p1, p.limiter, n.x, n.y);
    out[8u * s] = w_hi(lr.l); out[8u * s + 1u] = w_hi(lr.r); out[8u * s + 2u] = c_hi(h); out[8u * s + 3u] = c_hi(f);
    out[8u * s + 4u] = w_lo(lr.l); out[8u * s + 5u] = w_lo(lr.r); out[8u * s + 6u] = c_lo(h); out[8u * s + 7u] = c_lo(f);
}
"#;

// ---------------------------------------------------------------------------------------------------------------------
// Direct (bowshock-cfd mirror) formulation in df: the primitive array `w` and the face fluxes hold `(hi, lo)` vec4 pairs.

/// cons -> prim (df) of every interior cell. `w` has two vec4 per cell of the ghosted array: `(hi, lo)` at `2 k`, `2 k + 1`.
/// Problem size `[ni, nj]`.
pub const PRIM_EXT: &str = r#"
@group(1) @binding(0) var<storage, read> u: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> ulo: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read_write> w: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y >= p.nj) { return; }
    let c = gid.y * p.ni + gid.x;
    let k = 2u * pidx(p.ni, i32(gid.x), i32(gid.y));
    let q = dcons_to_prim(u[c], ulo[c]);
    w[k] = w_hi(q);
    w[k + 1u] = w_lo(q);
}
"#;

/// Ghost-layer fill in df (as `shaders::GHOST`). Problem size `[nj + ni]`.
pub const GHOST_EXT: &str = r#"
@group(1) @binding(0) var<storage, read_write> w: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

fn wget(i: i32, j: i32) -> W {
    let k = 2u * pidx(p.ni, i, j);
    return w_from_hl(w[k], w[k + 1u]);
}
fn wset(i: i32, j: i32, v: W) {
    let k = 2u * pidx(p.ni, i, j);
    w[k] = w_hi(v);
    w[k + 1u] = w_lo(v);
}
fn dghost(bc: u32, fs: W, edge: W, mirror: W, f: vec4<f32>, outward: f32) -> W {
    if (bc == 0u) { return fs; }
    if (bc == 1u) { return edge; }
    if (bc == 2u) { return dreflect_wall(mirror, f.x, f.y); }
    return dfar_ghost(bc, fs, edge, f, outward);
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let t = gid.x;
    let fs = W(D(p.fs_rho, p.fs_lo_rho), D(p.fs_u, p.fs_lo_u), D(p.fs_v, p.fs_lo_v), D(p.fs_p, p.fs_lo_p));
    let ni = i32(p.ni);
    let nj = i32(p.nj);
    if (t < p.nj) {
        let j = i32(t);
        for (var k: i32 = 1; k <= NG; k++) {
            var f = iface[t * (p.ni + 1u)];
            wset(-k, j, dghost(p.bc0, fs, wget(0, j), wget(k - 1, j), f, -1.0));
            f = iface[t * (p.ni + 1u) + p.ni];
            wset(ni + k - 1, j, dghost(p.bc1, fs, wget(ni - 1, j), wget(ni - k, j), f, 1.0));
        }
    } else if (t < p.nj + p.ni) {
        let iu = t - p.nj;
        let i = i32(iu);
        for (var k: i32 = 1; k <= NG; k++) {
            var f = jface[iu];
            wset(i, -k, dghost(p.bc2, fs, wget(i, 0), wget(i, k - 1), f, -1.0));
            f = jface[p.nj * p.ni + iu];
            wset(i, nj + k - 1, dghost(p.bc3, fs, wget(i, nj - 1), wget(i, nj - k), f, 1.0));
        }
    }
}
"#;

/// Face flux (df) through every i-face. Problem size `[ni + 1, nj]`.
pub const FLUX_I_EXT: &str = r#"
@group(1) @binding(0) var<storage, read> w: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> iface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read_write> fi: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

fn wget(i: i32, j: i32) -> W {
    let k = 2u * pidx(p.ni, i, j);
    return w_from_hl(w[k], w[k + 1u]);
}

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x > p.ni || gid.y >= p.nj) { return; }
    let i = i32(gid.x);
    let j = i32(gid.y);
    let k = gid.y * (p.ni + 1u) + gid.x;
    let f = iface[k];
    let c = dface_flux(wget(i - 2, j), wget(i - 1, j), wget(i, j), wget(i + 1, j), p.limiter, f.x, f.y);
    fi[2u * k] = c_hi(c);
    fi[2u * k + 1u] = c_lo(c);
}
"#;

/// Face flux (df) through every j-face. Problem size `[ni, nj + 1]`.
pub const FLUX_J_EXT: &str = r#"
@group(1) @binding(0) var<storage, read> w: array<vec4<f32>>;
@group(1) @binding(1) var<storage, read> jface: array<vec4<f32>>;
@group(1) @binding(2) var<storage, read_write> fj: array<vec4<f32>>;
@group(1) @binding(3) var<uniform> p: Params;

fn wget(i: i32, j: i32) -> W {
    let k = 2u * pidx(p.ni, i, j);
    return w_from_hl(w[k], w[k + 1u]);
}

@compute @workgroup_size(32, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= p.ni || gid.y > p.nj) { return; }
    let i = i32(gid.x);
    let j = i32(gid.y);
    let k = gid.y * p.ni + gid.x;
    let f = jface[k];
    let c = dface_flux(wget(i, j - 2), wget(i, j - 1), wget(i, j), wget(i, j + 1), p.limiter, f.x, f.y);
    fj[2u * k] = c_hi(c);
    fj[2u * k + 1u] = c_lo(c);
}
"#;

/// Cell residual in df, collapsed to f32 `res` (as `shaders::RESIDUAL`); stage 1: `dt`, `rho_res`. `[ni, nj]`.
pub const RESIDUAL_EXT: &str = r#"
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

fn dsum(gl: D, wl: f32, gr: D, wr: f32, gb: D, wb: f32, gt: D, wt: f32, pa: D) -> D {
    return dadd(dsub(dsub(dmulf(gl, wl), dmulf(gr, wr)), dsub(dmulf(gt, wt), dmulf(gb, wb))), pa);
}

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
    let gl = c_from(fi[2u * (j * (p.ni + 1u) + i)], fi[2u * (j * (p.ni + 1u) + i) + 1u]);
    let gr = c_from(fi[2u * (j * (p.ni + 1u) + i + 1u)], fi[2u * (j * (p.ni + 1u) + i + 1u) + 1u]);
    let gb = c_from(fj[2u * (j * p.ni + i)], fj[2u * (j * p.ni + i) + 1u]);
    let gt = c_from(fj[2u * ((j + 1u) * p.ni + i)], fj[2u * ((j + 1u) * p.ni + i) + 1u]);
    let kc = 2u * pidx(p.ni, i32(i), i32(j));
    let wc = w_from_hl(w[kc], w[kc + 1u]);
    let geo = cell[c];
    let inv_v = 1.0 / geo.y;
    let z = dc(0.0);
    let s = vec4<f32>(
        dsum(gl.a, fl.w, gr.a, fr.w, gb.a, fb.w, gt.a, ft.w, z).h,
        dsum(gl.b, fl.w, gr.b, fr.w, gb.b, fb.w, gt.b, ft.w, z).h,
        dsum(gl.c, fl.w, gr.c, fr.w, gb.c, fb.w, gt.c, ft.w, dmulf(wc.p, geo.x)).h,
        dsum(gl.d, fl.w, gr.d, fr.w, gb.d, fb.w, gt.d, ft.w, z).h);
    let r = s * inv_v;
    res[c] = r;
    if (p.stage == 1u) {
        rho_res[c] = r.x;
        let cs = sqrt(GAMMA * wc.p.h / wc.r.h);
        let lam_l = (abs(wc.u.h * fl.x + wc.v.h * fl.y) + cs) * fl.z;
        let lam_r = (abs(wc.u.h * fr.x + wc.v.h * fr.y) + cs) * fr.z;
        let lam_b = (abs(wc.u.h * fb.x + wc.v.h * fb.y) + cs) * fb.z;
        let lam_t = (abs(wc.u.h * ft.x + wc.v.h * ft.y) + cs) * ft.z;
        dt[c] = p.cfl * geo.x / (0.5 * (lam_l + lam_r + lam_b + lam_t));
    }
}
"#;
