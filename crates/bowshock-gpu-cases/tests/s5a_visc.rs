//! Stage S5a: the 3D viscous kernels on the GPU (`gpu/viscous3.rs`) against the CPU `viscous3`, stage by stage, on tiny grids: the same padded primitive state, the
//! same solid mask, symmetry sides, eddy viscosity and wall eddy viscosity go into both, and every output is compared: the cell gradients and viscosity (`cell_pass`),
//! the viscous fluxes of the i, j and k faces (`face_pass`), the diffusive spectral radius of every face (`lambda_pass`), the viscous residual
//! (`add_viscous_residual`), the diffusion rates, and the viscous wall forces by class (`wall_tractions`, reduced on the device).
//! The error of a quantity is `max |gpu - cpu| / max |cpu|` over the fluid cells / the faces that carry a flux.
//! `cargo test --release -p bowshock-gpu-cases --test s5a_visc -- --nocapture` (through gpuq).

use bowshock_cfd::gas3::{freestream3, reflect3, Prim3};
use bowshock_cfd::gpu::TractionFace3;
use bowshock_cfd::grid3::{Face3, Grid3, Part, SolidBox, PART_FLUID};
use bowshock_cfd::implicit::ImplicitParams;
use bowshock_cfd::solver::{Turbulence, Viscous};
use bowshock_cfd::solver3::{Config3, Scheme3};
use bowshock_cfd::viscous3::{
    add_viscous_residual, cell_pass, diffusion_rates, face_dirs, face_pass, face_shape, lambda_pass, wall_tractions, Grad3, SymSides,
};
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier};
use bowshock_core::engine::grid3::{body_grid3, Fins3};
use bowshock_core::FinProfile;

const VISC: Viscous = Viscous { re_c: 2.0e3, sutherland: 0.368, turbulence: Turbulence::Laminar, wall_i_start: 0, roughness: 0.0 };

/// Deterministic noise in `[-1, 1]`.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0
    }
}

fn box_grid() -> Grid3 {
    let (n, h) = ([7usize, 6, 5], [0.11, 0.09, 0.13]);
    let (mut x, mut y, mut z) = (vec![], vec![], vec![]);
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                // a slightly stretched, sheared grid: skewed faces, not an orthogonal box
                let (fi, fj, fk) = (i as f64, j as f64, k as f64);
                x.push(fi * h[0] + 0.02 * fj + 0.01 * fk * fi / 7.0);
                y.push(fj * h[1] * (1.0 + 0.05 * fi / 7.0) + 0.015 * fk);
                z.push(fk * h[2] + 0.01 * fj * (fi / 7.0));
            }
        }
    }
    let mut g = Grid3::try_from_nodes(n[0], n[1], n[2], x, y, z).unwrap();
    g.mark_solid(SolidBox { i: 2..4, j: 0..2, k: 1..3 }, Some(Part::Body(0)));
    g.mark_solid(SolidBox { i: 5..6, j: 3..5, k: 0..2 }, Some(Part::Fin(0)));
    g.mark_solid(SolidBox { i: 0..1, j: 4..6, k: 3..5 }, None);
    g
}

fn tiny_base(c: &mut Case3) {
    let p = c.grid.base.as_mut().expect("base-wake grid");
    p.n_up = 6;
    p.n_nose = 10;
    p.n_seg = p.n_seg.iter().map(|&n| (n / 4).max(4)).collect();
    p.n_wake = 8;
    p.n_gap = 6;
    p.nj_out = 10;
}

fn wedge_grid() -> Grid3 {
    let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0);
    let mut c = Case3::wedge(2.0, body, 0);
    tiny_base(&mut c);
    body_grid3(&c.grid).expect("wedge grid")
}

fn sector_grid(n: u32) -> Grid3 {
    let nose = Body { nose: Nose::Cone { half_angle: 10f64.to_radians() } };
    let body = BaseBody::cylinder(nose, 10.0 - nose.nose_length());
    let fins = Fins3 {
        count: n,
        root_chord: 1.0,
        tip_chord: 1.0,
        span: 1.0,
        sweep: 0.0,
        thickness: 0.08,
        te_to_base: 0.0,
        profile: FinProfile::Square,
        planform: None,
    };
    let mut c = Case3::sector(2.0, body, fins, Tier::Euler, 0);
    c.grid.nk = 6;
    let s = &mut c.grid.sting;
    s.n_up = 10;
    s.n_nose = 16;
    s.n_seg = vec![12];
    s.n_ext = 10;
    s.nj = 24;
    c.grid.fin_chord_cells = Some(16);
    body_grid3(&c.grid).expect("sector grid")
}

/// Padded primitive state of smooth, noisy, physical fields on the cell centres; the ghost layers are the reflection of the nearest cell about the face (the
/// symmetry planes) or a copy (the rest); solid cells hold a finite free-stream state.
fn state(g: &Grid3, solid: &[bool], sym: &SymSides, seed: u64) -> Vec<Prim3> {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let mut rng = Rng(seed);
    let scale = g.xc.iter().fold(1e-9f64, |m, x| m.max(x[0].abs()).max(x[1].abs()).max(x[2].abs()));
    let cells: Vec<Prim3> = (0..ni * nj * nk)
        .map(|c| {
            let x = g.xc[c];
            let s = (x[0] / scale, x[1] / scale, x[2] / scale);
            let noise = 0.03;
            if solid[c] {
                return [1.0, 0.0, 0.0, 0.0, 1.0 / 1.4];
            }
            [
                1.0 + 0.2 * (3.0 * s.0 + 1.0).sin() * (2.0 * s.1).cos() + noise * rng.next(),
                0.6 + 0.3 * (2.0 * s.1 + s.2).sin() + noise * rng.next(),
                0.2 * (4.0 * s.0).cos() + 0.1 * s.2 + noise * rng.next(),
                0.15 * (3.0 * s.1 - s.0).sin() + noise * rng.next(),
                (1.0 + 0.15 * (2.0 * s.2 + s.0).cos() + 0.5 * noise * rng.next()) / 1.4,
            ]
        })
        .collect();
    let idx = |i: isize, j: isize, k: isize| ((k + 2) as usize * (nj + 4) + (j + 2) as usize) * (ni + 4) + (i + 2) as usize;
    let mut w = vec![[1.0, 0.0, 0.0, 0.0, 1.0 / 1.4]; (ni + 4) * (nj + 4) * (nk + 4)];
    for k in -2..nk as isize + 2 {
        for j in -2..nj as isize + 2 {
            for i in -2..ni as isize + 2 {
                let (ci, cj, ck) = (i.clamp(0, ni as isize - 1), j.clamp(0, nj as isize - 1), k.clamp(0, nk as isize - 1));
                let cell = cells[g.cell(ci as usize, cj as usize, ck as usize)];
                let outside = [i < 0, i >= ni as isize, j < 0, j >= nj as isize, k < 0, k >= nk as isize];
                let mut state = cell;
                for (side, &out) in outside.iter().enumerate() {
                    if out && sym[side] {
                        let face: &Face3 = match side {
                            0 => &g.iface[(ck as usize * nj + cj as usize) * (ni + 1)],
                            1 => &g.iface[(ck as usize * nj + cj as usize) * (ni + 1) + ni],
                            2 => &g.jface[ck as usize * (nj + 1) * ni + ci as usize],
                            3 => &g.jface[(ck as usize * (nj + 1) + nj) * ni + ci as usize],
                            4 => &g.kface[cj as usize * ni + ci as usize],
                            _ => &g.kface[nk * ni * nj + cj as usize * ni + ci as usize],
                        };
                        state = reflect3(&state, face.n);
                    }
                }
                w[idx(i, j, k)] = state;
            }
        }
    }
    w
}

struct Case {
    name: String,
    grid: Grid3,
    sym: SymSides,
    laminar: bool,
}

/// Relative maximum error of `a` against `b` over the entries where `keep` holds, scaled by the maximum of `|b|` (`1e-30` floor).
fn err(a: &[f64], b: &[f64], keep: &dyn Fn(usize) -> bool) -> f64 {
    let scale = b.iter().enumerate().filter(|&(q, _)| keep(q)).fold(1e-30f64, |m, (_, v)| m.max(v.abs()));
    a.iter().zip(b).enumerate().filter(|&(q, _)| keep(q)).fold(0.0f64, |m, (_, (x, y))| m.max((x - y).abs() / scale))
}

struct Errors {
    grad: f64,
    mu: f64,
    fv: f64,
    lambda: f64,
    dr: f64,
    residual: f64,
    traction: f64,
}

macro_rules! probe {
    ($module:ident, $ctx:expr, $case:expr, $w:expr, $mu_t:expr, $wall:expr, $faces:expr, $ncl:expr) => {{
        let cfg = Config3 {
            freestream: freestream3(2.0, 0.0),
            cfl: 0.5,
            limiter: bowshock_cfd::flux::Limiter::Minmod,
            first_order_iters: 0,
            low_mach: false,
            scheme: Scheme3::Explicit,
        };
        let mut lazy = bowshock_cfd::gpu::$module::LazySolver3::new($ctx, &$case.grid, &cfg).map_err(|e| format!("new: {e}"))?;
        lazy.set_viscous3(&$case.grid, VISC, &$case.sym).map_err(|e| format!("set_viscous3: {e}"))?;
        lazy.set_traction_faces($faces, $ncl).map_err(|e| format!("traction faces: {e}"))?;
        lazy.viscous_probe3($w, $mu_t, $wall).map_err(|e| format!("probe: {e}"))?
    }};
}

fn compare(case: &Case, f32_precision: bool) -> Result<Errors, String> {
    let g = &case.grid;
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let nc = ni * nj * nk;
    let solid: Vec<bool> = g.part.iter().map(|&p| p != PART_FLUID).collect();
    let w = state(g, &solid, &case.sym, 7);
    let mut rng = Rng(11);
    let mu_t: Vec<f64> = (0..nc).map(|c| if case.laminar || solid[c] { 0.0 } else { 1e-3 * (1.0 + 0.5 * rng.next()) }).collect();
    let wall_mut: Vec<f64> = (0..nc).map(|c| if case.laminar || solid[c] { 0.0 } else { 5e-4 * (1.0 + 0.5 * rng.next()) }).collect();
    let wall3: Vec<[f64; 3]> = if case.laminar { Vec::new() } else { wall_mut.iter().map(|&m| [0.0, 0.0, m]).collect() };

    // the CPU
    let (mut grad, mut mu) = (vec![[0.0; 12] as Grad3; nc], vec![0.0; nc]);
    cell_pass(&VISC, g, &solid, &w, &mut grad, &mut mu);
    let dirs = face_dirs(g, &solid, &case.sym);
    let (mut fvi, mut fvj, mut fvk) = (vec![[0.0; 5]; g.iface.len()], vec![[0.0; 5]; g.jface.len()], vec![[0.0; 5]; g.kface.len()]);
    face_pass(g, &solid, &w, &dirs, &grad, &mu, &mu_t, &wall3, &mut fvi, &mut fvj, &mut fvk);
    let mut res = vec![[0.0; 5]; nc];
    add_viscous_residual(g, &solid, &fvi, &fvj, &fvk, &mut res);
    let (mut li, mut lj, mut lk) = (vec![0.0; g.iface.len()], vec![0.0; g.jface.len()], vec![0.0; g.kface.len()]);
    lambda_pass(g, &solid, &w, &dirs, &mu, &mu_t, &wall3, &mut li, &mut lj, &mut lk);
    let idx = |i: usize, j: usize, k: usize| ((k + 2) * (nj + 4) + (j + 2)) * (ni + 4) + (i + 2);
    let mut dr = vec![[0.0; 3]; nc];
    for k in 0..nk {
        for j in 0..nj {
            for i in 0..ni {
                let c = g.cell(i, j, k);
                if !solid[c] {
                    dr[c] = diffusion_rates(g, i, j, k, mu[c], mu_t[c], w[idx(i, j, k)][0]);
                }
            }
        }
    }
    // the viscous wall forces: class = (family, side of the solid)
    let tr = wall_tractions(g, &solid, &dirs, &fvi, &fvj, &fvk);
    let mut faces = Vec::new();
    let mut cpu_forces = [[0.0f64; 3]; 6];
    for t in &tr {
        let [n0, n1, _] = face_shape(g, t.dir);
        let q = (t.idx[2] * n1 + t.idx[1]) * n0 + t.idx[0];
        let face = match t.dir {
            0 => &g.iface[q],
            1 => &g.jface[q],
            _ => &g.kface[q],
        };
        let sigma = if t.n[0] * face.n[0] + t.n[1] * face.n[1] + t.n[2] * face.n[2] > 0.0 { 1.0 } else { -1.0 };
        let class = 2 * t.dir + (sigma > 0.0) as usize;
        faces.push(TractionFace3 { class, dir: t.dir, q, sigma, area: face.area, cx: face.c[0] });
        let f = [t.tau[0] * t.area, t.tau[2] * t.area, face.c[0] * t.tau[2] * t.area];
        for a in 0..3 {
            cpu_forces[class][a] += f[a];
        }
    }

    // the GPU: solid cells carry a finite state already
    let ctx = luma::Context::new().map_err(|e| format!("context: {e}"))?;
    let wall = (!case.laminar).then_some(wall_mut.as_slice());
    let p = if f32_precision { probe!(f32s, &ctx, case, &w, &mu_t, wall, &faces, 6) } else { probe!(df32s, &ctx, case, &w, &mu_t, wall, &faces, 6) };

    let fluid = |c: usize| !solid[c];
    let flat = |v: &[[f64; 12]]| -> Vec<f64> { v.iter().flatten().copied().collect() };
    let flat5 = |v: &[[f64; 5]]| -> Vec<f64> { v.iter().flatten().copied().collect() };
    let by_comp = |n: usize| move |q: usize| fluid(q / n);
    let fv_err = [(&p.fv[0], &fvi), (&p.fv[1], &fvj), (&p.fv[2], &fvk)].iter().fold(0.0f64, |m, (a, b)| m.max(err(&flat5(a), &flat5(b), &|_| true)));
    let lam_err = [(&p.lambda[0], &li), (&p.lambda[1], &lj), (&p.lambda[2], &lk)].iter().fold(0.0f64, |m, (a, b)| m.max(err(a, b, &|_| true)));
    let dr_cpu: Vec<f64> = dr.iter().flatten().copied().collect();
    let dr_gpu: Vec<f64> = p.dr.iter().flatten().copied().collect();
    let trac_gpu: Vec<f64> = p.tractions.iter().flatten().copied().collect();
    let trac_cpu: Vec<f64> = cpu_forces.iter().flatten().copied().collect();
    Ok(Errors {
        grad: err(&flat(&p.grad), &flat(&grad), &by_comp(12)),
        mu: err(&p.mu, &mu, &fluid),
        fv: fv_err,
        lambda: lam_err,
        dr: err(&dr_gpu, &dr_cpu, &by_comp(3)),
        residual: err(&flat5(&p.residual), &flat5(&res), &by_comp(5)),
        traction: err(&trac_gpu, &trac_cpu, &|_| true),
    })
}

#[test]
fn viscous_kernels_match_cpu() {
    let cases = vec![
        Case { name: "box, solid blocks, laminar".into(), grid: box_grid(), sym: [false; 6], laminar: true },
        Case {
            name: "box, solid blocks, mu_t + wall, k and j symmetry planes".into(),
            grid: box_grid(),
            sym: [false, false, true, false, true, true],
            laminar: false,
        },
        Case { name: "wedge, k planes symmetric, laminar".into(), grid: wedge_grid(), sym: [false, false, false, false, true, true], laminar: true },
        Case { name: "wedge, k planes symmetric, mu_t + wall".into(), grid: wedge_grid(), sym: [false, false, false, false, true, true], laminar: false },
        Case {
            name: "sector N 4 (fin), k planes symmetric, mu_t + wall".into(),
            grid: sector_grid(4),
            sym: [false, false, false, false, true, true],
            laminar: false,
        },
        Case {
            name: "sector N 3 (fin), k planes symmetric, laminar".into(),
            grid: sector_grid(3),
            sym: [false, false, false, false, true, true],
            laminar: true,
        },
    ];
    let _ = ImplicitParams::DEFAULT;
    println!("\n| case | cells | precision | grad | mu | Fv | lambda | diff. rates | residual | wall forces |\n|---|---|---|---|---|---|---|---|---|---|");
    let mut bad = Vec::new();
    for case in &cases {
        for (f32_precision, label, tol) in [(true, "f32", 2e-5), (false, "df32", 1e-9)] {
            match compare(case, f32_precision) {
                Ok(e) => {
                    let worst = [e.grad, e.mu, e.fv, e.lambda, e.dr, e.residual, e.traction].into_iter().fold(0.0f64, f64::max);
                    println!(
                        "| {} | {}x{}x{} | {label} | {:.1e} | {:.1e} | {:.1e} | {:.1e} | {:.1e} | {:.1e} | {:.1e} |",
                        case.name, case.grid.ni, case.grid.nj, case.grid.nk, e.grad, e.mu, e.fv, e.lambda, e.dr, e.residual, e.traction
                    );
                    if worst.is_nan() || worst > tol {
                        bad.push(format!("{} {label}: worst {worst:.2e} > {tol:.0e}", case.name));
                    }
                }
                Err(e) => bad.push(format!("{} {label}: {e}", case.name)),
            }
        }
    }
    assert!(bad.is_empty(), "{} rows are not green:\n{}", bad.len(), bad.join("\n"));
}
