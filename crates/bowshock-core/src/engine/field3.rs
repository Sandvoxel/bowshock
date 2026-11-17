//! A [`Field3`] from the 3D solver's state: the grid, the conserved state (`Grid3::cell` order, from the CPU solver or downloaded from
//! the GPU), the free stream and the case's body.
//!
//! The walls are found generically: every face between a fluid and a solid cell, and the `j = 0` rows of the sting grid (`i >=
//! axis_cells`) while that grid exists. The wall pressure is the one of the force integration (`run3::accumulate`): the pressure of the
//! first two fluid cells normal to the wall extrapolated linearly to the face centre, the first cell's alone when there is no second
//! fluid cell. The viewer's `sum -cp A n` over the walls therefore reproduces the pressure force `run3::forces_on` reports (a test
//! below).
//!
//! The part of a face is decided in [`classify`], the one place that knows how the solver marks the body and the fins.

use crate::engine::case3::Case3;
use crate::field3::{Field3, Part, Symmetry, WallFace};
use bowshock_cfd::gas3::{cons_to_prim3, Cons3};
use bowshock_cfd::grid3::{Face3, Grid3};
use bowshock_cfd::solver3::Solver3;
use std::f32::consts::PI;

/// What a wall face is, as far as the solver's topology goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Surface {
    /// A `j = 0` row of the sting grid: the body, then the sting.
    BodyRow,
    /// A face between a fluid cell and a solid box (a fin).
    SolidBox,
}

/// The part of the vehicle of a wall face of kind `what` whose centroid is at `x` (nose diameters from the tip). The only function that
/// reads the solver's topology to name parts; with a per-cell part id in the grid it is the one to replace.
fn classify(case: &Case3, what: Surface, x: f64) -> Part {
    match what {
        Surface::SolidBox => Part::Fin(0),
        Surface::BodyRow if x <= case.body.body.nose_length() => Part::Nose,
        Surface::BodyRow if x <= case.body.x_base() => Part::Body,
        Surface::BodyRow => Part::Other,
    }
}

/// [`Field3`] of the CPU solver `s` after `iteration` iterations, lengths in metres through `scale` (metres per nose diameter).
pub fn field3_of_solver(case: &Case3, s: &Solver3, iteration: usize, scale: f64) -> Field3 {
    field3_of(case, &s.grid, &s.u, iteration, scale)
}

/// [`Field3`] of the state `u` on `g` after `iteration` iterations, lengths in metres through `scale` (metres per nose diameter).
pub fn field3_of(case: &Case3, g: &Grid3, u: &[Cons3], iteration: usize, scale: f64) -> Field3 {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let fs = case.freestream();
    let mut solid = vec![false; ni * nj * nk];
    for b in &g.solid {
        for k in b.k.clone() {
            for j in b.j.clone() {
                for i in b.i.clone() {
                    solid[g.cell(i, j, k)] = true;
                }
            }
        }
    }
    let prim: Vec<[f32; 5]> = u.iter().zip(&solid).map(|(q, &s)| (if s { fs } else { cons_to_prim3(q) }).map(|v| v as f32)).collect();
    let nodes: Vec<[f32; 3]> = (0..g.x.len()).map(|n| [g.x[n], g.y[n], g.z[n]].map(|v| (v * scale) as f32)).collect();
    let walls = walls_of(case, g, u, &solid, &fs);
    let fin_theta = case.grid.fins.as_ref().map_or_else(Vec::new, |f| {
        let n = f64::from(f.count);
        (0..f.count)
            .map(|m| std::f64::consts::PI / n + std::f64::consts::TAU * f64::from(m) / n)
            .filter(|&t| t <= std::f64::consts::PI + 1e-9)
            .map(|t| t as f32)
            .collect()
    });
    let mut f = Field3 {
        mach: case.mach,
        alpha_deg: case.alpha.to_degrees(),
        iteration,
        freestream: fs,
        ni,
        nj,
        nk,
        mesh_id: 0,
        nodes,
        solid,
        prim,
        eddy_ratio: Vec::new(),
        walls,
        fin_theta,
        symmetry: Symmetry::Mirror { theta0: 0.0, theta1: PI },
    };
    f.mesh_id = mesh_id(&f);
    f
}

/// Hash of what a viewer builds its mesh from: the grid size, the nodes, the solid flags and the topology and part of the walls.
pub fn mesh_id(f: &Field3) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut put = |bytes: &[u8]| {
        for &b in bytes {
            h = (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for n in [f.ni, f.nj, f.nk] {
        put(&(n as u64).to_le_bytes());
    }
    for p in &f.nodes {
        for v in p {
            put(&v.to_bits().to_le_bytes());
        }
    }
    put(&f.solid.iter().map(|&s| u8::from(s)).collect::<Vec<_>>());
    for w in &f.walls {
        for n in w.nodes {
            put(&n.to_le_bytes());
        }
        put(&w.cell.to_le_bytes());
        put(format!("{:?}", w.part).as_bytes());
    }
    h
}

/// Every wall face of the solved domain with its pressure coefficient.
fn walls_of(case: &Case3, g: &Grid3, u: &[Cons3], solid: &[bool], fs: &[f64; 5]) -> Vec<WallFace> {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let cell = |i: usize, j: usize, k: usize| (k * nj + j) * ni + i;
    let is_solid = |i: usize, j: usize, k: usize| solid[cell(i, j, k)];
    let p = |c: usize| cons_to_prim3(&u[c])[4];
    let node = |i: usize, j: usize, k: usize| (k * (nj + 1) + j) * (ni + 1) + i;
    let q_inf = 0.5 * fs[0] * (fs[1] * fs[1] + fs[2] * fs[2] + fs[3] * fs[3]);
    let dist = |c: usize, f: &Face3| {
        let x = g.xc[c];
        ((x[0] - f.c[0]).powi(2) + (x[1] - f.c[1]).powi(2) + (x[2] - f.c[2]).powi(2)).sqrt()
    };
    let mut walls = Vec::new();
    // `a`: the first fluid cell, `b`: the fluid cell behind it (if any), `into`: +1 when the fluid lies on the +index side of the face
    // (the face normal then points into the flow), `corners`: the face's nodes in the order Grid3 builds its faces from
    let mut add = |what: Surface, a: usize, b: Option<usize>, f: &Face3, into: f64, corners: [(usize, usize, usize); 4]| {
        let p0 = p(a);
        let pw = match b {
            Some(b) => {
                let (d0, d1) = (dist(a, f), dist(b, f));
                p0 + (p0 - p(b)) * d0 / (d1 - d0)
            }
            None => p0,
        };
        let mut ids = corners.map(|(i, j, k)| node(i, j, k));
        // wind counter-clockwise seen from the fluid: the area vector (p2 - p0) x (p3 - p1) / 2 along the normal into the flow
        let pt = |n: usize| [g.x[n], g.y[n], g.z[n]];
        let (d02, d13) = (sub(pt(ids[2]), pt(ids[0])), sub(pt(ids[3]), pt(ids[1])));
        let s = cross(d02, d13);
        if s[0] * f.n[0] * into + s[1] * f.n[1] * into + s[2] * f.n[2] * into < 0.0 {
            ids.swap(1, 3);
        }
        walls.push(WallFace {
            nodes: ids.map(|n| n as u32),
            cell: a as u32,
            part: classify(case, what, f.c[0]),
            cp: ((pw - fs[4]) / q_inf) as f32,
            cf: [0.0; 3],
        });
    };
    for k in 0..nk {
        for i in g.axis_cells..ni {
            if !is_solid(i, 0, k) {
                let f = &g.jface[k * (nj + 1) * ni + i];
                let b = (nj > 1 && !is_solid(i, 1, k)).then(|| cell(i, 1, k));
                add(Surface::BodyRow, cell(i, 0, k), b, f, 1.0, [(i, 0, k), (i, 0, k + 1), (i + 1, 0, k + 1), (i + 1, 0, k)]);
            }
        }
    }
    for k in 0..nk {
        for j in 0..nj {
            for i in 1..ni {
                let f = &g.iface[(k * nj + j) * (ni + 1) + i];
                let c = [(i, j, k), (i, j + 1, k), (i, j + 1, k + 1), (i, j, k + 1)];
                match (is_solid(i - 1, j, k), is_solid(i, j, k)) {
                    (false, true) => add(Surface::SolidBox, cell(i - 1, j, k), (i >= 2 && !is_solid(i - 2, j, k)).then(|| cell(i - 2, j, k)), f, -1.0, c),
                    (true, false) => add(Surface::SolidBox, cell(i, j, k), (i + 1 < ni && !is_solid(i + 1, j, k)).then(|| cell(i + 1, j, k)), f, 1.0, c),
                    _ => {}
                }
            }
        }
    }
    for k in 0..nk {
        for j in 1..nj {
            for i in 0..ni {
                let f = &g.jface[(k * (nj + 1) + j) * ni + i];
                let c = [(i, j, k), (i, j, k + 1), (i + 1, j, k + 1), (i + 1, j, k)];
                match (is_solid(i, j - 1, k), is_solid(i, j, k)) {
                    (false, true) => add(Surface::SolidBox, cell(i, j - 1, k), (j >= 2 && !is_solid(i, j - 2, k)).then(|| cell(i, j - 2, k)), f, -1.0, c),
                    (true, false) => add(Surface::SolidBox, cell(i, j, k), (j + 1 < nj && !is_solid(i, j + 1, k)).then(|| cell(i, j + 1, k)), f, 1.0, c),
                    _ => {}
                }
            }
        }
    }
    for k in 1..nk {
        for j in 0..nj {
            for i in 0..ni {
                let f = &g.kface[(k * nj + j) * ni + i];
                let c = [(i, j, k), (i + 1, j, k), (i + 1, j + 1, k), (i, j + 1, k)];
                match (is_solid(i, j, k - 1), is_solid(i, j, k)) {
                    (false, true) => add(Surface::SolidBox, cell(i, j, k - 1), (k >= 2 && !is_solid(i, j, k - 2)).then(|| cell(i, j, k - 2)), f, -1.0, c),
                    (true, false) => add(Surface::SolidBox, cell(i, j, k), (k + 1 < nk && !is_solid(i, j, k + 1)).then(|| cell(i, j, k + 1)), f, 1.0, c),
                    _ => {}
                }
            }
        }
    }
    walls
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::run3::forces_on;

    /// The tiny ANF case after a few implicit steps: a state with real pressures on the walls.
    fn stepped(mach: f64, alpha_deg: f64, steps: usize) -> (Case3, Solver3) {
        let case = Case3::tiny_anf(mach, alpha_deg.to_radians());
        let mut s = case.build_solver().unwrap();
        for _ in 0..steps {
            s.step();
        }
        (case, s)
    }

    fn area_vector(f: &Field3, w: &WallFace) -> [f64; 3] {
        let p = |n: u32| f.nodes[n as usize].map(f64::from);
        let (d02, d13) = (sub(p(w.nodes[2]), p(w.nodes[0])), sub(p(w.nodes[3]), p(w.nodes[1])));
        let s = cross(d02, d13);
        [0.5 * s[0], 0.5 * s[1], 0.5 * s[2]]
    }

    #[test]
    fn the_walls_reproduce_the_pressure_force() {
        let (case, s) = stepped(2.0, 4.0, 40);
        let f = field3_of_solver(&case, &s, 40, 1.0);
        let reported = forces_on(&case, &s.grid, &s.u, s.cfg.freestream[4], case.body.x_base());
        // CN = 2 / S_ref sum(-cp A_z), CA = 2 / S_ref sum(-cp A_x) over the body and fin walls (the sting is Part::Other)
        let s_ref = std::f64::consts::FRAC_PI_4;
        let (mut fx, mut fz, mut mz) = (0.0, 0.0, 0.0);
        for w in f.walls.iter().filter(|w| w.part != Part::Other) {
            let a = area_vector(&f, w);
            let xc = 0.25 * w.nodes.iter().map(|&n| f64::from(f.nodes[n as usize][0])).sum::<f64>();
            fx += -f64::from(w.cp) * a[0];
            fz += -f64::from(w.cp) * a[2];
            mz += -f64::from(w.cp) * a[2] * xc;
        }
        let (cn, ca, xcp) = (2.0 / s_ref * fz, 2.0 / s_ref * fx, mz / fz);
        assert!(reported.cn.abs() > 0.01, "the state has a normal force: {reported:?}");
        assert!((cn - reported.cn).abs() < 2e-4 * reported.cn.abs().max(1.0), "CN {cn} vs {}", reported.cn);
        assert!((ca - reported.ca).abs() < 2e-4 * reported.ca.abs().max(1.0), "CA {ca} vs {}", reported.ca);
        assert!((xcp - reported.x_cp).abs() < 2e-3, "x_cp {xcp} vs {}", reported.x_cp);
    }

    #[test]
    fn every_wall_faces_the_flow_and_the_parts_are_named() {
        let (case, s) = stepped(2.0, 4.0, 5);
        let f = field3_of_solver(&case, &s, 5, 1.0);
        assert_eq!(f.cells(), s.grid.ni * s.grid.nj * s.grid.nk);
        assert_eq!(f.nodes.len(), s.grid.x.len());
        for w in &f.walls {
            let a = area_vector(&f, w);
            let centre = {
                let (ni, nj) = (f.ni, f.nj);
                let n = w.cell as usize;
                f.centre(n % ni, n / ni % nj, n / (ni * nj))
            };
            let face = w.nodes.map(|n| f.nodes[n as usize]);
            let mid = [0, 1, 2].map(|m| 0.25 * face.iter().map(|p| p[m]).sum::<f32>());
            let toward = [0, 1, 2].map(|m| f64::from(centre[m] - mid[m]));
            assert!(a[0] * toward[0] + a[1] * toward[1] + a[2] * toward[2] > 0.0, "wall {w:?} faces into the body");
            assert!(f.is_fluid(w.cell as usize));
        }
        let count = |p: fn(&Part) -> bool| f.walls.iter().filter(|w| p(&w.part)).count();
        // the body rows: one face per fluid cell of row j = 0 behind the axis cells (the fin roots are solid cells of that row)
        let rows = (0..s.grid.nk).flat_map(|k| (s.grid.axis_cells..s.grid.ni).map(move |i| (i, k))).filter(|&(i, k)| !f.solid[s.grid.cell(i, 0, k)]).count();
        assert_eq!(count(|p| matches!(p, Part::Nose | Part::Body | Part::Other)), rows);
        assert!(count(|p| *p == Part::Nose) > 0 && count(|p| *p == Part::Body) > 0 && count(|p| *p == Part::Other) > 0);
        assert!(count(|p| *p == Part::Fin(0)) > 0);
        assert_eq!(f.fin_theta, vec![PI / 4.0, 3.0 * PI / 4.0]);
        assert_eq!(f.alpha_deg, 4.0f64.to_radians().to_degrees());
    }

    #[test]
    fn the_mesh_id_follows_the_mesh_and_not_the_state() {
        let (case, mut s) = stepped(2.0, 4.0, 2);
        let a = field3_of_solver(&case, &s, 2, 1.0);
        s.step();
        let b = field3_of_solver(&case, &s, 3, 1.0);
        assert_eq!(a.mesh_id, b.mesh_id);
        assert_ne!(a.prim, b.prim);
        let c = field3_of_solver(&case, &s, 3, 0.1);
        assert_ne!(a.mesh_id, c.mesh_id);
    }

    #[test]
    fn the_free_stream_has_no_wall_pressure() {
        let case = Case3::tiny_anf(2.0, 0.0);
        let s = case.build_solver().unwrap();
        let f = field3_of_solver(&case, &s, 0, 1.0);
        assert!(f.walls.iter().all(|w| w.cp.abs() < 1e-6));
        assert!(f.prim.iter().all(|p| (p[0] - 1.0).abs() < 1e-6));
        assert!((f.mach_number(0) - 2.0).abs() < 1e-5);
    }
}
