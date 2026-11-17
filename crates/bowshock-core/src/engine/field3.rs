//! A [`Field3`] from the 3D solver's state: the grid, the conserved state (`Grid3::cell` order, from the CPU solver or downloaded from
//! the GPU), the free stream and the case's body.
//!
//! The walls are the faces of `run3::wall_faces`, the same ones the force integration (`run3::wall_forces`, and the device-side reduction of the GPU runner) sums: every
//! face between a fluid cell and a wall cell (a solid cell with a part) and the `j = 0` body rows of a sting grid. The part of a face is
//! its [`Wall`] class (the part id of the cell behind it and the direction of the face), the wall pressure the one of the force integration
//! (`run3::wall_pressure`): the pressure of the first two fluid cells normal to the wall extrapolated linearly to the face centre, the
//! first cell's alone when there is no second fluid cell. The viewer's `sum -cp A n` over the walls therefore reproduces the pressure force
//! the integration reports (a test below). The symmetry of the solved domain is the grid's.

use crate::engine::case3::Case3;
use crate::engine::grid3::{Domain3, FinPhase};
use crate::engine::run3::{wall_faces, wall_pressure, Wall, WallFaceRef};
use crate::field3::{Field3, Part, Symmetry, WallFace};
use bowshock_cfd::gas3::{cons_to_prim3, Cons3};
use bowshock_cfd::grid3::{self, Grid3, PART_FLUID};
use bowshock_cfd::solver3::Solver3;
use std::f32::consts::PI;
use std::fmt;
use std::sync::Arc;

/// The receiver of the live snapshots of a 3D run ([`Analysis::on_field3`](crate::Analysis::on_field3)) and how often it is called. It is
/// carried by the [`Case3`]; a runner that holds the state `u` of the grid `g` after `it` iterations calls [`Snapshots3::due`] and then
/// [`Snapshots3::send`].
#[derive(Clone)]
pub struct Snapshots3 {
    /// Iterations between two snapshots: a multiple of [`SAMPLE_INTERVAL`](crate::progress::SAMPLE_INTERVAL).
    pub every: usize,
    /// Length of a nose diameter in the unit of the field's coordinates (metres for a dimensional body).
    pub scale: f64,
    call: Arc<dyn Fn(Field3) + Send + Sync>,
}

impl Snapshots3 {
    /// A receiver `f` called about every `every` iterations (rounded up to a multiple of the progress sampling interval).
    pub fn new(every: usize, scale: f64, f: impl Fn(Field3) + Send + Sync + 'static) -> Snapshots3 {
        let step = crate::progress::SAMPLE_INTERVAL;
        Snapshots3 { every: every.max(1).div_ceil(step) * step, scale, call: Arc::new(f) }
    }

    /// Whether a snapshot is due after `iteration` iterations.
    pub fn due(&self, iteration: usize) -> bool {
        iteration.is_multiple_of(self.every)
    }

    /// Hands the field of the state `u` on `g` after `iteration` iterations to the receiver.
    pub fn send(&self, case: &Case3, g: &Grid3, u: &[Cons3], iteration: usize) {
        (self.call)(field3_of(case, g, u, iteration, self.scale));
    }
}

impl fmt::Debug for Snapshots3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Snapshots3").field("every", &self.every).field("scale", &self.scale).finish()
    }
}

/// The part of the vehicle a wall face belongs to. The only function that names the viewer's parts from the grid's: the face's [`Wall`] class
/// (from the part id of the cell behind it and the direction of the face) and the part behind it.
fn part_of(w: &WallFaceRef) -> Part {
    match (w.class, w.part) {
        (Wall::Body(0), _) => Part::Nose,
        (Wall::Body(_), _) => Part::Body,
        (Wall::Base, _) => Part::Base,
        (Wall::Sting, _) => Part::Other,
        (Wall::Jet, _) => Part::Jet,
        (_, grid3::Part::Fin(n)) => Part::Fin(n as u8),
        (_, grid3::Part::Appendage(n)) => Part::Appendage(n as u8),
        (Wall::FinSide | Wall::FinLe | Wall::FinTe | Wall::FinTip | Wall::Appendage, _) => Part::Other,
    }
}

/// The symmetry of the solved domain of `g`, as the viewer replicates it (azimuth `0..theta_span`).
fn symmetry_of(g: &Grid3) -> Symmetry {
    let span = g.theta_span as f32;
    match g.sym {
        grid3::Symmetry::Half => Symmetry::Mirror { theta0: 0.0, theta1: PI },
        grid3::Symmetry::Sector { .. } => Symmetry::Mirror { theta0: 0.0, theta1: span },
        grid3::Symmetry::Wedge => Symmetry::Axisymmetric { theta0: 0.0, theta1: span },
    }
}

/// The azimuths of the fin mid-planes inside the solved domain.
fn fin_theta_of(case: &Case3, g: &Grid3) -> Vec<f32> {
    let Some(f) = &case.grid.fins else { return Vec::new() };
    let n = f64::from(f.count);
    let at = |phase: f64| (0..f.count).map(move |m| phase + std::f64::consts::TAU * f64::from(m) / n);
    let thetas: Vec<f64> = match case.grid.domain {
        Domain3::Half { phase: FinPhase::Between } => at(std::f64::consts::PI / n).collect(),
        Domain3::Half { phase: FinPhase::InPlane } => at(0.0).collect(),
        Domain3::Sector => vec![0.0],
        Domain3::Wedge { .. } => Vec::new(),
    };
    thetas.into_iter().filter(|&t| t <= g.theta_span + 1e-9).map(|t| t as f32).collect()
}

/// [`Field3`] of the CPU solver `s` after `iteration` iterations, lengths in metres through `scale` (metres per nose diameter).
pub fn field3_of_solver(case: &Case3, s: &Solver3, iteration: usize, scale: f64) -> Field3 {
    field3_of(case, &s.grid, &s.u, iteration, scale)
}

/// [`Field3`] of the state `u` on `g` after `iteration` iterations, lengths in metres through `scale` (metres per nose diameter).
pub fn field3_of(case: &Case3, g: &Grid3, u: &[Cons3], iteration: usize, scale: f64) -> Field3 {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let fs = case.freestream();
    let solid: Vec<bool> = g.part.iter().map(|&p| p != PART_FLUID).collect();
    let prim: Vec<[f32; 5]> = u.iter().zip(&solid).map(|(q, &s)| (if s { fs } else { cons_to_prim3(q) }).map(|v| v as f32)).collect();
    let nodes: Vec<[f32; 3]> = (0..g.x.len()).map(|n| [g.x[n], g.y[n], g.z[n]].map(|v| (v * scale) as f32)).collect();
    let walls = walls_of(g, u, &fs);
    let fin_theta = fin_theta_of(case, g);
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
        symmetry: symmetry_of(g),
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
fn walls_of(g: &Grid3, u: &[Cons3], fs: &[f64; 5]) -> Vec<WallFace> {
    let (ni, nj) = (g.ni, g.nj);
    let node = |i: usize, j: usize, k: usize| (k * (nj + 1) + j) * (ni + 1) + i;
    let q_inf = 0.5 * fs[0] * (fs[1] * fs[1] + fs[2] * fs[2] + fs[3] * fs[3]);
    let pt = |n: usize| [g.x[n], g.y[n], g.z[n]];
    let mut walls = Vec::new();
    for w in &wall_faces(g) {
        let pw = wall_pressure(g, w, |c| cons_to_prim3(&u[c])[4]);
        let mut ids = w.corners.map(|(i, j, k)| node(i, j, k));
        // wind counter-clockwise seen from the fluid: the area vector (p2 - p0) x (p3 - p1) / 2 along the normal into the flow, which is the
        // face normal when the fluid is on its +index side (`sign` -1) and its negative otherwise
        let (d02, d13) = (sub(pt(ids[2]), pt(ids[0])), sub(pt(ids[3]), pt(ids[1])));
        let s = cross(d02, d13);
        let into = -w.sign;
        if s[0] * w.face.n[0] * into + s[1] * w.face.n[1] * into + s[2] * w.face.n[2] * into < 0.0 {
            ids.swap(1, 3);
        }
        walls.push(WallFace { nodes: ids.map(|n| n as u32), cell: w.a as u32, part: part_of(w), cp: ((pw - fs[4]) / q_inf) as f32, cf: [0.0; 3] });
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
        // the body rows: one face per fluid cell of row j = 0 inside the grid's body runs, off the axis (the fin roots are solid cells of that row)
        let g = &s.grid;
        let rows = (0..g.nk)
            .flat_map(|k| (0..g.ni).map(move |i| (i, k)))
            .filter(|&(i, k)| g.wall_j0.iter().any(|(r, _)| r.contains(&i)) && !g.is_axis(i, 0) && !f.solid[g.cell(i, 0, k)])
            .count();
        assert_eq!(count(|p| matches!(p, Part::Nose | Part::Body | Part::Other)), rows);
        assert!(count(|p| *p == Part::Nose) > 0 && count(|p| *p == Part::Body) > 0 && count(|p| *p == Part::Other) > 0, "the sting is a wall too, and hidden");
        assert!(count(|p| *p == Part::Fin(0)) > 0);
        assert_eq!(f.symmetry, Symmetry::Mirror { theta0: 0.0, theta1: PI });
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

    #[test]
    fn the_symmetry_the_fins_and_the_parts_come_from_the_grid_of_each_domain() {
        // a fin sector: the sector's two mirror planes, the half fin on the first, the walls of the fin and the body
        let mut sector = Case3::tiny_anf(2.0, 0.0);
        sector.grid.domain = Domain3::Sector;
        sector.grid.nk = 6;
        let s = sector.build_solver().unwrap();
        let f = field3_of_solver(&sector, &s, 0, 1.0);
        assert_eq!(f.symmetry, Symmetry::Mirror { theta0: 0.0, theta1: PI / 4.0 });
        assert_eq!(f.fin_theta, vec![0.0]);
        assert_eq!(f.symmetry.tiles().len(), 8, "four fins: eight copies of the sector");
        assert!(f.walls.iter().any(|w| w.part == Part::Fin(0)) && f.walls.iter().any(|w| w.part == Part::Nose));
        // a finless wedge on the base-wake grid: any rotation is a symmetry, no fins, and the base is a wall of its own
        let wedge = Case3::wedge(2.0, Case3::anf_body(), 0);
        let s = wedge.build_solver().unwrap();
        let f = field3_of_solver(&wedge, &s, 0, 1.0);
        assert_eq!((f.nk, f.fin_theta.len()), (1, 0));
        assert!(matches!(f.symmetry, Symmetry::Axisymmetric { theta0, theta1 } if theta0 == 0.0 && (theta1 - 2.0f32.to_radians()).abs() < 1e-6));
        assert!(f.walls.iter().any(|w| w.part == Part::Base), "the base of the base-wake grid");
        assert!(f.walls.iter().any(|w| w.part == Part::Nose) && f.walls.iter().any(|w| w.part == Part::Body));
        assert!(f.walls.iter().all(|w| w.cp.abs() < 1e-6), "the free stream has no wall pressure");
    }
}
