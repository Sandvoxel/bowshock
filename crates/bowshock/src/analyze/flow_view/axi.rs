//! The one place the flow viewer names the axisymmetric 2D solver's [`Field`]: [`field3_from_2d`] turns it into the one-cell wedge of a
//! [`Field3`] the viewer draws. Deleting the 2D solver deletes this file, its call in `cfd::FieldRef::from_2d` and the 2D fixtures of the tests.

use bowshock_core::{Field, Field3, Part, Symmetry, WallFace};

/// Width of the wedge a 2D field is revolved into (radians): 5 degrees, 72 of them fill the circle.
pub const WEDGE: f32 = std::f32::consts::TAU / 72.0;

/// The one-cell wedge `theta = 0 .. WEDGE` of the axisymmetric field `f`: node `(i, j, k)` is `(x, r sin(theta_k), r cos(theta_k))`, the
/// state of cell `(i, j)` is that of the 2D cell with its radial velocity turned to the wedge's middle azimuth, the walls are the
/// body surface faces of the 2D wall list, and the symmetry is [`Symmetry::Axisymmetric`].
pub fn field3_from_2d(f: &Field) -> Field3 {
    let (ni, nj) = (f.ni, f.nj);
    let mut nodes = Vec::with_capacity(2 * (ni + 1) * (nj + 1));
    for k in 0..2 {
        let (s, c) = (WEDGE * k as f32).sin_cos();
        for n in 0..(ni + 1) * (nj + 1) {
            nodes.push([f.x[n], f.r[n] * s, f.r[n] * c]);
        }
    }
    let (s, c) = (0.5 * WEDGE).sin_cos();
    let prim: Vec<[f32; 5]> = (0..ni * nj).map(|q| [f.rho[q], f.u[q], f.v[q] * s, f.v[q] * c, f.p[q]]).collect();
    let [rho, u, v, p] = f.freestream;
    let mut out = Field3 {
        mach: f.mach,
        alpha_deg: 0.0,
        iteration: f.iteration,
        freestream: [rho, u, v, 0.0, p],
        ni,
        nj,
        nk: 1,
        mesh_id: 0,
        nodes,
        solid: f.solid.clone(),
        prim,
        eddy_ratio: Vec::new(),
        walls: Vec::new(),
        fin_theta: Vec::new(),
        symmetry: Symmetry::Axisymmetric { theta0: 0.0, theta1: WEDGE },
    };
    // the body surface faces, the ones `Field::wall` gives a Cp for (the same filter, in the same order)
    let cps = f.wall();
    let mut next = cps.iter();
    for i in 0..ni {
        let j = f.first_fluid_row(i);
        if j + 1 >= nj {
            continue;
        }
        let (a, b) = (f.node(i, j), f.node(i + 1, j));
        if 0.5 * (a.0 + b.0) <= 0.0 || 0.5 * (a.1 + b.1) <= 0.0 {
            continue;
        }
        let Some(w) = next.next() else { break };
        let id = |i: usize, j: usize, k: usize| ((k * (nj + 1) + j) * (ni + 1) + i) as u32;
        let mut nodes4 = [id(i, j, 0), id(i + 1, j, 0), id(i + 1, j, 1), id(i, j, 1)];
        // counter-clockwise seen from the fluid: the normal (p2 - p0) x (p3 - p1) towards the cell
        let p = |n: u32| out.nodes[n as usize];
        let (d02, d13) = (sub(p(nodes4[2]), p(nodes4[0])), sub(p(nodes4[3]), p(nodes4[1])));
        let n = cross(d02, d13);
        let (face, cell) = (out.centre(i, j, 0), [0, 1, 2].map(|m| 0.25 * nodes4.iter().map(|&q| p(q)[m]).sum::<f32>()));
        let toward = [face[0] - cell[0], face[1] - cell[1], face[2] - cell[2]];
        if n[0] * toward[0] + n[1] * toward[1] + n[2] * toward[2] < 0.0 {
            nodes4.swap(1, 3);
        }
        out.walls.push(WallFace { nodes: nodes4, cell: (j * ni + i) as u32, part: Part::Body, cp: w.cp as f32, cf: [0.0; 3] });
    }
    out.mesh_id = bowshock_core::engine::field3::mesh_id(&out);
    out
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
