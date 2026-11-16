//! OBJ and binary STL export of the rocket's shaded model.
//!
//! The triangles are the viewport's own ([`Mesh::from_rocket`]): the airframe's skin, the fins, lugs and buttons and, if asked, the
//! internal solids. Faces that exist only to draw a cutaway (the hatched wall caps and the flat section fills) and the shock cord,
//! which the viewport draws as a hairline, are left out.
//!
//! Units are millimetres. The viewport puts the nose tip at `z = 0` with `+Z` running aft and `+Y` up; the export turns the model half
//! a turn about the Y axis, `(x, y, z) -> (-x, y, length - z)`, so the nose points along `+Z` (up) with the tail on the XY plane and
//! the rocket keeps its handedness (a plain flip of Z would mirror it, and a lug or a canted fin would end up on the wrong side).
//! Every triangle is wound counter-clockwise seen from outside, the side its normals point to (the bore of a hollow tube faces
//! the axis, as in the viewport).

use std::collections::HashMap;
use std::fmt::Write;

use bowshock_model::Rocket;

use crate::viewport::{GpuVertex, Marks, Mesh, LAYER_INTERNAL, LAYER_RADIAL, LAYER_SHELL};

/// Smallest triangle area kept, mm².
const MIN_AREA: f64 = 1e-12;
/// Quantum of the vertex and normal de-duplication in the OBJ, mm (normals: the same step on the unit vector).
const QUANTUM: f64 = 1e-4;

/// One exported triangle, in millimetres.
#[derive(Clone, Copy, Debug)]
struct Tri {
    p: [[f64; 3]; 3],
    n: [[f64; 3]; 3],
    /// The system slot (0 structure, 1 aero, 2 propulsion, 3 recovery, 4 avionics): the OBJ's group.
    system: u8,
}

/// The names of the system slots, as the OBJ groups them.
const SYSTEMS: [&str; 5] = ["structure", "aero", "propulsion", "recovery", "avionics"];

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn unit(a: [f64; 3]) -> [f64; 3] {
    let l = dot(a, a).sqrt();
    if l > 0.0 {
        [a[0] / l, a[1] / l, a[2] / l]
    } else {
        [0.0, 0.0, 1.0]
    }
}

/// The model's triangles in the export frame, outward wound, degenerate ones dropped.
fn triangles(rocket: &Rocket, internals: bool) -> Vec<Tri> {
    let mesh = Mesh::from_rocket(rocket, Marks::default());
    let length_mm = rocket.length() * 1000.0;
    let mut out = Vec::new();
    // The cutaway's faces (`mesh.caps`) are not part of the model.
    for range in [mesh.inner.clone(), mesh.outer.clone()] {
        for tri in mesh.vertices[range.start as usize..range.end as usize].as_chunks::<3>().0 {
            let layer = tri[0].layer;
            let keep = layer == LAYER_SHELL || layer == LAYER_RADIAL || (internals && layer == LAYER_INTERNAL);
            if !keep {
                continue;
            }
            let place = |v: &GpuVertex| {
                let [x, y, z] = v.position.map(f64::from);
                let [nx, ny, nz] = v.normal.map(f64::from);
                ([-x * 1000.0, y * 1000.0, length_mm - z * 1000.0], [-nx, ny, -nz])
            };
            let (a, b, c) = (place(&tri[0]), place(&tri[1]), place(&tri[2]));
            let mut t = Tri { p: [a.0, b.0, c.0], n: [a.1, b.1, c.1], system: tri[0].system as u8 };
            if t.p.iter().flatten().chain(t.n.iter().flatten()).any(|v| !v.is_finite()) {
                continue;
            }
            let face = cross(sub(t.p[1], t.p[0]), sub(t.p[2], t.p[0]));
            if dot(face, face).sqrt() / 2.0 < MIN_AREA {
                continue;
            }
            let mean = [t.n[0][0] + t.n[1][0] + t.n[2][0], t.n[0][1] + t.n[1][1] + t.n[2][1], t.n[0][2] + t.n[1][2] + t.n[2][2]];
            if dot(face, mean) < 0.0 {
                t.p.swap(1, 2);
                t.n.swap(1, 2);
            }
            out.push(t);
        }
    }
    out
}

fn quantise(v: [f64; 3]) -> (i64, i64, i64) {
    let q = |x: f64| (x / QUANTUM).round() as i64;
    (q(v[0]), q(v[1]), q(v[2]))
}

/// Wavefront OBJ text of the rocket, in millimetres, one group per system. `internals` adds the internal parts.
pub fn obj(rocket: &Rocket, internals: bool) -> String {
    let mut tris = triangles(rocket, internals);
    tris.sort_by_key(|t| t.system);
    let name = if rocket.name.trim().is_empty() { "rocket" } else { rocket.name.trim() };
    let mut out = String::new();
    let _ = writeln!(out, "# Bowshock {name} · units: mm");
    let _ = writeln!(out, "# nose along +Z, tail on the XY plane");
    let mut positions: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut normals: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let (mut v_lines, mut vn_lines, mut faces) = (String::new(), String::new(), String::new());
    let mut group = None;
    for t in &tris {
        if group != Some(t.system) {
            group = Some(t.system);
            let _ = writeln!(faces, "g {}", SYSTEMS[usize::from(t.system).min(SYSTEMS.len() - 1)]);
        }
        let mut idx = [(0usize, 0usize); 3];
        for i in 0..3 {
            let p = t.p[i];
            let next = positions.len() + 1;
            let vi = *positions.entry(quantise(p)).or_insert_with(|| {
                let _ = writeln!(v_lines, "v {:.4} {:.4} {:.4}", p[0], p[1], p[2]);
                next
            });
            let n = unit(t.n[i]);
            let next = normals.len() + 1;
            let ni = *normals.entry(quantise(n)).or_insert_with(|| {
                let _ = writeln!(vn_lines, "vn {:.4} {:.4} {:.4}", n[0], n[1], n[2]);
                next
            });
            idx[i] = (vi, ni);
        }
        let _ = writeln!(faces, "f {}//{} {}//{} {}//{}", idx[0].0, idx[0].1, idx[1].0, idx[1].1, idx[2].0, idx[2].1);
    }
    out.push_str(&v_lines);
    out.push_str(&vn_lines);
    out.push_str(&faces);
    out
}

/// Binary STL of the rocket, in millimetres. `internals` adds the internal parts.
pub fn stl(rocket: &Rocket, internals: bool) -> Vec<u8> {
    let tris = triangles(rocket, internals);
    let name = if rocket.name.trim().is_empty() { "rocket" } else { rocket.name.trim() };
    let mut header = format!("Bowshock {name} (mm)").into_bytes();
    header.truncate(80);
    header.resize(80, b' ');
    let mut out = Vec::with_capacity(84 + 50 * tris.len());
    out.extend_from_slice(&header);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for t in &tris {
        let n = unit(cross(sub(t.p[1], t.p[0]), sub(t.p[2], t.p[0])));
        for v in n.iter().chain(t.p.iter().flatten()) {
            out.extend_from_slice(&(*v as f32).to_le_bytes());
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example() -> Rocket {
        Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).expect("the example loads")
    }

    fn max_z(obj: &str) -> f64 {
        obj.lines().filter_map(|l| l.strip_prefix("v ")).map(|l| l.split(' ').nth(2).unwrap().parse::<f64>().unwrap()).fold(f64::MIN, f64::max)
    }

    #[test]
    fn obj_of_the_example_is_a_millimetre_model_with_the_nose_up() {
        let rocket = example();
        let text = obj(&rocket, false);
        let vertices = text.lines().filter(|l| l.starts_with("v ")).count();
        assert!(vertices > 1000, "{vertices} vertices");
        let length_mm = rocket.length() * 1000.0;
        assert!((max_z(&text) - length_mm).abs() < 0.5, "max Z {} vs length {length_mm}", max_z(&text));
        assert!(text.contains("g structure") && text.contains("g aero"));
        assert!(text.lines().any(|l| l.starts_with("f ") && l.contains("//")));
        // Every index the faces use exists.
        let vn = text.lines().filter(|l| l.starts_with("vn ")).count();
        for face in text.lines().filter_map(|l| l.strip_prefix("f ")) {
            for corner in face.split(' ') {
                let (v, n) = corner.split_once("//").unwrap();
                assert!((1..=vertices).contains(&v.parse::<usize>().unwrap()) && (1..=vn).contains(&n.parse::<usize>().unwrap()));
            }
        }
    }

    #[test]
    fn internals_add_triangles() {
        let rocket = example();
        let (without, with) = (triangles(&rocket, false).len(), triangles(&rocket, true).len());
        assert!(with > without, "{with} vs {without}");
    }

    #[test]
    fn stl_length_is_header_count_and_fifty_bytes_a_triangle() {
        let rocket = example();
        let bytes = stl(&rocket, false);
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        assert!(count > 1000);
        assert_eq!(bytes.len(), 84 + 50 * count);
        assert!(bytes.starts_with(b"Bowshock "));
        assert_eq!(count, triangles(&rocket, false).len());
    }

    #[test]
    fn triangles_are_wound_outward() {
        for t in triangles(&example(), true) {
            let face = cross(sub(t.p[1], t.p[0]), sub(t.p[2], t.p[0]));
            let mean = [t.n[0][0] + t.n[1][0] + t.n[2][0], t.n[0][1] + t.n[1][1] + t.n[2][1], t.n[0][2] + t.n[1][2] + t.n[2][2]];
            assert!(dot(face, mean) >= 0.0);
        }
    }
}
