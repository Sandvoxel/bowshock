//! Triangle emitters for the viewport mesh: surfaces of revolution, tubes, discs, boxes, thin segments and extruded fin plates.
//!
//! Every emitter appends [`GpuVertex`]es carrying the part's [`Tag`] (state, system, layer, exploded-view offset) so the shader
//! can colour by system and the view modes can treat the layers differently. Winding is counter-clockwise seen from the side
//! the normals point to.

use std::f32::consts::TAU;

use glam::{Vec2, Vec3};

use super::radial::{tilt_dir, tilt_point};
use super::scene::Vertex;

/// Per-vertex data the GPU reads.
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuVertex {
    /// World position, metres.
    pub position: [f32; 3],
    /// Unit normal.
    pub normal: [f32; 3],
    /// Where the vertex moves to in the exploded view (added to `position` times the animation).
    pub explode: [f32; 3],
    /// Distance along the axis to the part's fore and aft faces, metres: the shader draws a thin edge line near either.
    pub edge: [f32; 2],
    /// Part state: [`super::scene::NORMAL`], `GHOST`, `HOVERED` or `SELECTED`.
    pub state: f32,
    /// 0 for the airframe's base colour, 1 for the fin colour (radial parts).
    pub material: f32,
    /// System colour slot: 0 structure, 1 aero, 2 propulsion, 3 recovery, 4 avionics.
    pub system: f32,
    /// One of the `LAYER_*` constants.
    pub layer: f32,
}

/// Airframe skin: nose, tubes, transitions. Cut away in Cutaway, a faint shell in X-ray.
pub const LAYER_SHELL: f32 = 0.0;
/// Fins, lugs, buttons: outside the airframe and never cut.
pub const LAYER_RADIAL: f32 = 1.0;
/// Internal solids: tubes, rings, recovery gear, avionics, the motor. Cut in Cutaway.
pub const LAYER_INTERNAL: f32 = 2.0;
/// A cut wall (hatched `cut-face`).
pub const LAYER_CAP: f32 = 3.0;
/// A cut solid (flat in its system colour).
pub const LAYER_FILL: f32 = 4.0;
/// Shock cord: internal but never cut (too thin to cap).
pub const LAYER_CORD: f32 = 5.0;

/// A far distance: no edge line.
const NO_EDGE: [f32; 2] = [1.0e3, 1.0e3];

/// What every vertex of one part carries.
#[derive(Debug, Clone, Copy)]
pub struct Tag {
    pub state: f32,
    pub material: f32,
    pub system: f32,
    pub layer: f32,
    pub explode: Vec3,
}

/// A growing triangle list with the current [`Tag`].
#[derive(Debug, Default)]
pub struct Sink {
    pub vertices: Vec<GpuVertex>,
    pub tag: Option<Tag>,
}

impl Sink {
    fn tag(&self) -> Tag {
        self.tag.unwrap_or(Tag { state: 0.0, material: 0.0, system: 0.0, layer: LAYER_SHELL, explode: Vec3::ZERO })
    }

    fn vertex(&self, p: Vec3, n: Vec3, edge: [f32; 2]) -> GpuVertex {
        let t = self.tag();
        GpuVertex {
            position: p.to_array(),
            normal: n.to_array(),
            explode: t.explode.to_array(),
            edge,
            state: t.state,
            material: t.material,
            system: t.system,
            layer: t.layer,
        }
    }

    /// One triangle, wound counter-clockwise as seen from the side its vertex normals point to.
    pub fn tri(&mut self, p: [Vec3; 3], n: [Vec3; 3]) {
        let flip = (p[1] - p[0]).cross(p[2] - p[0]).dot(n[0] + n[1] + n[2]) < 0.0;
        let order = if flip { [0, 2, 1] } else { [0, 1, 2] };
        for i in order {
            let v = self.vertex(p[i], n[i], NO_EDGE);
            self.vertices.push(v);
        }
    }

    /// A planar polygon with one normal, as a fan (convex polygons only).
    pub fn fan(&mut self, points: &[Vec3], normal: Vec3) {
        for i in 1..points.len().saturating_sub(1) {
            self.tri([points[0], points[i], points[i + 1]], [normal; 3]);
        }
    }

    /// A planar polygon facing away from `inside`, as a fan.
    pub fn face(&mut self, points: &[Vec3], inside: Vec3) {
        let normal = (points[1] - points[0]).cross(points[2] - points[0]).normalize_or_zero();
        let normal = if normal.dot(points[0] - inside) < 0.0 { -normal } else { normal };
        self.fan(points, normal);
    }

    /// Takes triangles another emitter wrote as [`Vertex`]es (the radial parts' own).
    pub fn adopt(&mut self, source: &[Vertex]) {
        for v in source {
            let g = self.vertex(Vec3::from(v.position), Vec3::from(v.normal), NO_EDGE);
            self.vertices.push(GpuVertex { state: v.state, material: v.material, ..g });
        }
    }

    /// Revolves a fore-to-aft profile of `(z, r)` points about Z with `sides` facets. Normals follow the profile's slope, so a
    /// smooth profile shades smoothly. Vertices carry their distance to `faces` (the part's fore and aft stations) for the edge
    /// lines between stacked parts.
    pub fn revolve(&mut self, profile: &[(f32, f32)], sides: usize, faces: Option<(f32, f32)>) {
        if profile.len() < 2 {
            return;
        }
        let normals: Vec<(f32, f32)> = (0..profile.len())
            .map(|i| {
                let (a, b) = (profile[i.saturating_sub(1)], profile[(i + 1).min(profile.len() - 1)]);
                let (tz, tr) = (b.0 - a.0, b.1 - a.1);
                let len = tz.hypot(tr).max(f32::MIN_POSITIVE);
                (-tr / len, tz / len)
            })
            .collect();
        let at = |s: &Sink, i: usize, sin: f32, cos: f32| {
            let (z, r) = profile[i];
            let (nz, nr) = normals[i];
            let edge = faces.map_or(NO_EDGE, |(f, a)| [z - f, a - z]);
            s.vertex(Vec3::new(r * cos, r * sin, z), Vec3::new(nr * cos, nr * sin, nz), edge)
        };
        for k in 0..sides {
            let (s0, c0) = (TAU * k as f32 / sides as f32).sin_cos();
            let (s1, c1) = (TAU * (k + 1) as f32 / sides as f32).sin_cos();
            for i in 0..profile.len() - 1 {
                let (p00, p10, p01, p11) = (at(self, i, s0, c0), at(self, i, s1, c1), at(self, i + 1, s0, c0), at(self, i + 1, s1, c1));
                // A zero radius collapses an edge of the quad into a point: skip the degenerate triangle.
                if profile[i].1 > 0.0 {
                    self.vertices.extend([p00, p10, p11]);
                }
                if profile[i + 1].1 > 0.0 {
                    self.vertices.extend([p00, p11, p01]);
                }
            }
        }
    }

    /// A flat annulus (a disc when `r_in` is 0) at `z`, facing +Z (`front`) or -Z.
    pub fn annulus(&mut self, r_in: f32, r_out: f32, z: f32, front: bool, sides: usize, faces: Option<(f32, f32)>) {
        let n = Vec3::new(0.0, 0.0, if front { 1.0 } else { -1.0 });
        let edge = faces.map_or(NO_EDGE, |(f, a)| [z - f, a - z]);
        for k in 0..sides {
            let (s0, c0) = (TAU * k as f32 / sides as f32).sin_cos();
            let (s1, c1) = (TAU * (k + 1) as f32 / sides as f32).sin_cos();
            let pt = |r: f32, s: f32, c: f32| Vec3::new(r * c, r * s, z);
            let (a, b, c, d) = (pt(r_in, s0, c0), pt(r_out, s0, c0), pt(r_out, s1, c1), pt(r_in, s1, c1));
            let tris: Vec<[Vec3; 3]> = if r_in > 0.0 { vec![[a, b, c], [a, c, d]] } else { vec![[b, c, a]] };
            for t in tris {
                let flip = (t[1] - t[0]).cross(t[2] - t[0]).z * n.z < 0.0;
                let ord = if flip { [0, 2, 1] } else { [0, 1, 2] };
                for i in ord {
                    let v = self.vertex(t[i], n, edge);
                    self.vertices.push(v);
                }
            }
        }
    }

    /// A tube from `z0` to `z1`: outer wall, inner wall (when `r_in` is more than 0) and both end faces, or a solid cylinder.
    pub fn tube(&mut self, r_out: f32, r_in: f32, z0: f32, z1: f32, sides: usize) {
        self.revolve(&[(z0, r_out), (z1, r_out)], sides, None);
        if r_in > 0.0 {
            self.revolve(&[(z1, r_in), (z0, r_in)], sides, None);
        }
        self.annulus(r_in, r_out, z0, false, sides, None);
        self.annulus(r_in, r_out, z1, true, sides, None);
    }

    /// An axis-aligned box centred on the axis: half-width `hw` along X, half-height `hh` along Y, from `z0` to `z1`.
    pub fn cuboid(&mut self, hw: f32, hh: f32, z0: f32, z1: f32) {
        let c = |x: f32, y: f32, z: f32| Vec3::new(x * hw, y * hh, z);
        let centre = Vec3::new(0.0, 0.0, (z0 + z1) / 2.0);
        for (a, b, cc, d) in [
            (c(-1.0, -1.0, z0), c(1.0, -1.0, z0), c(1.0, 1.0, z0), c(-1.0, 1.0, z0)),
            (c(-1.0, -1.0, z1), c(1.0, -1.0, z1), c(1.0, 1.0, z1), c(-1.0, 1.0, z1)),
            (c(-1.0, -1.0, z0), c(1.0, -1.0, z0), c(1.0, -1.0, z1), c(-1.0, -1.0, z1)),
            (c(-1.0, 1.0, z0), c(1.0, 1.0, z0), c(1.0, 1.0, z1), c(-1.0, 1.0, z1)),
            (c(-1.0, -1.0, z0), c(-1.0, 1.0, z0), c(-1.0, 1.0, z1), c(-1.0, -1.0, z1)),
            (c(1.0, -1.0, z0), c(1.0, 1.0, z0), c(1.0, 1.0, z1), c(1.0, -1.0, z1)),
        ] {
            self.face(&[a, b, cc, d], centre);
        }
    }

    /// A thin 6-sided cylinder between two points.
    pub fn segment(&mut self, a: Vec3, b: Vec3, radius: f32) {
        let axis = (b - a).normalize_or_zero();
        if axis == Vec3::ZERO {
            return;
        }
        let helper = if axis.z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
        let e1 = axis.cross(helper).normalize();
        let e2 = axis.cross(e1);
        const SIDES: usize = 6;
        let ring = |i: usize| {
            let (s, c) = (TAU * i as f32 / SIDES as f32).sin_cos();
            e1 * c + e2 * s
        };
        for i in 0..SIDES {
            let (r0, r1) = (ring(i), ring(i + 1));
            let (a0, a1, b0, b1) = (a + r0 * radius, a + r1 * radius, b + r0 * radius, b + r1 * radius);
            self.tri([a0, a1, b1], [r0, r1, r1]);
            self.tri([a0, b1, b0], [r0, r1, r0]);
        }
    }

    /// A fin plate: the polygon `outline` of `(z, height)` points (root edge at height 0) standing on a surface of radius `base`
    /// at `angle` (0 is +Y, increasing towards +X), `thickness` thick, its root sunk slightly into the parent, canted by `cant`
    /// radians about its span line through the root's leading edge (see [`tilt_point`]). Any simple polygon, convex or not.
    pub fn plate(&mut self, outline: &[(f32, f32)], z0: f32, base: f32, angle: f32, cant: f32, thickness: f32) {
        const SINK: f32 = 0.0005;
        if outline.len() < 3 {
            return;
        }
        let (s, c) = angle.sin_cos();
        let (u, t) = (Vec3::new(s, c, 0.0), Vec3::new(c, -s, 0.0));
        let t = tilt_dir(t, angle, cant);
        let half = t * (thickness / 2.0);
        let point = |&(z, h): &(f32, f32)| tilt_point(u * (base + h - if h <= 0.0 { SINK } else { 0.0 }) + Vec3::Z * (z0 + z), z0, angle, cant);
        let pts: Vec<Vec3> = outline.iter().map(point).collect();
        let flat: Vec<Vec2> = outline.iter().map(|&(z, h)| Vec2::new(z, h)).collect();
        for tri in triangulate(&flat) {
            let (a, b, cc) = (pts[tri[0]], pts[tri[1]], pts[tri[2]]);
            self.tri([a + half, b + half, cc + half], [t; 3]);
            self.tri([a - half, b - half, cc - half], [-t; 3]);
        }
        let centre = pts.iter().sum::<Vec3>() / pts.len() as f32;
        for i in 0..pts.len() {
            let j = (i + 1) % pts.len();
            if (pts[j] - pts[i]).length_squared() < 1e-12 {
                continue;
            }
            self.face(&[pts[i] + half, pts[j] + half, pts[j] - half, pts[i] - half], centre);
        }
    }

    /// A fin's fillet: two concave strips of quarter-circle section `radius` along the root from `z0` for `length`, one each
    /// side of a plate `thickness` thick, on a surface of radius `base` at `angle` and canted like the plate (see [`plate`](Self::plate)).
    pub fn fillet(&mut self, z0: f32, length: f32, base: f32, angle: f32, cant: f32, thickness: f32, radius: f32) {
        const SINK: f32 = 0.0005;
        const STEPS: usize = 6;
        if radius <= 0.0 || length <= 0.0 {
            return;
        }
        let (s, c) = angle.sin_cos();
        let (u, t) = (Vec3::new(s, c, 0.0), Vec3::new(c, -s, 0.0));
        let half = thickness / 2.0;
        // The section in (sideways, height): from the plate's face up at `radius` round to the surface `radius` away.
        let section: Vec<(f32, f32)> = (0..=STEPS)
            .map(|k| {
                let phi = std::f32::consts::PI + k as f32 / STEPS as f32 * std::f32::consts::FRAC_PI_2;
                (half + radius + radius * phi.cos(), radius + radius * phi.sin())
            })
            .collect();
        let at = |side: f32, z: f32, (lat, h): (f32, f32)| {
            tilt_point(u * (base + h - if h <= 0.0 { SINK } else { 0.0 }) + t * (side * lat) + Vec3::Z * z, z0, angle, cant)
        };
        for side in [1.0f32, -1.0] {
            let normal = |(lat, h): (f32, f32)| {
                // Towards the centre of the circle the section is cut from.
                let n = Vec2::new(half + radius - lat, radius - h).normalize_or_zero();
                tilt_dir(t * (side * n.x) + u * n.y, angle, cant)
            };
            for k in 0..STEPS {
                let (a, b) = (section[k], section[k + 1]);
                let (p00, p01) = (at(side, z0, a), at(side, z0, b));
                let (p10, p11) = (at(side, z0 + length, a), at(side, z0 + length, b));
                let (na, nb) = (normal(a), normal(b));
                self.tri([p00, p10, p11], [na, na, nb]);
                self.tri([p00, p11, p01], [na, nb, nb]);
            }
            // The ends close the strip.
            let mid = at(side, z0 + length / 2.0, (half + radius / 3.0, radius / 3.0));
            for z in [z0, z0 + length] {
                let mut ring: Vec<Vec3> = section.iter().map(|&q| at(side, z, q)).collect();
                ring.push(at(side, z, (half, 0.0)));
                self.face(&ring, mid);
            }
        }
    }
}

/// Ear-clipping triangulation of a simple polygon (either winding): triples of indices.
pub fn triangulate(poly: &[Vec2]) -> Vec<[usize; 3]> {
    let n = poly.len();
    let area: f32 = (0..n).map(|i| poly[i].perp_dot(poly[(i + 1) % n])).sum::<f32>() / 2.0;
    let mut idx: Vec<usize> = if area >= 0.0 { (0..n).collect() } else { (0..n).rev().collect() };
    let mut out = Vec::with_capacity(n.saturating_sub(2));
    let inside = |p: Vec2, a: Vec2, b: Vec2, c: Vec2| {
        let (d1, d2, d3) = ((b - a).perp_dot(p - a), (c - b).perp_dot(p - b), (a - c).perp_dot(p - c));
        d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0
    };
    let mut guard = 0;
    while idx.len() > 3 && guard < n * n {
        guard += 1;
        let m = idx.len();
        let ear = (0..m).find(|&i| {
            let (a, b, c) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
            let (pa, pb, pc) = (poly[a], poly[b], poly[c]);
            if (pb - pa).perp_dot(pc - pb) <= 1e-12 {
                return false;
            }
            idx.iter().all(|&o| o == a || o == b || o == c || poly[o] == pa || poly[o] == pb || poly[o] == pc || !inside(poly[o], pa, pb, pc))
        });
        match ear {
            Some(i) => {
                let (a, b, c) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
                out.push([a, b, c]);
                idx.remove(i);
            }
            // Degenerate input (collinear runs): drop the flattest corner so the loop always ends.
            None => {
                idx.remove(0);
            }
        }
    }
    if idx.len() == 3 {
        out.push([idx[0], idx[1], idx[2]]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn polygon_area(poly: &[Vec2], tris: &[[usize; 3]]) -> f32 {
        tris.iter().map(|t| (poly[t[1]] - poly[t[0]]).perp_dot(poly[t[2]] - poly[t[0]]).abs() / 2.0).sum()
    }

    #[test]
    fn triangulation_covers_convex_and_concave_polygons() {
        let square = [Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(0.0, 1.0)];
        assert!((polygon_area(&square, &triangulate(&square)) - 1.0).abs() < 1e-6);
        // An L shape (concave), area 3, in clockwise order too.
        let l = [Vec2::new(0.0, 0.0), Vec2::new(2.0, 0.0), Vec2::new(2.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 2.0), Vec2::new(0.0, 2.0)];
        assert!((polygon_area(&l, &triangulate(&l)) - 3.0).abs() < 1e-6);
        let reversed: Vec<Vec2> = l.iter().rev().copied().collect();
        assert!((polygon_area(&reversed, &triangulate(&reversed)) - 3.0).abs() < 1e-6);
    }

    #[test]
    fn solids_emit_whole_triangles_facing_outward() {
        let mut s = Sink::default();
        s.tube(0.04, 0.035, 0.0, 0.2, 16);
        s.cuboid(0.02, 0.01, 0.0, 0.05);
        s.plate(&[(0.0, 0.0), (0.03, 0.06), (0.08, 0.06), (0.1, 0.0)], 0.0, 0.04, 0.5, 0.2, 0.003);
        assert!(!s.vertices.is_empty() && s.vertices.len() % 3 == 0);
        for tri in s.vertices.chunks(3) {
            let [a, b, c] = [0, 1, 2].map(|i| Vec3::from(tri[i].position));
            let n = Vec3::from(tri[0].normal) + Vec3::from(tri[1].normal) + Vec3::from(tri[2].normal);
            assert!((b - a).cross(c - a).dot(n) >= -1e-9);
        }
    }
}
