//! A [`Field3`] made ready to draw as a 3D scene: the colour ranges, the wall mesh of the vehicle, the cut sheets built on demand, and
//! (later) the lookup of what is under the pointer. The GPU side is [`super::gpu3`].
//!
//! Built once per picture, off the UI thread: [`Prepared3::new`] makes one pass over the cells for the ranges and one over the wall
//! faces for the mesh. Nothing per cell goes to the GPU: a sheet (a constant-`k` meridian layer or a constant-`i` cross-flow layer of
//! cells) is a few tens of thousands of vertices, built from the field when the sheet changes ([`Prepared3::sheet`]).
//!
//! # Coordinates
//!
//! Vertices stay in the solver's frame `(x, y, z)`, `x` downstream from the nose tip; the shader applies the symmetry tile and then
//! [`world`]: the viewport's frame (`+Z` along the rocket, `+Y` up = the lee side, `+X` = solver `y`), a cyclic permutation, so
//! handedness is kept and the viewport's camera can be used as it is.

use super::map::{GREY_DARK, GREY_LIGHT, RDBU, VIRIDIS};
use bowshock_core::{Field3, Part, Tile};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// The viewport's world frame of the solver point `p`: `(y, z, x)`.
pub fn world(p: [f32; 3]) -> [f32; 3] {
    [p[1], p[2], p[0]]
}

/// What a cut sheet is coloured by, in the order of a [`SheetVertex`]'s values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scalar3 {
    /// Mach number, from 0.
    Mach,
    /// Pressure coefficient, symmetric about 0.
    Cp,
    /// Magnitude of the density gradient on a log scale.
    Schlieren,
    /// Eddy viscosity over molecular viscosity (RANS runs).
    Eddy,
}

impl Scalar3 {
    /// All four, in the order of the buttons.
    pub const ALL: [Scalar3; 4] = [Scalar3::Mach, Scalar3::Cp, Scalar3::Schlieren, Scalar3::Eddy];

    /// The button label.
    pub fn name(self) -> &'static str {
        match self {
            Scalar3::Mach => "Mach",
            Scalar3::Cp => "Cp",
            Scalar3::Schlieren => "Schlieren",
            Scalar3::Eddy => "Eddy ratio",
        }
    }

    /// Index among [`Scalar3::ALL`] (and in a vertex's values).
    pub fn index(self) -> usize {
        self as usize
    }

    /// The stops this scalar is coloured with.
    pub fn stops(self, dark: bool) -> &'static [[u8; 3]] {
        match (self, dark) {
            (Scalar3::Cp, _) => &RDBU,
            (Scalar3::Schlieren, true) => &GREY_DARK,
            (Scalar3::Schlieren, false) => &GREY_LIGHT,
            _ => &VIRIDIS,
        }
    }
}

/// What the wall surface is coloured by, in the order of a [`WallVertex`]'s values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WallScalar {
    /// Pressure coefficient, symmetric about 0.
    Cp,
    /// Magnitude of the skin-friction coefficient vector, from 0.
    Cf,
}

impl WallScalar {
    /// Both, in the order of the buttons.
    pub const ALL: [WallScalar; 2] = [WallScalar::Cp, WallScalar::Cf];

    /// The button label.
    pub fn name(self) -> &'static str {
        match self {
            WallScalar::Cp => "Wall Cp",
            WallScalar::Cf => "Skin friction",
        }
    }

    /// Index among [`WallScalar::ALL`].
    pub fn index(self) -> usize {
        self as usize
    }

    /// The stops this scalar is coloured with.
    pub fn stops(self) -> &'static [[u8; 3]] {
        match self {
            WallScalar::Cp => &RDBU,
            WallScalar::Cf => &VIRIDIS,
        }
    }
}

/// One vertex of the wall surface: a node of a wall face as seen from one part, with the mean normal and the two wall scalars there.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct WallVertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    /// Cp, |Cf|.
    pub values: [f32; 2],
}

/// One vertex of a cut sheet: a point of the sheet and the four scalars there ([`Scalar3`] order; the schlieren as log10).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct SheetVertex {
    pub pos: [f32; 3],
    pub values: [f32; 4],
}

/// The wall surface of the vehicle: one vertex per `(node, part)`, two triangles per wall face.
#[derive(Debug, Default)]
pub struct WallMesh {
    pub vertices: Vec<WallVertex>,
    /// Three indices per triangle, counter-clockwise seen from the fluid; the triangles of the vehicle come first, those of
    /// [`Part::Other`] (the sting) after them.
    pub triangles: Vec<u32>,
    /// Indices (not triangles) of the vehicle's part of `triangles`.
    pub body_indices: u32,
    /// The wall face (index into [`Field3::walls`]) of each triangle.
    pub face_of: Vec<u32>,
}

/// A cut sheet: a layer of cells drawn on the mid-surface of its two node layers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sheet {
    /// Cell layer `k` around the axis: the meridian sheet.
    Meridian(usize),
    /// Cell layer `i` along the axis: the cross-flow sheet.
    Cross(usize),
}

/// The vertices and triangles of a [`Sheet`].
#[derive(Debug, Default)]
pub struct SheetMesh {
    /// Changes with every mesh built: the GPU side uploads a mesh when it sees a new one.
    pub id: u64,
    pub vertices: Vec<SheetVertex>,
    /// Three indices per triangle, flow cells only.
    pub triangles: Vec<u32>,
    /// The cell `(i, j, k)` index of each triangle (two per cell).
    pub cell_of: Vec<u32>,
}

/// Floor of the density gradient (per metre) before the logarithm: nothing below it is told from the free stream.
const GRADIENT_FLOOR: f32 = 1e-3;
/// Decades of density gradient the schlieren ramp spans.
pub const SCHLIEREN_DECADES: f32 = 3.0;

static NEXT_ID: AtomicU64 = AtomicU64::new(1);

/// The part of the grid a cut sheet shows: the grid reaches tens of body lengths, the picture is of the rocket and its shock. The sheets
/// are built from the cells that touch it and clipped to it exactly on the GPU.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Window {
    /// Axial range (the solver's `x`).
    pub x: (f32, f32),
    /// Largest distance from the axis.
    pub r_max: f32,
}

impl Window {
    /// A little ahead of the nose, to a quarter of a body length behind the base, out to about half a body length (at least three
    /// radii of the widest part) from the axis; for the vehicle box `bounds` in the world frame.
    pub fn around((lo, hi): ([f32; 3], [f32; 3])) -> Window {
        let length = (hi[2] - lo[2]).max(1e-6);
        let radius = hi[0].abs().max(lo[0].abs()).max(hi[1].abs()).max(lo[1].abs());
        Window { x: (lo[2] - 0.2 * length, lo[2] + 1.25 * length), r_max: (0.45 * length).max(3.0 * radius) }
    }

    /// This window grown by `s` about the vehicle `bounds` (world frame): the axial range from the nose and the radius.
    pub fn scaled(&self, s: f32, (lo, hi): ([f32; 3], [f32; 3])) -> Window {
        let (nose, tail) = (lo[2], hi[2]);
        Window { x: (nose - s * (nose - self.x.0), tail + s * (self.x.1 - tail)), r_max: self.r_max * s }
    }

    /// The whole grid of `f`.
    pub fn everything(f: &Field3) -> Window {
        let (lo, hi) = f.bounds();
        let r = f.nodes.iter().map(|p| p[1].hypot(p[2])).fold(0.0f32, f32::max);
        Window { x: (lo[0] - 1.0, hi[0] + 1.0), r_max: r + 1.0 }
    }

    /// Whether the point `p` (solver frame) is inside.
    pub fn contains(&self, p: [f32; 3]) -> bool {
        p[0] >= self.x.0 && p[0] <= self.x.1 && p[1].hypot(p[2]) <= self.r_max
    }

    /// Whether a quad with these corners (solver frame) touches the window.
    pub fn overlaps(&self, corners: [[f32; 3]; 4]) -> bool {
        let (mut x_lo, mut x_hi, mut r_lo) = (f32::INFINITY, f32::NEG_INFINITY, f32::INFINITY);
        for p in corners {
            x_lo = x_lo.min(p[0]);
            x_hi = x_hi.max(p[0]);
            r_lo = r_lo.min(p[1].hypot(p[2]));
        }
        x_hi >= self.x.0 && x_lo <= self.x.1 && r_lo <= self.r_max
    }
}

/// How far from the vehicle the cut sheets reach.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Extent {
    /// The default window: the rocket and its shock.
    #[default]
    Near,
    /// 2.5 times that, for a bow shock or a wake that leaves it.
    Wide,
    /// The whole grid.
    Far,
}

impl Extent {
    /// All three, in the order of the buttons.
    pub const ALL: [Extent; 3] = [Extent::Near, Extent::Wide, Extent::Far];

    /// The button label.
    pub fn name(self) -> &'static str {
        match self {
            Extent::Near => "Near",
            Extent::Wide => "Wide",
            Extent::Far => "Far field",
        }
    }
}

/// Which copies of the solved domain a sheet is drawn in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TileSel {
    /// Only the tile with this index.
    One(u32),
    /// Every tile.
    All,
}

/// A sheet mesh and where it is drawn.
#[derive(Clone, Debug)]
pub struct SheetDraw {
    pub mesh: Arc<SheetMesh>,
    pub tiles: TileSel,
}

/// The `(tile, layer)` pairs that put cell layers on the whole plane through the axis that contains layer `k` of the solved domain: the
/// layer itself (tile 0) and the one on the other side of the axis, which lies in the solved range or in another tile (the half
/// domain's windward side `theta + pi`, or a copy of a sector under the symmetry).
pub fn plane_through(f: &Field3, tiles: &[Tile], k: usize) -> [(usize, usize); 2] {
    let azimuth = |t: &Tile, layer: usize| {
        let th = f.theta_of_layer(layer);
        let q = t.apply([0.0, th.sin(), th.cos()]);
        q[1].atan2(q[2])
    };
    let own = azimuth(&tiles[0], k);
    let target = own + std::f32::consts::PI;
    let dist = |a: f32| {
        let d = (a - target).rem_euclid(std::f32::consts::TAU);
        d.min(std::f32::consts::TAU - d)
    };
    let mut best = (f32::INFINITY, 0usize, k);
    for (ti, t) in tiles.iter().enumerate() {
        for layer in 0..f.nk {
            let d = dist(azimuth(t, layer));
            if d < best.0 {
                best = (d, ti, layer);
            }
        }
    }
    [(0, k), (best.1, best.2)]
}

/// A 3D field ready to draw.
#[derive(Debug)]
pub struct Prepared3 {
    /// Changes with every picture: the GPU side uploads a picture's wall mesh when it sees a new one.
    pub id: u64,
    /// The field.
    pub field: Arc<Field3>,
    /// The copies of the solved domain that make the whole flow.
    pub tiles: Vec<Tile>,
    /// The colour range of each [`Scalar3`]: Mach from 0, Cp symmetric about 0, the schlieren in log10 over [`SCHLIEREN_DECADES`],
    /// the eddy ratio from 0.
    pub ranges: [(f32, f32); 4],
    /// The colour range of each [`WallScalar`]: Cp symmetric about 0, |Cf| from 0.
    pub wall_ranges: [(f32, f32); 2],
    /// The wall surface.
    pub walls: WallMesh,
    /// The box of the vehicle over all tiles in the world frame (`min`, `max`); the whole grid for a field without wall faces.
    pub bounds: ([f32; 3], [f32; 3]),
    /// The part of the grid the cut sheets show.
    pub window: Window,
    /// The axial position of the middle of each cell layer `i` (the stations a cross-flow sheet can be put at), ascending.
    pub stations: Vec<f32>,
}

impl Prepared3 {
    /// Derives everything the scene draws from `field`.
    pub fn new(field: Arc<Field3>) -> Prepared3 {
        let f = &*field;
        let tiles = f.symmetry.tiles();
        let ranges = scalar_ranges(f);
        let walls = wall_mesh(f);
        let wall_ranges = {
            // the colour range holds 95 % of the wall pressures: the blunt fin edges set stagnation peaks that would turn the rest of the
            // body white (what saturates is still told from the rest by its colour)
            let mut cp: Vec<f32> = f.walls.iter().filter(|w| w.part != Part::Other).map(|w| w.cp.abs()).filter(|c| c.is_finite()).collect();
            let at = ((cp.len() as f32 * 0.95) as usize).min(cp.len().saturating_sub(1));
            let top = if cp.is_empty() { 0.0 } else { *cp.select_nth_unstable_by(at, f32::total_cmp).1 };
            let cf_max = f.walls.iter().filter(|w| w.part != Part::Other).map(|w| w.cf.iter().map(|c| c * c).sum::<f32>().sqrt()).fold(0.0f32, f32::max);
            let cp = ((top * 20.0).ceil() / 20.0).max(0.05);
            [(-cp, cp), (0.0, if cf_max > 0.0 { cf_max } else { 1e-3 })]
        };
        let bounds = bounds_of(f, &tiles);
        let window = Window::around(bounds);
        let stations = (0..f.ni).map(|i| 0.5 * (f.node(i, f.nj / 2, 0)[0] + f.node(i + 1, f.nj / 2, 0)[0])).collect();
        Prepared3 { id: NEXT_ID.fetch_add(1, Ordering::Relaxed), field, tiles, ranges, wall_ranges, walls, bounds, window, stations }
    }

    /// The four scalars of cell `(i, j, k)` (NaN in a solid cell): Mach, Cp, log10 of the density gradient, eddy ratio.
    pub fn cell_values(&self, i: usize, j: usize, k: usize) -> [f32; 4] {
        cell_values(&self.field, i, j, k)
    }

    /// The vertices and triangles of `sheet`.
    pub fn sheet(&self, sheet: Sheet, window: Window) -> SheetMesh {
        let f = &*self.field;
        match sheet {
            Sheet::Meridian(k) => meridian_sheet(f, k.min(f.nk - 1), window),
            Sheet::Cross(i) => cross_sheet(f, i.min(f.ni - 1), window),
        }
    }

    /// The window of `extent` around the vehicle (the whole grid for [`Extent::Far`]).
    pub fn window_of(&self, extent: Extent) -> Window {
        match extent {
            Extent::Near => self.window,
            Extent::Wide => self.window.scaled(2.5, self.bounds),
            Extent::Far => Window::everything(&self.field),
        }
    }

    /// What to draw for `sheet`: a meridian sheet is the whole plane through the axis (the layer and the one opposite it, found through
    /// the symmetry tiles when that is not in the solved range), or, `tiled`, the layer in every copy of the domain; a cross-flow sheet
    /// is the disc: the layer in every copy.
    pub fn sheet_draws(&self, sheet: Sheet, window: Window, tiled: bool) -> Vec<SheetDraw> {
        let f = &*self.field;
        match sheet {
            Sheet::Cross(i) => vec![SheetDraw { mesh: Arc::new(self.sheet(Sheet::Cross(i), window)), tiles: TileSel::All }],
            Sheet::Meridian(k) if tiled => vec![SheetDraw { mesh: Arc::new(self.sheet(Sheet::Meridian(k), window)), tiles: TileSel::All }],
            Sheet::Meridian(k) => {
                let k = k.min(f.nk - 1);
                let [(t0, l0), (t1, l1)] = plane_through(f, &self.tiles, k);
                let first = Arc::new(self.sheet(Sheet::Meridian(l0), window));
                let second = if l1 == l0 { first.clone() } else { Arc::new(self.sheet(Sheet::Meridian(l1), window)) };
                vec![SheetDraw { mesh: first, tiles: TileSel::One(t0 as u32) }, SheetDraw { mesh: second, tiles: TileSel::One(t1 as u32) }]
            }
        }
    }
}

fn cell_values(f: &Field3, i: usize, j: usize, k: usize) -> [f32; 4] {
    let c = f.cell_index(i, j, k);
    if f.solid[c] {
        return [f32::NAN; 4];
    }
    [f.mach_number(c), f.pressure_coefficient(c), f.density_gradient_at(i, j, k).max(GRADIENT_FLOOR).log10(), f.eddy_ratio.get(c).copied().unwrap_or(0.0)]
}

fn scalar_ranges(f: &Field3) -> [(f32, f32); 4] {
    let (mut mach_max, mut cp_max, mut eddy_max) = (0.0f32, 0.0f32, 0.0f32);
    for c in 0..f.cells() {
        if f.solid[c] {
            continue;
        }
        let (m, cp) = (f.mach_number(c), f.pressure_coefficient(c));
        if m.is_finite() {
            mach_max = mach_max.max(m);
        }
        if cp.is_finite() {
            cp_max = cp_max.max(cp.abs());
        }
        if let Some(&e) = f.eddy_ratio.get(c) {
            if e.is_finite() {
                eddy_max = eddy_max.max(e);
            }
        }
    }
    // the schlieren range from a thin sample of the cells: the gradient is the one costly scalar
    let step = ((f.cells() as f32 / 40_000.0).cbrt().ceil() as usize).max(1);
    let mut g: Vec<f32> = Vec::new();
    for k in (0..f.nk).step_by(step) {
        for j in (0..f.nj).step_by(step) {
            for i in (0..f.ni).step_by(step) {
                let v = f.density_gradient_at(i, j, k);
                if v.is_finite() {
                    g.push(v);
                }
            }
        }
    }
    let top = if g.is_empty() {
        1.0
    } else {
        // the strongest gradients (a shock in one cell) set the top; the last half per cent is left to saturate
        let n = ((g.len() as f32 * 0.995) as usize).min(g.len() - 1);
        *g.select_nth_unstable_by(n, f32::total_cmp).1
    };
    let hi = top.max(10.0).log10();
    let cp = ((cp_max * 10.0).ceil() / 10.0).max(0.2);
    [(0.0, ((mach_max * 4.0).ceil() / 4.0).max(1.0)), (-cp, cp), (hi - SCHLIEREN_DECADES, hi), (0.0, eddy_max.max(1.0))]
}

fn wall_mesh(f: &Field3) -> WallMesh {
    #[derive(Default, Clone, Copy)]
    struct Acc {
        normal: [f32; 3],
        cp: f32,
        cf: f32,
        n: f32,
    }
    let mut index: HashMap<(u32, Part), u32> = HashMap::new();
    let mut accs: Vec<Acc> = Vec::new();
    let mut sources: Vec<u32> = Vec::new();
    let mut mesh = WallMesh::default();
    let area = |nodes: [u32; 4]| {
        let p = |n: u32| f.nodes[n as usize];
        let (a, b) = ([0, 1, 2].map(|m| p(nodes[2])[m] - p(nodes[0])[m]), [0, 1, 2].map(|m| p(nodes[3])[m] - p(nodes[1])[m]));
        [0.5 * (a[1] * b[2] - a[2] * b[1]), 0.5 * (a[2] * b[0] - a[0] * b[2]), 0.5 * (a[0] * b[1] - a[1] * b[0])]
    };
    for pass in 0..2 {
        for (fi, w) in f.walls.iter().enumerate() {
            if (w.part == Part::Other) != (pass == 1) {
                continue;
            }
            let a = area(w.nodes);
            let cf = w.cf.iter().map(|c| c * c).sum::<f32>().sqrt();
            let mut ids = [0u32; 4];
            for (slot, &n) in w.nodes.iter().enumerate() {
                let id = *index.entry((n, w.part)).or_insert_with(|| {
                    accs.push(Acc::default());
                    sources.push(n);
                    (accs.len() - 1) as u32
                });
                let acc = &mut accs[id as usize];
                for m in 0..3 {
                    acc.normal[m] += a[m];
                }
                acc.cp += w.cp;
                acc.cf += cf;
                acc.n += 1.0;
                ids[slot] = id;
            }
            mesh.triangles.extend([ids[0], ids[1], ids[2], ids[0], ids[2], ids[3]]);
            mesh.face_of.extend([fi as u32, fi as u32]);
        }
        if pass == 0 {
            mesh.body_indices = mesh.triangles.len() as u32;
        }
    }
    mesh.vertices = accs
        .iter()
        .zip(&sources)
        .map(|(a, &n)| {
            let len = a.normal.iter().map(|c| c * c).sum::<f32>().sqrt().max(1e-30);
            WallVertex { pos: f.nodes[n as usize], normal: a.normal.map(|c| c / len), values: [a.cp / a.n, a.cf / a.n] }
        })
        .collect();
    mesh
}

/// The box of the vehicle's wall nodes over all tiles, in the world frame.
fn bounds_of(f: &Field3, tiles: &[Tile]) -> ([f32; 3], [f32; 3]) {
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    let mut grow = |p: [f32; 3]| {
        for t in tiles {
            let q = world(t.apply(p));
            for a in 0..3 {
                lo[a] = lo[a].min(q[a]);
                hi[a] = hi[a].max(q[a]);
            }
        }
    };
    let mut any = false;
    for w in f.walls.iter().filter(|w| w.part != Part::Other) {
        any = true;
        for &n in &w.nodes {
            grow(f.nodes[n as usize]);
        }
    }
    if !any {
        let (a, b) = f.bounds();
        for c in 0..8 {
            grow([if c & 1 == 0 { a[0] } else { b[0] }, if c & 2 == 0 { a[1] } else { b[1] }, if c & 4 == 0 { a[2] } else { b[2] }]);
        }
    }
    (lo, hi)
}

/// The mean of the finite values of `cells` (each of four scalars separately; NaN when there is none).
fn mean_of(cells: impl IntoIterator<Item = [f32; 4]>) -> [f32; 4] {
    let (mut sum, mut n) = ([0.0f32; 4], 0u32);
    for v in cells.into_iter().filter(|v| v[0].is_finite()) {
        sum.iter_mut().zip(v).for_each(|(s, v)| *s += v);
        n += 1;
    }
    if n == 0 {
        [f32::NAN; 4]
    } else {
        sum.map(|s| s / n as f32)
    }
}

/// Mid-surface point between nodes `a` and `b`.
fn mid(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [0, 1, 2].map(|m| 0.5 * (a[m] + b[m]))
}

fn meridian_sheet(f: &Field3, k: usize, window: Window) -> SheetMesh {
    let (ni, nj) = (f.ni, f.nj);
    let cells: Vec<[f32; 4]> = (0..ni * nj).map(|c| cell_values(f, c % ni, c / ni, k)).collect();
    let mut mesh = SheetMesh { id: NEXT_ID.fetch_add(1, Ordering::Relaxed), vertices: Vec::with_capacity((ni + 1) * (nj + 1)), ..Default::default() };
    for j in 0..=nj {
        for i in 0..=ni {
            let around = [(i.wrapping_sub(1), j.wrapping_sub(1)), (i, j.wrapping_sub(1)), (i.wrapping_sub(1), j), (i, j)]
                .into_iter()
                .filter(|&(ci, cj)| ci < ni && cj < nj)
                .map(|(ci, cj)| cells[cj * ni + ci]);
            mesh.vertices.push(SheetVertex { pos: mid(f.node(i, j, k), f.node(i, j, k + 1)), values: mean_of(around) });
        }
    }
    let node = |i: usize, j: usize| (j * (ni + 1) + i) as u32;
    for j in 0..nj {
        for i in 0..ni {
            if f.solid[f.cell_index(i, j, k)] {
                continue;
            }
            let (a, b, c, d) = (node(i, j), node(i + 1, j), node(i + 1, j + 1), node(i, j + 1));
            if !window.overlaps([a, b, c, d].map(|n| mesh.vertices[n as usize].pos)) {
                continue;
            }
            mesh.triangles.extend([a, b, c, a, c, d]);
            mesh.cell_of.extend([f.cell_index(i, j, k) as u32; 2]);
        }
    }
    mesh
}

fn cross_sheet(f: &Field3, i: usize, window: Window) -> SheetMesh {
    let (nj, nk) = (f.nj, f.nk);
    let cells: Vec<[f32; 4]> = (0..nj * nk).map(|c| cell_values(f, i, c % nj, c / nj)).collect();
    let mut mesh = SheetMesh { id: NEXT_ID.fetch_add(1, Ordering::Relaxed), vertices: Vec::with_capacity((nj + 1) * (nk + 1)), ..Default::default() };
    for k in 0..=nk {
        for j in 0..=nj {
            let around = [(j.wrapping_sub(1), k.wrapping_sub(1)), (j, k.wrapping_sub(1)), (j.wrapping_sub(1), k), (j, k)]
                .into_iter()
                .filter(|&(cj, ck)| cj < nj && ck < nk)
                .map(|(cj, ck)| cells[ck * nj + cj]);
            mesh.vertices.push(SheetVertex { pos: mid(f.node(i, j, k), f.node(i + 1, j, k)), values: mean_of(around) });
        }
    }
    let node = |j: usize, k: usize| (k * (nj + 1) + j) as u32;
    for k in 0..nk {
        for j in 0..nj {
            if f.solid[f.cell_index(i, j, k)] {
                continue;
            }
            let (a, b, c, d) = (node(j, k), node(j + 1, k), node(j + 1, k + 1), node(j, k + 1));
            if !window.overlaps([a, b, c, d].map(|n| mesh.vertices[n as usize].pos)) {
                continue;
            }
            mesh.triangles.extend([a, b, c, a, c, d]);
            mesh.cell_of.extend([f.cell_index(i, j, k) as u32; 2]);
        }
    }
    mesh
}

/// Decodes a fixture written by `field3_dump` (CBOR, raw deflate).
pub fn decode_fixture(bytes: &[u8]) -> Result<Field3, String> {
    use std::io::Read;
    let mut cbor = Vec::new();
    flate2::read::DeflateDecoder::new(bytes).read_to_end(&mut cbor).map_err(|e| format!("not a deflated flow field: {e}"))?;
    ciborium::from_reader(cbor.as_slice()).map_err(|e| format!("not a flow field: {e}"))
}

#[cfg(test)]
pub(super) mod tests {
    use super::super::axi::field3_from_2d;
    use super::super::prepared::tests::small2d;
    use super::super::prepared::Prepared;
    use super::*;
    use std::time::Instant;

    /// The committed fixture: the Army-Navy finner at Mach 2, 4 degrees, 20 352 cells (`field3_dump --case tiny`).
    pub(in crate::analyze) fn anf() -> Arc<Field3> {
        Arc::new(decode_fixture(include_bytes!("../../../tests/data/anf_m2_a4.field3")).expect("the fixture decodes"))
    }

    #[test]
    fn the_fixture_is_the_finner_at_mach_2_with_its_fins() {
        let f = anf();
        assert_eq!((f.mach, f.alpha_deg), (2.0, 4.0));
        assert_eq!(f.cells(), 20352);
        assert_eq!(f.nodes.len(), (f.ni + 1) * (f.nj + 1) * (f.nk + 1));
        assert_eq!(f.fin_theta.len(), 2);
        assert!(f.walls.iter().any(|w| matches!(w.part, Part::Fin(0))));
        assert_eq!(f.mesh_id, bowshock_core::engine::field3::mesh_id(&f));
    }

    #[test]
    fn the_solvers_two_degree_wedge_is_drawn_in_as_many_copies_as_fit_the_circle() {
        use bowshock_core::engine::case3::Case3;
        let case = Case3::wedge(2.0, Case3::anf_body(), 0);
        let s = case.build_solver().unwrap();
        let f = bowshock_core::engine::field3::field3_of_solver(&case, &s, 0, 1.0);
        let p = Prepared3::new(Arc::new(f));
        assert_eq!(p.tiles.len(), 180);
        assert!(p.tiles.len() <= super::super::gpu3::MAX_TILES, "the GPU side has room for every copy");
        assert!(p.sheet_draws(Sheet::Meridian(0), p.window, true).iter().all(|d| !d.mesh.triangles.is_empty()));
    }

    #[test]
    fn the_ranges_follow_the_flow() {
        let p = Prepared3::new(anf());
        let (m, cp, s) = (p.ranges[Scalar3::Mach.index()], p.ranges[Scalar3::Cp.index()], p.ranges[Scalar3::Schlieren.index()]);
        assert_eq!(m.0, 0.0);
        assert!(m.1 >= 2.0 && m.1 <= 3.0, "{m:?}");
        assert_eq!(cp.0, -cp.1);
        assert!(cp.1 >= 0.2);
        assert!((s.1 - s.0 - SCHLIEREN_DECADES).abs() < 1e-6);
        assert_eq!(p.ranges[Scalar3::Eddy.index()], (0.0, 1.0), "an Euler run has no eddy viscosity");
        // an Euler run has no friction: the range is a placeholder; the wall Cp range holds 95 % of the wall faces
        assert!(p.wall_ranges[WallScalar::Cf.index()].1 > 0.0);
        let wall = p.wall_ranges[WallScalar::Cp.index()];
        let faces: Vec<_> = p.field.walls.iter().filter(|w| w.part != Part::Other).collect();
        let inside = faces.iter().filter(|w| w.cp >= wall.0 && w.cp <= wall.1).count();
        assert!(inside as f32 >= 0.945 * faces.len() as f32, "{inside} of {} inside {wall:?}", faces.len());
    }

    #[test]
    fn the_wall_mesh_has_two_triangles_a_face_vehicle_first_and_normals_out_of_the_body() {
        let p = Prepared3::new(anf());
        let f = &*p.field;
        let m = &p.walls;
        assert_eq!(m.triangles.len(), 6 * f.walls.len());
        assert_eq!(m.face_of.len(), 2 * f.walls.len());
        let body = f.walls.iter().filter(|w| w.part != Part::Other).count();
        assert_eq!(m.body_indices as usize, 6 * body);
        assert!(m.triangles[..m.body_indices as usize / 3 * 3].chunks(3).enumerate().all(|(t, _)| f.walls[m.face_of[t] as usize].part != Part::Other));
        assert!(m.triangles[m.body_indices as usize..]
            .chunks(3)
            .enumerate()
            .all(|(t, _)| f.walls[m.face_of[m.body_indices as usize / 3 + t] as usize].part == Part::Other));
        // vertices are shared inside a part: fewer than four per face
        assert!(m.vertices.len() < 3 * f.walls.len(), "{} vertices for {} faces", m.vertices.len(), f.walls.len());
        assert!(m.triangles.iter().all(|&i| (i as usize) < m.vertices.len()));
        assert!(m.vertices.iter().all(|v| (v.normal.iter().map(|c| c * c).sum::<f32>().sqrt() - 1.0).abs() < 1e-4));
        // a body wall face's mean vertex normal points away from the axis at the face's centre
        for (fi, w) in f.walls.iter().enumerate().filter(|(_, w)| matches!(w.part, Part::Nose | Part::Body)) {
            let t = m.face_of.iter().position(|&x| x as usize == fi).unwrap();
            let ids = [m.triangles[3 * t], m.triangles[3 * t + 1], m.triangles[3 * t + 2]];
            let n: Vec<[f32; 3]> = ids.iter().map(|&i| m.vertices[i as usize].normal).collect();
            let centre = [0, 1, 2].map(|c| ids.iter().map(|&i| m.vertices[i as usize].pos[c]).sum::<f32>() / 3.0);
            let out: f32 = n.iter().map(|n| n[1] * centre[1] + n[2] * centre[2]).sum();
            assert!(out > 0.0, "{w:?} {n:?} {centre:?}");
        }
    }

    #[test]
    fn the_bounds_hold_the_whole_rocket_in_the_world_frame() {
        let p = Prepared3::new(anf());
        let (lo, hi) = p.bounds;
        // Z runs along the rocket from the tip: the finner is 10 diameters long; the half domain is mirrored, so X and Y are centred
        assert!(lo[2] <= 0.01 && hi[2] > 9.0 && hi[2] < 12.0, "{lo:?} {hi:?}");
        assert!((lo[0] + hi[0]).abs() < 1e-3 && (lo[1] + hi[1]).abs() < 1e-3, "{lo:?} {hi:?}");
        assert!(hi[0] > 0.5 && hi[0] < 2.0, "the fins reach about 1.5 diameters: {hi:?}");
    }

    #[test]
    fn a_meridian_sheet_is_the_mid_surface_of_its_layer_with_flow_cells_only() {
        let p = Prepared3::new(anf());
        let f = &*p.field;
        let k = f.layer_of_theta(0.0);
        let s = p.sheet(Sheet::Meridian(k), p.window);
        assert_eq!(s.vertices.len(), (f.ni + 1) * (f.nj + 1));
        let touches = |c: usize| {
            let (i, j) = (c % f.ni, c / f.ni);
            let q = |di: usize, dj: usize| mid(f.node(i + di, j + dj, k), f.node(i + di, j + dj, k + 1));
            p.window.overlaps([q(0, 0), q(1, 0), q(1, 1), q(0, 1)])
        };
        let fluid = (0..f.ni * f.nj).filter(|&c| f.is_fluid(f.cell_index(c % f.ni, c / f.ni, k)) && touches(c)).count();
        let all_fluid = (0..f.ni * f.nj).filter(|&c| f.is_fluid(f.cell_index(c % f.ni, c / f.ni, k))).count();
        assert_eq!(s.triangles.len(), 6 * fluid);
        assert_eq!(s.cell_of.len(), 2 * fluid);
        assert!(fluid < all_fluid, "the window leaves the far field out: {fluid} of {all_fluid}");
        // the far field is there when asked for
        let far = p.sheet(Sheet::Meridian(k), p.window_of(Extent::Far));
        assert_eq!(far.triangles.len(), 6 * all_fluid);
        assert!(p.sheet(Sheet::Meridian(k), p.window_of(Extent::Wide)).triangles.len() > s.triangles.len());
        // the window holds the vehicle with room to spare: nose to a little past the base, out past the fins
        assert!(p.window.x.0 < 0.0 && p.window.x.1 > 10.0 && p.window.r_max > 4.0);
        let (i, j) = (f.ni / 2, 3);
        let v = s.vertices[j * (f.ni + 1) + i];
        let want = mid(f.node(i, j, k), f.node(i, j, k + 1));
        assert!((0..3).all(|m| (v.pos[m] - want[m]).abs() < 1e-6));
        assert!(v.values[0].is_finite() && v.values[0] > 0.0 && v.values[0] < 4.0, "{v:?}");
    }

    #[test]
    fn a_cross_sheet_is_a_constant_station_surface() {
        let p = Prepared3::new(anf());
        let f = &*p.field;
        let i = f.ni * 3 / 5;
        let s = p.sheet(Sheet::Cross(i), p.window);
        assert_eq!(s.vertices.len(), (f.nj + 1) * (f.nk + 1));
        let x0 = f.node(i, 0, 0)[0];
        let x1 = f.node(i + 1, 0, 0)[0];
        assert!(s.vertices.iter().all(|v| v.pos[0] >= x0 - 1e-4 && v.pos[0] <= x1 + 1e-4));
        // outside the body the sheet covers the half circle; the fins make some cells solid
        assert!(!s.triangles.is_empty() && s.triangles.len() <= 6 * f.nj * f.nk);
    }

    #[test]
    fn a_sheet_of_the_wedge_has_the_values_the_2d_picture_has() {
        // the one-cell wedge of the 2D fixture: the meridian sheet carries the 2D picture's three scalars at every node
        let f = Arc::new(field3_from_2d(&small2d(2.0)));
        let flat = Prepared::new(f.clone());
        let s = {
            let p = Prepared3::new(f);
            p.sheet(Sheet::Meridian(0), p.window_of(Extent::Far))
        };
        assert_eq!(s.vertices.len(), flat.vertices.len());
        for (a, b) in s.vertices.iter().zip(&flat.vertices) {
            for m in 0..3 {
                let (x, y) = (a.values[m], b.values[m]);
                assert!((x.is_nan() && y.is_nan()) || (x - y).abs() < 1e-3 * (1.0 + x.abs()), "{a:?} vs {b:?}");
            }
        }
    }

    #[test]
    fn a_plane_through_the_axis_is_the_layer_and_the_one_across_it_in_some_tile() {
        // the half domain, 0 to pi: the lee layer (tile 0) and the windward one, which is the last layer under the mirror
        let p = Prepared3::new(anf());
        let f = &*p.field;
        assert_eq!(plane_through(f, &p.tiles, 0), [(0, 0), (1, f.nk - 1)]);
        assert_eq!(plane_through(f, &p.tiles, f.nk - 1), [(0, f.nk - 1), (1, 0)]);
        // a one-cell wedge of an axisymmetric flow: the half turn is 36 of its 72 copies
        let w = Prepared3::new(Arc::new(field3_from_2d(&small2d(2.0))));
        assert_eq!(plane_through(&w.field, &w.tiles, 0), [(0, 0), (36, 0)]);
    }

    #[test]
    fn a_meridian_sheet_draw_is_the_plane_and_a_tiled_one_is_every_copy() {
        let p = Prepared3::new(anf());
        let draws = p.sheet_draws(Sheet::Meridian(0), p.window, false);
        assert_eq!(draws.iter().map(|d| d.tiles).collect::<Vec<_>>(), [TileSel::One(0), TileSel::One(1)]);
        assert_ne!(draws[0].mesh.id, draws[1].mesh.id);
        assert_eq!(p.sheet_draws(Sheet::Meridian(0), p.window, true).iter().map(|d| d.tiles).collect::<Vec<_>>(), [TileSel::All]);
        assert_eq!(p.sheet_draws(Sheet::Cross(10), p.window, false).iter().map(|d| d.tiles).collect::<Vec<_>>(), [TileSel::All]);
        // an axisymmetric wedge: both halves of the plane are the same layer
        let w = Prepared3::new(Arc::new(field3_from_2d(&small2d(2.0))));
        let d = w.sheet_draws(Sheet::Meridian(0), w.window, false);
        assert_eq!(d[0].mesh.id, d[1].mesh.id, "one mesh drawn twice");
        assert_eq!(d[1].tiles, TileSel::One(36));
    }

    /// A body of revolution (a 10 m long cone and tube, 1 m across) in a half domain `ni x nj x nk` cells, its interior solid, with a made-up
    /// smooth flow: the scale of what the 3D solver will give (`synthetic(200, 100, 100)` is 2 million cells) without running it.
    pub(in crate::analyze) fn synthetic(ni: usize, nj: usize, nk: usize) -> Field3 {
        let (length, r_far, jb) = (10.0f32, 5.0f32, nj / 5);
        let body_r = |x: f32| if x <= 0.0 { 0.0 } else { (0.5 * x / 5.7).min(0.5) };
        let mut nodes = Vec::with_capacity((ni + 1) * (nj + 1) * (nk + 1));
        for k in 0..=nk {
            let th = std::f32::consts::PI * k as f32 / nk as f32;
            for j in 0..=nj {
                for i in 0..=ni {
                    let x = -0.2 * length + 1.4 * length * i as f32 / ni as f32;
                    let rb = body_r(x);
                    let r = if j <= jb { rb * j as f32 / jb as f32 } else { rb + (r_far - rb) * ((j - jb) as f32 / (nj - jb) as f32).powi(2) };
                    nodes.push([x, r * th.sin(), r * th.cos()]);
                }
            }
        }
        let cells = ni * nj * nk;
        let mut f = Field3 {
            mach: 2.0,
            alpha_deg: 4.0,
            iteration: 1000,
            freestream: [1.0, 2.0, 0.0, 0.0, 1.0 / bowshock_core::field::GAMMA],
            ni,
            nj,
            nk,
            mesh_id: 1,
            nodes,
            solid: vec![false; cells],
            prim: vec![[1.0, 2.0, 0.0, 0.0, 0.714]; cells],
            eddy_ratio: Vec::new(),
            walls: Vec::new(),
            fin_theta: Vec::new(),
            symmetry: bowshock_core::Symmetry::Mirror { theta0: 0.0, theta1: std::f32::consts::PI },
        };
        for k in 0..nk {
            for j in 0..nj {
                for i in 0..ni {
                    let c = f.cell_index(i, j, k);
                    let ce = f.centre(i, j, k);
                    let r = ce[1].hypot(ce[2]);
                    if j < jb && body_r(ce[0]) > 1e-3 {
                        f.solid[c] = true;
                        continue;
                    }
                    // a shock-like jump along a cone and a smooth decay away from the body
                    let shock = 0.5 * 0.2 * ce[0].max(0.0) + 0.6;
                    let behind = if r < shock { 1.0 } else { 0.0 };
                    f.prim[c] = [
                        1.0 + 0.4 * behind * (-(r - 0.5).abs()).exp(),
                        2.0 - 0.3 * behind,
                        0.1 * ce[2] / (1.0 + r),
                        0.1 * ce[1] / (1.0 + r),
                        0.714 * (1.0 + 0.3 * behind),
                    ];
                }
            }
        }
        // the wall faces: the top of the solid rows, where the body is
        for k in 0..nk {
            for i in 0..ni {
                let ce = f.centre(i, jb, k);
                if body_r(ce[0]) <= 1e-3 || f.solid[f.cell_index(i, jb, k)] {
                    continue;
                }
                let id = |ii: usize, kk: usize| f.node_index(ii, jb, kk) as u32;
                f.walls.push(bowshock_core::WallFace {
                    nodes: [id(i, k), id(i + 1, k), id(i + 1, k + 1), id(i, k + 1)],
                    cell: f.cell_index(i, jb, k) as u32,
                    part: if ce[0] < 5.7 { Part::Nose } else { Part::Body },
                    cp: 0.05 * (ce[0] * 0.7).sin(),
                    cf: [0.0; 3],
                });
            }
        }
        f
    }

    /// Cost of preparing a picture and of a sheet on the `out/field3/anf-level0.field3` fixture (352 800 cells), `cargo nextest run
    /// --release -p bowshock prepared3_cost --run-ignored only --no-capture`.
    #[test]
    #[ignore = "needs out/field3/anf-level0.field3 from tools/gpuq (field3_dump --case level0)"]
    fn prepared3_cost_on_the_level_0_finner() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../out/field3/anf-level0.field3");
        let f = Arc::new(decode_fixture(&std::fs::read(path).expect("the level 0 fixture")).unwrap());
        let t0 = Instant::now();
        let p = Prepared3::new(f.clone());
        let new_ms = t0.elapsed().as_secs_f64() * 1e3;
        let t1 = Instant::now();
        let k = f.layer_of_theta(0.0);
        let m = p.sheet(Sheet::Meridian(k), p.window);
        let meridian_ms = t1.elapsed().as_secs_f64() * 1e3;
        let t2 = Instant::now();
        let c = p.sheet(Sheet::Cross(f.ni / 2), p.window);
        let cross_ms = t2.elapsed().as_secs_f64() * 1e3;
        println!(
            "PREPARED3 {} cells {} walls: new {new_ms:.1} ms, meridian sheet {meridian_ms:.1} ms ({} vertices), cross sheet {cross_ms:.1} ms ({} vertices)",
            f.cells(),
            f.walls.len(),
            m.vertices.len(),
            c.vertices.len()
        );
    }
}
