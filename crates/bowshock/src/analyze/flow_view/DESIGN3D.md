# Flow viewer in 3D: design (branch `cfd-3d-viz`)

Scope: `Field3` (new files in `bowshock-core`), the 3D viewer, the 2D mode on top of it, the demo/drive/test path. The solver facade
(`Analysis`, `run_lift`, remote protocol) is `solver3d`'s; until it lands the app gets `Field3` from a demo painter and from fixtures.

## 0. Findings about the contract (read these first)
1. Solver axes: `freestream3 = [1, M cos a, 0, M sin a, 1/GAMMA]`: the air moves towards **+z**, so z (theta = 0) is the **lee** side and
   theta = pi (-z) the windward side; the pitch plane is y = 0. At a != 0 the two halves of a side view are *not* mirror images, so the
   2D mode draws two meridian sheets (azimuth phi up, phi + pi down), not today's mirror. Velocity is Cartesian `(u, v, w)` = (x, y, z)
   (the 2D `Field` has axial/radial); tiles therefore carry a full 3x3 matrix.
2. The solver works in nose diameters, `Field3` in metres (scale at construction, like `Field`); `alpha` is stored in radians.
3. `Grid3` has no wall list. Walls = faces between a fluid and a solid cell (fin `SolidBox`es) plus `j = 0` faces for `i >= axis_cells`
   (body and sting). The sting grid has no base face; sting walls are `Part::Other`, the future base-wake grid gets `Part::Base`.
   Wall Cp uses the two-cell linear extrapolation of `run3::accumulate`.
4. The eddy-viscosity ratio needs the molecular viscosity: the solver stage fills it (`nut` empty for Euler).
5. Size: 2 M cells = 40 MB of `[f32; 5]` plus 24 MB of nodes, so JSON is out for fixtures; fixtures stay tiny (ANF level 0, 9 k cells).
   Snapshots repeat nodes/walls every time; `mesh_id` (hash of nodes, solid flags and wall topology) lets the viewer skip rebuilding.
6. serde: `#[cfg_attr(feature = "serde", ...)]` like `Field`. No `Arc` inside `Field3` (serde needs `rc`, not enabled).

## 1. `Field3` (`bowshock-core/src/field3.rs`, `engine/field3.rs`)
```rust
pub struct Field3 {
    pub mach: f64, pub alpha: f64 /* rad */, pub iteration: usize,
    pub wake: Wake,                         // grid treatment, as Field
    pub freestream: [f64; 5],               // [rho, u, v, w, p], solver units
    pub ni: usize, pub nj: usize, pub nk: usize,
    pub mesh_id: u64,
    pub nodes: Vec<[f32; 3]>,               // (ni+1)(nj+1)(nk+1), index (k*(nj+1)+j)*(ni+1)+i, metres, (x, r sin t, r cos t)
    pub solid: Vec<bool>,                   // per cell, Grid3::cell order (k*nj+j)*ni+i
    pub prim: Vec<[f32; 5]>,                // per cell [rho, u, v, w, p]; solid cells hold the free stream
    pub nut: Vec<f32>,                      // per cell eddy-viscosity ratio; empty unless RANS
    pub walls: Vec<WallFace>,               // every wall quad of the solved domain
    pub fin_theta: Vec<f32>,                // azimuth of each fin mid-plane inside the solved range; empty without fins
    pub symmetry: Symmetry,
}
pub struct WallFace { pub nodes: [u32; 4], pub cell: u32, pub part: Part, pub cp: f32, pub cf: [f32; 3] }
// nodes counter-clockwise seen from the fluid; cell = adjacent fluid cell; cf = 0 for Euler
pub enum Part { Nose, Body, Base, Fin(u8), Other }
pub enum Symmetry {
    Full,                                              // whole circle solved
    Mirror { theta0: f32, theta1: f32 },               // flow mirror-symmetric about both bounding half-planes; (theta1-theta0) = pi/m
    Periodic { theta0: f32, theta1: f32 },             // copies rotated by theta1-theta0; 2pi/(theta1-theta0) integer
    Axisymmetric { theta0: f32, theta1: f32 },         // one-cell wedge: any rotation is a symmetry
}
pub struct Tile { pub m: [[f32; 3]; 3], pub mirrored: bool }   // acts on (x, y, z); det < 0 iff mirrored (flip triangle winding)
impl Symmetry { pub fn theta_range(&self) -> (f32, f32); pub fn tiles(&self) -> Vec<Tile>; }
impl Field3 {
    pub fn cells(&self) -> usize; pub fn node(&self, i: usize, j: usize, k: usize) -> [f32; 3];
    pub fn cell_index(&self, i: usize, j: usize, k: usize) -> usize; pub fn is_fluid(&self, c: usize) -> bool;
    pub fn mach_number(&self, c: usize) -> f32; pub fn pressure_coefficient(&self, c: usize) -> f32;   // + pub fn mach_numbers()/pressure_coefficients()
    pub fn density_gradient_at(&self, i: usize, j: usize, k: usize) -> f32;   // |grad rho| / rho_inf, 1/m, via the cell-centre metric;
    pub fn density_gradient(&self) -> Vec<f32>;                               // whole-field version, NaN in solid cells
    pub fn theta_of_layer(&self, k: usize) -> f32; pub fn layer_of_theta(&self, theta: f32) -> usize;
}
```
Tiles: `Full` = identity. `Periodic`: n = round(2pi/w) rotations by m w about x. `Mirror`: n = round(2pi/w) (even); tile m covers
`[theta0 + m w, theta0 + (m+1) w]`: even m rotate by m w, odd m reflect about the half-plane `theta0` then rotate by (m+1) w (half domain,
w = pi: identity and y -> -y). `Axisymmetric`: n = round(2pi/w) rotations by 2pi/n. A constructor `debug_assert`s the divisibility.
Engine constructor: `engine::field3::field3_of(case: &Case3, g: &Grid3, u: &[Cons3], iteration: usize, scale: f64) -> Field3`
(`Symmetry::Mirror {0, pi}`, walls found as in `accumulate`, parts by x against `case.body` nose length and `x_base`, fins by `FinSet`
order, `fin_theta` from the case's fin placement). Real data to draw: `bowshock-validate/examples/field3_dump.rs` (new) runs the tiny ANF
case on the GPU (`GpuRunner3::run_on`, `state_f64`) and writes a deflated-CBOR fixture; `crates/bowshock/tests/data/anf_m2_a4.field3`.

## 2. Module layout (`analyze/flow_view/`)
`mod.rs` (was `flow_view.rs`: State, Msg, viewer, `centre`) · `meridian.rs` (`Meridian`: one cell layer of a `Field3` as the 2D quad mesh
with `x, r, solid, rho, u_axial, v_radial, p`: the shape of today's `Field`) · `prepared.rs` (2D `Prepared`, now built from a `Meridian`)
· `gpu.rs` + `flow.wgsl` (2D, unchanged) · `scene3.rs` (`Prepared3`: ranges, wall mesh, sheet vertex builders, CPU picking) · `gpu3.rs` +
`scene3.wgsl` (3D pipeline) · `map.rs` (colour maps; adds `Scalar::Eddy`, `WallScalar::{Cp, Cf}`) · `camera3.rs` (world mapping, framing).
`analyze/cfd/fields.rs`: `FieldRef(Arc<Field3>)`; `FieldStore` keyed by `(mach, alpha millideg)`. The only 2D-`Field` code is
`flow_view/axi.rs`: `fn field3_from_2d(&Field) -> Field3` (one-cell wedge theta 0..w, v_y = v sin(w/2), v_z = v cos(w/2), walls from
`Field::wall()`/`surface()`), called where `cfd::run` wraps a solver `Field` into `FieldRef`. Deleting the 2D solver = delete `axi.rs` and
that call. Nothing else in the viewer names `bowshock_core::Field`.

## 3. Render pipeline (3D)
* **CPU, once per picture, off the UI thread** (`Task`, serial-checked; the old picture stays until the new one is ready): `Prepared3::new(Arc<Field3>)`:
  colour ranges for Mach, Cp, eddy ratio (one pass over `prim`), schlieren range from every 8th cell, the **wall mesh** (vertices deduplicated per
  `(node, part)` so colours are smooth on a part and sharp at the fin root; position, normal, Cp, |cf|, part) and the fit box. Nothing per cell goes to the GPU.
* **Sheets built on demand** (slider/step/azimuth changes, ~1 ms): meridian sheet = cell layer k drawn on the mid-surface of node layers k and k+1;
  cross-flow sheet = cell layer i. Vertex = mean of the adjacent flow cells' scalar (as today's `Prepared`), solid-adjacent cells ignored, all-solid
  vertices dropped (so the sheet stops at the body and fin surfaces). Scalars are computed only for the cells of the visible sheets
  (schlieren via `density_gradient_at`). Sheet vertex arrays are tiny (<= a few 10^4 vertices) and re-uploaded on change only.
* **GPU** (`iced` `shader` primitive like `viewport/pipeline.rs`: 4x MSAA colour + `Depth32Float` offscreen pass, premultiplied composite into the
  widget rectangle). Wall pipeline: opaque, headlight Lambert times the LUT colour of Cp or |cf|; sheet pipeline: unlit LUT colour, depth-tested, `discard`
  on NaN; grid-line pipeline for cell edges (optional). Tiles are **instances**: a storage array of `mat3 + mirrored` (<= 128), mirrored tiles flip
  winding (`front_facing`). Uniforms: view-proj from `viewport::camera::Camera`, LUT, ranges, scalar index, light dir. World mapping: world (X, Y, Z) =
  solver (y, z, x), a cyclic permutation (right-handed): +Z along the rocket as in the viewport, +Y up = the lee side, the free stream drawn as an arrow.
* **Camera**: `viewport::camera::Camera` reused (turntable, orbit = right-drag or one-finger scroll like the viewport, middle-drag pan, wheel and
  `widgets::pinch` zoom about the cursor, `F`/double-click fits). Presets Side / Top / Front set yaw and pitch. The canvas overlay (axes gizmo, free-stream arrow,
  colour bar, readout, slider labels) sits on top as today.
* **Hover**: CPU ray (`Camera::ray`) against the visible sheets' triangles and the wall triangles per tile (inverse tile transform), brute force
  (<= 2 x 10^5 triangles, ~1 ms release, measured in the tests). Readout: sheet cell (i, j, k), Mach, Cp, rho, p, eddy ratio; or wall: part, Cp, |cf|, x.

## 4. State and messages
`State` keeps `scalar` (Mach/Cp/Schlieren/Eddy), `wall_scalar`, `grid`, `mode: Mode {Side, Scene}`, `camera` (2D) and `camera3`, `sheets: {meridian: Option<usize /*k*/>,
cross: Option<usize /*i*/>, tiled: bool, walls: bool}`, `selected` Mach, `selected_alpha`, `machs`, `alphas`. New `Msg`: `Mode(Mode)`, `WallScalar`, `Orbit(Vector)`,
`Meridian(SheetMsg)`, `Cross(SheetMsg)` (`Toggle | Step(i32) | Pick(usize)`), `Tiled`, `Walls`, `AlphaStep(i32)`, `Prepared(u64, Arc<Prepared3>)`, `View(Preset)`.
`Pan/Zoom/Fit/Hover/Resized` keep their meaning in both modes. `sync_flow`/`Solving.picture` carry `Arc<Field3>`; a snapshot with the same `mesh_id` and
`ni,nj,nk` reuses the wall mesh and the camera; a new one refits only when the camera is still the fit. Mode default: `Side` when `nk == 1` (and then
the toggle is hidden), else `Scene`. The angle stepper shows only with >= 2 angles in the stored result.

## 5. 2D mode = today's picture
`Mode::Side` builds two `Meridian`s (layer at azimuth phi, default the pitch plane lee side, and at phi + pi; for an axisymmetric field both are layer 0, so the picture is
today's mirror) and feeds today's `Prepared`/`gpu.rs`/`Overlay`/`Xf` code: mirrored GPU draw, axis, ticks, hover (`cell_at`), grid toggle, wall Cp strip sharing the x axis (a
second curve for the lower half when it differs), Cp/Mach/Schlieren colour bar, Fit/zoom/pan. Regression: `Prepared::new(Meridian(field3_from_2d(f)))` equals today's
`Prepared::new(f)` array for array (scalars, ranges, vertices, `cell_at`, `wall_cp_at`, `surface`) on the existing test fields, until `Field` is deleted.

## 6. Tests, scripts, measurements
* core: `Symmetry::tiles` covers the circle exactly once (area/angle sum, mirrored flags, det signs) for each variant; `field3_of` on a coarse `tiny_anf` grid with
  the free stream: Cp ~ 0 on walls, every fluid-solid face found once (count equals the `accumulate` face count), part tags, mesh_id stable, serde round trip (feature test).
* app (`nextest`): `Prepared3` ranges, dedup of wall vertices, sheet builders on a synthetic painted field (known Mach on a cone), CPU picking of a hand-made quad,
  mode/sheet/step message logic, `FieldStore` keyed by alpha, the 2D regression of section 5, a device test like `the_picture_renders_on_a_device_...` for `gpu3.rs`
  (frame time for the ANF fixture and a synthetic 0.5 M and 2 M cell field: submit-to-done, 300 frames; reported in `ARCHITECTURE.md` with the machine).
* drive: `scripts/flow3d.drive` (BOWSHOCK_CFD_DEMO=`flow3d` plays a Field3 painted on the example rocket's real 3D grid, `solve_demo.rs`) -> `out/flow3d/`:
  scene-wall-cp, scene-meridian-mach, scene-cross-section, scene-schlieren-tiled, scene-cf, side-2d (nk = 1, equals `flow-view.drive` output), hover readout, orbit, dark and light.
  `flow-view.drive` keeps running unchanged against the adapter and its screenshots must stay pixel-comparable to the current ones.
* Fixtures from the real solver: `field3_dump` (ANF M 2, alpha 4 deg, level 0) feeds `scene-anf-*` shots.
* Snapshot cost, measured: solver-side `field3_of` for 9 k and a synthetic 2 M cell grid, and viewer `Prepared3::new` (ms, release), stated in the docs.

## 7. Commit sequence (each: unit tests, `cargo +stable fmt --all --check`, `cargo +stable clippy --workspace --all-targets -- -D warnings`, nextest app crate, drive shots in `out/`)
1. `Field3`, `Part`, `WallFace`, `Symmetry`/`Tile` + derived fields + serde (core, new files + 2 registration lines) with tests.
2. `engine::field3::field3_of` + `field3_dump` example + fixture + tests.
3. `FieldRef`/`FieldStore`/events/`Solving.picture` switched to `Arc<Field3>`; `axi.rs` adapter; viewer reads a `Meridian` (2D mode only, `flow-view.drive` unchanged, regression test).
4. `Prepared3` (ranges, wall mesh, sheet builders, picking) with CPU tests.
5. `gpu3.rs` + `scene3.wgsl`, `camera3.rs`: walls and one meridian sheet on screen; device test; first screenshots.
6. State/messages/controls: mode toggle, sheets, tiling, steppers, wall scalar, hover readout, free-stream arrow; drive script `flow3d.drive`.
7. Angle-of-attack stepper + `FieldStore` keys; live snapshots in 3D (demo `flow3d` stream), prepare off-thread.
8. Performance pass (0.5 M / 2 M cell numbers), `ARCHITECTURE.md`, `design` conformance review, final screenshots.
