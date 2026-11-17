# Bowshock app architecture

One `App` (in `app/mod.rs`) owns one shared context, `Ctx` (`app/ctx.rs`), and one state struct per area. A single top-level
`app::Message` (`app/message.rs`) wraps each area's own `Message`. Modules work in parallel by owning their own file(s) and never needing to
touch `app/mod.rs`.

```
main.rs                  CLI (app / drive / gallery) and the `app()` builder only
app/mod.rs               App lifecycle (new, launch, update, after_update, view, subscription)
app/ctx.rs               Ctx, Mode, Selection, Pending
app/message.rs           Message, KeyPress, the global keyboard map (`shortcut`)
app/run_report.rs        RunReport: the last background run (a line per flight, its outcome as `simulate::Message::Done` lands); `Ctx::{start_run, finish_run_line}`; cleared by the `Frame` clock, `DismissRun` and switching to Simulate
app/route.rs             message routing, modal interception (`intercept_modals`), key dispatch
app/env.rs               Env: centralized BOWSHOCK_* / $HOME config plus `animate` / `freeze_flash` (`Env::from_process`, `Env::for_tests`)
app/worker.rs            offload, perform, StreamHandle, spawn_stream, spawn_iter: off-thread work and cancellable streams
app/dialogs.rs           every rfd file dialog; BOWSHOCK_EXPORT_FILE override in `save_file`
app/selection_set.rs     PickMode, MultiSet<Id>: primary + extras + range anchor, shared by app/multi.rs and simulate/mod.rs
app/document.rs          Document: the rocket + path + dirty + undo/redo (every mutation goes through it)
app/physics.rs           cached balance (mass, CG, CP, margin), warnings, stages, motor db, per-stage CFD plans / drag tables (bowshock-sim / -motors)
app/drag.rs              shared drags (bin -> viewport, grab, move) + delete, built on Document's drag support. `end_carry(ctx, Gesture, Over)` is the one decision point for what a release or a click does to a part in hand, in drag and drop and in click to place (`ctx.click_to_place`): every overlay, the lanes, the bin and the window root report to it; a part dropped on the bin (`ctx.over_bin`, set by the bin) is deleted through the same `delete_parts` Delete uses, or discarded if it came from the bin
app/files.rs             New / Open / Save / Save As / the built-in example, unsaved-changes guard; `.ork` import, which first registers the thrust curves stored in the file (`use_embedded_curves`) so they never reach the ThrustCurve lookup
app/motor_fetch.rs       after a load (`Ctx::fetch_wanted`, `load_ticket`), looks up motors the database cannot resolve on ThrustCurve.org (one worker task; exactly-one-match rule; `Env::fetch_motors` gates it) and adds them via `add_downloaded`
app/ui.rs                shared UI helpers: `label`, `mm`; re-exports the unit readouts from `units`
chrome/                  top bar, status strip, config menu, issues list, motor picker (State + Message)
chrome/menu_bar.rs       the File / Edit / View menu bar: labels, the dropdown overlay and each menu's rows as data (`TopMenu`, `Ctx::menu`, `app::Message::Menu`); the bar's x positions are shared constants with `top_bar.rs`
chrome/run_toast.rs      the run toast under the top bar (non-modal) while `ctx.run` is set and the mode is not Simulate; the Simulate button reads `Running…` while flights are pending
build/mod.rs             Build screen composition (pane_grid) only
build/parts_bin.rs       parts bin
build/inspector.rs       inspector
build/lanes.rs + lanes/     Station Lanes: `scene` (bars, sub-rows, stages, mass), `geometry` (stations <-> px, ruler, hit tests), `edit`
                           (move / resize / landing / snap on the Rocket), `canvas` (drawing + mouse), `menu` ("Add here…").
                           The lanes always fit the whole rocket (zoom/pan by hand, reset returns); they do not follow the viewport camera.
                           The Build screen's centre pane is the viewport over the fixed-height lanes (`lanes::height`), not a pane_grid pane.
viewport/                3D viewport (View = state, Message) + overlays (assembly, surface)
viewport/manipulate/     selection / handle overlay: `mod` (state, Edit), `query` (hit tests, drag math), `paint` (labels, shapes), `program` (canvas::Program), `draw` (overlay painters)
viewport/capture.rs      One-shot offscreen render of the model at a chosen pixel size (image export): `Capture` request, read back in `Primitive::prepare`, pixels returned on a oneshot channel.
export/                  File > Export 3D model… / Export image… (State + Message, modal dialogs); `mesh_export.rs` writes OBJ / binary STL (mm) from the viewport mesh.
print/                   File > Print / export PDF…: `pdf.rs` (PDF 1.4 writer, Helvetica core fonts), `templates.rs` (full-size fin / ring / wrap / marking templates, tiling), `report.rs` (parts list, design report); built on a worker thread.
prefs/                   Edit > Preferences… (top bar Preferences button, ⌘, / Ctrl+,): the modal window, a section list and a page each: `appearance.rs` (theme), `editor.rs` (how parts are placed: drag and drop or click to place), `units.rs` (unit set select and per-quantity rows), `compute.rs` (the machines the solves run on: switches, per-machine connection test, use one / share), `storage.rs` (files and caches, sizes measured off-thread, Open folder / Clear). Every control applies at once through `Ctx::{set_dark, set_click_to_place, set_units, save_settings}` and is kept in `app/settings.rs`; the app intercepts all other messages while it is open (`route.rs`).
simulate/mod.rs          Simulate screen (`format.rs`: flight-only formatting: bearing, landing, spin). A staged flight's result carries one `bowshock_sim::Branch` per separation (the dropped stage flown on its own, `flight/branch.rs`): `plots.rs` draws each as a dashed line from the separation (the `Dropped stages` header toggle hides them), a thin `StripRow` under the events strip and a dashed ground `Track::dropped`; `results.rs` adds a section per branch after the recovery
analyze/mod.rs           Analyze screen
analyze/cfd/             CFD table method: `plan` (document -> `bowshock-core` mapping, table keys, coarse (default) or medium (Refine) grid), `fresh` (table status), `run` (off-thread solve, events), `share` (the work queue that hands Mach numbers to the machines that are on), `remote` (a machine over SSH through `bowshock-remote`), `interp` (table -> plot points, Mach / alpha interpolation). Both solves call the one `bowshock_core::Analysis::run(&[Condition])`: the drag table one `Condition::from(mach)` per node (zero incidence, the drag solver), the stability solve both angles of a Mach number as `Condition::new(mach, alpha)` (the 3D solver; its points carry `PolarPoint::forces`, which `cfd/run.rs` stores as `CfdLiftPoint`). A remote machine gets the same call as one `Job::Solve`.
                         Tables are stored in the document (`bowshock_model::CfdTable`, a cache: not an edit, not in undo).
                         The final flow field of each point (`bowshock_core::Field3`, shared as `Arc`) rides the solve's events (`Event::PointField`; a local solve also sends `Event::Field` snapshots every 500 iterations) into `cfd/fields.rs`' `FieldStore`: session-only, beside the table state, keyed by Mach node and angle of attack, never saved, dropped when the geometry or the table it belongs to changes. The 2D solver's `Field` becomes a `Field3` in `flow_view/axi.rs` (`FieldRef::from_2d`), the one place the viewer names it. The 3D stability solve brings real `Field3`s: live snapshots of the running point (`Analysis::on_field3`, every 500 iterations, local machine only) as `Event::Field`, and the final field of each angle of each Mach number as `Event::PointField`; `FieldStore::replace_stability` keeps them beside the drag solve's (their Mach numbers are the stability solve's own), so the Flow field view lists those Mach numbers too and its angle stepper walks 0 (the drag solve's meridian) and the stability angles.
analyze/solve.rs         Live state of a CFD solve (`Tracker`): units (Mach points, or Mach x angle), per-point outcome, weighted progress fraction and time left from the measured throughput. Pure, no clock.
analyze/solve_view.rs    The solve on screen: headline, bar, the Cd(Mach) (or CN) curve filling in, the point in flight with its monitor trace, the points list; replaces the plots while a solve runs.
analyze/solve_demo.rs    `BOWSHOCK_CFD_DEMO`: scripted solve events for screenshots (the driver cannot wait on a minutes-long real solve) and the screen-level tests. `flow3d-mid` / `flow3d-done` stream the committed finned 3D fixture (`BOWSHOCK_FLOW3_FIXTURE` overrides the file) as the flow of each point, two angles of attack per Mach node when done.
analyze/flow_view.rs     The flow viewer, in the solve screen (the live picture) and as the `Flow field` view of a finished table. `State` (mode, scalar, grid, camera, Mach and angle steppers), the side view's screen transform `Xf`, canvas overlay (body, ticks, hover readout, wall Cp under it) and the Cd(Mach) picker. Two modes (`Mode`): `Side` (a meridian plane, drawn by `gpu.rs`) and `Scene` (a 3D scene, `gpu3.rs`). Files in `flow_view/`: `meridian.rs` (one cell layer of a `Field3` as a 2D quad mesh), `prepared.rs` (a meridian made ready: Mach, Cp, schlieren per cell, ranges, one vertex per node, the cell under the pointer), `map.rs` (colour maps), `gpu.rs` + `flow.wgsl` (the side view: a `shader` primitive per half-plane), `scene3.rs` (`Prepared3`: ranges, wall mesh, sheet builders, the plane through the axis), `gpu3.rs` + `scene3.wgsl` (the scene), `pick3.rs` (CPU ray picking for the hover), `scene_view.rs` (`SceneState`, the palette, the overlay canvas, orbit / pan / zoom), `axi.rs` (the 2D `Field` adapter). See "The flow viewer" below.
widgets/                 shared widgets (read-only for screens; do not edit from a screen task); `modal.rs` (modal card + `scrim`), `unit_input/` (the unit field: `builder`, `logic`, `widget`, `cells`)
units.rs                 unit set (UnitSystem), all SI/display formatting (`readout`, `with_unit`, `bare`, `len`, `len_unit`, `num_scaled`, `with_unit_scaled`, `signed`), expression parsing
```

## The module contract

Every area (`parts_bin`, `viewport`, `lanes`, `inspector`, `simulate`, `analyze`, `chrome`) has the same shape:

```rust
pub enum Message { .. }                 // Debug + Clone; the area's own messages
pub struct State { .. }                 // viewport's is `View`
impl State {
    pub fn new() -> Self;
    pub fn update(&mut self, msg: Message, ctx: &mut Ctx) -> Task<app::Message>;
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message>;      // the app maps it with `.map(app::Message::X).boxed()`
    pub fn on_key(&self, key: &keyboard::Key, mods: keyboard::Modifiers, ctx: &Ctx) -> Option<Message>;
}
```

`simulate` and `analyze` also have `tick(now)` / `animating()` (frame ticks), `viewport::View` has `sync(&Ctx)`, `tick`,
`animating`, `fit`.

`app::Message` already has a variant for each area: `PartsBin(..)`, `Viewport(..)`, `Lanes(..)`, `Inspector(..)`,
`Simulate(..)`, `Analyze(..)`, `Chrome(..)`. **To add a message, add a variant to your own `Message` and handle it in your
own `update`.** Nothing in `app/mod.rs` changes. If your view needs a message from another area (e.g. the lanes select a part
the inspector shows), do not send it: put the effect in shared state (`ctx.selection`, `ctx.doc`, `ctx.drag`) and every area
sees it on its next `view`. If you really need to trigger another area's behaviour, return
`Task::done(app::Message::Other(..))` from `update`.

`view` borrows `ctx` for the element's lifetime (`'a`), so it can hand out `&'a str` / `&'a [T]` slices of document data
without cloning. Anything computed in `view` (a formatted `String`) is owned by the widget.

### Keyboard

One place: `app::shortcut` (global: Esc, F5, F2, Delete, Ctrl/Cmd+Z/Shift+Z/Y/N/O/S/Shift+S/E/L) and then, for keys that no
widget handled (`!captured`, so typing in a text field never triggers a shortcut), the active mode's areas are asked in order
via `on_key`. Add your shortcut in your own `on_key`; return your own `Message`. Build asks: parts bin, viewport, lanes,
inspector. Simulate and Analyze ask their one area. Modifier-free plain-character shortcuts must check
`modifiers.is_empty()` (Shift-only if intended): the global map already owns the command-key combinations.

## `Ctx`: the shared state

```rust
pub struct Ctx {
    pub doc: Document,          // the open document: rocket(), edit*(), undo/redo, path, dirty, revision
    pub selection: Selection,   // None | Part(PartId) | Separation(stage)  (any part class: axial, radial group, internal)
    pub drag: Drags,            // axial / radial drag in progress + symmetry + angle_snap
    pub physics: Physics,       // balance(), issues(), stages(), motors(), margin_with_motor(), phase(), cfd_plan(stage), drag_table(rocket, stage)
    pub units: UnitSystem,      // the display unit set: one unit per Family (Preferences > Units), kept in app/settings.rs (user config, not the rocket); format with crate::units::{with_unit, bare, readout, len, len_unit} (also re-exported by app::ui)
    pub cursor: Option<Point>,  // window coordinates, app-wide
    pub mode: Mode,             // Build | Simulate | Analyze
    pub dark: bool,             // ctx.tokens() gives the design tokens of the current theme
    pub compare_configs: bool,  // "Compare all in lanes"
    pub notice: Option<(String, Instant)>,  // ctx.notify("text") shows it in the status strip for 3 s
    pub camera_fit: bool,       // set true to ask the viewport to frame the rocket once its mesh is current
    pub env: env::Env,          // runtime config read once from the process (BOWSHOCK_* vars, animate, freeze_flash)
}
```

`Ctx` is passed `&Ctx` to `view`/`on_key` and `&mut Ctx` to `update`. The app calls `Physics::sync` (recompute when the
document's revision or the phase changed), `View::sync` (rebuild the mesh) and drops a selection that no longer exists after
every message, so **an area never refreshes physics or the mesh by hand**.

## Editing the rocket and undo

Never mutate a `Rocket` directly. Everything goes through `ctx.doc`:

```rust
ctx.doc.edit("Resize fin", |r| r.set_radial(id, kind, count));      // one undo step (recorded only if something changed)
ctx.doc.edit_merging("Edit root chord", "root_chord:42", |r| ..);    // consecutive calls with the same key = ONE step
ctx.doc.seal();                                                      // end the merge run
```

* Scrubbing a field or dragging a handle sends many messages: use `edit_merging` with a key naming the thing being edited
  (`"resize:{id}"`), and call `ctx.doc.seal()` when the interaction ends (`viewport::Message::EditEnd` does it for handles; the
  app also seals on every mouse release). A run that ends exactly where it began leaves no step.
* A drag that takes parts out of the rocket and puts them back uses `begin_drag(label)` / `drag_update(|r| ..)` /
  `end_drag(commit)` through `app::drag` (`start_axial`, `start_radial`, `grab`, `finish_axial`, `finish_radial`, `cancel`). A
  cancelled drag restores the **whole** pre-drag rocket (materials, finishes, overrides, internals, motors in every
  configuration). Add new drag kinds (e.g. an internal part in the lanes) as a field of `Drags` plus a start/finish pair in
  `app/drag.rs`.
* Undo/redo (Ctrl/Cmd+Z, Ctrl/Cmd+Shift+Z, Ctrl+Y), dirty flag, the file name and the dirty square in the top bar,
  New/Open/Save/Save As (`.bowshock` is binary: `BOWSHOCK` magic, a `u16` version, then deflated CBOR; `Rocket::to_bytes` /
  `from_bytes`, layout and evolution rules in `bowshock-model/src/format.rs`) all live in `app/document.rs` and
  `app/files.rs`. `Document::revision()` increases on every change including live drag updates, undo and redo; derived data
  is stale when its revision differs.
* Configurations are part of the rocket: switching, adding, renaming, deleting a configuration are `ctx.doc.edit(..)` calls
  (see `chrome/mod.rs`, `chrome/config_picker.rs`), so they undo.

## Selection

`ctx.selection` is app-level and shared by the viewport, lanes and inspector. Set it directly
(`ctx.selection = Selection::Part(id)`); it is validated after every message. `Selection::Separation(n)` is the separation of
stage `n` from the stage ahead (`Stage::index` of the stage below the joint, from 1; the lower stage separates). The joint is
the rocket's own: `Rocket::separation_joints(n)` lists the eligible ones and `set_separation_joint(n, joint)` moves it (inspector
select = one `Move separation` step; the diamond in the viewport and the line in Station Lanes drag it through
`app::drag::{move_separation, finish_separation}`, the document's drag support, so Esc restores it). While a part is being
carried by a drag it is out of the rocket but
stays selected.

**Multi-selection** (`app/multi.rs`, on `app/selection_set.rs`'s `MultiSet<PartId>`; Simulate's flight selection uses `MultiSet<FlightId>`): `ctx.selection` stays one value, `Selection::Part(primary)`; the parts selected with it live
in a private list tied to that primary, read with `ctx.selected_parts()` (primary first) / `ctx.is_multi()`. Shift-click and
Cmd/Ctrl-click in the viewport (`viewport::Message::Toggle`) and in Station Lanes (`lanes::Message::Toggle`) call
`ctx.toggle_part(id)`; a plain click calls `ctx.select_one(..)`; Esc (no drag open) or an empty click clears. Anything that sets
`ctx.selection` directly makes a single selection, because the extras are void unless the selection is still their primary
(validated after every message, like the rest of the selection). With several parts selected the viewport draws brackets on every
one and a single `N PARTS` tag and **no handles** (a handle edits one part's dimension; the kit has no multi-handle gesture), the
lanes bracket every selected bar, the inspector (`build/inspector/multi.rs`) shows only the fields all of them share (the same
`Dim`, material kind, finish, mass override; a mixed value is empty with a `—` placeholder, `UnitInput::mixed`) and a typed value
sets every part as one undo step, Delete removes all of them as one step, and the status strip's `SELECTED` sums their mass.

## Status strip while dragging (`app/live.rs`)

A handle drag, a lane move/resize, a carried part or a scrubbed field is *live* while the document's drag or merged-edit run is
open (`Document::editing_live`, closed by the release). The balance before the first change is the baseline; each readout of the
strip whose value differs from it (`ctx.changing()`) is drawn `ink` at weight 600 (`status::readout_live` and friends) until the
release.

## Symmetry groups (design/components/Symmetry)

A radial group is one `RadialPart` (shared kind, start angle, count). Copies are lettered A, B, C (`bowshock_model::instance_letter`)
and a copy can loosen single fields from the group: `RadialPart::overrides` lists `(copy, RadialField, value)`
(`Rocket::unlink_field`, `relink_field`, `relink_copy`, `set_instance_dim`). **Everything that needs a copy's values reads
`RadialPart::effective(copy)` / `Rocket::effective_params(id, copy)` (the spec's `effective_params`), never the group's own**:
the mesh, picking, handles, `Rocket::component_mass` and `bowshock_sim::from_model` (copies with equal effective values fly
as one `FinSet`, so a canted fin B makes a set of two and a set of one) all do. Changing the count drops the overrides
(`set_radial`), so the inspector asks first. `Rocket::detach_radial` / `split_all_radial` freeze the effective values into the free
parts; `rejoin_radial(part, group, Winner)` merges a free part back (group's or part's settings win). Documents without
overrides serialise exactly as before (the list is skipped when empty), so there is no format-version bump.

`ctx.symmetry` (`app/symmetry.rs`) is the view state the three surfaces share: `instance` (the clicked copy of the selected group,
set by a viewport click or the header letter; read with `copy_in(rocket, id)`) and `rejoin` (a free part and the group it is about
to join; set by the inspector's *Rejoin group* or by dropping the part's bar on the group's bar in the lanes, answered in the
inspector bar). Both are validated after every message. The inspector's symmetry UI is `build/inspector/symmetry.rs`
(`Link`, `linked`, `bar`): right-click a field of a group for *Unlink for this fin* / *Relink field*, link icons after the fields
once the group has overrides, the confirmation before a count change, and the rejoin choice.

## Pods / strap-ons (model `pod.rs`, viewport `pods.rs`)

A pod set is a `RadialPart` of kind `RadialKind::Pod { gap, body }` (count, angle, offset like fins; always on a core part, pods do not
nest). Its `PodBody` holds its own axial stack and the radial / internal parts on and in it, all in the one id space, so every id-taking
`Rocket` method works for a part inside a pod (it runs on `Rocket::pod_rocket`, the pod as a small rocket, and writes back).
`Rocket::parts()/radials()/internals()/stations()/index_of()` are **core only**; use `ids()`, `pods()`, `pod_of(id)`, `pod_frame(pod)`.
Mass includes every copy (parallel axis about the core axis); `bowshock-sim` flies each copy as an extra body (see `from_model.rs`).
The viewport looks at a pod copy through `Camera::in_frame(Pose)`, a camera in the pod's own coordinates, so picking, handles,
brackets and the fin-landing surface reuse the core's code on the pod's small rocket; the mesh carries the pod's own mesh by the pose.
Grabbing any part of a pod carries the whole pod; the `POD ×N` tag under a selected pod part selects the set. Not supported: detaching
copies, nested pods, motors in pods.

## Physics

`ctx.physics` is a cache, current when `view` runs: `balance()` (mass, length, ref diameter, CG, CP, margin, CNα for the phase
on show at liftoff, `None` for an empty rocket), `issues()` (non-`Ok` warnings, each with an optional `part` to
select), `stages()`, `is_staged()`, `phase()`, `config_margin(id)`, `margin_with_motor(rocket, mount, motor)`,
`margins_with_motors(rocket, mount, indices)` (the picker's whole column at once), `mounts_of_stage(rocket, stage)`, `motors()`
(the `MotorDb`: the bundled ThrustCurve.org catalogue, cached downloads `<motorId>.motor` and the user's `.eng` / `.rse` files from
`ctx.env.motors_dir`), `motor_origin(index)`, `add_downloaded(motor)`, `add_embedded(motor)` (a curve stored in an imported `.ork`), `gauge()` (animated margin gauge). Phase is a view
setting (`Phase::Full` or `Phase::UpperFrom(k)`), set from the status strip.

It also caches the CFD plan of every stage: `cfd_plan(stage)` and `drag_table(rocket, stage)` are rebuilt once when
`(revision, phase, units)` changes (`Physics::sync(rocket, revision, units)`), so Simulate's `view` and Analyze (CFD plan
status, the Dispersion drag tables) never build a vehicle or a `cfd::Plan` themselves.

Motors are identified by `Motor::id` (`tc:<ThrustCurve motorId>` or `eng:<curve hash>`), and a configuration's `MotorRef`
carries that id, so two curves with one designation stay two motors. Look motors up with `from_model::find_motor` (id first,
then designation and maker) and make references with `from_model::motor_ref`; never match on the designation alone.

An OpenRocket 26.xx `.ork` stores user-defined curves as `thrustcurves/<digest>.rse` in its ZIP and names each motor by `<digest>`;
`ork::import` returns them raw (`embedded_curves`, `motor_digests`; the model crate has no motor parser). On import,
`files::use_embedded_curves` takes every referenced `(designation, maker, digest)` the database cannot resolve and has a stored curve,
parses it with `bowshock_motors::parse_rse`, and adds its first engine under the *design's* name (`Motor::renamed`: one digest under
three names is three motors, ids `eng:<hash of name + data>`) through `Physics::add_embedded`: a `File`-origin (USER) motor kept as
`<designation>-<hash>.eng` in the motors directory, so the saved document's `MotorRef` id resolves next session. The imported
references get those ids, and the load-time ThrustCurve lookup (`motor_fetch`) then finds nothing missing for them.

## Overlays and the viewport

The viewport pane is `viewport::View::view`; overlays (`assembly` for axial drags, `surface` for radial drags,
`manipulate` for selection/handles) are canvases that report to `viewport::Message` (`SnapNode`, `Surface`, `DropAxial`,
`DropRadial`, `Hover`, `Select`, `Grab`, `Edit`); `View::update` turns them into document edits
and drag calls. Camera messages (`Orbit`, `Pan`, `Zoom`, `Fit`, `Snap`, `Home`, `Roll`, `Tick`) are the same enum. The mesh
(`View::sync`) rebuilds from `ctx.doc.revision()`, hover, selection and the drag ghosts.

A part picked up from the rocket (`Drag::origin`) is out of the document while carried, with its radial and internal parts held in
`assembly::Origin` (motors: from `Document::drag_base`). Its ghost is the part landed by the same call a drop uses
(`assembly::land_carried`) with the part and all it carries drawn in the ghost state (`Marks::ghost` = `mesh::Ghost`, its
`carried`), so fins, rings and the loaded motor appear as they will. With no target (`Marks::free`) the same ghost, alone as a
small rocket (`assembly::free_rocket`), follows the cursor on the plane through the rocket's axis that faces the camera. A part
from the bin has no `origin`: its ghost is the bare part.

Handle edits (DirectEdit, Snapping): the model lists every part's handles (`Rocket::handle_specs`), reads and writes a dimension
(`Rocket::dim`, `clamp_dim`, `set_dim`, the same calls the Inspector uses) and offers snap targets (`Rocket::snap_targets`).
`viewport/handles.rs` places the handles for the camera, `viewport/snap.rs` picks the catching target, and `manipulate`
drives one drag as `Edit::Begin`, `Dim`/`Vertex`..., `End` (one undo step; `Cancel` restores). `BOWSHOCK_DRIVE_MODIFIERS`
(`shift`, `alt`, `ctrl`) holds modifiers for drive scripts (`scripts/directedit-snap.drive`).

## Runtime plumbing (`app/env.rs`, `app/worker.rs`, `app/dialogs.rs`, `widgets/modal.rs`)

* `ctx.env` (`Env`) is the one place that reads `BOWSHOCK_*` variables and `$HOME` fallbacks (read once at launch;
  `Env::for_tests()` touches nothing on the host). It also carries `animate` and `freeze_flash`; nothing else holds global
  animation flags. Read config from `ctx.env`, never `std::env`.
* `app::worker`: `offload` / `perform` run one closure on a thread; `spawn_stream` / `spawn_iter` return a `Task` plus a
  `StreamHandle` (serial, `request_cancel`, `abort`) for cancellable beat + worker streams (CFD solves, optimiser, scripted demo).
* `app::dialogs` wraps every `rfd` file dialog (`pick_file`, `pick_files`, `save_file`, `pick_bytes`, `pick_named_text`);
  `save_file` honours `ctx.env.export_file` so headless runs skip the dialog. Screens never import `rfd`.
* `widgets::modal::modal` is the one dimmed-scrim card (export, print, scale, unsaved changes); `widgets::modal::scrim` is the bare
  scrim. `App::intercept_modals` (in `app/route.rs`) owns Enter/Escape while one is open.

## Chrome (top bar, status strip, config menu, motor picker)

`chrome::top_bar(ctx)` and `chrome::status_strip(&chrome_state, ctx)` return `Element<app::Message>`; menus and the picker
are `chrome::State::overlay(ctx)`. The strip formats everything with `ctx.units`. Chrome-only state (open menu, rename
buffer, picker) is `chrome::State`; its messages are `chrome::Message`. The motor picker is
`chrome/config_picker.rs` (`Picker`, `Message`, `Outcome`). Its ThrustCurve.org source returns `Outcome::Debounce`, `Search` and
`Download`, which `chrome::State::update` turns into tasks that run on worker threads (`config_picker::debounce`, `search`,
`download`); `$BOWSHOCK_THRUSTCURVE_API` points the client elsewhere (`scripts/thrustcurve-offline.drive`).

## The flow viewer (`analyze/flow_view*`)

The data is a `bowshock_core::Field3` (`field3.rs`: grid nodes, per-cell `[rho, u, v, w, p]`, solid flags, the wall faces with Cp and
skin friction, the symmetry the solved domain stands for). The solver world is x along the rocket, z up the lee side, the free stream
towards +z at angle of attack; the scene's world is `(X, Y, Z) = (solver y, z, x)`, so the camera is `viewport::camera::Camera`.
The viewer has two modes; the toggle (`Side view | 3D`) is hidden for a field with one cell round the axis (the 2D solver's), which
opens side-on.

* **Side** (`State`, `gpu.rs`): one meridian layer of the field (`Meridian`, then `Prepared`) as the picture it always was. At an
  angle of attack the flow is not mirror-symmetric, so the layer on the lee side is drawn above the axis and the layer across the
  axis (`scene3::plane_through`) below, each its own `Prepared` and its own `shader` widget (`gpu::Half`), one colour scale for both;
  the wall Cp under the picture has a dashed curve for the lower half. For a one-cell field both halves are the same and the lower
  one is not built. `Fit range` sets the colour range to what is on screen (robust percentiles of the cells in view).
* **Scene** (`SceneState`, `gpu3.rs`, `scene3.wgsl`): the vehicle's wall faces coloured by Cp or skin friction, a *meridian* sheet
  (the whole plane through the axis: layer `k` and the layer opposite it, found through the symmetry tiles, or one layer in every copy
  with `Tile`) and a *cross-flow* sheet (one cell layer along the rocket, a disc), coloured by Mach, Cp or schlieren. The sheets cover
  a window of the grid (`Extent`: Near / Wide / Far); a fragment `discard` clips them at its edge. Everything is drawn once per
  symmetry tile (instances), so the half-domain ANF grid shows as the whole rocket. Orbit: right or left drag; pan: middle drag or
  Shift; zoom: wheel or pinch; `F` or a double click frames it. The hover is a CPU ray (`pick3.rs`) against the sheets' cells and the
  wall faces. The colour bars, the axes of the solver's frame, the free-stream arrow (`V∞`, Mach and angle) and the readout are a
  canvas over the picture.

`Prepared3` (ranges, wall mesh, the stations of the grid, sheet builders) is made on the UI thread when a picture arrives: a snapshot
of the same mesh (`Field3::mesh_id`) keeps the user's choices (sheets, station, plane, camera, a fitted range is refitted) and builds
the picture again; a new mesh starts at the defaults. Measured in release on an RTX 3090 and this machine's CPU
(`flow3_cost_at_solver_sizes`, `snapshot_cost_at_solver_sizes`,
both `#[ignore]`: `cargo nextest run --release -p bowshock <name> --run-ignored only --no-capture`; the level 0 finned ANF case is real
solver output, `field3_dump --case level0`; the 0.5 M and 2 M cases are synthetic bodies without fins, 7 - 17 k wall faces):

| case | cells | `Prepared3::new` | whole meridian plane | cross-flow disc | frame (1280 x 720, 4x MSAA) | first frame, upload | `set_field` of a snapshot |
|---|---|---|---|---|---|---|---|
| ANF level 0 | 352 800 | 4.2 ms | 4.0 ms | 0.4 ms | 0.12 ms | 0.9 ms | 10.6 ms |
| synthetic | 499 200 | 4.1 ms | 1.8 ms | 0.4 ms | 0.14 ms | 0.4 ms | 10.5 ms |
| synthetic | 2 000 000 | 13.2 ms | 3.1 ms | 1.6 ms | 0.17 ms | 0.7 ms | 28 ms |

The solver side of a live 3D snapshot is `field3_of` on the thread that runs the solve (5 - 6 ms at 353 000 cells; the GPU runner has the state on the host already for its normal-force monitor): a 750-iteration GPU solve of two angles took 1.80 s plain and 1.85 s with a snapshot every 100 iterations (five times the app's rate) and the final fields kept, +2.8 % (`gpu_snapshots_and_kept_fields_cost_nothing_measurable`, RTX 3090); the app's 16-run stability table (12 250 iterations, 16 snapshots, 16 final fields) took 33.3 s before the facade unification and 30.3 s after it. A snapshot comes every 500 iterations (seconds apart), so a 10 - 30 ms pause on the UI thread is accepted for now; moving
`Prepared3::new` into a `Task` is the next step if a real 2 M cell field is slower. On the solver side `field3_of` took 0.53 ms for
20 352 cells and 6.2 ms for 352 800 (the CBOR fixture encoding, 0.7 s for the latter, is only the dump tool). The side view's
`Prepared::new` was 38 ms for 27 280 cells (M1 Max, debug build); its frame cost was 0.70 ms for 27 280 cells.

Drive screenshots of GPU content need the wgpu test backend: `ICED_TEST_BACKEND=wgpu cargo run -p bowshock -- drive ...`. The
default software renderer draws no `shader` widgets, so the picture area is blank. `scripts/flow3d.drive` with
`BOWSHOCK_CFD_DEMO=flow3d-mid,flow3d-done` plays the committed finned-finner fixture (`tests/data/anf_m2_a4.field3`, Mach 2, 4 degrees,
20 352 cells; its velocities are scaled to each point's Mach number and its disturbance to the angle, nothing is solved) as a solve in
flight and as a finished table with two angles per Mach number; `scripts/flow-view.drive` keeps the 2D wedge picture.

## Tests and the drive harness

Logic is tested on the types (document, physics, drag via `App::update`, picker). UI is exercised headless:

```
cargo run -q -p bowshock -- drive [--example] <script> [out-dir]
```

`--example` loads the built-in example rocket (`assets/examples/hpr_l1_cert.bowshock`, kept in step with the model by
`app::files::tests::example_is_current`; regenerate with `BOWSHOCK_WRITE_EXAMPLE=1 cargo test -p bowshock example_is_current`).
Script syntax is in `drive.rs`; `scripts/shell.drive` exercises the chrome. A drive script cannot press shortcuts with
modifiers, so click the top-bar buttons or use `--example`.
