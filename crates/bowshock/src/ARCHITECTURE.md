# Bowshock app architecture

One `App` (in `app/mod.rs`) owns one shared context, `Ctx`, and one state struct per area. A single top-level
`app::Message` wraps each area's own `Message`. Modules work in parallel by owning their own file(s) and never needing to
touch `app/mod.rs`.

```
main.rs                  CLI (app / drive / gallery) and the `app()` builder only
app/mod.rs               App, Ctx, Message, Selection, routing, the global keyboard map
app/document.rs          Document: the rocket + path + dirty + undo/redo (every mutation goes through it)
app/physics.rs           cached balance (mass, CG, CP, margin), warnings, stages, motor db (bowshock-sim / -motors)
app/drag.rs              shared drags (bin -> viewport, grab, move) + delete, built on Document's drag support
app/files.rs             New / Open / Save / Save As / the built-in example, unsaved-changes guard
app/ui.rs                shared UI helpers: `label`, `mm`, unit-system readouts
chrome/                  top bar, status strip, config menu, issues list, motor picker (State + Message)
build/mod.rs             Build screen composition (pane_grid) only
build/parts_bin.rs       parts bin
build/inspector.rs       inspector
build/lanes.rs + lanes/     Station Lanes: `scene` (bars, sub-rows, stages, mass), `geometry` (stations <-> px, ruler, hit tests), `edit`
                           (move / resize / landing / snap on the Rocket), `canvas` (drawing + mouse), `menu` ("Add here…").
                           The lanes always fit the whole rocket (zoom/pan by hand, reset returns); they do not follow the viewport camera.
                           The Build screen's centre pane is the viewport over the fixed-height lanes (`lanes::height`), not a pane_grid pane.
viewport/                3D viewport (View = state, Message) + overlays (assembly, manipulate, surface)
simulate/mod.rs          Simulate screen
analyze/mod.rs           Analyze screen
analyze/cfd.rs           CFD table method: document -> `bowshock-core` mapping, the off-thread solve, table keys, table -> plot points.
                         Tables are stored in the document (`bowshock_model::CfdTable`, a cache: not an edit, not in undo).
widgets/                 shared widgets (read-only for screens; do not edit from a screen task)
units.rs                 unit set (UnitSystem: one unit per Family, named presets), formatting, expression parsing
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
    pub physics: Physics,       // balance(), burnout(), issues(), warnings(), stages(), motors(), margin_with_motor(), phase()
    pub units: UnitSystem,      // the display unit set: one unit per Family (top bar select + Custom… panel), kept in app/settings.rs (user config, not the rocket); format with app::ui::{with_unit, bare, readout, len, len_unit}
    pub cursor: Option<Point>,  // window coordinates, app-wide
    pub mode: Mode,             // Build | Simulate | Analyze
    pub dark: bool,             // ctx.tokens() gives the design tokens of the current theme
    pub compare_configs: bool,  // "Compare all in lanes"
    pub notice: Option<(String, Instant)>,  // ctx.notify("text") shows it in the status strip for 3 s
    pub camera_fit: bool,       // set true to ask the viewport to frame the rocket once its mesh is current
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
stage `n` from the stage ahead (`Stage::index`, from 2). While a part is being carried by a drag it is out of the rocket but
stays selected.

**Multi-selection** (`app/multi.rs`): `ctx.selection` stays one value, `Selection::Part(primary)`; the parts selected with it live
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
on show at liftoff, `None` for an empty rocket), `burnout()`, `issues()` (non-`Ok` warnings, each with an optional `part` to
select), `stages()`, `is_staged()`, `phase()`, `config_margin(id)`, `margin_with_motor(rocket, mount, motor)`,
`margins_with_motors(rocket, mount, indices)` (the picker's whole column at once), `mounts_of_stage(rocket, stage)`, `motors()`
(the `MotorDb`: the bundled ThrustCurve.org catalogue, cached downloads `<motorId>.motor` and the user's `.eng` files from
`user_motors_dir()`), `motor_origin(index)`, `add_downloaded(motor)`, `gauge()` (animated margin gauge). Phase is a view
setting (`Phase::Full` or `Phase::UpperFrom(k)`), set from the status strip.

Motors are identified by `Motor::id` (`tc:<ThrustCurve motorId>` or `eng:<curve hash>`), and a configuration's `MotorRef`
carries that id, so two curves with one designation stay two motors. Look motors up with `from_model::find_motor` (id first,
then designation and maker) and make references with `from_model::motor_ref`; never match on the designation alone.

## Overlays and the viewport

The viewport pane is `viewport::View::view`; overlays (`assembly` for axial drags, `surface` for radial drags,
`manipulate` for selection/handles) are canvases that report to `viewport::Message` (`SnapNode`, `Surface`, `DropAxial`,
`DropRadial`, `Hover`, `Select`, `Grab`, `Edit`); `View::update` turns them into document edits
and drag calls. Camera messages (`Orbit`, `Pan`, `Zoom`, `Fit`, `Snap`, `Home`, `Roll`, `Tick`) are the same enum. The mesh
(`View::sync`) rebuilds from `ctx.doc.revision()`, hover, selection and the drag ghosts.

Handle edits (DirectEdit, Snapping): the model lists every part's handles (`Rocket::handle_specs`), reads and writes a dimension
(`Rocket::dim`, `clamp_dim`, `set_dim`, the same calls the Inspector uses) and offers snap targets (`Rocket::snap_targets`).
`viewport/handles.rs` places the handles for the camera, `viewport/snap.rs` picks the catching target, and `manipulate`
drives one drag as `Edit::Begin`, `Dim`/`Vertex`..., `End` (one undo step; `Cancel` restores). `BOWSHOCK_DRIVE_MODIFIERS`
(`shift`, `alt`, `ctrl`) holds modifiers for drive scripts (`scripts/directedit-snap.drive`).

## Chrome (top bar, status strip, config menu, motor picker)

`chrome::top_bar(ctx)` and `chrome::status_strip(&chrome_state, ctx)` return `Element<app::Message>`; menus and the picker
are `chrome::State::overlay(ctx)`. The strip formats everything with `ctx.units`. Chrome-only state (open menu, rename
buffer, picker) is `chrome::State`; its messages are `chrome::Message`. The motor picker is
`chrome/config_picker.rs` (`Picker`, `Message`, `Outcome`). Its ThrustCurve.org source returns `Outcome::Debounce`, `Search` and
`Download`, which `chrome::State::update` turns into tasks that run on worker threads (`config_picker::debounce`, `search`,
`download`); `$BOWSHOCK_THRUSTCURVE_API` points the client elsewhere (`scripts/thrustcurve-offline.drive`).

## Tests and the drive harness

Logic is tested on the types (document, physics, drag via `App::update`, picker). UI is exercised headless:

```
cargo run -q -p bowshock -- drive [--example] <script> [out-dir]
```

`--example` loads the built-in example rocket (`assets/examples/hpr_l1_cert.bowshock`, kept in step with the model by
`app::files::tests::example_is_current`; regenerate with `BOWSHOCK_WRITE_EXAMPLE=1 cargo test -p bowshock example_is_current`).
Script syntax is in `drive.rs`; `scripts/shell.drive` exercises the chrome. A drive script cannot press shortcuts with
modifiers, so click the top-bar buttons or use `--example`.
