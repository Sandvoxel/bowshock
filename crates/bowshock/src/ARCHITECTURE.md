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
app/ui.rs                shared UI helpers: `label`, `mm`, unit-system readouts, `three_columns`
chrome/                  top bar, status strip, config menu, issues list, motor picker (State + Message)
build/mod.rs             Build screen composition (pane_grid) only
build/parts_bin.rs       parts bin
build/inspector.rs       inspector
build/lanes.rs           Station Lanes
viewport/                3D viewport (View = state, Message) + overlays (assembly, manipulate, surface)
simulate/mod.rs          Simulate screen
analyze/mod.rs           Analyze screen
widgets/                 shared widgets (read-only for screens; do not edit from a screen task)
units.rs                 unit systems, formatting, expression parsing
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
    pub units: UnitSystem,      // the display unit system (top bar select); format with app::ui::{with_unit, bare, readout}
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
  New/Open/Save/Save As (`.bowshock` JSON via `Rocket::to_json` / `from_json`) all live in `app/document.rs` and
  `app/files.rs`. `Document::revision()` increases on every change including live drag updates, undo and redo; derived data
  is stale when its revision differs.
* Configurations are part of the rocket: switching, adding, renaming, deleting a configuration are `ctx.doc.edit(..)` calls
  (see `chrome/mod.rs`, `chrome/config_picker.rs`), so they undo.

## Selection

`ctx.selection` is app-level and shared by the viewport, lanes and inspector. Set it directly
(`ctx.selection = Selection::Part(id)`); it is validated after every message. `Selection::Separation(n)` is the separation of
stage `n` from the stage ahead (`Stage::index`, from 2). While a part is being carried by a drag it is out of the rocket but
stays selected.

## Physics

`ctx.physics` is a cache, current when `view` runs: `balance()` (mass, length, ref diameter, CG, CP, margin, CNα for the phase
on show at liftoff, `None` for an empty rocket), `burnout()`, `issues()` (non-`Ok` warnings, each with an optional `part` to
select), `stages()`, `is_staged()`, `phase()`, `config_margin(id)`, `margin_with_motor(rocket, mount, motor)`,
`mounts_of_stage(rocket, stage)`, `motors()` (the bundled `MotorDb`), `gauge()` (animated margin gauge). Phase is a view
setting (`Phase::Full` or `Phase::UpperFrom(k)`), set from the status strip.

## Overlays and the viewport

The viewport pane is `viewport::View::view`; overlays (`assembly` for axial drags, `surface` for radial drags,
`manipulate` for selection/handles) are canvases that report to `viewport::Message` (`SnapNode`, `Surface`, `DropAxial`,
`DropRadial`, `Hover`, `Select`, `Grab`, `Resize`, `ResizeRadial`, `EditEnd`); `View::update` turns them into document edits
and drag calls. Camera messages (`Orbit`, `Pan`, `Zoom`, `Fit`, `Snap`, `Home`, `Roll`, `Tick`) are the same enum. The mesh
(`View::sync`) rebuilds from `ctx.doc.revision()`, hover, selection and the drag ghosts.

## Chrome (top bar, status strip, config menu, motor picker)

`chrome::top_bar(ctx)` and `chrome::status_strip(&chrome_state, ctx)` return `Element<app::Message>`; menus and the picker
are `chrome::State::overlay(ctx)`. The strip formats everything with `ctx.units`. Chrome-only state (open menu, rename
buffer, picker) is `chrome::State`; its messages are `chrome::Message`. The motor picker is
`chrome/config_picker.rs` (`Picker`, `Message`, `Outcome`).

## Tests and the drive harness

Logic is tested on the types (document, physics, drag via `App::update`, picker). UI is exercised headless:

```
cargo run -q -p bowshock -- drive [--example] <script> [out-dir]
```

`--example` loads the built-in example rocket (`assets/examples/hpr_l1_cert.bowshock`, kept in step with the model by
`app::files::tests::example_is_current`; regenerate with `BOWSHOCK_WRITE_EXAMPLE=1 cargo test -p bowshock example_is_current`).
Script syntax is in `drive.rs`; `scripts/shell.drive` exercises the chrome. A drive script cannot press shortcuts with
modifiers, so click the top-bar buttons or use `--example`.
