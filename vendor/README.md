# vendor/

Patched copies of two crates from [iced](https://github.com/iced-rs/iced) at the rev the app pins
(`b2fbd3cb6289603f8cbbb2387a5242231f509d13`). They replace the git crates of the same name through the `[patch]` section of the
root `Cargo.toml`; everything else in iced is still the pinned git checkout. `vendor/` is `exclude`d from the workspace, so
`--workspace` builds, tests and clippy do not touch it.

## Why: trackpad pinch

winit 0.30 reports a Mac trackpad pinch as `WindowEvent::PinchGesture { delta, phase, .. }`. iced's `iced_winit::conversion`
turns only the window events it knows into `iced::Event`s and returns `None` for the rest, so widgets never see a pinch
(the kit's Navigation spec calls this out). iced has no hook for raw winit events, and `iced::Event` has no variant for it, so
the smallest working change is one new `mouse::Event` variant in `iced_core` and one conversion arm in `iced_winit`.

## The diff from upstream

`iced_core` (`vendor/iced_core`):

- `src/mouse/event.rs`: `mouse::Event::PinchGesture { delta: f32, phase: GesturePhase }` and the `GesturePhase`
  (`Started` / `Moved` / `Ended`) enum. `delta` is winit's magnification change since the last event (positive as the fingers
  spread).
- `src/mouse.rs`: exports `GesturePhase` (reachable from the app as `iced::advanced::mouse::GesturePhase`; the app re-exports
  it as `widgets::pinch::Phase`).

`iced_winit` (`vendor/iced_winit`):

- `src/conversion.rs`: `WindowEvent::PinchGesture` becomes `Event::Mouse(mouse::Event::PinchGesture { .. })` (`Cancelled` maps
  to `Ended`).

Both crates' `Cargo.toml` are the upstream ones with the workspace inheritance (`version.workspace = true`, `foo.workspace =
true`, `[lints] workspace = true`) resolved by hand, since they no longer sit in iced's workspace; `iced_winit` points at the
same iced rev and winit fork rev as upstream for its other iced / winit dependencies. No other file differs.

No other iced crate matches exhaustively on `mouse::Event`, so nothing else needed changing (it built clean).

## How the app uses it

`crates/bowshock/src/widgets/pinch.rs` maps the delta to a zoom factor (`exp(delta)`, clamped per event). The viewport
(`viewport/scene.rs`), Station Lanes (`build/lanes/canvas.rs`), the plots (`widgets/plot/view.rs`, where a pinch is one level on
the zoom stack that closes when the fingers lift) and the ground track (`widgets/ground_track.rs`) zoom about the cursor with
it. Two-finger scroll is unchanged (`ScrollDelta::Pixels`: orbit / pan).

## Updating iced

When the pinned rev moves, re-copy `core/` and `winit/` from the new rev, redo the three edits above, and bump the revs in
`vendor/iced_winit/Cargo.toml`. If upstream gains pinch support, delete this directory and the `[patch]` / `exclude` lines in the
root `Cargo.toml`, and read the event from upstream's variant in `widgets/pinch.rs` and the four call sites.
