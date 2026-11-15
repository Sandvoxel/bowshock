# Navigation

Moving around the rocket, and the gestures for each input device.

**Focus a part (double-click)**
- **Double-click any part** in the viewport (or its bar in Station Lanes) to focus it. The camera animates (250 ms) to an **orthographic** view square-on to the part and zooms until the part fills the viewport with a margin of `space-8`.
- The view direction is the one that shows the part's main dimensions: **side-on** for tubes, nose cones, transitions and fins; **aft-on** for rings, bulkheads and anything defined around the axis.
- The part is also selected, so its handles are ready (see **DirectEdit**). In ortho the handles move in true scale, which makes exact edits easier.
- The view toolbar shows the focus as a tag (`ORTHO · SIDE · FIN SET A`) with `Esc · back`.
- **Esc**, or pinching out past the fit, returns the camera to where it was before. Double-clicking another part moves the focus without stacking.
- Double-clicking empty space (or `F`) fits the whole rocket in the current projection.
- Internal parts get focus in Cutaway automatically, so they aren't hidden behind the shell.

**Controls**

| Action | Mouse | Mac trackpad |
|---|---|---|
| Orbit (3D) | Right-drag | Two-finger scroll |
| Pan | Middle-drag | `Shift` + two-finger scroll |
| Zoom to cursor | Wheel | **Pinch** |
| Roll around the rocket axis | `Alt` + right-drag | Two-finger rotate |
| Focus a part | Double-click it | Double-click it |
| Back from focus | `Esc` | `Esc`, or pinch out past the fit |
| Fit the whole rocket | `F`, or double-click empty space | Two-finger double-tap (smart zoom) |
| Station Lanes, plots, ground track | Wheel zooms, middle-drag pans | Pinch zooms, two-finger scroll pans |

- Pinch in a plot zooms the x-axis around the cursor and adds a level to the zoom stack when your fingers lift, so `Esc` steps back the same way as after a band zoom (see **PlotZoom**).
- **On Mac**, every `Ctrl` shortcut in this kit is `⌘` and `Alt` is `⌥`. `Ctrl`-click is not used for anything, since macOS treats it as a right-click.
- Trackpad scrolling uses the pixel deltas the OS sends, so it is smooth, not stepped like a wheel.

**iced:** pinch and rotate arrive from winit as `PinchGesture` and `RotationGesture` on macOS. As far as I know iced doesn't pass these through to widgets yet, so expect to forward them from the window event loop (or a small patch). Two-finger scroll arrives as `mouse::ScrollDelta::Pixels`, which is how trackpad and wheel are told apart. The focus animation interpolates a `Camera { target, distance, orientation, ortho_blend }` and blends the projection matrix from perspective to orthographic.
