# Bowshock UI kit: read me first

This folder is the UI design system for **Bowshock**, an open-source rocket design and simulation app (an OpenRocket / RASAero alternative) built in Rust with **iced**.

## What's here
| Path | What it is |
|---|---|
| `README.md` | The brand book: principles, layout, colour roles, type, spacing, logo, the builder interaction model, viewport language, iced notes, voice |
| `tokens/tokens.json` | Source of truth for every token (colours in dark and light, type, spacing, radius, sizes), with usage notes |
| `tokens/tokens.css` | The same as CSS custom properties. `data-theme="light"` on the root switches theme; dark is the default |
| `tokens/bowshock.dark.tokens.json`, `bowshock.light.tokens.json` | W3C Design Tokens (DTCG) format, one file per theme, for Tokens Studio, Style Dictionary and similar |
| `tokens/theme.rs` | The tokens as Rust for iced: `Palette::DARK`, `Palette::LIGHT`, `size::*`, `text::*` |
| `components/<Name>/README.md` | The spec for one component or screen: anatomy, states, behaviour, iced notes |
| `components/<Name>/preview.html` | A visual reference, as an HTML fragment that needs `tokens/tokens.css` and `components/bundle.css` |
| `components/<Name>/<Name>-dark.png`, `-light.png` | Rendered previews |
| `components/bundle.css` | Shared classes used by the previews |
| `assets/icons/` | Parts-bin rail icons: SVG masters (`currentColor`) and PNGs at 24, 48 and 96 px |
| `gallery.html` | Every component on one self-contained page with a theme toggle. Open it in a browser |

## How to use it
1. Read `README.md` first.
2. Take values from the tokens, never from the previews. In Rust, use `tokens/theme.rs`.
3. Before building or changing a widget, read its `components/<Name>/README.md`; use the PNGs or `gallery.html` to see it.
4. `BuilderScreen`, `StagingScreen`, `SimulateScreen`, `SimulateCompare` and `AnalyzeScreen` are the full reference screens: match their composition and density.

## Hard rules
- Square corners everywhere (`radius-0`). No drop shadows, no gradients in the UI chrome.
- `accent` (shock orange) means "what you are acting on": selection brackets, focus, the primary button, live drag values, handles. Nothing else.
- CG is `cg` with the quartered-circle glyph and CP is `cp` with the dot-in-ring glyph, everywhere. CG and CP lines are dotted.
- System colours (`part-body`, `sys-aero`, `sys-propulsion`, `sys-recovery`, `sys-avionics`) are shared by Station Lanes and the Cutaway / X-ray views.
- Every number uses the mono font with tabular figures. Values are stored in SI and formatted only at the view.
- No component tree: structure is shown by Station Lanes (one preset lane per system), which stay quiet at rest and show extra information only during a drag.
- Symmetry is picked in the Parts bin footer (KSP-style). Linked groups can be unlinked per value, detached or split.
- Every geometric dimension has a direct-edit handle on the model: squares for lengths, circles for angles and shapes. Diamonds mark separations.
- Snapping: a plain drag moves on the grid step, **`Shift` snaps** to meaningful targets (part faces, neighbouring tube diameters, stock sizes, calibers), `Ctrl` is coarse. On Mac `Ctrl` is `⌘`. Double-click a part to focus it in an orthographic view; trackpad pinch zooms.
- A **configuration** is the motor (and timing) choice for one airframe. It is switched from the first cell of the status strip.
- Stages are never placed by hand: the separation is derived from in-line motor mounts.
- Charts: one y-axis per plot, never dual-axis. Compared series use `series-1…4` in fixed order and the colour follows the flight. Text is never in a series colour.
- The logo is the detached bow shock + Von Kármán nose in `components/Logo/README.md`. Never use colour blocks.

## Components and screens
- `components/AnalyzeScreen/`: Analyze: aero vs Mach (margin, CP/CG, stacked Cd), method + conditions, values at cursor
- `components/BuilderScreen/`: Build mode in Cutaway: fin A of a linked set selected, height mid-drag; lane colours match the cutaway
- `components/Button/`: Primary, secondary, ghost, icon; hover, focus, disabled
- `components/ConfigPicker/`: One airframe, several motor configurations; switched from the first cell of the status strip
- `components/Cover/`: 
- `components/DirectEdit/`: Click a part on the rocket, then drag its handles to resize it in place
- `components/GroundTrack/`: Top-down view of where each flight goes and lands, for drift and recovery distance
- `components/InsideViews/`: Solid, Cutaway, X-ray, Exploded; colours match the Station Lanes
- `components/Inspector/`: Selection properties in fixed sections
- `components/Logo/`: The mark: a detached bow shock ahead of a Von Kármán nose, Mach 2.4
- `components/Navigation/`: Double-click a part to focus it in an orthographic view; mouse and Mac trackpad controls
- `components/PartsBin/`: Category rail + sized part list; replaces a tile grid
- `components/PlotZoom/`: Graph zoom: drag an x-range band, Enter to zoom in, repeat; Esc steps back out
- `components/SegmentedControl/`: Mutually exclusive modes: view mode, fin count, cross-section
- `components/SimulateCompare/`: Simulate with three flights overlaid: one colour per flight across every plot, the ground track and the results
- `components/SimulateScreen/`: Simulate: flights + conditions, events timeline, linked x-zoom plots, results
- `components/Snapping/`: Hold Shift during any drag to snap to the values that matter: neighbouring parts, stock sizes, calibers
- `components/StabilityGauge/`: Margin in calibers on a ruled scale; three states
- `components/Staging/`: Stages are found from the motor mounts; the separation's inspector sets the timing
- `components/StagingScreen/`: Build mode with a two-stage rocket: separation line selected, Config menu open, Phase switch in the status strip
- `components/StationLanes/`: Parts laid out along the rocket, one lane per system; replaces a component tree
- `components/StatusStrip/`: Always-on vehicle summary, cell-ruled
- `components/Symmetry/`: KSP-style: pick symmetry in the Parts bin, place once, split later
- `components/UnitInput/`: Exact values, scrub to drag, unit cycling, and expressions that reference other fields with {
- `components/ViewportOverlay/`: Dragging a part in: attach nodes, ghost, snap feedback
- `components/WarningChip/`: Health states with icon + words
