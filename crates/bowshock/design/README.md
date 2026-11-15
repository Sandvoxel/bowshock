# Bowshock

**An open-source rocket design and flight-analysis workbench. The UI kit for the builder.**

Bowshock's interface takes the *feel* of a KSP-style vehicle assembly building: drag parts off a palette, snap them onto a live 3D rocket, and watch CG and CP move as you build. Underneath it's an engineering tool where every shape is an exact number you can type. This kit covers what that interface looks like and how it behaves. It targets **iced** (Rust), so every token here maps to an iced `Theme`/`Palette` value or a custom widget style.

---

## 1. Principles

1. **The rocket is the brightest thing on screen.** The chrome is dark and quiet, and the viewport ground (`bg-000`) is the deepest surface. The airframe (`part-body`) carries the most luminance, so the eye lands on the vehicle, not the panels.
2. **Every gesture is a number.** Dragging a handle, a Station Lanes bar or a slider always shows the exact value it's writing, in `num-label` next to the cursor and live in the inspector. Nothing in the viewport is "approximately" placed.
3. **Stability is always visible.** CG, CP and the stability margin are never more than a glance away: on the model, in the status strip and in the gauge. They take the place of KSP's CoM/CoL markers.
4. **One colour, one meaning.** Shock orange (`accent`) means *what you're acting on*. Blue is CG, red is CP, everywhere. Amber is a warning and soft red is an error. No colour is ever decorative.
5. **Drawn like a technical drawing.** Square corners everywhere, 1 px frames and rules instead of cards and shadows, mono uppercase labels, corner registration marks and scale bars. Selection is shown with **corner brackets**, never with tinted washes. If something looks soft or bubbly, it's off-brand.
6. **Dense and calm, not cramped.** Controls are 26 px high (`control-h`), so a whole fin set fits in the inspector without scrolling. The density comes from tight vertical rhythm, never from shrinking text below 11 px.

## 2. Layout anatomy

```
┌──────────┬──────────────┬──────┬──────────────────────────────┬────────┬──────────┐ topbar-h 40
│ BOWSHOCK │ hpr_l1_cert ■│ ↶ ↷  │    [Build|Simulate|Analyze]  │ mm·g·s │ Simulate │
├──┬───────┴──────────┬───┴──────┴──────────────────────────────┬─┴────────┴──────────┤
│▣ │ AIRFRAME       6 │ ┌[3D|2D][Solid|Cut|X-ray|Expl] 1mm┐ ▣   │ Fin set  Trapezoid ▾│
│△ │ [search      /]  │                                     cube  │ 01 GEOMETRY         │
│▤ │ ☑ size to Ø76.2  │        CG ◐       ◉ CP        ⌜ ⌝         │ Fin count [2|3|4|5] │
│◠ │ ▭ Nose cone      │   <====|=============|===|==[▲]>           │ Root chord  120.00  │
│■ │ ▭ Body tube      │                               ⌞ ⌟         │ …                   │
│▷ │ ▭ Transition     │ └ CG CP M0.30 ┘  [hint bar]       100 mm ┘ │ 02 POSITION         │
│⌑ │ …                ├─ FROM TIP mm 0 ─ 200 ─ 400 ─ 600 ─ 800 ──┤ 03 MATERIAL         │
│  │                  │ STRUCTURE [Nose][Body tube    |  |]     │ 04 MASS             │
│  │                  │ AERO             ▯      ⌜Fins×3⌝           │                     │
│  │ SYMMETRY [1|2|3] │ PROPULSION             [|Motor mount|]     │                     │
│  │                  │ RECOVERY  - - [Chute] - -  │   ┊           │                     │
│  │ angle snap  15°  │ AVIONICS    ▯              │CG ┊CP         │                     │
├──┴──────────────────┴────────────────────────────────────────────┴─────────────────────┤
│ CONFIG H128W ▴│ MASS 1.284 │ LEN 1142.0 │ ◐ CG 712.4 │ ◉ CP 858.7 │ STAB ▮▮▮| 1.92 │ … ⚠ 1 │ statusbar-h 36
└──────────────────────────────────────────────────────────────────────────────────────┘
```

- **Left**: the **Parts bin**, a category rail (`rail-w`) plus a part list whose default sizes follow the selection, with the **Symmetry** picker (1/2/3/4/6/8, angle snap) pinned at its foot, KSP-style. There is no tile grid.
- **Centre top**: the **Viewport**. Its mode is 3D, Profile (2D side view) or Split. Overlays are the view toolbar (top left: 3D/2D · **Solid / Cutaway / X-ray / Exploded** · snap step), view cube + axis triad (top right), the CG/CP legend + flight condition (bottom left), the key-hint bar (bottom centre) and the scale bar (bottom right). Corner registration marks frame the drawing area.
- **Centre bottom**: **Station Lanes**, every part drawn as a bar along the rocket's length (measured from the nose tip), **one lane per system** (Structure · Aero · Propulsion · Recovery · Avionics), with CG/CP lines running through. This is Bowshock's structure view; **there is no component tree**. When the camera is side-on, the map shares the viewport's scale, so parts line up with the model above them.
- **Right**: the **Inspector** for the selection. Numbered sections in a fixed order: *01 Geometry → 02 Position → 03 Material & finish → 04 Mass → 05 Notes*.
- **Other modes**: **Simulate** (flights + conditions · event timeline + linked time plots · results · ground track; tick several flights to overlay them, see **SimulateCompare** and **GroundTrack**) and **Analyze** (analysis + method + conditions · margin, CP/CG and stacked Cd vs Mach · values at cursor) keep the same frame. See **SimulateScreen** and **AnalyzeScreen**.
- **Bottom**: the **Status strip**, the always-on vehicle summary in ruled cells. Its first cell is **Config**, the motor configuration being flown, placed in front of the numbers it changes. A staged rocket adds a **Phase** switch beside it. Clicking a readout jumps to the related analysis.
- Both side columns and the Station Lanes collapse (`Tab` toggles the columns, `L` the lanes) for a full-screen viewport, KSP-style.

## 3. Colour

The dark theme is the default; Light is for daylight work and printing reports. Colour tokens are listed in **Colors**, and here is how they're meant to be used:

| Role | Tokens | Rule |
|---|---|---|
| Surfaces | `bg-000` → `bg-400` | Viewport deepest, then window, panel, input, hover. Each step is one level of "closer to you". |
| Text | `ink`, `ink-muted`, `ink-faint` | Values use `ink`, labels use `ink-muted`, uppercase mono section heads use `ink-faint`. All pass 4.5:1 on `bg-300` and below. |
| Lines | `line-100`, `line-200` | Hairlines are decorative. Any control boundary uses `line-200` (3:1). |
| Selection | `accent`, `accent-ink` | Corner brackets and outlines on the selected part (viewport and Station Lanes), focus, primary button, the live value being dragged. `accent-soft` is only for drop targets. |
| Physics | `cg`, `cp` | Always paired with their glyphs (◐ quartered circle = CG, ◉ dot-in-ring = CP), so they're told apart by **shape and lightness**, not hue alone. |
| State | `ok`, `warn`, `err` | Always with an icon or a word. `err` is never used for CP and `cp` is never used for an error. |
| Viewport | `grid-*`, `part-*`, `axis-*` | Model shading bases and gizmo axes. `axis-z` is the rocket's long axis. |
| Systems | `part-body` (Structure), `sys-aero`, `sys-propulsion`, `sys-recovery`, `sys-avionics` | **One colour per system, shared by Station Lanes and the Cutaway / X-ray views.** A part is the same colour in both places. Lane bar fills use the matching `lane-*` tint. |
| Charts | `series-1…4` (aqua, yellow, violet, magenta) | Categorical slots in fixed order for comparisons and stacked breakdowns, validated for colour-blind separation in both themes. Single-series plots use `ink`. CG/CP plots keep `cg`/`cp`. More than 4 series → fold into Other or use small multiples. |
| Cutaway | `cut-face`, `cut-interior` | Hatching on sliced walls, rings and bulkheads, and the exposed inside of the airframe. |

## 4. Typography

- **IBM Plex Sans** for interface text, and **IBM Plex Mono** with tabular figures for **every number** and every label (section heads, status keys, Station Lanes row names, viewport annotations). Plex's engineering-drawing roots suit the tool better than a generic UI sans. Every number, whether editable or read-only. Digits must not jitter while you scrub a value, and columns of values should line up.
- Sizes stay within 11–20 px. The single large size (`num-readout`, 20 px) is reserved for the stability margin.
- Units are part of the value, set in `ink-muted` mono at 11 px directly after the number: `152.40 mm`. Clicking the unit cycles it (mm → cm → in).
- Precision follows the unit: mm with 2 decimals, in with 3, kg with 3, degrees with 1, calibers with 2.

## 5. Spacing & density

- A 4 px grid (`space-1` … `space-8`). Panel padding is `space-3`, rows inside a section are `space-2` apart, and sections are separated by a hairline plus `space-3`.
- Inspector field = label column (96 px) + input. Labels never wrap: they truncate with a tooltip.
- Parts bin rows are `part-row-h` (44 px), Station Lanes lanes are `lane-h` (28 px), growing by 22 px sub-rows when parts overlap, and both are ruled with hairlines rather than spaced with gaps.
- **Corners are square** (`radius-0`). `radius-1` exists only to soften 1 px anti-aliasing on tiny marks and must never read as a curve.

## 6. Logo & iconography

- **The mark** is a detached bow shock (`accent`, a hyperbola tending to the Mach 2.4 angle) standing ahead of a Von Kármán nose (`ink`), the same mark as in the top bar. The lockup is the mark + **BOWSHOCK** in Plex Sans 600. There's no colour-block motif anywhere: the mark *is* the brand. See **Logo** for construction, sizes, clear space and the app icon.

- Stroked outline icons, 1.5 px at 14 px size with square caps, with `currentColor` (in iced: SVG handles tinted via `svg::Style`). Keep them geometric, with straight segments and no rounded joins.
- **Part icons are silhouettes, not pictograms.** Each part shows its actual side profile (an ogive nose, a tube, a trapezoidal fin) filled with its `part-*` shading colour. That way the parts bin, the Station Lanes and the viewport all share one visual vocabulary.
- CG/CP glyphs are drawn, not icon-fonted: CG is a ring with two opposite filled quadrants, and CP is a ring with a centred dot.

## 7. The builder interaction model

KSP's assembly building, adapted for exact engineering:

| Gesture | Result | Visual |
|---|---|---|
| Hover a part in the viewport | Highlight + framed label with name and key dimension | 1 px `ink-muted` outline, and its Station Lanes bar highlights too |
| Click (viewport or Station Lanes) | Select. The inspector shows it | 1.5 px `accent` outline + `accent` **corner brackets** (leg `bracket`, 8 px) with a mono tag ("FIN SET ×3"), in both the viewport and the Station Lanes. No fill |
| Drag a row from the parts bin (into the viewport **or** onto the Station Lanes at an exact position) | A ghost part follows the cursor. Valid **attach nodes** light up | Nodes are `ok` rings at the fore/aft of bodies. Invalid spots show a red ⊘ cursor |
| Drop near a node | Snaps into the stack and auto-matches the diameter | Brief `accent-soft` pulse on the new part, a status message "Diameter matched to 76.2 mm" |
| Radial parts (fins, rail buttons, lugs) | Placed with the **Symmetry** count set in the Parts bin (KSP-style): drop one and get N evenly spaced, **linked** copies. Edit one and all change. Later you can unlink one value, detach one fin, or split all | Symmetry picker at the foot of the Parts bin. `X` / wheel changes N mid-drag. Siblings get dashed brackets, and the inspector shows `LINKED ×N` with Detach / Split all. See **Symmetry** |
| Drag the axial handle | Slides the part along its parent | `axis-z` arrow handle. A live offset label in `num-label` follows the cursor |
| Click a part, then drag its handles | Resizes it in place. **Every geometric dimension has a handle**: lengths, diameters, thicknesses, angles, positions and shape settings. The common ones show on select, the rest with `Alt` (thickness and angles around the axis in an aft-view inset, internal dimensions in Cutaway) | Square handles for lengths, round handles for angles and shapes, `accent` edge. The active one fills, a framed tag shows the live value and the change, and a dashed line marks the old size. Moves on the view's grid step; **hold `Shift` to snap** to other parts' faces, neighbouring tubes' diameters and stock sizes; `Ctrl` (`⌘`) for coarse steps; type digits for an exact value. See **Snapping**. See **DirectEdit** |
| Fin vertices (Profile mode) | Reshape the planform freely | Vertex dots, with the planform polygon drawn in `accent` |
| See inside | **Solid / Cutaway / X-ray / Exploded** in the view toolbar (`1`–`4`). Selecting an internal part auto-ghosts the shell in any mode | Cutaway hatches cut faces and shows internals in system colours. See **InsideViews** |
| Orbit / pan / zoom | Turntable around the rocket axis; MMB pans along the axis; wheel or **trackpad pinch** zooms to the cursor | View cube shows orientation, and clicking a face snaps to it. See **Navigation** for the full mouse and Mac trackpad table |
| **Double-click a part** | Focus: the camera turns square-on in an **orthographic** view and zooms until the part fills the viewport. `Esc` goes back | A `ORTHO · SIDE · FIN SET A` tag in the view toolbar with `Esc · back`. See **Navigation** |
| Type `{` in any value field | Pick mode: click another field, or a part in the viewport or lanes, to insert a live **reference** (`0.5 × [Body tube · Ø]`) | Targets get dashed `accent` outlines, and the reference appears as a chip. Committed formula fields show an `ƒ` badge. See **UnitInput** |
| Drag along x in any graph | A full-height band selects an x-range across every linked plot. **Enter** zooms in (y re-fits), and you can repeat to go deeper. **Esc** steps back one level, **Shift+Esc** returns to full | `accent-soft` band with `accent` edges, a breadcrumb of zoom levels, and an overview strip. No box select anywhere. See **PlotZoom** |
| Click the **Config** cell in the status strip | Switch between motor configurations of the same airframe, or add one through the motor picker, which only offers motors that fit the mount | The menu opens upward and lists each configuration with its stability margin. Station Lanes can show the other configurations' motors dashed and their CGs as ticks. See **ConfigPicker** |
| Add a second motor mount in line with the first | The rocket becomes two-stage. The separation is found automatically: the first joint at or aft of the upper motor's aft end. Timing is set in the separation's Inspector and stored per configuration | A thin `accent` seam with a diamond marker on the rocket and a dashed line through every lane, stage names either side, and a **Phase** switch in the status strip to view the full stack or an upper stage alone. If no joint exists there, a `warn` chip offers to split the tube. See **Staging** |
| Edit a field in the inspector | The viewport updates live and the gauge animates the new margin | The edited part flashes the `accent` outline once |

**Snapping and keys.** One rule across the app: a plain drag moves on the grid step, **holding `Shift` snaps to the values that matter** (faces of other parts, the diameters of neighbouring tubes, stock sizes, calibers, flight events in plots), and `Ctrl` gives coarse steps. On Mac, `Ctrl` is `⌘` and `Alt` is `⌥`. `Alt` alone means "show every handle". Dropping a new part always attaches to the nearest node with no key. See **Snapping**.

**States every part can be in**: default · hovered · selected · drag-ghost (60 % opacity) · snap-target · linked sibling (dashed brackets) · invalid (`err` outline) · auto-sized (dimension in `ink-muted` on a dashed field with an `AUTO` badge) · overridden mass (value in `warn`).

## 8. The viewport's visual language

- Ground is `bg-000` with a minor grid every 10 mm and a major grid every 100 mm, which fades out with distance. The rocket's centreline is drawn as a dashed `grid-major` line.
- Parts are shaded from their `part-*` base with simple lambert lighting from top left, with a thin `bg-000` 50 % edge line so stacked parts separate.
- CG/CP markers sit on the centreline with their value label above (`num-label`, in `cg`/`cp`). Between them, a bracket shows the margin in calibers: `ok` when ≥ 1.0, `warn` from 0.5 to 1.0, `err` below 0.5.
- Dimension lines (Profile mode) are 1 px `ink-muted` with mono labels.
- Overlays float `space-4` from the viewport edges, on `bg-200` with a 1 px `line-200` frame, square, with **no shadow**. Inside an overlay, groups are split by vertical rules, not gaps.
- Corner registration marks (20 px, `line-200`) sit in the viewport's four corners, and a scale bar sits bottom right. The viewport should read like a drawing sheet.

## 9. Implementing in iced

- **Theme**: build a `Theme::custom_with_fn("Bowshock Dark", palette, …)` from these tokens. `background` = `bg-100`, `text` = `ink`, `primary` = `accent`, `success` = `ok`, `danger` = `err` (and `warning` = `warn` on iced 0.13+). Anything the stock palette doesn't cover (`bg-300`, `line-200`, `cg`, `cp`, `part-*`) goes in your own `Tokens` struct, which the widget style functions read.
- **Fonts**: embed IBM Plex Sans and IBM Plex Mono (both OFL) with `iced::font::load` / `include_bytes!`, set Plex Sans as `default_font`, and use Plex Mono via a `Font` constant for numbers and labels.
- **Viewport**: the `shader` widget (a wgpu `Primitive`) for 3D, and `canvas` for Profile mode and all 2D overlays (CG/CP labels, dimension lines). Floating overlays are a `stack![]` on top of the viewport.
- **Custom widgets to build**: `UnitInput` (a text input + drag-scrub + unit cycling), `PartsBin` (rail + rows that start drags), `StationLanes` (a `canvas` with bars, ruler and CG/CP lines), `StabilityGauge` (canvas), `SegmentedControl`, `StatusStrip`, `ViewCube`. Every container style returns `border::Radius` 0.
- **Resizable regions**: `pane_grid` with bin | (viewport / Station Lanes) | inspector.

## 10. Voice

Plain engineering English. Write "Fin set", not "Fins!", and "Stability 1.92 cal", not "Looking stable 👍". Warnings say what's wrong and by how much: "Stability 0.74 cal, below 1.0 minimum". Never use emoji in the UI.
