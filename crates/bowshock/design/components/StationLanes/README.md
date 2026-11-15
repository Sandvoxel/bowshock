# StationLanes

Where everything sits in the rocket. Every part is a bar **along the rocket's length**, at its real position measured from the nose tip ("station" is the aerospace word for that distance), with **one lane per system**. You can read position, length and overlap at a glance, and CG/CP lines run straight through. It replaces a component tree.

**Quiet at rest.** Extra information appears only while you're doing something that needs it, and the panel never changes height.

**Lanes are preset, one per system, in a fixed order.** A part's lane is decided by what it does, so every part has exactly one obvious home:

| Lane | Colour | What goes in it |
|---|---|---|
| **Structure** | `part-body` | Nose cone, body tubes, transitions, couplers, bulkheads, centering rings |
| **Aero** | `sys-aero` | Fins, canards, rail buttons, launch lugs |
| **Propulsion** | `sys-propulsion` | Motor mount tube, motor, engine block, thrust ring, retainer |
| **Recovery** | `sys-recovery` | Parachutes, drogue, shock cord, harness, deployment charges |
| **Avionics & payload** | `sys-avionics` | Altimeters, batteries, trackers, payload, ballast |

The same colour marks a system everywhere: the lane's dot, its bars' edges, and that system's parts in **Cutaway** and **X-ray** (see InsideViews). Turn on cutaway and the magenta motor in 3D is the magenta bar in Propulsion.

**Lanes manage themselves**
- **Sub-rows appear automatically** when parts in one lane overlap (here the motor sits inside the mount, so Propulsion is two rows high), and disappear when they stop overlapping.
- **Empty lanes hide.** A minimum-diameter rocket with no avionics just doesn't show that lane.
- **No user-defined lanes.** That would turn the view into organising work. User *groups* (e.g. "av-bay") may come later as an overlay.

**Balance**
- **CG and CP are dotted lines** through every lane (`cg` blue, `cp` red), so they read as markers and not as parts. Their stations are labelled in the footer under the lanes.
- **Margin bar.** A 3 px bar in the footer spans CG to CP, with the stability margin in calibers beneath it. It is `ok` at 1 cal or more, `warn` from 0.5 to 1, and `err` below 0.5 or when CP is ahead of CG.
- **Mass shading** is hidden at rest. While a part is being dragged, while the cursor is on the CG label, or when pinned with `M`, a faint bar chart of mass per 50 mm draws behind the ruler ticks, so you can see which lump is pulling the CG. It reshapes live during a drag.

**Adding parts**
1. **Drag from the Parts bin onto any lane.** A dashed `accent` ghost bar follows the cursor, an `accent` tag on the ruler shows the exact station (`905.0`), and holding `Shift` snaps it to neighbouring parts' faces (see Snapping in the brand book). A part dropped on the wrong lane goes to its own lane and the ghost jumps there, so you can't misfile anything.
2. **Right-click empty space in a lane → Add here…** opens a menu of only that system's parts, with the station prefilled.
3. **Anything added in the 3D view appears here automatically.** It's one model with two views.

**Editing**
- Click a bar = select (the same selection as the viewport and inspector), shown with `accent` corner brackets, never a fill.
- **Drag a bar** sideways to move the part. **Drag an end** to change its length. CG/CP and the margin bar move live.
- **The drag tag gives two distances**: from the tip, and from the nearest face of the part it sits in or snaps to (`400.0 from tip · 171.4 from tube front`). Fins read `from aft`.
- **Dragging an internal part shows its container** as two dashed walls at the ends of the tube it lives in. The drag stops at those walls.
- **Very short parts** (rings, bulkheads, rail buttons, charges) are drawn as small blocks at least 6 px wide, wider than true scale so they can be seen and grabbed. Hovering one names it and gives its station. Symmetry groups are one bar with a link glyph (`Fins ×3`).
- Overlap that's physical (a chute inside a tube) is normal. Overlap that isn't (two rings at one station) gets a `warn` dot.
- Wheel = zoom along the length, MMB = pan. When the 3D camera is side-on, the lanes lock to the viewport's scale, so parts sit directly under the model. `L` collapses everything to the 24 px ruler.

**Configurations**
- The motor bar in Propulsion is the motor of the current configuration (see **ConfigPicker**). Clicking it opens the configuration menu.
- With **Compare all in lanes** on, the other configurations' motors are dashed outlines in the same sub-row, and each one's CG is a small `cg` tick in the footer. Hovering a tick names its configuration. The margin bar always belongs to the current configuration.

**Stages**
- A rocket with motor mounts in line is staged, and the split is found from the mounts (see **Staging**). A dashed separation line runs through every lane at that joint, and the ruler names the stage on each side. The line can be selected but not dragged. Stages are numbered in firing order, so the booster is Stage 1.
- The ruler keeps running from the tip through the whole stack. Each stage has its own motor in Propulsion.
- The **Phase** switch in the status strip chooses what flies: the full stack, or an upper stage alone. Parts that have dropped away fade to 30 % and CG, CP and the margin bar recompute for what's left.
- Dragging a part across the separation line moves it to the other stage. Moving a motor mount or the joint moves the line.

**iced:** a `canvas::Program`. Each component's `(start, end, system)` gives a bar. Sub-rows come from greedy interval packing per lane, hit-testing is done on the CPU, and drags emit `MoveAxial(id, Δ)` / `Resize(id, end, Δ)`.
