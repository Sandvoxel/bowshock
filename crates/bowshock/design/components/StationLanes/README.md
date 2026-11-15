# StationLanes

Where everything sits in the rocket. Every part is a bar **along the rocket's length**, at its real position measured from the nose tip ("station" is the aerospace word for that distance), with **one lane per system**. You can read position, length and overlap at a glance, and CG/CP lines run straight through. It replaces a component tree.

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

**Adding parts**
1. **Drag from the Parts bin onto any lane.** A dashed `accent` ghost bar follows the cursor, an `accent` tag on the ruler shows the exact station (`905.0`), and it snaps to neighbouring parts' faces (`Shift` = free). A part dropped on the wrong lane goes to its own lane and the ghost jumps there, so you can't misfile anything.
2. **Right-click empty space in a lane → Add here…** opens a menu of only that system's parts, with the station prefilled.
3. **Anything added in the 3D view appears here automatically.** It's one model with two views.

**Editing**
- Click a bar = select (the same selection as the viewport and inspector), shown with `accent` corner brackets, never a fill.
- **Drag a bar** sideways to move the part. **Drag an end** to change its length. CG/CP move live.
- Very short parts (rings, rail buttons, charges) are slivers with a tooltip. Symmetry groups are one bar with a link glyph (`Fins ×3`).
- Overlap that's physical (a chute inside a tube) is normal. Overlap that isn't (two rings at one station) gets a `warn` dot.
- Wheel = zoom along the length, MMB = pan. When the 3D camera is side-on, the lanes lock to the viewport's scale, so parts sit directly under the model. `L` collapses everything to the 24 px ruler.

**iced:** a `canvas::Program`. Each component's `(start, end, system)` gives a bar. Sub-rows come from greedy interval packing per lane, hit-testing is done on the CPU, and drags emit `MoveAxial(id, Δ)` / `Resize(id, end, Δ)`.
