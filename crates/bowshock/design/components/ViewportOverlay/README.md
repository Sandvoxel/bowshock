# ViewportOverlay: drag & snap

What the viewport shows while a part is being dragged in from the palette or moved. This is the KSP "attach node" idea adapted to an axisymmetric stack.

- **Attach nodes** are rings on the centreline at the fore and aft faces of each body component. The node that one of the dragged part's *ends* is nearest (inside the node's catch disc: at least 20 px, growing with the body's projected radius; see *Held by its ends* below) becomes the **snap target**: a 7 px `ok` ring with a soft `ok` halo. Other nodes stay faint (50 %).
- **Ghost part**: the dragged part at 60 % opacity with a dashed `accent` outline, already sized to its default length and **auto-matched** diameter.
- A dashed `ok` leader joins the ghost to its snap target.
- **Cursor label** (a framed, square tag): what will happen on release ("Body tube → snap aft"), with dimension feedback beneath the ghost in `num-label`.
- **Invalid** (no node in range, or the part can't go there): ⊘ in `err` with a reason.
- Radial parts (fins, rail buttons, lugs) don't use nodes. They snap to the **surface** of the tube under the cursor and immediately show all N symmetric copies as one linked group (count from the Symmetry picker in the Parts bin, and `X` / wheel changes it mid-drag). See Symmetry.
- Release animates the part into place over 120 ms, then gives it a one-time `accent` outline flash.

## Held by its ends (placing)

- A carried part is **held** by one point: where it was grabbed if it was picked up off the rocket, its middle if it came from the parts bin. The hold point is under the cursor, the part's axis parallel to the rocket's with its nose end towards the rocket's nose. That *free pose* is where the part's fore and aft faces are in the air, and the one place both the ghost and the snapping take the part's position from, so what is drawn is what snaps.
- A node catches the part when one of the part's **ends** projects inside the node's catch disc, never the cursor: a joint between two parts takes the nearer of the part's fore and aft ends, the node in front of the first part takes only the part's **aft** end, the node behind the last part only its **fore** end. An empty rocket's one node takes the part anywhere. Nodes of a pod's stack work the same along the pod's axis. Failing a node, the tube side under the cursor takes the part (a pod); failing that, the node beside the rocket that an end is nearest along the axis (the hold point within 48 px of the axis, the end not more than 48 px past the rocket's ends).
- While the part is free, a small hollow ring (`ink-muted`) marks each of its ends; the end that is caught fills with `accent`.
- **Every carried part has a ghost in the air** while it is over the viewport and no target has it: an axial part (as big as it will be when it snaps), the `count` copies of a radial group round an axis at the radius of the selected body (else the widest), an internal part sized for the selected body's bore (else the widest). It is not part of the rocket: not listed, not pickable, outside the framing.
- **Click to place** (Preferences > Editor): the part stays in hand after you let go and follows the cursor with no button held; a left press puts it down on its target (a press with no target does nothing), Esc puts it back. The hint line reads `Click to place · Esc to put back`. Right-drag orbit and the wheel keep working; clicks on the toolbar, the cube and the menus do not drop it.
- **The parts bin deletes.** A part picked up off the rocket (axial, radial or internal) that is released over the bin (or clicked there, clicking to place) is deleted with everything it carries, as Delete deletes it: one undo step, `Delete part`. The bin shows `Drop to delete` over the `err` soft ground while it is. A part that came from the bin is put back (`Drop to put back`, no undo step).
