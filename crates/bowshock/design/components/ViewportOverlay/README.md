# ViewportOverlay: drag & snap

What the viewport shows while a part is being dragged in from the palette or moved. This is the KSP "attach node" idea adapted to an axisymmetric stack.

- **Attach nodes** are rings on the centreline at the fore and aft faces of each body component. The node nearest the cursor (within 40 px) becomes the **snap target**: a 7 px `ok` ring with a soft `ok` halo. Other nodes stay faint (50 %).
- **Ghost part**: the dragged part at 60 % opacity with a dashed `accent` outline, already sized to its default length and **auto-matched** diameter.
- A dashed `ok` leader joins the ghost to its snap target.
- **Cursor label** (a framed, square tag): what will happen on release ("Body tube → snap aft"), with dimension feedback beneath the ghost in `num-label`.
- **Invalid** (no node in range, or the part can't go there): ⊘ in `err` with a reason.
- Radial parts (fins, rail buttons, lugs) don't use nodes. They snap to the **surface** of the tube under the cursor and immediately show all N symmetric copies as one linked group (count from the Symmetry picker in the Parts bin, and `X` / wheel changes it mid-drag). See Symmetry.
- Release animates the part into place over 120 ms, then gives it a one-time `accent` outline flash.
