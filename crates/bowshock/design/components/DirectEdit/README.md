# DirectEdit

Editing the rocket by grabbing it. **Every dimension that has a place on the part gets a handle**, so nearly everything you can type in the Inspector you can also drag on the model. Drag a handle and the part resizes in place while the matching Inspector field updates with it. There's no separate mode; handles belong to the selection.

**The rule:** if a field in the Inspector is a length, diameter, thickness, angle or position, it has a handle. Fields that aren't geometry (material, finish, motor choice, mass override) don't, and stay in the Inspector.

**The loop**
1. **Click a part** to select it. It gets the `accent` outline and corner brackets, and its **primary handles** appear.
2. **Hold `Alt`** (or switch the viewport to 2D Profile) to show **every** handle on that part. Hovering any handle names its dimension and value.
3. **Drag a handle.** It fills with `accent`, an arrow shows its direction, and a framed tag gives the live value and the change (`+58.0`). The matching Inspector field shows its dragging state.
4. **Release** to commit, or press `Esc` to put it back.

**Handle shapes**
- **Square**: a length, diameter, thickness or position. Moves in a straight line.
- **Circle**: an angle or a shape setting. Moves along an arc or a curve.
- **Diamond** is not a handle: it marks a separation (see **Staging**).

**Handles by part** (primary handles in bold, the rest appear with `Alt`)

| Part | Handles |
|---|---|
| Nose cone | **Length** (tip), **base diameter** (base edge, follows the tube unless unlinked), shoulder length and shoulder diameter, wall thickness (in Cutaway), shape parameter (circle on the curve: slides through the shape family, e.g. Haack C or ogive ratio) |
| Body tube | **Length** (fore and aft faces), **outer diameter** (top edge), wall thickness (in Cutaway) |
| Transition | **Fore and aft diameter**, **length**, shoulder lengths, wall thickness, shape parameter |
| Tube coupler, payload bay | **Length**, position along the airframe, wall thickness |
| Fin set | **Height**, **root chord**, **tip chord**, **sweep**, **position** (drag the fin), fillet radius (circle at the root), and in the aft-view inset: thickness, cant angle (circle), angle of the first fin |
| Freeform fin | Every vertex is a square handle; double-click an edge to add one, `Del` to remove |
| Rail button, launch lug | **Position along** the body, angle around it (circle), length (lug) |
| Centering ring, bulkhead | **Position**, **thickness**, inner diameter (in Cutaway) |
| Motor mount | **Length**, **position**, overhang past the aft end, diameter |
| Parachute, shock cord | **Packed position** and packed length (in Cutaway) |
| Strap-on body | **Position along** the core, gap to the core, angle of the first body (in the aft view) |
| Ballast, avionics | **Position** |

**Seeing what you're editing**
- **Thickness and angles around the axis** can't be seen side-on, so with `Alt` held a small **aft-view inset** appears beside the part with its own handles.
- **Internal dimensions** (wall, shoulder, ring inner diameter) show when the viewport is in **Cutaway**, or when the part is selected through the shell with `Alt`-click.
- Handles closer than 16 px to each other hide all but the primary ones, and a hint asks you to zoom in. A part smaller than 24 px on screen shows no handles.

**Rules**
- **One handle, one dimension.** A handle only moves along its own direction, so a drag can't change two things at once.
- **Snapping.** A plain drag moves in the grid step from the view toolbar (1 mm, 1°). **Hold `Shift` to snap to meaningful targets**: faces of other parts, the diameters of neighbouring tubes, stock sizes, caliber multiples. `Ctrl` (`⌘` on Mac) gives a coarse step (5 mm or 15°). See **Snapping**.
- **Exact values while dragging.** Typing digits during a drag replaces the tag with a field. `Enter` commits that exact number. `{` inserts a reference, as in any field (see **UnitInput**).
- **Old size stays visible.** A dashed `ink-muted` line marks where the edge was until you release.
- **Linked parts follow.** Dragging a handle on one fin of a linked set reshapes all of them (see **Symmetry**). A value unlinked on that fin changes on that fin only.
- **Dependent parts follow.** Lengthening a tube moves everything aft of it; changing a tube's diameter resizes parts sized to it (nose base, couplers, fin root curve).
- **Limits stop the drag.** A handle stops where the part would become invalid (zero length, a wall thicker than the radius, a ring bigger than its tube), and the tag turns `warn` with the reason.
- **Double-click a handle** to jump to its field in the Inspector with the value selected.
- **Live feedback.** CG, CP and the margin in Station Lanes and the status strip update during the drag.

**Drawing**
- Handles are 8 px squares or 9 px circles, `bg-000` fill with a 1.5 px `accent` edge. The active one is filled `accent`.
- Handle labels (with `Alt`) are mono, uppercase, `ink-muted`.
- The value tag is the framed tag used elsewhere in the viewport: title in `ink-muted`, value in `accent`, both mono.
- In 3D, handles face the screen and keep a constant pixel size at any zoom.

**iced:** each part type exposes `fn handles(&self, view: ViewMode) -> Vec<Handle { dim, kind: Linear(axis) | Angular(center) | Curve(param), primary: bool }>`. Handles are drawn in the viewport's overlay pass and hit-tested on the CPU before parts, in screen space with a 12 px hit box. A drag emits `Resize { id, dim, value }`, the same message the Inspector field sends, so there's one edit path.
