# Symmetry

KSP-style symmetry groups for radial parts (fins, rail buttons, lugs, pods). You place one and get N copies that **share their settings**. The link can be loosened one field at a time, or broken for one or all instances later.

**Placing**
- **Symmetry lives in the Parts bin**, pinned at its foot like KSP's symmetry button: pick 1 / 2 / 3 / 4 / 6 / 8 before you drag. While dragging, `X` / `Shift+X` or the wheel changes N live. Release creates a **linked group**. Angle snap (off / 15° / 45°) sits next to it.
- Copies are spaced evenly (360°/N) starting from the drop angle. Holding `Shift` while dropping snaps the first instance to the angle-snap step set in the Parts bin footer (15° by default) or to a sibling set's angle.

**Three states** (see the preview)
1. **Linked**: one set of settings drives every instance. Selecting any one selects the group: the clicked instance gets solid `accent` brackets and its siblings dashed ones. The inspector shows a mono `LINKED ×N` tag with *Detach* and *Split all*. Fields look normal: editing one edits the group.
2. **Per-value override**: right-click a field → *Unlink for this fin* to make **just that value** per-fin (e.g. one canted fin for a roll test). A small broken-link `warn` icon appears after that field only, the value shows in the override style, and the tag reads `LINKED ×3 · 1 OVR`. *Relink field* puts it back to the group value.
3. **Detached / split**:
   - **Detach** (on one instance): it leaves the group as a free part and keeps its current values. The rest stay linked as ×(N−1).
   - **Split all**: every instance becomes free, and each keeps the group's values at that moment.
   - **Rejoin group**: select a free part plus a group (or drop it onto the group's bar in the Station Lanes), then choose whose settings win.

**Station Lanes**: a linked group is one bar with a link glyph (`Fins ×3`). Overrides are listed in mono on the bar (`B: cant`). Detached instances get their own bar, stacked in the Aero lane under their former group.

**Rules**
- Angular position is never shared: each instance keeps its own angle, and the group only stores the count and start angle.
- Changing N on a linked group re-spaces it. Changing N with overrides present asks first, because the overrides would be lost.
- Split, detach and rejoin are single undo steps.

**iced / model note:** store a `SymmetryGroup { members: Vec<ComponentId>, shared: FinParams, overrides: HashMap<ComponentId, FieldMask> }`. Detach removes a member and copies `shared` into a free component. The UI only reads `effective_params(id)`.
