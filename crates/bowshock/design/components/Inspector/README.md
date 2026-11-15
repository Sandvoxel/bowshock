# Inspector

The right-hand column that edits whatever is selected. It's the exact, typed counterpart to dragging in the viewport.

- Width `right-panel-w` (304 px, resizable 260–440). `bg-200`.
- **Header** (44 px): part silhouette, part name (`ui-title`, double-click to rename), a **sub-type select** (Trapezoidal / Elliptical / Freeform, which swaps the geometry fields), and an overflow menu.
- **Symmetry bar** (linked radial parts only), under the header: a `LINKED ×N` tag plus *Detach* / *Split all*. Fields look normal and edit the whole group. Right-click a field → *Unlink for this fin* to make one value per-fin, which adds a small `warn` broken-link icon to that field only. See Symmetry.
- **Sections in a fixed order**, each a numbered uppercase mono head (`01 GEOMETRY`) followed by fields `space-2` apart, separated by a hairline:
  1. **Geometry**: the part's own shape.
  2. **Position**: *Relative to* (Top / Middle / Bottom of parent, or After previous) + offset, then cant and rotation where they apply.
  3. **Material & finish**: material from the database (shown with its density) and surface finish (shown with its roughness).
  4. **Mass**: the computed value, with an *Override…* action that turns it into a `UnitInput` in the override state.
  5. **Notes** (collapsed by default).
- Derived values (sweep angle from sweep length and chord) show as `auto` fields. Typing into one flips which of the pair is driven.
- Inline health chips at the end (`ok` / `warn` / `err`) for checks specific to this part, such as fin flutter or a chute packing in its tube.
- **Multi-select** shows only the fields the selected parts share. Mixed values display as `—` and typing sets all of them.
- Nothing is selected → the inspector shows **Rocket**: name, designer, reference diameter and length, and stage list.

**iced:** `scrollable(column![header, section(...), ...])`. A section is a small helper `fn section(title, Vec<Element>) -> Element`.
