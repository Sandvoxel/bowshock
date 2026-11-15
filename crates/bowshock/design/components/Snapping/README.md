# Snapping

One rule everywhere: **hold `Shift` while dragging to snap to the values that matter.** A plain drag moves in the grid step set in the view toolbar, and `Ctrl` (`⌘` on Mac) moves in coarse steps. This applies to direct-edit handles (see **DirectEdit**), bars in Station Lanes, scrubbing a field, the cutaway plane, symmetry angles and the band in a plot.

**What you see while `Shift` is held**
- **Candidate ticks** along the handle's path, one per target in range, each labelled faintly with what it is (`Ø98.0 · booster tube`, `STOCK 24 in · 609.6`).
- When the handle comes within **8 px** of a target it catches: the tick turns `ok` and thicker, a dotted `ok` guide runs to the thing it matched, and the drag tag gains a second line with an `ok` square naming the match (`= payload tube, fore`, `flush with mount aft`).
- Release sets the value. It is a plain value, not a link: to keep two parts equal later, type `{` in the field and pick the other part (see **UnitInput**).
- With no target nearby the drag falls back to the grid step, so `Shift` never makes a drag feel stuck.

**Targets, in priority order** (the nearest wins; ties go to the higher one)

| | Lengths and positions | Diameters | Angles and thickness |
|---|---|---|---|
| **1. Other parts** | Faces of every part along the axis: tube ends, couplers, motor mount ends, motor aft end, ring faces, fin root ends, the separation joint | **Adjacent tubes fore and aft** (outer diameter); for parts inside a tube, its **inner diameter**; for rings, the motor mount's outer diameter | Sibling sets' angles, 0° cant |
| **2. Matching** | Same length as a sibling part of the same kind | Same as any other tube in the rocket | Evenly spaced positions for the symmetry count |
| **3. Stock** | Stock lengths of the chosen tube material (from the parts library) | Stock tube sizes; motor diameters (24, 29, 38, 54, 75, 98 mm) for mounts | Stock sheet thickness for fins (1.6, 3.2, 4.8 mm) |
| **4. Calibers** | Nose length at 3, 4, 5 calibers; fin span at 1, 1.5, 2 calibers | | |
| **5. Grid** | Round numbers on the coarse step | | 15° |

**Placing new parts is different.** Dropping a part from the Parts bin always attaches to the nearest attach node (see **ViewportOverlay**); that needs no key.

**iced:** each drag asks the model for `fn snap_targets(&self, part, dim) -> Vec<Target { value, label, source: Option<PartId>, priority }>` once at drag start. While `Shift` is held the handle's value is replaced by the nearest target within 8 screen px, converted through the current zoom. The overlay draws the ticks and guide from the same list.
