# PartsBin

Where parts come from. A narrow **category rail** (icons only, like a vehicle-assembly parts bin) next to a **part list** whose rows are already sized for what you have selected.

- **Rail** (`rail-w`, 44 px): Airframe · Fins & guides · Internal · Recovery · Mass · Motors · Presets. The active category gets a 2 px `accent` inside edge. Hovering shows the name, and the keys `1`–`7` switch category.
- **Rows** (`part-row-h`, 44 px): the true side silhouette, the name, and the **default size in mono** (`Ø76.2 × 600.0`). Rows are ruled with hairlines, with no cards, tiles or gaps.
- **Size to selection** (on by default): defaults follow the selected part's diameter, so a tube dragged onto a 76.2 mm rocket is already 76.2 mm. This replaces the "fix the diameter after you add it" step.
- **Hover** `bg-400`. **Dragging**: an `accent-soft` row with an `accent` inside edge while the ghost goes into the viewport. **Unavailable**: 45 % opacity, with the reason in a tooltip ("Payload bay needs a coupler above it").
- **Every row is drag-and-drop.** Drag it into the viewport to snap onto an attach node, or onto Station Lanes to drop it at an exact station (the ruler shows it live). Radial parts arrive as a linked symmetry group.
- Click = attach aft of the selection (or at the best node). Double-click = attach and focus the first field in the inspector.
- **Symmetry footer** (pinned at the bottom, KSP-style): the count for radial parts (1 / 2 / 3 / 4 / 6 / 8, with `X` cycling it, also mid-drag) and angle snap (off / 15° / 45°). It applies to the next radial part you place. See Symmetry.
- `/` focuses search. Search covers every category and the material and preset libraries.
- **Presets** holds user-saved assemblies (a whole fin can, a nose with its shoulder and bulkhead) and parts imported from vendor catalogues.

**iced:** a `row![rail, list]` inside the left `pane_grid` pane. Each row is a `mouse_area` that starts a drag on press-and-move (more than 4 px) and adds on click.
