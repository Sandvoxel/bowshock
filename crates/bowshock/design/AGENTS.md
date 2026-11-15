# Bowshock UI kit — read me first (for agents)

This folder is the design system for **Bowshock**, an open-source rocket design app (OpenRocket / RASAero alternative) built in Rust with **iced**.

## How to use it
1. Read `README.md` first: principles, layout, colour roles, type, interaction model, iced implementation notes.
2. Use `tokens.json` as the single source of truth for colours (dark + light themes), type, spacing, radius and sizes. `tokens.css` is the same thing as CSS variables.
3. Before building or changing a widget, read its spec in `components/<Name>/README.md`. `preview.html` next to it is a visual reference: open it in a browser alongside `tokens.css` and `components/bundle.css`.
4. `components/BuilderScreen/` is the full reference screen: match its composition and density.

## Hard rules
- Square corners everywhere (`radius-0`). No drop shadows, no gradients in the UI chrome.
- `accent` (shock orange) means "what you are acting on": selection brackets, focus, primary button, live drag value. Nothing else.
- CG is `cg` with the quartered-circle glyph and CP is `cp` with the dot-in-ring glyph, everywhere.
- System colours (`part-body`, `sys-aero`, `sys-propulsion`, `sys-recovery`, `sys-avionics`) are shared by Station Lanes and the Cutaway / X-ray views.
- Every number uses the mono font with tabular figures. Values are stored in SI and formatted only at the view.
- No component tree: structure is shown by Station Lanes (one lane per system).
- Symmetry is picked in the Parts bin footer (KSP-style). Linked groups can be unlinked per value, detached or split.
- The logo is the detached bow shock + Von Kármán nose described in `components/Logo/README.md`. Never use colour blocks.

## Components
- `components/BuilderScreen/`
- `components/Button/`
- `components/Cover/`
- `components/InsideViews/`
- `components/Inspector/`
- `components/Logo/`
- `components/PartsBin/`
- `components/SegmentedControl/`
- `components/StabilityGauge/`
- `components/StationLanes/`
- `components/StatusStrip/`
- `components/Symmetry/`
- `components/UnitInput/`
- `components/ViewportOverlay/`
- `components/WarningChip/`
