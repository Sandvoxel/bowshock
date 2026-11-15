# AnalyzeScreen

Analyze mode: the vehicle's aerodynamics across the speed range. This is where Bowshock's RASAero-style aero and GPU-solved CFD tables live.

**Left: what and how**
- **Analyses** list: Aerodynamics (vs Mach), Stability (vs angle of attack), Drag breakdown, Fin flutter, Dispersion (Monte Carlo). The selected one has `bg-300` with an `accent` inside edge.
- **Method**: Barrowman · Extended (RASAero-style) · **CFD table**, with a provenance line ("42 Mach × 9 AoA points · solved on GPU (Luma) · 6 min ago"), **Re-solve**, and **Compare** to overlay another method or an imported OpenRocket / RASAero result.
- **Conditions**: Mach range, angle of attack, altitude (sets Reynolds number), which CG the margin uses (liftoff / burnout), surface finish.

**Centre: linked plots vs Mach** (shared x, so PlotZoom works across all three; the transonic region is the usual place to drag in on)
1. **Static margin (cal)**: single `ink` line with dashed threshold lines: `ok` at 1.0 (minimum) and `warn` at 3.0 (over-stable).
2. **CP and CG (mm from tip)**: CP in `cp`, CG at liftoff solid `cg`, CG at burnout dashed `cg`, all direct-labelled. It's one quantity in one unit, so a single axis.
3. **Drag coefficient, stacked by source**: skin friction, pressure, base, fins as a stacked area in `series-1…4` (fixed order, validated for colour-blind separation), with the total as an `ink` line on top. Labelled in place, and the right panel repeats the values, which covers the low-contrast light-theme slots.

**Right: values at the cursor**: everything at the hovered Mach (`M 1.10` in `accent`), the stability and drag breakdowns with colour keys, the worst cases over the range, and export (table view, CSV, RASAero `.CDX1`).

**Rules carried over**: one y-axis per plot, single series in `ink` unless the plot is a comparison, categorical colours never cycled (more than 4 series → fold into Other or small multiples), and a table view is always one click away.
