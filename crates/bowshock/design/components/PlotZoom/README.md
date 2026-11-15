# PlotZoom

How every graph in Bowshock zooms. There's no box select: you **drag a range along x**, the selection is a **full-height band** (infinitely tall, so y never matters), **Enter** zooms into it, and you can repeat that as deep as you like. **Esc** steps back out one level at a time.

**Selecting**
- **Click-drag anywhere in a plot** draws an x-range band: `accent-soft` fill with 1.5 px `accent` edges, spanning the full height of the plot and **every plot linked to the same x-axis** (altitude, velocity, acceleration and so on stack and share time).
- While dragging, the header shows the range and width in mono `accent`: `2.10 – 4.80 s · Δ 2.70 s`.
- After release the band stays. **Drag either edge handle** to adjust it, or **drag inside the band** to slide it. A click outside the band (without dragging) clears it.
- Holding `Shift` snaps the band's edges to flight events (burnout, apogee, deployment) within 6 px, the same `Shift`-to-snap rule as everywhere else. On a trackpad, pinch zooms x around the cursor and adds a level to the zoom stack when you lift your fingers.

**Zooming: a stack**
- **Enter** zooms x to the band, and **y re-fits to the data in view** (with 8 % headroom) on each plot. The band clears so you can drag again right away.
- **Repeat** drag + Enter to go deeper. Each zoom pushes a level onto the stack.
- **Esc** pops one level (back to the previous range). **Shift+Esc** returns to the full range. Esc with a band showing clears the band first, then pops on the next press.
- The **breadcrumb** in the header shows the stack (`0 – 24 s › 2.10 – 4.80 s › 3.00 – 3.40 s`). Click any crumb to jump straight to that level. The current level has an `accent` frame.
- When zoomed, a 26 px **overview strip** above the plots shows the whole flight with the current window framed in `accent` and the rest dimmed. Dragging the window frame pans, and clicking elsewhere on the strip recentres the window.

**Other input**
- **Hover** = crosshair + snapped point + a framed mono readout (`228.9 m @ 2.60 s`), synced across linked plots.
- **Wheel** zooms x around the cursor without adding stack levels (it modifies the current level). **MMB-drag** or `Space`+drag pans x.
- **Double-click** = Shift+Esc (back to full).
- Zoom state is per view, so the Simulate tab and Analyze tab keep their own stacks.

**Look**
- Single series per plot, drawn as a 2 px `ink` line. Never two y-axes: a second quantity gets its own plot below, sharing x.
- Recessive grid (`line-100`), mono tick labels in `ink-faint`, the zero line in `line-200`, and flight events as dotted verticals with uppercase mono tags.
- The series title sits top-left inside the plot (`ALTITUDE · m`), so no legend box is needed for a single series.

**iced:** a `canvas::Program` per plot, with the app holding one `XView { stack: Vec<Range<f64>>, band: Option<Range<f64>> }` per linked group. Plots read the top of the stack and re-fit y from their own data each frame. Key handling (`Enter`, `Esc`) goes on the focused plot group.
