# SimulateCompare

Simulate with several flights overlaid. Same layout as **SimulateScreen**; ticking more than one flight switches every plot and the results panel into compare.

**Turning it on**
- Each row in the Flights list has a checkbox. Tick up to **four**. A ticked box fills with that flight's series colour, which is the flight's colour everywhere on the screen.
- Colours are given in tick order from `series-1…4` (aqua, yellow, violet, magenta) and **stay with the flight**: unticking one never repaints the others. A fifth tick is refused with a tooltip.
- The highlighted row is the **reference** (`REF` badge). Its events draw on the plots and the timeline, and the results panel shows the others as differences from it. Click another row to change the reference.

**Plots**
- Every linked plot draws one line per ticked flight, 2 px, on **one shared axis**: flights are only compared when the quantity and units match, never on a second y-axis.
- A **legend row** under the header names each colour. The altitude plot also labels each line at its right end, in a margin kept clear for labels on every linked plot so their time axes stay aligned. Labels and values are in text colours, never in the series colour.
- Event lines and the events timeline belong to the reference only, so the plots don't fill with three sets of dashed lines.
- **Hover** gives one crosshair across all plots with a dot on each line and a tooltip listing every flight's value at that time, in legend order. Hidden lines are left out of the tooltip.
- **x-zoom** (see **PlotZoom**) applies to all flights at once.
- Clicking a line or its legend entry makes that flight the reference.

**Results panel**
- Becomes **Compare**: for each key result (apogee, max velocity, landing), one row per flight with its colour square, value, and the difference from the reference (`+273`, `−34`; the reference reads `ref`).
- **Landing** gives distance and bearing from the pad, with an `ok` or `warn` chip for inside or outside the field radius.
- **Ground track** shows every compared flight from above, in the same colours, with the crosshair's moment as a dot on each track (see **GroundTrack**).

**Comparing configurations**
- **Run all configurations** in the Simulate button's menu makes one flight per motor configuration and ticks them, which is the quickest way to compare motors (see **ConfigPicker**).

**Accessibility**
- Identity is never colour alone: the legend, the apogee labels and the tooltip all name the flight.
- The results panel is the table view of the comparison. Every number in it moves the crosshair to when it happened.
