# SimulateScreen

Simulate mode: define flights, run them, and read the results. Same frame as Build (top bar, status strip) with three columns.

**Left: Flights + Conditions**
- **Flights list**: one row per simulation case (name, motor · key condition, apogee in mono, a status square in `ok`/`warn`/`err`). The selected flight has `bg-300` with an `accent` inside edge. **Compare** checkboxes overlay up to 4 flights in the plots (see **SimulateCompare**), using the `series-1…4` colours in fixed order.
- **Conditions** for the selected flight: motor, ejection delay, rail length, launch angle, wind speed and direction (the direction it blows from) + turbulence, site altitude, temperature, and **field radius**, the distance from the pad the rocket has to land within. All are `UnitInput`s, so `{` references work here too (e.g. delay = optimum delay − 0.5).
- **Run all** (the primary button; `F5` and the top bar's *Simulate* do the same) runs every flight and **Run selected** runs the selected flights. The top bar's Simulate caret menu has *Run selected flights* and *One flight per configuration*. Results update in place, and a stale result shows its values in `ink-faint` until re-run.

**Centre: plots**
- A header with the **PlotZoom** breadcrumb and key hints. **Events timeline** across the top: ignition → rail exit → burnout → apogee → drogue → main → landing, with flight-phase bars in system colours (propulsion while burning, recovery under canopy). Apogee is highlighted in `accent`.
- **Linked plots** sharing time: altitude, vertical velocity, acceleration (one series each, `ink`, never dual-axis). Drag x → band → `Enter` to zoom in, `Esc` to step back. Hover gives a synced crosshair with values on every plot.
- A plot picker (not shown) adds more linked plots: Mach, drag force, angle of attack, stability vs time, thrust.
- **Dropped stages.** On a staged rocket each separated stage (the booster of a two-stage rocket, `Stage 1`, `Stage 2`... otherwise) is flown on its own after the separation, from where and how fast the rocket was then, with its own mass, aerodynamics and recovery devices, until it lands. The events timeline gets a **second thin row per dropped stage** under the main one, named at the left (`BOOSTER`), a bar from the separation to its landing and a tick for each of its own events (a parachute deploying, the landing). On the plots each dropped stage is drawn from the separation in its flight's colour, **dashed**, and named at its end (`Booster`); several flights compared keep each stage in its flight's colour. A **Dropped stages** checkbox in the plots header (shown only when a flight on show has one, on by default) hides those lines and rows.

**Right: results**
- **Headline**: apogee in `num-readout`, then time to apogee and the **optimum delay**, with a `warn` chip when the chosen delay is off.
- **Ascent**: max velocity, max Mach, max accel, rail exit velocity (`ok` chip if ≥ 15 m/s), stability off the rail (`ok` if ≥ 1 cal).
- **Recovery**: descent rates, ground-hit speed (checked against the limit), flight time.
- **One section per dropped stage** after the recovery, titled with its name (`04 BOOSTER`): the separation time, its max velocity, its descent rate (the canopy's, or the vertical speed of a stage with no device), the ground-hit speed with the same limit chip, its flight time, and **Lands** with distance, bearing and the `in field` / `outside field` chip. The ground track section is numbered after them.
- **Ground track**: the flight seen from above (see **GroundTrack**), north up, pad at the centre, with the hover crosshair's position shown as a dot.
- Every number is clickable: it moves the plot crosshair to the time it happened.
