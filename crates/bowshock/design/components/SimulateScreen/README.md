# SimulateScreen

Simulate mode: define flights, run them, and read the results. Same frame as Build (top bar, status strip) with three columns.

**Left: Flights + Conditions**
- **Flights list**: one row per simulation case (name, motor · key condition, apogee in mono, a status square in `ok`/`warn`/`err`). The selected flight has `bg-300` with an `accent` inside edge. **Compare** checkboxes overlay up to 4 flights in the plots (see **SimulateCompare**), using the `series-1…4` colours in fixed order.
- **Conditions** for the selected flight: motor, ejection delay, rail length, launch angle, wind speed and direction (the direction it blows from) + turbulence, site altitude, temperature, and **field radius**, the distance from the pad the rocket has to land within. All are `UnitInput`s, so `{` references work here too (e.g. delay = optimum delay − 0.5).
- **Run** (`F5`) runs the selected flight and **Run all** runs every flight. Results update in place, and a stale result shows its values in `ink-faint` until re-run.

**Centre: plots**
- A header with the **PlotZoom** breadcrumb and key hints. **Events timeline** across the top: ignition → rail exit → burnout → apogee → drogue → main → landing, with flight-phase bars in system colours (propulsion while burning, recovery under canopy). Apogee is highlighted in `accent`.
- **Linked plots** sharing time: altitude, vertical velocity, acceleration (one series each, `ink`, never dual-axis). Drag x → band → `Enter` to zoom in, `Esc` to step back. Hover gives a synced crosshair with values on every plot.
- A plot picker (not shown) adds more linked plots: Mach, drag force, angle of attack, stability vs time, thrust.

**Right: results**
- **Headline**: apogee in `num-readout`, then time to apogee and the **optimum delay**, with a `warn` chip when the chosen delay is off.
- **Ascent**: max velocity, max Mach, max accel, rail exit velocity (`ok` chip if ≥ 15 m/s), stability off the rail (`ok` if ≥ 1 cal).
- **Recovery**: descent rates, ground-hit speed (checked against the limit), flight time.
- **Ground track**: the flight seen from above (see **GroundTrack**), north up, pad at the centre, with the hover crosshair's position shown as a dot.
- Every number is clickable: it moves the plot crosshair to the time it happened.
