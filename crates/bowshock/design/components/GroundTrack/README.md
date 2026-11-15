# GroundTrack

The flight seen from above: where the rocket goes and where it comes down. It answers "how far will it drift, and will it land in the field?", which the altitude-against-time plots can't.

**What's drawn**
- **North up, equal scale** in both directions, so distances and angles read true. Pad at the centre as a small cross labelled `PAD`.
- **Range rings** every 100 m (50 m when zoomed in), labelled on the south side, plus a scale bar bottom right and a north arrow top right.
- **Wind** top left: speed and the direction it blows from, with an arrow pointing the way it blows.
- **Field radius**, set in Conditions, as a dashed `warn` circle labelled `FIELD 300 m`.
- **One track per flight**, 2 px. Each track shows its **apogee** as a hollow ring and its **landing** as a filled square, both with a 2 px surface ring so overlapping marks stay separate. The landing gets a label with distance and bearing from the pad (`337 m · 067°`) in `ink-muted`.
- The ascent usually leans **into** the wind (weathercocking) and the descent drifts downwind, so a track normally hooks back on itself: that's expected, not a bug.

**Colour**
- A single flight draws in `ink`, with apogee in `accent`.
- Compared flights use `series-1…4` in the same order and the same colours as the time plots (see **SimulateCompare**). Colour follows the flight, never its position in the list.
- Text is never in a series colour. The legend row above the map names each colour.

**Interaction**
- **Linked to the time plots.** Hovering any time plot puts a dot on each track at that moment. Hovering a track does the reverse: it moves the time crosshair and shows a tooltip with time, altitude and distance from pad.
- Wheel zooms around the cursor, drag pans, double-click fits all tracks. Rings switch to 50 m or 500 m spacing as you zoom.
- Clicking a landing marker selects that flight.
- A landing outside the field radius gives that flight a `warn` status square in the Flights list and a chip in its results.

**Dispersion**
- **Run dispersion** (in the Run button's menu) flies the selected flight many times, varying wind speed and direction, launch angle and motor impulse within their tolerances.
- Each landing is a small dot in the flight's colour at 45 % opacity, with a dashed ellipse containing 95 % of them. The caption gives the ellipse size and how many land outside the field.

**Where it appears**
- **SimulateScreen** right panel, section 04, compact (280 px wide, rings every 50 m).
- **SimulateCompare** right panel, with every compared flight.
- Full size as a plot in the centre column, from the plot picker.

**iced:** a `canvas::Program` with its own pan and zoom transform, independent of the time-axis zoom stack. Tracks are polylines decimated to about 2 px per segment. Dispersion runs on a worker thread and streams dots in as runs finish.
