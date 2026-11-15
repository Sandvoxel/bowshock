# StabilityGauge

The static margin in calibers, read at a glance. It appears in the status strip (compact) and at the top of the Analyze view (full).

- Headline value in `num-readout` (20 px mono), unit `cal`, plus the same margin in length units on the right.
- A ruled scale from 0 to 3 cal (ticks every 0.1, longer every 1.0) over a thin band strip: `err` below 0.5, `warn` from 0.5 to 1.0, `ok` above 1.0. These thresholds are user-configurable in project settings, and the bands follow them.
- The marker is a 2 px `ink` needle. While a value is being dragged anywhere in the app, the pointer **animates live** (spring, about 120 ms) so you can feel the effect of each change.
- The state word (Stable / Marginal / Unstable) is always present, so colour is never the only signal.
- Over-stable warning (above 3 cal, weathercocking risk): the pointer pins to the right end with a `warn` "Over-stable" label.
- Margin is evaluated at the selected condition (default Mach 0.3, AoA 0°). The condition is shown in a tooltip.

**iced:** a `canvas::Program` with an animated `f32` driven by a `time::every` subscription only while it's moving.
