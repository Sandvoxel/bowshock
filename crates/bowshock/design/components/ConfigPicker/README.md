# ConfigPicker

One rocket, several ways to fly it. A **configuration** is a named choice of what goes in the motor mounts and how the flight is timed. The airframe is shared; switching configuration never changes a part.

**What a configuration holds**
- One motor per motor mount (so one per stage on a staged rocket), with its delay.
- Staging timing: when each stage separates and when the next motor ignites (see **Staging**).
- Recovery timing: deployment events and delays.
- Ballast added to rebalance for that motor.

Everything else (tubes, fins, motor mounts, and so where the rocket separates) belongs to the rocket and is the same in every configuration. Inspector sections that are stored per configuration carry a small `PER CONFIG` badge.

**Where it lives**
- The **first cell of the status strip**, `CONFIG H128W ▴`. It sits in front of the numbers it changes, so the strip reads as one line: for this configuration, here are the mass, CG, CP and margin.
- The cell is present in Build, Simulate and Analyze, and is the only place the current configuration is switched.
- On a staged rocket the name lists motors in firing order, `I284W → H128W`, and a **Phase** switch appears beside the cell (see **Staging**).
- Shortcut: clicking the motor bar in Station Lanes opens the same menu.

**The menu** opens upward from the cell, as wide as the Parts bin.
- One row per configuration: name on the left, its stability margin on the right in `ok`, `warn` or `err`, so an unstable configuration is visible before you pick it. The current one is highlighted.
- **New configuration…** (`Ctrl L`) opens the motor picker. **Duplicate current** copies timing and ballast so only the motor needs changing. **Rename…** (`F2`); names default to the motor designations.
- **Compare all in lanes** toggles the dashed motors and CG ticks in Station Lanes.
- Deleting is on the row's right-click menu. The last configuration can't be deleted.

**The motor picker**
- The header states the constraint it filters by: mount diameter and maximum length, read from the selected motor mount.
- Search by designation or maker; the impulse-class segmented control narrows by class and allows several at once.
- Columns: motor, total impulse, length, mass, and **With this motor**, the stability margin the rocket would have, computed before you commit.
- Motors that don't fit stay listed in `ink-faint` with the reason in the last column, so you can see what a longer mount would allow.
- On a staged rocket the picker has one tab per stage.
- **Add configuration** is the single primary button. The new configuration becomes current.

**In other modes**
- **Simulate:** the Simulate button runs every flight, and *One flight per configuration* in its menu makes a flight for each configuration.
- **Analyze:** with more than one configuration, plots draw one series per configuration using the chart series colours, labelled by name.

**iced:** `Rocket { parts, configs: Vec<Config>, current: ConfigId }`, with `Config { name, motors: BTreeMap<MountId, MotorChoice>, events, ballast }`. The margin column is a cheap recompute of CG only, since CP doesn't depend on the motor. The menu is an `overlay` anchored to the cell's top edge.
