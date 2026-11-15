# OpenRocket reference harness

`Harness.java` is a single-file Java program that runs OpenRocket's own code headless and writes what Bowshock is compared
against (see `docs/openrocket-comparison.md`).

```sh
tools/openrocket-harness/run.sh <file.ork> <outdir>
# = java -Djava.awt.headless=true -cp OpenRocket.jar Harness.java <file.ork> <outdir>
```

The jar is the one inside `/Applications/OpenRocket.app` (`OPENROCKET_JAR` overrides it; Java 17+). It writes
`<outdir>/<name>.json` and `<outdir>/motors/*.eng`.

JSON contents, per flight configuration:

* `launch_mass_kg`, `launch_cg_x`, `burnout_*`, `structure_*`: `MassCalculator`, CG from the nose tip.
* `aero`: `BarrowmanCalculator` at alpha = 0 and 2 deg, Mach 0.1, 0.15 .. 2.0 (ISA sea level): `cp_x`, `cna`, `cn`, `cd`, `cd_axial`
  (what the flight simulation uses off axis), `cd_friction`/`cd_pressure`/`cd_base`.
* `aero_components`: the same per component at a few Mach numbers.
* `motors`: designation, mount, ignition, delay, and `eng_file`, the thrust curve exported as RASP `.eng` from OpenRocket's
  database (so Bowshock flies the identical curve; the file name carries the database digest because two database entries
  can share a designation).

`simulations`: every stored simulation run four ways, `stored` (the file's conditions), `wind` (the stored mean wind with the random gusts
switched off: a deterministic reference for flights with wind), `calm` (no wind or turbulence) and
`calm_fine` (calm, 2 ms step: OpenRocket's 50 ms step overshoots the rail end, which inflates its rail-exit speed by about 2 %).
Each has apogee, max velocity/Mach/acceleration, time to apogee, rail-exit velocity, stability off the rail, ground-hit velocity,
descent rate, flight time, the peak angle of attack in the 2 s after the rail (`peak_aoa_2s_after_rail_rad`), the landing distance from the pad,
events and a decimated time series. `components` lists every component with its effective mass
(`getMass`, overrides included), CG, absolute position and radii, for checking the importer.

`-Dopenrocket.*` is not needed; `OPENROCKET_JAVA_OPTS` is passed to `java` by `run.sh`.
