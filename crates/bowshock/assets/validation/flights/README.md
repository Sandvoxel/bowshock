# Flight-data validation rockets (DESIGN3D item 56)

Six public rockets with a measured apogee, full geometry and a catalogue motor, flown by `flight_validation` (`crates/bowshock/src/analyze/cfd/tests/flights.rs`).

## What is in here

* `<id>.json`: one rocket. Lengths in inches, weights in pounds, altitudes in feet, temperature in degrees F, wind in mph, as the RASAero II files give them. `source` names the
  page and the example file the numbers were read from; `measured` is the flight's altitude above ground level with its instrument; `rasaero.table_ft` is the RASAero II
  prediction in the published comparison table and `rasaero.file_ft` the one stored in the example file (the two differ for some flights: the file is a later release).
* `tables/<id>.3d.json`, `tables/<id>.legacy.json`: the `CfdTable` the app's own `solve` made for the rocket (the 3D engine, `BOWSHOCK_ENGINE=3d`, and the legacy engine), written by
  `FLIGHTVAL_SOLVE=1`. Our own output.

## Sources and licence notes

* Measured apogees and the RASAero II predictions: Charles E. Rogers, *RASAero II Comparisons with Altitude Data*, <https://www.rasaero.com/comparisons-alt.htm>. Altitudes are facts
  cited with their source; the page is not copied. The same apogees are in the *Rocket Flight Database* (A. Yu, v1.2, <https://doi.org/10.5281/zenodo.19976138>, CC BY 4.0,
  <https://github.com/AidanSYu/rocket-flight-database>), which has no geometry and which we used to cross-check the table.
* Geometry, launch weight, centre of gravity, nozzle, launch site: the example `.CDX1` files of the free RASAero II download (<https://www.rasaero.com/dl_software_ii.htm>,
  `RASAero_II_Setup_Version_1.0.2.0.zip`, `Examples`). The files are not redistributed; the dimensions are transcribed (a fact, not a creative work) into our own format. The
  fin position follows the RASAero II manual (Fig. 38: the fin can is the last part of the body tube; the fin location is measured from the base).
* Rocket designs and flights belong to their builders (Corky Borgman, Steve Sawyer, Ray Kinsel, Mike Pasaretti, ...); only their published numbers are used.
* Motors: the app's bundled thrust-curve catalogue (ThrustCurve.org data, credit and terms as in `bowshock-motors`).

## What the harness does with the files

* The tables' `other` column (the build-up's rail buttons and lugs, the part the solver does not take) is re-derived from the fixture whenever a table is loaded (`refresh_other` in `flights.rs`): the column does not depend on the solve, so a stored table follows a fixture that gained a rail guide without a new solve. Byrum, Ion Drive and Rabia had no rail guide in the fixtures although the example files give one (0.375 in diameter, 0.3438 / 0.3438 / 0.5 in high; Rabia's is on the fin can); the tables stored for them were solved without it.
* The rail guide is one guide of the file, flown as two buttons (RASAero II's convention: "the software then calculates the rail guide drag based on two of the rail guides"), at 55 % and 85 % of the length.
* `nozzle_diameter_in` (the nozzle exit diameter of the example file's flight input) is not an input of the flight: the app's motor files carry no nozzle, so the flight takes the exit from the thrust (`ThrustCurve::nozzle_exit_diameter`, 0.5-0.8 of the case) for the base drag while the motor burns. RASAero II's own values are 0 (Byrum, Cancer Descending, Ion Drive), 2.488 in (Don't Debate This, 64 % of its 98 mm case), 1.187 in (Rabia, 56 % of 54 mm) and 1.625 in (Torrent, 55 % of 75 mm).
* The last columns of the printed table read the geometric apogee as a barometric altimeter would (all six measured altitudes are barometric): the altimeter converts pressure to height with the standard atmosphere, so on a hot day (the fixtures' 80-100 F against the standard 9-11 C at these sites) it reads 5-9 % below the height of the rocket (`indicated_above_pad`, the hydrostatic column of the site's own temperature profile).

## Where the fixtures differ from the example files

* Cancer Descending: the fin is 3 in further forward (`te_to_base` 5.05 against the file's 2.0) and Don't Debate This 0.87 in (1.12 against 0.25): the 3D mesher refuses a fin tip that reaches behind the root trailing edge by more than half the root's distance to the base (DESIGN3D items 56, 73).
* Torrent: the file's 7 in fin can of 4.125 in is flown as the 4 in tube (a 1.25 slope step the 3D mesher refuses under a fin root).
* Ion Drive: the file measures the fin from the base of the body tube, which is the start of the 1.5 in boattail (its `Location` 12.25 less the 9.5 in chord = 2.75 in); the fixture measures from the base of the boattail, so its fin stands 1.5 in further aft than the file's. Not corrected: the drag build-up and the table do not depend on the fin station on a cylinder.
* Don't Debate This: the file's fin leading-edge radius (0.025 in) is not in the fixture.
* The motors are the same as RASAero II's motor file (`J570W`, `M1297W`, `N5800-CS`, `K550W`, `L730`, `M1850GG`): same total impulse within 0.2 % and same burn time, except Torrent's, where the fixture's file (the thrust curve of ThrustCurve.org simfile 5f4294d20002e9000000010a) burns 3.30 s against 3.20 s, has the same propellant mass (3.375 kg) and a lighter case (4.5 kg total against 5.851 kg), which does not change the flight (the dry mass follows from the launch weight less the motor, so the burnout mass is the launch weight less the propellant either way).

## Surface finish per rocket (`roughness_in`)

The example files' `Surface` setting (RASAero II's equivalent sand roughness table: Smooth (Zero Roughness) 0, Sheet Metal 0.00016 in, Smooth Paint 0.00025 in, ...): Byrum, Cancer Descending, Ion Drive, Rabia, Torrent: `Smooth (Zero Roughness)` (0 kept); Don't Debate This: `Smooth Paint` (with `Turbulence` True), which the app maps to 20 um (`Finish::SmoothPaint`, RASAero II's own value is 6.35 um). `FLIGHTVAL_ROUGH_UM=<um>` gives every rocket one roughness and `FLIGHTVAL_TAG=<tag>` names the tables `<id>.<engine>-<tag>.json`, to solve and fly a variant beside the stored tables.

