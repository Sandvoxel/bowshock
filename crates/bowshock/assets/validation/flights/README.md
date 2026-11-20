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
