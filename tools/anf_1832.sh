#!/bin/bash
# the Finner (G-F5), zero incidence, 4-fin sector, GPU: Coarse tier fins on / off at the four Mach numbers of the table, Euler fins on, and Fine at Mach 1.832
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
run() { env "$@" cargo test --release -p bowshock-core --test arcas_study -- --ignored --nocapture 2>&1 | grep -E "^STUDY|panicked|Error|error" ; }
for m in 1.832 2.375 2.718 3.147; do
  run STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=on
  run STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=off
  run STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=on STUDY_TIER=euler
done
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=on STUDY_TIER=fine
