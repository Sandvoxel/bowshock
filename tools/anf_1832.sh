#!/bin/bash
# the Finner at Mach 1.832 (G-F5), zero incidence, sector, GPU: Coarse tier fins on / off / Euler,
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
run() { env "$@" cargo test --release -p bowshock-core --test arcas_study -- --ignored --nocapture 2>&1 | grep -E "^STUDY|panicked|Error|error" ; }
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=on
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=off
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=on STUDY_TIER=euler
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=on STUDY_TIER=fine
