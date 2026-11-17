#!/bin/bash
# the Finner at Mach 1.832 (G-F5), zero incidence, sector, GPU: Coarse tier fins on / off / Euler, and the Lift-tier half domain through the gate
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
run() { env "$@" cargo test --release -p bowshock-core --test arcas_study -- --ignored --nocapture 2>&1 | grep -E "^STUDY|panicked|Error|error" ; }
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=on
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=off
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=on STUDY_TIER=euler
run STUDY_BODY=anf STUDY_MACH=1.832 STUDY_FINS=on STUDY_TIER=fine
BOWSHOCK_FIN_TIER=lift BOWSHOCK_GATE_MACHS=1.832 cargo test --release -p bowshock-core --test fin_lift_gates -- --ignored --nocapture gate_f5_anf_cd0 2>&1 | grep -E "^GATE|^INFO|panicked"
