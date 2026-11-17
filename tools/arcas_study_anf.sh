#!/bin/bash
# the Army-Navy Finner at the supersonic points of G-F5, zero incidence, sector, Coarse tier, GPU
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
cargo test --release -p bowshock-core --test arcas_study --no-run 2>&1 | tail -1
run() { env "$@" cargo test --release -p bowshock-core --test arcas_study -- --ignored --nocapture 2>&1 | grep -E "^STUDY|panicked|Error|error" ; }
for m in 1.832 2.375 2.718 3.147; do
  run STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=on
  run STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=off
  run STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=on STUDY_TIER=euler
done
