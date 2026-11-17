#!/bin/bash
# the Finner (G-F5) with the full-chord wedge section (sharp leading edge, 0.08 cal at the trailing edge; STUDY_LE=0.999), 4-fin sector, GPU, Coarse tier at the four Mach numbers
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
for m in 1.832 2.375 2.718 3.147; do
  env STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=on STUDY_LE=0.999 cargo test --release -p bowshock-core --test arcas_study -- --ignored --nocapture 2>&1 | grep -E "^STUDY|panicked|Error|error"
done
