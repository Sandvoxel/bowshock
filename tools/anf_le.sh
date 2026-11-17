#!/bin/bash
# the Finner (G-F5) with the full-chord wedge section (sharp leading edge, 0.08 cal at the trailing edge; STUDY_LE=0.999), without (Z0 0) and with the leading-edge radius
# of 0.004 cal (STUDY_Z0=0.1: half thickness 10 % of t/2), 4-fin sector, GPU, Coarse tier at the four Mach numbers
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
for z in 0.1 0.0; do
  for m in 1.832 2.375 2.718 3.147; do
    env STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=on STUDY_LE=0.999 STUDY_Z0=$z cargo test --release -p bowshock-core --test arcas_study -- --ignored --nocapture 2>&1 | grep -E "^STUDY|panicked|Error|error" | sed "s/^STUDY/STUDY Z0=$z/"
  done
done
