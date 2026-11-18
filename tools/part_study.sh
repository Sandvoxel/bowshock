#!/bin/bash
# part by part, fins on / off, Coarse sector on the GPU: the Finner (sourced section) at Mach 1.832 and 3.147, the Arcas at Mach 1.5 (DESIGN3D item 40)
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
BIN=$(cargo test --release -p bowshock-core --test arcas_study --no-run --message-format=json 2>/dev/null | grep -o '"executable":"[^"]*arcas_study-[0-9a-f]*"' | head -1 | cut -d'"' -f4)
run() { env "$@" "$BIN" --ignored --nocapture 2>&1 | grep -E "^STUDY|panicked"; }
for m in 1.832 3.147; do
  run STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=on
  run STUDY_BODY=anf STUDY_MACH=$m STUDY_FINS=off
done
run STUDY_MACH=1.5 STUDY_FINS=on
run STUDY_MACH=1.5 STUDY_FINS=off
