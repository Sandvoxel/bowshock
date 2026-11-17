#!/bin/bash
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
cargo test --release -p bowshock-core --test arcas_study --no-run 2>&1 | tail -1
run() { env "$@" cargo test --release -p bowshock-core --test arcas_study -- --ignored --nocapture 2>&1 | grep -E "^STUDY|panicked|Error|error" ; }
for m in 1.5; do
  run STUDY_MACH=$m STUDY_FINS=on
  run STUDY_MACH=$m STUDY_FINS=off
  run STUDY_MACH=$m STUDY_FINS=on STUDY_CHORD=64
  run STUDY_MACH=$m STUDY_FINS=on STUDY_SPAN=48
  run STUDY_MACH=$m STUDY_FINS=on STUDY_TIER=fine
  run STUDY_MACH=$m STUDY_FINS=off STUDY_TIER=fine
  run STUDY_MACH=$m STUDY_FINS=on STUDY_LEVEL=1
  run STUDY_MACH=$m STUDY_FINS=off STUDY_LEVEL=1
  run STUDY_MACH=$m STUDY_FINS=on STUDY_TIER=euler
  run STUDY_MACH=$m STUDY_FINS=off STUDY_TIER=euler
  run STUDY_MACH=$m STUDY_FINS=on STUDY_PROFILE=blunt
done
for m in 1.8 2.3 2.96; do
  run STUDY_MACH=$m STUDY_FINS=on
  run STUDY_MACH=$m STUDY_FINS=off
  run STUDY_MACH=$m STUDY_FINS=on STUDY_TIER=euler
  run STUDY_MACH=$m STUDY_FINS=off STUDY_TIER=euler
done
