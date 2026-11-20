#!/bin/bash
# the incidence response of the gate articles (DESIGN3D item 67): `tests/stab_study.rs` for a list of "ROCKET MACH ALPHAS [PHASE]" cases; each prints STAB lines (CN, x_cp, nose / afterbody / fin split per incidence)
export BOWSHOCK_WEDGE_BACKEND=gpu
cargo test --release -p bowshock-core --test stab_study --no-run 2>&1 | tail -1
while read -r rocket mach alphas phase; do
  [ -z "$rocket" ] && continue
  echo "CASE $rocket $mach $alphas ${phase:-inplane}"
  STUDY_ROCKET=$rocket STUDY_MACH=$mach STUDY_ALPHAS=$alphas STUDY_PHASE=${phase:-inplane} \
    cargo test --release -p bowshock-core --test stab_study -- --ignored --nocapture 2>&1 | grep -E "^STAB|panicked|Error"
done
