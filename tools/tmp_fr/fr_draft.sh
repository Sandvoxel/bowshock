#!/bin/bash
# usage: fr_draft.sh "rocket1 rocket2" machs
for r in $1; do
  echo "=== ROCKET $r"
  DIAG_ROCKET=$r DIAG_RES=${DIAG_RES:-draft} DIAG_GRID=1 DIAG_COLLAPSE=1 DIAG_MACHS=$2 \
    cargo test --release -p bowshock diag56_point -- --ignored --nocapture 2>&1 | grep -v "^\s*\(Compiling\|Finished\|Running\)" 
done
