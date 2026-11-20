#!/bin/bash
# usage: fr_multi.sh RES "rocket1 rocket2" machs [ALPHA]
res=$1; shift
for r in $1; do
  echo "=== ROCKET $r res $res"
  DIAG_ROCKET=$r DIAG_RES=$res DIAG_GRID=1 DIAG_COLLAPSE=1 DIAG_MACHS=$2 ${3:+DIAG_ALPHA=$3} \
    cargo test --release -p bowshock diag56_point -- --ignored --nocapture 2>&1 | grep -v "^\s*\(Compiling\|Finished\|Running\)" | grep -v "^GRID\|^HIST"
done
