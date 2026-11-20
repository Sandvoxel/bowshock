#!/bin/bash
# usage: fr_probe.sh ROCKET RES MACH  then probes "ENV=VAL ..." one per line on stdin-free list below
r=$1; res=$2; m=$3; shift 3
for probe in "$@"; do
  echo "=== PROBE $r $res M$m: $probe"
  env $probe DIAG_ROCKET=$r DIAG_RES=$res DIAG_MACHS=$m DIAG_COLLAPSE=1 \
    cargo test --release -p bowshock diag56_point -- --ignored --nocapture 2>&1 | grep "^POINT\|^ROLLBACK\|^COLLAPSED\|^SNAP\|panicked" | cut -c1-420 | head -${PROBE_LINES:-30}
done
