#!/bin/bash
# wakedt_r2.sh (DESIGN3D item 60, 3080L half): the facade path of the app (seeded sector, `analyze::cfd::tests::diag56`): the real Rabia short-fin-can fixture on the Draft tier at Mach 2.3 and
# 1.3, the example rocket on the Coarse tier at Mach 2.3: the plain sum against the cap 1 from iteration 300 / 600 at cfl_max 25 / 40 (`BOWSHOCK_DT_CAPK=cap,from,cfl`).
cargo test --release -p bowshock --no-run 2>&1 | tail -1
export DIAG_HIST_EVERY=50
run() { # rocket res mach maxit [BOWSHOCK_DT_CAPK]
  echo "=== $1 $2 M$3 maxit $4 capk ${5:-none}"
  if [ -n "${5:-}" ]; then export BOWSHOCK_DT_CAPK=$5; else unset BOWSHOCK_DT_CAPK; fi
  DIAG_ROCKET=$1 DIAG_RES=$2 DIAG_MACHS=$3 DIAG_MAXIT=$4 cargo test --release -p bowshock diag56_point -- --ignored --nocapture 2>&1 | grep -E "^POINT|^ROLLBACK|^HIST|panicked"
}
variants() {
  run "$@"
  for from in 300 600; do for cfl in 25 40; do run "$@" "1,$from,$cfl"; done; done
}
variants rabia-short-fin-can draft 2.3 3000
variants rabia-short-fin-can draft 1.3 3000
variants example coarse 2.3 6000
