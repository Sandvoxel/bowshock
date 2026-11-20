#!/bin/bash
# wakedt_r1.sh (DESIGN3D item 60, start-up that survives, 3090 half): finned_table, cold start, the example sector on the Draft tier at Mach 1.3 / 2.3 / 3.0 and on the Coarse tier at
# Mach 2.3: the plain sum at cfl_max 100 against the cap 1 from iteration 300 / 600 at cfl_max 25 / 40.
cargo test --release -p bowshock-gpu-cases --test euler3d tiny_anf_dt_k_cap -- --nocapture 2>&1 | grep -E "^M |^\||test result|panicked"
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
run() { # tier mach maxit [flags]
  t=$1; m=$2; mx=$3; shift 3
  echo "=== $t M$m $*"
  "$BIN" --tier "$t" --mach "$m" --history-every 50 --max-iter "$mx" --drift-tol 0 "$@" 2>&1 | grep -E "^TABLE|^HIST|panicked|Error"
}
variants() { # tier mach maxit
  run "$@"
  for from in 300 600; do for cfl in 25 40; do run "$@" --dt-k-cap 1 --dt-k-from $from --dt-k-cfl $cfl; done; done
}
for m in 1.3 2.3 3.0; do variants draft $m 3000; done
variants coarse 2.3 6000
