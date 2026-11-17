#!/bin/bash
# finned_cfl.sh: Draft sector points at lower cfl_max (the default 50 below Mach 1 and 20 above diverge after the ramp on the example rocket's finned sector)
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
for a in "0.5 10" "0.5 20" "0.9 10" "0.9 20" "3.0 10" "3.0 5"; do
  set -- $a
  tier=draft; if [ "$1" = 0.5 ]; then tier=draft-safe; fi
  "$BIN" --tier $tier --mach $1 --cfl-max $2 --max-iter 4000 2>&1 | grep -E "^TABLE|panicked" | sed "s/^TABLE/TABLE cfl_max=$2/"
done
