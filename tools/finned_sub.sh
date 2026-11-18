#!/bin/bash
# finned_sub.sh: sub_iters 3 against 2 on the G-F1 finned sector (ANF body, 4 thin fins, Coarse), f32 GPU, to the stop rule
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
for m in 0.5 2.0; do
  for s in 3 2; do
    "$BIN" --gf1 --mach $m --sub $s 2>&1 | grep -E "^TABLE|panicked" | sed "s/^TABLE/TABLE sub_iters=$s/"
  done
done
