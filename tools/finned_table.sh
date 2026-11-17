#!/bin/bash
# usage: ft.sh TIERLIST...   (tier names: coarse draft draft-safe)  runs every Mach of the table, one process per point under gpumem
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
for t in "$@"; do
  for m in 0.1 0.5 0.8 0.9 1.0 1.1 1.3 1.7 2.3 3.0; do
    tier=$t
    if [ "$t" = draft ] && [ "$m" = 0.1 -o "$m" = 0.5 ]; then tier=draft-safe; fi
    tools/gpuq/gpumem --label "$tier-M$m" -- "$BIN" --tier "$tier" --mach "$m" 2>&1 | grep -E "^TABLE|^MEM|panicked|Error" 
  done
done
