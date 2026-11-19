cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
for s in 1 1.333 1.5 1.778 2.0 2.25; do
  echo "=== refine $s (guard on)"
  "$BIN" --anf --tier coarse --mach 2.741 --refine "$s" --grid-only 2>&1 | grep -E "^GRID|panicked|Error"
  tools/gpuq/gpumem --label "s$s" -- "$BIN" --anf --tier coarse --mach 2.741 --refine "$s" --max-iter 260 2>&1 | grep -E "^TABLE|^MEM|panicked|rror|needs"
done
for s in 2.0 2.25; do
  echo "=== refine $s (guard off)"
  BOWSHOCK_GPU_MEM_GUARD=off tools/gpuq/gpumem --label "s$s-noguard" -- "$BIN" --anf --tier coarse --mach 2.741 --refine "$s" --max-iter 260 2>&1 | grep -E "^TABLE|^MEM|panicked|rror|needs"
done
