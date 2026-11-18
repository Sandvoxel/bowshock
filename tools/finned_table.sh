#!/bin/bash
# finned_table.sh TIER...   (tier names: coarse draft; draft below Mach 0.8 is the safe variant)
# Runs every Mach number of the table, one process per point under gpumem. Environment: MACHS (default the ten of the table), CFL (cfl_max), SUB (sub_iters),
# CHORD (cells over the fin root chord), EXTRA (further finned_table flags), CFL_SUBSONIC (cfl_max below Mach 1 when it differs from CFL).
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
MACHS=${MACHS:-"0.1 0.5 0.8 0.9 1.0 1.1 1.3 1.7 2.3 3.0"}
for t in "$@"; do
  for m in $MACHS; do
    tier=$t
    if [ "$t" = draft ] && awk "BEGIN{exit !($m < 0.8)}"; then tier=draft-safe; fi
    flags=""
    cfl=${CFL:-}
    if [ -n "${CFL_SUBSONIC:-}" ] && awk "BEGIN{exit !($m < 1.0)}"; then cfl=$CFL_SUBSONIC; fi
    [ -n "$cfl" ] && flags="$flags --cfl-max $cfl"
    [ -n "${SUB:-}" ] && flags="$flags --sub $SUB"
    [ -n "${CHORD:-}" ] && flags="$flags --chord $CHORD"
    tools/gpuq/gpumem --label "$tier-M$m" -- "$BIN" --tier "$tier" --mach "$m" $flags ${EXTRA:-} 2>&1 | grep -E "^TABLE|^GF1-RATIO|^MEM|^WORST|panicked|Error" | sed "s/^TABLE/TABLE [cfl=${cfl:-default} sub=${SUB:-default} chord=${CHORD:-default}]/"
  done
done
