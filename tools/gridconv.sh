#!/bin/bash
# gridconv.sh MACH S...   (DESIGN3D item 47: the grid-convergence family of the Basic Finner sector)
# One process per grid under gpumem: `finned_table --anf --tier ${TIER:-coarse} --mach MACH --refine S` for every refinement factor S (1 is the production Coarse grid; the
# family is geometrically similar, `Case3::with_refinement`). Prints the GRID, TABLE, FINFACES, TABLE-RUN and MEM lines of each. Environment: TIER (coarse|fine), EXTRA (further flags).
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
M=$1; shift
for s in "$@"; do
  "$BIN" --anf --tier "${TIER:-coarse}" --mach "$M" --refine "$s" --grid-only ${EXTRA:-} 2>&1 | grep -E "^GRID|panicked|Error"
  tools/gpuq/gpumem --label "${TIER:-coarse}-M$M-s$s" -- "$BIN" --anf --tier "${TIER:-coarse}" --mach "$M" --refine "$s" ${EXTRA:-} 2>&1 | grep -E "^TABLE|^FINFACES|^MEM|panicked|Error" | sed "s/^TABLE/TABLE [s=$s]/; s/^FINFACES/FINFACES [s=$s]/"
done
