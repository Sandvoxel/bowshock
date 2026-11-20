#!/bin/bash
# comp_sweep.sh [--wedge] MACH COMP...   (DESIGN3D item 54: dilatation-dissipation compressibility corrections of the SST model on the Basic Finner)
# The grid-converged sector (Coarse tier, 64 chord cells, drift tolerance 5e-4: the settings of item 47's chord-64 row, as `transition_sweep.sh`) at Mach MACH, or with `--wedge` the finless
# Army-Navy body on the axisymmetric wedge, one process per correction under gpumem: `finned_table --anf [--wedge] --comp COMP` (`off` is the plain model). Prints a `SWEEP` header, the
# TABLE / FINFACES / MEM lines of each.
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="${CARGO_TARGET_DIR:-target}/release/examples/finned_table"
wedge=""
if [ "$1" = "--wedge" ]; then wedge="--wedge"; shift; fi
M=$1; shift
for comp in "$@"; do
  echo "SWEEP M$M $wedge comp $comp"
  if [ -n "$wedge" ]; then extra=""; else extra="--chord 64 --drift-tol 5e-4"; fi
  tools/gpuq/gpumem --label "comp-M$M-$comp${wedge:+-wedge}" -- "$BIN" --anf $wedge --tier coarse --mach "$M" $extra --comp "$comp" 2>&1 | grep -E "^TABLE|^FINFACES|^MT2|^MEM|panicked|Error" | sed "s/^/[comp $comp $wedge] /"
done
