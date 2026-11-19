#!/bin/bash
# transition_sweep.sh MACH XTR:FIN...   (DESIGN3D item 50: prescribed transition of the Basic Finner sector)
# The grid-converged sector (Coarse tier, 64 chord cells, drift tolerance 5e-4: the settings of item 47's chord-64 row) at Mach MACH, one process per configuration under gpumem:
# `finned_table --anf --x-tr XTR --fin-tr FIN` (XTR calibers from the nose tip on the body, FIN the laminar fraction of the fin chord from the leading edge; `0:0` is the fully turbulent
# baseline, run without the flags). Prints a `SWEEP` header, the TABLE / FINFACES / MEM lines of each.
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="${CARGO_TARGET_DIR:-target}/release/examples/finned_table"
M=$1; shift
for cfg in "$@"; do
  x=${cfg%%:*}; f=${cfg##*:}
  flags=""
  if [ "$x" != "0" ] || [ "$f" != "0" ]; then flags="--x-tr $x --fin-tr $f"; fi
  echo "SWEEP M$M x_tr $x fin_tr $f"
  tools/gpuq/gpumem --label "tr-M$M-$x-$f" -- "$BIN" --anf --tier coarse --mach "$M" --chord 64 --drift-tol 5e-4 $flags 2>&1 | grep -E "^TABLE|^FINFACES|^MEM|panicked|Error" | sed "s/^/[x_tr $x fin $f] /"
done
