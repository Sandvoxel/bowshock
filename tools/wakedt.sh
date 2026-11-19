#!/bin/bash
# wakedt.sh TIER MACH [MODE...]   (DESIGN3D item 60: the pseudo-time step's k rate). One finned_table run per DtMode (sum, sum-capk) on the example sector, history every 50
# iterations, no stop rule (EXTRA overrides, MAXIT iterations, default 6000), the time step diagnostic of the final state. Prints TABLE / DTDIAG / HIST / TABLE-ROLLBACKS lines.
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
tier=$1; mach=$2; shift 2
for mode in ${@:-sum sum-capk}; do
  echo "=== $tier M$mach dt-mode $mode"
  tools/gpuq/gpumem --label "wakedt-$tier-M$mach-$mode" -- "$BIN" --tier "$tier" --mach "$mach" --dt-mode "$mode" --dt-diag --history-every 50 --max-iter "${MAXIT:-6000}" ${EXTRA:---drift-tol 0} 2>&1 | grep -E "^TABLE|^DTDIAG|^HIST|^MEM|panicked|Error"
done
