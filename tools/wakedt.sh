#!/bin/bash
# wakedt.sh TIER MACH [MODE...]   (DESIGN3D item 60: the pseudo-time step's k rate). One finned_table run per DtMode (sum, sum-capk) and CFLS entry (cfl_max; default the case's),
# on the example sector, history every 50 iterations, no stop rule (EXTRA overrides, MAXIT iterations, default 6000), the time step diagnostic of the final state (DIAG=0: off).
# Prints TABLE / DTDIAG / HIST / TABLE-ROLLBACKS lines.
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
tier=$1; mach=$2; shift 2
for mode in ${@:-sum sum-capk}; do
  for cfl in ${CFLS:-default}; do
    flags="${FLAGS:-}"; [ "$cfl" != default ] && flags="$flags --cfl-max $cfl"
    [ "${DIAG:-1}" != 0 ] && flags="$flags --dt-diag"
    echo "=== $tier M$mach dt-mode $mode cfl $cfl"
    tools/gpuq/gpumem --label "wakedt-$tier-M$mach-$mode-$cfl" -- "$BIN" --tier "$tier" --mach "$mach" --dt-mode "$mode" $flags --history-every 50 --max-iter "${MAXIT:-6000}" ${EXTRA:---drift-tol 0} 2>&1 | grep -E "^TABLE|^DTDIAG|^HIST|^MEM|panicked|Error"
  done
done
