#!/bin/bash
# wakedt.sh TIER MACH [CAP...]   (DESIGN3D item 60: the pseudo-time step's k rate). One finned_table run per cap of the k rate (`--dt-k-cap`: `inf` the plain sum) and CFLS entry
# (cfl_max; default the case's), on the example sector (RABIA=1: the Rabia short-fin-can), history every HIST (50) iterations, MAXIT iterations (6000), no stop rule unless EXTRA is set
# (default `--drift-tol 0`), FLAGS added. The time step diagnostic of the final state with DIAG=1. Prints TABLE / DTDIAG / HIST / TABLE-ROLLBACKS lines.
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
tier=$1; mach=$2; shift 2
for cap in ${@:-inf 1}; do
  for cfl in ${CFLS:-default}; do
    flags="${FLAGS:-}"; [ "$cfl" != default ] && flags="$flags --cfl-max $cfl"
    [ "${DIAG:-0}" = 1 ] && flags="$flags --dt-diag"
    [ -n "${RABIA:-}" ] && flags="$flags --rabia"
    echo "=== $tier M$mach cap $cap cfl $cfl ${RABIA:+rabia}"
    tools/gpuq/gpumem --label "wakedt-$tier-M$mach-cap$cap-$cfl" -- "$BIN" --tier "$tier" --mach "$mach" --dt-k-cap "$cap" $flags --history-every "${HIST:-50}" --max-iter "${MAXIT:-6000}" ${EXTRA:---drift-tol 0} 2>&1 | grep -E "^TABLE|^DTDIAG|^HIST|^MEM|panicked|Error"
  done
done
