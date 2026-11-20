export BOWSHOCK_ENGINE=3d DIAG_RES=draft DIAG_MAXIT=3000 BOWSHOCK_FAIL_DIAG=1 DIAG_ROCKET=rabia-short-fin-can
T='cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture'
run() { echo "==== $1"; shift; env "$@" $T 2>&1 | grep -E "^POINT|ROLLBACK|panicked|^error|solves:|FAILDIAG|^SNAP" | cut -c1-420 | head -${LINES_MAX:-14}; }
run m1.3 DIAG_MACHS=1.3
run m2.3 DIAG_MACHS=2.3
run m2.3-thick0.12 DIAG_MACHS=2.3 FLIGHTVAL_FIN_THICK_IN=0.12
run m2.3-square DIAG_MACHS=2.3 FLIGHTVAL_SECTION=square
run m2.3-sweep4 DIAG_MACHS=2.3 FLIGHTVAL_SWEEP_IN=4
run m2.3-noshoulder DIAG_MACHS=2.3 FLIGHTVAL_SHOULDER=0
run m2.3-blunt0 DIAG_MACHS=2.3 FLIGHTVAL_BLUNT_IN=0
run m2.3-chord48 DIAG_MACHS=2.3 DIAG_CHORD=48
run m2.3-plain DIAG_MACHS=2.3 DIAG_PLAIN=1
