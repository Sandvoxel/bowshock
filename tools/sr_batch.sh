export BOWSHOCK_ENGINE=3d DIAG_MACHS=0.8 DIAG_RES=draft DIAG_ROCKET=rabia-short-fin-can DIAG_MAXIT=4000
T='cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture'
run() { echo "==== $1"; shift; env "$@" $T 2>&1 | grep -E "^POINT|ROLLBACK|panicked" | cut -c1-200; }
run base A=1
run plain DIAG_PLAIN=1
run cflmax20 DIAG_CFLMAX=20
run first600 DIAG_FIRST=600
run across2 DIAG_ACROSS=2
run chord64 DIAG_CHORD=64
run lowmach0 DIAG_LOWMACH=0
run thick0.12 FLIGHTVAL_FIN_THICK_IN=0.12
run thick0.25 FLIGHTVAL_FIN_THICK_IN=0.25
run sweep7 FLIGHTVAL_SWEEP_IN=7
run example DIAG_ROCKET=example
