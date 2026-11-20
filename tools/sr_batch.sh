export BOWSHOCK_ENGINE=3d DIAG_MACHS=0.8 DIAG_RES=draft DIAG_ROCKET=rabia-short-fin-can DIAG_MAXIT=4000
T='cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture'
run() { echo "==== $1"; shift; env "$@" $T 2>&1 | grep -E "^POINT|ROLLBACK|panicked" | cut -c1-200; }
run chord96 DIAG_CHORD=96
run chord128 DIAG_CHORD=128
run square FLIGHTVAL_SECTION=square
run hex FLIGHTVAL_SECTION=hexagonal
run chord64-first400 DIAG_CHORD=64 DIAG_FIRST=400
