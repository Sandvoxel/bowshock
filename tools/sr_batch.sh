export BOWSHOCK_ENGINE=3d DIAG_MACHS=0.8 DIAG_RES=draft DIAG_ROCKET=rabia-short-fin-can DIAG_MAXIT=4000
T='cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture'
run() { echo "==== $1"; shift; env "$@" $T 2>&1 | grep -E "^POINT|ROLLBACK|panicked|^error" | cut -c1-200; }
run lerc4 DIAG_LERC=4
run lerc2 DIAG_LERC=2
run lerc8 DIAG_LERC=8
run square FLIGHTVAL_SECTION=square
run chord128 DIAG_CHORD=128
run hex FLIGHTVAL_SECTION=hexagonal
