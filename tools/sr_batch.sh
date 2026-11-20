export BOWSHOCK_ENGINE=3d DIAG_RES=draft DIAG_MAXIT=8000
T='cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture'
run() { echo "==== $1"; shift; env "$@" $T 2>&1 | grep -E "^POINT|ROLLBACK|panicked|^error|solves:|FAILDIAG" | cut -c1-330 | head -30; }
run rabia DIAG_ROCKET=rabia-short-fin-can DIAG_MACHS=0.8,1.3,2.3 BOWSHOCK_FAIL_DIAG=1
run example DIAG_ROCKET=example DIAG_MACHS=0.8,1.3,2.3
