export BOWSHOCK_ENGINE=3d DIAG_RES=draft DIAG_MAXIT=8000
T='cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture'
run() { echo "==== $1"; shift; env "$@" $T 2>&1 | grep -E "^POINT|ROLLBACK|panicked|^error|solves:" | cut -c1-300 | head -30; }
run rabia DIAG_ROCKET=rabia-short-fin-can DIAG_MACHS=0.8,1.3,2.3
run example DIAG_ROCKET=example DIAG_MACHS=0.8,2.3
echo "==== coarse"; env DIAG_RES=coarse DIAG_MAXIT=30000 DIAG_ROCKET=rabia-short-fin-can DIAG_MACHS=0.8,1.3 $T 2>&1 | grep -E "^POINT|ROLLBACK|solves:" | cut -c1-300
echo "==== coarse example"; env DIAG_RES=coarse DIAG_ROCKET=example DIAG_MACHS=0.8 $T 2>&1 | grep -E "^POINT|ROLLBACK|solves:" | cut -c1-300
