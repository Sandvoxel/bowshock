export BOWSHOCK_ENGINE=3d DIAG_MAXIT=4000
T='cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture'
run() { echo "==== $1"; shift; env "$@" $T > /tmp/sr_run.log 2>&1; grep -E "^POINT|solves:" /tmp/sr_run.log | cut -c1-200; echo "rollbacks: $(grep -c '^ROLLBACK' /tmp/sr_run.log)"; }
run rabia-draft-0.8,1.3,2.3 DIAG_RES=draft DIAG_ROCKET=rabia-short-fin-can DIAG_MACHS=0.8,1.3,2.3
run example-draft-0.8,1.3,2.3 DIAG_RES=draft DIAG_ROCKET=example DIAG_MACHS=0.8,1.3,2.3
run rabia-coarse-1.3,2.3 DIAG_RES=coarse DIAG_ROCKET=rabia-short-fin-can DIAG_MACHS=1.3,2.3 DIAG_MAXIT=3000
