export BOWSHOCK_ENGINE=3d DIAG_RES=draft DIAG_MAXIT=3000 DIAG_ROCKET=rabia-short-fin-can DIAG_MACHS=2.3
T='cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture'
run() { echo "==== $1"; shift; env "$@" $T 2>&1 | grep -E "^POINT|^ROLLBACK|^error|solves:" | cut -c1-200 | sed -n '1,12p'; }
run base A=1
run k8f05 TMP_LE_K=8 TMP_LE_FLOOR=0.05
run k8f03 TMP_LE_K=8 TMP_LE_FLOOR=0.03
run k16f02 TMP_LE_K=16 TMP_LE_FLOOR=0.02
run k4f1-sweep4 FLIGHTVAL_SWEEP_IN=4
run k4f1-blunt0 FLIGHTVAL_BLUNT_IN=0
run chord48 DIAG_CHORD=48
run chord64 DIAG_CHORD=64
run chord96 DIAG_CHORD=96
echo "==== COARSE M1.3 base"; env DIAG_RES=coarse DIAG_MACHS=1.3 DIAG_MAXIT=2500 $T 2>&1 | grep -E "^POINT|^ROLLBACK|solves:" | cut -c1-200 | sed -n '1,12p'
echo "==== COARSE M1.3 k8f03"; env TMP_LE_K=8 TMP_LE_FLOOR=0.03 DIAG_RES=coarse DIAG_MACHS=1.3 DIAG_MAXIT=2500 $T 2>&1 | grep -E "^POINT|^ROLLBACK|solves:" | cut -c1-200 | sed -n '1,12p'
