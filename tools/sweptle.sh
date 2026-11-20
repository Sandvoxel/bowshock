#!/bin/bash
# the swept rounded fin edge of the Rabia fixture (DESIGN3D item 66): `tools/sweptle.sh <res> <machs> <rocket> [ENV=VALUE ...]` prints the POINT / ROLLBACK lines of one diag56 run
export BOWSHOCK_ENGINE=3d DIAG_MAXIT=${DIAG_MAXIT:-30000}
res=$1 machs=$2 rocket=$3; shift 3
echo "==== $res $machs $rocket $*"
env DIAG_RES=$res DIAG_MACHS=$machs DIAG_ROCKET=$rocket "$@" cargo nextest run --release -p bowshock diag56 --run-ignored only --no-capture 2>&1 | grep -E "^POINT|ROLLBACK|panicked|^error" | cut -c1-300
