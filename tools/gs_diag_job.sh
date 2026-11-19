#!/bin/bash
# gs_diag_job.sh N MACH CHORD ITERS [extra gs_diag args]   (DESIGN3D item 52) the four runs of one G-S point: sector and half domain, each with the default stop rule and for ITERS fixed iterations
n=$1; m=$2; c=$3; it=$4; shift 4
export BOWSHOCK_WEDGE_BACKEND=gpu
RUN="cargo run --release -q -p bowshock-core --example gs_diag --"
for d in ${DOMS:-sector half}; do
  echo "== $d N$n M$m chord $c default stop"
  $RUN run $d $n $m --chord $c "$@" 2>&1
  [ "$it" -gt 0 ] && { echo "== $d N$n M$m chord $c fixed $it"; $RUN run $d $n $m --chord $c --iters $it "$@" 2>&1; }
done
