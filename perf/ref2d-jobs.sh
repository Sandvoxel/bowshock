#!/usr/bin/env bash
# Regenerates crates/bowshock-cfd/fixtures/ref2d.json through the GPU job queue (tools/gpuq), one job per group.
#
#   perf/ref2d-jobs.sh submit [GROUP...]      queue the groups (default: all); prints `GROUP=JOBID` lines. The job snapshots the worktree
#                                             this is run in (any worktree: the queue is shared through ~/.cache/bowshock-gpuq).
#   perf/ref2d-jobs.sh collect GROUP=JOBID... copy the group files of finished jobs into out/ref2d and merge all groups into the fixture
#
# Commit the generator first: the git sha of a group is the HEAD of the worktree at submit time (REF2D_SHA; the snapshot has no .git).
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
gpuq="$here/tools/gpuq/gpuq"
all=(g3 table-rans table-euler euler-base arcas tp2005 plates cones ogive)
spool=${GPUQ_DIR:-$HOME/.cache/bowshock-gpuq}

case ${1:-} in
  submit)
    shift
    groups=("$@")
    [ ${#groups[@]} -gt 0 ] || groups=("${all[@]}")
    cd "$here"
    sha=$(git rev-parse HEAD)
    dirty=0
    if git status --porcelain | grep -q .; then dirty=1; fi
    for g in "${groups[@]}"; do
      id=$("$gpuq" submit --host local --name "ref2d-$g" --timeout 3000 -- \
        bash -c "REF2D_SHA=$sha REF2D_DIRTY=$dirty cargo run --release -p bowshock-validate --example ref2d -- --group $g --out out/ref2d/$g.json")
      echo "$g=$id"
    done
    ;;
  collect)
    shift
    mkdir -p "$here/out/ref2d"
    for pair in "$@"; do
      g=${pair%%=*}
      id=${pair#*=}
      cp "$spool/done/$id/out/ref2d/$g.json" "$here/out/ref2d/$g.json"
    done
    cd "$here"
    cargo run --release -p bowshock-validate --example ref2d -- --merge out/ref2d --out crates/bowshock-cfd/fixtures/ref2d.json
    ;;
  *)
    sed -n 2,8p "$0"
    exit 2
    ;;
esac
