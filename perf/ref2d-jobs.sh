#!/usr/bin/env bash
# Regenerates crates/bowshock-cfd/fixtures/ref2d.json through the GPU job queue (tools/gpuq), one job per group.
#
#   perf/ref2d-jobs.sh submit            queue every group (run from any worktree; the job snapshots the worktree it is run in)
#   perf/ref2d-jobs.sh collect JOBID...  copy the group files of finished jobs (job ids in the order of `submit`) and merge them
#
# Commit the generator first: the git sha in the fixture is the HEAD of the worktree at submit time (REF2D_SHA; the snapshot has no .git).
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
gpuq="$here/tools/gpuq/gpuq"
groups=(g3 table-rans table-euler euler-base arcas tp2005 plates cones ogive)
spool=${GPUQ_DIR:-$HOME/.cache/bowshock-gpuq}

case ${1:-} in
  submit)
    cd "$here"
    sha=$(git rev-parse HEAD)
    dirty=0
    if git status --porcelain | grep -q .; then dirty=1; fi
    for g in "${groups[@]}"; do
      "$gpuq" submit --host local --name "ref2d-$g" --timeout 3000 -- \
        bash -c "REF2D_SHA=$sha REF2D_DIRTY=$dirty cargo run --release -p bowshock-validate --example ref2d -- --group $g --out out/ref2d/$g.json"
    done
    ;;
  collect)
    shift
    mkdir -p "$here/out/ref2d"
    i=0
    for id in "$@"; do
      g=${groups[$i]}
      cp "$spool/done/$id/out/ref2d/$g.json" "$here/out/ref2d/$g.json"
      i=$((i + 1))
    done
    cd "$here"
    cargo run --release -p bowshock-validate --example ref2d -- --merge out/ref2d --out crates/bowshock-cfd/fixtures/ref2d.json
    ;;
  *)
    sed -n 2,7p "$0"
    exit 2
    ;;
esac
