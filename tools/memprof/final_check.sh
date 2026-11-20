#!/usr/bin/env bash
# The memory figures of DESIGN3D item 53 on the product code (the trims of item 49 reduced to the one in `GpuRunner3::new`): process peak (gpumem) of the sector and the half domain
# (compile + 300 iterations of the product path, `--reclaim` for the steady figure) and of a chain of three solvers of two shapes in one process.
set -u
cd "$(dirname "$0")/../.."
cargo build --release -p bowshock-validate --example mem_items >/dev/null 2>&1 || cargo build --release -p bowshock-validate --example mem_items || exit 1
bin=${CARGO_TARGET_DIR:-target}/release/examples/mem_items
for a in "--chain anf --chord 32 --reclaim --run-on 300" "--chain anf --reclaim --run-on 300" "--chain anfhalf --reclaim --run-on 300" "--chain anfhalf --across 2 --reclaim --run-on 300" "--chain anf,anfhalf,anf --iters 50 --run-on 100" "--chain anfhalf,anf,anfhalf --iters 50 --run-on 100"; do
  echo "=== $a"
  tools/gpuq/gpumem --label "$a" -- "$bin" $a --sub 2 2>&1 | grep -E "^MEM |MEMITEM (==|.*compile|.*run_on)|rror|panick"
done
