#!/usr/bin/env bash
# Device-memory profile of the 3D GPU step (DESIGN3D items 48 and 49): builds and runs `mem_items` (crates/bowshock-validate/examples/mem_items.rs)
# against the pinned Luma, which has `Context::memory_stats` / `trim` since 231c986. (Item 48's per-buffer live intervals needed a patch of Luma 7fafed6: branch `memprof`, 2c0e356.)
# Usage: tools/memprof/run.sh <example args...>   e.g.  --chain anf --sub 2 --reclaim --run-on 200
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
cargo run --release -p bowshock-validate --example "${MEMPROF_EXAMPLE:-mem_items}" -- "$@"
