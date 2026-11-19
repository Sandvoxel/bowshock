#!/usr/bin/env bash
# The trim of `GpuRunner3::new` when the grid shape changes (DESIGN3D item 53): a chain of runners of different shapes in one process (sector, half domain, sector), process peak
# with the trim (default) and without (BWTRIM_NOSHAPE) and with none of the three trims.
set -u
cd "$(dirname "$0")/../.."
cargo build --release -p bowshock-validate --example mem_items >/dev/null 2>&1 || cargo build --release -p bowshock-validate --example mem_items || exit 1
bin=${CARGO_TARGET_DIR:-target}/release/examples/mem_items
for v in default noshape notrim; do
  case $v in default) e=() ;; noshape) e=(BWTRIM_NOSHAPE=1) ;; notrim) e=(BWTRIM_NOPRE=1 BWTRIM_NOPOST=1 BWTRIM_NOSHAPE=1) ;; esac
  echo "=== $v"
  for chain in anf,anfhalf,anf anfhalf,anf,anfhalf; do
    env "${e[@]}" tools/gpuq/gpumem --label "$v-$chain" -- "$bin" --chain "$chain" --sub 2 --iters 50 --run-on 100 2>&1 | grep -E "^MEM |rror|panick"
  done
done
