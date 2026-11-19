#!/usr/bin/env bash
# Device memory of the sector (688 k) and half-domain compile with and without the `Context::trim` calls of the product (DESIGN3D item 53): runs `mem_items` for each chain under
# gpumem (process peak, fast_peak) with BWTRIM_* switches. Usage (inside a gpuq job): tools/memprof/trim_ab.sh [variants...]  (default: all)
set -u
cd "$(dirname "$0")/../.."
cargo build --release -p bowshock-validate --example mem_items >/dev/null 2>&1 || cargo build --release -p bowshock-validate --example mem_items || exit 1
bin=${CARGO_TARGET_DIR:-target}/release/examples/mem_items
variants=${*:-"default nopre nopost notrim"}
for v in $variants; do
  case $v in
    default) e=() ;;
    nopre) e=(BWTRIM_NOPRE=1) ;;
    nopost) e=(BWTRIM_NOPOST=1) ;;
    notrim) e=(BWTRIM_NOPRE=1 BWTRIM_NOPOST=1 BWTRIM_NOSHAPE=1) ;;
    *) echo "unknown variant $v"; exit 2 ;;
  esac
  for chain in anf anfhalf; do
    echo "=== variant $v chain $chain"
    env "${e[@]}" tools/gpuq/gpumem --label "$v-$chain" -- "$bin" --chain "$chain" ${MEM_ARGS:-} --sub 2 --reclaim --run-on 300 2>&1 | grep -E "MEMITEM|MEM " 
  done
done
