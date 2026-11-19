#!/usr/bin/env bash
# Calibration points of the device-memory guard (`device_bytes3`, DESIGN3D item 53): the process peak (gpumem, fast_peak) and Luma's resident bytes of the compile of the Finner sector at
# 32 chord cells and uniform refinements S (cells grow as S^3) and of the half domain (1 and 2 cells across the fin), f32, SST, sub_iters 2, guard off.
# Usage (inside a gpuq job): tools/memprof/guard_cal.sh "sector:1 sector:1.5:--ext sector:1.5:--sub,5 .." (kind:value[:flags with commas for spaces]; default: sector 1 1.5 1.75, half 1 2)
set -u
cd "$(dirname "$0")/../.."
cargo build --release -p bowshock-validate --example mem_items >/dev/null 2>&1 || cargo build --release -p bowshock-validate --example mem_items || exit 1
bin=${CARGO_TARGET_DIR:-target}/release/examples/mem_items
export BOWSHOCK_GPU_MEM_GUARD=off
list=${1:-sector:1 sector:1.5 sector:1.75 half:1 half:2}
for p in $list; do
  kind=${p%%:*}; v=${p#*:}; extra=""
  case $v in *:*) extra=${v#*:}; v=${v%%:*}; extra=${extra//,/ } ;; esac
  case $kind in
    sector) args=(--chain anf --chord 32 --refine "$v") ;;
    half) args=(--chain anfhalf --across "$v") ;;
  esac
  echo "=== $p"
  tools/gpuq/gpumem --label "$p" -- "$bin" "${args[@]}" $extra --sub 2 --iters 50 --reclaim 2>&1 | grep -E "MEMITEM (==|.*compile|.*run_monitored)|^MEM |rror|panick"
done
