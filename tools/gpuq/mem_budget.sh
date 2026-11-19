#!/usr/bin/env bash
# mem_budget.sh: the biggest grids of the example rocket's coarse solve under the memory budget of a smaller card (DESIGN3D item 57), emulated on a big one with Luma's `LUMA_MEMORY_BUDGET`
# (80 % of the device memory: bytes of array data kept on the device before spilling to the host) and the facade's guard (`BOWSHOCK_GPU_MEM_GUARD`, on by default). One block per case: the
# `MEMBUDGET` lines of `engine3d_memory_budget` (cells, ms per iteration or the guard's refusal) and the `MEM` line of `gpumem` (process peak, MiB).
#   tools/gpuq/gpuq submit --host local -- tools/gpuq/mem_budget.sh
set -u
export BOWSHOCK_ENGINE=3d
GIB=1073741824
cases=(
  "24gb-default none on"
  "12gb $((12 * GIB * 8 / 10)) on"
  "8gb $((8 * GIB * 8 / 10)) on"
  "6gb $((6 * GIB * 8 / 10)) on"
  "6gb-guard-off $((6 * GIB * 8 / 10)) off"
  "4gb-guard-off $((4 * GIB * 8 / 10)) off"
)
for c in "${cases[@]}"; do
  read -r name budget guard <<< "$c"
  echo "=== $name budget $budget guard $guard"
  if [ "$budget" = none ]; then unset LUMA_MEMORY_BUDGET; else export LUMA_MEMORY_BUDGET=$budget; fi
  env BOWSHOCK_GPU_MEM_GUARD=$guard tools/gpuq/gpumem --label "$name" -- cargo nextest run --release -p bowshock engine3d_memory_budget --run-ignored only --no-capture 2>&1 | grep -E "MEMBUDGET|^MEM |panicked|warning:.*memory"
done
