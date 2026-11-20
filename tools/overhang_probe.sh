#!/usr/bin/env bash
# Cancer Descending diag probes (DESIGN3D item 73): one drag point per variant, Draft or Coarse. Usage: overhang_probe.sh RES MACH "ENV=.. ENV=.." ["ENV=.." ...]
res=$1; mach=$2; shift 2
for v in "$@"; do
  echo "#### variant [$v] res $res mach $mach"
  env DIAG_ROCKET=${DIAG_ROCKET:-cancer-descending} DIAG_RES=$res DIAG_MACHS=$mach DIAG_COLLAPSE=1 $v cargo test --release -p bowshock diag56_point -- --ignored --nocapture 2>&1 | grep -E "^POINT|^COLLAPSED|^HIST|^ROLLBACK|panicked|error" | cut -c1-300
done
