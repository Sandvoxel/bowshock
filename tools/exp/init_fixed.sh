#!/usr/bin/env bash
# Cold vs wedge-initialised finned sector at a fixed iteration count (no stop rule on the sector): PROG lines every 50 iterations. Through gpuq:
#   MACHS="0.5 0.9 1.3 2.3" TIER=draft N=10000 bash tools/exp/init_fixed.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
MACHS=${MACHS:-"0.5 0.9 1.3 2.3"}
TIER=${TIER:-draft}
N=${N:-10000}
cargo build --release -p bowshock-validate --example exp_init 2>&1 | tail -1
BIN="${CARGO_TARGET_DIR:-target}/release/examples/exp_init"
for m in $MACHS; do
  tier=$TIER
  if [ "$TIER" = draft ] && awk "BEGIN{exit !($m < 0.8)}"; then tier=draft-safe; fi
  "$BIN" --tier "$tier" --mach "$m" --fixed "$N" --cold | sed "s/^PROG/PROG-cold M$m/"
  "$BIN" --tier "$tier" --mach "$m" --fixed "$N" | sed "s/^PROG sector/PROG-wedge M$m sector/;s/^PROG wedge/PROG-wedgerun M$m/"
done
