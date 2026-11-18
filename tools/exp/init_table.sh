#!/usr/bin/env bash
# Cold vs wedge-initialised finned sector (experiment `exp_init`): per Mach number the cold baseline then the wedge-initialised run; the INIT lines are the table.
# Run on the GPU through gpuq:  MACHS="0.5 0.9 1.3 2.3" TIER=draft bash tools/exp/init_table.sh   (draft below Mach 0.8 is the safe variant, as tools/finned_table.sh)
set -euo pipefail
cd "$(dirname "$0")/../.."
MACHS=${MACHS:-"0.5 0.9 1.3 2.3"}
TIER=${TIER:-draft}
cargo build --release -p bowshock-validate --example exp_init 2>&1 | tail -1
BIN="${CARGO_TARGET_DIR:-target}/release/examples/exp_init"
for m in $MACHS; do
  tier=$TIER
  if [ "$TIER" = draft ] && awk "BEGIN{exit !($m < 0.8)}"; then tier=draft-safe; fi
  "$BIN" --tier "$tier" --mach "$m" --cold
  "$BIN" --tier "$tier" --mach "$m"
done
