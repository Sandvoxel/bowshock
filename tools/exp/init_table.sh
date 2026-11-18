#!/usr/bin/env bash
# Cold vs wedge-initialised finned sector (experiment `exp_init`): per Mach number the cold baseline then the wedge-initialised run; the INIT lines are the table.
# Run on the GPU through gpuq:  MACHS="0.5 0.9 1.3 2.3" TIER=draft bash tools/exp/init_table.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
MACHS=${MACHS:-"0.5 0.9 1.3 2.3"}
TIER=${TIER:-draft}
cargo build --release -p bowshock-validate --example exp_init
for m in $MACHS; do
  ./target/release/examples/exp_init --tier "$TIER" --mach "$m" --cold
  ./target/release/examples/exp_init --tier "$TIER" --mach "$m"
done
