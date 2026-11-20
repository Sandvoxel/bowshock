#!/bin/bash
# integration verification, local 3090: app-path full solve first (timing), then the gate groups
set -u
echo "#### full solve"; env BOWSHOCK_ENGINE=3d gpumem --label int-full -- cargo nextest run --release -p bowshock engine3d_finned_timing --run-ignored only --no-capture 2>&1 | grep -E "PERF|SUM|wall|peak|gpumem|PASS|FAIL|panicked" | cut -c1-300
for g in fins lift xsector; do echo "#### gates $g"; tools/release_gates.sh $g; done
