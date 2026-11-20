#!/bin/bash
# periodstop confirming run: the example rocket's full solve with the default drift rule and with the period-aware one (BOWSHOCK_DRIFT_PERIODS=1), twice back to back
for v in default periods default periods; do
  echo "#### variant $v load $(cat /proc/loadavg)"
  if [ $v = periods ]; then export BOWSHOCK_DRIFT_PERIODS=1; else unset BOWSHOCK_DRIFT_PERIODS; fi
  env BOWSHOCK_ENGINE=3d cargo nextest run --release -p bowshock engine3d_finned_timing --run-ignored only --no-capture 2>&1 | grep -E "^DRAG M|^LIFT M|^\| (2D|3D)|^SUM|^PERF|wall \(app"
done
