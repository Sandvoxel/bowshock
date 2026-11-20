#!/bin/bash
# DESIGN3D item 66: the Rabia fixture at Mach 0.8 / 1.3 / 2.3 on the default grid and with `DIAG_LE_ROWS=1` (rounded fins keep the root's first chord cell down the span)
R=rabia-short-fin-can
for res in draft coarse; do
  tools/sweptle.sh $res 0.8,1.3,2.3 $R
  tools/sweptle.sh $res 0.8,1.3,2.3 $R DIAG_LE_ROWS=1
done
