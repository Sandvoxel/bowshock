#!/bin/bash
# the first chord cell of the round edge, kept down the span (rows), as a fraction of the radius
R=rabia-short-fin-can
for c in 6 8 12; do tools/sweptle.sh draft 0.8,1.3,2.3 $R BOWSHOCK_LE_ROWS=1 BOWSHOCK_LE_CELLS=$c; done
tools/sweptle.sh draft 0.8,1.3,2.3 $R BOWSHOCK_LE_ROWS=0 BOWSHOCK_LE_CELLS=8
