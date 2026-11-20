#!/bin/bash
# Draft matrix of the two grid changes (rows: per-row chord nodes of a rounded section; ramp: the fade of the bend beside the edges)
R=rabia-short-fin-can
M=0.8,1.3,2.3
tools/sweptle.sh draft $M $R BOWSHOCK_LE_ROWS=0 BOWSHOCK_RAMP_OFF=1
tools/sweptle.sh draft $M $R BOWSHOCK_LE_ROWS=1 BOWSHOCK_RAMP_OFF=1
tools/sweptle.sh draft $M $R BOWSHOCK_LE_ROWS=0
tools/sweptle.sh draft $M $R BOWSHOCK_LE_ROWS=1 BOWSHOCK_LE_FLOOR=0.5
tools/sweptle.sh draft 0.8 $R BOWSHOCK_LE_ROWS=1 DIAG_SNAP=100 DIAG_SNAP_EVERY=50 DIAG_MAXIT=800
tools/sweptle.sh draft 1.3 $R BOWSHOCK_LE_ROWS=1 DIAG_SNAP=100 DIAG_SNAP_EVERY=100 DIAG_MAXIT=2800
