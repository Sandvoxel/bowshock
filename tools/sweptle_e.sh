#!/bin/bash
# where the state collapses: snapshots of the extrema every 50 iterations
tools/sweptle.sh coarse 0.8 byrum DIAG_SNAP=300 DIAG_SNAP_EVERY=50 DIAG_MAXIT=800
tools/sweptle.sh coarse 0.8 rabia-short-fin-can DIAG_SNAP=100 DIAG_SNAP_EVERY=50 DIAG_MAXIT=1000
tools/sweptle.sh draft 0.5 cancer-descending DIAG_SNAP=100 DIAG_SNAP_EVERY=250 DIAG_MAXIT=3000
