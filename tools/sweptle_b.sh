#!/bin/bash
R=rabia-short-fin-can
tools/sweptle.sh coarse 1.3,2.3,0.8 $R BOWSHOCK_LE_ROWS=1
tools/sweptle.sh coarse 1.3,2.3,0.8 $R BOWSHOCK_LE_ROWS=0
tools/sweptle.sh draft 0.5,1.1,1.5 cancer-descending BOWSHOCK_LE_ROWS=1
tools/sweptle.sh coarse 0.5,0.8 byrum BOWSHOCK_LE_ROWS=1
