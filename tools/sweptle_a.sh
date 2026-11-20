#!/bin/bash
R=rabia-short-fin-can
tools/sweptle.sh draft 0.8,1.3,2.3 $R BOWSHOCK_LE_FLOOR=0
tools/sweptle.sh draft 0.8,1.3,2.3 $R BOWSHOCK_LE_FLOOR=0.5
tools/sweptle.sh coarse 1.3,2.3,0.8 $R BOWSHOCK_LE_FLOOR=0
