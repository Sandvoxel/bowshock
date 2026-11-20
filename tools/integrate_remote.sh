#!/bin/bash
# integration verification, remote 3080 Laptop: wedge, gpucases, xlift
set -u
for g in wedge gpucases xlift; do echo "#### gates $g"; tools/release_gates.sh $g; done
