#!/bin/bash
# gridconv_chord.sh MACH...   (DESIGN3D item 47) the production grid s = 1 with 64 and 128 cells over the fin root chord (32 in production), drift tolerance 5e-4: where the leading-edge face converges
for M in "$@"; do
  for n in 64 128; do
    echo "DIAG chord $n M$M"; EXTRA="--drift-tol 5e-4 --chord $n" tools/gridconv.sh $M 1
  done
done
