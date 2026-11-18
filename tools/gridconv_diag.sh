#!/bin/bash
# gridconv_diag.sh MACH...   (DESIGN3D item 47: sensitivities on the production grid s = 1, drift tolerance 5e-4 unless noted)
# for every Mach: the drift tolerance 1e-4 (iteration convergence), the Fine tier (fin sides y+ 1 instead of 15, Case3 sector), 4 cells across the fin thickness.
for M in "$@"; do
  echo "DIAG drift-tol 1e-4 M$M"; EXTRA="--drift-tol 1e-4" tools/gridconv.sh $M 1
  echo "DIAG fine tier M$M"; TIER=fine EXTRA="--drift-tol 5e-4" tools/gridconv.sh $M 1
  echo "DIAG fin-cells 4 M$M"; EXTRA="--drift-tol 5e-4 --fin-cells 4" tools/gridconv.sh $M 1
done
