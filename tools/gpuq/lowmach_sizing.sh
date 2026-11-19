#!/usr/bin/env bash
# lowmach_sizing.sh MACH...: the example rocket's sector at each MACH on the Coarse tier, sized for Mach 3 (the app's shared grid) and for its own Mach number (per-point), at refinement
# 0.75 / 1 / 1.333 (`Case3::with_refinement`): which sizing moves less under refinement and where each converges (DESIGN3D item 57). One `GRID` line (cells per direction, the domain) and
# `FINSCALE` multiplies the first cell of the fin stack. `SIZINGS` (default `3.0 own`) and `REFS` (default `0.75 1 1.333`) choose the sizings (Mach numbers, `own` = the node's) and refinements. One `TABLE` line (CD and its parts, y+ max, iterations) per run; roughness 60 um as the app (7.87e-4 calibres).
set -u
KS=7.87e-4
bin=${CARGO_TARGET_DIR:-target}/release/examples/finned_table
cargo build --release -q -p bowshock-validate --example finned_table || exit 1
for m in "$@"; do
  for sizing in ${SIZINGS:-3.0 own}; do
    [ "$sizing" = own ] && sizing=$m
    for r in ${REFS:-0.75 1 1.333}; do
      echo "=== M$m sized-for $sizing refine $r"
      "$bin" --tier coarse --mach "$m" --mach-max "$sizing" --refine "$r" --ks-body $KS --ks-fin $KS ${FINSCALE:+--fin-y1-scale $FINSCALE} --grid-only 2>&1 | grep -E "^GRID|rror"
      "$bin" --tier coarse --mach "$m" --mach-max "$sizing" --refine "$r" --ks-body $KS --ks-fin $KS ${FINSCALE:+--fin-y1-scale $FINSCALE} 2>&1 | grep -E "^TABLE |rror"
    done
  done
done
