#!/usr/bin/env bash
# lift_sizing.sh MACH...: the example rocket's Lift-tier half domain at 2 degrees (the app's angle node: three fins, roughness 60 um) at each MACH with the walls sized for Mach 3 (the app's
# shared sizing), for its own Mach number and for max(own, 1), at refinement 0.75 / 1 / 1.333 (DESIGN3D item 57). One `CELLS` and one `INCIDENCE {json}` line per run (CN, x_cp, CA, y+ max,
# wall seconds). `SIZINGS` (default `3.0 own 1.0`) and `REFS` (default `0.75 1 1.333`) choose the runs; a sizing that equals an earlier one for the Mach number is skipped.
set -u
bin=${CARGO_TARGET_DIR:-target}/release/examples/incidence_rans
cargo build --release -q -p bowshock-validate --example incidence_rans || exit 1
for m in "$@"; do
  seen=" "
  for sizing in ${SIZINGS:-3.0 own 1.0}; do
    case "$sizing" in
      own) mm=$m ;;
      3.0) mm=3.0 ;;
      *) mm=$(awk -v f="$sizing" -v m="$m" 'BEGIN { print (m > f) ? m : f }') ;;   # a floor: max(own, floor)
    esac
    case "$seen" in *" $mm "*) continue ;; esac
    seen="$seen$mm "
    for r in ${REFS:-0.75 1 1.333}; do
      echo "=== M$m alpha 2 sized-for $mm refine $r"
      "$bin" --body example --sector-fins example3 --tier lift --mach "$m" --alpha 2 --mach-max "$mm" --refine "$r" 2>&1 | grep -E "^CELLS|^INCIDENCE|rror|NONFINITE"
    done
  done
done
