#!/bin/bash
# usage: variant mach   (variant: A free-flight Re, B Re_L 4e6, C Re_L 4e6 + sting 0.63)
v=$1; m=$2
extra=""; [ $v != A ] && extra="--re-l 4e6"; [ $v = C ] && extra="$extra --sting 0.63"
for ph in between inplane; do
  echo "== $v M$m $ph"
  cargo run --release -q -p bowshock-validate --example incidence_rans -- --body anf --mach $m --alphas 0,1,2,5 --phase $ph --max-iter 6000 $extra 2>&1 | grep -v "^warning"
done
