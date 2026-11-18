#!/bin/bash
# finned_rough.sh gpu|cpu: the G-F1 sector (Coarse) with and without wall roughness (ks_body 2e-5, ks_fin 1e-4 nose diameters, the Arcas values), 400 iterations from the free
# stream, at Mach 2 and 0.5: fin and body friction on the GPU (f32; run it through gpuq) or on the f64 CPU solver (no GPU: run it directly, about 5 minutes per point).
# A port difference shows as GPU != CPU at the same iteration, a model effect as both agreeing.
BIN="${CARGO_TARGET_DIR:-target}/release/examples/finned_table"
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
mode=${1:-gpu}
for m in 2.0 0.5; do
  for ks in "0 0" "2e-5 1e-4"; do
    set -- $ks
    if [ "$mode" = cpu ]; then
      "$BIN" --gf1 --tier coarse --mach $m --ks-body $1 --ks-fin $2 --max-iter 400 --cpu 2>&1 | grep -E "^TABLE-CPU" | sed "s/^/[CPU ks_body=$1 ks_fin=$2] /"
    else
      "$BIN" --gf1 --tier coarse --mach $m --ks-body $1 --ks-fin $2 --max-iter 400 2>&1 | grep -E "^TABLE |^GF1-RATIO" | sed "s/^/[GPU ks_body=$1 ks_fin=$2] /"
    fi
  done
done
