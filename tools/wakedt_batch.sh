#!/bin/bash
# wakedt_batch.sh: the whole DESIGN3D item 60 study in one queue job (one build): the GPU agreement test of DtMode::SumCapK, the Draft example sector at Mach 0.5 and 2.3
# (cfl_max scan of the capped step, the ramp factor), the Rabia short-fin-can sector at Mach 0.8 on the Draft tier (sum against capped, time-step diagnostic).
cargo test --release -p bowshock-gpu-cases --test euler3d tiny_anf_sum_capk -- --nocapture 2>&1 | grep -E "^M |^\||test result|panicked|targets"
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
export DIAG=0
CFLS="50 25" tools/wakedt.sh draft 0.5 sum-capk
FLAGS="--growth 1.05" CFLS="25" tools/wakedt.sh draft 2.3 sum-capk
FLAGS="--growth 1.1" CFLS="25" tools/wakedt.sh draft 2.3 sum-capk
BIN="$CARGO_TARGET_DIR/release/examples/finned_table"
for spec in "sum default" "sum-capk 25"; do
  set -- $spec
  fl=""; [ "$2" != default ] && fl="--cfl-max $2"
  echo "=== rabia draft M0.8 dt-mode $1 cfl $2"
  tools/gpuq/gpumem --label "wakedt-rabia-$1-$2" -- "$BIN" --rabia --tier draft --mach 0.8 --dt-mode "$1" $fl --dt-diag --history-every 250 --max-iter 8000 2>&1 | grep -E "^TABLE|^DTDIAG|^HIST|^MEM|panicked|Error"
done
