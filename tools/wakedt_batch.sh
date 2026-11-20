#!/bin/bash
# wakedt_batch.sh: DESIGN3D item 60, the supersonic points (one queue job, one build): the GPU agreement test of `ImplicitParams::dt_k_cap`, then Draft sectors at Mach 1.3 and 3.0 (the
# example rocket) and the Rabia short-fin-can at Mach 2.3: the plain sum at the case's cfl_max (100) against the cap 1 at cfl_max 25.
cargo test --release -p bowshock-gpu-cases --test euler3d tiny_anf_dt_k_cap -- --nocapture 2>&1 | grep -E "^M |^\||test result|panicked|targets"
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
export MAXIT=3000
for m in 1.3 3.0; do
  tools/wakedt.sh draft $m inf
  CFLS=25 tools/wakedt.sh draft $m 1
done
RABIA=1 tools/wakedt.sh draft 2.3 inf
RABIA=1 CFLS=25 tools/wakedt.sh draft 2.3 1
