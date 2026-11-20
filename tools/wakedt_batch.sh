#!/bin/bash
# wakedt_batch.sh: the DESIGN3D item 60 study in one queue job (one build): the GPU agreement test of `ImplicitParams::dt_k_cap`, then Draft sectors at Mach 0.5 and 2.3 (the example
# rocket) and the Rabia short-fin-can at Mach 0.8, finite caps of the k rate at the case's cfl_max (100, the rollback controller on).
cargo test --release -p bowshock-gpu-cases --test euler3d tiny_anf_dt_k_cap -- --nocapture 2>&1 | grep -E "^M |^\||test result|panicked|targets"
cargo build --release -p bowshock-validate --example finned_table 2>&1 | tail -1
export MAXIT=3000
tools/wakedt.sh draft 0.5 2 4 16
tools/wakedt.sh draft 2.3 2 4 16
RABIA=1 HIST=250 tools/wakedt.sh draft 0.8 4 16
