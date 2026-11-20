#!/bin/bash
# wakedt_batch.sh: the GPU agreement test of `ImplicitParams::dt_k_cap` (DESIGN3D item 60), one queue job. The studies of the item ran through `tools/wakedt.sh TIER MACH CAP...`.
cargo test --release -p bowshock-gpu-cases --test euler3d tiny_anf_dt_k_cap -- --nocapture 2>&1 | grep -E "^M |^\||test result|panicked|targets"
