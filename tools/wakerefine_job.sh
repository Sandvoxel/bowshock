#!/bin/bash
# DESIGN3D item 65: one gpuq job of the wake / boattail refinement study (`crates/bowshock-core/tests/wakerefine.rs`): `WR_MACHS`, `WR_VARIANTS` in the environment
export BOWSHOCK_WEDGE_BACKEND=${BOWSHOCK_WEDGE_BACKEND:-gpu}
cargo test --release -p bowshock-core --test wakerefine -- --ignored --nocapture "$@" 2>&1 | grep -E "^WR|panicked|^error|test result"
