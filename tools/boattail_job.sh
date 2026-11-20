#!/bin/bash
# DESIGN3D item 64: one gpuq job of the boattail diagnosis (`crates/bowshock-core/tests/boattail.rs`): `tools/boattail_job.sh bt1|bt2` with `BT_MACHS`, `BT_VARIANTS`, `BT_STEPS`, `BT_EVERY` in the environment
export BOWSHOCK_WEDGE_BACKEND=${BOWSHOCK_WEDGE_BACKEND:-gpu}
cargo test --release -p bowshock-core --test boattail -- --ignored --nocapture "$@" 2>&1 | grep -E "^BT|panicked|^error|test result"
