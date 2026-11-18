#!/bin/bash
# the release-candidate gate table, one group per gpuq job (DESIGN3D item 41): `tools/release_gates.sh <group>` prints the GATE / INFO / KNOWN lines of the group
#   wedge     G-V1 .. G-V5, G-P1 on the GPU wedge runner (BOWSHOCK_WEDGE_BACKEND=gpu)
#   fins      G-F1, G-F4, G-S on the Coarse tier (GPU)
#   lift      G-F5, G-L1 on the Lift tier (GPU)
#   gpucases  the bowshock-gpu-cases suite (summary lines)
#   cpu       G-W1, G-W4, G-J, G-F2, G-F3 on the CPU
set -u
export BOWSHOCK_WEDGE_BACKEND=${BOWSHOCK_WEDGE_BACKEND:-gpu}
t() { cargo test --release -p bowshock-core --test "$@" 2>&1 | grep -E "^(GATE|INFO)|test result|panicked|^test .* (ok|FAILED)$|warning: unused" ; }
case "${1:-}" in
  wedge)
    t wedge_gates -- --ignored --nocapture --test-threads=1 gate_v1 gate_v2 gate_v3 gate_v4_example_table gate_v4_arcas_fins_off gate_v5 gate_p1 ;;
  fins)
    export BOWSHOCK_FIN_TIER=coarse
    t fin_lift_gates -- --ignored --nocapture --test-threads=1 gate_f1 gate_f4 gate_s ;;
  lift)
    export BOWSHOCK_FIN_TIER=lift
    t fin_lift_gates -- --ignored --nocapture --test-threads=1 gate_f5 gate_l1 ;;
  gpucases)
    cargo test --release -p bowshock-gpu-cases 2>&1 | grep -E "test result|FAILED|panicked|^test .* FAILED" ;;
  cpu)
    unset BOWSHOCK_WEDGE_BACKEND
    t wedge_gates -- --ignored --nocapture --test-threads=1 gate_w1 gate_w4
    t fin_gates -- --ignored --nocapture --test-threads=1 gate_f2 gate_f3
    t wedge_gates -- --ignored --nocapture --test-threads=1 gate_j ;;
  *) echo "usage: $0 wedge|fins|lift|gpucases|cpu"; exit 2 ;;
esac
