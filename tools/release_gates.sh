#!/bin/bash
# the release-candidate gate table, one group per gpuq job (DESIGN3D item 41): `tools/release_gates.sh <group>` prints the GATE / INFO / KNOWN lines of the group
#   wedge     G-V1 .. G-V5, G-P1 on the GPU wedge runner (BOWSHOCK_WEDGE_BACKEND=gpu); G-V1 / G-V2 / G-P1 print INFO naming the test that covers them
#   fins      G-F1, G-F4, G-S and G-F5s (the Finner CD0 on the Coarse sector) on the Coarse tier (GPU)
#   lift      G-F5, G-L1 on the Lift tier (GPU)
#   xlift     G-X1 (Arcas), G-X2 (Lewis sounding-rocket stage, NASA TN D-3182), G-X3 (AEDC GRM): CN_alpha and x_cp on the Lift half domain (GPU), DESIGN3D item 55
#   xsector   G-X4 (Arcas long), G-X5 (Lewis stage), G-X6 (GRM): the axial force without the base on the Coarse sector (GPU)
#   gpucases  the bowshock-gpu-cases suite (summary lines)
#   fingates  G-F2, G-F3 (Euler fins on the stack grids) on the GPU runner
#   cpu       G-W1, G-W4, G-J on the CPU
set -u
export BOWSHOCK_WEDGE_BACKEND=${BOWSHOCK_WEDGE_BACKEND:-gpu}
t() { cargo test --release -p bowshock-core --test "$@" 2>&1 | grep -E "^(GATE|INFO)|test result|panicked|^test .* (ok|FAILED)$|warning: unused" ; }
case "${1:-}" in
  wedge)
    t wedge_gates -- --ignored --nocapture --test-threads=1 gate_v1 gate_v2 gate_v3 gate_v4_example_table gate_v4_arcas_fins_off gate_v5 gate_p1 ;;
  fingates)
    t fin_gates -- --ignored --nocapture --test-threads=1 gate_f2 gate_f3 ;;
  fins)
    export BOWSHOCK_FIN_TIER=coarse
    t fin_lift_gates -- --ignored --nocapture --test-threads=1 gate_f1 gate_f4 gate_s gate_f5s ;;
  lift)
    export BOWSHOCK_FIN_TIER=lift
    t fin_lift_gates -- --ignored --nocapture --test-threads=1 gate_f5_anf gate_l1 ;;
  xlift)
    export BOWSHOCK_FIN_TIER=lift
    t rocket_gates -- --ignored --nocapture --test-threads=1 gate_x1 gate_x2 gate_x3 ;;
  xsector)
    export BOWSHOCK_FIN_TIER=coarse
    t rocket_gates -- --ignored --nocapture --test-threads=1 gate_x4 gate_x5 gate_x6 ;;
  gpucases)
    cargo test --release -p bowshock-gpu-cases 2>&1 | grep -E "test result|FAILED|panicked|^test .* FAILED" ;;
  cpu)
    unset BOWSHOCK_WEDGE_BACKEND
    t wedge_gates -- --ignored --nocapture --test-threads=1 gate_w1 gate_w4
    t wedge_gates -- --ignored --nocapture --test-threads=1 gate_j ;;
  *) echo "usage: $0 wedge|fins|lift|xlift|xsector|gpucases|cpu"; exit 2 ;;
esac
