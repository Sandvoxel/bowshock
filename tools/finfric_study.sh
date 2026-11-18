#!/bin/bash
# why the Finner's fin friction is below the flat plate where G-F1's is not (DESIGN3D item 41): Coarse sector, Mach 2, the Finner's flow, fin variants
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
BIN=$(cargo test --release -p bowshock-core --test arcas_study --no-run --message-format=json 2>/dev/null | grep -o '"executable":"[^"]*arcas_study-[0-9a-f]*"' | head -1 | cut -d'"' -f4)
run() { echo "== $*"; env STUDY_BODY=anf STUDY_MACH=2.0 STUDY_FINS=on "$@" "$BIN" --ignored --nocapture 2>&1 | grep -E "^STUDY |STUDY-FINFRICTION|panicked"; }
run                                                     # the Finner: chord 1, span 1, t 0.08, sourced section
run STUDY_FIN_DW=1 STUDY_FIN_T=0.02                     # chord 1, double wedge, t/c 2 %
run STUDY_FIN_T=0.02                                    # chord 1, sourced section, t 0.02
run STUDY_FIN_CHORD=0.5 STUDY_FIN_DW=1 STUDY_FIN_T=0.01 # G-F1's fin (chord = span = 0.5, t/c 2 %) at the Finner's Reynolds number
run STUDY_FIN_CHORD=0.5 STUDY_FIN_T=0.04                # chord 0.5, the sourced section, t/c 8 %
