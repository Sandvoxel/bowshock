#!/bin/bash
# device memory of the sector grids by tier and precision: the peak of the process around one run of tests/arcas_study (gpumem)
export BOWSHOCK_WEDGE_BACKEND=gpu
cd crates/bowshock-core
BIN=$(cargo test --release -p bowshock-core --test arcas_study --features gpu --no-run --message-format=json 2>/dev/null | grep -o '"executable":"[^"]*arcas_study-[0-9a-f]*"' | head -1 | cut -d'"' -f4)
[ -x "$BIN" ] || { echo "no test binary"; exit 1; }
cd ../..
mem() { label=$1; shift; env "$@" tools/gpuq/gpumem --label "$label" -- "$BIN" --ignored --nocapture 2>&1 | grep -E "^MEM|^STUDY"; }
for prec in f32 ext; do
  p=f32; [ $prec = ext ] && p=ext
  mem "anf-wedge-coarse-$prec" BOWSHOCK_WEDGE_PRECISION=$p STUDY_BODY=anf STUDY_MACH=2.375 STUDY_FINS=off STUDY_TIER=coarse
  mem "anf-sector-draft-$prec" BOWSHOCK_WEDGE_PRECISION=$p STUDY_BODY=anf STUDY_MACH=2.375 STUDY_FINS=on STUDY_TIER=draft
  mem "anf-sector-coarse-$prec" BOWSHOCK_WEDGE_PRECISION=$p STUDY_BODY=anf STUDY_MACH=2.375 STUDY_FINS=on STUDY_TIER=coarse
  mem "anf-sector-euler-$prec" BOWSHOCK_WEDGE_PRECISION=$p STUDY_BODY=anf STUDY_MACH=2.375 STUDY_FINS=on STUDY_TIER=euler
done
