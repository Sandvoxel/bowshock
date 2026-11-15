#!/usr/bin/env bash
# Compares Bowshock with OpenRocket on the same rockets (see docs/openrocket-comparison.md).
#
#   tools/compare-openrocket/run.sh [workdir]
#
# 1. copies the rockets into <workdir>/rockets/ (the user's own designs are only ever read, never modified),
# 2. runs tools/openrocket-harness on each (OpenRocket's own numbers + thrust curves -> <workdir>/ref/),
# 3. runs `cargo run --example compare_openrocket`, whose Markdown goes to <workdir>/comparison.md.
#
# Environment: OPENROCKET_JAR (default: the jar inside /Applications/OpenRocket.app), SPACEPORT_FINAL and SPACEPORT_SUBSCALE
# (paths of the user's .ork files).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WORK="${1:-/tmp/bowshock-openrocket}"
JAR="${OPENROCKET_JAR:-/Applications/OpenRocket.app/Contents/Resources/app/OpenRocket.jar}"
FINAL="${SPACEPORT_FINAL:-$HOME/Desktop/Spaceport Design (FINAL NEEDS REVIEW).ork}"
SUBSCALE="${SPACEPORT_SUBSCALE:-$HOME/Downloads/old/SpaceportSubscale Rev 4.ork}"
mkdir -p "$WORK/rockets" "$WORK/ref"

# OpenRocket's bundled examples live inside the jar.
unzip -o -q "$JAR" 'datafiles/examples/*' -d "$WORK/jar"
EX="$WORK/jar/datafiles/examples"
cp "$EX/A simple model rocket.ork" "$WORK/rockets/simple_model_rocket.ork"
cp "$EX/Dual parachute deployment.ork" "$WORK/rockets/dual_parachute.ork"
cp "$EX/Two-stage rocket.ork" "$WORK/rockets/two_stage.ork"
# Pods (1-, 2- and 4-fin sets on pods: OpenRocket's roll-angle dependence) and tube fins.
cp "$EX/Pods--airframes and winglets.ork" "$WORK/rockets/pods_winglets.ork"
cp "$EX/Pods--powered with recovery deployment.ork" "$WORK/rockets/pods_powered.ork"
cp "$EX/Tube fin rocket.ork" "$WORK/rockets/tube_fin_rocket.ork"
# The same small rocket with a conical nose: OpenRocket's ogive wave-drag quirk (see the report) does not apply to it.
unzip -p "$EX/A simple model rocket.ork" rocket.ork | sed 's|<shape>ogive</shape>|<shape>conical</shape>|' > "$WORK/rockets/simple_conical_nose.ork"
[ -f "$FINAL" ] && cp "$FINAL" "$WORK/rockets/spaceport_final.ork"
[ -f "$SUBSCALE" ] && cp "$SUBSCALE" "$WORK/rockets/spaceport_subscale_rev4.ork"

for f in "$WORK"/rockets/*.ork; do
  "$ROOT/tools/openrocket-harness/run.sh" "$f" "$WORK/ref" >/dev/null
done

ARGS=()
for f in "$WORK"/rockets/*.ork; do ARGS+=("$(basename "$f" .ork)=$f"); done
(cd "$ROOT" && cargo run -q --release -p bowshock-sim --example compare_openrocket -- "$WORK/ref" "${ARGS[@]}") | tee "$WORK/comparison.md"
