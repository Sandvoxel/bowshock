#!/usr/bin/env bash
# Diagnostic runner (DESIGN3D item 48): builds an example against a copy of the pinned Luma rev (7fafed6, from the cargo git checkout) patched with
# tools/memprof/luma-memstats.patch (env-gated `LUMA_MEMSTATS=<file>`: live intervals and sizes of every buffer of each recording; no behaviour change),
# runs it, and prints the stats file.   Usage: tools/memprof/run.sh <example args...>   e.g.  --case sector --sub 3
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
copy=${MEMPROF_LUMA:-$HOME/.cache/bowshock-memprof-luma}
rm -rf "$copy"; mkdir -p "$copy"
cp -r "$(ls -d "$HOME"/.cargo/git/checkouts/luma-*/7fafed6 | head -1)"/. "$copy"/
patch -p1 -d "$copy" < "$here/luma-memstats.patch" >/dev/null
stats=${MEMPROF_STATS:-$(mktemp /tmp/memstats.XXXXXX)}
: > "$stats"
cd "$root"
cp "$here/mem_items.rs" crates/bowshock-validate/examples/mem_items.rs
LUMA_MEMSTATS="$stats" cargo run --release \
  --config "patch.\"https://github.com/flipnoteman/Luma.git\".luma.path=\"$copy\"" \
  -p bowshock-validate --example "${MEMPROF_EXAMPLE:-mem_items}" -- "$@"
echo "=== LUMA_MEMSTATS $stats"
cat "$stats"
