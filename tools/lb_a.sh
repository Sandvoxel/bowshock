export BOWSHOCK_ENGINE=3d
cargo build --release -p bowshock-validate --example mem_items 2>&1 | tail -1
M="$CARGO_TARGET_DIR"/release/examples/mem_items
for spec in "--chain anf@2 --reclaim --run-on 300" "--chain anf@1 --reclaim --run-on 300" "--chain anf@3 --reclaim --run-on 300" "--chain gf1@2 --reclaim --run-on 300" "--chain anfhalf@2 --reclaim --run-on 300" "--chain anfhalf@2 --reclaim --run-on 300 --ext" "--chain wedge,anfwedge --reclaim --run-on 300" "--chain wedge --ext --reclaim --run-on 300" "--chain wedge,anf,anf,anfhalf,anfhalf,anf --run-on 100"; do
  echo "=== mem_items $spec"
  tools/gpuq/gpumem --label "$spec" -- $M $spec 2>&1 | grep -E "^MEM|panicked|Error"
done
echo "=== snapshot test"
cargo nextest run --release -p bowshock-gpu-cases a_host_snapshot --no-capture 2>&1 | tail -8
echo "=== full solve"
tools/gpuq/gpumem --label full-local -- cargo nextest run --release -p bowshock engine3d_finned_timing --run-ignored only --no-capture 2>&1 | grep -E "SUM|PERF|^MEM|PASS|FAIL"
