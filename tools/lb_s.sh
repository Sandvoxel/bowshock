export BOWSHOCK_ENGINE=3d
cargo build --release -p bowshock-validate --example mem_items 2>&1 | tail -1
M="$CARGO_TARGET_DIR"/release/examples/mem_items
for spec in "--chain anf@1 --reclaim --run-on 300" "--chain anf@3 --reclaim --run-on 300" "--chain anf@4 --reclaim --run-on 300" "--chain anf@2 --ext --reclaim --run-on 300" "--chain anf@2 --euler --reclaim --run-on 300" "--chain anf@1 --euler --reclaim --run-on 300" "--chain gf1@2 --reclaim --run-on 300" "--chain anfhalf@1 --reclaim --run-on 300" "--chain anfhalf@3 --reclaim --run-on 300" "--chain anfhalf@2 --euler --reclaim --run-on 300"; do
  echo "=== $spec"
  tools/gpuq/gpumem --label "$spec" -- $M $spec 2>&1 | grep -E "^MEMITEM (==|.*(compile|run_on|wall_forces))|^MEM |panicked|Error"
done
