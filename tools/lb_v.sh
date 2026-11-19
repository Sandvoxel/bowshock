# variant comparison of the compile peak and the steady bytes (DESIGN3D item 49): fast process sampler, ANF sector sub 2 and the ANF half domain
export BOWSHOCK_ENGINE=3d
M="$CARGO_TARGET_DIR"/release/examples/mem_items
if [ -f tools/memprof/run.sh ] && grep -q "luma-memstats" tools/memprof/run.sh; then
  # item 48's tree (Luma 7fafed6 + memstats patch)
  RUN="tools/memprof/run.sh"
else
  cargo build --release -p bowshock-validate --example mem_items 2>&1 | tail -1
  RUN="$M"
fi
for spec in "--chain anf@2 --reclaim --run-on 300" "--chain anfhalf@2 --reclaim --run-on 300"; do
  echo "=== $spec"
  tools/gpuq/gpumem --label "$spec" -- $RUN $spec 2>&1 | grep -E "^MEMITEM (==|.*(compile|run_on|wall_forces))|^MEM "
done
