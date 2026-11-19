export BOWSHOCK_ENGINE=3d
echo "=== full solve"
tools/gpuq/gpumem --label full-remote2 -- cargo nextest run --release -p bowshock engine3d_finned_timing --run-ignored only --no-capture 2>&1 | grep -E "SUM|PERF|^MEM|PASS|FAIL"
echo "=== gate wedge"
bash tools/release_gates.sh wedge
echo "=== gate gpucases"
bash tools/release_gates.sh gpucases
