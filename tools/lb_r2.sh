bash tools/lb_v.sh
export BOWSHOCK_ENGINE=3d
echo "=== full solve"
tools/gpuq/gpumem --label full-remote4 -- cargo nextest run --release -p bowshock engine3d_finned_timing --run-ignored only --no-capture 2>&1 | grep -E "SUM|PERF|^MEM|PASS|FAIL|\| 3D|\| runs|\| build"
