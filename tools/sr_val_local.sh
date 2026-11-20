export BOWSHOCK_WEDGE_BACKEND=gpu
echo "#### gates lift"; tools/release_gates.sh lift 2>&1 | grep -E "G-F5|G-L1|test result|FAILED" | cut -c1-230
echo "#### gates fins"; tools/release_gates.sh fins 2>&1 | grep -E "^GATE|test result|FAILED" | cut -c1-230
export BOWSHOCK_ENGINE=3d FLIGHTVAL_SOLVE=1 FLIGHTVAL_OUT=$PWD/out
mkdir -p out
FLIGHTVAL_CASES=rabia-short-fin-can,torrent,dont-debate-this cargo nextest run --release -p bowshock flight_validation --run-ignored only --no-capture 2>&1 | grep -E "POINT|solved|^\|" | cut -c1-300
