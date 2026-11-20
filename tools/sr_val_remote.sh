export FLIGHTVAL_SOLVE=1 FLIGHTVAL_OUT=$PWD/out
mkdir -p out
echo "#### legacy six"; cargo nextest run --release -p bowshock flight_validation --run-ignored only --no-capture 2>&1 | grep -E "solved|^\|" | cut -c1-300
echo "#### 3d three"
BOWSHOCK_ENGINE=3d FLIGHTVAL_CASES=cancer-descending,byrum,ion-drive cargo nextest run --release -p bowshock flight_validation --run-ignored only --no-capture 2>&1 | grep -E "POINT|solved|^\|" | cut -c1-300
