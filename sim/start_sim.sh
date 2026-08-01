#!/bin/bash
# -----------------------------------------------------------------------------
# Start ArduPilot SITL together with a Gazebo world, for testing without a vehicle.
#
# The companion computer connects to the UDP endpoint below; set "SIM_MODE": true in
# config.json so it listens on udpin:0.0.0.0:14560.
#
# Expected layout (siblings of this repository):
#   ../ardupilot          ArduPilot source tree (sim_vehicle.py)
#   ../ardupilot_gazebo   ardupilot_gazebo plugin (built)
#
# Usage:
#   ./start_sim.sh [world_name]
# -----------------------------------------------------------------------------
set -e

WORLD="${1:-runway}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
ARDU_DIR="${ARDUPILOT_DIR:-$ROOT_DIR/../ardupilot}"
GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$ROOT_DIR/../ardupilot_gazebo}"
PARAM_FILE="$SCRIPT_DIR/params/sim.parm"

# Where SITL should forward MAVLink. Override with environment variables if your
# addresses differ.
GCS_ENDPOINT="${GCS_ENDPOINT:-udp:127.0.0.1:14550}"
COMPANION_ENDPOINT="${COMPANION_ENDPOINT:-udp:127.0.0.1:14560}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$GAZEBO_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$GAZEBO_DIR/models:$GAZEBO_DIR/worlds:$GZ_SIM_RESOURCE_PATH"

echo "=== SUAS simulation ==="
echo "World      : $WORLD"
echo "ArduPilot  : $ARDU_DIR"
echo "GCS        : $GCS_ENDPOINT"
echo "Companion  : $COMPANION_ENDPOINT"

echo "[1/2] Starting Gazebo..."
gz sim -r "${WORLD}.sdf" &
GZ_PID=$!
sleep 4

echo "[2/2] Starting ArduCopter SITL..."
cd "$ARDU_DIR"
sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    --out="$GCS_ENDPOINT" \
    --out="$COMPANION_ENDPOINT" \
    --add-param-file="$PARAM_FILE" \
    --map \
    --console &
SITL_PID=$!

echo ""
echo "Running. Press Ctrl+C to stop both."
trap 'kill $GZ_PID $SITL_PID 2>/dev/null' EXIT INT TERM
wait
