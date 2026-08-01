#!/bin/bash
# -----------------------------------------------------------------------------
# Launch the SUAS vision system with the correct OpenCV build.
#
# NO SUDO NEEDED: add your user to the 'gpio' group and Jetson.GPIO will use the
# /dev/gpiochip (libgpiod) backend, which reaches the servo/motor GPIO without root.
# Do NOT use sudo: as root, pip --user packages in ~/.local are invisible, so
# pymavlink/cv2/numpy fail with ModuleNotFoundError.
#
# Why PYTHONPATH: the pip `opencv-python` wheel (built WITHOUT GStreamer) shadows the
# JetPack OpenCV. The camera reader opens UDP/RTSP streams through a GStreamer pipeline,
# so the JetPack build (/usr/lib/python3.10/dist-packages) is put first. Nothing needs to
# be uninstalled -- the camera works and pymavlink/numpy stay where they are.
#
# At startup, if GPIO_ENABLED is true, both payload servos are moved to their neutral
# (closed) angle.
#
# Usage:
#   ./run.sh                  # with the camera preview window
#   ./run.sh --no-display     # SSH / headless
#   ./run.sh --no-mapping     # disable photo capture
#   (every suas.main argument is passed straight through)
# -----------------------------------------------------------------------------
cd "$(dirname "$0")" || exit 1
export PYTHONPATH="/usr/lib/python3.10/dist-packages:${PYTHONPATH}"

# Stop the RTSP FFMPEG fallback from blocking forever in cap.read() on a half-open TCP
# connection (camera or Wi-Fi loss). Without this, read() never returns, the camera
# reader's reconnect stalls on the same thread and the stream dies permanently (the
# 'stale' safety gate still holds the vehicle, but the mission is over). A 2s socket
# timeout makes a stall return ok=False, which triggers the existing reconnect.
# (The GStreamer hardware path has its own latency handling; this only affects FFMPEG.)
export OPENCV_FFMPEG_CAPTURE_OPTIONS="rtsp_transport;tcp|stimeout;2000000"

# --- Move both payload servos to neutral (only when GPIO is enabled) ----------
GPIO_ON="$(python3 -c 'from suas.config import GPIO_ENABLED; print(1 if GPIO_ENABLED else 0)' 2>/dev/null)"
if [ "$GPIO_ON" = "1" ]; then
    echo ">> Startup: moving both servos to their neutral angle..."
    python3 -m suas.payload.drop_sequence --init \
        || echo ">> WARNING: servo init failed -- continuing."
fi

# --- Manual gimbal nadir confirmation ----------------------------------------
# In manual-nadir mode the gimbal is not wired to the flight controller, so the code
# CANNOT read its angle -- verification is the operator's responsibility. The gimbal must
# be pointed fully down by RC before takeoff (lock mode holds it). Ask here so it is not
# forgotten, and refuse to start without confirmation.
MANUAL_NADIR="$(python3 -c 'from suas.config import GIMBAL_MANUAL_NADIR, SIM_MODE; print(1 if (GIMBAL_MANUAL_NADIR and not SIM_MODE) else 0)' 2>/dev/null)"
if [ "$MANUAL_NADIR" = "1" ]; then
    echo ""
    echo "=================================================================="
    echo ">> MANUAL GIMBAL NADIR CONFIRMATION (GIMBAL_MANUAL_NADIR=True)"
    echo ">> The code cannot read the gimbal angle (no gimbal telemetry to the FC)."
    echo ">> BEFORE takeoff:"
    echo ">>   1) Point the gimbal fully DOWN (nadir, -90) with the RC transmitter"
    echo ">>   2) CONFIRM the straight-down view on the video feed"
    echo ">>   3) Do not touch that control for the rest of the mission"
    echo "=================================================================="
    if [ -t 0 ]; then
        read -r -p ">> Is the gimbal fully DOWN (nadir) and confirmed? Type 'y' to continue: " ans
        case "$ans" in
            y|Y|yes|YES) echo ">> Confirmed -- starting." ;;
            *) echo ">> ABORTED: no gimbal confirmation. Point it to nadir and rerun."; exit 1 ;;
        esac
    else
        echo ">> WARNING: no TTY (headless) -- interactive confirmation skipped."
        echo ">> MAKE SURE the gimbal is manually at nadir!"
    fi
fi

exec python3 -m suas.main "$@"
