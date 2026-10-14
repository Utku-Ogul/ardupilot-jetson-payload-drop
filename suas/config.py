"""
All settings, in one place.

Edit this file to adapt the system; nothing else reads settings from anywhere else.

OPERATING ALTITUDE: values are tuned for a 50 m survey altitude (SUAS 2026 requires
>= 150 ft AGL). The code has no flight-altitude constant of its own -- the altitude comes
from the uploaded mission. Metric thresholds (tolerances, cluster radius, fences) scale
with altitude, angular ones (tilt) scale inversely. Keep IBVS_ALT_MAX == PAYLOAD_ALT_MAX.
"""
import math
import os

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# =============================================================================
# ENVIRONMENT PROFILE -- simulation vs real vehicle
# Flip ONLY this switch when moving between the two. The camera source, MAVLink endpoint
# and several safety behaviours follow from it.
# =============================================================================
SIM_MODE = False                    # Gazebo SITL -> True | real vehicle -> False

# --- Network (documentation only -- not read by the flight code) --------------
COMPANION_IP = "192.168.144.30"       # this Jetson
SIM_HOST_IP = "192.168.144.50"        # Gazebo + SITL + simulated camera stream (8554)
GROUND_STATION_IP = "192.168.144.10"  # Mission Planner

# --- MAVLink connection ------------------------------------------------------
# SIM : SITL sends UDP to the Jetson (.30:14560); listen on every interface.
# REAL: flight controller over USB (ArduPilot SERIAL0, MAVLink2; baud is meaningless on USB).
#   Do NOT use a bare /dev/ttyACM0: the Cube exposes TWO CDC ports (MAVLink + SLCAN) in no
#   guaranteed order, and a brownout can re-enumerate them. The udev by-id symlink is
#   stable -- find it once with:  ls -l /dev/serial/by-id/   (-if00 = MAVLink, -if02 = SLCAN)
#   Also remove ModemManager (it grabs ttyACM devices) and add the user to 'dialout'.
MAVLINK_CONNECT_REAL = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_340037000B51323031393637-if00"
MAVLINK_CONNECT_SIM = "udpin:0.0.0.0:14560"
MAVLINK_CONNECT = MAVLINK_CONNECT_SIM if SIM_MODE else MAVLINK_CONNECT_REAL

# --- Live annotated video stream to the ground station -----------------------
# Publishes the detection overlay (boxes drawn) as RTP/UDP H.264 through the SIYI HM30 link
# using the Orin hardware encoder -- a SEPARATE stream from the camera's raw RTSP feed.
# Watch it in Mission Planner / QGC (UDP H.264, port 5600). See docs/GROUND_STATION.md.
# Must be launched via run.sh (GStreamer-enabled OpenCV); under a bare `python3` the stream
# disables itself silently (flight unaffected). Costs nothing when disabled.
STREAM_ENABLED = False
STREAM_HOST = GROUND_STATION_IP     # SIYI reserves .20 for its GCS -- use .20 if your PC is .20
STREAM_PORT = 5600                  # QGC / Mission Planner default video port
STREAM_WIDTH = 960                  # downscaled from 1080p; 640x360 if the raw feed shares the link
STREAM_HEIGHT = 540
STREAM_FPS = 15                     # nominal; the real push rate follows the tracker loop (~8 FPS)
STREAM_BITRATE = 2_000_000          # bps CBR; 800_000 when the raw camera feed shares the link
STREAM_IDR_SEC = 1.0                # keyframe interval [s] -- a short GOP recovers fast on UDP

# --- Camera ------------------------------------------------------------------
CAMERA_MODE = "rtsp"                # "rtsp" (H.264) | "udp" (Gazebo RTP/JPEG)
CAMERA_UDP_PORT = 5601
CAMERA_FPS = 30
# The simulated camera is matched to the SIYI A8 mini optics (1920x1080, 81 deg HFOV), so the
# simulation stays representative and IBVS_FX / pixel_to_gps agree in both profiles.
CAMERA_RTSP_URL_SIM = "rtsp://192.168.144.50:8554/main.264"   # Gazebo PC stream
CAMERA_RTSP_URL_REAL = "rtsp://192.168.144.25:8554/main.264"  # SIYI A8 mini
CAMERA_RTSP_URL = CAMERA_RTSP_URL_SIM if SIM_MODE else CAMERA_RTSP_URL_REAL
CAMERA_WIDTH = 1920                 # A8 mini 1080p (1x zoom = widest)
CAMERA_HEIGHT = 1080
CAMERA_HFOV_DEG = 81.0              # A8 mini 1x horizontal FOV (manufacturer spec; IBVS is
                                    # closed-loop and insensitive to fx, only the saved GPS
                                    # scales with it). Measure with tools/calibrate_camera.py.
CAMERA_STALE_S = 0.5                # a frame older than this is stale -> no commands from it

# =============================================================================
# DETECTOR -- YOLO, two classes: mannequin (0) and tent (1)
# =============================================================================
# Model: YOLO11m-P2 (extra P2 head for small targets), fine-tuned at imgsz=640, 160 epochs.
# The weights are NOT tracked in git (see models/README.md) -- copy them onto the Jetson.
MODEL_DIR = os.path.join(_REPO_ROOT, "models")
MODEL_FILE = "v14_best.pt"
MODEL_PATH = os.path.join(MODEL_DIR, MODEL_FILE)
# The tracker's class logic keys off these label STRINGS; the detector warns if the loaded
# weights use different names.
CLASS_NAMES = {0: "mannequin", 1: "tent"}

TILE_SIZE = 640                     # tile size [px]; also the imgsz while tiling
IMGSZ = 960                         # full-frame (tiling off) inference size. At 50 m a 1.7 m
                                    # person is ~38 px in the 1080p frame, ~19 px after
                                    # letterboxing to 960 (25 px at 1280). 960 + TensorRT is
                                    # the speed/recall compromise; re-measure on 50 m imagery.
# A TensorRT engine has a FIXED input shape, so the working imgsz is part of its file name
# (v14_best_960.engine). Changing IMGSZ therefore looks for the matching engine and falls
# back to the .pt (with a warning, ~2x slower) if it does not exist -- a wrong-size engine is
# never loaded silently. Build it ON the Jetson: python3 tools/export_tensorrt.py
# NOTE: with tiling ON the working size is TILE_SIZE -- rebuild the engine at that size.
USE_TENSORRT = True
MODEL_ENGINE_PATH = os.path.join(MODEL_DIR, "%s_%d.engine"
                                 % (os.path.splitext(MODEL_FILE)[0], IMGSZ))
# Fixed engine batch = crops per frame: 1 with tiling off, 9 with 1080p tiling on.
# Must match the engine (tools/export_tensorrt.py reads this), or tiles are silently dropped.
ENGINE_BATCH = 1
TILE_OVERLAP = 0.2                  # fractional overlap between tiles
TILE_ADD_FULL_FRAME = True          # also run the full frame alongside the tiles
NMS_IOU = 0.6                       # IoU for the class-aware global NMS
# Detection floor. SIM: low (weak synthetic mannequin). REAL: at 50 m the target is ~38 px
# and confidence drops; false positives are filtered at the candidate stage.
DETECT_CONF = 0.15 if SIM_MODE else 0.20
# Tiling is OFF: it is CPU-bound on the Orin (~2 FPS), not real-time. Full frame runs ~8 FPS.
# If recall at 50 m proves too low, survey tiling is the first card to play
# (TILING_IN_SURVEY=True + ENGINE_BATCH=9 + a 640 batch-9 engine): tiles keep native resolution.
TILING_IN_SURVEY = False
TILING_IN_IBVS = False              # during IBVS the target is large and near -> speed

# --- Detection confirmation ---------------------------------------------------
DETECT_CONFIRM_SEC = 3.0            # visible this long -> the first target is saved
DETECT_LOST_GRACE = 0.5             # a longer loss resets the timer (absorbs flicker)

# --- Target position estimation ----------------------------------------------
# Multi-sample + median + 2-sigma outlier rejection is ALWAYS active.
# ATTITUDE GATE: only without a gimbal (simulation) -- a tilted body shifts the nadir
# projection there. With the real gimbal the camera stays at nadir, and gating on body
# attitude would only discard valid frames.
ACQUIRE_TILT_GATE = SIM_MODE
ACQUIRE_TILT_MAX_DEG = 2.5          # (gate on) skip samples above this tilt; angular, so it
                                    # shrinks with altitude: 2.5 deg ~= 2.2 m at 50 m
ACQUIRE_MAX_SPEED = 6.0             # m/s no samples above this ground speed. Frame and pose
                                    # are ~0.45 s apart (RTSP + inference), so the error grows
                                    # with speed (1.4 m at 3 m/s, 5.4 m at 12 m/s) and fast
                                    # transit legs would drag the median.
ACQUIRE_MIN_SAMPLES = 8             # samples required before a candidate can be selected
ACQUIRE_MAX_SAMPLES = 300           # sample pool ceiling
ACQUIRE_REFINE = True               # keep refining in TARGET_SAVED on later passes

# --- Multi-candidate selection: visit the MOST-SEEN target per class ----------
TARGET_CLUSTER_RADIUS_M = 20.0      # same-class projections within this are one target.
                                    # Projection scatter grows with altitude (~17 m at 50 m);
                                    # too small splits one target into under-sampled pieces.
                                    # Cross-class duplicates are merged within HALF of it.
TARGET_WAIT_SEC = 60.0              # hold over each target [s]. Must EXCEED the payload
                                    # sequence (6 s + MOTOR_RUN_S = 51 s), otherwise the hold
                                    # ends and the motor is cut mid-run. Do not go below 55 s.
TARGET_MAX_ATTEMPTS = 3             # GPS_GOTO arrival failures per target (separate from the
                                    # IBVS ladder). The target is deferred to the back of the
                                    # queue; past this limit it is skipped.
GOTO_TIMEOUT_S = 240.0              # GPS_GOTO arrival timeout [s] -> target deferred.
                                    # Range = GOTO_TIMEOUT_S x WPNAV_SPEED; keep
                                    # GOTO_TIMEOUT_S >= 1.5 x (farthest target / WPNAV_SPEED).
AFTER_VISIT_ACTION = "HOVER"        # after all targets: "HOVER" | "RTL" | "LAND"
# Mean confidence required to SELECT a candidate (recall is DETECT_CONF's job).
CANDIDATE_MIN_CONF = 0.20 if SIM_MODE else 0.25
FIRE_MIN_CONF = 0.25                # release: minimum confidence of the centred detection.
                                    # Keep <= CANDIDATE_MIN_CONF, otherwise a target can be
                                    # accepted and centred but never released.
GIMBAL_NADIR_TOL_DEG = 5.0          # real gimbal: pitch ~= -90 +/- this counts as nadir

# --- Gimbal: MANUAL NADIR ------------------------------------------------------
# The SIYI A8 is controlled only by RC through the HM30 (no UART to the flight controller), so
# ArduPilot cannot read its angle. In SIYI "lock mode" the gimbal holds the commanded earth-
# frame angle even when the vehicle tilts. While True, gimbal_nadir_ok is unconditionally
# True (operator-confirmed nadir) and main.py skips the nadir command/verify loop; every
# other safety gate is retained. THE OPERATOR must point it fully down before takeoff and
# confirm the straight-down view. Set False if gimbal telemetry is ever wired to the FC.
GIMBAL_MANUAL_NADIR = True

# --- Blind GPS drop (last resort) --------------------------------------------
# If IBVS cannot centre a target on ANY rung of the ladder but the vehicle is HEALTHY, fly to
# the target's SAVED GPS position and release there instead of skipping it (a drop within
# 15 m still scores; the target was confirmed during the survey).
# SAFETY INVARIANT: the blind drop skips the identity and IBVS-centring gates but keeps EVERY
# vehicle-safety gate (link / GUIDED / armed / attitude / altitude envelope / ground speed ~0
# / close to the saved GPS). Genuine danger still results in HOVERING, never a blind drop.
GPS_DROP_FALLBACK = True
GPS_DROP_FIRE_RADIUS_M = 5.0        # m   blind-drop acceptance radius
GPS_DROP_MAX_SPEED = 0.5            # m/s no blind release above this ground speed
GPS_DROP_SETTLE_MAX_S = 6.0         # s   max extra wait for the vehicle to stop after arrival

# =============================================================================
# IBVS -- visual servoing
# Pixel error -> METRIC ground error (altitude and focal length) -> body-frame velocity.
# P-only with velocity-slew damping, no derivative. Commands use MAV_FRAME_BODY_OFFSET_NED,
# which removes the yaw-induced flyaway class. IBVS clamps its own command to IBVS_MAX_VEL,
# so the autopilot's (faster) WPNAV_SPEED does not make it more aggressive.
# =============================================================================
IBVS_FX = (CAMERA_WIDTH / 2) / math.tan(math.radians(CAMERA_HFOV_DEG) / 2)  # px (derived)
IBVS_KV = 0.4                       # 1/s   proportional gain (tau = 2.5 s, well damped)
IBVS_MAX_VEL = 0.8                  # m/s   command ceiling
IBVS_SLEW = 0.5                     # m/s^2 acceleration limit (deliberately gentle)
IBVS_ERROR_ALPHA = 0.3              # error EMA low-pass coefficient
IBVS_ALT_MIN = 2.0                  # m     GSD floor (prevents gain blow-up)
IBVS_ALT_MAX = 60.0                 # m     gain ceiling AND the IBVS altitude envelope (outside
                                    #       it IBVS aborts). Keep == PAYLOAD_ALT_MAX.
IBVS_LATCH_SPEED = 0.15             # m/s   no latch/release while still moving
IBVS_CENTER_FRAMES = 15             # consecutive settled frames (~1.9 s at ~8 FPS)

# --- Escalating centring tolerance: an EXPLICIT ladder ------------------------
# ((tolerance_m, attempts), ...) tried left to right; each rung has its own attempt budget.
# When the last rung fails, the blind GPS drop follows (GPS_DROP_FALLBACK). The tolerance is
# the IBVS deadband/latch radius. Measured closing speed is ~0.135 m/s, so at 50 m (arrival
# error ~17 m) an 8 m rung needs ~67 s and an 11 m rung ~44 s against IBVS_TIMEOUT_S = 60 s.
# The 11 m rung plus ~5 m ballistic scatter is ~16 m -- the closest to the 15 m scoring radius.
IBVS_CENTER_TOL_LADDER = ((8.0, 1), (11.0, 1))
IBVS_CENTER_TOL_M = IBVS_CENTER_TOL_LADDER[0][0]        # first rung (ibvs.reset() default)
IBVS_CENTER_TOL_MAX_M = IBVS_CENTER_TOL_LADDER[-1][0]   # last rung (info / logs)
IBVS_CENTER_TOL_ATTEMPTS_TOTAL = sum(n for _, n in IBVS_CENTER_TOL_LADDER)

# --- Safety watchdogs (any of these -> zero velocity + abort) -----------------
IBVS_TIMEOUT_S = 60.0               # s  hard timeout per IBVS attempt
IBVS_TARGET_FENCE_M = 25.0          # m  horizontal fence measured from the SAVED TARGET -- the
                                    #    real flyaway lock (a centring vehicle approaches the
                                    #    target, a flyaway departs). Must exceed the saved-GPS
                                    #    error (~17 m at 50 m). Also the release radius on the
                                    #    normal path (the position proof there is ibvs.centered).
IBVS_LOSS_GRACE_S = 7.0             # s  detection-loss grace (zero velocity meanwhile)
IBVS_DIVERGE_S = 3.0                # s  error above its minimum this long = divergence
IBVS_DIVERGE_MARGIN_M = 0.5         # m  divergence margin floor
IBVS_DIVERGE_MARGIN_PX = 50         # px pixel-noise component (scaled by altitude)
IBVS_DIVERGE_PITCH_DEG = 3.5        # deg braking-pitch component: alt * tan(this)

# --- Camera axis -> command axis signs (verify in SITL; flip here, not in code) ---
IBVS_SIGN_FWD = -1                  # forward (nose) axis
IBVS_SIGN_RIGHT = +1                # right axis

# --- GPS goto -----------------------------------------------------------------
GPS_SETTLE_SEC = 2.0                # minimum settle time after arrival before IBVS [s]
GOTO_RADIUS_M = 3.0                 # arrival acceptance radius [m]
GOTO_SETTLE_MAX_SPEED = 0.5         # m/s soft IBVS entry: the real ground speed must also be
                                    #     below this (braking from 5 m/s takes ~2 s / 5 m).
                                    #     Only delays, never commands motion.
GOTO_MIN_ALT_M = 1.0                # goto altitude floor (blocks an accidental land command)

# --- Stabilisation before a mapping photo -------------------------------------
STABILIZE_PITCH_MAX = 3.0           # deg -- PHOTOS ONLY; the release does not use this
STABILIZE_ROLL_MAX = 3.0
STABILIZE_TIMEOUT = 3.0             # s  give up and capture anyway. Keep the mission's
                                    #    NAV_DELAY >= this + 1 s (4 s), or the photo is taken
                                    #    while leaving the waypoint.

# --- Payload release tilt gate (separate from photos -- wind tolerant) -------
# In wind the vehicle leans to hold position; a strict tilt gate would block the drop. The
# release is approved by the IBVS latch (or a settled hover on the blind path) and the speed
# gates; this is only a sanity ceiling against impossible attitudes (glitch / tumble). The
# frozen-attitude protection stays in pose.fire_tilt_ok. Keep it ABOVE ANGLE_MAX.
FIRE_TILT_MAX_DEG = 35.0

# --- Mapping / photos ---------------------------------------------------------
PHOTO_DIR = "survey_photos"
# Mapping start sequence. Photo capture AND target recording (mannequin/tent candidates)
# start when the AUTO mission reaches this item; before it (takeoff, waypoint lap, transit)
# the detector runs but nothing is recorded. 0 = from the start of AUTO. Normally chosen in
# the pre-flight prompt or with --photo-start-wp.
PHOTO_START_WP = 0

# --- Photo upload to the ground station (scp) --------------------------------
UPLOAD_ENABLED = False
UPLOAD_HOST = GROUND_STATION_IP
UPLOAD_USER = "user"
UPLOAD_REMOTE_DIR = "/home/user/survey_photos"

# =============================================================================
# PAYLOAD
# On AT_TARGET (or a blind GPS drop), after IBVS centring and every safety gate,
# suas/payload/drop_sequence.py runs as a SEPARATE process, selected by target class.
# SAFETY: GPIO_ENABLED and PAYLOAD_ENABLED only when the payload is wired, bench tested and
# INTENTIONALLY armed for this flight. Set both False for a survey-only flight.
# =============================================================================
GPIO_ENABLED = True                 # master switch for all GPIO output
PAYLOAD_ENABLED = True              # arm the payload actuation
PAYLOAD_REVERSE = False             # invert both motors (wiring reversed)

# Release gates -- ALL must pass before anything actuates
PAYLOAD_ALT_MIN = 2.0               # m minimum release altitude
PAYLOAD_ALT_MAX = 60.0              # m maximum release altitude (== IBVS_ALT_MAX)

# --- After IBVS: SETTLE, THEN RELEASE ----------------------------------------
# AT_TARGET first holds the vehicle over the target with zero velocity (minimum time plus
# speed gates), then triggers the payload. The gates are retried at ~2 Hz in this window, so
# one bad frame (confidence/EKF/link flap) cannot lose the target.
FIRE_SETTLE_MIN_S = 1.5             # s minimum settled time before release
FIRE_SETTLE_MAX_S = 8.0             # s retry-window ceiling (then move on without releasing)

# =============================================================================
# PAYLOAD HARDWARE -- pins, angles and timings (BOARD pin numbering)
# Read by suas/payload/drop_sequence.py and suas/payload/indicators.py.
# =============================================================================
# Target class -> servo pin, motor channel, release angle, motor direction (+1 fwd / -1 rev;
# PAYLOAD_REVERSE multiplies on top). Both mechanisms open the same way: 90 -> 10 deg.
PAYLOAD_TARGETS = {
    "mannequin": {"servo_pin": 12, "motor": "B", "release_angle": 10.0, "direction": 1},
    "tent":      {"servo_pin": 7,  "motor": "A", "release_angle": 10.0, "direction": 1},
}
SERVO_NEUTRAL_ANGLE = 90.0          # deg pre-drop position, shared (also the --init position)
SERVO_RANGE_DEG = 180.0             # deg SG90 physical travel
SERVO_MIN_US = 500                  # us  pulse for 0 deg
SERVO_MAX_US = 2500                 # us  pulse for SERVO_RANGE_DEG
SERVO_MOVE_S = 1.0                  # s   neutral -> release travel time
SERVO_STEP_DT = 0.02                # s   delay between motion steps
SERVO_SETTLE_S = 1.5                # s   (--init) let both servos settle

MOTOR_A_PINS = [32, 33]             # HW-231 channel A [forward, reverse]
MOTOR_B_PINS = [29, 31]             # HW-231 channel B [forward, reverse]
MOTOR_PWM_FREQ = 100                # Hz  software PWM
MOTOR_SPEED = 100                   # %   full power
MOTOR_RUN_S = 45.0                  # s   run time including the ramp. Sequence = 6 s + this;
                                    #     change TARGET_WAIT_SEC with it.
MOTOR_RAMP_S = 1.5                  # s   soft-start ramp (prevents inrush / brownout)
WAIT_BEFORE_SERVO = 2.0             # s   pause before the servo moves
WAIT_AFTER_SERVO = 2.0              # s   pause before the motor runs

BUZZER_PIN = 13
LED_STATUS_PIN = 15
AUX_PINS = [16, 18]

# =============================================================================
# SAFETY -- telemetry validity and shutdown
# =============================================================================
POSE_STALE_S = 1.0                  # s position/attitude older than this is stale
GPS_EKF_STALE_S = 1.5               # s separate, wider window for the 2 Hz GPS_RAW_INT and
                                    #   EKF_STATUS_REPORT streams (1.0 s flapped on one lost
                                    #   packet). Stays below LINK_TIMEOUT_S.
POSE_MIN_SATS = 6                   # minimum satellites (have_fix gate)
LINK_TIMEOUT_S = 3.0                # s no autopilot heartbeat for this long = link lost
SHUTDOWN_SAFE_MODE = "BRAKE"        # park mode on exit, only if the companion still owns
                                    # GUIDED (None = leave the mode alone)
