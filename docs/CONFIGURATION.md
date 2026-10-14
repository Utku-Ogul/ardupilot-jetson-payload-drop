# Configuration Reference

All settings live in **`suas/config.py`**, grouped into the sections below. Each value
carries a comment with the reasoning behind it. Nothing else reads settings from anywhere
else, so that file is the whole picture.

After every edit run the consistency checks:

```bash
python3 -m unittest discover -s tests -p test_config.py -v
```

Some values are derived and should not be set directly: `MAVLINK_CONNECT` and
`CAMERA_RTSP_URL` (from `SIM_MODE`), `MODEL_PATH` and `MODEL_ENGINE_PATH` (from
`MODEL_FILE` and `IMGSZ`), `IBVS_FX` (from the camera width and field of view), and the
`IBVS_CENTER_TOL_*` helpers (from `IBVS_CENTER_TOL_LADDER`).

The values below are the ones flown at SUAS 2026 (50 m survey altitude).

---

## `profile`

| Setting | Value | Meaning |
|---|---|---|
| `SIM_MODE` | `False` | `True` for SITL. Selects the MAVLink endpoint and camera URL, lowers the detection thresholds, disables the gimbal gate, makes flyaway/divergence aborts recoverable instead of parking, and **blocks all real payload actuation**. |

---

## `network`

Documentation only, except that `GROUND_STATION_IP` is the default for `STREAM_HOST` and
`UPLOAD_HOST`. The values are the SIYI HM30 subnet we flew on.

| Setting | Value | Meaning |
|---|---|---|
| `COMPANION_IP` | `"192.168.144.30"` | This Jetson. |
| `GROUND_STATION_IP` | `"192.168.144.10"` | Mission Planner PC. Default for `STREAM_HOST` and `UPLOAD_HOST`. |
| `SIM_HOST_IP` | `"192.168.144.50"` | Gazebo + SITL host. |

---

## `flight_controller`

| Setting | Value | Meaning |
|---|---|---|
| `MAVLINK_CONNECT_REAL` | `/dev/serial/by-id/usb-CubePilot_CubeOrange+_…-if00` | Device path or network endpoint used when `SIM_MODE` is `False`. **Change it to your own board's by-id path.** |
| `MAVLINK_CONNECT_SIM` | `udpin:0.0.0.0:14560` | Endpoint used when `SIM_MODE` is `True`. |

Use a `/dev/serial/by-id/` symlink, never a bare `/dev/ttyACM0`: many boards expose two CDC
ports in no guaranteed order, and a brownout can re-enumerate them. Find yours with
`ls -l /dev/serial/by-id/` — the MAVLink interface usually ends in `-if00`.

---

## `camera`

| Setting | Value | Meaning |
|---|---|---|
| `CAMERA_MODE` | `"rtsp"` | `"rtsp"` (H.264) or `"udp"` (RTP/JPEG). |
| `CAMERA_RTSP_URL_REAL` | `rtsp://192.168.144.25:8554/main.264` | Stream URL for real flight (SIYI A8 mini). Check your camera's manual for the port and path. |
| `CAMERA_RTSP_URL_SIM` | `rtsp://192.168.144.50:8554/main.264` | Stream URL in simulation. |
| `CAMERA_UDP_PORT` | `5601` | Port when `CAMERA_MODE` is `"udp"`. |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `1920` / `1080` | Frames are resized to this; the detector and IBVS assume a fixed image centre. |
| `CAMERA_HFOV_DEG` | `81.0` | **Measure this.** Both `IBVS_FX` and `pixel_to_gps` derive from it, so a 10 % error becomes metres of geolocation error at a 50 m survey altitude. Use `tools/calibrate_camera.py`. |
| `CAMERA_FPS` | `30` | Informational. |
| `CAMERA_STALE_S` | `0.5` | A frame older than this is stale; no velocity or actuation is commanded from a stale frame. |

---

## `gimbal`

| Setting | Value | Meaning |
|---|---|---|
| `GIMBAL_MANUAL_NADIR` | `True` | `True` when the gimbal is RC-only with no telemetry to the flight controller. The nadir gate then passes unconditionally and `run.sh` asks the operator to confirm. Set `False` to restore real angle verification. |
| `GIMBAL_NADIR_TOL_DEG` | `5.0` | Accepted deviation from −90° pitch. Only used when `GIMBAL_MANUAL_NADIR` is `False`. |

The entire geolocation model assumes a **nadir-pointing camera**. If it is not pointing
down, coordinates are garbage — which is why this gate is fail-closed.

---

## `detector`

| Setting | Value | Meaning |
|---|---|---|
| `MODEL_FILE` | `"v14_best.pt"` | Weights filename inside `models/`. Not tracked in git — copy it onto the Jetson. |
| `MODEL_ENGINE_PATH` | derived: `models/<stem>_<IMGSZ>.engine` | TensorRT engine file. The name embeds the working size, so a wrong-size engine is never loaded. |
| `USE_TENSORRT` | `True` | Use the FP16 engine when it exists (falls back to the `.pt` with a warning). Build it on the Jetson: `tools/export_tensorrt.py`. |
| `ENGINE_BATCH` | `1` | Fixed engine batch = crops per frame: `1` with tiling off, `9` with 1080p tiling on. |
| `CLASS_NAMES` | `{0: "mannequin", 1: "tent"}` | Class index → label. All downstream logic keys off these **strings**. |
| `DETECT_CONF` | `0.20` (`0.15` in SIM) | Detection floor. Low on purpose: at 50 m a person is ~38 px and confidence drops; false positives are filtered at the candidate stage. |
| `IMGSZ` | `960` | Full-frame inference size. At 50 m a 1.7 m person is ~19 px after letterboxing to 960 (25 px at 1280) — 960 + TensorRT is the speed/recall compromise. |
| `TILE_SIZE` | `640` | Tile size, and the inference size while tiling. |
| `TILE_OVERLAP` | `0.2` | Fractional overlap between tiles. |
| `TILE_ADD_FULL_FRAME` | `True` | Also run the full frame alongside the tiles. |
| `TILING_IN_SURVEY` | `False` | Tiling during the survey. Improves small-target recall but is CPU-bound (~2 FPS). |
| `TILING_IN_IBVS` | `False` | Tiling during visual servoing. The target is large and near, so this costs speed for nothing. |
| `NMS_IOU` | `0.6` | IoU for the class-aware global NMS. |

---

## `target_selection`

| Setting | Value | Meaning |
|---|---|---|
| `DETECT_CONFIRM_SEC` | `3.0` | How long a target must be visible before its position is saved. |
| `DETECT_LOST_GRACE` | `0.5` | A loss longer than this resets the confirmation timer. Short, so single-frame flicker does not. |
| `ACQUIRE_TILT_GATE` | follows `SIM_MODE` | Only sample from level frames. **`True` if you have no gimbal** (tilting shifts the nadir projection); `False` with a gimbal, where filtering on body attitude would wrongly discard valid frames. |
| `ACQUIRE_TILT_MAX_DEG` | `2.5` | Tilt limit when the gate is on (angular: 2.5° ≈ 2.2 m at 50 m). |
| `ACQUIRE_MAX_SPEED` | `6.0` | No samples above this ground speed [m/s]: frame and pose are ~0.45 s apart, so fast transit legs would bias the position. |
| `ACQUIRE_MIN_SAMPLES` | `8` | Minimum samples before a position is trusted. |
| `ACQUIRE_MAX_SAMPLES` | `300` | Sample pool ceiling. |
| `ACQUIRE_REFINE` | `True` | Keep refining targets on later survey passes. |
| `TARGET_CLUSTER_RADIUS_M` | `20.0` | Same-class detections within this radius are one target. Scales with altitude (projection scatter ~17 m at 50 m). Cross-class duplicates are merged within half of it. |
| `TARGET_WAIT_SEC` | `60.0` | Hold time over each target. **Must exceed the payload sequence** (6 s + `MOTOR_RUN_S` = 51 s by default) or the motor is cut mid-run. `tests/test_config.py` verifies this. |
| `TARGET_MAX_ATTEMPTS` | `3` | GPS_GOTO arrival failures per target before it is skipped. Separate from the IBVS ladder. |
| `CANDIDATE_MIN_CONF` | `0.25` (`0.20` in SIM) | Mean confidence required to *select* a candidate. Re-measure on real 50 m imagery when the model changes. |
| `AFTER_VISIT_ACTION` | `"HOVER"` | After all targets: `"HOVER"`, `"RTL"` or `"LAND"`. |

---

## `ibvs`

Visual servoing. Keep `IBVS_MAX_VEL` and `IBVS_SLEW` **below** the autopilot's
`WPNAV_SPEED` and `WPNAV_ACCEL`, so the approach stays the gentler of the two.

| Setting | Value | Meaning |
|---|---|---|
| `IBVS_KV` | `0.4` | Proportional gain [1/s]. |
| `IBVS_MAX_VEL` | `0.8` | Velocity ceiling [m/s]. |
| `IBVS_SLEW` | `0.5` | Acceleration limit [m/s²] — this is where the damping comes from. |
| `IBVS_ERROR_ALPHA` | `0.3` | Error EMA low-pass coefficient. |
| `IBVS_ALT_MIN` / `IBVS_ALT_MAX` | `2.0` / `60.0` | Altitude envelope [m]. Keep `IBVS_ALT_MAX` equal to `PAYLOAD_ALT_MAX`, or there is a band where IBVS centres but release is blocked. |
| `IBVS_CENTER_TOL_LADDER` | `((8.0, 1), (11.0, 1))` | Explicit escalation program `((tolerance_m, attempts), ...)`, tried left to right. The tolerance is the IBVS deadband/latch radius. After the last rung: blind GPS drop. |
| `IBVS_LATCH_SPEED` | `0.15` | Never latch or fire above this commanded speed [m/s]. |
| `IBVS_CENTER_FRAMES` | `15` | Consecutive settled frames required (~1.9 s at the ~8 FPS tracker loop). |
| `IBVS_TIMEOUT_S` | `60.0` | Hard timeout per IBVS attempt. |
| `IBVS_TARGET_FENCE_M` | `25.0` | Geofence measured **from the saved target** — the real flyaway lock. Must exceed the saved-GPS error (~17 m at 50 m). Also the release radius on the normal path. |
| `IBVS_LOSS_GRACE_S` | `7.0` | Detection-loss grace before aborting. |
| `IBVS_DIVERGE_S` | `3.0` | Error must stay above its minimum this long to count as divergence. |
| `IBVS_DIVERGE_MARGIN_M` | `0.5` | Divergence margin floor [m]. |
| `IBVS_DIVERGE_MARGIN_PX` | `50` | Pixel-noise component (altitude scaled). |
| `IBVS_DIVERGE_PITCH_DEG` | `3.5` | Braking-pitch component, resolution independent. |
| `IBVS_SIGN_FWD` | `-1` | Forward axis sign. **Verify in SITL before flying.** |
| `IBVS_SIGN_RIGHT` | `+1` | Right axis sign. **Verify in SITL before flying.** |

**Verifying the signs.** Watch the `IBVS ex=... ey=... -> v_fwd=... v_right=...` log line.
The error must *shrink*. If the target is to the right (`ex > 0`) and `g_err` grows, flip
`IBVS_SIGN_RIGHT`; if the target is below (`ey > 0`) and `g_err` grows, flip
`IBVS_SIGN_FWD`. An inverted sign drives the aircraft *away* until a watchdog stops it.

---

## `goto`

| Setting | Value | Meaning |
|---|---|---|
| `GPS_SETTLE_SEC` | `2.0` | Minimum stabilisation after arrival, before handing over to IBVS. |
| `GOTO_SETTLE_MAX_SPEED` | `0.5` | IBVS also waits until the real ground speed is below this [m/s] — braking from 5 m/s takes ~2 s. |
| `GOTO_RADIUS_M` | `3.0` | Arrival acceptance radius. |
| `GOTO_TIMEOUT_S` | `240.0` | Arrival timeout → the target is deferred. Keep ≥ 1.5 × (farthest target / `WPNAV_SPEED`). |
| `GOTO_MIN_ALT_M` | `1.0` | Altitude floor — blocks an accidental land command. |

---

## `payload`

| Setting | Value | Meaning |
|---|---|---|
| `GPIO_ENABLED` | `True` | Master switch for all GPIO output. |
| `PAYLOAD_ENABLED` | `True` | Arms the payload actuation. |
| `PAYLOAD_REVERSE` | `False` | Globally invert the motor direction. |
| `PAYLOAD_ALT_MIN` / `PAYLOAD_ALT_MAX` | `2.0` / `60.0` | Release altitude envelope [m]. |
| `FIRE_MIN_CONF` | `0.25` | Minimum confidence of the centred detection (identity gate). Keep ≤ `CANDIDATE_MIN_CONF`. |
| `FIRE_TILT_MAX_DEG` | `35.0` | Sanity ceiling only. In wind the aircraft legitimately leans to hold position, and a strict gate would block the drop forever. Keep this **above** the autopilot's `ANGLE_MAX`. |
| `FIRE_SETTLE_MIN_S` | `1.5` | Minimum settled time over the target before releasing. |
| `FIRE_SETTLE_MAX_S` | `8.0` | Retry-window ceiling. Gates are retried at ~2 Hz throughout, so one bad frame does not lose the target. |
| `GPS_DROP_FALLBACK` | `True` | Blind release at the saved GPS if IBVS cannot centre. |
| `GPS_DROP_FIRE_RADIUS_M` | `5.0` | Acceptance radius for the blind release. |
| `GPS_DROP_MAX_SPEED` | `0.5` | No release above this ground speed [m/s]. |
| `GPS_DROP_SETTLE_MAX_S` | `6.0` | Maximum extra wait for the aircraft to stop. |

> **Set `GPIO_ENABLED` and `PAYLOAD_ENABLED` to `False`** for every flight until the
> payload has been bench tested with `tools/payload_test.py`. When both are `True` and
> `SIM_MODE` is `False`, the startup banner warns that the payload is armed.

---

## `payload_hardware`

Pins are BOARD (physical header) numbering. `tests/test_config.py` detects collisions.

| Setting | Value | Meaning |
|---|---|---|
| `PAYLOAD_TARGETS` | mannequin: pin 12 / B / 10° · tent: pin 7 / A / 10° | Per class: `servo_pin`, `motor` (`"A"`/`"B"`), `release_angle`, `direction` (`1`/`-1`). Class names must match `CLASS_NAMES`. |
| `SERVO_NEUTRAL_ANGLE` | `90.0` | Pre-drop position, shared by all targets. Also the `--init` position. |
| `SERVO_RANGE_DEG` | `180.0` | Physical servo travel. |
| `SERVO_MIN_US` / `SERVO_MAX_US` | `500` / `2500` | Pulse widths for 0° and full travel. |
| `SERVO_MOVE_S` | `1.0` | Time to travel from neutral to the release angle. |
| `SERVO_STEP_DT` | `0.02` | Delay between motion steps (smaller = smoother). |
| `SERVO_SETTLE_S` | `1.5` | `--init` settling time. |
| `MOTOR_A_PINS` | `[32, 33]` | `[forward, reverse]`. |
| `MOTOR_B_PINS` | `[29, 31]` | `[forward, reverse]`. |
| `MOTOR_PWM_FREQ` | `100` | Software PWM frequency [Hz]. |
| `MOTOR_SPEED` | `100` | Duty at full power [%]. |
| `MOTOR_RUN_S` | `45.0` | Run time including the ramp [s]. Change `TARGET_WAIT_SEC` with it. |
| `MOTOR_RAMP_S` | `1.5` | Soft-start ramp — prevents the inrush that browns out the Jetson. |
| `WAIT_BEFORE_SERVO` | `2.0` | Pause before the servo moves. |
| `WAIT_AFTER_SERVO` | `2.0` | Pause before the motor runs. |
| `BUZZER_PIN` | `13` | |
| `LED_STATUS_PIN` | `15` | |
| `AUX_PINS` | `[16, 18]` | Spare outputs. |

The full sequence takes `SERVO_MOVE_S + WAIT_BEFORE_SERVO + SERVO_MOVE_S + WAIT_AFTER_SERVO +
MOTOR_RUN_S` (51 s by default: settle at neutral, pause, sweep, pause, motor), which must fit
inside `TARGET_WAIT_SEC` with margin.

---

## `mapping`

| Setting | Value | Meaning |
|---|---|---|
| `PHOTO_DIR` | `"survey_photos"` | Output directory (gitignored). |
| `PHOTO_START_WP` | `0` | Mapping start sequence: photo capture **and target recording** start when the mission reaches it (`0` = from the start of AUTO). Usually set by the startup prompt or `--photo-start-wp`. |
| `STABILIZE_PITCH_MAX` / `STABILIZE_ROLL_MAX` | `3.0` | Tilt limit before a photo. **Photos only** — the airdrop uses `FIRE_TILT_MAX_DEG`. |
| `STABILIZE_TIMEOUT` | `3.0` | Give up waiting and capture anyway. Keep the mission's `NAV_DELAY` ≥ this + 1 s. |
| `UPLOAD_ENABLED` | `False` | Copy each photo to the ground station over scp. |
| `UPLOAD_HOST` | `GROUND_STATION_IP` | |
| `UPLOAD_USER` | `"user"` | |
| `UPLOAD_REMOTE_DIR` | `/home/user/survey_photos` | |

---

## `streaming`

Live H.264 stream of the annotated frames. See [GROUND_STATION.md](GROUND_STATION.md).

| Setting | Value | Meaning |
|---|---|---|
| `STREAM_ENABLED` | `False` | Costs nothing when off. |
| `STREAM_HOST` | `GROUND_STATION_IP` | Where to send it. |
| `STREAM_PORT` | `5600` | QGC / Mission Planner default video port. |
| `STREAM_WIDTH` / `STREAM_HEIGHT` | `960` / `540` | Downscaled from the source. |
| `STREAM_FPS` | `15` | Nominal; the real rate follows the tracker loop. |
| `STREAM_BITRATE` | `2_000_000` | bps, constant bitrate. Lower it if the raw camera feed shares the link. |
| `STREAM_IDR_SEC` | `1.0` | Keyframe interval — a short GOP recovers faster on lossy UDP. |

Requires launching through `run.sh`; otherwise it disables itself silently and logs one
warning. Streaming errors can never affect telemetry, IBVS or the payload path.

---

## `safety`

| Setting | Value | Meaning |
|---|---|---|
| `POSE_STALE_S` | `1.0` | Position/attitude older than this is stale. |
| `GPS_EKF_STALE_S` | `1.5` | Separate, wider window for the 2 Hz GPS and EKF streams. `POSE_STALE_S` is far too tight for a 0.5 s interval — one dropped packet used to flap the gates and cause spurious hard stops. Keep it **below** `LINK_TIMEOUT_S`. |
| `POSE_MIN_SATS` | `6` | Minimum satellites for `have_fix`. |
| `LINK_TIMEOUT_S` | `3.0` | No autopilot heartbeat for this long = link lost. |
| `SHUTDOWN_SAFE_MODE` | `"BRAKE"` | Park mode on exit, applied **only** if the companion still owns GUIDED. `None` leaves the mode alone. |

---

## What `tests/test_config.py` checks

- `TARGET_WAIT_SEC` at least 5 s longer than the payload sequence (would cut the motor mid-run)
- Altitude envelopes that do not line up between IBVS, payload and goto
- A centring ladder that is not widening, or whose widest rung exceeds the target fence
- `GPS_EKF_STALE_S` at or above `LINK_TIMEOUT_S`
- Confidence thresholds ordered so a target can be selected but never released
- The TensorRT engine name / batch not matching the working size and tiling
- GPIO pin collisions between servos, motors, the buzzer, the LED and aux outputs
- A payload target class that is not one of the model classes (it would never actuate),
  invalid motor channel / release angle / direction, or two targets sharing a servo or motor
