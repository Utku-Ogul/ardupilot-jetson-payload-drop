# Configuration Reference

All settings live in **`config.json`** in the repository root. Edit that file — not the
Python.

```bash
python3 tools/show_config.py            # what resolved, and from where
python3 tools/show_config.py --changed  # only your overrides
python3 tools/show_config.py --check    # validation only (exit 1 on error)
```

## How it works

`suas/config.py` holds every default and the reasoning behind it. `config.json` overrides
those defaults. Section names exist only for readability — the loader flattens them, so
moving a key between sections changes nothing.

- A key that is **absent** or **`null`** keeps its default. You can leave keys in place as
  documentation without them taking effect.
- Keys and sections starting with `_` are comments.
- An **unrecognised key is reported as an error**, not silently ignored. A misspelled
  safety setting that quietly kept its default is exactly the failure mode this project
  cannot afford.

Some values are derived and cannot be set directly: `MAVLINK_CONNECT` (from `SIM_MODE`),
`CAMERA_RTSP_URL` (from `SIM_MODE`), `MODEL_PATH`, and `IBVS_FX` (from the camera width
and field of view).

---

## `profile`

| Key | Default | Meaning |
|---|---|---|
| `SIM_MODE` | `false` | `true` for SITL. Selects the MAVLink endpoint and camera URL, lowers the detection thresholds, disables the gimbal gate, makes flyaway/divergence aborts recoverable instead of parking, and **blocks all real payload actuation**. |

---

## `network`

Informational, except where noted. Only the camera URL and stream host are used directly.
All three ship **blank** — fill in the addresses your own network uses.

| Key | Default | Meaning |
|---|---|---|
| `COMPANION_IP` | `""` | This machine. Documentation only. |
| `GROUND_STATION_IP` | `""` | Default for `STREAM_HOST` and `UPLOAD_HOST`. |
| `SIM_HOST_IP` | `""` | Default host in `CAMERA_RTSP_URL_SIM`. |

---

## `flight_controller`

| Key | Default | Meaning |
|---|---|---|
| `MAVLINK_CONNECT_REAL` | `""` — **required** | Device path or network endpoint used when `SIM_MODE` is false. |
| `MAVLINK_CONNECT_SIM` | `udpin:0.0.0.0:14560` | Endpoint used when `SIM_MODE` is true. |

Use a `/dev/serial/by-id/` symlink, never a bare `/dev/ttyACM0`: many boards expose two CDC
ports in no guaranteed order, and a brownout can re-enumerate them. Find yours with
`ls -l /dev/serial/by-id/` — the MAVLink interface usually ends in `-if00`.

---

## `camera`

| Key | Default | Meaning |
|---|---|---|
| `CAMERA_MODE` | `"rtsp"` | `"rtsp"` (H.264) or `"udp"` (RTP/JPEG). |
| `CAMERA_RTSP_URL_REAL` | `""` — **required** in `rtsp` mode | Stream URL for real flight, e.g. `rtsp://<camera-ip>:8554/main.264`. Check your camera's manual for the port and path. |
| `CAMERA_RTSP_URL_SIM` | derived from `SIM_HOST_IP` | Stream URL in simulation. |
| `CAMERA_UDP_PORT` | `5601` | Port when `CAMERA_MODE` is `"udp"`. |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `1920` / `1080` | Frames are resized to this; the detector and IBVS assume a fixed image centre. |
| `CAMERA_HFOV_DEG` | `81.0` | **Measure this.** Both `IBVS_FX` and `pixel_to_gps` derive from it, so a 10 % error becomes metres of geolocation error at 20 m altitude. Use `tools/calibrate_camera.py`. |
| `CAMERA_FPS` | `30` | Informational. |
| `CAMERA_STALE_S` | `0.5` | A frame older than this is stale; no velocity or actuation is commanded from a stale frame. |

---

## `gimbal`

| Key | Default | Meaning |
|---|---|---|
| `GIMBAL_MANUAL_NADIR` | `true` | `true` when the gimbal is RC-only with no telemetry to the flight controller. The nadir gate then passes unconditionally and `run.sh` asks the operator to confirm. Set `false` to restore real angle verification. |
| `GIMBAL_NADIR_TOL_DEG` | `5.0` | Accepted deviation from −90° pitch. Only used when `GIMBAL_MANUAL_NADIR` is false. |

The entire geolocation model assumes a **nadir-pointing camera**. If it is not pointing
down, coordinates are garbage — which is why this gate is fail-closed.

---

## `detector`

| Key | Default | Meaning |
|---|---|---|
| `MODEL_FILE` | `"best.pt"` | Weights filename inside `models/`. |
| `MODEL_ENGINE_FILE` | `"best.engine"` | TensorRT engine filename. |
| `USE_TENSORRT` | `false` | Off by default: tiling here is CPU-bound (letterbox + NMS), so an engine does not help. A single full-frame `.pt` inference is the faster real-time path. |
| `CLASS_NAMES` | `{"0":"mannequin","1":"tent"}` | Class index → label. All downstream logic keys off these **strings**. |
| `DETECT_CONF` | `0.25` | Detection floor on real imagery. |
| `DETECT_CONF_SIM` | `0.15` | Detection floor in simulation (weaker synthetic targets). |
| `IMGSZ` | `640` | Inference size for full-frame (non-tiled) inference. `960` finds smaller targets but runs slower. |
| `TILE_SIZE` | `640` | Tile size, and the inference size while tiling. |
| `TILE_OVERLAP` | `0.2` | Fractional overlap between tiles. |
| `TILE_ADD_FULL_FRAME` | `true` | Also run the full frame alongside the tiles. |
| `TILING_IN_SURVEY` | `false` | Tiling during the survey. Improves small-target recall but is CPU-bound (~2 FPS). |
| `TILING_IN_IBVS` | `false` | Tiling during visual servoing. The target is large and near, so this costs speed for nothing. |
| `NMS_IOU` | `0.6` | IoU for the class-aware global NMS. |

---

## `target_selection`

| Key | Default | Meaning |
|---|---|---|
| `DETECT_CONFIRM_SEC` | `3.0` | How long a target must be visible before its position is saved. |
| `DETECT_LOST_GRACE` | `0.5` | A loss longer than this resets the confirmation timer. Short, so single-frame flicker does not. |
| `ACQUIRE_TILT_GATE` | follows `SIM_MODE` | Only sample from level frames. **`true` if you have no gimbal** (tilting shifts the nadir projection); `false` with a gimbal, where filtering on body attitude would wrongly discard valid frames. |
| `ACQUIRE_TILT_MAX_DEG` | `6.0` | Tilt limit when the gate is on. |
| `ACQUIRE_MIN_SAMPLES` | `8` | Minimum samples before a position is trusted. |
| `ACQUIRE_MAX_SAMPLES` | `300` | Sample pool ceiling. |
| `ACQUIRE_REFINE` | `true` | Keep refining targets on later survey passes. |
| `TARGET_CLUSTER_RADIUS_M` | `8.0` | Same-class detections within this radius are one target. |
| `TARGET_WAIT_SEC` | `45.0` | Hold time over each target. **Must exceed the payload sequence duration** (~26 s by default) or the motor is cut mid-run. `show_config.py --check` verifies this. |
| `TARGET_MAX_ATTEMPTS` | `3` | IBVS attempts per target before the tolerance escalates. |
| `CANDIDATE_MIN_CONF` | `0.5` | Mean confidence required to *select* a candidate. Deliberately higher than `DETECT_CONF`, which is tuned for recall. |
| `CANDIDATE_MIN_CONF_SIM` | `0.2` | Same, in simulation. |
| `AFTER_VISIT_ACTION` | `"HOVER"` | After all targets: `"HOVER"`, `"RTL"` or `"LAND"`. |

---

## `ibvs`

Visual servoing. Keep `IBVS_MAX_VEL` and `IBVS_SLEW` **below** the autopilot's
`WPNAV_SPEED` and `WPNAV_ACCEL`, so the approach stays the gentler of the two.

| Key | Default | Meaning |
|---|---|---|
| `IBVS_KV` | `0.4` | Proportional gain [1/s]. |
| `IBVS_MAX_VEL` | `0.8` | Velocity ceiling [m/s]. |
| `IBVS_SLEW` | `0.5` | Acceleration limit [m/s²] — this is where the damping comes from. |
| `IBVS_ERROR_ALPHA` | `0.3` | Error EMA low-pass coefficient. |
| `IBVS_ALT_MIN` / `IBVS_ALT_MAX` | `2.0` / `30.0` | Altitude envelope [m]. Keep `IBVS_ALT_MAX` aligned with `PAYLOAD_ALT_MAX`, or there is a band where IBVS centres but release is blocked. |
| `IBVS_CENTER_TOL_M` | `2.0` | Acceptance radius and deadband [m]. A pixel-perfect tolerance lets parallax jitter trip the divergence watchdog. |
| `IBVS_CENTER_TOL_MAX_M` | `10.0` | Ceiling for the escalating tolerance. |
| `IBVS_CENTER_TOL_STEP_M` | `4.0` | Escalation step (2 → 6 → 10). `0` disables escalation. |
| `IBVS_LATCH_SPEED` | `0.15` | Never latch or fire above this commanded speed [m/s]. |
| `IBVS_CENTER_FRAMES` | `15` | Consecutive settled frames required. |
| `IBVS_TIMEOUT_S` | `30.0` | Hard timeout. |
| `IBVS_TARGET_FENCE_M` | `12.0` | Geofence measured **from the saved target** — the real flyaway lock. Also the release radius on the normal path. |
| `IBVS_LOSS_GRACE_S` | `5.0` | Detection-loss grace before aborting. |
| `IBVS_DIVERGE_S` | `3.0` | Error must stay above its minimum this long to count as divergence. |
| `IBVS_DIVERGE_MARGIN_M` | `0.5` | Divergence margin floor [m]. |
| `IBVS_DIVERGE_MARGIN_PX` | `50` | Pixel-noise component (altitude scaled). |
| `IBVS_DIVERGE_PITCH_DEG` | `3.5` | Braking-pitch component, resolution independent. |
| `IBVS_SIGN_FWD` | `-1` | Forward axis sign. **Verify in SITL before flying.** |
| `IBVS_SIGN_RIGHT` | `1` | Right axis sign. **Verify in SITL before flying.** |

**Verifying the signs.** Watch the `IBVS ex=... ey=... -> v_fwd=... v_right=...` log line.
The error must *shrink*. If the target is to the right (`ex > 0`) and `g_err` grows, flip
`IBVS_SIGN_RIGHT`; if the target is below (`ey > 0`) and `g_err` grows, flip
`IBVS_SIGN_FWD`. An inverted sign drives the aircraft *away* until a watchdog stops it.

---

## `goto`

| Key | Default | Meaning |
|---|---|---|
| `GPS_SETTLE_SEC` | `2.0` | Stabilisation after arrival, before handing over to IBVS. |
| `GOTO_RADIUS_M` | `3.0` | Arrival acceptance radius. |
| `GOTO_TIMEOUT_S` | `60.0` | Overall arrival timeout. |
| `GOTO_MIN_ALT_M` | `1.0` | Altitude floor — blocks an accidental land command. |

---

## `payload`

| Key | Default | Meaning |
|---|---|---|
| `GPIO_ENABLED` | `true` | Master switch for all GPIO output. |
| `PAYLOAD_ENABLED` | `true` | Arms the payload actuation. |
| `PAYLOAD_REVERSE` | `false` | Globally invert the motor direction. |
| `PAYLOAD_ALT_MIN` / `PAYLOAD_ALT_MAX` | `2.0` / `30.0` | Release altitude envelope [m]. |
| `FIRE_MIN_CONF` | `0.35` | Minimum confidence of the centred detection (identity gate). |
| `FIRE_TILT_MAX_DEG` | `35.0` | Sanity ceiling only. In wind the aircraft legitimately leans to hold position, and a strict gate would block the drop forever. Keep this **above** the autopilot's `ANGLE_MAX`. |
| `FIRE_SETTLE_MIN_S` | `1.5` | Minimum settled time over the target before releasing. |
| `FIRE_SETTLE_MAX_S` | `8.0` | Retry-window ceiling. Gates are retried at ~2 Hz throughout, so one bad frame does not lose the target. |
| `GPS_DROP_FALLBACK` | `true` | Blind release at the saved GPS if IBVS cannot centre. |
| `GPS_DROP_FIRE_RADIUS_M` | `5.0` | Acceptance radius for the blind release. |
| `GPS_DROP_MAX_SPEED` | `0.5` | No release above this ground speed [m/s]. |
| `GPS_DROP_SETTLE_MAX_S` | `6.0` | Maximum extra wait for the aircraft to stop. |

> **Set `GPIO_ENABLED` and `PAYLOAD_ENABLED` to `false`** for every flight until the
> payload has been bench tested with `tools/payload_test.py`. When both are true and
> `SIM_MODE` is false, the startup banner warns that the payload is armed.

---

## `payload_hardware`

Pins are BOARD (physical header) numbering. `show_config.py --check` detects collisions.

| Key | Default | Meaning |
|---|---|---|
| `PAYLOAD_TARGETS` | mannequin/tent | Per class: `servo_pin`, `motor` (`"A"`/`"B"`), `release_angle`, `direction` (`1`/`-1`). Class names must match `CLASS_NAMES`. |
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
| `MOTOR_RUN_S` | `20.0` | Run time including the ramp [s]. |
| `MOTOR_RAMP_S` | `1.5` | Soft-start ramp — prevents the inrush that browns out the Jetson. |
| `WAIT_BEFORE_SERVO` | `2.0` | Pause before the servo moves. |
| `WAIT_AFTER_SERVO` | `2.0` | Pause before the motor runs. |
| `BUZZER_PIN` | `13` | |
| `LED_STATUS_PIN` | `15` | |
| `AUX_PINS` | `[16, 18]` | Spare outputs. |

The full sequence takes `WAIT_BEFORE_SERVO + SERVO_MOVE_S + WAIT_AFTER_SERVO + MOTOR_RUN_S`
(~26 s by default), which must fit inside `TARGET_WAIT_SEC`.

---

## `mapping`

| Key | Default | Meaning |
|---|---|---|
| `PHOTO_DIR` | `"survey_photos"` | Output directory (gitignored). |
| `PHOTO_START_WP` | `0` | Capture from this waypoint sequence onward; `0` means all. |
| `STABILIZE_PITCH_MAX` / `STABILIZE_ROLL_MAX` | `3.0` | Tilt limit before a photo. **Photos only** — the airdrop uses `FIRE_TILT_MAX_DEG`. |
| `STABILIZE_TIMEOUT` | `10.0` | Give up waiting and capture anyway. |
| `UPLOAD_ENABLED` | `false` | Copy each photo to the ground station over scp. |
| `UPLOAD_HOST` | `GROUND_STATION_IP` | |
| `UPLOAD_USER` | `"user"` | |
| `UPLOAD_REMOTE_DIR` | `/home/user/survey_photos` | |

---

## `streaming`

Live H.264 stream of the annotated frames. See [GROUND_STATION.md](GROUND_STATION.md).

| Key | Default | Meaning |
|---|---|---|
| `STREAM_ENABLED` | `false` | Costs nothing when off. |
| `STREAM_HOST` | `GROUND_STATION_IP` | Where to send it. |
| `STREAM_PORT` | `5600` | QGC / Mission Planner default video port. |
| `STREAM_WIDTH` / `STREAM_HEIGHT` | `960` / `540` | Downscaled from the source. |
| `STREAM_FPS` | `15` | Nominal; the real rate follows the tracker loop. |
| `STREAM_BITRATE` | `2000000` | bps, constant bitrate. Lower it if the raw camera feed shares the link. |
| `STREAM_IDR_SEC` | `1.0` | Keyframe interval — a short GOP recovers faster on lossy UDP. |

Requires launching through `run.sh`; otherwise it disables itself silently and logs one
warning. Streaming errors can never affect telemetry, IBVS or the payload path.

---

## `safety`

| Key | Default | Meaning |
|---|---|---|
| `POSE_STALE_S` | `1.0` | Position/attitude older than this is stale. |
| `GPS_EKF_STALE_S` | `1.5` | Separate, wider window for the 2 Hz GPS and EKF streams. `POSE_STALE_S` is far too tight for a 0.5 s interval — one dropped packet used to flap the gates and cause spurious hard stops. Keep it **below** `LINK_TIMEOUT_S`. |
| `POSE_MIN_SATS` | `6` | Minimum satellites for `have_fix`. |
| `LINK_TIMEOUT_S` | `3.0` | No autopilot heartbeat for this long = link lost. |
| `SHUTDOWN_SAFE_MODE` | `"BRAKE"` | Park mode on exit, applied **only** if the companion still owns GUIDED. `null` leaves the mode alone. |

---

## What `--check` validates

`tools/show_config.py --check` reports errors (exit code 1) and warnings:

- `config.json` is missing, malformed, or contains unrecognised keys
- Model weights missing; `USE_TENSORRT` on without an engine
- Flight controller device path missing; a bare `/dev/ttyACM*` path
- `TARGET_WAIT_SEC` shorter than the payload sequence (would cut the motor mid-run)
- Altitude envelopes that do not overlap between IBVS, payload and goto
- A centring tolerance ladder that exceeds the target fence
- `GPS_EKF_STALE_S` at or above `LINK_TIMEOUT_S`
- Confidence thresholds ordered so a target can be selected but never released
- GPIO pin collisions between servos, motors, the buzzer, the LED and aux outputs
- A payload target class that is not one of the model classes (it would never actuate)
- The payload being armed, or `SIM_MODE` being left on
