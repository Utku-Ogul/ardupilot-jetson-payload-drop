# SUAS — Autonomous Aerial Search, Geolocation and Payload Delivery

An onboard companion-computer system for a multirotor UAV. It flies a survey mission,
detects ground targets with a YOLO model, estimates their GPS positions, flies back to
each one, centres on it with visual servoing, and releases a payload.

Everything runs on a Jetson bolted to the aircraft, talking to an ArduPilot flight
controller over USB. All settings live in a single `config.json`, so you can adapt it to
your own airframe, camera and release mechanism without touching the code.

> **Safety notice.** This software commands a real aircraft and actuates real hardware.
> Read [docs/SAFETY.md](docs/SAFETY.md) before flying it. Keep `GPIO_ENABLED` and
> `PAYLOAD_ENABLED` set to `false` until the payload has been bench tested.

---

## What it actually does

```
AUTO survey            Fly the uploaded waypoint mission, photographing each waypoint
      |                (GPS-tagged) and running detection continuously.
      v
Target confirmation    A detection visible for 3 s becomes a candidate. Each candidate
      |                accumulates GPS samples; the position is a median with 2-sigma
      |                outlier rejection, refined on every later pass.
      v
Mission complete       The most-seen candidate per class enters a visit queue,
      |                nearest first. Brief false positives lose.
      v
GPS goto               Switch to GUIDED, fly to the estimated position, settle.
      |
      v
Visual servoing        Close the loop on the camera: drive the pixel error to zero with
      |                body-frame velocity commands, altitude-normalised.
      v
Settle, then release   Hold position over the target, confirm every safety gate, then run
      |                the payload sequence in a separate process.
      v
Next target / hover
```

If visual servoing cannot centre the target, the acceptance tolerance widens in steps
(2 m → 6 m → 10 m). If it still fails, the aircraft flies to the saved GPS position and
performs a blind release — but only while every *vehicle* safety gate still passes.

---

## Hardware we used

This is the exact setup the system was developed and flown on. None of it is mandatory —
see [Adapting it](#adapting-it-to-your-hardware) — but it is what the defaults assume.

| Part | What we used | Notes |
|---|---|---|
| **Companion computer** | **NVIDIA Jetson Orin NX 16 GB**, JetPack 6.x (L4T R36.x) | Runs detection, the state machine and the payload GPIO. 16 GB is comfortable; 8 GB also works with full-frame inference. |
| **Flight controller** | **CubePilot Orange Cube+**, ArduCopter | Connected to the Jetson over **USB** (`/dev/serial/by-id/...`). |
| **Camera / gimbal** | **SIYI A8 mini**, 1080p, ~81° HFOV, 3-axis gimbal | RTSP H.264 stream over the video link. Must point **straight down (nadir)** — the whole geolocation model assumes it. |
| **Video / data link** | **SIYI HM30** | Transparent IP bridge — the Jetson, the camera and the ground station all sit on its subnet. Also carries the gimbal RC control. |
| **Ground station** | Mission Planner on Windows | Uploads the survey mission; can display the annotated video feed. |
| **Payload servos** | **2 × SG90** hobby servos | One per target class. Software PWM from the Jetson header. Need a **separate 5 V supply** with common ground. |
| **Payload motor driver** | **HW-231 dual MC33886** H-bridge board | Two independent channels (A and B), one per target class. No enable pin — each channel is driven by two logic inputs. |
| **Payload motors** | 2 × brushed DC | Soft-started through a ramp to avoid browning out the Jetson. |

### How the payload works

Two independent release mechanisms, one per target class. When a target is centred and
every gate passes:

1. The **servo** for that class rotates from its neutral angle to its release angle
   (mannequin `90° → 30°` on pin 7, tent `90° → 150°` on pin 12 — they open in opposite
   directions).
2. After a short pause, the **motor** for that class runs forward at 100 % for 20 s,
   paying out the payload.

The whole sequence runs as a **separate process** with its own deadman, so if the tracker
dies the motor is de-energised instead of being left at full power. Pins, angles and
timings are all in `config.json` under `payload_hardware`.

### Wiring

Pin numbers are BOARD (physical header) numbering.

| Function | Pin(s) |
|---|---|
| Servo 1 (mannequin) | 7 |
| Servo 2 (tent) | 12 |
| Motor A (tent) | 32 forward, 33 reverse |
| Motor B (mannequin) | 29 forward, 31 reverse |
| Buzzer | 13 |
| Status LED | 15 |
| Spare outputs | 16, 18 |
| Common ground | 6 |

> **3.3 V warning.** The MC33886 wants a 3.5 V logic high and the Jetson drives 3.3 V,
> which is marginal. If a motor behaves erratically, add a 5 V level shifter
> (74HCT244 / 74AHCT125) on the input lines. The logic low is always fine.
>
> **Servo power.** Do not run SG90s from the 3.3 V rail. Use a separate 5 V supply and tie
> its ground to the Jetson ground.

---

## Setting up the Jetson

Everything below runs on the Jetson itself, in the repository root.

### 1. System packages

```bash
sudo apt-get update
sudo apt-get install -y python3-pip git \
    gstreamer1.0-plugins-bad gstreamer1.0-libav

# ModemManager grabs /dev/ttyACM* and will fight you for the flight controller
sudo apt-get remove --purge modemmanager

# Serial + GPIO access without sudo (log out and back in afterwards)
sudo usermod -aG dialout $USER
sudo usermod -aG gpio $USER
```

`gstreamer1.0-plugins-bad` provides `h264parse`, which enables the hardware video decode
path. Without it the camera still works via FFMPEG software decode — just more expensive.

### 2. Python packages

```bash
git clone <your-fork-url> suas
cd suas
pip install -r requirements.txt
pip install Jetson.GPIO
```

Then install **torch and torchvision from the NVIDIA JetPack wheels** — *not* from PyPI,
because the PyPI builds have no CUDA support for Jetson:

<https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/>

> **`torchvision` is required.** The detector uses `torchvision.ops.batched_nms` to merge
> boxes across tiles. Without it you get duplicate detections.

### 3. The OpenCV situation (important)

There are two OpenCV builds on a Jetson, and it matters which one you get:

| Build | GStreamer | |
|---|:---:|---|
| JetPack system OpenCV (`/usr/lib/python3.10/dist-packages`) | **yes** | needed for the camera pipeline and video streaming |
| pip `opencv-python` | no | shadows the system build |

`run.sh` puts the JetPack build first on `PYTHONPATH`, so **always launch through
`run.sh`**. Nothing needs to be uninstalled. Under a bare `python3` the camera still works
through the FFMPEG fallback, but the outbound video stream silently disables itself.

### 4. Do not use sudo

As root, `pip --user` packages in `~/.local` are invisible and you get
`ModuleNotFoundError` for pymavlink, cv2 and numpy. GPIO works without root once your user
is in the `gpio` group, because `Jetson.GPIO` uses the `/dev/gpiochip` (libgpiod) backend.

### 5. Model weights

Put your trained YOLO weights at `models/best.pt`, with two classes named `mannequin` and
`tent`. See [models/README.md](models/README.md) for training or changing the classes.

---

## Configuration — one file

**Edit `config.json`. Do not edit the Python.**

Every setting lives there, grouped into readable sections. `suas/config.py` holds the
defaults and the reasoning behind each one; `config.json` overrides them. A key that is
absent or `null` keeps its default, so you can leave keys in place as documentation.

Two settings ship **blank**, because they are specific to your own hardware and nothing can
connect until you fill them in — `tools/show_config.py --check` reports both as errors
while they are empty:

| Setting | Section | What it is |
|---|---|---|
| `MAVLINK_CONNECT_REAL` | `flight_controller` | Your flight controller's device path. Find it with `ls -l /dev/serial/by-id/` and use the full by-id path, not `/dev/ttyACM0`. A network endpoint such as `udpin:0.0.0.0:14550` also works. |
| `CAMERA_RTSP_URL_REAL` | `camera` | Your camera's stream URL, e.g. `rtsp://<camera-ip>:8554/main.264`. The port and path are in your camera's manual. |

The rest you will probably want to review:

| Setting | Section | What it is |
|---|---|---|
| `SIM_MODE` | `profile` | `true` for SITL, `false` for a real vehicle. Selects the camera source, MAVLink endpoint and several safety behaviours. |
| `CAMERA_HFOV_DEG` | `camera` | Horizontal field of view. Geolocation accuracy depends on it — measure it with `tools/calibrate_camera.py`. |
| `GPIO_ENABLED`, `PAYLOAD_ENABLED` | `payload` | **Keep both `false`** until the payload is wired and bench tested. |
| `GIMBAL_MANUAL_NADIR` | `gimbal` | `true` if your gimbal has no MAVLink telemetry and the operator aims it by RC (our setup). |
| `PAYLOAD_TARGETS` | `payload_hardware` | Which servo pin, motor channel and release angle belongs to each target class. |

After editing, always verify:

```bash
python3 tools/show_config.py            # what resolved, and from where
python3 tools/show_config.py --changed  # only your overrides
python3 tools/show_config.py --check    # validation only
```

This flags typos (a misspelled key is *ignored*, which is dangerous for a safety setting),
missing files, pin collisions, and inconsistent thresholds — for example a
`TARGET_WAIT_SEC` shorter than the payload sequence, which would cut the motor mid-run.

---

## Flight controller parameters

The companion **refuses to start** on a real vehicle if the critical safety parameters are
wrong. The three that block startup:

| Parameter | Value | Why |
|---|---|---|
| `MAV_GCS_SYSID` | **254** | The companion connects as system 254; this is what makes its heartbeat a real deadman. |
| `FENCE_ENABLE` | **1** | The last-wall geofence. |
| `GUID_TIMEOUT` | **3** | A stalled companion must not leave a live velocity setpoint running. |

Full list, wiring and gimbal setup: [docs/FLIGHT_CONTROLLER.md](docs/FLIGHT_CONTROLLER.md).

---

## Verify, then fly

```bash
python3 -m unittest discover -s tests   # geometry conventions, no hardware needed
python3 tools/show_config.py            # configuration sanity
python3 tools/bench_test.py             # read-only: link, telemetry, preflight, gates
python3 tools/preview.py                # live camera + detector, with tuning keys
python3 tools/payload_test.py           # bench-test the release (payload OFF the aircraft)
```

Then:

```bash
./run.sh                    # with the preview window
./run.sh --no-display       # headless / over SSH
./run.sh --no-mapping       # no photo capture
./run.sh --photo-start-wp 5 # start photographing at waypoint 5
```

At startup `run.sh` selects the correct OpenCV, moves both payload servos to neutral, and
(in manual-nadir mode) asks the operator to confirm the gimbal is pointing down.

Work through [docs/PREFLIGHT.md](docs/PREFLIGHT.md) before the first flight. It builds up
from desk tests to simulation to a field ladder with the payload disarmed.

---

## Repository layout

```
config.json               ALL your settings — this is the file you edit
run.sh                    Launch script
suas/
  settings.py             config.json loader (typo detection, defaults)
  config.py               Defaults + the reasoning behind every value
  main.py                 Entry point and main loop
  tracker.py              Mission state machine
  mission.py              Mission download and completion detection
  detection/              base (interface) · camera · tiled_yolo
  navigation/             geo (pixel -> GPS) · ibvs (visual servoing)
  vehicle/                connection · control · pose (health gates)
  mapping/                photo (waypoint capture) · exif
  payload/                drop_sequence · servo · motor · indicators
  streaming/              rtp (annotated H.264 to the ground)
models/                   YOLO weights
tools/                    show_config · bench_test · preview · calibrate_camera ·
                          payload_test · export_tensorrt
tests/                    Unit tests (no hardware)
sim/                      SITL + Gazebo launch and parameters
docs/                     Safety, flight controller, setup, preflight, ground station
```

---

## Adapting it to your hardware

**Different detector.** Subclass `BaseDetector` in `suas/detection/base.py`, implement
`detect()` and `draw()` (optionally `detect_all()` / `draw_all()` for multiple targets),
and return it from `_build_detector()` in `suas/tracker.py`.

**Different payload.** `suas/payload/drop_sequence.py` is a standalone script driven
entirely by `config.json`. The tracker only launches it with `--target <class>`, so you can
replace the mechanism completely without touching the state machine.

**Different classes.** Update `CLASS_NAMES` and `PAYLOAD_TARGETS` in `config.json`.
`tools/show_config.py` verifies they agree; the detector warns if the loaded weights
disagree.

**No gimbal.** Set `ACQUIRE_TILT_GATE` to `true`, so target samples are only taken from
level frames (a tilted airframe shifts the nadir projection).

**Different flight controller.** Anything ArduPilot works. The MAVLink layer is confined
to `suas/vehicle/`.

---

## Documentation

- [docs/SAFETY.md](docs/SAFETY.md) — the layered safety model, every gate, and what happens when each fails
- [docs/CONFIGURATION.md](docs/CONFIGURATION.md) — every setting in `config.json`
- [docs/FLIGHT_CONTROLLER.md](docs/FLIGHT_CONTROLLER.md) — ArduPilot parameters, wiring, gimbal
- [docs/SETUP.md](docs/SETUP.md) — companion computer, network, camera, payload wiring
- [docs/PREFLIGHT.md](docs/PREFLIGHT.md) — bench and pre-flight checklist
- [docs/GROUND_STATION.md](docs/GROUND_STATION.md) — watching the annotated video feed

---

## License

MIT — see [LICENSE](LICENSE).
