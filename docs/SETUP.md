# Companion Computer Setup

Reference hardware: NVIDIA Jetson Orin NX on JetPack 6.x. Any Linux machine that can reach
the flight controller and the camera will run the flight code; only `suas/payload/*`
requires Jetson GPIO.

---

## 1. System packages

```bash
sudo apt-get update
sudo apt-get install -y python3-pip gstreamer1.0-plugins-bad gstreamer1.0-libav

# ModemManager grabs /dev/ttyACM* devices and will fight you for the flight controller
sudo apt-get remove --purge modemmanager

# Serial and GPIO access without sudo (log out and back in afterwards)
sudo usermod -aG dialout $USER
sudo usermod -aG gpio $USER
```

> `gstreamer1.0-plugins-bad` provides `h264parse`. With it present, the camera reader uses
> the hardware decode path; without it, it automatically falls back to FFMPEG software
> decode. Both work — hardware decode is just cheaper.

---

## 2. Python packages

```bash
pip install -r requirements.txt
```

Then install `torch` and `torchvision` from the
[NVIDIA JetPack wheels](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/),
not from PyPI. **`torchvision` is required** — the detector uses it for cross-tile NMS,
and without it overlapping boxes are never merged.

For the payload:

```bash
pip install Jetson.GPIO
```

### The OpenCV situation

There are two OpenCV builds on a Jetson and it matters which one you get:

| Build | GStreamer | Notes |
|---|:---:|---|
| JetPack system OpenCV (`/usr/lib/python3.10/dist-packages`) | **yes** | Needed for the camera pipeline and the video stream |
| pip `opencv-python` | no | Shadows the system build |

`run.sh` puts the JetPack build first on `PYTHONPATH`, so **always launch through
`run.sh`**. Nothing needs to be uninstalled. Under a bare `python3`, the camera still
works via the FFMPEG fallback, but the outbound video stream silently disables itself.

The diagnostic tools (`tools/preview.py`, `tools/calibrate_camera.py`) re-exec themselves
with the same `PYTHONPATH`, so they can be run directly.

---

## 3. Sudo is not needed

Do **not** run this with `sudo`. As root, `pip --user` packages in `~/.local` are
invisible, and you get `ModuleNotFoundError` for pymavlink, cv2 and numpy.

GPIO works without root because `Jetson.GPIO` uses the `/dev/gpiochip` (libgpiod) backend
once your user is in the `gpio` group.

---

## 4. Configuration

**All settings live in `config.json` in the repository root. Do not edit the Python.**

`suas/config.py` holds the defaults and the reasoning behind each one; `config.json`
overrides them. A key that is absent or `null` keeps its default.

After every edit:

```bash
python3 tools/show_config.py            # what resolved, and from where
python3 tools/show_config.py --changed  # only your overrides
python3 tools/show_config.py --check    # validation only
```

An **unrecognised key is reported as an error**, not silently ignored — a misspelled
safety setting that quietly kept its default is exactly the failure this cannot afford.

Full reference: [CONFIGURATION.md](CONFIGURATION.md).

---

## 5. Network

The companion needs to reach the camera; the ground station needs to reach the companion
if you want the live annotated video stream.

With a video transmitter acting as a transparent IP bridge, put all three devices on one
subnet. Your radio's manual states which subnet it uses and which addresses it reserves
for itself — pick free addresses on that subnet:

| Device | Where its address is used |
|---|---|
| Companion computer | `COMPANION_IP` — documentation only |
| Camera | inside `CAMERA_RTSP_URL_REAL` |
| Ground station PC | `STREAM_HOST` (falls back to `GROUND_STATION_IP`) |

`COMPANION_IP`, `GROUND_STATION_IP` and `SIM_HOST_IP` live in `config.json` (section
`network`) and ship blank. Only the camera URL and the stream host are read by the code;
the rest are there so your whole setup is recorded in one place.

Check reachability before flying:

```bash
ping <camera-ip>
ffplay rtsp://<camera-ip>:8554/main.264
```

---

## 6. Camera

Set the stream URL and optics in `config.json` (section `camera`):

```json
"camera": {
  "CAMERA_MODE": "rtsp",
  "CAMERA_RTSP_URL_REAL": "rtsp://<camera-ip>:8554/main.264",
  "CAMERA_WIDTH": 1920,
  "CAMERA_HEIGHT": 1080,
  "CAMERA_HFOV_DEG": 81.0
}
```

Use `"CAMERA_MODE": "udp"` for an RTP/JPEG source instead of RTSP. Measure
`CAMERA_HFOV_DEG` rather than trusting the spec — see below.

Verify frames arrive:

```bash
python3 tools/preview.py --camera-only
python3 tools/preview.py                 # with detection
```

### Field of view

`CAMERA_HFOV_DEG` drives both `IBVS_FX` and `pixel_to_gps`. The manufacturer spec is a
starting point; a 10% error becomes metres of geolocation error at 20 m altitude.

```bash
python3 tools/calibrate_camera.py capture --cols 9 --rows 6
python3 tools/calibrate_camera.py solve   --cols 9 --rows 6 --square 25
```

Take 15–25 photos of a flat, rigid checkerboard at varied angles, distances and image
corners. Copy the reported `CAMERA_HFOV_DEG` into the config.

> IBVS is closed-loop and fairly insensitive to focal-length error; the saved GPS position
> is not. Calibrate if geolocation accuracy matters to you.

---

## 7. Payload wiring

Default pin map (BOARD / physical numbering):

| Function | Pins |
|---|---|
| Servo 1 (target class 1) | 7 |
| Servo 2 (target class 2) | 12 |
| Motor A | 32 (forward), 33 (reverse) |
| Motor B | 29 (forward), 31 (reverse) |
| Buzzer | 13 |
| Status LED | 15 |
| Spare outputs | 16, 18 |
| Common ground | 6 |

Change them in `config.json` under `payload_hardware` — no code edits needed.
`python3 tools/show_config.py --check` detects pin collisions.

**Power.** Hobby servos need 5 V from a separate supply with a common ground — do not run
them from the 3.3 V rail. Motor supply goes to the driver board, not through the companion.

**Logic levels.** Many H-bridge drivers specify a 3.5 V logic high while the Jetson drives
3.3 V, which is marginal. If the motor behaves erratically, add a 5 V level shifter
(74HCT244 / 74AHCT125) on the input lines. The low level is always fine.

Bench test before you ever arm with the payload connected:

```bash
python3 tools/payload_test.py
```

---

## 8. Running

```bash
./run.sh                    # preview window
./run.sh --no-display       # headless / SSH
./run.sh --no-mapping       # no photo capture
./run.sh --photo-start-wp 5 # start photographing at waypoint 5
```

At startup `run.sh`:

1. Puts the GStreamer-enabled OpenCV first on `PYTHONPATH`.
2. Moves both payload servos to neutral (when `GPIO_ENABLED` is true).
3. Prompts for manual gimbal-nadir confirmation (when `GIMBAL_MANUAL_NADIR` is true).
4. Starts `python3 -m suas.main`.

---

## 9. Autostart (optional)

```ini
# /etc/systemd/system/suas.service
[Unit]
Description=SUAS companion
After=network-online.target

[Service]
Type=simple
User=YOUR_USER
WorkingDirectory=/home/YOUR_USER/suas
ExecStart=/home/YOUR_USER/suas/run.sh --no-display --no-ask-mapping
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now suas
journalctl -u suas -f
```

> With no TTY, the interactive mapping prompt and the gimbal confirmation are skipped —
> pass `--photo-start-wp N` explicitly, and make sure the gimbal is correctly aimed.
> `SIGTERM` from `systemctl stop` triggers the full neutralise-and-cleanup path.
