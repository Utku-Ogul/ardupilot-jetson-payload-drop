# Bench and Pre-Flight Checklist

Work down this list in order. Each stage assumes the previous one passed.

Read [SAFETY.md](SAFETY.md) first.

---

## Stage 0 — Desk (no hardware)

```bash
python3 -m unittest discover -s tests   # geometry conventions
python3 tools/show_config.py            # your settings, and a consistency check
```

The geometry tests lock the pixel→GPS axis convention. A failure here means saved target
positions will be wrong, so fix it before anything else.

`show_config.py` reports typos in `config.json` (an unrecognised key is *ignored*, which is
dangerous for a safety setting), missing files, GPIO pin collisions and inconsistent
thresholds. Fix every ERROR before continuing.

---

## Stage 1 — Bench, read-only

Companion connected to the flight controller over USB. **Propellers off.**

```bash
python3 tools/bench_test.py
```

This never arms, never commands motion and never actuates the payload. Check:

- [ ] `LINK-01` — heartbeat received, correct system id
- [ ] `LINK-02` — stream rates near their targets
- [ ] `GIMB-01` — a gimbal attitude message is arriving (unless `GIMBAL_MANUAL_NADIR`)
- [ ] `PRE-01` — critical parameters pass
- [ ] `GATE-01..05` — link, GPS, EKF, gimbal and tilt gates

`GATE-02` (`have_fix`) fails indoors — that is expected. Repeat outside.

With the camera connected:

```bash
python3 tools/bench_test.py --camera --detect
```

- [ ] `CAM-01/02` — frames arrive at the configured resolution
- [ ] `DET-01` — the model loads and the class names match

---

## Stage 2 — Camera and detector

```bash
python3 tools/preview.py
```

- [ ] Live image, sensible frame rate
- [ ] Targets are detected at your intended operating altitude
- [ ] Class labels are correct (a mannequin is not reported as a tent)
- [ ] Tune `DETECT_CONF` with `+`/`-` and copy the value you settle on into `config.json`

If small targets are missed, try `f` to raise the image size, or enable
`TILING_IN_SURVEY` — but check the frame rate, since tiling is CPU-bound.

### Camera calibration

- [ ] `CAMERA_HFOV_DEG` measured with `tools/calibrate_camera.py`, not left at the spec
      value (skip only if you accept metre-level geolocation error)

---

## Stage 3 — Payload bench, off the aircraft

**Payload mechanism on the bench, not mounted. Nobody near the mechanism.**

```bash
python3 tools/payload_test.py
```

- [ ] Servo init moves both servos to neutral
- [ ] Each target class actuates the **correct** servo and the **correct** motor
- [ ] The release angles are right for your mechanism
- [ ] The motor turns in the direction that actually releases the load
      (invert with `PAYLOAD_REVERSE` if not)
- [ ] Ctrl+C during the motor run stops it immediately
- [ ] Killing the parent process stops the motor (deadman)

Adjust pins, angles and timings in `config.json` under `payload_hardware`, then re-run
`python3 tools/show_config.py --check` to confirm there are no pin collisions.

---

## Stage 4 — Simulation

Set `"SIM_MODE": true` in `config.json`, then:

```bash
./sim/start_sim.sh
./run.sh
```

- [ ] AUTO is detected and the state machine enters MONITORING
- [ ] Targets are detected and candidates accumulate
- [ ] Mission completion triggers the GUIDED takeover
- [ ] GPS goto reaches the target
- [ ] **IBVS reduces the error** — watch the `g_err` log line

### Axis signs — do this before any real IBVS flight

The `IBVS ex=... ey=... -> v_fwd=... v_right=...` log line at ~2 Hz tells you everything:

- The error must **shrink**. If `g_err` grows while the target is off-centre, a sign is
  inverted.
- Target to the RIGHT (`ex > 0`) and the error grows → flip `IBVS_SIGN_RIGHT`.
- Target BELOW (`ey > 0`) and the error grows → flip `IBVS_SIGN_FWD`.

- [ ] Both signs verified in simulation

An inverted sign drives the aircraft *away* from the target until a watchdog stops it.

---

## Stage 5 — Field, payload DISARMED

`"GPIO_ENABLED": false`, `"PAYLOAD_ENABLED": false` in `config.json`. Pilot on the sticks,
mode switch ready.

Build up in this order, aborting to a manual mode at any surprise:

1. [ ] **GUIDED acquisition** — the companion can put the vehicle into GUIDED
2. [ ] **Goto response** — the vehicle actually flies to a commanded position
3. [ ] **Body-frame axes** — a small manual velocity nudge moves it the expected way
4. [ ] **IBVS signs on the real airframe** — `g_err` decreases
5. [ ] **Deadman** — kill the companion process in flight; confirm the flight controller
       brakes (`GUID_TIMEOUT`) and then acts on `FS_GCS`
6. [ ] **Abort** — a pilot mode change immediately parks the state machine

Then a full end-to-end mission with the payload still disarmed:

- [ ] Survey → detection → mission complete → goto → IBVS → settle → **hold with no
      actuation** → next target → hover
- [ ] Photos land in `survey_photos/` with correct GPS EXIF

---

## Stage 6 — Payload armed

**Only after every stage above has passed.**

- [ ] `"GPIO_ENABLED": true`
- [ ] `"PAYLOAD_ENABLED": true`
- [ ] `"SIM_MODE": false` — confirm it in the startup banner
- [ ] `python3 tools/show_config.py --check` passes
- [ ] The startup banner shows the "PAYLOAD ARMED" warning
- [ ] Payload physically loaded and secured
- [ ] Drop zone clear of people

---

## Every flight

- [ ] `python3 tools/show_config.py --check` clean
- [ ] `SIM_MODE` correct (check the startup banner)
- [ ] Mission uploaded, ending in `NAV_LOITER_UNLIM`
- [ ] Gimbal pointing down and confirmed on video (if `GIMBAL_MANUAL_NADIR`)
- [ ] Geofence appropriate for the site
- [ ] Battery charged, propellers secure
- [ ] Pilot ready with a mode switch
- [ ] `logs/suas.log` is being written — it is your only forensic record

---

## After the flight

- [ ] Review `logs/suas.log` for warnings, aborts and gate blocks
- [ ] Check `logs/payload_last_run.log` if the payload fired
- [ ] Confirm the photos in `survey_photos/` are geotagged
- [ ] Compare the saved target positions against where the targets actually were — that
      error is your geolocation accuracy, and it tells you whether calibration is worth it
