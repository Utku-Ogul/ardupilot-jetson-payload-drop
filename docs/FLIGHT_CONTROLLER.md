# Flight Controller Setup (ArduPilot)

Everything the flight controller needs so the companion computer can take over in GUIDED
and so the safety backstops actually exist.

Load parameters through your ground control station (Mission Planner: **CONFIG → Full
Parameter List**; QGroundControl: **Vehicle Setup → Parameters**), then **write** and
**reboot**. Back up your existing parameters first.

---

## 1. Critical parameters

If any of these are wrong, the system does not work — and the first three **block the
flight** at startup (`preflight_param_check`).

| Parameter | Value | If it is wrong |
|---|---|---|
| `FENCE_ENABLE` | **1** | No last-wall geofence. **Blocks startup.** |
| `GUID_TIMEOUT` | **3** | A stalled companion leaves a live velocity setpoint running. **Blocks startup.** |
| `SYSID_ENFORCE` | **0** | The companion's (sysid 254) GUIDED / goto / velocity commands are silently rejected. **Blocks startup.** |
| `SYSID_MYGCS` | **255** | The ground station. `FS_GCS` must watch the ground link, not the companion. On newer firmware this is `MAV_GCS_SYSID`. |
| `SERIAL0_PROTOCOL` | **2** (MAVLink2) | The companion cannot talk over USB. |
| `ARMING_CHECK` | **1** | Arming with an uncalibrated or unhealthy vehicle. |

The companion connects with `source_system = 254`, distinct from the ground station (255).
`SYSID_MYGCS = 255` makes `FS_GCS` watch the ground link (SUAS 2026: 15 s comms loss ->
RTL); if the companion itself stops, `GUID_TIMEOUT` brakes the vehicle within 3 s.
Do not set `SYSID_ENFORCE = 1` -- the companion would lose command authority in the air.

---

## 2. Failsafe and fence

| Parameter | Suggested | Meaning |
|---|---|---|
| `FS_GCS_ENABLE` | 1 | RTL when the ground-station heartbeat stops |
| `FS_GCS_TIMEOUT` | 15 | Seconds from heartbeat loss to failsafe (SUAS 2026 Appendix B) |
| `FS_OPTIONS` | 0 | Bit 1 off: RTL on GCS loss in AUTO too |
| `FS_THR_ENABLE` | 1 | RC loss -> RTL. Also required by `lua/comms_loss_termination.lua` |
| `FENCE_TYPE` | 3 | Circle + altitude ceiling |
| `FENCE_ACTION` | 4 (Brake) or 1 (RTL) | What happens at the boundary |
| `FENCE_RADIUS` | site dependent | Horizontal limit from home |
| `FENCE_ALT_MAX` | 80 | Altitude ceiling (50 m survey + margin; must be well above the mission altitude) |
| `FENCE_MARGIN` | 6 | Stopping-distance allowance |
| `FS_EKF_ACTION` | 1 | Land if the EKF diverges |
| `FS_EKF_THRESH` | 0.8 | EKF failsafe threshold |
| `FS_VIBE_ENABLE` | 1 | Vibration failsafe |
| `BATT_MONITOR` | > 0 | Battery monitoring on |
| `BATT_FS_LOW_ACT` | >= 1 | Low-battery action |

> **`FENCE_TYPE` note.** Value `7` includes the polygon bit, which prevents arming unless a
> polygon fence has actually been uploaded. Use `3` unless you are deliberately using a
> polygon.

### `FENCE_ACTION`: Brake or RTL?

The companion's own philosophy is "stop and wait for the operator", which pairs naturally
with **Brake (4)**. Choose **RTL (1)** if your site or rules require an automatic return.
Never use `0`.

---

## 3. Navigation speed

The visual servoing limits in `suas/config.py` must stay **below** the autopilot's
navigation limits, so IBVS is always the gentler of the two. IBVS clamps its own command to
`IBVS_MAX_VEL`, so a faster `WPNAV_SPEED` does not make centring more aggressive -- it only
speeds up the GUIDED transit to the target, after which `GOTO_SETTLE_MAX_SPEED` waits for
the vehicle to actually stop before IBVS starts.

| Parameter | As flown | Paired config value |
|---|---|---|
| `WPNAV_SPEED` | 500 (5 m/s, GUIDED goto) | `IBVS_MAX_VEL`: 0.8, `GOTO_TIMEOUT_S`: 240 |
| `WPNAV_ACCEL` | 250 (2.5 m/s²) | `GOTO_SETTLE_MAX_SPEED`: 0.5 |
| `WPNAV_RADIUS` | 500 (5 m) | SUAS waypoint threshold < 50 ft |
| `RTL_ALT` | 5000 (50 m) | survey altitude -- RTL never descends first |
| `ANGLE_MAX` | 2000–3000 (20–30°) | `FIRE_TILT_MAX_DEG`: 35 |

A larger `ANGLE_MAX` gives more tilt authority in wind. Keep `FIRE_TILT_MAX_DEG` above it
so a legitimate wind lean never blocks the payload release.

---

## 4. Mission settings

| Parameter | Value | Why |
|---|---|---|
| `MIS_RESTART` | 0 | Resume rather than restart the mission |

**Mission structure.** Build your survey grid, then append a terminal
`NAV_LOITER_UNLIM`. The companion detects mission completion from the last item that
actually emits `MISSION_ITEM_REACHED` — ArduCopter never emits one for a terminal
`LOITER_UNLIM` — so it triggers on the item *before* it (usually the final waypoint's
DELAY). That way the last photo and its delay both complete.

**SUAS mission layout.** `TAKEOFF` (50 m) -> the required waypoint lap -> optional
`DO_CHANGE_SPEED` -> mapping grid (`NAV_WAYPOINT` + `NAV_DELAY 4 s` each) -> terminal
`NAV_LOITER_UNLIM`. At startup choose the **first mapping waypoint** as the mapping start
sequence: photos and target recording start there, so targets seen on the lap are ignored.
Keep `NAV_DELAY >= STABILIZE_TIMEOUT + 1 s`. Do not put a `DO_` command directly before the
terminal loiter. Restart `run.sh` after uploading a new mission.

---

## 5. Connecting the companion computer

Use the **stable by-id path**, never a bare `/dev/ttyACM0`. Many flight controllers expose
two CDC ports in no guaranteed order, and a brownout can re-enumerate them.

```bash
ls -l /dev/serial/by-id/
```

Pick the MAVLink interface (commonly the one ending `-if00`) and put it in
`MAVLINK_CONNECT_REAL` in `suas/config.py`.

Then, on the companion:

```bash
sudo apt-get remove --purge modemmanager   # it grabs ttyACM devices
sudo usermod -aG dialout $USER             # then log out and back in
```

> Baud rate is meaningless over USB CDC — throughput is USB speed. If you move to a real
> UART (a TELEM port), 57600 becomes a bottleneck; raise it to 921600 in
> `suas/vehicle/connection.py`.

---

## 6. Gimbal

The system assumes a **nadir-pointing camera**. Both `pixel_to_gps` and IBVS depend on it,
so a gimbal that is not pointing down produces garbage coordinates.

**Option A — gimbal wired to the flight controller (preferred).** Configure your gimbal
driver (e.g. `MNT1_TYPE`) so ArduPilot can command and read it. Set
`GIMBAL_MANUAL_NADIR = False` in `suas/config.py`. The companion commands nadir at startup and before each
target, and verifies it through telemetry.

Verify before flying: use a MAVLink inspector to confirm which message your firmware emits
(`GIMBAL_DEVICE_ATTITUDE_STATUS`, `MOUNT_ORIENTATION` or `MOUNT_STATUS`), and that
physical nadir really reads pitch ≈ −90°. A sign error keeps the gate permanently closed
and IBVS will never start.

**Option B — RC-only gimbal.** If the gimbal is controlled purely by RC with no telemetry
to the flight controller, set `GIMBAL_MANUAL_NADIR = True`. The gate then passes
unconditionally and `run.sh` prompts the operator to confirm before every flight. The
operator must point the gimbal fully down before takeoff and leave it there. All other
safety gates are unaffected.

---

## 7. Calibration

Nothing here substitutes for a properly calibrated vehicle:

- Accelerometer (6-position) and level
- Compass, with the external compass as primary
- RC and ESC calibration, plus a props-off motor test
- Battery monitor (voltage multiplier and amps-per-volt)
- Vibration check in a hover

And for this system specifically: **camera field of view**. `CAMERA_HFOV_DEG` defaults to
the manufacturer spec, and both `IBVS_FX` and `pixel_to_gps` derive from it, so a 10%
error becomes metres of geolocation error at a 50 m survey altitude. Measure it:

```bash
python3 tools/calibrate_camera.py capture --cols 9 --rows 6
python3 tools/calibrate_camera.py solve   --cols 9 --rows 6 --square 25
```

---

## 8. Simulation differences

`sim/params/sim.parm` is for SITL and is **not** what you write to a real vehicle. The
notable differences:

| Parameter | Simulation | Real vehicle |
|---|---|---|
| `ARMING_CHECK` | 0 | **1** |
| `FENCE_RADIUS` | 120 | your site |
| `FENCE_ACTION` | 4 (Brake) | your choice |
| `SIM_BATT_VOLTAGE` | set | not applicable |
