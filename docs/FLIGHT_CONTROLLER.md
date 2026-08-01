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
| `MAV_GCS_SYSID` | **254** | The companion deadman never works. On firmware < 4.5 this is `SYSID_MYGCS`. |
| `FENCE_ENABLE` | **1** | No last-wall geofence. **Blocks startup.** |
| `GUID_TIMEOUT` | **3** | A stalled companion leaves a live velocity setpoint running. **Blocks startup.** |
| `SERIAL0_PROTOCOL` | **2** (MAVLink2) | The companion cannot talk over USB. |
| `ARMING_CHECK` | **1** | Arming with an uncalibrated or unhealthy vehicle. |

`MAV_GCS_SYSID = 254` is the single most important line here. The companion connects with
`source_system = 254`, and that is what makes its heartbeat a real deadman rather than
decoration.

---

## 2. Failsafe and fence

| Parameter | Suggested | Meaning |
|---|---|---|
| `FS_GCS_ENABLE` | 1 | Act when the companion heartbeat stops |
| `FS_GCS_TIMEOUT` | 5 | Seconds from heartbeat loss to failsafe |
| `FENCE_TYPE` | 3 | Circle + altitude ceiling |
| `FENCE_ACTION` | 4 (Brake) or 1 (RTL) | What happens at the boundary |
| `FENCE_RADIUS` | site dependent | Horizontal limit from home |
| `FENCE_ALT_MAX` | survey altitude + margin | Altitude ceiling |
| `FENCE_MARGIN` | 6 | Stopping-distance allowance |
| `FS_EKF_ACTION` | 1 | Land if the EKF diverges |
| `FS_EKF_THRESH` | 0.8 | EKF failsafe threshold |
| `FS_THR_ENABLE` | 1 | RC loss behaviour |
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

The visual servoing limits in `config.json` must stay **below** the autopilot's
navigation limits, so IBVS is always the gentler of the two.

| Parameter | Suggested | Paired config value |
|---|---|---|
| `WPNAV_SPEED` | 150 (1.5 m/s) | `IBVS_MAX_VEL`: 0.8 |
| `WPNAV_ACCEL` | 50 (0.5 m/s²) | `IBVS_SLEW`: 0.5 |
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

Add a `WP_DELAY` of a few seconds to each waypoint if you want stabilised mapping photos.

---

## 5. Connecting the companion computer

Use the **stable by-id path**, never a bare `/dev/ttyACM0`. Many flight controllers expose
two CDC ports in no guaranteed order, and a brownout can re-enumerate them.

```bash
ls -l /dev/serial/by-id/
```

Pick the MAVLink interface (commonly the one ending `-if00`) and put it in
`MAVLINK_CONNECT_REAL` in `config.json` (section `flight_controller`).

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
`"GIMBAL_MANUAL_NADIR": false` in `config.json`. The companion commands nadir at startup and before each
target, and verifies it through telemetry.

Verify before flying: use a MAVLink inspector to confirm which message your firmware emits
(`GIMBAL_DEVICE_ATTITUDE_STATUS`, `MOUNT_ORIENTATION` or `MOUNT_STATUS`), and that
physical nadir really reads pitch ≈ −90°. A sign error keeps the gate permanently closed
and IBVS will never start.

**Option B — RC-only gimbal.** If the gimbal is controlled purely by RC with no telemetry
to the flight controller, set `"GIMBAL_MANUAL_NADIR": true`. The gate then passes
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
error becomes metres of geolocation error at 20 m altitude. Measure it:

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
