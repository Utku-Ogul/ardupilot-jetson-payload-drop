# Safety Model

This system commands a real aircraft and actuates real hardware. Read this document
before flying it.

The design principle throughout is:

> **On genuine danger, STOP and hand control to the operator. Only continue through
> conditions that are known artefacts.**

---

## Three layers

| Layer | Component | Role |
|---|---|---|
| **A** | Companion software (this repo) | Conservative intent. Can be wrong; can crash. |
| **B** | ArduPilot parameters | Bug-resistant clamps. Applied even if the companion misbehaves. |
| **C** | Geofence and failsafes | The last wall. |

Layer A is never trusted on its own. If the companion hangs, crashes or commands
nonsense, layers B and C still bound the aircraft. This is why
[docs/FLIGHT_CONTROLLER.md](FLIGHT_CONTROLLER.md) is not optional.

---

## Companion heartbeat and the GCS failsafe

The flight controller's `SYSID_MYGCS` is the **ground station** (Mission Planner, system id
255), so `FS_GCS` watches the ground link: if its heartbeat is lost for `FS_GCS_TIMEOUT`
(15 s, SUAS 2026 Appendix B) the vehicle returns to launch in every mode (`FS_OPTIONS=0`).
RC loss is handled separately by `FS_THR_ENABLE`, and 90 s of comms loss terminates the
flight via [`lua/comms_loss_termination.lua`](../lua/README.md).

The companion still sends a 1 Hz heartbeat as system id **254** (distinct from the ground
station), but it is **not** the `FS_GCS` source. If the companion hangs or crashes in
GUIDED, the flight-controller-side brake is `GUID_TIMEOUT`: the velocity setpoint is
dropped after at most 3 s and the vehicle stops and holds; the pilot takes over.

The heartbeat is emitted only while BOTH loops are alive -- the main loop (`mark_alive()`)
and the velocity-commanding tracker loop (`mark_alive_worker()`) -- so the companion is
only reported alive when it really is, and the deadman semantics return if `SYSID_MYGCS`
is ever set back to 254.

The companion's commands are accepted because `SYSID_ENFORCE=0`. With `1` the flight
controller only obeys the ground station and silently rejects every GUIDED, goto and
velocity command from the companion -- so preflight treats it as critical.

---

## Startup gate

On a real vehicle (`SIM_MODE = False`), `preflight_param_check()` reads the flight
controller's safety parameters. If a **critical** one is wrong *or unreadable*, `main.py`
exits before starting the tracker.

Critical (blocking):

- `FENCE_ENABLE >= 1`
- `0 < GUID_TIMEOUT <= 3`
- `SYSID_ENFORCE == 0`

An unreadable parameter also blocks. On a healthy link these read reliably at startup; if
they do not, there is a link or configuration problem, and staying on the ground is the
correct outcome.

Warnings only (loud, non-blocking): `FS_GCS_ENABLE`, `FS_OPTIONS` (bit 1 must be off),
`ARMING_CHECK`, `ANGLE_MAX`, `FS_EKF_ACTION`, `FS_THR_ENABLE`, `FS_VIBE_ENABLE`,
`BATT_MONITOR`, `BATT_FS_LOW_ACT`, and `SYSID_MYGCS` (expected 255; 254 means `FS_GCS`
would watch the companion instead of the ground station).

> `ARMING_CHECK = 0` lets the vehicle arm without its normal pre-arm validation. It is a
> warning rather than a block by choice; making it blocking is a one-line change in
> `suas/vehicle/connection.py::preflight_param_check`.

---

## Telemetry validity gates

Every gate is **fail-closed** and includes a freshness check, so a frozen stream is never
mistaken for a healthy one.

| Gate | Requires |
|---|---|
| `have_fix` | Fresh position, non-zero coordinates, 3D fix, `>= POSE_MIN_SATS` satellites, HDOP < 1.4 |
| `ekf_ok` | Fresh EKF report, attitude + horizontal velocity + absolute horizontal position flags, low variance |
| `link_alive` | Autopilot heartbeat within `LINK_TIMEOUT_S` |
| `gimbal_nadir_ok` | A measurement exists, is fresh, and pitch is within `GIMBAL_NADIR_TOL_DEG` of −90° |
| `fire_tilt_ok` | Fresh attitude and tilt below `FIRE_TILT_MAX_DEG` |
| `camera.stale` | A frame arrived within `CAMERA_STALE_S` |

Two details worth knowing:

- **Heartbeats are filtered by source.** Only the actual autopilot
  (`MAV_COMP_ID_AUTOPILOT1`) counts. On a shared bus (mavlink-router, telemetry radio,
  multi-endpoint SITL) a foreign ground-station heartbeat would otherwise keep the link
  deadman falsely alive and mask the real mode and armed state.
- **Gimbal attitude is filtered by system id.** It is the only positive input to a
  fail-closed gate, so a foreign packet must not be able to fake "nadir". The filter is on
  system only — a legitimate gimbal's component id varies by firmware, and filtering that
  too would keep the gate permanently closed.

---

## Visual servoing watchdogs

Any of these aborts IBVS with zero velocity:

| Watchdog | Threshold | On a real vehicle | In simulation |
|---|---|---|---|
| Timeout | `IBVS_TIMEOUT_S` (30 s) | defer, try the next target | same |
| Detection loss | `IBVS_LOSS_GRACE_S` (5 s) | defer, try the next target | same |
| Target geofence | `IBVS_TARGET_FENCE_M` (12 m) | **hard stop → hover** | recoverable |
| Divergence | error above its minimum for `IBVS_DIVERGE_S` | **hard stop → hover** | recoverable |
| Not GUIDED | any mode change | **hard stop → hover** | same |
| GPS/EKF unhealthy | fail-closed | **hard stop → hover** | same |
| Gimbal not at nadir | fail-closed | **hard stop → hover** | gate unused |
| Altitude envelope | outside `IBVS_ALT_MIN..MAX` | **hard stop → hover** | same |

The geofence is anchored to the **saved target**, not to where IBVS started. Centring
means *approaching* the target, so displacement from the start point grows along a
perfectly legitimate approach and used to trip early. Distance *from the target* only
grows during a real flyaway.

The divergence margin is altitude-scaled and takes the maximum of a fixed floor, a
pixel-noise term and a braking-pitch term, so it behaves identically at any altitude and
resolution rather than tripping spuriously.

---

## Payload release gates

Nothing actuates unless **every** condition below holds. The gates are re-evaluated at
about 2 Hz throughout the settling window, so a single-frame glitch does not permanently
lose a target.

| Gate | Normal path | Blind GPS drop |
|---|:---:|:---:|
| `GPIO_ENABLED` and `PAYLOAD_ENABLED` | yes | yes |
| Link alive | yes | yes |
| GPS fix fresh and valid | yes | yes |
| EKF healthy | yes | yes |
| Mode is GUIDED | yes | yes |
| Vehicle armed | yes | yes |
| Attitude sane and fresh | yes | yes |
| Altitude within `PAYLOAD_ALT_MIN..MAX` | yes | yes |
| Within the release radius | `IBVS_TARGET_FENCE_M` | `GPS_DROP_FIRE_RADIUS_M` |
| Not moving | `ibvs.speed <= IBVS_LATCH_SPEED` | `ground_speed <= GPS_DROP_MAX_SPEED` |
| Detection class matches the target | yes | **skipped** |
| Detection confidence `>= FIRE_MIN_CONF` | yes | **skipped** |
| IBVS reports centred | yes | **skipped** |

Plus, independently: **`SIM_MODE = True` blocks all real actuation**, even if the flags
are left enabled. That is a deliberate second layer so a simulation or bench run cannot
drive the mechanism.

### Why the blind drop is safe

It skips only the *identity* and *centring* gates. The target was already confirmed during
the survey (enough samples above `CANDIDATE_MIN_CONF`), and every *vehicle safety* gate
still applies. Genuine danger — link loss, an operator mode change, unhealthy GPS or EKF,
a gimbal not at nadir, being outside the altitude envelope, a real flyaway — results in
hovering and never a blind drop.

### Why the tilt gate is loose

In wind the aircraft **leans** to hold position: ground speed is zero but the body is
tilted. A strict 3° gate reads that legitimate lean as "not stable" and can block the
release indefinitely.

So the release is not gated on tilt magnitude. Approval already requires either the IBVS
centring latch or a settled hover over the saved position, and the speed gates prevent
releasing while moving. `FIRE_TILT_MAX_DEG` (35°) is only a sanity ceiling against
physically impossible attitudes, and the frozen-attitude protection is retained.

The strict 3° threshold is still used — but only for **mapping photos**, where it does no
harm.

---

## The payload process

The release sequence runs as a **separate process**, which matters for safety:

- Its GPIO setup and cleanup cannot disturb the indicator pins owned by the main process.
- It installs `PR_SET_PDEATHSIG` plus a `getppid()` watchdog, so if the tracker dies the
  motor is de-energised rather than left running at full power.
- `SIGTERM`, `SIGINT`, a crash and a parent death all de-energise the motor (both H-bridge
  inputs low).
- An `atexit` hook and a hard-cap thread cut the motor after the maximum run time even if
  the main thread hangs.
- The motor soft-starts through a ramp, avoiding the inrush current that can brown out the
  companion computer.
- The software PWM is deliberately **lock-free**: a single float assignment is atomic under
  the GIL, so a signal handler firing mid-call cannot deadlock the de-energise path.

The tracker terminates it with `SIGTERM` → wait → `SIGKILL`, and never overwrites a live
process handle.

---

## Shutdown behaviour

On exit (Ctrl+C, `SIGTERM`, or an unhandled error) the system:

1. Sends zero velocity three times, guarding against packet loss.
2. Switches to `SHUTDOWN_SAFE_MODE` (`BRAKE`) **only if the companion still owns GUIDED**.
   If the pilot has taken over with RTL, LOITER or STABILIZE, the mode is left alone —
   never hijack the operator's recovery mode.
3. Stops the tracker, closes the camera, terminates the payload child and drives all GPIO
   outputs low.

---

## Before you arm

1. `GPIO_ENABLED = False` and `PAYLOAD_ENABLED = False` for every flight until the payload
   has been bench tested with `tools/payload_test.py`.
2. Verify the axis signs (`IBVS_SIGN_FWD`, `IBVS_SIGN_RIGHT`) in SITL **before** flying
   IBVS. An inverted sign drives the aircraft away from the target until a watchdog stops
   it.
3. Confirm `SIM_MODE` matches reality. The startup banner states it loudly.
4. Have a pilot on the sticks with a mode switch ready. Every state respects an operator
   mode change by parking.

See [docs/PREFLIGHT.md](PREFLIGHT.md) for the full checklist.
