"""MAVLink connection, companion heartbeat (GCS failsafe deadman) and preflight checks."""
import threading
import time

from pymavlink import mavutil

from suas.config import (MAVLINK_CONNECT, SIM_MODE, CAMERA_RTSP_URL,
                         GPIO_ENABLED, PAYLOAD_ENABLED, GIMBAL_MANUAL_NADIR)
from suas.utils.logger import get_logger

log = get_logger("connection")

# Liveness timestamps. The heartbeat daemon only emits a GCS heartbeat while BOTH are
# fresh: the main loop (orchestration) AND the tracker (which commands velocity). That
# way, if the TRACKER thread hangs while a live IBVS setpoint is in flight, the heartbeat
# also stops -> the flight controller brakes on GUID_TIMEOUT, then RTLs on FS_GCS.
# (Feeding it from main alone would not catch a tracker hang.)
_last_alive = [0.0]          # main loop liveness
_last_alive_worker = [0.0]   # tracker (velocity-commanding) loop liveness
_HB_STALE_S = 1.5


def mark_alive():
    """Called every iteration of the main/control loop (companion deadman heartbeat)."""
    _last_alive[0] = time.monotonic()


def mark_alive_worker():
    """Called every iteration of the tracker loop. If the tracker hangs the heartbeat
    stops, and the live IBVS setpoint is cut by the flight controller's failsafe."""
    _last_alive_worker[0] = time.monotonic()


def _heartbeat_loop(mav):
    lock = getattr(mav, "send_lock", None)
    while True:
        now = time.monotonic()
        if (now - _last_alive[0]) < _HB_STALE_S and (now - _last_alive_worker[0]) < _HB_STALE_S:
            try:
                if lock is not None:
                    lock.acquire()
                try:
                    mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                finally:
                    if lock is not None:
                        lock.release()
            except Exception:
                pass
        time.sleep(1.0)


def start_companion_heartbeat(mav):
    """Start the 1 Hz GCS heartbeat daemon that feeds the FC-side FS_GCS deadman.

    The companion (sysid 254) must be distinct from the ground station (255) for this
    to be a real deadman.
    """
    mark_alive()
    mark_alive_worker()
    threading.Thread(target=_heartbeat_loop, args=(mav,), daemon=True).start()
    log.info("Companion GCS heartbeat started (sysid=254, 1 Hz -- FS_GCS deadman)")


def connect(endpoint: str = None) -> mavutil.mavfile:
    """Open the MAVLink link.

    endpoint overrides config (main.py passes --connect); None uses MAVLINK_CONNECT.
    """
    endpoint = endpoint or MAVLINK_CONNECT
    if not endpoint:
        key = "MAVLINK_CONNECT_SIM" if SIM_MODE else "MAVLINK_CONNECT_REAL"
        raise SystemExit(
            f"{key} is empty. Set it in config.json (section flight_controller), "
            f"or pass --connect.\n"
            "  Serial: run  ls -l /dev/serial/by-id/  and use the full by-id path\n"
            "          (usually the interface ending in -if00), not /dev/ttyACM0.\n"
            "  Network: use e.g. udpin:0.0.0.0:14550 or tcp:127.0.0.1:5760.")
    log.info("Connecting MAVLink: %s", endpoint)
    # Unique GCS identity (254), separate from vehicle (1) and ground station (255).
    # Must match MAV_GCS_SYSID on the flight controller; this makes the companion its
    # own deadman.
    #
    # A USB/serial link can drop on brownout, cable movement or re-enumeration, so we
    # enable autoreconnect for serial devices. UDP (SITL) does not accept that kwarg
    # (TypeError), so only add it for serial device paths.
    is_serial = not any(endpoint.startswith(p) for p in ("udp", "tcp"))
    kw = dict(source_system=254,
              source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
              baud=57600)   # NOTE: baud is meaningless over USB CDC (throughput is USB
                            # speed). If you move to a real UART (TELEM), 57600 becomes a
                            # bottleneck -> raise it to 921600.
    if is_serial:
        kw["autoreconnect"] = True
    mav = mavutil.mavlink_connection(endpoint, **kw)
    # Several threads (main + tracker + mapping) read the same connection -> serialise recv.
    mav.recv_lock = threading.Lock()
    # The same threads also WRITE to it (1 Hz heartbeat + tracker velocity + mission +
    # gimbal) -> serialise sends too, as a defence against frame interleaving on a real
    # serial link.
    mav.send_lock = threading.Lock()

    # Report progress instead of hanging silently: wait for a heartbeat, but say what we
    # are waiting for every 5 seconds.
    waited = 0
    while mav.wait_heartbeat(timeout=5) is None:
        waited += 5
        if is_serial:
            log.warning("Waiting for heartbeat (%ds)... Does the path '%s' exist "
                        "(ls -l /dev/serial/by-id/)? Is the FC powered? Is ModemManager "
                        "removed? Is the user in the 'dialout' group?", waited, endpoint)
        else:
            log.warning("Waiting for heartbeat (%ds)... Is SITL running? Is the network "
                        "route to the simulator up?", waited)

    log.info("Heartbeat received -- system=%d component=%d",
             mav.target_system, mav.target_component)
    return mav


def _read_param(mav, name, timeout=1.5):
    """Read a single FC parameter (PARAM_REQUEST_READ -> PARAM_VALUE). None if absent."""
    lock = getattr(mav, "recv_lock", None)
    if lock:
        lock.acquire()
    try:
        mav.mav.param_request_read_send(
            mav.target_system, mav.target_component, name.encode(), -1)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg is None:
                break
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode(errors="ignore")
            if pid.rstrip("\x00") == name:
                return msg.param_value
        return None
    except Exception as e:
        log.warning("Parameter read failed (%s): %s", name, e)
        return None
    finally:
        if lock:
            lock.release()


def preflight_param_check(mav):
    """Read the FC safety parameters before flight.

    Every mismatch is logged loudly. The CRITICAL set (fence / deadman / arming:
    FENCE_ENABLE, GUID_TIMEOUT, MAV_GCS_SYSID == 254) is BLOCKING on a real vehicle:
    if this returns False, main.py refuses to start the tracker. In SIM it only warns.

    Returns: crit_ok (bool). The single most important check is MAV_GCS_SYSID == 254 --
    without it the companion FS_GCS deadman never works.

    NOTE: an unreadable critical parameter also blocks. On a healthy USB link these read
    reliably at startup; if they do not, there is a link or parameter problem and staying
    on the ground is the correct outcome.
    """
    # Profile banner + SIM_MODE human-error gate.
    # If SIM_MODE is accidentally left True on a real flight, flyaway/divergence aborts
    # degrade from hard-stop to "continue", and the gimbal-nadir gate is skipped. The
    # camera would also point at the simulator IP (so it shows up early), but warn loudly.
    log.warning("=" * 64)
    log.warning("PROFILE -> SIM_MODE=%s  (%s)", SIM_MODE,
                "SIMULATION" if SIM_MODE else "REAL VEHICLE")
    log.warning("Camera source: %s", CAMERA_RTSP_URL)
    if SIM_MODE:
        log.warning('!! SIM_MODE=True -- set "SIM_MODE": false in config.json for real flight !!')
        log.warning("   (While True: flyaway/divergence aborts continue instead of parking,")
        log.warning("    the gimbal-nadir safety gate is skipped, camera uses the sim URL.)")
    elif GPIO_ENABLED and PAYLOAD_ENABLED:
        log.warning("!! PAYLOAD ARMED -- SIM_MODE=False + GPIO_ENABLED + PAYLOAD_ENABLED !!")
        log.warning("   At AT_TARGET / blind drop the REAL servo and motor will actuate.")
        log.warning("   If the payload is not wired, bench-tested and intended for this")
        log.warning("   flight, set both flags to False.")
    if not SIM_MODE and GIMBAL_MANUAL_NADIR:
        log.warning("! MANUAL GIMBAL NADIR -- the operator must point the gimbal down via RC")
        log.warning("  (lock mode holds it). No FC gimbal telemetry -> the gimbal_nadir_ok")
        log.warning("  verification is SKIPPED. Confirm the down-facing view before takeoff.")
    log.warning("=" * 64)

    ok_all = True
    crit_ok = True
    # (parameter, predicate, expected description, CRITICAL?)
    # 'critical' entries are BLOCKING on a real vehicle (companion deadman + fence + arming
    # core). The rest (batt/ekf/vibe/angle) are loud warnings only.
    checks = (
        ("FENCE_ENABLE", lambda v: v >= 1, ">=1", True),
        ("GUID_TIMEOUT", lambda v: 0 < v <= 3, "0 < v <= 3s", True),
        # Non-blocking by choice: with FS_GCS_ENABLE=0 a companion dropout leaves the
        # vehicle hovering (via GUID_TIMEOUT) instead of triggering RTL. Set to True below
        # if you want RTL enforced.
        ("FS_GCS_ENABLE", lambda v: v >= 1, ">=1", False),
        ("ARMING_CHECK", lambda v: int(round(v)) != 0, "!= 0", False),
        ("ANGLE_MAX", lambda v: v <= 3000, "<=3000 cdeg (30 deg)", False),
        ("FS_EKF_ACTION", lambda v: v >= 1, ">=1", False),
        ("FS_THR_ENABLE", lambda v: v >= 1, ">=1", False),
        ("FS_VIBE_ENABLE", lambda v: v >= 1, ">=1", False),
        ("BATT_MONITOR", lambda v: v > 0, ">0", False),
        ("BATT_FS_LOW_ACT", lambda v: v >= 1, ">=1", False),
    )
    for name, good, desc, crit in checks:
        v = _read_param(mav, name)
        if v is None:
            log.warning("PREFLIGHT: could not read %s -- verify manually", name)
            ok_all = False
            if crit:
                crit_ok = False
        elif not good(v):
            log.warning("PREFLIGHT WARNING: %s=%.2f (expected %s) -- safety backstop is WRONG",
                        name, v, desc)
            ok_all = False
            if crit:
                crit_ok = False

    gcs = _read_param(mav, "MAV_GCS_SYSID")
    if gcs is None:
        gcs = _read_param(mav, "SYSID_MYGCS")   # parameter name on firmware < 4.5
    if gcs is None:
        log.warning("PREFLIGHT: could not read MAV_GCS_SYSID/SYSID_MYGCS -- verify it is 254")
        ok_all = False
        crit_ok = False
    elif int(round(gcs)) != 254:
        log.warning("PREFLIGHT WARNING: GCS_SYSID=%d != 254 -> the companion FS_GCS deadman "
                    "will NOT work", int(round(gcs)))
        ok_all = False
        crit_ok = False

    if ok_all:
        log.info("PREFLIGHT: FC safety parameters OK (fence/deadman/arming/batt/ekf/vibe)")
    elif not crit_ok and not SIM_MODE:
        log.warning("PREFLIGHT: critical parameter(s) wrong or unreadable -> real flight is "
                    "BLOCKED (see main.py).")
    return crit_ok
