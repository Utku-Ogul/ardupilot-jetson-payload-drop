#!/usr/bin/env python3
"""
Automated bench test: companion computer <-> flight controller, on the ground.

Runs the automatable part of the pre-flight checklist and prints PASS/FAIL.

SAFE AND READ-ONLY: it NEVER arms, never sends a motion (velocity) command and never
actuates the payload.
  * Default : link, telemetry stream rates, preflight parameters, health/freshness gates.
  * --gimbal: sends set_gimbal_nadir (the gimbal PHYSICALLY moves) and verifies the
              nadir telemetry.
  * --camera: opens the video stream and checks frames, resolution and staleness.
  * --detect: runs the detector on one frame (heavy).

Arming, props-off, setpoint-sign and payload-rig checks are deliberately NOT here -- they
must stay manual for safety. See docs/PREFLIGHT.md.

Usage (camera tests need the GStreamer-enabled OpenCV, as in run.sh):
  export PYTHONPATH="/usr/lib/python3.10/dist-packages:$PYTHONPATH"
  python3 tools/bench_test.py
  python3 tools/bench_test.py --gimbal --camera
  python3 tools/bench_test.py --connect udpin:0.0.0.0:14560     # against SITL

The connection defaults to config.MAVLINK_CONNECT.
"""
import argparse
import os
import sys
import threading
import time
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil   # noqa: E402

_RESULTS = []


def record(tid, ok, detail=""):
    _RESULTS.append((tid, ok, detail))
    tag = "PASS" if ok is True else ("FAIL" if ok is False else "SKIP")
    print(f"  [{tag}]  {tid:10} {detail}")


def section(name):
    print(f"\n-- {name} " + "-" * max(0, 60 - len(name)))


def bench_connect(conn_str, hb_timeout=15):
    """Bounded connect, so a dead link fails cleanly instead of blocking forever."""
    print(f"Connecting: {conn_str}")
    is_serial = not any(conn_str.startswith(p) for p in ("udp", "tcp"))
    kw = dict(source_system=254,
              source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, baud=57600)
    if is_serial:
        kw["autoreconnect"] = True
    mav = mavutil.mavlink_connection(conn_str, **kw)
    mav.recv_lock = threading.Lock()
    mav.send_lock = threading.Lock()
    t0 = time.time()
    hb = None
    while time.time() - t0 < hb_timeout:
        hb = mav.wait_heartbeat(timeout=5)
        if hb is not None:
            break
    return mav, hb


def main():
    ap = argparse.ArgumentParser(description="Read-only bench test")
    ap.add_argument("--connect", default=None,
                    help="MAVLink connection string (default: config.MAVLINK_CONNECT)")
    ap.add_argument("--duration", type=float, default=8.0,
                    help="stream-rate measurement window [s]")
    ap.add_argument("--gimbal", action="store_true",
                    help="send set_gimbal_nadir and verify the nadir telemetry")
    ap.add_argument("--camera", action="store_true", help="test the camera stream")
    ap.add_argument("--detect", action="store_true", help="run the detector on one frame (heavy)")
    args = ap.parse_args()

    from suas.config import (CAMERA_HEIGHT, CAMERA_WIDTH, FIRE_TILT_MAX_DEG,
                             MAVLINK_CONNECT, POSE_MIN_SATS, SIM_MODE)
    conn = args.connect or MAVLINK_CONNECT

    print("=" * 68)
    print("BENCH TEST -- READ-ONLY (no arming, no motion, no actuation)")
    print(f"SIM_MODE={SIM_MODE}  |  connection={conn}")
    print("=" * 68)

    # --- Link -------------------------------------------------------------
    section("LINK / TELEMETRY")
    mav, hb = bench_connect(conn)
    if hb is None:
        record("LINK-01", False,
               "no heartbeat -- check the link, device path, permissions and ModemManager")
        print("\nCould not establish a link -> cannot continue.")
        return _summary()
    record("LINK-01", True,
           f"heartbeat received: system={mav.target_system} component={mav.target_component}")

    from suas.vehicle import control
    from suas.vehicle.pose import PoseReader
    pose = PoseReader(mav)
    pose.request_streams()

    # --- Stream rates -----------------------------------------------------
    want = {"HEARTBEAT": 1, "GLOBAL_POSITION_INT": 10, "ATTITUDE": 20,
            "GPS_RAW_INT": 2, "SYS_STATUS": 2, "EKF_STATUS_REPORT": 2}
    gimbal_types = ("GIMBAL_DEVICE_ATTITUDE_STATUS", "MOUNT_ORIENTATION", "MOUNT_STATUS")
    print(f"  ...measuring streams for {args.duration:.0f}s")
    cnt = Counter()
    t0 = time.time()
    while time.time() - t0 < args.duration:
        m = mav.recv_match(blocking=True, timeout=1.0)
        if m is not None:
            cnt[m.get_type()] += 1
    dur = time.time() - t0
    for name, target in want.items():
        hz = cnt.get(name, 0) / dur
        ok = hz >= target * 0.5   # suspicious below half the requested rate
        record("LINK-02", ok if cnt.get(name, 0) > 0 else False,
               f"{name}: {hz:.1f} Hz (target ~{target})")
    g_hz = max(cnt.get(t, 0) for t in gimbal_types) / dur
    g_which = max(gimbal_types, key=lambda t: cnt.get(t, 0)) if g_hz > 0 else "-"
    record("GIMB-01", g_hz > 0, f"gimbal message: {g_which} @ {g_hz:.1f} Hz "
           + ("(REQUIRED on a real vehicle unless GIMBAL_MANUAL_NADIR is set)"
              if not SIM_MODE else "(sim: no gimbal, expected)"))

    for _ in range(20):           # refresh pose so the gates see fresh data
        pose.update()
        time.sleep(0.05)

    # --- Preflight parameters ---------------------------------------------
    section("PREFLIGHT PARAMETERS")
    from suas.vehicle.connection import preflight_param_check
    crit_ok = preflight_param_check(mav)
    record("PRE-01", bool(crit_ok),
           "critical parameters OK" if crit_ok
           else "critical parameter wrong or unreadable (a real flight would be BLOCKED)")

    # --- Health / freshness gates ------------------------------------------
    section("GATES (have_fix / ekf_ok / link_alive / gimbal / fire_tilt)")
    # preflight_param_check spends 2-3s reading parameters WITHOUT calling pose.update(),
    # so the link/attitude/gimbal/EKF timestamps went stale. Refresh right before
    # measuring the gates, otherwise link_alive/fire_tilt/gimbal_nadir report a FALSE fail
    # (feed for >=2s to cover the 1 Hz heartbeat).
    for _ in range(40):
        pose.update()
        time.sleep(0.05)
    p = pose.pose
    record("GATE-01", pose.link_alive,
           f"link_alive={pose.link_alive} (mode={pose.mode} armed={pose.armed})")
    record("GATE-02", pose.have_fix,
           f"have_fix={pose.have_fix} (fix={p.fix_type} sats={p.sats} hdop={p.hdop:.2f}; "
           f"needs >={POSE_MIN_SATS} sats and hdop<1.4)")
    record("GATE-03", pose.ekf_ok,
           f"ekf_ok={pose.ekf_ok} (vel_var={p.ekf_vel_var:.2f} comp_var={p.ekf_compass_var:.2f})")
    record("GATE-04", pose.gimbal_nadir_ok if not SIM_MODE else None,
           f"gimbal_nadir_ok={pose.gimbal_nadir_ok} "
           f"(pitch={p.gimbal_pitch_deg:.0f} roll={p.gimbal_roll_deg:.0f})"
           + ("" if not SIM_MODE else "  [SIM: always True]"))
    record("GATE-05", pose.fire_tilt_ok,
           f"fire_tilt_ok={pose.fire_tilt_ok} (pitch={p.pitch_deg:.1f} roll={p.roll_deg:.1f} "
           f"< {FIRE_TILT_MAX_DEG})")

    # --- Gimbal command (optional) -----------------------------------------
    if args.gimbal and not SIM_MODE:
        section("GIMBAL NADIR COMMAND (--gimbal)")
        print("  sending set_gimbal_nadir (the gimbal will PHYSICALLY move)...")
        control.set_gimbal_nadir(mav)
        ok = False
        t0 = time.time()
        while time.time() - t0 < 8:
            pose.update()
            if pose.gimbal_nadir_ok:
                ok = True
                break
            time.sleep(0.3)
        record("GIMB-02", ok, f"nadir verified, pitch={pose.pose.gimbal_pitch_deg:.0f}"
               + ("" if ok else " -> check the gimbal driver, wiring and message sign"))
    elif args.gimbal:
        record("GIMB-02", None, "SIM_MODE=True -> no gimbal, skipped")

    # --- Camera (optional) --------------------------------------------------
    cam = None
    if args.camera:
        section("CAMERA (--camera)")
        try:
            from suas.detection.camera import CameraReader
            cam = CameraReader()
            frame = None
            t0 = time.time()
            while time.time() - t0 < 10 and frame is None:
                frame = cam.read()
                time.sleep(0.1)
            if frame is None:
                record("CAM-01", False, "no frames (check the stream URL and the network)")
            else:
                h, w = frame.shape[:2]
                record("CAM-01", True, f"frame received {w}x{h}")
                record("CAM-02", (w == CAMERA_WIDTH and h == CAMERA_HEIGHT),
                       f"resolution {w}x{h} (config {CAMERA_WIDTH}x{CAMERA_HEIGHT})")
                time.sleep(0.6)
                record("CAM-03", not cam.stale, f"stale={cam.stale} (expecting a fresh stream)")
        except Exception as e:
            record("CAM-01", False, f"could not open the camera: {e}")

    # --- Detection (optional) -----------------------------------------------
    if args.detect and cam is not None:
        section("DETECTION (--detect)")
        try:
            from suas.detection.tiled_yolo import TiledYoloDetector
            det = TiledYoloDetector()
            frame = cam.read()
            dets = det.detect_all(frame) if frame is not None else []
            labels = Counter(d.label for d in dets)
            record("DET-01", True, f"model loaded; detections: {dict(labels)} ({len(dets)} boxes)")
            det.close()
        except Exception as e:
            record("DET-01", False, f"detection error: {e}")

    if cam is not None:
        try:
            cam.close()
        except Exception:
            pass

    return _summary()


def _summary():
    section("SUMMARY")
    p = sum(1 for _, ok, _ in _RESULTS if ok is True)
    f = sum(1 for _, ok, _ in _RESULTS if ok is False)
    s = sum(1 for _, ok, _ in _RESULTS if ok is None)
    print(f"  PASS={p}  FAIL={f}  SKIP={s}  (total {len(_RESULTS)})")
    if f:
        print("  FAILED:", ", ".join(tid for tid, ok, _ in _RESULTS if ok is False))
    print("\n  NOTE: arming, props-off setpoint-sign, deadman and payload-rig checks are")
    print("        MANUAL for safety -- see docs/PREFLIGHT.md.")
    return 0 if f == 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(130)
