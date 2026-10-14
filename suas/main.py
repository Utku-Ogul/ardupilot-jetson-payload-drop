"""
Entry point.

Usage:
  python3 -m suas.main [options]

Options:
  --connect STR       MAVLink connection string (default: config.MAVLINK_CONNECT)
  --no-mapping        Disable waypoint photo capture
  --no-display        Do not open the camera preview window (headless / SSH)
  --photo-start-wp N  Mapping start sequence: photo capture AND target recording start here
  --no-ask-mapping    Skip the interactive pre-flight mapping prompt
"""
import argparse
import signal
import sys
import time

from suas.config import (MAVLINK_CONNECT, SHUTDOWN_SAFE_MODE, SIM_MODE,
                         GIMBAL_MANUAL_NADIR, PHOTO_START_WP)
from suas.detection.camera import CameraReader
from suas.mission import MissionWatcher
from suas.payload import indicators
from suas.tracker import State, SuasTracker
from suas.utils.logger import get_logger
from suas.vehicle import control
from suas.vehicle.connection import (connect, mark_alive, preflight_param_check,
                                     start_companion_heartbeat)
from suas.vehicle.pose import PoseReader

log = get_logger("main")


def _setup_mapping_start(mission, default_start: int, ask: bool) -> int:
    """Interactive PRE-FLIGHT selection of where mapping begins.

    Flow:
      1. The operator uploads the mission from the ground station.
      2. Pressing Enter downloads the mission and lists it.
      3. The operator enters the mapping start sequence. From that item on, photos are
         taken AND the tracker records mannequin/tent candidates. Before it (takeoff,
         waypoint lap, transit) the detector runs but nothing is recorded.

    If ask is False (--no-ask-mapping), or there is no terminal (e.g. a systemd service),
    this prompts for nothing and returns default_start. An SSH session has a terminal, so
    the prompt also appears with --no-display.
    """
    if not ask or not sys.stdin.isatty():
        if not sys.stdin.isatty():
            log.warning(
                "No TTY -> skipping the interactive mapping-start prompt. To set the "
                "starting waypoint explicitly use --photo-start-wp N (using seq>=%d for now).",
                default_start)
        return default_start

    print()
    print("--- MAPPING START (pre-flight setup) ---------------------------------")
    print("Upload the mission from your ground station, then press ENTER.")

    # Keep retrying until a non-empty mission downloads.
    while True:
        try:
            cmd = input("Mission uploaded -> ENTER  |  skip this prompt -> 'q': ").strip().lower()
        except EOFError:
            return default_start
        if cmd == "q":
            log.info("Mapping start skipped -> default (seq>=%d)", default_start)
            return default_start
        if mission.download() and mission.items:
            mission.mark_downloaded()          # do not re-download in flight
            break
        print("! Mission empty or unavailable. Upload it and try again.")

    print()
    print(mission.describe())
    print("[WP] = NAV_WAYPOINT (photo point) | <- LAST = end of mission")
    print("Choose the FIRST MAPPING waypoint (not the lap): nothing before it is recorded.")
    first_wp = mission.first_waypoint_seq()
    print()

    while True:
        try:
            raw = input(
                f"Start mapping at which sequence? "
                f"[ENTER = first NAV_WAYPOINT ({first_wp}) | 0 for all]: "
            ).strip()
        except EOFError:
            raw = ""
        if raw == "":
            chosen = first_wp if first_wp is not None else 0
            break
        if raw.isdigit():
            chosen = int(raw)
            break
        print("! Enter a valid number (e.g. 7).")

    n_photo = sum(1 for s in (mission.wp_seqs or []) if s >= chosen)
    print(f"OK: mapping from seq>={chosen} -> {n_photo} NAV_WAYPOINT(s) will be photographed; "
          f"target recording starts when seq {chosen} is reached.")
    print("----------------------------------------------------------------------")
    print()
    return chosen


def main():
    ap = argparse.ArgumentParser(description="SUAS autonomous vision and payload system")
    ap.add_argument("--connect", default=MAVLINK_CONNECT,
                    help="MAVLink connection string (e.g. udpin:0.0.0.0:14560)")
    ap.add_argument("--no-mapping", action="store_true",
                    help="disable waypoint photo capture")
    ap.add_argument("--no-display", action="store_true",
                    help="do not open the camera preview window")
    ap.add_argument("--photo-start-wp", type=int, default=None,
                    help="mapping start sequence: photo capture AND target recording "
                         "start here (skips the prompt)")
    ap.add_argument("--no-ask-mapping", action="store_true",
                    help="skip the interactive pre-flight mapping-start prompt")
    args = ap.parse_args()

    log.info("=" * 32)
    log.info("  SUAS starting")
    log.info("=" * 32)
    log.info("Connection: %s", args.connect)

    mav = connect(args.connect)
    # Critical FC safety parameters. A mismatch BLOCKS a real flight.
    crit_ok = preflight_param_check(mav)
    if not SIM_MODE and not crit_ok:
        log.error("=" * 64)
        log.error("REAL FLIGHT BLOCKED -- critical flight-controller safety parameter(s) wrong.")
        log.error("Verify FENCE_ENABLE>=1 / 0<GUID_TIMEOUT<=3 / SYSID_ENFORCE=0 (see docs/SAFETY.md).")
        log.error("The companion will NOT start.")
        log.error("=" * 64)
        sys.exit(2)      # stay on the ground: no flight without fence/deadman/arming checks

    pose = PoseReader(mav)
    pose.request_streams()               # ask for fresh telemetry (have_fix/ekf_ok/link gates)

    if not SIM_MODE and GIMBAL_MANUAL_NADIR:
        # Manual nadir: the gimbal is not wired to the flight controller, so ArduPilot can
        # neither command nor read it -- commanding nadir and verifying it would be
        # meaningless. The operator points it down by RC before takeoff and lock mode holds
        # it; gimbal_nadir_ok is unconditionally True.
        log.warning("MANUAL GIMBAL NADIR mode: before takeoff, point the gimbal fully DOWN by "
                    "RC and CONFIRM the straight-down view. There is no automatic angle check.")
    elif not SIM_MODE:
        # Real vehicle: command the camera to nadir and VERIFY via telemetry (not
        # fire-and-forget). A lost mount command would leave gimbal_nadir_ok False forever
        # and IBVS would never start, so retry while pumping telemetry.
        control.set_gimbal_nadir(mav)
        t0 = time.time()
        while time.time() - t0 < 8.0:
            pose.update()
            if pose.gimbal_nadir_ok:
                log.info("Gimbal nadir verified (pitch=%.0f deg)", pose.pose.gimbal_pitch_deg)
                break
            control.set_gimbal_nadir(mav)    # idempotent -> keep asserting
            time.sleep(0.5)
        else:
            log.warning("Gimbal nadir NOT verified within 8s (pitch=%.0f deg) -> IBVS MAY NOT "
                        "START. Configure and verify the gimbal driver and its attitude stream "
                        "before flight.", pose.pose.gimbal_pitch_deg)

    start_companion_heartbeat(mav)       # 1 Hz companion heartbeat (sysid 254; not the FS_GCS source)
    cam = CameraReader()

    # Watches the mission: downloads it, exposes waypoint sequences and completion.
    mission = MissionWatcher(mav, pose)

    # Mapping start. Precedence: --photo-start-wp (explicit) > interactive prompt > config.
    # The same sequence starts photo capture AND the tracker's target recording (the
    # recording gate also applies with --no-mapping).
    if args.photo_start_wp is not None:
        start_wp = args.photo_start_wp
        log.info("Mapping start (CLI): WP seq>=%d", start_wp)
    else:
        # Asked whenever there is a terminal -- also with --no-display (SSH is headless but
        # interactive). Skipping it would silently fall back to PHOTO_START_WP.
        start_wp = _setup_mapping_start(mission, PHOTO_START_WP, not args.no_ask_mapping)
        log.info("Mapping start: WP seq>=%d", start_wp)
    if start_wp <= 0:
        log.warning("Mapping start is 0 -> targets are recorded from the START of AUTO, including "
                    "takeoff, the waypoint lap and transit. Pass --photo-start-wp N (the first "
                    "mapping waypoint) unless that is intended.")

    photo = None
    if not args.no_mapping:
        from suas.mapping.photo import PhotoCapture
        photo = PhotoCapture(mav, pose, cam, mission, start_wp=start_wp)
        log.info("Mapping module active")

    # Start the tracker AFTER the interactive setup, so it does not log over the prompts.
    tracker = SuasTracker(mav, pose, cam, show_window=not args.no_display,
                          record_start_wp=start_wp)
    tracker.start()

    # Shutdown: NEUTRALISE the vehicle (leave no live setpoint), park it safely, clean up.
    cleaned = {"done": False}

    def _neutralize_cleanup():
        if cleaned["done"]:
            return
        cleaned["done"] = True
        log.info("Neutralising and cleaning up...")
        for _ in range(3):                      # zero velocity, 3x against UDP loss
            try:
                control.stop(mav)
            except Exception:
                pass
        # Only park if the companion still owns GUIDED. If the pilot has taken over with
        # RTL/LOITER/STABILIZE (mode != GUIDED), do NOT force BRAKE -- never hijack the
        # operator's recovery mode. control.stop is sent unconditionally (the FC ignores it
        # outside GUIDED) and the FC's own GUID_TIMEOUT/FS_GCS already park the vehicle.
        if SHUTDOWN_SAFE_MODE and pose.mode == "GUIDED":
            try:
                control.set_mode(mav, SHUTDOWN_SAFE_MODE)
            except Exception:
                pass
        for fn in (tracker.stop, cam.close, indicators.cleanup):
            try:
                fn()
            except Exception:
                pass

    def _shutdown(sig, frame):
        signal.signal(sig, signal.SIG_IGN)       # ignore repeats (re-entrancy)
        log.info("Shutting down...")
        _neutralize_cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    log.info("Main loop running -- press Ctrl+C to exit")
    last_stream_req = time.monotonic()
    try:
        while True:
            mark_alive()            # companion deadman heartbeat (beats while the loop lives)
            try:
                pose.update()
                # Periodically (idempotently) RE-REQUEST the streams: a lost
                # SET_MESSAGE_INTERVAL, an FC reboot or a USB reconnect stops them, and
                # have_fix/ekf_ok/gimbal_nadir_ok would die silently. ~8 bytes / 10 s of
                # uplink, telemetry only -- it does not touch the control path.
                now_m = time.monotonic()
                if now_m - last_stream_req > 10.0:
                    pose.request_streams()
                    last_stream_req = now_m
                mission.update()    # in AUTO: download the mission and watch for completion

                if photo:
                    photo.listen()

                # Once the mission reaches its final item, send the tracker to the target.
                if tracker.state == State.TARGET_SAVED and mission.complete:
                    tracker.trigger_goto()

                # New AUTO mission: if the tracker returned to WAITING_AUTO, clear the stale
                # 'complete' latch so the second mission does not start early and is
                # re-downloaded.
                if tracker.state == State.WAITING_AUTO and mission.complete:
                    mission.reset()
                    if photo:
                        photo.reset()   # otherwise the second survey captures zero photos
            except Exception:
                # A transient error must NOT kill the main process (that would skip cleanup
                # and leave a live setpoint).
                log.exception("Main loop error -- continuing")
            time.sleep(0.05)
    finally:
        _neutralize_cleanup()


if __name__ == "__main__":
    main()
