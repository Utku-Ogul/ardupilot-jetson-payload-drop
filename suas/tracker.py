"""
Mission state machine.

States:
  WAITING_AUTO  -> waiting for the vehicle to enter AUTO
  MONITORING    -> surveying; detections are clustered into candidates. The detector runs
                   from takeoff, but RECORDING (candidate sampling) only starts once the
                   mission reaches the mapping start item (record_start_wp), so targets
                   seen during takeoff, the waypoint lap or transit are not recorded.
  TARGET_SAVED  -> a target position is saved; waiting for the mission to finish
  GPS_GOTO      -> flying to the target's GPS position in GUIDED
  IBVS_ACTIVE   -> visual servoing to centre on the target
  GPS_DROP      -> last resort: IBVS could not centre, fly to the saved GPS and drop blind
  AT_TARGET     -> centred; settle, then release the payload
  WAIT_HOLD     -> hold over the target, then move to the next one
  HOVERING      -> stopped, waiting for the operator
"""
import math
import os
import subprocess
import sys
import threading
import time
from enum import Enum, auto
from typing import Optional

import cv2

from suas.config import (ACQUIRE_MAX_SAMPLES, ACQUIRE_MAX_SPEED, ACQUIRE_MIN_SAMPLES,
                         ACQUIRE_REFINE,
                         ACQUIRE_TILT_GATE, ACQUIRE_TILT_MAX_DEG,
                         AFTER_VISIT_ACTION, CAMERA_HEIGHT, CAMERA_WIDTH,
                         CANDIDATE_MIN_CONF, DETECT_CONFIRM_SEC, DETECT_LOST_GRACE,
                         FIRE_MIN_CONF, FIRE_SETTLE_MAX_S, FIRE_SETTLE_MIN_S,
                         GOTO_MIN_ALT_M, GOTO_RADIUS_M, GOTO_SETTLE_MAX_SPEED,
                         GOTO_TIMEOUT_S,
                         GPIO_ENABLED, GPS_DROP_FALLBACK, GPS_DROP_FIRE_RADIUS_M,
                         GPS_DROP_MAX_SPEED, GPS_DROP_SETTLE_MAX_S, GPS_SETTLE_SEC,
                         IBVS_ALT_MAX, IBVS_ALT_MIN, IBVS_CENTER_TOL_ATTEMPTS_TOTAL,
                         IBVS_CENTER_TOL_LADDER, IBVS_CENTER_TOL_M,
                         IBVS_DIVERGE_MARGIN_M, IBVS_DIVERGE_MARGIN_PX,
                         IBVS_DIVERGE_PITCH_DEG, IBVS_DIVERGE_S, IBVS_FX,
                         IBVS_LATCH_SPEED, IBVS_LOSS_GRACE_S, IBVS_TARGET_FENCE_M,
                         IBVS_TIMEOUT_S, PAYLOAD_ALT_MAX, PAYLOAD_ALT_MIN,
                         PAYLOAD_ENABLED, PAYLOAD_REVERSE, PAYLOAD_TARGETS, SIM_MODE,
                         STREAM_BITRATE, STREAM_ENABLED, STREAM_FPS, STREAM_HEIGHT,
                         STREAM_HOST, STREAM_IDR_SEC, STREAM_PORT, STREAM_WIDTH,
                         TARGET_CLUSTER_RADIUS_M, TARGET_MAX_ATTEMPTS, TARGET_WAIT_SEC,
                         TILING_IN_IBVS, TILING_IN_SURVEY)
from suas.detection.base import Detection
from suas.navigation.geo import gps_distance_m, pixel_to_gps
from suas.navigation.ibvs import IBVSController
from suas.utils.logger import get_logger
from suas.vehicle import control
from suas.vehicle.connection import mark_alive_worker

log = get_logger("tracker")

# Repository root, used to launch the payload drop sequence as a module.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class State(Enum):
    WAITING_AUTO = auto()
    MONITORING = auto()
    TARGET_SAVED = auto()
    GPS_GOTO = auto()
    IBVS_ACTIVE = auto()
    GPS_DROP = auto()    # last resort: IBVS failed to centre -> saved GPS + blind release
    AT_TARGET = auto()
    WAIT_HOLD = auto()   # hold over the target for TARGET_WAIT_SEC, then move on
    HOVERING = auto()


def _build_detector():
    """Construct the detector. Swap this out to plug in your own (see detection/base.py)."""
    from suas.detection.tiled_yolo import TiledYoloDetector
    return TiledYoloDetector()


class SuasTracker:

    def __init__(self, mav, pose_reader, camera_reader, show_window: bool = True,
                 record_start_wp: int = 0):
        self._mav = mav
        self._pose = pose_reader
        self._cam = camera_reader
        self._show_window = show_window
        self._det = _build_detector()
        self._ibvs = IBVSController()

        self._state: State = State.WAITING_AUTO
        self._target_lat: float = 0.0
        self._target_lon: float = 0.0
        self._target_alt: float = 0.0     # safe altitude captured when the target was saved
        self._target_label: str = ""      # CURRENT target class (IBVS tracks this class)
        self._fire_det: Optional[Detection] = None   # the detection that achieved centring
                                                     # (identity/confidence gate for release)
        self._visit_queue: list = []      # targets still to visit (nearest first)
        self._current_target: Optional[dict] = None  # the queue entry being visited now
                                                     # (kept so a recoverable abort can
                                                     #  push it to the back of the queue)
        self._wait_start: Optional[float] = None     # WAIT_HOLD start time
        self._confirm_start: Optional[float] = None
        self._last_seen: Optional[float] = None      # last detection time (flicker grace)
        self._candidates: list = []       # {samples, n_seen, clat, clon, label}
        self._refine_log: float = 0.0     # TARGET_SAVED leader-log throttle
        self._cand_log: float = 0.0       # MONITORING candidate-table log throttle
        self._guided_req_at: Optional[float] = None  # last set_mode(GUIDED) send time
        self._guided_first: Optional[float] = None   # first GUIDED request (timeout)
        self._goto_sent_at: Optional[float] = None   # last goto send (periodic resend)
        self._goto_start_at: Optional[float] = None  # GPS_GOTO start (overall timeout)
        self._goto_requested: bool = False           # main -> tracker: mission done, go
        self._settle_start: Optional[float] = None
        self._goto_slow_log: float = 0.0  # "waiting to settle" log throttle
        self._drop_arrived_at: Optional[float] = None   # GPS_DROP arrival (settle timer)
        self._fire_settle_start: Optional[float] = None # AT_TARGET settle-then-release window
        self._fire_try_at: float = 0.0    # last release-gate attempt (~2 Hz throttle)
        # IBVS_ACTIVE watchdog state (populated on the GPS_GOTO -> IBVS transition)
        self._ibvs_start_at: Optional[float] = None
        self._ibvs_last_seen: Optional[float] = None
        self._ibvs_err_min: float = float("inf")     # smallest metric error seen
        self._ibvs_div_since: Optional[float] = None # divergence start time
        self._running: bool = False
        self._payload_proc: Optional[subprocess.Popen] = None
        self._payload_started: bool = False
        # Target RECORDING gate (mapping start item). While record_start_wp > 0, MONITORING
        # does not sample candidates until the AUTO mission reaches that item; the detector
        # keeps running and drawing boxes. Once open, it stays open for that AUTO mission.
        self._record_start_wp: int = max(0, int(record_start_wp))
        self._record_open: bool = self._record_start_wp <= 0
        self._monitor_since: float = 0.0  # MONITORING entry time (stale-message filter)

        # Optional live video stream of the annotated frames. Never affects flight.
        self._streamer = None
        if STREAM_ENABLED and not STREAM_HOST:
            log.warning('Live stream is enabled but STREAM_HOST is empty -- set it (or '
                        'GROUND_STATION_IP) in suas/config.py. Streaming stays off.')
        elif STREAM_ENABLED:
            try:
                from suas.streaming.rtp import RtpH264Publisher
                self._streamer = RtpH264Publisher(STREAM_HOST, STREAM_PORT, STREAM_WIDTH,
                                                  STREAM_HEIGHT, STREAM_FPS, STREAM_BITRATE,
                                                  STREAM_IDR_SEC)
                self._streamer.open()
            except Exception:
                log.exception("Could not start the live stream -- continuing (flight unaffected)")
                self._streamer = None

    # --- Public --------------------------------------------------------------

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        log.info("Tracker started")

    def stop(self):
        self._running = False
        for _ in range(3):                 # neutralise: leave no live setpoint
            try:
                control.stop(self._mav)
            except Exception:
                pass
        self._stop_payload()               # de-energise the motor child process
        self._det.close()
        if self._streamer is not None:
            self._streamer.close()
        if self._show_window:
            cv2.destroyAllWindows()

    @property
    def state(self) -> State:
        return self._state

    def trigger_goto(self):
        """Called from main.py when the mission reaches its final item.

        Only meaningful in TARGET_SAVED. This just raises a FLAG; the actual target
        selection and state transition happen on the tracker thread (_commit_goto), so
        _candidates/_target/_state are only ever mutated by one thread.
        """
        if self._state == State.TARGET_SAVED:
            self._goto_requested = True

    # --- Main loop -----------------------------------------------------------

    def _loop(self):
        try:
            while self._running:
                mark_alive_worker()      # tracker alive -> companion heartbeat keeps beating
                try:
                    self._pose.update()
                    frame = self._cam.read()
                    # No image, or a stale (frozen) stream -> treat it as a detection loss,
                    # so IBVS never commands velocity from a frozen frame.
                    if frame is None or self._cam.stale:
                        self._step([])
                        time.sleep(0.02)
                        continue

                    # Tiling off during IBVS (target is large/near -> speed), on during the
                    # survey (recall) -- subject to the config switches.
                    self._set_tiling(self._state == State.IBVS_ACTIVE)
                    dets = self._detect_all(frame)
                    self._step(dets)

                    # Render the annotated frame if the window OR the ground stream needs it
                    # (headless flights still stream). Drawing needs no display.
                    if self._streamer is not None or self._show_window:
                        vis = self._draw(frame, dets)
                        self._overlay(vis)
                        if self._streamer is not None:
                            self._streamer.write(vis)   # write() swallows its own errors
                        if self._show_window:
                            # Keep HighGUI in its OWN try block: a headless imshow failure
                            # must not fall through to the outer handler (which zeroes the
                            # setpoint) and stop IBVS. On error, disable the preview for good.
                            try:
                                cv2.imshow("SUAS Tracker", vis)
                                if cv2.waitKey(1) & 0xFF == ord("q"):
                                    self._running = False
                                    break
                            except Exception:
                                log.warning("Preview window (HighGUI) error -> preview DISABLED "
                                            "(flight unaffected)")
                                self._show_window = False
                except Exception:
                    # A transient error (parse/cv2/telemetry) must not kill the loop.
                    # Fail safe: send zero velocity and carry on, leaving no live setpoint.
                    log.exception("Tracker loop error -- stopping and continuing")
                    try:
                        control.stop(self._mav)
                    except Exception:
                        pass
                time.sleep(0.02)
        finally:
            # However this thread exits: neutralise the vehicle and end the payload child.
            for _ in range(3):
                try:
                    control.stop(self._mav)
                except Exception:
                    pass
            self._stop_payload()
            if self._streamer is not None:
                try:
                    self._streamer.close()
                except Exception:
                    pass

    # --- State machine -------------------------------------------------------

    def _step(self, dets: list):
        s = self._state
        mode = self._pose.mode

        # LINK DEADMAN: in any state that commands motion, a stale telemetry link means we
        # must not command from a stale pose -> stop and hover (the FC-side FS_GCS also applies).
        if s in (State.GPS_GOTO, State.IBVS_ACTIVE, State.GPS_DROP, State.AT_TARGET) \
                and not self._pose.link_alive:
            log.warning("Link lost (telemetry stale) -> stop + HOVERING")
            for _ in range(3):
                control.stop(self._mav)
            self._state = State.HOVERING
            return

        if s == State.WAITING_AUTO:
            if mode == "AUTO":
                log.info("AUTO detected -> MONITORING")
                self._state = State.MONITORING
                self._monitor_since = time.time()
                self._record_open = self._record_start_wp <= 0
                if not self._record_open:
                    log.info("Target recording starts at mapping item seq>=%d "
                             "(detector already running)", self._record_start_wp)
                self._confirm_start = None
                self._candidates = []         # new mission -> clear the candidate pool
                self._visit_queue = []
                self._current_target = None   # no leftover deferred target

        elif s == State.MONITORING:
            if mode != "AUTO":
                log.info("AUTO lost -> WAITING_AUTO")
                self._state = State.WAITING_AUTO
                self._confirm_start = None
                self._last_seen = None
                return

            # Nothing is recorded before the mapping start item (takeoff / lap / transit).
            if not self._update_record_gate():
                return

            if dets:
                now = time.time()
                if self._confirm_start is None:
                    self._confirm_start = now
                    log.debug("Detection started, counting toward confirmation...")
                self._last_seen = now
                # Add EVERY detection in this frame to its per-class candidate cluster.
                for d in dets:
                    self._collect_target_sample(d)
                # Visibility: one line when a candidate first becomes ready, plus a
                # ~0.5 Hz table of all candidates (both classes).
                self._announce_new_ready()
                if now - self._cand_log > 2.0:
                    self._cand_log = now
                    log.info("Candidates %s", self._candidate_table())
                # Once the first VALID candidate is ready, move to TARGET_SAVED so
                # main.trigger_goto has something to act on. The FINAL choice is made at the
                # end of the mission (trigger_goto -> _commit_goto), which builds the queue.
                if now - self._confirm_start >= DETECT_CONFIRM_SEC:
                    best = self._best_candidate()
                    if best is not None:
                        self._set_target_from(best, "First candidate ready -> TARGET_SAVED")
                        self._confirm_start = None
                        self._state = State.TARGET_SAVED
            else:
                # Do not reset the timer on a single dropped frame -- allow a short grace.
                # A REAL loss beyond the grace resets the confirmation timer but KEEPS the
                # sample pool, so the next pass keeps accumulating.
                if (self._confirm_start is not None and self._last_seen is not None
                        and time.time() - self._last_seen > DETECT_LOST_GRACE):
                    log.debug("Detection lost for >%.1fs -- timer reset (%d candidates kept)",
                              DETECT_LOST_GRACE, len(self._candidates))
                    self._confirm_start = None
                    self._last_seen = None

        elif s == State.TARGET_SAVED:
            # main.py raises the flag via trigger_goto() when the mission ends; the FINAL
            # selection happens HERE, on the tracker thread.
            if self._goto_requested:
                self._goto_requested = False
                self._commit_goto()
                return
            # Meanwhile, if we see targets again (a new survey pass), keep refining: track
            # the leading candidate so multi-pass averaging reduces single-pass bias.
            if ACQUIRE_REFINE and dets and mode == "AUTO":
                for d in dets:
                    self._collect_target_sample(d)
                self._announce_new_ready()                # second target shows up immediately
                best = self._best_candidate()
                if best is not None:
                    tlat, tlon, nclean, ntot = self._robust_centroid(best["samples"])
                    self._target_lat = tlat
                    self._target_lon = tlon
                    now = time.time()
                    if now - self._refine_log > 1.0:      # ~1 Hz log throttle
                        self._refine_log = now
                        log.info("Leading candidate [%s]: %d frames (%d/%d clean) -> "
                                 "%.6f, %.6f | %s",
                                 best["label"], best["n_seen"], nclean, ntot,
                                 tlat, tlon, self._candidate_table())

        elif s == State.GPS_GOTO:
            now = time.time()

            # 0) Overall arrival timeout. If we enter GUIDED but never reach the target
            # (wind / position bias / unreachable coordinate), do not resend forever.
            if self._goto_start_at is not None and now - self._goto_start_at > GOTO_TIMEOUT_S:
                log.warning("GPS_GOTO: overall timeout (%.0fs) -> deferring this target and "
                            "moving to the next", GOTO_TIMEOUT_S)
                self._goto_start_at = None
                for _ in range(3):     # vehicle is healthy but could not arrive -> stop, defer
                    control.stop(self._mav)
                self._defer_or_skip_current("GPS_GOTO timeout")
                return

            # 1) GUIDED must actually be active first. ArduCopter SILENTLY drops a guided
            #    goto (current=2) that arrives while not in GUIDED, so a one-shot goto
            #    would simply vanish.
            if mode != "GUIDED":
                # If the FC entered a FAILSAFE/protective mode (battery/RC/fence/GCS ->
                # RTL/LAND/BRAKE) during the GUIDED acquisition window, do NOT force GUIDED
                # back and override the failsafe. Park unconditionally.
                if mode in ("RTL", "SMART_RTL", "AUTO_RTL", "LAND", "BRAKE"):
                    log.warning("GPS_GOTO: FC failsafe/protective mode (%s) -> abandoning the "
                                "GUIDED request -> HOVERING", mode)
                    self._goto_sent_at = None
                    self._guided_first = None
                    self._state = State.HOVERING
                    return
                if self._goto_sent_at is not None:
                    # We already sent a goto in GUIDED and the mode then changed -> the
                    # operator intervened.
                    log.warning("Left GUIDED (operator?) -> HOVERING")
                    self._goto_sent_at = None
                    self._state = State.HOVERING
                    return
                if self._guided_first is None:
                    self._guided_first = now
                if now - self._guided_first > 5.0:
                    log.warning("Could not enter GUIDED within 5s -> HOVERING")
                    self._state = State.HOVERING
                    return
                if self._guided_req_at is None or now - self._guided_req_at > 1.0:
                    control.set_mode(self._mav, "GUIDED")   # no ACK wait -> retry periodically
                    self._guided_req_at = now
                return

            # 2) GUIDED is active -> fly to the target. Safety gates first:
            p = self._pose.pose
            if not self._pose.have_fix or not self._pose.ekf_ok:
                log.warning("GPS_GOTO: GPS fix stale/absent or EKF unhealthy -> HOVERING")
                self._state = State.HOVERING
                return
            if self._target_alt <= GOTO_MIN_ALT_M:
                log.warning("GPS_GOTO: unsafe target altitude %.1fm -> HOVERING", self._target_alt)
                self._state = State.HOVERING
                return
            # Resend the goto periodically until we arrive (mode race + packet loss).
            # Use the SAVED safe altitude, so a stale/zero live altitude cannot turn this
            # into a land command.
            if self._goto_sent_at is None:
                self._settle_start = None    # fresh entry -> settle timer starts on ARRIVAL
            if self._goto_sent_at is None or now - self._goto_sent_at > 1.0:
                control.goto_global(self._mav,
                                    self._target_lat, self._target_lon, self._target_alt)
                self._goto_sent_at = now

            # RE-ACQUIRE EN ROUTE: if we see targets on the way, sample them to refresh the
            # positions of DEFERRED targets in the queue. Sampling is attitude/gimbal gated
            # (_collect_target_sample), so a tilted or invalid frame cannot shift a position.
            # This never ADDS targets to the queue -- it only refreshes known ones.
            if dets:
                for d in dets:
                    self._collect_target_sample(d)
                self._refresh_queue_positions()

            dist = gps_distance_m(p.lat, p.lon, self._target_lat, self._target_lon)
            # The arrival settle timer starts ON ARRIVAL, not when the goto was issued --
            # otherwise a long transit exhausts it and we hand over to IBVS while still moving.
            if dist < GOTO_RADIUS_M:
                if self._settle_start is None:
                    self._settle_start = now
            else:
                self._settle_start = None
            # SOFT ENTRY: the minimum settle time alone is not enough -- braking from
            # 5 m/s takes ~2 s / 5 m, so the vehicle may still be moving when it expires and
            # IBVS would start against momentum. Also require a low real ground speed.
            # This only delays; it never commands motion (bounded by GOTO_TIMEOUT_S).
            settled = self._pose.ground_speed <= GOTO_SETTLE_MAX_SPEED
            arrived = (dist < GOTO_RADIUS_M and self._settle_start is not None
                       and now - self._settle_start >= GPS_SETTLE_SEC)
            if arrived and not settled and now - self._goto_slow_log > 2.0:
                self._goto_slow_log = now
                log.info("At target, waiting to settle (ground speed %.2f > %.2f m/s) "
                         "before IBVS", self._pose.ground_speed, GOTO_SETTLE_MAX_SPEED)
            if arrived and settled:
                self._ibvs.reset()
                # Apply this target's CURRENT ladder rung (IBVS_CENTER_TOL_LADDER);
                # _defer_or_skip_current advances it when a rung's budget is used up.
                tol = (self._current_target or {}).get("center_tol", IBVS_CENTER_TOL_M)
                self._ibvs.center_tol = tol
                log.info("Arrived (dist=%.1fm) -> IBVS (centring tolerance=%.1fm)", dist, tol)
                self._goto_sent_at = None
                self._goto_start_at = None
                # Initialise the IBVS watchdogs
                self._ibvs_start_at = now
                self._ibvs_last_seen = now
                self._ibvs_err_min = float("inf")
                self._ibvs_div_since = None
                self._state = State.IBVS_ACTIVE

        elif s == State.IBVS_ACTIVE:
            now = time.time()
            p = self._pose.pose

            # (a) GUIDED / pilot override -- NEVER send velocity outside GUIDED
            if mode != "GUIDED":
                log.warning("IBVS: not in GUIDED (operator intervention?) -> HOVERING")
                return self._abort_ibvs()
            # (b) pose health -- fresh GPS fix, healthy EKF, inside the altitude envelope.
            # have_fix includes a freshness check, which prevents commanding velocity from a
            # FROZEN GLOBAL_POSITION_INT (where the target geofence would be computed from a
            # stale position and never trip).
            if not self._pose.have_fix or not self._pose.ekf_ok:
                log.warning("IBVS: GPS fix stale/absent or EKF unhealthy -> HOVERING")
                return self._abort_ibvs()
            if not SIM_MODE and not self._pose.gimbal_nadir_ok:
                log.warning("IBVS: gimbal not at nadir (pitch=%.0f deg) -> HOVERING",
                            p.gimbal_pitch_deg)
                return self._abort_ibvs()
            if not (IBVS_ALT_MIN <= p.alt <= IBVS_ALT_MAX):
                log.warning("IBVS: outside the altitude envelope (alt=%.1fm) -> HOVERING", p.alt)
                return self._abort_ibvs()
            # (c) timeout
            if now - self._ibvs_start_at > IBVS_TIMEOUT_S:
                log.warning("IBVS: timeout (%.0fs) -> deferring this target and moving on",
                            IBVS_TIMEOUT_S)
                return self._abort_ibvs(recoverable=True)
            # (d) horizontal geofence, measured from the SAVED TARGET (not from the start).
            # Centring means APPROACHING the target, so displacement from the start grows
            # along a legitimate approach and used to trip before arrival. Distance from the
            # target only grows during a REAL flyaway -- that is the true lock.
            dist_to_tgt = gps_distance_m(p.lat, p.lon, self._target_lat, self._target_lon)
            if dist_to_tgt > IBVS_TARGET_FENCE_M:
                # On a real vehicle (where the gimbal makes the saved GPS trustworthy)
                # exceeding this is a REAL flyaway -> HARD STOP and hover: park on a control
                # fault rather than continuing autonomously. In simulation, without a gimbal,
                # it is usually a projection artefact -> recoverable, so the mission survives.
                log.warning("IBVS: distance from target %.1fm > %.1fm (flyaway lock)",
                            dist_to_tgt, IBVS_TARGET_FENCE_M)
                return self._abort_ibvs(recoverable=SIM_MODE)

            # Pick the detection of the CURRENT target class nearest the image centre, so
            # another class or object in the same frame is not followed.
            det = self._pick_ibvs_det(dets)

            # (e) detection loss -- zero velocity during the grace; abort if exceeded
            if det is None:
                self._ibvs.notify_lost()
                # Reset the DIVERGENCE timer on a detection loss. If the gap exceeds
                # IBVS_DIVERGE_S but not IBVS_LOSS_GRACE_S, a stale _ibvs_div_since would
                # trigger a false divergence on reacquisition (divergence requires the error
                # to grow CONTINUOUSLY, and a loss breaks that continuity). A long loss is
                # still caught by the loss grace, and a real flyaway by the target fence.
                self._ibvs_div_since = None
                control.stop(self._mav)
                if now - self._ibvs_last_seen > IBVS_LOSS_GRACE_S:
                    log.warning("IBVS: detection lost for %.1fs -> deferring and moving on",
                                IBVS_LOSS_GRACE_S)
                    return self._abort_ibvs(recoverable=True)
                return
            self._ibvs_last_seen = now

            # Produce the command (body frame, altitude normalised)
            v_fwd, v_right = self._ibvs.compute(det.cx, det.cy, p.alt)

            # (f) divergence detection -- the metric error stays above its minimum too long.
            # The first 2 seconds are exempt (slew ramp-up and settling).
            err = self._ibvs.g_err
            # The margin must use the SAME GSD envelope as g_err (ibvs.compute clamps the
            # altitude to IBVS_ALT_MAX), otherwise it inflates at altitude and blinds the
            # watchdog.
            alt_c = min(max(p.alt, IBVS_ALT_MIN), IBVS_ALT_MAX)
            # margin = max(floor, PIXEL-noise term, PITCH-transient term). The pixel term
            # shrinks with resolution; the pitch term is resolution INDEPENDENT and prevents
            # false divergence at high resolution.
            div_margin = max(IBVS_DIVERGE_MARGIN_M,
                             IBVS_DIVERGE_MARGIN_PX * alt_c / IBVS_FX,
                             alt_c * math.tan(math.radians(IBVS_DIVERGE_PITCH_DEG)))
            if err < self._ibvs_err_min:
                self._ibvs_err_min = err
                self._ibvs_div_since = None
            elif (now - self._ibvs_start_at > 2.0
                  and err > self._ibvs_err_min + div_margin):
                if self._ibvs_div_since is None:
                    self._ibvs_div_since = now
                elif now - self._ibvs_div_since > IBVS_DIVERGE_S:
                    # Divergence means IBVS is making the error worse (bad tracking/control).
                    # On a real vehicle that indicates a fault -> HARD STOP and park.
                    # In simulation it is a tilt/projection artefact -> recoverable.
                    log.warning("IBVS: diverging (error %.1fm > min + %.1fm)", err, div_margin)
                    return self._abort_ibvs(recoverable=SIM_MODE)
            else:
                self._ibvs_div_since = None

            control.send_velocity_body(self._mav, v_fwd, v_right, 0.0)

            if self._ibvs.centered:
                for _ in range(3):        # guarantee the zero against UDP loss
                    control.stop(self._mav)
                self._fire_det = det      # the detection that centred -> release identity gate
                log.info("Target centred (g_err=%.2fm, %s %.2f) -> AT_TARGET",
                         err, det.label, det.confidence)
                self._state = State.AT_TARGET

        elif s == State.GPS_DROP:
            # LAST RESORT: IBVS could not centre this target on any rung of the ladder, but
            # the vehicle is HEALTHY -> fly to the target's SAVED GPS (survey robust
            # centroid), settle, release blind. Same safety gates as GPS_GOTO; genuine
            # DANGER -> HOVERING (never a blind drop). If it cannot be reached, skip it
            # (the ladder is exhausted, so there is no retry loop).
            now = time.time()

            # 0) Overall arrival timeout -- if we cannot reach the saved GPS, we cannot drop
            if self._goto_start_at is not None and now - self._goto_start_at > GOTO_TIMEOUT_S:
                log.warning("GPS-DROP: could not reach the saved GPS (%.0fs) -> no drop, "
                            "skipping this target", GOTO_TIMEOUT_S)
                self._goto_start_at = None
                for _ in range(3):
                    control.stop(self._mav)
                self._current_target = None
                self._start_next_target()
                return

            # 1) GUIDED is REQUIRED here. GPS_DROP is always entered from IBVS_ACTIVE, which
            #    is already in GUIDED, so mode != GUIDED means OPERATOR or FC intervention
            #    (a pilot mode change, or an FC failsafe RTL/LAND). Re-asserting GUIDED like
            #    GPS_GOTO does would be WRONG here: it would fight the operator and violate
            #    the "mode change -> HOVERING, NEVER a blind drop" invariant. Park unconditionally.
            if mode != "GUIDED":
                log.warning("GPS-DROP: not in GUIDED (operator/FC intervention, mode=%s) -> "
                            "stop + HOVERING", mode)
                for _ in range(3):
                    control.stop(self._mav)
                self._goto_sent_at = None
                self._current_target = None
                self._state = State.HOVERING
                return

            # 2) Safety gates -- DANGER means HOVERING, not a blind drop
            p = self._pose.pose
            if not self._pose.have_fix or not self._pose.ekf_ok:
                log.warning("GPS-DROP: GPS fix stale/absent or EKF unhealthy -> HOVERING")
                self._current_target = None
                self._state = State.HOVERING
                return
            if self._target_alt <= GOTO_MIN_ALT_M:
                log.warning("GPS-DROP: unsafe target altitude %.1fm -> HOVERING", self._target_alt)
                self._current_target = None
                self._state = State.HOVERING
                return

            # 3) Resend the goto until we arrive (using the SAVED safe altitude).
            dist = gps_distance_m(p.lat, p.lon, self._target_lat, self._target_lon)
            if dist >= GOTO_RADIUS_M:
                self._drop_arrived_at = None             # not there yet -> clear any arrival
                if self._goto_sent_at is None or now - self._goto_sent_at > 1.0:
                    control.goto_global(self._mav,
                                        self._target_lat, self._target_lon, self._target_alt)
                    self._goto_sent_at = now
                return

            # 4) ARRIVED -> zero velocity (hold and decelerate). Tie the settle timer to the
            #    ARRIVAL, not to entering the state: wait until the vehicle has genuinely
            #    stopped (minimum GPS_SETTLE_SEC and ground speed below the limit), THEN
            #    attempt the blind drop. Bounded by GPS_SETTLE_SEC + GPS_DROP_SETTLE_MAX_S
            #    and the overall GOTO_TIMEOUT_S above, so this cannot spin forever.
            control.stop(self._mav)
            if self._drop_arrived_at is None:
                self._drop_arrived_at = now
                log.info("GPS-DROP: reached the saved position (dist=%.1fm) -> waiting for the "
                         "vehicle to settle, then releasing blind", dist)
            waited = now - self._drop_arrived_at
            # Settle condition: STOPPED (ground speed) plus the wind-tolerant tilt gate
            # (fire_tilt_ok rejects only frozen or impossible attitudes, never a legitimate
            # wind lean). A strict tilt gate could otherwise delay the drop indefinitely in
            # wind. settle_over remains the upper bound.
            settled = (waited >= GPS_SETTLE_SEC
                       and self._pose.ground_speed <= GPS_DROP_MAX_SPEED
                       and self._pose.fire_tilt_ok)
            settle_over = waited >= GPS_SETTLE_SEC + GPS_DROP_SETTLE_MAX_S
            if not settled and not settle_over:
                return                                   # still decelerating -> keep holding
            # Settled (or the ceiling was hit) -> ONE blind drop attempt. The gates still
            # apply; if the vehicle is moving the release is blocked.
            self._on_target_blind()
            self._wait_start = time.time()
            self._goto_sent_at = None
            self._goto_start_at = None
            self._drop_arrived_at = None
            log.info("Holding over the target for %ds (%s)",
                     int(TARGET_WAIT_SEC), self._target_label)
            self._state = State.WAIT_HOLD

        elif s == State.AT_TARGET:
            # SETTLE, THEN RELEASE. IBVS has finished centring; first bring the vehicle to a
            # stop over the target (zero velocity in GUIDED) and VERIFY it has stopped, and
            # only then trigger the payload.
            #
            # The release is NOT one-shot: the gates are retried at ~2 Hz throughout the
            # window, so a single-frame confidence/EKF/link flap cannot permanently lose the
            # target.
            control.stop(self._mav)                        # zero velocity -> hold over target
            if mode != "GUIDED":                           # we are waiting -> respect the operator
                log.warning("AT_TARGET: not in GUIDED (operator/FC intervention, mode=%s) -> "
                            "HOVERING", mode)
                self._fire_settle_start = None
                self._state = State.HOVERING
                return
            now = time.time()
            if self._fire_settle_start is None:
                self._fire_settle_start = now
                self._fire_try_at = 0.0
                log.info("AT_TARGET -- centred on %s -> SETTLING (min %.1fs), then payload",
                         self._target_label, FIRE_SETTLE_MIN_S)
            held = now - self._fire_settle_start
            # "Settled" = the IBVS command speed is ~0 AND the real ground speed is ~0
            # (both, so that if one source freezes the other still catches it).
            steady = (self._ibvs.speed <= IBVS_LATCH_SPEED and
                      self._pose.ground_speed <= GPS_DROP_MAX_SPEED)
            fired = False
            if held >= FIRE_SETTLE_MIN_S and steady and (now - self._fire_try_at) >= 0.5:
                self._fire_try_at = now
                fired = self._on_target()                  # all gates + payload (True = fired)
            if fired or held >= FIRE_SETTLE_MAX_S:
                if not fired:
                    log.warning("AT_TARGET: the release gates never passed within the %.0fs "
                                "settling window (steady=%s) -> NO payload, moving to the hold",
                                FIRE_SETTLE_MAX_S, steady)
                self._fire_settle_start = None
                self._wait_start = time.time()
                log.info("Holding over the target for %ds (%s)",
                         int(TARGET_WAIT_SEC), self._target_label)
                self._state = State.WAIT_HOLD

        elif s == State.WAIT_HOLD:
            control.stop(self._mav)            # zero velocity in GUIDED = hold position
            if mode != "GUIDED":
                log.info("HOLD: operator intervention (mode=%s) -> HOVERING", mode)
                self._state = State.HOVERING
                return
            if self._wait_start is not None and time.time() - self._wait_start >= TARGET_WAIT_SEC:
                log.info("Hold complete (%s) -> next target", self._target_label)
                self._start_next_target()

        elif s == State.HOVERING:
            control.stop(self._mav)
            # Leaving GUIDED (the operator switched modes, or went straight to AUTO for a
            # new survey) restarts the machine. In AUTO, WAITING_AUTO immediately advances
            # to MONITORING and the queue is cleared.
            if mode != "GUIDED":
                log.info("Leaving HOVERING (mode=%s) -> WAITING_AUTO", mode)
                self._stop_payload()            # de-energise any running payload child
                self._payload_started = False   # allow it to fire again on the next mission
                self._state = State.WAITING_AUTO

    # --- Helpers -------------------------------------------------------------

    def _update_record_gate(self) -> bool:
        """Is target recording open? It opens when the AUTO mission reaches the mapping
        start item (record_start_wp) and then stays open for that mission. Either signal:
          * MISSION_ITEM_REACHED.seq >= start  -> the start item (or a later one) was REACHED
          * MISSION_CURRENT.seq      >  start  -> the start item was PASSED (a missed REACHED,
                                                  or a mission resumed further on)
        MISSION_CURRENT == start is deliberately NOT used: it is reported while still flying
        TOWARD that item, which would record the transit. Messages received before entering
        MONITORING (left over from a previous sortie) are ignored.
        """
        if self._record_open:
            return True
        msgs = self._mav.messages
        now = time.time()
        mir = msgs.get("MISSION_ITEM_REACHED")
        mc = msgs.get("MISSION_CURRENT")
        reached = (mir is not None and getattr(mir, "_timestamp", now) >= self._monitor_since
                   and mir.seq >= self._record_start_wp)
        passed = (mc is not None and getattr(mc, "_timestamp", now) >= self._monitor_since
                  and mc.seq > self._record_start_wp)
        if reached or passed:
            self._record_open = True
            log.info("Mapping start item reached (seq>=%d) -> target RECORDING STARTED",
                     self._record_start_wp)
        return self._record_open

    def _collect_target_sample(self, det: Detection):
        """Project a detection to GPS and add it to a positional CANDIDATE cluster.

        A survey can see several objects and false positives, so each candidate counts the
        total number of frames it was seen in (n_seen); the most-seen candidate wins at the
        end of the mission.

        The ATTITUDE GATE (ACQUIRE_TILT_GATE) is only for a GIMBAL-LESS setup: on a tilted
        frame the nadir projection shifts, so only level frames are usable. With a gimbal
        the camera stays at nadir, so the gate should be off (see config).
        """
        if not self._pose.have_fix:
            return
        if not SIM_MODE and not self._pose.gimbal_nadir_ok:
            return                                    # camera not looking down -> useless projection
        p = self._pose.pose
        if p.alt <= GOTO_MIN_ALT_M:
            return
        if (ACQUIRE_TILT_GATE
                and (abs(p.pitch_deg) > ACQUIRE_TILT_MAX_DEG
                     or abs(p.roll_deg) > ACQUIRE_TILT_MAX_DEG)):
            return                                    # too tilted -> projection is distorted
        # SPEED GATE: frame and pose are ~0.45 s apart, so the projection error grows with
        # ground speed. Fast transit samples would drag the median -- reject them. This only
        # discards samples; it can never cause an extra actuation.
        if self._pose.ground_speed > ACQUIRE_MAX_SPEED:
            return
        lat, lon = pixel_to_gps(det.cx, det.cy, p.lat, p.lon, p.alt, p.yaw_deg)
        cand = self._nearest_candidate(lat, lon, det.label)
        if cand is None:                              # new (class, position) -> new candidate
            cand = {"samples": [], "n_seen": 0, "clat": lat, "clon": lon, "label": det.label}
            self._candidates.append(cand)
        cand["samples"].append((lat, lon, float(det.confidence)))
        if len(cand["samples"]) > ACQUIRE_MAX_SAMPLES:
            cand["samples"].pop(0)                    # sliding window for the centroid
        cand["n_seen"] += 1                           # TOTAL sightings (the selection metric)
        cand["clat"] = sum(s[0] for s in cand["samples"]) / len(cand["samples"])
        cand["clon"] = sum(s[1] for s in cand["samples"]) / len(cand["samples"])

    def _nearest_candidate(self, lat: float, lon: float, label: str):
        """Nearest SAME-CLASS candidate within TARGET_CLUSTER_RADIUS_M, else None.

        Two different classes in the same place remain separate targets.
        """
        best, best_d = None, TARGET_CLUSTER_RADIUS_M
        for c in self._candidates:
            if c["label"] != label:
                continue
            d = gps_distance_m(lat, lon, c["clat"], c["clon"])
            if d <= best_d:
                best, best_d = c, d
        return best

    def _best_candidate(self):
        """The MOST-SEEN candidate that has enough samples (None if there is none).

        "Seen longest on camera" is the most reliable signal; false positives are brief.
        """
        ready = [c for c in self._candidates if self._cand_ready(c)]
        return max(ready, key=lambda c: c["n_seen"]) if ready else None

    @staticmethod
    def _robust_centroid(samples):
        """Robust centre from (lat, lon, conf) samples: median plus 2-sigma outlier rejection.

        Returns: (lat, lon, clean_count, total_count).
        """
        n = len(samples)
        lats = sorted(v[0] for v in samples)
        lons = sorted(v[1] for v in samples)
        mlat = lats[n // 2] if n % 2 else (lats[n // 2 - 1] + lats[n // 2]) / 2
        mlon = lons[n // 2] if n % 2 else (lons[n // 2 - 1] + lons[n // 2]) / 2
        slat = math.sqrt(sum((v - mlat) ** 2 for v in lats) / n)
        slon = math.sqrt(sum((v - mlon) ** 2 for v in lons) / n)
        clean = [v for v in samples
                 if abs(v[0] - mlat) <= 2.0 * slat + 1e-9
                 and abs(v[1] - mlon) <= 2.0 * slon + 1e-9]
        if len(clean) < 3:
            clean = samples                           # too few left -> use them all
        tlat = sum(v[0] for v in clean) / len(clean)
        tlon = sum(v[1] for v in clean) / len(clean)
        return tlat, tlon, len(clean), n

    @staticmethod
    def _cand_conf(c):
        """Mean detection confidence of a candidate (used to reject false positives)."""
        s = c["samples"]
        return sum(v[2] for v in s) / len(s) if s else 0.0

    def _cand_ready(self, c):
        """Can this candidate be treated as a target? Enough samples AND mean confidence
        above CANDIDATE_MIN_CONF.

        DETECT_CONF is deliberately low for small-target recall; SELECTION demands more.
        """
        return (len(c["samples"]) >= ACQUIRE_MIN_SAMPLES
                and self._cand_conf(c) >= CANDIDATE_MIN_CONF)

    def _candidate_table(self, top: int = 6) -> str:
        """One-line summary of every candidate, most-seen first, e.g.
             [2 ready/5] mannequin 418f/0.55* | tent 335f/0.61* | mannequin 12f/0.31-
        'Nf' = frames seen (n_seen), '/x.xx' = mean confidence, '*' = ready (_cand_ready).
        """
        if not self._candidates:
            return "no candidates"
        ranked = sorted(self._candidates, key=lambda c: -c["n_seen"])
        nready = sum(1 for c in ranked if self._cand_ready(c))
        parts = ["%s %df/%.2f%s" % (c["label"], c["n_seen"], self._cand_conf(c),
                                    "*" if self._cand_ready(c) else "-") for c in ranked[:top]]
        extra = "" if len(ranked) <= top else "  (+%d more)" % (len(ranked) - top)
        return "[%d ready/%d] %s%s" % (nready, len(ranked), " | ".join(parts), extra)

    def _announce_new_ready(self):
        """Log ONE line the first time each candidate becomes ready (visibility only).

        Without it only the leading candidate was logged, so the second target could stay
        invisible on the terminal for the whole survey. Changes no decision or gate.
        """
        for c in self._candidates:
            if c.get("ready_logged") or not self._cand_ready(c):
                continue
            c["ready_logged"] = True
            lat, lon, nclean, ntot = self._robust_centroid(c["samples"])
            log.info("NEW TARGET READY: %s | %d frames, mean conf %.2f (%d/%d clean) -> "
                     "%.6f, %.6f | %s", c["label"], c["n_seen"], self._cand_conf(c),
                     nclean, ntot, lat, lon, self._candidate_table())

    def _set_target_from(self, cand, why: str):
        """Write a candidate's robust centre to the target and freeze the safe altitude."""
        tlat, tlon, nclean, ntot = self._robust_centroid(cand["samples"])
        self._target_lat = tlat
        self._target_lon = tlon
        p = self._pose.pose
        if self._pose.have_fix and p.alt > GOTO_MIN_ALT_M:
            self._target_alt = p.alt
        log.info("%s [%s]: %d frames (%d/%d clean) -> lat=%.6f lon=%.6f | tracking %d candidates",
                 why, cand["label"], cand["n_seen"], nclean, ntot, tlat, tlon,
                 len(self._candidates))

    def _ready_targets(self):
        """The most-seen ready candidate PER CLASS (multi-target).

        One target for each class; if only one class was seen, a single entry.
        """
        ready = [c for c in self._candidates if self._cand_ready(c)]
        by_label = {}
        for c in ready:
            lb = c["label"]
            if lb not in by_label or c["n_seen"] > by_label[lb]["n_seen"]:
                by_label[lb] = c
        winners = list(by_label.values())
        # CLASS-CONFUSION filter.
        # A model can confuse the classes on a small or oblique target, producing a winner
        # of each class for the SAME physical object -> a second visit to the same place and
        # the WRONG payload (wrong servo/motor). If two winners of different classes are
        # closer than HALF the intra-class radius (stricter, so genuinely adjacent targets
        # survive), drop the less-seen one. This only ever REDUCES the number of visits and
        # releases -- it can never create an extra or incorrect actuation.
        if len(winners) > 1:
            cross_r = TARGET_CLUSTER_RADIUS_M / 2.0
            drop = set()
            cents = [self._robust_centroid(w["samples"])[:2] for w in winners]
            for i in range(len(winners)):
                for j in range(i + 1, len(winners)):
                    if winners[i]["label"] == winners[j]["label"]:
                        continue
                    d = gps_distance_m(cents[i][0], cents[i][1], cents[j][0], cents[j][1])
                    if d <= cross_r:
                        lose = j if winners[i]["n_seen"] >= winners[j]["n_seen"] else i
                        drop.add(lose)
                        log.warning("CLASS CONFUSION: [%s](%d frames) & [%s](%d frames) ~%.1fm "
                                    "apart (<%.1fm) -> dropping the less-seen one (prevents a "
                                    "phantom twin and the wrong payload)",
                                    winners[i]["label"], winners[i]["n_seen"],
                                    winners[j]["label"], winners[j]["n_seen"], d, cross_r)
            winners = [w for k, w in enumerate(winners) if k not in drop]
        return winners

    def _commit_goto(self):
        """Build the VISIT QUEUE (best candidate per class), NEAREST FIRST, and start.

        Runs on the tracker thread, triggered by the trigger_goto flag. With no targets we
        hover. If only one was found, only it is visited.
        """
        targets = self._ready_targets()
        if not targets:
            log.warning("goto requested -> no valid target candidates -> HOVERING")
            self._state = State.HOVERING
            return
        p = self._pose.pose
        q = []
        for c in targets:
            tlat, tlon, _, _ = self._robust_centroid(c["samples"])
            dist = gps_distance_m(p.lat, p.lon, tlat, tlon) if self._pose.have_fix else 0.0
            # "cand"      : candidate reference -> refreshed if we see it en route
            # "rung"      : index into IBVS_CENTER_TOL_LADDER (0 = strictest)
            # "attempts"  : IBVS attempts on the CURRENT rung (reset when the rung advances)
            # "goto_fails": GPS_GOTO arrival failures -- a SEPARATE counter, so an arrival
            #               timeout can never use up the ladder and block the blind drop
            q.append({"lat": tlat, "lon": tlon, "label": c["label"],
                      "n_seen": c["n_seen"], "dist": dist, "cand": c,
                      "rung": 0, "attempts": 0, "goto_fails": 0,
                      "center_tol": IBVS_CENTER_TOL_LADDER[0][0]})
        q.sort(key=lambda t: t["dist"])              # NEAREST FIRST
        self._visit_queue = q
        log.info("Visit queue (%d targets, nearest first): %s", len(q),
                 ", ".join(f"{t['label']}@{t['dist']:.0f}m({t['n_seen']} frames)" for t in q))
        self._start_next_target()

    def _start_next_target(self):
        """Pop the next target and fly to it. An empty queue means we are done -> HOVERING."""
        if not self._visit_queue:
            log.info("All targets visited (final action=%s)", AFTER_VISIT_ACTION)
            self._current_target = None
            if AFTER_VISIT_ACTION in ("RTL", "LAND"):
                control.set_mode(self._mav, AFTER_VISIT_ACTION)   # HOVERING remains the backstop
            self._state = State.HOVERING
            return
        t = self._visit_queue.pop(0)
        self._current_target = t                     # kept so a recoverable abort can defer it
        # It may have been re-seen and refreshed while queued -> use the latest centre.
        if t.get("cand") is not None and t["cand"]["samples"]:
            t["lat"], t["lon"], _, _ = self._robust_centroid(t["cand"]["samples"])
        self._target_lat, self._target_lon, self._target_label = t["lat"], t["lon"], t["label"]
        p = self._pose.pose
        if self._pose.have_fix and p.alt > GOTO_MIN_ALT_M:
            self._target_alt = p.alt                 # refresh the safe survey altitude
        rung = t.get("rung", 0)
        rung_tol, rung_budget = IBVS_CENTER_TOL_LADDER[min(rung, len(IBVS_CENTER_TOL_LADDER) - 1)]
        log.info("FLYING TO TARGET -> %s (distance %.0fm, %d frames | rung %d/%d: %.1fm, "
                 "attempt %d/%d) lat=%.6f lon=%.6f alt=%.1fm | %d remaining",
                 t["label"], t["dist"], t["n_seen"], rung + 1, len(IBVS_CENTER_TOL_LADDER),
                 rung_tol, t.get("attempts", 0) + 1, rung_budget,
                 t["lat"], t["lon"], self._target_alt, len(self._visit_queue))
        self._guided_req_at = None
        self._guided_first = None
        self._goto_sent_at = None
        self._goto_start_at = time.time()            # overall arrival timeout
        self._stop_payload()                         # end the previous target's child process
        self._payload_started = False                # allow a release at each target
        self._fire_settle_start = None               # fresh settle-then-release window
        if not SIM_MODE:
            control.set_gimbal_nadir(self._mav)      # re-assert nadir before each target
        self._state = State.GPS_GOTO

    def _refresh_queue_positions(self):
        """Refresh the positions of DEFERRED queued targets from their candidate clusters,
        so returning to one uses the latest centre rather than a stale estimate.

        Only updates positions; never adds or removes queue entries.
        """
        for t in self._visit_queue:
            c = t.get("cand")
            if c is not None and c["samples"]:
                t["lat"], t["lon"], _, _ = self._robust_centroid(c["samples"])

    def _pick_ibvs_det(self, dets):
        """For IBVS: the detection of the CURRENT target class nearest the image centre."""
        if not dets:
            return None
        same = [d for d in dets if d.label == self._target_label] if self._target_label else dets
        if not same:
            return None
        cx0, cy0 = CAMERA_WIDTH // 2, CAMERA_HEIGHT // 2
        return min(same, key=lambda d: (d.cx - cx0) ** 2 + (d.cy - cy0) ** 2)

    def _detect_all(self, frame):
        """Get every detection (uses detect_all when available, else wraps detect)."""
        fn = getattr(self._det, "detect_all", None)
        if fn is not None:
            return fn(frame)
        d = self._det.detect(frame)
        return [d] if d is not None else []

    def _set_tiling(self, ibvs: bool):
        """Toggle tiling per state. A no-op if the detector does not support it."""
        if hasattr(self._det, "tiling"):
            self._det.tiling = TILING_IN_IBVS if ibvs else TILING_IN_SURVEY

    def _draw(self, frame, dets):
        """Draw every box (uses draw_all when available, else the first box)."""
        fn = getattr(self._det, "draw_all", None)
        if fn is not None:
            return fn(frame, dets)
        return self._det.draw(frame, dets[0] if dets else None)

    def _abort_ibvs(self, recoverable: bool = False):
        """Abort IBVS: zero velocity first (3x against UDP loss) so the vehicle STOPS.

        recoverable=True -- "could not centre, but the vehicle is HEALTHY". The target is
          pushed to the BACK of the queue and we move to the next one, returning later.
          When the whole IBVS_CENTER_TOL_LADDER is used up, the blind GPS drop follows (or the
          target is skipped if that is disabled -- infinite-loop guard). A deferred target's
          position is refreshed if we see it en route.
          Callers: detection loss and timeout are ALWAYS recoverable; geofence and
          divergence use recoverable=SIM_MODE (an artefact in sim; a real fault in flight).
        recoverable=False -- the vehicle or telemetry is UNHEALTHY (link/mode/GPS-EKF/gimbal/
          altitude), or this is a real flyaway/divergence. Stop, HOVER and skip the remaining
          targets so the operator can take over. GENUINE DANGER ALWAYS PARKS.
        """
        for _ in range(3):
            control.stop(self._mav)
        if recoverable and self._current_target is not None:
            # Healthy vehicle -> defer and continue; once the attempts run out
            # (allow_gps_drop) fall back to a blind drop at the saved GPS rather than
            # skipping. The blind drop is safe precisely because this branch means the
            # vehicle is healthy -- dangerous aborts never reach it.
            self._defer_or_skip_current("IBVS abort (recoverable)", allow_gps_drop=True)
            return
        # Dangerous abort (or outside the queue mechanism): stop, hover, skip the rest.
        if self._visit_queue:
            log.warning("IBVS abort -> HOVERING | skipping %d unvisited target(s): %s",
                        len(self._visit_queue),
                        ", ".join(t["label"] for t in self._visit_queue))
        self._current_target = None
        self._state = State.HOVERING

    def _defer_or_skip_current(self, why: str, allow_gps_drop: bool = False):
        """Handle a recoverable failure while the vehicle is HEALTHY: push the current target
        to the BACK of the queue and move on, so the other target(s) are handled first and we
        return afterwards. The caller must already have stopped the vehicle (control.stop).

        Two independent paths with independent budgets:

        allow_gps_drop=True  -> IBVS COULD NOT CENTRE (vehicle over the target, healthy).
            Escalating ladder IBVS_CENTER_TOL_LADDER: each rung has its own attempt budget;
            when it is used up the next rung (wider tolerance) starts with a fresh budget.
            After the last rung: blind drop at the saved GPS (GPS_DROP_FALLBACK) instead of
            skipping, so the target can still score.

        allow_gps_drop=False -> GPS_GOTO COULD NOT ARRIVE (arrival timeout). A wider
            tolerance cannot reach an unreachable coordinate and a blind drop would fly to the
            same place, so the separate "goto_fails" counter is used and the target is
            skipped after TARGET_MAX_ATTEMPTS. This never consumes the IBVS ladder.

        With a single target "defer" degenerates to an immediate retry of the same target.
        With no current target we hover, to be safe.
        """
        cur = self._current_target
        if cur is None:
            self._state = State.HOVERING
            return

        def _next_hint():
            return ("moving to the next target." if self._visit_queue
                    else "no other target queued -> retrying the SAME target now.")

        # (A) GPS_GOTO arrival failure -- separate budget, no ladder
        if not allow_gps_drop:
            cur["goto_fails"] = cur.get("goto_fails", 0) + 1
            if cur["goto_fails"] < TARGET_MAX_ATTEMPTS:
                self._visit_queue.append(cur)
                log.warning("%s -> %s deferred (arrival attempt %d/%d) | %s", why, cur["label"],
                            cur["goto_fails"], TARGET_MAX_ATTEMPTS, _next_hint())
                self._current_target = None
                self._start_next_target()
                return
            log.warning("%s -> %s could not be REACHED %d times -> SKIPPING (a blind drop would "
                        "fly to the same coordinate)", why, cur["label"], TARGET_MAX_ATTEMPTS)
            self._current_target = None
            self._start_next_target()
            return

        # (B) IBVS centring failure -- escalating ladder
        rung = min(cur.get("rung", 0), len(IBVS_CENTER_TOL_LADDER) - 1)
        tol, budget = IBVS_CENTER_TOL_LADDER[rung]
        cur["attempts"] = cur.get("attempts", 0) + 1
        if cur["attempts"] < budget:                 # attempts left on this rung
            self._visit_queue.append(cur)
            log.warning("%s -> %s deferred (rung %d/%d: %.1fm, attempt %d/%d) | %s",
                        why, cur["label"], rung + 1, len(IBVS_CENTER_TOL_LADDER), tol,
                        cur["attempts"], budget, _next_hint())
            self._current_target = None
            self._start_next_target()
            return
        if rung + 1 < len(IBVS_CENTER_TOL_LADDER):   # next rung, fresh budget
            new_tol, new_budget = IBVS_CENTER_TOL_LADDER[rung + 1]
            cur["rung"] = rung + 1
            cur["attempts"] = 0
            cur["center_tol"] = new_tol              # drives the IBVS deadband + latch
            self._visit_queue.append(cur)
            log.warning("%s -> %s not centred on rung %d (%.1fm) -> RUNG %d: tolerance "
                        "%.1f->%.1fm, %d attempt(s)", why, cur["label"], rung + 1, tol,
                        rung + 2, tol, new_tol, new_budget)
            self._current_target = None
            self._start_next_target()
            return
        if GPS_DROP_FALLBACK:                        # ladder exhausted -> last resort
            log.warning("%s -> %s not centred on the whole ladder (%s, %d attempts) -> "
                        "LAST RESORT: blind drop at the saved GPS", why, cur["label"],
                        " + ".join("%.0fm x%d" % r for r in IBVS_CENTER_TOL_LADDER),
                        IBVS_CENTER_TOL_ATTEMPTS_TOTAL)
            self._begin_gps_drop()                   # _current_target stays = the dropped target
            return
        log.warning("%s -> %s ladder exhausted and GPS_DROP_FALLBACK=False -> SKIPPING",
                    why, cur["label"])
        self._current_target = None
        self._start_next_target()

    def _begin_gps_drop(self):
        """Enter the last-resort blind GPS drop.

        Writes the target's most recent robust centre to the target fields, refreshes the
        safe altitude and switches to State.GPS_DROP, which flies there, settles and
        releases blind (see _step GPS_DROP and _on_target_blind). _current_target is kept
        (once the drop finishes, WAIT_HOLD -> _start_next_target picks the next one; this
        target is NOT requeued, so it is never retried). Only called by
        _defer_or_skip_current, which has already stopped the vehicle.
        """
        cur = self._current_target
        if cur is not None and cur.get("cand") is not None and cur["cand"]["samples"]:
            self._target_lat, self._target_lon, _, _ = self._robust_centroid(cur["cand"]["samples"])
            self._target_label = cur["label"]
        elif cur is not None:
            self._target_lat, self._target_lon, self._target_label = \
                cur["lat"], cur["lon"], cur["label"]
        else:                                        # defensive: no target -> no blind drop
            self._state = State.HOVERING
            return
        p = self._pose.pose
        if self._pose.have_fix and p.alt > GOTO_MIN_ALT_M:
            self._target_alt = p.alt                 # refresh the safe altitude for the goto
        self._guided_req_at = None
        self._guided_first = None
        self._goto_sent_at = None
        self._goto_start_at = time.time()            # blind-drop arrival timeout
        self._drop_arrived_at = None                 # clear the arrival/settle timer
        self._payload_started = False                # let the blind drop trigger the payload
        if not SIM_MODE:
            control.set_gimbal_nadir(self._mav)      # not required, but keep it consistent
        log.warning("BLIND GPS DROP -> %s | saved GPS lat=%.6f lon=%.6f alt=%.1fm "
                    "(IBVS could not centre)",
                    self._target_label, self._target_lat, self._target_lon, self._target_alt)
        self._state = State.GPS_DROP

    def _fire_allowed(self, blind: bool = False) -> bool:
        """The payload release gate -- EVERY condition must hold or nothing actuates.

        This exists so a heavy aircraft cannot trigger the servo/motor while still moving or
        in an unsafe state.

        blind=False (normal IBVS path): the full gate set, including identity and IBVS centring.
        blind=True  (last-resort GPS drop): IBVS could not centre, so the IDENTITY and
          IBVS-CENTRING gates are SKIPPED (the target was already confirmed during the survey
          via _cand_ready; a live detection is not required). EVERY other vehicle-safety gate
          is retained: link/GUIDED/armed/attitude/altitude, plus proximity to the saved GPS
          (GPS_DROP_FIRE_RADIUS_M) and the REAL ground speed (GPS_DROP_MAX_SPEED).
        """
        p = self._pose.pose
        gate = "Blind-drop gate" if blind else "Release gate"
        if not GPIO_ENABLED:
            log.info("%s: GPIO_ENABLED=False", gate)
            return False
        if not self._pose.link_alive:
            log.warning("%s: link lost", gate)
            return False
        if not self._pose.have_fix:
            log.warning("%s: no/stale GPS fix", gate)
            return False
        # The EKF must be healthy too: GPS_RAW quality can look fine while the EKF position
        # estimate diverges, and we must not release on a bad position estimate.
        if not self._pose.ekf_ok:
            log.warning("%s: EKF unhealthy", gate)
            return False
        # Identity and IBVS-centring gates apply ONLY on the normal path.
        if not blind:
            if self._fire_det is None or self._fire_det.label != self._target_label:
                log.warning("%s: the centred detection is not the target class (%s)",
                            gate, self._target_label)
                return False
            if self._fire_det.confidence < FIRE_MIN_CONF:
                log.warning("%s: detection confidence too low (%.2f < %.2f)", gate,
                            self._fire_det.confidence, FIRE_MIN_CONF)
                return False
            if not self._ibvs.centered:
                log.warning("%s: IBVS is not centred", gate)
                return False
        if self._pose.mode != "GUIDED":
            log.warning("%s: mode is not GUIDED (%s)", gate, self._pose.mode)
            return False
        if not self._pose.armed:
            log.warning("%s: vehicle is not ARMED", gate)
            return False
        # Wind-tolerant tilt gate: a legitimate hold-lean does not block the release; only a
        # frozen ATTITUDE or a physically impossible attitude does. Stillness and centring
        # are enforced by the separate gates below.
        if not self._pose.fire_tilt_ok:
            log.warning("%s: attitude outside sanity limits / ATTITUDE stale", gate)
            return False
        if not (PAYLOAD_ALT_MIN <= p.alt <= PAYLOAD_ALT_MAX):
            log.warning("%s: altitude out of range (%.1fm)", gate, p.alt)
            return False
        # NORMAL PATH -> the release radius is IBVS_TARGET_FENCE_M.
        # Rationale: ibvs.centered is ALREADY a visual position confirmation -- the aircraft
        # is directly over the LIVE target. _target_lat/lon, by contrast, is a survey
        # ESTIMATE produced by pixel_to_gps, and correcting that error is exactly why IBVS
        # exists. Gating the release on a tight "aircraft to saved GPS" distance would
        # penalise the target for the very error IBVS just removed, causing a centred target
        # to be skipped without a release. The legitimate job of a distance gate is to bound
        # a flyaway, and IBVS_TARGET_FENCE_M already enforces that throughout IBVS -- so any
        # target reaching AT_TARGET is guaranteed to be within the fence. Aligning the two
        # leaves a single, consistent error budget. Position safety is unchanged: identity
        # (class + confidence), ibvs.centered, stillness, GUIDED/armed, attitude, altitude
        # and GPS/EKF/link gates all still apply.
        # The blind path has no IBVS, so it uses its own tighter constant.
        fire_radius = GPS_DROP_FIRE_RADIUS_M if blind else IBVS_TARGET_FENCE_M
        d = gps_distance_m(p.lat, p.lon, self._target_lat, self._target_lon)
        if d > fire_radius:
            log.warning("%s: too far from the target (%.1fm > %.1fm)", gate, d, fire_radius)
            return False
        # MOTION gate: the IBVS command speed on the normal path; the REAL ground speed on
        # the blind path (where IBVS is not running).
        if blind:
            if self._pose.ground_speed > GPS_DROP_MAX_SPEED:
                log.warning("%s: still moving (%.2f m/s > %.2f)", gate,
                            self._pose.ground_speed, GPS_DROP_MAX_SPEED)
                return False
        else:
            if self._ibvs.speed > IBVS_LATCH_SPEED:
                log.warning("%s: still moving (%.2f m/s)", gate, self._ibvs.speed)
                return False
        return True

    def _on_target(self) -> bool:
        """Attempt the payload release. True = released, False = a gate blocked it.

        AT_TARGET calls this at ~2 Hz throughout the settling window (it is NOT one-shot),
        so a transient gate failure cannot permanently lose the target. _fire_allowed
        already logs which gate blocked, so we do not log again here (it would spam).
        """
        # SAFETY: all gates BEFORE actuation. If blocked, do NOT set _payload_started.
        if not self._fire_allowed():
            return False
        from suas.payload import indicators
        # Run the buzzer off-thread so its ~1.2s does not block the loop.
        threading.Thread(target=indicators.buzzer, kwargs={"pulses": 3}, daemon=True).start()
        indicators.led(True)

        log.info("AT_TARGET -- settled, TRIGGERING PAYLOAD (%s)", self._target_label)
        self._run_payload()
        return True

    def _on_target_blind(self):
        """Last-resort blind payload release at the target's saved GPS.

        IBVS could not centre, but the vehicle is healthy and at the saved position.
        _fire_allowed(blind=True) applies every safety gate except identity and IBVS
        centring. If a gate blocks, NO payload is released (we still advance to the hold and
        then the next target); safety is preserved.
        """
        log.warning("GPS-DROP AT_TARGET -- attempting a blind release at the saved position (%s)",
                    self._target_label)
        if not self._fire_allowed(blind=True):
            log.warning("BLIND GPS DROP CANCELLED -- a safety gate did not pass")
            return
        from suas.payload import indicators
        threading.Thread(target=indicators.buzzer, kwargs={"pulses": 3}, daemon=True).start()
        indicators.led(True)
        self._run_payload()

    def _run_payload(self):
        """Launch the payload drop sequence as a SEPARATE process.

        The target class selects the servo, motor and release angle; the single source of
        truth for that mapping is PAYLOAD_TARGETS in suas/config.py.

        This does NOT block the tracker loop:
          * subprocess.Popen returns immediately; the state machine keeps running.
          * stdin=DEVNULL makes the script's input() raise EOFError and start automatically.
          * A separate process means its GPIO setmode/cleanup cannot disturb the indicator
            pins owned by this process.

        Fires once per target (reset in WAITING_AUTO and at each new target). An unknown
        target class NEVER triggers a motor.
        """
        if SIM_MODE:
            # Extra defence on top of _fire_allowed: a simulation or bench run must never
            # drive the real payload, even if the flags are left True.
            log.info("SIM_MODE=True -- the real payload/servo/motor will NOT actuate (skipped)")
            return
        if not PAYLOAD_ENABLED:
            log.info("Payload disabled (PAYLOAD_ENABLED=False) -- skipped")
            return
        if not GPIO_ENABLED:
            log.info("GPIO disabled (GPIO_ENABLED=False) -- payload not started")
            return
        if self._payload_started:
            log.warning("Payload already triggered for this target -- not restarting")
            return
        if self._payload_proc is not None and self._payload_proc.poll() is None:
            log.warning("The previous payload child is still running -- stopping it first")
            self._stop_payload()   # never overwrite a live handle -> no orphaned motor

        # Target class -> the --target argument that selects the servo and motor. The set of
        # valid classes comes from PAYLOAD_TARGETS in suas/config.py, so changing the classes
        # does not need a code edit. A class with no entry there (or an empty label) NEVER
        # triggers a motor.
        target = self._target_label
        if target not in PAYLOAD_TARGETS:
            log.warning("Payload: '%s' has no entry in PAYLOAD_TARGETS (suas/config.py) -> no "
                        "actuation (skipped). Configured targets: %s",
                        target, sorted(PAYLOAD_TARGETS) or "(none)")
            return

        log_dir = os.path.join(_REPO_ROOT, "logs")
        try:
            os.makedirs(log_dir, exist_ok=True)
        except Exception:
            pass
        log_path = os.path.join(log_dir, "payload_last_run.log")

        cmd = [sys.executable, "-m", "suas.payload.drop_sequence", "--target", target]
        if PAYLOAD_REVERSE:
            cmd.append("--reverse")

        try:
            with open(log_path, "wb") as logf:   # the child dups the fd; we close our copy
                self._payload_proc = subprocess.Popen(
                    cmd,
                    cwd=_REPO_ROOT,            # so `-m suas.payload...` resolves
                    stdin=subprocess.DEVNULL,  # input() -> EOFError -> start automatically
                    stdout=logf,
                    stderr=subprocess.STDOUT,
                )
            self._payload_started = True
            log.info("Payload sequence started (pid=%d) -> output: %s",
                     self._payload_proc.pid, log_path)
        except Exception as e:
            log.error("Could not start the payload sequence: %s", e)

    def _stop_payload(self):
        """Terminate the payload child safely: terminate(SIGTERM) -> wait -> kill(SIGKILL).

        The child's SIGTERM handler de-energises the motor in well under a second, so no
        orphaned full-power motor is left running.
        """
        proc = self._payload_proc
        if proc is None:
            return
        try:
            if proc.poll() is None:
                proc.terminate()               # SIGTERM -> the child's emergency stop
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    proc.kill()                # SIGKILL (the child's PDEATHSIG is a backup)
                    try:
                        proc.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        pass
        except Exception:
            pass
        self._payload_proc = None

    def _overlay(self, frame):
        p = self._pose.pose
        lines = [
            f"State : {self._state.name}",
            f"Mode  : {self._pose.mode}",
            f"Alt   : {p.alt:.1f}m  Yaw: {p.yaw_deg:.0f}",
            f"Pitch : {p.pitch_deg:.1f}  Roll: {p.roll_deg:.1f}",
            "Record: " + ("ON" if self._record_open
                          else "WAITING (seq>=%d)" % self._record_start_wp),
        ]
        for i, txt in enumerate(lines):
            cv2.putText(frame, txt, (10, 22 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 1)
