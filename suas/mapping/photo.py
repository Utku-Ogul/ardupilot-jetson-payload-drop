"""
Waypoint-triggered photo capture for aerial mapping.

Flow:
  MISSION_ITEM_REACHED arrives
  -> wait for pitch/roll below threshold (stabilisation)
  -> grab a frame -> embed GPS EXIF -> save -> optionally copy to the ground station
"""
import datetime
import os
import subprocess
import threading
import time

import cv2

from suas.config import (PHOTO_DIR, PHOTO_START_WP, STABILIZE_TIMEOUT,
                         UPLOAD_ENABLED, UPLOAD_HOST, UPLOAD_USER, UPLOAD_REMOTE_DIR)
from suas.mapping.exif import write_exif
from suas.utils.logger import get_logger
from suas.vehicle.connection import mark_alive, mark_alive_worker

log = get_logger("mapping")


def _upload_async(local_path: str):
    """Copy a photo to the ground station over scp, in the background. No-op when disabled."""
    if not UPLOAD_ENABLED:
        return
    if not UPLOAD_HOST:
        log.warning("Photo upload is enabled but UPLOAD_HOST is empty -- skipping upload.")
        return

    def _worker():
        cmd = ["scp", "-o", "StrictHostKeyChecking=no",
               local_path, f"{UPLOAD_USER}@{UPLOAD_HOST}:{UPLOAD_REMOTE_DIR}/"]
        try:
            result = subprocess.run(cmd, capture_output=True, timeout=30)
            if result.returncode == 0:
                log.info("Uploaded -> %s:%s", UPLOAD_HOST, local_path)
            else:
                log.warning("Upload failed: %s", result.stderr.decode(errors="replace"))
        except Exception as e:
            log.warning("Upload error: %s", e)

    threading.Thread(target=_worker, daemon=True).start()


class PhotoCapture:

    def __init__(self, mav, pose_reader, camera_reader, mission, start_wp: int = PHOTO_START_WP):
        self._mav = mav
        self._pose = pose_reader
        self._cam = camera_reader
        self._mission = mission        # MissionWatcher -- supplies the NAV_WAYPOINT sequence set
        self._count = 0
        self._start_wp = start_wp
        self._captured = set()         # sequences already photographed (duplicate guard)
        os.makedirs(PHOTO_DIR, exist_ok=True)
        if start_wp > 0:
            log.info("PhotoCapture ready -> %s/  [photos from WP%d onward]", PHOTO_DIR, start_wp)
        else:
            log.info("PhotoCapture ready -> %s/  [all waypoints]", PHOTO_DIR)

    # --- Called from the main loop ------------------------------------------

    def listen(self):
        """Non-blocking; call this from the main loop.

        Reads MISSION_ITEM_REACHED via messages.get (NOT recv_match, which would steal
        messages from the pose reader). Only real NAV_WAYPOINTs are photographed --
        DELAY and other command types are skipped.
        """
        mir = self._mav.messages.get("MISSION_ITEM_REACHED")
        if mir is None or mir.seq in self._captured:
            return

        seq = mir.seq
        self._captured.add(seq)

        if not self._mission.is_waypoint(seq):
            log.debug("[MAPPING] seq %d skipped (not a NAV_WAYPOINT)", seq)
            return
        if seq < self._start_wp:
            log.debug("[MAPPING] WP %d skipped (photos start at WP%d)", seq, self._start_wp)
            return

        self._capture(seq)

    def reset(self):
        """Clear per-sortie state before a new AUTO survey (call with mission.reset()).

        On a second sortie the waypoint sequence numbers restart at 0, so a stale
        _captured set would treat them as already photographed and capture nothing.
        _count is NOT reset, so filenames stay unique. Also seed the set with any latched
        MISSION_ITEM_REACHED from the previous sortie, so it cannot trigger a false first
        capture.
        """
        self._captured = set()
        mir = self._mav.messages.get("MISSION_ITEM_REACHED")
        if mir is not None:
            self._captured.add(mir.seq)
        log.info("PhotoCapture reset (new survey) -- counter kept (%d photos), _captured cleared",
                 self._count)

    # --- Internal -----------------------------------------------------------

    def _capture(self, seq: int):
        log.info("[MAPPING] WP %d -> waiting for stabilisation...", seq)
        self._wait_stable()

        frame = self._cam.read()
        if frame is None:
            log.error("[MAPPING] No frame available -- WP %d skipped", seq)
            return

        self._count += 1
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"wp_{self._count:03d}_seq{seq}_{ts}.jpg"
        path = os.path.join(PHOTO_DIR, name)

        cv2.imwrite(path, frame)

        p = self._pose.pose
        # EXIF GPSAltitude is AMSL (GPSAltitudeRef=0 marks it as such), so use alt_amsl --
        # pose.alt is home-relative and writing that would record the wrong datum.
        write_exif(path, p.lat, p.lon, p.alt_amsl, heading=p.yaw_deg)

        log.info("[MAPPING] Saved -> %s  [lat=%.6f lon=%.6f rel=%.1fm AMSL=%.1fm]",
                 name, p.lat, p.lon, p.alt, p.alt_amsl)

        _upload_async(path)

    def _wait_stable(self):
        t0 = time.time()
        while time.time() - t0 < STABILIZE_TIMEOUT:
            # This blocks the MAIN thread for up to STABILIZE_TIMEOUT, so keep feeding the
            # companion deadman -- otherwise the 1 Hz heartbeat stops and the FC triggers a
            # spurious FS_GCS RTL in the middle of the survey. The loop also pulls telemetry,
            # so it is genuinely alive.
            mark_alive()
            mark_alive_worker()
            self._pose.update()
            if self._pose.is_stable:
                log.debug("[MAPPING] Stable (pitch=%.1f deg roll=%.1f deg)",
                          self._pose.pose.pitch_deg, self._pose.pose.roll_deg)
                return
            time.sleep(0.05)
        log.warning("[MAPPING] Stabilisation timed out (%.1fs) -- capturing anyway",
                    STABILIZE_TIMEOUT)
