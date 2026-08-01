"""
Thread-safe camera reader.

CAMERA_MODE = "udp"  -> GStreamer UDP RTP/JPEG (direct simulator stream)
CAMERA_MODE = "rtsp" -> RTSP H.264 (real camera)

Frames are always delivered at CAMERA_WIDTH x CAMERA_HEIGHT, because the detector
and IBVS assume a fixed image centre.
"""
import threading
import time
from typing import Optional

import cv2
import numpy as np

from suas.config import (CAMERA_MODE, CAMERA_UDP_PORT, CAMERA_RTSP_URL,
                         CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_STALE_S)
from suas.utils.logger import get_logger

log = get_logger("camera")


class CameraReader:

    def __init__(self):
        if CAMERA_MODE == "rtsp" and not CAMERA_RTSP_URL:
            raise RuntimeError(
                "The camera stream URL is empty. Set CAMERA_RTSP_URL_REAL (or "
                "CAMERA_RTSP_URL_SIM) in config.json, section camera -- "
                "e.g. rtsp://<camera-ip>:8554/main.264")
        cap = self._open()
        if not cap.isOpened():
            raise RuntimeError(
                f"Could not open camera (mode={CAMERA_MODE}). "
                "Check the connection and your GStreamer installation."
            )
        self._cap = cap
        self._frame: Optional[np.ndarray] = None
        self._frame_ts = 0.0          # monotonic timestamp of the last frame (staleness)
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        log.info("Camera started [mode=%s]", CAMERA_MODE)

    # --- Open ---------------------------------------------------------------

    def _open(self) -> cv2.VideoCapture:
        if CAMERA_MODE == "udp":
            pipeline = (
                f"udpsrc port={CAMERA_UDP_PORT} "
                f"! application/x-rtp,encoding-name=JPEG,payload=26 "
                f"! rtpjpegdepay ! jpegdec ! videoconvert "
                f"! video/x-raw,format=BGR,width={CAMERA_WIDTH},height={CAMERA_HEIGHT} "
                f"! appsink drop=1 sync=false"
            )
            return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        # RTSP. Hardware decode (nvv4l2decoder) only works properly when h264parse is
        # available; without it the decoder reports "Unsupported Codec" and can segfault
        # on shutdown. So if h264parse is missing we go straight to FFMPEG (software
        # decode). Installing gstreamer plugins-bad enables the hardware path automatically.
        if self._gst_has("h264parse"):
            gst = (
                f"rtspsrc location={CAMERA_RTSP_URL} latency=200 ! "
                f"rtph264depay ! h264parse ! nvv4l2decoder ! "
                f"nvvidconv ! video/x-raw,format=BGRx,width={CAMERA_WIDTH},height={CAMERA_HEIGHT} ! "
                f"videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false"
            )
            cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
            if cap.isOpened() and self._probe(cap):
                log.info("RTSP open (GStreamer hardware decode): %s", CAMERA_RTSP_URL)
                return cap
            cap.release()
            log.warning("GStreamer hardware decode failed -- falling back to FFMPEG")
        else:
            log.info("h264parse missing -> using FFMPEG software decode for RTSP: %s",
                     CAMERA_RTSP_URL)
        return cv2.VideoCapture(CAMERA_RTSP_URL, cv2.CAP_FFMPEG)

    @staticmethod
    def _gst_has(element: str) -> bool:
        """Is a GStreamer element (e.g. h264parse) registered on this system?"""
        import subprocess
        try:
            return subprocess.run(["gst-inspect-1.0", element],
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL).returncode == 0
        except Exception:
            return False

    @staticmethod
    def _probe(cap: cv2.VideoCapture, tries: int = 30) -> bool:
        """Are frames actually arriving? (isOpened can be True with no frames.)"""
        for _ in range(tries):
            ok, frame = cap.read()
            if ok and frame is not None:
                return True
            time.sleep(0.1)
        return False

    # --- Read loop ----------------------------------------------------------

    def _read_loop(self):
        fails = 0
        while self._running:
            ok, frame = self._cap.read()
            if ok and frame is not None:
                fails = 0
                # The detector and IBVS assume a fixed CAMERA_WIDTH x CAMERA_HEIGHT centre.
                if frame.shape[1] != CAMERA_WIDTH or frame.shape[0] != CAMERA_HEIGHT:
                    frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
                with self._lock:
                    self._frame = frame
                    self._frame_ts = time.monotonic()
                continue

            # Read failed: the stream may have dropped or frozen (Wi-Fi loss, camera
            # brownout). SAFETY: while this happens the 'stale' gate in tracker.py already
            # halts all actuation, so reconnecting only restores mission capability -- we
            # never command from a blind or stale frame.
            fails += 1
            if fails * 0.02 >= CAMERA_STALE_S:   # consecutive failures spanning the stale window
                log.warning("Camera stream lost/frozen -- reconnecting (mode=%s)", CAMERA_MODE)
                try:
                    self._cap.release()
                except Exception:
                    pass
                if not self._running:            # exit promptly if close() was called
                    break
                time.sleep(0.5)                  # backoff -- avoid a reconnect storm
                try:
                    new_cap = self._open()
                    if new_cap.isOpened():
                        self._cap = new_cap
                        log.info("Camera reconnected")
                    else:
                        new_cap.release()
                except Exception:
                    log.exception("Camera reconnect failed -- will retry")
                fails = 0
            else:
                time.sleep(0.02)                 # no 100% CPU busy-spin on a dead stream

    # --- Public -------------------------------------------------------------

    def read(self) -> Optional[np.ndarray]:
        """Return the most recent frame, or None if none has arrived yet."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    @property
    def stale(self) -> bool:
        """Has the video stream frozen? (last frame older than CAMERA_STALE_S)

        Never command velocity or actuate the payload from a stale frame.
        """
        return (time.monotonic() - self._frame_ts) > CAMERA_STALE_S

    def close(self):
        self._running = False
        self._cap.release()
        log.info("Camera closed")
