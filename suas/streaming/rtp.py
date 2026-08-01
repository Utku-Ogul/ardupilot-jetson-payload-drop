"""
Low-latency live H.264 stream of the annotated (boxed) frames to the ground station.

Pipeline: cv2 BGR frame -> appsrc -> videoconvert(I420) -> nvvidconv(NVMM/NV12)
          -> nvv4l2h264enc (hardware H.264) -> rtph264pay -> udpsink (RTP/UDP)

This is a SEPARATE stream from the camera's raw RTSP feed; it carries the detection
overlay. Watch it with QGroundControl / Mission Planner (UDP H.264, default port 5600),
GStreamer, VLC or ffplay -- see docs/GROUND_STATION.md.

REQUIREMENT: cv2.VideoWriter(..., cv2.CAP_GSTREAMER) only works with a GStreamer-enabled
OpenCV build. The JetPack system OpenCV has GStreamer; the pip `opencv-python` wheel does
not. run.sh puts the JetPack build first on PYTHONPATH, so streaming works in production.
On an interpreter without GStreamer, isOpened() returns False and this module disables
itself silently (logging one warning). Streaming NEVER brings down the tracker loop or
IBVS -- all write() exceptions are swallowed.
"""
import cv2

from suas.utils.logger import get_logger

log = get_logger("stream")


class RtpH264Publisher:
    def __init__(self, host, port, width, height, fps, bitrate, idr_sec):
        self._host = host
        self._port = int(port)
        self._w = int(width)
        self._h = int(height)
        self._fps = int(fps)
        self._bitrate = int(bitrate)
        # Keyframe (IDR) interval in frames. On lossy UDP a SHORT GOP recovers quickly
        # (the picture returns at the next IDR); a long GOP saves bandwidth but a single
        # lost packet corrupts the image until the next IDR.
        self._idr = max(1, int(round(fps * float(idr_sec))))
        self._writer = None
        self._ok = False

    def _pipeline(self):
        return (
            "appsrc is-live=true do-timestamp=true format=TIME block=true "
            # Leaky queue: if the encoder falls behind the main loop, drop the OLD frame
            # so latency does not accumulate (for live viewing, freshness beats completeness).
            "! queue max-size-buffers=2 leaky=downstream "
            # BGR -> I420 on the CPU. Cheap, because we send a reduced resolution.
            "! videoconvert ! video/x-raw,format=I420 "
            # CPU I420 -> NVMM NV12 (hardware VIC): the memory/format the encoder wants.
            "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
            "! nvv4l2h264enc "
            f"  bitrate={self._bitrate} "
            "  control-rate=constant_bitrate "   # CBR -> predictable bandwidth on a narrow link
            "  preset-level=1 "                  # ultrafast -> lowest encode latency
            "  maxperf-enable=true "             # pin the encoder clock (no DVFS ramp delay)
            "  insert-sps-pps=true "             # SPS/PPS on every IDR -> late joiners resync
            f"  iframeinterval={self._idr} idrinterval={self._idr} "
            "  profile=2 "                       # Main (no B-frames -> no reordering latency)
            # h264parse is deliberately NOT used: it is absent on this JetPack install.
            # nvv4l2h264enc emits a byte stream that rtph264pay can payload directly.
            # config-interval=1 repeats SPS/PPS in-stream once per second.
            # mtu=1400 keeps RTP packets below the path MTU, so there is no IP
            # fragmentation on the radio bridge (a lost fragment would ruin the frame).
            "! rtph264pay config-interval=1 pt=96 mtu=1400 "
            f"! udpsink host={self._host} port={self._port} sync=false async=false"
        )

    def open(self):
        pipe = self._pipeline()
        try:
            # fourcc=0 -> manual appsrc pipeline (GStreamer encodes, not OpenCV).
            # isColor=True -> appsrc caps become video/x-raw,format=BGR.
            self._writer = cv2.VideoWriter(
                pipe, cv2.CAP_GSTREAMER, 0, float(self._fps), (self._w, self._h), True
            )
        except Exception:
            log.exception("Streaming: could not create VideoWriter")
            self._writer = None

        self._ok = bool(self._writer and self._writer.isOpened())
        if self._ok:
            log.info("Live stream ON -> udp://%s:%d  %dx%d@%dfps  %d kbps  (IDR=%d frames)",
                     self._host, self._port, self._w, self._h, self._fps,
                     self._bitrate // 1000, self._idr)
        else:
            log.warning("Live stream OFF: OpenCV has no GStreamer support, or the pipeline "
                        "failed to open. Make sure you launched via run.sh. Flight is unaffected.")
        return self._ok

    def write(self, bgr):
        """Push one annotated frame. Errors NEVER propagate to the caller."""
        if not self._ok:
            return
        try:
            if bgr.shape[1] != self._w or bgr.shape[0] != self._h:
                # CPU resize (source 1080p -> stream resolution): lowers both CPU and bandwidth.
                bgr = cv2.resize(bgr, (self._w, self._h), interpolation=cv2.INTER_AREA)
            self._writer.write(bgr)
        except Exception:
            # A single bad frame must not kill the stream, let alone the flight.
            log.debug("Streaming: frame write failed (ignored)")

    def close(self):
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception:
                pass
        self._writer = None
        self._ok = False
