#!/usr/bin/env python3
"""
Live camera + detector preview, with on-the-fly tuning.

Uses the SAME CameraReader and detector as the flight code (no parallel pipeline), so
what you see here is what happens in flight. Tuning changes live only in this process's
MEMORY -- config.json is never modified.

Run (from the repository root):
    python3 tools/preview.py                 # preview window + tuning keys
    python3 tools/preview.py --no-display    # SSH/headless: print detections and FPS
    python3 tools/preview.py --save          # save every frame that has a detection
    python3 tools/preview.py --camera-only   # just count frames, do not load the model

Keys (with the window focused):
    q : quit           t : toggle tiling        + / - : confidence threshold +/- 0.05
    f : cycle imgsz (640 -> 960 -> 1280)   [only meaningful while tiling is OFF]
    s : save the current frame              p : print the current settings
"""
import os
import sys

# Put the GStreamer-enabled JetPack OpenCV first, exactly like run.sh does. This must
# happen BEFORE cv2 is imported, so the camera opens through the real hardware-decode
# pipeline. Under a bare python3 the pip cv2 (no GStreamer) would fall back to FFMPEG.
_GST = "/usr/lib/python3.10/dist-packages"
if ("--_reexec" not in sys.argv) and os.path.isdir(_GST) \
        and (_GST not in os.environ.get("PYTHONPATH", "")):
    os.environ["PYTHONPATH"] = _GST + os.pathsep + os.environ.get("PYTHONPATH", "")
    os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;2000000")
    os.execv(sys.executable, [sys.executable, os.path.abspath(__file__), *sys.argv[1:], "--_reexec"])

# Make `import suas...` work no matter where this is launched from.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import argparse   # noqa: E402
import time       # noqa: E402

import cv2        # noqa: E402

from suas import config as cfg                       # noqa: E402
from suas.detection.camera import CameraReader       # noqa: E402
import suas.detection.tiled_yolo as detector_module   # noqa: E402  (for live overrides)
from suas.detection.tiled_yolo import TiledYoloDetector  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "preview_out")


def _print_settings(det):
    imgsz = cfg.TILE_SIZE if det.tiling else detector_module.IMGSZ
    print("-" * 62)
    print(f"  CAMERA : {cfg.CAMERA_MODE}  {cfg.CAMERA_RTSP_URL}")
    print(f"           {cfg.CAMERA_WIDTH}x{cfg.CAMERA_HEIGHT} @ HFOV {cfg.CAMERA_HFOV_DEG} deg")
    print(f"  MODEL  : {os.path.basename(cfg.MODEL_PATH)}  classes={det._names}")
    print(f"  CONF   : {detector_module.DETECT_CONF:.2f}   IMGSZ (in use): {imgsz}   "
          f"TILING: {'ON' if det.tiling else 'OFF'}   ADD_FULL: {cfg.TILE_ADD_FULL_FRAME}")
    print(f"  NMS_IOU: {cfg.NMS_IOU}   TILE: {cfg.TILE_SIZE}   OVERLAP: {cfg.TILE_OVERLAP}")
    print("-" * 62)


def _camera_only(seconds: int, show: bool):
    """Just verify that frames arrive (no model load)."""
    gst = "YES" if "GStreamer:                   YES" in cv2.getBuildInformation() else "NO"
    print(f"cv2={cv2.__version__} (GStreamer={gst})  mode={cfg.CAMERA_MODE}  "
          f"source={cfg.CAMERA_RTSP_URL}")
    print("Opening the camera...")
    try:
        cam = CameraReader()
    except Exception as e:
        print(f"FAILED to open the camera: {e}")
        print("  -> Check the network link, that the stream is running, and the URL.")
        return 1

    t0, n, last = time.time(), 0, None
    while time.time() - t0 < seconds:
        f = cam.read()
        if f is not None:
            n += 1
            last = f.shape
            if show:
                cv2.imshow("camera (q to quit)", f)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        time.sleep(0.01)
    cam.close()
    if show:
        cv2.destroyAllWindows()
    if n:
        print(f"OK: {n} frames in {seconds}s (~{n / seconds:.1f} FPS), last shape {last}")
        return 0
    print("No frames received.")
    return 1


def main():
    ap = argparse.ArgumentParser(description="Live camera and detector preview")
    ap.add_argument("--no-display", action="store_true", help="do not open a window (SSH/headless)")
    ap.add_argument("--save", action="store_true", help="save frames that contain detections")
    ap.add_argument("--imgsz", type=int, default=None,
                    help="initial full-frame imgsz (e.g. 960); only used while tiling is OFF")
    ap.add_argument("--camera-only", action="store_true",
                    help="only check the camera stream; do not load the model")
    ap.add_argument("--seconds", type=int, default=5, help="duration for --camera-only")
    ap.add_argument("--_reexec", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.camera_only:
        return _camera_only(args.seconds, not args.no_display)

    if args.imgsz:
        detector_module.IMGSZ = args.imgsz   # this process only; config.json is untouched
        print(f">> full-frame imgsz set to {args.imgsz} (config unchanged)")

    show = not args.no_display
    if args.save:
        os.makedirs(OUT_DIR, exist_ok=True)

    print("\n>> Loading the model (the first inference can take a moment)...")
    det = TiledYoloDetector()
    print(">> Opening the camera...")
    try:
        cam = CameraReader()
    except Exception as e:
        print(f"!! Could not open the camera: {e}")
        det.close()
        return 1

    _print_settings(det)
    if show:
        print("  Keys: q=quit  t=tiling  +/-=conf  f=imgsz  s=save  p=settings\n")
        try:
            cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("preview", 1280, 720)
        except Exception:
            print("  (Could not open a window -> switching to headless)")
            show = False

    imgsz_cycle = [640, 960, 1280]
    ema_fps = 0.0
    saved = 0
    last_print = 0.0
    frames = 0

    try:
        while True:
            frame = cam.read()
            if frame is None or cam.stale:
                time.sleep(0.02)
                continue

            t0 = time.time()
            dets = det.detect_all(frame)          # the REAL pipeline
            dt = time.time() - t0
            fps = 1.0 / dt if dt > 1e-6 else 0.0
            ema_fps = fps if ema_fps == 0 else 0.9 * ema_fps + 0.1 * fps
            frames += 1

            now = time.time()
            if now - last_print > 0.5:            # ~2 Hz console output (headless lifeline)
                last_print = now
                imgsz = cfg.TILE_SIZE if det.tiling else detector_module.IMGSZ
                summary = "  ".join(
                    f"{d.label}:{d.confidence:.2f}@({d.cx},{d.cy}) {d.w}x{d.h}" for d in dets
                ) or "-"
                print(f"[{ema_fps:4.1f} FPS | conf>={detector_module.DETECT_CONF:.2f} "
                      f"imgsz={imgsz} tiling={'on' if det.tiling else 'off'}] "
                      f"{len(dets)} detection(s): {summary}")

            if show or args.save:
                vis = det.draw_all(frame, dets)
                imgsz = cfg.TILE_SIZE if det.tiling else detector_module.IMGSZ
                hud = [
                    f"FPS:{ema_fps:.1f}  det:{len(dets)}",
                    f"conf>={detector_module.DETECT_CONF:.2f}  imgsz:{imgsz}  "
                    f"tiling:{'ON' if det.tiling else 'OFF'}",
                ]
                for i, txt in enumerate(hud):
                    cv2.putText(vis, txt, (10, 26 + i * 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)

                if args.save and dets:
                    saved += 1
                    fn = os.path.join(OUT_DIR, f"det_{time.strftime('%H%M%S')}_{saved:04d}.jpg")
                    cv2.imwrite(fn, vis)

                if show:
                    try:
                        cv2.imshow("preview", vis)
                        k = cv2.waitKey(1) & 0xFF
                        if k == ord("q"):
                            break
                        elif k == ord("t"):
                            det.tiling = not det.tiling
                            print(f">> tiling -> {'ON' if det.tiling else 'OFF'}")
                        elif k in (ord("+"), ord("=")):
                            detector_module.DETECT_CONF = round(
                                min(0.95, detector_module.DETECT_CONF + 0.05), 2)
                            print(f">> conf -> {detector_module.DETECT_CONF:.2f}")
                        elif k in (ord("-"), ord("_")):
                            detector_module.DETECT_CONF = round(
                                max(0.05, detector_module.DETECT_CONF - 0.05), 2)
                            print(f">> conf -> {detector_module.DETECT_CONF:.2f}")
                        elif k == ord("f"):
                            cur = detector_module.IMGSZ
                            nxt = (imgsz_cycle[(imgsz_cycle.index(cur) + 1) % len(imgsz_cycle)]
                                   if cur in imgsz_cycle else 640)
                            detector_module.IMGSZ = nxt
                            print(f">> full-frame imgsz -> {nxt} (only used while tiling is OFF)")
                        elif k == ord("s"):
                            saved += 1
                            os.makedirs(OUT_DIR, exist_ok=True)
                            fn = os.path.join(OUT_DIR,
                                              f"snap_{time.strftime('%H%M%S')}_{saved:04d}.jpg")
                            cv2.imwrite(fn, vis)
                            print(f">> saved: {fn}")
                        elif k == ord("p"):
                            _print_settings(det)
                    except Exception:
                        print(">> Window error -> switching to headless")
                        show = False
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n>> Ctrl-C")
    finally:
        print(f"\n>> Done. {frames} frames processed."
              + (f" {saved} saved -> {OUT_DIR}" if saved else ""))
        det.close()
        cam.close()
        if show:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
