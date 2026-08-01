#!/usr/bin/env python3
"""
Camera intrinsic calibration (fx / fy / cx / cy -> HFOV).

WHY: config.CAMERA_HFOV_DEG defaults to the MANUFACTURER SPEC. Both IBVS_FX and
geo.pixel_to_gps derive from it, so a 10% error becomes metres of geolocation and drop
error at 20 m altitude. A checkerboard measures the real focal length, and the horizontal
field of view follows from it.

Uses the same CameraReader as the flight code, so it captures through the real pipeline.

USAGE
1) CAPTURE (hold the checkerboard at different angles, distances and image corners):
     python3 tools/calibrate_camera.py capture --cols 9 --rows 6
       SPACE or c = save the frame (only while the board is detected)
       q          = finish
       --auto     = save automatically about once per second while the board is visible
                    (useful over SSH)
   Take 15-25 photos: near/far, all four corners, slightly tilted. The board must be
   FLAT and RIGID.

2) SOLVE (computes fx/fy/cx/cy + HFOV and writes calibration/result.json):
     python3 tools/calibrate_camera.py solve --cols 9 --rows 6 --square 25

3) Copy the reported CAMERA_HFOV_DEG into config.json (section "camera").

NOTE: --cols/--rows are INNER CORNER counts = (squares - 1) in each direction. A board
with 10x7 squares is --cols 9 --rows 6. --square is the square edge in millimetres; it
does NOT affect fx, only the reported scale. Capture at FULL resolution (fx is in pixels).
"""
import os
import sys

# Prefer the GStreamer-enabled JetPack OpenCV (needed for capture; harmless for solve).
_GST = "/usr/lib/python3.10/dist-packages"
if ("--_reexec" not in sys.argv) and os.path.isdir(_GST) \
        and (_GST not in os.environ.get("PYTHONPATH", "")):
    os.environ["PYTHONPATH"] = _GST + os.pathsep + os.environ.get("PYTHONPATH", "")
    os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;2000000")
    os.execv(sys.executable, [sys.executable, os.path.abspath(__file__), *sys.argv[1:], "--_reexec"])

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import argparse   # noqa: E402
import glob       # noqa: E402
import json       # noqa: E402
import math       # noqa: E402
import time       # noqa: E402

import cv2        # noqa: E402
import numpy as np  # noqa: E402

DEFAULT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "calibration")


def cmd_capture(args):
    from suas.detection.camera import CameraReader   # same pipeline as the flight code

    os.makedirs(args.dir, exist_ok=True)
    pattern = (args.cols, args.rows)
    show = not args.no_display and not args.auto

    print(f">> Opening the camera... board inner corners = {args.cols}x{args.rows}")
    try:
        cam = CameraReader()
    except Exception as e:
        print(f"!! Could not open the camera: {e}")
        return 1
    if show:
        try:
            cv2.namedWindow("calibration", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("calibration", 1280, 720)
        except Exception:
            show = False
    print("   SPACE/c = save   q = finish" + ("   (--auto: automatic)" if args.auto else ""))

    n = len(glob.glob(os.path.join(args.dir, "calib_*.jpg")))
    last_auto = 0.0
    find_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
                  | cv2.CALIB_CB_FAST_CHECK)
    try:
        while True:
            frame = cam.read()
            if frame is None or cam.stale:
                time.sleep(0.02)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern, find_flags)

            save = False
            if args.auto and found and (time.time() - last_auto) > 1.5:
                save = True
                last_auto = time.time()

            if show:
                vis = frame.copy()
                if found:
                    cv2.drawChessboardCorners(vis, pattern, corners, found)
                cv2.putText(vis,
                            f"saved:{n}  board:{'FOUND' if found else 'no'}  (SPACE=save q=quit)",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0) if found else (0, 0, 255), 2)
                try:
                    cv2.imshow("calibration", vis)
                    k = cv2.waitKey(1) & 0xFF
                    if k == ord("q"):
                        break
                    if k in (ord(" "), ord("c")) and found:
                        save = True
                except Exception:
                    show = False
            elif args.auto:
                if found:
                    print(f"  board visible... ({n} saved)", end="\r")
                time.sleep(0.03)

            if save:
                n += 1
                fn = os.path.join(args.dir, f"calib_{n:03d}.jpg")
                cv2.imwrite(fn, frame)   # full resolution, NO overlay
                print(f"\n>> saved [{n}]: {os.path.basename(fn)} "
                      f"({frame.shape[1]}x{frame.shape[0]})")
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        if show:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
    print(f"\n>> {n} photo(s) in {args.dir}\n   Next: python3 tools/calibrate_camera.py solve "
          f"--cols {args.cols} --rows {args.rows} --square {args.square}")
    return 0


def cmd_solve(args):
    pattern = (args.cols, args.rows)
    imgs = sorted(glob.glob(os.path.join(args.dir, "*.jpg"))
                  + glob.glob(os.path.join(args.dir, "*.png")))
    if not imgs:
        print(f"!! No photos in {args.dir}. Run 'capture' first.")
        return 1

    # 3D object points on the Z=0 plane. The square size only sets the scale, not fx.
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    objpoints, imgpoints = [], []
    size = None
    ok_n = 0
    print(f">> Processing {len(imgs)} photo(s) (board {args.cols}x{args.rows})...")
    for p in imgs:
        img = cv2.imread(p)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]  # (w, h)
        found, corners = cv2.findChessboardCorners(gray, pattern, find_flags)
        if not found:
            print(f"   x  no board: {os.path.basename(p)}")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix)
        objpoints.append(objp)
        imgpoints.append(corners)
        ok_n += 1
        print(f"   ok {os.path.basename(p)}")

    if ok_n < 6:
        print(f"!! Not enough usable photos ({ok_n} < 6). Capture more, with the whole board "
              f"sharp and visible at varied angles.")
        return 1

    print(f"\n>> Running calibrateCamera on {ok_n} photo(s)...")
    ret, k_mat, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)
    w, h = size
    fx, fy = float(k_mat[0, 0]), float(k_mat[1, 1])
    cx, cy = float(k_mat[0, 2]), float(k_mat[1, 2])
    hfov = math.degrees(2 * math.atan((w / 2.0) / fx))
    vfov = math.degrees(2 * math.atan((h / 2.0) / fy))
    cx_off = 100.0 * (cx - w / 2.0) / w
    cy_off = 100.0 * (cy - h / 2.0) / h

    print("\n" + "=" * 60)
    print(f"  CALIBRATION RESULT  ({ok_n} photos, {w}x{h})")
    print("=" * 60)
    print(f"  reprojection error : {ret:.3f} px   "
          f"({'good' if ret < 1.0 else 'HIGH -- capture better photos'})")
    print(f"  fx / fy            : {fx:.1f} / {fy:.1f} px   "
          f"(difference {abs(fx - fy) / fx * 100:.1f}%)")
    print(f"  cx / cy            : {cx:.1f} / {cy:.1f}   "
          f"(offset from centre {cx_off:+.1f}% / {cy_off:+.1f}%)")
    print(f"  HFOV (horizontal)  : {hfov:.2f} deg")
    print(f"  VFOV (vertical)    : {vfov:.2f} deg")
    print(f"  distortion (k1,k2,p1,p2,k3): {np.round(dist.ravel(), 4).tolist()}")
    print("-" * 60)
    print('  >> For config.json ("camera"):  "CAMERA_HFOV_DEG": %.2f' % hfov)
    if abs(cx_off) > 3 or abs(cy_off) > 3:
        print("  ! The principal point is more than 3% off centre. The code assumes the")
        print("    image centre (W/2, H/2); if you need more accuracy, extend ibvs.py and")
        print("    geo.py to use cx/cy explicitly.")
    print("=" * 60)

    out = os.path.join(args.dir, "result.json")
    with open(out, "w") as f:
        json.dump({"n_images": ok_n, "image_size": [w, h], "reproj_err": ret,
                   "fx": fx, "fy": fy, "cx": cx, "cy": cy,
                   "hfov_deg": hfov, "vfov_deg": vfov,
                   "cx_off_pct": cx_off, "cy_off_pct": cy_off,
                   "dist": dist.ravel().tolist(),
                   "board": [args.cols, args.rows], "square_mm": args.square}, f, indent=2)
    print(f"  Result written to: {out}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="Camera intrinsic calibration")
    sub = ap.add_subparsers(dest="mode", required=True)
    for name in ("capture", "solve"):
        s = sub.add_parser(name)
        s.add_argument("--cols", type=int, default=9, help="inner corners across = squares - 1")
        s.add_argument("--rows", type=int, default=6, help="inner corners down = squares - 1")
        s.add_argument("--square", type=float, default=25.0,
                       help="square edge in mm (does not affect fx)")
        s.add_argument("--dir", default=DEFAULT_DIR, help="photo directory")
        if name == "capture":
            s.add_argument("--no-display", action="store_true")
            s.add_argument("--auto", action="store_true",
                           help="save automatically while the board is visible")
        s.add_argument("--_reexec", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    return cmd_capture(args) if args.mode == "capture" else cmd_solve(args)


if __name__ == "__main__":
    sys.exit(main())
