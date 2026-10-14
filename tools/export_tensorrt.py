#!/usr/bin/env python3
"""
Convert the YOLO weights (config MODEL_FILE) into a TensorRT FP16 engine.

RUN THIS ON THE TARGET JETSON. A TensorRT engine is specific to the JetPack, CUDA and GPU
version it was built on -- never copy one between machines, and never commit one to the
repository (.gitignore excludes it).

FIXED SHAPE (dynamic=False): TensorRT then compiles a single profile, which takes minutes.
A fully dynamic shape takes hours to build, which is why it is pinned. Shape and batch
come from the same config the detector uses, so they cannot silently disagree:
  * imgsz = IMGSZ with tiling off (default), TILE_SIZE with tiling on
  * batch = ENGINE_BATCH = crops per frame (1 with tiling off, 9 with 1080p tiling on)
The output name embeds the imgsz (e.g. models/v14_best_960.engine), so changing IMGSZ
makes the detector look for the matching engine instead of loading a wrong-size one.

Usage (from the repository root):
    python3 tools/export_tensorrt.py
    python3 tools/export_tensorrt.py --imgsz 1280      # writes the matching file name
"""
import argparse
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suas.config import (ENGINE_BATCH, IMGSZ, MODEL_DIR, MODEL_ENGINE_PATH,  # noqa: E402
                         MODEL_FILE, MODEL_PATH, TILE_SIZE, TILING_IN_IBVS,
                         TILING_IN_SURVEY)


def main():
    tiling = TILING_IN_SURVEY or TILING_IN_IBVS
    default_imgsz = TILE_SIZE if tiling else IMGSZ
    ap = argparse.ArgumentParser(
        description="Build a TensorRT FP16 engine from the YOLO weights (run on the target).")
    ap.add_argument("--imgsz", type=int, default=default_imgsz,
                    help=f"inference size (default {default_imgsz}: TILE_SIZE with tiling on, "
                         f"else IMGSZ)")
    ap.add_argument("--batch", type=int, default=ENGINE_BATCH,
                    help=f"fixed batch = crops per frame (default ENGINE_BATCH={ENGINE_BATCH})")
    ap.add_argument("--device", default="0", help="CUDA device index (default 0)")
    args = ap.parse_args()

    # The configured name for the configured size; an explicit other --imgsz gets its own name.
    if args.imgsz == default_imgsz:
        target = MODEL_ENGINE_PATH
    else:
        target = os.path.join(MODEL_DIR, "%s_%d.engine"
                              % (os.path.splitext(MODEL_FILE)[0], args.imgsz))

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics is not installed: pip install ultralytics")
    if not os.path.exists(MODEL_PATH):
        raise SystemExit(f"Model not found: {MODEL_PATH} (see models/README.md)")

    if tiling and args.batch == 1:
        print("[export] WARNING: tiling is ON but batch=1 -> tiles would be dropped. Use --batch 9.")
    if not tiling and args.batch != 1:
        print(f"[export] WARNING: tiling is OFF (1 crop per frame) but batch={args.batch} "
              f"-> wasted compute.")

    print(f"[export] Loading: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print(f"[export] classes: {model.names}")
    print(f"[export] Building a TensorRT FP16 engine (imgsz={args.imgsz}, fixed shape, "
          f"batch={args.batch})... this takes a few minutes")
    t0 = time.time()
    out = str(model.export(format="engine", imgsz=args.imgsz, half=True,
                           dynamic=False, batch=args.batch, device=args.device))

    # ultralytics writes <weights>.engine next to the .pt -> move it to the target name.
    if os.path.exists(out) and os.path.abspath(out) != os.path.abspath(target):
        shutil.move(out, target)

    if os.path.exists(target):
        print(f"[export] Done in {(time.time() - t0) / 60:.1f} min -> {target}")
        print('[export] Make sure USE_TENSORRT = True in suas/config.py, then check the FPS '
              'with: python3 tools/preview.py')
        return 0
    print("[export] The engine was not produced -- check the ultralytics output and your "
          "TensorRT installation.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
