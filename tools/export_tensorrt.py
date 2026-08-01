#!/usr/bin/env python3
"""
Convert models/best.pt into a TensorRT FP16 engine (models/best.engine).

RUN THIS ON THE TARGET JETSON. A TensorRT engine is specific to the JetPack, CUDA and GPU
version it was built on -- never copy one between machines, and never commit one to the
repository (.gitignore excludes it).

Only useful when config.USE_TENSORRT is True. With it False (the default) the detector
uses the .pt weights, which is the faster option when tiling is disabled.

FIXED SHAPE (dynamic=False, imgsz=640, batch=9): TensorRT then compiles a single profile,
which takes minutes. A fully dynamic shape takes hours to build, which is why this is
pinned. imgsz must match config.TILE_SIZE, since every crop is letterboxed to it. batch=9
matches the crop count produced by survey tiling at 1080p; the detector pads a smaller
batch up to this size.

Usage (from the repository root):
    python3 tools/export_tensorrt.py
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suas.config import MODEL_ENGINE_PATH, MODEL_PATH, TILE_SIZE  # noqa: E402

ENGINE_BATCH = 9   # must match TiledYoloDetector._engine_batch


def main():
    ap = argparse.ArgumentParser(
        description="Build a TensorRT FP16 engine from the YOLO weights (run on the target).")
    ap.add_argument("--imgsz", type=int, default=TILE_SIZE,
                    help=f"inference size; must match config.TILE_SIZE (default {TILE_SIZE})")
    ap.add_argument("--batch", type=int, default=ENGINE_BATCH,
                    help=f"fixed batch size; must match the detector (default {ENGINE_BATCH})")
    ap.add_argument("--device", default="0", help="CUDA device index (default 0)")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics is not installed: pip install ultralytics")
    if not os.path.exists(MODEL_PATH):
        raise SystemExit(f"Model not found: {MODEL_PATH}")

    print(f"[export] Loading: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print(f"[export] Building a TensorRT FP16 engine "
          f"(imgsz={args.imgsz}, fixed shape, batch={args.batch})... this takes a few minutes")
    model.export(format="engine", imgsz=args.imgsz, half=True,
                 dynamic=False, batch=args.batch, device=args.device)

    if os.path.exists(MODEL_ENGINE_PATH):
        print(f"[export] Done -> {MODEL_ENGINE_PATH}")
        print('[export] Set "USE_TENSORRT": true in config.json to use it.')
        return 0
    print("[export] best.engine was not produced -- check the ultralytics output and your "
          "TensorRT installation.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
