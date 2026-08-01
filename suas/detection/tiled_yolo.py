"""
Tiled YOLO detector -- two classes: mannequin (0) and tent (1).

Splits the frame into overlapping TILE_SIZE tiles, runs them (plus, optionally, the full
frame) through YOLO as a single batch, maps the boxes back to full-frame coordinates and
merges them with a class-aware global NMS.

Tiling improves recall for small/distant targets at high resolution. At low resolution it
degrades to a single tile, i.e. a plain full-frame inference.

Interface:
  detect(frame)     -> Optional[Detection]   single highest-confidence target
  detect_all(frame) -> list[Detection]       every target, both classes
  .tiling (bool)    -> the tracker disables it during IBVS (speed) and enables it during
                       the survey (recall).
"""
import os
from typing import List, Optional

import numpy as np

from suas.config import (MODEL_PATH, MODEL_ENGINE_PATH, USE_TENSORRT,
                         TILE_SIZE, IMGSZ, TILE_OVERLAP, DETECT_CONF, NMS_IOU,
                         TILE_ADD_FULL_FRAME, TILING_IN_SURVEY, CLASS_NAMES)
from suas.detection.base import BaseDetector, Detection
from suas.utils.logger import get_logger

log = get_logger("detector")


def _tile_starts(dim: int, size: int, step: int):
    """Tile start coordinates along one axis (the last tile is aligned to the edge).

    If dim <= size this returns [0], so tiling naturally becomes a no-op.
    """
    s = [0]
    while s[-1] + size < dim:
        s.append(min(s[-1] + step, dim - size))
    out = []
    for v in s:
        v = max(0, v)
        if v not in out:
            out.append(v)
    return out


class TiledYoloDetector(BaseDetector):

    # Box colours per class (BGR)
    _COLORS = {"mannequin": (0, 255, 0), "tent": (0, 165, 255)}

    def __init__(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            raise RuntimeError(
                "ultralytics is not installed.\n  pip install ultralytics"
            )
        # Prefer a TensorRT engine when enabled; otherwise fall back to the .pt weights.
        if USE_TENSORRT and os.path.exists(MODEL_ENGINE_PATH):
            path = MODEL_ENGINE_PATH
            log.info("Loading TensorRT engine: %s", path)
        else:
            if USE_TENSORRT:
                log.warning("TensorRT engine not found (%s) -> using .pt (slower on Jetson). "
                            "Build one with: python3 tools/export_tensorrt.py", MODEL_ENGINE_PATH)
            path = MODEL_PATH
            log.info("Loading YOLO model: %s", path)
        if not os.path.exists(path):
            raise RuntimeError(
                f"Model not found: {path}\n"
                "See models/README.md for how to obtain or train the weights."
            )

        self._model = YOLO(path)
        # A fixed-batch TensorRT engine expects exactly this many crops (it is not
        # dynamic). Survey tiling produces 9 crops at 1080p; IBVS produces 1, so _raw()
        # pads up to _engine_batch. This must match the batch used by tools/export_tensorrt.py.
        # The .pt path accepts any batch size and is never padded.
        self._is_engine = (path == MODEL_ENGINE_PATH)
        self._engine_batch = 9
        self._names = self._model.names          # {0: 'mannequin', 1: 'tent'}
        # All class logic downstream (clustering, per-class targets, IBVS same-class
        # selection, the payload identity gate) keys off the label STRING. If a retrained
        # model renames or reorders classes this breaks silently, so warn loudly.
        got = {int(k): str(v) for k, v in dict(self._names).items()}
        if got != CLASS_NAMES:
            log.warning("MODEL CLASSES DIFFER from the expected mapping! expected=%s got=%s "
                        "-- the tracker's class logic may break. Check your weights.",
                        CLASS_NAMES, got)
        else:
            log.info("Model classes verified: %s", got)
        self.tiling = TILING_IN_SURVEY           # the tracker toggles this per state

        try:
            import torch
            self._device = 0 if torch.cuda.is_available() else "cpu"
            self._half = bool(torch.cuda.is_available())
        except Exception:
            self._device, self._half = "cpu", False
        try:
            import torchvision  # noqa: F401  -- REQUIRED for cross-tile NMS
        except ImportError:
            log.warning("torchvision missing -> cross-tile NMS is DISABLED. Overlapping "
                        "tile/full-frame boxes will not be merged (duplicate detections). "
                        "Install torchvision.")
        log.info("TiledYoloDetector ready [tiling=%s device=%s half=%s classes=%s]",
                 self.tiling, self._device, self._half, self._names)

    # --- Inference ----------------------------------------------------------

    def _raw(self, frame: np.ndarray):
        """Tiling + class-aware global NMS -> (boxes xyxy, scores, classes) arrays."""
        h, w = frame.shape[:2]
        crops, offs = [], []
        if self.tiling:
            step = max(1, int(TILE_SIZE * (1 - TILE_OVERLAP)))
            for y in _tile_starts(h, TILE_SIZE, step):
                for x in _tile_starts(w, TILE_SIZE, step):
                    x2, y2 = min(x + TILE_SIZE, w), min(y + TILE_SIZE, h)
                    crops.append(frame[y:y2, x:x2])
                    offs.append((x, y))
        if TILE_ADD_FULL_FRAME or not self.tiling:
            crops.append(frame)                   # also run the full frame (large/near targets)
            offs.append((0, 0))

        # A fixed-batch engine needs exactly _engine_batch crops -> pad by repeating the
        # last crop. The extra results are discarded because zip(res, offs) stops at
        # len(offs). One batched call keeps it fast.
        pred_crops = crops
        if self._is_engine and len(crops) != self._engine_batch:
            if len(crops) < self._engine_batch:
                pred_crops = crops + [crops[-1]] * (self._engine_batch - len(crops))
            else:
                pred_crops = crops[:self._engine_batch]
        imgsz = TILE_SIZE if self.tiling else IMGSZ
        res = self._model.predict(pred_crops, imgsz=imgsz, conf=DETECT_CONF,
                                  device=self._device, half=self._half, verbose=False)
        boxes, scores, classes = [], [], []
        for r, (ox, oy) in zip(res, offs):
            if r.boxes is None or len(r.boxes) == 0:
                continue
            xy = r.boxes.xyxy.cpu().numpy().copy()
            xy[:, [0, 2]] += ox                   # tile -> full-frame coordinates
            xy[:, [1, 3]] += oy
            boxes.append(xy)
            scores.append(r.boxes.conf.cpu().numpy())
            classes.append(r.boxes.cls.cpu().numpy().astype(int))
        if not boxes:
            return np.empty((0, 4)), np.empty((0,)), np.empty((0,), dtype=int)
        return self._nms(np.concatenate(boxes), np.concatenate(scores), np.concatenate(classes))

    @staticmethod
    def _nms(boxes, scores, classes):
        """Class-aware global NMS via torchvision. Returns input unchanged if unavailable."""
        if len(boxes) == 0:
            return boxes, scores, classes
        try:
            import torch
            import torchvision
            keep = torchvision.ops.batched_nms(
                torch.as_tensor(boxes, dtype=torch.float32),
                torch.as_tensor(scores, dtype=torch.float32),
                torch.as_tensor(classes, dtype=torch.long),
                NMS_IOU).cpu().numpy()
            return boxes[keep], scores[keep], classes[keep]
        except Exception:
            return boxes, scores, classes

    def _to_detection(self, box, score, cls) -> Detection:
        x1, y1, x2, y2 = (int(v) for v in box)
        bw, bh = x2 - x1, y2 - y1
        return Detection(
            cx=x1 + bw // 2, cy=y1 + bh // 2, w=bw, h=bh,
            confidence=float(score),
            label=self._names.get(int(cls), str(int(cls))),
        )

    # --- Public -------------------------------------------------------------

    def detect_all(self, frame: np.ndarray) -> List[Detection]:
        """Every detection (both classes) -- for multi-target tracking."""
        b, s, c = self._raw(frame)
        return [self._to_detection(b[i], s[i], c[i]) for i in range(len(b))]

    def detect(self, frame: np.ndarray) -> Optional[Detection]:
        """The single highest-confidence detection, regardless of class."""
        b, s, c = self._raw(frame)
        if len(b) == 0:
            return None
        i = int(np.argmax(s))
        return self._to_detection(b[i], s[i], c[i])

    def _box(self, out, d: Detection):
        import cv2
        x1, y1 = d.cx - d.w // 2, d.cy - d.h // 2
        col = self._COLORS.get(d.label, (0, 255, 0))
        cv2.rectangle(out, (x1, y1), (x1 + d.w, y1 + d.h), col, 2)
        cv2.putText(out, f"{d.label} {d.confidence:.2f}", (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

    def draw(self, frame: np.ndarray, det: Optional[Detection]) -> np.ndarray:
        import cv2
        out = frame.copy()
        if det:
            self._box(out, det)
        fh, fw = out.shape[:2]
        cv2.drawMarker(out, (fw // 2, fh // 2), (200, 200, 200), cv2.MARKER_CROSS, 18, 1)
        return out

    def draw_all(self, frame: np.ndarray, dets) -> np.ndarray:
        """Draw every detection in its class colour, plus the image-centre crosshair."""
        import cv2
        out = frame.copy()
        for d in (dets or []):
            self._box(out, d)
        fh, fw = out.shape[:2]
        cv2.drawMarker(out, (fw // 2, fh // 2), (200, 200, 200), cv2.MARKER_CROSS, 18, 1)
        return out

    def close(self):
        try:
            del self._model
        except Exception:
            pass
