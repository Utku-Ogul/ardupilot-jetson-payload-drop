"""
Detector interface.

To plug in your own detector:
  1. Subclass BaseDetector.
  2. Implement detect() and draw().
  3. Optionally implement detect_all() / draw_all() for multi-target support --
     the tracker uses them when present and falls back to detect()/draw() otherwise.
  4. Return it from _build_detector() in suas/tracker.py.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Detection:
    cx: int            # centre x [px]
    cy: int            # centre y [px]
    w: int             # width [px]
    h: int             # height [px]
    confidence: float  # 0.0 - 1.0
    label: str         # class name


class BaseDetector(ABC):

    @abstractmethod
    def detect(self, frame: np.ndarray) -> Optional[Detection]:
        """Find the primary target in the frame. Returns None if there is none."""
        ...

    @abstractmethod
    def draw(self, frame: np.ndarray, det: Optional[Detection]) -> np.ndarray:
        """Draw the detection and any overlays onto a copy of the frame."""
        ...

    def close(self):
        """Release resources (model, capture, ...)."""
        pass
