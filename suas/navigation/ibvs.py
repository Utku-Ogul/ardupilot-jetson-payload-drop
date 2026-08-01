"""
IBVS -- Image Based Visual Servoing.

Converts a pixel error into a METRIC ground error (using altitude and focal length),
then into a BODY-frame horizontal velocity command (v_fwd = forward/nose, v_right = right).

Design principles -- "make runaway impossible":
  * Altitude-normalised error -> identical behaviour at every altitude (no fixed-pixel gain).
  * P-only + velocity slew damping -> no derivative noise amplification, no input kick.
  * Body-frame output -> yaw is never commanded; the yaw-induced flyaway class is gone.
  * Metric deadband + speed-gated latch -> no false centre lock while still moving.

The hard safety watchdogs (timeout / geofence / divergence / detection loss / mode)
live in tracker.py's IBVS_ACTIVE branch; this module only produces commands.
"""
import math
import time

from suas.config import (IBVS_FX, IBVS_KV, IBVS_MAX_VEL, IBVS_SLEW,
                         IBVS_ERROR_ALPHA, IBVS_ALT_MIN, IBVS_ALT_MAX,
                         IBVS_CENTER_TOL_M, IBVS_LATCH_SPEED, IBVS_CENTER_FRAMES,
                         IBVS_SIGN_FWD, IBVS_SIGN_RIGHT,
                         CAMERA_WIDTH, CAMERA_HEIGHT)
from suas.utils.logger import get_logger

log = get_logger("ibvs")

CX = CAMERA_WIDTH // 2
CY = CAMERA_HEIGHT // 2


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class IBVSController:

    def __init__(self):
        self.reset()

    def reset(self):
        self._ex_f = 0.0
        self._ey_f = 0.0
        self._init = True              # seed the filter with the first sample (no warm-up ramp)
        self._v_fwd_prev = 0.0
        self._v_right_prev = 0.0
        self._t_prev = time.time()
        self._t_log = 0.0              # diagnostic log throttle
        self._center_cnt = 0
        self.g_err = float("inf")      # last metric ground error [m] -- read by watchdog/latch
        self.speed = 0.0               # last commanded speed magnitude [m/s]
        # Centring tolerance (deadband + latch). reset() restores the default; the tracker
        # overwrites it per target with the escalating tolerance ladder. See config.
        self.center_tol = IBVS_CENTER_TOL_M
        log.debug("IBVS reset (center_tol=%.1fm)", self.center_tol)

    def notify_lost(self):
        """Called when the detection is lost: clear the latch counter and slew history so
        flicker cannot produce a false lock, and so velocity ramps from zero on reacquire."""
        self._center_cnt = 0
        self._v_fwd_prev = 0.0
        self._v_right_prev = 0.0

    @property
    def centered(self) -> bool:
        return self._center_cnt >= IBVS_CENTER_FRAMES

    def compute(self, cx: int, cy: int, alt: float) -> tuple[float, float]:
        """
        Input  : detection centre (cx, cy) [px], altitude alt [m] (pose.alt = relative_alt).
        Output : (v_fwd, v_right) BODY-frame horizontal velocity [m/s] (vertical always 0).
        Side   : updates self.g_err [m] and self.speed [m/s] for the tracker's watchdogs.
        """
        now = time.time()
        dt = _clamp(now - self._t_prev, 0.01, 0.10)
        self._t_prev = now

        # 1) Raw pixel error relative to the image centre
        ex = float(cx - CX)     # + = target is RIGHT of centre
        ey = float(cy - CY)     # + = target is BELOW centre

        # 2) EMA low-pass. Seed the filter with the FIRST sample -- ramping up from zero
        #    inflates g_err artificially and used to trip the divergence watchdog.
        a = IBVS_ERROR_ALPHA
        if self._init:
            self._ex_f = ex
            self._ey_f = ey
            self._init = False
        else:
            self._ex_f = a * ex + (1 - a) * self._ex_f
            self._ey_f = a * ey + (1 - a) * self._ey_f

        # 3) Clamp altitude, then ground sampling distance (metres per pixel)
        alt_c = _clamp(alt, IBVS_ALT_MIN, IBVS_ALT_MAX)
        gsd = alt_c / IBVS_FX

        # 4) Metric ground error
        gx = self._ex_f * gsd      # ground offset along image-right [m]
        gy = self._ey_f * gsd      # ground offset along image-down  [m]
        self.g_err = math.hypot(gx, gy)

        # 5) Deadband, then 6) proportional velocity law (body frame), Kv in 1/s.
        # The deadband is the escalating tolerance the tracker set for this target.
        if self.g_err < self.center_tol:
            v_fwd_raw = 0.0
            v_right_raw = 0.0
        else:
            v_fwd_raw = IBVS_SIGN_FWD * IBVS_KV * gy
            v_right_raw = IBVS_SIGN_RIGHT * IBVS_KV * gx

        # 7) Saturation -- direction-preserving magnitude clamp
        v_mag = math.hypot(v_fwd_raw, v_right_raw)
        if v_mag > IBVS_MAX_VEL:
            s = IBVS_MAX_VEL / v_mag
            v_fwd_raw *= s
            v_right_raw *= s

        # 8) Slew (acceleration) limit -- this is where the damping comes from
        max_dv = IBVS_SLEW * dt
        v_fwd = self._v_fwd_prev + _clamp(v_fwd_raw - self._v_fwd_prev, -max_dv, max_dv)
        v_right = self._v_right_prev + _clamp(v_right_raw - self._v_right_prev, -max_dv, max_dv)
        self._v_fwd_prev = v_fwd
        self._v_right_prev = v_right
        self.speed = math.hypot(v_fwd, v_right)

        # Diagnostics (~2 Hz) -- used to verify the axis signs.
        # Expected: the error shrinks. If it grows, that axis' sign is inverted:
        #   ex>0 (target RIGHT) and g_err rising -> flip IBVS_SIGN_RIGHT
        #   ey>0 (target BELOW) and g_err rising -> flip IBVS_SIGN_FWD
        if now - self._t_log > 0.5:
            self._t_log = now
            log.info("IBVS ex=%+.0f ey=%+.0f -> v_fwd=%+.2f v_right=%+.2f | g_err=%.2fm",
                     ex, ey, v_fwd, v_right, self.g_err)

        # Centre counter: inside the metric deadband AND commanding a small speed (both).
        if self.g_err < self.center_tol and self.speed < IBVS_LATCH_SPEED:
            self._center_cnt += 1
            log.debug("Centred: %d/%d (g_err=%.2fm v=%.2f)",
                      self._center_cnt, IBVS_CENTER_FRAMES, self.g_err, self.speed)
        else:
            self._center_cnt = 0

        return v_fwd, v_right
