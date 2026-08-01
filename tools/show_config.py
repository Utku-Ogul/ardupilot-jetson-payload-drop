#!/usr/bin/env python3
"""
Show the resolved configuration and check it for mistakes.

Prints every setting with its source -- `config.json` or the built-in default -- so you
can confirm your edits actually took effect, then runs a set of consistency checks.

Run this after editing config.json and before every flight:

    python3 tools/show_config.py            # everything
    python3 tools/show_config.py --changed  # only what you overrode
    python3 tools/show_config.py --check    # validation only (exit code 1 on error)
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suas import config as cfg      # noqa: E402
from suas import settings           # noqa: E402

# Display groups. Anything not listed still gets printed, under "other".
GROUPS = [
    ("Profile", ["SIM_MODE"]),
    ("Network", ["COMPANION_IP", "GROUND_STATION_IP", "SIM_HOST_IP"]),
    ("Flight controller", ["MAVLINK_CONNECT", "MAVLINK_CONNECT_REAL", "MAVLINK_CONNECT_SIM"]),
    ("Camera", ["CAMERA_MODE", "CAMERA_RTSP_URL", "CAMERA_RTSP_URL_REAL", "CAMERA_RTSP_URL_SIM",
                "CAMERA_UDP_PORT", "CAMERA_WIDTH", "CAMERA_HEIGHT", "CAMERA_HFOV_DEG",
                "CAMERA_FPS", "CAMERA_STALE_S", "IBVS_FX"]),
    ("Gimbal", ["GIMBAL_MANUAL_NADIR", "GIMBAL_NADIR_TOL_DEG"]),
    ("Detector", ["MODEL_PATH", "MODEL_ENGINE_PATH", "CLASS_NAMES", "USE_TENSORRT",
                  "DETECT_CONF", "IMGSZ", "TILE_SIZE", "TILE_OVERLAP", "TILE_ADD_FULL_FRAME",
                  "TILING_IN_SURVEY", "TILING_IN_IBVS", "NMS_IOU"]),
    ("Target selection", ["DETECT_CONFIRM_SEC", "DETECT_LOST_GRACE", "ACQUIRE_TILT_GATE",
                          "ACQUIRE_TILT_MAX_DEG", "ACQUIRE_MIN_SAMPLES", "ACQUIRE_MAX_SAMPLES",
                          "ACQUIRE_REFINE", "TARGET_CLUSTER_RADIUS_M", "TARGET_WAIT_SEC",
                          "TARGET_MAX_ATTEMPTS", "CANDIDATE_MIN_CONF", "AFTER_VISIT_ACTION"]),
    ("IBVS", ["IBVS_KV", "IBVS_MAX_VEL", "IBVS_SLEW", "IBVS_ERROR_ALPHA", "IBVS_ALT_MIN",
              "IBVS_ALT_MAX", "IBVS_CENTER_TOL_M", "IBVS_CENTER_TOL_MAX_M",
              "IBVS_CENTER_TOL_STEP_M", "IBVS_LATCH_SPEED", "IBVS_CENTER_FRAMES",
              "IBVS_TIMEOUT_S", "IBVS_TARGET_FENCE_M", "IBVS_LOSS_GRACE_S", "IBVS_DIVERGE_S",
              "IBVS_DIVERGE_MARGIN_M", "IBVS_DIVERGE_MARGIN_PX", "IBVS_DIVERGE_PITCH_DEG",
              "IBVS_SIGN_FWD", "IBVS_SIGN_RIGHT"]),
    ("Goto", ["GPS_SETTLE_SEC", "GOTO_RADIUS_M", "GOTO_TIMEOUT_S", "GOTO_MIN_ALT_M"]),
    ("Payload", ["GPIO_ENABLED", "PAYLOAD_ENABLED", "PAYLOAD_REVERSE", "PAYLOAD_ALT_MIN",
                 "PAYLOAD_ALT_MAX", "FIRE_MIN_CONF", "FIRE_TILT_MAX_DEG", "FIRE_SETTLE_MIN_S",
                 "FIRE_SETTLE_MAX_S", "GPS_DROP_FALLBACK", "GPS_DROP_FIRE_RADIUS_M",
                 "GPS_DROP_MAX_SPEED", "GPS_DROP_SETTLE_MAX_S"]),
    ("Payload hardware", ["PAYLOAD_TARGETS", "SERVO_NEUTRAL_ANGLE", "SERVO_RANGE_DEG",
                          "SERVO_MIN_US", "SERVO_MAX_US", "SERVO_MOVE_S", "SERVO_STEP_DT",
                          "SERVO_SETTLE_S", "MOTOR_A_PINS", "MOTOR_B_PINS", "MOTOR_PWM_FREQ",
                          "MOTOR_SPEED", "MOTOR_RUN_S", "MOTOR_RAMP_S", "WAIT_BEFORE_SERVO",
                          "WAIT_AFTER_SERVO", "BUZZER_PIN", "LED_STATUS_PIN", "AUX_PINS"]),
    ("Mapping", ["PHOTO_DIR", "PHOTO_START_WP", "STABILIZE_PITCH_MAX", "STABILIZE_ROLL_MAX",
                 "STABILIZE_TIMEOUT", "UPLOAD_ENABLED", "UPLOAD_HOST", "UPLOAD_USER",
                 "UPLOAD_REMOTE_DIR"]),
    ("Streaming", ["STREAM_ENABLED", "STREAM_HOST", "STREAM_PORT", "STREAM_WIDTH",
                   "STREAM_HEIGHT", "STREAM_FPS", "STREAM_BITRATE", "STREAM_IDR_SEC"]),
    ("Safety", ["POSE_STALE_S", "GPS_EKF_STALE_S", "POSE_MIN_SATS", "LINK_TIMEOUT_S",
                "SHUTDOWN_SAFE_MODE"]),
]


def _fmt(value):
    if isinstance(value, str) and len(value) > 58:
        return value[:55] + "..."
    return repr(value) if isinstance(value, str) else str(value)


def show(changed_only: bool):
    overridden = set(settings.overridden_keys())
    shown = set()
    for title, names in GROUPS:
        rows = []
        for name in names:
            if not hasattr(cfg, name):
                continue
            shown.add(name)
            src = "config.json" if name in overridden else "default"
            if changed_only and src == "default":
                continue
            rows.append((name, _fmt(getattr(cfg, name)), src))
        if not rows:
            continue
        print(f"\n{title}")
        print("-" * 78)
        for name, value, src in rows:
            marker = "*" if src == "config.json" else " "
            print(f" {marker} {name:<26} {value:<32} {src}")

    extra = [k for k in overridden if k not in shown]
    if extra:
        print("\nOther (set in config.json)")
        print("-" * 78)
        for name in sorted(extra):
            if hasattr(cfg, name):
                print(f" * {name:<26} {_fmt(getattr(cfg, name)):<32} config.json")
    if not changed_only:
        print("\n  '*' = overridden in config.json")


def check() -> int:
    """Consistency checks. Returns the number of errors (warnings do not count)."""
    errors, warnings = [], []

    def err(msg):
        errors.append(msg)

    def warn(msg):
        warnings.append(msg)

    # Loader problems (missing file, bad JSON, unknown keys)
    for msg in settings.load_errors:
        err(f"config.json: {msg}")

    # Model
    if not os.path.exists(cfg.MODEL_PATH):
        err(f"model weights not found: {cfg.MODEL_PATH} (see models/README.md)")
    if cfg.USE_TENSORRT and not os.path.exists(cfg.MODEL_ENGINE_PATH):
        warn(f"USE_TENSORRT is on but {cfg.MODEL_ENGINE_PATH} is missing -- the detector "
             f"will fall back to the .pt weights. Build it with tools/export_tensorrt.py")

    # Required values -- these ship blank on purpose, because they are specific to your
    # own hardware. Nothing can fly until they are filled in.
    if not cfg.MAVLINK_CONNECT:
        err(f"{'MAVLINK_CONNECT_SIM' if cfg.SIM_MODE else 'MAVLINK_CONNECT_REAL'} is empty "
            f"-- set it in config.json (section flight_controller). For a serial link use "
            f"the full path from: ls -l /dev/serial/by-id/")
    if cfg.CAMERA_MODE == "rtsp" and not cfg.CAMERA_RTSP_URL:
        err(f"{'CAMERA_RTSP_URL_SIM' if cfg.SIM_MODE else 'CAMERA_RTSP_URL_REAL'} is empty "
            f"-- set it in config.json (section camera), "
            f"e.g. rtsp://<camera-ip>:8554/main.264")
    if cfg.STREAM_ENABLED and not cfg.STREAM_HOST:
        err("STREAM_ENABLED is on but STREAM_HOST is empty -- set STREAM_HOST (or "
            "GROUND_STATION_IP) in config.json.")
    if cfg.UPLOAD_ENABLED and not cfg.UPLOAD_HOST:
        err("UPLOAD_ENABLED is on but UPLOAD_HOST is empty -- set UPLOAD_HOST (or "
            "GROUND_STATION_IP) in config.json.")

    # Flight controller device
    if not cfg.SIM_MODE and cfg.MAVLINK_CONNECT.startswith("/dev/"):
        if not os.path.exists(cfg.MAVLINK_CONNECT):
            err(f"flight controller not found at {cfg.MAVLINK_CONNECT} "
                f"(check: ls -l /dev/serial/by-id/)")
        if "/dev/ttyACM" in cfg.MAVLINK_CONNECT:
            warn("using a bare /dev/ttyACM* path -- this is not stable across reboots or "
                 "brownouts. Prefer a /dev/serial/by-id/ symlink.")

    # Altitude envelopes must overlap, or IBVS centres in a band where release is blocked.
    if cfg.PAYLOAD_ALT_MAX > cfg.IBVS_ALT_MAX:
        warn(f"PAYLOAD_ALT_MAX ({cfg.PAYLOAD_ALT_MAX}) > IBVS_ALT_MAX ({cfg.IBVS_ALT_MAX}): "
             f"above {cfg.IBVS_ALT_MAX} m IBVS aborts before it can centre.")
    if cfg.PAYLOAD_ALT_MIN < cfg.IBVS_ALT_MIN:
        warn(f"PAYLOAD_ALT_MIN ({cfg.PAYLOAD_ALT_MIN}) < IBVS_ALT_MIN ({cfg.IBVS_ALT_MIN}).")
    if cfg.GOTO_MIN_ALT_M >= cfg.PAYLOAD_ALT_MIN:
        warn("GOTO_MIN_ALT_M >= PAYLOAD_ALT_MIN: goto would be rejected at release altitude.")

    # The hold must outlast the payload sequence, or the motor is cut mid-run.
    seq = (cfg.WAIT_BEFORE_SERVO + cfg.SERVO_MOVE_S + cfg.WAIT_AFTER_SERVO
           + cfg.MOTOR_RUN_S + cfg.SERVO_MOVE_S)
    if cfg.TARGET_WAIT_SEC <= seq:
        err(f"TARGET_WAIT_SEC ({cfg.TARGET_WAIT_SEC}s) must EXCEED the payload sequence "
            f"(~{seq:.0f}s), otherwise the hold ends and the motor is cut mid-run.")
    elif cfg.TARGET_WAIT_SEC < seq + 5:
        warn(f"TARGET_WAIT_SEC ({cfg.TARGET_WAIT_SEC}s) leaves under 5s margin over the "
             f"payload sequence (~{seq:.0f}s).")

    # Centring tolerance ladder
    if cfg.IBVS_CENTER_TOL_M > cfg.IBVS_CENTER_TOL_MAX_M:
        err("IBVS_CENTER_TOL_M is greater than IBVS_CENTER_TOL_MAX_M.")
    if cfg.IBVS_CENTER_TOL_MAX_M > cfg.IBVS_TARGET_FENCE_M:
        warn(f"IBVS_CENTER_TOL_MAX_M ({cfg.IBVS_CENTER_TOL_MAX_M}) exceeds the target fence "
             f"({cfg.IBVS_TARGET_FENCE_M}): the widest tolerance can never be satisfied.")

    # Freshness windows
    if cfg.GPS_EKF_STALE_S >= cfg.LINK_TIMEOUT_S:
        warn("GPS_EKF_STALE_S should stay below LINK_TIMEOUT_S so a dead stream is caught "
             "before the link deadman fires.")

    # Detection thresholds
    if cfg.CANDIDATE_MIN_CONF < cfg.DETECT_CONF:
        warn(f"CANDIDATE_MIN_CONF ({cfg.CANDIDATE_MIN_CONF}) is below DETECT_CONF "
             f"({cfg.DETECT_CONF}): target selection is no stricter than raw detection, so "
             f"false positives are more likely to be chosen.")
    if cfg.FIRE_MIN_CONF > cfg.CANDIDATE_MIN_CONF:
        warn(f"FIRE_MIN_CONF ({cfg.FIRE_MIN_CONF}) exceeds CANDIDATE_MIN_CONF "
             f"({cfg.CANDIDATE_MIN_CONF}): a selected target may be unable to pass the "
             f"release gate.")

    # Payload hardware
    pins = {}
    for label, plist in [("motor A", cfg.MOTOR_A_PINS), ("motor B", cfg.MOTOR_B_PINS),
                         ("buzzer", [cfg.BUZZER_PIN]), ("LED", [cfg.LED_STATUS_PIN]),
                         ("aux", list(cfg.AUX_PINS))]:
        for p in plist:
            if p in pins:
                err(f"pin {p} is assigned to both {pins[p]} and {label}.")
            pins[p] = label
    for name, spec in cfg.PAYLOAD_TARGETS.items():
        p = spec.get("servo_pin")
        if p in pins:
            err(f"pin {p} is assigned to both {pins[p]} and servo '{name}'.")
        pins[p] = f"servo '{name}'"
        if spec.get("motor") not in ("A", "B"):
            err(f"target '{name}': motor must be 'A' or 'B', got {spec.get('motor')!r}.")
        angle = spec.get("release_angle")
        if not isinstance(angle, (int, float)) or not 0 <= angle <= cfg.SERVO_RANGE_DEG:
            err(f"target '{name}': release_angle {angle} is outside 0..{cfg.SERVO_RANGE_DEG}.")
    for name in cfg.PAYLOAD_TARGETS:
        if name not in cfg.CLASS_NAMES.values():
            err(f"payload target '{name}' is not one of the model classes "
                f"{sorted(cfg.CLASS_NAMES.values())} -- it would never actuate.")

    if len(cfg.MOTOR_A_PINS) != 2 or len(cfg.MOTOR_B_PINS) != 2:
        err("MOTOR_A_PINS and MOTOR_B_PINS must each be [forward_pin, reverse_pin].")

    # Live posture
    if not cfg.SIM_MODE and cfg.GPIO_ENABLED and cfg.PAYLOAD_ENABLED:
        warn("PAYLOAD IS ARMED (SIM_MODE=False, GPIO_ENABLED, PAYLOAD_ENABLED): the real "
             "servo and motor WILL actuate. Intended only after a bench test.")
    if cfg.SIM_MODE:
        warn("SIM_MODE is True -- set it to False for a real flight.")

    print("\nChecks")
    print("-" * 78)
    for m in errors:
        print(f"  ERROR    {m}")
    for m in warnings:
        print(f"  WARNING  {m}")
    if not errors and not warnings:
        print("  All checks passed.")
    else:
        print(f"\n  {len(errors)} error(s), {len(warnings)} warning(s)")
    return len(errors)


def main():
    ap = argparse.ArgumentParser(description="Show and validate the resolved configuration")
    ap.add_argument("--changed", action="store_true",
                    help="show only settings overridden in config.json")
    ap.add_argument("--check", action="store_true", help="run the checks only")
    args = ap.parse_args()

    print("=" * 78)
    print(f"  Resolved configuration   (source: {settings.CONFIG_PATH})")
    print("=" * 78)

    if not args.check:
        show(args.changed)
    return 1 if check() else 0


if __name__ == "__main__":
    sys.exit(main())
