#!/usr/bin/env python3
"""
Target-selective payload drop sequence: 2 servos + a dual H-bridge driving 2 motors.

The tracker (suas/tracker.py) launches this as a SEPARATE process once a target has been
centred and every safety gate has passed. The target class selects which servo and motor
are used, per the `payload_hardware` section of config.json:

    --target <class>  ->  that class's servo moves neutral -> release angle,
                          then its motor runs forward at full power
    --init            ->  move EVERY servo to the neutral angle (run.sh does this at
                          startup, before flight)
    --reverse         ->  temporarily invert the motor direction (if the wiring is reversed)

The release angle is per target; the neutral start angle is shared. Every pin, angle and
timing comes from config.json -- do not hard-code them here.

Running it as its own process is deliberate:
  * Its GPIO.setmode/cleanup cannot disturb the indicator pins owned by the main process.
  * If the parent dies, the deadman below de-energises the motor.

SAFETY
    * SIGTERM/SIGINT/crash/parent-death ALWAYS de-energise the motor (both inputs low =
      brake). A systemd stop, `kill <pid>` or a parent crash never leaves the motor at
      100%.
    * PR_SET_PDEATHSIG plus a getppid() watchdog: if the parent (tracker) dies, this
      process stops the motor and exits, so no orphaned full-power motor is left running.
    * atexit plus a hard-cap thread: even if the main thread hangs, the motor is cut after
      the maximum run time.
    * The motor soft-starts (ramp) to full power, avoiding inrush and brownout.

Manual bench use (from the repository root):
    python3 -m suas.payload.drop_sequence --target mannequin
    python3 -m suas.payload.drop_sequence --target tent --reverse
    python3 -m suas.payload.drop_sequence --init

Prefer tools/payload_test.py, which wraps this with a confirmation prompt.
"""
import argparse
import atexit
import ctypes
import os
import signal
import sys
import threading
import time
import warnings

warnings.filterwarnings("ignore")

# Jetson.GPIO only exists on the companion hardware. Import it lazily so that this module
# can still be imported elsewhere for its constants (TARGETS, timings) -- tools/payload_test.py
# and the documentation both read them from here, which keeps this the single source of
# truth. Actually running a sequence without the library raises a clear error in main().
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:                                  # pragma: no cover - hardware only
    GPIO = None
    GPIO_AVAILABLE = False

# --------------------------------------------------------------------- #
# All pins, angles and timings come from config.json (payload_hardware).
# Edit that file, not this one.
# --------------------------------------------------------------------- #
from suas.config import (MOTOR_A_PINS, MOTOR_B_PINS,  # noqa: E402
                         MOTOR_PWM_FREQ, MOTOR_RAMP_S, MOTOR_RUN_S, MOTOR_SPEED,
                         PAYLOAD_TARGETS, SERVO_MAX_US, SERVO_MIN_US, SERVO_MOVE_S,
                         SERVO_NEUTRAL_ANGLE, SERVO_RANGE_DEG, SERVO_SETTLE_S,
                         SERVO_STEP_DT, WAIT_AFTER_SERVO, WAIT_BEFORE_SERVO)

# Target -> servo pin, motor channel, release angle, motor direction, display name.
# The display name is derived so config.json does not have to carry it.
TARGETS = {
    key: {**spec, "name": spec.get("name", key.upper())}
    for key, spec in PAYLOAD_TARGETS.items()
}
# Servos moved to neutral at startup: every servo any target uses.
INIT_SERVO_PINS = sorted({spec["servo_pin"] for spec in TARGETS.values()})

SERVO_NEUTRAL = SERVO_NEUTRAL_ANGLE
SERVO_RANGE = SERVO_RANGE_DEG
PWM_FREQ = MOTOR_PWM_FREQ
A_IN1, A_IN2 = MOTOR_A_PINS
B_IN1, B_IN2 = MOTOR_B_PINS

# Global state, reachable from signal handlers / atexit / the deadman thread.
_servos = []
_motor = None
_shutting_down = False


class _Null:
    """No-op stand-in when a signal arrives before the motor exists."""
    def __getattr__(self, _):
        return lambda *a, **k: None


def shutdown(servos, motor):
    """Safe shutdown: stop ALL threads first, then a SINGLE GPIO.cleanup().

    Each driver's own cleanup() calls the global GPIO.cleanup(), so shutting one down in
    isolation would break the still-running threads of the others (a half-released pin).
    Hence: stop the motor and servo threads manually, then one GPIO.cleanup() at the end.
    """
    if motor is not None:
        try:
            motor.stop()                        # both channels to duty 0 (de-energised)
            motor.a.stop_threads()
            motor.b.stop_threads()
        except Exception:
            pass
    for s in servos:
        try:
            s.detach()
            s._running = False
            if s._thread is not None:
                s._thread.join(timeout=1.0)
        except Exception:
            pass
    try:
        if GPIO is not None:
            GPIO.cleanup()
    except Exception:
        pass
    print(">> Shut down: motor stopped, servos relaxed, pins released.")


def _safe_shutdown():
    """Re-entrant-safe shutdown shared by atexit, signals and the normal exit path."""
    global _shutting_down
    if _shutting_down:
        return
    _shutting_down = True
    shutdown(_servos, _motor or _Null())


def _emergency_stop(reason):
    """De-energise the motor FIRST (most critical), then clean up, then exit."""
    try:
        (_motor or _Null()).stop()      # both inputs low -> motor cut within one PWM period
    except Exception:
        pass
    print(f"\n!! EMERGENCY STOP ({reason}) -> motor cut.")
    _safe_shutdown()
    os._exit(0)


def _sig_handler(signum, frame):
    # Ignore a repeat of the same signal, then stop immediately.
    try:
        signal.signal(signum, signal.SIG_IGN)
    except Exception:
        pass
    _emergency_stop(f"signal {signum}")


def _install_deadman():
    """If the parent (tracker) dies, do not leave the motor running."""
    # (a) Linux: ask the kernel to send us SIGTERM when the parent exits.
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        pr_set_pdeathsig = 1
        libc.prctl(pr_set_pdeathsig, signal.SIGTERM)
    except Exception:
        pass

    # (b) Backup: if we get re-parented to init (pid 1), the parent is gone -> stop.
    def _watch():
        while True:
            if os.getppid() == 1:
                _emergency_stop("parent died (getppid=1)")
            time.sleep(0.5)
    threading.Thread(target=_watch, daemon=True).start()


def _hard_cap():
    """Cut the motor after the maximum run time even if the main thread hangs."""
    time.sleep(MOTOR_RUN_S + 2.0)
    _emergency_stop("maximum run time exceeded")


# ----------------------------- modes -------------------------------------- #

def _run_init():
    """Startup: move both servos to the neutral angle, let them settle, exit.

    No motor involved. When the process exits the pulses stop, leaving the servos at
    roughly the neutral position (unloaded).
    """
    from suas.payload.servo import Servo   # imported here: needs Jetson.GPIO

    print("=== SERVO INIT: %s -> %.0f deg ==="
          % (", ".join(f"pin {p}" for p in INIT_SERVO_PINS), SERVO_NEUTRAL))
    for pin in INIT_SERVO_PINS:
        # auto_detach=False so the servo holds the neutral angle under torque while settling.
        s = Servo(signal_pin=pin, min_us=SERVO_MIN_US, max_us=SERVO_MAX_US,
                  angle_range=SERVO_RANGE, auto_detach=False)
        _servos.append(s)
        s.set_angle(SERVO_NEUTRAL)
    time.sleep(SERVO_SETTLE_S)
    print(">> Both servos at %.0f deg." % SERVO_NEUTRAL)
    # _safe_shutdown runs via finally/atexit


def _run_drop(spec, reverse):
    """Drop: wait, move the selected servo to its release angle, wait, run the motor."""
    global _motor
    from suas.payload.servo import Servo   # imported here: needs Jetson.GPIO
    servo_pin = spec["servo_pin"]
    motor_ch = spec["motor"]
    name = spec["name"]
    release_angle = spec["release_angle"]
    direction = spec.get("direction", 1)
    effective_dir = direction * (-1 if reverse else 1)
    dir_text = "REVERSE" if effective_dir < 0 else "FORWARD"

    print("=== PAYLOAD [%s]: servo pin %d %.0f->%.0f | motor %s %s %d%%, %.0fs (ramp %.1fs) ==="
          % (name, servo_pin, SERVO_NEUTRAL, release_angle, motor_ch, dir_text,
             MOTOR_SPEED, MOTOR_RUN_S, MOTOR_RAMP_S))

    # 1) Move to the neutral position and let it settle.
    # auto_detach=False keeps holding torque at neutral and at the release angle, so the
    # payload weight cannot back-drive the servo.
    servo = Servo(signal_pin=servo_pin, min_us=SERVO_MIN_US, max_us=SERVO_MAX_US,
                  angle_range=SERVO_RANGE, auto_detach=False)
    _servos.append(servo)
    servo.set_angle(SERVO_NEUTRAL)
    time.sleep(SERVO_MOVE_S)
    print(">> Servo at %.0f deg, READY." % SERVO_NEUTRAL)

    # 2) Go-ahead. The tracker passes stdin=DEVNULL, so input() raises EOFError and the
    #    sequence starts automatically; on a terminal you press Enter.
    try:
        input(">> Press ENTER to start (Ctrl+C to cancel) ... ")
    except EOFError:
        print("(no TTY; starting automatically.)")

    # 3) Settling pause before the servo moves.
    print(">> Waiting %.1fs before actuating the servo ..." % WAIT_BEFORE_SERVO)
    time.sleep(WAIT_BEFORE_SERVO)

    # 4) Sweep the servo from neutral to the release angle.
    print(">> SERVO moving (%.1fs): %.0f -> %.0f ..."
          % (SERVO_MOVE_S, SERVO_NEUTRAL, release_angle))
    steps = max(1, int(round(SERVO_MOVE_S / max(1e-3, SERVO_STEP_DT))))
    for i in range(1, steps + 1):
        angle = SERVO_NEUTRAL + (release_angle - SERVO_NEUTRAL) * (i / steps)
        servo.set_angle(angle)
        time.sleep(SERVO_STEP_DT)
    servo.set_angle(release_angle)              # settle exactly on target

    # 5) Pause before the motor runs.
    print(">> Servo at %.0f deg -- waiting %.1fs before the motor ..."
          % (release_angle, WAIT_AFTER_SERVO))
    time.sleep(WAIT_AFTER_SERVO)

    # 6) Motor: soft-start to full power for MOTOR_RUN_S.
    print(">> MOTOR %s soft-start (%s) -> %d%%, %.0fs ..."
          % (motor_ch, dir_text, MOTOR_SPEED, MOTOR_RUN_S))
    from suas.payload.motor import DualMotor   # only set up motor GPIO during a real drop
    _motor = DualMotor(a_in1=A_IN1, a_in2=A_IN2, b_in1=B_IN1, b_in2=B_IN2,
                       pwm_freq=PWM_FREQ)
    ch = _motor.channel(motor_ch)
    threading.Thread(target=_hard_cap, daemon=True).start()   # maximum-run-time safety net
    ch.ramp(MOTOR_SPEED * effective_dir, ramp_s=MOTOR_RAMP_S)

    end = time.perf_counter() + (MOTOR_RUN_S - MOTOR_RAMP_S)
    while time.perf_counter() < end:
        time.sleep(0.1)

    print(">> Time elapsed, stopping the motor.")
    _motor.stop()


def main():
    ap = argparse.ArgumentParser(description="Target-selective payload drop sequence")
    ap.add_argument("--target", choices=sorted(TARGETS.keys()),
                    help="target class (mannequin -> servo pin 7 + motor B, "
                         "tent -> servo pin 12 + motor A)")
    ap.add_argument("--init", action="store_true",
                    help="move both servos to the neutral angle (no motor)")
    ap.add_argument("--reverse", action="store_true",
                    help="temporarily invert the motor direction")
    args = ap.parse_args()

    if not args.init and not args.target:
        ap.error("provide --target <mannequin|tent> or --init.")

    if not GPIO_AVAILABLE:
        ap.error("Jetson.GPIO is not installed -- this sequence drives real hardware and "
                 "can only run on the companion computer.\n"
                 "  pip install Jetson.GPIO && sudo usermod -aG gpio $USER")

    # Install the signal handlers and atexit hook FIRST, so we are safe even before the
    # servo and motor objects exist.
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)
    atexit.register(_safe_shutdown)

    try:
        if args.init:
            _run_init()                          # no motor -> no deadman needed
        else:
            _install_deadman()                   # motor will run -> parent-death safety
            _run_drop(TARGETS[args.target], args.reverse)
    except KeyboardInterrupt:
        print("\nCtrl+C -- safe shutdown.")
    finally:
        _safe_shutdown()


if __name__ == "__main__":
    sys.exit(main())
