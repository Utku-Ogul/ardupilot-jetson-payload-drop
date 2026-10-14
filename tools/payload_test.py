#!/usr/bin/env python3
"""
Bench test for the payload path, without flying.

Lets you trigger the "target found" moment by hand, with no camera, flight controller or
imagery involved.

THIS RUNS THE REAL CODE. It does not simulate the sequence: it launches
suas/payload/drop_sequence.py as a separate process with the same arguments, working
directory and stdin=DEVNULL that the tracker uses in flight. What you see here is what
happens in flight -- except that the output goes to your terminal instead of a log file.

Target mapping (single source of truth: drop_sequence.TARGETS):
    mannequin -> servo on pin 12 + motor B (pins 29/31),  90 -> 10 deg
    tent      -> servo on pin 7  + motor A (pins 32/33),  90 -> 10 deg
(as shipped -- the actual mapping is PAYLOAD_TARGETS in suas/config.py)

SAFETY
    * Every actuation prints exactly what will happen and requires confirmation.
    * Ctrl+C terminates the child the same way the tracker does: SIGTERM, wait, SIGKILL,
      so no orphaned full-power motor is left running.
    * The sequence's own safety net (deadman / atexit / hard cap) is active regardless.

Usage (from the repository root):
    python3 tools/payload_test.py              # interactive menu
    python3 tools/payload_test.py --init       # move both servos to neutral only
    python3 tools/payload_test.py --target tent --yes    # no prompt (scripted bench runs)
"""
import argparse
import os
import signal
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suas.payload.drop_sequence import (MOTOR_RUN_S, MOTOR_SPEED,  # noqa: E402
                                        SERVO_NEUTRAL, TARGETS)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_child = None


def _run(extra_args, title):
    """Launch drop_sequence exactly the way the tracker does, and stream its output."""
    global _child
    cmd = [sys.executable, "-m", "suas.payload.drop_sequence", *extra_args]
    print(f"\n>> {title}")
    print(f">> command: {' '.join(cmd)}\n" + "-" * 62)
    try:
        _child = subprocess.Popen(cmd, cwd=REPO_ROOT, stdin=subprocess.DEVNULL)
        rc = _child.wait()
    except KeyboardInterrupt:
        _stop_child()
        rc = 130
    finally:
        _child = None
    print("-" * 62 + f"\n>> finished (exit code {rc})\n")
    return rc


def _stop_child():
    """Terminate the child the same way tracker._stop_payload does."""
    proc = _child
    if proc is None or proc.poll() is not None:
        return
    print("\n>> Ctrl+C -> stopping the payload process (SIGTERM)...")
    try:
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            print(">> did not exit -> SIGKILL")
            proc.kill()
            proc.wait(timeout=2.0)
    except Exception:
        pass


def _describe(key):
    spec = TARGETS[key]
    return (f"{spec['name']}: servo pin {spec['servo_pin']} "
            f"{SERVO_NEUTRAL:.0f} -> {spec['release_angle']:.0f} deg, then motor "
            f"{spec['motor']} at {MOTOR_SPEED}% for {MOTOR_RUN_S:.0f}s")


def _confirm(key, assume_yes):
    print("\n" + "=" * 62)
    print("  ABOUT TO ACTUATE THE REAL PAYLOAD")
    print("=" * 62)
    print("  " + _describe(key))
    print("  Make sure the mechanism is clear and nobody is near the payload.")
    print("=" * 62)
    if assume_yes:
        return True
    try:
        return input("  Type 'y' to continue: ").strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        return False


def _menu(assume_yes):
    keys = sorted(TARGETS.keys())
    while True:
        print("\n" + "=" * 62)
        print("  PAYLOAD BENCH TEST")
        print("=" * 62)
        for i, k in enumerate(keys, 1):
            print(f"  {i}) {_describe(k)}")
        print(f"  {len(keys) + 1}) servo init only (both servos -> {SERVO_NEUTRAL:.0f} deg)")
        print("  q) quit")
        try:
            choice = input("\n  Selection: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if choice == "q":
            return 0
        if choice == str(len(keys) + 1):
            _run(["--init"], "SERVO INIT")
            continue
        if choice.isdigit() and 1 <= int(choice) <= len(keys):
            key = keys[int(choice) - 1]
            if _confirm(key, assume_yes):
                _run(["--target", key], f"PAYLOAD DROP [{TARGETS[key]['name']}]")
            else:
                print(">> cancelled.")
            continue
        print("  ! Invalid selection.")


def main():
    ap = argparse.ArgumentParser(description="Payload bench test")
    ap.add_argument("--target", choices=sorted(TARGETS.keys()),
                    help="run this target's sequence directly (no menu)")
    ap.add_argument("--init", action="store_true",
                    help="only move both servos to the neutral angle")
    ap.add_argument("--yes", action="store_true",
                    help="skip the confirmation prompt (scripted bench runs)")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, lambda *_: (_stop_child(), sys.exit(130)))

    if args.init:
        return _run(["--init"], "SERVO INIT")
    if args.target:
        if not _confirm(args.target, args.yes):
            print(">> cancelled.")
            return 1
        return _run(["--target", args.target],
                    f"PAYLOAD DROP [{TARGETS[args.target]['name']}]")
    return _menu(args.yes)


if __name__ == "__main__":
    sys.exit(main())
