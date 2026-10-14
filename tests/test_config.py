"""
Consistency checks for suas/config.py. No hardware required.

Every setting lives in one file, so these tests are the guard against the mistakes that are
easy to make there and dangerous in flight: two functions on one pin, a hold time shorter
than the payload sequence (the motor is cut mid-run), or an IBVS envelope that does not
match the release envelope (the target is centred but never released).

Run from the repository root after every config edit:
    python3 -m unittest discover -s tests -p test_config.py -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suas import config as cfg   # noqa: E402

# Duration of drop_sequence._run_drop: settle at neutral (SERVO_MOVE_S), pause, sweep
# (SERVO_MOVE_S), pause, motor run.
PAYLOAD_SEQUENCE_S = (cfg.SERVO_MOVE_S + cfg.WAIT_BEFORE_SERVO + cfg.SERVO_MOVE_S
                      + cfg.WAIT_AFTER_SERVO + cfg.MOTOR_RUN_S)


class TimingTest(unittest.TestCase):

    def test_hold_outlasts_payload_sequence(self):
        # WAIT_HOLD ends -> the payload child is terminated. Keep >= 5 s margin for the
        # child's start-up under flight load.
        self.assertGreaterEqual(cfg.TARGET_WAIT_SEC, PAYLOAD_SEQUENCE_S + 5.0,
                                "TARGET_WAIT_SEC must exceed the payload sequence "
                                "(%.1f s) by at least 5 s" % PAYLOAD_SEQUENCE_S)

    def test_gps_ekf_window_below_link_timeout(self):
        self.assertLess(cfg.GPS_EKF_STALE_S, cfg.LINK_TIMEOUT_S)


class EnvelopeTest(unittest.TestCase):

    def test_ibvs_and_release_ceiling_match(self):
        # Otherwise there is a band where IBVS centres but the release gate blocks.
        self.assertEqual(cfg.IBVS_ALT_MAX, cfg.PAYLOAD_ALT_MAX)

    def test_release_floor_inside_ibvs_envelope(self):
        self.assertGreaterEqual(cfg.PAYLOAD_ALT_MIN, cfg.IBVS_ALT_MIN)

    def test_goto_floor_below_release_floor(self):
        self.assertLess(cfg.GOTO_MIN_ALT_M, cfg.PAYLOAD_ALT_MIN)


class LadderTest(unittest.TestCase):

    def test_ladder_shape(self):
        ladder = cfg.IBVS_CENTER_TOL_LADDER
        self.assertGreater(len(ladder), 0)
        tols = [t for t, _ in ladder]
        self.assertEqual(tols, sorted(tols), "ladder must widen left to right")
        for tol, attempts in ladder:
            self.assertGreater(tol, 0)
            self.assertGreaterEqual(int(attempts), 1)

    def test_widest_rung_inside_target_fence(self):
        # A rung wider than the fence would latch outside the flyaway lock.
        self.assertLessEqual(cfg.IBVS_CENTER_TOL_MAX_M, cfg.IBVS_TARGET_FENCE_M)


class ConfidenceTest(unittest.TestCase):

    def test_selection_above_detection_floor(self):
        self.assertGreaterEqual(cfg.CANDIDATE_MIN_CONF, cfg.DETECT_CONF)

    def test_release_not_stricter_than_selection(self):
        # Otherwise a target can be accepted and centred but never released.
        self.assertLessEqual(cfg.FIRE_MIN_CONF, cfg.CANDIDATE_MIN_CONF)


class DetectorTest(unittest.TestCase):

    def test_engine_matches_working_size(self):
        tiling = cfg.TILING_IN_SURVEY or cfg.TILING_IN_IBVS
        size = cfg.TILE_SIZE if tiling else cfg.IMGSZ
        self.assertTrue(cfg.MODEL_ENGINE_PATH.endswith("_%d.engine" % size),
                        "engine file name must carry the working imgsz (%d)" % size)
        self.assertEqual(cfg.ENGINE_BATCH, 9 if tiling else 1)


class PayloadTest(unittest.TestCase):

    def test_targets_are_model_classes(self):
        classes = set(cfg.CLASS_NAMES.values())
        for name in cfg.PAYLOAD_TARGETS:
            self.assertIn(name, classes)

    def test_target_specs(self):
        for name, spec in cfg.PAYLOAD_TARGETS.items():
            self.assertIn(spec["motor"], ("A", "B"), name)
            self.assertTrue(0 <= spec["release_angle"] <= cfg.SERVO_RANGE_DEG, name)
            self.assertIn(spec.get("direction", 1), (1, -1), name)

    def test_each_target_has_its_own_servo_and_motor(self):
        specs = list(cfg.PAYLOAD_TARGETS.values())
        self.assertEqual(len({s["servo_pin"] for s in specs}), len(specs))
        self.assertEqual(len({s["motor"] for s in specs}), len(specs))

    def test_no_pin_collisions(self):
        pins = {}
        groups = [("MOTOR_A_PINS", cfg.MOTOR_A_PINS), ("MOTOR_B_PINS", cfg.MOTOR_B_PINS),
                  ("BUZZER_PIN", [cfg.BUZZER_PIN]), ("LED_STATUS_PIN", [cfg.LED_STATUS_PIN]),
                  ("AUX_PINS", cfg.AUX_PINS)]
        groups += [("servo:" + n, [s["servo_pin"]]) for n, s in cfg.PAYLOAD_TARGETS.items()]
        for owner, plist in groups:
            for p in plist:
                self.assertNotIn(p, pins, "pin %d used by %s and %s" % (p, pins.get(p), owner))
                pins[p] = owner

    def test_motor_pin_pairs(self):
        self.assertEqual(len(cfg.MOTOR_A_PINS), 2)
        self.assertEqual(len(cfg.MOTOR_B_PINS), 2)


if __name__ == "__main__":
    unittest.main()
