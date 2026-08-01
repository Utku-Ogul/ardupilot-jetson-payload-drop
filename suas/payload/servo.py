"""
SG90-class hobby servo driver for the Jetson 40-pin header.

Uses software PWM on a plain GPIO pin by default (hardware PWM is optional), so the
servo can be driven from any free output pin without conflicting with the motor pins.

POWER NOTE: an SG90 needs 5 V. Use a SEPARATE 5 V supply and tie its ground to the
Jetson ground. Do not power the servo from the 3.3 V rail.

Usage:
    from suas.payload.servo import Servo
    with Servo(signal_pin=7) as s:
        s.set_angle(90)
"""
import atexit
import threading
import time
import warnings

import Jetson.GPIO as GPIO

# The "Carrier board is not from a Jetson Developer Kit" warning is cosmetic on this
# board; the pin map is still selected from the correct table.
warnings.filterwarnings("ignore")


class Servo:
    """Drives a single hobby servo from the Jetson 40-pin header.

    signal_pin    : BOARD pin number.
    min_us/max_us : pulse widths corresponding to 0 and angle_range degrees.
    angle_range   : total angular travel (default 180 degrees).
    freq_hz       : pulse frequency (50 Hz for an SG90).
    auto_detach   : stop pulsing settle_s after the last command, which silences the
                    servo's buzz/jitter. Pass False when you need continuous holding
                    torque (e.g. holding a payload against its weight).
    """

    def __init__(self, signal_pin=7, min_us=500, max_us=2500,
                 angle_range=180.0, freq_hz=50, use_hardware_pwm=False,
                 auto_detach=True, settle_s=0.6):
        self.pin = signal_pin
        self.min_us = float(min_us)
        self.max_us = float(max_us)
        self.range = float(angle_range)
        self.freq_hz = float(freq_hz)
        self.period = 1.0 / self.freq_hz          # seconds (50 Hz -> 0.02)
        self.use_hw = use_hardware_pwm

        self._auto_detach = auto_detach
        self._settle_s = float(settle_s)
        self._last_cmd = time.perf_counter()

        self._lock = threading.Lock()
        self._target_us = (self.min_us + self.max_us) / 2.0   # centre
        self._angle = None
        self._detached = False
        self._running = False
        self._thread = None

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

        if self.use_hw:
            self._pwm = GPIO.PWM(self.pin, self.freq_hz)
            self._pwm.start(0.0)
        else:
            self._pwm = None
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

        atexit.register(self.cleanup)
        self.set_angle(90)        # start centred

    def _us_for_angle(self, angle):
        angle = max(0.0, min(self.range, float(angle)))
        frac = angle / self.range if self.range else 0.0
        return self.min_us + (self.max_us - self.min_us) * frac

    def _us_to_duty(self, us):
        """Pulse width (us) -> duty percentage, for hardware PWM."""
        return (us / 1_000_000.0) / self.period * 100.0

    def set_angle(self, angle):
        """Move to the given angle (0..angle_range degrees). Returns the clamped angle."""
        angle = max(0.0, min(self.range, float(angle)))
        us = self._us_for_angle(angle)
        with self._lock:
            self._target_us = us
            self._angle = angle
            self._detached = False           # set_angle always re-attaches
            self._last_cmd = time.perf_counter()   # reset the auto-detach timer
        if self.use_hw and self._pwm is not None:
            self._pwm.ChangeDutyCycle(self._us_to_duty(us))
        return angle

    def detach(self):
        """Stop pulsing -> the servo relaxes and stops buzzing (position roughly holds).
        The next set_angle() re-engages it automatically."""
        with self._lock:
            self._detached = True
        if self.use_hw and self._pwm is not None:
            self._pwm.ChangeDutyCycle(0.0)
        else:
            try:
                GPIO.output(self.pin, GPIO.LOW)
            except Exception:
                pass

    @property
    def angle(self):
        return self._angle

    def _loop(self):
        """Software PWM thread.

        The HIGH pulse is timed with a busy-wait so that OS scheduling jitter cannot
        distort the pulse width (which is the commanded position); the remainder of the
        period is slept, where jitter does not matter. After settle_s the pulse is
        auto-detached, which silences servo buzz.
        """
        while self._running:
            with self._lock:
                us = self._target_us
                quiet = self._detached or (
                    self._auto_detach
                    and (time.perf_counter() - self._last_cmd) > self._settle_s
                )
            if quiet or us <= 0.0:
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(self.period)
                continue
            t_high = us / 1_000_000.0
            # Start the HIGH pulse under the lock, so a concurrent detach() that pulled
            # the pin LOW cannot be followed by this loop starting a fresh full pulse.
            with self._lock:
                if self._detached:
                    GPIO.output(self.pin, GPIO.LOW)
                    continue
                start = time.perf_counter()
                GPIO.output(self.pin, GPIO.HIGH)
            # Busy-wait for the pulse (~1-2.5 ms, 50 times a second -> ~10% of one core).
            while (time.perf_counter() - start) < t_high:
                pass
            GPIO.output(self.pin, GPIO.LOW)
            rest = self.period - (time.perf_counter() - start)
            if rest > 0:
                time.sleep(rest)

    def cleanup(self):
        try:
            self._running = False
            if self._thread is not None:
                self._thread.join(timeout=1.0)
            if self.use_hw and self._pwm is not None:
                self._pwm.stop()
            GPIO.output(self.pin, GPIO.LOW)
            GPIO.cleanup()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.cleanup()
