"""
Dual DC motor driver for a two-channel H-bridge board (e.g. HW-231 / dual MC33886).

Each channel is a full H-bridge with no enable pin: it is driven by two logic inputs
(IN1 = forward PWM, IN2 = reverse PWM). Stopping means driving both inputs LOW, which
also de-energises the motor.

Default pin map (BOARD numbering):
    Motor A: pin 32 = IN1 (forward), pin 33 = IN2 (reverse)
    Motor B: pin 29 = IN1 (forward), pin 31 = IN2 (reverse)
    Common ground: pin 6 -> module GND (also common with the battery negative)

Motor supply: 5-28 V to the module VS/V+; motor leads to the channel OUT1/OUT2.

3.3 V WARNING: the MC33886 needs VIH = 3.5 V, and the Jetson drives 3.3 V, so the logic
high is marginal. If the motor behaves erratically, add a 5 V level shifter
(74HCT244 / 74AHCT125) on the IN lines. The LOW level (0 V) is always fine.
"""
import atexit
import threading
import time
import warnings

import Jetson.GPIO as GPIO

# The "Carrier board is not from a Jetson Developer Kit" warning is cosmetic here.
warnings.filterwarnings("ignore")


class _SoftPWM:
    """Software PWM on a single pin, driven by its own thread.

    SAFETY: deliberately lock-free. A single float assignment is atomic under the GIL,
    so if a signal handler (SIGTERM/SIGINT) fires in the middle of this call, the
    de-energise path (duty=0 / pin LOW) cannot deadlock on a lock this thread holds.
    """

    def __init__(self, pin, freq_hz):
        self.pin = pin
        self.period = 1.0 / float(freq_hz)
        self._duty = 0.0           # 0..100
        self._running = False
        self._thread = None

    def start(self, duty):
        self._duty = max(0.0, min(100.0, float(duty)))
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_duty(self, duty):
        self._duty = max(0.0, min(100.0, float(duty)))   # atomic; no lock by design

    def _loop(self):
        while self._running:
            d = self._duty
            if d <= 0.0:
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(self.period)
            elif d >= 100.0:
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(self.period)
            else:
                t_on = self.period * d / 100.0
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(t_on)
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(self.period - t_on)

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            GPIO.output(self.pin, GPIO.LOW)
        except Exception:
            pass


class _Channel:
    """One motor channel (one H-bridge).

    Does NOT call GPIO setmode/setup/cleanup -- the parent DualMotor owns those, so a
    single GPIO.cleanup() is safe. forward = IN1 PWM, IN2 low; reverse = the opposite;
    stop = both low (brake).
    """

    def __init__(self, in1, in2, pwm_freq, dead_time):
        self.in1 = in1
        self.in2 = in2
        self.dead_time = max(0.0, float(dead_time))
        self._dir = None                       # "F" | "R" | None (stopped)
        self._pwm1 = _SoftPWM(in1, pwm_freq)
        self._pwm2 = _SoftPWM(in2, pwm_freq)
        self._pwm1.start(0)
        self._pwm2.start(0)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, float(v)))

    def _change_dir(self, new_dir):
        """If the direction actually changes, stop first and wait out the dead time
        (avoids the current spike of a reversal under load)."""
        if self._dir is not None and new_dir != self._dir:
            self._pwm1.set_duty(0)
            self._pwm2.set_duty(0)
            if self.dead_time > 0:
                time.sleep(self.dead_time)
        self._dir = new_dir

    def forward(self, speed):
        speed = self._clamp(abs(speed), 0, 100)
        self._change_dir("F")
        self._pwm2.set_duty(0)
        self._pwm1.set_duty(speed)

    def reverse(self, speed):
        speed = self._clamp(abs(speed), 0, 100)
        self._change_dir("R")
        self._pwm1.set_duty(0)
        self._pwm2.set_duty(speed)

    def drive(self, speed):
        """Speed and direction in one call: -100..100 (positive = forward, 0 = stop)."""
        speed = self._clamp(speed, -100, 100)
        if speed > 0:
            self.forward(speed)
        elif speed < 0:
            self.reverse(-speed)
        else:
            self.stop()

    def ramp(self, target, ramp_s=1.5, hz=50):
        """Soft-start from 0 to target (%, -100..100).

        Prevents the inrush/stall current spike that can brown out the Jetson. The
        direction is constant (the sign of target does not change), so the dead time is
        only triggered on the first step. NOTE: stop() is never ramped -- it cuts instantly.
        """
        target = self._clamp(target, -100, 100)
        steps = max(1, int(ramp_s * hz))
        for i in range(1, steps + 1):
            self.drive(target * i / steps)
            time.sleep(1.0 / hz)
        self.drive(target)

    def stop(self):
        """Stop the motor (IN1 = IN2 = 0 -> low-side brake). Resets the direction."""
        self._pwm1.set_duty(0)
        self._pwm2.set_duty(0)
        self._dir = None

    def stop_threads(self):
        self._pwm1.stop()
        self._pwm2.stop()


class DualMotor:
    """Two DC motors on a dual H-bridge board, exposed as .a and .b.

    Sets up every pin in one place and releases them with a single GPIO.cleanup().
    """

    def __init__(self, a_in1=32, a_in2=33, b_in1=29, b_in2=31,
                 pwm_freq=100, dead_time=0.05):
        self.pins = [a_in1, a_in2, b_in1, b_in2]
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pins, GPIO.OUT, initial=GPIO.LOW)   # all four LOW -> both motors stopped
        self.a = _Channel(a_in1, a_in2, pwm_freq, dead_time)
        self.b = _Channel(b_in1, b_in2, pwm_freq, dead_time)
        atexit.register(self.cleanup)

    def channel(self, name: str) -> _Channel:
        """Return channel 'A' or 'B' by name."""
        return self.a if str(name).upper() == "A" else self.b

    def stop(self):
        """Stop both motors (de-energise: every IN pin low)."""
        self.a.stop()
        self.b.stop()

    def cleanup(self):
        try:
            self.a.stop()
            self.b.stop()
            self.a.stop_threads()
            self.b.stop_threads()
            for p in self.pins:
                GPIO.output(p, GPIO.LOW)
            GPIO.cleanup()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.cleanup()
