"""
Simple GPIO indicators (buzzer, status LED, spare outputs).

These are separate from the payload drop pins: the drop sequence runs in its own process
(see drop_sequence.py) so its GPIO setup/cleanup cannot disturb these pins.

GPIO_ENABLED = False -> every call is logged but nothing is driven (simulation/bench).
GPIO_ENABLED = True  -> requires Jetson.GPIO:
                          pip install Jetson.GPIO
                          sudo usermod -aG gpio $USER   (then re-login)
"""
import atexit
import time

from suas.config import AUX_PINS, BUZZER_PIN, GPIO_ENABLED, LED_STATUS_PIN
from suas.utils.logger import get_logger

log = get_logger("gpio")

# Pin numbers use BOARD (physical) numbering and come from suas/config.py.
_ALL_PINS = [BUZZER_PIN, LED_STATUS_PIN, *AUX_PINS]

_enabled = GPIO_ENABLED
if _enabled:
    try:
        import Jetson.GPIO as GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        for _p in _ALL_PINS:
            GPIO.setup(_p, GPIO.OUT, initial=GPIO.LOW)
        log.info("GPIO initialised (BOARD mode) -- pins: %s", _ALL_PINS)
    except ImportError:
        _enabled = False
        log.error("Jetson.GPIO not found -- GPIO indicators disabled")
    except Exception as e:
        # Library present but the board is not recognised / the pins are busy / no
        # permission. Disable the indicators rather than killing the import of main.py:
        # they are cosmetic, and the payload runs in its own process anyway.
        _enabled = False
        log.error("GPIO init failed (%s) -- indicators disabled. Check the 'gpio' group "
                  "membership and that no other process holds the pins.", e)
else:
    log.info("GPIO indicators disabled (simulation mode)")


def _set(pin: int, state: bool):
    if _enabled:
        import Jetson.GPIO as GPIO
        GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)


def buzzer(pulses: int = 3, on_ms: int = 200, off_ms: int = 200):
    """Beep the buzzer `pulses` times."""
    log.info("Buzzer: %d pulses", pulses)
    for _ in range(pulses):
        _set(BUZZER_PIN, True)
        time.sleep(on_ms / 1000)
        _set(BUZZER_PIN, False)
        time.sleep(off_ms / 1000)


def led(state: bool):
    """Turn the status LED on or off."""
    log.debug("LED -> %s", "ON" if state else "OFF")
    _set(LED_STATUS_PIN, state)


def aux(num: int, state: bool):
    """Drive spare output `num` (1-based index into AUX_PINS)."""
    if not 1 <= num <= len(AUX_PINS):
        log.error("Invalid aux output: %d (expected 1..%d)", num, len(AUX_PINS))
        return
    _set(AUX_PINS[num - 1], state)


_cleaned = False


def cleanup():
    """Drive every pin LOW, then release them.

    Driving LOW first matters: on an abnormal exit (uncaught exception -> atexit) we must
    not leave an output latched HIGH.
    """
    global _cleaned
    if _cleaned:
        return
    _cleaned = True
    if _enabled:
        import Jetson.GPIO as GPIO
        for _p in _ALL_PINS:
            try:
                GPIO.output(_p, GPIO.LOW)
            except Exception:
                pass
        try:
            GPIO.cleanup()
        except Exception:
            pass
    log.info("GPIO cleaned up (pins LOW)")


# Uncaught exception or normal exit -> interpreter exit -> atexit -> pins LOW.
atexit.register(cleanup)
