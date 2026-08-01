"""Shared logging setup: stdout + rotating file handler."""
import logging
import logging.handlers
import os
import sys

_FMT = "%(asctime)s [%(name)-12s] %(levelname)s: %(message)s"
# Include the date: the Orin has no RTC (clock is set from the flight controller),
# and this file is often the only forensic record after an in-flight anomaly.
_DATE = "%Y-%m-%d %H:%M:%S"

# Rotating file log, in addition to stdout. Set up once at import time; if it cannot
# be opened (permissions/disk) we silently fall back to console only, so that logging
# never raises during flight.
_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "logs")
_file_handler = None
try:
    os.makedirs(_LOG_DIR, exist_ok=True)
    _file_handler = logging.handlers.RotatingFileHandler(
        os.path.join(_LOG_DIR, "suas.log"), maxBytes=5_000_000, backupCount=5, encoding="utf-8")
    _file_handler.setFormatter(logging.Formatter(_FMT, datefmt=_DATE))
except Exception:
    _file_handler = None


def get_logger(name: str) -> logging.Logger:
    log = logging.getLogger(name)
    if not log.handlers:
        h = logging.StreamHandler(sys.stdout)
        h.setFormatter(logging.Formatter(_FMT, datefmt=_DATE))
        log.addHandler(h)
        if _file_handler is not None:
            log.addHandler(_file_handler)   # one shared file across all loggers
        log.setLevel(logging.DEBUG)
    return log
