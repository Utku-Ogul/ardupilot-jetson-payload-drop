"""Embed GPS coordinates into a JPEG's EXIF metadata (piexif)."""
import piexif

from suas.utils.logger import get_logger

log = get_logger("exif")


def _to_rational(val: float) -> tuple:
    d = int(abs(val))
    m_f = (abs(val) - d) * 60
    m = int(m_f)
    s = (m_f - m) * 60
    return ((d, 1), (m, 1), (int(s * 10000), 10000))


def write_exif(path: str, lat: float, lon: float, alt: float, heading: float = None):
    """Write GPS EXIF tags into a JPEG (plus heading if given). Failures are non-fatal."""
    try:
        exif_dict = piexif.load(path)
    except Exception:
        exif_dict = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}}

    gps = {
        piexif.GPSIFD.GPSLatitudeRef: b"N" if lat >= 0 else b"S",
        piexif.GPSIFD.GPSLatitude: _to_rational(lat),
        piexif.GPSIFD.GPSLongitudeRef: b"E" if lon >= 0 else b"W",
        piexif.GPSIFD.GPSLongitude: _to_rational(lon),
        piexif.GPSIFD.GPSAltitudeRef: 0 if alt >= 0 else 1,
        piexif.GPSIFD.GPSAltitude: (int(abs(alt) * 100), 100),
        piexif.GPSIFD.GPSMapDatum: b"WGS-84",
    }
    if heading is not None:
        gps[piexif.GPSIFD.GPSImgDirectionRef] = b"T"
        gps[piexif.GPSIFD.GPSImgDirection] = (int((heading % 360) * 100), 100)
    exif_dict["GPS"] = gps

    try:
        piexif.insert(piexif.dump(exif_dict), path)
        log.debug("EXIF written: %s (lat=%.6f lon=%.6f alt=%.1f hdg=%s)",
                  path, lat, lon, alt, f"{heading:.0f}" if heading is not None else "-")
    except Exception as e:
        log.warning("EXIF write failed: %s", e)
