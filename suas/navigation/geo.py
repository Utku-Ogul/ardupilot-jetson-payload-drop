"""
Pixel -> GPS projection.

Assumes a nadir-pointing camera (straight down) over flat ground.
"""
import math

from suas.config import CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_HFOV_DEG


def pixel_to_gps(
    px: int, py: int,
    drone_lat: float, drone_lon: float, drone_alt: float,
    yaw_deg: float,
) -> tuple[float, float]:
    """
    Convert a camera pixel to a GPS coordinate.

    px, py   : pixel position (0,0 = top-left)
    drone_*  : vehicle position and altitude above ground
    yaw_deg  : vehicle heading (clockwise from north, degrees)
    returns  : (lat, lon)
    """
    hfov = math.radians(CAMERA_HFOV_DEG)

    # Focal length in pixels, assuming square pixels (fx == fy). This matches the
    # pinhole model used by the simulator (which derives vfov from hfov and aspect)
    # and IBVS_FX exactly, so the vertical offset is computed with fx as well.
    # (A previous 'vfov ~= hfov * H/W' linear approximation introduced ~12% vertical
    #  error at 16:9 / 81 deg and was removed.)
    fx = (CAMERA_WIDTH / 2) / math.tan(hfov / 2)

    # Ground offset in metres along the image axes: pixel_offset * altitude / fx
    dx_body = (px - CAMERA_WIDTH / 2) * drone_alt / fx    # image-RIGHT ground offset [m]
    dy_body = (py - CAMERA_HEIGHT / 2) * drone_alt / fx   # image-DOWN  ground offset [m]

    # Rotate into the NED frame using vehicle yaw.
    # Nadir convention: image-UP -> forward/NORTH, image-RIGHT -> EAST.
    # Since dx_body is image-RIGHT and dy_body is image-DOWN:
    #   body-forward = -dy_body, body-right = +dx_body.
    # test_geo.py locks this convention in place (a 90 deg axis swap here biases the
    # saved target position for detections near the frame edge).
    yaw = math.radians(yaw_deg)
    north = -dy_body * math.cos(yaw) - dx_body * math.sin(yaw)
    east = -dy_body * math.sin(yaw) + dx_body * math.cos(yaw)

    # Metres -> degrees (WGS-84 approximation)
    r = 6378137.0
    dlat = north / r
    dlon = east / (r * math.cos(math.radians(drone_lat)))

    return (
        drone_lat + math.degrees(dlat),
        drone_lon + math.degrees(dlon),
    )


def gps_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two GPS points, in metres."""
    r = 6378137.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return r * 2 * math.asin(math.sqrt(a))
