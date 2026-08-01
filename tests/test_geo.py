"""
Convention tests for pixel_to_gps(). No MAVLink or camera required.

These LOCK the nadir-camera axis convention:
  image UP    (py < centre) -> NORTH (latitude increases)
  image RIGHT (px > centre) -> EAST  (longitude increases)
  yaw = 90 deg (nose east)  -> an up-pixel maps to the EAST

A 90-degree axis swap here silently biases the saved target position for detections near
the frame edge, so this test exists to prevent that regression.

Run from the repository root:
    python3 -m unittest discover -s tests
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suas.config import CAMERA_HEIGHT, CAMERA_WIDTH   # noqa: E402
from suas.navigation.geo import gps_distance_m, pixel_to_gps   # noqa: E402

CX = CAMERA_WIDTH // 2
CY = CAMERA_HEIGHT // 2

LAT0, LON0 = -35.36, 149.16
ALT = 20.0


class TestPixelToGps(unittest.TestCase):

    def test_centre_pixel_maps_to_same_position(self):
        lat, lon = pixel_to_gps(CX, CY, LAT0, LON0, ALT, 0.0)
        self.assertAlmostEqual(lat, LAT0, places=5)
        self.assertAlmostEqual(lon, LON0, places=5)

    def test_up_pixel_goes_north(self):
        # Top of the image (py < centre), nose north -> north offset -> latitude increases
        lat, lon = pixel_to_gps(CX, 20, LAT0, LON0, ALT, 0.0)
        self.assertGreater(lat, LAT0)
        self.assertAlmostEqual(lon, LON0, places=5)   # pure north -> longitude unchanged

    def test_right_pixel_goes_east(self):
        # Right of the image (px > centre), nose north -> east offset -> longitude increases
        lat, lon = pixel_to_gps(CAMERA_WIDTH - 20, CY, LAT0, LON0, ALT, 0.0)
        self.assertGreater(lon, LON0)
        self.assertAlmostEqual(lat, LAT0, places=5)   # pure east -> latitude unchanged

    def test_down_pixel_goes_south(self):
        lat, _ = pixel_to_gps(CX, CAMERA_HEIGHT - 20, LAT0, LON0, ALT, 0.0)
        self.assertLess(lat, LAT0)

    def test_yaw_90_maps_up_pixel_to_east(self):
        # Nose EAST (yaw = 90) -> an up-pixel (body-forward) points EAST
        lat, lon = pixel_to_gps(CX, 20, LAT0, LON0, ALT, 90.0)
        self.assertGreater(lon, LON0)
        self.assertAlmostEqual(lat, LAT0, places=5)

    def test_yaw_180_maps_up_pixel_to_south(self):
        lat, lon = pixel_to_gps(CX, 20, LAT0, LON0, ALT, 180.0)
        self.assertLess(lat, LAT0)
        self.assertAlmostEqual(lon, LON0, places=5)

    def test_offset_scales_with_altitude(self):
        # Ground offset is proportional to altitude: doubling the altitude doubles the offset.
        near = pixel_to_gps(CX, 20, LAT0, LON0, ALT, 0.0)
        far = pixel_to_gps(CX, 20, LAT0, LON0, ALT * 2, 0.0)
        d_near = gps_distance_m(LAT0, LON0, *near)
        d_far = gps_distance_m(LAT0, LON0, *far)
        self.assertGreater(d_far, d_near)
        self.assertAlmostEqual(d_far / d_near, 2.0, places=2)


class TestGpsDistance(unittest.TestCase):

    def test_zero_distance(self):
        self.assertAlmostEqual(gps_distance_m(LAT0, LON0, LAT0, LON0), 0.0, places=6)

    def test_known_latitude_offset(self):
        # One degree of latitude is about 111.3 km.
        d = gps_distance_m(0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(d / 1000.0, 111.3, delta=0.5)

    def test_symmetry(self):
        a = gps_distance_m(LAT0, LON0, LAT0 + 0.001, LON0 + 0.001)
        b = gps_distance_m(LAT0 + 0.001, LON0 + 0.001, LAT0, LON0)
        self.assertAlmostEqual(a, b, places=9)


class TestIbvsFocalLength(unittest.TestCase):

    def test_fx_matches_geo_projection(self):
        """IBVS_FX and the fx inside pixel_to_gps must be identical, otherwise the metric
        error used for control and the saved position would disagree."""
        from suas.config import CAMERA_HFOV_DEG, IBVS_FX
        expected = (CAMERA_WIDTH / 2) / math.tan(math.radians(CAMERA_HFOV_DEG) / 2)
        self.assertAlmostEqual(IBVS_FX, expected, places=6)


if __name__ == "__main__":
    unittest.main()
