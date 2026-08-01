"""Vehicle state from MAVLink: position, attitude, GPS/EKF health, link liveness."""
import math
import time
from dataclasses import dataclass

from pymavlink import mavutil

from suas.config import (STABILIZE_PITCH_MAX, STABILIZE_ROLL_MAX, FIRE_TILT_MAX_DEG,
                         POSE_STALE_S, GPS_EKF_STALE_S, POSE_MIN_SATS, LINK_TIMEOUT_S,
                         GIMBAL_NADIR_TOL_DEG, SIM_MODE, GIMBAL_MANUAL_NADIR)
from suas.utils.logger import get_logger

log = get_logger("pose")

# MOUNT_ORIENTATION (newer firmware, degrees) is not defined in every pymavlink dialect,
# so fall back to the numeric id (265). Older dialects use MOUNT_STATUS (158, centidegrees).
_MOUNT_ORIENTATION_ID = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_MOUNT_ORIENTATION", 265)
# GIMBAL_DEVICE_ATTITUDE_STATUS (gimbal protocol v2, quaternion). Modern ArduPilot emits
# this INSTEAD of MOUNT_ORIENTATION. If it is not parsed, gimbal_nadir_ok stays False
# forever and IBVS never starts on the real vehicle.
_GIMBAL_DEV_ATT_ID = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS", 285)


@dataclass
class Pose:
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0        # relative altitude [m] (home-referenced) -- used by IBVS/goto/fence
    alt_amsl: float = 0.0   # AMSL altitude [m] -- for correct EXIF geotag datum
    vx: float = 0.0         # north velocity [m/s]
    vy: float = 0.0         # east velocity [m/s] -- used by the blind-drop motion gate
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    # Validity / health signals used by the safety gates
    fix_type: int = -1      # -1 = GPS_RAW_INT never arrived (sentinel)
    sats: int = 0
    hdop: float = 99.0
    t_pos: float = 0.0      # monotonic time of last GLOBAL_POSITION_INT
    t_att: float = 0.0      # monotonic time of last ATTITUDE
    t_gps: float = 0.0      # monotonic time of last GPS_RAW_INT
    t_ekf: float = 0.0      # monotonic time of last EKF_STATUS_REPORT
    t_gimbal: float = 0.0   # monotonic time of last gimbal attitude message
    ekf_flags: int = 0
    ekf_vel_var: float = 1.0
    ekf_compass_var: float = 1.0
    gimbal_pitch_deg: float = 999.0   # nadir ~= -90; 999 = no measurement yet
    gimbal_roll_deg: float = 0.0      # ~0 at nadir. Logged only -- there is deliberately no
                                      # roll gate (roll decoding is unreliable at the nadir
                                      # singularity and would cause nuisance trips).


class PoseReader:
    """Reads position, attitude, GPS/EKF health and link liveness from MAVLink."""

    def __init__(self, mav: mavutil.mavfile):
        self._mav = mav
        self.pose = Pose()
        self._mode = ""
        self._last_hb = 0.0
        self._armed = False

    def update(self):
        """Non-blocking -- drains all pending messages (recv is serialised by a lock)."""
        lock = getattr(self._mav, "recv_lock", None)
        if lock is not None:
            lock.acquire()
        try:
            self._drain()
        finally:
            if lock is not None:
                lock.release()

    def _drain(self):
        while True:
            msg = self._mav.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()

            if t == "GLOBAL_POSITION_INT":
                self.pose.lat = msg.lat / 1e7
                self.pose.lon = msg.lon / 1e7
                self.pose.alt = msg.relative_alt / 1e3
                self.pose.alt_amsl = msg.alt / 1e3      # mm -> m, AMSL (for EXIF)
                self.pose.vx = msg.vx / 100.0           # cm/s -> m/s (north)
                self.pose.vy = msg.vy / 100.0           # cm/s -> m/s (east)
                self.pose.t_pos = time.monotonic()

            elif t == "ATTITUDE":
                self.pose.roll_deg = math.degrees(msg.roll)
                self.pose.pitch_deg = math.degrees(msg.pitch)
                self.pose.yaw_deg = math.degrees(msg.yaw)
                self.pose.t_att = time.monotonic()

            elif t == "GPS_RAW_INT":
                self.pose.fix_type = msg.fix_type
                self.pose.sats = msg.satellites_visible
                self.pose.hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.0
                self.pose.t_gps = time.monotonic()

            elif t == "EKF_STATUS_REPORT":
                self.pose.ekf_flags = msg.flags
                self.pose.ekf_vel_var = msg.velocity_variance
                self.pose.ekf_compass_var = msg.compass_variance
                self.pose.t_ekf = time.monotonic()

            elif t in ("GIMBAL_DEVICE_ATTITUDE_STATUS", "MOUNT_ORIENTATION", "MOUNT_STATUS"):
                # Gimbal telemetry is the only positive input to the fail-closed
                # gimbal_nadir_ok gate, so reject foreign-system packets on a shared bus.
                # Filter on SYSTEM only -- a legitimate gimbal's component id varies by
                # firmware, and filtering that would keep gimbal_nadir_ok permanently False.
                if msg.get_srcSystem() != self._mav.target_system:
                    continue
                if t == "GIMBAL_DEVICE_ATTITUDE_STATUS":
                    # Gimbal protocol v2 (quaternion), q = [w, x, y, z].
                    # PREFLIGHT: verify with a MAVLink inspector which message your FC
                    # emits, and that physical nadir really reads pitch ~= -90 (a sign
                    # error keeps this gate permanently closed).
                    try:
                        w, x, y, z = msg.q
                        self.pose.gimbal_pitch_deg = math.degrees(
                            math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))
                        self.pose.gimbal_roll_deg = math.degrees(
                            math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
                        self.pose.t_gimbal = time.monotonic()
                    except Exception:
                        pass    # malformed packet -> keep old value; the freshness gate covers us
                elif t == "MOUNT_ORIENTATION":              # compatible firmware (degrees)
                    self.pose.gimbal_pitch_deg = msg.pitch  # nadir ~= -90
                    self.pose.gimbal_roll_deg = getattr(msg, "roll", 0.0)
                    self.pose.t_gimbal = time.monotonic()
                else:                                       # MOUNT_STATUS (centidegrees)
                    self.pose.gimbal_pitch_deg = msg.pointing_a / 100.0
                    self.pose.gimbal_roll_deg = msg.pointing_b / 100.0
                    self.pose.t_gimbal = time.monotonic()

            elif t == "HEARTBEAT":
                # SAFETY: only accept heartbeats from the actual autopilot. On a shared bus
                # (mavlink-router / telemetry radio / multi-endpoint SITL) a foreign GCS
                # heartbeat would keep the link deadman falsely alive and mask mode/armed.
                if (msg.get_srcSystem() == self._mav.target_system and
                        msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1):
                    self._last_hb = time.monotonic()
                    self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mapping = {v: k for k, v in self._mav.mode_mapping().items()}
                    self._mode = mapping.get(msg.custom_mode, "UNKNOWN")

    # --- Mode / status ------------------------------------------------------

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def ground_speed(self) -> float:
        """Horizontal ground speed [m/s] from GLOBAL_POSITION_INT vx/vy.

        Used by the blind GPS-drop gate: only release the payload while (nearly) stopped.
        Freshness is tied to the same t_pos as have_fix, so a stale velocity is never
        mistaken for "stopped".
        """
        return math.hypot(self.pose.vx, self.pose.vy)

    @property
    def is_stable(self) -> bool:
        """Strict 3-degree tilt gate -- PHOTO stabilisation only (mapping).

        The airdrop does NOT use this: in wind the vehicle legitimately leans to hold
        position. A frozen ATTITUDE stream makes this return False (no false 'stable').
        """
        if self.pose.t_att <= 0.0 or (time.monotonic() - self.pose.t_att) >= POSE_STALE_S:
            return False
        return (abs(self.pose.pitch_deg) < STABILIZE_PITCH_MAX and
                abs(self.pose.roll_deg) < STABILIZE_ROLL_MAX)

    @property
    def fire_tilt_ok(self) -> bool:
        """Wind-tolerant tilt gate for PAYLOAD RELEASE.

        In wind the vehicle leans to hold position; that legitimate lean must not block
        the drop. So instead of the strict 3 degrees we use FIRE_TILT_MAX_DEG -- a sanity
        ceiling above ANGLE_MAX that only rejects physically impossible attitudes (sensor
        glitch / tumble). The frozen-ATTITUDE protection (freshness) is retained.
        Centring and stillness are enforced by separate gates in the tracker.
        """
        if self.pose.t_att <= 0.0 or (time.monotonic() - self.pose.t_att) >= POSE_STALE_S:
            return False
        return (abs(self.pose.pitch_deg) < FIRE_TILT_MAX_DEG and
                abs(self.pose.roll_deg) < FIRE_TILT_MAX_DEG)

    # --- Validity / health gates --------------------------------------------

    @property
    def have_fix(self) -> bool:
        """Is the position trustworthy enough to command from? (fresh + non-zero + good GPS)"""
        now = time.monotonic()
        fresh = (now - self.pose.t_pos) < POSE_STALE_S and self.pose.t_pos > 0.0
        nonzero = abs(self.pose.lat) > 1e-6 and abs(self.pose.lon) > 1e-6
        if self.pose.fix_type < 0:
            # The FC is not streaming GPS_RAW_INT -- degrade gracefully rather than lock
            # up; freshness and non-zero are enough (FC-side FS_GPS/FENCE/EKF remain).
            return fresh and nonzero
        # GPS_RAW_INT is present -> on a real vehicle also require freshness, so a frozen
        # quality snapshot is not mistaken for "good". It streams at 2 Hz, so POSE_STALE_S
        # (1.0s) was too tight (a single dropped packet caused flapping) -> GPS_EKF_STALE_S.
        gps_fresh = SIM_MODE or (now - self.pose.t_gps) < GPS_EKF_STALE_S
        return (fresh and nonzero and gps_fresh and self.pose.fix_type >= 3 and
                self.pose.sats >= POSE_MIN_SATS and self.pose.hdop < 1.4)

    @property
    def ekf_ok(self) -> bool:
        """Is the EKF position estimate healthy? (attitude + horizontal velocity +
        absolute horizontal position + low variance)"""
        f = self.pose.ekf_flags
        if f == 0:
            return True   # EKF_STATUS_REPORT never arrived -> degrade gracefully (FC FS_EKF remains)
        # Flags present -> on a real vehicle require freshness too, so we never trust a
        # frozen "healthy" snapshot indefinitely. Streams at 2 Hz -> GPS_EKF_STALE_S.
        if not SIM_MODE and (self.pose.t_ekf <= 0.0 or
                             (time.monotonic() - self.pose.t_ekf) >= GPS_EKF_STALE_S):
            return False
        need = (mavutil.mavlink.EKF_ATTITUDE |
                mavutil.mavlink.EKF_VELOCITY_HORIZ |
                mavutil.mavlink.EKF_POS_HORIZ_ABS)
        return ((f & need) == need and
                self.pose.ekf_vel_var < 0.8 and self.pose.ekf_compass_var < 0.8)

    @property
    def gimbal_nadir_ok(self) -> bool:
        """Is the gimbal pointing straight down (nadir, pitch ~= -90)?

        FAIL-CLOSED on a real vehicle: pixel_to_gps and IBVS assume a nadir camera, so an
        unverified / frozen / drooping gimbal yields garbage coordinates and garbage
        velocity. This gate is only consulted on non-SIM paths.

        To pass on a real vehicle we require: (a) at least one measurement (not the
        sentinel), (b) freshness (a frozen stream must not read as 'nadir' forever),
        (c) within the nadir tolerance. Any failure -> False -> IBVS and sampling stop.

        If your gimbal has no MAVLink telemetry, real IBVS will never start; either wire
        it up or use GIMBAL_MANUAL_NADIR (operator-confirmed, see config).
        """
        if SIM_MODE:
            return True                     # no gimbal in sim; this gate is unused there
        if GIMBAL_MANUAL_NADIR:
            return True                     # operator aimed it manually; no telemetry to verify
        if self.pose.gimbal_pitch_deg > 900.0:
            return False                    # no measurement -> cannot verify -> do not trust
        if self.pose.t_gimbal <= 0.0 or (time.monotonic() - self.pose.t_gimbal) >= POSE_STALE_S:
            return False                    # stale / frozen mount stream
        return abs(self.pose.gimbal_pitch_deg + 90.0) < GIMBAL_NADIR_TOL_DEG

    @property
    def link_alive(self) -> bool:
        """Did an FC heartbeat arrive within LINK_TIMEOUT_S? (telemetry deadman)"""
        return (time.monotonic() - self._last_hb) < LINK_TIMEOUT_S

    # --- Stream rate request ------------------------------------------------

    def request_streams(self):
        """Request the messages we need at explicit rates.

        Stream rates are not guaranteed on a real link, and the have_fix / ekf_ok /
        link_alive gates depend on them. Uses SET_MESSAGE_INTERVAL (interval in us).
        """
        streams = [
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
            (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
            # MISSION_CURRENT (#42) is the only reliable source for mission COMPLETION.
            # ArduCopter does NOT emit MISSION_ITEM_REACHED for a terminal
            # NAV_LOITER_UNLIM, so without this stream 'complete' never becomes True and
            # the tracker stays stuck in TARGET_SAVED instead of taking over in GUIDED.
            (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 2),
            (_GIMBAL_DEV_ATT_ID, 5),                                # gimbal, modern FW (#285)
            (_MOUNT_ORIENTATION_ID, 5),                             # gimbal, compatible FW (#265)
            (mavutil.mavlink.MAVLINK_MSG_ID_MOUNT_STATUS, 5),       # gimbal, oldest FW (#158)
        ]
        # Use send_lock (not recv_lock): these are OUTBOUND command_long_send calls, and
        # the main loop re-requests them every ~10s. Serialising with the other senders
        # (heartbeat, tracker velocity, control/gimbal) avoids frame interleaving on a
        # real serial link.
        lock = getattr(self._mav, "send_lock", None)
        for msg_id, hz in streams:
            interval_us = int(1e6 / hz)
            if lock is not None:
                lock.acquire()
            try:
                self._mav.mav.command_long_send(
                    self._mav.target_system, self._mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, interval_us, 0, 0, 0, 0, 0)
            finally:
                if lock is not None:
                    lock.release()
        log.info("Telemetry stream rates requested (%d messages)", len(streams))
