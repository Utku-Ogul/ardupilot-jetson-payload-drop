"""Outbound MAVLink commands: mode, goto, velocity, gimbal."""
from contextlib import nullcontext

from pymavlink import mavutil

from suas.config import GOTO_MIN_ALT_M
from suas.utils.logger import get_logger

log = get_logger("control")


def _slock(mav):
    """Return mav.send_lock if present, else a no-op context.

    Serialises every *_send call across threads. On a real serial link this guards
    against frame interleaving and sequence races (the lock is created in connection.py).
    """
    return getattr(mav, "send_lock", None) or nullcontext()


# Velocity type_mask: ignore position/acceleration and the yaw angle (bit 10), but USE
# yaw_rate (bit 11) so we can send yaw_rate=0 and actively hold heading. Merely ignoring
# yaw falls through to WP_YAW_BEHAVIOR, which can rotate the nose.
_VEL_YAWRATE_MASK = 0b0000011111000111


def set_mode(mav: mavutil.mavfile, mode: str):
    mode_id = mav.mode_mapping().get(mode)
    if mode_id is None:
        log.error("Unknown mode: %s", mode)
        return
    # COMMAND_LONG / DO_SET_MODE is more reliable than the legacy set_mode_send and is
    # acknowledged with COMMAND_ACK. We do not block on the ACK here (recv_lock/threads);
    # the tracker verifies via HEARTBEAT.custom_mode (pose.mode) and resends if needed.
    with _slock(mav):
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id,
            0, 0, 0, 0, 0)
    log.info("Mode command sent -> %s", mode)


def goto_global(mav: mavutil.mavfile, lat: float, lon: float, alt: float):
    """Fly to a GPS coordinate in GUIDED mode."""
    # SAFETY: a GUIDED position target is NOT subject to GUID_TIMEOUT, and a stale/zero
    # altitude (relative_alt=0 means ground level) is effectively a land command. The
    # minimum-altitude fence is the last wall; this gate is the second line of defence.
    if alt <= GOTO_MIN_ALT_M:
        log.error("GOTO REJECTED -- unsafe altitude %.1fm (<= %.1fm)", alt, GOTO_MIN_ALT_M)
        return
    with _slock(mav):
        mav.mav.mission_item_int_send(
            mav.target_system, mav.target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,      # current = 2 -> guided-mode waypoint
            1,      # autocontinue
            0, 0, 0, 0,
            int(lat * 1e7), int(lon * 1e7),
            alt)
    log.info("GOTO -> lat=%.6f lon=%.6f alt=%.1fm", lat, lon, alt)


def send_velocity_body(mav: mavutil.mavfile, vx_fwd: float, vy_right: float, vz_down: float = 0.0):
    """BODY-frame velocity command [m/s]: vx=forward(nose), vy=right, vz=down. GUIDED only.

    ArduCopter rotates the body velocity into NE internally using the live yaw, so this
    code never touches yaw and the yaw-mismatch flyaway class cannot occur.
    """
    with _slock(mav):
        mav.mav.set_position_target_local_ned_send(
            0,
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,   # 9 -- body frame
            _VEL_YAWRATE_MASK,    # vx,vy,vz + yaw_rate=0 (active heading hold)
            0, 0, 0,
            vx_fwd, vy_right, vz_down,
            0, 0, 0,
            0, 0.0)               # yaw=0 (ignored), yaw_rate=0.0 (used -> heading lock)


def stop(mav: mavutil.mavfile):
    """Zero velocity -- frame independent, sent in the body frame."""
    send_velocity_body(mav, 0.0, 0.0, 0.0)


def set_gimbal_nadir(mav: mavutil.mavfile):
    """Point the gimbal straight down (nadir, pitch=-90 deg). Real vehicle only.

    pixel_to_gps and IBVS both ASSUME a nadir camera. If the gimbal is not pointing down,
    the target projection is garbage and IBVS can drive the body velocity toward the
    horizon (only the target geofence would catch it).

    Sends DO_MOUNT_CONFIGURE (MAVLINK_TARGETING) + DO_MOUNT_CONTROL(pitch=-90); newer
    firmware also gets a gimbal-manager PITCHYAW command. One of them will apply,
    depending on firmware. Verified via pose.gimbal_nadir_ok.
    """
    try:
        with _slock(mav):
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE, 0,
                mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING, 0, 0, 0, 0, 0, 0)
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, 0,
                -90.0, 0.0, 0.0, 0, 0, 0,
                mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
        log.info("Gimbal nadir command sent (pitch=-90 deg, MAVLINK_TARGETING)")
    except Exception as e:
        log.warning("Gimbal nadir (mount) command failed: %s", e)
    try:   # newer firmware gimbal manager; older dialects lack the command -> swallow
        with _slock(mav):
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0,
                -90.0, 0.0, float("nan"), float("nan"), 0, 0, 0)
    except Exception:
        pass
