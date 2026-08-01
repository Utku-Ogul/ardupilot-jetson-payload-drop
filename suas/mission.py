"""
MissionWatcher -- downloads the uploaded mission once and exposes:

  1) wp_seqs  : the set of NAV_WAYPOINT sequence numbers (mapping uses these; DELAY and
                other command types are skipped)
  2) items    : a readable summary of every mission item (for the pre-flight listing)
  3) complete : has the survey finished? This is the trigger for the GUIDED takeover.
                It deliberately does NOT rely on a terminal LOITER_UNLIM, because
                ArduCopter never emits MISSION_ITEM_REACHED for one.

The download can be triggered two ways:
  * main.py calls download() during the pre-flight interactive setup (after the operator
    uploads the mission from the ground station), then locks it with mark_downloaded().
  * With no interactive setup, update() downloads automatically on entering AUTO.

The single MAVLink connection is shared by main + tracker + mapping, so receives are
serialised with mav.recv_lock.
"""
from pymavlink import mavutil

from suas.utils.logger import get_logger
from suas.vehicle.connection import mark_alive, mark_alive_worker

log = get_logger("mission")


def _cmd_name(cmd: int) -> str:
    """Turn a command id into a short readable name (e.g. 16 -> WAYPOINT)."""
    try:
        name = mavutil.mavlink.enums["MAV_CMD"][cmd].name
        return name.replace("MAV_CMD_NAV_", "").replace("MAV_CMD_", "")
    except Exception:
        return f"CMD_{cmd}"


class MissionWatcher:

    def __init__(self, mav, pose_reader):
        self._mav = mav
        self._pose = pose_reader
        self._tried = False         # has the automatic download been attempted?
        self.count = None           # total number of mission items
        self.last_seq = None        # last item sequence (count-1), usually LOITER_UNLIM
        self.wp_seqs = None         # NAV_WAYPOINT sequence set
        self._complete_seq = None   # last item that actually EMITS ITEM_REACHED (the
                                    # terminal LOITER is excluded) -- the GUIDED takeover
                                    # trigger watches this, so a trailing DELAY is honoured
        self.items = []             # [{seq, command, name, lat, lon, alt, delay}]
        self._complete = False
        self._mc_seen_low = False   # has MISSION_CURRENT been seen BELOW last_seq at least once?

    # --- Called from the main loop ------------------------------------------

    def update(self):
        # If the interactive setup did not download it, do so on entering AUTO.
        # IMPORTANT: only set _tried on SUCCESS. Otherwise a single failed attempt (the
        # mission had not been uploaded yet) permanently disables the download and
        # last_seq stays None, so 'complete' can never be evaluated.
        if not self._tried and self._pose.mode == "AUTO":
            if self.download():
                self._tried = True

        if self.last_seq is not None and not self._complete:
            if self._reached_last_item():
                self._complete = True
                log.info("[MISSION] Survey finished (reached seq=%s, last item=%d) -> "
                         "GUIDED takeover allowed", self._complete_seq, self.last_seq)

    def _reached_last_item(self) -> bool:
        """Has the survey finished? (GUIDED takeover trigger.) Either signal suffices:

        * MISSION_ITEM_REACHED.seq >= _complete_seq -- the PRIMARY path.
          We trigger on the last item that actually emits ITEM_REACHED (usually the DELAY
          after the final waypoint) rather than on the terminal LOITER_UNLIM. This means:
            (1) the final waypoint's photo AND its DELAY complete properly,
            (2) we do not depend on the terminal LOITER, which never reports reached,
            (3) the trigger rests on a signal that is known to arrive.

        * MISSION_CURRENT.seq >= last_seq -- BACKUP path, used if the vehicle actually
          entered the terminal LOITER (MISSION_CURRENT is streamed at 2 Hz by
          pose.request_streams). Normally the primary path fires first.

        NOTE: if no target has been saved yet the tracker is not in TARGET_SAVED, so main
        never calls trigger_goto; the vehicle simply loiters and keeps searching until one is.
        """
        msgs = self._mav.messages
        mir = msgs.get("MISSION_ITEM_REACHED")
        if mir is not None and self._complete_seq is not None and mir.seq >= self._complete_seq:
            return True
        # Backup path, guarded by a "made real progress" condition. MISSION_CURRENT is
        # streamed, so the cache refills ~0.5 s after reset(). If the flight controller
        # RESUMES a mission at the terminal loiter (MIS_RESTART=0), MISSION_CURRENT.seq
        # would arrive fresh at last_seq and latch a false 'complete', skipping the second
        # survey. Requiring the sequence to have been seen BELOW last_seq first proves the
        # mission genuinely advanced. In a normal flight (seq 0, 1, 2, ...) this always holds.
        mc = msgs.get("MISSION_CURRENT")
        if mc is not None:
            if mc.seq < self.last_seq:
                self._mc_seen_low = True
            elif self._mc_seen_low:
                return True
        return False

    @property
    def complete(self) -> bool:
        return self._complete

    def reset(self):
        """Clear state for a new AUTO mission (when the tracker returns to WAITING_AUTO).

        This makes the second sortie re-download the mission and clears the stale
        'complete' latch.
        """
        self._tried = False
        self._complete = False
        self.count = None
        self.last_seq = None
        self.wp_seqs = None
        self._complete_seq = None
        self._mc_seen_low = False
        self.items = []
        # Drop stale completion messages so they cannot latch a false 'complete' on the
        # next sortie. MISSION_CURRENT is streamed and refills quickly (which is why the
        # _mc_seen_low guard above exists), but clearing the latched, event-driven
        # MISSION_ITEM_REACHED here still matters. Mirrors PhotoCapture.reset().
        self._mav.messages.pop("MISSION_CURRENT", None)
        self._mav.messages.pop("MISSION_ITEM_REACHED", None)

    def is_waypoint(self, seq: int) -> bool:
        """Is this sequence a real NAV_WAYPOINT? (If the mission is unknown, accept all.)"""
        return self.wp_seqs is None or seq in self.wp_seqs

    def first_waypoint_seq(self):
        """The first NAV_WAYPOINT sequence (None if there is none) -- interactive default."""
        return min(self.wp_seqs) if self.wp_seqs else None

    def mark_downloaded(self):
        """Lock out the automatic download after a successful interactive one."""
        self._tried = True

    # --- Download ------------------------------------------------------------

    def download(self) -> bool:
        """Download the mission from the flight controller.

        On success fills count/last_seq/wp_seqs/items and returns True. Returns False if
        the mission is empty or could not be fetched (safe to retry). Does NOT modify
        _tried -- update() and mark_downloaded() own that.
        """
        lock = getattr(self._mav, "recv_lock", None)
        if lock:
            lock.acquire()
        try:
            tgt_sys = self._mav.target_system
            tgt_comp = self._mav.target_component
            log.info("[MISSION] Downloading mission... (target sys=%s comp=%s)", tgt_sys, tgt_comp)

            # MISSION_REQUEST_LIST / MISSION_COUNT: over UDP either the request or the
            # reply can be dropped, so retry a few times (a single attempt was the main
            # cause of spurious "could not fetch" errors).
            cnt = None
            for attempt in range(1, 6):
                mark_alive()
                mark_alive_worker()   # a blocking download must not starve the deadman
                self._mav.mav.mission_request_list_send(tgt_sys, tgt_comp)
                cnt = self._mav.recv_match(type="MISSION_COUNT", blocking=True, timeout=1.5)
                if cnt is not None:
                    break
                log.warning("[MISSION] No MISSION_COUNT reply (attempt %d/5)...", attempt)

            # Distinguish the two failure modes -- they need different fixes.
            if cnt is None:
                log.error("[MISSION] MISSION_COUNT never arrived -> the flight controller is "
                          "UNREACHABLE. Check: does the connection log show the autopilot "
                          "system id? Is the link up? Is target_system=%s correct?", tgt_sys)
                return False
            if cnt.count == 0:
                log.warning("[MISSION] No mission loaded on the flight controller (count=0) -> "
                            "upload one from the ground station, then retry.")
                return False

            self.count = cnt.count
            self.last_seq = cnt.count - 1
            seqs = set()
            items = []
            for i in range(cnt.count):
                item = None
                for _ in range(5):        # individual items can be dropped too -> retry
                    mark_alive()
                    mark_alive_worker()
                    self._mav.mav.mission_request_int_send(tgt_sys, tgt_comp, i)
                    item = self._mav.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=1.5)
                    if item is not None and item.seq == i:
                        break
                if item is None:
                    log.warning("[MISSION] Could not fetch item %d -- skipping", i)
                    continue
                if item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                    seqs.add(item.seq)
                items.append({
                    "seq": item.seq,
                    "command": item.command,
                    "name": _cmd_name(item.command),
                    "lat": item.x / 1e7,   # MISSION_ITEM_INT: degrees * 1e7
                    "lon": item.y / 1e7,
                    "alt": item.z,
                    "delay": item.param1,  # hold/delay [s] for a NAV_WAYPOINT
                })
            # Close the transaction (some flight controllers wait for the ACK and would
            # otherwise resend).
            self._mav.mav.mission_ack_send(
                tgt_sys, tgt_comp, mavutil.mavlink.MAV_MISSION_ACCEPTED)
            self.wp_seqs = seqs
            self.items = items
            # GUIDED takeover trigger: a terminal NAV_LOITER_UNLIM never emits
            # ITEM_REACHED, so walk backwards and take the first non-loiter item as the
            # "survey finished" signal (usually the final waypoint's DELAY). That way the
            # last photo and its delay both complete, and only the infinitely-waiting
            # terminal loiter is skipped. If the mission ends in a plain waypoint,
            # _complete_seq == last_seq.
            loiter = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
            self._complete_seq = next((it["seq"] for it in reversed(items)
                                       if it["command"] != loiter), self.last_seq)
            log.info("[MISSION] count=%d  last_seq=%d  finish_seq=%s  NAV_WAYPOINTs=%s",
                     self.count, self.last_seq, self._complete_seq, sorted(seqs))
            return True
        except Exception as e:
            log.warning("[MISSION] download error: %s", e)
            return False
        finally:
            if lock:
                lock.release()

    # --- Listing -------------------------------------------------------------

    def describe(self) -> str:
        """Render the downloaded mission as a readable table (interactive setup)."""
        if not self.items:
            return "(mission not downloaded)"
        out = [
            "  TYPE  seq  command            p1      lat        lon",
            "  ----  ---  -----------------  -----   ---------  ---------",
        ]
        for it in self.items:
            is_wp = it["command"] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            typ = "[WP]" if is_wp else "    "
            end = "  <- LAST" if it["seq"] == self.last_seq else ""
            out.append(
                f"  {typ}  {it['seq']:>3}  {it['name']:<17}  {it['delay']:>4.0f}s   "
                f"{it['lat']:>9.5f}  {it['lon']:>9.5f}{end}"
            )
        return "\n".join(out)
