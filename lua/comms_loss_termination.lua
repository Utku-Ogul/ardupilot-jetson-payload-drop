-- comms_loss_termination.lua -- SUAS 2026 Appendix B:
-- "Automatic Flight Termination After 90 Seconds of Comms Loss" (rotary wing: throttle closed).
-- Written for ArduCopter 4.6.3 (tag Copter-4.6.3).
--
-- WHAT IT DOES: while the vehicle is ARMED, if the RC (and/or GCS) link stays lost for TERM_S
-- seconds it calls arming:disarm() (Method::SCRIPTING -> AP_Arming_Copter::disarm does NOT
-- refuse in the air; only the MAVLINK/DDS methods require land_complete). Motors stop =
-- "throttle closed". The 15 s RTL is NOT this script's job (FS_THR_ENABLE=1 / FS_GCS_ENABLE=1).
--
-- BINDINGS USED (verified against bindings.desc @ Copter-4.6.3):
--   rc:has_valid_input()  gcs:last_seen()  arming:is_armed()  arming:disarm()  vehicle:get_mode()
--   gcs:send_text()  param:get()  millis()  uint32_t_ud:toint()
--
-- SETTINGS (Mission Planner -> Full Parameter List, no reboot needed; 0 = default):
--   SCR_USER1 = termination delay in seconds          (0 -> 90)
--   SCR_USER2 = policy: 0 = RC loss alone is enough (recommended);
--                       1 = RC OR GCS loss (strictest reading; a telemetry dropout alone
--                           would bring the vehicle down!);
--                       2 = RC AND GCS both lost
--   SCR_USER3 = 1 -> DRY RUN: disarm is NOT called, only messages are sent (bench / first try)
--
-- INSTALL: copy to the SD card under /APM/scripts/ (MP -> MAVFtp), SCR_ENABLE=1, reboot.
-- "TERM: active ..." must appear in Messages. Test: props OFF, arm, switch the transmitter off
-- -> warning at 60 s, disarm at 90 s.

local DEFAULT_TERM_S = 90
local LOOP_MS        = 200          -- 5 Hz
local WARN_BEFORE_S  = 30           -- warn this long before terminating

local SEV_EMERG, SEV_CRIT, SEV_WARN, SEV_INFO = 0, 2, 4, 6

local rc_lost_since   = nil         -- ms (integer) - when the RC loss started
local armed_since     = nil         -- ms - last arm time (GCS silence before it does not count)
local warned          = false
local terminated      = false
local announced       = false
local last_status_ms  = 0
local last_alive_ms   = 0           -- periodic "active" reminder while disarmed
local ANNOUNCE_MS     = 120000      -- 2 min (so it is seen even if MP connects after boot)

local function now_ms()
  return millis():toint()
end

local function cfg()
  local t = param:get('SCR_USER1') or 0
  if t <= 0 then t = DEFAULT_TERM_S end
  local pol = math.floor((param:get('SCR_USER2') or 0) + 0.5)
  local dry = (param:get('SCR_USER3') or 0) >= 0.5
  return math.floor(t * 1000), pol, dry
end

-- GCS silence (ms). nil if no heartbeat was ever seen (the counter does not start).
-- Silence before arming does not count (if MP was closed before arming, arming must not terminate).
local function gcs_silence_ms(now)
  local seen = gcs:last_seen()
  if seen == nil then return nil end
  local seen_i = seen:toint()
  if seen_i == 0 then return nil end
  local ref = seen_i
  if armed_since ~= nil and armed_since > ref then ref = armed_since end
  local d = now - ref
  if d < 0 then d = 0 end              -- millis() wrap (49.7 days) guard
  return d
end

local dry_reported = false

local function terminate(reason, dry)
  if dry then
    -- DRY RUN: no latch; report once while the condition lasts
    if not dry_reported then
      dry_reported = true
      gcs:send_text(SEV_CRIT, string.format("TERM(DRY): %s -> no disarm", reason))
    end
    return
  end
  if terminated then return end
  gcs:send_text(SEV_EMERG, string.format("TERMINATION: %s -> DISARM", reason))
  if arming:disarm() then
    terminated = true           -- disarm succeeded; cleared once disarmed is seen
  else
    gcs:send_text(SEV_EMERG, "TERMINATION: disarm false, retrying")
  end
end

function update()
  local now = now_ms()
  local term_ms, policy, dry = cfg()

  -- "active" announcement: once at boot + every 2 min while disarmed
  if (not announced) or ((not arming:is_armed()) and now - last_alive_ms >= ANNOUNCE_MS) then
    announced = true
    last_alive_ms = now
    gcs:send_text(SEV_INFO, string.format("TERM: active %ds policy %d%s", term_ms // 1000, policy, dry and " DRY" or ""))
  end

  -- On the ground / disarmed: do nothing, reset the counters.
  if not arming:is_armed() then
    rc_lost_since = nil
    armed_since = nil
    warned = false
    terminated = false
    dry_reported = false
    return update, LOOP_MS
  end
  if armed_since == nil then
    armed_since = now
    gcs:send_text(SEV_INFO, string.format("TERM: armed, watching RC (%ds%s)", term_ms // 1000, dry and " DRY" or ""))
  end

  -- RC state
  local rc_ok = rc:has_valid_input()
  if rc_ok then
    if rc_lost_since ~= nil then
      gcs:send_text(SEV_INFO, "TERM: RC back, counter reset")
    end
    rc_lost_since = nil
  elseif rc_lost_since == nil then
    rc_lost_since = now
    gcs:send_text(SEV_WARN, "TERM: RC loss started")
  end
  local rc_lost_ms = rc_lost_since and (now - rc_lost_since) or 0
  if rc_lost_ms < 0 then rc_lost_since = now; rc_lost_ms = 0 end   -- millis() wrap guard

  -- GCS state (last traffic from SYSID_MYGCS)
  local gcs_ms = gcs_silence_ms(now) or 0

  -- Policy
  local trigger, reason = false, ""
  if policy == 1 then
    if rc_lost_ms >= term_ms then trigger, reason = true, string.format("RC lost %ds", rc_lost_ms // 1000)
    elseif gcs_ms >= term_ms then trigger, reason = true, string.format("GCS lost %ds", gcs_ms // 1000) end
  elseif policy == 2 then
    if rc_lost_ms >= term_ms and gcs_ms >= term_ms then trigger, reason = true, "RC and GCS lost" end
  else -- 0: RC only
    if rc_lost_ms >= term_ms then trigger, reason = true, string.format("RC lost %ds", rc_lost_ms // 1000) end
  end

  -- Warning (WARN_BEFORE_S before terminating, once)
  local worst = rc_lost_ms
  if policy == 1 and gcs_ms > worst then worst = gcs_ms end
  if policy == 2 then worst = math.min(rc_lost_ms, gcs_ms) end
  if (not warned) and worst >= (term_ms - WARN_BEFORE_S * 1000) and worst < term_ms then
    warned = true
    gcs:send_text(SEV_CRIT, string.format("TERM: TERMINATION in %ds (RC %ds GCS %ds)",
                  (term_ms - worst) // 1000, rc_lost_ms // 1000, gcs_ms // 1000))
  end
  if worst < (term_ms - WARN_BEFORE_S * 1000) then warned = false end

  if trigger then
    terminate(reason, dry)
  else
    dry_reported = false
  end

  -- Short status every 10 s (only while RC is lost or GCS is silent > 5 s; SILENT in normal flight)
  if (rc_lost_ms > 0 or gcs_ms >= 5000) and now - last_status_ms >= 10000 then
    last_status_ms = now
    gcs:send_text(SEV_INFO, string.format("TERM: RC lost %ds, GCS silent %ds, mode %d", rc_lost_ms // 1000, gcs_ms // 1000, vehicle:get_mode()))
  end

  return update, LOOP_MS
end

return update()
