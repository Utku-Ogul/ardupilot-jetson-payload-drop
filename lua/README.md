# Flight-controller Lua scripts

ArduPilot Lua scripts required for SUAS 2026 that the stock firmware cannot cover with
parameters alone. They run **on the flight controller**, independent of the Jetson.

| File | Purpose |
|---|---|
| `comms_loss_termination.lua` | Appendix B: *automatic flight termination after 90 s of comms loss* (rotary wing: throttle closed) |
| `scripting.param` | `SCR_ENABLE` + the script's `SCR_USER1..3` settings (Mission Planner → *Load from file*) |

The other Appendix B items are parameters, not scripts:

| Requirement | How |
|---|---|
| RTL after 15 s of comms loss | RC: `FS_THR_ENABLE=1`. Ground station: `SYSID_MYGCS=255`, `FS_GCS_ENABLE=1`, `FS_GCS_TIMEOUT=15`, `FS_OPTIONS=0` |
| Termination by the safety pilot | a spare RC channel with `RCx_OPTION=81` (Disarm) or `31` (Motor E-Stop) |
| Termination by the GCS | Mission Planner → Actions → Terminate (`MAV_CMD_DO_FLIGHTTERMINATION`) / force disarm |

## comms_loss_termination.lua

Written against ArduCopter 4.6.3 (`Copter-4.6.3`). While **armed**, if the RC link stays lost
for `SCR_USER1` seconds (default 90) the script calls `arming:disarm()`. A scripting disarm is
accepted in the air (only the MAVLink/DDS disarm methods require `land_complete`), so the
motors stop. It does nothing while disarmed.

RC loss is read from the flight controller's own RC failsafe flag (`rc:has_valid_input()`),
the same event that triggers the 1 s RTL, so **`FS_THR_ENABLE` must stay 1** — with 0 the flag
is never set and the script cannot see the loss.

| Parameter | Meaning | Recommended |
|---|---|---|
| `SCR_USER1` | termination delay in seconds (`0` → 90) | `0` |
| `SCR_USER2` | `0` = RC loss alone · `1` = RC **or** GCS loss · `2` = RC **and** GCS loss | `0` |
| `SCR_USER3` | `1` = DRY RUN (messages only, no disarm) | `1` on the bench, **`0` for competition** |

Policy `1` is the strictest reading of the handbook, but a laptop/telemetry dropout alone
would then bring down a vehicle the pilot still controls; `0` is recommended.

Messages (Mission Planner → Messages): `TERM: active 90s policy 0` at boot and every 2 min
while disarmed, `TERM: armed, watching RC` on arming, `TERM: RC loss started`,
`TERM: TERMINATION in 30s` at 60 s, `TERMINATION: RC lost 90s -> DISARM` at 90 s
(`TERM(DRY): ... -> no disarm` in dry-run mode).

### Install

1. Copy `comms_loss_termination.lua` to the SD card under `/APM/scripts/`
   (Mission Planner → CONFIG → MAVFtp → `APM/scripts` → Upload, or copy the card on a PC).
2. Load `scripting.param` (or set `SCR_ENABLE=1`), write, **reboot** the flight controller.
3. After the reboot, Messages must show `TERM: active ...` and no `Lua:` error line.
   A script error does **not** block arming, so check this line before every flight.
4. Before competition flights set `SCR_USER3=0`.

### Test

- Logic (no hardware): `pip install lupa && python3 tests/test_comms_loss_termination.py`
- Bench (props OFF): arm, switch the transmitter off → `Radio Failsafe` within ~1 s. On the
  ground the flight controller disarms on failsafe immediately, so the 90 s path cannot be
  seen on the bench.
- The 90 s path in SITL: take off in GUIDED, `param set SIM_RC_FAIL 1` → RTL after ~1 s,
  warning at 60 s, disarm at 90 s. `SIM_RC_FAIL 0` before 90 s resets the counter.
