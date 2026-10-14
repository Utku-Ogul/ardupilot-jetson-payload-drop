#!/usr/bin/env python3
"""Logic test for lua/comms_loss_termination.lua with a mocked ArduPilot Lua API.

This is NOT real flight-controller behaviour -- it exercises the script's state machine.
The bindings themselves were checked against the Copter-4.6.3 source (lua/README.md).

Requires lupa:  pip install lupa
Run:            python3 tests/test_comms_loss_termination.py
"""
import os
import sys


def main():
    # Imported here so `python3 -m unittest discover -s tests` does not need lupa.
    from lupa import lua53

    HERE = os.path.dirname(os.path.abspath(__file__))
    SCRIPT = os.path.join(os.path.dirname(HERE), "lua", "comms_loss_termination.lua")

    L = lua53.LuaRuntime(unpack_returned_tuples=True)
    src = open(SCRIPT, encoding="utf-8").read()
    state = {"t": 0, "armed": False, "rc_ok": True, "gcs_seen": 0,
             "params": {"SCR_USER1": 0, "SCR_USER2": 0, "SCR_USER3": 0},
             "disarms": 0, "msgs": [], "mode": 4, "disarm_ok": True, "gcs_alive": True}


    class U32:
        def __init__(self, v): self.v = int(v) & 0xFFFFFFFF
        def toint(self): return self.v


    def millis(): return U32(state["t"])


    class RC:
        def has_valid_input(self): return state["rc_ok"]


    class GCS:
        def last_seen(self): return U32(state["gcs_seen"])
        def send_text(self, sev, txt): state["msgs"].append((state["t"], sev, txt))


    class ARM:
        def is_armed(self): return state["armed"]

        def disarm(self):
            state["disarms"] += 1
            if state["disarm_ok"]:
                state["armed"] = False
            return state["disarm_ok"]


    class VEH:
        def get_mode(self): return state["mode"]


    class PARAM:
        def get(self, n): return state["params"].get(n, 0)


    g = L.globals()
    g.millis = millis; g.rc = RC(); g.gcs = GCS(); g.arming = ARM(); g.vehicle = VEH(); g.param = PARAM()
    fn, delay = L.execute(src)


    def run(sec):
        nonlocal fn, delay
        end = state["t"] + sec * 1000
        while state["t"] < end:
            state["t"] += delay
            if state["gcs_seen"] and state["gcs_alive"]:
                state["gcs_seen"] = state["t"]
            fn, delay = fn()


    fails = 0


    def check(cond, label):
        nonlocal fails
        print(("PASS " if cond else "FAIL ") + label)
        fails += 0 if cond else 1


    def cycle():
        state["armed"] = False; run(1); state["armed"] = True


    def has(text): return any(text in m[2] for m in state["msgs"])


    state.update(armed=False, rc_ok=False); run(200)
    check(state["disarms"] == 0, "T1 disarmed -> nothing")
    state.update(armed=True, rc_ok=True, gcs_seen=1000); run(200)
    check(state["disarms"] == 0, "T2 armed + RC ok -> nothing")
    state["rc_ok"] = False; run(89)
    check(state["disarms"] == 0 and has("TERMINATION in"), "T3a warning by 89 s, no disarm")
    run(2)
    check(state["disarms"] == 1 and not state["armed"], "T3b disarm at 91 s")
    state.update(disarms=0, msgs=[]); cycle(); state["rc_ok"] = False; run(60); state["rc_ok"] = True
    run(5); state["rc_ok"] = False; run(60)
    check(state["disarms"] == 0, "T4 counter resets when RC returns"); run(35)
    check(state["disarms"] == 1, "T4b then disarms")
    state.update(disarms=0, rc_ok=True, gcs_alive=False); cycle(); state["gcs_seen"] = state["t"]; run(200)
    check(state["disarms"] == 0, "T5 policy 0: GCS loss alone -> nothing")
    state["params"]["SCR_USER2"] = 1; state.update(disarms=0); cycle(); state["gcs_seen"] = state["t"]; run(95)
    check(state["disarms"] == 1, "T6 policy 1: GCS lost 90 s -> disarm")
    state.update(disarms=0, gcs_seen=0); cycle(); run(200)
    check(state["disarms"] == 0, "T7 GCS never seen -> nothing")
    state["params"]["SCR_USER2"] = 2; state.update(disarms=0, rc_ok=False, gcs_alive=True); cycle()
    state["gcs_seen"] = state["t"]; run(120)
    check(state["disarms"] == 0, "T8a policy 2: RC only -> nothing")
    state["gcs_alive"] = False; run(95)
    check(state["disarms"] == 1, "T8b RC + GCS -> disarm")
    state["params"].update(SCR_USER1=30, SCR_USER2=0, SCR_USER3=1); state.update(disarms=0, rc_ok=False, msgs=[])
    cycle(); run(40)
    check(state["disarms"] == 0 and has("DRY"), "T9 DRY RUN 30 s: message, no disarm")
    state["params"]["SCR_USER3"] = 0; state.update(disarms=0, disarm_ok=False, msgs=[]); run(2)
    check(state["disarms"] >= 2, "T10 disarm returns false -> retries")
    state.update(disarm_ok=True); run(1)
    check(not state["armed"], "T10b stops once disarm succeeds")
    state.update(disarms=0); run(30)
    check(state["disarms"] == 0, "T10c no further calls after disarm")
    # T11: policy 1, MP closed 200 s BEFORE arming -> must not terminate at arm, only at arm + 90 s
    state["params"].update(SCR_USER1=0, SCR_USER2=1, SCR_USER3=0)
    state.update(disarms=0, rc_ok=True, gcs_alive=False, disarm_ok=True)
    state["armed"] = False; state["gcs_seen"] = state["t"]; run(200); state["armed"] = True; run(60)
    check(state["disarms"] == 0, "T11a GCS silence before arming is not counted")
    run(35)
    check(state["disarms"] == 1, "T11b disarm at arm + 95 s")
    # T12: disarmed 5 min -> "TERM: active" at least twice more; arming -> one "armed" message
    state.update(armed=False, msgs=[], rc_ok=True); run(300)
    n = sum(1 for m in state["msgs"] if "TERM: active" in m[2])
    check(n >= 2, "T12a disarmed 5 min: periodic 'active' announcement (%d)" % n)
    state.update(msgs=[]); state["armed"] = True; run(30)
    check(sum(1 for m in state["msgs"] if "armed" in m[2]) == 1 and not has("TERM: active"),
          "T12b one 'armed' message on arming, no 'active' announcement while armed")
    # Every STATUSTEXT must fit the 50-character MAVLink limit.
    long_msgs = [m[2] for m in state["msgs"] if len(m[2]) > 50]
    check(not long_msgs, "T13 all messages <= 50 characters")
    print("RESULT:", "ALL TESTS PASSED" if fails == 0 else f"{fails} FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
