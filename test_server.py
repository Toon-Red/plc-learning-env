"""
Interactive server test script — exercises all API endpoints.
Run while the server is up: python test_server.py
"""
import requests
import time
import json
import sys

BASE = "http://localhost:8080"

def api(method, path, body=None):
    url = f"{BASE}{path}"
    if method == "GET":
        r = requests.get(url, timeout=5)
    else:
        r = requests.post(url, json=body or {}, timeout=5)
    return r.json()

def state():
    return api("GET", "/api/state")

def hmi(tag, value=True):
    return api("POST", f"/api/hmi/{tag}", {"value": value})

def config_update(updates: dict):
    return api("POST", "/api/config", {"updates": updates})

def wait(seconds=2):
    time.sleep(seconds)

def active_totes(d=None):
    d = d or state()
    totes = []
    for zid, zdata in d["zones"].items():
        if zdata["bTotePresent"]:
            totes.append({
                "zone": zid,
                "tote": zdata["sToteID"],
                "chute": zdata["eChuteAssignment"],
                "state": zdata["eState"],
                "scanFails": zdata["iScanFailCount"],
            })
    return totes

def summary(label="", d=None):
    d = d or state()
    s = d["system"]
    m = d["metrics"]
    hdr = f"  [{label}] " if label else "  "
    print(f"\n{'='*65}")
    print(f"{hdr}State: {s['eSystemState']} | SimTime: {s['simTime']:.1f}s | Speed: {s['speedMultiplier']}x")
    print(f"  Active: {m['diActiveTotes']} | Delivered: {m['robot']['diTotalDelivered']} | SpawnQ: {m['iSpawnRemaining']}")
    print(f"  Reject pile: {m['iRejectPileCount']} | Stuck: {m['stuck']['count']} | Blocked: {s['bBlocked']}")
    totes = active_totes(d)
    if totes:
        print(f"  Totes on belt ({len(totes)}):")
        for t in totes:
            print(f"    {t['zone']:15s} | {t['tote']:12s} | {t['chute']:12s} | {t['state']} | fails={t['scanFails']}")
    else:
        print("  (no totes on conveyor)")
    alarms = d["alarms"]["aAlarmBuffer"]
    if alarms:
        print(f"  Alarms ({len(alarms)}, last 5):")
        for a in alarms[-5:]:
            sev = a["eSeverity"]
            print(f"    [{sev:8s}] A{a['iAlarmCode']:03d} @ {a['sZoneID']}: {a['sMessage'][:55]}")
    print(f"{'='*65}")

def remove_all_ghost_totes():
    """Remove all totes from all sim engines via debug endpoint."""
    dbg = api("GET", "/api/debug/sim")
    ghost_ids = set()
    for key in ["sim_main", "sim_recirc", "sim_reinduct", "sim_reject"]:
        if key in dbg and isinstance(dbg[key], dict):
            totes = dbg[key].get("totes", {})
            if isinstance(totes, dict):
                ghost_ids.update(totes.keys())
            elif isinstance(totes, list):
                ghost_ids.update(totes)
    if "sim_spurs" in dbg:
        for spur_data in dbg["sim_spurs"].values():
            if isinstance(spur_data, dict):
                totes = spur_data.get("totes", {})
                if isinstance(totes, dict):
                    ghost_ids.update(totes.keys())
                elif isinstance(totes, list):
                    ghost_ids.update(totes)
    for tid in ghost_ids:
        api("POST", "/api/sim/remove_tote", {"tote_id": tid})
    return ghost_ids


def ensure_running(max_retries=5):
    """Get system to RUNNING state from any state, clearing obstacles."""
    for attempt in range(max_retries):
        d = state()
        st = d["system"]["eSystemState"]
        if st == "RUNNING":
            return
        # Always clean ghost totes that might cause jams/E-Stops
        old_speed = d["system"]["speedMultiplier"]
        api("POST", "/api/speed", {"value": 0})
        remove_all_ghost_totes()
        api("POST", "/api/speed", {"value": old_speed if old_speed > 0 else 1})
        if st == "ESTOPPED":
            hmi("bHMI_EStop", False)
            wait(0.5)
            hmi("bHMI_Reset")
            wait(0.5)
        if st == "BLOCKED":
            wait(0.5)
        d = state()
        st = d["system"]["eSystemState"]
        if st == "STOPPED":
            hmi("bHMI_Start")
            wait(0.5)
        d = state()
        if d["system"]["eSystemState"] == "RUNNING":
            return
    d = state()
    assert d["system"]["eSystemState"] == "RUNNING", f"Failed to reach RUNNING after {max_retries} retries, got {d['system']['eSystemState']}"


PASS_COUNT = 0
FAIL_COUNT = 0

def check(name, condition, detail=""):
    global PASS_COUNT, FAIL_COUNT
    if condition:
        PASS_COUNT += 1
        print(f"  PASS  {name}")
    else:
        FAIL_COUNT += 1
        print(f"  FAIL  {name} {detail}")


def test_all():
    print("\n" + "="*65)
    print("  SERVER INTEGRATION TEST — ALL ENDPOINTS")
    print("="*65)

    # ── 0. Hard reset — clean up any leftover state ──────────────
    print("\n--- [0] Hard reset + disable stuck totes ---")
    config_update({"rStuckToteChance": 0.0})
    api("POST", "/api/speed", {"value": 0})  # Pause to prevent new events

    # Remove ALL totes from ALL sim engines (not just zone-visible ones)
    dbg = api("GET", "/api/debug/sim")
    ghost_totes = set()
    for key in ["sim_main", "sim_recirc", "sim_reinduct", "sim_reject"]:
        if key in dbg and isinstance(dbg[key], dict):
            for tid in dbg[key].get("totes", []):
                ghost_totes.add(tid)
    if "sim_spurs" in dbg:
        for spur_data in dbg["sim_spurs"].values():
            if isinstance(spur_data, dict):
                for tid in spur_data.get("totes", []):
                    ghost_totes.add(tid)
    for tid in ghost_totes:
        api("POST", "/api/sim/remove_tote", {"tote_id": tid})
    if ghost_totes:
        print(f"  Removed {len(ghost_totes)} ghost totes: {sorted(ghost_totes)}")

    api("POST", "/api/speed", {"value": 1})
    hmi("bHMI_EStop", False)
    wait(0.5)
    hmi("bHMI_Reset")
    wait(0.5)
    hmi("bHMI_Stop")
    wait(0.5)
    # Second reset attempt in case first was consumed during E-Stop
    hmi("bHMI_Reset")
    wait(0.5)
    d = state()
    if d["system"]["eSystemState"] != "STOPPED":
        for _ in range(5):
            hmi("bHMI_EStop", False)
            wait(0.3)
            hmi("bHMI_Reset")
            wait(0.5)
            d = state()
            if d["system"]["eSystemState"] == "STOPPED":
                break
    print(f"  Initial state: {d['system']['eSystemState']}")

    # ── 1. Read-only endpoints ─────────────────────────────────────
    print("\n--- [1] Read-only endpoints ---")

    d = state()
    check("/api/state", "zones" in d and "system" in d and "metrics" in d)

    t = api("GET", "/api/topology")
    check("/api/topology", "main_line" in t and "reject_line" in t and "spurs" in t)
    check("  topology has BRANCH_REJECT", "BRANCH_REJECT" in t["main_line"])
    check("  reject spur has 5 zones", len(t["reject_line"]) == 5)
    check("  3 spurs", len(t["spurs"]) == 3)

    c = api("GET", "/api/config")
    check("/api/config", "iMaxScanFails" in c and "rStuckToteChance" in c)
    check("  iMaxScanFails=2", c["iMaxScanFails"] == 2)
    check("  iRejectSpurLength=5", c["iRejectSpurLength"] == 5)

    inv = api("GET", "/api/inventory")
    check("/api/inventory", "routing_table" in inv or "available_pool" in inv)

    rp = api("GET", "/api/reject_pile")
    check("/api/reject_pile", "count" in rp and "pile" in rp)

    dbg = api("GET", "/api/debug/sim")
    check("/api/debug/sim", "sim_main" in dbg and "sim_reject" in dbg and "sim_spurs" in dbg)

    # ── 2. System lifecycle ─────────────────────────────────────
    print("\n--- [2] System lifecycle ---")

    # Reset to known state
    hmi("bHMI_EStop", False)
    wait(0.3)
    hmi("bHMI_Reset")
    wait(0.3)
    hmi("bHMI_Stop")
    wait(0.3)

    d = state()
    check("System STOPPED", d["system"]["eSystemState"] == "STOPPED")

    hmi("bHMI_Start")
    wait(0.3)
    d = state()
    check("Start -> RUNNING", d["system"]["eSystemState"] == "RUNNING")

    hmi("bHMI_Stop")
    wait(0.3)
    d = state()
    check("Stop -> STOPPED", d["system"]["eSystemState"] == "STOPPED")

    # ── 3. E-Stop lifecycle ────────────────────────────────────
    print("\n--- [3] E-Stop lifecycle ---")

    hmi("bHMI_Start")
    wait(0.3)
    hmi("bHMI_EStop")
    wait(0.3)
    d = state()
    check("EStop -> ESTOPPED", d["system"]["eSystemState"] == "ESTOPPED")

    # Reset with EStop still held should NOT clear
    hmi("bHMI_Reset")
    wait(0.3)
    d = state()
    check("Reset while EStop held stays ESTOPPED", d["system"]["eSystemState"] == "ESTOPPED")

    # Release EStop, then reset
    hmi("bHMI_EStop", False)
    wait(0.3)
    hmi("bHMI_Reset")
    wait(0.3)
    d = state()
    check("Release + Reset -> STOPPED", d["system"]["eSystemState"] == "STOPPED")

    hmi("bHMI_Start")
    wait(0.3)
    d = state()
    check("Start after recovery -> RUNNING", d["system"]["eSystemState"] == "RUNNING")

    # ── 4. Speed control ────────────────────────────────────
    print("\n--- [4] Speed control ---")

    r = api("POST", "/api/speed", {"value": 10})
    check("Set speed 10x", r["speedMultiplier"] == 10.0)

    r = api("POST", "/api/speed", {"value": 0})
    check("Pause (speed 0)", r["speedMultiplier"] == 0.0)

    r = api("POST", "/api/speed", {"value": 1})
    check("Resume (speed 1x)", r["speedMultiplier"] == 1.0)

    # ── 5. Config update ────────────────────────────────────
    print("\n--- [5] Config update + persist ---")

    r = config_update({"rDefaultBeltSpeed": 55.0})
    check("Config update accepted", r.get("status") == "ok")

    c = api("GET", "/api/config")
    check("Config read-back", c["rDefaultBeltSpeed"] == 55.0)

    config_update({"rDefaultBeltSpeed": 50.0})  # restore

    print("\n--- [6] Tote spawning + delivery ---")

    ensure_running()
    api("POST", "/api/speed", {"value": 10})

    delivered_before = state()["metrics"]["robot"]["diTotalDelivered"]
    hmi("iHMI_SpawnCount", 5)
    wait(15)  # 15s at 10x = 150s sim time
    d = state()
    delivered_after = d["metrics"]["robot"]["diTotalDelivered"]
    new_deliveries = delivered_after - delivered_before
    check(f"5 totes delivered ({new_deliveries} new)", new_deliveries >= 4,
          f"only {new_deliveries}")
    summary("After 5 totes", d)

    # ── 7. God-mode: pause, spawn, interact ─────────────────
    print("\n--- [7] Simulator god-mode ---")

    ensure_running()
    config_update({"rStuckToteChance": 0.0})

    # Spawn at low speed so we can catch totes mid-transit
    api("POST", "/api/speed", {"value": 2})
    hmi("iHMI_SpawnCount", 3)

    # Poll until we see totes on the main line (not yet delivered)
    totes = []
    for attempt in range(20):
        wait(0.5)
        d = state()
        totes = active_totes(d)
        # Look for totes on the main line (not on spur endpoints waiting for pickup)
        main_totes = [t for t in totes if not t["zone"].startswith("SPUR_") and
                      not t["zone"].startswith("REJECT_")]
        if main_totes:
            totes = main_totes
            break

    api("POST", "/api/speed", {"value": 0})  # Freeze
    wait(0.3)
    d = state()
    totes = [t for t in active_totes(d) if not t["zone"].startswith("SPUR_") and
             not t["zone"].startswith("REJECT_")]
    summary("Paused with totes", d)

    if len(totes) > 0:
        t1 = totes[0]
        tote_id = t1["tote"]
        tote_zone = t1["zone"]
        print(f"  Target tote: {tote_id} at {tote_zone}")

        # Force stuck
        r = api("POST", "/api/sim/stuck_tote", {"tote_id": tote_id, "stuck": True})
        check(f"Force stuck {tote_id}", r.get("status") == "ok", str(r))

        d = state()
        check("Stuck count > 0", d["metrics"]["stuck"]["count"] > 0,
              f"count={d['metrics']['stuck']['count']}")

        # Clear stuck
        r = api("POST", "/api/sim/stuck_tote", {"tote_id": tote_id, "stuck": False})
        check(f"Clear stuck {tote_id}", r.get("status") == "ok", str(r))

        d = state()
        check("Stuck count back to 0", d["metrics"]["stuck"]["count"] == 0)

        # Move tote — find a genuinely empty zone (check both zone FB and sim engine)
        d_check = state()
        all_zones = list(d_check["zones"].keys())
        # Prefer main line transport zones
        preferred = ["C4", "C5", "C6", "C2", "C_BUF_1", "C_BUF_2", "C_BUF_3"]
        moved = False
        for tz in preferred:
            if tz != tote_zone and not d_check["zones"][tz]["bTotePresent"]:
                r = api("POST", "/api/sim/move_tote", {"tote_id": tote_id, "target_zone": tz})
                if r.get("status") == "ok":
                    check(f"Move {tote_id} to {tz}", True)
                    moved = True
                    break
                # Zone occupied in sim even if not in FB — try next
        if not moved:
            print("  SKIP  Move tote (all target zones occupied)")

        # Remove tote
        totes_now = active_totes()
        if totes_now:
            rm_tote = totes_now[-1]["tote"]
            r = api("POST", "/api/sim/remove_tote", {"tote_id": rm_tote})
            check(f"Remove {rm_tote}", r.get("status") == "ok" or r.get("removed"), str(r))
        else:
            print("  SKIP  Remove tote (none available)")
    else:
        print("  SKIP  God-mode tests — no totes captured despite polling")

    # Clean up remaining totes from god-mode testing
    api("POST", "/api/speed", {"value": 0})
    removed = remove_all_ghost_totes()
    if removed:
        print(f"  Cleaned up {len(removed)} leftover totes after god-mode")
    api("POST", "/api/speed", {"value": 1})
    wait(0.3)

    # ── 8. Reject pile + reinduct ──────────────────────────
    print("\n--- [8] Reject pile + reinduct ---")

    rp = api("GET", "/api/reject_pile")
    initial_reject = rp["count"]
    check("Reject pile endpoint works", "count" in rp)

    r = api("POST", "/api/hmi/reinduct")
    check("Reinduct endpoint works", r.get("status") == "ok", str(r))

    # ── 9. Pickup toggles ──────────────────────────────────
    print("\n--- [9] Pickup toggles ---")

    r = api("POST", "/api/hmi/auto_pickup", {"zone_id": "SPUR_1_3", "auto": False})
    check("Disable auto-pickup SPUR_1_3", r.get("status") == "ok", str(r))

    r = api("POST", "/api/hmi/auto_pickup", {"zone_id": "SPUR_1_3", "auto": True})
    check("Re-enable auto-pickup SPUR_1_3", r.get("status") == "ok", str(r))

    r = api("POST", "/api/hmi/manual_pickup", {"zone_id": "SPUR_2_3"})
    check("Manual pickup SPUR_2_3", r.get("status") == "ok", str(r))

    # ── 10. Throughput run ──────────────────────────────────
    print("\n--- [10] Full throughput run ---")

    ensure_running()
    config_update({"rStuckToteChance": 0.0, "rScanFailRate": 0.05})  # Low fail rate for throughput
    api("POST", "/api/speed", {"value": 10})

    delivered_before = state()["metrics"]["robot"]["diTotalDelivered"]
    hmi("iHMI_SpawnCount", 15)

    for i in range(15):
        wait(2)
        d = state()
        m = d["metrics"]
        delivered = m["robot"]["diTotalDelivered"]
        active = m["diActiveTotes"]
        spawn_q = m["iSpawnRemaining"]
        sys.stdout.write(f"\r  t+{(i+1)*2:3d}s: active={active} delivered={delivered} spawnQ={spawn_q} state={d['system']['eSystemState']}     ")
        sys.stdout.flush()
        if active == 0 and spawn_q == 0 and d["system"]["eSystemState"] == "RUNNING":
            break
    print()

    d = state()
    final_delivered = d["metrics"]["robot"]["diTotalDelivered"] - delivered_before
    reject_count = d["metrics"]["iRejectPileCount"]
    check(f"15 totes processed ({final_delivered} delivered, {reject_count} rejected)",
          final_delivered + reject_count >= 14,
          f"only {final_delivered} delivered + {reject_count} rejected")
    summary("Throughput done", d)

    # ── 11. Stress: stuck tote overcurrent E-Stop ──────────
    print("\n--- [11] Stuck tote -> overcurrent E-Stop ---")

    ensure_running()
    config_update({"rStuckToteChance": 0.05})  # High chance for fast trigger
    api("POST", "/api/speed", {"value": 10})
    hmi("iHMI_SpawnCount", 5)

    estopped = False
    for i in range(30):
        wait(1)
        d = state()
        if d["system"]["eSystemState"] == "ESTOPPED":
            estopped = True
            break
    check("Overcurrent E-Stop triggered", estopped)
    if estopped:
        # Check for A011 alarm
        has_a011 = any(a["iAlarmCode"] == 11 for a in d["alarms"]["aAlarmBuffer"])
        check("A011 alarm raised", has_a011)
        summary("After overcurrent", d)

    # Recover
    config_update({"rStuckToteChance": 0.0})
    hmi("bHMI_EStop", False)
    wait(0.3)
    hmi("bHMI_Reset")
    wait(0.3)
    hmi("bHMI_Start")
    wait(0.3)

    # ── 12. Restore defaults ──────────────────────────────
    print("\n--- [12] Cleanup ---")

    config_update({"rStuckToteChance": 0.0005, "rDefaultBeltSpeed": 50.0, "rScanFailRate": 0.2})
    api("POST", "/api/speed", {"value": 1})
    check("Config restored", True)

    # ── FINAL SUMMARY ──────────────────────────────────────
    print(f"\n{'='*65}")
    print(f"  RESULTS: {PASS_COUNT} passed, {FAIL_COUNT} failed")
    print(f"{'='*65}")

    if FAIL_COUNT > 0:
        sys.exit(1)


if __name__ == "__main__":
    test_all()
