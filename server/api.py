"""
server/api.py
=============
Thin FastAPI server — GVL bridge only.

This server has ZERO logic. It reads GVL singletons and pushes state to the
HMI via WebSocket. HMI writes go to GVL_HMI tags. PLC scan cycle runs in a
background thread.

Endpoints:
    GET  /api/topology             — zone graph for HMI layout (called once at startup)
    GET  /api/state                — full snapshot (zones, alarms, metrics)
    GET  /api/config               — GVL_Config values (all fields)
    POST /api/config               — update GVL_Config + persist to config.json
    POST /api/hmi/{tag}            — write a single HMI tag (pulse or value)
    POST /api/speed                — set simulation speed multiplier
    GET  /api/inventory            — routing table + pool status
    POST /api/inventory/load       — load sample inventory
    GET  /api/reject_pile          — current reject pile contents
    POST /api/sim/remove_tote      — remove tote from system (god-mode)
    POST /api/sim/move_tote        — teleport tote to another zone
    POST /api/sim/stuck_tote       — force stuck/unstuck on a tote
    POST /api/hmi/reinduct         — trigger re-induct from reject pile
    POST /api/hmi/auto_pickup      — toggle auto/manual pickup per spur
    POST /api/hmi/manual_pickup    — force immediate pickup at spur endpoint
    GET  /api/topology/descriptors — current topology as JSON descriptor list
    POST /api/topology/compile     — validate & preview a descriptor list
    POST /api/topology/apply       — compile + restart PLC with new topology
    POST /api/topology/save        — save descriptors to topology.json
    POST /api/topology/load        — load descriptors from topology.json
    GET  /api/topology/templates   — list available template names
    POST /api/topology/from_template — generate descriptors from template params
    WS   /ws                       — real-time state push every 500ms

Loads config.json at startup (missing keys keep defaults).
Serves static HMI files from ../hmi/ directory.
"""

import asyncio
import json
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse, Response
from pydantic import BaseModel

from plc.prg_main import PRG_Main
from plc.gvl.gvl_config import CFG
from plc.gvl.gvl_hmi import HMI
from plc.gvl.gvl_alarms import ALARMS
from plc.gvl.gvl_inventory import INVENTORY
from plc.gvl.gvl_io import IO
from plc.sim.sim_clock import SIM_CLOCK
from plc.types.structs import ST_ZoneDescriptor
from plc.topology.templates import default_sortation
from plc.topology.topology_builder import TopologyBuilder
from plc.topology.serializer import save_topology, load_topology


# ── PLC Runtime ──────────────────────────────────────────────────────────────

plc: PRG_Main = None  # Initialized in lifespan
_plc_thread: threading.Thread = None
_plc_running: bool = False
_speed_multiplier: float = 1.0  # Simulation speed: 0.25, 1, 5, 10, etc.


def _plc_loop() -> None:
    """Background thread: runs PLC scan cycle at configured rate."""
    global _plc_running
    while _plc_running:
        if _speed_multiplier <= 0:
            # Paused — skip ticks entirely, just sleep
            time.sleep(0.05)
            continue

        t_start = time.perf_counter()
        plc.tick()
        elapsed = time.perf_counter() - t_start

        # Sleep remaining time, adjusted by speed multiplier
        target_dt = CFG.rScanCycleRate_s / _speed_multiplier
        sleep_time = max(0.0, target_dt - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)


CONFIG_PATH = str(Path(__file__).resolve().parent.parent / "config.json")
TOPOLOGY_PATH = str(Path(__file__).resolve().parent.parent / "topology.json")
LAYOUT_PATH = str(Path(__file__).resolve().parent.parent / "builder_layout.json")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Start PLC thread on server startup, stop on shutdown."""
    global plc, _plc_thread, _plc_running

    # Load config.json (missing keys keep defaults)
    loaded = CFG.load_from_file(CONFIG_PATH)
    if loaded:
        print(f"[PLC] Loaded config from {CONFIG_PATH}")
    else:
        print(f"[PLC] No config.json found — using defaults")

    # Reset sim clock for fresh start
    SIM_CLOCK.reset()

    plc = PRG_Main()
    INVENTORY.load_sample_inventory()

    _plc_running = True
    _plc_thread = threading.Thread(target=_plc_loop, daemon=True, name="PLC-ScanCycle")
    _plc_thread.start()

    yield

    _plc_running = False
    if _plc_thread:
        _plc_thread.join(timeout=2.0)


# ── FastAPI App ──────────────────────────────────────────────────────────────

HMI_DIR = Path(__file__).resolve().parent.parent / "hmi"

app = FastAPI(
    title="PLC Conveyor Sortation",
    version="1.0.0",
    lifespan=lifespan,
)


# ── REST Endpoints ───────────────────────────────────────────────────────────

@app.get("/api/topology")
def get_topology():
    """Zone graph for HMI layout auto-generation. Called once at startup."""
    return plc.get_topology()


@app.get("/api/state")
def get_state():
    """Full snapshot: zones, alarms, metrics."""
    return {
        "zones": plc.get_zone_snapshot(),
        "alarms": ALARMS.to_dict(),
        "metrics": plc.get_metrics(),
        "system": {
            "eSystemState": plc.eSystemState.name,
            "bRunning": plc.eSystemState.name == "RUNNING",
            "bBlocked": plc.eSystemState.name == "BLOCKED",
            "simTime": SIM_CLOCK.now(),
            "speedMultiplier": _speed_multiplier,
            "iRejectPileCount": INVENTORY.get_reject_count(),
            "iStuckToteCount": plc.sim_stuck.get_stuck_count(),
        },
    }


@app.get("/api/debug/sim")
def get_sim_debug():
    """Debug: raw sim engine state for all tracks."""
    tracks = {}
    for track_name, sim in plc._topology.sim_engines.items():
        tracks[track_name] = sim.to_dict()

    transfer_areas = {}
    for key, tote in plc._processor.transfer_areas.items():
        transfer_areas[key] = tote.sToteID if tote else None

    return {
        "tracks": tracks,
        "transfer_areas": transfer_areas,
        "stuck": plc.sim_stuck.to_dict(),
    }


@app.get("/api/config")
def get_config():
    """Current GVL_Config values."""
    return CFG.to_dict()


class ConfigUpdate(BaseModel):
    updates: dict


@app.post("/api/config")
def update_config(body: ConfigUpdate):
    """Update GVL_Config values. System should be STOPPED. Persists to config.json."""
    CFG.update_from_dict(body.updates)
    CFG.save_to_file(CONFIG_PATH)
    return {"status": "ok", "config": CFG.to_dict()}


@app.get("/favicon.ico")
def favicon():
    """Prevent 404 for browser favicon requests."""
    return Response(status_code=204)


# ── HMI Convenience Endpoints ────────────────────────────────────────────────
# NOTE: These MUST be defined BEFORE the generic /api/hmi/{tag} route so
# FastAPI matches them first (routes are matched in definition order).

@app.post("/api/hmi/reinduct")
def hmi_reinduct():
    """Trigger re-induct of one tote from reject pile."""
    HMI.write("bHMI_ReinductTote", True)
    return {"status": "ok", "rejectCount": INVENTORY.get_reject_count()}


@app.post("/api/hmi/auto_pickup")
def hmi_auto_pickup(body: dict):
    """Toggle auto/manual pickup for a specific spur zone."""
    zone_id = body.get("zone_id", "")
    auto = body.get("auto", True)
    if not zone_id:
        return JSONResponse(status_code=400, content={"error": "zone_id required"})
    HMI.dictAutoPickup[zone_id] = auto
    return {"status": "ok", "zone_id": zone_id, "auto": auto}


@app.post("/api/hmi/manual_pickup")
def hmi_manual_pickup(body: dict):
    """Force immediate pickup at a spur endpoint."""
    zone_id = body.get("zone_id", "")
    if not zone_id:
        return JSONResponse(status_code=400, content={"error": "zone_id required"})
    HMI.sHMI_ManualPickup = zone_id
    return {"status": "ok", "zone_id": zone_id}


@app.post("/api/hmi/{tag}")
def write_hmi_tag(tag: str, body: dict = None):
    """
    Write a single HMI tag. For pulse commands (bHMI_Start, etc.)
    send {"value": true}. For speed override send {"zone": "C1", "value": 75.0}.
    """
    if body is None:
        body = {}
    value = body.get("value", True)

    if tag == "rHMI_SpeedOverride":
        zone = body.get("zone", "")
        speed = body.get("value", CFG.rDefaultBeltSpeed)
        HMI.rHMI_SpeedOverride[zone] = speed
        return {"status": "ok", "tag": tag, "zone": zone, "value": speed}

    if tag == "eMergePriorityOverride":
        from plc.types.enums import E_MergePriority
        HMI.eMergePriorityOverride = E_MergePriority[value]
        return {"status": "ok", "tag": tag, "value": value}

    try:
        HMI.write(tag, value)
    except KeyError:
        return JSONResponse(status_code=400, content={"error": f"Unknown HMI tag: {tag}"})
    return {"status": "ok", "tag": tag, "value": value}


@app.post("/api/speed")
def set_speed(body: dict):
    """Set simulation speed multiplier. 0 = paused, 1 = real-time, 10 = 10x."""
    global _speed_multiplier
    _speed_multiplier = max(0.0, float(body.get("value", 1.0)))
    return {"status": "ok", "speedMultiplier": _speed_multiplier}


@app.get("/api/inventory")
def get_inventory():
    """Routing table and pool status."""
    return INVENTORY.to_dict()


@app.post("/api/inventory/load")
def load_inventory():
    """Load sample inventory data."""
    INVENTORY.load_sample_inventory()
    return {"status": "ok", "inventory": INVENTORY.to_dict()}


# ── Reject Pile ──────────────────────────────────────────────────────────────

@app.get("/api/reject_pile")
def get_reject_pile():
    """Current reject pile contents."""
    return {
        "count": INVENTORY.get_reject_count(),
        "pile": INVENTORY.reject_pile,
    }


# ── Simulator God-Mode ───────────────────────────────────────────────────────


def _find_tote_sim(tote_id: str):
    """Find which SimEngine holds a tote. Returns (sim, zone_id) or (None, None).

    Checks sim engines first (physics layer), then falls back to zone FBs
    (logic layer) for totes in transit between engines during handoffs.
    """
    all_sims = [plc.sim_main, plc.sim_recirc, plc.sim_reinduct, plc.sim_reject]
    all_sims.extend(plc.sim_spurs.values())

    # Primary: check sim engine tote tracking
    for sim in all_sims:
        if sim.get_tote_position(tote_id) is not None:
            # Find zone_id from zone_tote_map
            for zid, tid in sim.zone_tote_map.items():
                if tid == tote_id:
                    return sim, zid
            # Tote exists in engine but not at a zone center — still return engine
            return sim, ""

    return None, None


def _find_tote_zone_fb(tote_id: str):
    """Find which zone FB holds a tote (fallback for handoff windows)."""
    for zid, fb in plc.zone_fbs.items():
        st = fb._stCurrentTote
        if st and st.sToteID == tote_id:
            return zid
    return ""


def _find_zone_sim(zone_id: str):
    """Find which SimEngine owns a zone."""
    all_sims = [plc.sim_main, plc.sim_recirc, plc.sim_reinduct, plc.sim_reject]
    all_sims.extend(plc.sim_spurs.values())
    for sim in all_sims:
        if zone_id in sim._zone_map:
            return sim
    return None


@app.post("/api/sim/remove_tote")
def sim_remove_tote(body: dict):
    """Remove a tote from the system entirely. Adds to reject pile."""
    tote_id = body.get("tote_id", "")
    if not tote_id:
        return JSONResponse(status_code=400, content={"error": "tote_id required"})

    # Find and remove from whichever sim engine holds it
    sim, _ = _find_tote_sim(tote_id)
    if sim:
        sim.remove_tote(tote_id)

    # Get barcode before removing from tracker
    barcode = plc.fbTracker.get_barcode_for_tote(tote_id)

    # Clear from ALL zone FBs (not just first match — handoff can leave stale refs)
    from plc.types.enums import E_ConveyorState
    for fb in plc.zone_fbs.values():
        st = fb._stCurrentTote
        if st and st.sToteID == tote_id:
            fb._stCurrentTote = None
            fb.bTotePresent = False
            fb.eState = E_ConveyorState.IDLE

    # If not found in sim but was in zone FBs, still counts as removed
    if not sim and not _find_tote_zone_fb(tote_id):
        return JSONResponse(status_code=404, content={"error": f"Tote {tote_id} not found in any sim"})

    # Remove from tracker → reject pile
    if barcode:
        INVENTORY.reject_tote(tote_id, barcode)
    plc.fbTracker._remove_tote(tote_id)

    # Clear stuck state
    plc.sim_stuck.clear_tote(tote_id)

    return {"status": "ok", "tote_id": tote_id, "removed": True}


@app.post("/api/sim/move_tote")
def sim_move_tote(body: dict):
    """Teleport a tote to a different zone. Zone must be empty."""
    tote_id = body.get("tote_id", "")
    target_zone = body.get("target_zone", "")
    if not tote_id or not target_zone:
        return JSONResponse(status_code=400, content={"error": "tote_id and target_zone required"})

    # Find source sim engine
    source_sim, _ = _find_tote_sim(tote_id)
    if source_sim is None:
        return JSONResponse(status_code=404, content={"error": f"Tote {tote_id} not found"})

    # Find target sim engine
    target_sim = _find_zone_sim(target_zone)
    if target_sim is None:
        return JSONResponse(status_code=400, content={"error": f"Zone {target_zone} not found"})

    if target_sim.is_zone_occupied(target_zone):
        return JSONResponse(status_code=409, content={"error": f"Zone {target_zone} is occupied"})

    # Remove from source, spawn in target
    source_sim.remove_tote(tote_id)
    target_sim.spawn_tote(tote_id, target_zone)

    # Clear old zone FB state and let the new zone acquire on next tick
    from plc.types.enums import E_ConveyorState
    for fb in plc.zone_fbs.values():
        st = fb._stCurrentTote
        if st and st.sToteID == tote_id:
            fb._stCurrentTote = None
            fb.bTotePresent = False
            fb.eState = E_ConveyorState.IDLE

    return {"status": "ok", "tote_id": tote_id, "target_zone": target_zone}


@app.post("/api/sim/stuck_tote")
def sim_stuck_tote(body: dict):
    """Force a tote to be stuck or unstuck."""
    tote_id = body.get("tote_id", "")
    stuck = body.get("stuck", True)
    if not tote_id:
        return JSONResponse(status_code=400, content={"error": "tote_id required"})

    if stuck:
        # Find zone via sim engine or zone FB fallback
        _, zone_id = _find_tote_sim(tote_id)
        if not zone_id:
            zone_id = _find_tote_zone_fb(tote_id)
        if not zone_id:
            return JSONResponse(status_code=404, content={"error": f"Tote {tote_id} not found"})

        plc.sim_stuck.force_stuck(tote_id, zone_id)
    else:
        plc.sim_stuck.clear_tote(tote_id)

    return {"status": "ok", "tote_id": tote_id, "stuck": stuck}


# ── Topology Builder API ─────────────────────────────────────────────────────

def _stop_plc() -> None:
    """Stop the PLC scan cycle thread."""
    global _plc_running, _plc_thread
    _plc_running = False
    if _plc_thread:
        _plc_thread.join(timeout=2.0)
        _plc_thread = None


def _start_plc() -> None:
    """Start the PLC scan cycle thread."""
    global _plc_running, _plc_thread
    _plc_running = True
    _plc_thread = threading.Thread(target=_plc_loop, daemon=True, name="PLC-ScanCycle")
    _plc_thread.start()


def _restart_plc_with_topology(descriptors: List[ST_ZoneDescriptor]) -> None:
    """Stop PLC → create new PRG_Main with given topology → restart PLC."""
    global plc
    _stop_plc()
    SIM_CLOCK.reset()
    ALARMS.acknowledge_all()
    ALARMS.clear_resolved()
    plc = PRG_Main(topology_descriptors=descriptors)
    INVENTORY.load_sample_inventory()
    _start_plc()


@app.get("/api/topology/descriptors")
def get_topology_descriptors():
    """Current topology as JSON descriptor list."""
    descriptors = list(plc._topology.descriptors.values())
    return [d.to_dict() for d in descriptors]


@app.post("/api/topology/compile")
def compile_topology(body: dict):
    """Validate & preview a descriptor list (no apply).

    Returns compiled graph metadata (zone count, tracks, errors) without
    restarting the PLC. Use this to preview before applying.
    """
    try:
        raw_descriptors = body.get("descriptors", [])
        descriptors = [ST_ZoneDescriptor.from_dict(d) for d in raw_descriptors]

        if not descriptors:
            return JSONResponse(status_code=400, content={"error": "No descriptors provided"})

        graph = TopologyBuilder.compile(descriptors)

        return {
            "status": "ok",
            "zone_count": len(graph.zone_fbs),
            "tracks": list(graph.track_zone_order.keys()),
            "track_process_order": graph.track_process_order,
            "track_zone_order": graph.track_zone_order,
            "spawn_zones": graph.spawn_zones,
            "pickup_zones": graph.pickup_zones,
            "scanner_zones": graph.scanner_zones,
            "force_release_zones": graph.force_release_zones,
            "cross_transfers": [
                {"source": ct.sSourceZone, "target": ct.sTargetZone,
                 "source_track": ct.sSourceTrack, "target_track": ct.sTargetTrack}
                for ct in graph.cross_transfers
            ],
        }
    except Exception as e:
        return JSONResponse(status_code=400, content={"error": str(e)})


@app.post("/api/topology/apply")
def apply_topology(body: dict):
    """Compile + restart PLC with new topology.

    Stops the PLC thread, creates a new PRG_Main with the given descriptors,
    and restarts. All state is reset (fresh start with new layout).
    """
    try:
        raw_descriptors = body.get("descriptors", [])
        descriptors = [ST_ZoneDescriptor.from_dict(d) for d in raw_descriptors]

        if not descriptors:
            return JSONResponse(status_code=400, content={"error": "No descriptors provided"})

        # Validate first (compile without applying)
        TopologyBuilder.compile(descriptors)

        # Apply: stop → recreate → restart
        _restart_plc_with_topology(descriptors)

        return {
            "status": "ok",
            "zone_count": len(plc.zone_fbs),
            "tracks": list(plc._topology.track_zone_order.keys()),
        }
    except Exception as e:
        return JSONResponse(status_code=400, content={"error": str(e)})


@app.post("/api/topology/save")
def save_topology_file(body: dict = None):
    """Save current (or provided) descriptors to topology.json."""
    try:
        if body and body.get("descriptors"):
            descriptors = [ST_ZoneDescriptor.from_dict(d) for d in body["descriptors"]]
        else:
            descriptors = list(plc._topology.descriptors.values())

        save_topology(descriptors, TOPOLOGY_PATH)
        return {"status": "ok", "path": TOPOLOGY_PATH, "zone_count": len(descriptors)}
    except Exception as e:
        return JSONResponse(status_code=400, content={"error": str(e)})


@app.post("/api/topology/load")
def load_topology_file():
    """Load descriptors from topology.json and apply them."""
    try:
        descriptors = load_topology(TOPOLOGY_PATH)

        # Validate first
        TopologyBuilder.compile(descriptors)

        # Apply
        _restart_plc_with_topology(descriptors)

        return {
            "status": "ok",
            "zone_count": len(plc.zone_fbs),
            "tracks": list(plc._topology.track_zone_order.keys()),
            "descriptors": [d.to_dict() for d in descriptors],
        }
    except FileNotFoundError:
        return JSONResponse(status_code=404, content={"error": f"Topology file not found: {TOPOLOGY_PATH}"})
    except Exception as e:
        return JSONResponse(status_code=400, content={"error": str(e)})


@app.post("/api/builder/layout/save")
def save_builder_layout(body: dict = None):
    """Save the builder grid layout (cell positions, directions, visual arrangement)."""
    try:
        if not body or not body.get("cells"):
            return JSONResponse(status_code=400, content={"error": "No cells data provided"})
        p = Path(LAYOUT_PATH)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(body["cells"], indent=2) + "\n", encoding="utf-8")
        return {"status": "ok", "cell_count": len(body["cells"])}
    except Exception as e:
        return JSONResponse(status_code=400, content={"error": str(e)})


@app.get("/api/builder/layout/load")
def load_builder_layout():
    """Load the builder grid layout."""
    try:
        p = Path(LAYOUT_PATH)
        if not p.exists():
            return {"cells": None}
        data = json.loads(p.read_text(encoding="utf-8"))
        return {"cells": data}
    except Exception as e:
        return JSONResponse(status_code=400, content={"error": str(e)})


@app.get("/api/topology/templates")
def list_topology_templates():
    """List available topology template names with their parameters."""
    return {
        "templates": [
            {
                "name": "default_sortation",
                "description": "Standard sortation system with chutes, reject, and recirc",
                "params": {
                    "num_chutes": {"type": "int", "default": CFG.iNumberOfChutes, "description": "Number of sort chutes"},
                    "spur_length": {"type": "int", "default": CFG.iSpurLength, "description": "Zones per spur"},
                    "reject_length": {"type": "int", "default": CFG.iRejectSpurLength, "description": "Zones in reject spur"},
                },
            },
        ],
    }


@app.post("/api/topology/from_template")
def topology_from_template(body: dict):
    """Generate descriptors from a template name + params. Does NOT apply."""
    template_name = body.get("template", "")
    params = body.get("params", {})

    if template_name == "default_sortation":
        descriptors = default_sortation(
            num_chutes=params.get("num_chutes"),
            spur_length=params.get("spur_length"),
            reject_length=params.get("reject_length"),
        )
        return {
            "status": "ok",
            "template": template_name,
            "zone_count": len(descriptors),
            "descriptors": [d.to_dict() for d in descriptors],
        }

    return JSONResponse(status_code=400, content={"error": f"Unknown template: {template_name}"})


# ── WebSocket ────────────────────────────────────────────────────────────────

_ws_clients: List[WebSocket] = []


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    _ws_clients.append(ws)
    try:
        while True:
            # Push state every 500ms
            state = {
                "zones": plc.get_zone_snapshot(),
                "alarms": ALARMS.to_dict(),
                "metrics": plc.get_metrics(),
                "system": {
                    "eSystemState": plc.eSystemState.name,
                    "bRunning": plc.eSystemState.name == "RUNNING",
                    "bBlocked": plc.eSystemState.name == "BLOCKED",
                    "simTime": round(SIM_CLOCK.now(), 2),
                    "speedMultiplier": _speed_multiplier,
                    "iRejectPileCount": INVENTORY.get_reject_count(),
                    "iStuckToteCount": plc.sim_stuck.get_stuck_count(),
                },
            }
            await ws.send_json(state)
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        if ws in _ws_clients:
            _ws_clients.remove(ws)


# ── Static files (HMI) ──────────────────────────────────────────────────────

@app.get("/")
def serve_index():
    """Serve the HMI index.html."""
    index_path = HMI_DIR / "index.html"
    if index_path.exists():
        return FileResponse(index_path)
    return JSONResponse({"error": "HMI not found. Place index.html in hmi/ directory."}, 404)


# Mount static files AFTER explicit routes so /api/* and /ws take priority
if HMI_DIR.exists():
    app.mount("/", StaticFiles(directory=str(HMI_DIR), html=True), name="hmi")
