"""
prg_main.py
============
IEC 61131-3 equivalent: PROGRAM PRG_Main

Top-level program that wires the entire sortation system.
Uses topology builder to create FBs and SimEngines from zone descriptors.
TopologyProcessor handles generic zone processing for any layout.

Topology is defined by a list of ST_ZoneDescriptor (from templates, JSON, or HMI editor).
Default: default_sortation() template matching the original hardcoded layout.

Scan cycle order (every tick):
    0. Advance simulation clock
    1. Read HMI commands (spawn queue, start/stop/reset, reinduct)
    2. E-Stop processing (manual + software + overcurrent)
    2b. BLOCKED auto-resume check
    3. SimEngine tick — all tracks (physics first)
    3b. Stuck tote simulation
    4. Spawn logic (infeed from HMI count input)
    4b. Re-induct logic (reject pile → REINDUCT)
    5. Process ALL zones via TopologyProcessor (generic, graph-driven)
    6. Scanner + ToteTracker + scan fail handling
    7. Recirc count E-Stop check (after merge acceptance)
    8. Robot pickup timers (all pickup zones)
    9. BLOCKED state check (reject spur full)
   10. System-level jam detection (RELEASING state timeout per zone)

System-level E-Stop conditions (any one triggers ALL conveyors to stop):
  1. Manual E-Stop button (HMI.bHMI_EStop)
  2. Recirc limit: tote recirculates >= CFG.iMaxRecircCount times → system E-Stop
  3. Physical jam: zone in RELEASING state for > CFG.rJamTimeout_s → system E-Stop
  4. Motor overcurrent: stuck tote causes motor spike → system E-Stop
  Manual RESET required after any E-Stop. Reset → STOPPED → Start → RUNNING.

BLOCKED state:
  RUNNING → (reject spur full) → BLOCKED → (reject cleared) → RUNNING
  All conveyors pause. No manual reset needed — auto-resumes when reject spur clears.

State machine:
  STOPPED → (Start) → RUNNING → (Stop) → STOPPED
  RUNNING → (reject full) → BLOCKED → (reject cleared) → RUNNING
  Any state → (E-Stop) → ESTOPPED → (Reset, E-Stop cleared) → STOPPED → (Start) → RUNNING
"""

from typing import List, Dict, Optional, Tuple

from plc.gvl.gvl_config import CFG
from plc.gvl.gvl_hmi import HMI
from plc.gvl.gvl_alarms import ALARMS
from plc.gvl.gvl_inventory import INVENTORY
from plc.gvl.gvl_io import IO
from plc.types.enums import (
    E_ConveyorState, E_ChuteTarget, E_SystemState, E_MergePriority,
)
from plc.types.structs import ST_ToteData, ST_ZoneDescriptor

from plc.function_blocks.fb_estop import FB_EStop
from plc.function_blocks.fb_conveyor_zone import FB_ConveyorZone
from plc.function_blocks.fb_branch_conveyor import FB_BranchConveyor
from plc.function_blocks.fb_merge_conveyor import FB_MergeConveyor
from plc.function_blocks.fb_barcode_scanner import FB_BarcodeScanner
from plc.function_blocks.fb_tote_tracker import FB_ToteTracker

from plc.sim.sim_engine import SimEngine
from plc.sim.sim_robot_pickup import SimRobotPickup
from plc.sim.sim_stuck_tote import SimStuckTote, E_StuckType
from plc.sim.sim_clock import SIM_CLOCK

from plc.topology.templates import default_sortation
from plc.topology.topology_builder import TopologyBuilder, ST_TopologyGraph
from plc.topology.topology_processor import TopologyProcessor


class PRG_Main:
    """
    Top-level PLC program. One instance runs the entire system.

    CODESYS equivalent:
        PROGRAM PRG_Main
        VAR
            fbEStop         : FB_EStop;
            fbScanner       : FB_BarcodeScanner;
            fbTracker       : FB_ToteTracker;
            fbMergeMain     : FB_MergeConveyor;
            fbMergeRecirc   : FB_MergeConveyor;
            (* ... zone FB instances ... *)
        END_VAR
    """

    def __init__(self, topology_descriptors: Optional[List[ST_ZoneDescriptor]] = None) -> None:
        # ── System state ───────────────────────────────────────────────────
        self.eSystemState:      E_SystemState = E_SystemState.STOPPED
        self._bResetPulse:      bool  = False   # HMI reset consumed once per tick
        self._bReinductPulse:   bool  = False   # HMI reinduct consumed once per tick
        self._bSystemEStop:     bool  = False   # Software-triggered E-Stop (jam, recirc)
        self._iSpawnRemaining:  int   = 0       # Totes queued for infeed from HMI
        self._iToteCounter:     int   = 0       # Monotonic tote ID generator
        self._sLastScannerToteID: str = ""     # Last tote ID seen in scanner zone

        # ── Per-zone jam detection timers ─────────────────────────────────
        # zone_id → SIM_CLOCK time when zone entered RELEASING, or None
        self._jam_timers: Dict[str, Optional[float]] = {}

        # ── Core FBs ──────────────────────────────────────────────────────
        self.fbEStop:    FB_EStop          = FB_EStop()
        self.fbTracker:  FB_ToteTracker    = FB_ToteTracker()

        # ── Robot pickup (simulator layer) ────────────────────────────────
        self.sim_robot:  SimRobotPickup = SimRobotPickup()

        # ── Stuck tote simulation ─────────────────────────────────────────
        self.sim_stuck:  SimStuckTote = SimStuckTote()

        # ── Build topology ────────────────────────────────────────────────
        if topology_descriptors is None:
            topology_descriptors = default_sortation(
                CFG.iNumberOfChutes, CFG.iSpurLength, CFG.iRejectSpurLength
            )

        self._topology: ST_TopologyGraph = TopologyBuilder.compile(topology_descriptors)
        self._processor: TopologyProcessor = TopologyProcessor(self._topology)

        # ── Topology-derived references (generic, no hardcoded names) ────
        self.zone_fbs:   Dict[str, FB_ConveyorZone] = self._topology.zone_fbs
        self.zone_order: List[str] = self._topology.track_zone_order.get("main", [])
        self.branch_fbs: Dict[str, FB_BranchConveyor] = self._topology.branch_fbs

        # Scanner zone (first scanner in topology)
        self._scanner_zone_id: Optional[str] = (
            self._topology.scanner_zones[0] if self._topology.scanner_zones else None
        )
        self.fbScanner: FB_BarcodeScanner = FB_BarcodeScanner(self._scanner_zone_id or "")

        # Merge FBs — find by track membership, not hardcoded name
        self.merge_main_fb:   Optional[FB_MergeConveyor] = None
        self.merge_recirc_fb: Optional[FB_MergeConveyor] = None
        for merge_id, fb in self._topology.merge_fbs.items():
            track = self._topology.zone_to_track.get(merge_id, "")
            if track == "main" and not self.merge_main_fb:
                self.merge_main_fb = fb
            elif not self.merge_recirc_fb:
                self.merge_recirc_fb = fb

        # Spawn points: [(zone_id, sim_engine, downstream_zone_id)]
        self._spawn_points: list = []
        for spawn_id in self._topology.spawn_zones:
            track = self._topology.zone_to_track[spawn_id]
            sim = self._topology.sim_engines[track]
            zone_ids = self._topology.track_zone_order.get(track, [])
            idx = zone_ids.index(spawn_id) if spawn_id in zone_ids else -1
            ds_id = zone_ids[idx + 1] if idx >= 0 and idx + 1 < len(zone_ids) else None
            self._spawn_points.append((spawn_id, sim, ds_id))
        self._primary_spawn = self._spawn_points[0] if self._spawn_points else None
        self._reinduct_spawn = self._spawn_points[1] if len(self._spawn_points) > 1 else None

        # Pickup endpoints: [(zone_id, sim_engine, is_reject)]
        self._pickup_endpoints: list = []
        for zone_id in self._topology.pickup_zones:
            track = self._topology.zone_to_track[zone_id]
            sim = self._topology.sim_engines[track]
            is_reject = (track == "reject")
            self._pickup_endpoints.append((zone_id, sim, is_reject))

        self.recirc_zone_order:  List[str] = self._topology.track_zone_order.get("recirc", [])
        self.reinduct_zone_order: List[str] = self._topology.track_zone_order.get("reinduct", [])
        self.reject_zone_order:  List[str] = self._topology.track_zone_order.get("reject", [])

        # Spur tracks: detect by pickup endpoint (not hardcoded name prefix)
        self.spur_zone_fbs:    Dict[str, List[str]] = {}
        self.spur_zone_order:  Dict[str, List[str]] = {}
        self.sim_spurs:        Dict[str, SimEngine] = {}
        for zone_id in self._topology.pickup_zones:
            track = self._topology.zone_to_track[zone_id]
            if track == "reject":
                continue
            zone_ids = self._topology.track_zone_order.get(track, [])
            sim = self._topology.sim_engines.get(track)
            self.spur_zone_fbs[track] = zone_ids
            self.spur_zone_order[track] = zone_ids
            if sim:
                self.sim_spurs[track] = sim

        # SimEngine instances (legacy attributes for backward compat)
        self.sim_main:     SimEngine = self._topology.sim_engines.get("main", SimEngine())
        self.sim_recirc:   SimEngine = self._topology.sim_engines.get("recirc", SimEngine())
        self.sim_reinduct: SimEngine = self._topology.sim_engines.get("reinduct", SimEngine())
        self.sim_reject:   SimEngine = self._topology.sim_engines.get("reject", SimEngine())

        # ── Initialize jam timers for all zones ──────────────────────────
        for zone_id in self.zone_fbs:
            self._jam_timers[zone_id] = None

    # ── Transfer area backward-compat properties ────────────────────────────
    # These use default template zone names. For custom topologies they
    # safely return None/empty — the processor handles transfers generically.

    def _safe_transfer_tote(self, src, tgt):
        try:
            return self._processor.get_transfer_tote(src, tgt)
        except (KeyError, AttributeError):
            return None

    def _safe_transfer_tote_id(self, src, tgt):
        try:
            return self._processor.get_transfer_tote_id(src, tgt)
        except (KeyError, AttributeError):
            return ""

    @property
    def _postlast_tote(self) -> Optional[ST_ToteData]:
        return self._safe_transfer_tote("C_POST_LAST", "MERGE_RECIRC")

    @_postlast_tote.setter
    def _postlast_tote(self, val):
        self._processor.transfer_areas.setdefault("C_POST_LAST→MERGE_RECIRC", val)

    @property
    def _postlast_tote_id(self) -> str:
        return self._safe_transfer_tote_id("C_POST_LAST", "MERGE_RECIRC")

    @_postlast_tote_id.setter
    def _postlast_tote_id(self, val):
        self._processor.transfer_tote_ids["C_POST_LAST→MERGE_RECIRC"] = val

    @property
    def _recirc_tote(self) -> Optional[ST_ToteData]:
        return self._safe_transfer_tote("C_RECIRC", "MERGE_MAIN")

    @_recirc_tote.setter
    def _recirc_tote(self, val):
        self._processor.transfer_areas.setdefault("C_RECIRC→MERGE_MAIN", val)

    @property
    def _recirc_tote_id(self) -> str:
        return self._safe_transfer_tote_id("C_RECIRC", "MERGE_MAIN")

    @_recirc_tote_id.setter
    def _recirc_tote_id(self, val):
        self._processor.transfer_tote_ids["C_RECIRC→MERGE_MAIN"] = val

    @property
    def _reinduct_tote(self) -> Optional[ST_ToteData]:
        return self._safe_transfer_tote("C_RI_1", "MERGE_RECIRC")

    @_reinduct_tote.setter
    def _reinduct_tote(self, val):
        self._processor.transfer_areas.setdefault("C_RI_1→MERGE_RECIRC", val)

    @property
    def _reinduct_tote_id(self) -> str:
        return self._safe_transfer_tote_id("C_RI_1", "MERGE_RECIRC")

    @_reinduct_tote_id.setter
    def _reinduct_tote_id(self, val):
        self._processor.transfer_tote_ids["C_RI_1→MERGE_RECIRC"] = val

    # ══════════════════════════════════════════════════════════════════════
    # SCAN CYCLE (called every tick by the run loop)
    # ══════════════════════════════════════════════════════════════════════

    def tick(self) -> None:
        """One complete PLC scan cycle."""
        # ── 0. Advance simulation clock ───────────────────────────────────
        SIM_CLOCK.advance(CFG.rScanCycleRate_s)

        # ── 1. Read HMI commands ──────────────────────────────────────────
        self._process_hmi_commands()

        # ── 2. E-Stop processing ──────────────────────────────────────────
        bEStopActive = self._process_estop()

        # ── 2b. BLOCKED auto-resume check ─────────────────────────────────
        self._check_blocked_auto_resume()

        # ── 3. SimEngine tick — all tracks ────────────────────────────────
        self._tick_all_sim_engines()

        # ── 3b. Stuck tote simulation ─────────────────────────────────────
        self._process_stuck_totes()

        # Derive bEnable from system state (RUNNING = enabled)
        bEnable = (self.eSystemState == E_SystemState.RUNNING)

        # ── 4. Spawn logic (only when RUNNING) ───────────────────────────
        if bEnable:
            self._spawn_logic()

        # ── 4b. Re-induct logic (only when RUNNING) ─────────────────────
        if bEnable:
            self._reinduct_logic()

        # ── 5. Process ALL zones via TopologyProcessor ───────────────────
        self._processor.process_all_tracks(
            bEStopActive, bEnable, self._bResetPulse, self.fbTracker,
        )

        # ── 6. Scanner + ToteTracker + scan fail handling ─────────────────
        self._process_scanner(bEStopActive, bEnable)
        self._process_tracker()
        self._process_scan_fails()

        # ── 7. Recirc count E-Stop check ──────────────────────────────────
        self._check_recirc_count_estop()

        # ── 8. Robot pickup timers (all pickup zones) ────────────────────
        self._process_robot_pickups()

        # ── 9. BLOCKED state check (reject spur full) ───────────────────
        if self.eSystemState == E_SystemState.RUNNING:
            self._check_blocked_state()

        # ── 10. System-level jam detection ────────────────────────────────
        if bEnable:
            self._check_jam_detection()

    # ══════════════════════════════════════════════════════════════════════
    # STEP 1: HMI COMMANDS
    # ══════════════════════════════════════════════════════════════════════

    def _process_hmi_commands(self) -> None:
        """Read and consume HMI pulse commands."""
        # Start: only valid from STOPPED state
        if HMI.read_and_clear_pulse("bHMI_Start"):
            if self.eSystemState == E_SystemState.STOPPED:
                self.eSystemState = E_SystemState.RUNNING

        # Stop: graceful stop from RUNNING state
        if HMI.read_and_clear_pulse("bHMI_Stop"):
            if self.eSystemState == E_SystemState.RUNNING:
                self.eSystemState = E_SystemState.STOPPED

        # Reset pulse: consumed once, passed to E-Stop FB + all zone FBs
        self._bResetPulse = HMI.read_and_clear_pulse("bHMI_Reset")

        # Reinduct pulse: re-introduce one tote from reject pile
        self._bReinductPulse = HMI.read_and_clear_pulse("bHMI_ReinductTote")

        # Spawn count: "Add N Totes" button — queue N totes for infeed
        spawn_count = HMI.read_and_clear_count("iHMI_SpawnCount")
        if spawn_count > 0:
            available = len(INVENTORY.available_pool)
            actual = min(spawn_count, available)
            self._iSpawnRemaining += actual

        if HMI.read_and_clear_pulse("bHMI_PopulateSampleInventory"):
            INVENTORY.load_sample_inventory()

        if HMI.read_and_clear_pulse("bHMI_AckAll"):
            ALARMS.acknowledge_all()

    # ══════════════════════════════════════════════════════════════════════
    # STEP 2: E-STOP
    # ══════════════════════════════════════════════════════════════════════

    def _process_estop(self) -> bool:
        """
        Process E-Stop FB. Returns True if E-Stop is active.

        Three E-Stop sources (all trigger system-wide stop):
          1. Manual HMI button (HMI.bHMI_EStop)
          2. Software: recirc limit exceeded (_bSystemEStop)
          3. Software: jam detected (_bSystemEStop)

        Reset clears software E-Stop flag. Manual HMI button must be
        released by the user before reset is allowed.
        """
        # Reset clears software E-Stop (jam/recirc condition assumed resolved)
        if self._bResetPulse:
            self._bSystemEStop = False

        bEStopInput = HMI.bHMI_EStop or self._bSystemEStop

        self.fbEStop.set_inputs(bEStopInput=bEStopInput, bReset=self._bResetPulse)
        self.fbEStop.execute()

        if self.fbEStop.is_active():
            self.eSystemState = E_SystemState.ESTOPPED
        elif self._bResetPulse and self.eSystemState == E_SystemState.ESTOPPED:
            # Reset after E-Stop → STOPPED (NOT running). User must press Start.
            self.eSystemState = E_SystemState.STOPPED
            # Reconcile tracker with physical reality — remove ghost totes
            self._reconcile_tracker_after_reset()

        return self.fbEStop.is_active()

    def _trigger_system_estop(self, alarm_code: int, zone_id: str, extra: str = "") -> None:
        """
        Programmatic system E-Stop. Called when jam or recirc limit is detected.
        Raises alarm and sets software E-Stop flag.
        """
        ALARMS.raise_alarm(alarm_code, zone_id=zone_id, extra=extra)
        self._bSystemEStop = True

    def _reconcile_tracker_after_reset(self) -> None:
        """
        After E-Stop recovery, remove ghost totes from tracker,
        reset jam timers, and clear stuck tote states.
        """
        # Collect all tote IDs that physically exist in simulation
        live_totes: set = set()

        # All sim engine tracks
        for sim in self._topology.sim_engines.values():
            live_totes.update(sim.get_all_positions().keys())

        # Holding areas between sim tracks (waiting for merge acceptance)
        for tote_id in self._processor.get_all_transfer_tote_ids():
            live_totes.add(tote_id)

        # Remove ghost totes from tracker
        tracker_totes = set(self.fbTracker._tote_structs.keys())
        for tote_id in tracker_totes - live_totes:
            self.fbTracker._remove_tote(tote_id)

        # Reset all jam timers — start fresh after recovery
        for zone_id in self._jam_timers:
            self._jam_timers[zone_id] = None

        # Clear all stuck tote states
        self.sim_stuck.clear_all()

    # ══════════════════════════════════════════════════════════════════════
    # STEP 3: SIM ENGINE TICK
    # ══════════════════════════════════════════════════════════════════════

    def _tick_all_sim_engines(self) -> None:
        """Tick all SimEngine instances with current motor states."""
        def collect(zone_ids):
            m, s = {}, {}
            for zid in zone_ids:
                fb = self.zone_fbs[zid]
                m[zid] = fb.bMotorRun
                s[zid] = fb.rActualSpeed
            return m, s

        for track_name, sim in self._topology.sim_engines.items():
            zone_ids = self._topology.track_zone_order.get(track_name, [])
            m, s = collect(zone_ids)
            sim.tick(m, s)

    # ══════════════════════════════════════════════════════════════════════
    # STEP 4: SPAWN LOGIC
    # ══════════════════════════════════════════════════════════════════════

    def _spawn_logic(self) -> None:
        """
        Place totes on primary spawn zone one at a time from the HMI queue.
        Only spawns when spawn zone and its downstream are clear,
        inventory has stock, and under max totes.
        """
        if self._iSpawnRemaining <= 0:
            return
        if self._primary_spawn is None:
            return

        # Check inventory
        if INVENTORY.is_pool_empty():
            self._iSpawnRemaining = 0
            return

        spawn_id, sim, ds_id = self._primary_spawn

        # Wait for spawn area to clear (backpressure)
        if sim.is_zone_occupied(spawn_id):
            return
        if ds_id and sim.is_zone_occupied(ds_id):
            return

        # Respect max totes on system
        if self.fbTracker.get_active_count() >= CFG.iMaxTotesOnSystem:
            return

        # Spawn one tote
        tote_id = self._generate_tote_id()
        if sim.spawn_tote(tote_id, spawn_id):
            self.fbTracker.set_inputs(bRegisterTote=True, sNewToteID=tote_id)
            self.fbTracker.execute()
            self.fbTracker.set_inputs(bRegisterTote=False)
            self._iSpawnRemaining -= 1

    def _generate_tote_id(self) -> str:
        self._iToteCounter += 1
        return f"TOTE_{self._iToteCounter:04d}"

    # ══════════════════════════════════════════════════════════════════════
    # STEP 6: SCANNER + TRACKER
    # ══════════════════════════════════════════════════════════════════════

    def _process_scanner(self, bEStopActive: bool, bEnable: bool) -> None:
        """Run the barcode scanner on the scanner zone."""
        if not self._scanner_zone_id:
            return
        fb_scan_zone = self.zone_fbs[self._scanner_zone_id]
        tote_data    = fb_scan_zone.get_tote_data()
        tote_id      = tote_data.sToteID if tote_data else ""
        known_barcode = self.fbTracker.get_barcode_for_tote(tote_id)

        # Track last tote ID seen in scanner (for scan fail handling)
        if tote_id:
            self._sLastScannerToteID = tote_id

        self.fbScanner.set_inputs(
            bEnable=bEnable,
            bEStop=bEStopActive,
            bTotePresent=fb_scan_zone.has_tote(),
            sToteID=tote_id,
            rBeltSpeed=fb_scan_zone.get_speed(),
            sKnownBarcode=known_barcode,
        )
        self.fbScanner.execute()

    def _process_tracker(self) -> None:
        """Feed scanner results into ToteTracker. Advances routing queue."""
        spur_counts = self.fbTracker.get_assignment_counts(CFG.iNumberOfChutes)

        self.fbTracker.set_inputs(
            bScanComplete=self.fbScanner.scan_succeeded(),
            sToteIDScanned=self.fbScanner.sToteID if self.fbScanner.scan_succeeded() else "",
            sBarcodeScanned=self.fbScanner.get_barcode() if self.fbScanner.scan_succeeded() else "",
            aSpurCounts=spur_counts,
        )
        self.fbTracker.execute()

    def _process_scan_fails(self) -> None:
        """
        When scanner reports bNoRead (tote exited scan zone without successful read),
        increment the tote's iScanFailCount. If it hits the limit, assign REJECT.
        """
        if not self.fbScanner.is_no_read():
            return

        tote_id = self._sLastScannerToteID
        if not tote_id:
            return

        struct = self.fbTracker.get_tote_struct(tote_id)
        if struct is None:
            return

        struct.iScanFailCount += 1

        if struct.iScanFailCount >= CFG.iMaxScanFails:
            struct.eChuteAssignment = E_ChuteTarget.REJECT
            ALARMS.raise_alarm(
                13,
                zone_id=self._scanner_zone_id or "SCANNER",
                extra=f"tote {tote_id} failed scan {struct.iScanFailCount} times",
            )

    # ══════════════════════════════════════════════════════════════════════
    # STEP 7: RECIRC COUNT E-STOP CHECK
    # ══════════════════════════════════════════════════════════════════════

    def _check_recirc_count_estop(self) -> None:
        """
        After any merge accepts a tote, check if its recirc count
        exceeds the limit. The merge FB already incremented iRecircCount.
        """
        for merge_id, merge_fb in self._topology.merge_fbs.items():
            if not (merge_fb.is_accepting_from_input1() or merge_fb.is_accepting_from_input2()):
                continue
            tote = merge_fb.get_tote_data()
            if tote and tote.iRecircCount >= CFG.iMaxRecircCount:
                self._trigger_system_estop(
                    6,
                    zone_id=merge_id,
                    extra=f"tote {tote.sToteID} recirc={tote.iRecircCount}",
                )
                return

    # ══════════════════════════════════════════════════════════════════════
    # STEP 8: ROBOT PICKUP
    # ══════════════════════════════════════════════════════════════════════

    def _process_robot_pickups(self) -> None:
        """Run robot pickup timers for all pickup endpoints (spurs + reject)."""
        spur_endpoints: Dict[str, dict] = {}

        # Build endpoint info from topology-derived pickup list
        # _pickup_endpoints: [(zone_id, sim_engine, is_reject)]
        endpoint_sim_map: Dict[str, SimEngine] = {}
        for zone_id, sim, is_reject in self._pickup_endpoints:
            fb = self.zone_fbs[zone_id]
            tote_id_sim = sim.get_tote_in_zone(zone_id)
            tote_data = fb.get_tote_data()
            track = self._topology.zone_to_track.get(zone_id, "")

            spur_endpoints[zone_id] = {
                "spur_index":   track,
                "tote_present": tote_id_sim is not None,
                "tote_id":      tote_id_sim or "",
                "plc_tote_id":  tote_data.sToteID if tote_data else "",
                "plc_barcode":  tote_data.sBarcode if tote_data else "",
                "plc_chute":    tote_data.eChuteAssignment.value if tote_data else -1,
                "is_reject":    is_reject,
            }
            endpoint_sim_map[zone_id] = sim

        # Process manual pickup command
        manual_zone = HMI.sHMI_ManualPickup
        if manual_zone:
            HMI.sHMI_ManualPickup = ""  # Clear pulse
            self.sim_robot.manual_pickup(manual_zone)

        # Update auto-pickup toggles from HMI
        self.sim_robot.auto_pickup = dict(HMI.dictAutoPickup)

        results = self.sim_robot.tick(spur_endpoints)

        for result in results:
            is_reject = spur_endpoints.get(result.zone_id, {}).get("is_reject", False)
            sim = endpoint_sim_map.get(result.zone_id)

            if is_reject:
                # Reject pickup: remove from sim, add to reject pile
                if sim:
                    sim.remove_tote(result.tote_id)
                tote_data = self.fbTracker.get_tote_struct(result.tote_id)
                barcode = tote_data.sBarcode if tote_data else ""
                INVENTORY.reject_tote(result.tote_id, barcode)
            else:
                # Normal spur pickup: remove from sim
                if sim:
                    sim.remove_tote(result.tote_id)

            # Remove from PLC tracker
            self.fbTracker.set_inputs(
                bRemoveTote=True,
                sRemovedToteID=result.tote_id,
            )
            self.fbTracker.execute()
            self.fbTracker.set_inputs(bRemoveTote=False)
            self.fbTracker.execute()  # Reset _bPrevRemove for next iteration

    # ══════════════════════════════════════════════════════════════════════
    # STEP 9: BLOCKED STATE CHECK
    # ══════════════════════════════════════════════════════════════════════

    def _check_blocked_state(self) -> None:
        """
        If reject spur's last zone has a tote waiting (spur is full),
        transition to BLOCKED. All conveyors pause until reject clears.
        """
        if not self.reject_zone_order:
            return

        last_reject_id = self.reject_zone_order[-1]
        fb = self.zone_fbs[last_reject_id]

        # Check if reject spur is full: last zone has a tote in WAITING state
        if fb.has_tote() and fb.eState == E_ConveyorState.WAITING:
            self.eSystemState = E_SystemState.BLOCKED
            ALARMS.raise_alarm(12, zone_id=last_reject_id)

    def _check_blocked_auto_resume(self) -> None:
        """
        If system is BLOCKED and reject spur last zone no longer has a tote,
        auto-resume to RUNNING.
        """
        if self.eSystemState != E_SystemState.BLOCKED:
            return

        if not self.reject_zone_order:
            self.eSystemState = E_SystemState.RUNNING
            return

        last_reject_id = self.reject_zone_order[-1]
        fb = self.zone_fbs[last_reject_id]

        if not fb.has_tote():
            self.eSystemState = E_SystemState.RUNNING

    # ══════════════════════════════════════════════════════════════════════
    # STEP 3b: STUCK TOTE SIMULATION
    # ══════════════════════════════════════════════════════════════════════

    def _process_stuck_totes(self) -> None:
        """
        Run stuck tote simulation. Physical jams prevent tote movement.
        Overcurrent events trigger immediate E-Stop.
        """
        # Collect all tote IDs and their zones from all sim engines
        all_tote_ids = []
        tote_zone_map = {}

        for sim in self._topology.sim_engines.values():
            for zone_id, tote_id in sim.zone_tote_map.items():
                if tote_id:
                    all_tote_ids.append(tote_id)
                    tote_zone_map[tote_id] = zone_id

        events = self.sim_stuck.tick(all_tote_ids, tote_zone_map)

        for event in events:
            if event.stuck_type == E_StuckType.OVERCURRENT:
                self._trigger_system_estop(
                    11,
                    zone_id=event.zone_id,
                    extra=f"tote {event.tote_id} motor overcurrent",
                )

    # ══════════════════════════════════════════════════════════════════════
    # STEP 4b: RE-INDUCT LOGIC
    # ══════════════════════════════════════════════════════════════════════

    def _reinduct_logic(self) -> None:
        """
        Re-introduce rejected totes from the reject pile onto REINDUCT.
        Triggered by HMI pulse or auto-reinduct toggle.
        """
        # Check if we should reinduct
        should_reinduct = self._bReinductPulse
        if not should_reinduct and HMI.bHMI_AutoReinduct:
            should_reinduct = INVENTORY.get_reject_count() > 0

        if not should_reinduct:
            return

        # Check reject pile has totes
        if INVENTORY.get_reject_count() == 0:
            return

        # Need a reinduct spawn point
        if self._reinduct_spawn is None:
            return

        reinduct_id, reinduct_sim, reinduct_ds = self._reinduct_spawn

        # Backpressure: wait for reinduct track to clear
        if reinduct_sim.is_zone_occupied(reinduct_id):
            return
        if reinduct_ds and reinduct_sim.is_zone_occupied(reinduct_ds):
            return

        # Respect max totes
        if self.fbTracker.get_active_count() >= CFG.iMaxTotesOnSystem:
            return

        # Pop from reject pile
        reject_info = INVENTORY.reinduct_from_reject()
        if reject_info is None:
            return

        # Spawn with new tote ID (old one was retired)
        tote_id = self._generate_tote_id()
        if reinduct_sim.spawn_tote(tote_id, reinduct_id):
            # Register with tracker — same barcode, reset scan fail count
            self.fbTracker.set_inputs(bRegisterTote=True, sNewToteID=tote_id)
            self.fbTracker.execute()
            self.fbTracker.set_inputs(bRegisterTote=False)

            # The barcode is already claimed via register; update it with the reject barcode
            struct = self.fbTracker.get_tote_struct(tote_id)
            if struct:
                # Override with the rejected barcode (register may have claimed a new one)
                old_barcode = struct.sBarcode
                struct.sBarcode = reject_info["barcode"]
                struct.iScanFailCount = 0  # Reset scan fail count
                # Return the incorrectly claimed barcode back to pool
                if old_barcode and old_barcode != reject_info["barcode"]:
                    INVENTORY.available_pool.append(old_barcode)
                    if tote_id in INVENTORY.active_totes:
                        INVENTORY.active_totes[tote_id] = reject_info["barcode"]

    # ══════════════════════════════════════════════════════════════════════
    # STEP 10: SYSTEM-LEVEL JAM DETECTION
    # ══════════════════════════════════════════════════════════════════════

    def _check_jam_detection(self) -> None:
        """
        Per-zone check: if a zone is in RELEASING state for > CFG.rJamTimeout_s,
        it means the motor is ON trying to push a tote out but the beam break
        hasn't cleared. This indicates a physical jam → system E-Stop.

        Runs once per tick AFTER all zones have executed.
        """
        for zone_id, fb in self.zone_fbs.items():
            if fb.eState == E_ConveyorState.RELEASING:
                # Zone is in RELEASING — start or check timer
                if self._jam_timers[zone_id] is None:
                    self._jam_timers[zone_id] = SIM_CLOCK.now()
                else:
                    elapsed = SIM_CLOCK.now() - self._jam_timers[zone_id]
                    if elapsed >= CFG.rJamTimeout_s:
                        self._trigger_system_estop(
                            1,
                            zone_id=zone_id,
                            extra=f"RELEASING for {elapsed:.1f}s -- physical jam",
                        )
                        self._jam_timers[zone_id] = None  # Reset timer
            else:
                # Not in RELEASING — clear timer
                self._jam_timers[zone_id] = None

    # ══════════════════════════════════════════════════════════════════════
    # PUBLIC INTERFACE
    # ══════════════════════════════════════════════════════════════════════

    def start(self) -> None:
        """Start the system (equivalent to HMI Start button)."""
        if self.eSystemState == E_SystemState.STOPPED:
            self.eSystemState = E_SystemState.RUNNING

    def stop(self) -> None:
        """Graceful stop — motors wind down, zones stop accepting."""
        if self.eSystemState == E_SystemState.RUNNING:
            self.eSystemState = E_SystemState.STOPPED

    def get_system_state(self) -> E_SystemState:
        return self.eSystemState

    def get_zone_count(self) -> int:
        return len(self.zone_fbs)

    def get_all_zone_ids(self) -> List[str]:
        return list(self.zone_fbs.keys())

    def get_metrics(self) -> dict:
        return {
            "eSystemState":    self.eSystemState.name,
            "diActiveTotes":   self.fbTracker.get_active_count(),
            "diTotalProcessed": self.fbTracker.diTotalProcessed,
            "iSpawnRemaining": self._iSpawnRemaining,
            "iRejectPileCount": INVENTORY.get_reject_count(),
            "robot":           self.sim_robot.get_metrics(),
            "tracker":         self.fbTracker.get_outputs(),
            "stuck":           self.sim_stuck.to_dict(),
        }

    def get_topology(self) -> dict:
        """
        Export the zone topology graph for HMI auto-generation.
        Generic: driven entirely by compiled topology tracks and connection graph.
        """
        conn = self._topology.connection_graph

        # ── Build flat zones list from all tracks ────────────────────
        zones = []
        for track_name in self._topology.track_process_order:
            zone_ids = self._topology.track_zone_order.get(track_name, [])
            for zone_id in zone_ids:
                desc = self._topology.descriptors.get(zone_id)
                upstream_list = [src for src, _ in conn.get_upstream(zone_id)]
                downstream_list = [tgt for tgt, _ in conn.get_downstream(zone_id)]
                zones.append({
                    "id": zone_id,
                    "type": desc.eZoneType if desc else "transport",
                    "track": track_name,
                    "upstream": upstream_list,
                    "downstream": downstream_list,
                    "pickup": bool(desc and desc.bIsPickupPoint),
                    "spawn": bool(desc and desc.bIsSpawnPoint),
                })

        # ── Build tracks list (ordered by tote path for display) ─────
        display_order = self._compute_display_order()
        tracks = []
        for track_name in display_order:
            zone_ids = self._topology.track_zone_order.get(track_name, [])
            tracks.append({
                "name": track_name,
                "zones": list(zone_ids),
            })

        # ── Cross-track transfers ────────────────────────────────────
        cross_transfers = [
            {
                "source": xfer.sSourceZone,
                "target": xfer.sTargetZone,
                "source_track": xfer.sSourceTrack,
                "target_track": xfer.sTargetTrack,
            }
            for xfer in self._topology.cross_transfers
        ]

        # ── Backward compat keys for existing tests ──────────────────
        spurs = {}
        for track_name, zone_ids in self._topology.track_zone_order.items():
            if track_name.startswith("spur_"):
                chute_idx = int(track_name.split("_")[1])
                spurs[chute_idx] = list(zone_ids)

        return {
            "tracks": tracks,
            "display_order": display_order,
            "zones": zones,
            "cross_transfers": cross_transfers,
            "main_line": list(self._topology.track_zone_order.get("main", [])),
            "recirc_line": list(self._topology.track_zone_order.get("recirc", [])),
            "reinduct_line": list(self._topology.track_zone_order.get("reinduct", [])),
            "reject_line": list(self._topology.track_zone_order.get("reject", [])),
            "spurs": spurs,
            "config": {
                "iNumberOfChutes": CFG.iNumberOfChutes,
                "iSpurLength": CFG.iSpurLength,
                "iRejectSpurLength": CFG.iRejectSpurLength,
            },
        }

    def _compute_display_order(self) -> List[str]:
        """
        Order tracks by tote path for HMI display.
        Walk cross-track transfers: main → primary targets (recirc) → divert
        targets (spurs, reject) → remaining feeds (reinduct).
        """
        all_tracks = set(self._topology.track_zone_order.keys())
        ordered = []
        visited = set()

        # Index cross-transfers by source zone
        xfer_by_source: Dict[str, List] = {}
        for xfer in self._topology.cross_transfers:
            xfer_by_source.setdefault(xfer.sSourceZone, []).append(xfer)

        def add_track(name):
            if name not in visited and name in all_tracks:
                visited.add(name)
                ordered.append(name)

        # Start with main
        add_track("main")

        # BFS: walk each track's zones, collect outgoing cross-transfers
        i = 0
        while i < len(ordered):
            track_name = ordered[i]
            zone_ids = self._topology.track_zone_order.get(track_name, [])
            primary_targets = []
            divert_targets = []
            for zone_id in zone_ids:
                desc = self._topology.descriptors.get(zone_id)
                for xfer in xfer_by_source.get(zone_id, []):
                    is_divert = (desc and desc.sDivertDownstream == xfer.sTargetZone)
                    if is_divert:
                        divert_targets.append(xfer.sTargetTrack)
                    else:
                        primary_targets.append(xfer.sTargetTrack)
            # Primary downstream first (recirc), then diverts (spurs, reject)
            for t in primary_targets:
                add_track(t)
            for t in divert_targets:
                add_track(t)
            i += 1

        # Any remaining tracks not reachable from main
        for t in all_tracks:
            add_track(t)

        return ordered

    def get_zone_snapshot(self) -> dict:
        """Get current state of all zones for HMI WebSocket push."""
        snapshot = {}
        for zone_id, fb in self.zone_fbs.items():
            tote_data = fb.get_tote_data()
            snapshot[zone_id] = {
                "eState": fb.eState.name,
                "bMotorRun": fb.bMotorRun,
                "bTotePresent": fb.has_tote(),
                "bFaulted": fb.bFaulted,
                "rActualSpeed": fb.rActualSpeed,
                "sToteID": tote_data.sToteID if tote_data else "",
                "sBarcode": tote_data.sBarcode if tote_data else "",
                "eChuteAssignment": tote_data.eChuteAssignment.name if tote_data else "UNASSIGNED",
                "iRecircCount": tote_data.iRecircCount if tote_data else 0,
                "iScanFailCount": tote_data.iScanFailCount if tote_data else 0,
                "bReady": fb.is_ready(),
            }
        return snapshot
