"""
function_blocks/fb_tote_tracker.py
====================================
IEC 61131-3 equivalent:

    FUNCTION_BLOCK FB_ToteTracker
    VAR_INPUT
        sBarcodeScanned  : STRING;    (* From FB_BarcodeScanner on successful scan *)
        sToteIDScanned   : STRING;    (* Which tote was just scanned *)
        bScanComplete    : BOOL;      (* Rising edge = new scan result ready *)
        bRegisterTote    : BOOL;      (* Rising edge = new tote entering system *)
        sNewToteID       : STRING;
        bRemoveTote      : BOOL;      (* Rising edge = robot removed tote *)
        sRemovedToteID   : STRING;
        aSpurCounts      : ARRAY OF INT;
    END_VAR
    VAR_OUTPUT
        diActiveTotes    : DINT;
        diTotalProcessed : DINT;
    END_VAR

Canonical struct registry:
  - _tote_structs[tote_id] = ST_ToteData  — THE authoritative object per tote
  - Conveyors hold REFERENCES to these objects (not copies)
  - Routing engine writes eChuteAssignment directly onto the struct
  - PLC code reads from the struct wherever it is — always sees latest value

Simulated routing delay:
  - When scan completes, routing request goes into _pending_routing queue
  - Each tick: pending items count down their delay_ticks_remaining
  - When a pending item reaches 0: routing engine runs, writes to struct
  - The PLC zone holding the tote sees the struct update automatically (same ref)
  - This simulates a real WMS response without hardcoding wait logic into PLC code
"""

import random
from typing import Optional, Dict, List
from dataclasses import dataclass

from plc.types.enums import E_ChuteTarget, E_RoutingStrategy
from plc.types.structs import ST_ToteData
from plc.gvl.gvl_inventory import INVENTORY
from plc.gvl.gvl_config import CFG
from plc.gvl.gvl_alarms import ALARMS
from plc.function_blocks.fb_routing_engine import FB_RoutingEngine
from plc.sim.sim_clock import SIM_CLOCK


@dataclass
class _PendingRouting:
    """
    A routing request waiting for its simulated delay to expire.
    Not exposed outside this module.
    """
    tote_id:             str
    barcode:             str
    spur_counts:         list
    ticks_remaining:     int


class FB_ToteTracker:
    """
    Central tote registry and routing coordinator.
    One instance in PRG_Main.
    """

    def __init__(self) -> None:
        # ── VAR_INPUT ──────────────────────────────────────────────────────
        self.sBarcodeScanned:  str   = ""
        self.sToteIDScanned:   str   = ""
        self.bScanComplete:    bool  = False
        self.bRegisterTote:    bool  = False
        self.sNewToteID:       str   = ""
        self.bRemoveTote:      bool  = False
        self.sRemovedToteID:   str   = ""
        self.aSpurCounts:      list  = []

        # ── VAR_OUTPUT ─────────────────────────────────────────────────────
        self.diActiveTotes:    int   = 0
        self.diTotalProcessed: int   = 0

        # ── VAR (internal) ─────────────────────────────────────────────────
        # Canonical ST_ToteData objects — shared by reference with conveyor zones
        self._tote_structs:     Dict[str, ST_ToteData]    = {}

        # Routing requests waiting for their simulated delay
        self._pending_routing:  List[_PendingRouting]     = []

        self._fbRoutingEngine:  FB_RoutingEngine          = FB_RoutingEngine()
        self._bPrevScan:        bool                      = False
        self._bPrevRegister:    bool                      = False
        self._bPrevRemove:      bool                      = False

    # ── Input interface ────────────────────────────────────────────────────

    def set_inputs(
        self,
        *,
        sBarcodeScanned:  str  = "",
        sToteIDScanned:   str  = "",
        bScanComplete:    bool = False,
        bRegisterTote:    bool = False,
        sNewToteID:       str  = "",
        bRemoveTote:      bool = False,
        sRemovedToteID:   str  = "",
        aSpurCounts:      list = None,
    ) -> None:
        self.sBarcodeScanned  = sBarcodeScanned
        self.sToteIDScanned   = sToteIDScanned
        self.bScanComplete    = bScanComplete
        self.bRegisterTote    = bRegisterTote
        self.sNewToteID       = sNewToteID
        self.bRemoveTote      = bRemoveTote
        self.sRemovedToteID   = sRemovedToteID
        self.aSpurCounts      = aSpurCounts if aSpurCounts is not None else []

    # ── Execution ──────────────────────────────────────────────────────────

    def execute(self) -> None:
        """
        One scan cycle — processes rising-edge events and advances routing queue.
        PLC code does not know or care about the pending queue — it just reads
        the struct value whenever it needs it.
        """

        # ── Rising edge: register new tote entering system ─────────────────
        if self.bRegisterTote and not self._bPrevRegister:
            self._register_tote(self.sNewToteID)

        # ── Rising edge: scanner returned a result ─────────────────────────
        if self.bScanComplete and not self._bPrevScan:
            self._enqueue_routing(self.sToteIDScanned, self.sBarcodeScanned)

        # ── Rising edge: robot removed a tote ─────────────────────────────
        if self.bRemoveTote and not self._bPrevRemove:
            self._remove_tote(self.sRemovedToteID)

        # ── Advance pending routing queue ──────────────────────────────────
        # This is the "lazy" part — runs every tick, resolves items when ready
        self._tick_routing_queue()

        # ── Update previous-edge values ───────────────────────────────────
        self._bPrevScan     = self.bScanComplete
        self._bPrevRegister = self.bRegisterTote
        self._bPrevRemove   = self.bRemoveTote

        # ── Update output counters ─────────────────────────────────────────
        self.diActiveTotes = len(self._tote_structs)

    # ── Internal: tote lifecycle ───────────────────────────────────────────

    def _register_tote(self, tote_id: str) -> None:
        """
        New tote entered the system. Create canonical struct, claim barcode.
        Struct starts with sBarcode="" and eChuteAssignment=UNASSIGNED.
        Zone holds a reference to this struct — routing will update it in place.
        """
        if not tote_id or tote_id in self._tote_structs:
            return

        barcode = INVENTORY.claim_next_tote(tote_id)
        if barcode is None:
            ALARMS.raise_alarm(7, zone_id="INFEED")
            barcode = ""

        struct = ST_ToteData()
        struct.sToteID          = tote_id
        struct.sBarcode         = barcode     # Known to system, not yet scanned by sensor
        struct.eChuteAssignment = E_ChuteTarget.UNASSIGNED
        struct.bIsActive        = True

        self._tote_structs[tote_id] = struct

    def _enqueue_routing(self, tote_id: str, barcode_scanned: str) -> None:
        """
        Scanner returned a result. Check for mismatch, then queue routing.
        Re-scan behavior:
          - Confirm barcode matches (mismatch → alarm)
          - If tote already has a valid chute assignment, don't re-queue
          - If tote is UNASSIGNED but already in pending queue, don't duplicate
          - If tote is UNASSIGNED and not in queue, add to queue
        """
        if not tote_id:
            return

        struct = self._tote_structs.get(tote_id)
        if struct is None:
            return

        known_barcode = struct.sBarcode

        # Mismatch: struct already had a barcode AND scan returned something different
        if known_barcode and barcode_scanned and known_barcode != barcode_scanned:
            ALARMS.raise_alarm(
                10,
                zone_id="C3_SCAN",
                extra=f"tote {tote_id}: known='{known_barcode}', scanned='{barcode_scanned}'",
            )
            return

        # Accept scanned barcode (first scan or consistent re-scan)
        if barcode_scanned:
            struct.sBarcode      = barcode_scanned
            struct.bScanComplete = True
            struct.tLastScan     = SIM_CLOCK.now()

        # Already has a valid chute assignment — don't re-queue
        chute_val = struct.eChuteAssignment.value
        if 1 <= chute_val <= CFG.iNumberOfChutes:
            return

        # REJECT assignment (from scan fail limit) — don't route
        if struct.eChuteAssignment == E_ChuteTarget.REJECT:
            return

        # Already in pending queue — don't duplicate
        if self.is_tote_in_pending_queue(tote_id):
            return

        # Queue routing with randomized delay
        delay = random.randint(
            CFG.iRoutingDelayMin_ticks,
            CFG.iRoutingDelayMax_ticks,
        )
        self._pending_routing.append(_PendingRouting(
            tote_id=tote_id,
            barcode=barcode_scanned or known_barcode,
            spur_counts=list(self.aSpurCounts),
            ticks_remaining=delay,
        ))

    def _tick_routing_queue(self) -> None:
        """
        Advance all pending routing requests by one tick.
        When a request reaches 0, run the routing engine and write the result
        directly onto the canonical struct. The conveyor holding this tote
        sees the update automatically on the next scan (same reference).
        """
        still_pending = []
        for item in self._pending_routing:
            item.ticks_remaining -= 1
            if item.ticks_remaining <= 0:
                self._resolve_routing(item)
            else:
                still_pending.append(item)
        self._pending_routing = still_pending

    def _resolve_routing(self, item: _PendingRouting) -> None:
        """
        Routing delay expired — run the strategy, write result to struct.
        After this call, any conveyor holding a reference to this struct
        will see the updated eChuteAssignment on its next scan.

        If routing returns UNASSIGNED (all chutes at capacity), the tote
        stays UNASSIGNED and recirculates. The scanner will re-add it to
        the routing queue on the next pass through C3_SCAN.
        """
        struct = self._tote_structs.get(item.tote_id)
        if struct is None:
            return   # Tote was removed before routing resolved

        self._fbRoutingEngine.set_inputs(
            sBarcode=item.barcode,
            aSpurCounts=item.spur_counts,
            iNumChutes=CFG.iNumberOfChutes,
        )
        self._fbRoutingEngine.execute()

        if self._fbRoutingEngine.is_complete():
            struct.eChuteAssignment = self._fbRoutingEngine.get_assignment()
            self.diTotalProcessed += 1
        elif self._fbRoutingEngine.get_assignment() == E_ChuteTarget.UNASSIGNED:
            # All chutes at capacity — tote stays UNASSIGNED, will recirculate
            # and get re-queued when it passes through the scanner again
            pass
        else:
            struct.eChuteAssignment = E_ChuteTarget.REJECT
            ALARMS.raise_alarm(4, zone_id="C3_SCAN", extra=f"tote {item.tote_id}")
            self.diTotalProcessed += 1

    def _remove_tote(self, tote_id: str) -> None:
        """Robot picked up tote. Remove from registry, release barcode to pool."""
        if not tote_id:
            return
        # Remove any pending routing for this tote
        self._pending_routing = [
            p for p in self._pending_routing if p.tote_id != tote_id
        ]
        self._tote_structs.pop(tote_id, None)
        INVENTORY.release_tote(tote_id)

    # ── Output accessors (named methods — storage can swap underneath) ─────

    def get_tote_struct(self, tote_id: str) -> Optional[ST_ToteData]:
        """
        Returns the canonical ST_ToteData reference for a tote.
        Called by PRG_Main at spawn time to get the reference to pass to the zone.
        """
        return self._tote_structs.get(tote_id)

    def get_chute_for_tote(self, tote_id: str) -> E_ChuteTarget:
        """
        Returns the current chute assignment. Reads directly from struct.
        UNASSIGNED if tote not registered or routing not yet resolved.
        """
        struct = self._tote_structs.get(tote_id)
        return struct.eChuteAssignment if struct else E_ChuteTarget.UNASSIGNED

    def get_barcode_for_tote(self, tote_id: str) -> str:
        struct = self._tote_structs.get(tote_id)
        return struct.sBarcode if struct else ""

    def is_tote_registered(self, tote_id: str) -> bool:
        return tote_id in self._tote_structs

    def get_active_count(self) -> int:
        return len(self._tote_structs)

    def get_pending_routing_count(self) -> int:
        """How many totes are waiting for chute assignment."""
        return len(self._pending_routing)

    def get_assignment_counts(self, num_chutes: int) -> list:
        """
        Count how many totes on the system are assigned to each chute.
        Returns list of length num_chutes (index 0 = chute 1).
        This is the LOGICAL capacity — includes totes still in transit
        to their chute, not just physically on spur zones.
        """
        counts = [0] * num_chutes
        for struct in self._tote_structs.values():
            chute_val = struct.eChuteAssignment.value
            if 1 <= chute_val <= num_chutes:
                counts[chute_val - 1] += 1
        return counts

    def is_tote_in_pending_queue(self, tote_id: str) -> bool:
        """Check if a tote is already waiting for routing resolution."""
        return any(p.tote_id == tote_id for p in self._pending_routing)

    def get_all_assignments(self) -> dict:
        """Full tote → chute map for HMI snapshot."""
        return {
            tid: s.eChuteAssignment.name
            for tid, s in self._tote_structs.items()
        }

    def get_outputs(self) -> dict:
        return {
            "diActiveTotes":        self.diActiveTotes,
            "diTotalProcessed":     self.diTotalProcessed,
            "iPendingRouting":      self.get_pending_routing_count(),
            "assignments":          self.get_all_assignments(),
        }
