"""
gvl/gvl_io.py
=============
IEC 61131-3 equivalent: GVL_IO  {attribute 'qualified_only'}

Simulated physical I/O state. Mirrors what hardware I/O cards would expose.
Written by: PLC scan cycle (PRG_Main / FB outputs)
Read by:    HMI (display only — NEVER written by HMI)

In CODESYS this would be linked to actual I/O addresses (%IX, %QX).
Here it's a live dict kept in sync by PRG_Main every tick.
"""

import threading
from typing import Dict
from plc.types.structs import ST_ZoneStatus
from plc.types.enums import E_SystemState


class GVL_IO:
    """
    Thread-safe I/O image. WebSocket server reads this every tick to push
    state to the HMI. PRG_Main writes this after every Calculate phase.

    All zone data is keyed by zone_id string (e.g. "INFEED", "C1", "BRANCH_1").
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()

        # ── System-level ───────────────────────────────────────────────────
        self.bEStopActive:    bool           = False
        self.eSystemState:    E_SystemState  = E_SystemState.STOPPED
        self.bSystemRunning:  bool           = False
        self.diActiveTotes:   int            = 0
        self.diTotalProcessed: int           = 0

        # ── Zone status map ────────────────────────────────────────────────
        # zone_id → ST_ZoneStatus
        self.zone_states: Dict[str, ST_ZoneStatus] = {}

        # ── Spur status (per chute index 1..N) ────────────────────────────
        # chute_index (int) → dict with keys: bSpurFull, bSpurEmpty, diToteCount
        self.spur_states: Dict[int, dict] = {}

    def update_zone(self, zone_id: str, status: ST_ZoneStatus) -> None:
        with self._lock:
            self.zone_states[zone_id] = status

    def update_spur(self, chute_index: int, full: bool, empty: bool, count: int) -> None:
        with self._lock:
            self.spur_states[chute_index] = {
                "bSpurFull":    full,
                "bSpurEmpty":   empty,
                "diToteCount":  count,
            }

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "bEStopActive":     self.bEStopActive,
                "eSystemState":     self.eSystemState.name,
                "bSystemRunning":   self.bSystemRunning,
                "diActiveTotes":    self.diActiveTotes,
                "diTotalProcessed": self.diTotalProcessed,
                "zone_states":      {k: v.to_dict() for k, v in self.zone_states.items()},
                "spur_states":      self.spur_states,
            }


# Singleton
IO = GVL_IO()
