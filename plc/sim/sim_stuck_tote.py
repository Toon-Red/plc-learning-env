"""
sim/sim_stuck_tote.py
=====================
Stuck tote simulation layer.
NOT PLC code — pure simulation.

Randomly causes totes to get "stuck" on conveyor zones:
  - Per-tick probability: CFG.rStuckToteChance (default 0.05%)
  - When stuck, two outcomes:
    25% motor overcurrent → immediate E-Stop (alarm A011 CRITICAL)
    75% physical jam → tote stops moving, 25% per-tick self-clear chance

The PLC layer sees the jam as a zone staying in RELEASING state
(motor running but tote not moving). The stuck tote engine informs
SimEngine which totes should not advance.
"""

import random
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Set, Optional

from plc.gvl.gvl_config import CFG


class E_StuckType(IntEnum):
    PHYSICAL_JAM    = 0   # Tote stuck, may self-clear
    OVERCURRENT     = 1   # Motor overcurrent, immediate E-Stop


@dataclass
class StuckEvent:
    """Returned when a new stuck event occurs."""
    tote_id:    str
    zone_id:    str
    stuck_type: E_StuckType


@dataclass
class _StuckState:
    """Internal tracking for a stuck tote."""
    tote_id:    str
    zone_id:    str
    stuck_type: E_StuckType


class SimStuckTote:
    """
    Manages stuck tote events across all sim engines.
    One instance in PRG_Main.
    """

    def __init__(self) -> None:
        # tote_id → stuck state (only physical jams persist; overcurrent is instant)
        self._stuck_totes: Dict[str, _StuckState] = {}

    def tick(self, all_tote_ids: List[str], tote_zone_map: Dict[str, str]) -> List[StuckEvent]:
        """
        Called every tick. Checks for new stuck events and self-clearing.

        Args:
            all_tote_ids: list of all active tote IDs across all sim engines
            tote_zone_map: tote_id → zone_id for all active totes

        Returns:
            List of new StuckEvents (overcurrent events need E-Stop processing)
        """
        events: List[StuckEvent] = []

        # ── Self-clear check for existing physical jams ─────────────────
        cleared = []
        for tote_id, state in self._stuck_totes.items():
            if state.stuck_type == E_StuckType.PHYSICAL_JAM:
                if random.random() < CFG.rStuckSelfClearChance:
                    cleared.append(tote_id)
        for tote_id in cleared:
            del self._stuck_totes[tote_id]

        # ── New stuck events ────────────────────────────────────────────
        if CFG.rStuckToteChance <= 0:
            return events

        for tote_id in all_tote_ids:
            if tote_id in self._stuck_totes:
                continue  # Already stuck
            if random.random() < CFG.rStuckToteChance:
                zone_id = tote_zone_map.get(tote_id, "UNKNOWN")
                if random.random() < CFG.rOvercurrentChance:
                    # Motor overcurrent — instant E-Stop
                    event = StuckEvent(
                        tote_id=tote_id,
                        zone_id=zone_id,
                        stuck_type=E_StuckType.OVERCURRENT,
                    )
                    events.append(event)
                    # Overcurrent doesn't persist as "stuck" — E-Stop handles it
                else:
                    # Physical jam — tote stops moving
                    state = _StuckState(
                        tote_id=tote_id,
                        zone_id=zone_id,
                        stuck_type=E_StuckType.PHYSICAL_JAM,
                    )
                    self._stuck_totes[tote_id] = state
                    events.append(StuckEvent(
                        tote_id=tote_id,
                        zone_id=zone_id,
                        stuck_type=E_StuckType.PHYSICAL_JAM,
                    ))

        return events

    def get_stuck_tote_ids(self) -> Set[str]:
        """Returns set of tote IDs that are currently physically jammed."""
        return set(self._stuck_totes.keys())

    def is_stuck(self, tote_id: str) -> bool:
        return tote_id in self._stuck_totes

    def clear_all(self) -> None:
        """Clear all stuck states (e.g. after E-Stop reset)."""
        self._stuck_totes.clear()

    def clear_tote(self, tote_id: str) -> None:
        """Clear stuck state for a specific tote."""
        self._stuck_totes.pop(tote_id, None)

    def force_stuck(self, tote_id: str, zone_id: str) -> None:
        """Force a tote to be stuck (simulator god-mode)."""
        self._stuck_totes[tote_id] = _StuckState(
            tote_id=tote_id,
            zone_id=zone_id,
            stuck_type=E_StuckType.PHYSICAL_JAM,
        )

    def get_stuck_count(self) -> int:
        return len(self._stuck_totes)

    def to_dict(self) -> dict:
        return {
            "stuck_totes": {
                tid: {
                    "zone_id": s.zone_id,
                    "stuck_type": s.stuck_type.name,
                }
                for tid, s in self._stuck_totes.items()
            },
            "count": len(self._stuck_totes),
        }
