"""
gvl/gvl_hmi.py
==============
IEC 61131-3 equivalent: GVL_HMI  {attribute 'qualified_only'}

Command tags written by the HMI operator interface.
Written by: HMI only (via API server)
Read by:    PRG_Main (CHECK INPUTS phase)

CRITICAL: PRG_Main consumes these tags and clears rising-edge flags
after each scan so they act as one-shot commands, exactly as
CODESYS F_TRIG / R_TRIG would handle HMI button presses.
"""

import threading
from typing import Dict
from plc.types.enums import E_MergePriority


class GVL_HMI:
    """
    Thread-safe HMI command register.
    The HMI writes here. PRG_Main reads and resets pulse flags each tick.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()

        # ── System commands (pulse — cleared by PRG_Main after one scan) ──
        self.bHMI_Start:       bool = False
        self.bHMI_Stop:        bool = False
        self.bHMI_EStop:       bool = False   # Latched by FB_EStop
        self.bHMI_Reset:       bool = False
        self.iHMI_SpawnCount:  int  = 0       # Add N totes to infeed queue

        # ── Re-induct (rejected tote re-introduction) ────────────────────
        self.bHMI_ReinductTote:  bool = False  # Pulse: re-induct one tote from reject pile
        self.bHMI_AutoReinduct:  bool = False  # Persistent: auto-reinduct when reject pile non-empty

        # ── Pickup controls ──────────────────────────────────────────────
        self.dictAutoPickup:     Dict[str, bool] = {}  # zone_id → auto pickup (default True)
        self.sHMI_ManualPickup:  str  = ""     # Pulse-like: zone_id to manually pick up

        # ── Per-zone speed overrides (persistent until changed) ────────────
        # zone_id → speed % (0.0–100.0); -1.0 = use default
        self.rHMI_SpeedOverride: Dict[str, float] = {}

        # ── Merge priority override ─────────────────────────────────────────
        self.eMergePriorityOverride: E_MergePriority = E_MergePriority.INPUT2_PRIORITY

        # ── Alarm acknowledgement ──────────────────────────────────────────
        # Index into GVL_Alarms.aAlarmBuffer to acknowledge
        self.iHMI_AckAlarmIndex: int  = -1   # -1 = no ack pending
        self.bHMI_AckAll:        bool = False

        # ── Inventory management ───────────────────────────────────────────
        self.bHMI_PopulateSampleInventory: bool = False  # Load sample tote list

    def write(self, key: str, value) -> None:
        """Generic setter used by API server. Key must be a known attribute."""
        with self._lock:
            if not hasattr(self, key):
                raise KeyError(f"GVL_HMI has no tag: {key}")
            setattr(self, key, value)

    def read_and_clear_pulse(self, key: str) -> bool:
        """
        Read a boolean pulse flag and immediately clear it.
        Mirrors CODESYS R_TRIG pattern for HMI button presses.
        Returns True once on the scan the button was pressed.
        """
        with self._lock:
            val = getattr(self, key)
            if val:
                setattr(self, key, False)
            return val

    def read_and_clear_count(self, key: str) -> int:
        """
        Read an integer count and immediately clear it to 0.
        Used for "Add N totes" style commands.
        """
        with self._lock:
            val = getattr(self, key)
            if val > 0:
                setattr(self, key, 0)
            return val

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "bHMI_Start":                   self.bHMI_Start,
                "bHMI_Stop":                    self.bHMI_Stop,
                "bHMI_EStop":                   self.bHMI_EStop,
                "bHMI_Reset":                   self.bHMI_Reset,
                "iHMI_SpawnCount":              self.iHMI_SpawnCount,
                "bHMI_ReinductTote":            self.bHMI_ReinductTote,
                "bHMI_AutoReinduct":            self.bHMI_AutoReinduct,
                "dictAutoPickup":               dict(self.dictAutoPickup),
                "sHMI_ManualPickup":            self.sHMI_ManualPickup,
                "rHMI_SpeedOverride":           self.rHMI_SpeedOverride,
                "eMergePriorityOverride":        self.eMergePriorityOverride.name,
                "iHMI_AckAlarmIndex":           self.iHMI_AckAlarmIndex,
                "bHMI_AckAll":                  self.bHMI_AckAll,
                "bHMI_PopulateSampleInventory": self.bHMI_PopulateSampleInventory,
            }


# Singleton
HMI = GVL_HMI()
