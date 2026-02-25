"""
gvl/gvl_inventory.py
====================
IEC 61131-3 equivalent: GVL_Inventory  {attribute 'qualified_only'}

Master tote inventory and routing table.
This is the list that "feeds" the warehouse — totes exist here before
they are physically placed on the infeed conveyor.

Written by: external API (add totes), robot pickup (remove totes), HMI sample populate
Read by:    PRG_Main (spawn logic), FB_ToteTracker (routing lookup)

Analogy in a real warehouse: this would be the WMS (Warehouse Management System)
feeding the PLC with "which barcode goes to which chute." Here we simulate that
as an in-memory dict that can be updated via API at runtime.
"""

import threading
import random
from typing import Dict, Optional

from plc.types.enums import E_ChuteTarget


# ── Sample data for easy testing ──────────────────────────────────────────
SAMPLE_INVENTORY = {
    "BC-1001": E_ChuteTarget.CHUTE_1,
    "BC-1002": E_ChuteTarget.CHUTE_1,
    "BC-1003": E_ChuteTarget.CHUTE_2,
    "BC-1004": E_ChuteTarget.CHUTE_2,
    "BC-1005": E_ChuteTarget.CHUTE_3,
    "BC-1006": E_ChuteTarget.CHUTE_3,
    "BC-1007": E_ChuteTarget.CHUTE_1,
    "BC-1008": E_ChuteTarget.CHUTE_2,
    "BC-1009": E_ChuteTarget.CHUTE_3,
    "BC-1010": E_ChuteTarget.CHUTE_1,
    "BC-2001": E_ChuteTarget.CHUTE_2,
    "BC-2002": E_ChuteTarget.CHUTE_1,
    "BC-2003": E_ChuteTarget.CHUTE_3,
    "BC-3001": E_ChuteTarget.CHUTE_2,
    "BC-3002": E_ChuteTarget.CHUTE_3,
}


class GVL_Inventory:
    """
    Two separate data structures:

    routing_table: barcode → E_ChuteTarget
        The permanent routing rules. "BC-1001 always goes to Chute 1."
        Populated from config or external API. Does not shrink when totes are removed.

    available_pool: set of barcodes available to spawn
        Totes in this pool are "waiting at the dock" to enter the system.
        When spawned → moved to active_totes.
        When robot removes → barcode returned to available_pool (infinite cycle).

    active_totes: set of tote IDs currently on the conveyor system
        Used to enforce iMaxTotesOnSystem cap.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()

        # barcode → chute assignment
        self.routing_table: Dict[str, E_ChuteTarget] = {}

        # barcodes available to spawn (not currently on system)
        self.available_pool: list = []

        # tote IDs currently active on the conveyor
        self.active_totes: Dict[str, str] = {}  # tote_id → barcode

        # Reject pile: totes removed from reject spur, awaiting re-induction
        self.reject_pile: list = []  # list of {"tote_id": str, "barcode": str}

    # ── Inventory management ───────────────────────────────────────────────

    def load_sample_inventory(self) -> None:
        """Populate with built-in sample data. Called by HMI sample button."""
        with self._lock:
            self.routing_table = dict(SAMPLE_INVENTORY)
            self.available_pool = list(SAMPLE_INVENTORY.keys())
            random.shuffle(self.available_pool)

    def add_barcode(self, barcode: str, chute: E_ChuteTarget) -> None:
        """Add a single barcode+routing rule. Called by external API."""
        with self._lock:
            self.routing_table[barcode] = chute
            if barcode not in self.available_pool:
                self.available_pool.append(barcode)

    def remove_barcode(self, barcode: str) -> None:
        """Remove a barcode from routing table entirely."""
        with self._lock:
            self.routing_table.pop(barcode, None)
            if barcode in self.available_pool:
                self.available_pool.remove(barcode)

    # ── Spawn / removal ────────────────────────────────────────────────────

    def claim_next_tote(self, tote_id: str) -> Optional[str]:
        """
        Pull the next available barcode for a spawning tote.
        Returns the barcode string, or None if pool is empty.
        Marks it as active (on system).
        """
        with self._lock:
            if not self.available_pool:
                return None
            barcode = self.available_pool.pop(0)
            self.active_totes[tote_id] = barcode
            return barcode

    def release_tote(self, tote_id: str) -> None:
        """
        Called when robot removes a tote from a normal spur.
        Returns the barcode to the available pool for re-use.
        """
        with self._lock:
            barcode = self.active_totes.pop(tote_id, None)
            if barcode and barcode in self.routing_table:
                self.available_pool.append(barcode)

    def reject_tote(self, tote_id: str, barcode: str) -> None:
        """
        Called when tote is removed from reject spur.
        Adds to reject pile (NOT back to available pool).
        """
        with self._lock:
            self.active_totes.pop(tote_id, None)
            self.reject_pile.append({"tote_id": tote_id, "barcode": barcode})

    def reinduct_from_reject(self) -> Optional[dict]:
        """
        Pop one tote from reject pile for re-induction.
        Returns {"tote_id": str, "barcode": str} or None if empty.
        """
        with self._lock:
            if not self.reject_pile:
                return None
            return self.reject_pile.pop(0)

    def get_reject_count(self) -> int:
        with self._lock:
            return len(self.reject_pile)

    # ── Lookup ─────────────────────────────────────────────────────────────

    def get_barcode_for_tote(self, tote_id: str) -> str:
        """
        Returns the barcode assigned to an active tote.
        Called by FB_BarcodeScanner on a successful scan attempt.
        Returns "" if tote_id not found in active_totes.
        """
        with self._lock:
            return self.active_totes.get(tote_id, "")

    def lookup_chute(self, barcode: str) -> E_ChuteTarget:
        """
        Returns the chute assignment for a barcode.
        Returns E_ChuteTarget.RECIRCULATE if barcode not in routing table.
        """
        with self._lock:
            return self.routing_table.get(barcode, E_ChuteTarget.RECIRCULATE)

    def is_pool_empty(self) -> bool:
        with self._lock:
            return len(self.available_pool) == 0

    def active_count(self) -> int:
        with self._lock:
            return len(self.active_totes)

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "iRoutingTableSize":   len(self.routing_table),
                "iAvailablePoolSize":  len(self.available_pool),
                "iActiveToteCount":    len(self.active_totes),
                "iRejectPileCount":    len(self.reject_pile),
                "routing_table": {
                    k: v.name for k, v in self.routing_table.items()
                },
                "available_pool":      list(self.available_pool),
                "reject_pile":         list(self.reject_pile),
            }


# Singleton
INVENTORY = GVL_Inventory()
