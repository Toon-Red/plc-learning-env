"""
sim/sim_robot_pickup.py
========================
Simulator-level robot pickup logic.
NOT PLC code — this is pure simulation.

Each spur endpoint has a virtual robot that:
  1. Detects when a tote arrives (beam break at last spur zone)
  2. Starts a random timer (rPickupIntervalMin_s to rPickupIntervalMax_s)
  3. When timer expires, "picks up" the tote (removes from SimEngine)
  4. Validates tote data against PLC struct (metrics tracking)
  5. Returns tote_id so PRG_Main can clean up PLC-side references

The PLC never sees any of this — it just sees the beam break clear
when the tote is removed, and the zone naturally transitions to IDLE.
"""

import random
from dataclasses import dataclass, field
from typing import Any, Optional, Dict, List

from plc.gvl.gvl_config import CFG
from plc.sim.sim_clock import SIM_CLOCK


@dataclass
class _PickupTimer:
    """Active pickup timer for one spur endpoint."""
    zone_id:        str
    spur_index:     Any            # Track name (str) identifying this endpoint
    tote_id:        str
    start_time:     float
    duration_s:     float
    completed:      bool = False


@dataclass
class PickupResult:
    """Returned when a robot completes a pickup."""
    tote_id:          str
    spur_index:       Any           # Track name (str) identifying this endpoint
    zone_id:          str
    plc_tote_id:      str = ""     # What the PLC zone reported
    plc_barcode:      str = ""     # What the PLC struct had
    plc_chute:        int = -1     # What the PLC struct had for chute assignment
    matched:          bool = False # Did sim tote match PLC data?


class SimRobotPickup:
    """
    Manages robot pickup timers for all spur endpoints.
    One instance in PRG_Main.
    """

    def __init__(self) -> None:
        # Active timers: zone_id → _PickupTimer
        self._timers: Dict[str, _PickupTimer] = {}

        # Auto/manual pickup toggle: zone_id → bool (True=auto, default True)
        self.auto_pickup: Dict[str, bool] = {}

        # Queue of manual pickup results to process
        self._manual_queue: List[str] = []  # zone_ids

        # ── Metrics (simulator-level package tracking) ─────────────────────
        self.diTotalDelivered:   int = 0
        self.diTotalMismatched:  int = 0
        self.delivered_log:      List[PickupResult] = []

    def tick(
        self,
        spur_endpoints: Dict[str, dict],
    ) -> List[PickupResult]:
        """
        Called every sim tick. Checks all spur endpoints for tote presence
        and manages pickup timers.

        Args:
            spur_endpoints: Dict of zone_id → {
                "spur_index": int,       # 1-based chute number
                "tote_present": bool,    # SimEngine says tote is here
                "tote_id": str,          # From SimEngine
                "plc_tote_id": str,      # From PLC zone's get_tote_data()
                "plc_barcode": str,      # From PLC zone struct
                "plc_chute": int,        # From PLC zone struct eChuteAssignment
            }

        Returns:
            List of PickupResult for totes picked up this tick.
        """
        results: List[PickupResult] = []
        now = SIM_CLOCK.now()

        # Process manual pickup queue first
        for manual_zone in self._manual_queue:
            if manual_zone in spur_endpoints:
                info = spur_endpoints[manual_zone]
                tote_id = info.get("tote_id", "")
                if tote_id:
                    result = PickupResult(
                        tote_id=tote_id,
                        spur_index=info.get("spur_index", 0),
                        zone_id=manual_zone,
                        plc_tote_id=info.get("plc_tote_id", ""),
                        plc_barcode=info.get("plc_barcode", ""),
                        plc_chute=info.get("plc_chute", -1),
                    )
                    result.matched = (tote_id == result.plc_tote_id)
                    if result.matched:
                        self.diTotalDelivered += 1
                    else:
                        self.diTotalMismatched += 1
                    self.delivered_log.append(result)
                    results.append(result)
                    self._timers.pop(manual_zone, None)
        self._manual_queue.clear()

        for zone_id, info in spur_endpoints.items():
            tote_present = info.get("tote_present", False)
            tote_id      = info.get("tote_id", "")

            # Skip if already picked up manually this tick
            if any(r.zone_id == zone_id for r in results):
                continue

            if tote_present and tote_id:
                # Check auto/manual toggle (default=True=auto)
                is_auto = self.auto_pickup.get(zone_id, True)
                if not is_auto:
                    # Manual mode: don't start/advance timers, tote just sits
                    continue

                # Start timer if not already running for this zone
                if zone_id not in self._timers:
                    duration = random.uniform(
                        CFG.rPickupIntervalMin_s,
                        CFG.rPickupIntervalMax_s,
                    )
                    self._timers[zone_id] = _PickupTimer(
                        zone_id=zone_id,
                        spur_index=info.get("spur_index", 0),
                        tote_id=tote_id,
                        start_time=now,
                        duration_s=duration,
                    )

                # Check if timer expired
                timer = self._timers[zone_id]
                if now - timer.start_time >= timer.duration_s:
                    # Pickup complete — validate against PLC data
                    result = PickupResult(
                        tote_id=tote_id,
                        spur_index=timer.spur_index,
                        zone_id=zone_id,
                        plc_tote_id=info.get("plc_tote_id", ""),
                        plc_barcode=info.get("plc_barcode", ""),
                        plc_chute=info.get("plc_chute", -1),
                    )

                    # Validate: sim tote_id matches PLC tote_id
                    result.matched = (tote_id == result.plc_tote_id)

                    if result.matched:
                        self.diTotalDelivered += 1
                    else:
                        self.diTotalMismatched += 1

                    self.delivered_log.append(result)
                    results.append(result)

                    # Remove timer
                    del self._timers[zone_id]

            else:
                # No tote present — clear any stale timer
                self._timers.pop(zone_id, None)

        return results

    def manual_pickup(self, zone_id: str) -> None:
        """Queue a manual pickup for the given zone. Processed next tick."""
        self._manual_queue.append(zone_id)

    def get_active_timer_count(self) -> int:
        return len(self._timers)

    def get_metrics(self) -> dict:
        return {
            "diTotalDelivered":  self.diTotalDelivered,
            "diTotalMismatched": self.diTotalMismatched,
            "iActiveTimers":     len(self._timers),
        }
