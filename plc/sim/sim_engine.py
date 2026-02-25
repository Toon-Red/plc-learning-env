"""
sim/sim_engine.py
=================
Physics simulation layer. Runs BEFORE the PLC layer every tick.

Responsibilities:
  - Track every tote's position in cm (floating point)
  - Move totes each tick based on the motor state of the zone they're in
  - Apply random belt slip
  - Update bBeamBreak signals in GVL_IO based on tote positions vs sensor positions
  - Tell FBs which tote is in which zone (for scan mismatch detection)

The PLC FBs see ONLY beam break states — they have no knowledge of position.
This mirrors real hardware: the sensor is the boundary between physics and logic.

Execution contract (called by PRG_Main each tick):
    sim.tick(motor_states)   # motor_states = {zone_id: bool}

Zone registration (called by PRG_Main at startup):
    sim.register_zone(zone_id, start_cm, length_cm, sensor_offset_cm, gap_after_cm)

Tote spawn/removal:
    sim.spawn_tote(tote_id, zone_id)    # place tote at start of zone
    sim.remove_tote(tote_id)            # robot pickup or manual removal
"""

import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from plc.gvl.gvl_config import CFG

# Import after module-level to avoid circular; used in dataclass default_factory
from plc.sim.sim_clock import SIM_CLOCK


@dataclass
class SimZone:
    """
    Physical layout descriptor for one conveyor zone.
    Registered at startup by PRG_Main.
    """
    zone_id:          str
    start_cm:         float          # Absolute position of zone start on the line
    length_cm:        float          # Physical length of this zone (along belt travel)
    width_cm:         float          # Belt width (perpendicular to travel)
    sensor_offset_cm: float          # Beam break position relative to zone start
    gap_after_cm:     float          # Gap between this zone's end and the next zone's start

    @property
    def end_cm(self) -> float:
        return self.start_cm + self.length_cm

    @property
    def sensor_cm(self) -> float:
        """Absolute position of the beam break sensor."""
        return self.start_cm + self.sensor_offset_cm

    @property
    def next_zone_start_cm(self) -> float:
        return self.end_cm + self.gap_after_cm


@dataclass
class SimTote:
    """
    Physics state of one tote. Lives only in SimEngine — never in ST_ToteData.

    position_cm: absolute position of the tote's FRONT (leading/downstream edge).
    The tote body occupies [position_cm - tote_length, position_cm].
    Movement is in the positive cm direction.
    """
    tote_id:         str
    position_cm:     float          # Front (leading) edge of tote
    tote_length_cm:  float          # Physical tote length (from GVL_Config at spawn)
    owner_zone_id:   str            # Zone currently "responsible" for this tote
    spawned_at:      float = field(default_factory=lambda: SIM_CLOCK.now())

    @property
    def rear_cm(self) -> float:
        """Trailing (upstream) edge of tote."""
        return self.position_cm - self.tote_length_cm

    def is_sensor_triggered(self, sensor_cm: float) -> bool:
        """
        Returns True if this tote's body overlaps the sensor position.
        Sensor fires if: rear_cm <= sensor_cm <= front_cm
        Handles both normal and spanning cases.
        """
        return self.rear_cm <= sensor_cm <= self.position_cm


class SimEngine:
    """
    Physics simulation engine.

    Zone order matters: zones must be registered in conveyor-line order
    (upstream to downstream) so the engine can determine ownership and
    compute transit windows correctly.
    """

    def __init__(self) -> None:
        # Ordered list of zones as registered (upstream → downstream)
        self._zones: List[SimZone] = []
        # zone_id → SimZone for fast lookup
        self._zone_map: Dict[str, SimZone] = {}
        # tote_id → SimTote
        self._totes: Dict[str, SimTote] = {}

        # Output: zone_id → beam_break_state (written each tick, read by PLC FBs)
        self.beam_breaks: Dict[str, bool] = {}

        # Output: zone_id → tote_id (None if zone empty) — for scan mismatch detection
        self.zone_tote_map: Dict[str, Optional[str]] = {}

    # ── Zone registration ──────────────────────────────────────────────────

    def register_zone(
        self,
        zone_id:          str,
        start_cm:         float,
        length_cm:        float,
        width_cm:         Optional[float] = None,   # None = use default
        sensor_offset_cm: Optional[float] = None,   # None = center
        gap_after_cm:     float = None,
    ) -> None:
        """
        Register a zone with the simulation engine.
        Must be called in upstream→downstream order at startup.
        width_cm: belt/zone width. Totes wider than this cannot enter this zone.
        """
        if width_cm is None:
            width_cm = CFG.rDefaultZoneWidth_cm
        if sensor_offset_cm is None:
            sensor_offset_cm = length_cm / 2.0
        if gap_after_cm is None:
            gap_after_cm = CFG.rDefaultGap_cm

        zone = SimZone(
            zone_id=zone_id,
            start_cm=start_cm,
            length_cm=length_cm,
            width_cm=width_cm,
            sensor_offset_cm=sensor_offset_cm,
            gap_after_cm=gap_after_cm,
        )
        self._zones.append(zone)
        self._zone_map[zone_id] = zone
        self.beam_breaks[zone_id] = False
        self.zone_tote_map[zone_id] = None

    def get_zone(self, zone_id: str) -> Optional[SimZone]:
        return self._zone_map.get(zone_id)

    def get_zone_sequence(self) -> List[str]:
        """Returns zone IDs in registered order (upstream to downstream)."""
        return [z.zone_id for z in self._zones]

    # ── Tote lifecycle ─────────────────────────────────────────────────────

    def spawn_tote(self, tote_id: str, zone_id: str) -> bool:
        """
        Place a new tote at the START of the given zone.
        Returns False if zone already has a tote (spawn blocked).
        """
        if self.zone_tote_map.get(zone_id) is not None:
            return False

        zone = self._zone_map.get(zone_id)
        if zone is None:
            return False

        # Width check: tote must fit on this belt
        if CFG.rToteWidth_cm > zone.width_cm:
            return False   # Tote physically too wide for this zone

        # Tote front starts at zone start (just entering)
        tote = SimTote(
            tote_id=tote_id,
            position_cm=zone.start_cm + CFG.rToteLength_cm,  # front is tote_length in
            tote_length_cm=CFG.rToteLength_cm,
            owner_zone_id=zone_id,
        )
        self._totes[tote_id] = tote
        self.zone_tote_map[zone_id] = tote_id   # Immediate map update (don't wait for tick)
        return True

    def remove_tote(self, tote_id: str) -> None:
        """Remove a tote from simulation (robot pickup, or fault removal)."""
        tote = self._totes.pop(tote_id, None)
        if tote:
            self._clear_zone_tote(tote_id)

    def get_tote_in_zone(self, zone_id: str) -> Optional[str]:
        """Returns tote_id of tote currently in zone, or None."""
        return self.zone_tote_map.get(zone_id)

    def get_tote_position(self, tote_id: str) -> Optional[float]:
        """Returns front-edge position in cm, or None if tote not found."""
        tote = self._totes.get(tote_id)
        return tote.position_cm if tote else None

    # ── Tick ──────────────────────────────────────────────────────────────

    def tick(self, motor_states: Dict[str, bool], speed_pct: Dict[str, float]) -> None:
        """
        Advance simulation one tick.
        Called by PRG_Main BEFORE the PLC layer executes.

        motor_states: zone_id → bool (TRUE = belt running)
        speed_pct:    zone_id → float (0–100, belt speed percentage)
        """
        self._move_totes(motor_states, speed_pct)
        self._update_ownership()
        self._update_beam_breaks()

    # ── Internal ──────────────────────────────────────────────────────────

    def _move_totes(
        self,
        motor_states: Dict[str, bool],
        speed_pct:    Dict[str, float],
    ) -> None:
        """
        Move each tote forward by speed * tick_rate if its owner zone motor is running.
        If owner motor is OFF, check the next zone downstream — if its motor is ON,
        the downstream belt pulls the tote forward (simulates real conveyor handoff).
        Apply random slip: CFG.rSlipChance chance the tote doesn't advance this tick.
        Clamp tote position at track end (physical bumper at last zone).
        Collision detection: totes cannot pass through the tote immediately ahead.
        """
        tick_duration_s = CFG.rScanCycleRate_s
        max_speed = CFG.rBeltSpeedMax_cm_per_s

        # Track boundary: end of the last registered zone
        track_end = self._zones[-1].end_cm if self._zones else 0.0

        # Process downstream totes first so their final positions are known
        # when upstream totes check for collisions.
        sorted_totes = sorted(self._totes.values(), key=lambda t: -t.position_cm)

        for tote in sorted_totes:
            owner = tote.owner_zone_id
            drive_zone = owner

            # If owner motor is off, check downstream zone (belt pull).
            # Belt pull only applies when the tote front has reached or passed
            # the owner zone's end — i.e. the tote is physically exiting the zone
            # into the gap/next zone. If the tote front is still inside the zone,
            # the downstream belt can't reach it (each zone has its own belt).
            if not motor_states.get(owner, False):
                owner_zone = self._zone_map.get(owner)
                if owner_zone and tote.position_cm >= owner_zone.end_cm:
                    next_zone = self._get_next_zone(owner)
                    if next_zone and motor_states.get(next_zone, False):
                        drive_zone = next_zone
                    else:
                        continue
                else:
                    continue

            # Belt slip: random chance of zero movement this tick
            if random.random() < CFG.rSlipChance:
                continue

            pct   = speed_pct.get(drive_zone, CFG.rDefaultBeltSpeed) / 100.0
            delta = max_speed * pct * tick_duration_s
            new_pos = tote.position_cm + delta

            # Clamp at track end — tote can't advance past the physical bumper
            if new_pos > track_end:
                new_pos = track_end

            # Collision: can't advance past the rear of the nearest tote ahead
            for other in sorted_totes:
                if other is tote:
                    continue
                if other.position_cm > tote.position_cm:
                    gap_limit = other.rear_cm - 0.5  # 0.5 cm min gap
                    if new_pos > gap_limit:
                        new_pos = max(tote.position_cm, gap_limit)

            tote.position_cm = new_pos

    def _update_ownership(self) -> None:
        """
        Determine which zone owns each tote based on current position.
        A zone owns a tote when the tote's center is within that zone's bounds.
        If center is in the gap between zones, ownership stays with the upstream zone
        until the tote reaches the next zone's start.
        """
        # Reset zone→tote mapping
        for zone_id in self.zone_tote_map:
            self.zone_tote_map[zone_id] = None

        for tote in self._totes.values():
            center_cm = tote.position_cm - (tote.tote_length_cm / 2.0)
            new_owner = self._find_zone_for_position(center_cm, tote.owner_zone_id)
            if new_owner and new_owner != tote.owner_zone_id:
                tote.owner_zone_id = new_owner

            owner = tote.owner_zone_id
            if self.zone_tote_map.get(owner) is None:
                self.zone_tote_map[owner] = tote.tote_id
            # If two totes somehow end up in the same zone, that's a critical
            # simulation error — the PLC layer should prevent this via its own logic

    def _get_next_zone(self, zone_id: str) -> Optional[str]:
        """Return the zone_id of the next zone downstream, or None if last."""
        for i, zone in enumerate(self._zones):
            if zone.zone_id == zone_id:
                if i + 1 < len(self._zones):
                    return self._zones[i + 1].zone_id
                return None
        return None

    def _find_zone_for_position(
        self, center_cm: float, current_owner: str
    ) -> Optional[str]:
        """
        Find the zone whose bounds contain center_cm.
        If in a gap, returns the current owner (tote still belongs to last zone).
        """
        for zone in self._zones:
            if zone.start_cm <= center_cm <= zone.end_cm:
                return zone.zone_id

        # In a gap — keep current owner
        return current_owner

    def _update_beam_breaks(self) -> None:
        """
        For each zone, determine if any tote's body overlaps its sensor.
        A beam break is TRUE if sensor_cm is within [tote.rear_cm, tote.position_cm].
        Multiple totes can only trigger one zone's sensor at a time in normal operation,
        but a large tote CAN span two zones and trigger both simultaneously.
        """
        # Reset all beam breaks
        for zone_id in self.beam_breaks:
            self.beam_breaks[zone_id] = False

        for tote in self._totes.values():
            for zone in self._zones:
                if tote.is_sensor_triggered(zone.sensor_cm):
                    self.beam_breaks[zone.zone_id] = True

    def _clear_zone_tote(self, tote_id: str) -> None:
        for zone_id, tid in self.zone_tote_map.items():
            if tid == tote_id:
                self.zone_tote_map[zone_id] = None
                break

    # ── Accessors (named methods — storage can change underneath) ─────────

    def get_beam_break(self, zone_id: str) -> bool:
        """Read beam break state for a zone. Used by PLC FBs."""
        return self.beam_breaks.get(zone_id, False)

    def is_zone_occupied(self, zone_id: str) -> bool:
        """True if any tote is currently in this zone."""
        return self.zone_tote_map.get(zone_id) is not None

    def active_tote_count(self) -> int:
        return len(self._totes)

    def get_all_positions(self) -> Dict[str, float]:
        """Returns tote_id → front position in cm. For HMI/debug."""
        return {tid: t.position_cm for tid, t in self._totes.items()}

    def to_dict(self) -> dict:
        """Full snapshot for HMI WebSocket push."""
        return {
            "totes": {
                tid: {
                    "position_cm":   t.position_cm,
                    "rear_cm":       t.rear_cm,
                    "owner_zone_id": t.owner_zone_id,
                }
                for tid, t in self._totes.items()
            },
            "beam_breaks":    dict(self.beam_breaks),
            "zone_tote_map":  dict(self.zone_tote_map),
            "active_count":   len(self._totes),
        }


# Singleton
SIM = SimEngine()
