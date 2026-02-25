"""
gvl/gvl_config.py
=================
IEC 61131-3 equivalent: GVL_Config  {attribute 'qualified_only'}

System configuration parameters.
Loaded from config.json at startup.
Can be modified via HMI API when system is STOPPED.
Written by: config loader, HMI (when stopped only)
Read by:    all FBs, PRG_Main
"""

import json
from pathlib import Path
from plc.types.enums import E_MergePriority, E_RoutingStrategy

# Enum fields that need name↔value conversion for JSON
_ENUM_FIELDS = {
    "eMergePriorityDefault": E_MergePriority,
    "eRoutingStrategy":      E_RoutingStrategy,
}


class GVL_Config:
    """
    All values here map directly to CODESYS GVL_Config constants/variables.
    Attribute naming follows IEC 61131-3 Hungarian notation.
    """

    # ── Topology ──────────────────────────────────────────────────────────
    iNumberOfChutes:       int   = 3        # N normal chutes (reject is separate)

    # ── Belt speeds ────────────────────────────────────────────────────────
    rDefaultBeltSpeed:     float = 50.0     # % of max speed (0.0–100.0)

    # ── Timing ─────────────────────────────────────────────────────────────
    rJamTimeout_s:         float = 20.0     # Seconds in RELEASING before system E-Stop (jam)
    rScanDelay_s:          float = 0.2      # Simulated scanner processing time
    rSpawnInterval_s:      float = 3.0      # Auto-spawn rate (seconds between totes)
    rPickupIntervalMin_s:  float = 5.0      # Robot pickup min interval (seconds)
    rPickupIntervalMax_s:  float = 15.0     # Robot pickup max interval (seconds)
    rScanCycleRate_s:      float = 0.1      # PLC scan cycle tick (100ms = 10Hz)

    # ── Tote management ────────────────────────────────────────────────────
    iMaxTotesOnSystem:     int   = 20       # Hard cap; spawn pauses when reached
    iMaxRecircCount:       int   = 5        # Loops before system E-Stop (recirc limit)
    iSpurLength:           int   = 3        # Zones per normal spur (configurable N)
    iMaxScanFails:         int   = 2        # Failed scanner passes before reject
    iRejectSpurLength:     int   = 5        # Zones in reject spur (longer than normal)

    # ── Merge behavior ─────────────────────────────────────────────────────
    eMergePriorityDefault: E_MergePriority = E_MergePriority.INPUT2_PRIORITY

    # ── Routing strategy ───────────────────────────────────────────────────
    eRoutingStrategy:      E_RoutingStrategy = E_RoutingStrategy.LEAST_LOADED

    # ── Physics / simulation ───────────────────────────────────────────────
    rDefaultZoneLength_cm:   float = 60.0   # Physical length of each zone in cm
    rDefaultGap_cm:          float = 2.0    # Gap between zones in cm
    rToteLength_cm:          float = 40.0   # Physical tote length in cm (along belt travel)
    rToteWidth_cm:           float = 30.0   # Physical tote width in cm (perpendicular to travel)
    rDefaultZoneWidth_cm:    float = 60.0   # Belt/zone width in cm (must be >= tote width to accept)
    rBeltSpeedMax_cm_per_s:  float = 120.0  # Max belt speed in cm/s at 100%
    rSlipChance:             float = 0.02   # 2% chance per tick tote doesn't advance
    rScanFailRate:           float = 0.20   # 20% fail rate per attempt at 50% speed
    iScanZoneTransitTicks:   int   = 10     # How many ticks at 100% speed to cross scan zone

    # ── Stuck tote simulation ─────────────────────────────────────────────
    rStuckToteChance:        float = 0.0005 # Per-tick probability of tote getting stuck (0.05%)
    rOvercurrentChance:      float = 0.25   # Chance a stuck tote causes motor overcurrent (vs physical jam)
    rStuckSelfClearChance:   float = 0.25   # Per-tick chance a physically jammed tote self-clears
    iRoutingDelayMin_ticks:  int   = 1      # Min ticks before routing result appears on struct
    iRoutingDelayMax_ticks:  int   = 5      # Max ticks before routing result appears on struct

    def to_dict(self) -> dict:
        """Export all config values to a JSON-safe dict."""
        return {
            # Topology
            "iNumberOfChutes":       self.iNumberOfChutes,
            # Belt speeds
            "rDefaultBeltSpeed":     self.rDefaultBeltSpeed,
            # Timing
            "rJamTimeout_s":         self.rJamTimeout_s,
            "rScanDelay_s":          self.rScanDelay_s,
            "rSpawnInterval_s":      self.rSpawnInterval_s,
            "rPickupIntervalMin_s":  self.rPickupIntervalMin_s,
            "rPickupIntervalMax_s":  self.rPickupIntervalMax_s,
            "rScanCycleRate_s":      self.rScanCycleRate_s,
            # Tote management
            "iMaxTotesOnSystem":     self.iMaxTotesOnSystem,
            "iMaxRecircCount":       self.iMaxRecircCount,
            "iSpurLength":           self.iSpurLength,
            "iMaxScanFails":         self.iMaxScanFails,
            "iRejectSpurLength":     self.iRejectSpurLength,
            # Merge / routing
            "eMergePriorityDefault": self.eMergePriorityDefault.name,
            "eRoutingStrategy":      self.eRoutingStrategy.name,
            # Physics / simulation
            "rDefaultZoneLength_cm":   self.rDefaultZoneLength_cm,
            "rDefaultGap_cm":          self.rDefaultGap_cm,
            "rToteLength_cm":          self.rToteLength_cm,
            "rToteWidth_cm":           self.rToteWidth_cm,
            "rDefaultZoneWidth_cm":    self.rDefaultZoneWidth_cm,
            "rBeltSpeedMax_cm_per_s":  self.rBeltSpeedMax_cm_per_s,
            "rSlipChance":             self.rSlipChance,
            "rScanFailRate":           self.rScanFailRate,
            "iScanZoneTransitTicks":   self.iScanZoneTransitTicks,
            "iRoutingDelayMin_ticks":  self.iRoutingDelayMin_ticks,
            "iRoutingDelayMax_ticks":  self.iRoutingDelayMax_ticks,
            # Stuck tote simulation
            "rStuckToteChance":        self.rStuckToteChance,
            "rOvercurrentChance":      self.rOvercurrentChance,
            "rStuckSelfClearChance":   self.rStuckSelfClearChance,
        }

    def update_from_dict(self, d: dict) -> None:
        """Apply a partial dict of config updates. Handles enum name→value conversion."""
        for key, val in d.items():
            if not hasattr(self, key):
                continue
            if key in _ENUM_FIELDS:
                setattr(self, key, _ENUM_FIELDS[key][val])
            else:
                setattr(self, key, val)

    def save_to_file(self, path: str) -> None:
        """Persist current config to a JSON file."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")

    def load_from_file(self, path: str) -> bool:
        """
        Load config from a JSON file. Returns True if file was found and loaded.
        Missing keys keep their current (default) values.
        """
        p = Path(path)
        if not p.exists():
            return False
        data = json.loads(p.read_text(encoding="utf-8"))
        self.update_from_dict(data)
        return True


# Singleton — imported everywhere as: from plc.gvl.gvl_config import CFG
CFG = GVL_Config()
