"""
types/structs.py
================
IEC 61131-3 equivalent: TYPE ... END_TYPE (STRUCT)

Naming convention mirrors CODESYS: ST_<Name>
Using Python dataclasses — translates directly to CODESYS STRUCT blocks.
All fields have explicit types and defaults, matching VAR declarations in ST.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, List, Any
import time
import uuid

from plc.types.enums import E_ChuteTarget, E_AlarmSeverity
from plc.sim.sim_clock import SIM_CLOCK


@dataclass
class ST_ToteData:
    """
    IEC 61131-3 equivalent:

        TYPE ST_ToteData :
        STRUCT
            sToteID          : STRING;
            sBarcode         : STRING;
            eChuteAssignment : E_ChuteTarget;
            iCurrentZone     : INT;
            tEnteredSystem   : LREAL;
            tLastScan        : LREAL;
            bScanComplete    : BOOL;
            iRecircCount     : INT;
            bIsActive        : BOOL;
        END_STRUCT
        END_TYPE

    Passed between FBs as a value — no logic inside this struct.
    The FB that holds the tote owns a copy; passing downstream = copy.
    """
    sToteID:          str          = field(default_factory=lambda: str(uuid.uuid4())[:8].upper())
    sBarcode:         str          = ""
    eChuteAssignment: E_ChuteTarget = E_ChuteTarget.UNASSIGNED
    iCurrentZone:     int          = -1   # -1 = not yet placed
    tEnteredSystem:   float        = field(default_factory=lambda: SIM_CLOCK.now())
    tLastScan:        float        = 0.0
    bScanComplete:    bool         = False
    iRecircCount:     int          = 0
    iScanFailCount:   int          = 0    # Failed scanner passes (2 = reject)
    bIsActive:        bool         = True

    def to_dict(self) -> dict:
        """Serialize for GVL / HMI consumption."""
        return {
            "sToteID":          self.sToteID,
            "sBarcode":         self.sBarcode,
            "eChuteAssignment": self.eChuteAssignment.name,
            "iCurrentZone":     self.iCurrentZone,
            "tEnteredSystem":   self.tEnteredSystem,
            "tLastScan":        self.tLastScan,
            "bScanComplete":    self.bScanComplete,
            "iRecircCount":     self.iRecircCount,
            "iScanFailCount":   self.iScanFailCount,
            "bIsActive":        self.bIsActive,
        }

    @classmethod
    def empty(cls) -> "ST_ToteData":
        """Returns an inactive/empty tote (represents 'no tote present')."""
        t = cls()
        t.bIsActive = False
        t.sToteID   = ""
        return t


@dataclass
class ST_AlarmEntry:
    """
    IEC 61131-3 equivalent:

        TYPE ST_AlarmEntry :
        STRUCT
            sTimestamp   : STRING;
            iAlarmCode   : INT;
            sZoneID      : STRING;
            sMessage     : STRING;
            eSeverity    : E_AlarmSeverity;
            bAcknowledged: BOOL;
        END_STRUCT
        END_TYPE
    """
    sTimestamp:    str             = field(default_factory=lambda: time.strftime("%H:%M:%S"))
    iAlarmCode:    int             = 0
    sZoneID:       str             = ""
    sMessage:      str             = ""
    eSeverity:     E_AlarmSeverity = E_AlarmSeverity.WARNING
    bAcknowledged: bool            = False

    def to_dict(self) -> dict:
        return {
            "sTimestamp":    self.sTimestamp,
            "iAlarmCode":    self.iAlarmCode,
            "sZoneID":       self.sZoneID,
            "sMessage":      self.sMessage,
            "eSeverity":     self.eSeverity.name,
            "bAcknowledged": self.bAcknowledged,
        }


@dataclass
class ST_ZoneStatus:
    """
    Snapshot of a single conveyor zone's status for GVL_IO.
    Written by FB_ConveyorZone every scan, read by HMI.

        TYPE ST_ZoneStatus :
        STRUCT
            sZoneID      : STRING;
            eState       : E_ConveyorState;
            bMotorRun    : BOOL;
            bTotePresent : BOOL;
            bFaulted     : BOOL;
            rActualSpeed : REAL;
            sToteID      : STRING;
            sBarcode     : STRING;
        END_STRUCT
        END_TYPE
    """
    sZoneID:      str   = ""
    eState:       int   = 0     # E_ConveyorState value (int for easy JSON)
    bMotorRun:    bool  = False
    bTotePresent: bool  = False
    bFaulted:     bool  = False
    rActualSpeed: float = 0.0
    sToteID:      str   = ""
    sBarcode:     str   = ""

    def to_dict(self) -> dict:
        return {
            "sZoneID":      self.sZoneID,
            "eState":       self.eState,
            "bMotorRun":    self.bMotorRun,
            "bTotePresent": self.bTotePresent,
            "bFaulted":     self.bFaulted,
            "rActualSpeed": self.rActualSpeed,
            "sToteID":      self.sToteID,
            "sBarcode":     self.sBarcode,
        }


# ══════════════════════════════════════════════════════════════════════════════
# TOPOLOGY DESCRIPTOR — RCT2-style segment-based track building
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class ST_ZoneDescriptor:
    """
    IEC 61131-3 equivalent:

        TYPE ST_ZoneDescriptor :
        STRUCT
            sZoneID           : STRING;
            eZoneType         : STRING;
            rLength_cm        : LREAL;
            (* ... per-object config ... *)
            sDownstream       : STRING;
            sDivertDownstream : STRING;
            sUpstream2        : STRING;
            sTrackGroup       : STRING;
        END_STRUCT
        END_TYPE

    Flat descriptor for a single zone in the topology.
    String-based connections. Per-object config uses None = use CFG default.
    Simple to serialize as JSON.
    """
    sZoneID:           str                          # Unique ID
    eZoneType:         str                          # "transport", "branch", "merge", "scanner", "infeed", "end"

    # Per-object config (None = use global default)
    rLength_cm:        Optional[float] = None       # Zone length
    rGap_cm:           Optional[float] = None       # Gap to next zone
    rWidth_cm:         Optional[float] = None       # Belt width
    rSensorOffset_cm:  Optional[float] = None       # Sensor position (None = center)
    rMaxSpeed:         Optional[float] = None       # Max belt speed %

    # Type-specific params
    iTargetChute:      Optional[int]   = None       # Branch: which chute number
    eMergePriority:    Optional[str]   = None       # Merge: priority mode name

    # Behavior flags
    bIsSpawnPoint:     bool = False                  # Totes can spawn here
    bIsPickupPoint:    bool = False                  # Robot/human pickup here
    bForceRelease:     bool = False                  # End-of-track force release

    # Connection ports (zone ID strings)
    sDownstream:       Optional[str] = None          # Primary downstream
    sDivertDownstream: Optional[str] = None          # Branch: divert output
    sUpstream2:        Optional[str] = None          # Merge: second input source

    # Visual / grouping
    sTrackGroup:       str = "main"                  # SimEngine track group name

    def to_dict(self) -> dict:
        """Serialize for JSON."""
        d = {
            "sZoneID":           self.sZoneID,
            "eZoneType":         self.eZoneType,
            "sTrackGroup":       self.sTrackGroup,
        }
        # Only include non-None optional fields
        for fld in [
            "rLength_cm", "rGap_cm", "rWidth_cm", "rSensorOffset_cm",
            "rMaxSpeed", "iTargetChute", "eMergePriority",
        ]:
            val = getattr(self, fld)
            if val is not None:
                d[fld] = val
        # Always include flags if True
        for fld in ["bIsSpawnPoint", "bIsPickupPoint", "bForceRelease"]:
            val = getattr(self, fld)
            if val:
                d[fld] = val
        # Include connections if set (sUpstream2 no longer serialized — compiler infers merge inputs)
        for fld in ["sDownstream", "sDivertDownstream"]:
            val = getattr(self, fld)
            if val is not None:
                d[fld] = val
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "ST_ZoneDescriptor":
        """Deserialize from JSON dict."""
        return cls(
            sZoneID=d["sZoneID"],
            eZoneType=d["eZoneType"],
            rLength_cm=d.get("rLength_cm"),
            rGap_cm=d.get("rGap_cm"),
            rWidth_cm=d.get("rWidth_cm"),
            rSensorOffset_cm=d.get("rSensorOffset_cm"),
            rMaxSpeed=d.get("rMaxSpeed"),
            iTargetChute=d.get("iTargetChute"),
            eMergePriority=d.get("eMergePriority"),
            bIsSpawnPoint=d.get("bIsSpawnPoint", False),
            bIsPickupPoint=d.get("bIsPickupPoint", False),
            bForceRelease=d.get("bForceRelease", False),
            sDownstream=d.get("sDownstream"),
            sDivertDownstream=d.get("sDivertDownstream"),
            sUpstream2=d.get("sUpstream2"),
            sTrackGroup=d.get("sTrackGroup", "main"),
        )


@dataclass
class ST_CrossTrackTransfer:
    """
    Describes a tote handoff point between two SimEngine tracks.
    Auto-generated by TopologyBuilder where sDownstream/sDivertDownstream
    crosses track group boundaries.
    """
    sSourceZone:   str     # Zone releasing the tote (end of source track)
    sTargetZone:   str     # Zone accepting the tote (start of target track)
    sSourceTrack:  str     # SimEngine track group of source
    sTargetTrack:  str     # SimEngine track group of target
    sTransferKey:  str = ""  # Unique key "{source}→{target}"

    def __post_init__(self):
        if not self.sTransferKey:
            self.sTransferKey = f"{self.sSourceZone}→{self.sTargetZone}"
