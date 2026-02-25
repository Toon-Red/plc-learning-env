"""
gvl/gvl_alarms.py
=================
IEC 61131-3 equivalent: GVL_Alarms  {attribute 'qualified_only'}

Circular alarm buffer. Written by FBs via raise_alarm().
Read by HMI for display and acknowledgement.
"""

import threading
import time
from collections import deque
from typing import List

from plc.types.structs import ST_AlarmEntry
from plc.types.enums import E_AlarmSeverity


# ── Alarm code registry ────────────────────────────────────────────────────
# Matches Section 9 of REQUIREMENTS.md
ALARM_CODES = {
    1:  "Jam detected",                             # A001 FAULT
    2:  "Spur full — tote passed through",          # A002 WARNING  (recirculates, not a stop)
    3:  "No-read: barcode not found",               # A003 WARNING
    4:  "Unknown barcode — assigned RECIRCULATE",   # A004 WARNING
    5:  "E-Stop activated",                         # A005 CRITICAL
    6:  "Recirculate limit exceeded",               # A006 FAULT
    7:  "Max totes on system — spawn paused",       # A007 WARNING
    8:  "Merge deadlock",                           # A008 FAULT
    10: "Scan mismatch — unexpected tote",          # A010 CRITICAL
    11: "Motor overcurrent — stuck tote",           # A011 CRITICAL
    12: "Reject spur full — system blocked",        # A012 FAULT
    13: "Tote rejected — scan fail limit reached",  # A013 WARNING
}

ALARM_SEVERITIES = {
    1:  E_AlarmSeverity.FAULT,
    2:  E_AlarmSeverity.WARNING,
    3:  E_AlarmSeverity.WARNING,
    4:  E_AlarmSeverity.WARNING,
    5:  E_AlarmSeverity.CRITICAL,
    6:  E_AlarmSeverity.FAULT,
    7:  E_AlarmSeverity.WARNING,
    8:  E_AlarmSeverity.FAULT,
    10: E_AlarmSeverity.CRITICAL,
    11: E_AlarmSeverity.CRITICAL,
    12: E_AlarmSeverity.FAULT,
    13: E_AlarmSeverity.WARNING,
}

BUFFER_SIZE = 100


class GVL_Alarms:
    """
    Thread-safe alarm registry.
    aAlarmBuffer is a deque acting as a circular buffer (newest at index 0).
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.aAlarmBuffer: deque = deque(maxlen=BUFFER_SIZE)
        self.diActiveAlarmCount: int = 0
        self.diUnackAlarmCount:  int = 0

    def raise_alarm(self, code: int, zone_id: str = "", extra: str = "") -> None:
        """
        Called by FBs to raise an alarm.
        Does not raise duplicates for the same code+zone if still unacknowledged.
        """
        with self._lock:
            # Deduplicate: don't re-raise if same code+zone is already active+unacked
            for entry in self.aAlarmBuffer:
                if entry.iAlarmCode == code and entry.sZoneID == zone_id and not entry.bAcknowledged:
                    return

            message = ALARM_CODES.get(code, f"Unknown alarm {code}")
            if extra:
                message = f"{message}: {extra}"

            entry = ST_AlarmEntry(
                sTimestamp=time.strftime("%H:%M:%S"),
                iAlarmCode=code,
                sZoneID=zone_id,
                sMessage=message,
                eSeverity=ALARM_SEVERITIES.get(code, E_AlarmSeverity.WARNING),
                bAcknowledged=False,
            )
            self.aAlarmBuffer.appendleft(entry)
            self._update_counts()

    def acknowledge(self, index: int) -> None:
        with self._lock:
            lst = list(self.aAlarmBuffer)
            if 0 <= index < len(lst):
                lst[index].bAcknowledged = True
                self.aAlarmBuffer = deque(lst, maxlen=BUFFER_SIZE)
                self._update_counts()

    def acknowledge_all(self) -> None:
        with self._lock:
            for entry in self.aAlarmBuffer:
                entry.bAcknowledged = True
            self._update_counts()

    def clear_resolved(self) -> None:
        """Remove acknowledged alarms from buffer."""
        with self._lock:
            self.aAlarmBuffer = deque(
                [e for e in self.aAlarmBuffer if not e.bAcknowledged],
                maxlen=BUFFER_SIZE,
            )
            self._update_counts()

    def _update_counts(self) -> None:
        self.diActiveAlarmCount = len(self.aAlarmBuffer)
        self.diUnackAlarmCount  = sum(1 for e in self.aAlarmBuffer if not e.bAcknowledged)

    def has_critical(self) -> bool:
        with self._lock:
            return any(
                e.eSeverity == E_AlarmSeverity.CRITICAL and not e.bAcknowledged
                for e in self.aAlarmBuffer
            )

    def to_dict(self) -> dict:
        with self._lock:
            return {
                "diActiveAlarmCount": self.diActiveAlarmCount,
                "diUnackAlarmCount":  self.diUnackAlarmCount,
                "aAlarmBuffer": [e.to_dict() for e in self.aAlarmBuffer],
            }


# Singleton
ALARMS = GVL_Alarms()
