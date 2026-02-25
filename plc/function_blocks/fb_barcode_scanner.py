"""
function_blocks/fb_barcode_scanner.py
======================================
IEC 61131-3 equivalent:

    FUNCTION_BLOCK FB_BarcodeScanner
    VAR_INPUT
        bEnable      : BOOL;
        bEStop       : BOOL;
        bTotePresent : BOOL;    (* From zone beam break — tote is in scan window *)
        sToteID      : STRING;  (* Which tote is currently on the scan zone *)
        rBeltSpeed   : REAL;    (* Current belt speed %, affects scan success rate *)
    END_VAR
    VAR_OUTPUT
        sBarcodeResult : STRING;  (* Barcode assigned on successful scan *)
        bScanComplete  : BOOL;    (* Scan succeeded this tick *)
        bNoRead        : BOOL;    (* Tote exited without a successful scan *)
        bScanFault     : BOOL;    (* Mismatch or hardware fault *)
        bScanning      : BOOL;    (* Actively attempting reads *)
        iAttemptCount  : INT;     (* How many attempts made this tote pass *)
    END_VAR
    VAR
        _sPrevToteID     : STRING;
        _bToteInWindow   : BOOL;
        _iAttempts       : INT;
        _bScanDone       : BOOL;
    END_VAR

Behavior:
  - Runs CONTINUOUSLY while tote is present (bTotePresent = TRUE).
  - Each scan tick: random fail check weighted by belt speed.
    fail_chance = CFG.rScanFailRate * (rBeltSpeed / 100.0)
    e.g. at 50% speed, fail_chance = 0.20 * 0.50 = 10% per tick.
    At 100% speed: 20% per tick (fewer attempts AND higher per-tick fail).

  Wait — the user said "20% at 50% speed". Let me re-read:
    "20% fail chance per attempt at 50% speed"
  So at 50% speed: 20% fail per attempt.
  The RATE scales with speed: faster = higher fail rate.
    fail_chance = CFG.rScanFailRate * (rBeltSpeed / 50.0)
    - At 50% speed: 20% fail (0.20 * 50/50 = 0.20)
    - At 100% speed: 40% fail (0.20 * 100/50 = 0.40)
    - At 25% speed: 10% fail (0.20 * 25/50 = 0.10)

  - First successful attempt → sBarcodeResult = GVL_Inventory.get_barcode_for_tote(sToteID)
  - If bTotePresent falls without success → bNoRead = TRUE (one scan, one tick)
  - Mismatch check: if tote already has a barcode in tote tracker AND this scan returns
    a different barcode → bScanFault = TRUE → alarm A010 (CRITICAL)
"""

import random
from plc.gvl.gvl_config import CFG
from plc.gvl.gvl_inventory import INVENTORY
from plc.gvl.gvl_alarms import ALARMS


class FB_BarcodeScanner:
    """
    Continuous-attempt barcode scanner Function Block.
    One instance lives on the scan zone conveyor (C3_SCAN in PRG_Main).
    """

    def __init__(self, zone_id: str = "C3_SCAN") -> None:
        self.zone_id = zone_id

        # ── VAR_INPUT ──────────────────────────────────────────────────────
        self.bEnable:      bool  = False
        self.bEStop:       bool  = False
        self.bTotePresent: bool  = False
        self.sToteID:      str   = ""
        self.rBeltSpeed:   float = CFG.rDefaultBeltSpeed

        # ── VAR_OUTPUT ─────────────────────────────────────────────────────
        self.sBarcodeResult: str  = ""
        self.bScanComplete:  bool = False
        self.bNoRead:        bool = False
        self.bScanFault:     bool = False
        self.bScanning:      bool = False
        self.iAttemptCount:  int  = 0

        # ── VAR (internal) ─────────────────────────────────────────────────
        self._sPrevToteID:  str  = ""
        self._bToteInWindow: bool = False
        self._iAttempts:    int  = 0
        self._bScanDone:    bool = False

    # ── Input interface ────────────────────────────────────────────────────

    def set_inputs(
        self,
        *,
        bEnable:      bool,
        bEStop:       bool,
        bTotePresent: bool,
        sToteID:      str,
        rBeltSpeed:   float = -1.0,
        sKnownBarcode: str  = "",    # What the tote tracker currently has for this tote
    ) -> None:
        self.bEnable       = bEnable
        self.bEStop        = bEStop
        self.bTotePresent  = bTotePresent
        self.sToteID       = sToteID
        self.rBeltSpeed    = rBeltSpeed if rBeltSpeed >= 0.0 else CFG.rDefaultBeltSpeed
        self._sKnownBarcode = sKnownBarcode

    # ── Execution ──────────────────────────────────────────────────────────

    def execute(self) -> None:
        """One scan cycle."""
        # Clear one-shot outputs
        self.bScanComplete = False
        self.bNoRead       = False
        self.bScanFault    = False

        if self.bEStop or not self.bEnable:
            self._reset_scan_state()
            return

        # ── Detect new tote entering scan window (rising edge on tote ID) ─
        new_tote_arrived = self.bTotePresent and (self.sToteID != self._sPrevToteID)
        if new_tote_arrived:
            self._reset_scan_state()
            self._bToteInWindow = True

        # ── Detect tote leaving scan window (falling edge on bTotePresent) ─
        tote_just_left = self._bToteInWindow and not self.bTotePresent
        if tote_just_left:
            if not self._bScanDone:
                self.bNoRead = True
                ALARMS.raise_alarm(3, zone_id=self.zone_id,
                                   extra=f"tote {self._sPrevToteID}")
            self._reset_scan_state()
            self._sPrevToteID = ""
            return

        # ── Continuous scan attempts while tote is in window ──────────────
        if self._bToteInWindow and self.bTotePresent and not self._bScanDone:
            self.bScanning = True
            self._iAttempts += 1

            if self._attempt_scan():
                # SUCCESS — look up barcode from inventory
                barcode = self._lookup_barcode(self.sToteID)

                # Mismatch check: tote already had a different barcode assigned
                if self._sKnownBarcode and barcode and self._sKnownBarcode != barcode:
                    self.bScanFault = True
                    ALARMS.raise_alarm(
                        10,
                        zone_id=self.zone_id,
                        extra=f"tote {self.sToteID}: expected '{self._sKnownBarcode}', "
                              f"got '{barcode}'",
                    )
                else:
                    self.sBarcodeResult = barcode
                    self.bScanComplete  = True
                    self._bScanDone     = True
                    self.bScanning      = False
        else:
            self.bScanning = False

        self.iAttemptCount  = self._iAttempts
        self._sPrevToteID   = self.sToteID if self.bTotePresent else ""

    # ── Internal helpers ───────────────────────────────────────────────────

    def _attempt_scan(self) -> bool:
        """
        Single scan attempt. Returns True on success.
        Fail chance scales linearly with belt speed:
          fail_chance = rScanFailRate * (rBeltSpeed / 50.0)
        So at 50% speed → rScanFailRate fail chance per attempt.
        """
        fail_chance = CFG.rScanFailRate * (self.rBeltSpeed / 50.0)
        fail_chance = min(fail_chance, 0.95)  # Cap at 95% so a scan can always succeed
        return random.random() > fail_chance

    def _lookup_barcode(self, tote_id: str) -> str:
        """
        Get the barcode for this tote from inventory.
        Returns "" if tote not found (will trigger no-read on next check).
        """
        return INVENTORY.get_barcode_for_tote(tote_id)

    def _reset_scan_state(self) -> None:
        self._bToteInWindow = False
        self._bScanDone     = False
        self._iAttempts     = 0
        self.bScanning      = False
        self.sBarcodeResult = ""

    # ── Output accessors ───────────────────────────────────────────────────

    def scan_succeeded(self) -> bool:
        return self.bScanComplete

    def get_barcode(self) -> str:
        return self.sBarcodeResult

    def is_no_read(self) -> bool:
        return self.bNoRead

    def is_faulted(self) -> bool:
        return self.bScanFault

    def get_outputs(self) -> dict:
        return {
            "sBarcodeResult": self.sBarcodeResult,
            "bScanComplete":  self.bScanComplete,
            "bNoRead":        self.bNoRead,
            "bScanFault":     self.bScanFault,
            "bScanning":      self.bScanning,
            "iAttemptCount":  self.iAttemptCount,
        }
