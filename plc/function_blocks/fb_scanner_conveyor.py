"""
function_blocks/fb_scanner_conveyor.py
=======================================
IEC 61131-3 equivalent:

    FUNCTION_BLOCK FB_ScannerConveyor EXTENDS FB_ConveyorZone
    VAR
        fbScanner : FB_BarcodeScanner;  (* Composed, not inherited *)
    END_VAR
    VAR_OUTPUT
        bScanComplete   : BOOL;
        sBarcodeResult  : STRING;
        bNoRead         : BOOL;
        bScanFault      : BOOL;
        bScanning       : BOOL;
        iAttemptCount   : INT;
    END_VAR

Conveyor zone with integrated barcode scanner (composition).
The zone IS the scanner — one object handles belt + scanning.
PRG_Main reads scan results directly from the zone FB.
"""

from plc.function_blocks.fb_conveyor_zone import FB_ConveyorZone
from plc.function_blocks.fb_barcode_scanner import FB_BarcodeScanner


class FB_ScannerConveyor(FB_ConveyorZone):
    """
    Conveyor zone with integrated barcode scanner.
    Composes FB_BarcodeScanner — scanner runs automatically
    whenever a tote is present on the zone.
    """

    def __init__(self, zone_id: str) -> None:
        super().__init__(zone_id)

        # Composed scanner FB
        self._fbScanner = FB_BarcodeScanner(zone_id)

        # Scanner outputs (mirrored from internal FB)
        self.bScanComplete:  bool = False
        self.sBarcodeResult: str  = ""
        self.bNoRead:        bool = False
        self.bScanFault:     bool = False
        self.bScanning:      bool = False
        self.iAttemptCount:  int  = 0

    def execute(self) -> None:
        """Run zone state machine, then feed scanner from zone state."""
        super().execute()

        # Feed scanner inputs from zone state
        tote = self._stCurrentTote
        self._fbScanner.set_inputs(
            bEnable=self.bEnable,
            bEStop=self.bEStop,
            bTotePresent=self.bBeamBreak,
            sToteID=tote.sToteID if tote else "",
            rBeltSpeed=self.rActualSpeed,
        )
        self._fbScanner.execute()

        # Mirror scanner outputs
        self.bScanComplete  = self._fbScanner.bScanComplete
        self.sBarcodeResult = self._fbScanner.sBarcodeResult
        self.bNoRead        = self._fbScanner.bNoRead
        self.bScanFault     = self._fbScanner.bScanFault
        self.bScanning      = self._fbScanner.bScanning
        self.iAttemptCount  = self._fbScanner.iAttemptCount

    def get_scanner(self) -> FB_BarcodeScanner:
        """Access the internal scanner FB."""
        return self._fbScanner

    def get_outputs(self) -> dict:
        d = super().get_outputs()
        d.update({
            "bScanComplete":  self.bScanComplete,
            "sBarcodeResult": self.sBarcodeResult,
            "bNoRead":        self.bNoRead,
            "bScanFault":     self.bScanFault,
            "bScanning":      self.bScanning,
            "iAttemptCount":  self.iAttemptCount,
        })
        return d
