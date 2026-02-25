"""
function_blocks/fb_estop.py
============================
IEC 61131-3 equivalent:

    FUNCTION_BLOCK FB_EStop
    VAR_INPUT
        bEStopInput  : BOOL;    (* Physical button OR HMI trigger *)
        bReset       : BOOL;    (* Rising edge — only valid when input is cleared *)
    END_VAR
    VAR_OUTPUT
        bEStopActive  : BOOL;   (* Latched; broadcast to ALL FBs *)
        bResetAllowed : BOOL;   (* TRUE only when physical trigger is released *)
    END_VAR
    VAR
        _bLatched    : BOOL;
        _bPrevReset  : BOOL;    (* For rising-edge detection on bReset *)
    END_VAR

Rules:
- Latching: once triggered, bEStopActive stays TRUE until explicit reset.
- Reset is ONLY allowed when bEStopInput is FALSE (trigger physically released).
- bEStopActive feeds into every FB — it is never optional.
- Alarm A005 (CRITICAL) is raised on activation.
"""

from plc.types.enums import E_AlarmSeverity
from plc.gvl.gvl_alarms import ALARMS


class FB_EStop:
    """
    Latching E-Stop Function Block.

    Call execute() once per scan cycle.
    Read bEStopActive to broadcast to all other FBs.
    """

    # ── VAR_INPUT ──────────────────────────────────────────────────────────
    bEStopInput: bool
    bReset:      bool

    # ── VAR_OUTPUT ─────────────────────────────────────────────────────────
    bEStopActive:  bool
    bResetAllowed: bool

    # ── VAR (internal) ─────────────────────────────────────────────────────
    _bLatched:   bool
    _bPrevReset: bool

    def __init__(self) -> None:
        self.bEStopInput   = False
        self.bReset        = False
        self.bEStopActive  = False
        self.bResetAllowed = False
        self._bLatched     = False
        self._bPrevReset   = False

    # ── Public interface ───────────────────────────────────────────────────

    def set_inputs(self, *, bEStopInput: bool, bReset: bool) -> None:
        """
        Write VAR_INPUT values before calling execute().
        Named parameters enforce explicit assignment — mirrors CODESYS FB call syntax:
            fbEStop(bEStopInput := bHwEstop OR GVL_HMI.bHMI_EStop,
                    bReset      := bResetSignal);
        """
        self.bEStopInput = bEStopInput
        self.bReset      = bReset

    def execute(self) -> None:
        """
        Single scan cycle execution.
        PRG_Main calls this in the CALCULATE phase.

        Logic (mirrors IEC 61131-3 sequential execution):
          1. If input is active → latch.
          2. bResetAllowed only when input is physically cleared.
          3. Rising edge on bReset + allowed → unlatch.
          4. Raise alarm on activation edge.
        """
        # Step 1 — Latch on trigger
        if self.bEStopInput:
            if not self._bLatched:
                # Rising edge of E-Stop — raise critical alarm once
                ALARMS.raise_alarm(5, zone_id="ESTOP")
            self._bLatched = True

        # Step 2 — Reset gate
        self.bResetAllowed = (not self.bEStopInput) and self._bLatched

        # Step 3 — Rising edge reset (R_TRIG equivalent)
        _reset_rising_edge = self.bReset and not self._bPrevReset
        if _reset_rising_edge and self.bResetAllowed:
            self._bLatched = False

        # Step 4 — Output
        self.bEStopActive = self._bLatched

        # Step 5 — Store previous reset for edge detection next scan
        self._bPrevReset = self.bReset

    def is_active(self) -> bool:
        """Convenience accessor — readable in PRG_Main conditional checks."""
        return self.bEStopActive

    def is_reset_allowed(self) -> bool:
        return self.bResetAllowed

    def get_outputs(self) -> dict:
        """Returns VAR_OUTPUT values for GVL_IO update."""
        return {
            "bEStopActive":  self.bEStopActive,
            "bResetAllowed": self.bResetAllowed,
        }
