"""
function_blocks/fb_branch_conveyor.py
======================================
IEC 61131-3 equivalent:

    FUNCTION_BLOCK FB_BranchConveyor EXTENDS FB_ConveyorZone
    VAR_INPUT
        bStraightDownstreamClear : BOOL;    (* Next zone on main line ready *)
        bDivertDownstreamClear   : BOOL;    (* Spur/chute entry zone ready *)
        iTargetChute             : INT;     (* Which chute 1-based this branch serves *)
    END_VAR
    VAR_OUTPUT
        bDivertActive    : BOOL;            (* Gate energised — divert path selected *)
        stToteToStraight : ST_ToteData;     (* Passthrough to main line (by reference) *)
        stToteToDivert   : ST_ToteData;     (* Passthrough to spur (by reference) *)
    END_VAR

On-the-fly divert — tote never stops at the gate:
  - While tote is in this zone, _bWillDivert is evaluated every scan from the
    canonical struct's eChuteAssignment (written by routing engine, possibly delayed).
  - bDownstreamClear is set to the relevant path's clear signal before the
    base state machine runs each scan.
  - This means if routing resolves while the tote is in WAITING, the correct
    path is selected on the next scan automatically.
  - UNASSIGNED → bWillDivert=False → straight path (tote continues downstream).

Passthrough outputs:
  - stToteToStraight / stToteToDivert are valid for exactly ONE scan after
    the tote exits (same contract as stTotePassThrough in the base class).
  - PRG_Main reads whichever output is relevant and passes it to the
    matching downstream zone's set_inputs(stIncomingTote=...).
"""

from typing import Optional

from plc.function_blocks.fb_conveyor_zone import FB_ConveyorZone
from plc.types.structs import ST_ToteData


class FB_BranchConveyor(FB_ConveyorZone):
    """
    Conveyor zone with two exits: straight-through and divert.
    Inherits the full state machine from FB_ConveyorZone.
    Adds divert-gate logic on top without modifying any base state.
    """

    def __init__(self, zone_id: str, target_chute: int) -> None:
        super().__init__(zone_id)

        # ── VAR_INPUT (additional) ─────────────────────────────────────────
        self.bStraightDownstreamClear: bool = False
        self.bDivertDownstreamClear:   bool = False
        self.iTargetChute:             int  = target_chute   # 1-based chute number

        # ── VAR_OUTPUT (additional) ────────────────────────────────────────
        self.bDivertActive:    bool                  = False
        self.stToteToStraight: Optional[ST_ToteData] = None
        self.stToteToDivert:   Optional[ST_ToteData] = None

        # ── VAR (internal) ─────────────────────────────────────────────────
        self._bWillDivert: bool = False   # Latched divert decision for current tote

    # ── Input interface ────────────────────────────────────────────────────

    def set_inputs(
        self,
        *,
        bStraightDownstreamClear: bool = False,
        bDivertDownstreamClear:   bool = False,
        **kwargs,
    ) -> None:
        """
        Override: replace single bDownstreamClear with two path-specific signals.
        bDownstreamClear is NOT exposed — computed internally before state machine.
        """
        self.bStraightDownstreamClear = bStraightDownstreamClear
        self.bDivertDownstreamClear   = bDivertDownstreamClear
        # Pass bDownstreamClear=False; execute() will overwrite it before state dispatch
        super().set_inputs(bDownstreamClear=False, **kwargs)

    # ── Execution ──────────────────────────────────────────────────────────

    def execute(self) -> None:
        """
        Pre-compute divert decision from tote struct, route appropriate
        downstream clear signal into bDownstreamClear, then run base state machine.
        Post-process: split stTotePassThrough into directional outputs.
        """
        # Reset per-scan directional outputs (valid for one scan only)
        self.stToteToStraight = None
        self.stToteToDivert   = None

        # ── Compute divert decision ────────────────────────────────────────
        # _stCurrentTote is set in _state_idle → ACCEPTING transition.
        # On the ACCEPTING scan it may still be None; divert defaults to False
        # and is corrected on the next (TRANSPORTING) scan.
        if self._stCurrentTote is not None:
            bShouldDivert = (
                self._stCurrentTote.eChuteAssignment.value == self.iTargetChute
            )

            if bShouldDivert and self.bDivertDownstreamClear:
                # Normal divert: spur has space
                self._bWillDivert     = True
                self.bDownstreamClear = True
            elif bShouldDivert and not self.bDivertDownstreamClear:
                # Spur is full: bypass straight so tote recirculates
                self._bWillDivert     = False
                self.bDownstreamClear = self.bStraightDownstreamClear
            else:
                # Not our chute: pass straight
                self._bWillDivert     = False
                self.bDownstreamClear = self.bStraightDownstreamClear
        else:
            # No tote present — default to straight (value irrelevant; IDLE will ignore it)
            self._bWillDivert     = False
            self.bDownstreamClear = self.bStraightDownstreamClear

        # ── Run base state machine ─────────────────────────────────────────
        super().execute()

        # ── Route passthrough to directional output ───────────────────────
        # stTotePassThrough is set by _state_releasing when beam break clears.
        if self.stTotePassThrough is not None:
            if self._bWillDivert:
                self.stToteToDivert   = self.stTotePassThrough
            else:
                self.stToteToStraight = self.stTotePassThrough
            self.stTotePassThrough = None   # Don't expose base output directly

    # ── Output writer override ─────────────────────────────────────────────

    def _write_outputs(self) -> None:
        """Write base outputs, then add bDivertActive."""
        super()._write_outputs()
        # Gate is energised while the tote is present AND will be diverted
        self.bDivertActive = self._bWillDivert and (self._stCurrentTote is not None)

    # ── Output accessors ───────────────────────────────────────────────────

    def is_diverting(self) -> bool:
        """True while current tote is assigned to this branch's chute."""
        return self.bDivertActive

    def get_straight_tote(self) -> Optional[ST_ToteData]:
        """Reference to tote that just exited straight. Valid one scan only."""
        return self.stToteToStraight

    def get_diverted_tote(self) -> Optional[ST_ToteData]:
        """Reference to tote that just diverted. Valid one scan only."""
        return self.stToteToDivert

    def get_outputs(self) -> dict:
        d = super().get_outputs()
        d.update({
            "bDivertActive": self.bDivertActive,
            "bWillDivert":   self._bWillDivert,
        })
        return d
