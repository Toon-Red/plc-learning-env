"""
function_blocks/fb_conveyor_zone.py
====================================
IEC 61131-3 equivalent:

    FUNCTION_BLOCK FB_ConveyorZone
    VAR_INPUT
        bEnable          : BOOL;
        bEStop           : BOOL;
        bReset           : BOOL;
        bBeamBreak       : BOOL;    (* TRUE = tote at center sensor *)
        bDownstreamClear : BOOL;    (* TRUE = next zone ready to accept *)
        rSpeedPercent    : REAL;    (* 0.0–100.0 *)
        stIncomingTote   : ST_ToteData;    (* Reference — shared with ToteTracker registry *)
        bAcceptTote      : BOOL;    (* Upstream says: push tote into me *)
    END_VAR
    VAR_OUTPUT
        bMotorRun        : BOOL;
        bTotePresent     : BOOL;
        bFaulted         : BOOL;
        bReady           : BOOL;    (* Idle, no tote, can accept *)
        bOutputOccupied  : BOOL;    (* I have a tote — tell upstream to hold *)
        stTotePassThrough: ST_ToteData;    (* Reference passed downstream, not a copy *)
        rActualSpeed     : REAL;
    END_VAR
    VAR
        eState           : E_ConveyorState;
        _stCurrentTote   : ST_ToteData;
        _bPrevBeamBreak  : BOOL;
        _bPrevReset      : BOOL;
        _bPrevEStop      : BOOL;
    END_VAR

Scan cycle behavior:
- CHECK INPUTS phase: caller sets inputs via set_inputs()
- CALCULATE phase: caller calls execute()
- EXECUTE phase: caller reads outputs via get_outputs() or direct attributes

One tote per zone — will NOT accept if bOutputOccupied is TRUE.
Each zone only knows: do I have a tote (beam break)? Is downstream clear?
If downstream isn't clear, zone WAITS. Waiting is normal, NOT a fault.
FAULTED state only entered via E-Stop. Reset clears it.
Jam detection is done at the PRG_Main system level (not per-zone).
"""

from typing import Optional

from plc.types.enums import E_ConveyorState
from plc.types.structs import ST_ToteData, ST_ZoneStatus
from plc.gvl.gvl_config import CFG
from plc.gvl.gvl_io import IO


class FB_ConveyorZone:
    """
    Base conveyor zone Function Block.
    All other conveyor types (Branch, Merge) extend this class.

    The pattern mirrors IEC 61131-3 FB extension:
        FUNCTION_BLOCK FB_BranchConveyor EXTENDS FB_ConveyorZone
    """

    def __init__(self, zone_id: str) -> None:
        """
        VAR declaration equivalent.
        zone_id is the unique identifier (e.g. "INFEED", "C3_SCAN", "BRANCH_1").
        """
        self.zone_id: str = zone_id

        # ── VAR_INPUT ──────────────────────────────────────────────────
        self.bEnable:          bool                  = False
        self.bEStop:           bool                  = False
        self.bReset:           bool                  = False
        self.bBeamBreak:       bool                  = False
        self.bDownstreamClear: bool                  = False
        self.rSpeedPercent:    float                 = CFG.rDefaultBeltSpeed
        self.stIncomingTote:   Optional[ST_ToteData] = None
        self.bAcceptTote:      bool                  = False  # Upstream pushing tote in

        # ── VAR_OUTPUT ─────────────────────────────────────────────────
        self.bMotorRun:          bool                  = False
        self.bTotePresent:       bool                  = False
        self.bFaulted:           bool                  = False
        self.bReady:             bool                  = False
        self.bOutputOccupied:    bool                  = False
        self.stTotePassThrough:  Optional[ST_ToteData] = None
        self.rActualSpeed:       float                 = 0.0

        # ── VAR (internal) ───────────────────────────────────────────────
        self.eState:           E_ConveyorState      = E_ConveyorState.IDLE
        self._stCurrentTote:   Optional[ST_ToteData] = None
        self._bPrevBeamBreak:  bool                 = False
        self._bPrevReset:      bool                 = False
        self._bPrevEStop:      bool                 = False

    # ── Input interface ────────────────────────────────────────────────────

    def set_inputs(
        self,
        *,
        bEnable:          bool,
        bEStop:           bool,
        bReset:           bool,
        bBeamBreak:       bool,
        bDownstreamClear: bool,
        rSpeedPercent:    float                 = -1.0,   # -1 = use default
        stIncomingTote:   Optional[ST_ToteData] = None,
        bAcceptTote:      bool                  = False,
    ) -> None:
        """
        Write all VAR_INPUT values before calling execute().
        Mirrors CODESYS named parameter call:
            fbZone(bEnable := ..., bEStop := ..., ...);
        """
        self.bEnable          = bEnable
        self.bEStop           = bEStop
        self.bReset           = bReset
        self.bBeamBreak       = bBeamBreak
        self.bDownstreamClear = bDownstreamClear
        self.rSpeedPercent    = rSpeedPercent if rSpeedPercent >= 0.0 else CFG.rDefaultBeltSpeed
        self.stIncomingTote   = stIncomingTote
        self.bAcceptTote      = bAcceptTote

    # ── Execution ────────────────────────────────────────────────────────

    def execute(self) -> None:
        """
        One scan cycle of the zone state machine.
        Call order in PRG_Main: upstream zones first, then downstream.

        Structure mirrors IEC 61131-3 execution model:
          - Global transitions (E-Stop, Reset) checked FIRST
          - State CASE dispatch
          - Output assignment LAST
        """
        # ── Clear one-scan outputs from previous scan ──────────────────────────────────
        self.stTotePassThrough = None

        # ── Global transitions (override all states) ────────────────────────────────────

        if self.bEStop:
            self._transition_to(E_ConveyorState.FAULTED)
            self._set_motor(False)
            self._write_outputs()
            self._bPrevEStop = True
            return

        # Rising edge reset — only when not E-Stopped
        _reset_edge = self.bReset and not self._bPrevReset
        if _reset_edge and self.eState == E_ConveyorState.FAULTED:
            self._transition_to(E_ConveyorState.RESETTING)

        # ── State machine CASE dispatch ───────────────────────────────────────────────

        if self.eState == E_ConveyorState.IDLE:
            self._state_idle()

        elif self.eState == E_ConveyorState.ACCEPTING:
            self._state_accepting()

        elif self.eState == E_ConveyorState.TRANSPORTING:
            self._state_transporting()

        elif self.eState == E_ConveyorState.WAITING:
            self._state_waiting()

        elif self.eState == E_ConveyorState.RELEASING:
            self._state_releasing()

        elif self.eState == E_ConveyorState.FAULTED:
            self._state_faulted()

        elif self.eState == E_ConveyorState.RESETTING:
            self._state_resetting()

        # ── Outputs ─────────────────────────────────────────────────────────────────
        self._write_outputs()

        # ── Store previous values for next scan's edge detection ─────────────────
        self._bPrevBeamBreak = self.bBeamBreak
        self._bPrevReset     = self.bReset
        self._bPrevEStop     = self.bEStop

    # ── State handlers ─────────────────────────────────────────────────────────

    def _state_idle(self) -> None:
        """No tote. Ready to accept. Motor off."""
        self._set_motor(False)

        # Accept incoming tote: upstream pushes (bAcceptTote + valid tote reference)
        if self.bEnable and self.bAcceptTote and self.stIncomingTote is not None:
            self._stCurrentTote = self.stIncomingTote
            self._transition_to(E_ConveyorState.ACCEPTING)

    def _state_accepting(self) -> None:
        """Tote entering. Motor runs to pull it fully in."""
        self._set_motor(True)
        # Once beam break confirms tote is centered, move to transporting
        if self.bBeamBreak:
            self._transition_to(E_ConveyorState.TRANSPORTING)

    def _state_transporting(self) -> None:
        """Tote present and centered. Advance if downstream is clear."""
        if self.bDownstreamClear:
            self._set_motor(True)
            self._transition_to(E_ConveyorState.RELEASING)
        else:
            self._set_motor(False)
            self._transition_to(E_ConveyorState.WAITING)

    def _state_waiting(self) -> None:
        """
        Downstream blocked. Motor off. Tote sits here until downstream clears.
        Waiting is NORMAL behavior — no fault, no timer. The zone just holds.
        If beam break clears while waiting (e.g. robot removed tote), go to IDLE.
        """
        self._set_motor(False)

        # Beam break cleared while waiting = tote was removed externally (robot pickup)
        if not self.bBeamBreak:
            self._stCurrentTote = None
            self._transition_to(E_ConveyorState.IDLE)
            return

        # Downstream cleared — resume
        if self.bDownstreamClear:
            self._set_motor(True)
            self._transition_to(E_ConveyorState.RELEASING)

    def _state_releasing(self) -> None:
        """
        Motor running, advancing tote to downstream zone.
        When beam break clears (tote fully exited) → IDLE.
        If downstream becomes occupied before tote exits, revert to WAITING
        to avoid false jam detection from normal backpressure.
        """
        # Beam break cleared = tote has physically left this zone
        _beam_break_falling_edge = self._bPrevBeamBreak and not self.bBeamBreak
        if _beam_break_falling_edge or not self.bBeamBreak:
            # Pass tote reference downstream via output
            self.stTotePassThrough = self._stCurrentTote
            self._stCurrentTote    = None
            self._transition_to(E_ConveyorState.IDLE)
            return

        # Tote still here — if downstream became occupied (e.g. another tote
        # arrived from a merge or branch), revert to WAITING. This prevents
        # false jam detection from normal backpressure.
        if not self.bDownstreamClear:
            self._set_motor(False)
            self._transition_to(E_ConveyorState.WAITING)
            return

        # Normal: motor ON, pushing tote out
        self._set_motor(True)

    def _state_faulted(self) -> None:
        """Motor off. Waiting for reset command. Only entered via E-Stop."""
        self._set_motor(False)

    def _state_resetting(self) -> None:
        """
        Reset transition after E-Stop clear.
        If beam break is still active, tote is physically here — keep the struct
        and transition based on downstream status. If beam clear, go IDLE.
        """
        self.stTotePassThrough = None
        self._set_motor(False)

        if self.bBeamBreak and self._stCurrentTote is not None:
            # Tote is still physically on this zone
            if self.bDownstreamClear:
                self._transition_to(E_ConveyorState.TRANSPORTING)
            else:
                self._transition_to(E_ConveyorState.WAITING)
        else:
            # Beam clear or no tote data — zone is empty
            self._stCurrentTote = None
            self._transition_to(E_ConveyorState.IDLE)

    # ── Internal helpers ─────────────────────────────────────────────────────────

    def _transition_to(self, new_state: E_ConveyorState) -> None:
        self.eState = new_state

    def _set_motor(self, running: bool) -> None:
        self.bMotorRun    = running and self.bEnable and not self.bEStop
        self.rActualSpeed = self.rSpeedPercent if self.bMotorRun else 0.0

    def _write_outputs(self) -> None:
        """
        Assign all VAR_OUTPUT values from current internal state.
        Must be last step in execute() — mirrors CODESYS output assignment.
        """
        self.bTotePresent    = self._stCurrentTote is not None
        self.bOutputOccupied = self._stCurrentTote is not None
        self.bFaulted        = (self.eState == E_ConveyorState.FAULTED)
        self.bReady          = (
            self.eState == E_ConveyorState.IDLE
            and self.bEnable
            and not self.bEStop
            and not self.bFaulted
            and self._stCurrentTote is None
        )

        # Write zone status to GVL_IO (EXECUTE phase equivalent)
        IO.update_zone(self.zone_id, ST_ZoneStatus(
            sZoneID      = self.zone_id,
            eState       = int(self.eState),
            bMotorRun    = self.bMotorRun,
            bTotePresent = self.bTotePresent,
            bFaulted     = self.bFaulted,
            rActualSpeed = self.rActualSpeed,
            sToteID      = self._stCurrentTote.sToteID if self._stCurrentTote else "",
            sBarcode     = self._stCurrentTote.sBarcode if self._stCurrentTote else "",
        ))

    # ── Output accessors (named methods per user requirement) ──────────────────────

    def is_ready(self) -> bool:
        """Zone is idle and can accept a tote."""
        return self.bReady

    def has_tote(self) -> bool:
        """Zone currently holds a tote."""
        return self.bTotePresent

    def is_faulted(self) -> bool:
        return self.bFaulted

    def get_tote_data(self) -> Optional[ST_ToteData]:
        """Returns the current tote reference."""
        return self._stCurrentTote

    def get_passthrough_tote(self) -> Optional[ST_ToteData]:
        """Returns the tote that just exited this zone (one scan valid)."""
        return self.stTotePassThrough

    def force_release_tote(self) -> Optional[ST_ToteData]:
        """
        Force-release a tote from an end-of-track zone.
        Used by PRG_Main for zones at the end of a SimEngine track where the
        tote is clamped at the track boundary and the beam break can never
        clear naturally. Only acts when zone is in RELEASING state.
        Returns the tote struct, or None if not in RELEASING state.
        """
        if self.eState != E_ConveyorState.RELEASING or self._stCurrentTote is None:
            return None
        tote = self._stCurrentTote
        self.stTotePassThrough = tote
        self._stCurrentTote = None
        self._set_motor(False)
        self._transition_to(E_ConveyorState.IDLE)
        return tote

    def get_state(self) -> E_ConveyorState:
        return self.eState

    def get_speed(self) -> float:
        return self.rActualSpeed

    def get_outputs(self) -> dict:
        """Full VAR_OUTPUT snapshot for debugging / test assertions."""
        return {
            "bMotorRun":         self.bMotorRun,
            "bTotePresent":      self.bTotePresent,
            "bFaulted":          self.bFaulted,
            "bReady":            self.bReady,
            "bOutputOccupied":   self.bOutputOccupied,
            "rActualSpeed":      self.rActualSpeed,
            "eState":            self.eState.name,
            "sToteID":           self._stCurrentTote.sToteID if self._stCurrentTote else "",
        }
