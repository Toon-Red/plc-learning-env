"""
function_blocks/fb_merge_conveyor.py
=====================================
IEC 61131-3 equivalent:

    FUNCTION_BLOCK FB_MergeConveyor EXTENDS FB_ConveyorZone
    VAR_INPUT
        bInput1Ready   : BOOL;            (* Upstream input 1 has tote ready *)
        stInput1Tote   : ST_ToteData;     (* Reference from input 1 *)
        bInput2Ready   : BOOL;            (* Upstream input 2 has tote ready *)
        stInput2Tote   : ST_ToteData;     (* Reference from input 2 *)
        eMergePriority : E_MergePriority; (* Active arbitration mode *)
    END_VAR
    VAR_OUTPUT
        bAcceptingFromInput1 : BOOL;      (* True on the scan that accepted from input 1 *)
        bAcceptingFromInput2 : BOOL;      (* True on the scan that accepted from input 2 *)
    END_VAR

Two-input merge with four priority modes:
  - INPUT1_PRIORITY: Input 1 always wins when both ready
  - INPUT2_PRIORITY: Input 2 always wins (default — recirculation first)
  - FIFO:           Whoever signaled ready FIRST wins (upstream ready timestamp)
  - ALTERNATE:      Strict 1-2-1-2 toggle; skips to other if next-in-turn is empty

Recirculation count:
  - When accepting from INPUT2 (loop return), increments stTote.iRecircCount.
  - PRG_Main monitors recirc count for system E-Stop (not handled in FB).
"""

from typing import Optional

from plc.sim.sim_clock import SIM_CLOCK

from plc.function_blocks.fb_conveyor_zone import FB_ConveyorZone
from plc.types.enums import E_ConveyorState, E_MergePriority
from plc.types.structs import ST_ToteData
from plc.gvl.gvl_config import CFG


class FB_MergeConveyor(FB_ConveyorZone):
    """
    Two-input merge conveyor zone.
    Inherits the full state machine from FB_ConveyorZone.
    Adds input arbitration, deadlock detection, and recirc count tracking.
    """

    def __init__(self, zone_id: str) -> None:
        super().__init__(zone_id)

        # ── VAR_INPUT (additional) ─────────────────────────────────────────
        self.bInput1Ready:   bool                  = False
        self.stInput1Tote:   Optional[ST_ToteData] = None
        self.bInput2Ready:   bool                  = False
        self.stInput2Tote:   Optional[ST_ToteData] = None
        self.eMergePriority: E_MergePriority       = CFG.eMergePriorityDefault

        # ── VAR_OUTPUT (additional) ────────────────────────────────────────
        self.bAcceptingFromInput1: bool = False
        self.bAcceptingFromInput2: bool = False

        # ── VAR (internal) ─────────────────────────────────────────────────
        self._tInput1ReadySince:    float = 0.0    # FIFO: timestamp input 1 first became ready
        self._tInput2ReadySince:    float = 0.0    # FIFO: timestamp input 2 first became ready
        self._bPrevInput1Ready:     bool  = False  # Edge detection
        self._bPrevInput2Ready:     bool  = False
        self._bLastAcceptWas1:      bool  = True   # ALTERNATE: toggle tracker

    # ── Input interface ────────────────────────────────────────────────────

    def set_inputs(
        self,
        *,
        bInput1Ready:   bool                  = False,
        stInput1Tote:   Optional[ST_ToteData] = None,
        bInput2Ready:   bool                  = False,
        stInput2Tote:   Optional[ST_ToteData] = None,
        eMergePriority: Optional[E_MergePriority] = None,
        **kwargs,
    ) -> None:
        """
        Override: replace bAcceptTote/stIncomingTote with two-input signals.
        bAcceptTote and stIncomingTote are computed internally based on priority.
        """
        self.bInput1Ready  = bInput1Ready
        self.stInput1Tote  = stInput1Tote
        self.bInput2Ready  = bInput2Ready
        self.stInput2Tote  = stInput2Tote
        if eMergePriority is not None:
            self.eMergePriority = eMergePriority
        # Pass to base — bAcceptTote/stIncomingTote set in execute() preamble
        super().set_inputs(bAcceptTote=False, stIncomingTote=None, **kwargs)

    # ── Execution ──────────────────────────────────────────────────────────

    def execute(self) -> None:
        """
        Pre: track FIFO timestamps, select input, set base accept signals.
        Base: run inherited state machine.
        Post: check deadlock, update edge detection.
        """
        # Reset per-scan directional outputs
        self.bAcceptingFromInput1 = False
        self.bAcceptingFromInput2 = False

        # ── FIFO timestamp tracking (rising edge on inputReady) ────────────
        if self.bInput1Ready and not self._bPrevInput1Ready:
            self._tInput1ReadySince = SIM_CLOCK.now()
        if self.bInput2Ready and not self._bPrevInput2Ready:
            self._tInput2ReadySince = SIM_CLOCK.now()

        # ── Input selection (only when IDLE — can accept) ─────────────────
        if self.eState == E_ConveyorState.IDLE:
            selected = self._select_input()
            if selected == 1:
                self.stIncomingTote = self.stInput1Tote
                self.bAcceptTote    = True
                self.bAcceptingFromInput1 = True
                self._bLastAcceptWas1     = True
            elif selected == 2:
                self.stIncomingTote = self.stInput2Tote
                self.bAcceptTote    = True
                self.bAcceptingFromInput2 = True
                self._bLastAcceptWas1     = False
                # Increment recirc count on the canonical struct
                if self.stInput2Tote is not None:
                    self.stInput2Tote.iRecircCount += 1
            else:
                self.bAcceptTote    = False
                self.stIncomingTote = None
        else:
            self.bAcceptTote    = False
            self.stIncomingTote = None

        # ── Run base state machine ─────────────────────────────────────────
        super().execute()

        # ── Update edge detection ──────────────────────────────────────────
        self._bPrevInput1Ready = self.bInput1Ready
        self._bPrevInput2Ready = self.bInput2Ready

    # ── Priority logic ─────────────────────────────────────────────────────

    def _select_input(self) -> Optional[int]:
        """
        Apply merge priority to determine which input to accept from.
        Returns 1 for INPUT1, 2 for INPUT2, None if no input ready.

        IEC 61131-3 equivalent: CASE eMergePriority OF ... END_CASE
        """
        has1 = self.bInput1Ready and self.stInput1Tote is not None
        has2 = self.bInput2Ready and self.stInput2Tote is not None

        if not has1 and not has2:
            return None
        if has1 and not has2:
            return 1
        if has2 and not has1:
            return 2

        # ── Both ready — apply priority mode ──────────────────────────────
        if self.eMergePriority == E_MergePriority.INPUT1_PRIORITY:
            return 1

        elif self.eMergePriority == E_MergePriority.INPUT2_PRIORITY:
            return 2

        elif self.eMergePriority == E_MergePriority.FIFO:
            # Earlier ready-timestamp wins; tie-break → input 1
            return 1 if self._tInput1ReadySince <= self._tInput2ReadySince else 2

        elif self.eMergePriority == E_MergePriority.ALTERNATE:
            # Toggle; skip to other handled above (only one ready → accepted already)
            return 2 if self._bLastAcceptWas1 else 1

        return 1  # Fallback

    # ── Output accessors ───────────────────────────────────────────────────

    def is_accepting_from_input1(self) -> bool:
        return self.bAcceptingFromInput1

    def is_accepting_from_input2(self) -> bool:
        return self.bAcceptingFromInput2

    def get_outputs(self) -> dict:
        d = super().get_outputs()
        d.update({
            "bAcceptingFromInput1": self.bAcceptingFromInput1,
            "bAcceptingFromInput2": self.bAcceptingFromInput2,
            "eMergePriority":      self.eMergePriority.name,
        })
        return d
