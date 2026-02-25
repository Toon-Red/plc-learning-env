"""
tests/test_fb_branch_conveyor.py
Tests for FB_BranchConveyor on-the-fly divert logic.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from plc.function_blocks.fb_branch_conveyor import FB_BranchConveyor
from plc.types.enums import E_ChuteTarget, E_ConveyorState
from plc.types.structs import ST_ToteData
from plc.gvl.gvl_alarms import ALARMS


@pytest.fixture(autouse=True)
def reset_alarms():
    ALARMS.aAlarmBuffer.clear()
    ALARMS.diActiveAlarmCount = 0
    ALARMS.diUnackAlarmCount  = 0
    yield


def _make_tote(tote_id: str, chute: E_ChuteTarget) -> ST_ToteData:
    """Helper: build a minimal ST_ToteData with an assigned chute."""
    st = ST_ToteData()
    st.sToteID          = tote_id
    st.sBarcode         = "BC-TEST"
    st.eChuteAssignment = chute
    st.bIsActive        = True
    return st


def _push_tote_to_transporting(fb: FB_BranchConveyor, tote: ST_ToteData) -> None:
    """
    Drive the branch FB from IDLE to TRANSPORTING with the given tote.
    Three ticks: IDLE→ACCEPTING, ACCEPTING→TRANSPORTING (beam break), stabilise.
    """
    # Tick 1: IDLE → ACCEPTING (bAcceptTote + stIncomingTote)
    fb.set_inputs(
        bEnable=True, bEStop=False, bReset=False,
        bBeamBreak=False,
        bStraightDownstreamClear=True,
        bDivertDownstreamClear=True,
        stIncomingTote=tote,
        bAcceptTote=True,
    )
    fb.execute()

    # Tick 2: ACCEPTING → TRANSPORTING (beam break fires)
    fb.set_inputs(
        bEnable=True, bEStop=False, bReset=False,
        bBeamBreak=True,
        bStraightDownstreamClear=True,
        bDivertDownstreamClear=True,
        stIncomingTote=None,
        bAcceptTote=False,
    )
    fb.execute()


class TestBranchDivertDecision:

    def test_tote_assigned_to_target_chute_sets_divert_active(self):
        """Tote assigned to chute 1 on a branch serving chute 1 → bDivertActive=True."""
        tote = _make_tote("T1", E_ChuteTarget.CHUTE_1)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)
        assert fb.bDivertActive is True
        assert fb.is_diverting() is True

    def test_tote_assigned_to_other_chute_goes_straight(self):
        """Tote assigned to chute 2 on a branch serving chute 1 → straight, no divert."""
        tote = _make_tote("T2", E_ChuteTarget.CHUTE_2)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)
        assert fb.bDivertActive is False
        assert fb.is_diverting() is False

    def test_unassigned_tote_goes_straight(self):
        """UNASSIGNED tote (routing not yet resolved) → straight path, never diverts."""
        tote = _make_tote("T3", E_ChuteTarget.UNASSIGNED)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)
        assert fb.bDivertActive is False

    def test_reject_chute_tote_goes_straight_through_non_target_branch(self):
        """REJECT tote (chute 99) does not divert at chute-1 branch."""
        tote = _make_tote("T4", E_ChuteTarget.REJECT)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)
        assert fb.bDivertActive is False


class TestBranchPassthrough:

    def test_straight_passthrough_populated_for_straight_tote(self):
        """After tote exits straight, stToteToStraight is set for one scan."""
        tote = _make_tote("T5", E_ChuteTarget.CHUTE_2)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)

        # TRANSPORTING → RELEASING (straight clear, beam still on)
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=True,
            bStraightDownstreamClear=True,
            bDivertDownstreamClear=True,
        )
        fb.execute()

        # RELEASING → IDLE (beam break falls → tote exits)
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=False,
            bStraightDownstreamClear=True,
            bDivertDownstreamClear=True,
        )
        fb.execute()

        assert fb.stToteToStraight is tote
        assert fb.stToteToDivert   is None

    def test_divert_passthrough_populated_for_diverted_tote(self):
        """After tote exits divert, stToteToDivert is set for one scan."""
        tote = _make_tote("T6", E_ChuteTarget.CHUTE_1)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)

        # TRANSPORTING → RELEASING (divert clear)
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=True,
            bStraightDownstreamClear=True,
            bDivertDownstreamClear=True,
        )
        fb.execute()

        # RELEASING → IDLE (beam break clears)
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=False,
            bStraightDownstreamClear=True,
            bDivertDownstreamClear=True,
        )
        fb.execute()

        assert fb.stToteToDivert   is tote
        assert fb.stToteToStraight is None

    def test_passthrough_cleared_on_next_scan(self):
        """stToteToDivert / stToteToStraight are reset to None the following scan."""
        tote = _make_tote("T7", E_ChuteTarget.CHUTE_1)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)

        for _ in range(2):   # TRANSPORTING → RELEASING → IDLE
            fb.set_inputs(
                bEnable=True, bEStop=False, bReset=False,
                bBeamBreak=(fb.eState != E_ConveyorState.RELEASING),
                bStraightDownstreamClear=True,
                bDivertDownstreamClear=True,
            )
            fb.execute()

        assert fb.stToteToDivert is tote   # Set on the IDLE scan after exit

        # One more scan — outputs reset
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=False,
            bStraightDownstreamClear=True,
            bDivertDownstreamClear=True,
        )
        fb.execute()
        assert fb.stToteToDivert   is None
        assert fb.stToteToStraight is None


class TestBranchWaiting:

    def test_bypasses_straight_when_spur_full(self):
        """
        Tote assigned CHUTE_1 on BRANCH_1 — divert path BLOCKED.
        Straight path clear — tote bypasses via straight to recirculate.
        """
        tote = _make_tote("T8", E_ChuteTarget.CHUTE_1)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)

        # TRANSPORTING with divert blocked, straight clear
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=True,
            bStraightDownstreamClear=True,
            bDivertDownstreamClear=False,     # SPUR FULL
        )
        fb.execute()

        # Should bypass straight (RELEASING), not WAITING
        assert fb.eState == E_ConveyorState.RELEASING
        assert fb._bWillDivert is False

    def test_waits_when_both_paths_blocked(self):
        """When both divert AND straight are blocked, tote must wait."""
        tote = _make_tote("T9", E_ChuteTarget.CHUTE_1)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)

        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=True,
            bStraightDownstreamClear=False,   # BOTH blocked
            bDivertDownstreamClear=False,
        )
        fb.execute()
        assert fb.eState == E_ConveyorState.WAITING

    def test_diverts_when_spur_clears(self):
        """When spur has space, tote diverts normally."""
        tote = _make_tote("T9b", E_ChuteTarget.CHUTE_1)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)

        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=True,
            bStraightDownstreamClear=True,
            bDivertDownstreamClear=True,
        )
        fb.execute()
        assert fb.eState == E_ConveyorState.RELEASING
        assert fb._bWillDivert is True


class TestBranchRoutingResolvedWhileWaiting:

    def test_routing_resolves_during_waiting_updates_divert(self):
        """
        Tote enters with UNASSIGNED → goes to WAITING (straight blocked).
        Routing resolves mid-wait → struct updated → next tick re-evaluates.
        Once straight clears AND divert path matches, tote exits correctly.
        """
        tote = _make_tote("T10", E_ChuteTarget.UNASSIGNED)
        fb   = FB_BranchConveyor(zone_id="BRANCH_1", target_chute=1)
        _push_tote_to_transporting(fb, tote)

        # TRANSPORTING — straight blocked, unassigned → wait on straight
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=True,
            bStraightDownstreamClear=False,  # blocked → WAITING
            bDivertDownstreamClear=True,
        )
        fb.execute()
        assert fb.eState == E_ConveyorState.WAITING
        assert fb.bDivertActive is False

        # Routing resolves in-place on the struct
        tote.eChuteAssignment = E_ChuteTarget.CHUTE_1

        # Next tick: re-evaluates → now _bWillDivert=True, divert path is clear
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False,
            bBeamBreak=True,
            bStraightDownstreamClear=False,
            bDivertDownstreamClear=True,
        )
        fb.execute()

        assert fb.bDivertActive is True
        assert fb.eState == E_ConveyorState.RELEASING
