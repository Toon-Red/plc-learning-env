"""
tests/test_fb_merge_conveyor.py
Tests for FB_MergeConveyor: priority modes, deadlock, recirc counting.
"""

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from plc.function_blocks.fb_merge_conveyor import FB_MergeConveyor
from plc.types.enums import E_ChuteTarget, E_ConveyorState, E_MergePriority
from plc.types.structs import ST_ToteData
from plc.gvl.gvl_alarms import ALARMS
from plc.gvl.gvl_config import CFG


@pytest.fixture(autouse=True)
def reset_alarms():
    ALARMS.aAlarmBuffer.clear()
    ALARMS.diActiveAlarmCount = 0
    ALARMS.diUnackAlarmCount  = 0
    yield


def _make_tote(tote_id: str) -> ST_ToteData:
    st = ST_ToteData()
    st.sToteID  = tote_id
    st.sBarcode = f"BC-{tote_id}"
    st.bIsActive = True
    return st


def _tick_idle(fb, *, bInput1Ready=False, stInput1Tote=None,
               bInput2Ready=False, stInput2Tote=None,
               bDownstreamClear=True, eMergePriority=None):
    """Run one scan with the given inputs. Merge must be IDLE to accept."""
    kwargs = dict(
        bEnable=True, bEStop=False, bReset=False, bBeamBreak=False,
        bDownstreamClear=bDownstreamClear,
        bInput1Ready=bInput1Ready, stInput1Tote=stInput1Tote,
        bInput2Ready=bInput2Ready, stInput2Tote=stInput2Tote,
    )
    if eMergePriority is not None:
        kwargs["eMergePriority"] = eMergePriority
    fb.set_inputs(**kwargs)
    fb.execute()


def _accept_to_transporting(fb, *, bInput1Ready=False, stInput1Tote=None,
                             bInput2Ready=False, stInput2Tote=None,
                             eMergePriority=None):
    """Drive merge from IDLE through ACCEPTING to TRANSPORTING."""
    # Tick 1: IDLE → ACCEPTING
    kwargs = dict(
        bInput1Ready=bInput1Ready, stInput1Tote=stInput1Tote,
        bInput2Ready=bInput2Ready, stInput2Tote=stInput2Tote,
    )
    if eMergePriority is not None:
        kwargs["eMergePriority"] = eMergePriority
    _tick_idle(fb, **kwargs)

    # Tick 2: ACCEPTING → TRANSPORTING (beam break fires)
    fb.set_inputs(
        bEnable=True, bEStop=False, bReset=False, bBeamBreak=True,
        bDownstreamClear=True,
        bInput1Ready=False, stInput1Tote=None,
        bInput2Ready=False, stInput2Tote=None,
    )
    fb.execute()


# ── Priority mode tests ──────────────────────────────────────────────────

class TestMergePriorityModes:

    def test_input1_priority_selects_input1_when_both_ready(self):
        tote1 = _make_tote("T1")
        tote2 = _make_tote("T2")
        fb = FB_MergeConveyor("MERGE")
        _tick_idle(fb,
                   bInput1Ready=True, stInput1Tote=tote1,
                   bInput2Ready=True, stInput2Tote=tote2,
                   eMergePriority=E_MergePriority.INPUT1_PRIORITY)
        assert fb.bAcceptingFromInput1 is True
        assert fb.bAcceptingFromInput2 is False

    def test_input2_priority_selects_input2_when_both_ready(self):
        tote1 = _make_tote("T1")
        tote2 = _make_tote("T2")
        fb = FB_MergeConveyor("MERGE")
        _tick_idle(fb,
                   bInput1Ready=True, stInput1Tote=tote1,
                   bInput2Ready=True, stInput2Tote=tote2,
                   eMergePriority=E_MergePriority.INPUT2_PRIORITY)
        assert fb.bAcceptingFromInput1 is False
        assert fb.bAcceptingFromInput2 is True

    def test_fifo_selects_earlier_ready(self):
        """Input 1 becomes ready first → FIFO selects input 1."""
        tote1 = _make_tote("T1")
        tote2 = _make_tote("T2")
        fb = FB_MergeConveyor("MERGE")

        # Input 1 becomes ready first (rising edge)
        _tick_idle(fb,
                   bInput1Ready=True, stInput1Tote=tote1,
                   bInput2Ready=False, stInput2Tote=None,
                   eMergePriority=E_MergePriority.FIFO)
        # Not accepted yet — only input 1 ready, will accept input 1
        # But let's test with BOTH ready. Reset by releasing tote first.
        fb2 = FB_MergeConveyor("MERGE2")
        # Simulate: input 1 goes ready at t=0
        fb2.set_inputs(
            bEnable=True, bEStop=False, bReset=False, bBeamBreak=False,
            bDownstreamClear=True,
            bInput1Ready=True, stInput1Tote=tote1,
            bInput2Ready=False, stInput2Tote=None,
            eMergePriority=E_MergePriority.FIFO,
        )
        fb2.execute()
        # input 1 was accepted (only one ready) — that's fine, but let's test tie-break
        # when both are ready simultaneously.
        # For a cleaner test: use a fresh FB, set timestamps manually
        fb3 = FB_MergeConveyor("MERGE3")
        fb3._tInput1ReadySince = 100.0   # Earlier
        fb3._tInput2ReadySince = 200.0   # Later
        fb3._bPrevInput1Ready  = True    # Already tracked (no new rising edge)
        fb3._bPrevInput2Ready  = True
        _tick_idle(fb3,
                   bInput1Ready=True, stInput1Tote=tote1,
                   bInput2Ready=True, stInput2Tote=tote2,
                   eMergePriority=E_MergePriority.FIFO)
        assert fb3.bAcceptingFromInput1 is True
        assert fb3.bAcceptingFromInput2 is False

    def test_fifo_selects_input2_when_earlier(self):
        """Input 2 ready timestamp is earlier → FIFO selects input 2."""
        tote1 = _make_tote("T1")
        tote2 = _make_tote("T2")
        fb = FB_MergeConveyor("MERGE")
        fb._tInput1ReadySince = 200.0
        fb._tInput2ReadySince = 100.0
        fb._bPrevInput1Ready  = True
        fb._bPrevInput2Ready  = True
        _tick_idle(fb,
                   bInput1Ready=True, stInput1Tote=tote1,
                   bInput2Ready=True, stInput2Tote=tote2,
                   eMergePriority=E_MergePriority.FIFO)
        assert fb.bAcceptingFromInput2 is True

    def test_alternate_toggles(self):
        """ALTERNATE mode: first accepts from 2 (default _bLastAcceptWas1=True), then 1."""
        tote_a = _make_tote("TA")
        tote_b = _make_tote("TB")
        fb = FB_MergeConveyor("MERGE")

        # First accept: _bLastAcceptWas1=True → selects input 2
        _accept_to_transporting(fb,
                                bInput1Ready=True, stInput1Tote=tote_a,
                                bInput2Ready=True, stInput2Tote=tote_b,
                                eMergePriority=E_MergePriority.ALTERNATE)

        # Release tote: TRANSPORTING→RELEASING (beam off, downstream clear)
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False, bBeamBreak=False,
            bDownstreamClear=True,
            bInput1Ready=False, stInput1Tote=None,
            bInput2Ready=False, stInput2Tote=None,
        )
        fb.execute()
        # One more tick: RELEASING→IDLE (beam still off)
        fb.set_inputs(
            bEnable=True, bEStop=False, bReset=False, bBeamBreak=False,
            bDownstreamClear=True,
            bInput1Ready=False, stInput1Tote=None,
            bInput2Ready=False, stInput2Tote=None,
        )
        fb.execute()
        assert fb.eState == E_ConveyorState.IDLE

        # Second accept: _bLastAcceptWas1=False → selects input 1
        tote_c = _make_tote("TC")
        tote_d = _make_tote("TD")
        _tick_idle(fb,
                   bInput1Ready=True, stInput1Tote=tote_c,
                   bInput2Ready=True, stInput2Tote=tote_d,
                   eMergePriority=E_MergePriority.ALTERNATE)
        assert fb.bAcceptingFromInput1 is True

    def test_alternate_skips_empty_input(self):
        """ALTERNATE: next-in-turn is input 2 but has no tote → accepts from input 1."""
        tote1 = _make_tote("T1")
        fb = FB_MergeConveyor("MERGE")
        # _bLastAcceptWas1=True → next turn is input 2, but input 2 has no tote
        _tick_idle(fb,
                   bInput1Ready=True, stInput1Tote=tote1,
                   bInput2Ready=False, stInput2Tote=None,
                   eMergePriority=E_MergePriority.ALTERNATE)
        assert fb.bAcceptingFromInput1 is True


# ── Single input tests ───────────────────────────────────────────────────

class TestMergeSingleInput:

    def test_only_input1_ready_accepts_input1(self):
        tote = _make_tote("T1")
        fb = FB_MergeConveyor("MERGE")
        _tick_idle(fb, bInput1Ready=True, stInput1Tote=tote)
        assert fb.bAcceptingFromInput1 is True

    def test_only_input2_ready_accepts_input2(self):
        tote = _make_tote("T2")
        fb = FB_MergeConveyor("MERGE")
        _tick_idle(fb, bInput2Ready=True, stInput2Tote=tote)
        assert fb.bAcceptingFromInput2 is True

    def test_no_input_ready_stays_idle(self):
        fb = FB_MergeConveyor("MERGE")
        _tick_idle(fb)
        assert fb.eState == E_ConveyorState.IDLE
        assert fb.bAcceptingFromInput1 is False
        assert fb.bAcceptingFromInput2 is False


# ── Recirculation count ──────────────────────────────────────────────────

class TestMergeRecircCount:

    def test_input2_increments_recirc_count(self):
        """Accepting from INPUT2 increments the tote's iRecircCount."""
        tote = _make_tote("T_RECIRC")
        assert tote.iRecircCount == 0
        fb = FB_MergeConveyor("MERGE")
        _tick_idle(fb, bInput2Ready=True, stInput2Tote=tote)
        assert tote.iRecircCount == 1

    def test_input1_does_not_increment_recirc_count(self):
        """Accepting from INPUT1 does NOT touch iRecircCount."""
        tote = _make_tote("T_IN1")
        fb = FB_MergeConveyor("MERGE")
        _tick_idle(fb, bInput1Ready=True, stInput1Tote=tote)
        assert tote.iRecircCount == 0

    def test_recirc_count_increments_but_no_alarm(self):
        """
        Merge FB increments recirc count but does NOT raise alarm.
        Recirc limit E-Stop is handled by PRG_Main (system-level).
        """
        tote = _make_tote("T_OVER")
        tote.iRecircCount = CFG.iMaxRecircCount - 1  # One below limit
        fb = FB_MergeConveyor("MERGE")
        _tick_idle(fb, bInput2Ready=True, stInput2Tote=tote)
        # Count incremented to limit
        assert tote.iRecircCount == CFG.iMaxRecircCount
        # No alarm from FB — PRG_Main handles it
        assert not any(e.iAlarmCode == 6 for e in ALARMS.aAlarmBuffer)
