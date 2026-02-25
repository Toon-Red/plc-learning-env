"""
tests/test_fb_estop.py
Verifies FB_EStop latching, reset gating, and alarm behavior.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from plc.function_blocks.fb_estop import FB_EStop
from plc.gvl.gvl_alarms import ALARMS


@pytest.fixture(autouse=True)
def clear_alarms():
    ALARMS.aAlarmBuffer.clear()
    ALARMS.diActiveAlarmCount = 0
    ALARMS.diUnackAlarmCount  = 0
    yield


def make_estop() -> FB_EStop:
    return FB_EStop()


class TestFBEStop:

    def test_initial_state_is_inactive(self):
        fb = make_estop()
        assert fb.is_active() is False

    def test_activates_on_input(self):
        fb = make_estop()
        fb.set_inputs(bEStopInput=True, bReset=False)
        fb.execute()
        assert fb.is_active() is True

    def test_latches_after_input_released(self):
        """E-Stop must stay active even after physical button released."""
        fb = make_estop()
        fb.set_inputs(bEStopInput=True, bReset=False)
        fb.execute()
        fb.set_inputs(bEStopInput=False, bReset=False)
        fb.execute()
        assert fb.is_active() is True

    def test_reset_not_allowed_while_input_held(self):
        """Cannot reset while button is still pressed."""
        fb = make_estop()
        fb.set_inputs(bEStopInput=True, bReset=False)
        fb.execute()
        fb.set_inputs(bEStopInput=True, bReset=True)
        fb.execute()
        assert fb.is_active() is True

    def test_reset_clears_latch_when_input_released(self):
        fb = make_estop()
        # Trigger
        fb.set_inputs(bEStopInput=True, bReset=False)
        fb.execute()
        # Release physical button
        fb.set_inputs(bEStopInput=False, bReset=False)
        fb.execute()
        # Now send reset pulse
        fb.set_inputs(bEStopInput=False, bReset=True)
        fb.execute()
        assert fb.is_active() is False

    def test_reset_requires_rising_edge(self):
        """Holding bReset=True continuously should not re-reset (already cleared)."""
        fb = make_estop()
        fb.set_inputs(bEStopInput=True, bReset=False)
        fb.execute()
        fb.set_inputs(bEStopInput=False, bReset=True)
        fb.execute()
        assert fb.is_active() is False
        # bReset still held — should stay cleared
        fb.execute()
        assert fb.is_active() is False

    def test_reset_allowed_flag(self):
        fb = make_estop()
        fb.set_inputs(bEStopInput=True, bReset=False)
        fb.execute()
        assert fb.is_reset_allowed() is False
        fb.set_inputs(bEStopInput=False, bReset=False)
        fb.execute()
        assert fb.is_reset_allowed() is True

    def test_alarm_raised_on_activation(self):
        fb = make_estop()
        fb.set_inputs(bEStopInput=True, bReset=False)
        fb.execute()
        assert ALARMS.diActiveAlarmCount == 1
        assert ALARMS.aAlarmBuffer[0].iAlarmCode == 5

    def test_alarm_not_duplicated_on_hold(self):
        """Holding E-Stop should not keep raising new alarms."""
        fb = make_estop()
        for _ in range(5):
            fb.set_inputs(bEStopInput=True, bReset=False)
            fb.execute()
        assert ALARMS.diActiveAlarmCount == 1
