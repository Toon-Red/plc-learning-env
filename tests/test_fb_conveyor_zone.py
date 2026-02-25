"""
tests/test_fb_conveyor_zone.py
Tests the base conveyor zone state machine.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from plc.function_blocks.fb_conveyor_zone import FB_ConveyorZone
from plc.types.enums import E_ConveyorState
from plc.types.structs import ST_ToteData
from plc.gvl.gvl_config import CFG


def make_zone(zone_id="TEST") -> FB_ConveyorZone:
    return FB_ConveyorZone(zone_id)


def make_tote(barcode="BC-1001") -> ST_ToteData:
    t = ST_ToteData()
    t.sBarcode = barcode
    return t


def tick(zone: FB_ConveyorZone, **kwargs) -> None:
    """Helper: set inputs and execute one scan."""
    zone.set_inputs(**kwargs)
    zone.execute()


class TestInitialState:

    def test_starts_idle(self):
        z = make_zone()
        assert z.get_state() == E_ConveyorState.IDLE

    def test_starts_not_faulted(self):
        z = make_zone()
        assert z.is_faulted() is False

    def test_starts_not_ready_when_disabled(self):
        z = make_zone()
        tick(z, bEnable=False, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True)
        assert z.is_ready() is False

    def test_ready_when_enabled_and_idle(self):
        z = make_zone()
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True)
        assert z.is_ready() is True


class TestToteHandling:

    def test_accepts_tote_when_idle_and_ready(self):
        z = make_zone()
        tote = make_tote()
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        assert z.get_state() == E_ConveyorState.ACCEPTING

    def test_will_not_accept_second_tote(self):
        z = make_zone()
        tote1 = make_tote("BC-1001")
        tote2 = make_tote("BC-1002")
        # Accept first tote
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote1, bAcceptTote=True)
        assert z.get_state() == E_ConveyorState.ACCEPTING
        # Try to shove second tote in
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=True,
             stIncomingTote=tote2, bAcceptTote=True)
        # Should be TRANSPORTING, not accepting a new one
        assert z.get_state() == E_ConveyorState.TRANSPORTING
        assert z.get_tote_data().sBarcode == "BC-1001"

    def test_transports_when_downstream_clear(self):
        z = make_zone()
        tote = make_tote()
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        # Beam break fires (tote centered)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=True)
        assert z.get_state() == E_ConveyorState.TRANSPORTING
        assert z.bMotorRun is True

    def test_waits_when_downstream_blocked(self):
        # State machine takes one scan per transition (matches real PLC CASE behavior).
        # Tick 1: IDLE → ACCEPTING (tote accepted, no beam break yet)
        # Tick 2: ACCEPTING → TRANSPORTING (beam break fires)
        # Tick 3: TRANSPORTING → WAITING (downstream blocked)
        z = make_zone()
        tote = make_tote()
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        assert z.get_state() == E_ConveyorState.TRANSPORTING  # beam break just fired
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)  # downstream still blocked
        assert z.get_state() == E_ConveyorState.WAITING
        assert z.bMotorRun is False

    def test_releases_when_downstream_clears(self):
        z = make_zone()
        tote = make_tote()
        # Tick 1: IDLE → ACCEPTING
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        # Tick 2: ACCEPTING → TRANSPORTING (beam break fires, downstream blocked)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        # Tick 3: TRANSPORTING → WAITING (downstream still blocked)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        assert z.get_state() == E_ConveyorState.WAITING
        # Tick 4: downstream clears → RELEASING
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=True)
        assert z.get_state() == E_ConveyorState.RELEASING
        assert z.bMotorRun is True


class TestEStop:

    def test_estop_stops_motor_immediately(self):
        z = make_zone()
        tote = make_tote()
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        tick(z, bEnable=True, bEStop=True, bReset=False,
             bBeamBreak=True, bDownstreamClear=True)
        assert z.bMotorRun is False
        assert z.get_state() == E_ConveyorState.FAULTED

    def test_estop_from_any_state(self):
        for state_setup in [
            dict(bBeamBreak=False, bDownstreamClear=True, bAcceptTote=False),
            dict(bBeamBreak=True,  bDownstreamClear=True, bAcceptTote=False),
            dict(bBeamBreak=True,  bDownstreamClear=False, bAcceptTote=False),
        ]:
            z = make_zone()
            tick(z, bEnable=True, bEStop=False, bReset=False, **state_setup)
            tick(z, bEnable=True, bEStop=True, bReset=False,
                 bBeamBreak=True, bDownstreamClear=True)
            assert z.get_state() == E_ConveyorState.FAULTED


class TestWaitingBehavior:
    """Waiting is NORMAL — zone holds tote until downstream clears or robot picks it."""

    def test_waiting_is_not_a_fault(self):
        """Zone can wait indefinitely without faulting."""
        z = make_zone()
        tote = make_tote()
        # Get tote to WAITING state
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        assert z.get_state() == E_ConveyorState.WAITING
        # Many more ticks — still WAITING, never FAULTED
        for _ in range(100):
            tick(z, bEnable=True, bEStop=False, bReset=False,
                 bBeamBreak=True, bDownstreamClear=False)
        assert z.get_state() == E_ConveyorState.WAITING
        assert not z.is_faulted()

    def test_robot_pickup_clears_waiting(self):
        """Beam break clearing while WAITING = tote removed (robot pickup) → IDLE."""
        z = make_zone()
        tote = make_tote()
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        assert z.get_state() == E_ConveyorState.WAITING
        # Beam break clears (robot picked up tote)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=False)
        assert z.get_state() == E_ConveyorState.IDLE
        assert not z.has_tote()


class TestReset:

    def test_reset_clears_fault(self):
        z = make_zone()
        tick(z, bEnable=True, bEStop=True, bReset=False,
             bBeamBreak=False, bDownstreamClear=True)
        assert z.get_state() == E_ConveyorState.FAULTED
        # E-Stop cleared, then reset
        tick(z, bEnable=True, bEStop=False, bReset=True,
             bBeamBreak=False, bDownstreamClear=True)
        assert z.get_state() == E_ConveyorState.IDLE

    def test_reset_preserves_tote_when_beam_active(self):
        """After E-Stop + Reset, if beam break is still active, tote persists."""
        z = make_zone()
        tote = make_tote()
        # Get tote onto zone: IDLE → ACCEPTING → TRANSPORTING
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=True)
        assert z.get_state() == E_ConveyorState.TRANSPORTING
        assert z.has_tote()

        # E-Stop with tote present
        tick(z, bEnable=True, bEStop=True, bReset=False,
             bBeamBreak=True, bDownstreamClear=True)
        assert z.get_state() == E_ConveyorState.FAULTED
        assert z.has_tote()  # Tote struct preserved in FAULTED

        # Release E-Stop + Reset — beam still active (tote physically here)
        tick(z, bEnable=True, bEStop=False, bReset=True,
             bBeamBreak=True, bDownstreamClear=True)
        # Tote should be preserved, zone should be TRANSPORTING (downstream clear)
        assert z.has_tote()
        assert z.get_tote_data().sBarcode == "BC-1001"
        assert z.get_state() == E_ConveyorState.TRANSPORTING

    def test_reset_preserves_tote_waiting_when_blocked(self):
        """After E-Stop + Reset, beam active + downstream blocked → WAITING."""
        z = make_zone()
        tote = make_tote()
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=False, bDownstreamClear=True,
             stIncomingTote=tote, bAcceptTote=True)
        tick(z, bEnable=True, bEStop=False, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        # E-Stop
        tick(z, bEnable=True, bEStop=True, bReset=False,
             bBeamBreak=True, bDownstreamClear=False)
        assert z.get_state() == E_ConveyorState.FAULTED
        # Release + Reset — downstream still blocked
        tick(z, bEnable=True, bEStop=False, bReset=True,
             bBeamBreak=True, bDownstreamClear=False)
        assert z.has_tote()
        assert z.get_state() == E_ConveyorState.WAITING

    def test_reset_not_active_during_normal_operation(self):
        z = make_zone()
        # Reset pulse when not faulted should have no effect
        tick(z, bEnable=True, bEStop=False, bReset=True,
             bBeamBreak=False, bDownstreamClear=True)
        assert z.get_state() == E_ConveyorState.IDLE  # stays IDLE, not RESETTING
