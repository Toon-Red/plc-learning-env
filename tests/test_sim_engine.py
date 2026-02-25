"""
tests/test_sim_engine.py
Tests the physics simulation layer.
"""

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from plc.sim.sim_engine import SimEngine
from plc.gvl.gvl_config import CFG
from plc.gvl.gvl_alarms import ALARMS


@pytest.fixture(autouse=True)
def clear_alarms():
    ALARMS.aAlarmBuffer.clear()
    ALARMS.diActiveAlarmCount = 0
    ALARMS.diUnackAlarmCount  = 0
    yield


def make_sim_two_zones() -> SimEngine:
    """Two 60cm zones with a 2cm gap. Default sensor at center."""
    sim = SimEngine()
    sim.register_zone("Z1", start_cm=0.0,   length_cm=60.0, gap_after_cm=2.0)
    sim.register_zone("Z2", start_cm=62.0,  length_cm=60.0, gap_after_cm=2.0)
    return sim


def make_sim_three_zones() -> SimEngine:
    sim = SimEngine()
    sim.register_zone("Z1", start_cm=0.0,   length_cm=60.0, gap_after_cm=2.0)
    sim.register_zone("Z2", start_cm=62.0,  length_cm=60.0, gap_after_cm=2.0)
    sim.register_zone("Z3", start_cm=124.0, length_cm=60.0, gap_after_cm=2.0)
    return sim


def motors_on(*zone_ids) -> dict:
    return {z: True for z in zone_ids}


def motors_off(*zone_ids) -> dict:
    return {z: False for z in zone_ids}


def speeds(*zone_ids, pct=50.0) -> dict:
    return {z: pct for z in zone_ids}


class TestZoneRegistration:

    def test_registers_zones(self):
        sim = make_sim_two_zones()
        assert sim.get_zone("Z1") is not None
        assert sim.get_zone("Z2") is not None

    def test_sensor_at_center_by_default(self):
        sim = make_sim_two_zones()
        z1 = sim.get_zone("Z1")
        assert z1.sensor_cm == 30.0   # 0 + 60/2

    def test_zone_sequence_order(self):
        sim = make_sim_two_zones()
        assert sim.get_zone_sequence() == ["Z1", "Z2"]


class TestToteSpawn:

    def test_spawns_in_empty_zone(self):
        sim = make_sim_two_zones()
        result = sim.spawn_tote("T1", "Z1")
        assert result is True
        assert sim.get_tote_in_zone("Z1") == "T1"

    def test_blocked_if_zone_occupied(self):
        sim = make_sim_two_zones()
        sim.spawn_tote("T1", "Z1")
        sim.tick(motors_off("Z1", "Z2"), speeds("Z1", "Z2"))
        result = sim.spawn_tote("T2", "Z1")
        assert result is False

    def test_remove_tote(self):
        sim = make_sim_two_zones()
        sim.spawn_tote("T1", "Z1")
        sim.remove_tote("T1")
        assert sim.get_tote_in_zone("Z1") is None
        assert sim.active_tote_count() == 0


class TestBeamBreaks:

    def test_beam_break_fires_when_tote_at_sensor(self):
        sim = make_sim_two_zones()
        sim.spawn_tote("T1", "Z1")
        # Run ticks until tote reaches Z1 sensor (cm 30)
        for _ in range(100):
            sim.tick(motors_on("Z1"), speeds("Z1", "Z2"))
            if sim.get_beam_break("Z1"):
                break
        assert sim.get_beam_break("Z1") is True

    def test_beam_break_false_when_zone_empty(self):
        sim = make_sim_two_zones()
        sim.tick(motors_on("Z1", "Z2"), speeds("Z1", "Z2"))
        assert sim.get_beam_break("Z1") is False
        assert sim.get_beam_break("Z2") is False

    def test_tote_triggers_second_zone_sensor(self):
        sim = make_sim_two_zones()
        sim.spawn_tote("T1", "Z1")
        # Run until Z2 sensor fires
        fired = False
        for _ in range(500):
            sim.tick(motors_on("Z1", "Z2"), speeds("Z1", "Z2"))
            if sim.get_beam_break("Z2"):
                fired = True
                break
        assert fired, "Tote never reached Z2 sensor"

    def test_small_tote_can_be_between_zones(self):
        """A tote smaller than the gap triggers neither sensor while in the gap."""
        sim = SimEngine()
        # 60cm zones with a 10cm gap. Tote length = 5cm (smaller than gap).
        sim.register_zone("Z1", start_cm=0.0,  length_cm=60.0, gap_after_cm=10.0)
        sim.register_zone("Z2", start_cm=70.0, length_cm=60.0, gap_after_cm=2.0)
        CFG.rToteLength_cm = 5.0  # Override for this test
        try:
            sim.spawn_tote("T1", "Z1")
            # Run until tote is somewhere in the 60–70cm gap
            in_gap = False
            for _ in range(500):
                sim.tick(motors_on("Z1", "Z2"), speeds("Z1", "Z2"))
                tote = sim._totes.get("T1")
                if tote and 62.0 <= tote.rear_cm and tote.position_cm <= 70.0:
                    in_gap = True
                    break
            if in_gap:
                # Neither beam break should be active
                assert sim.get_beam_break("Z1") is False
                assert sim.get_beam_break("Z2") is False
        finally:
            CFG.rToteLength_cm = 40.0  # Restore

    def test_large_tote_spans_two_zones(self):
        """A tote larger than a zone can trigger two sensors simultaneously."""
        sim = SimEngine()
        # 30cm zones, 1cm gap, tote = 40cm (larger than zone)
        sim.register_zone("Z1", start_cm=0.0,  length_cm=30.0, gap_after_cm=1.0)
        sim.register_zone("Z2", start_cm=31.0, length_cm=30.0, gap_after_cm=1.0)
        CFG.rToteLength_cm = 40.0
        sim.spawn_tote("T1", "Z1")
        both_triggered = False
        for _ in range(200):
            sim.tick(motors_on("Z1", "Z2"), speeds("Z1", "Z2"))
            if sim.get_beam_break("Z1") and sim.get_beam_break("Z2"):
                both_triggered = True
                break
        assert both_triggered, "Large tote should trigger both sensors when spanning zones"


class TestMotorAndSlip:

    def test_tote_doesnt_move_when_motor_off(self):
        sim = make_sim_two_zones()
        sim.spawn_tote("T1", "Z1")
        initial_pos = sim.get_tote_position("T1")
        for _ in range(10):
            sim.tick(motors_off("Z1", "Z2"), speeds("Z1", "Z2"))
        assert sim.get_tote_position("T1") == initial_pos

    def test_faster_speed_advances_tote_further(self):
        sim1 = make_sim_two_zones()
        sim2 = make_sim_two_zones()
        sim1.spawn_tote("T1", "Z1")
        sim2.spawn_tote("T1", "Z1")
        TICKS = 20
        for _ in range(TICKS):
            sim1.tick(motors_on("Z1", "Z2"), speeds("Z1", "Z2", pct=25.0))
            sim2.tick(motors_on("Z1", "Z2"), speeds("Z1", "Z2", pct=75.0))
        assert sim2.get_tote_position("T1") > sim1.get_tote_position("T1")
