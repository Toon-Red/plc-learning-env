"""
tests/test_fb_scanner_routing_tracker.py
Tests for FB_BarcodeScanner, FB_RoutingEngine, FB_ToteTracker.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from plc.function_blocks.fb_barcode_scanner import FB_BarcodeScanner
from plc.function_blocks.fb_routing_engine import FB_RoutingEngine
from plc.function_blocks.fb_tote_tracker import FB_ToteTracker
from plc.types.enums import E_ChuteTarget, E_RoutingStrategy
from plc.types.structs import ST_ToteData
from plc.gvl.gvl_alarms import ALARMS
from plc.gvl.gvl_inventory import INVENTORY, GVL_Inventory
from plc.gvl.gvl_config import CFG


@pytest.fixture(autouse=True)
def reset_state():
    ALARMS.aAlarmBuffer.clear()
    ALARMS.diActiveAlarmCount = 0
    ALARMS.diUnackAlarmCount  = 0
    # Fresh inventory for each test
    INVENTORY.routing_table.clear()
    INVENTORY.available_pool.clear()
    INVENTORY.active_totes.clear()
    yield


# ═══════════════════════════════════════════════════════════════════
# FB_RoutingEngine
# ═══════════════════════════════════════════════════════════════════

class TestFBRoutingEngine:

    def test_least_loaded_picks_emptiest_spur(self):
        fb = FB_RoutingEngine()
        fb.set_inputs(
            sBarcode="BC-TEST",
            eStrategy=E_RoutingStrategy.LEAST_LOADED,
            aSpurCounts=[3, 0, 1],   # chute1=3, chute2=0, chute3=1
            iNumChutes=3,
        )
        fb.execute()
        # Chute 2 is emptiest
        assert fb.get_assignment() == E_ChuteTarget.CHUTE_2

    def test_least_loaded_picks_chute1_on_tie(self):
        fb = FB_RoutingEngine()
        fb.set_inputs(
            sBarcode="BC-TEST",
            eStrategy=E_RoutingStrategy.LEAST_LOADED,
            aSpurCounts=[0, 0, 0],
            iNumChutes=3,
        )
        fb.execute()
        assert fb.get_assignment() == E_ChuteTarget.CHUTE_1

    def test_round_robin_rotates(self):
        fb = FB_RoutingEngine()
        results = []
        for _ in range(4):
            fb.set_inputs(
                sBarcode="BC-TEST",
                eStrategy=E_RoutingStrategy.ROUND_ROBIN,
                aSpurCounts=[0, 0, 0],
                iNumChutes=3,
            )
            fb.execute()
            results.append(fb.get_assignment())
        # All 3 chutes are normal destinations, rotation: 1→2→3→1
        assert results[0] == E_ChuteTarget.CHUTE_1
        assert results[1] == E_ChuteTarget.CHUTE_2
        assert results[2] == E_ChuteTarget.CHUTE_3
        assert results[3] == E_ChuteTarget.CHUTE_1  # wraps around

    def test_fixed_table_uses_lookup(self):
        INVENTORY.add_barcode("BC-FIXED", E_ChuteTarget.CHUTE_2)
        fb = FB_RoutingEngine()
        fb.set_inputs(
            sBarcode="BC-FIXED",
            eStrategy=E_RoutingStrategy.FIXED_TABLE,
            aSpurCounts=[0, 0, 0],
            iNumChutes=3,
        )
        fb.execute()
        assert fb.get_assignment() == E_ChuteTarget.CHUTE_2

    def test_fixed_table_falls_back_to_least_loaded(self):
        """Barcode not in table → fallback to LEAST_LOADED."""
        fb = FB_RoutingEngine()
        fb.set_inputs(
            sBarcode="BC-UNKNOWN",
            eStrategy=E_RoutingStrategy.FIXED_TABLE,
            aSpurCounts=[0, 3, 0],   # chute1 emptiest
            iNumChutes=3,
        )
        fb.execute()
        assert fb.get_assignment() == E_ChuteTarget.CHUTE_1

    def test_no_assignment_for_empty_barcode(self):
        fb = FB_RoutingEngine()
        fb.set_inputs(sBarcode="", eStrategy=E_RoutingStrategy.LEAST_LOADED,
                      aSpurCounts=[0, 0, 0], iNumChutes=3)
        fb.execute()
        assert fb.is_complete() is False


# ═══════════════════════════════════════════════════════════════════
# FB_BarcodeScanner
# ═══════════════════════════════════════════════════════════════════

class TestFBBarcodeScanner:

    def test_no_scan_when_no_tote(self):
        fb = FB_BarcodeScanner()
        fb.set_inputs(bEnable=True, bEStop=False, bTotePresent=False,
                      sToteID="", rBeltSpeed=50.0)
        fb.execute()
        assert fb.bScanning is False
        assert fb.bScanComplete is False

    def test_scans_when_tote_present(self):
        """With enough ticks at slow speed, scan should eventually succeed."""
        INVENTORY.add_barcode("BC-001", E_ChuteTarget.CHUTE_1)
        INVENTORY.claim_next_tote("T1")   # puts T1→BC-001 in active_totes

        fb = FB_BarcodeScanner()
        succeeded = False
        for _ in range(100):
            fb.set_inputs(bEnable=True, bEStop=False, bTotePresent=True,
                          sToteID="T1", rBeltSpeed=10.0)  # slow = low fail rate
            fb.execute()
            if fb.bScanComplete:
                succeeded = True
                break
        assert succeeded, "Scanner should succeed within 100 attempts at low speed"
        assert fb.get_barcode() == "BC-001"

    def test_no_read_when_tote_leaves_without_scan(self):
        fb = FB_BarcodeScanner()
        # Force all attempts to fail by patching _attempt_scan
        fb._attempt_scan = lambda: False
        # Tote arrives
        fb.set_inputs(bEnable=True, bEStop=False, bTotePresent=True,
                      sToteID="T_NOREAD", rBeltSpeed=50.0)
        fb.execute()
        # Tote leaves
        fb.set_inputs(bEnable=True, bEStop=False, bTotePresent=False,
                      sToteID="", rBeltSpeed=50.0)
        fb.execute()
        assert fb.bNoRead is True
        assert ALARMS.diActiveAlarmCount >= 1

    def test_mismatch_raises_critical_alarm(self):
        INVENTORY.add_barcode("BC-A", E_ChuteTarget.CHUTE_1)
        INVENTORY.claim_next_tote("T_MISMATCH")
        fb = FB_BarcodeScanner()
        fb._attempt_scan = lambda: True
        fb.set_inputs(bEnable=True, bEStop=False, bTotePresent=True,
                      sToteID="T_MISMATCH", rBeltSpeed=50.0,
                      sKnownBarcode="BC-DIFFERENT")
        fb.execute()
        assert fb.bScanFault is True
        assert any(e.iAlarmCode == 10 for e in ALARMS.aAlarmBuffer)

    def test_estop_disables_scanner(self):
        fb = FB_BarcodeScanner()
        fb.set_inputs(bEnable=True, bEStop=True, bTotePresent=True,
                      sToteID="T1", rBeltSpeed=50.0)
        fb.execute()
        assert fb.bScanning is False


# ═══════════════════════════════════════════════════════════════════
# FB_ToteTracker
# ═══════════════════════════════════════════════════════════════════

class TestFBToteTracker:

    def test_register_tote_claims_barcode(self):
        INVENTORY.add_barcode("BC-REG", E_ChuteTarget.CHUTE_1)
        tracker = FB_ToteTracker()
        tracker.set_inputs(bRegisterTote=True, sNewToteID="T_REG")
        tracker.execute()
        assert tracker.is_tote_registered("T_REG")
        assert tracker.get_barcode_for_tote("T_REG") == "BC-REG"

    def test_scan_complete_assigns_chute(self):
        INVENTORY.add_barcode("BC-SCAN", E_ChuteTarget.CHUTE_1)

        tracker = FB_ToteTracker()
        # Register via proper rising-edge path (claim happens inside _register_tote)
        tracker.set_inputs(bRegisterTote=True, sNewToteID="T_SCAN")
        tracker.execute()

        tracker.set_inputs(
            bRegisterTote=False,
            bScanComplete=True,
            sToteIDScanned="T_SCAN",
            sBarcodeScanned="BC-SCAN",
            aSpurCounts=[0, 0, 0],
        )
        tracker.execute()

        # Drain routing queue (max iRoutingDelayMax_ticks ticks)
        for _ in range(CFG.iRoutingDelayMax_ticks):
            tracker.set_inputs(bScanComplete=False, aSpurCounts=[0, 0, 0])
            tracker.execute()

        assert tracker.get_chute_for_tote("T_SCAN") != E_ChuteTarget.UNASSIGNED

    def test_remove_tote_releases_to_inventory(self):
        INVENTORY.add_barcode("BC-REM", E_ChuteTarget.CHUTE_2)

        tracker = FB_ToteTracker()
        tracker.set_inputs(bRegisterTote=True, sNewToteID="T_REM")
        tracker.execute()

        initial_pool = len(INVENTORY.available_pool)
        tracker.set_inputs(bRegisterTote=False, bRemoveTote=True, sRemovedToteID="T_REM")
        tracker.execute()

        assert not tracker.is_tote_registered("T_REM")
        assert len(INVENTORY.available_pool) == initial_pool + 1

    def test_mismatch_raises_alarm_a010(self):
        tracker = FB_ToteTracker()
        # Inject struct directly — tote already known with one barcode
        struct = ST_ToteData()
        struct.sToteID          = "T_MIS"
        struct.sBarcode         = "BC-EXPECTED"
        struct.eChuteAssignment = E_ChuteTarget.UNASSIGNED
        struct.bIsActive        = True
        tracker._tote_structs["T_MIS"] = struct

        tracker.set_inputs(
            bScanComplete=True,
            sToteIDScanned="T_MIS",
            sBarcodeScanned="BC-DIFFERENT",
        )
        tracker.execute()
        assert any(e.iAlarmCode == 10 for e in ALARMS.aAlarmBuffer)

    def test_rising_edge_only_fires_once(self):
        """bRegisterTote held high should only register once."""
        INVENTORY.add_barcode("BC-EDGE", E_ChuteTarget.CHUTE_1)
        tracker = FB_ToteTracker()
        for _ in range(3):
            tracker.set_inputs(bRegisterTote=True, sNewToteID="T_EDGE")
            tracker.execute()
        assert tracker.get_active_count() == 1
