"""
function_blocks/fb_routing_engine.py
======================================
IEC 61131-3 equivalent:

    FUNCTION_BLOCK FB_RoutingEngine
    VAR_INPUT
        sBarcode       : STRING;
        eStrategy      : E_RoutingStrategy;
        aSpurCounts    : ARRAY[1..N] OF INT;  (* tote count per spur *)
        iNumChutes     : INT;
    END_VAR
    VAR_OUTPUT
        eChuteAssignment : E_ChuteTarget;
        bAssignComplete  : BOOL;
    END_VAR
    VAR
        _iRoundRobinNext : INT := 1;
    END_VAR

Strategy dispatch mirrors a PLC CASE statement:

    CASE eStrategy OF
        E_RoutingStrategy.LEAST_LOADED:  (* find min spur count *)
        E_RoutingStrategy.ROUND_ROBIN:   (* rotate counter *)
        E_RoutingStrategy.FIXED_TABLE:   (* lookup table, fallback to LEAST_LOADED *)
    END_CASE

To add a new strategy:
  1. Add the enum value to E_RoutingStrategy in enums.py
  2. Add a CASE branch here as a new method _strategy_<name>()
  3. Add the dispatch line in execute()
  Nothing else changes.
"""

from plc.types.enums import E_RoutingStrategy, E_ChuteTarget
from plc.gvl.gvl_config import CFG
from plc.gvl.gvl_inventory import INVENTORY


class FB_RoutingEngine:
    """
    Chute assignment logic. Called by FB_ToteTracker after a successful scan.
    Stateful only for ROUND_ROBIN (tracks next chute index).
    All other strategies are stateless per-call.
    """

    def __init__(self) -> None:
        # ── VAR_INPUT ──────────────────────────────────────────────────────
        self.sBarcode:    str               = ""
        self.eStrategy:   E_RoutingStrategy = CFG.eRoutingStrategy
        self.aSpurCounts: list              = []   # index 0 = chute 1
        self.iNumChutes:  int               = CFG.iNumberOfChutes

        # ── VAR_OUTPUT ─────────────────────────────────────────────────────
        self.eChuteAssignment: E_ChuteTarget = E_ChuteTarget.UNASSIGNED
        self.bAssignComplete:  bool          = False

        # ── VAR (internal) ─────────────────────────────────────────────────
        self._iRoundRobinNext: int = 1   # 1-based chute index

    # ── Input interface ────────────────────────────────────────────────────

    def set_inputs(
        self,
        *,
        sBarcode:    str,
        eStrategy:   E_RoutingStrategy = None,
        aSpurCounts: list              = None,
        iNumChutes:  int               = None,
    ) -> None:
        self.sBarcode    = sBarcode
        self.eStrategy   = eStrategy   if eStrategy   is not None else CFG.eRoutingStrategy
        self.aSpurCounts = aSpurCounts if aSpurCounts is not None else []
        self.iNumChutes  = iNumChutes  if iNumChutes  is not None else CFG.iNumberOfChutes

    # ── Execution ──────────────────────────────────────────────────────────

    def execute(self) -> None:
        """
        CASE dispatch — one branch per strategy.
        Add new strategies by adding a new CASE branch below.
        """
        self.bAssignComplete  = False
        self.eChuteAssignment = E_ChuteTarget.UNASSIGNED

        if not self.sBarcode:
            return

        # ── CASE eStrategy ────────────────────────────────────────────────
        if self.eStrategy == E_RoutingStrategy.LEAST_LOADED:
            self.eChuteAssignment = self._strategy_least_loaded()

        elif self.eStrategy == E_RoutingStrategy.ROUND_ROBIN:
            self.eChuteAssignment = self._strategy_round_robin()

        elif self.eStrategy == E_RoutingStrategy.FIXED_TABLE:
            self.eChuteAssignment = self._strategy_fixed_table()

        # END_CASE
        # ─────────────────────────────────────────────────────────────────

        self.bAssignComplete = (self.eChuteAssignment != E_ChuteTarget.UNASSIGNED)

    # ── Strategy implementations ───────────────────────────────────────────

    def _strategy_least_loaded(self) -> E_ChuteTarget:
        """
        Assign to the chute whose spur has the fewest totes.
        All N chutes are normal destinations (reject is separate branch).
        If all spurs are tied, assigns chute 1.
        If all chutes are at capacity (>= iSpurLength), returns UNASSIGNED
        so the tote recirculates until a slot opens.
        """
        if self.iNumChutes <= 0:
            return E_ChuteTarget.UNASSIGNED

        # Pad spur counts if not enough data provided
        counts = list(self.aSpurCounts) + [0] * self.iNumChutes
        counts = counts[:self.iNumChutes]

        best_chute = 1
        best_count = counts[0]   # 0-indexed: counts[0] = chute 1

        for chute_1based in range(1, self.iNumChutes + 1):
            count = counts[chute_1based - 1]
            if count < best_count:
                best_count = count
                best_chute = chute_1based

        # Capacity check: if best chute is already at spur capacity,
        # all chutes are full — return UNASSIGNED so tote recirculates
        if best_count >= CFG.iSpurLength:
            return E_ChuteTarget.UNASSIGNED

        return E_ChuteTarget(best_chute)

    def _strategy_round_robin(self) -> E_ChuteTarget:
        """
        Rotate through chutes 1 → 2 → ... → N → 1.
        All N chutes are normal destinations (reject is separate branch).
        State: _iRoundRobinNext persists between calls.
        If the next chute is at capacity, try subsequent chutes.
        If ALL chutes are at capacity, return UNASSIGNED (tote recirculates).
        """
        if self.iNumChutes <= 0:
            return E_ChuteTarget.UNASSIGNED

        # Pad spur counts
        counts = list(self.aSpurCounts) + [0] * self.iNumChutes
        counts = counts[:self.iNumChutes]

        # Try each chute starting from current rotation position
        for _ in range(self.iNumChutes):
            candidate = self._iRoundRobinNext

            # Advance counter for next iteration/call
            self._iRoundRobinNext = (self._iRoundRobinNext % self.iNumChutes) + 1

            # Check capacity
            if counts[candidate - 1] < CFG.iSpurLength:
                return E_ChuteTarget(candidate)

        # All chutes at capacity
        return E_ChuteTarget.UNASSIGNED

    def _strategy_fixed_table(self) -> E_ChuteTarget:
        """
        Lookup barcode in GVL_Inventory routing table.
        Falls back to LEAST_LOADED if barcode not in table.
        """
        result = INVENTORY.lookup_chute(self.sBarcode)
        if result == E_ChuteTarget.RECIRCULATE:
            # Not in fixed table — fall back to least loaded
            return self._strategy_least_loaded()
        return result

    # ── Output accessors ───────────────────────────────────────────────────

    def get_assignment(self) -> E_ChuteTarget:
        return self.eChuteAssignment

    def is_complete(self) -> bool:
        return self.bAssignComplete

    def get_outputs(self) -> dict:
        return {
            "eChuteAssignment": self.eChuteAssignment.name,
            "bAssignComplete":  self.bAssignComplete,
        }
