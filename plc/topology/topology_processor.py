"""
topology/topology_processor.py
==============================
Generic tick cycle driven by topology graph.
Replaces hardcoded steps 5-13 in PRG_Main.tick().

Three zone processing paths:
  1. Generic (transport/infeed/end/scanner): downstream_clear + upstream_tote → execute()
  2. Branch: two downstream clear signals → execute() → handle divert cross-track
  3. Merge: two upstream inputs (one may be cross-track) → execute() → handle accept spawns

Transfer areas: Dict[str, Optional[ST_ToteData]] keyed by "{source}→{target}".
When a zone at end-of-track releases, tote goes into transfer area.
When target zone executes, it pulls from transfer area.

Signal bindings: Pre-computed at init time. Each zone gets a _ZoneBinding with
lambdas for downstream_clear, merge inputs, branch outputs. The scan loop just
calls these — no graph walking or transfer area searching per tick.
"""

from typing import Dict, List, Optional, Tuple

from plc.types.structs import ST_ToteData, ST_CrossTrackTransfer
from plc.types.enums import E_MergePriority
from plc.gvl.gvl_config import CFG
from plc.gvl.gvl_hmi import HMI
from plc.function_blocks.fb_conveyor_zone import FB_ConveyorZone
from plc.function_blocks.fb_branch_conveyor import FB_BranchConveyor
from plc.function_blocks.fb_merge_conveyor import FB_MergeConveyor


class _ZoneBinding:
    """
    Pre-computed signal bindings for a single zone.
    Created once at topology-apply time, called every scan.
    Analogous to PLC pointer-based variable binding (ADR/DEREF).
    """
    __slots__ = (
        'fn_downstream_clear',
        'fn_input1', 'fn_input2',
        'fn_accept_input1', 'fn_accept_input2',
        'fn_straight_clear', 'fn_divert_clear',
    )

    def __init__(self):
        self.fn_downstream_clear = _false       # () -> bool
        # Merge-specific
        self.fn_input1 = None                   # () -> Optional[ST_ToteData]
        self.fn_input2 = None                   # () -> Optional[ST_ToteData]
        self.fn_accept_input1 = None            # () -> None  (clear transfer + spawn)
        self.fn_accept_input2 = None            # () -> None
        # Branch-specific
        self.fn_straight_clear = None           # () -> bool
        self.fn_divert_clear = None             # () -> bool


# Shared no-op lambdas (avoid creating identical closures)
def _false():
    return False

def _none():
    return None


class TopologyProcessor:
    """
    Generic zone processing engine driven by an ST_TopologyGraph.
    Processes all tracks in dependency order. Within each track,
    zones process upstream→downstream.

    Signal bindings are pre-computed in __init__ so the scan loop
    is a simple call chain with no graph traversal.
    """

    def __init__(self, topology) -> None:
        """
        Args:
            topology: ST_TopologyGraph from TopologyBuilder.compile()
        """
        self._topo = topology
        self._conn = topology.connection_graph

        # Transfer areas: "{source_zone}→{target_zone}" → Optional[ST_ToteData]
        self.transfer_areas: Dict[str, Optional[ST_ToteData]] = {}
        self.transfer_tote_ids: Dict[str, str] = {}  # Same key → tote_id

        # Initialize transfer areas for all cross-track connections
        for xfer in topology.cross_transfers:
            self.transfer_areas[xfer.sTransferKey] = None
            self.transfer_tote_ids[xfer.sTransferKey] = ""

        # ── Compile-time signal bindings ─────────────────────────────────
        self._bindings: Dict[str, _ZoneBinding] = {}
        self._build_bindings()

    # ════════════════════════════════════════════════════════════════════════
    # BINDING CONSTRUCTION (runs once at init)
    # ════════════════════════════════════════════════════════════════════════

    def _build_bindings(self) -> None:
        """Create pre-bound signal lambdas for every zone."""
        topo = self._topo

        for track, zone_ids in topo.track_zone_order.items():
            sim = topo.sim_engines.get(track)
            for i, zone_id in enumerate(zone_ids):
                fb = topo.zone_fbs[zone_id]
                desc = topo.descriptors[zone_id]
                b = _ZoneBinding()

                if isinstance(fb, FB_BranchConveyor):
                    self._bind_branch(b, zone_id, track)
                elif isinstance(fb, FB_MergeConveyor):
                    self._bind_merge(b, zone_id, track, zone_ids, i, sim)
                else:
                    self._bind_downstream(b, zone_id, desc, track, zone_ids, i)

                self._bindings[zone_id] = b

    def _bind_downstream(self, b: _ZoneBinding, zone_id: str, desc, track: str,
                         zone_ids: List[str], index: int) -> None:
        """Bind fn_downstream_clear for a generic (non-branch) zone."""
        ta = self.transfer_areas

        if index + 1 < len(zone_ids):
            # Normal: next zone on same track
            nfb = self._topo.zone_fbs[zone_ids[index + 1]]
            b.fn_downstream_clear = lambda _fb=nfb: _fb.is_ready()
            return

        # Last zone on track — find cross-track transfer if any
        for xfer in self._topo.cross_transfers:
            if xfer.sSourceZone == zone_id:
                key = xfer.sTransferKey
                b.fn_downstream_clear = lambda _k=key: ta.get(_k) is None
                return

        # No transfer — pickup point or dead end
        b.fn_downstream_clear = _false

    def _bind_branch(self, b: _ZoneBinding, zone_id: str, track: str) -> None:
        """Bind fn_straight_clear and fn_divert_clear for a branch zone."""
        ta = self.transfer_areas
        conn = self._conn

        # ── Straight downstream ──────────────────────────────────────────
        straight_down = conn.get_primary_downstream(zone_id)
        if straight_down and self._topo.zone_to_track.get(straight_down) == track:
            sfb = self._topo.zone_fbs[straight_down]
            b.fn_straight_clear = lambda _fb=sfb: _fb.is_ready()
        elif straight_down:
            # Cross-track straight (rare)
            for xfer in self._topo.cross_transfers:
                if xfer.sSourceZone == zone_id and xfer.sTargetZone == straight_down:
                    key = xfer.sTransferKey
                    b.fn_straight_clear = lambda _k=key: ta.get(_k) is None
                    break
            else:
                b.fn_straight_clear = _false
        else:
            b.fn_straight_clear = _false

        # ── Divert downstream ────────────────────────────────────────────
        divert_down = conn.get_divert_downstream(zone_id)
        if divert_down:
            dfb = self._topo.zone_fbs[divert_down]
            b.fn_divert_clear = lambda _fb=dfb: _fb.is_ready()
        else:
            b.fn_divert_clear = _false

    def _bind_merge(self, b: _ZoneBinding, zone_id: str, track: str,
                    zone_ids: List[str], index: int, sim) -> None:
        """Bind merge inputs and downstream clear from inferred sources."""
        ta = self.transfer_areas

        # ── Downstream clear (same as generic) ───────────────────────────
        if index + 1 < len(zone_ids):
            nfb = self._topo.zone_fbs[zone_ids[index + 1]]
            b.fn_downstream_clear = lambda _fb=nfb: _fb.is_ready()
        else:
            b.fn_downstream_clear = _false

        # ── Get inferred input sources ───────────────────────────────────
        input1_zone, input2_zone = self._topo.merge_input_sources.get(
            zone_id, (None, None)
        )

        def _needs_transfer_area(src_zone: str) -> bool:
            """Check if source uses transfer area (cross-track or cycle wrap)."""
            src_track = self._topo.zone_to_track.get(src_zone, "")
            if src_track != track:
                return True  # Cross-track
            # Same track: check if cycle wrap (source is after merge in order)
            src_idx = zone_ids.index(src_zone) if src_zone in zone_ids else -1
            return src_idx > index  # Later in zone order = cycle wrap

        # ── Input1 (typically same-track / main feed) ────────────────────
        if input1_zone:
            if _needs_transfer_area(input1_zone):
                key = f"{input1_zone}\u2192{zone_id}"
                b.fn_input1 = lambda _k=key: ta.get(_k)
                b.fn_accept_input1 = self._make_xfer_acceptor(key, sim, zone_id)
            else:
                # Same-track direct passthrough
                i1_fb = self._topo.zone_fbs.get(input1_zone)
                if isinstance(i1_fb, FB_BranchConveyor):
                    b.fn_input1 = lambda _fb=i1_fb: _fb.get_straight_tote()
                elif i1_fb:
                    b.fn_input1 = lambda _fb=i1_fb: _fb.get_passthrough_tote()
                else:
                    b.fn_input1 = _none
                b.fn_accept_input1 = None
        else:
            b.fn_input1 = _none

        # ── Input2 (typically cross-track / alternate feed) ──────────────
        if input2_zone:
            if _needs_transfer_area(input2_zone):
                key = f"{input2_zone}\u2192{zone_id}"
                b.fn_input2 = lambda _k=key: ta.get(_k)
                b.fn_accept_input2 = self._make_xfer_acceptor(key, sim, zone_id)
            else:
                # Same-track direct passthrough (edge case)
                i2_fb = self._topo.zone_fbs.get(input2_zone)
                if i2_fb:
                    b.fn_input2 = lambda _fb=i2_fb: _fb.get_passthrough_tote()
                else:
                    b.fn_input2 = _none
                b.fn_accept_input2 = None
        else:
            b.fn_input2 = _none

    def _make_xfer_acceptor(self, key: str, sim, zone_id: str):
        """Create a closure that clears a transfer area and spawns tote in sim."""
        ta = self.transfer_areas
        ta_ids = self.transfer_tote_ids

        def _accept():
            tote_id = ta_ids.get(key, "")
            if tote_id and sim:
                sim.spawn_tote(tote_id, zone_id)
            ta[key] = None
            ta_ids[key] = ""
        return _accept

    # ════════════════════════════════════════════════════════════════════════
    # PUBLIC: Process all tracks
    # ════════════════════════════════════════════════════════════════════════

    def process_all_tracks(
        self,
        bEStopActive: bool,
        bEnable: bool,
        bReset: bool,
        fbTracker=None,
    ) -> None:
        """
        Process every track in dependency order.
        Within each track, zones are processed upstream→downstream.
        Handles cross-track transfers automatically.

        Args:
            bEStopActive: E-Stop is active
            bEnable: System is RUNNING
            bReset: Reset pulse this tick
            fbTracker: FB_ToteTracker for spawn zone tote lookup
        """
        for track in self._topo.track_process_order:
            self._process_track(track, bEStopActive, bEnable, bReset, fbTracker)

    # ════════════════════════════════════════════════════════════════════════
    # PRIVATE: Process one track (uses pre-bound signals)
    # ════════════════════════════════════════════════════════════════════════

    def _process_track(
        self,
        track: str,
        bEStopActive: bool,
        bEnable: bool,
        bReset: bool,
        fbTracker,
    ) -> None:
        """Process all zones in a single track, upstream→downstream."""
        zone_ids = self._topo.track_zone_order.get(track, [])
        sim = self._topo.sim_engines.get(track)
        if not sim:
            return

        for i, zone_id in enumerate(zone_ids):
            fb = self._topo.zone_fbs[zone_id]
            desc = self._topo.descriptors[zone_id]
            b = self._bindings[zone_id]
            beam_break = sim.get_beam_break(zone_id)
            speed = HMI.rHMI_SpeedOverride.get(zone_id, CFG.rDefaultBeltSpeed)

            if isinstance(fb, FB_MergeConveyor):
                # ── Merge: use pre-bound input signals ───────────────────
                priority = (E_MergePriority[desc.eMergePriority]
                            if desc.eMergePriority
                            else CFG.eMergePriorityDefault)
                input1_tote = b.fn_input1()
                input2_tote = b.fn_input2()

                fb.set_inputs(
                    bEnable=bEnable,
                    bEStop=bEStopActive,
                    bReset=bReset,
                    bBeamBreak=beam_break,
                    bDownstreamClear=b.fn_downstream_clear(),
                    rSpeedPercent=speed,
                    bInput1Ready=input1_tote is not None,
                    stInput1Tote=input1_tote,
                    bInput2Ready=input2_tote is not None,
                    stInput2Tote=input2_tote,
                    eMergePriority=priority,
                )
                fb.execute()

                # Handle acceptance via pre-bound acceptors
                if fb.is_accepting_from_input1() and b.fn_accept_input1:
                    b.fn_accept_input1()
                if fb.is_accepting_from_input2() and b.fn_accept_input2:
                    b.fn_accept_input2()

            elif isinstance(fb, FB_BranchConveyor):
                # ── Branch: use pre-bound straight/divert clear ──────────
                upstream_tote, accept_signal = self._get_upstream_handoff(
                    zone_id, desc, track, sim, fbTracker, zone_ids, i,
                )
                fb.set_inputs(
                    bEnable=bEnable,
                    bEStop=bEStopActive,
                    bReset=bReset,
                    bBeamBreak=beam_break,
                    bStraightDownstreamClear=b.fn_straight_clear(),
                    bDivertDownstreamClear=b.fn_divert_clear(),
                    rSpeedPercent=speed,
                    stIncomingTote=upstream_tote,
                    bAcceptTote=accept_signal,
                )
                fb.execute()

            else:
                # ── Generic zone: use pre-bound downstream clear ─────────
                upstream_tote, accept_signal = self._get_upstream_handoff(
                    zone_id, desc, track, sim, fbTracker, zone_ids, i,
                )
                fb.set_inputs(
                    bEnable=bEnable,
                    bEStop=bEStopActive,
                    bReset=bReset,
                    bBeamBreak=beam_break,
                    bDownstreamClear=b.fn_downstream_clear(),
                    rSpeedPercent=speed,
                    stIncomingTote=upstream_tote,
                    bAcceptTote=accept_signal,
                )
                fb.execute()

        # After processing all zones in this track, handle end-of-track releases
        self._handle_track_end_releases(track, zone_ids, sim)

        # Handle branch divert cross-track transfers for this track
        self._handle_branch_diverts_for_track(track, sim)

    # ════════════════════════════════════════════════════════════════════════
    # UPSTREAM HANDOFF (kept as method — spawn logic is stateful)
    # ════════════════════════════════════════════════════════════════════════

    def _get_upstream_handoff(
        self,
        zone_id: str,
        desc: 'ST_ZoneDescriptor',
        track: str,
        sim,
        fbTracker,
        track_zone_ids: List[str],
        zone_index: int,
    ) -> Tuple[Optional[ST_ToteData], bool]:
        """Get the tote being passed from upstream."""
        # Spawn point: tote comes from ToteTracker on spawn
        if desc.bIsSpawnPoint and zone_index == 0 and fbTracker:
            return self._get_spawn_handoff(zone_id, track, sim, fbTracker, track_zone_ids)

        # First zone on track: check upstream sources
        if zone_index == 0:
            # Check for branch divert upstream (branch → spur/reject first zone)
            for xfer in self._topo.cross_transfers:
                if xfer.sTargetZone == zone_id and xfer.sTargetTrack == track:
                    # If the source is a branch and this is a divert connection,
                    # read the diverted tote directly from the branch FB
                    src_fb = self._topo.zone_fbs.get(xfer.sSourceZone)
                    if isinstance(src_fb, FB_BranchConveyor):
                        tote = src_fb.get_diverted_tote()
                        if tote is not None:
                            return tote, True

                    # Otherwise check transfer area (end-of-track → merge input)
                    tote = self.transfer_areas.get(xfer.sTransferKey)
                    if tote is not None:
                        return tote, True
            return None, False

        # Normal: get passthrough from previous zone on same track
        prev_zone_id = track_zone_ids[zone_index - 1]
        prev_fb = self._topo.zone_fbs[prev_zone_id]

        # If previous is a branch, get the straight-through tote
        if isinstance(prev_fb, FB_BranchConveyor):
            tote = prev_fb.get_straight_tote()
            return tote, tote is not None

        tote = prev_fb.get_passthrough_tote()
        return tote, tote is not None

    def _get_spawn_handoff(
        self,
        zone_id: str,
        track: str,
        sim,
        fbTracker,
        track_zone_ids: List[str],
    ) -> Tuple[Optional[ST_ToteData], bool]:
        """Handle spawn zone tote acquisition from sim."""
        tote_id = sim.get_tote_in_zone(zone_id)
        if not tote_id:
            return None, False

        fb = self._topo.zone_fbs[zone_id]

        # Guard: don't re-acquire a tote that the next zone already accepted
        if len(track_zone_ids) > 1:
            next_zone_id = track_zone_ids[1]
            next_tote = self._topo.zone_fbs[next_zone_id].get_tote_data()
            if next_tote and next_tote.sToteID == tote_id:
                return None, False

        struct = fbTracker.get_tote_struct(tote_id)
        if struct and not fb.has_tote():
            return struct, True
        return None, False

    # ════════════════════════════════════════════════════════════════════════
    # BRANCH DIVERT CROSS-TRACK TRANSFERS
    # ════════════════════════════════════════════════════════════════════════

    def _handle_branch_diverts_for_track(self, track: str, sim) -> None:
        """Handle branch divert totes that need to move to a different sim engine."""
        for zone_id, branch_fb in self._topo.branch_fbs.items():
            if self._topo.zone_to_track.get(zone_id) != track:
                continue

            diverted_tote = branch_fb.get_diverted_tote()
            if diverted_tote is None:
                continue

            tote_id = diverted_tote.sToteID
            divert_down = self._conn.get_divert_downstream(zone_id)
            if not divert_down:
                continue

            divert_track = self._topo.zone_to_track.get(divert_down, "")
            if divert_track != track:
                # Cross-track divert: remove from source sim, spawn in target sim
                sim.remove_tote(tote_id)
                target_sim = self._topo.sim_engines.get(divert_track)
                if target_sim:
                    target_sim.spawn_tote(tote_id, divert_down)

    # ════════════════════════════════════════════════════════════════════════
    # END-OF-TRACK RELEASES
    # ════════════════════════════════════════════════════════════════════════

    def _handle_track_end_releases(
        self, track: str, zone_ids: List[str], sim,
    ) -> None:
        """
        At the end of each track, check if the last zone has released a tote.
        If so, move the tote struct into the appropriate transfer area.
        Also handles force_release for end-of-track zones.
        """
        for xfer in self._topo.cross_transfers:
            if xfer.sSourceTrack != track:
                continue

            # Transfer area already occupied
            if self.transfer_areas.get(xfer.sTransferKey) is not None:
                continue

            source_fb = self._topo.zone_fbs.get(xfer.sSourceZone)
            if not source_fb:
                continue

            # Try normal passthrough first
            tote = source_fb.get_passthrough_tote()

            # Force release for end-of-track zones
            if tote is None and xfer.sSourceZone in self._topo.force_release_zones:
                tote = source_fb.force_release_tote()

            if tote is not None:
                tote_id = tote.sToteID
                sim.remove_tote(tote_id)
                self.transfer_areas[xfer.sTransferKey] = tote
                self.transfer_tote_ids[xfer.sTransferKey] = tote_id

    # ════════════════════════════════════════════════════════════════════════
    # PUBLIC: Access transfer areas (for PRG_Main backward compat)
    # ════════════════════════════════════════════════════════════════════════

    def get_transfer_tote(self, source_zone: str, target_zone: str) -> Optional[ST_ToteData]:
        """Get the tote in a specific transfer area."""
        key = f"{source_zone}→{target_zone}"
        return self.transfer_areas.get(key)

    def get_transfer_tote_id(self, source_zone: str, target_zone: str) -> str:
        """Get the tote ID in a specific transfer area."""
        key = f"{source_zone}→{target_zone}"
        return self.transfer_tote_ids.get(key, "")

    def get_all_transfer_tote_ids(self) -> List[str]:
        """Get all tote IDs currently in transfer areas."""
        return [tid for tid in self.transfer_tote_ids.values() if tid]
