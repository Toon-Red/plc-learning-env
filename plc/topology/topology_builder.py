"""
topology/topology_builder.py
============================
Compiles a list of ST_ZoneDescriptor into an ST_TopologyGraph.
Creates all FBs, SimEngines, connection graph, and detects cross-track transfers.
"""

from typing import Dict, List, Optional
from dataclasses import dataclass, field

from plc.types.structs import ST_ZoneDescriptor, ST_CrossTrackTransfer
from plc.gvl.gvl_config import CFG
from plc.sim.sim_engine import SimEngine
from plc.function_blocks.fb_conveyor_zone import FB_ConveyorZone
from plc.function_blocks.fb_branch_conveyor import FB_BranchConveyor
from plc.function_blocks.fb_merge_conveyor import FB_MergeConveyor
from plc.topology.zone_registry import ZoneRegistry
from plc.topology.connection_graph import ConnectionGraph


@dataclass
class ST_TopologyGraph:
    """
    Runtime data structures produced by TopologyBuilder.compile().
    Everything PRG_Main needs to run the system generically.
    """
    zone_fbs:            Dict[str, FB_ConveyorZone]    = field(default_factory=dict)
    sim_engines:         Dict[str, SimEngine]           = field(default_factory=dict)
    zone_to_track:       Dict[str, str]                 = field(default_factory=dict)
    track_process_order: List[str]                      = field(default_factory=list)
    track_zone_order:    Dict[str, List[str]]           = field(default_factory=dict)
    cross_transfers:     List[ST_CrossTrackTransfer]     = field(default_factory=list)
    spawn_zones:         List[str]                       = field(default_factory=list)
    pickup_zones:        List[str]                       = field(default_factory=list)
    scanner_zones:       List[str]                       = field(default_factory=list)
    force_release_zones: List[str]                       = field(default_factory=list)
    branch_fbs:          Dict[str, FB_BranchConveyor]   = field(default_factory=dict)
    merge_fbs:           Dict[str, FB_MergeConveyor]    = field(default_factory=dict)
    descriptors:         Dict[str, ST_ZoneDescriptor]   = field(default_factory=dict)
    connection_graph:    Optional[ConnectionGraph]       = None
    # Merge input classification: merge_id → (input1_zone, input2_zone)
    # input1 = same-track (main feed), input2 = cross-track (alternate feed)
    merge_input_sources: Dict[str, tuple]                = field(default_factory=dict)


class TopologyBuilder:
    """Compiles zone descriptors into a complete runtime topology."""

    @staticmethod
    def compile(descriptors: List[ST_ZoneDescriptor]) -> ST_TopologyGraph:
        """
        Main entry point.
        Input: flat list of zone descriptors with string connections.
        Output: fully populated ST_TopologyGraph.
        """
        graph = ST_TopologyGraph()
        graph.descriptors = {d.sZoneID: d for d in descriptors}

        # ── 1. Build connection graph ────────────────────────────────────
        conn = ConnectionGraph(descriptors)
        errors = conn.validate()
        if errors:
            raise ValueError(f"Topology validation failed: {errors}")
        graph.connection_graph = conn

        # ── 2. Create FB instances ───────────────────────────────────────
        for desc in descriptors:
            fb = ZoneRegistry.create_fb(desc)
            graph.zone_fbs[desc.sZoneID] = fb

            if isinstance(fb, FB_BranchConveyor):
                graph.branch_fbs[desc.sZoneID] = fb
            elif isinstance(fb, FB_MergeConveyor):
                graph.merge_fbs[desc.sZoneID] = fb

            # Collect behavioral zones
            if desc.bIsSpawnPoint:
                graph.spawn_zones.append(desc.sZoneID)
            if desc.bIsPickupPoint:
                graph.pickup_zones.append(desc.sZoneID)
            if desc.eZoneType == "scanner":
                graph.scanner_zones.append(desc.sZoneID)
            if desc.bForceRelease:
                graph.force_release_zones.append(desc.sZoneID)

        # ── 3. Group zones by track ──────────────────────────────────────
        track_zones: Dict[str, List[str]] = {}
        for desc in descriptors:
            track = desc.sTrackGroup
            track_zones.setdefault(track, []).append(desc.sZoneID)
            graph.zone_to_track[desc.sZoneID] = track

        # ── 4. Sort each track upstream→downstream ───────────────────────
        for track, zone_ids in track_zones.items():
            ordered = conn.topological_order(zone_ids)
            graph.track_zone_order[track] = ordered

        # ── 4b. Infer merge input sources (same-track=input1, cross-track=input2)
        for merge_id in graph.merge_fbs:
            merge_track = graph.zone_to_track[merge_id]
            merge_zone_ids = graph.track_zone_order.get(merge_track, [])
            merge_idx = merge_zone_ids.index(merge_id) if merge_id in merge_zone_ids else 0
            upstreams = conn.get_upstream(merge_id)

            # Categorize upstream sources
            same_track_normal = []   # same track, earlier in order (direct passthrough)
            same_track_cycle = []    # same track, later in order (cycle wrap via transfer)
            cross_track = []         # different track (via transfer area)

            for src_id, _port in upstreams:
                src_track = graph.zone_to_track.get(src_id, "")
                if src_track == merge_track:
                    src_idx = merge_zone_ids.index(src_id) if src_id in merge_zone_ids else -1
                    if src_idx > merge_idx:
                        same_track_cycle.append(src_id)
                    else:
                        same_track_normal.append(src_id)
                else:
                    cross_track.append(src_id)

            # Assign input1 (direct passthrough) and input2 (via transfer area)
            # Priority: same-track normal → input1, cycle/cross → input2
            input1 = same_track_normal[0] if same_track_normal else None
            input2 = None
            if same_track_cycle:
                input2 = same_track_cycle[0]
            if cross_track:
                if input1 is None:
                    input1 = cross_track[0]
                elif input2 is None:
                    input2 = cross_track[0]

            graph.merge_input_sources[merge_id] = (input1, input2)

        # ── 5. Create SimEngine per track ────────────────────────────────
        for track, zone_ids in graph.track_zone_order.items():
            sim = SimEngine()
            pos = 0.0
            for zone_id in zone_ids:
                desc = graph.descriptors[zone_id]
                length = desc.rLength_cm if desc.rLength_cm is not None else CFG.rDefaultZoneLength_cm
                gap = desc.rGap_cm if desc.rGap_cm is not None else CFG.rDefaultGap_cm
                width = desc.rWidth_cm if desc.rWidth_cm is not None else None
                sensor = desc.rSensorOffset_cm

                sim.register_zone(
                    zone_id=zone_id,
                    start_cm=pos,
                    length_cm=length,
                    width_cm=width,
                    sensor_offset_cm=sensor,
                    gap_after_cm=gap,
                )
                pos += length + gap

            graph.sim_engines[track] = sim

        # ── 6. Detect cross-track transfers ──────────────────────────────
        for desc in descriptors:
            src_track = desc.sTrackGroup

            # Primary downstream crossing
            if desc.sDownstream:
                tgt_track = graph.zone_to_track.get(desc.sDownstream, src_track)
                if tgt_track != src_track:
                    graph.cross_transfers.append(ST_CrossTrackTransfer(
                        sSourceZone=desc.sZoneID,
                        sTargetZone=desc.sDownstream,
                        sSourceTrack=src_track,
                        sTargetTrack=tgt_track,
                    ))

            # Divert downstream crossing (branch → spur)
            if desc.sDivertDownstream:
                tgt_track = graph.zone_to_track.get(desc.sDivertDownstream, src_track)
                if tgt_track != src_track:
                    graph.cross_transfers.append(ST_CrossTrackTransfer(
                        sSourceZone=desc.sZoneID,
                        sTargetZone=desc.sDivertDownstream,
                        sSourceTrack=src_track,
                        sTargetTrack=tgt_track,
                    ))

        # ── 6b. Detect cyclic same-track wrap-arounds ──────────────────
        # If the last zone in a track has a downstream that is earlier in
        # the same track (cycle), add a cross-transfer for the wrap-around.
        for track, zone_ids in graph.track_zone_order.items():
            if len(zone_ids) < 2:
                continue
            last_zone_id = zone_ids[-1]
            last_desc = graph.descriptors.get(last_zone_id)
            if not last_desc or not last_desc.sDownstream:
                continue
            ds_track = graph.zone_to_track.get(last_desc.sDownstream, "")
            if ds_track == track and last_desc.sDownstream in zone_ids:
                # Cycle detected: last zone wraps back to earlier zone
                graph.cross_transfers.append(ST_CrossTrackTransfer(
                    sSourceZone=last_zone_id,
                    sTargetZone=last_desc.sDownstream,
                    sSourceTrack=track,
                    sTargetTrack=track,
                ))

        # ── 6c. Auto-mark end-of-track transfer sources as force_release ─
        # Any cross-transfer source that is the LAST zone on its track (no
        # same-track downstream that is LATER in the zone order) needs force
        # release. Branch zones with same-track primary downstream skip this.
        for ct in graph.cross_transfers:
            src_desc = graph.descriptors.get(ct.sSourceZone)
            if not src_desc:
                continue
            src_track_zones = graph.track_zone_order.get(ct.sSourceTrack, [])
            # Check if this zone has a same-track downstream that is
            # physically after it in the sim layout (not a cycle wrap)
            has_later_same_track_ds = False
            src_idx = src_track_zones.index(ct.sSourceZone) if ct.sSourceZone in src_track_zones else -1
            if src_desc.sDownstream:
                ds_track = graph.zone_to_track.get(src_desc.sDownstream, "")
                if ds_track == ct.sSourceTrack:
                    ds_idx = src_track_zones.index(src_desc.sDownstream) if src_desc.sDownstream in src_track_zones else -1
                    if ds_idx > src_idx:
                        has_later_same_track_ds = True
            if src_desc.sDivertDownstream:
                ds_track = graph.zone_to_track.get(src_desc.sDivertDownstream, "")
                if ds_track == ct.sSourceTrack:
                    ds_idx = src_track_zones.index(src_desc.sDivertDownstream) if src_desc.sDivertDownstream in src_track_zones else -1
                    if ds_idx > src_idx:
                        has_later_same_track_ds = True
            if not has_later_same_track_ds and ct.sSourceZone not in graph.force_release_zones:
                graph.force_release_zones.append(ct.sSourceZone)

        # ── 7. Determine track processing order ─────────────────────────
        # Tracks that feed into other tracks must process first.
        graph.track_process_order = TopologyBuilder._compute_track_order(
            graph.track_zone_order.keys(),
            graph.cross_transfers,
        )

        return graph

    @staticmethod
    def _compute_track_order(
        tracks,
        cross_transfers: List[ST_CrossTrackTransfer],
    ) -> List[str]:
        """
        Compute track processing order to match the current PRG_Main execution:
          1. Main line (branches produce diverted totes)
          2. Spur tracks + reject (consume branch diverts)
          3. Recirc loop (C_POST_LAST → MERGE_RECIRC → C_RECIRC)
          4. Reinduct (REINDUCT → C_RI_1 → MERGE_RECIRC)

        The key insight: main must process FIRST because branches on the main
        line detect diverted totes, which are immediately transferred to spur
        sim engines. The transfer areas (end-of-track to merge inputs) are
        populated AFTER a track finishes processing and consumed at the start
        of the next track's merge zones.

        For the recirc/main cycle: main processes, C_POST_LAST → transfer area,
        then recirc processes and MERGE_RECIRC reads from transfer area.
        C_RECIRC → transfer area, then on NEXT tick main's MERGE_MAIN reads it.
        """
        all_tracks = list(tracks)

        # Priority groups:
        # 0: "main" always first
        # 1: spur/reject tracks (consume branch diverts from main)
        # 2: "recirc" (consumes from C_POST_LAST end-of-main)
        # 3: "reinduct" (feeds into recirc merge)
        def sort_key(track):
            if track == "main":
                return (0, track)
            elif track.startswith("spur_") or track == "reject":
                return (1, track)
            elif track == "recirc":
                return (2, track)
            elif track == "reinduct":
                return (3, track)
            else:
                return (4, track)

        return sorted(all_tracks, key=sort_key)
