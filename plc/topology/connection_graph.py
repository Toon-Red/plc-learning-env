"""
topology/connection_graph.py
============================
Directed graph of zone connections built from ST_ZoneDescriptor list.
"""

from typing import Dict, List, Tuple, Optional
from plc.types.structs import ST_ZoneDescriptor


class ConnectionGraph:
    """
    Directed graph of zone connections.
    Built from descriptor sDownstream/sDivertDownstream/sUpstream2 fields.
    """

    def __init__(self, descriptors: List[ST_ZoneDescriptor]) -> None:
        self._desc_map: Dict[str, ST_ZoneDescriptor] = {
            d.sZoneID: d for d in descriptors
        }
        # Forward edges: zone_id → [(target_id, port_type)]
        self._downstream: Dict[str, List[Tuple[str, str]]] = {}
        # Reverse edges: zone_id → [(source_id, port_type)]
        self._upstream: Dict[str, List[Tuple[str, str]]] = {}

        for d in descriptors:
            self._downstream[d.sZoneID] = []
            self._upstream.setdefault(d.sZoneID, [])

        for d in descriptors:
            if d.sDownstream:
                self._downstream[d.sZoneID].append((d.sDownstream, "primary"))
                self._upstream.setdefault(d.sDownstream, []).append(
                    (d.sZoneID, "primary")
                )
            if d.sDivertDownstream:
                self._downstream[d.sZoneID].append((d.sDivertDownstream, "divert"))
                self._upstream.setdefault(d.sDivertDownstream, []).append(
                    (d.sZoneID, "divert")
                )
            # NOTE: sUpstream2 no longer used for edges — merge inputs are
            # inferred by TopologyBuilder from same-track vs cross-track upstream.

    def get_downstream(self, zone_id: str) -> List[Tuple[str, str]]:
        """Returns [(target_id, "primary"|"divert")] for a zone."""
        return self._downstream.get(zone_id, [])

    def get_upstream(self, zone_id: str) -> List[Tuple[str, str]]:
        """Returns [(source_id, "primary"|"divert"|"merge_input2")] for a zone."""
        return self._upstream.get(zone_id, [])

    def get_primary_downstream(self, zone_id: str) -> Optional[str]:
        """Returns the primary downstream zone ID, or None."""
        for target, port in self._downstream.get(zone_id, []):
            if port == "primary":
                return target
        return None

    def get_divert_downstream(self, zone_id: str) -> Optional[str]:
        """Returns the divert downstream zone ID, or None."""
        for target, port in self._downstream.get(zone_id, []):
            if port == "divert":
                return target
        return None

    def get_primary_upstream(self, zone_id: str) -> Optional[str]:
        """Returns the primary upstream zone ID, or None."""
        for source, port in self._upstream.get(zone_id, []):
            if port == "primary":
                return source
        return None

    def get_merge_input2_source(self, zone_id: str) -> Optional[str]:
        """Returns the merge input2 source zone ID, or None."""
        for source, port in self._upstream.get(zone_id, []):
            if port == "merge_input2":
                return source
        return None

    def topological_order(self, zone_ids: List[str]) -> List[str]:
        """
        Sort a subset of zones in upstream→downstream order.
        Uses Kahn's algorithm. For cyclic tracks (e.g. recirc loops),
        falls back to a linear walk from the cross-track entry point.
        """
        subset = set(zone_ids)
        # Build local in-degree
        in_degree = {z: 0 for z in subset}
        local_edges: Dict[str, List[str]] = {z: [] for z in subset}

        for z in subset:
            for target, _ in self._downstream.get(z, []):
                if target in subset:
                    local_edges[z].append(target)
                    in_degree[target] += 1

        # Kahn's
        queue = [z for z in subset if in_degree[z] == 0]
        result = []
        while queue:
            # Stable sort: pick by original zone_ids order
            queue.sort(key=lambda x: zone_ids.index(x))
            node = queue.pop(0)
            result.append(node)
            for neighbor in local_edges.get(node, []):
                in_degree[neighbor] -= 1
                if in_degree[neighbor] == 0:
                    queue.append(neighbor)

        # ── Cycle handling ──────────────────────────────────────────
        # If Kahn's didn't include all nodes, the remainder forms a cycle.
        # Walk from the cross-track entry point (zone fed by another track).
        if len(result) < len(subset):
            remaining = subset - set(result)

            # Find entry: zone in the cycle with an upstream from outside
            entry = None
            for z in zone_ids:
                if z not in remaining:
                    continue
                for src, _ in self._upstream.get(z, []):
                    if src not in subset:
                        entry = z
                        break
                if entry:
                    break

            # Fallback: first remaining zone in original order
            if not entry:
                for z in zone_ids:
                    if z in remaining:
                        entry = z
                        break

            # Linear walk following downstream within the subset
            visited = set()
            current = entry
            while current and current not in visited and current in remaining:
                visited.add(current)
                result.append(current)
                next_zone = None
                for tgt, _ in self._downstream.get(current, []):
                    if tgt in remaining and tgt not in visited:
                        next_zone = tgt
                        break
                current = next_zone

            # Any remaining unvisited (multiple disconnected cycles)
            for z in zone_ids:
                if z in remaining and z not in visited:
                    result.append(z)

        return result

    def all_zone_ids(self) -> List[str]:
        return list(self._desc_map.keys())

    def validate(self) -> List[str]:
        """Returns list of error messages (empty = valid)."""
        errors = []
        all_ids = set(self._desc_map.keys())

        for d in self._desc_map.values():
            if d.sDownstream and d.sDownstream not in all_ids:
                errors.append(
                    f"Zone '{d.sZoneID}': sDownstream '{d.sDownstream}' not found"
                )
            if d.sDivertDownstream and d.sDivertDownstream not in all_ids:
                errors.append(
                    f"Zone '{d.sZoneID}': sDivertDownstream '{d.sDivertDownstream}' not found"
                )

        return errors
