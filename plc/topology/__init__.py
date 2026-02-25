"""
plc/topology/
=============
RCT2-style segment-based track builder.
Converts zone descriptors into runtime topology graphs.
"""

from plc.topology.zone_registry import ZoneRegistry
from plc.topology.connection_graph import ConnectionGraph
from plc.topology.topology_builder import TopologyBuilder
from plc.topology.topology_processor import TopologyProcessor
from plc.topology.templates import default_sortation
from plc.topology.serializer import save_topology, load_topology

__all__ = [
    "ZoneRegistry",
    "ConnectionGraph",
    "TopologyBuilder",
    "TopologyProcessor",
    "default_sortation",
    "save_topology",
    "load_topology",
]
