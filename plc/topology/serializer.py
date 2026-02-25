"""
topology/serializer.py
======================
JSON save/load for topology descriptor lists.
"""

import json
from typing import List
from pathlib import Path

from plc.types.structs import ST_ZoneDescriptor


def save_topology(descriptors: List[ST_ZoneDescriptor], filepath: str) -> None:
    """Save a list of zone descriptors to a JSON file."""
    data = [d.to_dict() for d in descriptors]
    p = Path(filepath)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def load_topology(filepath: str) -> List[ST_ZoneDescriptor]:
    """Load a list of zone descriptors from a JSON file."""
    p = Path(filepath)
    if not p.exists():
        raise FileNotFoundError(f"Topology file not found: {filepath}")
    data = json.loads(p.read_text(encoding="utf-8"))
    return [ST_ZoneDescriptor.from_dict(d) for d in data]


def descriptors_to_json(descriptors: List[ST_ZoneDescriptor]) -> str:
    """Serialize descriptors to a JSON string."""
    data = [d.to_dict() for d in descriptors]
    return json.dumps(data, indent=2)


def json_to_descriptors(json_str: str) -> List[ST_ZoneDescriptor]:
    """Deserialize descriptors from a JSON string."""
    data = json.loads(json_str)
    return [ST_ZoneDescriptor.from_dict(d) for d in data]
