"""
topology/zone_registry.py
=========================
Factory pattern: maps eZoneType strings to FB classes.

    "transport" / "infeed" / "end" → FB_ConveyorZone
    "branch"                       → FB_BranchConveyor(zone_id, target_chute)
    "merge"                        → FB_MergeConveyor(zone_id)
    "scanner"                      → FB_ScannerConveyor(zone_id)
"""

from plc.types.structs import ST_ZoneDescriptor
from plc.types.enums import E_ChuteTarget
from plc.function_blocks.fb_conveyor_zone import FB_ConveyorZone
from plc.function_blocks.fb_branch_conveyor import FB_BranchConveyor
from plc.function_blocks.fb_merge_conveyor import FB_MergeConveyor


class ZoneRegistry:
    """Creates the appropriate FB instance for a zone descriptor."""

    @staticmethod
    def create_fb(desc: ST_ZoneDescriptor) -> FB_ConveyorZone:
        """
        Factory method: creates an FB instance based on eZoneType.
        Returns the created FB (always a subclass of FB_ConveyorZone).
        """
        zone_type = desc.eZoneType.lower()

        if zone_type == "branch":
            target = desc.iTargetChute
            if target is None:
                target = E_ChuteTarget.REJECT.value
            return FB_BranchConveyor(desc.sZoneID, target_chute=target)

        elif zone_type == "merge":
            return FB_MergeConveyor(desc.sZoneID)

        elif zone_type == "scanner":
            # Scanner zones use FB_ScannerConveyor if available, else base zone
            try:
                from plc.function_blocks.fb_scanner_conveyor import FB_ScannerConveyor
                return FB_ScannerConveyor(desc.sZoneID)
            except ImportError:
                return FB_ConveyorZone(desc.sZoneID)

        else:
            # "transport", "infeed", "end" — all use base zone FB
            return FB_ConveyorZone(desc.sZoneID)
