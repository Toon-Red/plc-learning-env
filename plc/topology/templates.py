"""
topology/templates.py
=====================
Built-in topology templates. default_sortation() produces the EXACT same
topology as the current hardcoded _build_topology() in PRG_Main.
"""

from typing import List
from plc.types.structs import ST_ZoneDescriptor
from plc.types.enums import E_ChuteTarget
from plc.gvl.gvl_config import CFG


def default_sortation(
    num_chutes: int = None,
    spur_length: int = None,
    reject_length: int = None,
) -> List[ST_ZoneDescriptor]:
    """
    Generate the standard sortation topology matching the current _build_topology().

    Layout:
        INFEED → C1 → MERGE_MAIN → C2 → C3_SCAN → C4 → C5 → C6
        → [BRANCH_i → C_BUF_i] × N → BRANCH_REJECT → C_POST_LAST
        → MERGE_RECIRC → C_RECIRC → (back to MERGE_MAIN)
        + REINDUCT → C_RI_1 → (into MERGE_RECIRC)
        + SPUR_i_1..3 per chute (pickup at last zone)
        + REJECT_1..5 (pickup at last zone)
    """
    if num_chutes is None:
        num_chutes = CFG.iNumberOfChutes
    if spur_length is None:
        spur_length = CFG.iSpurLength
    if reject_length is None:
        reject_length = CFG.iRejectSpurLength

    zones: List[ST_ZoneDescriptor] = []

    # ══════════════════════════════════════════════════════════════════════
    # MAIN LINE
    # ══════════════════════════════════════════════════════════════════════

    # Build main line zone ID list first (for wiring sDownstream)
    main_ids = ["INFEED", "C1", "MERGE_MAIN", "C2", "C3_SCAN", "C4", "C5", "C6"]
    for i in range(1, num_chutes + 1):
        main_ids.append(f"BRANCH_{i}")
        main_ids.append(f"C_BUF_{i}")
    main_ids.append("BRANCH_REJECT")
    main_ids.append("C_POST_LAST")

    # INFEED
    zones.append(ST_ZoneDescriptor(
        sZoneID="INFEED",
        eZoneType="infeed",
        bIsSpawnPoint=True,
        sDownstream="C1",
        sTrackGroup="main",
    ))

    # C1
    zones.append(ST_ZoneDescriptor(
        sZoneID="C1",
        eZoneType="transport",
        sDownstream="MERGE_MAIN",
        sTrackGroup="main",
    ))

    # MERGE_MAIN: input1=C1 (same-track), input2=C_RECIRC (cross-track, inferred by compiler)
    zones.append(ST_ZoneDescriptor(
        sZoneID="MERGE_MAIN",
        eZoneType="merge",
        eMergePriority="INPUT2_PRIORITY",
        sDownstream="C2",
        sTrackGroup="main",
    ))

    # C2
    zones.append(ST_ZoneDescriptor(
        sZoneID="C2",
        eZoneType="transport",
        sDownstream="C3_SCAN",
        sTrackGroup="main",
    ))

    # C3_SCAN (scanner zone)
    zones.append(ST_ZoneDescriptor(
        sZoneID="C3_SCAN",
        eZoneType="scanner",
        sDownstream="C4",
        sTrackGroup="main",
    ))

    # C4 → C5 → C6
    zones.append(ST_ZoneDescriptor(
        sZoneID="C4",
        eZoneType="transport",
        sDownstream="C5",
        sTrackGroup="main",
    ))
    zones.append(ST_ZoneDescriptor(
        sZoneID="C5",
        eZoneType="transport",
        sDownstream="C6",
        sTrackGroup="main",
    ))
    zones.append(ST_ZoneDescriptor(
        sZoneID="C6",
        eZoneType="transport",
        sDownstream=f"BRANCH_1" if num_chutes > 0 else "BRANCH_REJECT",
        sTrackGroup="main",
    ))

    # BRANCH_i + C_BUF_i per chute
    for i in range(1, num_chutes + 1):
        next_id = f"C_BUF_{i}"
        divert_target = f"SPUR_{i}_1"

        zones.append(ST_ZoneDescriptor(
            sZoneID=f"BRANCH_{i}",
            eZoneType="branch",
            iTargetChute=i,
            sDownstream=next_id,
            sDivertDownstream=divert_target,
            sTrackGroup="main",
        ))

        # C_BUF_i: buffer after branch
        if i < num_chutes:
            buf_downstream = f"BRANCH_{i + 1}"
        else:
            buf_downstream = "BRANCH_REJECT"

        zones.append(ST_ZoneDescriptor(
            sZoneID=f"C_BUF_{i}",
            eZoneType="transport",
            sDownstream=buf_downstream,
            sTrackGroup="main",
        ))

    # BRANCH_REJECT
    zones.append(ST_ZoneDescriptor(
        sZoneID="BRANCH_REJECT",
        eZoneType="branch",
        iTargetChute=E_ChuteTarget.REJECT.value,
        sDownstream="C_POST_LAST",
        sDivertDownstream="REJECT_1",
        sTrackGroup="main",
    ))

    # C_POST_LAST (end of main, feeds into MERGE_RECIRC cross-track)
    zones.append(ST_ZoneDescriptor(
        sZoneID="C_POST_LAST",
        eZoneType="end",
        bForceRelease=True,
        sDownstream="MERGE_RECIRC",
        sTrackGroup="main",
    ))

    # ══════════════════════════════════════════════════════════════════════
    # RECIRC LINE: MERGE_RECIRC → C_RECIRC
    # ══════════════════════════════════════════════════════════════════════

    zones.append(ST_ZoneDescriptor(
        sZoneID="MERGE_RECIRC",
        eZoneType="merge",
        eMergePriority="INPUT1_PRIORITY",
        sDownstream="C_RECIRC",
        sTrackGroup="recirc",
    ))

    zones.append(ST_ZoneDescriptor(
        sZoneID="C_RECIRC",
        eZoneType="transport",
        bForceRelease=True,
        sDownstream="MERGE_MAIN",
        sTrackGroup="recirc",
    ))

    # ══════════════════════════════════════════════════════════════════════
    # RE-INDUCT LINE: REINDUCT → C_RI_1
    # ══════════════════════════════════════════════════════════════════════

    zones.append(ST_ZoneDescriptor(
        sZoneID="REINDUCT",
        eZoneType="infeed",
        bIsSpawnPoint=True,
        sDownstream="C_RI_1",
        sTrackGroup="reinduct",
    ))

    zones.append(ST_ZoneDescriptor(
        sZoneID="C_RI_1",
        eZoneType="transport",
        bForceRelease=True,
        sDownstream="MERGE_RECIRC",
        sTrackGroup="reinduct",
    ))

    # ══════════════════════════════════════════════════════════════════════
    # SPUR TRACKS (one per chute)
    # ══════════════════════════════════════════════════════════════════════

    for chute_idx in range(1, num_chutes + 1):
        for z in range(1, spur_length + 1):
            spur_zone_id = f"SPUR_{chute_idx}_{z}"
            is_last = (z == spur_length)

            if is_last:
                downstream = None
            else:
                downstream = f"SPUR_{chute_idx}_{z + 1}"

            zones.append(ST_ZoneDescriptor(
                sZoneID=spur_zone_id,
                eZoneType="transport",
                bIsPickupPoint=is_last,
                sDownstream=downstream,
                sTrackGroup=f"spur_{chute_idx}",
            ))

    # ══════════════════════════════════════════════════════════════════════
    # REJECT SPUR TRACK
    # ══════════════════════════════════════════════════════════════════════

    for z in range(1, reject_length + 1):
        reject_zone_id = f"REJECT_{z}"
        is_last = (z == reject_length)

        if is_last:
            downstream = None
        else:
            downstream = f"REJECT_{z + 1}"

        zones.append(ST_ZoneDescriptor(
            sZoneID=reject_zone_id,
            eZoneType="transport",
            bIsPickupPoint=is_last,
            sDownstream=downstream,
            sTrackGroup="reject",
        ))

    return zones
