"""
types/enums.py
==============
IEC 61131-3 equivalent: TYPE ... END_TYPE (enumeration)

Naming convention mirrors CODESYS: E_<Name>
All enums use integer values so they can be serialized to GVL dicts
and displayed directly in the HMI without mapping.
"""

from enum import IntEnum


class E_ConveyorState(IntEnum):
    """
    State machine states for FB_ConveyorZone and all derived FBs.
    Gaps between values intentional — allows inserting sub-states later
    without renumbering, matching IEC 61131-3 ENUM best practice.
    """
    IDLE          = 0   # No tote, motor off, ready to accept
    ACCEPTING     = 10  # Beam break rising edge detected, tote entering
    TRANSPORTING  = 20  # Tote present, motor running, waiting for downstream
    WAITING       = 30  # Downstream blocked, motor paused (accumulation)
    RELEASING     = 40  # Downstream just cleared, advancing tote
    FAULTED       = 50  # Jam timeout, E-Stop, or motor fault
    RESETTING     = 60  # Fault cleared, returning to IDLE


class E_ChuteTarget(IntEnum):
    """
    Routing assignment for a tote. CHUTE_1..N are the sortation destinations.
    UNASSIGNED = not yet scanned.
    RECIRCULATE = intentional re-loop (unknown barcode, operator decision).
    NO_READ = scanner could not read barcode; routes to reject chute.
    REJECT = explicit reject chute (last chute N, configurable).
    """
    UNASSIGNED    = 0
    CHUTE_1       = 1
    CHUTE_2       = 2
    CHUTE_3       = 3
    CHUTE_4       = 4
    CHUTE_5       = 5
    CHUTE_6       = 6
    CHUTE_7       = 7
    CHUTE_8       = 8
    RECIRCULATE   = 90
    NO_READ       = 91
    REJECT        = 99


class E_MergePriority(IntEnum):
    """
    Merge conveyor arbitration mode.
    Controls which input path gets right-of-way when both are ready.
    """
    INPUT1_PRIORITY = 0  # Infeed always wins
    INPUT2_PRIORITY = 1  # Loop return always wins (default)
    FIFO            = 2  # First to arrive at merge wins
    ALTERNATE       = 3  # Strict 1-2-1-2 alternation


class E_AlarmSeverity(IntEnum):
    """
    Alarm classification. Mirrors industrial standard:
    WARNING  = informational, system keeps running
    FAULT    = unit-level stop, rest of system continues
    CRITICAL = system-wide stop required
    """
    WARNING  = 1
    FAULT    = 2
    CRITICAL = 3


class E_RoutingStrategy(IntEnum):
    """
    Chute assignment algorithm selector.
    CASE/ENUM dispatch in FB_RoutingEngine — add new strategies here,
    then add the matching CASE branch in fb_routing_engine.py.

        TYPE E_RoutingStrategy :
        (
            LEAST_LOADED := 1,
            ROUND_ROBIN  := 2,
            FIXED_TABLE  := 3
        );
        END_TYPE
    """
    LEAST_LOADED = 1   # Assign to chute with fewest totes in spur (default)
    ROUND_ROBIN  = 2   # Rotate chute assignments 1→2→N→1→2→N
    FIXED_TABLE  = 3   # Lookup barcode in static table; fallback to LEAST_LOADED


class E_SystemState(IntEnum):
    """
    Top-level system state exposed to HMI.
    Mirrors PackML top-level mode concepts.
    """
    STOPPED   = 0
    RUNNING   = 1
    FAULTED   = 2
    ESTOPPED  = 3
    RESETTING = 4
    BLOCKED   = 5    # Reject spur full — soft pause, auto-resume when cleared
