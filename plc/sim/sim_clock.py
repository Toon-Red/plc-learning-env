"""
sim/sim_clock.py
=================
Virtual simulation clock for deterministic tick-based timing.

All components that need time (jam timers, spawn timers, transit timeouts,
robot pickup timers, FIFO timestamps) use SIM_CLOCK.now() instead of time.time().

Two modes:
  - Real-time (default): SIM_CLOCK.now() returns time.time(). Used by standalone
    FB tests that don't go through PRG_Main.
  - Virtual time: PRG_Main calls SIM_CLOCK.advance(CFG.rScanCycleRate_s) each tick.
    Time advances deterministically regardless of CPU speed.

Usage:
    from plc.sim.sim_clock import SIM_CLOCK
    now = SIM_CLOCK.now()
"""

import time as _time


class SimClock:

    def __init__(self) -> None:
        self._virtual_time: float = 0.0
        self._use_virtual:  bool  = False

    def now(self) -> float:
        """Current time — virtual or real depending on mode."""
        if self._use_virtual:
            return self._virtual_time
        return _time.time()

    def advance(self, dt: float) -> None:
        """Advance virtual clock by dt seconds. Switches to virtual mode on first call."""
        self._use_virtual = True
        self._virtual_time += dt

    def reset(self) -> None:
        """Reset to real-time mode. Used in test teardown."""
        self._virtual_time = 0.0
        self._use_virtual  = False

    def is_virtual(self) -> bool:
        return self._use_virtual


# Singleton
SIM_CLOCK = SimClock()
