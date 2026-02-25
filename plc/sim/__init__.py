# Lazy imports to avoid circular dependency
# (structs.py → sim_clock → sim/__init__ → sim_engine → gvl_config → types → structs)
# Import directly: from plc.sim.sim_engine import SIM, SimEngine
