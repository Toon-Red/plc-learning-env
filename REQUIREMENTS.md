# Conveyor Sortation System — Requirements & Deliverables
**Status**: FINAL — All clarifications resolved
**Last updated**: 2026-02-11

---

## 1. Purpose

Build a PLC-style conveyor sortation simulation in Python, structured to mirror IEC 61131-3
conventions exactly (GVLs, Function Blocks, Programs, ENUMs, STRUCTs) so the entire codebase
translates directly to CODESYS with minimal rework.

A web-based HMI acts as the operator interface. The HMI has **zero logic** — it only reads and
writes GVL_HMI tags via API. All decisions, routing, sequencing, and fault handling live in the
PLC-layer backend.

---

## 2. Guiding Principles (Non-Negotiable)

1. **Check Inputs → Calculate → Execute** — the scan cycle order, always.
2. **One tote per conveyor zone maximum** — zone will not accept a new tote if bTotePresent is TRUE.
3. **HMI contains no logic** — it reads state, writes commands to GVL_HMI, nothing else.
4. **Modular FBs** — every conveyor type is a reusable Function Block. The topology is wired in
   PRG_Main by instantiating FBs and connecting outputs to inputs. No spaghetti.
5. **E-Stop is absolute** — overrides all other logic, latching, requires explicit reset.
6. **N chutes is a config value** — changing it should require only a config change, not a code change.
7. **All state lives in GVL** — no hidden state inside classes that isn't exposed to the GVL layer.

---

## 3. Architecture

```
┌─────────────────────────────────────────────────────────┐
│                        HMI (Browser)                    │
│   Reads GVL_IO, GVL_Alarms  |  Writes GVL_HMI only     │
│   NO LOGIC — display and command only                   │
└─────────────────┬───────────────────────────────────────┘
                  │ REST / WebSocket  [TBD — see Q1]
┌─────────────────▼───────────────────────────────────────┐
│                    HMI Server (FastAPI)                  │
│   Thin API layer — reads/writes GVL dict only           │
│   No logic here either                                  │
└─────────────────┬───────────────────────────────────────┘
                  │ In-process
┌─────────────────▼───────────────────────────────────────┐
│                    PLC Runtime (Python)                  │
│                                                         │
│   GVL_IO       GVL_HMI     GVL_Config   GVL_Alarms     │
│                                                         │
│   Types/                                               │
│     E_ConveyorState   E_MergePriority   E_ChuteTarget   │
│     ST_ToteData       ST_AlarmEntry                     │
│                                                         │
│   FunctionBlocks/                                       │
│     FB_ConveyorZone    FB_BranchConveyor                │
│     FB_MergeConveyor   FB_BarcodeScanner                │
│     FB_ToteTracker     FB_ChuteSpur    FB_EStop         │
│                                                         │
│   PRG_Main  (scan cycle loop — orchestration only)      │
└─────────────────────────────────────────────────────────┘
```

### 3.1 File / Folder Structure (planned)

```
New Strat/
├── REQUIREMENTS.md           ← this file
├── README.md                 ← PLC-to-Python mapping guide
├── config.json               ← GVL_Config values (N chutes, speeds, timers, etc.)
│
├── plc/
│   ├── gvl/
│   │   ├── __init__.py
│   │   ├── gvl_io.py         ← simulated physical I/O
│   │   ├── gvl_hmi.py        ← HMI command tags (written by frontend)
│   │   ├── gvl_config.py     ← system configuration
│   │   └── gvl_alarms.py     ← alarm registry (circular buffer)
│   │
│   ├── types/
│   │   ├── __init__.py
│   │   ├── enums.py          ← E_ConveyorState, E_MergePriority, E_ChuteTarget, etc.
│   │   └── structs.py        ← ST_ToteData, ST_AlarmEntry
│   │
│   ├── function_blocks/
│   │   ├── __init__.py
│   │   ├── fb_conveyor_zone.py
│   │   ├── fb_branch_conveyor.py
│   │   ├── fb_merge_conveyor.py
│   │   ├── fb_barcode_scanner.py
│   │   ├── fb_tote_tracker.py
│   │   ├── fb_chute_spur.py
│   │   └── fb_estop.py
│   │
│   └── programs/
│       ├── __init__.py
│       └── prg_main.py       ← scan cycle, wires FBs together, no logic of its own
│
├── server/
│   ├── __init__.py
│   └── api.py                ← FastAPI app, thin GVL bridge only
│
├── hmi/
│   └── index.html            ← [TBD — see Q2, Q3]
│
└── tests/
    ├── test_fb_conveyor_zone.py
    ├── test_fb_branch_conveyor.py
    ├── test_fb_merge_conveyor.py
    ├── test_fb_barcode_scanner.py
    ├── test_fb_tote_tracker.py
    ├── test_fb_chute_spur.py
    └── test_prg_main.py
```

---

## 4. Conveyor Topology

### 4.1 Main Loop (fixed structure)

```
  [TOTE SPAWN]
       │
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  INFEED — totes enter the system here
  │  zone_id: "INFEED"  │
  └────┬────────────────┘
       │
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C1
  └────┬────────────────┘
       │                           ┌──────────────────────────┐
  ┌────▼────────────────┐          │  Loop Return (from C_END) │
  │  FB_MergeConveyor   │◄─────────┘
  │  zone_id: "MERGE"   │  Input1 = Infeed path
  └────┬────────────────┘  Input2 = Loop return path
       │                   Priority: [TBD — see Q5]
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C2
  └────┬────────────────┘
       │
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C3_SCAN — barcode scanner mounted here
  │  + FB_BarcodeScanner│
  └────┬────────────────┘
       │
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C4
  └────┬────────────────┘
       │
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C5
  └────┬────────────────┘
       │
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C6
  └────┬────────────────┘
       │
  ┌────▼─────────────────────────┐
  │  FB_BranchConveyor           │  BRANCH_1
  │  Condition: tote → Chute 1?  ├──────────────► [FB_ChuteSpur: SPUR_1]
  └────┬─────────────────────────┘
       │ (pass-through if not chute 1)
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C7
  └────┬────────────────┘
       │
  ┌────▼─────────────────────────┐
  │  FB_BranchConveyor           │  BRANCH_2
  │  Condition: tote → Chute 2?  ├──────────────► [FB_ChuteSpur: SPUR_2]
  └────┬─────────────────────────┘
       │
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C8
  └────┬────────────────┘
       │
  ┌────▼─────────────────────────┐
  │  FB_BranchConveyor           │  BRANCH_N
  │  Condition: tote → Chute N?  ├──────────────► [FB_ChuteSpur: SPUR_N]
  └────┬─────────────────────────┘
       │ (pass-through — missed all chutes OR assigned RECIRCULATE)
  ┌────▼────────────────┐
  │  FB_ConveyorZone    │  C_END → feeds back to MERGE (Input2)
  └─────────────────────┘
```

### 4.2 Chute Spur (one per chute, N total)

```
  [FB_BranchConveyor divert output]
           │
  ┌────────▼────────────┐
  │  FB_ConveyorZone    │  SPUR_A  (first spur zone)
  └────────┬────────────┘
           │
  ┌────────▼────────────┐
  │  FB_ConveyorZone    │  SPUR_B
  └────────┬────────────┘
           │
  ┌────────▼────────────┐
  │  FB_ConveyorZone    │  SPUR_C  (last spur zone)
  └────────┬────────────┘
           │
      [ROBOT PICKUP]  ← tote removed at random interval (timer range in GVL_Config)
```

`FB_ChuteSpur` wraps these 3 zones. When SPUR_C is occupied AND the random pickup
timer fires → tote is removed, SPUR_C clears, SPUR_B advances, etc.

---

## 5. Function Block Specifications

### 5.1 FB_ConveyorZone

The base building block. All other conveyor FBs extend this.

```
VAR_INPUT:
    bEnable          : BOOL    -- zone is powered and allowed to run
    bEStop           : BOOL    -- immediate stop, overrides all
    bReset           : BOOL    -- rising edge clears faults
    bBeamBreak       : BOOL    -- TRUE = tote detected at center sensor
    bDownstreamClear : BOOL    -- TRUE = next zone is ready to accept
    rSpeedPercent    : REAL    -- 0.0 to 100.0, modifiable per zone
    stIncomingTote   : ST_ToteData  -- tote data passed from upstream

VAR_OUTPUT:
    bMotorRun        : BOOL    -- drive the belt motor
    bTotePresent     : BOOL    -- tote is in this zone
    bFaulted         : BOOL    -- zone is in fault state
    bReady           : BOOL    -- zone is idle and ready to accept
    bOutputOccupied  : BOOL    -- tells upstream: I already have a tote
    stTotePassThrough: ST_ToteData  -- tote data forwarded downstream
    rActualSpeed     : REAL    -- current running speed

VAR (internal):
    eState           : E_ConveyorState
    fbJamTimer       : TON     -- jam detection timer
    _stCurrentTote   : ST_ToteData  -- internal tote tracking
    _bPrevBeamBreak  : BOOL    -- edge detection
```

**State machine** (`E_ConveyorState`):
- `IDLE` — no tote, motor off, ready to accept
- `ACCEPTING` — tote entering, beam break rising edge detected
- `TRANSPORTING` — tote present, motor running, waiting for downstream clear
- `WAITING` — tote present, downstream blocked, motor paused (accumulation)
- `RELEASING` — downstream cleared, motor back on, advancing tote
- `FAULTED` — jam timeout expired OR E-Stop OR motor fault
- `RESETTING` — fault cleared, returning to IDLE

**Key rules**:
- Will not enter `ACCEPTING` if `bOutputOccupied = TRUE` (one tote max)
- Jam timer starts when state enters `WAITING`; if duration > `tJamTimeout` → `FAULTED`
- E-Stop transitions to `FAULTED` from ANY state immediately

---

### 5.2 FB_BranchConveyor

Extends `FB_ConveyorZone`. Adds divert capability.

```
VAR_INPUT (additional):
    bDivertEnabled   : BOOL    -- this branch is active/configured
    bDivertCondition : BOOL    -- TRUE = divert this tote to spur (from ToteTracker)
    bSpurClear       : BOOL    -- TRUE = spur is ready to accept (from FB_ChuteSpur)

VAR_OUTPUT (additional):
    bDivertActive    : BOOL    -- divert gate is engaged
    bPassThrough     : BOOL    -- tote is passing to main line downstream
```

**Logic**:
- When tote arrives AND `bDivertCondition = TRUE` AND `bSpurClear = TRUE` → divert
- When tote arrives AND `bDivertCondition = TRUE` AND `bSpurClear = FALSE` → wait
  (this is the "spur full" jam condition — see Section 9 Alarms)
- When `bDivertCondition = FALSE` → pass through to main line downstream

---

### 5.3 FB_MergeConveyor

Extends `FB_ConveyorZone`. Manages two input paths into one output.

```
VAR_INPUT (additional):
    bInput1Present   : BOOL    -- tote ready at Input 1 (infeed path)
    bInput2Present   : BOOL    -- tote ready at Input 2 (loop return path)
    eMergePriority   : E_MergePriority
    stInput1Tote     : ST_ToteData
    stInput2Tote     : ST_ToteData

VAR_OUTPUT (additional):
    bBlockInput1     : BOOL    -- hold signal sent upstream to Input 1
    bBlockInput2     : BOOL    -- hold signal sent upstream to Input 2
    iLastAcceptedInput: INT    -- 1 or 2 (for FIFO tracking)
```

**E_MergePriority options**:
- `INPUT1_PRIORITY` — Input 1 always gets right-of-way when both present
- `INPUT2_PRIORITY` — Input 2 always gets right-of-way when both present
- `FIFO` — whichever arrived first goes first
- `ALTERNATE` — strictly alternates 1, 2, 1, 2...
- `CONFIGURABLE` — [TBD, see Q5]

---

### 5.4 FB_BarcodeScanner

Mounted on C3_SCAN. Simulates a camera scan.

```
VAR_INPUT:
    bTrigger         : BOOL    -- rising edge = tote in scan position (from zone beam break)
    bEStop           : BOOL
    stToteInZone     : ST_ToteData  -- tote data from conveyor zone

VAR_OUTPUT:
    sBarcodeResult   : STRING  -- scanned barcode string
    bScanComplete    : BOOL    -- scan finished successfully
    bScanFault       : BOOL    -- hardware fault (simulated)
    bNoRead          : BOOL    -- tote present but no barcode returned
    tScanDuration    : TIME    -- simulated scan processing time
```

**Logic**:
- On rising edge of bTrigger → simulate scan delay (configurable, e.g. 200ms)
- If tote has barcode in ST_ToteData → return it (successful scan)
- If tote has no barcode → `bNoRead = TRUE` → alarm raised
- [TBD — see Q4] for how NoRead totes are handled

---

### 5.5 FB_ToteTracker

Lookup table and zone tracking for all active totes.

```
VAR_INPUT:
    sBarcodeIn       : STRING  -- barcode just scanned
    iZoneUpdate      : INT     -- zone ID reporting a tote moved
    sToteIDUpdate    : STRING  -- which tote moved
    bRegisterTote    : BOOL    -- rising edge = new tote entering system
    bRemoveTote      : BOOL    -- rising edge = tote leaving system (robot pickup)

VAR_OUTPUT:
    eChuteAssignment : E_ChuteTarget   -- which chute this barcode routes to
    bLookupComplete  : BOOL
    bUnknownBarcode  : BOOL    -- barcode not in routing table
    diActiveToteCount: DINT    -- total totes currently on system
    diTotalProcessed : DINT    -- lifetime counter
```

**Internal data**:
- `aRoutingTable`: array mapping barcode patterns → chute assignments (loaded from config.json)
- `aToteRegistry`: array of ST_ToteData for all active totes (max configurable)

---

### 5.6 FB_ChuteSpur

Wraps 3 `FB_ConveyorZone` instances (A → B → C).

```
VAR_INPUT:
    bEnable          : BOOL
    bEStop           : BOOL
    bReset           : BOOL
    bIncomingTote    : BOOL    -- tote arriving from branch divert
    stIncomingTote   : ST_ToteData
    tPickupIntervalMin: TIME   -- robot pickup timing range min
    tPickupIntervalMax: TIME   -- robot pickup timing range max

VAR_OUTPUT:
    bSpurReady       : BOOL    -- at least one zone open (ready to accept)
    bSpurFull        : BOOL    -- all 3 zones occupied
    bSpurEmpty       : BOOL    -- all 3 zones empty
    diToteCount      : DINT    -- how many zones are occupied (0–3)
    bFaulted         : BOOL    -- any internal zone faulted
```

**Robot pickup logic**:
- Random timer (between `tPickupIntervalMin` and `tPickupIntervalMax`) runs when SPUR_C is occupied
- On timer expiry → tote is removed from SPUR_C → totes cascade forward (B→C, A→B)
- Removed tote is deregistered in `FB_ToteTracker`

---

### 5.7 FB_EStop

```
VAR_INPUT:
    bEStopInput      : BOOL    -- physical button OR HMI button (GVL_HMI.bHMI_EStop)
    bReset           : BOOL    -- rising edge, only valid when bEStopInput is FALSE

VAR_OUTPUT:
    bEStopActive     : BOOL    -- broadcast to ALL FBs every scan
    bResetAllowed    : BOOL    -- TRUE only when physical trigger is cleared
```

**Rules**:
- Latching: once active, bEStopActive stays TRUE until explicit reset
- Reset is only allowed when `bEStopInput` is FALSE (button/trigger physically released)
- `bEStopActive` is passed into every single FB — it is never optional

---

## 6. GVL Specifications

### GVL_Config (loaded from config.json, modifiable at HMI when system stopped)
```
iNumberOfChutes      : INT     = 3
rDefaultBeltSpeed    : REAL    = 50.0     -- % of max speed
tJamTimeout          : TIME    = T#5S     -- per zone jam detection
tScanDelay           : TIME    = T#200MS  -- simulated barcode scan time
tSpawnInterval       : TIME    = T#3S     -- auto-spawn rate
bAutoSpawnEnabled    : BOOL    = TRUE
tPickupIntervalMin   : TIME    = T#5S     -- robot pickup min
tPickupIntervalMax   : TIME    = T#15S    -- robot pickup max
iMaxTotesOnSystem    : INT     = 20
eMergePriorityDefault: E_MergePriority = FIFO
```

### GVL_IO (written by PLC scan cycle, read by HMI — never written by HMI)
```
-- Per zone (indexed by zone_id):
bBeamBreak[zone]     : BOOL
bMotorRun[zone]      : BOOL
eZoneState[zone]     : E_ConveyorState
rZoneSpeed[zone]     : REAL
bZoneFaulted[zone]   : BOOL
sToteID[zone]        : STRING   -- which tote is in this zone, "" if empty

-- Spur-level:
bSpurFull[chute]     : BOOL
diSpurToteCount[chute]: DINT

-- System-level:
bEStopActive         : BOOL
bSystemRunning       : BOOL
diActiveTotes        : DINT
diTotalProcessed     : DINT
```

### GVL_HMI (written by HMI, read by PRG_Main — never written by PLC)
```
bHMI_Start           : BOOL    -- start system
bHMI_Stop            : BOOL    -- controlled stop
bHMI_EStop           : BOOL    -- E-stop from HMI
bHMI_Reset           : BOOL    -- reset all faults
bHMI_ManualSpawn     : BOOL    -- spawn one tote manually
rHMI_SpeedOverride[zone]: REAL -- per-zone speed override
eMergePriorityOverride: E_MergePriority
bHMI_AckAlarm[index] : BOOL    -- acknowledge specific alarm
```

### GVL_Alarms
```
aAlarmBuffer[100]    : ST_AlarmEntry    -- circular buffer, newest first
diActiveAlarmCount   : DINT
diUnackAlarmCount    : DINT
```

---

## 7. Type Definitions

### E_ConveyorState
```
IDLE, ACCEPTING, TRANSPORTING, WAITING, RELEASING, FAULTED, RESETTING
```

### E_ChuteTarget
```
UNASSIGNED, CHUTE_1..CHUTE_N, RECIRCULATE, NO_READ
```

### E_MergePriority
```
INPUT1_PRIORITY, INPUT2_PRIORITY, FIFO, ALTERNATE, CONFIGURABLE
```

### ST_ToteData
```
sToteID          : STRING   -- UUID assigned at spawn
sBarcode         : STRING   -- barcode value
eChuteAssignment : E_ChuteTarget
iCurrentZone     : INT      -- zone_id of current location
tEnteredSystem   : TIME     -- timestamp
tLastScan        : TIME     -- timestamp of last barcode scan
bScanComplete    : BOOL     -- has this tote been scanned?
iRecircCount     : INT      -- how many times has this tote looped?
```

### ST_AlarmEntry
```
sTimestamp       : STRING
iAlarmCode       : INT
sZoneID          : STRING
sMessage         : STRING
eSeverity        : E_AlarmSeverity   -- WARNING, FAULT, CRITICAL
bAcknowledged    : BOOL
```

### E_AlarmSeverity
```
WARNING, FAULT, CRITICAL
```

---

## 8. Scan Cycle — PRG_Main

PRG_Main does NOT contain logic. It wires FBs and calls them in order.

```
Every scan tick:

  1. CHECK INPUTS
     - Read GVL_HMI command tags
     - Update simulated beam breaks from tote position simulation
     - Call FB_EStop(bEStopInput := GVL_HMI.bHMI_EStop OR hw_estop_input)

  2. CALCULATE (call all FBs in upstream-to-downstream order)
     - fbEStop(...)
     - fbConveyor_INFEED(bEStop := fbEStop.bEStopActive, ...)
     - fbConveyor_C1(...)
     - fbMerge(bInput1Present := fbConveyor_C1.bTotePresent, ...)
     - fbConveyor_C2(...)
     - fbScanner(bTrigger := fbConveyor_C3.bBeamBreak, ...)
     - fbToteTracker(sBarcodeIn := fbScanner.sBarcodeResult, ...)
     - fbConveyor_C4(...)
     - fbConveyor_C5(...)
     - fbConveyor_C6(...)
     - fbBranch1(bDivertCondition := (fbToteTracker.eChuteAssignment = CHUTE_1), ...)
     - fbSpur1(...)
     - fbConveyor_C7(...)
     - fbBranch2(bDivertCondition := (fbToteTracker.eChuteAssignment = CHUTE_2), ...)
     - fbSpur2(...)
     - ... (repeat for N chutes)
     - fbConveyor_C_END(...)

  3. EXECUTE
     - Write motor states to GVL_IO
     - Write zone states to GVL_IO
     - Write alarm updates to GVL_Alarms
     - Write system-level status to GVL_IO
```

---

## 9. Alarm Definitions

| Code | Severity | Trigger | Message |
|------|----------|---------|---------|
| A001 | FAULT    | Zone jam timer expired | "Jam detected: zone [ID]" |
| A002 | FAULT    | Spur full, tote can't divert | "Spur full: chute [N], tote [ID] blocked at branch" |
| A003 | WARNING  | Barcode no-read | "No-read: tote [ID] at scan zone" |
| A004 | WARNING  | Unknown barcode | "Unknown barcode [X]: tote assigned RECIRCULATE" |
| A005 | CRITICAL | E-Stop activated | "E-Stop active" |
| A006 | FAULT    | Tote exceeded max recirculate count | "Recirculate limit: tote [ID] looped [N] times" |
| A007 | WARNING  | Max totes on system reached | "Tote limit reached: spawn paused" |
| A008 | FAULT    | Merge blocked (both inputs present, neither can advance) | "Merge deadlock: zone [ID]" |

---

## 10. HMI Requirements

**Display only (reads GVL_IO):**
- Top-down conveyor layout — each zone shown as a rectangle
  - Green outline = motor running
  - Orange fill = tote present (show tote ID + barcode if scanned)
  - Red fill = faulted
  - Grey = idle/disabled
- Chute spurs shown beside main loop with tote count (0/1/2/3 occupied)
- Live tote count and total processed counter
- Alarm banner — active unacknowledged alarms listed with severity color coding
- Merge zone shows which input is currently active

**Controls (writes GVL_HMI only):**
- Start / Stop / E-Stop / Reset buttons
- Manual Spawn Tote button
- Per-zone speed slider (writes rHMI_SpeedOverride)
- Merge priority dropdown
- Alarm acknowledge button per alarm entry

**Configuration panel (writes GVL_Config via API when system stopped):**
- Number of chutes
- Default speed
- Jam timeout
- Spawn interval / auto-spawn toggle
- Robot pickup timing range

**Absolute rule: no conditionals, no calculations, no state logic in the frontend.**

---

## 11. Open Questions — Answers Required Before Coding

### Q1 — Real-time protocol
WebSocket gives true push (backend pushes state every scan tick).
REST polling means HMI requests state every N ms.
- **Option A**: WebSocket — HMI receives updates automatically, lower latency
- **Option B**: REST polling (e.g., every 100ms) — simpler to implement, slight lag
- **Option C**: Both — WebSocket for live state, REST for config/commands

### Q2 — HMI frontend framework
- **Option A**: Plain HTML + vanilla JavaScript — no build step, runs from file, easiest
- **Option B**: HTML + HTMX — minimal JS, server-driven updates
- **Option C**: React — more maintainable at scale, requires Node.js build step

### Q3 — Tote spawning and barcode assignment
When a tote is created, it needs a barcode. Two sub-questions:
- **Q3a — Barcode values**: Random UUIDs? Or a pre-defined list in config.json?
- **Q3b — Chute routing rules**: Static lookup table in config (barcode X → chute 2)?
  Or assigned randomly at spawn time?

### Q4 — No-read / unknown barcode behavior
A tote reaches the scan zone but gets no barcode (or has an unknown barcode).
- **Option A**: Assign RECIRCULATE — goes back around the loop, alarm raised
- **Option B**: Assign a default chute (configurable "reject chute")
- **Option C**: Hold the scan zone and raise a CRITICAL alarm — operator must intervene

### Q5 — Merge priority default and CONFIGURABLE mode
What should "CONFIGURABLE" mean in the context of the merge?
- Custom Python callable? (e.g., user defines a function that takes both tote data and returns which input wins)
- A threshold-based rule (e.g., "Input2 priority if recirculate count > 2")?
- Or is FIFO/ALTERNATE sufficient and CONFIGURABLE not needed?

### Q6 — Tote recirculation limit
If a tote loops the system multiple times (e.g., spur always full, or no-read), how many loops
before it's removed/escalated?
- What is the max recirculate count before alarm A006 fires?
- When A006 fires, should the tote be force-removed or held at merge?

### Q7 — Scan cycle rate
What should the simulated tick rate be? This controls visual speed.
- **100ms** (10 Hz) — feels like a real PLC, responsive HMI
- **250ms** (4 Hz) — slower, easier to watch individual steps
- **Configurable** — set in config.json

---

## 12. Deliverables

| # | Deliverable | Description |
|---|-------------|-------------|
| D1 | Project structure + GVLs | All GVL modules with full variable definitions |
| D2 | Type definitions | All ENUMs and STRUCTs |
| D3 | FB_ConveyorZone | Base zone FB with full state machine and tests |
| D4 | FB_BranchConveyor | Divert extension + tests |
| D5 | FB_MergeConveyor | Merge logic with all priority modes + tests |
| D6 | FB_BarcodeScanner | Scan simulation + tests |
| D7 | FB_ToteTracker | Routing table + zone tracking + tests |
| D8 | FB_ChuteSpur | Spur wrapper + robot pickup timer + tests |
| D9 | FB_EStop | Latching E-stop + tests |
| D10 | PRG_Main | Scan cycle wiring all FBs, topology defined |
| D11 | FastAPI server | Thin GVL bridge, WebSocket + REST endpoints |
| D12 | HMI frontend | Visual conveyor layout, controls, alarms |
| D13 | config.json | All configurable parameters |
| D14 | Test suite | Unit tests per FB, integration test for full scan cycle |
| D15 | README / CODESYS translation guide | How each Python class maps to a CODESYS FB |

**Build order**: D1 → D2 → D3 (+ D9) → D4 → D5 → D6 → D7 → D8 → D10 → D11 → D12 → D13 → D14 → D15

---

## 11. Simulation Engine (NEW — not in original spec)

A physics layer runs BENEATH the PLC layer. The PLC FBs never see tote positions —
they only see beam break states, exactly as they would on real hardware.

### 11.1 Execution Order Per Tick
1. **SimEngine runs first**: moves all totes based on motor states from the PREVIOUS tick,
   updates bBeamBreak signals in GVL_IO.
2. **PLC layer runs second**: reads bBeamBreak, runs all FBs, writes bMotorRun outputs.

This matches real hardware timing: sensor reads are always one scan behind the motor command.

### 11.2 Tote Physics
- Position tracked as a float in real-world units (cm). Configurable in GVL_Config.
- Each tick: `position += belt_speed_cm_per_tick * (0 if slip_event else 1)`
- Slip event: configurable random chance (rSlipChance, default ~2%) per tick that the
  tote doesn't advance that tick. Simulates belt slip/rub.
- Tote has a configurable physical length (rToteLength_cm, default e.g. 40cm).
- A tote occupies positions [position, position + rToteLength_cm].

### 11.3 Zone Layout
- Each zone has: start_cm, length_cm, sensor_position_cm (configurable, default = center).
- Beam break = TRUE when any part of the tote body overlaps the sensor position.
- A tote can be small enough that NEITHER beam break is active (in the gap between zones).
- A tote can be large enough to trigger BOTH beam breaks simultaneously (spans two zones).

### 11.4 Transit Timeout
- When a zone's beam break falls (tote exits), the zone starts a transit timeout timer.
- Expected transit time = gap_to_next_sensor_cm / belt_speed_cm_per_tick
- Timeout threshold = 2x expected transit time.
- If next zone's beam break has not fired within threshold → A009 fault (tote lost in transit).
- Max totes = number of zones (one tote per conveyor, physically enforced by beam breaks).

### 11.5 Scanner Simulation
- Scanner continuously attempts to read while tote is in the scan zone.
- Attempts per zone pass = zone_length_cm / belt_speed_cm_per_tick (speed-dependent).
- Each attempt: random fail chance = rScanFailRate * (rActualSpeed / 100.0).
  Default rScanFailRate = 0.20 (20% at 50% speed → ~10% at 100% speed... scales linearly).
- If ANY attempt succeeds → scan complete, barcode assigned.
- If tote exits scan zone with no successful read → NO_READ → goes to reject chute.

### 11.6 Scan Mismatch (CRITICAL fault)
- Each zone tracks which tote it holds (sToteID in ST_ZoneStatus).
- If scanner returns a barcode that maps to a DIFFERENT tote ID than what the zone
  currently tracks → CRITICAL fault A010, full system stop.
- If zone expects a tote (bTotePresent=TRUE) but SimEngine shows no tote at that position
  → CRITICAL fault A010.

---

## 12. Routing Strategy (UPDATED — replaces static lookup table)

Chute assignment uses a strategy pattern implemented as a CASE/ENUM dispatch.
No static lookup table by default — assignment is always dynamic.

### E_RoutingStrategy
```
LEAST_LOADED   = 1   (* Default: assign to chute with fewest totes in spur *)
ROUND_ROBIN    = 2   (* Rotate through chutes 1→2→3→1→2→3... *)
FIXED_TABLE    = 3   (* Lookup table: barcode prefix → specific chute *)
```

### FB_RoutingEngine (new FB)
Called by FB_ToteTracker when a tote is scanned.
```
VAR_INPUT:
    sBarcode         : STRING
    eStrategy        : E_RoutingStrategy
    aSpurCounts[N]   : INT      (* current tote count per spur, from GVL_IO *)
VAR_OUTPUT:
    eChuteAssignment : E_ChuteTarget
    bAssignComplete  : BOOL
```
CASE eStrategy:
  LEAST_LOADED: find chute with min spur count → assign
  ROUND_ROBIN:  internal counter, wrap at N → assign
  FIXED_TABLE:  lookup barcode in routing table dict → assign, fallback to LEAST_LOADED

### Strategy switching
Changed via `eRoutingStrategy` in GVL_Config (set in config.json).
Takes effect on the next unassigned tote. No restart needed for strategy change
(GVL_Config can be updated at runtime via HMI API).

---

## 13. Updated Deliverables

| # | Deliverable | Status |
|---|-------------|--------|
| D1 | GVL modules | DONE |
| D2 | Types (enums, structs) | DONE |
| D3 | FB_ConveyorZone + tests | DONE (23/23) |
| D9 | FB_EStop + tests | DONE |
| D_SIM | SimEngine (physics layer) | DONE |
| D4 | FB_BranchConveyor + tests | DONE (10/10) |
| D5 | FB_MergeConveyor + tests | DONE (15/15) |
| D6 | FB_BarcodeScanner + tests | DONE |
| D7 | FB_ToteTracker + FB_RoutingEngine + tests | DONE |
| D8 | Spur zones (N conveyors, no wrapper FB) | DONE |
| D10 | PRG_Main + topology export | DONE |
| D_CLOCK | SimClock (virtual time) | DONE |
| D_ROBOT | SimRobotPickup (timer manager) | DONE |
| D11 | FastAPI server + WebSocket (port 8080) | DONE |
| D12 | HMI frontend (dark SCADA, auto-layout) | DONE |
| D13 | config.json | PENDING |
| D14 | Integration test (full tote lifecycle) | PENDING |
| D15 | README / CODESYS translation guide | PENDING |

**New alarm codes:**
- A009: Tote lost in transit (transit timeout exceeded)
- A010: Scan mismatch / unexpected tote (CRITICAL)

---

## 14. Resolved Clarifications

| Q | Decision |
|---|----------|
| Q1 | HMI protocol | WebSocket — backend pushes GVL state every scan tick |
| Q2 | Frontend style | Dark/charcoal SCADA theme, SVG top-down layout, color-coded zones, alarm panel, KPI counters |
| Q3 | Barcode source | Scanner reads from tote data after simulated delay |
| Q3b | Scanner behavior | Continuous attempts while tote in zone. Speed-dependent attempt count. 20% fail rate at 50% speed. |
| Q4 | No-read behavior | Always routes to reject chute (last chute = reject). Alarm A003. |
| Q5 | Merge priority default | INPUT2_PRIORITY (loop return first — drains recirculating totes before new infeed) |
| Q5b | Merge FIFO | Merge records timestamp when each input side becomes occupied. First to occupy wins. |
| Q6 | Recirculation limit | 3 loops max, then alarm A006, tote held at merge |
| Q7 | Scan cycle rate | 100ms (10 Hz) |
| Q8 | Scanner discovery | Tote has no barcode at spawn. Scanner assigns barcode from GVL_Inventory on successful read. |
| Q9 | Branch full | Tote passes through and recirculates (no blocking) |
| Q10 | Branch passthrough | Only diverts if tote.eChuteAssignment == this_branch_chute |
| Q11 | Spur pickup timer | Only runs when SPUR_C is occupied. Starts fresh when tote arrives. |
| Q12 | Spur cascade | Natural zone handoff on next scan (not instant internal cascade) |
| Q13 | Buffer zones | 1 zone between each branch |
| Q14 | Re-scan behavior | Always re-scan on every pass through C3_SCAN. Mismatch (different result than struct) = CRITICAL fault. NO_READ always → reject chute. |
| Q15 | Spawn logic | Timer fires AND INFEED is empty |
| Q16 | HMI serving | FastAPI serves index.html |
| Q17 | HMI layout | Auto-generates from iNumberOfChutes |
| Q18 | Tote animation | Static — zone lights up when occupied |
| Q19 | Speed controls | Global default + per-zone override |
| Q20 | Routing table on HMI | Yes — each zone shows tote ID, barcode (-- if unscanned), chute (UNASSIGNED until assigned). Live update. |
| Q21 | config.json | Yes — load at startup, update in-memory via HMI |
| Q22 | Integration test | Full lifecycle: spawn → scan → chute assigned → divert → spur → robot pickup → removed |
| Q23 | Routing algorithm | Strategy pattern as CASE/ENUM dispatch. Default: LEAST_LOADED. Switchable via GVL_Config. |
| Q24 | Scan mismatch fault | CRITICAL — full system stop |
| Q25 | Zone transit units | Centimeters, configurable in GVL_Config |
| Q26 | Sensor position | Configurable per zone, default = center (0.5 × zone_length) |
| Q27 | Belt slip | Random skip tick (~2% chance per tick tote doesn't advance) |
| Q28 | Tote size | Has configurable physical length. Can span two zones or fit in gap between zones. |
| Q29 | Transit timeout | 2x expected transit time (calculated from zone length and speed) |
| Q30 | Sim order | SimEngine first (moves totes, updates beam breaks), then PLC layer |
| Q31 | Max totes | No hard cap — one per conveyor, so physically bounded by zone count |
| Q32 | Routing delay | Simulated 1-5 tick delay (random) before chute assignment appears on struct. PLC code doesn't know about delay. |
| Q33 | Struct references | FB_ToteTracker holds canonical ST_ToteData. All zones hold references. Routing writes in-place; zones see update automatically. |
| Q34 | On-the-fly divert | Branch reads struct's eChuteAssignment each scan. If routing resolves while tote in WAITING, divert updates on next scan. |
| Q35 | Tote width | Configurable in GVL_Config (rToteWidth_cm). SimEngine rejects spawn if tote too wide for zone. |
| Q36 | FIFO timing | "Upstream ready time" — merge tracks when each input zone first signaled ready (rising edge). NOT tote entry time. |
| Q37 | Alternate skip | If next-in-turn input has no tote, skip to other input. Only strictly alternates when BOTH have totes. |
| Q38 | Deadlock A008 | Fires when BOTH inputs have totes ready AND merge zone itself is occupied AND downstream blocked for > rJamTimeout_s. FAULT severity. |
| Q39 | Recirc count | Merge increments iRecircCount when accepting from INPUT2. If > iMaxRecircCount, raises A006 (RecircLimit, FAULT). |
| Q40 | Spur = N conveyors | No special FB. Spurs are just N standard FB_ConveyorZone instances chained together by PRG_Main. |
| Q41 | Robot pickup | Pure simulator logic (NOT PLC logic). SimEngine removes tote after random timer. PLC sees beam break clear → zone IDLE. |
| Q42 | Spur cascade | Natural conveyor flow. When robot takes tote, beam break clears → zone IDLE → upstream sees downstream clear → releases. |
| Q43 | Spur full signal | Implicit. If spur entry zone is occupied, branch's divert-downstream-clear is False → tote passes through. No special flag. |
| Q44 | Spur length | Configurable N (iSpurLength in CFG, default 3). Each spur = N FB_ConveyorZone instances. |
| Q45 | Separate sim tracks | Each spur is a separate SimEngine instance (independent 1D track). Main line is one SimEngine. Divert = remove from main + spawn on spur. |
| Q46 | Reject branch | Dead-end spur (same as other spurs) but human worker picks up tote, replaces bad barcode, puts on re-induct conveyor line. Reject = last chute. |
| Q47 | Re-induct line | Separate input conveyor for manually re-introduced totes. Feeds into MERGE INPUT2 via merge conveyor. Tote gets rescanned on re-entry. |
| Q48 | End-of-line behavior | Tote that passes ALL branches (spurs full) loops back to MERGE INPUT2 from C_END. Keeps circulating up to iMaxRecircCount (5) loops, then E-STOP. |
| Q49 | PLC scan order | Upstream to downstream — each conveyor checks E-stop, beam break, incoming struct, downstream clear in sequence. Natural PLC FB execution order. |
| Q50 | Sim vs PLC separation | Simulator handles physics (tote movement, beam breaks, robot pickup timers, spawn logic). PLC handles logic (state machines, routing, alarms). Strictly separated. |
| Q51 | Tote lifecycle lists | INPUT (available to spawn), INTERNAL (on system, managed by PLC), DELIVERED (robot picked up). Tracked at simulator level like package tracking. |
| Q52 | Robot validation | Robot at spur end scans tote, validates against what PLC reports. Tracks successful vs failed deliveries as metrics. |
| Q53 | No-read simulation | Simulator randomly gives totes with unreadable barcodes (e.g., barcode on wrong side). Scanner fails → NO_READ → reject chute. |
| Q54 | Buffer zones | Not hard-coded for timing. Physical layout with configurable zone count. Simulation physics (speed + zone length) determines actual transit time. |
| Q55 | SimClock | Virtual simulation clock (plc/sim/sim_clock.py). All timers use SIM_CLOCK.now() instead of time.time(). PRG_Main advances by rScanCycleRate_s each tick. Deterministic, testable. |
| Q56 | Passthrough clear | stTotePassThrough cleared at start of each execute(). Valid for exactly one scan cycle between upstream.execute() and downstream.set_inputs(). |
| Q57 | Server setup | Single FastAPI server on port 8080. Serves REST/WebSocket API and static HMI files from one process. |
| Q58 | HMI layout | Dynamic linear rendering. Main line horizontal, branch spurs expand below as parallel rows (collapsible). Arrows show flow direction and loop-back paths. |
| Q59 | WebSocket rate | Fixed 500ms interval push. Server pushes full zone + alarm state every 500ms regardless of scan cycle. |
| Q60 | Speed control | Number input field (not buttons/slider). User types any value: 0.25, 1, 5, 10, etc. Controls simulation speed multiplier. |
| Q61 | Zone details | Side panel on click. Click any zone box to show full details (state, tote, timers, alarms) in a right-side panel. Main view stays clean. |
| Q62 | Collapse mode | Transport chains collapsed by default. Show as compact arrow with zone count (e.g., "-> 4 zones ->"). Click to expand. Major FBs (merge, scan, branch) always visible. |
| Q63 | Topology definition | Building blocks — FBs wiring into each other in code (like real PLC). Topology defined by FB connections in PRG_Main. HMI auto-generates from topology graph. |
| Q64 | Re-induct merge | Separate merge FB that feeds into the recirculation line (after last branch). Re-inducted totes merge with recirc totes at MERGE_RECIRC, loop to main MERGE, get scanned, then route properly. |
| Q65 | Zone visual style | Distinct colors per type: Merge=blue, Branch=orange, Scanner=green, Transport=gray, Infeed=yellow. State shown by border/glow effects. |
| Q66 | HMI panels | All panels: alarm panel (scrollable list), metrics dashboard (delivered/mismatched/active counts), controls toolbar (Start/Stop/E-Stop/speed), zone detail side panel on click. |
| Q67 | E-Stop HMI | Large red E-STOP button at top + separate blue RESET button. E-Stop latches via GVL write. Reset only works when E-Stop released. Matches real SCADA HMI style. |
| Q68 | Auto-start | Wait for Start click. Page loads showing empty conveyor layout. User clicks Start to begin spawning totes. Allows inspecting layout first. |
| Q69 | Topology discovery | PRG_Main exports topology graph: {zone_id: {type, upstream_ids, downstream_ids, position_hint}}. HMI reads once at startup to render layout. Straight line with branch expansions below. |
