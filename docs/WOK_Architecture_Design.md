# WOK cFS Flight Software — Architecture & Design Document

**Document:** WOK-AD-001  
**Version:** 5.0  
**Date:** July 2026  
**Status:** As-built, verified on cFS Draco v7.0.0

---

## 1. Mission Overview

WOK (Water Observation Kit) is a lunar seismic survey and ice prospecting rover that autonomously traverses a set of waypoints, deploys geophones, records stacked seismic shots from a dual-mode electromagnetic source, picks first-break arrivals onboard, evaluates for ice signatures, and optionally drills for sampling with evolved-gas volatile analysis. Every data product is stamped with the surveyed site pose, so seismic and drill results at the same site form a calibration tie point — the basis of the Vp→ice-content transfer function that lets the seismic survey estimate ice content at the sites that were not drilled.

**Operational loop:**

    IDLE → MOVE → [HOLD] → DEPLOY → [VERIFY] → [THUMP → RECORD] ×N → EVALUATE → [DRILL] → STOW → MOVE → ... → DONE

THUMP starts the acquisition *before* arming and firing (pre-trigger), so the shot lands inside the record window and its t₀ is captured as a sample index. THUMP→RECORD repeats `WOK_SEIS_STACK_TARGET` (default 3) times per site; SEIS stacks the records aligned on each shot's t₀.

In per-pod placement mode (`SET_PLACEMENT`, CP-3/4/5) DEPLOY is the placement sub-sequence (MOVE_TO each pod → place → seating-force gate with bounded reseats), VERIFY fires a low-energy coupling test shot with targeted pod reseats and the partial-array policy, and STOW is the retrieval mirror with per-pod recapture confirmation. Fixed-array mode (default) never enters VERIFY and preserves the legacy command stream exactly.

**Seismic source:** a capacitor-bank-driven EM actuator striking a coupling plate (an earlier design used a drop weight; a few `Source*` telemetry names date from it). Two modes, selectable per shot (and per waypoint via `LOAD_WAYPOINT`):

- **IMPULSE** (default) — cap discharge → single strike. Commanded energy sets the bank charge level; gravity-independent.
- **SWEEP** — the same actuator driven with a linear swept-sine pilot (force-limited, for long-offset work). The record window extends to sweep + listen time; SEIS cross-correlates the record against the analytically regenerated pilot onboard (chunked across HK ticks) and the resulting equivalent-impulse trace feeds the existing picker/stacker/EVALUATE chain unchanged.

t₀ is electrical: FIRE (discharge or sweep start) latches the ADC sample index on the shared clock — no mechanical release detection. ARM = bank charge (non-blocking, draws from the battery bus; the sequencer's arm timeout covers a bank that cannot charge on a sagging bus).

Any critical fault triggers transition to SAFE mode (NAV stop, GEO stow, SEIS stop, DRILL e-stop), where the vehicle awaits ground commands. Drill is conditional on ice detection + battery voltage + waypoint drill flag.

**Mission modes:** In the default *drill-inline* mode, flagged sites drill as they are found. In *survey-first* mode the rover scores every site, then returns to the argmax-score site and drills it. Either way, the **tie-point mission-success gate** guarantees at least one drill inside the survey footprint before DONE — the calibration tie is what converts the seismic survey from relative to absolute ice content (see §4.1 Targeting).

### 1.1 Terminology

- **Pod** — one geophone package (hardware). **Channel** — the data stream from one pod (four channels, pods at 1.5/3.0/4.5/6.0 m).
- **Head wave / refractor** — the arrival that travels along the top of the faster second layer; "ice-fast" in the code means a refractor velocity above `WOK_SEIS_EVAL_V2_ICE_MPS` (1200 m/s), taken as ice-cemented regolith.
- **Fold** — number of shots summed into a site's stack. **Stack correlation** — zero-lag normalised correlation of each new record against the running stack (telemetry key `coherence_pct`).
- **CP-1 … CP-6** — the coupling-and-placement requirement set (internally WOK-CP-001): CP-1 the ground-coupling truth model in the sim; CP-2 the plate transfer metric and reseat branch; CP-3 per-pod placement; CP-4 test-shot coupling verification with targeted reseats; CP-5 the per-pod sync/pick-consistency check; CP-6 per-pod recapture confirmation on retrieval.
- **EDI** — extraction-difficulty index used by `wok_refraction.py` when ranking prospects: binding ease (pore ice cheap, hydrated minerals expensive) × a depth factor.
- **ppm** — parts per million by mass of sample (1 wt% = 10⁴ ppm), never a gas mixing ratio.

## 2. Software Architecture

### 2.1 Framework

WOK runs on NASA's core Flight System (cFS) Draco release (v7.0.0). The flight software consists of six application-layer apps communicating over the cFS Software Bus (SB), with all hardware access isolated behind a Hardware Abstraction Layer (HAL).

### 2.2 Application Set

| App | cFS Name | Priority | Role |
|-----|----------|----------|------|
| wok_sequencer | WOK_SEQ | 80 | Mission executive, state machine, safety inhibits, SeqId enforcement |
| wok_seis | WOK_SEIS | 85 | Seismic ADC acquisition, EM source control, sweep correlation, evaluation, QC |
| wok_nav | WOK_NAV | 90 | Waypoint driving, localization, fault detection |
| wok_geo | WOK_GEO | 90 | Geophone deploy/stow, endstop/jam/timeout monitoring |
| wok_drill | WOK_DRILL | 90 | Drill arm deploy, drilling, thermal sampling, volatile analysis |
| wok_hk_health | WOK_HK | 95 | Battery/temp/CPU monitoring, watchdog, FDIR-lite |

Priority ordering ensures the sequencer runs first, then acquisition, then mobility/mechanism/drill control, with health monitoring at the lowest priority.

### 2.3 Software Bus Architecture

```
                         cFS / cFE
                    ES  SB  EVS  TBL  TIME
                              |
                       (Software Bus)
                              |
          +---+-------+-------+-------+-------+-------+
          |   |       |       |       |       |       |
         SEQ NAV     GEO    SEIS   DRILL  HK_HEALTH
         (SM) (drive) (mech) (ADC)  (ice)  (FDIR)
          |   |       |       |       |       |
          +---+-------+-------+-------+-------+
                              |
                        WOK HAL (C API)
                              |
                   +----------+----------+
                   |                     |
              linux_sim            flight_stub
```

### 2.4 Message Flow

The sequencer orchestrates the mission by sending commands to child apps and receiving status telemetry back. All sequenced commands carry a monotonic SeqId to prevent stale telemetry from advancing state (see Section 4.1).

    SEQ --CMD_NAV--> NAV --NAV_STATUS--> SEQ
    SEQ --CMD_GEO--> GEO --GEO_STATUS--> SEQ
    SEQ --CMD_SEIS-> SEIS -SEIS_STATUS-> SEQ
    SEQ --CMD_DRILL> DRILL -DRILL_HK---> SEQ
    SEQ_MODE_TLM --------------------------> HK
    HK --HK_TLM--> (ground + SEQ safety check)

SCH_LAB sends periodic SEND_HK wakeups (every 4th 10 ms tick, about 25 Hz, with the stock SCH_LAB table) to all six apps, which triggers telemetry publication through TO_LAB to the ground console.

## 3. Message ID Allocation

All WOK MIDs are allocated at topic offset 0xD0+ to avoid collisions with stock Draco apps (TO_LAB=0x80, SAMPLE_APP=0x82, CI_LAB=0x84).

### 3.1 Command MIDs (base 0x1800 | 0xD0 = 0x18D0)

| MID | Name | Direction | Description |
|-----|------|-----------|-------------|
| 0x18D0 | WOK_SEQ_CMD | Ground → SEQ | Sequencer commands |
| 0x18D1 | WOK_SEQ_SEND_HK | SCH → SEQ | Housekeeping wakeup (the state-machine tick, ~25 Hz with the stock table) |
| 0x18D2 | WOK_NAV_CMD | SEQ/Ground → NAV | GoTo, Stop, SetSpeed, SetPose, Realign |
| 0x18D3 | WOK_NAV_SEND_HK | SCH → NAV | Housekeeping wakeup |
| 0x18D4 | WOK_GEO_CMD | SEQ/Ground → GEO | Deploy, Stow, Abort, PlacePod, RetrievePod, ReseatPod |
| 0x18D5 | WOK_GEO_SEND_HK | SCH → GEO | Housekeeping wakeup |
| 0x18D6 | WOK_SEIS_CMD | SEQ/Ground → SEIS | StartAcq, StopAcq, Source arm/fire, ConfigSource, Evaluate, DownlinkFile, PlateReseat, SetPodGeometry |
| 0x18D7 | WOK_SEIS_SEND_HK | SCH → SEIS | Housekeeping wakeup |
| 0x18D8 | WOK_HK_CMD | Ground → HK | Thresholds, ClearFaults |
| 0x18D9 | WOK_HK_SEND_HK | SCH → HK | Housekeeping wakeup |
| 0x18DA | WOK_DRILL_CMD | SEQ/Ground → DRILL | DeployArm, Drill, Sample, E-Stop |
| 0x18DB | WOK_DRILL_SEND_HK | SCH → DRILL | Housekeeping wakeup |

### 3.2 Telemetry MIDs (base 0x0800 | 0xD0 = 0x08D0)

| MID | Name | Source | Description |
|-----|------|--------|-------------|
| 0x08D0 | WOK_SEQ_TLM | SEQ | HK: counters, state, waypoint, faults, SeqId |
| 0x08D1 | WOK_SEQ_MODE_TLM | SEQ | Mode: state, loop count, total WPs |
| 0x08D2 | WOK_NAV_TLM | NAV | HK: counters, drive state, faults |
| 0x08D3 | WOK_NAV_STATUS_TLM | NAV | Pose, heading, speed, tilt, arrived, SeqId |
| 0x08D4 | WOK_GEO_TLM | GEO | HK: counters, mech state, faults |
| 0x08D5 | WOK_GEO_STATUS_TLM | GEO | Endstops, motor current, timeouts, SeqId |
| 0x08D6 | WOK_SEIS_TLM | SEIS | HK: counters, acquiring, faults |
| 0x08D7 | WOK_SEIS_STATUS_TLM | SEIS | QC flags, samples, source/bank/sweep, eval, shot t₀, picks, fold, SeqId |
| 0x08D8 | WOK_SEIS_QUICKLOOK | SEIS | RMS, peak, DC offset per channel (from the real trace tail) |
| 0x08D9 | WOK_HK_TLM | HK | Battery, temp, tilt, CPU, faults, watchdog |
| 0x08DA | WOK_DRILL_HK_TLM | DRILL | HK: state, faults, arm angle, depth, SeqId |
| 0x08DB | WOK_DRILL_STATUS_TLM | DRILL | Detailed arm/drill/sample state + sensors, WOB, ROP |
| 0x08DC | WOK_SEIS_FILECHUNK | SEIS | Bulk file-downlink chunk stream (see §4.4) |

### 3.3 CCSDS Packet Format

Draco's default configuration (no extended headers) as deployed:

Commands — 8-byte header:

    Bytes 0-5:   Primary header (StreamID, Sequence, Length)
    Bytes 6-7:   Command secondary header (function code + checksum)
    Bytes 8+:    Application payload

Telemetry — 16-byte header:

    Bytes 0-5:   Primary header (StreamID, Sequence, Length)
    Bytes 6-11:  Telemetry secondary header (timestamp)
    Bytes 12-15: Alignment padding
    Bytes 16+:   Application payload

The ground tools (`wok_console.py`, `wok_bridge.py`) build and parse
exactly these offsets.

## 4. Application Design

### 4.1 WOK_SEQ (Sequencer)

**Purpose:** Mission executive. Owns the operational loop, enforces safety inhibits, and implements SeqId enforcement to prevent stale SB telemetry from advancing state prematurely.

**SeqId Enforcement:** Every sequenced command carries a monotonic `uint16 SeqId` (0 reserved for ground/don't-care). Child apps echo the SeqId in their status telemetry. The sequencer only accepts a state confirmation when the echoed SeqId matches the pending value. This eliminates the primary sequencing hazard where async SB delivery could cause stale prior-cycle telemetry to trigger premature state transitions.

**State Machine:**

| State | Entry Condition | Exit Condition |
|-------|----------------|----------------|
| IDLE | Power-on / sequence complete | START command |
| MOVE | START or previous STOW complete | NAV reports ARRIVED (SeqId match) |
| DEPLOY | NAV arrived at waypoint (site pose snapshotted here) | Fixed array: GEO reports DEPLOYED (SeqId match). Per-pod placement (CP-3): the placement sub-sequence — MOVE_TO(pod_i) → PLACE → wait terminal pod state, far pod first, then a return leg to the shot point and SET_POD_GEOMETRY to SEIS |
| HOLD | MOVE complete at a recal-flagged waypoint (before DEPLOY) | Ground-recalibration hold: publishes an announcing event, waits for RELEASE (ground corrects with NAV SET_POSE/REALIGN meanwhile), auto-proceeds after `WOK_SEQ_HOLD_TIMEOUT_SEC` with an event so an unattended mission never deadlocks |
| VERIFY | Array placed (per-pod placement mode only, between DEPLOY and THUMP) | CP-4 coupling verification: one low-energy test shot scores every pod (spreading-normalized RMS vs array median — plate factor cancels); failing pods get a targeted drive-back reseat + re-verification on a bounded budget; partial-array policy applies (≥`WOK_SEQ_MIN_VERIFIED_PODS` proceeds; below it, CRITICAL event, proceeds FLAGGED); source config restored for the stack shots. Coupling record archived per test shot (`coup_NNNN.wok`) |
| THUMP | GEO deployed / array verified, or another strike needed | Sends per-site source config (if non-default), starts acquisition (pre-trigger), then arm+fire; exits on FIRED (SeqId match) or `WOK_SEQ_SOURCE_TIMEOUT_SEC` wall-clock timeout (covers worst-case bank charge + a full sweep) |
| RECORD | Shot delivered (sweep complete in sweep mode) | SEIS reports acquisition complete (SeqId match); loops back to THUMP until StackTarget strikes done. **Plate verification (CP-2):** an impulse record whose plate transfer ratio (near-pod RMS / plate force, vs a running site-to-site baseline) fails is kept out of the stack by SEIS; SEQ commands a plate reseat and re-shoots the same strike, up to `WOK_SEIS_MAX_PLATE_RESEAT` per site, then accepts degraded (site flagged, stack header marked) |
| EVALUATE | Final record complete | SEIS reports eval confidence > 0 (SeqId match) |
| DRILL | Ice flagged + battery OK + drill flag | Drill cycle complete (4 substates, SeqId gated) |
| STOW | Evaluate/drill complete | Fixed array: GEO reports STOWED (SeqId match). Per-pod placement: the retrieval mirror — drive to each placed pod (near first), recapture (CARRIED confirmed before departure, CP-6), return to the shot point, then advance |
| SAFE | Any critical fault / ABORT command | RESUME command (ground) |
| DONE | All waypoints complete | START command (restart) |

**Drill Sub-States:** DEPLOY → DRILLING → SAMPLING → STOW (each SeqId-gated).

**Survey geometry:** On DEPLOY entry the sequencer snapshots NAV pose as the site position (start frame) and stamps it into SEIS StartAcq and DRILL StartDrill. Every trace file and drill log therefore carries its own source-receiver geometry — the linkage that makes co-located seismic + drill products a calibration tie point.

**Timing:** State machine runs exclusively on SEND_HK ticks. Child telemetry updates cached data only — never advances the state machine directly. Timeouts use wall-clock time (`CFE_TIME`), never tick counts — the SEND_HK rate is platform-dependent.

**Command Codes:**

| CC | Name | Payload | Description |
|----|------|---------|-------------|
| 0 | NOOP | — | Connectivity test |
| 1 | RESET_CTRS | — | Zero cmd/err counters |
| 2 | START | — | Begin waypoint loop |
| 3 | ABORT | — | Immediate SAFE entry |
| 4 | RESUME | — | Resume from SAFE |
| 5 | SET_WAYPOINT | uint16 idx | Override waypoint index |
| 6 | SET_MODE | uint8 mode | 0 = drill-inline, 1 = survey-first |
| 7 | LOAD_WAYPOINT | idx, drill, srcmode, df, dl, energy, recal | Upload/replace one waypoint (incl. per-site source mode + impulse energy + recal-hold flag; zeros preserve prior behavior) |
| 8 | SET_WP_COUNT | uint16 n | Set active waypoint count |
| 9 | SET_PLACEMENT | uint8 p | 0 = fixed array (legacy, default), 1 = per-pod placement (CP-3) |
| 10 | RELEASE | — | Release a recalibration HOLD (rejected outside HOLD) |

**Targeting:** Every surveyed site's `EvalIceFrac` and absolute pose are recorded. In **survey-first** mode no site drills inline; after the last waypoint the sequencer returns to the argmax-score site (navigating by pose delta) and drills it. The **tie-point mission-success gate** applies in both modes: a mission may not end DONE with zero drills — the rover returns to its best-scoring site and drills regardless of score (a dry tie still calibrates the transfer function's dry end), battery permitting.

**Safety Inhibits:** Battery voltage (22V, 23.5V for drill), tilt angle (30°), temperature (55°C) — compile-time defaults in `wok_platform_cfg.h`, ground-settable via SET_THRESHOLDS. Checked on every state machine cycle.

### 4.2 WOK_NAV (Navigation)

**Purpose:** Execute relative-step missions in the persistent start frame. Each step is a body-frame `(DeltaForwardM, DeltaLateralM)` offset that NAV decomposes into a `(turn, drive, turn-back)` primitive sequence. WOK does no external localization — pose is dead-reckoned from completed sub-states for ground monitoring only.

**Frame model:** The start frame is captured at app init (rover heading at SEQ START). All deltas are interpreted in that frame: `+ DeltaForwardM` along the start-heading axis; `+ DeltaLateralM` is left of forward. Every step ends with the rover restored to the start-frame heading via `TURN_BACK`, so per-step math is independent of accumulated history.

**Command Codes:**

| CC | Name | Payload | Description |
|----|------|---------|-------------|
| 0 | NOOP | — | Connectivity test |
| 1 | RESET_CTRS | — | Zero counters |
| 2 | GOTO | SeqId, float DeltaForwardM, float DeltaLateralM | Body-frame relative step |
| 3 | STOP | SeqId | Immediate halt |
| 4 | SET_SPEED | SeqId, float m/s | Override max speed |
| 5 | SET_POSE | SeqId, float x, float y | Overwrite the dead-reckoned start-frame pose with ground truth (bookkeeping only; rejected while driving) |
| 6 | REALIGN | SeqId, float heading_error_deg | Corrective in-place turn of −error, then declare the current heading start-frame (drift zeroed; rejected unless IDLE/ARRIVED/STOPPED) |

All commands include a `uint16 SeqId` field echoed back in status telemetry.

**Drive States:** IDLE, DRIVING, ARRIVED, STOPPED, FAULT

**Sub-states (within DRIVING):** TURN_TO → DRIVE → TURN_BACK. Each sub-state issues one HAL primitive and gates completion on the matching encoder/heading accumulator. TURN_TO and TURN_BACK are skipped when `|PlanTurnDeg| < 1°` (e.g. pure-forward steps).

**Pose telemetry:** `PoseX_m`, `PoseY_m`, `HeadingDriftDeg` — dead-reckoned, start-frame, informational only. Not used in arrival decisions. Heading drift compounds across steps (every TURN_BACK restores the *believed* start heading, not the true one), degrading site stamps over a long traverse.

**Ground recalibration:** two NAV ground commands close the loop. `SET_POSE (x, y)` overwrites the dead-reckoned pose with ground-supplied truth — bookkeeping only, accepted from any non-driving state. `REALIGN (heading_error_deg)` is the physical correction: rejected unless the drive is IDLE/ARRIVED/STOPPED, it executes one corrective in-place turn of −error through the normal turn machinery (skipped below the same ~1° threshold step planning uses) and then declares the current physical heading to be the start-frame heading — `HeadingDriftDeg` reads as "believed drift since the last realign" and `RealignCount` in status TLM counts corrections. Note the corrective turn rides the same drivetrain as every other turn: a systematic per-turn error leaves its one-turn residual after a realign.

**Fault Detection (per HK tick during DRIVING):**
- `STUCK` (0x0100) — no encoder/heading progress for `StuckTimeoutMs` (3 s default)
- `OVERCURRENT` (0x0400) — drive motor current exceeds `CurrentFaultThreshold_A` (4 A default)
- `TILT_EXCEED` (0x0004) — tilt above `TiltFaultThreshold_deg` (30° default; also caught at HK level)

`SLIP` (encoder vs IMU heading mismatch) is reserved (0x0200) but not yet implemented — meaningful only with both sensors live and well-calibrated.

### 4.3 WOK_GEO (Geophone Mechanism)

**Purpose:** Deploy and stow the geophone array. Monitor mechanism health.

**Command Codes:**

| CC | Name | Payload | Description |
|----|------|---------|-------------|
| 0 | NOOP | SeqId | Connectivity test |
| 1 | RESET_CTRS | SeqId | Zero counters |
| 2 | DEPLOY | SeqId | Extend geophone arm (legacy fixed array) |
| 3 | STOW | SeqId | Retract geophone arm (legacy fixed array) |
| 4 | ABORT | SeqId | Emergency stop motor (halts pod motions too) |
| 5 | PLACE_POD | SeqId, pod, x, y | Place one pod at (x,y) — CP-3. GEO runs the seat → force-proxy → reseat loop internally (≤ `WOK_GEO_MAX_RESEAT`) and lands PLACED_OK or PLACED_MARGINAL |
| 6 | RETRIEVE_POD | SeqId, pod | Recapture one placed pod; CARRIED again = recapture confirmed (CP-6) |
| 7 | RESEAT_POD | SeqId, pod | One commanded reseat of a placed pod (CP-4, test-shot-triggered; no internal retry — the test shot is the arbiter) |

**Mechanism States:** STOWED, DEPLOYING, DEPLOYED, STOWING, FAULT (legacy array)

**Per-pod States (CP-3):** CARRIED, PLACING, PLACED_OK, PLACED_MARGINAL, RETRIEVING — plus an array rollup (PlacedMask/MarginalMask/ActivePod) in status TLM alongside per-pod seat force and reseat counts. MARGINAL = the seating-force gate (`WOK_GEO_SEAT_FORCE_MIN_N`) exhausted its reseat budget: flagged for the sequencer and the archive, arbitrated by the CP-4 test shot — never fatal on its own.

**Monitoring:** Endstop switches (deployed/stowed), motor current, jam detection (current spike + no endstop), timeout detection; per-pod placement/retrieval timeouts.

### 4.4 WOK_SEIS (Seismic Acquisition + EM Source + Stacking + Picks)

**Purpose:** Configure ADC, control the dual-mode EM source, acquire and archive raw records with hardware-referenced shot t₀, correlate sweep records onboard, stack shots per site, pick first breaks onboard, perform QC, evaluate for ice signatures, serve bulk file downlinks.

**Command Codes:**

| CC | Name | Payload | Description |
|----|------|---------|-------------|
| 0 | NOOP | SeqId | Connectivity test |
| 1 | RESET_CTRS | SeqId | Zero counters |
| 2 | START_ACQ | SeqId, dur, Fs, gain, ch_mask, Flags, site (x,y,hdg), StackIdx, StackTarget | Begin recording; StackIdx 0 opens a new site stack. In sweep mode the window auto-extends by the sweep duration (record = sweep + listen). Flags bit 0 = accept a plate-degraded record into the stack (CP-2). |
| 3 | STOP_ACQ | SeqId | Abort recording |
| 4 | THUMP_ARM | SeqId | Arm source: apply the shot config, charge the bank (non-blocking) |
| 5 | THUMP_FIRE | SeqId | Fire: cap discharge (impulse) or pilot sweep start |
| 6 | THUMP_RESET | SeqId | Safe the source to idle (dumps the bank if charged) |
| 7 | EVALUATE | SeqId | Evaluate data for ice signature |
| 8 | DOWNLINK_FILE | SeqId, char name[24], uint32 ChunkIdx | Serve a stored product: chunk N (request/response) or 0xFFFFFFFF (paced stream) |
| 9 | CONFIG_SOURCE | SeqId, mode, energy, f0, f1, dur | Source mode + shot parameters; zeros keep current values. Persistent until reconfigured. |
| 10 | PLATE_RESEAT | SeqId | Lift and re-press the source coupling plate (CP-2; rejected while the source is charging/armed/sweeping) |
| 11 | SET_POD_GEOMETRY | SeqId, per-pod x/y/offset/attempt, marginal mask | Actual placed geometry for the site (CP-3 placement missions): overrides the static offsets for the picker, inversion, and archived headers until the next call |

**Acquisition path:** Samples are drained from the HAL into a per-shot trace buffer each HK tick and archived as `seis_NNNN.wok` on completion (112-byte v3 header + interleaved int32 samples; format in `wok_seis_msg.h`). The source and ADC share one hardware clock: firing while acquiring latches the shot's per-channel sample index (`StrikeSampleIdx`) — the electrical t₀ reference for every arrival pick.

**Sweep correlation:** after a sweep record completes, the app regenerates the pilot analytically (`WOK_HAL_Src_PilotSample` — the same contract function the source hardware drives) and cross-correlates the record against it over the stack window. The work is chunked at `WOK_SEIS_CORR_SAMPS_PER_TICK` output samples per HK tick so the main loop never blocks longer than one tick; `Acquiring` stays asserted in TLM until the correlated trace is stacked, so the sequencer's RECORD state waits naturally. The output is delayed by `WOK_SEIS_CORR_LEAD_FRAC/(f1−f0)` to convert the zero-phase correlation (Klauder) wavelet to an effectively causal one — onset picks on the equivalent-impulse trace then agree with true impulse picks (±0.25 ms in the physics harness). The correlated trace is archived as `corr_NNNN.wok` and is what feeds the site stack — sweep-mode stacks are stacks of correlated traces, and the picker/EVALUATE chain runs on them unchanged.

**Stacking:** Each site's records are summed aligned on their own strike t₀ over a 2 s window ([−200 ms, +1800 ms]). Fold and zero-lag stack correlation (telemetry key `coherence_pct`) are tracked; after the site's final strike the stack is archived as `stack_NNNN.wok` (header v2 with fold, coherence, picks).

**First-break picker:** STA/LTA with the LTA frozen from the pre-strike noise window (standard for triggered records), onset refined to the first amplitude excursion above 4× noise RMS. `FirstBreakMs[4]` + `PickQuality[4]` (0–100) go to Status TLM and the stack file header.

**Onboard evaluate (pick-based):** EVALUATE runs a two-layer refraction grid inversion over the picks — v1 from the near pod pair, (v2, h1) grid with implicit direct/head-wave branch assignment, cells weighted by pick-misfit likelihood × a uniform-in-slowness prior. The verdict is the posterior mass on ice-fast refractors (`EvalIceFrac`); the site is flagged at ≥ 0.30. `EvalV1_mps`, `EvalV2_mps`, `EvalDepthM`, `EvalIceFrac` are appended to Status TLM as the evidence behind the flag. The HAL evaluate is only a fallback when no picks exist. Ground-side, `ground/wok_refraction.py` runs the same inversion with uncertainty ranges plus drill-log competence/evolved-gas analysis and the tie-point report. If the picks cannot support an inversion (v₁ outside 100–600 m/s) the app falls back to `WOK_HAL_Seis_Evaluate`, which in the Linux sim reads the geology model directly: a **simulation oracle with no flight equivalent**, reported by event so a run cannot lean on it unnoticed.

**QC Flags:** Computed from the real trace — clipped-sample counts per channel (during drain), dead channel from trace-tail RMS, dropout detection.

**Quicklook Telemetry:** RMS, peak amplitude, and DC offset per channel over the most recent 1024 recorded samples — published during acquisition for ground monitoring.

**File downlink:** Any stored product (`seis_`/`corr_`/`stack_`/`coup_`/`drill_` file) is served as 1024-byte chunks on MID 0x08DC. Two modes: a paced fire-and-forget stream (a few chunks per HK tick), or ground-driven stop-and-wait requests by chunk index — the robust mode the console's `seis.fetch` uses. Requires `fs.mqueue.msg_max ≥ 64` on Linux hosts (see Build & Install Guide).

### 4.5 WOK_DRILL (Drill / Ice Prospector)

**Purpose:** Deploy drill arm, drill regolith with a drilling-response log, thermal vaporization sampling with an evolved-gas record, volatile analysis.

**Command Codes:**

| CC | Name | Payload | Description |
|----|------|---------|-------------|
| 0 | NOOP | SeqId | Connectivity test |
| 1 | RESET_CTRS | SeqId | Zero counters |
| 2 | DEPLOY_ARM | SeqId, float angle | Deploy arm to angle |
| 3 | STOW_ARM | SeqId | Retract arm |
| 4 | START_DRILL | SeqId, float depth, float rpm, site (x,y) | Begin drilling; site stamps the log file |
| 5 | STOP_DRILL | SeqId | Stop motor |
| 6 | START_SAMPLE | SeqId, uint16 heat_sec, uint16 cool_sec | Thermal cycle |
| 7 | STOP_SAMPLE | SeqId | Abort sampling |
| 8 | ESTOP | SeqId | Emergency stop all |

**App States:** IDLE, DEPLOYING, ARM_READY, DRILLING, DRILL_DONE, SAMPLING, SAMPLE_DONE, STOWING, FAULT

**Drilling-response log:** WOB (weight-on-bit) is read from the HAL and a smoothed ROP is computed onboard; both are appended to STATUS TLM. Through DRILLING and SAMPLING the app logs (t, phase, depth, WOB, torque, current, cell temp, vapor pressure, ppm) rows at ≥500 ms spacing, archived per cycle as `drill_NNNN.wok` (format in `wok_drill_msgids.h`). The (depth, WOB, torque, ROP) columns form the regolith competence log; the (temp, ppm) columns during heating form the evolved-gas curve — the volatile *release temperature* is what classifies how the water is bound (adsorbed ≈110 °C, pore ice ≈160 °C, hydrated mineral ≈320 °C), which drives extraction cost. Volatiles bound above the 300 °C cell limit are intentionally under-recovered; ground corrects total abundance from the release curve.

**Fault Detection:** Arm jam, arm timeout, drill stall (torque), overcurrent, over-depth, thermal overtemp. All trigger FAULT state + e-stop.

**Default Limits:** Max depth 5.0m, max RPM 300, torque limit 3.0 Nm, current limit 5.0A, cell temp limit 350°C, arm timeout 8s, min battery 23.5V. Default sequenced drill depth 2.5 m — deep enough to punch through the ~1.5–2 m dry surface layer into the ice-bearing zone.

### 4.6 WOK_HK (Health / FDIR)

**Purpose:** System health monitoring, watchdog, fault detection and response.

**Command Codes:**

| CC | Name | Payload | Description |
|----|------|---------|-------------|
| 0 | NOOP | — | Connectivity test |
| 1 | RESET_CTRS | — | Zero counters |
| 2 | SET_THRESHOLDS | float V, float C, float deg | Update fault thresholds |
| 3 | CLEAR_FAULTS | — | Acknowledge and clear fault flags |

**Monitored Parameters:** Battery voltage, battery SOC, board temperature, motor temperature, tilt, CPU load, watchdog status.

**FDIR Logic:** Thresholds are compile-time defaults (`wok_platform_cfg.h`), ground-settable via SET_THRESHOLDS. Any critical fault sets the SafeTrigger flag and fires an event. The sequencer checks this flag on every cycle. Faults re-assert on the next HK cycle if the condition persists — CLEAR_FAULTS only works if the underlying condition resolves.

**Fault Bitmask:**

| Bit | Name | Trigger |
|-----|------|---------|
| 0x0001 | BATTERY_LOW | Voltage below threshold |
| 0x0002 | TEMP_HIGH | Board temp above threshold |
| 0x0004 | TILT_EXCEED | Tilt angle above threshold |
| 0x0008 | WATCHDOG | App missed heartbeat |
| 0x0010 | COMM_LOSS | Communication timeout *(reserved — not yet raised by any app)* |
| 0x0100 | NAV_STUCK | No mobility progress |
| 0x0200 | NAV_SLIP | Encoder mismatch *(reserved — not yet raised by any app)* |
| 0x0400 | OVERCURRENT | Motor current exceeded |

## 5. HAL Boundary

All hardware access goes through the WOK HAL API. The HAL is a per-subsystem static library (v2) linked to all WOK apps.

**Subsystems (8 headers; the sim implements them in 10 source files, the flight stub in 2):**

| Subsystem | Header | Functions |
|-----------|--------|-----------|
| Mobility | wok_hal_mobility.h | Init, Stop, SetSpeed, DriveForward, Turn, GetDistanceTraveled, GetHeadingChange, IsMoving, GetSpeed, GetTilt, GetMotorCurrent |
| Geo Mechanism | wok_hal_geo.h | Init, Deploy, Stow, EmergencyStop, GetState, GetEndstops, GetMotorCurrent, PlacePod, ReseatPod, RetrievePod, GetPodState, GetPodSeating |
| Seismic | wok_hal_seis.h | Init, Configure, SetSite, SetPodGeometry, StartAcquisition, StopAcquisition, GetSamples, GetStatus, GetStrikeInfo, GetQuicklook, Evaluate |
| EM Source | (in wok_hal_seis.h) | Init, Configure(mode/energy/pilot), Arm, Fire, Reset, Abort, GetState, GetBank, GetShotInfo, GetSweepProgress, ReseatPlate + the contract pilot `WOK_HAL_Src_PilotSample` |
| Drill | wok_hal_drill.h | Init, SetSite, DeployArm, StowArm, StartDrill, StopDrill, StartSample, StopSample, EmergencyStop, GetWOB, Get* |
| Power | wok_hal_power.h | Init, GetBatteryV, GetBatterySOC, GetBoardTemp, GetMotorTemp, GetCPULoad |
| Storage | wok_hal_storage.h | Init, WriteFile, ReadFile, GetFreeSpace |
| Watchdog | wok_hal_watchdog.h | Init, Pet |

Key contract points: `Seis_SetSite`/`Drill_SetSite` carry the surveyed pose into data products; `Seis_GetStrikeInfo` returns the shot's per-channel sample index (source drive and ADC must share a clock on real hardware); `Drill_GetWOB` reads the axial load cell; `Storage_ReadFile` backs the file-downlink path. v5.0: the source contract is dual-mode (`WOK_HAL_Src_*`) — arm charges a capacitor bank, abort is a safe bank dump, and the sweep pilot waveform is defined analytically in the header so the sim, the app's correlator, and a flight driver agree bit-for-bit.

**Implementations:**

- `linux_sim` — Simulated physics for desktop testing. A **shared subsurface geology model** (`wok_hal_sim_geology.c`: 3 layers, a Gaussian ice lens peaking at 5 wt%, per-binding volatile release spectra) is the single source of truth for BOTH instruments: the seismic sim synthesizes real first arrivals (direct vs head wave, offset moveout, geometric spreading) from it — as damped-sine wavelets for impulse shots, or as the delayed contract pilot for sweeps — and the drill sim derives torque/ROP/volatiles-vs-depth from the same model, so a correct inversion of the synthetic seismic agrees with what the synthetic drill finds. All geology functions are pure: each app links its own copy of the HAL lib, and cross-app consistency comes from determinism, not shared memory. Plus kinematic drive model, timed geo transitions, the EM source (bank charges at 50 W from the battery model, inhibited below a 23.0 V bus; impulse force ∝ √energy, 8.5–9.5 N at the 50 J default) on the same monotonic clock as the ADC, and a battery discharge model (note: the battery drains in real time — a running core reaches the 23.5 V drill inhibit after ~25–30 min and the 22.0 V SAFE threshold after ~45–55 min; restart the core between mission runs).
- `flight_stub` — Returns WOK_HAL_ERROR for everything. Copy and fill in real driver calls.

### 5.1 Onboard Data Products

All science products are flat binary files in `WOK_DATA_DIR` (`/tmp/wok_data` in
the sim, `/cf/wok_data` on flight storage), little-endian, fetchable via the
file-downlink path (`seis.fetch` in the console → `wok_archive/`).

| File | Producer | Contents |
|------|----------|----------|
| `seis_NNNN.wok` | SEIS, per shot | 112-byte v3 header (site pose, pod offsets, shot t₀ sample index, force, fs, SeqId, source mode/energy/pilot params, bank state) + interleaved int32 traces. The archival raw product — the full sweep record in sweep mode. |
| `corr_NNNN.wok` | SEIS, per sweep shot | Same header; the onboard-correlated equivalent-impulse trace over the 2 s stack window (t₀ at the window's PRE index). Same NNNN as its raw record. |
| `stack_NNNN.wok` | SEIS, per site | Same header (fold, coherence %, FirstBreakMs[4], PickQuality[4]) + the shot-aligned 2 s stacked window. Built from raw records (impulse) or correlated traces (sweep). |
| `coup_NNNN.wok` | SEIS, per site (placement missions) | Header-only coupling record: per-pod test-shot scores, seat attempts, plate transfer metric, sync/pick-consistency residuals. |
| `drill_NNNN.wok` | DRILL, per cycle | 40-byte header (site pose, target, RPM, heat/cool) + 36-byte rows (t, phase, depth, WOB, torque, current, cell temp, vapor pressure, ppm). Stiffness log + evolved-gas curve. |

Struct definitions: `wok_seis_msg.h` (`WOK_Seis_FileHeader_t`) and
`wok_drill_msgids.h` (`WOK_Drill_LogFileHeader_t`, `WOK_Drill_LogRow_t`).

## 6. Configuration

All runtime behavior is compile-time configured via `wok_platform_cfg.h` and `wok_drill_platform_cfg.h` (a few sequencer defaults — waypoint count, drill depth/RPM — live in `wok_seq_app.h`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| WOK_SEQ_MAX_WAYPOINTS | 64 | Maximum waypoints in table |
| WOK_SEQ_NUM_WAYPOINTS | 7 | Default waypoints loaded |
| WOK_NAV_DEFAULT_MAX_SPEED_MPS | 0.5 | Maximum drive speed |
| WOK_NAV_DEFAULT_ARRIVAL_TOL_M | 0.3 | Waypoint arrival tolerance |
| WOK_SEIS_DEFAULT_FS_HZ | 4000 | Sample rate (pick quantization ±0.25 ms over the 6 m array) |
| WOK_SEIS_DEFAULT_DURATION_SEC | 6 | Record window per strike |
| WOK_SEIS_MAX_CHANNELS | 4 | ADC channel count |
| WOK_SEIS_POD_OFFSETS_M_INIT | 1.5/3.0/4.5/6.0 | Geophone offsets from the strike point |
| WOK_SEIS_STACK_TARGET | 3 | Strikes stacked per site |
| WOK_SEIS_TRACE_MAX_SAMPS_PER_CH | 40960 | Per-shot trace buffer (≈10.2 s @ 4 kHz — covers max sweep + listen; 640 KB) |
| WOK_SRC_DEFAULT_ENERGY_J | 50.0 | Impulse bank energy (drop-weight-equivalent amplitude) |
| WOK_SRC_MAX_ENERGY_J | 200.0 | Bank charge ceiling |
| WOK_SRC_DEFAULT_SWEEP_F0/F1_HZ | 100/1600 | Pilot band (Klauder width 1/BW sets pick compatibility) |
| WOK_SRC_DEFAULT_SWEEP_DUR_S | 3.0 | Pilot duration (max WOK_SRC_SWEEP_MAX_DUR_SEC = 4) |
| WOK_SRC_MIN_CHARGE_BATT_V | 23.0 | Bank charge inhibited below this bus voltage |
| WOK_SEIS_CORR_SAMPS_PER_TICK | 1000 | Correlation output samples/chan per HK tick (main-loop bound) |
| WOK_SEQ_SOURCE_TIMEOUT_SEC | 30 | THUMP-state timeout (worst-case charge + full sweep) |
| WOK_SEIS_PLATE_WIN_SAMPS | 1200 | Plate-metric near-pod RMS window (300 ms @ 4 kHz after t₀) |
| WOK_SEIS_PLATE_COUP_FRAC | 0.70 | Plate verification fails below this fraction of the running baseline |
| WOK_SEIS_PLATE_EMA_ALPHA | 0.25 | Baseline EMA update weight (accepted shots only) |
| WOK_SEIS_MAX_PLATE_RESEAT | 2 | Plate reseat attempts per site before accepting degraded |
| WOK_GEO_MAX_RESEAT | 2 | Pod reseat attempts before placing MARGINAL (CP-3) |
| WOK_GEO_SEAT_FORCE_MIN_N | 26.0 | Seating-force proxy gate (arm force below this → reseat) |
| WOK_GEO_POD_TIMEOUT_MS | 12000 | Per-pod place/reseat/retrieve motion timeout |
| WOK_SEIS_POD_SCORE_MIN | 0.70 | Test-shot pod coupling score threshold (vs array median, CP-4) |
| WOK_SEQ_TEST_ENERGY_J | 10.0 | Coupling test-shot bank energy (charges in 0.2 s) |
| WOK_SEQ_TEST_DUR_SEC | 3 | Test-shot record window |
| WOK_SEQ_VERIFY_RESEAT_BUDGET | 2 | Targeted (test-shot-triggered) pod reseats per site |
| WOK_SEQ_MIN_VERIFIED_PODS | 3 | Partial-array policy minimum (below → CRITICAL, proceed flagged) |
| WOK_SEIS_SYNC_RESID_MAX_MS | 0.5 | Test-shot moveout-residual threshold → sync-suspect (CP-5; reported, not acted on) |
| WOK_SEQ_HOLD_TIMEOUT_SEC | 120 | Recalibration-hold auto-proceed timeout (unattended missions never deadlock) |
| WOK_SIM_TURN_BIAS_DEG | 0.0 | Sim-only: unsensed heading bias per completed turn (env-overridable); default 0 keeps regressions byte-identical |
| WOK_HK_DEFAULT_BATT_MIN_V | 22.0 | Battery low threshold |
| WOK_HK_DEFAULT_TEMP_MAX_C | 55.0 | Temperature high threshold |
| WOK_HK_DEFAULT_TILT_MAX_DEG | 30.0 | Tilt fault threshold |
| WOK_DRILL_MIN_BATTERY_V | 23.5 | Minimum battery for drill ops |
| WOK_DRILL_DEFAULT_MAX_DEPTH_M | 5.0 | Maximum drill depth |
| WOK_DRILL_DEFAULT_MAX_RPM | 300 | Maximum drill RPM |
| WOK_DRILL_DEFAULT_ARM_TIMEOUT_MS | 8000 | Arm motion timeout |

## 7. Directory Layout

```
wok-cfs/                          This repository
├── apps/
│   ├── wok_common/               Shared WOK infrastructure
│   │   ├── msgs/inc/             wok_msgids.h, wok_mission_types.h, wok_platform_cfg.h
│   │   ├── hal/inc/              Per-subsystem HAL headers (8 files)
│   │   ├── hal/src/linux_sim/    Simulated HAL (10 .c + 4 private .h — incl. shared geology & coupling truth models)
│   │   └── hal/src/flight_stub/  Stub HAL (fill with real drivers)
│   ├── wok_sequencer/fsw/        SEQ app source
│   ├── wok_nav/fsw/              NAV app source
│   ├── wok_geo/fsw/              GEO app source
│   ├── wok_seis/fsw/             SEIS app source
│   ├── wok_drill/fsw/            DRILL app source
│   └── wok_hk_health/fsw/        HK app source
├── cfs_integration/              SCH_LAB, TO_LAB, startup patches
├── ground/                       Ground control tools
│   ├── wok_console.py            CLI mission console
│   ├── wok_bridge.py             WebSocket ↔ UDP bridge
│   ├── wok_index_html.html       React web UI (served by wok_sim.sh)
│   └── wok_refraction.py         Site analysis: refraction inversion, tie points, prospect ranking
├── tools/                        Dev/verification tooling
│   ├── wok_sim.sh                One-command sim stack up/down/status
│   ├── run_src_harness.sh        Build + run the physics harness modes (CI)
│   ├── smoke_test.sh             Boot a built tree, check all six apps init (CI)
│   ├── wok_mission_tools.py      Scriptable mission commands, fetch, validate
│   ├── wok_regress.py            Regression comparison vs archived baseline
│   └── wok_src_harness.c         Standalone sim-HAL physics harness
├── docs/                         Documentation
├── install.sh                    Install into cFS tree
└── README.md
```
