# WOK Ground Console — User Guide

**Document:** WOK-GC-001  
**Version:** 5.0  
**Date:** July 2026

---

## Overview

The WOK ground console (`wok_console.py`) is an interactive terminal for commanding and monitoring the WOK seismic rover mission over cFS. It communicates via CI_LAB (UDP command uplink) and TO_LAB (UDP telemetry downlink).

## Starting the Console

    python3 ~/wok-cfs/ground/wok_console.py

With options:

    python3 wok_console.py --host 127.0.0.1 --cmd-port 1234 --tlm-port 2234

Default ports: CI_LAB command uplink on 1234, TO_LAB telemetry downlink on 2234.

Tab completion is enabled. Command history is saved to `~/.wok_history`.

## Prerequisites

Before using the console:

1. cFS must be running (`./core-cpu1`)
2. TO_LAB must be enabled:

       ~/wok-cfs-build/build/exe/host/cmdUtil --pktid=0x1880 --cmdcode=6 --endian=LE --string="16:127.0.0.1"

3. Start the console (it must be running when telemetry arrives — it binds the UDP port on startup)

## Command Reference

### Sequencer (SEQ)

| Command | Description |
|---------|-------------|
| `seq.noop` | Verify command path |
| `seq.reset` | Reset cmd/err counters |
| `seq.start` | Begin waypoint loop |
| `seq.abort` | Abort to SAFE immediately |
| `seq.resume` | Resume from SAFE |
| `seq.waypoint <idx>` | Override waypoint index |
| `seq.mode <0\|1>` | Mission mode: 0 = drill-inline, 1 = survey-first (score all sites, then drill the best) |
| `seq.placement <0\|1>` | Placement mode: 0 = fixed array (legacy, default), 1 = per-pod placement — DEPLOY/VERIFY/STOW run the WOK-CP-001 placement, coupling-verification, and retrieval sub-sequences |
| `seq.loadwp <idx> <fwd> <lat> [drill] [srcmode] [energy_J] [recal]` | Upload/replace one waypoint (start-frame deltas, metres). `srcmode` 0 = impulse (default), 1 = sweep; `energy_J` impulse bank energy, 0 = flight default; `recal` 1 = hold after MOVE for ground recalibration. Zeros preserve prior behavior exactly. |
| `seq.release` | Release a recalibration HOLD (rejected outside HOLD; the hold also auto-proceeds after `WOK_SEQ_HOLD_TIMEOUT_SEC`) |
| `seq.wpcount <n>` | Set the number of active waypoints |
| `seq.hk` | Request housekeeping TLM |

Mode TLM shows targeting status: `mission_mode`, `drills_completed`,
`best_site`, `best_score`. The tie-point mission-success gate guarantees at
least one drill inside the survey footprint: if the loop ends with zero
drills, the rover returns to the best-scoring site and drills it (even a
dry tie anchors the Vp→ice transfer function).

### Navigation (NAV)

| Command | Description |
|---------|-------------|
| `nav.noop` | Verify command path |
| `nav.reset` | Reset counters |
| `nav.goto <df> <dl>` | Relative-step move: forward + lateral (start frame, +left), metres |
| `nav.stop` | Immediate halt |
| `nav.speed <m/s>` | Set max speed |
| `nav.setpose <x> <y>` | Overwrite the dead-reckoned pose with ground truth (bookkeeping only; rejected while driving) |
| `nav.realign <err_deg>` | Corrective in-place turn of −error, then declare the current heading start-frame (drift zeroed, `realign_count` increments; rejected unless idle/arrived/stopped) |
| `nav.hk` | Request housekeeping TLM |

### Geophone (GEO)

| Command | Description |
|---------|-------------|
| `geo.noop` | Verify command path |
| `geo.reset` | Reset counters |
| `geo.deploy` | Extend geophone arm (legacy fixed array) |
| `geo.stow` | Retract geophone arm (legacy fixed array) |
| `geo.abort` | Emergency stop mechanism (halts pod motions too) |
| `geo.place <pod> <x> <y>` | Place one pod at (x,y) — contingency/manual path (sequencer-driven in placement missions); GEO runs the seat → force-proxy → reseat loop internally |
| `geo.retrieve <pod>` | Recapture one placed pod (CARRIED again = confirmed) — the recovery path after e.g. a SAFE abort mid-placement |
| `geo.reseat <pod>` | One commanded reseat of a placed pod (no internal retry — the test shot is the arbiter) |
| `geo.hk` | Request housekeeping TLM |

The per-pod commands are sequencer-driven in placement mode; the console
forms above exist for contingency and bench work. GEO status TLM carries
the per-pod states, seating forces, reseat counts, and placed/marginal
masks (decoded by the console and the dashboard).

### Seismic (SEIS)

| Command | Description |
|---------|-------------|
| `seis.noop` | Verify command path |
| `seis.reset` | Reset counters |
| `seis.start [dur] [fs] [gain] [ch] [x] [y] [hdg] [sidx] [starget]` | Start acquisition |
| `seis.stop` | Stop acquisition |
| `seis.thump_arm` | Arm the source (charge the capacitor bank) |
| `seis.thump_fire` | Fire the source (impulse strike, or sweep start) |
| `seis.thump_reset` | Safe the source back to idle (dumps the bank if charged) |
| `seis.source <mode> [energy_J] [f0] [f1] [dur_s]` | Configure the EM source: mode 0 = impulse (cap discharge, `energy_J` sets amplitude), 1 = sweep (linear pilot `f0`–`f1` Hz over `dur_s`). Zero/omitted values keep the onboard setting. |
| `seis.plate_reseat` | Lift and re-press the source coupling plate — contingency/manual path (sequencer-driven on a plate-metric failure); rejected while charging/armed/sweeping |
| `seis.evaluate` | Evaluate data for ice signature |
| `seis.fetch <filename>` | Downlink a stored data product to `wok_archive/` |
| `seis.hk` | Request housekeeping TLM |

`seis.start` arguments (all optional, defaults in parentheses):

- `dur` — duration in seconds (6)
- `fs` — sample rate in Hz (4000)
- `gain` — ADC gain setting 0-3 (1)
- `ch` — channel mask in hex (0F = all 4 channels)
- `x y hdg` — site pose stamp, start frame (0 0 0)
- `sidx starget` — stack index / stack target (0 1 = unstacked single record)

Example: `seis.start 6 4000 1 0F 3.0 0.0` — one 6-second record stamped at site (3,0).

`seis.fetch` drives a stop-and-wait chunk transfer (retries per chunk), reassembles the file, and writes it to `wok_archive/`. Works for any onboard product: `seis.fetch stack_0004.wok`, `seis.fetch drill_0002.wok`, `seis.fetch seis_0012.wok`, `seis.fetch corr_0012.wok`, `seis.fetch coup_0004.wok` (per-site coupling record: pod scores, seat attempts, plate metric, sync residuals).

SET_POD_GEOMETRY (CC 11) is sequencer-computed (actual placed geometry) and has no console form; SEIS status TLM carries the plate metric (`plate_ratio`, `plate_baseline`, `plate_ok`, `plate_reseats`, `plate_degraded`), the test-shot pod scores (`pod_scores`, `pod_score_ok_mask`, `test_shots`), and the sync residuals (`sync_residual_ms`, `sync_suspect_mask`) — all decoded by the console.

**Dual-mode source workflow (v5.0):** the source defaults to impulse mode
at the flight-default energy — nothing to configure for legacy operations.
For a sweep shot: `seis.source 1` (optionally with band/duration), then the
usual `seis.start` → `seis.thump_arm` → `seis.thump_fire`. In sweep mode
the record window automatically extends by the sweep duration, and after
the record completes the app cross-correlates it against the regenerated
pilot (watch `corr_progress_pct` in TLM); the equivalent-impulse trace is
archived as `corr_NNNN.wok` alongside the raw sweep record `seis_NNNN.wok`,
and site stacks are built from the correlated traces. Per-site source
selection for sequenced missions goes through `seq.loadwp`'s
`srcmode`/`energy_J` fields.

### Drill (DRILL)

| Command | Description |
|---------|-------------|
| `drill.noop` | Verify command path |
| `drill.reset` | Reset counters |
| `drill.deploy [angle]` | Deploy arm (default 90 deg) |
| `drill.stow` | Retract arm |
| `drill.start [depth] [rpm] [x] [y]` | Begin drilling (default 1.0m, 200 RPM, site 0,0) |
| `drill.stop` | Stop drill motor |
| `drill.sample [heat_s] [cool_s]` | Thermal sample (default 60s heat, 30s cool) |
| `drill.abort` | Emergency stop all drill ops |
| `drill.hk` | Request housekeeping TLM |

### Health / HK

| Command | Description |
|---------|-------------|
| `hk.noop` | Verify command path |
| `hk.reset` | Reset counters |
| `hk.thresholds [V] [C] [deg]` | Set fault thresholds (22.0, 55.0, 30.0) |
| `hk.clear` | Clear all fault flags |
| `hk.hk` | Request housekeeping TLM |

### Convenience Commands

| Command | Description |
|---------|-------------|
| `allnoop` | NOOP all 6 apps — quick connectivity check |
| `allreset` | Reset counters on all 6 apps |
| `allhk` | Request HK from all 6 apps |
| `quickstart [delay]` | NOOP all → wait → SEQ START |
| `safestop` | SEIS stop → DRILL e-stop → NAV stop → GEO stow → SEQ ABORT |

### Telemetry Commands

| Command | Description |
|---------|-------------|
| `tlm [app]` | Show latest telemetry (seq/nav/geo/seis/hk/drill/all) |
| `tlm.verbose` | Toggle live packet printing |

### Test Suites

| Command | Description |
|---------|-------------|
| `test.connectivity` | NOOP all + verify TLM arrives |
| `test.seq` | SEQ: NOOP, RESET, START, ABORT, RESUME |
| `test.nav` | NAV: NOOP, GOTO, STOP, SET_POSE, REALIGN (incl. reject-while-driving) |
| `test.geo` | GEO: NOOP, DEPLOY, STOW, ABORT |
| `test.seis` | SEIS: NOOP, START_ACQ, STOP_ACQ |
| `test.hk` | HK: NOOP, SET_THRESHOLDS, CLEAR_FAULTS |
| `test.drill` | DRILL: NOOP, DEPLOY, STOW, ABORT |
| `test.all` | Run all test suites in sequence |

## Ground Archive

The console writes to a `wok_archive/` directory (created in the working
directory):

- Files fetched with `seis.fetch` (raw traces, stacks, drill logs)
- `drill_status.csv` — every DRILL STATUS packet received is appended
  automatically (depth, WOB, torque, current, ROP, cell temp, vapor
  pressure, volatile ppm, sample state) — a ready-to-plot record of every
  drill cycle observed during the session

SEIS STATUS telemetry now includes strike-to-trace sync (`strike_sample_idx`,
`strike_valid`, `trace_samples`) and, after each site's final strike, the
onboard first-break picks (`first_break_ms`, `pick_quality`, `stack_fold`,
`coherence_pct`). v5.0 adds the EM source status: `source_mode`, `bank_V`,
`bank_soc_pct` (percent of the shot's charge target), `sweep_progress_pct`,
and `corr_progress_pct` (onboard correlation, 100 = idle/complete). The
`thumper` state field reads IDLE/CHARGING/ARMED/FIRED/SWEEPING. DRILL
STATUS includes `wob_N` and `rop_mps`.

## Web Bridge

`wok_bridge.py` serves the same command/telemetry streams to browser
clients over WebSocket:

    python3 wok_bridge.py [--cfs-host 127.0.0.1] [--cmd-port 1234] [--tlm-port 2234] [--ws-port 8765]

It is a **pure relay**: every telemetry value originates from the
HAL-backed FSW via TO_LAB (decoded to JSON and broadcast, including the
v4.0 targeting fields and file-chunk progress), and every command goes to
CI_LAB. There is no simulation mode — to run without hardware, run cFS
with the `linux_sim` HAL. On client connect the bridge sends the TO_LAB
enable automatically. Note the bridge and the console both bind the
telemetry port (2234); run one at a time. The bridge requires the
`websockets` Python package (`pip install websockets`), and the web
dashboard loads React from a CDN, so the browser needs internet access
even when the sim itself is fully local.

## Site Analysis Tool

`wok_refraction.py` processes fetched (or sim-local) data products into the
per-site prospecting quantities:

    python3 wok_refraction.py [data_dir] [--csv prospect_summary.csv]

Default `data_dir` is `wok_archive/` if present, else `/tmp/wok_data`.
Per stack file it inverts the onboard picks for v1, v2 [lo–hi], layer
thickness h1 [lo–hi], and the ice posterior fraction. Sweep-mode stacks
(header v3, source = sweep) are handled identically — they are stacks of
onboard-correlated equivalent-impulse traces, so the picks and geometry
read the same and the inversion is unchanged; the report labels each
stack's source mode. Per drill log it
reports the competence-log interface depth and decomposes the evolved-gas
heating curve into adsorbed / pore-ice / hydrated components — recovering
the total volatile abundance including what stayed bound above the 300 °C
cell limit. Sites with both products get an explicit tie-point line
(drill interface vs seismic h1, and the v2 ↔ wt% calibration pair).
It also reads per-site coupling records (`coup_NNNN.wok`) when present
and deweights low-coupling channels in the stack inversion.

## SeqId in Telemetry

Status telemetry from child apps (NAV, GEO, SEIS, DRILL) includes a `seq_id` field — the monotonic sequence ID echoed from the last command received. Ground commands send SeqId=0 (don't-care). The sequencer's HK telemetry shows the current SeqId counter, which increments with each sequenced command sent.

## Telemetry Display

`tlm` shows decoded telemetry from the most recent packet received for each app. Example output:

    wok> tlm seq
      SEQ_HK
        cmd_cnt              2
        err_cnt              0
        state                MOVE
        prev_state           IDLE
        waypoint_idx         0
        faults               NONE
        fault_raw            0
        seq_id               5

`tlm all` shows all apps. Faults are highlighted in red.

## Typical Workflow

1. Start cFS and enable TO_LAB
2. Launch console
3. Verify connectivity: `allnoop`
4. Check health: `tlm hk`
5. Start mission: `seq.start`
6. Monitor: `tlm` (repeat as needed)
7. If something goes wrong: `safestop`
8. Run full validation: `test.all`

## Debugging Tips

- Use `tlm.verbose` to see every packet as it arrives, with MID and size
- If `tlm` shows "no data received yet", ensure TO_LAB is enabled and the console port (2234) isn't blocked
- If commands don't reach apps, verify CI_LAB is listening: look for "CI_LAB listening on UDP port: 1234" in cFS output
- The `0x0880` packets in verbose mode are TO_LAB's own HK — this is normal
