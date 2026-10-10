# WOK cFS Flight Software

[![build](https://github.com/Aegis-Station-Infrastructure/wok-cfs/actions/workflows/build.yml/badge.svg)](https://github.com/Aegis-Station-Infrastructure/wok-cfs/actions/workflows/build.yml)

**Water Observation Kit — Lunar Seismic Survey & Ice Prospecting Rover**

WOK is a simulated lunar rover mission you can build and fly on any Linux
box. It is a simulation and educational project: nothing here is flight
software or flight-qualified, and every number in it is a design
assumption, not a measurement. The flight software is six apps on NASA's
core Flight System (cFS, Draco v7.0.0) that autonomously drive a rover
between survey sites, fire a seismic source, record and pick first
arrivals on a geophone array, decide onboard whether a site looks
ice-bearing, and drill to confirm. All hardware access goes through a
per-subsystem Hardware Abstraction Layer: the included `linux_sim` HAL
implements a physics simulation (a shared subsurface geology model feeds
both the seismic and drill instruments, so inversion algorithms can be
checked against the model's known ground truth), and a `flight_stub` HAL
is the template for porting to real hardware. A ground console, web
dashboard, and analysis tools complete the loop from command uplink to a
ranked ice-prospect map.

## Quick Start

Full detail and troubleshooting in the
[Build & Install Guide](docs/WOK_Build_Install_Guide.md) — the steps below
are the whole path.

```bash
# 0. One-time host setup (cFS pipes are POSIX mqueues; default depth starves downlink)
sudo sysctl fs.mqueue.msg_max=128

# 1. Clone cFS Draco v7.0.0 (pin the tag — the default branch is a different bundle)
git clone --branch v7.0.0 https://github.com/nasa/cFS.git ~/wok-cfs-build
cd ~/wok-cfs-build
git submodule init && git submodule update --recursive
cp cfe/cmake/Makefile.sample Makefile
cp -r cfe/cmake/sample_defs sample_defs

# 2. Clone this repo and install WOK into the cFS tree (apps, targets.cmake,
#    startup script, TO_LAB and SCH_LAB tables, ci_lab/to_lab port-offset patch)
git clone https://github.com/Aegis-Station-Infrastructure/wok-cfs.git ~/wok-cfs
cd ~/wok-cfs
./install.sh ~/wok-cfs-build

# 3. Build
cd ~/wok-cfs-build
make SIMULATION=native prep
make -j$(nproc)
make install

# 4. Run the flight software
cd build/exe/cpu1
./core-cpu1          # look for "App initialised" from all 6 WOK apps

# 5. Ground console (second terminal)
~/wok-cfs-build/build/exe/host/cmdUtil --pktid=0x1880 --cmdcode=6 --endian=LE --string="16:127.0.0.1"
python3 ~/wok-cfs/ground/wok_console.py
wok> allnoop         # all 6 apps should event a NOOP
wok> test.all        # full command/telemetry test suite
```

Alternatively, `tools/wok_sim.sh up` starts the core, the WebSocket
bridge, and the web dashboard in one command once the build exists.

## Your First Mission

With the core running and the console connected (`rm -rf /tmp/wok_data`
first for a clean slate):

```
wok> seq.start
```

The rover flies the stock 7-waypoint survey autonomously in ~12 minutes.
Watch the cFS terminal — the mission narrates itself via events:

```
WOK_SEQ: IDLE → MOVE (WP 0/7)
WOK_SEQ: Site 0 geometry — (0.00, 0.00) hdg 0.0
WOK_SEQ: DEPLOY → THUMP (WP 0/7)          ← 3 shot/record cycles per site
WOK_SEIS: seis_0003.wok — 24k samps x4 ch, strike@4160, site (0.0,0.0)
WOK_SEQ: Ice signature (78% conf) — DRILLING at WP 3
WOK_DRILL: drill_0002.wok — 512 rows, site (6.0,3.0), final 43750.6 ppm
WOK_SEQ: STOW → DONE (WP 7/7)             ← mission complete
```

**Success looks like:** `STOW → DONE (WP 7/7)`, at least one
`DRILLING at WP n` along the way, and `/tmp/wok_data/` holding
`seis_`/`stack_`/`drill_*.wok` products. Then run the analysis:

```bash
python3 ~/wok-cfs/ground/wok_refraction.py /tmp/wok_data
```

It inverts each site's picks into velocity/depth/ice-fraction, decomposes
the drill evolved-gas curves, fits the drill-to-seismic transfer function,
and prints a ranked prospect queue. A successful stock run recovers the
sim's buried ice lens: **site (6.0, 3.0) ranks #1, CONFIRMED (drilled), at
~5 wt% water** — matching the geology model's 5 wt% peak at that position.

## Architecture

```
                    cFS / cFE  (ES, SB, EVS, TBL, TIME)
                                 |
                          (Software Bus)
                                 |
        SEQ ──── NAV ──── GEO ──── SEIS ──── DRILL ──── HK
        (SM)    (drive)  (mech)  (ADC+src)   (ice)    (FDIR)
         │        │       │        │          │        │
         └────────┴───────┴────────┴──────────┴────────┘
                                 |
                       WOK HAL (per-subsystem)
                                 |
                     ┌───────────┴───────────┐
                     │                       │
                 linux_sim               flight_stub
               (sim physics)          (port to your HW)
```

The sequencer orchestrates the mission via SeqId-gated commands to child apps. The state machine runs only on the HK tick; child telemetry updates cached data but never advances state directly.

**Operational loop:** IDLE → MOVE → [HOLD] → DEPLOY → [VERIFY] → [THUMP → RECORD] ×3 → EVALUATE → [DRILL if ice flagged + battery OK] → STOW → MOVE → … → DONE

**Navigation recalibration:** NAV dead-reckons in the start frame, and heading drift compounds across steps (each step's turn-back restores the *believed* heading, not the true one), degrading site stamps over a long traverse. Two ground commands close the loop: `nav.setpose <x> <y>` overwrites the pose with ground truth (bookkeeping only), and `nav.realign <error_deg>` executes a corrective in-place turn and re-declares the start-frame heading. A per-waypoint recal flag (`seq.loadwp … recal=1`) holds the mission after MOVE for a ground pass — released by `seq.release` or a wall-clock timeout, so unattended missions never deadlock. The sim models the error source (`WOK_SIM_TURN_BIAS_DEG`, unsensed per-turn heading bias, default 0).

**Coupling & placement:** in per-pod placement mode (`seq.placement 1`) the rover creates the survey aperture itself — DEPLOY becomes a placement sub-sequence (drive to each pod position, place, gate on the arm's seating-force proxy with bounded reseats), VERIFY fires a low-energy test shot that scores every pod's actual ground coupling (plate factor cancels) and drives targeted reseats, and STOW is the retrieval mirror with per-pod recapture confirmation. Coupling is a confirmed, measured condition — not an endstop event: per-pod scores, seat attempts, plate transfer metric, and sync/pick-consistency residuals are telemetered and archived per site (`coup_NNNN.wok`), and `wok_refraction.py` deweights low-coupling channels. The default fixed-array mode preserves legacy behavior exactly. Sim truth toggles: `WOK_SIM_COUPLING=1`, `WOK_SIM_SYNC_ERR_MS=<ms>`.

Each site gets three stacked shots from a dual-mode EM source (capacitor-bank impulse by default, or a swept-sine pilot correlated onboard into an equivalent-impulse trace), recorded with a hardware-referenced electrical t₀, onboard STA/LTA first-break picks, and archived trace/stack files. Drill cycles log a drilling-response record (depth, WOB, torque, ROP) and an evolved-gas curve (volatile ppm vs cell temperature — the release temperature classifies how the water is bound). Every product is stamped with the surveyed site pose, so co-located seismic + drill results form the calibration tie point the prospecting case rests on.

The drill decision itself is onboard geophysics: EVALUATE inverts the site's first-break picks for refractor velocity and layer depth, and flags the site on the grid-posterior probability that the refractor is ice-cemented (Vp above 1200 m/s; "ice-fast" in the code). Four pods and three shots per site is simulation scale; a field survey would use 12–24 channels and reversed shots.

**Mission modes & the tie-point gate:** `seq.mode 1` selects survey-first targeting — score every site, then return to the best and drill it. Waypoints are ground-uploadable (`seq.loadwp` / `seq.wpcount`). In any mode, a mission cannot end with zero drills: the sequencer returns to its best-scoring site for at least one calibration tie, because that tie is what anchors the Vp→ice transfer function `wok_refraction.py` fits and extrapolates across the whole survey (ranked prospect queue with CONFIRMED-vs-inferred labels).

Any critical fault triggers SAFE (NAV stop, GEO stow, SEIS stop, DRILL e-stop), where the vehicle awaits ground commands.

**Data products** (fetch with `seis.fetch <name>` in the console): per-shot raw traces (`seis_NNNN.wok`), correlated equivalent-impulse traces for sweep shots (`corr_NNNN.wok`), per-site stacks with picks (`stack_NNNN.wok`), per-site coupling records from placement missions (`coup_NNNN.wok`), per-cycle drill logs (`drill_NNNN.wok`). Formats documented in the Architecture guide §5.1.

## Apps

| App | cFS Name | Priority | Role |
|-----|----------|----------|------|
| `wok_sequencer` | WOK_SEQ | 80 | Mission executive, state machine, safety inhibits, SeqId enforcement |
| `wok_seis` | WOK_SEIS | 85 | Seismic ADC acquisition, dual-mode EM source control, sweep correlation, ice-signature evaluation |
| `wok_nav` | WOK_NAV | 90 | Waypoint driving, pose tracking, ground-commanded recalibration |
| `wok_geo` | WOK_GEO | 90 | Geophone array: legacy deploy/stow + per-pod place/reseat/retrieve with seating-force gate |
| `wok_drill` | WOK_DRILL | 90 | Drill arm, drilling, thermal-vaporization sampling, volatile analysis |
| `wok_hk_health` | WOK_HK | 95 | System health monitoring, FDIR-lite |

## Ground System

- `ground/wok_console.py` — interactive command console (all commands, test suites, telemetry decode, file fetch). See the [Ground Console Guide](docs/WOK_Ground_Console_Guide.md).
- `ground/wok_bridge.py` + `ground/wok_index_html.html` — WebSocket bridge and React web dashboard (live state machine, telemetry panels, command buttons).
- `ground/wok_refraction.py` — site analysis: refraction inversion, evolved-gas decomposition, tie points, prospect ranking.
- `tools/wok_sim.sh` — one-command sim stack `up|down|status`.
- `tools/wok_mission_tools.py` — scriptable mission commands, file fetch, pick validation against the sim's geology truth.
- `tools/wok_regress.py` — regression comparison of two mission logs/product sets.
- `tools/wok_src_harness.c` — standalone physics harness that compiles the sim HAL without cFS; `tools/run_src_harness.sh` builds and runs its verification modes.
- `tools/smoke_test.sh` — boots a built cFS tree and checks all six apps initialise (what CI runs).

## Documentation

- [Operations Manual](docs/WOK_Operations_Manual.md) — Operator procedures: mission planning, monitoring, recalibration passes, coupling interpretation, contingencies, data handling
- [Architecture & Design](docs/WOK_Architecture_Design.md) — Full technical reference
- [Build & Install Guide](docs/WOK_Build_Install_Guide.md) — Step-by-step setup
- [Ground Console Guide](docs/WOK_Ground_Console_Guide.md) — All commands and test suites
- [HAL Porting Guide](docs/WOK_HAL_Porting_Guide.md) — Wiring real hardware
- [Sim Model Spec](docs/WOK_Sim_Model_Spec.md) — Every equation the sim uses to mimic bench/flight signals, with calibration notes

## Message IDs

Commands: `0x18D0`–`0x18DB` (per app: CMD + SEND_HK) | Telemetry: `0x08D0`–`0x08DC` (HK/status per app, plus SEIS quicklook and file-chunk streams)

Allocated at topic offset 0xD0+ to avoid collisions with stock Draco apps (TO_LAB=0x80, SAMPLE_APP=0x82, CI_LAB=0x84). Authoritative list: `apps/wok_common/msgs/inc/wok_msgids.h`.

## Linux host notes

- Raise the kernel POSIX message-queue limit before running (`sudo sysctl fs.mqueue.msg_max=128`) — the default of 10 silently clamps every cFS pipe and starves the file-downlink path. `install.sh` warns when it's too low.
- The sim battery drains in real wall-clock time: the drill inhibit arrives ~28 min after core start and SAFE ~53 min. Restart the core between mission runs.
- Sim data products land in `/tmp/wok_data`; clear it between missions to avoid mixing runs.

## Multi-instance (survey groups)

`install.sh` patches the bundled `ci_lab`/`to_lab` so they honour an
`AEGIS_PORT_OFFSET` environment variable
(`cfs_integration/port-offset.patch`), letting several WOK instances
coexist on one host — instance *i* runs at offset `i*100` (CI
`1234+offset`, TLM `2234+offset`). Each instance also needs its own
working directory (cFE keys SysV shared memory by `ftok()` on cwd files).
`tools/wok_mission_tools.py` honours `AEGIS_PORT_OFFSET` (or `--host`,
`--cmd-port`, `--tlm-port`) so it can drive a second instance.

## Status

Verified on cFS Draco v7.0.0 (the `v7.0.0` bundle tag) on Ubuntu-family Linux with the `linux_sim` HAL. The stock seven-site mission runs to DONE in about 12 minutes, drills twice, and the ground analysis recovers the simulator's buried ice lens at the right site and concentration. First-break picks land within one sample (0.25 ms) of the model's arrival times, which is a self-consistency check of the signal chain rather than a fidelity claim: the signal and the truth come from the same model. The flight HAL is a stub; the [HAL Porting Guide](docs/WOK_HAL_Porting_Guide.md) describes what real hardware has to provide, including the per-pod timing contract that wireless geophones would need.

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).
Copyright 2026 Aegis Station Infrastructure LLC. NASA cFS is not
included in this repository; the two files under `cfs_integration/`
derived from NASA cFS carry NASA's own Apache 2.0 notice.

This is a simulation and educational project. Nothing here is
flight-qualified.
