# WOK HAL — Porting Guide

**Document:** WOK-HP-001  
**Version:** 5.0  
**Date:** July 2026

---

## Overview

The WOK Hardware Abstraction Layer (HAL) is the single portability boundary between the flight software and the hardware. Every sensor read, actuator command, and peripheral interaction goes through the HAL headers. The flight apps never touch hardware directly.

The HAL is per-subsystem (v2): 8 headers in `hal/inc/` define the
contract. The Linux sim implements them in 10 source files (the extra
two are the shared geology and coupling truth models); the flight stub
implements the same contract in 2 files.

To port WOK to real hardware:

1. Copy the `flight_stub/` files to a new target directory
2. Replace each function body with real driver calls
3. Select it with the `WOK_HAL_BACKEND` cache variable in
   `wok_common/CMakeLists.txt`, which names the `hal/src/<backend>/`
   directory to compile (`linux_sim` by default, `flight_stub`, or your
   own name once you add it to the if/elseif there); see Step 4 of the
   walkthrough

The app logic is identical between desktop simulation and flight hardware — only the HAL implementation changes.

## Architecture

```
    WOK Apps (wok_sequencer, wok_nav, wok_geo, wok_seis, wok_drill, wok_hk_health)
                              |
              Per-subsystem HAL headers   <-- THE API CONTRACT
              wok_hal_mobility.h
              wok_hal_geo.h
              wok_hal_seis.h
              wok_hal_drill.h
              wok_hal_power.h
              wok_hal_watchdog.h
              wok_hal_storage.h
              wok_hal.h (master)
                              |
              +---------------+---------------+
              |                               |
         linux_sim/                     your_target/
    10 sim source files            your target source files
    (simulated physics)           (real BSP/driver calls)
```

Each app includes only its specific HAL header (e.g., wok_nav includes `wok_hal_mobility.h`, wok_drill includes `wok_hal_drill.h`). The monolithic `wok_hal.h` / `wok_hal_linux_sim.c` pattern does not exist in v2.

## HAL API Reference

All functions return `int32_t`: `WOK_HAL_OK` (0), `WOK_HAL_ERROR` (-1), or `WOK_HAL_TIMEOUT` (-2).

### System

| Function | Description |
|----------|-------------|
| `WOK_HAL_Init()` | Initialize all HAL subsystems. Called once at startup. |
| `WOK_HAL_Shutdown()` | Clean shutdown. Release resources. |

### Mobility

The mobility HAL exposes elementary motion primitives. NAV sequences these into relative-step missions — there is no notion of waypoints, position, or "frames" inside the HAL.

| Function | Description |
|----------|-------------|
| `WOK_HAL_Mobility_Init()` | Initialize motor controller, encoders, IMU |
| `WOK_HAL_Mobility_Stop()` | Immediate halt — cut motor power |
| `WOK_HAL_Mobility_SetSpeed(spd)` | Update default drive-speed cap |
| `WOK_HAL_Mobility_DriveForward(dist_m, spd_mps)` | Begin a straight-line drive of `dist_m` (≥ 0). Non-blocking. |
| `WOK_HAL_Mobility_Turn(angle_deg, spd_dps)` | Begin in-place rotation. Signed angle: `+ = left (CCW)`, `- = right (CW)`. Non-blocking. |
| `WOK_HAL_Mobility_GetDistanceTraveled(*m)` | Metres traveled since last `DriveForward` (latched after completion) |
| `WOK_HAL_Mobility_GetHeadingChange(*deg)` | Signed degrees rotated since last `Turn` (latched after completion) |
| `WOK_HAL_Mobility_IsMoving(*moving)` | 1 while last primitive is in progress, 0 once complete |
| `WOK_HAL_Mobility_GetSpeed(*spd)` | Current linear speed (m/s); 0 during turns |
| `WOK_HAL_Mobility_GetTilt(*deg)` | Tilt angle from IMU |
| `WOK_HAL_Mobility_GetMotorCurrent(*amps)` | Drive motor current |

**Typical hardware:** Motor controller via CAN/UART/PWM, wheel encoders (quadrature or absolute), IMU/gyro via SPI/I2C for heading-change integration, current-sense ADC.

**Implementation notes:**

- `DriveForward` and `Turn` are non-blocking — start the motion and return. NAV polls `GetDistanceTraveled` / `GetHeadingChange` and `IsMoving` to detect completion.
- The two `Get*` accumulators are *per-primitive*: each `DriveForward` zeroes `distance_traveled_m` before starting; each `Turn` zeroes `heading_change_deg`. After completion, the latched value persists until the next call to the same primitive.
- `Turn` accepts a signed angle. The sign carries direction; `GetHeadingChange` returns the signed accumulated rotation.
- `Stop` must be immediate — cut motor power, don't ramp down.
- The sim does not implement reverse driving (`dist_m < 0` returns ERROR). NAV handles "drive backward in the start frame" by turning 180°. Flight HALs may support reverse-drive if the rover does.

**Ground recalibration (REALIGN) — flight notes.** NAV's `REALIGN`
command executes its corrective turn through `Turn()` — there is no new
HAL surface to port — but three properties of the driver determine how
well the whole correction loop works:

- **Report what you measure, not what you commanded.**
  `GetHeadingChange` must return the encoder/IMU-*measured* rotation.
  The unsensed residual between actual and measured motion is exactly
  the drift that ground REALIGN exists to correct; a driver that
  synthesizes the accumulator from the commanded angle hides its own
  error from telemetry without removing it from the vehicle, and makes
  the dead-reckoned pose look better than it is. (The sim deliberately
  models this: its accumulators report the commanded turn while an
  unsensed `WOK_SIM_TURN_BIAS_DEG` acts on the true heading.)
- **Turn-execution accuracy is the post-realign residual floor.** The
  corrective turn rides the same drivetrain as every other turn, so a
  systematic per-turn error of `b` degrees leaves a `b`-degree residual
  after every realign — sim-quantified: at 2°/turn bias, site-stamp
  error regrew to ~0.2 m over one post-realign waypoint instead of
  recovering fully. Characterize the drivetrain's systematic turn error
  on the bench; if the floor is too coarse for the stamp budget, close
  the turn loop against the IMU *inside* the driver (`Turn` servos to
  the measured angle) rather than open-loop timing — iterating REALIGNs
  from ground converges to the floor and no further.
- **The heading error is a ground input, not a HAL measurement.**
  Ground derives it from external references (sun-sensor imagery,
  landmark triangulation, star tracker) and uplinks a signed degree
  value: `+` = the rover points left of its believed heading; NAV turns
  `-error`. Operationally REALIGN stops further error *growth* and
  `SET_POSE` clears the already-accumulated position error — a ground
  pass at a recal-hold waypoint should send both.

### Geophone Mechanism

| Function | Description |
|----------|-------------|
| `WOK_HAL_Geo_Init()` | Initialize actuator, endstop inputs |
| `WOK_HAL_Geo_Deploy()` | Command deploy (extend arm). Non-blocking. |
| `WOK_HAL_Geo_Stow()` | Command stow (retract arm). Non-blocking. |
| `WOK_HAL_Geo_EmergencyStop()` | Kill actuator immediately |
| `WOK_HAL_Geo_GetState(*state)` | 0=STOWED, 1=DEPLOYING, 2=DEPLOYED, 3=STOWING |
| `WOK_HAL_Geo_GetEndstops(*deployed, *stowed)` | Read endstop switch states |
| `WOK_HAL_Geo_GetMotorCurrent(*amps)` | Read actuator current |
| `WOK_HAL_Geo_PlacePod(pod, x, y)` | Place pod `pod` at (x,y) [start frame]. Non-blocking; poll `GetPodState` (CP-3) |
| `WOK_HAL_Geo_ReseatPod(pod)` | Lift + re-press a placed pod (fresh seat attempt). Non-blocking |
| `WOK_HAL_Geo_RetrievePod(pod)` | Recapture + stow a placed pod; POD_STOWED again = recapture confirmed |
| `WOK_HAL_Geo_GetPodState(pod, *state)` | 0=STOWED(carried), 1=PLACING, 2=PLACED, 3=RETRIEVING |
| `WOK_HAL_Geo_GetPodSeating(pod, *force_N, *attempt)` | Seating-force channel: peak arm force of the latest seat + attempt counter |

**Typical hardware:** Legacy array: linear actuator or lead screw via H-bridge PWM, limit switches via GPIO, current sense ADC. Per-pod placement (the flight baseline): a placement arm or dispenser with per-pod grippers/latches; the seating-force channel is the arm's press-force sensor (load cell or motor-current-derived) — the coupling-quality PROXY the GEO app gates on (`WOK_GEO_SEAT_FORCE_MIN_N`), with the seismic test shot as arbiter.

**Implementation notes:**

- `Deploy` and `Stow` are non-blocking — start the motion and return. The app polls `GetState` and `GetEndstops`.
- `GetState` should update based on endstop readings and current flow. If current is flowing but no endstop is hit, the mechanism is in motion. If current is high and no endstop after timeout, it's a jam.
- The sim HAL uses a 3-second timer to simulate transition time; per-pod motions are 4 s (place/retrieve) and 3 s (reseat), with seating forces drawn from the coupling truth model.
- The placement position is passed IN (`PlacePod(pod, x, y)`) because placement pose is NAV knowledge, not mechanism knowledge; the driver only needs it if the arm has its own fine positioning.
- `EmergencyStop` must also halt pod motions and leave pod state readable/honest (an interrupted fresh place retracts with the pod; an interrupted reseat or retrieval leaves the pod on the ground).

### Seismic Acquisition + EM Source

| Function | Description |
|----------|-------------|
| `WOK_HAL_Seis_Init()` | Initialize ADC/digitizer |
| `WOK_HAL_Seis_Configure(fs, gain, ch_mask)` | Set sample rate, gain, active channels |
| `WOK_HAL_Seis_SetSite(x, y, hdg)` | Site context stamp for data products (from NAV pose) |
| `WOK_HAL_Seis_SetPodGeometry(pod, x, y, attempt)` | Actual placed pod position + final seat attempt (CP-3 placement missions; the sim anchors coupling truth and per-channel moveout with it — a flight driver may no-op) |
| `WOK_HAL_Seis_StartAcquisition(dur)` | Begin recording for `dur` seconds |
| `WOK_HAL_Seis_StopAcquisition()` | Abort recording |
| `WOK_HAL_Seis_GetSamples(*buf, max, *actual)` | Drain buffered samples (interleaved; successive calls return successive samples) |
| `WOK_HAL_Seis_GetStatus(*acquiring, *samples)` | Check if still recording, sample count |
| `WOK_HAL_Seis_GetStrikeInfo(*sample_idx, *valid)` | Shot t₀ as a per-channel sample index within the current window (impulse discharge instant, or sweep start) |
| `WOK_HAL_Seis_GetQuicklook(*rms, *peak, *dc, nch)` | Compute quicklook stats per channel |
| `WOK_HAL_Seis_Evaluate(*flag, *confidence)` | Evaluate for ice signature (flag + 0.0-1.0) |
| `WOK_HAL_Src_Init()` | Initialize the source (bank charger, discharge switch, driver). Bank must come up safe/dumped. |
| `WOK_HAL_Src_Configure(mode, energy_j, f0, f1, dur_s)` | Configure the next shot: IMPULSE(0) uses `energy_j` as the bank charge target; SWEEP(1) uses the pilot band/duration. Rejected unless IDLE/FIRED. |
| `WOK_HAL_Src_Arm()` | Begin charging the bank to the shot target. Non-blocking; poll `GetState` for ARMED. |
| `WOK_HAL_Src_Fire()` | Deliver the shot: cap discharge (impulse, →FIRED) or pilot drive start (sweep, →SWEEPING→FIRED). Latches t₀. |
| `WOK_HAL_Src_Reset()` | FIRED → IDLE after a completed shot |
| `WOK_HAL_Src_Abort()` | Safe from ANY state: stop the drive, dump the bank through the bleed resistor, → IDLE. Must be immediate. |
| `WOK_HAL_Src_GetState(*state)` | 0=IDLE, 1=CHARGING, 2=ARMED, 3=FIRED, 4=SWEEPING |
| `WOK_HAL_Src_GetBank(*volts, *soc_pct)` | Bank voltage + charge as % of the shot target |
| `WOK_HAL_Src_GetShotInfo(*force_N, *timestamp_ms)` | Last shot's peak plate force + fire timestamp (ADC clock) |
| `WOK_HAL_Src_GetSweepProgress(*frac)` | 0.0–1.0 while sweeping |
| `WOK_HAL_Src_ReseatPlate()` | Lift + re-press the source coupling plate (CP-2; commanded when the plate transfer metric drops below family). Rejected while charging/armed/sweeping |

**Typical hardware:** 24-bit sigma-delta ADC (ADS1256, ADS131M04) via SPI, optional DMA, GPS/PPS timestamping. Source: capacitor bank (film/electrolytic, hundreds of volts) charged through a current-limited boost converter, IGBT/SCR discharge switch into a flat-coil EM actuator over a ground coupling plate; for sweep mode a PWM/linear driver modulates the coil with the pilot waveform; bleed resistor + dump relay for abort.

**Implementation notes:**

- `StartAcquisition` is non-blocking. Duration tracked internally.
- **t₀ is electrical and the source drive and ADC must share one clock.** `GetStrikeInfo` returns the fire instant (impulse discharge, or the first pilot sample of a sweep) as a sample index in the active acquisition — the t₀ reference every first-break pick is measured against. Latch the ADC's sample counter with the discharge-switch gate edge / drive-start strobe in hardware; a software timestamp taken in the fire handler is NOT adequate at 4 kHz. There is no mechanical release detection anymore.
- **Wireless pods (the flight baseline) void the shared-clock assumption** — this is the known porting-time contract evolution: `GetStrikeInfo` grows per-channel strike indices plus per-pod sync-quality/clock-error fields, and the picker consumes per-pod t₀. Budget guidance from the sim (CP-5): at a ±0.2 ms per-pod clock bias the pick chain stays within its ±0.5 ms spec; at ±1.0 ms it breaks. Size the RF-sync pulse / clock-discipline design to ~±0.25 ms or better. The test shot's sync/pick-consistency residuals (coup record) are the on-vehicle health check.
- `Src_Arm`/`Src_Fire` enforce state transitions (IDLE→CHARGING→ARMED→FIRE). Charging draws from the main battery bus; the charger should inhibit (not brown out the bus) below `WOK_SRC_MIN_CHARGE_BATT_V` — the app-level arm timeout covers a bank that cannot reach target.
- **The sweep pilot is part of the contract**: `WOK_HAL_Src_PilotSample()` (inline in `wok_hal_seis.h`) defines the exact waveform the driver must emit — a linear sweep with heavy Tukey tapers. The SEIS app regenerates it analytically for onboard correlation, so any deviation in the emitted pilot degrades every sweep-mode pick. Build the flight driver's waveform table by sampling that function at the drive rate.
- `Evaluate` runs analysis on the most recent recorded data. In the sim it derives deterministically from the shared geology model at the configured site; a flight implementation should derive it from the recorded traces (pick-based apparent velocity) rather than porting the sim logic.

### Drill

| Function | Description |
|----------|-------------|
| `WOK_HAL_Drill_Init()` | Initialize drill arm, motor, thermal subsystem |
| `WOK_HAL_Drill_SetSite(x, y)` | Site context stamp for the drill cycle's data products |
| `WOK_HAL_Drill_DeployArm(angle_deg)` | Deploy arm to target angle. Non-blocking. |
| `WOK_HAL_Drill_StowArm()` | Retract arm. Non-blocking. |
| `WOK_HAL_Drill_StartDrill(depth_m, rpm)` | Begin drilling. Non-blocking. |
| `WOK_HAL_Drill_StopDrill()` | Stop drill motor |
| `WOK_HAL_Drill_StartSample(heat_sec, cool_sec)` | Begin thermal vaporization cycle |
| `WOK_HAL_Drill_StopSample()` | Abort sampling |
| `WOK_HAL_Drill_EmergencyStop()` | E-stop all drill subsystems |
| `WOK_HAL_Drill_GetArmState(*state)` | 0=STOWED, 1=DEPLOYING, 2=READY, 3=STOWING |
| `WOK_HAL_Drill_GetArmAngle(*deg)` | Read current arm angle |
| `WOK_HAL_Drill_GetArmCurrent(*amps)` | Read arm motor current |
| `WOK_HAL_Drill_GetDrillMotorState(*state)` | 0=IDLE, 1=DRILLING, 2=DONE |
| `WOK_HAL_Drill_GetDepth(*m)` | Read current drill depth |
| `WOK_HAL_Drill_GetRPM(*rpm)` | Read drill RPM |
| `WOK_HAL_Drill_GetTorque(*nm)` | Read drill torque |
| `WOK_HAL_Drill_GetWOB(*n)` | Read weight-on-bit (axial load cell) — with torque + ROP this forms the drilling-response log |
| `WOK_HAL_Drill_GetMotorCurrent(*amps)` | Read drill motor current |
| `WOK_HAL_Drill_GetSampleState(*state)` | 0=IDLE, 1=HEATING, 2=ANALYZING, 3=COOLING, 4=DONE |
| `WOK_HAL_Drill_GetCellTemp(*c)` | Read sample cell temperature |
| `WOK_HAL_Drill_GetVaporPressure(*pa)` | Read vapor pressure |
| `WOK_HAL_Drill_GetVolatileConc(*ppm)` | Read volatile concentration |

**Typical hardware:** Arm: stepper/servo with angle encoder. Drill: brushless DC motor with depth encoder, torque sensor, current sense. Thermal: resistive heater with thermocouple, pressure transducer, gas chromatograph or mass spec.

**Implementation notes:**

- All motion commands are non-blocking. The app polls Get* functions each HK cycle.
- The drill app runs its own fault detection (overcurrent, over-depth, torque limit, thermal overtemp, arm timeout) and transitions to FAULT state autonomously.
- `EmergencyStop` must immediately halt all motors and heaters.

### Power / Thermal

| Function | Description |
|----------|-------------|
| `WOK_HAL_Power_Init()` | Initialize battery monitor, temp sensors |
| `WOK_HAL_Power_GetBatteryV(*v)` | Read battery voltage |
| `WOK_HAL_Power_GetBatterySOC(*pct)` | Read state of charge (percent) |
| `WOK_HAL_Power_GetBoardTemp(*c)` | Read board temperature (Celsius) |
| `WOK_HAL_Power_GetMotorTemp(*c)` | Read motor temperature (Celsius) |
| `WOK_HAL_Power_GetCPULoad(*pct)` | Read CPU load (percent) |

**Typical hardware:** Battery fuel gauge (MAX17048, BQ27441) via I2C, thermistors or digital temp sensors (TMP117, DS18B20) via I2C/1-Wire.

**Implementation notes:**

- Called on every SEND_HK tick (~25 Hz with the stock SCH_LAB table). Should be fast — read from cached register values, don't initiate a new conversion.
- The HK app uses these to detect BATTERY_LOW and TEMP_HIGH faults.

### Storage

| Function | Description |
|----------|-------------|
| `WOK_HAL_Storage_Init()` | Initialize filesystem |
| `WOK_HAL_Storage_WriteFile(name, data, size)` | Write a binary data file |
| `WOK_HAL_Storage_ReadFile(name, buf, max, *size)` | Read a stored product (backs the file-downlink path). Reject path separators — products are flat files. |
| `WOK_HAL_Storage_GetFreeSpace(*bytes)` | Check available storage |

**Typical hardware:** eMMC, SD card, or NOR flash with a filesystem (POSIX file API via OSAL, or fatfs for bare-metal).

### Watchdog

| Function | Description |
|----------|-------------|
| `WOK_HAL_Watchdog_Init(timeout_ms)` | Initialize hardware watchdog |
| `WOK_HAL_Watchdog_Pet()` | Reset watchdog timer |

**Typical hardware:** On-chip watchdog (IWDG on STM32, WDT on Zynq), or external watchdog IC.

## Porting Walkthrough

Paths below assume the cFS tree at `~/wok-cfs-build` (the Build & Install
Guide's convention) — adjust to your install location.

### Step 1: Copy the Stubs

    cp -r ~/wok-cfs-build/apps/wok_common/hal/src/flight_stub/ \
          ~/wok-cfs-build/apps/wok_common/hal/src/my_target/

This gives you `wok_hal_flight_stub.c` and `wok_hal_drill_flight_stub.c` as starting points.

### Step 2: Include Your BSP/Driver Headers

### Step 3: Implement Each Function

Start with Power and Watchdog (easiest), then Geo, Mobility, Seis/Source, and Drill (most complex).

### Step 4: Update CMakeLists

Set `WOK_HAL_BACKEND` to your directory name. The cFS top-level `make prep` does not forward extra `-D` options to the target sub-build, so either run `cmake -DWOK_HAL_BACKEND=my_target .` inside `build/native/default_cpu1` after `make prep`, or change the default in `apps/wok_common/CMakeLists.txt` (and extend its if/elseif with your backend name).

### Step 5: Build and Test

Run `test.all` in the ground console. The same test suite works regardless of HAL target.

## Validation Checklist

For each HAL subsystem, verify:

- [ ] Init returns WOK_HAL_OK
- [ ] All Get functions return valid values after Init
- [ ] Non-blocking commands return immediately
- [ ] State functions update over time
- [ ] Emergency stops take effect within 100ms
- [ ] Error conditions return WOK_HAL_ERROR, not crash
- [ ] All functions are safe to call from cFS app context (no blocking for >100ms)

## Linux Sim HAL Reference

| Subsystem | Simulation Model |
|-----------|-----------------|
| Mobility | Time-based primitives: 0.5 m/s drive, 30 deg/s turn (default). 3 m drive ≈ 6 s; 90° turn ≈ 3 s. Tilt noise 1.5–3.5°. Drive motor current 1.2 A; turn 1.5 A; 0 at rest. Integrates a ground-truth pose with an unsensed per-turn heading bias (`WOK_SIM_TURN_BIAS_DEG`, default 0) — accumulators report the commanded motion, truth via sim-internal `GetTruePose` only. |
| Geo | Timed 3-second transitions (legacy array); per-pod place/reseat/retrieve state machines (4 s / 3 s / 4 s) with seating forces drawn from the coupling truth model. Motor current 1.5A during motion, 0A at rest. |
| Seismic | Synthetic traces from the shared geology model: impulse shots inject a damped-sine wavelet at each pod's first-arrival time; sweep shots inject the delayed contract pilot. Broadband noise ±80 counts. Quicklook from the same waveform. Per-channel amplitudes scale with the coupling truth model (`WOK_SIM_COUPLING=1`); per-pod clock bias shifts arrivals (`WOK_SIM_SYNC_ERR_MS`). |
| Coupling truth | Seat-quality field + deterministic per-attempt pod/plate coupling draws, seating-force proxy, per-pod sync bias (`wok_hal_sim_coupling.c` — pure functions; see Sim Model Spec §10). |
| EM Source | Bank charges at 50 W from the battery model (inhibited below 23.0 V bus — arm then times out); 50 J default charges in ~1 s. Impulse force ∝ √energy (8.5–9.5 N at default). Sweep drive force-limited (0.02 N) over the pilot duration. Fire dumps the bank; stacked shots recharge between strikes. |
| Drill | Full state machine sim: arm deploy/stow (3s), drilling (progresses to target depth), thermal cycle (heat/cool timers), volatile detection. |
| Power | Linear discharge: 25.2V→18V at 0.001V/sec; SOC 92%→0% at 0.01%/sec; board temp 28°C±2°C; CPU load reads `/proc/loadavg` (1-min avg, scaled to 0–100%). |
| Storage | Real writes to /tmp/wok_data/. 1 GB free. |
| Watchdog | No-op. Always OK. |
