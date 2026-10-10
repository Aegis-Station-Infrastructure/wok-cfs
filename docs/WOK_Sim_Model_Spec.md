# WOK Simulation Model Specification

**Document:** WOK-SM-001  
**Version:** 1.3 (FSW 5.0)  
**Date:** October 2026  
**Scope:** `apps/wok_common/hal/src/linux_sim/` — every function and equation
the Linux sim uses to synthesize the signals the flight software would
receive from bench or flight hardware. This is the reference for (a) knowing
exactly what physics the FSW has been validated against, and (b) replacing
each model term with a measured value when bench data exists.

Each section ends with a **Bench calibration** note: what to measure on real
hardware to validate or replace that model term. Constants live in the cited
source file unless marked as platform config (`wok_platform_cfg.h`).

---

## 1. Conventions

- **Frames:** all site coordinates (x, y) are the mission start frame
  (x = forward at SEQ START, y = left), the same frame NAV dead-reckons in.
- **Timebases:** three clocks are used, deliberately mirroring real hardware:
  - `CLOCK_MONOTONIC` (ms) — shared by the ADC sample counter and the EM
    source (`wok_hal_sim_seis.c`). This is the sim's stand-in for the
    *hardware-shared clock* that the flight design requires for electrical t₀.
  - `time(NULL)` (1 s resolution) — mechanism timers (geo arm, drill arm,
    thermal cycle). Adequate because those transitions are seconds long.
    (The drill's depth/ROP integration is the exception: it runs on
    `CLOCK_MONOTONIC` ms.)
  - `gettimeofday` (µs) — mobility kinematic integration.
- **Noise:** all sensor noise is `rand()`-based **uniform**,
  `U(−a, +a) = ((rand()/RAND_MAX) − 0.5) · 2a`, seeded from wall time at app
  init. It is *not* Gaussian and has no spectral shaping — see §11.
- **Determinism:** every geology-derived quantity is a pure function of
  (x, y, depth), so all app-local HAL copies agree without shared memory.
  Randomness rides *on top of* deterministic truth, never inside it.
- **Thermal ambients are Earth-bench values** (25 °C cell, 28/35 °C
  battery), not lunar. The sim is a desktop stand-in for a bench test.
- **"ppm" is by mass of sample** (ppmw: µg of H₂O-equivalent per g of
  regolith; 1 wt% = 10⁴ ppmw), never a gas mixing ratio.

---

## 2. Shared subsurface geology model — the single source of truth

`wok_hal_sim_geology.c` — pure functions; both instruments derive from it,
so a correct seismic inversion must agree with what the drill finds.

**Layer column at (x, y):**

| Layer | Depth span | Vp | Ice (wt%) | Strength |
|---|---|---|---|---|
| L1 dry regolith | 0 → h₁(x,y) | 250 m/s | 0 | 1.0 |
| L2 (ice-bearing) | h₁ → 12 m | 800 + 350·w(x,y) m/s | w(x,y) | 1 + 0.5·w(x,y) |
| L3 basalt/megaregolith | > 12 m | 3200 m/s | 0 | 4.0 |

**Interface undulation:**
```
h₁(x,y) = 1.5 + 0.5·sin(0.4x)·cos(0.3y)        [m]
```

**Ice lens (lateral distribution, wt% water-equivalent):**
```
w(x,y) = 0.003 + 5.0·exp(−r²/(2σ²)),  r² = (x−6)² + (y−3)²,  σ = 2.5 m
```
Peak 5 wt% at (6, 3); 0.003 wt% trace adsorbed hydration everywhere.
Depth gating for point queries: above max(h₁, 1.2 m) (thermal
stability depth) or below 12 m the query returns the 0.003 wt%
background film, not zero — an out-of-gate evolved-gas measurement
reads ~30 ppm.

**First-arrival traveltime** for a surface source and receiver at offset x
(what the seismic sim delays every wavelet by):
```
t_direct = x / v₁
t_head   = x / v₂ + 2·h₁·cos(i_c)/v₁,   sin(i_c) = v₁/v₂   (only if v₂ > v₁)
t_arr    = min(t_direct, t_head)
```
Two-layer refraction only: no L3 arrival branch, no reflections, no
S-waves, no attenuation (Q = ∞), no dispersion.

**Volatile release spectrum** (binding classification ground truth).
Cumulative evolved fraction at cell temperature T is a sum of logistic
CDFs approximating Gaussian release peaks:
```
cdf(T; µ, σ) = 1 / (1 + exp(−(T − µ)/(0.6σ)))

evolved(T)/total = f_pore·cdf(T; 100, 20) + f_ads·cdf(T; 160, 30)
                 + f_hyd·cdf(T; 320, 30)
```
The modes are ordered by binding energy, as in a thermogravimetric /
evolved-gas sequence: free (pore) ice evaporates first, around the boiling
point at cell pressure; physisorbed films desorb above that; structurally
bound water (hydrated minerals) needs 300 °C and more. Partition: adsorbed
component = min(total, 0.003 wt%) (the background film); the lens-derived
remainder splits 85% pore ice / 15% hydrated mineral. The hydrated peak
(320 °C) sits above the 300 °C cell limit by design — the vapor cell
under-recovers it (~10–11%), and ground processing must recover the hidden
tail from the release-curve fit. The sim cell is sealed and starts at
bench ambient (25 °C), so nothing evolves before the ramp; a vented flight
cell would lose free ice below 0 °C during warm-up, so cold-loading and
sealing before the ramp is a flight requirement (§11).

**Velocities are deliberately stiffer than Apollo.** The Apollo 14/16
active seismic experiments found ~100 m/s regolith over ~300 m/s to about
8 m; Apollo 17 LSPE found ~250 m/s to >100 m over ~1.2 km/s basalt. The
sim's 250 / 800 / 3200 m/s column is chosen so the direct and head-wave
branches separate cleanly on a 6 m spread; a polar site is unsurveyed.
Replace with site data.

**Bench calibration:** replace the layer table and lens with site survey /
lab core data; replace release-mode (µ, σ) with TGA/EGA lab calibration of
actual regolith simulant. The logistic-CDF *shape* is the modeling choice
to preserve.

---

## 3. Seismic acquisition (ADC) and trace synthesis

`wok_hal_sim_seis.c`. Channels are geophone pods at fixed inline offsets
`{1.5, 3.0, 4.5, 6.0}` m from the source plate (platform config).

**Sample availability (the "ADC"):** samples/channel available at time t
after StartAcquisition is `floor((t·fs))`, capped at `dur·fs`; acquisition
self-stops at the cap. `GetSamples` synthesizes on drain (up to 1000
samples/chan per call), interleaved by active channel. Values are float-
computed then truncated to int32 "ADC counts" (24-bit signed range assumed
by the QC thresholds: clip at ±7,950,000 ≈ 95% FS).

**Per-sample synthesis** for pod ch at absolute sample index s:
```
val = n(s) + A_ch · g(t − t₀ − t_arr(ch))          for t ≥ t₀ + t_arr, else n(s)

n(s)  = U(−80, +80) counts                          (broadband noise floor)
A_ch  = F · 40 / (x_ch² + 1)   counts               (source coupling + spreading)
```
- `F` = shot peak plate force [N] (§4), `x_ch` = pod offset [m].
- `40 counts/N at 1 m` is the lumped plate-to-ground coupling ×
  geophone sensitivity × ADC gain constant (an 8.5 kN strike gives
  ~105 k counts at the 1.5 m pod, ~9 k counts at 6 m, on a 24-bit ADC
  with an 80-count noise floor).
- Geometric spreading `1/(x²+1)`: body-wave-like 1/x² softened to avoid the
  singularity at zero offset. No per-branch (direct vs head wave)
  amplitude difference is modeled.

**Impulse response wavelet** (mode = IMPULSE):
```
g(dt) = exp(−dt/τ) · sin(2π·f_c·dt),   f_c = 400 Hz,  τ = 12 ms
```
A causal damped sine — the minimum-phase stand-in for source-plate +
shallow-ground response. 400 Hz is a ~0.6 m wavelength at v₁ = 250 m/s,
short against the 1.5 m layer and the 1.5–6 m spread, and sits inside the
100–1600 Hz sweep band (§5); the coda is ~4τ ≈ 50 ms. The onset is a
mathematical step at t_arr, so first-break picks against this wavelet are
exact by construction (§11).

**Sweep response** (mode = SWEEP): ground motion is the *contract pilot*
delayed by the arrival time (i.e., the pilot convolved with a single-spike
arrival series — the same t_arr(ch) as impulse mode):
```
g(dt) = p(dt)                       (see §5 for p — the pilot definition)
```
No earth filtering of the pilot is applied (the arrival series is a delta,
not the damped sine): the correlation chain is validated against pure
time-delay physics.

**Electrical t₀ latch:** `Src_Fire()` reads the shared monotonic clock and
latches `strike_sample_idx = (t_fire − t_acq_start)·fs`, valid only while
acquiring. This is the sim's model of the flight requirement that the
discharge-gate edge latch the ADC sample counter in hardware.

**Quicklook:** RMS/peak/DC per channel over the most recent 256 available
samples of the same synthetic waveform (the FSW app computes its own from
the real drained trace; the HAL quicklook is a monitoring stand-in).

**Bench calibration:** measure counts/N transfer at 1 m (plate force cell →
geophone → ADC), actual spreading exponent on the bench floor / analog
site, real wavelet shape (replace damped sine), noise floor amplitude *and
spectrum*, and ADC clip level. The t₀ latch becomes a hardware capture —
verify latch-to-sample alignment ≤ 1 sample at 4 kHz.

---

## 4. EM source (capacitor bank + actuator)

`wok_hal_sim_seis.c` (source half). States: IDLE → CHARGING → ARMED →
FIRED (impulse) or → SWEEPING → FIRED (sweep).

**Bank energy bookkeeping:**
```
target_J  = E_cmd                      (impulse; default 50 J, max 200 J)
target_J  = P_sweep · T_sweep          (sweep; P_sweep = 10 W)

charge:   dE/dt = 50 W   while  V_batt ≥ 23.0 V, else 0   (charge inhibit)
fire:     E → 0.02·E                   (discharge / drive spends the bank)
abort:    E → 0                        (bleed dump, any state)
```
Every joule charged is drained from the local battery model
(`WOK_HAL_SimPower_Drain_J`, §7). The 23.0 V inhibit sits deliberately
*above* the 22.0 V mission SAFE threshold so a sagging bus manifests first
as a sequencer arm timeout, not a SAFE entry.

**Bank voltage telemetry** (ideal capacitor):
```
V_bank = sqrt(2E/C),   C = 2.5 mF     (→ 200 V at 50 J, 400 V at 200 J)
SOC%   = 100 · E / target_J
```

**Impulse plate force** (discharge → strike). Impulse-momentum scaling:
peak force ∝ √(stored energy). A 50 J strike delivered over ~1 ms is a
~7 N·s impulse and a kN-class peak, as for any hammer-and-plate source:
```
F = 8500 · sqrt(E/50 J) + U(0, 1000)   [N]     (8.5–9.5 kN at the 50 J default)
```
**Charging time** at default energy = 50 J / 50 W = 1.0 s.

**Sweep drive:** regulated (force-limited) at `F = 20 N` for the pilot
duration; `sweep_progress = elapsed/T_sweep`; FIRED when complete. The
20 N value is a *tuned* constant (see §5) chosen so per-shot arrivals sit
at the noise floor (raw SNR ≈ 3 at the near pod) and only emerge in
correlation — both physically representative of small force-limited
vibrators and required for onset-picker compatibility of the correlated
wavelet.

**Bench calibration:** charge power curve vs bus voltage (replace the
step inhibit with the real charger foldback), C and residual-after-fire,
measured F(E) for the actuator/plate (replace the √E anchor), regulated
sweep force, and the actual bank-to-battery efficiency.

---

## 5. Sweep pilot and onboard correlation (the sweep signal chain)

The pilot is **part of the HAL contract** (`WOK_HAL_Src_PilotSample`,
inline in `wok_hal_seis.h`) — the sim's synthesis, the FSW's correlation
reference, and a flight driver's waveform table must all be this function:
```
p(t) = w(t) · sin(2π·(f₀t + ½k t²)),   k = (f₁ − f₀)/T,   0 ≤ t < T

w(t): cosine (Tukey) taper, ramp fraction 0.5 of T at EACH end
      w = ½(1 − cos(πt/ramp)) rising;  mirrored falling
```
Defaults: f₀ = 100 Hz, f₁ = 1600 Hz, T = 3.0 s (max 4 s); f₁ must stay
< 0.45·fs. The heavy taper is deliberate: it shapes the correlation
(Klauder) wavelet's envelope into a smooth monotonic rise with no side-lobe
shelf, which is what lets the unchanged STA/LTA **onset** picker read the
correlated trace consistently with an impulse record.

**Onboard correlation** (FSW, `wok_seis_app.c` — included here because it
completes the signal chain the sim feeds). Over the stack window
[t₀ − PRE, t₀ + POST), output sample j:
```
c[j] = (G/E_p) · Σₙ r[t₀ − PRE + j − L + n] · p[n/fs]

E_p = Σ p²                          (pilot energy normalizer)
G   = 32                            (WOK_SEIS_CORR_GAIN: keeps the
                                     correlated noise floor above int32
                                     quantization so the picker's 4×noise
                                     onset refinement has a reference)
L   = round(2.5/(f₁ − f₀) · fs)     (WOK_SEIS_CORR_LEAD_FRAC: delays the
                                     zero-phase Klauder wavelet so its
                                     energy onset reads causal — aligned
                                     with impulse onsets — to the picker)
```
The `(taper = 0.5, F_sweep = 20 N, G = 32, lead = 2.5)` set is
**harness-tuned as a unit** (`tools/wok_src_harness.c`): impulse-vs-sweep
pick agreement within the ±0.5 ms pick spec at every pod across 8 noise
seeds (typically one 0.25 ms sample). Changing any one requires re-running
the harness.

**Bench calibration:** capture the emitted pilot with a reference
accelerometer on the plate and cross-correlate against the analytic pilot —
phase/amplitude deviation directly degrades sweep picks. Re-tune the lead
constant against bench impulse-vs-sweep shot pairs.

---

## 6. Drill (penetration, drilling-response log, thermal volatile analysis)

`wok_hal_sim_drill.c`. All material terms come from geology strength/ice
at (site, current bit depth) — the drill "measures" the same ground truth
the seismic sim radiates.

**Arm:** linear 3.0 s angle ramps (deploy/stow); current 2.0 A moving,
0.1 A holding, 0 stowed; ±0.05 A noise.

**Penetration (ROP)** — incremental integration each poll:
```
d(depth)/dt = 0.04 / S(x, y, depth)   [m/s],   S = geology strength (≥ 1)
```
(0.04 m/s dry-regolith base rate is sim-compressed ~100× for demo pacing —
planetary drills advance centimetres per minute; strength contrast, not
absolute rate, is the fidelity claim.)

**Drilling-response channels while drilling:**
```
torque  = 0.5·S + U(−0.1, 0.1) + 0.6·[5% chance]   [N·m]  (bit-bind spikes)
WOB     = 100 + 20·S + U(−5, 5)                     [N]
current = 1.5 · torque + U(−0.02, 0.02)             [A]
RPM     = commanded + U(−5, 5)
```
Ground processing uses a *median* torque baseline specifically because the
5% bit-bind spikes inflate a mean (this is modeled adversarial realism).

**Thermal vaporization cycle:** heat 5 °C/s from 25 °C to 300 °C → hold
10 s ("ANALYZE") → cool 3 °C/s to ambient. Cell temp telemetry ±0.5 °C.
The heat/cool durations carried by START_SAMPLE are recorded but not used
by the sim; these rates govern.

**Evolved-gas measurement** during heat/hold (the binding-classification
signal): with `total_ppm = 10000 · w(x, y, depth_latched)` (1 wt% = 10⁴ ppm,
depth latched at StartSample for depth registration):
```
ppm(T)  = total_ppm · evolved(T)/total          (§2 release spectrum)
          + U(−(0.02·ppm + 2), +(0.02·ppm + 2))          (2% + 2 ppm noise)
P_vapor = 3.6·(T − 25) + 0.01·ppm + U(−2, 2)    [Pa]   (+U(−5, 5) during the hold)
```
The logged (T, ppm) ramp *is* the release curve; at the 300 °C limit the
hydrated mode is only partially evolved (intentional under-recovery).

**Bench calibration:** ROP/torque/WOB vs simulant strength (replace linear
S-scalings with measured curves), motor K_t for the current model, real
heater ramp/loss profile, vapor-cell pressure model, EGA calibration of
ppm-vs-T for known simulant mixes.

---

## 7. Power / battery

`wok_hal_sim_power.c`. Linear-in-time discharge plus consumer sag:
```
V_batt(t) = 25.2 − 0.001·t − 0.05·(E_drained/1000)   [V]   (floor 18.0)
SOC(t)    = 92 − 0.01·t                              [%]   (floor 0)
```
- t = seconds since the app's power init; `E_drained` = joules drawn via
  `WOK_HAL_SimPower_Drain_J` (bank charging; 0.05 V/kJ sag).
- The time-linear term makes the battery a pure function of wall time, so
  per-app HAL copies agree; the sag term is local to the calling app's
  copy (only the EM source both drains and reads it). Real-time drain
  means a core idling ~28 min crosses the 23.5 V drill inhibit and one
  idling ~53 min hits the 22.0 V SAFE threshold — restart the core
  between mission runs.
- Board temp = 28 ± 2 °C uniform; motor temp = 35 ± 3 °C uniform;
  CPU% = host 1-min loadavg × 100 (clamped 0–100).
- Test hook: `WOK_HAL_SimPower_SetBattery(V)` re-bases the model (harness
  low-battery cases).

**Bench calibration:** replace with a discharge curve vs integrated load
(Ah counting) and measured bus sag vs charger draw.

---

## 8. Mobility and geophone mechanism

**Mobility** (`wok_hal_sim_mobility.c`) — time-based kinematics, no
dynamics: active primitive advances at constant commanded rate
(`progress += rate·dt`, dt capped at 2 s) until |target| reached, then
latches its per-primitive accumulator. Defaults 0.5 m/s drive, 30 °/s turn
(3 m ≈ 6 s, 90° ≈ 3 s). Drive current 1.2 A, turn 1.5 A, 0 at rest. Tilt =
1.5 + U(0, 2)° (noise only — no terrain model). No reverse drive; no slip,
no encoder error (NAV's SLIP fault is unreachable in sim).

The sim integrates the TRUE pose it drives, including an **unsensed
per-turn heading bias** (`WOK_SIM_TURN_BIAS_DEG`, default 0,
env-overridable): each completed turn lands `bias` degrees off the
commanded angle while the accumulators report the commanded value — so
NAV's dead reckoning diverges from truth exactly the way a drivetrain
turn-rate asymmetry would cause, and the ground REALIGN/SET_POSE path
has a real error to correct. Truth is exposed only through the
sim-internal `WOK_HAL_SimMobility_GetTruePose` (verification; the apps
never read it). The corrective realign turn rides the same biased
drivetrain, leaving a one-turn residual — physically honest.

**Geo mechanism** (`wok_hal_sim_geo.c`) — timed 3 s transitions
DEPLOYING→DEPLOYED / STOWING→STOWED; 1.5 A during motion, 0 at rest;
endstops derive from state. No jam model (GEO's jam detection is
unreachable in sim).

**Per-pod placement (CP-3)** — each pod is a timed state machine:
PLACING (4 s; 3 s for a reseat) → PLACED, RETRIEVING (4 s) → STOWED.
On PLACED the seating force latches from the coupling truth model
(`WOK_SimCoupling_SeatForceN` at the pod's commanded position and seat
attempt; `WOK_SIM_COUPLING` unset → nominal 30 N, every seat firm).
Reseat advances the attempt and redraws. E-stop resolves motions: an
interrupted fresh place retracts with the pod (STOWED), an interrupted
reseat or retrieval leaves the pod on the ground (PLACED). Because the
GEO sim's force draw and the SEIS sim's amplitude draw hash the same
(position, attempt), the seat the arm felt is the seat the traces show
— cross-app consistency by determinism, no shared state.

**Bench calibration:** measured speed/accel profiles, motor currents under
load, real endstop/jam signatures, encoder noise + slip statistics.

---

## 9. Storage and watchdog

Storage: real POSIX files in `/tmp/wok_data` (flight: `/cf/wok_data`);
flat names only (path separators rejected); free space hardcoded 1 GB.
Watchdog: no-op (always OK). Neither models latency, wear, or corruption.

---

## 10. Ground coupling model (pods + source plate) — CP-1

`wok_hal_sim_coupling.c` — pure functions, same purity contract as §2.
The coupling truth for WOK-CP-001: how well each geophone pod and the
source plate transfer ground motion, per surface position and per seating
attempt. Exists so reseat/retry logic and the degraded-array policy are
*rewardable* in sim: a reseat really can improve a bad seat, and a bad
seat really does depress amplitudes.

**Gating:** `WOK_SIM_COUPLING=1` in the environment at app start — each
app's HAL copy reads it independently (SEIS for trace factors, GEO for
seat forces).
Default **OFF**: every factor is exactly 1.0 and trace synthesis is
bit-identical to pre-CP-1 (×1.0f is an IEEE identity) — regression
missions are untouched. The sim stack inherits the caller's environment,
so `WOK_SIM_COUPLING=1 tools/wok_sim.sh up` enables it for a mission.

**Determinism:** "randomness" is a Wang/Jenkins-style integer hash of
(position quantized to cm, attempt, salt) mapped to [0,1) — one
independent stream per drawn quantity. The same attempt at the same spot
always lands the same way; attempt k+1 is an independent draw.

**Seat quality** — the probability one seating attempt lands a good seat:

```
Q(x,y) = clamp( 0.85
              − 0.60·exp(−r₁²/(2·0.8²))     r₁ = dist to patch (4.5, 0.0)
              − 0.75·exp(−r₂²/(2·1.2²))     r₂ = dist to patch (2.0, 5.0)
              + U_hash(−0.08, +0.08)        per 0.25 m cell (clast-scale),
              0.05, 0.95 )
```

Patch 1 sits **on the default survey line**: (4.5, 0) is pod 2 of site
(0,0) and pod 0 of site (3,0), so default-geometry array tests see one
marginal pod. Patch 2 is off-line, for harness statistics.

**Pod coupling** (amplitude transfer factor, drawn per attempt):

```
u₁, u₂ = hash(x_cm, y_cm, attempt, salt₁/salt₂)
C(x,y,a) = 0.90 + 0.10·u₂    if u₁ < Q(x,y)   (good seat)
         = 0.30 + 0.45·u₂    otherwise         (perched / in fluff)
```

**Seating-force proxy** (the arm force-feedback channel, WOK-CP-001):

```
F(x,y,a) = 30 N · (0.5 + 0.5·C) · (1 + U_hash(−0.12, +0.12))
```

Correlated with C (harness measures ≈0.89) but deliberately imperfect:
thresholding F at 26 N leaves a ~6% false-accept tail, which is why the
active test shot (§4.1) stays the arbiter.

**Plate coupling** — same construction, rover-pressed so easier/tighter:
good probability min(Q+0.10, 0.98); good → 0.92–1.00, bad → 0.55–0.80.

**Application** (`wok_hal_sim_seis.c`): factors are drawn at `SetSite`
at each pod's world position (offsets laid along the heading direction)
and the plate's site position. Pods default to seat attempt 0 at the
configured offsets; `WOK_HAL_Seis_SetPodGeometry` (CP-3 placement
missions) overrides a pod with its actual placed position and final
seat attempt — the amplitude factor redraws there, and the channel's
moveout offset becomes the true pod-to-site distance. Overrides persist
until the next call (placement missions refresh them every site). The
**plate seat attempt** (CP-2) persists across
`SetSite` calls at the same pose — a site's stacked shots share one
seating — and resets when the pose changes (driving to a new site
reseats naturally); `WOK_HAL_Src_ReseatPlate` advances it and redraws
the plate factor (rejected while the source is charging/armed/sweeping).
Each channel's **signal** amplitude is multiplied by
`plate_coup · pod_coup[ch]`; noise is electronic/ambient and unaffected;
arrival **times** are unaffected. The reported plate peak force
(`GetShotInfo`) is the actuator-side measurement and is deliberately NOT
scaled — a bad plate seat drops the near-pod response while the force
reads normal, which is exactly what makes the WOK-CP-001 transfer
ratio (near-pod response / plate force) a genuine coupling metric.

**Per-pod sync bias (CP-5, WOK-CP-001):** wireless pods time-stamp
against their own clocks, so each pod's t₀ reference — and therefore its
apparent arrival — is offset by a per-pod bias:

```
ε_p = U_hash(−1, +1)(pod position) × WOK_SIM_SYNC_ERR_MS
```

Gated by `WOK_SIM_SYNC_ERR_MS` (max bias magnitude, ms; unset → 0,
regression untouched). Deterministic per pod position — a pod's clock
error doesn't change between the shots of a site — and injected into
trace synthesis as an arrival-time shift (`t_arr + ε_p`). Bias only; no
per-shot jitter yet (a real RF-sync link adds shot-to-shot jitter that
partially averages in the stack — future refinement). The CP-4 test
shot doubles as the sync check: a known-t₀ event observed at every pod,
whose per-pod residuals against the best-fit moveout flag sync-suspect
(or badly picked) channels.

**Bench calibration** (a future hardware testbed): measured coupling
score vs simulant preparation replaces the good/bad ranges; seating-force
signature vs measured coupling replaces the proxy correlation + jitter;
real spectral effects of poor coupling (this model scales amplitude only
— a real bad seat also low-passes and rings) are a known gap (§11);
measured RF-sync/clock-discipline statistics replace the uniform bias.

---

## 11. Known fidelity gaps (ranked by likely bench impact)

1. **Noise is uniform white**, constant amplitude, uncorrelated across
   channels. Real geophone noise is colored, site-dependent, and partly
   coherent — the picker thresholds (4× RMS) and correlation SNR margins
   were tuned against the white model and must be re-checked on bench data.
2. **Single-arrival earth response**: no coda, reflections, L3 branch,
   S-waves, attenuation, or dispersion; sweep records contain the pure
   delayed pilot. Correlation side-lobe behavior against a reverberant
   earth is untested.
3. **No per-branch amplitudes**: head-wave and direct arrivals share the
   same 1/(x²+1) spreading; true head-wave amplitude decay is steeper.
4. **Bit-level ADC effects** (dither, DC drift, anti-alias filter phase)
   are absent; clip/dead/dropout QC paths are exercised only synthetically.
5. **Mechanism fault signatures** (geo jam, NAV slip) don't occur — those
   FDIR branches are validated only by inspection.
6. **Thermal**: heater ramp is lossless-linear; no sample-mass dependence.
7. **Timing**: sim latches t₀ in software off one process clock (µs-class
   jitter, hidden by the 0.25 ms sample quantum at 4 kHz). Flight must
   latch in hardware; the porting guide makes this a hard requirement.
8. **Coupling is a scalar amplitude factor** (§10): a real bad seat also
   changes the waveform (low-pass, resonant ringing) and couples noise
   differently; spectral-coherence coupling scores will need that
   structure once bench data defines it.
9. **Flat-layer forward model, single-ended spread.** The interface
   undulation (§2) dips up to ~11°, but traveltimes use h₁ at the shot
   point and the flat-layer formula, and there is no reverse shot. In
   the field, v₂ from an unreversed 4-pod spread is an apparent velocity
   and h₁ is known only under the source; the sim does not expose that
   error. Reversed shots (or a second source position) are the standard
   remedy and are a mission-design item, not a sim fix.
10. **First-break picks are exact by construction.** The impulse wavelet
   is a damped sine whose onset is a step at t_arr, and the near-pod SNR
   is ~10³, so the ±0.3 ms pick agreement quoted elsewhere is a
   self-consistency check of the signal chain, not a fidelity claim.
   Real onsets are emergent and band-limited.
11. **Sealed, bench-ambient vapor cell.** The sample starts at 25 °C and
   nothing evolves before the ramp. A vented cell in vacuum would lose
   free ice below 0 °C during warm-up; cold-loading and sealing before
   the ramp is a flight requirement.

---

## 12. Cross-checks that pin the models together

These invariants are what the verification suite actually asserts; any
model change must preserve them:

- Seismic picks at every pod within ±0.5 ms of §2's closed-form t_arr
  (`tools/wok_mission_tools.py validate`).
- Impulse and sweep picks at the same site within ±0.5 ms of each other
  (`tools/wok_src_harness.c` — the `tune` and `hal` modes assert this
  gate; `bank`/`coupling`/`plate`/`pods`/`sync` exercise the other
  models).
- Drill-measured interface (torque step) within ~0.25 m of h₁(x,y); vapor
  totals within ~0.1% of 10⁴·w(x,y,depth) after ground release-curve
  recovery (`ground/wok_refraction.py` vs truth).
- Sequencer mission timing: the 50 J default charge time (1.0 s) is the
  value the sequencer's arm timeout was set against.
- Sync (§10, `wok_src_harness.c sync` mode): measured pick shift under
  injected bias matches ε_p within the 0.25 ms sample quantum; at the
  ±0.2 ms RF-sync budget the worst |pick − truth| stays within the
  ±0.5 ms pick spec.
- Coupling (§10, `wok_src_harness.c coupling` mode): draws deterministic;
  per-channel amplitude ratio coupled/uncoupled equals
  plate·pod coupling within ±0.12 through the real sim HAL; with
  `WOK_SIM_COUPLING` unset, channel RMS follows pure 1/(x²+1) spreading
  (no coupling leak); reseat attempts must raise P(good seat) on bad
  ground; the force proxy stays correlated-but-imperfect.
