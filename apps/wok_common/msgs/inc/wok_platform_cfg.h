/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef WOK_PLATFORM_CFG_H
#define WOK_PLATFORM_CFG_H
/* Mission software version, reported in every app's init and NOOP events */
#define WOK_VERSION_STRING       "5.0.0"
/* SEQ subscribes to 7 MIDs (CMD + SEND_HK + 5 child TLM streams), so it
 * needs a deeper pipe than the other apps. */
#define WOK_SEQ_PIPE_DEPTH       32
#define WOK_NAV_PIPE_DEPTH       16
#define WOK_GEO_PIPE_DEPTH       16
#define WOK_SEIS_PIPE_DEPTH      16
#define WOK_HK_PIPE_DEPTH        16
#define WOK_DRILL_PIPE_DEPTH     16
#define WOK_SEQ_MAX_WAYPOINTS    64
/* THUMP-state wall-clock timeout. Must cover the worst case source
 * cycle: bank charge at max energy (200 J / 50 W = 4 s), plus a full
 * sweep (WOK_SRC_SWEEP_MAX_DUR_SEC), plus command/TLM latency margin.
 * (The drop-weight (pre-5.0) source's hardcoded 10 s only covered its
 * 1 s lift.) */
#define WOK_SEQ_SOURCE_TIMEOUT_SEC   30
/* Ground-recalibration hold (recal-flagged waypoints): how long the
 * sequencer waits in HOLD for a RELEASE before proceeding on its own —
 * an unattended mission must never deadlock on a missed ground pass. */
#define WOK_SEQ_HOLD_TIMEOUT_SEC    120
#define WOK_SEQ_DEFAULT_SETTLE_SEC   10
#define WOK_SEQ_DEFAULT_ACQUIRE_SEC  60
#define WOK_NAV_DEFAULT_MAX_SPEED_MPS    0.5f
#define WOK_NAV_DEFAULT_ARRIVAL_TOL_M    0.3f
#define WOK_NAV_DEFAULT_TILT_LIMIT_DEG  30.0f
/* Sim-only: unsensed heading bias added per completed turn event
 * (degrees; models drivetrain turn-rate asymmetry). The encoders/IMU
 * report the COMMANDED turn, so dead-reckoned pose diverges from sim
 * truth — the error the ground REALIGN path exists to correct. Default
 * 0 keeps every existing regression byte-identical; the sim also honors
 * a WOK_SIM_TURN_BIAS_DEG environment override at init (runtime-toggle
 * convention, same as WOK_SIM_COUPLING / WOK_SIM_SYNC_ERR_MS). */
#define WOK_SIM_TURN_BIAS_DEG            0.0f
#define WOK_GEO_DEFAULT_CURRENT_LIMIT_A  3.0f
#define WOK_GEO_DEFAULT_DEPLOY_TIMEOUT_MS 5000
#define WOK_GEO_DEFAULT_STOW_TIMEOUT_MS   5000
/* Per-pod placement (CP-3, WOK-CP-001): the seating-force
 * proxy gate and reseat budget. A seat whose arm force reads below the
 * threshold is re-seated up to MAX_RESEAT times; exhaustion places the
 * pod MARGINAL (flagged, not fatal — the CP-4 test shot arbitrates).
 * 26 N sits in the good/bad overlap of the sim's force model: it
 * catches most bad seats with a small false-accept tail (harness-
 * measured ~6%), which is the physically honest behavior of a proxy. */
#define WOK_GEO_MAX_RESEAT            2
#define WOK_GEO_SEAT_FORCE_MIN_N      26.0f
#define WOK_GEO_POD_TIMEOUT_MS        12000   /* place/reseat/retrieve   */
#define WOK_SEIS_MAX_CHANNELS     4
/* 4 kHz: first-break moveout across the 6 m pod array spans only ~10-20 ms;
 * at 1 kHz the picks quantize to +-1 ms and apparent Vp is unusable. */
#define WOK_SEIS_DEFAULT_FS_HZ   4000
#define WOK_SEIS_DEFAULT_GAIN     1
#define WOK_SEIS_DEFAULT_DURATION_SEC 6
/* Geophone pod offsets — inline array along the deploy boom, metres from
 * the source strike point. Fixed deployment geometry (single mechanism);
 * used for moveout in the sim and recorded in every trace file header. */
#define WOK_SEIS_POD_OFFSETS_M_INIT  { 1.5f, 3.0f, 4.5f, 6.0f }
/* ── EM source (capacitor-bank actuator) ──────────────────────────────
 * IMPULSE: commanded bank energy sets strike amplitude (peak plate force
 * ~8.5 kN at the 50 J default; impulse-momentum scaling, Sim Model Spec
 * §4).
 * SWEEP: linear pilot f0->f1 over the duration (heavy 50% Tukey tapers,
 * WOK_HAL_SRC_PILOT_TAPER_FRAC — see WOK_HAL_Src_PilotSample; the
 * taper/force/gain/lead set is harness-tuned as a unit). The band is wide (near-surface EM sources
 * work to ~2 kHz) because the correlated Klauder wavelet's width is
 * 1/bandwidth: the onset picker needs a compact wavelet to pick within
 * tolerance of an impulse record. f1 must stay below 0.45*fs. */
#define WOK_SRC_DEFAULT_MODE          0       /* IMPULSE                */
#define WOK_SRC_DEFAULT_ENERGY_J     50.0f
#define WOK_SRC_MAX_ENERGY_J        200.0f
#define WOK_SRC_DEFAULT_SWEEP_F0_HZ 100.0f
#define WOK_SRC_DEFAULT_SWEEP_F1_HZ 1600.0f
#define WOK_SRC_DEFAULT_SWEEP_DUR_S   3.0f
#define WOK_SRC_SWEEP_MAX_DUR_SEC     4       /* bounds the record window */
/* Bank charging is inhibited below this bus voltage (charger brownout
 * guard). Above the 22.0 V BATTERY_LOW SAFE threshold on purpose: a sag
 * shows up as an arm timeout (recording proceeds without a source)
 * before it becomes a mission-level fault. */
#define WOK_SRC_MIN_CHARGE_BATT_V    23.0f
/* Per-strike trace buffer: samples retained per channel. The sweep
 * record window is sweep duration + listen time, so the buffer covers
 * (WOK_SRC_SWEEP_MAX_DUR_SEC + 6 s listen) @ 4 kHz with margin.
 * MEMORY COST: 40960 x 4 ch x 4 B = 640 KB (was 384 KB at 24576);
 * the SEIS app also gains a 128 KB correlated-trace buffer and a 64 KB
 * pilot buffer — +448 KB total SEIS app footprint vs the drop-weight
 * (pre-5.0) source. Longer
 * manual acquisitions keep the first 40960 samples/chan (truncation is
 * evented). */
#define WOK_SEIS_TRACE_MAX_SAMPS_PER_CH  40960
/* Onboard sweep correlation, chunked so it never blocks the SEIS main
 * loop for more than one HK tick: output samples/chan computed per tick.
 * 1000 samps x 16k-tap pilot x 4 ch ≈ 64M MACs/tick (~ms on the sim
 * host; size to <1 tick on the flight target). Full 8000-sample window
 * completes in 8 ticks. */
#define WOK_SEIS_CORR_SAMPS_PER_TICK  1000
/* Zero-phase -> causal alignment for the correlated trace: the Klauder
 * wavelet leads its center by ~1/bandwidth, which an onset picker reads
 * as an early arrival. The correlator delays its output by
 * WOK_SEIS_CORR_LEAD_FRAC / (f1 - f0) seconds so equivalent-impulse
 * onsets align with true impulse onsets. Harness-tuned with the pilot
 * taper and the sim sweep drive force: impulse-vs-sweep pick deltas
 * within +-0.25 ms at every pod across 8 noise seeds — do not change
 * one without re-tuning the set. */
#define WOK_SEIS_CORR_LEAD_FRAC       2.5f
/* Correlation output gain: c = (sum r*p / Ep) * GAIN. Without it the
 * correlated noise floor (~1 count) quantizes away in the int32 trace
 * and the picker's 4x-noise onset refinement loses its reference. */
#define WOK_SEIS_CORR_GAIN           32.0f
/* Stacking: strikes summed per site, aligned on each record's strike t0.
 * Stack window is [-PRE, +POST] around t0. */
#define WOK_SEIS_STACK_TARGET         3
#define WOK_SEIS_STACK_PRE_SAMPS      800    /* 200 ms @ 4 kHz  */
#define WOK_SEIS_STACK_POST_SAMPS     7200   /* 1800 ms @ 4 kHz */
#define WOK_SEIS_STACK_WIN_SAMPS      (WOK_SEIS_STACK_PRE_SAMPS + WOK_SEIS_STACK_POST_SAMPS)
/* Onboard evaluate: two-layer refraction grid inversion over the picks.
 * Refractor Vp above the ice threshold = ice-cemented signature; the
 * confidence is the fraction of near-optimal (v2, h1) cells that are
 * ice-fast, so an underdetermined ridge honestly reads as uncertainty. */
#define WOK_SEIS_EVAL_V2_ICE_MPS   1200.0f  /* ice-cemented refractor    */
#define WOK_SEIS_EVAL_ICE_FRAC_MIN    0.30f /* flag site when P(ice) >=  */
#define WOK_SEIS_EVAL_V2_MIN_MPS    300.0f  /* grid: refractor Vp range  */
#define WOK_SEIS_EVAL_V2_MAX_MPS   4050.0f
#define WOK_SEIS_EVAL_V2_STEP_MPS    75.0f
#define WOK_SEIS_EVAL_H1_MIN_M        0.5f  /* grid: layer-1 thickness   */
#define WOK_SEIS_EVAL_H1_MAX_M        3.0f
#define WOK_SEIS_EVAL_H1_STEP_M       0.1f
#define WOK_SEIS_EVAL_PICK_SIGMA_MS   0.35f /* per-pick timing tolerance */
/* Plate coupling metric (WOK-CP-001): per impulse shot, the ratio
 * (near-pod signal RMS over the post-t0 window) / (plate peak force),
 * tracked against a running site-to-site EMA baseline. The ratio is
 * force-normalized, so commanded-energy changes cancel; only ground
 * transfer moves it. A shot below FRAC x baseline fails verification
 * and triggers the sequencer's plate-reseat branch, bounded per site;
 * on exhaustion the site proceeds with a degraded flag. Sweep shots are
 * not evaluated (per-shot arrivals sit at the noise floor pre-correlation). */
#define WOK_SEIS_PLATE_WIN_SAMPS     400    /* 100 ms @ 4 kHz after t0: the
                                               far-pod arrival (<= 25 ms)
                                               plus ~4 decay constants of
                                               the 12 ms impulse coda     */
/* Per-pod coupling score (CP-4, WOK-CP-001): from a low-energy
 * test shot, each pod's post-t0 signal RMS is normalized by geometric
 * spreading (x^2+1, the sim/flight amplitude model) and ratioed to the
 * array median. The plate factor is common-mode and cancels in the
 * ratio — this is what disambiguates pod-side from plate-side coupling
 * (the CP-2 metric conflates them). Spectral-coherence term deferred
 * until the coupling model has spectral structure (bench item). */
#define WOK_SEIS_POD_SCORE_MIN       0.70f  /* verified at/above this     */
/* Sequencer verification choreography (CP-4): the low-energy test shot
 * and the targeted-reseat budget. The test energy rides the bank model
 * (10 J charges in 0.2 s); the partial-array draft policy: proceed when >= MIN_VERIFIED pods score OK, flag
 * the site; below the minimum, proceed FLAGGED with a CRITICAL event
 * — never silently sacrifice the far pod, never stall the mission. */
/* Sync/pick-consistency threshold (CP-5): a pod whose
 * test-shot pick deviates from the array's best-fit moveout by more
 * than the pick spec is sync-suspect (or badly picked — a known-t0
 * event observed at every pod checks both). Reported, not acted on:
 * no mechanism fixes a clock; ground deweights and investigates. */
#define WOK_SEIS_SYNC_RESID_MAX_MS   0.5f
#define WOK_SEQ_TEST_ENERGY_J        10.0f
#define WOK_SEQ_TEST_DUR_SEC         3
#define WOK_SEQ_VERIFY_RESEAT_BUDGET 2      /* targeted reseats per site  */
#define WOK_SEQ_MIN_VERIFIED_PODS    3
#define WOK_SEIS_PLATE_COUP_FRAC     0.70f  /* fail below FRAC x baseline */
#define WOK_SEIS_PLATE_EMA_ALPHA     0.25f  /* baseline update, accepted shots */
#define WOK_SEIS_MAX_PLATE_RESEAT    2      /* reseat attempts per site   */
/* QC thresholds — assume a 24-bit signed ADC (±2^23 ≈ 8.39M counts FS).
 * Tune to actual ADC range when porting. */
#define WOK_SEIS_QC_CLIP_THRESHOLD   7950000.0f  /* peak > this counts → clipping (95% FS) */
#define WOK_SEIS_QC_DEAD_THRESHOLD        50.0f  /* RMS below this counts → dead channel */
#define WOK_HK_DEFAULT_BATT_MIN_V   22.0f
#define WOK_HK_DEFAULT_TEMP_MAX_C   55.0f
#define WOK_HK_DEFAULT_TILT_MAX_DEG 30.0f
#define WOK_DATA_DIR             "/cf/wok_data"
#define WOK_SEIS_FILE_MAGIC      0x53454953
#endif
