/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Seismic Acquisition + EM Source Subsystem (WOK_SEIS)
**
** File: wok_common/hal/inc/wok_hal_seis.h
**
** The seismic source is a capacitor-bank-driven electromagnetic actuator
** striking a coupling plate, controlled by the SEIS app as part of the
** source/record cycle. Two modes, selectable per shot:
**
**   IMPULSE — single cap discharge -> one strike. Commanded energy sets
**             the bank charge level (gravity-independent). Default mode;
**             replaces the drop-weight (pre-5.0) source.
**   SWEEP   — the same actuator driven with a linear swept sine (pilot)
**             for force-limited long-offset work. Recorded traces are
**             cross-correlated onboard against the analytically
**             regenerated pilot (WOK_HAL_Src_PilotSample below) to give
**             an equivalent-impulse trace.
**
** t0 is electrical: FIRE (impulse discharge, or sweep start) latches the
** ADC sample index directly on the shared clock — no mechanical release
** detection. See WOK_HAL_Seis_GetStrikeInfo.
*/

#ifndef WOK_HAL_SEIS_H
#define WOK_HAL_SEIS_H

#include "wok_hal.h"
#include <math.h>

/* ── Source States ───────────────────────────────────────────────── */
/* Values 0-3 keep the drop-weight (pre-5.0) source's meanings
 * (IDLE/ARMING/ARMED/FIRED) so telemetry decoders remain valid;
 * CHARGING is the bank-charge phase
 * of ARM. SWEEPING is new: sweep emission in progress after FIRE. */
#define WOK_HAL_SRC_IDLE       0   /* Bank safe/dumped, ready to arm    */
#define WOK_HAL_SRC_CHARGING   1   /* Bank charging toward target       */
#define WOK_HAL_SRC_ARMED      2   /* Bank at target charge, hold       */
#define WOK_HAL_SRC_FIRED      3   /* Shot delivered (strike or sweep end) */
#define WOK_HAL_SRC_SWEEPING   4   /* Sweep emission in progress        */

/* ── Source Modes ────────────────────────────────────────────────── */
#define WOK_HAL_SRC_MODE_IMPULSE  0
#define WOK_HAL_SRC_MODE_SWEEP    1

/* ── Seismic Acquisition ─────────────────────────────────────────── */

int32_t WOK_HAL_Seis_Init(void);
int32_t WOK_HAL_Seis_Configure(uint16_t fs, uint8_t gain, uint8_t ch_mask);

/**
 * @brief Set the survey-site context for subsequent acquisitions: rover
 *        position + heading in the mission start frame (from NAV pose).
 *        Stamped into data products; the sim also uses it to anchor the
 *        synthetic subsurface model at the correct location.
 */
int32_t WOK_HAL_Seis_SetSite(float x_m, float y_m, float heading_deg);

/**
 * @brief Per-pod source-receiver geometry for individually placed pods
 *        (CP-3): pod world position [start frame] and the seat attempt
 *        it finally seated at. Persists until the next call for the
 *        same pod (placement missions refresh it every site); never
 *        called in fixed-array missions, where the configured static
 *        offsets apply. The sim uses position + attempt to anchor the
 *        coupling truth model and derives each channel's moveout
 *        offset from the actual pod-to-site distance.
 */
int32_t WOK_HAL_Seis_SetPodGeometry(uint8_t pod, float x_m, float y_m,
                                    uint8_t seat_attempt);

int32_t WOK_HAL_Seis_StartAcquisition(uint16_t dur_sec);
int32_t WOK_HAL_Seis_StopAcquisition(void);

/**
 * @brief Drain acquired samples. Returns up to max_samples int32 values,
 *        interleaved by active channel (buf[s*nch + ch]). Successive calls
 *        return successive samples; *actual = 0 when drained. ADC counts,
 *        24-bit signed range carried in int32.
 */
int32_t WOK_HAL_Seis_GetSamples(int32_t *buf, uint32_t max_samples, uint32_t *actual);
int32_t WOK_HAL_Seis_GetStatus(uint8_t *acquiring, uint32_t *sample_count);
int32_t WOK_HAL_Seis_GetQuicklook(float *rms, float *peak, float *dc, uint8_t nch);

/**
 * @brief Shot-to-trace synchronization: sample index (per channel) of
 *        the last FIRE within the current/most recent acquisition
 *        window — the impulse discharge instant, or the sweep start.
 *        *valid = 0 if no shot occurred while acquiring. This is the t0
 *        reference every arrival-time pick is measured against. t0 is
 *        latched ELECTRICALLY (fire command -> discharge/drive-start
 *        edge latches the ADC sample counter); source drive and ADC
 *        must share one hardware clock.
 *
 *        CONTRACT EVOLUTION (WOK-CP-001):
 *        the flight baseline is WIRELESS pods, which void the shared-
 *        clock assumption — t0 becomes a per-pod quantity with per-pod
 *        clock error (RF sync pulse or disciplined pod clocks). This
 *        call evolves to per-channel strike indices plus per-pod
 *        sync-quality/clock-error fields, and the picker consumes
 *        per-pod t0. Implement at flight porting; the shared-clock
 *        form remains valid for the sim. The per-pod sync-error MODEL is sim-exercisable as of
 *        CP-5 (WOK_SIM_SYNC_ERR_MS + the test shot's residual check),
 *        so the picker/inversion chain is already hardened against
 *        the budgeted clock error before the contract lands.
 */
int32_t WOK_HAL_Seis_GetStrikeInfo(uint32_t *strike_sample_idx, uint8_t *valid);

/* ── EM Source ───────────────────────────────────────────────────── */

/**
 * @brief Initialize the source (bank charger, discharge switch, driver).
 *        Bank must come up SAFE (dumped).
 */
int32_t WOK_HAL_Src_Init(void);

/**
 * @brief Configure the next shot. Persistent until reconfigured.
 *
 *        IMPULSE: energy_j = bank charge target (J); sweep params ignored.
 *        SWEEP:   f0_hz/f1_hz/duration_s define the linear pilot
 *                 (WOK_HAL_Src_PilotSample); energy_j ignored — the bank
 *                 target is the drive energy for the sweep duration.
 *
 *        Rejected while charging/armed/sweeping (abort first).
 */
int32_t WOK_HAL_Src_Configure(uint8_t mode, float energy_j,
                              float f0_hz, float f1_hz, float duration_s);

/**
 * @brief Arm — begin charging the bank to the configured target.
 *        Non-blocking; poll GetState for ARMED. Only valid from IDLE.
 *        Charging draws from the main battery; a depressed bus voltage
 *        slows or stalls the charge (the app's arm timeout covers this).
 */
int32_t WOK_HAL_Src_Arm(void);

/**
 * @brief Fire — deliver the shot. Only valid from ARMED. Non-blocking.
 *        IMPULSE: discharge the bank into the actuator (single strike);
 *                 state goes FIRED immediately.
 *        SWEEP:   start the pilot drive; state is SWEEPING until the
 *                 sweep completes, then FIRED. Poll GetSweepProgress.
 *        The fire edge latches strike t0 (see GetStrikeInfo).
 */
int32_t WOK_HAL_Src_Fire(void);

/**
 * @brief Reset source to IDLE after a completed shot (FIRED only).
 */
int32_t WOK_HAL_Src_Reset(void);

/**
 * @brief Abort — safe the source from ANY state: stop the drive, dump
 *        the bank through the bleed resistor, return to IDLE. Must be
 *        immediate.
 */
int32_t WOK_HAL_Src_Abort(void);

/**
 * @brief Read source state: IDLE(0), CHARGING(1), ARMED(2), FIRED(3),
 *        SWEEPING(4).
 */
int32_t WOK_HAL_Src_GetState(uint8_t *state);

/**
 * @brief Read capacitor-bank telemetry: voltage (V) and state of charge
 *        as percent of the configured shot's target energy (100 = armed).
 */
int32_t WOK_HAL_Src_GetBank(float *bank_v, float *soc_pct);

/**
 * @brief Read last shot info: peak plate force (N) and fire timestamp
 *        (ms since boot, same clock as the ADC). Valid after FIRED.
 *        For sweeps the force is the regulated drive amplitude.
 */
int32_t WOK_HAL_Src_GetShotInfo(float *peak_force_n, uint32_t *timestamp_ms);

/**
 * @brief Sweep progress 0.0-1.0 (1.0 = complete). 0.0 when not sweeping
 *        and no sweep has run since arm.
 */
int32_t WOK_HAL_Src_GetSweepProgress(float *frac);

/**
 * @brief Reseat the source coupling plate: lift and re-press it at the
 *        current position (WOK-CP-001 — commanded when the
 *        plate transfer metric drops below the running family).
 *        Rejected while charging/armed/sweeping (abort or fire first).
 *        Sim: redraws the plate's ground-coupling factor at the next
 *        seat attempt; a real mechanism would actuate the plate press.
 */
int32_t WOK_HAL_Src_ReseatPlate(void);

/* ── Sweep pilot (part of the source contract) ───────────────────────
 *
 * The pilot the source hardware must drive in SWEEP mode, and the exact
 * reference the SEIS app regenerates for onboard cross-correlation:
 * a linear sweep f0->f1 over duration_s with cosine (Tukey) end tapers.
 * Defined here — inline, pure — so the sim's synthesis, the app's
 * correlator, and a flight driver's waveform table all agree
 * bit-for-bit.
 *
 * The taper fraction is deliberately heavy: it shapes the correlated
 * (Klauder) wavelet's envelope into a smooth monotonic rise with no
 * flat side-lobe shelf, which is what lets an ONSET picker (STA/LTA)
 * read the equivalent-impulse trace consistently with a true impulse
 * record. Tuned against the physics harness — do not reduce without
 * re-running the impulse-vs-sweep pick agreement check.
 */
#define WOK_HAL_SRC_PILOT_TAPER_FRAC  0.5f   /* cosine ramp each end */

static inline float WOK_HAL_Src_PilotSample(float t_s, float f0_hz,
                                            float f1_hz, float duration_s)
{
    if (duration_s <= 0.0f || t_s < 0.0f || t_s >= duration_s) return 0.0f;

    float k     = (f1_hz - f0_hz) / duration_s;      /* sweep rate Hz/s */
    float phase = 2.0f * 3.14159265f * (f0_hz * t_s + 0.5f * k * t_s * t_s);

    float taper = 1.0f;
    float ramp  = WOK_HAL_SRC_PILOT_TAPER_FRAC * duration_s;
    if (t_s < ramp)
        taper = 0.5f * (1.0f - cosf(3.14159265f * t_s / ramp));
    else if (t_s > duration_s - ramp)
        taper = 0.5f * (1.0f - cosf(3.14159265f * (duration_s - t_s) / ramp));

    return taper * sinf(phase);
}

/* ── Seismic Evaluation (computed, not hardware) ─────────────────── */

/**
 * @brief Evaluate the most recent acquisition against the ice-signature
 *        threshold. Returns flag=1 if signature detected, 0 otherwise.
 *        confidence is 0.0–1.0.
 *
 * This runs on the recorded data buffer — call after acquisition complete.
 * In the sim, the verdict derives from the synthetic subsurface model at
 * the configured site (elevated refractor Vp → ice-cemented), so results
 * are deterministic per site and agree with what the drill sim will find.
 * (The SEIS app computes EVALUATE from its own onboard first-break
 * picks; this HAL call is the no-picks fallback only.)
 */
int32_t WOK_HAL_Seis_Evaluate(uint8_t *flag, float *confidence);

#endif /* WOK_HAL_SEIS_H */
