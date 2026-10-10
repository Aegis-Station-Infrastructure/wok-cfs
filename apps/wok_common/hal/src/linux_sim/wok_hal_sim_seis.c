/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Seismic + EM Source
** File: wok_common/hal/src/linux_sim/wok_hal_sim_seis.c
**
** Seismic: synthetic traces derived from the shared subsurface geology
**   model (wok_hal_sim_geology.c). Each channel is a geophone pod at a
**   known offset from the source plate.
**   IMPULSE shot: a damped-sine P wavelet delayed by the model's
**     first-arrival traveltime (direct wave vs L1/L2 head wave), with
**     amplitude decaying with offset and scaling with commanded bank
**     energy. Background is broadband noise.
**   SWEEP shot: the pilot (WOK_HAL_Src_PilotSample — the contract
**     waveform) delayed by the same traveltime, i.e. the pilot convolved
**     with the geology model's arrival series, plus the same noise.
**   Both are deterministic per site (geology-driven) up to the noise.
** Timing: source and ADC share ONE monotonic clock. FIRE while acquiring
**   latches its per-channel sample index (the electrical t0 reference) —
**   impulse discharge instant, or sweep start.
** Source: capacitor bank charges at SIM_BANK_CHARGE_W drawn from the
**   battery model (charge inhibited when the bus is below
**   WOK_SRC_MIN_CHARGE_BATT_V — an arm left charging then never reaches
**   ARMED and the sequencer's arm timeout fires). Firing dumps the bank,
**   so stacked shots recharge between strikes. Impulse plate force
**   scales with sqrt(bank energy); the 50 J default reproduces the v4.0
**   drop-weight amplitude.
** Evaluate: derived from the geology model at the configured site —
**   deterministic per site, consistent with the drill sim.
** Coupling (WOK_SIM_COUPLING=1, default OFF): per-pod and plate
**   ground-coupling factors from the shared coupling truth model
**   (wok_hal_sim_coupling.c) scale each channel's SIGNAL amplitude
**   (noise is electronic/ambient and unaffected). Factors are drawn at
**   SetSite for seat attempt 0 — reseat attempts arrive with CP-2/3.
**   The reported plate peak force (GetShotInfo) is the actuator-side
**   measurement and is deliberately NOT scaled: a bad plate seat drops
**   the near-pod response while the force reads normal, which is what
**   makes the WOK-CP-001 transfer ratio a real coupling metric.
**   Disabled (the default), every factor is exactly 1.0 and synthesis
**   is bit-identical to pre-CP-1 — regression missions are untouched.
*/

/* clock_gettime/CLOCK_MONOTONIC under strict C99 */
#define _POSIX_C_SOURCE 199309L

#include "wok_hal_seis.h"
#include "wok_hal_power.h"
#include "wok_hal_sim_geology.h"
#include "wok_hal_sim_coupling.h"
#include "wok_hal_sim_power.h"
#include "wok_platform_cfg.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

/* ── Synthetic waveform parameters ───────────────────────────────── */
#define SIM_NOISE_COUNTS      80.0f   /* background noise amplitude     */
#define SIM_WAVELET_F0_HZ     25.0f   /* impulse P wavelet centre freq  */
#define SIM_WAVELET_TAU_S      0.15f  /* impulse wavelet decay constant */
#define SIM_AMP_COUNTS_N   40000.0f   /* counts per Newton at 1 m       */
#define SIM_CHUNK_MAX       1000u     /* samples/chan per GetSamples call */

/* ── EM source model ─────────────────────────────────────────────── */
#define SIM_BANK_CHARGE_W     50.0f   /* charger power (50 J default -> 1 s,
                                         matching the v4.0 1 s arm time) */
#define SIM_BANK_CAP_F         2.5e-3f /* bank capacitance: 200 J = 400 V */
#define SIM_IMPULSE_REF_N      8.5f   /* plate force at the 50 J default */
/* Regulated (force-limited) sweep drive. Weak by design — per-shot
 * arrivals sit at/below the noise floor and only emerge in the
 * correlation (the physical point of sweep mode). Also keeps the
 * correlated SNR moderate so the onset picker's threshold crossing
 * stays on the Klauder main lobe (harness-tuned with
 * WOK_SEIS_CORR_LEAD_FRAC / the pilot taper). */
#define SIM_SWEEP_FORCE_N      0.02f
#define SIM_SWEEP_DRIVE_W     10.0f   /* bank energy per second of sweep */

static const float SIM_POD_OFFSETS_M[WOK_SEIS_MAX_CHANNELS] =
    WOK_SEIS_POD_OFFSETS_M_INIT;

/* ── Shared monotonic clock (source + ADC) ───────────────────────── */
static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

/* ── Seismic State ───────────────────────────────────────────────── */
static struct {
    uint8_t  initialized;
    uint8_t  acquiring;
    uint16_t fs;
    uint8_t  gain;
    uint8_t  ch_mask;
    uint32_t sample_count;      /* samples/chan available so far        */
    uint32_t gen_pos;           /* samples/chan already drained         */
    uint32_t target_samples;    /* samples/chan for this acquisition    */
    uint64_t acq_start_ms;      /* monotonic — shared with the source   */
    /* Site context (start frame, from NAV pose via the app) */
    float    site_x_m, site_y_m, site_heading_deg;
    /* Ground coupling for the seated array/plate at this site (all 1.0
     * unless WOK_SIM_COUPLING is set — see file header). The plate seat
     * attempt persists across SetSite calls at the SAME pose (stacked
     * shots keep one seating; ReseatPlate advances it) and resets when
     * the site pose changes (driving to a new site reseats naturally). */
    uint8_t  coupling_on;
    float    pod_coup[WOK_SEIS_MAX_CHANNELS];
    float    plate_coup;
    uint8_t  plate_attempt;
    uint8_t  site_valid;        /* prev_site_* hold a real site         */
    float    prev_site_x, prev_site_y;
    /* Per-pod geometry overrides (CP-3, SetPodGeometry): actual placed
     * world positions + final seat attempts. Without an override a pod
     * sits at its configured offset along the heading, attempt 0. */
    uint8_t  pod_geom_valid[WOK_SEIS_MAX_CHANNELS];
    float    pod_gx[WOK_SEIS_MAX_CHANNELS], pod_gy[WOK_SEIS_MAX_CHANNELS];
    uint8_t  pod_gattempt[WOK_SEIS_MAX_CHANNELS];
    float    pod_off[WOK_SEIS_MAX_CHANNELS];   /* moveout offsets       */
    /* Per-pod clock/sync bias (CP-5, WOK_SIM_SYNC_ERR_MS): wireless
     * pods time-stamp against their own clocks, so each pod's t0
     * reference — and therefore its apparent arrival — is offset by a
     * per-pod bias. 0 unless the env magnitude is set. */
    float    sync_max_ms;
    float    pod_sync_s[WOK_SEIS_MAX_CHANNELS];
    /* Shot-in-window latch (t0 + the shot parameters that shaped it) */
    uint8_t  strike_valid;
    uint32_t strike_sample_idx;
    float    strike_force_n;
    uint8_t  strike_mode;       /* WOK_HAL_SRC_MODE_* at fire           */
    float    strike_f0, strike_f1, strike_dur_s;   /* sweep pilot params */
} g_seis;

/* ── Source State ────────────────────────────────────────────────── */
static struct {
    uint8_t  initialized;
    uint8_t  state;             /* WOK_HAL_SRC_*                        */
    uint8_t  mode;              /* configured shot mode                 */
    float    energy_target_j;   /* bank charge target for this shot     */
    float    sweep_f0, sweep_f1, sweep_dur_s;
    float    bank_j;            /* current bank energy                  */
    uint64_t last_update_ms;    /* incremental charge integration       */
    uint64_t sweep_start_ms;
    float    sweep_progress;    /* 0-1, latched at completion           */
    float    last_force_n;
    uint32_t last_timestamp_ms; /* fire time, ms (same clock as ADC)    */
} g_src;

/* Bank target for the configured shot: commanded energy (impulse) or
 * drive energy for the sweep duration. */
static float src_target_j(void)
{
    if (g_src.mode == WOK_HAL_SRC_MODE_SWEEP)
        return SIM_SWEEP_DRIVE_W * g_src.sweep_dur_s;
    return g_src.energy_target_j;
}

/* Advance the source model from the wall clock: integrate bank charge
 * (inhibited on a depressed bus), complete sweeps. */
static void src_update(void)
{
    uint64_t now = now_ms();

    if (g_src.state == WOK_HAL_SRC_CHARGING)
    {
        float dt_s = (float)(now - g_src.last_update_ms) / 1000.0f;
        float batt_v = 0.0f;
        if (WOK_HAL_Power_GetBatteryV(&batt_v) == WOK_HAL_OK &&
            batt_v >= WOK_SRC_MIN_CHARGE_BATT_V)
        {
            float dj = SIM_BANK_CHARGE_W * dt_s;
            float target = src_target_j();
            if (g_src.bank_j + dj >= target)
            {
                dj = target - g_src.bank_j;
                g_src.bank_j = target;
                g_src.state  = WOK_HAL_SRC_ARMED;
            }
            else
            {
                g_src.bank_j += dj;
            }
            WOK_HAL_SimPower_Drain_J(dj);   /* charging draws the battery */
        }
        /* else: bus too low — bank holds, ARMED never comes, and the
         * app/sequencer arm timeout fires. */
    }
    else if (g_src.state == WOK_HAL_SRC_SWEEPING)
    {
        float el = (float)(now - g_src.sweep_start_ms) / 1000.0f;
        if (el >= g_src.sweep_dur_s)
        {
            g_src.sweep_progress = 1.0f;
            g_src.bank_j        *= 0.02f;   /* drive spent the bank */
            g_src.state          = WOK_HAL_SRC_FIRED;
        }
        else
        {
            g_src.sweep_progress = el / g_src.sweep_dur_s;
        }
    }
    g_src.last_update_ms = now;
}

/* ── Trace synthesis ─────────────────────────────────────────────── */

/* Map active-channel slot (0..nch-1) to physical channel (pod) index. */
static int phys_channel(uint32_t slot)
{
    uint32_t seen = 0;
    for (int ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
    {
        if (g_seis.ch_mask & (1u << ch))
        {
            if (seen == slot) return ch;
            seen++;
        }
    }
    return (int)slot;
}

/* Synthetic sample for pod `ch` at absolute sample index `s` (per chan). */
static float synth_sample(int ch, uint32_t s)
{
    float val = ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f * SIM_NOISE_COUNTS;

    if (g_seis.strike_valid && s >= g_seis.strike_sample_idx)
    {
        float t0     = (float)g_seis.strike_sample_idx / (float)g_seis.fs;
        float t      = (float)s / (float)g_seis.fs;
        float offset = g_seis.pod_off[ch];
        float t_arr  = WOK_SimGeology_FirstArrivalSec(g_seis.site_x_m,
                                                      g_seis.site_y_m,
                                                      offset);
        float dt = t - (t0 + t_arr + g_seis.pod_sync_s[ch]);
        if (dt >= 0.0f)
        {
            float amp = g_seis.strike_force_n * SIM_AMP_COUNTS_N
                      / (offset * offset + 1.0f)
                      * g_seis.plate_coup * g_seis.pod_coup[ch];
            if (g_seis.strike_mode == WOK_HAL_SRC_MODE_SWEEP)
            {
                /* Ground motion = pilot convolved with the geology
                 * model's arrival series: the delayed contract pilot. */
                val += amp * WOK_HAL_Src_PilotSample(dt,
                                                     g_seis.strike_f0,
                                                     g_seis.strike_f1,
                                                     g_seis.strike_dur_s);
            }
            else
            {
                /* Damped sine wavelet, geometric spreading with offset */
                val += amp * expf(-dt / SIM_WAVELET_TAU_S)
                           * sinf(2.0f * 3.14159265f * SIM_WAVELET_F0_HZ * dt);
            }
        }
    }
    return val;
}

/* Advance sample_count from the wall clock; auto-stop at target. */
static void update_available(void)
{
    if (!g_seis.acquiring || g_seis.target_samples == 0) return;

    uint64_t dt_ms   = now_ms() - g_seis.acq_start_ms;
    uint32_t expected = (uint32_t)(dt_ms * g_seis.fs / 1000u);
    if (expected >= g_seis.target_samples)
    {
        g_seis.acquiring    = 0;
        g_seis.sample_count = g_seis.target_samples;
    }
    else
    {
        g_seis.sample_count = expected;
    }
}

/* ── Seismic ─────────────────────────────────────────────────────── */

int32_t WOK_HAL_Seis_Init(void)
{
    const char *coup_env, *sync_env;

    memset(&g_seis, 0, sizeof(g_seis));
    sync_env = getenv("WOK_SIM_SYNC_ERR_MS");
    g_seis.sync_max_ms = sync_env ? (float)atof(sync_env) : 0.0f;
    if (g_seis.sync_max_ms < 0.0f) g_seis.sync_max_ms = 0.0f;
    g_seis.initialized = 1;
    g_seis.fs = 1000;
    g_seis.gain = 1;
    g_seis.ch_mask = 0x0F;
    coup_env = getenv("WOK_SIM_COUPLING");
    g_seis.coupling_on = (coup_env != NULL && coup_env[0] != '\0'
                          && coup_env[0] != '0') ? 1 : 0;
    for (int ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
    {
        g_seis.pod_coup[ch] = 1.0f;
        g_seis.pod_off[ch]  = SIM_POD_OFFSETS_M[ch];
    }
    g_seis.plate_coup = 1.0f;
    srand((unsigned)time(NULL));
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Seis_Configure(uint16_t fs, uint8_t gain, uint8_t ch_mask)
{
    if (!g_seis.initialized) return WOK_HAL_ERROR;
    g_seis.fs = fs;
    g_seis.gain = gain;
    g_seis.ch_mask = ch_mask;
    return WOK_HAL_OK;
}

/* Recompute per-channel coupling factors and moveout offsets for the
 * current site. Pods with a geometry override (SetPodGeometry — actual
 * placed position + final seat attempt) use it; the rest sit at their
 * configured offsets along the heading, seat attempt 0. Coupling off →
 * factors stay 1.0, but overridden moveout offsets still apply, so
 * placement-mode geometry works with or without the coupling model. */
static void seis_seat_pods(void)
{
    float hdg_rad = g_seis.site_heading_deg * 3.14159265f / 180.0f;
    float chx = cosf(hdg_rad), shy = sinf(hdg_rad);

    for (int ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
    {
        float   px, py;
        uint8_t att;
        if (g_seis.pod_geom_valid[ch])
        {
            px  = g_seis.pod_gx[ch];
            py  = g_seis.pod_gy[ch];
            att = g_seis.pod_gattempt[ch];
            g_seis.pod_off[ch] = hypotf(px - g_seis.site_x_m,
                                        py - g_seis.site_y_m);
        }
        else
        {
            px  = g_seis.site_x_m + SIM_POD_OFFSETS_M[ch] * chx;
            py  = g_seis.site_y_m + SIM_POD_OFFSETS_M[ch] * shy;
            att = 0;
            g_seis.pod_off[ch] = SIM_POD_OFFSETS_M[ch];
        }
        g_seis.pod_coup[ch] = g_seis.coupling_on
                            ? WOK_SimCoupling_PodCoupling(px, py, att) : 1.0f;
        g_seis.pod_sync_s[ch] = (g_seis.sync_max_ms > 0.0f)
            ? WOK_SimCoupling_PodSyncBias(px, py)
              * g_seis.sync_max_ms * 1e-3f
            : 0.0f;
    }
    g_seis.plate_coup = g_seis.coupling_on
        ? WOK_SimCoupling_PlateCoupling(g_seis.site_x_m, g_seis.site_y_m,
                                        g_seis.plate_attempt)
        : 1.0f;
}

int32_t WOK_HAL_Seis_SetSite(float x_m, float y_m, float heading_deg)
{
    if (!g_seis.initialized) return WOK_HAL_ERROR;
    g_seis.site_x_m        = x_m;
    g_seis.site_y_m        = y_m;
    g_seis.site_heading_deg = heading_deg;

    /* A new site pose is a fresh plate seating; the same pose (stacked
     * shots) keeps the current seat attempt so ReseatPlate persists. */
    if (!g_seis.site_valid ||
        x_m != g_seis.prev_site_x || y_m != g_seis.prev_site_y)
    {
        g_seis.plate_attempt = 0;
        g_seis.prev_site_x   = x_m;
        g_seis.prev_site_y   = y_m;
        g_seis.site_valid    = 1;
    }

    seis_seat_pods();
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Seis_SetPodGeometry(uint8_t pod, float x_m, float y_m,
                                    uint8_t seat_attempt)
{
    if (!g_seis.initialized || pod >= WOK_SEIS_MAX_CHANNELS)
        return WOK_HAL_ERROR;
    g_seis.pod_geom_valid[pod] = 1;
    g_seis.pod_gx[pod]         = x_m;
    g_seis.pod_gy[pod]         = y_m;
    g_seis.pod_gattempt[pod]   = seat_attempt;
    seis_seat_pods();
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_ReseatPlate(void)
{
    if (!g_src.initialized || !g_seis.initialized) return WOK_HAL_ERROR;
    src_update();
    if (g_src.state == WOK_HAL_SRC_CHARGING ||
        g_src.state == WOK_HAL_SRC_ARMED    ||
        g_src.state == WOK_HAL_SRC_SWEEPING)
        return WOK_HAL_ERROR;   /* mechanism busy — abort/fire first */

    g_seis.plate_attempt++;
    if (g_seis.coupling_on)
        g_seis.plate_coup = WOK_SimCoupling_PlateCoupling(g_seis.site_x_m,
                                                          g_seis.site_y_m,
                                                          g_seis.plate_attempt);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Seis_StartAcquisition(uint16_t dur_sec)
{
    if (!g_seis.initialized) return WOK_HAL_ERROR;
    g_seis.acquiring         = 1;
    g_seis.sample_count      = 0;
    g_seis.gen_pos           = 0;
    g_seis.target_samples    = (uint32_t)dur_sec * g_seis.fs;
    g_seis.acq_start_ms      = now_ms();
    g_seis.strike_valid      = 0;
    g_seis.strike_sample_idx = 0;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Seis_StopAcquisition(void)
{
    if (!g_seis.initialized) return WOK_HAL_ERROR;
    g_seis.acquiring = 0;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Seis_GetSamples(int32_t *buf, uint32_t max_samples, uint32_t *actual)
{
    if (!g_seis.initialized || !buf || !actual) return WOK_HAL_ERROR;

    update_available();

    uint32_t nch = 0;
    for (int i = 0; i < WOK_SEIS_MAX_CHANNELS; i++)
        if (g_seis.ch_mask & (1u << i)) nch++;
    if (nch == 0) nch = 1;

    uint32_t avail = (g_seis.sample_count > g_seis.gen_pos)
                   ? g_seis.sample_count - g_seis.gen_pos : 0;
    uint32_t n = max_samples / nch;
    if (n > SIM_CHUNK_MAX) n = SIM_CHUNK_MAX;
    if (n > avail)         n = avail;

    for (uint32_t s = 0; s < n; s++)
    {
        uint32_t s_abs = g_seis.gen_pos + s;
        for (uint32_t slot = 0; slot < nch; slot++)
            buf[s * nch + slot] = (int32_t)synth_sample(phys_channel(slot), s_abs);
    }

    g_seis.gen_pos += n;
    *actual = n * nch;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Seis_GetStatus(uint8_t *acquiring, uint32_t *sample_count)
{
    if (!g_seis.initialized || !acquiring || !sample_count) return WOK_HAL_ERROR;

    update_available();
    *acquiring = g_seis.acquiring;
    *sample_count = g_seis.sample_count;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Seis_GetStrikeInfo(uint32_t *strike_sample_idx, uint8_t *valid)
{
    if (!g_seis.initialized || !strike_sample_idx || !valid) return WOK_HAL_ERROR;
    *strike_sample_idx = g_seis.strike_sample_idx;
    *valid             = g_seis.strike_valid;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Seis_GetQuicklook(float *rms, float *peak, float *dc, uint8_t nch)
{
    if (!g_seis.initialized || !rms || !peak || !dc) return WOK_HAL_ERROR;

    update_available();

    /* Stats over the most recent 256 available samples of the same
     * synthetic waveform GetSamples produces. */
    const uint32_t N = 256;
    uint32_t s_end   = g_seis.sample_count;
    uint32_t s_start = (s_end > N) ? s_end - N : 0;
    uint32_t count   = s_end - s_start;
    if (count == 0) count = 1;

    for (uint8_t ch = 0; ch < nch && ch < WOK_SEIS_MAX_CHANNELS; ch++)
    {
        if (!(g_seis.ch_mask & (1u << ch)))
        {
            rms[ch] = peak[ch] = dc[ch] = 0.0f;
            continue;
        }
        float sumsq = 0.0f, sum = 0.0f, pk = 0.0f;
        for (uint32_t s = s_start; s < s_end; s++)
        {
            float val = synth_sample(ch, s);
            sum   += val;
            sumsq += val * val;
            float a = val < 0.0f ? -val : val;
            if (a > pk) pk = a;
        }
        rms[ch]  = sqrtf(sumsq / (float)count);
        dc[ch]   = sum / (float)count;
        peak[ch] = pk;
    }
    return WOK_HAL_OK;
}

/* ── EM Source ───────────────────────────────────────────────────── */

int32_t WOK_HAL_Src_Init(void)
{
    memset(&g_src, 0, sizeof(g_src));
    g_src.initialized = 1;
    g_src.state       = WOK_HAL_SRC_IDLE;
    g_src.mode        = WOK_SRC_DEFAULT_MODE;
    g_src.energy_target_j = WOK_SRC_DEFAULT_ENERGY_J;
    g_src.sweep_f0    = WOK_SRC_DEFAULT_SWEEP_F0_HZ;
    g_src.sweep_f1    = WOK_SRC_DEFAULT_SWEEP_F1_HZ;
    g_src.sweep_dur_s = WOK_SRC_DEFAULT_SWEEP_DUR_S;
    g_src.last_update_ms = now_ms();
    /* Bank charging reads the battery through this app's power copy */
    WOK_HAL_Power_Init();
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_Configure(uint8_t mode, float energy_j,
                              float f0_hz, float f1_hz, float duration_s)
{
    if (!g_src.initialized) return WOK_HAL_ERROR;
    src_update();
    if (g_src.state != WOK_HAL_SRC_IDLE && g_src.state != WOK_HAL_SRC_FIRED)
        return WOK_HAL_ERROR;   /* abort first */

    if (mode == WOK_HAL_SRC_MODE_IMPULSE)
    {
        if (energy_j <= 0.0f) energy_j = WOK_SRC_DEFAULT_ENERGY_J;
        if (energy_j > WOK_SRC_MAX_ENERGY_J) energy_j = WOK_SRC_MAX_ENERGY_J;
        g_src.energy_target_j = energy_j;
    }
    else if (mode == WOK_HAL_SRC_MODE_SWEEP)
    {
        if (f1_hz <= f0_hz || f0_hz <= 0.0f) return WOK_HAL_ERROR;
        if (duration_s <= 0.0f ||
            duration_s > (float)WOK_SRC_SWEEP_MAX_DUR_SEC) return WOK_HAL_ERROR;
        g_src.sweep_f0    = f0_hz;
        g_src.sweep_f1    = f1_hz;
        g_src.sweep_dur_s = duration_s;
    }
    else
    {
        return WOK_HAL_ERROR;
    }
    g_src.mode = mode;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_Arm(void)
{
    if (!g_src.initialized) return WOK_HAL_ERROR;
    src_update();
    if (g_src.state != WOK_HAL_SRC_IDLE) return WOK_HAL_ERROR;
    g_src.state          = WOK_HAL_SRC_CHARGING;
    g_src.sweep_progress = 0.0f;
    g_src.last_update_ms = now_ms();
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_Fire(void)
{
    if (!g_src.initialized) return WOK_HAL_ERROR;
    src_update();
    if (g_src.state != WOK_HAL_SRC_ARMED) return WOK_HAL_ERROR;

    uint64_t fire_ms = now_ms();
    g_src.last_timestamp_ms = (uint32_t)fire_ms;

    if (g_src.mode == WOK_HAL_SRC_MODE_SWEEP)
    {
        /* Force-limited regulated drive; the bank spends across the sweep */
        g_src.last_force_n   = SIM_SWEEP_FORCE_N;
        g_src.sweep_start_ms = fire_ms;
        g_src.sweep_progress = 0.0f;
        g_src.state          = WOK_HAL_SRC_SWEEPING;
    }
    else
    {
        /* Cap discharge -> single strike. Plate force scales with the
         * stored energy (impulse ~ sqrt(E)); the 50 J default reproduces
         * the v4.0 drop-weight's 8.5-9.5 N. */
        g_src.last_force_n = SIM_IMPULSE_REF_N
                           * sqrtf(g_src.bank_j / WOK_SRC_DEFAULT_ENERGY_J)
                           + (float)(rand() % 100) / 100.0f;
        g_src.bank_j *= 0.02f;              /* discharged into the plate */
        g_src.state   = WOK_HAL_SRC_FIRED;
    }

    /* Latch electrical t0 into the sample stream: source and ADC share
     * the monotonic clock, so t0 becomes a per-channel sample index. */
    update_available();
    if (g_seis.acquiring)
    {
        uint64_t dt_ms = fire_ms - g_seis.acq_start_ms;
        g_seis.strike_sample_idx = (uint32_t)(dt_ms * g_seis.fs / 1000u);
        g_seis.strike_valid      = 1;
        g_seis.strike_force_n    = g_src.last_force_n;
        g_seis.strike_mode       = g_src.mode;
        g_seis.strike_f0         = g_src.sweep_f0;
        g_seis.strike_f1         = g_src.sweep_f1;
        g_seis.strike_dur_s      = g_src.sweep_dur_s;
    }
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_Reset(void)
{
    if (!g_src.initialized) return WOK_HAL_ERROR;
    src_update();
    if (g_src.state != WOK_HAL_SRC_FIRED) return WOK_HAL_ERROR;
    g_src.state = WOK_HAL_SRC_IDLE;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_Abort(void)
{
    if (!g_src.initialized) return WOK_HAL_ERROR;
    /* Safe from any state: stop the drive, dump the bank, back to IDLE */
    g_src.bank_j = 0.0f;
    g_src.state  = WOK_HAL_SRC_IDLE;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_GetState(uint8_t *state)
{
    if (!g_src.initialized || !state) return WOK_HAL_ERROR;
    src_update();
    *state = g_src.state;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_GetBank(float *bank_v, float *soc_pct)
{
    if (!g_src.initialized || !bank_v || !soc_pct) return WOK_HAL_ERROR;
    src_update();
    *bank_v = sqrtf(2.0f * g_src.bank_j / SIM_BANK_CAP_F);
    float target = src_target_j();
    *soc_pct = (target > 0.0f) ? 100.0f * g_src.bank_j / target : 0.0f;
    if (*soc_pct > 100.0f) *soc_pct = 100.0f;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_GetShotInfo(float *peak_force_n, uint32_t *timestamp_ms)
{
    if (!g_src.initialized || !peak_force_n || !timestamp_ms) return WOK_HAL_ERROR;
    *peak_force_n = g_src.last_force_n;
    *timestamp_ms = g_src.last_timestamp_ms;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Src_GetSweepProgress(float *frac)
{
    if (!g_src.initialized || !frac) return WOK_HAL_ERROR;
    src_update();
    *frac = g_src.sweep_progress;
    return WOK_HAL_OK;
}

/* ── Evaluate ────────────────────────────────────────────────────── */

int32_t WOK_HAL_Seis_Evaluate(uint8_t *flag, float *confidence)
{
    if (!g_seis.initialized || !flag || !confidence) return WOK_HAL_ERROR;

    /* Sim: verdict from the shared geology model at the configured site.
     * Elevated L2 refractor Vp (ice cementation) is the signature. This
     * keeps site decisions deterministic and consistent with the drill
     * sim. The SEIS app normally evaluates from its own onboard picks;
     * this is the no-picks fallback path. */
    WOK_SimGeoLayer_t col[WOK_SIM_GEO_NLAYERS];
    WOK_SimGeology_GetColumn(g_seis.site_x_m, g_seis.site_y_m, col);

    float ice      = col[1].ice_wtpct;
    float ice_frac = ice / 5.0f;
    if (ice_frac > 1.0f) ice_frac = 1.0f;

    if (ice > 0.5f)   /* ≥0.5 wt% water-equivalent → flag site */
    {
        *flag = 1;
        *confidence = 0.5f + 0.45f * ice_frac;
    }
    else
    {
        *flag = 0;
        *confidence = 0.05f + 0.3f * ice_frac;
    }
    /* Small measurement jitter so repeat evaluations aren't bit-identical */
    *confidence += ((float)rand() / (float)RAND_MAX - 0.5f) * 0.02f;
    if (*confidence < 0.01f) *confidence = 0.01f;
    if (*confidence > 0.99f) *confidence = 0.99f;

    return WOK_HAL_OK;
}
