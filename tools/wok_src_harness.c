/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK physics harness — dual-mode EM source verification.
**
** Build:
**   gcc -O2 -std=c99 wok_src_harness.c \
**       $REPO/apps/wok_common/hal/src/linux_sim/wok_hal_sim_seis.c \
**       $REPO/apps/wok_common/hal/src/linux_sim/wok_hal_sim_geology.c \
**       $REPO/apps/wok_common/hal/src/linux_sim/wok_hal_sim_coupling.c \
**       $REPO/apps/wok_common/hal/src/linux_sim/wok_hal_sim_geo.c \
**       $REPO/apps/wok_common/hal/src/linux_sim/wok_hal_sim_power.c \
**       -I$REPO/apps/wok_common/hal/inc \
**       -I$REPO/apps/wok_common/hal/src/linux_sim \
**       -I$REPO/apps/wok_common/msgs/inc -lm -o harness
**
** Modes:
**   ./harness tune  — fast synthetic loop (no wall clock): replicate the
**                     sim's trace synthesis directly, run the app's
**                     correlate/stack/pick chain, report impulse-vs-sweep
**                     pick deltas per pod. For tuning pilot taper, sweep
**                     force, and the causal-lead compensation.
**   ./harness hal   — end-to-end through the real sim HAL (wall-clock
**                     acquisitions): 3-shot impulse stack picks vs 3-shot
**                     sweep correlated stack picks at two sites; the
**                     +-0.5 ms agreement gate (verification req 1).
**   ./harness bank  — bank model: charge timing, deplete/recharge across
**                     stacked shots, low-battery arm stall (req 4).
**   ./harness coupling — CP-1 coupling truth model: determinism, seat
**                     statistics (reseat must improve a bad seat), the
**                     seating-force proxy correlation + false-accept
**                     tail, and end-to-end WOK_SIM_COUPLING=1 amplitude
**                     ratios through the real sim HAL vs the model.
**   ./harness plate — CP-2 plate reseat: ReseatPlate redraws the plate
**                     coupling per the truth model; SetSite keeps the
**                     seat attempt at the same pose and resets it at a
**                     new one. `plate scan` maps reseat outcomes over
**                     the fluff patch (mission-site targeting);
**                     `plate at <x> <y>` prints a position's draws
**                     (post-hoc mission verification).
**   ./harness pods  — CP-3 per-pod placement: place/reseat/retrieve state
**                     machine against the coupling truth model.
**   ./harness sync  — CP-5 per-pod clock error: pick deltas vs the
**                     injected WOK_SIM_SYNC_ERR_MS bias.
*/

#define _POSIX_C_SOURCE 200112L   /* clock_gettime + setenv */
#include "wok_hal_seis.h"
#include "wok_hal_power.h"
#include "wok_hal_sim_geology.h"
#include "wok_hal_sim_coupling.h"
#include "wok_hal_sim_power.h"
#include "wok_platform_cfg.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#define FS        4000.0f
#define NCH       4
#define PRE       WOK_SEIS_STACK_PRE_SAMPS
#define POST      WOK_SEIS_STACK_POST_SAMPS
#define WIN       WOK_SEIS_STACK_WIN_SAMPS
#define STA_SAMPS 40
#define TRIG      4.0f

static const float PODS[NCH] = WOK_SEIS_POD_OFFSETS_M_INIT;

/* Tunables (overridable from argv in tune mode) */
static float g_lead_frac = WOK_SEIS_CORR_LEAD_FRAC;
static float g_corr_gain = WOK_SEIS_CORR_GAIN;
static float g_swp_force = 0.02f;   /* = SIM_SWEEP_FORCE_N */

/* ================= app-chain replicas (mirror wok_seis_app.c) ======= */

/* STA/LTA picker with frozen pre-strike LTA — verbatim logic from
 * WOK_Seis_RunPicker. */
static void run_picker(const int32_t *stack, int nch, float fs,
                       float fb_ms[NCH])
{
    for (int ch = 0; ch < NCH; ch++) fb_ms[ch] = -1.0f;
    for (int slot = 0; slot < nch; slot++)
    {
        double noise_sq = 0.0;
        uint32_t n_noise = PRE - STA_SAMPS;
        for (uint32_t s = 0; s < n_noise; s++)
        {
            double v = (double)stack[s * nch + slot];
            noise_sq += v * v;
        }
        noise_sq /= (double)n_noise;
        if (noise_sq < 1.0) noise_sq = 1.0;
        float noise_rms = sqrtf((float)noise_sq);

        double sta_sq = 0.0;
        for (uint32_t s = PRE - STA_SAMPS; s < PRE; s++)
        {
            double v = (double)stack[s * nch + slot];
            sta_sq += v * v;
        }
        for (uint32_t s = PRE; s < WIN; s++)
        {
            double v_in  = (double)stack[s * nch + slot];
            double v_out = (double)stack[(s - STA_SAMPS) * nch + slot];
            sta_sq += v_in * v_in - v_out * v_out;
            float ratio = (float)sqrt((sta_sq / STA_SAMPS) / noise_sq);
            if (ratio > TRIG)
            {
                uint32_t onset = s;
                for (uint32_t b = s - STA_SAMPS + 1; b <= s; b++)
                {
                    int32_t av = stack[b * nch + slot];
                    if (av < 0) av = -av;
                    if ((float)av > 4.0f * noise_rms) { onset = b; break; }
                }
                fb_ms[slot] = (float)(onset - PRE) * 1000.0f / fs;
                break;
            }
        }
    }
}

/* Sweep correlation — the algorithm the SEIS app will run (chunking is
 * an app-side pacing detail; the math is identical). Output: WIN samples
 * per channel, sample j aligned so the stack window [strike-PRE .. +POST)
 * maps 1:1, with the causal-lead delay applied. */
static void correlate(const int32_t *trace, uint32_t count, int nch,
                      uint32_t strike, float fs,
                      float f0, float f1, float dur,
                      int32_t *corr)
{
    uint32_t np = (uint32_t)(dur * fs);
    static float pilot[65536];
    double ep = 0.0;
    for (uint32_t n = 0; n < np && n < 65536; n++)
    {
        pilot[n] = WOK_HAL_Src_PilotSample((float)n / fs, f0, f1, dur);
        ep += (double)pilot[n] * pilot[n];
    }
    if (ep <= 0.0) ep = 1.0;
    int lead = (int)(g_lead_frac / (f1 - f0) * fs + 0.5f);

    for (uint32_t j = 0; j < WIN; j++)
    {
        int64_t base = (int64_t)strike - PRE + (int64_t)j - lead;
        for (int slot = 0; slot < nch; slot++)
        {
            double acc = 0.0;
            for (uint32_t n = 0; n < np; n++)
            {
                int64_t i = base + n;
                if (i >= 0 && i < (int64_t)count)
                    acc += (double)trace[i * nch + slot] * pilot[n];
            }
            corr[j * nch + slot] = (int32_t)(acc / ep * g_corr_gain);
        }
    }
}

/* Accumulate one record's [-PRE,+POST) window around its strike into the
 * stack (mirrors WOK_Seis_AccumulateStack, minus coherence). */
static void accumulate(const int32_t *trace, uint32_t count, int nch,
                       uint32_t strike, int32_t *stack)
{
    if (strike < PRE || strike + POST > count) { printf("  !! stack window out of trace\n"); return; }
    uint32_t t0 = strike - PRE;
    for (uint32_t s = 0; s < (uint32_t)WIN * nch; s++)
        stack[s] += trace[t0 * nch + s];
}

/* ==================== fast synthetic generation ===================== */
/* Replicates wok_hal_sim_seis.c synth_sample without the wall clock.   */

static float synth(int mode, float site_x, float site_y, int ch,
                   uint32_t s, uint32_t strike, float force,
                   float f0, float f1, float dur)
{
    float val = ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f * 80.0f;
    if (s >= strike)
    {
        float t0    = (float)strike / FS;
        float t     = (float)s / FS;
        float off   = PODS[ch];
        float t_arr = WOK_SimGeology_FirstArrivalSec(site_x, site_y, off);
        float dt    = t - (t0 + t_arr);
        if (dt >= 0.0f)
        {
            float amp = force * 40000.0f / (off * off + 1.0f);
            if (mode == WOK_HAL_SRC_MODE_SWEEP)
                val += amp * WOK_HAL_Src_PilotSample(dt, f0, f1, dur);
            else
                val += amp * expf(-dt / 0.15f)
                           * sinf(2.0f * 3.14159265f * 25.0f * dt);
        }
    }
    return val;
}

#define SYN_DUR_S   10
#define SYN_COUNT   ((uint32_t)(SYN_DUR_S * FS))
#define SYN_STRIKE  ((uint32_t)(2.0f * FS))

static int32_t g_trace[SYN_COUNT * NCH];
static int32_t g_corr[WIN * NCH];
static int32_t g_stack_imp[WIN * NCH];
static int32_t g_stack_swp[WIN * NCH];

static void gen_record(int mode, float sx, float sy, float force,
                       float f0, float f1, float dur)
{
    for (uint32_t s = 0; s < SYN_COUNT; s++)
        for (int ch = 0; ch < NCH; ch++)
            g_trace[s * NCH + ch] =
                (int32_t)synth(mode, sx, sy, ch, s, SYN_STRIKE, force,
                               f0, f1, dur);
}

static int tune_site(float sx, float sy, int folds,
                     float imp_force, float swp_force,
                     float f0, float f1, float dur)
{
    float fb_imp[NCH], fb_swp[NCH];
    memset(g_stack_imp, 0, sizeof(g_stack_imp));
    memset(g_stack_swp, 0, sizeof(g_stack_swp));

    for (int k = 0; k < folds; k++)
    {
        gen_record(WOK_HAL_SRC_MODE_IMPULSE, sx, sy, imp_force, 0, 0, 0);
        accumulate(g_trace, SYN_COUNT, NCH, SYN_STRIKE, g_stack_imp);
        gen_record(WOK_HAL_SRC_MODE_SWEEP, sx, sy, swp_force, f0, f1, dur);
        correlate(g_trace, SYN_COUNT, NCH, SYN_STRIKE, FS, f0, f1, dur, g_corr);
        accumulate(g_corr, WIN, NCH, PRE, g_stack_swp);
    }
    run_picker(g_stack_imp, NCH, FS, fb_imp);
    run_picker(g_stack_swp, NCH, FS, fb_swp);

    int pass = 1;
    printf("  site (%.1f,%.1f):\n", (double)sx, (double)sy);
    for (int ch = 0; ch < NCH; ch++)
    {
        float truth = WOK_SimGeology_FirstArrivalSec(sx, sy, PODS[ch]) * 1000.0f;
        float d = fb_swp[ch] - fb_imp[ch];
        int ok = (fb_imp[ch] >= 0 && fb_swp[ch] >= 0 && fabsf(d) <= 0.5f);
        if (!ok) pass = 0;
        printf("    pod %d (%.1fm): truth %6.2f  imp %6.2f  swp %6.2f  "
               "delta %+5.2f ms  %s\n",
               ch, (double)PODS[ch], (double)truth,
               (double)fb_imp[ch], (double)fb_swp[ch], (double)d,
               ok ? "OK" : "FAIL");
    }
    return pass;
}

static int do_tune(void)
{
    printf("=== TUNE: synthetic impulse-vs-sweep pick agreement ===\n");
    printf("  taper %.2f, lead %.2f, gain %.0f, force %.3fN, %g-%g Hz %gs\n",
           (double)WOK_HAL_SRC_PILOT_TAPER_FRAC, (double)g_lead_frac,
           (double)g_corr_gain, (double)g_swp_force,
           (double)WOK_SRC_DEFAULT_SWEEP_F0_HZ,
           (double)WOK_SRC_DEFAULT_SWEEP_F1_HZ,
           (double)WOK_SRC_DEFAULT_SWEEP_DUR_S);
    int pass = 1;
    /* Dry site + lens site (the two geology regimes) */
    pass &= tune_site(0.0f, 0.0f, 3, 9.0f, g_swp_force,
                      WOK_SRC_DEFAULT_SWEEP_F0_HZ,
                      WOK_SRC_DEFAULT_SWEEP_F1_HZ,
                      WOK_SRC_DEFAULT_SWEEP_DUR_S);
    pass &= tune_site(6.0f, 3.0f, 3, 9.0f, g_swp_force,
                      WOK_SRC_DEFAULT_SWEEP_F0_HZ,
                      WOK_SRC_DEFAULT_SWEEP_F1_HZ,
                      WOK_SRC_DEFAULT_SWEEP_DUR_S);
    printf("TUNE %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

/* ========================= HAL-path test ============================ */

static void msleep(int ms)
{
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* Run one full shot through the real sim HAL: start acq, arm, fire,
 * drain until complete. Returns strike idx; trace in g_trace. */
static uint32_t hal_shot(int mode, float energy, float f0, float f1,
                         float dur, uint16_t acq_dur_s, uint32_t *count)
{
    uint8_t st = 0, acq = 1;
    uint32_t samp = 0, actual = 0, total = 0;

    if (WOK_HAL_Src_Configure((uint8_t)mode, energy, f0, f1, dur) != WOK_HAL_OK)
        printf("  !! configure failed\n");
    WOK_HAL_Seis_StartAcquisition(acq_dur_s);
    if (WOK_HAL_Src_Arm() != WOK_HAL_OK) printf("  !! arm failed\n");

    /* Poll to ARMED then fire (SEQ does this on HK ticks) */
    int guard = 0;
    do {
        msleep(50);
        WOK_HAL_Src_GetState(&st);
    } while (st != WOK_HAL_SRC_ARMED && ++guard < 600);
    if (st != WOK_HAL_SRC_ARMED) { printf("  !! never ARMED\n"); return 0; }
    if (WOK_HAL_Src_Fire() != WOK_HAL_OK) printf("  !! fire failed\n");

    /* Drain until acquisition completes (mirrors the app's HK drain) */
    static int32_t chunk[4096];
    while (1)
    {
        WOK_HAL_Seis_GetStatus(&acq, &samp);
        while (WOK_HAL_Seis_GetSamples(chunk, 4096, &actual) == WOK_HAL_OK &&
               actual > 0)
        {
            uint32_t n = actual / NCH;
            if ((total + n) * NCH <= sizeof(g_trace) / sizeof(g_trace[0]))
                memcpy(&g_trace[total * NCH], chunk, actual * sizeof(int32_t));
            total += n;
        }
        if (!acq) break;
        msleep(100);
    }
    WOK_HAL_Src_GetState(&st);
    if (st == WOK_HAL_SRC_FIRED) WOK_HAL_Src_Reset();
    else                         WOK_HAL_Src_Abort();

    uint32_t strike = 0; uint8_t valid = 0;
    WOK_HAL_Seis_GetStrikeInfo(&strike, &valid);
    if (!valid) printf("  !! no strike latched\n");
    *count = total;
    return strike;
}

static int hal_site(float sx, float sy)
{
    float fb_imp[NCH], fb_swp[NCH];
    uint32_t count = 0, strike;
    memset(g_stack_imp, 0, sizeof(g_stack_imp));
    memset(g_stack_swp, 0, sizeof(g_stack_swp));

    WOK_HAL_Seis_SetSite(sx, sy, 0.0f);
    WOK_HAL_Seis_Configure((uint16_t)FS, 1, 0x0F);

    printf("  site (%.1f,%.1f): 3 impulse shots...\n", (double)sx, (double)sy);
    for (int k = 0; k < 3; k++)
    {
        strike = hal_shot(WOK_HAL_SRC_MODE_IMPULSE, WOK_SRC_DEFAULT_ENERGY_J,
                          0, 0, 0, 6, &count);
        accumulate(g_trace, count, NCH, strike, g_stack_imp);
    }
    printf("  site (%.1f,%.1f): 3 sweep shots...\n", (double)sx, (double)sy);
    for (int k = 0; k < 3; k++)
    {
        strike = hal_shot(WOK_HAL_SRC_MODE_SWEEP, 0,
                          WOK_SRC_DEFAULT_SWEEP_F0_HZ,
                          WOK_SRC_DEFAULT_SWEEP_F1_HZ,
                          WOK_SRC_DEFAULT_SWEEP_DUR_S,
                          (uint16_t)(6 + WOK_SRC_SWEEP_MAX_DUR_SEC), &count);
        correlate(g_trace, count, NCH, strike, FS,
                  WOK_SRC_DEFAULT_SWEEP_F0_HZ, WOK_SRC_DEFAULT_SWEEP_F1_HZ,
                  WOK_SRC_DEFAULT_SWEEP_DUR_S, g_corr);
        accumulate(g_corr, WIN, NCH, PRE, g_stack_swp);
    }
    run_picker(g_stack_imp, NCH, FS, fb_imp);
    run_picker(g_stack_swp, NCH, FS, fb_swp);

    int pass = 1;
    for (int ch = 0; ch < NCH; ch++)
    {
        float truth = WOK_SimGeology_FirstArrivalSec(sx, sy, PODS[ch]) * 1000.0f;
        float d = fb_swp[ch] - fb_imp[ch];
        int ok = (fb_imp[ch] >= 0 && fb_swp[ch] >= 0 && fabsf(d) <= 0.5f);
        if (!ok) pass = 0;
        printf("    pod %d: truth %6.2f  imp %6.2f  swp %6.2f  delta %+5.2f ms  %s\n",
               ch, (double)truth, (double)fb_imp[ch], (double)fb_swp[ch],
               (double)d, ok ? "OK" : "FAIL");
    }
    return pass;
}

static int do_hal(void)
{
    printf("=== HAL: end-to-end impulse-vs-sweep picks (+-0.5 ms gate) ===\n");
    WOK_HAL_Seis_Init();
    WOK_HAL_Src_Init();
    int pass = 1;
    pass &= hal_site(0.0f, 0.0f);
    pass &= hal_site(6.0f, 3.0f);
    printf("HAL %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

/* ========================= bank-model test ========================== */

static int do_bank(void)
{
    printf("=== BANK: charge timing, deplete/recharge, low-battery stall ===\n");
    int pass = 1;
    uint8_t st; float bv, soc;
    WOK_HAL_Seis_Init();
    WOK_HAL_Src_Init();

    /* 1. Nominal charge: 50 J at 50 W -> ~1 s to ARMED */
    WOK_HAL_Src_Configure(WOK_HAL_SRC_MODE_IMPULSE, 50.0f, 0, 0, 0);
    WOK_HAL_Src_Arm();
    msleep(400);
    WOK_HAL_Src_GetState(&st);
    WOK_HAL_Src_GetBank(&bv, &soc);
    printf("  t=0.4s: state %u soc %.0f%% bank %.0fV %s\n", st, (double)soc,
           (double)bv, (st == WOK_HAL_SRC_CHARGING && soc > 10 && soc < 90)
                       ? "OK" : "FAIL");
    if (!(st == WOK_HAL_SRC_CHARGING && soc > 10 && soc < 90)) pass = 0;
    msleep(900);
    WOK_HAL_Src_GetState(&st);
    WOK_HAL_Src_GetBank(&bv, &soc);
    printf("  t=1.3s: state %u soc %.0f%% %s\n", st, (double)soc,
           (st == WOK_HAL_SRC_ARMED && soc >= 99.5f) ? "OK" : "FAIL");
    if (!(st == WOK_HAL_SRC_ARMED && soc >= 99.5f)) pass = 0;

    /* 2. Fire depletes the bank; re-arm recharges */
    WOK_HAL_Seis_StartAcquisition(3);
    WOK_HAL_Src_Fire();
    WOK_HAL_Src_GetState(&st);
    WOK_HAL_Src_GetBank(&bv, &soc);
    printf("  fired: state %u soc %.0f%% %s\n", st, (double)soc,
           (st == WOK_HAL_SRC_FIRED && soc < 5.0f) ? "OK" : "FAIL");
    if (!(st == WOK_HAL_SRC_FIRED && soc < 5.0f)) pass = 0;
    WOK_HAL_Src_Reset();
    WOK_HAL_Src_Arm();
    msleep(500);
    WOK_HAL_Src_GetBank(&bv, &soc);
    WOK_HAL_Src_GetState(&st);
    printf("  re-arm t=0.5s: state %u soc %.0f%% %s (recharge under way)\n",
           st, (double)soc,
           (st == WOK_HAL_SRC_CHARGING && soc > 10.0f) ? "OK" : "FAIL");
    if (!(st == WOK_HAL_SRC_CHARGING && soc > 10.0f)) pass = 0;
    WOK_HAL_Src_Abort();
    WOK_HAL_Src_GetState(&st);
    WOK_HAL_Src_GetBank(&bv, &soc);
    printf("  abort: state %u soc %.0f%% %s (safe dump)\n", st, (double)soc,
           (st == WOK_HAL_SRC_IDLE && soc < 1.0f) ? "OK" : "FAIL");
    if (!(st == WOK_HAL_SRC_IDLE && soc < 1.0f)) pass = 0;

    /* 3. Low battery: charge inhibited -> never ARMED (arm timeout case) */
    WOK_HAL_SimPower_SetBattery(22.5f);   /* above SAFE 22.0, below 23.0 */
    WOK_HAL_Src_Arm();
    msleep(2000);
    WOK_HAL_Src_GetState(&st);
    WOK_HAL_Src_GetBank(&bv, &soc);
    printf("  batt 22.5V, t=2.0s: state %u soc %.0f%% %s (charge stalled)\n",
           st, (double)soc,
           (st == WOK_HAL_SRC_CHARGING && soc < 1.0f) ? "OK" : "FAIL");
    if (!(st == WOK_HAL_SRC_CHARGING && soc < 1.0f)) pass = 0;

    /* 4. Battery restored: charge resumes to ARMED */
    WOK_HAL_SimPower_SetBattery(25.2f);
    msleep(1400);
    WOK_HAL_Src_GetState(&st);
    printf("  batt restored, t=+1.4s: state %u %s (charge resumed)\n", st,
           (st == WOK_HAL_SRC_ARMED) ? "OK" : "FAIL");
    if (st != WOK_HAL_SRC_ARMED) pass = 0;
    WOK_HAL_Src_Abort();

    printf("BANK %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

/* ====================== coupling-model test ========================= */

/* Signal RMS in [t_arr, t_arr+0.5 s] after the strike, per channel,
 * from the trace hal_shot leaves in g_trace. */
static void signal_rms(uint32_t strike, float sx, float sy, uint32_t count,
                       float rms[NCH])
{
    for (int ch = 0; ch < NCH; ch++)
    {
        float t_arr  = WOK_SimGeology_FirstArrivalSec(sx, sy, PODS[ch]);
        uint32_t s0  = strike + (uint32_t)(t_arr * FS);
        uint32_t s1  = s0 + (uint32_t)(0.5f * FS);
        if (s1 > count) s1 = count;
        double acc = 0.0; uint32_t n = 0;
        for (uint32_t s = s0; s < s1; s++, n++)
        {
            double v = (double)g_trace[s * NCH + ch];
            acc += v * v;
        }
        rms[ch] = (n > 0) ? sqrtf((float)(acc / n)) : 0.0f;
    }
}

/* One impulse shot at a site; returns per-channel signal RMS normalized
 * by the shot's reported plate force (removes the per-shot force jitter). */
static int coupled_shot_rms(float sx, float sy, float rms_per_n[NCH])
{
    uint32_t count = 0, strike;
    float force = 0.0f; uint32_t ts;
    WOK_HAL_Seis_SetSite(sx, sy, 0.0f);
    WOK_HAL_Seis_Configure((uint16_t)FS, 1, 0x0F);
    strike = hal_shot(WOK_HAL_SRC_MODE_IMPULSE, WOK_SRC_DEFAULT_ENERGY_J,
                      0, 0, 0, 6, &count);
    if (count == 0) return 0;
    WOK_HAL_Src_GetShotInfo(&force, &ts);
    if (force <= 0.0f) return 0;
    signal_rms(strike, sx, sy, count, rms_per_n);
    for (int ch = 0; ch < NCH; ch++) rms_per_n[ch] /= force;
    return 1;
}

#define COUP_NOM_N   400            /* nominal-ground sample positions   */
#define COUP_GOOD    0.90f          /* good-seat coupling threshold      */

static int do_coupling(void)
{
    printf("=== COUPLING: truth model + WOK_SIM_COUPLING sim wiring ===\n");
    int pass = 1;

    /* 1. Determinism + attempt independence */
    {
        int det = 1, varies = 0;
        const float px[3] = {0.7f, 4.5f, -2.3f}, py[3] = {1.1f, 0.0f, 5.9f};
        for (int i = 0; i < 3; i++)
            for (uint8_t a = 0; a < 4; a++)
            {
                if (WOK_SimCoupling_PodCoupling(px[i], py[i], a)
                        != WOK_SimCoupling_PodCoupling(px[i], py[i], a) ||
                    WOK_SimCoupling_SeatForceN(px[i], py[i], a)
                        != WOK_SimCoupling_SeatForceN(px[i], py[i], a) ||
                    WOK_SimCoupling_PlateCoupling(px[i], py[i], a)
                        != WOK_SimCoupling_PlateCoupling(px[i], py[i], a))
                    det = 0;
                if (a > 0 &&
                    WOK_SimCoupling_PodCoupling(px[i], py[i], a)
                        != WOK_SimCoupling_PodCoupling(px[i], py[i], 0))
                    varies = 1;
            }
        printf("  determinism: %s, attempts vary: %s\n",
               det ? "OK" : "FAIL", varies ? "OK" : "FAIL");
        if (!det || !varies) pass = 0;
    }

    /* 2. Seat statistics: nominal ground vs the fluff patch; reseat helps */
    {
        int n_nom = 0, good0_nom = 0, good3_nom = 0;
        for (int i = 0; i < 20; i++)
            for (int j = 0; j < 20; j++)
            {
                float x = 10.0f + 0.5f * i, y = 10.0f + 0.5f * j;
                float c0 = WOK_SimCoupling_PodCoupling(x, y, 0);
                float cm = c0;
                for (uint8_t a = 1; a < 3; a++)
                {
                    float c = WOK_SimCoupling_PodCoupling(x, y, a);
                    if (c > cm) cm = c;
                }
                n_nom++;
                if (c0 >= COUP_GOOD) good0_nom++;
                if (cm >= COUP_GOOD) good3_nom++;
            }
        float r0 = (float)good0_nom / n_nom, r3 = (float)good3_nom / n_nom;
        int ok = (r0 >= 0.75f && r0 <= 0.95f && r3 >= 0.97f);
        printf("  nominal ground (%d pos): good at attempt0 %.2f "
               "(expect ~0.85), within 3 attempts %.2f (>=0.97) %s\n",
               n_nom, (double)r0, (double)r3, ok ? "OK" : "FAIL");
        if (!ok) pass = 0;

        int n_pat = 0, good0_pat = 0, good3_pat = 0;
        for (int i = 0; i < 20; i++)
            for (int j = 0; j < 20; j++)
            {
                float x = 1.5f + 0.05f * i, y = 4.5f + 0.05f * j;
                float c0 = WOK_SimCoupling_PodCoupling(x, y, 0);
                float cm = c0;
                for (uint8_t a = 1; a < 3; a++)
                {
                    float c = WOK_SimCoupling_PodCoupling(x, y, a);
                    if (c > cm) cm = c;
                }
                n_pat++;
                if (c0 >= COUP_GOOD) good0_pat++;
                if (cm >= COUP_GOOD) good3_pat++;
            }
        float p0 = (float)good0_pat / n_pat, p3 = (float)good3_pat / n_pat;
        int ok2 = (p0 <= 0.45f && (p3 - p0) >= 0.15f);
        printf("  fluff patch (2,5) (%d pos): good at attempt0 %.2f "
               "(<=0.45), within 3 %.2f (reseat gain >=0.15) %s\n",
               n_pat, (double)p0, (double)p3, ok2 ? "OK" : "FAIL");
        if (!ok2) pass = 0;
        printf("  quality field: nominal %.2f, patch1 pod spot (4.5,0) "
               "%.2f, patch2 centre (2,5) %.2f\n",
               (double)WOK_SimCoupling_SeatQuality(15.0f, 15.0f),
               (double)WOK_SimCoupling_SeatQuality(4.5f, 0.0f),
               (double)WOK_SimCoupling_SeatQuality(2.0f, 5.0f));
    }

    /* 3. Seating-force proxy: correlated with coupling, but imperfect */
    {
        double sc = 0, sf = 0, scc = 0, sff = 0, scf = 0;
        int n = 0, n_bad = 0, bad_acc = 0, n_good = 0, good_rej = 0;
        const float thr_n = 26.0f;
        for (int i = 0; i < 40; i++)
            for (uint8_t a = 0; a < 4; a++)
            {
                /* half nominal ground, half fluff patch */
                float x = (i < 20) ? 10.0f + 0.5f * i : 1.5f + 0.05f * (i - 20);
                float y = (i < 20) ? 12.0f : 5.0f;
                float c = WOK_SimCoupling_PodCoupling(x, y, a);
                float f = WOK_SimCoupling_SeatForceN(x, y, a);
                sc += c; sf += f; scc += c * c; sff += f * f; scf += c * f;
                n++;
                if (c < COUP_GOOD)  { n_bad++;  if (f >= thr_n) bad_acc++; }
                else                { n_good++; if (f <  thr_n) good_rej++; }
            }
        double corr = (n * scf - sc * sf)
                    / (sqrt(n * scc - sc * sc) * sqrt(n * sff - sf * sf));
        float fa = n_bad  ? (float)bad_acc  / n_bad  : 0.0f;
        float fr = n_good ? (float)good_rej / n_good : 0.0f;
        int ok = (corr >= 0.5 && corr <= 0.995 && fa > 0.0f && fa < 0.5f);
        printf("  force proxy (%d draws): corr %.2f (0.5..0.995), "
               "false-accept %.2f at %.0fN (0<..<0.5), false-reject %.2f %s\n",
               n, corr, (double)fa, (double)thr_n, (double)fr,
               ok ? "OK" : "FAIL");
        if (!ok) pass = 0;
    }

    /* 4. End-to-end through the real sim HAL: per-channel amplitude
     *    ratio coupled/uncoupled must equal plate x pod coupling. */
    {
        float ref_nom[NCH], ref_pat[NCH], cpl_nom[NCH], cpl_pat[NCH];

        setenv("WOK_SIM_COUPLING", "0", 1);
        WOK_HAL_Seis_Init(); WOK_HAL_Src_Init();
        printf("  HAL reference shots (coupling off)...\n");
        if (!coupled_shot_rms(10.0f, 10.0f, ref_nom) ||
            !coupled_shot_rms(3.0f, 0.0f, ref_pat))
        { printf("  !! reference shots failed\n"); return 1; }

        /* env-off sanity: RMS follows pure geometric spreading (no
         * coupling leak) — rms*(off^2+1) flat across channels */
        float base = ref_pat[0] * (PODS[0] * PODS[0] + 1.0f);
        int flat = 1;
        for (int ch = 1; ch < NCH; ch++)
        {
            float v = ref_pat[ch] * (PODS[ch] * PODS[ch] + 1.0f);
            if (fabsf(v - base) / base > 0.15f) flat = 0;
        }
        printf("  coupling-off spreading shape: %s\n", flat ? "OK" : "FAIL");
        if (!flat) pass = 0;

        setenv("WOK_SIM_COUPLING", "1", 1);
        WOK_HAL_Seis_Init(); WOK_HAL_Src_Init();
        printf("  HAL coupled shots (WOK_SIM_COUPLING=1)...\n");
        if (!coupled_shot_rms(10.0f, 10.0f, cpl_nom) ||
            !coupled_shot_rms(3.0f, 0.0f, cpl_pat))
        { printf("  !! coupled shots failed\n"); return 1; }

        for (int site = 0; site < 2; site++)
        {
            float sx = site ? 3.0f : 10.0f, sy = site ? 0.0f : 10.0f;
            float *ref = site ? ref_pat : ref_nom;
            float *cpl = site ? cpl_pat : cpl_nom;
            float plate = WOK_SimCoupling_PlateCoupling(sx, sy, 0);
            printf("  site (%.0f,%.0f): plate %.2f\n",
                   (double)sx, (double)sy, (double)plate);
            for (int ch = 0; ch < NCH; ch++)
            {
                float pod   = WOK_SimCoupling_PodCoupling(sx + PODS[ch], sy, 0);
                float want  = plate * pod;
                float got   = (ref[ch] > 0.0f) ? cpl[ch] / ref[ch] : -1.0f;
                int ok = (got > 0.0f && fabsf(got - want) <= 0.12f);
                if (!ok) pass = 0;
                printf("    pod %d (%.1fm): model %.2f  measured %.2f  %s\n",
                       ch, (double)PODS[ch], (double)want, (double)got,
                       ok ? "OK" : "FAIL");
            }
        }
    }

    printf("COUPLING %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

/* ================= plate reseat (CP-2) ============================== */

/* Attempts needed until a good plate seat at (x,y); 255 if none by k=3 */
static int plate_first_good(float x, float y)
{
    for (int a = 0; a < 3; a++)
        if (WOK_SimCoupling_PlateCoupling(x, y, (uint8_t)a) >= 0.90f)
            return a;
    return 255;
}

static int do_plate(int argc, char **argv)
{
    if (argc > 2 && strcmp(argv[2], "at") == 0 && argc > 4)
    {
        float x = (float)atof(argv[3]), y = (float)atof(argv[4]);
        printf("plate draws at (%.2f, %.2f), quality %.2f, "
               "pod0 (heading 0) %.2f:\n",
               (double)x, (double)y,
               (double)WOK_SimCoupling_SeatQuality(x, y),
               (double)WOK_SimCoupling_PodCoupling(x + PODS[0], y, 0));
        for (int a = 0; a < 4; a++)
            printf("  attempt %d: plate %.2f\n", a,
                   (double)WOK_SimCoupling_PlateCoupling(x, y, (uint8_t)a));
        return 0;
    }

    if (argc > 2 && strcmp(argv[2], "scan") == 0)
    {
        /* Fluff patch 2 area, classified against the FSW's actual
         * trigger: with a healthy baseline (~0.96 plate x good pod0)
         * the 0.70 fraction trips near plate < ~0.65. A useful mission
         * target also needs pod0 well-seated, so the ratio drop is
         * attributable to the plate and a plate reseat can recover it. */
        int n = 0, no_trig = 0, rec1 = 0, rec2 = 0, exh = 0, badpod = 0;
        float ex_x = 0, ex_y = 0, r1_x = 0, r1_y = 0;
        for (int i = 0; i < 40; i++)
            for (int j = 0; j < 40; j++)
            {
                float x = 1.0f + 0.05f * i, y = 4.0f + 0.05f * j;
                n++;
                if (WOK_SimCoupling_PodCoupling(x + PODS[0], y, 0) < 0.90f)
                { badpod++; continue; }
                if (WOK_SimCoupling_PlateCoupling(x, y, 0) >= 0.65f)
                { no_trig++; continue; }
                if (WOK_SimCoupling_PlateCoupling(x, y, 1) >= 0.85f)
                { rec1++; r1_x = x; r1_y = y; }
                else if (WOK_SimCoupling_PlateCoupling(x, y, 2) >= 0.85f)
                    rec2++;
                else
                { exh++; ex_x = x; ex_y = y; }
            }
        printf("patch2 scan (%d pos): pod0-bad %.2f, no-trigger %.2f, "
               "trigger->recover@1 %.2f, @2 %.2f, exhaust %.2f\n", n,
               (double)badpod / n, (double)no_trig / n, (double)rec1 / n,
               (double)rec2 / n, (double)exh / n);
        printf("  example recover@1: (%.2f, %.2f)   exhaust: (%.2f, %.2f)\n",
               (double)r1_x, (double)r1_y, (double)ex_x, (double)ex_y);
        return 0;
    }

    /* Default: end-to-end reseat physics through the real sim HAL */
    printf("=== PLATE: reseat redraw + seat-attempt persistence (HAL) ===\n");
    int pass = 1;

    /* Pick a position where attempt 0 is bad and attempt 1 is good */
    float px = 0, py = 0;
    for (int i = 0; i < 1600 && px == 0; i++)
    {
        float x = 1.0f + 0.05f * (i / 40), y = 4.0f + 0.05f * (i % 40);
        if (plate_first_good(x, y) == 1) { px = x; py = y; }
    }
    if (px == 0) { printf("  !! no recover@1 position found\n"); return 1; }
    float p0 = WOK_SimCoupling_PlateCoupling(px, py, 0);
    float p1 = WOK_SimCoupling_PlateCoupling(px, py, 1);
    printf("  site (%.2f,%.2f): plate truth %.2f -> %.2f after reseat\n",
           (double)px, (double)py, (double)p0, (double)p1);

    setenv("WOK_SIM_COUPLING", "1", 1);
    WOK_HAL_Seis_Init(); WOK_HAL_Src_Init();
    float rms_a[NCH], rms_b[NCH], rms_c[NCH];

    if (!coupled_shot_rms(px, py, rms_a)) { printf("  !! shot A failed\n"); return 1; }
    if (WOK_HAL_Src_ReseatPlate() != WOK_HAL_OK)
        { printf("  !! reseat rejected\n"); pass = 0; }
    /* Same pose again — SetSite must NOT reset the seat attempt */
    if (!coupled_shot_rms(px, py, rms_b)) { printf("  !! shot B failed\n"); return 1; }

    float got = rms_b[0] / rms_a[0], want = p1 / p0;
    int ok = fabsf(got - want) / want <= 0.10f;
    if (!ok) pass = 0;
    printf("  near-pod ratio after reseat: model %.2f  measured %.2f  %s\n",
           (double)want, (double)got, ok ? "OK" : "FAIL");

    /* New pose resets the seat attempt (fresh seating campaign) */
    float q0 = WOK_SimCoupling_PlateCoupling(10.0f, 10.0f, 0);
    if (!coupled_shot_rms(10.0f, 10.0f, rms_c)) { printf("  !! shot C failed\n"); return 1; }
    float got2 = rms_c[0] / rms_a[0]
               * (WOK_SimCoupling_PodCoupling(px + PODS[0], py, 0)
                  / WOK_SimCoupling_PodCoupling(10.0f + PODS[0], 10.0f, 0));
    float want2 = q0 / p0;
    int ok2 = fabsf(got2 - want2) / want2 <= 0.12f;
    if (!ok2) pass = 0;
    printf("  new-site plate attempt reset: model %.2f  measured %.2f  %s\n",
           (double)want2, (double)got2, ok2 ? "OK" : "FAIL");

    printf("PLATE %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

/* ================= per-pod placement (CP-3) ========================= */

#include "wok_hal_geo.h"

/* Poll a pod to a target HAL state; 0 on timeout */
static int pod_wait(uint8_t pod, uint8_t want, int max_ms)
{
    uint8_t st = 0xFF;
    for (int t = 0; t < max_ms; t += 100)
    {
        WOK_HAL_Geo_GetPodState(pod, &st);
        if (st == want) return 1;
        msleep(100);
    }
    return 0;
}

static int do_pods(void)
{
    printf("=== PODS: per-pod place/seat-force/reseat/retrieve (HAL) ===\n");
    int pass = 1;
    float force; uint8_t att, st;

    setenv("WOK_SIM_COUPLING", "1", 1);
    WOK_HAL_Geo_Init();

    /* Invalid transitions rejected */
    int rej = (WOK_HAL_Geo_RetrievePod(0) != WOK_HAL_OK &&
               WOK_HAL_Geo_ReseatPod(0)   != WOK_HAL_OK &&
               WOK_HAL_Geo_PlacePod(9, 0, 0) != WOK_HAL_OK);
    printf("  invalid transitions rejected: %s\n", rej ? "OK" : "FAIL");
    if (!rej) pass = 0;

    /* Place at nominal ground: force must equal the truth model draw */
    if (WOK_HAL_Geo_PlacePod(0, 10.0f, 10.0f) != WOK_HAL_OK)
        { printf("  !! place failed\n"); return 1; }
    WOK_HAL_Geo_GetPodState(0, &st);
    int ok = (st == WOK_HAL_GEO_POD_PLACING) && pod_wait(0, WOK_HAL_GEO_POD_PLACED, 8000);
    WOK_HAL_Geo_GetPodSeating(0, &force, &att);
    float want = WOK_SimCoupling_SeatForceN(10.0f, 10.0f, 0);
    ok = ok && att == 0 && force == want;
    printf("  place (10,10): force %.2fN (truth %.2fN), attempt %u  %s\n",
           (double)force, (double)want, att, ok ? "OK" : "FAIL");
    if (!ok) pass = 0;

    /* Reseat: fresh attempt, force follows the truth redraw */
    if (WOK_HAL_Geo_ReseatPod(0) != WOK_HAL_OK) { printf("  !! reseat failed\n"); pass = 0; }
    ok = pod_wait(0, WOK_HAL_GEO_POD_PLACED, 8000);
    WOK_HAL_Geo_GetPodSeating(0, &force, &att);
    want = WOK_SimCoupling_SeatForceN(10.0f, 10.0f, 1);
    ok = ok && att == 1 && force == want;
    printf("  reseat: force %.2fN (truth %.2fN), attempt %u  %s\n",
           (double)force, (double)want, att, ok ? "OK" : "FAIL");
    if (!ok) pass = 0;

    /* Retrieve: recapture confirmed by POD_STOWED again (CP-6) */
    if (WOK_HAL_Geo_RetrievePod(0) != WOK_HAL_OK) { printf("  !! retrieve failed\n"); pass = 0; }
    ok = pod_wait(0, WOK_HAL_GEO_POD_STOWED, 8000);
    printf("  retrieve: recaptured %s\n", ok ? "OK" : "FAIL");
    if (!ok) pass = 0;

    /* Bad ground: find a position whose attempt-0 force fails the
     * 26 N proxy gate but attempt 1 or 2 passes — the seat the GEO
     * app's internal reseat loop is built to recover */
    float bx = 0, by = 0;
    for (int i = 0; i < 1600 && bx == 0; i++)
    {
        float x = 1.0f + 0.05f * (i / 40), y = 4.0f + 0.05f * (i % 40);
        if (WOK_SimCoupling_SeatForceN(x, y, 0) < 26.0f &&
            (WOK_SimCoupling_SeatForceN(x, y, 1) >= 26.0f ||
             WOK_SimCoupling_SeatForceN(x, y, 2) >= 26.0f))
        { bx = x; by = y; }
    }
    if (bx == 0) { printf("  !! no gate-recoverable position found\n"); pass = 0; }
    else
    {
        WOK_HAL_Geo_PlacePod(1, bx, by);
        pod_wait(1, WOK_HAL_GEO_POD_PLACED, 8000);
        WOK_HAL_Geo_GetPodSeating(1, &force, &att);
        ok = force < 26.0f;
        printf("  bad ground (%.2f,%.2f): attempt-0 force %.2fN < 26N gate  %s\n",
               (double)bx, (double)by, (double)force, ok ? "OK" : "FAIL");
        if (!ok) pass = 0;
    }

    /* Coupling off: nominal firm seat, exactly 30 N */
    setenv("WOK_SIM_COUPLING", "0", 1);
    WOK_HAL_Geo_Init();
    WOK_HAL_Geo_PlacePod(2, 2.0f, 5.0f);   /* even in the fluff patch */
    pod_wait(2, WOK_HAL_GEO_POD_PLACED, 8000);
    WOK_HAL_Geo_GetPodSeating(2, &force, &att);
    ok = (force == 30.0f);
    printf("  coupling off: force %.1fN (nominal 30)  %s\n",
           (double)force, ok ? "OK" : "FAIL");
    if (!ok) pass = 0;

    printf("PODS %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

/* ================= per-pod sync bias (CP-5) ========================= */

/* 3-shot stacked picks at (10,10) through the real sim HAL with the
 * current WOK_SIM_SYNC_ERR_MS environment; fb_ms per pod. */
static int sync_run_picks(float fb_ms[NCH])
{
    uint32_t count = 0, strike;
    WOK_HAL_Seis_Init(); WOK_HAL_Src_Init();
    WOK_HAL_Seis_SetSite(10.0f, 10.0f, 0.0f);
    WOK_HAL_Seis_Configure((uint16_t)FS, 1, 0x0F);
    memset(g_stack_imp, 0, sizeof(g_stack_imp));
    for (int k = 0; k < 3; k++)
    {
        strike = hal_shot(WOK_HAL_SRC_MODE_IMPULSE, WOK_SRC_DEFAULT_ENERGY_J,
                          0, 0, 0, 6, &count);
        if (count == 0) return 0;
        accumulate(g_trace, count, NCH, strike, g_stack_imp);
    }
    run_picker(g_stack_imp, NCH, FS, fb_ms);
    return 1;
}

static int do_sync(void)
{
    printf("=== SYNC: per-pod bias injection + pick-spec budget (HAL) ===\n");
    int pass = 1;
    const float MAG_MS = 0.20f;   /* RF-sync budget under test */
    float fb_ref[NCH], fb_sync[NCH];

    setenv("WOK_SIM_COUPLING", "0", 1);
    setenv("WOK_SIM_SYNC_ERR_MS", "0", 1);
    printf("  reference stack (sync off)...\n");
    if (!sync_run_picks(fb_ref)) { printf("  !! reference run failed\n"); return 1; }

    setenv("WOK_SIM_SYNC_ERR_MS", "0.20", 1);
    printf("  biased stack (WOK_SIM_SYNC_ERR_MS=%.2f)...\n", (double)MAG_MS);
    if (!sync_run_picks(fb_sync)) { printf("  !! biased run failed\n"); return 1; }

    float worst_err = 0.0f;
    for (int ch = 0; ch < NCH; ch++)
    {
        float bias  = WOK_SimCoupling_PodSyncBias(10.0f + PODS[ch], 10.0f) * MAG_MS;
        float delta = fb_sync[ch] - fb_ref[ch];
        float truth = WOK_SimGeology_FirstArrivalSec(10.0f, 10.0f, PODS[ch]) * 1000.0f;
        float err   = fb_sync[ch] - truth - bias;   /* residual pick error */
        float tot   = fb_sync[ch] - truth;          /* total error incl. bias */
        int ok = (fb_ref[ch] >= 0 && fb_sync[ch] >= 0 &&
                  fabsf(delta - bias) <= 0.30f);    /* quantum + refine slack */
        if (!ok) pass = 0;
        if (fabsf(tot) > worst_err) worst_err = fabsf(tot);
        printf("    pod %d: bias %+.2f  measured shift %+.2f  "
               "pick-truth %+.2f ms (spec 0.50)  %s\n",
               ch, (double)bias, (double)delta, (double)tot, ok ? "OK" : "FAIL");
        (void)err;
    }
    printf("  budget: worst |pick - truth| %.2f ms at ±%.2f ms sync bias — "
           "%s the ±0.50 ms spec\n",
           (double)worst_err, (double)MAG_MS,
           worst_err <= 0.50f ? "WITHIN" : "EXCEEDS (finding: tighten the sync budget or the picker onset lag)");

    setenv("WOK_SIM_SYNC_ERR_MS", "0", 1);
    printf("SYNC %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

int main(int argc, char **argv)
{
    srand(argc > 5 ? (unsigned)atoi(argv[5]) : 12345u);
    if (argc > 1 && strcmp(argv[1], "plate") == 0) return do_plate(argc, argv);
    if (argc > 1 && strcmp(argv[1], "pods")  == 0) return do_pods();
    if (argc > 1 && strcmp(argv[1], "sync")  == 0) return do_sync();
    if (argc > 2) g_lead_frac = (float)atof(argv[2]);
    if (argc > 3) g_swp_force = (float)atof(argv[3]);
    if (argc > 4) g_corr_gain = (float)atof(argv[4]);
    if (argc > 1 && strcmp(argv[1], "tune") == 0) return do_tune();
    if (argc > 1 && strcmp(argv[1], "hal")  == 0) return do_hal();
    if (argc > 1 && strcmp(argv[1], "bank") == 0) return do_bank();
    if (argc > 1 && strcmp(argv[1], "coupling") == 0) return do_coupling();
    fprintf(stderr, "usage: %s tune|hal|bank|coupling|pods|sync|plate [scan|at x y]\n", argv[0]);
    return 2;
}
