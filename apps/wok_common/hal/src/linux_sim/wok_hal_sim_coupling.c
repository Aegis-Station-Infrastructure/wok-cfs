/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Ground-Coupling Truth Model
** File: wok_common/hal/src/linux_sim/wok_hal_sim_coupling.c
**
** See wok_hal_sim_coupling.h for the model description. All functions
** are pure so every app's copy of the HAL lib computes identical truth.
*/

#include "wok_hal_sim_coupling.h"
#include <math.h>

/* ── Model parameters ────────────────────────────────────────────── */

#define COUP_BASE_QUALITY   0.85f  /* good-seat probability, nominal ground */
#define COUP_QUALITY_MIN    0.05f
#define COUP_QUALITY_MAX    0.95f
#define COUP_MICRO_VAR      0.08f  /* clast-scale quality jitter (±)      */
#define COUP_MICRO_CELL_M   0.25f  /* jitter cell size                    */

/* Unconsolidated "fluff" patches: Gaussian quality deficits. Patch 1
 * sits ON the default survey line (sites at y=0/y=3, pods trailing +x)
 * so a default-geometry array test sees one marginal pod position —
 * (4.5, 0) is pod 2 of site (0,0) and pod 0 of site (3,0). Patch 2 is
 * off-line for harness statistics on badly seating ground. */
#define COUP_P1_X_M         4.5f
#define COUP_P1_Y_M         0.0f
#define COUP_P1_SIGMA_M     0.8f
#define COUP_P1_DEPTH       0.60f
#define COUP_P2_X_M         2.0f
#define COUP_P2_Y_M         5.0f
#define COUP_P2_SIGMA_M     1.2f
#define COUP_P2_DEPTH       0.75f

/* Good/bad seat coupling-factor ranges (pod) */
#define COUP_POD_GOOD_MIN   0.90f  /* good: 0.90 + 0.10u                  */
#define COUP_POD_BAD_MIN    0.30f  /* bad:  0.30 + 0.45u                  */
#define COUP_POD_BAD_SPAN   0.45f

/* Plate: rover mass pressing a flat plate seats easier and tighter */
#define COUP_PLATE_Q_BONUS  0.10f
#define COUP_PLATE_Q_MAX    0.98f
#define COUP_PLATE_GOOD_MIN 0.92f  /* good: 0.92 + 0.08u                  */
#define COUP_PLATE_BAD_MIN  0.55f  /* bad:  0.55 + 0.25u                  */
#define COUP_PLATE_BAD_SPAN 0.25f

/* Seating-force proxy: F = NOM * (0.5 + 0.5C) * (1 ± 12%). The jitter
 * term is what keeps the proxy honest-but-imperfect (§4.2): thresholding
 * F alone leaves a false-accept tail for the test shot to catch. */
#define COUP_SEAT_FORCE_NOM_N  30.0f
#define COUP_SEAT_FORCE_JIT    0.12f

/* Hash salts — one independent stream per drawn quantity (the force
 * and plate salts appear inline at their single use sites below) */
#define COUP_SALT_MICRO   0x000C311u
#define COUP_SALT_SEAT    0x0A11CE1u
#define COUP_SALT_SEATC   0x0A11CE2u

/* ── Deterministic hash ──────────────────────────────────────────── */

/* Wang/Jenkins-style integer mix over (a, b, c, salt) -> [0,1).
 * Position inputs are centimetre-quantized so float noise in pose
 * arithmetic cannot flip a draw. */
static uint32_t coup_mix(uint32_t h, uint32_t k)
{
    h ^= k + 0x9E3779B9u + (h << 6) + (h >> 2);
    return h;
}

static float coup_hash01(int32_t a, int32_t b, uint32_t c, uint32_t salt)
{
    uint32_t h = salt;
    h = coup_mix(h, (uint32_t)a);
    h = coup_mix(h, (uint32_t)b);
    h = coup_mix(h, c);
    /* final avalanche */
    h ^= h >> 16; h *= 0x7FEB352Du;
    h ^= h >> 15; h *= 0x846CA68Bu;
    h ^= h >> 16;
    return (float)h / 4294967296.0f;
}

static int32_t coup_cm(float m) { return (int32_t)lroundf(m * 100.0f); }

/* ── Seat quality field ──────────────────────────────────────────── */

static float coup_patch(float x, float y, float px, float py,
                        float sigma, float depth)
{
    float dx = x - px, dy = y - py;
    return depth * expf(-(dx * dx + dy * dy) / (2.0f * sigma * sigma));
}

float WOK_SimCoupling_SeatQuality(float x_m, float y_m)
{
    float q = COUP_BASE_QUALITY
            - coup_patch(x_m, y_m, COUP_P1_X_M, COUP_P1_Y_M,
                         COUP_P1_SIGMA_M, COUP_P1_DEPTH)
            - coup_patch(x_m, y_m, COUP_P2_X_M, COUP_P2_Y_M,
                         COUP_P2_SIGMA_M, COUP_P2_DEPTH);

    /* Clast-scale variation on fixed cells (attempt-independent) */
    int32_t cx = (int32_t)floorf(x_m / COUP_MICRO_CELL_M);
    int32_t cy = (int32_t)floorf(y_m / COUP_MICRO_CELL_M);
    q += (coup_hash01(cx, cy, 0u, COUP_SALT_MICRO) - 0.5f)
       * 2.0f * COUP_MICRO_VAR;

    if (q < COUP_QUALITY_MIN) q = COUP_QUALITY_MIN;
    if (q > COUP_QUALITY_MAX) q = COUP_QUALITY_MAX;
    return q;
}

/* ── Seat draws ──────────────────────────────────────────────────── */

float WOK_SimCoupling_PodCoupling(float x_m, float y_m, uint8_t attempt)
{
    float q  = WOK_SimCoupling_SeatQuality(x_m, y_m);
    float u1 = coup_hash01(coup_cm(x_m), coup_cm(y_m), attempt, COUP_SALT_SEAT);
    float u2 = coup_hash01(coup_cm(x_m), coup_cm(y_m), attempt, COUP_SALT_SEATC);

    if (u1 < q)
        return COUP_POD_GOOD_MIN + (1.0f - COUP_POD_GOOD_MIN) * u2;
    return COUP_POD_BAD_MIN + COUP_POD_BAD_SPAN * u2;
}

float WOK_SimCoupling_SeatForceN(float x_m, float y_m, uint8_t attempt)
{
    float c  = WOK_SimCoupling_PodCoupling(x_m, y_m, attempt);
    float u3 = coup_hash01(coup_cm(x_m), coup_cm(y_m), attempt, 0x0F02CE3u);
    return COUP_SEAT_FORCE_NOM_N * (0.5f + 0.5f * c)
         * (1.0f + (u3 - 0.5f) * 2.0f * COUP_SEAT_FORCE_JIT);
}

float WOK_SimCoupling_PodSyncBias(float x_m, float y_m)
{
    return (coup_hash01(coup_cm(x_m), coup_cm(y_m), 0u, 0x05CC1CBu) - 0.5f)
         * 2.0f;
}

float WOK_SimCoupling_PlateCoupling(float x_m, float y_m, uint8_t attempt)
{
    float q = WOK_SimCoupling_SeatQuality(x_m, y_m) + COUP_PLATE_Q_BONUS;
    if (q > COUP_PLATE_Q_MAX) q = COUP_PLATE_Q_MAX;

    float u1 = coup_hash01(coup_cm(x_m), coup_cm(y_m), attempt, 0x0917E01u);
    float u2 = coup_hash01(coup_cm(x_m), coup_cm(y_m), attempt, 0x0917E02u);

    if (u1 < q)
        return COUP_PLATE_GOOD_MIN + (1.0f - COUP_PLATE_GOOD_MIN) * u2;
    return COUP_PLATE_BAD_MIN + COUP_PLATE_BAD_SPAN * u2;
}
