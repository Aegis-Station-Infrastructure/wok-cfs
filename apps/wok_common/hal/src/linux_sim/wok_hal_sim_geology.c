/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Shared Subsurface Geology Model
** File: wok_common/hal/src/linux_sim/wok_hal_sim_geology.c
**
** See wok_hal_sim_geology.h for the model description. All functions are
** pure so every app's copy of the HAL lib computes identical ground truth.
*/

#include "wok_hal_sim_geology.h"
#include <math.h>

/* ── Model parameters ────────────────────────────────────────────── */

/* Layer 1: dry surface regolith */
#define GEO_L1_VP_MPS        250.0f
#define GEO_L1_THICK_BASE_M    1.5f   /* mean thickness                */
#define GEO_L1_THICK_VAR_M     0.5f   /* undulation amplitude          */

/* Layer 2: regolith, ice-cemented inside the lens */
#define GEO_L2_VP_DRY_MPS    800.0f   /* no ice                        */
#define GEO_L2_VP_PER_WTPCT  350.0f   /* Vp increase per wt% ice       */
#define GEO_L2_BOTTOM_M       12.0f

/* Layer 3: basalt / megaregolith */
#define GEO_L3_VP_MPS       3200.0f
#define GEO_L3_STRENGTH        4.0f

/* Ice lens: Gaussian in (x,y), centred inside the default survey grid
 * (sites at (0,0),(3,0),(6,0),(6,3),(3,3),(0,3)). Peak over WP3 (6,3);
 * WP2 (6,0) and WP4 (3,3) see the flanks; origin is nearly dry. */
#define GEO_LENS_X_M           6.0f
#define GEO_LENS_Y_M           3.0f
#define GEO_LENS_SIGMA_M       2.5f
#define GEO_LENS_PEAK_WTPCT    5.0f
#define GEO_ICE_BACKGROUND_WTPCT 0.003f  /* trace hydration everywhere  */

/* Ice stability: no ice above this depth (too warm/diurnal) */
#define GEO_ICE_TOP_M          1.2f

/* Drilling hardness: ice-cemented regolith is stiffer */
#define GEO_STRENGTH_PER_WTPCT 0.5f

/* ── Internal helpers ────────────────────────────────────────────── */

static float geo_l1_thickness(float x, float y)
{
    return GEO_L1_THICK_BASE_M
         + GEO_L1_THICK_VAR_M * sinf(0.4f * x) * cosf(0.3f * y);
}

/* Lateral ice distribution (wt%) — Gaussian lens + background */
static float geo_lens_wtpct(float x, float y)
{
    float dx = x - GEO_LENS_X_M;
    float dy = y - GEO_LENS_Y_M;
    float r2 = dx * dx + dy * dy;
    float s2 = GEO_LENS_SIGMA_M * GEO_LENS_SIGMA_M;
    return GEO_ICE_BACKGROUND_WTPCT
         + GEO_LENS_PEAK_WTPCT * expf(-r2 / (2.0f * s2));
}

/* ── Public model ────────────────────────────────────────────────── */

void WOK_SimGeology_GetColumn(float x_m, float y_m,
                              WOK_SimGeoLayer_t layers[WOK_SIM_GEO_NLAYERS])
{
    float h1  = geo_l1_thickness(x_m, y_m);
    float ice = geo_lens_wtpct(x_m, y_m);

    layers[0].top_m     = 0.0f;
    layers[0].bottom_m  = h1;
    layers[0].vp_mps    = GEO_L1_VP_MPS;
    layers[0].ice_wtpct = 0.0f;
    layers[0].strength  = 1.0f;

    layers[1].top_m     = h1;
    layers[1].bottom_m  = GEO_L2_BOTTOM_M;
    layers[1].vp_mps    = GEO_L2_VP_DRY_MPS + GEO_L2_VP_PER_WTPCT * ice;
    layers[1].ice_wtpct = ice;
    layers[1].strength  = 1.0f + GEO_STRENGTH_PER_WTPCT * ice;

    layers[2].top_m     = GEO_L2_BOTTOM_M;
    layers[2].bottom_m  = 1.0e6f;
    layers[2].vp_mps    = GEO_L3_VP_MPS;
    layers[2].ice_wtpct = 0.0f;
    layers[2].strength  = GEO_L3_STRENGTH;
}

float WOK_SimGeology_IceWtPctAt(float x_m, float y_m, float depth_m)
{
    float h1 = geo_l1_thickness(x_m, y_m);
    float top = (h1 > GEO_ICE_TOP_M) ? h1 : GEO_ICE_TOP_M;

    if (depth_m < top || depth_m >= GEO_L2_BOTTOM_M)
        return GEO_ICE_BACKGROUND_WTPCT;

    return geo_lens_wtpct(x_m, y_m);
}

float WOK_SimGeology_StrengthAt(float x_m, float y_m, float depth_m)
{
    WOK_SimGeoLayer_t col[WOK_SIM_GEO_NLAYERS];
    WOK_SimGeology_GetColumn(x_m, y_m, col);

    for (int i = 0; i < WOK_SIM_GEO_NLAYERS; i++)
    {
        if (depth_m >= col[i].top_m && depth_m < col[i].bottom_m)
            return col[i].strength;
    }
    return col[WOK_SIM_GEO_NLAYERS - 1].strength;
}

/* Release-mode centres/widths (Celsius) — see header for the physics */
#define GEO_REL_ADSORBED_MU   110.0f
#define GEO_REL_ADSORBED_SIG   20.0f
#define GEO_REL_PORE_MU       160.0f
#define GEO_REL_PORE_SIG       25.0f
#define GEO_REL_HYDRATED_MU   320.0f
#define GEO_REL_HYDRATED_SIG   30.0f
#define GEO_LENS_PORE_FRAC      0.85f  /* lens ice: pore vs hydrated  */

/* Logistic CDF approximation of a Gaussian release peak */
static float geo_release_cdf(float t, float mu, float sig)
{
    return 1.0f / (1.0f + expf(-(t - mu) / (0.6f * sig)));
}

float WOK_SimGeology_EvolvedFraction(float x_m, float y_m, float depth_m,
                                     float temp_c)
{
    float total = WOK_SimGeology_IceWtPctAt(x_m, y_m, depth_m);
    if (total <= 0.0f) return 0.0f;

    /* Partition total volatiles into binding components: the background
     * hydration is adsorbed film; anything above it is lens-derived. */
    float adsorbed = (total < GEO_ICE_BACKGROUND_WTPCT)
                   ? total : GEO_ICE_BACKGROUND_WTPCT;
    float lens     = total - adsorbed;
    float pore     = lens * GEO_LENS_PORE_FRAC;
    float hydrated = lens * (1.0f - GEO_LENS_PORE_FRAC);

    float evolved =
        adsorbed * geo_release_cdf(temp_c, GEO_REL_ADSORBED_MU, GEO_REL_ADSORBED_SIG)
      + pore     * geo_release_cdf(temp_c, GEO_REL_PORE_MU,     GEO_REL_PORE_SIG)
      + hydrated * geo_release_cdf(temp_c, GEO_REL_HYDRATED_MU, GEO_REL_HYDRATED_SIG);

    return evolved / total;
}

float WOK_SimGeology_FirstArrivalSec(float x_m, float y_m, float offset_m)
{
    WOK_SimGeoLayer_t col[WOK_SIM_GEO_NLAYERS];
    WOK_SimGeology_GetColumn(x_m, y_m, col);

    float v1 = col[0].vp_mps;
    float v2 = col[1].vp_mps;
    float h1 = col[0].bottom_m;
    float x  = fabsf(offset_m);

    /* Direct wave through layer 1 */
    float t_direct = x / v1;

    /* L1/L2 head wave (refraction): t = x/v2 + 2*h1*cos(ic)/v1,
     * sin(ic) = v1/v2. Only exists when v2 > v1. */
    float t_head = t_direct;
    if (v2 > v1)
    {
        float sin_ic = v1 / v2;
        float cos_ic = sqrtf(1.0f - sin_ic * sin_ic);
        t_head = x / v2 + 2.0f * h1 * cos_ic / v1;
    }

    return (t_head < t_direct) ? t_head : t_direct;
}
