/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Shared Subsurface Geology Model
** File: wok_common/hal/src/linux_sim/wok_hal_sim_geology.h
**
** Single source of subsurface truth for the Linux sim. BOTH the seismic
** sim (arrival times, moveout) and the drill sim (torque, ROP, volatiles)
** derive their outputs from this model, so a correct inversion of the
** synthetic seismic data will agree with what the synthetic drill finds.
**
** All functions are PURE (no state): each app links its own copy of the
** HAL lib, so cross-app consistency comes from determinism, not from
** shared memory. Coordinates are the mission start frame (x = forward,
** y = left), the same frame NAV dead-reckons pose in.
**
** Model (3 layers):
**   L1: dry regolith      — Vp ~250 m/s, thickness 1–2 m (undulating)
**   L2: regolith, ice-cemented inside a Gaussian lens — Vp rises with
**       ice content (dry ~800 m/s → ~2500 m/s at peak ice)
**   L3: basalt/megaregolith below 12 m — Vp ~3200 m/s
**
** Sim-internal header: not part of the flight HAL contract.
*/

#ifndef WOK_HAL_SIM_GEOLOGY_H
#define WOK_HAL_SIM_GEOLOGY_H

#include <stdint.h>

#define WOK_SIM_GEO_NLAYERS  3

typedef struct
{
    float top_m;        /* depth to top of layer (m below surface)     */
    float bottom_m;     /* depth to bottom of layer                    */
    float vp_mps;       /* P-wave velocity                             */
    float ice_wtpct;    /* water-equivalent ice content, wt%           */
    float strength;     /* relative drilling hardness (1 = dry regolith) */
} WOK_SimGeoLayer_t;

/* Full layer column at surface position (x,y). */
void  WOK_SimGeology_GetColumn(float x_m, float y_m,
                               WOK_SimGeoLayer_t layers[WOK_SIM_GEO_NLAYERS]);

/* Ice content (wt%) at a point. */
float WOK_SimGeology_IceWtPctAt(float x_m, float y_m, float depth_m);

/* Relative drilling hardness at a point (1.0 = dry regolith). */
float WOK_SimGeology_StrengthAt(float x_m, float y_m, float depth_m);

/* First-break traveltime (seconds) for a surface source at (x,y) and a
 * surface receiver at horizontal offset_m: min(direct L1 wave, L1/L2
 * head wave). This is what the seis sim uses for per-channel moveout. */
float WOK_SimGeology_FirstArrivalSec(float x_m, float y_m, float offset_m);

/* Cumulative fraction (0-1) of the material's total volatiles evolved
 * by the time the vaporization cell reaches temp_c. The release
 * spectrum encodes HOW the water is bound — the commercially critical
 * distinction:
 *   adsorbed film   ~110 C  (background hydration everywhere)
 *   pore ice        ~160 C  (85% of lens-derived ice)
 *   hydrated mineral ~320 C (15% of lens ice; incompletely released
 *                            at the 300 C cell limit — intentional)
 * Ground classifies binding from the (T, ppm) curve in the drill log. */
float WOK_SimGeology_EvolvedFraction(float x_m, float y_m, float depth_m,
                                     float temp_c);

#endif /* WOK_HAL_SIM_GEOLOGY_H */
