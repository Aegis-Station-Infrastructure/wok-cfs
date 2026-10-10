/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Geophone Mechanism Subsystem (WOK_GEO)
** File: wok_common/hal/inc/wok_hal_geo.h
**
** Two contracts coexist (WOK-CP-001, per-pod added at CP-3):
**
**   LEGACY ARRAY MECHANISM — Deploy/Stow/endstops: the original single
**   deploy-stow mechanism. Remains the regression baseline and the
**   fixed-array abstraction for the algorithm chain.
**
**   PER-POD PLACEMENT — the flight baseline: rover-placed wireless
**   pods, one at a time, each seating a confirmed measured condition.
**   PlacePod seats pod `pod` at the given surface position (the
**   rover's pose when the arm sets it down — the position is passed
**   in because placement pose is NAV knowledge, not mechanism
**   knowledge; the sim also uses it to anchor the coupling truth
**   model). ReseatPod lifts and re-presses the same pod at the same
**   spot (a fresh seat attempt). RetrievePod recaptures and stows it;
**   recapture confirmation is the pod reading WOK_HAL_GEO_POD_STOWED
**   again (CP-6). GetPodSeating exposes the seating-force channel —
**   the arm force-feedback PROXY of coupling quality (§4.2): gate on
**   it cheaply, but the active test shot stays the arbiter.
*/

#ifndef WOK_HAL_GEO_H
#define WOK_HAL_GEO_H

#include "wok_hal.h"

/* Mechanism states (legacy array mechanism) */
#define WOK_HAL_GEO_STOWED    0
#define WOK_HAL_GEO_DEPLOYING 1
#define WOK_HAL_GEO_DEPLOYED  2
#define WOK_HAL_GEO_STOWING   3

/* Per-pod states */
#define WOK_HAL_GEO_POD_STOWED     0   /* carried on the rover          */
#define WOK_HAL_GEO_POD_PLACING    1   /* arm motion in progress        */
#define WOK_HAL_GEO_POD_PLACED     2   /* seated on the ground          */
#define WOK_HAL_GEO_POD_RETRIEVING 3   /* recapture in progress         */

#define WOK_HAL_GEO_MAX_PODS  4

int32_t WOK_HAL_Geo_Init(void);
int32_t WOK_HAL_Geo_Deploy(void);
int32_t WOK_HAL_Geo_Stow(void);
int32_t WOK_HAL_Geo_EmergencyStop(void);
int32_t WOK_HAL_Geo_GetState(uint8_t *state);
int32_t WOK_HAL_Geo_GetEndstops(uint8_t *deployed, uint8_t *stowed);
int32_t WOK_HAL_Geo_GetMotorCurrent(float *amps);

/* ── Per-pod placement (CP-3) ────────────────────────────────────── */

/**
 * @brief Place pod `pod` at surface position (x, y) [mission start
 *        frame]. Only valid from POD_STOWED. Non-blocking; poll
 *        GetPodState for POD_PLACED, then read the seating force.
 */
int32_t WOK_HAL_Geo_PlacePod(uint8_t pod, float x_m, float y_m);

/**
 * @brief Lift and re-press pod `pod` at its placed position — a fresh
 *        seat attempt. Only valid from POD_PLACED. Non-blocking; the
 *        pod passes through POD_PLACING again.
 */
int32_t WOK_HAL_Geo_ReseatPod(uint8_t pod);

/**
 * @brief Recapture and stow pod `pod`. Only valid from POD_PLACED.
 *        Non-blocking; POD_STOWED again = recapture confirmed (CP-6).
 */
int32_t WOK_HAL_Geo_RetrievePod(uint8_t pod);

int32_t WOK_HAL_Geo_GetPodState(uint8_t pod, uint8_t *state);

/**
 * @brief Seating diagnostics for pod `pod`: peak arm force of the most
 *        recent seat attempt (N; valid once POD_PLACED) and the seat
 *        attempt counter at this position (0 = first seat).
 */
int32_t WOK_HAL_Geo_GetPodSeating(uint8_t pod, float *seat_force_n,
                                  uint8_t *attempt);

#endif /* WOK_HAL_GEO_H */
