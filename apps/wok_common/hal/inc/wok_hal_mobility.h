/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Mobility Subsystem (WOK_NAV)
** File: wok_common/hal/inc/wok_hal_mobility.h
**
** Primitive contract — NAV sequences these into relative-step missions.
** Each call is non-blocking; poll the matching Get* for completion.
**
**   DriveForward(d, spd)  — start straight-line drive of length d
**   Turn(angle, spd)      — start in-place rotation of magnitude |angle|
**   GetDistanceTraveled() — metres traveled since last DriveForward
**   GetHeadingChange()    — degrees turned since last Turn
**   IsMoving()            — 1 while last DriveForward/Turn is in progress
**
** No notion of waypoints, position, or "frames" lives in the HAL — that
** is NAV's job. The HAL just exposes encoder + IMU primitives.
*/

#ifndef WOK_HAL_MOBILITY_H
#define WOK_HAL_MOBILITY_H

#include "wok_hal.h"

int32_t WOK_HAL_Mobility_Init(void);

/* Lifecycle */
int32_t WOK_HAL_Mobility_Stop(void);
int32_t WOK_HAL_Mobility_SetSpeed(float spd);

/* Elementary motion primitives — non-blocking */
int32_t WOK_HAL_Mobility_DriveForward(float dist_m, float spd_mps);
int32_t WOK_HAL_Mobility_Turn(float angle_deg, float spd_dps);

/* Per-primitive progress (zeroed on each new DriveForward / Turn) */
int32_t WOK_HAL_Mobility_GetDistanceTraveled(float *m);
int32_t WOK_HAL_Mobility_GetHeadingChange(float *deg);
int32_t WOK_HAL_Mobility_IsMoving(uint8_t *moving);

/* Continuous sensor reads */
int32_t WOK_HAL_Mobility_GetSpeed(float *spd);
int32_t WOK_HAL_Mobility_GetTilt(float *deg);
int32_t WOK_HAL_Mobility_GetMotorCurrent(float *amps);

#endif /* WOK_HAL_MOBILITY_H */
