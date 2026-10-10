/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Mobility internal interface
** File: wok_common/hal/src/linux_sim/wok_hal_sim_mobility.h
**
** Sim-internal header (not part of the flight HAL contract), following
** the wok_hal_sim_power.h pattern: hooks for tests/harnesses to read
** state a real vehicle could not report.
*/

#ifndef WOK_HAL_SIM_MOBILITY_H
#define WOK_HAL_SIM_MOBILITY_H

#include <stdint.h>

/**
 * @brief Ground-truth pose the sim actually drove (start frame, metres
 *        / degrees), including the unsensed per-turn heading bias
 *        (WOK_SIM_TURN_BIAS_DEG). This is what NAV's dead reckoning
 *        diverges FROM — verification only; the apps must never read it.
 *        Updated at motion completion (and at Stop for partial motions).
 */
int32_t WOK_HAL_SimMobility_GetTruePose(float *x_m, float *y_m,
                                        float *heading_deg);

#endif /* WOK_HAL_SIM_MOBILITY_H */
