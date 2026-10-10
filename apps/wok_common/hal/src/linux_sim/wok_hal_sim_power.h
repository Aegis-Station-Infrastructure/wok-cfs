/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Power (sim-internal extensions)
** File: wok_common/hal/src/linux_sim/wok_hal_sim_power.h
**
** Extensions to the power sim that are NOT part of the flight HAL
** contract (pattern: wok_hal_sim_geology.h). Each app links its own
** copy of the HAL lib, so these affect only the calling app's local
** view of the battery — the mission-level battery model stays the
** shared pure-function-of-time that keeps cross-app copies consistent.
*/

#ifndef WOK_HAL_SIM_POWER_H
#define WOK_HAL_SIM_POWER_H

#include <stdint.h>

/* Charge the local battery model with a consumer load: joules drawn sag
 * the LOCAL copy's bus voltage (0.05 V per kJ). Used by the EM-source
 * sim so bank charging draws from the battery model. */
void WOK_HAL_SimPower_Drain_J(float joules);

/* Test hook: force the battery baseline voltage (and restart the drain
 * clock). Lets the physics harness put the bus below the source's
 * charge-inhibit threshold without waiting hours of real time. */
void WOK_HAL_SimPower_SetBattery(float volts);

#endif /* WOK_HAL_SIM_POWER_H */
