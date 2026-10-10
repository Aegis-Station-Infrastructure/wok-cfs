/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Watchdog (no-op)
** File: wok_common/hal/src/linux_sim/wok_hal_sim_watchdog.c
*/

#include "wok_hal_watchdog.h"

int32_t WOK_HAL_Watchdog_Init(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Watchdog_Pet(void)
{
    return WOK_HAL_OK;
}
