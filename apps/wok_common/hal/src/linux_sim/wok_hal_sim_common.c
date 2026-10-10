/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Common (master init/shutdown)
** File: wok_common/hal/src/linux_sim/wok_hal_sim_common.c
*/

#include "wok_hal.h"
#include "wok_hal_mobility.h"
#include "wok_hal_geo.h"
#include "wok_hal_seis.h"
#include "wok_hal_power.h"
#include "wok_hal_storage.h"
#include "wok_hal_watchdog.h"
#include "wok_hal_drill.h"
#include <stdio.h>

int32_t WOK_HAL_Init(void)
{
    printf("[HAL-SIM] init\n");
    WOK_HAL_Mobility_Init();
    WOK_HAL_Geo_Init();
    WOK_HAL_Seis_Init();
    WOK_HAL_Src_Init();
    WOK_HAL_Power_Init();
    WOK_HAL_Storage_Init();
    WOK_HAL_Watchdog_Init(5000);
    WOK_HAL_Drill_Init();
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Shutdown(void)
{
    printf("[HAL-SIM] shutdown\n");
    return WOK_HAL_OK;
}
