/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Power & Thermal Subsystem (WOK_HK)
** File: wok_common/hal/inc/wok_hal_power.h
*/

#ifndef WOK_HAL_POWER_H
#define WOK_HAL_POWER_H

#include "wok_hal.h"

int32_t WOK_HAL_Power_Init(void);
int32_t WOK_HAL_Power_GetBatteryV(float *v);
int32_t WOK_HAL_Power_GetBatterySOC(float *pct);
int32_t WOK_HAL_Power_GetBoardTemp(float *c);
int32_t WOK_HAL_Power_GetMotorTemp(float *c);
int32_t WOK_HAL_Power_GetCPULoad(float *pct);

#endif /* WOK_HAL_POWER_H */
