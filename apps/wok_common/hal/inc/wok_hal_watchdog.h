/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Watchdog Subsystem (WOK_HK)
** File: wok_common/hal/inc/wok_hal_watchdog.h
*/

#ifndef WOK_HAL_WATCHDOG_H
#define WOK_HAL_WATCHDOG_H

#include "wok_hal.h"

int32_t WOK_HAL_Watchdog_Init(uint32_t timeout_ms);
int32_t WOK_HAL_Watchdog_Pet(void);

#endif /* WOK_HAL_WATCHDOG_H */
