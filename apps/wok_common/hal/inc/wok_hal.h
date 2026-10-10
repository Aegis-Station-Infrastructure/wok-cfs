/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Common Types and Master Init
** File: wok_common/hal/inc/wok_hal.h
**
** Base header only. Each subsystem has its own header:
**   wok_hal_mobility.h  — NAV app (drive, pose, tilt)
**   wok_hal_geo.h       — GEO app (deploy, stow, endstops)
**   wok_hal_seis.h      — SEIS app (ADC, EM source, evaluate)
**   wok_hal_power.h     — HK app (battery, temperature)
**   wok_hal_watchdog.h  — HK app (watchdog timer)
**   wok_hal_storage.h   — SEIS app (data file writes)
**   wok_hal_drill.h     — DRILL app (arm, drill motor, thermal)
**
** Apps include only the headers they need.
**
** All HAL functions return int32_t:
**   WOK_HAL_OK      (0)   — success
**   WOK_HAL_ERROR   (-1)  — general failure
**   WOK_HAL_TIMEOUT (-2)  — operation timed out
**
** All commands are non-blocking unless noted.
** Poll Get* functions for state changes.
*/

#ifndef WOK_HAL_H
#define WOK_HAL_H

#include <stdint.h>
#include <stddef.h>

/* ── Return Codes ────────────────────────────────────────────────── */
#define WOK_HAL_OK       0
#define WOK_HAL_ERROR   (-1)
#define WOK_HAL_TIMEOUT (-2)

/* ── Master Init / Shutdown ──────────────────────────────────────── */

/**
 * @brief Initialize all HAL subsystems. Called once at startup.
 *        Calls each subsystem's Init() internally.
 */
int32_t WOK_HAL_Init(void);

/**
 * @brief Clean shutdown. Release all resources.
 */
int32_t WOK_HAL_Shutdown(void);

#endif /* WOK_HAL_H */
