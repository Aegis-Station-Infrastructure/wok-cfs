/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK Drill Platform Configuration
** (companion to wok_common/msgs/inc/wok_platform_cfg.h)
**
** Compile-time defaults for the drill subsystem.
*/

#ifndef WOK_DRILL_PLATFORM_CFG_H
#define WOK_DRILL_PLATFORM_CFG_H

/* ── Drill Arm ────────────────────────────────────────────────────── */
#define WOK_DRILL_DEFAULT_ARM_ANGLE_DEG     90.0f   /* Default deploy angle */
#define WOK_DRILL_DEFAULT_ARM_TIMEOUT_MS    8000    /* Deploy/stow timeout */

/* ── Drill Motor ──────────────────────────────────────────────────── */
#define WOK_DRILL_DEFAULT_MAX_DEPTH_M       5.0f    /* Max penetration depth */
#define WOK_DRILL_DEFAULT_MAX_RPM           300.0f  /* Max drill motor RPM */
#define WOK_DRILL_DEFAULT_TORQUE_LIMIT_NM   3.0f    /* Torque fault threshold */
#define WOK_DRILL_DEFAULT_CURRENT_LIMIT_A   5.0f    /* Motor overcurrent threshold */

/* ── Thermal Vaporization ─────────────────────────────────────────── */
#define WOK_DRILL_DEFAULT_HEAT_SEC          60      /* Default heat phase duration */
#define WOK_DRILL_DEFAULT_COOL_SEC          30      /* Default cool-down duration */
#define WOK_DRILL_DEFAULT_MAX_CELL_TEMP_C   350.0f  /* Cell overtemp fault threshold */

/* ── Sequencer Drill Safety ───────────────────────────────────────── */
#define WOK_DRILL_MIN_BATTERY_V             23.5f   /* Min battery V to start drill */

#endif /* WOK_DRILL_PLATFORM_CFG_H */
