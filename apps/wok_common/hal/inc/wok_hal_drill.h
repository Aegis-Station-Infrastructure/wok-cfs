/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Drill / Ice Prospector (IP) Subsystem
**
** All functions return int32_t: WOK_HAL_OK (0), WOK_HAL_ERROR (-1), WOK_HAL_TIMEOUT (-2)
** All commands are non-blocking. Poll Get* functions for state changes.
*/

#ifndef WOK_HAL_DRILL_H
#define WOK_HAL_DRILL_H

#include "wok_hal.h"

/* ── Drill Arm States ─────────────────────────────────────────────── */
#define WOK_HAL_DRILL_ARM_STOWED     0
#define WOK_HAL_DRILL_ARM_DEPLOYING  1
#define WOK_HAL_DRILL_ARM_READY      2
#define WOK_HAL_DRILL_ARM_STOWING    3

/* ── Drill Motor States ───────────────────────────────────────────── */
#define WOK_HAL_DRILL_MOTOR_IDLE     0
#define WOK_HAL_DRILL_MOTOR_DRILLING 1
#define WOK_HAL_DRILL_MOTOR_DONE     2

/* ── Sample States ────────────────────────────────────────────────── */
#define WOK_HAL_DRILL_SAMPLE_IDLE     0
#define WOK_HAL_DRILL_SAMPLE_HEATING  1
#define WOK_HAL_DRILL_SAMPLE_ANALYZE  2
#define WOK_HAL_DRILL_SAMPLE_COOLING  3
#define WOK_HAL_DRILL_SAMPLE_DONE     4

/* ── Drill Arm ────────────────────────────────────────────────────── */

/**
 * @brief Initialize drill arm servos, drill motor, thermal cell, sensors.
 *        Called once at startup from WOK_HAL_Init().
 */
int32_t WOK_HAL_Drill_Init(void);

/**
 * @brief Command arm to target angle (degrees from stowed).
 *        Non-blocking — poll GetArmState/GetArmAngle for progress.
 */
int32_t WOK_HAL_Drill_DeployArm(float target_deg);

/**
 * @brief Retract arm to stowed position. Non-blocking.
 */
int32_t WOK_HAL_Drill_StowArm(void);

/**
 * @brief Kill all drill subsystem motors immediately.
 *        Arm, drill motor, and heater all stop.
 */
int32_t WOK_HAL_Drill_EmergencyStop(void);

/**
 * @brief Read arm state: STOWED(0), DEPLOYING(1), READY(2), STOWING(3)
 */
int32_t WOK_HAL_Drill_GetArmState(uint8_t *state);

/**
 * @brief Read current arm angle in degrees.
 */
int32_t WOK_HAL_Drill_GetArmAngle(float *deg);

/**
 * @brief Read arm servo current draw (amps).
 */
int32_t WOK_HAL_Drill_GetArmCurrent(float *amps);

/* ── Drill Motor ──────────────────────────────────────────────────── */

/**
 * @brief Set the survey-site context for the next drill cycle: rover
 *        position in the mission start frame (from NAV pose). Stamped
 *        into data products; the sim also uses it to anchor the synthetic
 *        subsurface model so drill results are site-consistent with the
 *        seismic sim.
 */
int32_t WOK_HAL_Drill_SetSite(float x_m, float y_m);

/**
 * @brief Begin drilling to target depth at given RPM. Non-blocking.
 *        Poll GetDepth/GetDrillMotorState for progress.
 */
int32_t WOK_HAL_Drill_StartDrill(float depth_m, float rpm);

/**
 * @brief Stop drill motor immediately.
 */
int32_t WOK_HAL_Drill_StopDrill(void);

/**
 * @brief Read current penetration depth (metres from surface).
 */
int32_t WOK_HAL_Drill_GetDepth(float *m);

/**
 * @brief Read drill motor speed (RPM).
 */
int32_t WOK_HAL_Drill_GetRPM(float *rpm);

/**
 * @brief Read drill torque (Newton-metres).
 */
int32_t WOK_HAL_Drill_GetTorque(float *nm);

/**
 * @brief Read weight-on-bit (Newtons) — axial force on the bit. With
 *        torque and penetration rate this forms the regolith
 *        stiffness/competence log.
 */
int32_t WOK_HAL_Drill_GetWOB(float *n);

/**
 * @brief Read drill motor current (amps).
 */
int32_t WOK_HAL_Drill_GetMotorCurrent(float *amps);

/**
 * @brief Read drill motor state: IDLE(0), DRILLING(1), DONE(2)
 */
int32_t WOK_HAL_Drill_GetDrillMotorState(uint8_t *state);

/* ── Thermal Vaporization & Analysis ──────────────────────────────── */

/**
 * @brief Begin thermal vaporization cycle. Non-blocking.
 *        Heats for heat_sec, holds for analysis, cools for cool_sec.
 *        Poll GetSampleState for progress.
 */
int32_t WOK_HAL_Drill_StartSample(uint16_t heat_sec, uint16_t cool_sec);

/**
 * @brief Abort sampling — cut heater, stop analysis.
 */
int32_t WOK_HAL_Drill_StopSample(void);

/**
 * @brief Read vaporization cell temperature (Celsius).
 */
int32_t WOK_HAL_Drill_GetCellTemp(float *c);

/**
 * @brief Read vapor pressure inside cell (Pascals).
 */
int32_t WOK_HAL_Drill_GetVaporPressure(float *pa);

/**
 * @brief Read volatile concentration (ppm H2O equivalent).
 */
int32_t WOK_HAL_Drill_GetVolatileConc(float *ppm);

/**
 * @brief Read sample state: IDLE(0), HEATING(1), ANALYZING(2), COOLING(3), DONE(4)
 */
int32_t WOK_HAL_Drill_GetSampleState(uint8_t *state);

#endif /* WOK_HAL_DRILL_H */
