/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Drill Flight Stub
** File: wok_common/hal/src/flight_stub/wok_hal_drill_flight_stub.c
**
** Stub implementation — returns WOK_HAL_ERROR for all functions.
** Copy this file to your target directory and replace each function
** body with real BSP/driver calls.
*/

#include "wok_hal.h"
#include "wok_hal_drill.h"

/* ── Arm ─────────────────────────────────────────────────────────── */

int32_t WOK_HAL_Drill_Init(void)
{
    /* TODO: Initialize arm servos, drill motor controller, thermal cell,
     *       pressure transducer, spectrometer interface */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_DeployArm(float target_deg)
{
    (void)target_deg;
    /* TODO: Command arm servos to target angle via CAN/RS-485 */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_StowArm(void)
{
    /* TODO: Command arm to stowed position */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_EmergencyStop(void)
{
    /* TODO: Kill all drill motors and heater — hardware-level disable */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetArmState(uint8_t *state)
{
    (void)state;
    /* TODO: Read arm position sensors + endstops to determine state */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetArmAngle(float *deg)
{
    (void)deg;
    /* TODO: Read arm encoder / joint angle sensor */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetArmCurrent(float *amps)
{
    (void)amps;
    /* TODO: Read arm servo current sense ADC */
    return WOK_HAL_ERROR;
}

/* ── Drill Motor ─────────────────────────────────────────────────── */

int32_t WOK_HAL_Drill_SetSite(float x_m, float y_m)
{
    (void)x_m;
    (void)y_m;
    /* TODO: Stamp site context into drill data records */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetWOB(float *n)
{
    (void)n;
    /* TODO: Read axial load cell */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_StartDrill(float depth_m, float rpm)
{
    (void)depth_m;
    (void)rpm;
    /* TODO: Start drill motor, set target depth and RPM */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_StopDrill(void)
{
    /* TODO: Stop drill motor immediately */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetDepth(float *m)
{
    (void)m;
    /* TODO: Read depth encoder or linear position sensor */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetRPM(float *rpm)
{
    (void)rpm;
    /* TODO: Read drill motor tachometer */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetTorque(float *nm)
{
    (void)nm;
    /* TODO: Compute torque from motor current or torque sensor */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetMotorCurrent(float *amps)
{
    (void)amps;
    /* TODO: Read drill motor current sense */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetDrillMotorState(uint8_t *state)
{
    (void)state;
    /* TODO: Determine drill state from motor + depth sensors */
    return WOK_HAL_ERROR;
}

/* ── Thermal Vaporization & Analysis ─────────────────────────────── */

int32_t WOK_HAL_Drill_StartSample(uint16_t heat_sec, uint16_t cool_sec)
{
    (void)heat_sec;
    (void)cool_sec;
    /* TODO: Enable heater, start thermal cycle controller */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_StopSample(void)
{
    /* TODO: Cut heater, abort analysis */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetCellTemp(float *c)
{
    (void)c;
    /* TODO: Read thermocouple via SPI ADC */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetVaporPressure(float *pa)
{
    (void)pa;
    /* TODO: Read pressure transducer */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetVolatileConc(float *ppm)
{
    (void)ppm;
    /* TODO: Read TDLAS or TCD spectrometer output */
    return WOK_HAL_ERROR;
}

int32_t WOK_HAL_Drill_GetSampleState(uint8_t *state)
{
    (void)state;
    /* TODO: Determine sample cycle state from thermal controller */
    return WOK_HAL_ERROR;
}
