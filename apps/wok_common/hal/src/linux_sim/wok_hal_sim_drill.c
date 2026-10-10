/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Drill Linux Sim Implementation
**
** Simulation models:
**   Arm:      3-second timed transitions, 2.0A during motion, 0.1A at rest
**   Drill:    penetration + torque derived from the shared subsurface
**             geology model (wok_hal_sim_geology.c) — ROP slows and torque
**             rises in ice-cemented layers, so the drill's drilling-response log
**             corroborates the seismic sim's velocity structure.
**   Sampling: Heat at 5 C/s to 300C, hold 10s, cool at 3 C/s.
**             Volatile conc = geology ice wt% at the latched sample depth
**             (1 wt% = 10,000 ppm), so drill ground truth is
**             site-consistent with the seismic evaluate.
*/

/* clock_gettime/CLOCK_MONOTONIC under strict C99 */
#define _POSIX_C_SOURCE 199309L

#include "wok_hal.h"
#include "wok_hal_drill.h"
#include "wok_hal_sim_geology.h"
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* ── Internal state ──────────────────────────────────────────────── */

static struct {
    uint8_t  initialized;

    /* Site context (start frame, from NAV pose via the app) */
    float    site_x_m;
    float    site_y_m;

    /* Arm */
    uint8_t  arm_state;          /* WOK_HAL_DRILL_ARM_* */
    float    arm_angle_deg;
    float    arm_target_deg;
    time_t   arm_cmd_time;
    float    arm_current;

    /* Drill motor */
    uint8_t  drill_state;        /* WOK_HAL_DRILL_MOTOR_* */
    float    drill_depth_m;
    float    drill_target_m;
    float    drill_rpm;
    float    drill_target_rpm;
    float    drill_torque;
    float    drill_current;
    time_t   drill_start_time;
    uint64_t depth_update_ms;    /* last depth integration step         */

    /* Thermal vaporization */
    uint8_t  sample_state;       /* WOK_HAL_DRILL_SAMPLE_* */
    float    cell_temp_c;
    float    vapor_pressure_pa;
    float    volatile_conc_ppm;
    float    sample_depth_m;     /* depth latched at StartSample        */
    uint16_t heat_sec;
    uint16_t cool_sec;
    time_t   sample_start_time;

    /* Emergency stop flag */
    uint8_t  estop;

} g_drill_sim;

/* ── Helpers ─────────────────────────────────────────────────────── */

#define ARM_TRANSITION_SEC   3.0
#define DRILL_RATE_MPS       0.04   /* base ROP in dry regolith (str=1);
                                       sim-compressed for demo pacing   */
#define NOMINAL_TORQUE_NM    0.5    /* per unit geology strength        */
#define SPIKE_TORQUE_NM      0.6    /* occasional bit-bind transient    */
#define HEAT_RATE_CPS        5.0    /* Celsius per second */
#define TARGET_TEMP_C        300.0
#define ANALYSIS_HOLD_SEC    10.0
#define COOL_RATE_CPS        3.0
#define AMBIENT_TEMP_C       25.0
#define PPM_PER_WTPCT        10000.0f /* 1 wt% water-equiv = 10,000 ppm */
#define ARM_MOTION_AMPS      2.0
#define ARM_IDLE_AMPS        0.1
#define DRILL_AMPS_PER_NM    1.5

static double elapsed_sec(time_t from)
{
    return difftime(time(NULL), from);
}

static uint64_t drill_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

static float rand_noise(float amplitude)
{
    return ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f * amplitude;
}

/* ── Arm ─────────────────────────────────────────────────────────── */

int32_t WOK_HAL_Drill_Init(void)
{
    memset(&g_drill_sim, 0, sizeof(g_drill_sim));
    g_drill_sim.arm_state     = WOK_HAL_DRILL_ARM_STOWED;
    g_drill_sim.drill_state   = WOK_HAL_DRILL_MOTOR_IDLE;
    g_drill_sim.sample_state  = WOK_HAL_DRILL_SAMPLE_IDLE;
    g_drill_sim.cell_temp_c   = AMBIENT_TEMP_C;
    g_drill_sim.initialized   = 1;
    srand((unsigned)time(NULL) ^ 0xDEAD);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_DeployArm(float target_deg)
{
    if (!g_drill_sim.initialized) return WOK_HAL_ERROR;
    if (g_drill_sim.estop)        return WOK_HAL_ERROR;

    g_drill_sim.arm_target_deg = target_deg;
    g_drill_sim.arm_state      = WOK_HAL_DRILL_ARM_DEPLOYING;
    g_drill_sim.arm_cmd_time   = time(NULL);
    g_drill_sim.arm_current    = ARM_MOTION_AMPS;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_StowArm(void)
{
    if (!g_drill_sim.initialized) return WOK_HAL_ERROR;

    g_drill_sim.arm_target_deg = 0.0f;
    g_drill_sim.arm_state      = WOK_HAL_DRILL_ARM_STOWING;
    g_drill_sim.arm_cmd_time   = time(NULL);
    g_drill_sim.arm_current    = ARM_MOTION_AMPS;
    g_drill_sim.estop          = 0;  /* clear e-stop on stow */
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_EmergencyStop(void)
{
    if (!g_drill_sim.initialized) return WOK_HAL_ERROR;

    g_drill_sim.estop         = 1;
    g_drill_sim.drill_state   = WOK_HAL_DRILL_MOTOR_IDLE;
    g_drill_sim.drill_rpm     = 0;
    g_drill_sim.drill_torque  = 0;
    g_drill_sim.drill_current = 0;
    g_drill_sim.arm_current   = 0;

    /* Stop sampling — cut heater */
    if (g_drill_sim.sample_state != WOK_HAL_DRILL_SAMPLE_IDLE &&
        g_drill_sim.sample_state != WOK_HAL_DRILL_SAMPLE_DONE)
    {
        g_drill_sim.sample_state = WOK_HAL_DRILL_SAMPLE_IDLE;
    }

    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetArmState(uint8_t *state)
{
    if (!g_drill_sim.initialized || !state) return WOK_HAL_ERROR;

    double dt = elapsed_sec(g_drill_sim.arm_cmd_time);
    float frac = (float)(dt / ARM_TRANSITION_SEC);
    if (frac > 1.0f) frac = 1.0f;

    if (g_drill_sim.arm_state == WOK_HAL_DRILL_ARM_DEPLOYING)
    {
        g_drill_sim.arm_angle_deg = frac * g_drill_sim.arm_target_deg;
        if (dt >= ARM_TRANSITION_SEC)
        {
            g_drill_sim.arm_state   = WOK_HAL_DRILL_ARM_READY;
            g_drill_sim.arm_angle_deg = g_drill_sim.arm_target_deg;
            g_drill_sim.arm_current = ARM_IDLE_AMPS;
        }
    }
    else if (g_drill_sim.arm_state == WOK_HAL_DRILL_ARM_STOWING)
    {
        float start_angle = g_drill_sim.arm_target_deg > 0 ?
            g_drill_sim.arm_target_deg : g_drill_sim.arm_angle_deg;
        g_drill_sim.arm_angle_deg = start_angle * (1.0f - frac);
        if (dt >= ARM_TRANSITION_SEC)
        {
            g_drill_sim.arm_state     = WOK_HAL_DRILL_ARM_STOWED;
            g_drill_sim.arm_angle_deg = 0.0f;
            g_drill_sim.arm_current   = 0.0f;
        }
    }

    *state = g_drill_sim.arm_state;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetArmAngle(float *deg)
{
    if (!g_drill_sim.initialized || !deg) return WOK_HAL_ERROR;
    /* Force state update */
    uint8_t s;
    WOK_HAL_Drill_GetArmState(&s);
    *deg = g_drill_sim.arm_angle_deg;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetArmCurrent(float *amps)
{
    if (!g_drill_sim.initialized || !amps) return WOK_HAL_ERROR;
    *amps = g_drill_sim.arm_current + rand_noise(0.05f);
    return WOK_HAL_OK;
}

/* ── Drill Motor ─────────────────────────────────────────────────── */

int32_t WOK_HAL_Drill_SetSite(float x_m, float y_m)
{
    if (!g_drill_sim.initialized) return WOK_HAL_ERROR;
    g_drill_sim.site_x_m = x_m;
    g_drill_sim.site_y_m = y_m;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_StartDrill(float depth_m, float rpm)
{
    if (!g_drill_sim.initialized) return WOK_HAL_ERROR;
    if (g_drill_sim.estop)        return WOK_HAL_ERROR;
    if (g_drill_sim.arm_state != WOK_HAL_DRILL_ARM_READY) return WOK_HAL_ERROR;

    g_drill_sim.drill_target_m   = depth_m;
    g_drill_sim.drill_target_rpm = rpm;
    g_drill_sim.drill_rpm        = rpm;
    g_drill_sim.drill_depth_m    = 0.0f;
    g_drill_sim.drill_state      = WOK_HAL_DRILL_MOTOR_DRILLING;
    g_drill_sim.drill_start_time = time(NULL);
    g_drill_sim.depth_update_ms  = drill_now_ms();
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_StopDrill(void)
{
    if (!g_drill_sim.initialized) return WOK_HAL_ERROR;
    g_drill_sim.drill_state   = WOK_HAL_DRILL_MOTOR_IDLE;
    g_drill_sim.drill_rpm     = 0;
    g_drill_sim.drill_torque  = 0;
    g_drill_sim.drill_current = 0;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetDepth(float *m)
{
    if (!g_drill_sim.initialized || !m) return WOK_HAL_ERROR;

    if (g_drill_sim.drill_state == WOK_HAL_DRILL_MOTOR_DRILLING)
    {
        /* Incremental integration: ROP = base rate / geology strength at
         * the current bit depth, so penetration slows through
         * ice-cemented material (the sim's stiffness signature). */
        uint64_t now = drill_now_ms();
        float dt_s = (float)(now - g_drill_sim.depth_update_ms) / 1000.0f;
        g_drill_sim.depth_update_ms = now;

        float strength = WOK_SimGeology_StrengthAt(g_drill_sim.site_x_m,
                                                   g_drill_sim.site_y_m,
                                                   g_drill_sim.drill_depth_m);
        if (strength < 1.0f) strength = 1.0f;
        g_drill_sim.drill_depth_m += dt_s * (float)DRILL_RATE_MPS / strength;

        if (g_drill_sim.drill_depth_m >= g_drill_sim.drill_target_m)
        {
            g_drill_sim.drill_depth_m = g_drill_sim.drill_target_m;
            g_drill_sim.drill_state   = WOK_HAL_DRILL_MOTOR_DONE;
            g_drill_sim.drill_rpm     = 0;
            g_drill_sim.drill_torque  = 0;
            g_drill_sim.drill_current = 0;
        }
    }

    *m = g_drill_sim.drill_depth_m;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetRPM(float *rpm)
{
    if (!g_drill_sim.initialized || !rpm) return WOK_HAL_ERROR;
    /* Update depth to check for completion */
    float d;
    WOK_HAL_Drill_GetDepth(&d);
    *rpm = g_drill_sim.drill_rpm + rand_noise(5.0f);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetTorque(float *nm)
{
    if (!g_drill_sim.initialized || !nm) return WOK_HAL_ERROR;

    if (g_drill_sim.drill_state == WOK_HAL_DRILL_MOTOR_DRILLING)
    {
        /* Torque tracks geology strength at the bit (drilling-response log),
         * plus occasional bit-bind transients (5% chance). */
        float strength = WOK_SimGeology_StrengthAt(g_drill_sim.site_x_m,
                                                   g_drill_sim.site_y_m,
                                                   g_drill_sim.drill_depth_m);
        float spike = ((rand() % 100) < 5) ? SPIKE_TORQUE_NM : 0.0f;
        g_drill_sim.drill_torque = (float)NOMINAL_TORQUE_NM * strength
                                 + rand_noise(0.1f) + spike;
    }
    else
    {
        g_drill_sim.drill_torque = 0.0f;
    }

    *nm = g_drill_sim.drill_torque;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetWOB(float *n)
{
    if (!g_drill_sim.initialized || !n) return WOK_HAL_ERROR;

    if (g_drill_sim.drill_state == WOK_HAL_DRILL_MOTOR_DRILLING)
    {
        /* Axial preload plus reaction that scales with material strength */
        float strength = WOK_SimGeology_StrengthAt(g_drill_sim.site_x_m,
                                                   g_drill_sim.site_y_m,
                                                   g_drill_sim.drill_depth_m);
        *n = 100.0f + 20.0f * strength + rand_noise(5.0f);
    }
    else
    {
        *n = 0.0f;
    }
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetMotorCurrent(float *amps)
{
    if (!g_drill_sim.initialized || !amps) return WOK_HAL_ERROR;

    if (g_drill_sim.drill_state == WOK_HAL_DRILL_MOTOR_DRILLING)
    {
        g_drill_sim.drill_current = g_drill_sim.drill_torque * DRILL_AMPS_PER_NM;
    }
    else
    {
        g_drill_sim.drill_current = 0.0f;
    }

    *amps = g_drill_sim.drill_current + rand_noise(0.02f);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetDrillMotorState(uint8_t *state)
{
    if (!g_drill_sim.initialized || !state) return WOK_HAL_ERROR;
    /* Update depth to check for completion */
    float d;
    WOK_HAL_Drill_GetDepth(&d);
    *state = g_drill_sim.drill_state;
    return WOK_HAL_OK;
}

/* ── Thermal Vaporization & Analysis ─────────────────────────────── */

int32_t WOK_HAL_Drill_StartSample(uint16_t heat_sec, uint16_t cool_sec)
{
    if (!g_drill_sim.initialized) return WOK_HAL_ERROR;
    if (g_drill_sim.estop)        return WOK_HAL_ERROR;

    g_drill_sim.heat_sec          = heat_sec;
    g_drill_sim.cool_sec          = cool_sec;
    g_drill_sim.sample_state      = WOK_HAL_DRILL_SAMPLE_HEATING;
    g_drill_sim.sample_start_time = time(NULL);
    g_drill_sim.cell_temp_c       = AMBIENT_TEMP_C;
    g_drill_sim.vapor_pressure_pa = 0.0f;
    g_drill_sim.volatile_conc_ppm = 0.0f;
    /* Depth registration: the analysed material is what the bit sits in
     * right now — latch it so the result is tagged to a confirmed depth. */
    g_drill_sim.sample_depth_m    = g_drill_sim.drill_depth_m;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_StopSample(void)
{
    if (!g_drill_sim.initialized) return WOK_HAL_ERROR;
    g_drill_sim.sample_state      = WOK_HAL_DRILL_SAMPLE_IDLE;
    g_drill_sim.cell_temp_c       = AMBIENT_TEMP_C;
    g_drill_sim.vapor_pressure_pa = 0.0f;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetCellTemp(float *c)
{
    if (!g_drill_sim.initialized || !c) return WOK_HAL_ERROR;

    double dt = elapsed_sec(g_drill_sim.sample_start_time);

    switch (g_drill_sim.sample_state)
    {
        case WOK_HAL_DRILL_SAMPLE_HEATING:
        {
            g_drill_sim.cell_temp_c = AMBIENT_TEMP_C + (float)(dt * HEAT_RATE_CPS);
            if (g_drill_sim.cell_temp_c >= TARGET_TEMP_C)
            {
                g_drill_sim.cell_temp_c   = TARGET_TEMP_C;
                g_drill_sim.sample_state  = WOK_HAL_DRILL_SAMPLE_ANALYZE;
                g_drill_sim.sample_start_time = time(NULL);
            }
            /* Evolved-gas ramp: cumulative volatiles released so far is a
             * function of cell temperature and how the water is bound
             * (pore ice ~100C, adsorbed ~160C, hydrated ~320C). The
             * (T, ppm) curve logged during this phase IS the binding
             * classification measurement. Total: 1 wt% = 10,000 ppm. */
            {
                float total = WOK_SimGeology_IceWtPctAt(g_drill_sim.site_x_m,
                                                        g_drill_sim.site_y_m,
                                                        g_drill_sim.sample_depth_m)
                            * PPM_PER_WTPCT;
                float frac  = WOK_SimGeology_EvolvedFraction(g_drill_sim.site_x_m,
                                                             g_drill_sim.site_y_m,
                                                             g_drill_sim.sample_depth_m,
                                                             g_drill_sim.cell_temp_c);
                g_drill_sim.volatile_conc_ppm = total * frac
                    + rand_noise(0.02f * total * frac + 2.0f);
                if (g_drill_sim.volatile_conc_ppm < 0.0f)
                    g_drill_sim.volatile_conc_ppm = 0.0f;
            }
            /* Cell pressure: thermal baseline + evolved-gas partial pressure */
            g_drill_sim.vapor_pressure_pa =
                (g_drill_sim.cell_temp_c - AMBIENT_TEMP_C) * 3.6f
                + 0.01f * g_drill_sim.volatile_conc_ppm;
            break;
        }
        case WOK_HAL_DRILL_SAMPLE_ANALYZE:
        {
            /* Hold at target temp; cumulative release plateaus (hydrated
             * component stays partly bound below its ~320C release peak) */
            float total = WOK_SimGeology_IceWtPctAt(g_drill_sim.site_x_m,
                                                    g_drill_sim.site_y_m,
                                                    g_drill_sim.sample_depth_m)
                        * PPM_PER_WTPCT;
            float frac  = WOK_SimGeology_EvolvedFraction(g_drill_sim.site_x_m,
                                                         g_drill_sim.site_y_m,
                                                         g_drill_sim.sample_depth_m,
                                                         TARGET_TEMP_C);
            g_drill_sim.cell_temp_c = TARGET_TEMP_C + rand_noise(2.0f);
            g_drill_sim.volatile_conc_ppm = total * frac
                + rand_noise(0.02f * total * frac + 2.0f);
            if (g_drill_sim.volatile_conc_ppm < 0.0f)
                g_drill_sim.volatile_conc_ppm = 0.0f;
            g_drill_sim.vapor_pressure_pa =
                (g_drill_sim.cell_temp_c - AMBIENT_TEMP_C) * 3.6f
                + 0.01f * g_drill_sim.volatile_conc_ppm + rand_noise(5.0f);
            if (dt >= ANALYSIS_HOLD_SEC)
            {
                g_drill_sim.sample_state      = WOK_HAL_DRILL_SAMPLE_COOLING;
                g_drill_sim.sample_start_time = time(NULL);
            }
            break;
        }
        case WOK_HAL_DRILL_SAMPLE_COOLING:
        {
            g_drill_sim.cell_temp_c = TARGET_TEMP_C - (float)(dt * COOL_RATE_CPS);
            if (g_drill_sim.cell_temp_c <= AMBIENT_TEMP_C + 5.0f)
            {
                g_drill_sim.cell_temp_c       = AMBIENT_TEMP_C;
                g_drill_sim.sample_state      = WOK_HAL_DRILL_SAMPLE_DONE;
                g_drill_sim.vapor_pressure_pa = 0.0f;
            }
            else
            {
                g_drill_sim.vapor_pressure_pa = (g_drill_sim.cell_temp_c - AMBIENT_TEMP_C) * 3.6f;
            }
            break;
        }
        default:
            break;
    }

    *c = g_drill_sim.cell_temp_c + rand_noise(0.5f);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetVaporPressure(float *pa)
{
    if (!g_drill_sim.initialized || !pa) return WOK_HAL_ERROR;
    /* Force temp update which drives pressure model */
    float t;
    WOK_HAL_Drill_GetCellTemp(&t);
    *pa = g_drill_sim.vapor_pressure_pa + rand_noise(2.0f);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetVolatileConc(float *ppm)
{
    if (!g_drill_sim.initialized || !ppm) return WOK_HAL_ERROR;
    /* Force temp update to advance state machine */
    float t;
    WOK_HAL_Drill_GetCellTemp(&t);
    *ppm = g_drill_sim.volatile_conc_ppm;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Drill_GetSampleState(uint8_t *state)
{
    if (!g_drill_sim.initialized || !state) return WOK_HAL_ERROR;
    /* Force temp update to advance state machine */
    float t;
    WOK_HAL_Drill_GetCellTemp(&t);
    *state = g_drill_sim.sample_state;
    return WOK_HAL_OK;
}
