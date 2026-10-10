/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Mobility (primitive contract)
** File: wok_common/hal/src/linux_sim/wok_hal_sim_mobility.c
**
** Time-based kinematic sim of the elementary motion primitives.
**
**   DriveForward(d, spd) — d > 0 only; rover does not reverse-drive in
**                          this sim. Negative deltas are NAV's job to
**                          handle by turning 180° first.
**   Turn(angle, spd)     — signed angle, + = left (CCW), - = right (CW).
**                          spd is the magnitude of the turn rate.
**
** Default speeds: 0.5 m/s drive, 30 deg/s turn.
** A 3 m drive ≈ 6 s; a 90 deg turn ≈ 3 s.
**
** Ground truth + turn bias (recalibration support): the sim integrates
** the TRUE pose it actually drives, including an UNSENSED heading bias
** added per completed turn event (WOK_SIM_TURN_BIAS_DEG platform-config
** default, env-overridable). The per-primitive accumulators report the
** COMMANDED motion — exactly what real encoders/IMU would believe — so
** NAV's dead reckoning diverges from truth when the bias is nonzero.
** Truth is exposed only through the sim-internal GetTruePose (harness /
** verification); the apps never read it.
*/

#include "wok_hal_mobility.h"
#include "wok_hal_sim_mobility.h"
#include "wok_platform_cfg.h"
#include <math.h>
#include <stdlib.h>
#include <sys/time.h>

typedef enum {
    SIM_MOTION_IDLE  = 0,
    SIM_MOTION_DRIVE = 1,
    SIM_MOTION_TURN  = 2,
} sim_motion_t;

static struct {
    uint8_t       initialized;

    sim_motion_t  motion;
    float         target;          /* m for DRIVE (≥0), signed deg for TURN  */
    float         progress;        /* signed; matches sign(target)            */
    float         speed_cmd;       /* magnitude: m/s (DRIVE) or deg/s (TURN)  */
    float         speed_now;       /* current m/s (zero during TURN)          */
    float         motor_current;
    float         tilt;
    float         max_speed;

    /* Latched per-primitive results — what GetDistanceTraveled /
     * GetHeadingChange return. Each primitive zeroes its own result on
     * entry and accumulates while the motion is active. */
    float         distance_traveled_m;
    float         heading_change_deg;

    /* Ground-truth pose (start frame) + the unsensed per-turn bias */
    float         true_x_m, true_y_m, true_heading_deg;
    float         turn_bias_deg;

    struct timeval last_update;
} g_mob;

/* Fold a completed (or aborted) motion into the true pose. A finished
 * TURN picks up the unsensed bias; an aborted one only its progress. */
static void integrate_truth(float amount, int completed)
{
    if (g_mob.motion == SIM_MOTION_DRIVE)
    {
        float h = g_mob.true_heading_deg * (float)(3.14159265358979323846 / 180.0);
        g_mob.true_x_m += amount * cosf(h);
        g_mob.true_y_m += amount * sinf(h);
    }
    else if (g_mob.motion == SIM_MOTION_TURN)
    {
        g_mob.true_heading_deg += amount;
        if (completed)
            g_mob.true_heading_deg += g_mob.turn_bias_deg;
    }
}

static double get_elapsed_sec(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    double dt = (now.tv_sec - g_mob.last_update.tv_sec) +
                (now.tv_usec - g_mob.last_update.tv_usec) / 1e6;
    g_mob.last_update = now;
    if (dt > 2.0) dt = 2.0;
    if (dt < 0.0) dt = 0.0;
    return dt;
}

/* Advance the active motion by elapsed time; latch the right accumulator
 * (distance for DRIVE, signed heading for TURN) just before transitioning
 * to IDLE so the final completed value is preserved for callers. */
static void advance_motion(void)
{
    if (g_mob.motion == SIM_MOTION_IDLE) {
        gettimeofday(&g_mob.last_update, NULL);
        g_mob.speed_now = 0.0f;
        return;
    }
    double dt = get_elapsed_sec();
    float step_mag = g_mob.speed_cmd * (float)dt;
    if (step_mag < 0.0f) step_mag = 0.0f;

    float dir           = (g_mob.target >= 0.0f) ? 1.0f : -1.0f;
    float remaining_mag = (g_mob.target - g_mob.progress) * dir;

    if (step_mag >= remaining_mag) {
        g_mob.progress = g_mob.target;
        integrate_truth(g_mob.target, 1);
        if (g_mob.motion == SIM_MOTION_DRIVE)
            g_mob.distance_traveled_m = g_mob.progress;
        else
            g_mob.heading_change_deg  = g_mob.progress;
        g_mob.motion        = SIM_MOTION_IDLE;
        g_mob.speed_now     = 0.0f;
        g_mob.motor_current = 0.0f;
    } else {
        g_mob.progress += dir * step_mag;
        g_mob.speed_now = (g_mob.motion == SIM_MOTION_DRIVE) ? g_mob.speed_cmd : 0.0f;
    }
}

int32_t WOK_HAL_Mobility_Init(void)
{
    const char *bias_env = getenv("WOK_SIM_TURN_BIAS_DEG");

    g_mob.initialized         = 1;
    g_mob.true_x_m            = 0.0f;
    g_mob.true_y_m            = 0.0f;
    g_mob.true_heading_deg    = 0.0f;
    g_mob.turn_bias_deg       = bias_env ? (float)atof(bias_env)
                                         : WOK_SIM_TURN_BIAS_DEG;
    g_mob.motion              = SIM_MOTION_IDLE;
    g_mob.target              = 0.0f;
    g_mob.progress            = 0.0f;
    g_mob.speed_cmd           = 0.0f;
    g_mob.speed_now           = 0.0f;
    g_mob.motor_current       = 0.0f;
    g_mob.tilt                = 2.0f;
    g_mob.max_speed           = 0.5f;
    g_mob.distance_traveled_m = 0.0f;
    g_mob.heading_change_deg  = 0.0f;
    gettimeofday(&g_mob.last_update, NULL);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_Stop(void)
{
    if (!g_mob.initialized) return WOK_HAL_ERROR;
    /* Fold partial progress into truth before idling (no bias — the
     * bias models a systematic error at turn completion). */
    advance_motion();
    integrate_truth(g_mob.progress, 0);
    g_mob.motion        = SIM_MOTION_IDLE;
    g_mob.speed_now     = 0.0f;
    g_mob.motor_current = 0.0f;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_SetSpeed(float spd)
{
    if (!g_mob.initialized) return WOK_HAL_ERROR;
    if (spd > 0.0f) g_mob.max_speed = spd;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_DriveForward(float dist_m, float spd_mps)
{
    if (!g_mob.initialized)  return WOK_HAL_ERROR;
    if (dist_m <= 0.0f)      return WOK_HAL_ERROR;
    g_mob.motion              = SIM_MOTION_DRIVE;
    g_mob.target              = dist_m;
    g_mob.progress            = 0.0f;
    g_mob.speed_cmd           = (spd_mps > 0.0f) ? spd_mps : g_mob.max_speed;
    g_mob.speed_now           = g_mob.speed_cmd;
    g_mob.motor_current       = 1.2f;
    g_mob.distance_traveled_m = 0.0f;
    gettimeofday(&g_mob.last_update, NULL);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_Turn(float angle_deg, float spd_dps)
{
    if (!g_mob.initialized) return WOK_HAL_ERROR;
    if (angle_deg == 0.0f)  return WOK_HAL_ERROR;
    g_mob.motion             = SIM_MOTION_TURN;
    g_mob.target             = angle_deg;
    g_mob.progress           = 0.0f;
    g_mob.speed_cmd          = (spd_dps > 0.0f) ? spd_dps : 30.0f;
    g_mob.speed_now          = 0.0f;
    g_mob.motor_current      = 1.5f;
    g_mob.heading_change_deg = 0.0f;
    gettimeofday(&g_mob.last_update, NULL);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_GetDistanceTraveled(float *m)
{
    if (!g_mob.initialized || !m) return WOK_HAL_ERROR;
    advance_motion();
    /* While DRIVE is active, expose the live progress; once IDLE, the latch
     * holds the final completed distance until next DriveForward zeros it. */
    if (g_mob.motion == SIM_MOTION_DRIVE)
        g_mob.distance_traveled_m = g_mob.progress;
    *m = g_mob.distance_traveled_m;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_GetHeadingChange(float *deg)
{
    if (!g_mob.initialized || !deg) return WOK_HAL_ERROR;
    advance_motion();
    if (g_mob.motion == SIM_MOTION_TURN)
        g_mob.heading_change_deg = g_mob.progress;
    *deg = g_mob.heading_change_deg;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_IsMoving(uint8_t *moving)
{
    if (!g_mob.initialized || !moving) return WOK_HAL_ERROR;
    advance_motion();
    *moving = (g_mob.motion != SIM_MOTION_IDLE) ? 1 : 0;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_GetSpeed(float *spd)
{
    if (!g_mob.initialized || !spd) return WOK_HAL_ERROR;
    advance_motion();
    *spd = g_mob.speed_now;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_GetTilt(float *deg)
{
    if (!g_mob.initialized || !deg) return WOK_HAL_ERROR;
    g_mob.tilt = 1.5f + (float)(rand() % 200) / 100.0f;
    *deg = g_mob.tilt;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Mobility_GetMotorCurrent(float *amps)
{
    if (!g_mob.initialized || !amps) return WOK_HAL_ERROR;
    *amps = g_mob.motor_current;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_SimMobility_GetTruePose(float *x_m, float *y_m,
                                        float *heading_deg)
{
    if (!g_mob.initialized || !x_m || !y_m || !heading_deg)
        return WOK_HAL_ERROR;
    advance_motion();
    *x_m         = g_mob.true_x_m;
    *y_m         = g_mob.true_y_m;
    *heading_deg = g_mob.true_heading_deg;
    return WOK_HAL_OK;
}
