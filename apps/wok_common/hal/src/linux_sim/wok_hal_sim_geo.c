/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Geophone Mechanism
** File: wok_common/hal/src/linux_sim/wok_hal_sim_geo.c
**
** Legacy array mechanism: timed transitions (3 s) DEPLOYING→DEPLOYED /
** STOWING→STOWED; motor current 1.5A during motion, 0A at rest.
**
** Per-pod placement (CP-3): each pod is a timed state machine —
** PLACING (4 s; 3 s for a reseat) → PLACED, RETRIEVING (4 s) → STOWED.
** On PLACED the seating force latches from the coupling truth model
** (WOK_SimCoupling_SeatForceN at the pod's position and seat attempt;
** WOK_SIM_COUPLING unset → nominal 30 N, every seat reads firm). The
** GEO app's proxy gate and reseat retries run against these values,
** and the SEIS sim draws the pod's amplitude factor from the same
** (position, attempt) — the seat the arm felt is the seat the traces
** show.
*/

#include "wok_hal_geo.h"
#include "wok_hal_sim_coupling.h"
#include <time.h>
#include <stdlib.h>

#define GEO_POD_PLACE_SEC    4
#define GEO_POD_RESEAT_SEC   3
#define GEO_POD_RETRIEVE_SEC 4
#define GEO_POD_NOMINAL_FORCE_N 30.0f

static struct {
    uint8_t  initialized;
    uint8_t  state;
    time_t   cmd_time;
    float    motor_current;
    uint8_t  coupling_on;
    struct {
        uint8_t state;          /* WOK_HAL_GEO_POD_*                  */
        uint8_t attempt;        /* seat attempt at current position   */
        uint8_t reseating;      /* current PLACING is a reseat        */
        float   x_m, y_m;
        float   seat_force_n;   /* latched when PLACED                */
        time_t  cmd_time;
    } pod[WOK_HAL_GEO_MAX_PODS];
} g_geo;

#define GEO_TRANSITION_SEC 3

int32_t WOK_HAL_Geo_Init(void)
{
    const char *coup_env = getenv("WOK_SIM_COUPLING");

    g_geo.initialized = 1;
    g_geo.state = WOK_HAL_GEO_STOWED;
    g_geo.motor_current = 0.0f;
    g_geo.coupling_on = (coup_env != NULL && coup_env[0] != '\0'
                         && coup_env[0] != '0') ? 1 : 0;
    for (int p = 0; p < WOK_HAL_GEO_MAX_PODS; p++)
    {
        g_geo.pod[p].state        = WOK_HAL_GEO_POD_STOWED;
        g_geo.pod[p].attempt      = 0;
        g_geo.pod[p].reseating    = 0;
        g_geo.pod[p].seat_force_n = 0.0f;
    }
    return WOK_HAL_OK;
}

/* Advance a pod's timed transition from the wall clock */
static void geo_pod_update(int p)
{
    double dt = difftime(time(NULL), g_geo.pod[p].cmd_time);

    if (g_geo.pod[p].state == WOK_HAL_GEO_POD_PLACING &&
        dt >= (g_geo.pod[p].reseating ? GEO_POD_RESEAT_SEC
                                      : GEO_POD_PLACE_SEC))
    {
        g_geo.pod[p].state = WOK_HAL_GEO_POD_PLACED;
        g_geo.pod[p].seat_force_n = g_geo.coupling_on
            ? WOK_SimCoupling_SeatForceN(g_geo.pod[p].x_m, g_geo.pod[p].y_m,
                                         g_geo.pod[p].attempt)
            : GEO_POD_NOMINAL_FORCE_N;
    }
    else if (g_geo.pod[p].state == WOK_HAL_GEO_POD_RETRIEVING &&
             dt >= GEO_POD_RETRIEVE_SEC)
    {
        g_geo.pod[p].state = WOK_HAL_GEO_POD_STOWED;
    }
}

/* Any pod (or the legacy mechanism) in motion drives the motor current */
static float geo_current(void)
{
    if (g_geo.motor_current > 0.0f) return g_geo.motor_current;
    for (int p = 0; p < WOK_HAL_GEO_MAX_PODS; p++)
        if (g_geo.pod[p].state == WOK_HAL_GEO_POD_PLACING ||
            g_geo.pod[p].state == WOK_HAL_GEO_POD_RETRIEVING)
            return 1.5f;
    return 0.0f;
}

int32_t WOK_HAL_Geo_PlacePod(uint8_t pod, float x_m, float y_m)
{
    if (!g_geo.initialized || pod >= WOK_HAL_GEO_MAX_PODS) return WOK_HAL_ERROR;
    geo_pod_update(pod);
    if (g_geo.pod[pod].state != WOK_HAL_GEO_POD_STOWED) return WOK_HAL_ERROR;
    g_geo.pod[pod].state     = WOK_HAL_GEO_POD_PLACING;
    g_geo.pod[pod].reseating = 0;
    g_geo.pod[pod].attempt   = 0;
    g_geo.pod[pod].x_m       = x_m;
    g_geo.pod[pod].y_m       = y_m;
    g_geo.pod[pod].cmd_time  = time(NULL);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_ReseatPod(uint8_t pod)
{
    if (!g_geo.initialized || pod >= WOK_HAL_GEO_MAX_PODS) return WOK_HAL_ERROR;
    geo_pod_update(pod);
    if (g_geo.pod[pod].state != WOK_HAL_GEO_POD_PLACED) return WOK_HAL_ERROR;
    g_geo.pod[pod].state     = WOK_HAL_GEO_POD_PLACING;
    g_geo.pod[pod].reseating = 1;
    g_geo.pod[pod].attempt++;
    g_geo.pod[pod].cmd_time  = time(NULL);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_RetrievePod(uint8_t pod)
{
    if (!g_geo.initialized || pod >= WOK_HAL_GEO_MAX_PODS) return WOK_HAL_ERROR;
    geo_pod_update(pod);
    if (g_geo.pod[pod].state != WOK_HAL_GEO_POD_PLACED) return WOK_HAL_ERROR;
    g_geo.pod[pod].state    = WOK_HAL_GEO_POD_RETRIEVING;
    g_geo.pod[pod].cmd_time = time(NULL);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_GetPodState(uint8_t pod, uint8_t *state)
{
    if (!g_geo.initialized || pod >= WOK_HAL_GEO_MAX_PODS || !state)
        return WOK_HAL_ERROR;
    geo_pod_update(pod);
    *state = g_geo.pod[pod].state;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_GetPodSeating(uint8_t pod, float *seat_force_n,
                                  uint8_t *attempt)
{
    if (!g_geo.initialized || pod >= WOK_HAL_GEO_MAX_PODS ||
        !seat_force_n || !attempt)
        return WOK_HAL_ERROR;
    geo_pod_update(pod);
    *seat_force_n = g_geo.pod[pod].seat_force_n;
    *attempt      = g_geo.pod[pod].attempt;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_Deploy(void)
{
    if (!g_geo.initialized) return WOK_HAL_ERROR;
    g_geo.state = WOK_HAL_GEO_DEPLOYING;
    g_geo.cmd_time = time(NULL);
    g_geo.motor_current = 1.5f;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_Stow(void)
{
    if (!g_geo.initialized) return WOK_HAL_ERROR;
    g_geo.state = WOK_HAL_GEO_STOWING;
    g_geo.cmd_time = time(NULL);
    g_geo.motor_current = 1.5f;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_EmergencyStop(void)
{
    if (!g_geo.initialized) return WOK_HAL_ERROR;
    g_geo.motor_current = 0.0f;
    /* Halt pod motions: an interrupted placement retracts with the pod
     * (fresh place -> STOWED, reseat -> back on the ground as-was); an
     * interrupted retrieval leaves the pod on the ground. */
    for (int p = 0; p < WOK_HAL_GEO_MAX_PODS; p++)
    {
        if (g_geo.pod[p].state == WOK_HAL_GEO_POD_PLACING)
            g_geo.pod[p].state = g_geo.pod[p].reseating
                               ? WOK_HAL_GEO_POD_PLACED
                               : WOK_HAL_GEO_POD_STOWED;
        else if (g_geo.pod[p].state == WOK_HAL_GEO_POD_RETRIEVING)
            g_geo.pod[p].state = WOK_HAL_GEO_POD_PLACED;
    }
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_GetState(uint8_t *state)
{
    if (!g_geo.initialized || !state) return WOK_HAL_ERROR;

    double dt = difftime(time(NULL), g_geo.cmd_time);

    if (g_geo.state == WOK_HAL_GEO_DEPLOYING && dt >= GEO_TRANSITION_SEC)
    {
        g_geo.state = WOK_HAL_GEO_DEPLOYED;
        g_geo.motor_current = 0.0f;
    }
    else if (g_geo.state == WOK_HAL_GEO_STOWING && dt >= GEO_TRANSITION_SEC)
    {
        g_geo.state = WOK_HAL_GEO_STOWED;
        g_geo.motor_current = 0.0f;
    }

    *state = g_geo.state;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_GetEndstops(uint8_t *deployed, uint8_t *stowed)
{
    if (!g_geo.initialized || !deployed || !stowed) return WOK_HAL_ERROR;
    /* Force state update */
    uint8_t s;
    WOK_HAL_Geo_GetState(&s);
    *deployed = (g_geo.state == WOK_HAL_GEO_DEPLOYED) ? 1 : 0;
    *stowed   = (g_geo.state == WOK_HAL_GEO_STOWED)   ? 1 : 0;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Geo_GetMotorCurrent(float *amps)
{
    if (!g_geo.initialized || !amps) return WOK_HAL_ERROR;
    for (int p = 0; p < WOK_HAL_GEO_MAX_PODS; p++) geo_pod_update(p);
    *amps = geo_current();
    return WOK_HAL_OK;
}
