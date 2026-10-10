/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Power & Thermal
** File: wok_common/hal/src/linux_sim/wok_hal_sim_power.c
**
** Battery starts 25.2V, drains 0.001V/sec. SOC starts 92%, drains 0.01%/sec.
** Board temp 28°C ± 2°C random noise. Motor temp 35°C ± 3°C.
*/

#include "wok_hal_power.h"
#include "wok_hal_sim_power.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Consumer loads sag the local copy's bus voltage (see header note on
 * per-app HAL copies). 0.05 V per kJ drawn. */
#define SIM_BATT_SAG_V_PER_KJ  0.05f

static struct {
    uint8_t  initialized;
    float    batt_v;
    float    batt_soc;
    float    drained_j;       /* local consumer draw (bank charging etc.) */
    time_t   start_time;
} g_pwr;

int32_t WOK_HAL_Power_Init(void)
{
    /* Idempotent: the EM-source sim init also brings up the local power
     * copy — don't restart the discharge clock if already running. */
    if (g_pwr.initialized) return WOK_HAL_OK;
    g_pwr.initialized = 1;
    g_pwr.batt_v = 25.2f;
    g_pwr.batt_soc = 92.0f;
    g_pwr.drained_j = 0.0f;
    g_pwr.start_time = time(NULL);
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Power_GetBatteryV(float *v)
{
    if (!g_pwr.initialized || !v) return WOK_HAL_ERROR;
    double dt = difftime(time(NULL), g_pwr.start_time);
    *v = g_pwr.batt_v - (float)(dt * 0.001)
       - g_pwr.drained_j * (SIM_BATT_SAG_V_PER_KJ / 1000.0f);
    if (*v < 18.0f) *v = 18.0f;
    return WOK_HAL_OK;
}

void WOK_HAL_SimPower_Drain_J(float joules)
{
    if (!g_pwr.initialized || joules <= 0.0f) return;
    g_pwr.drained_j += joules;
}

void WOK_HAL_SimPower_SetBattery(float volts)
{
    if (!g_pwr.initialized) WOK_HAL_Power_Init();
    g_pwr.batt_v     = volts;
    g_pwr.drained_j  = 0.0f;
    g_pwr.start_time = time(NULL);
}

int32_t WOK_HAL_Power_GetBatterySOC(float *pct)
{
    if (!g_pwr.initialized || !pct) return WOK_HAL_ERROR;
    double dt = difftime(time(NULL), g_pwr.start_time);
    *pct = g_pwr.batt_soc - (float)(dt * 0.01);
    if (*pct < 0.0f) *pct = 0.0f;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Power_GetBoardTemp(float *c)
{
    if (!g_pwr.initialized || !c) return WOK_HAL_ERROR;
    *c = 28.0f + (float)(rand() % 400 - 200) / 100.0f;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Power_GetMotorTemp(float *c)
{
    if (!g_pwr.initialized || !c) return WOK_HAL_ERROR;
    *c = 35.0f + (float)(rand() % 600 - 300) / 100.0f;
    return WOK_HAL_OK;
}

int32_t WOK_HAL_Power_GetCPULoad(float *pct)
{
    /* Sim: read /proc/loadavg's 1-minute load average and scale to a 0-100
     * "%". Real flight target should use a CPU utilization counter via PSP.
     * Direct file read avoids the _DEFAULT_SOURCE requirement of getloadavg. */
    if (!g_pwr.initialized || !pct) return WOK_HAL_ERROR;
    FILE *f = fopen("/proc/loadavg", "r");
    if (!f) return WOK_HAL_ERROR;
    double load1 = 0.0;
    int n = fscanf(f, "%lf", &load1);
    fclose(f);
    if (n != 1) return WOK_HAL_ERROR;
    float p = (float)(load1 * 100.0);
    if (p > 100.0f) p = 100.0f;
    if (p < 0.0f)   p = 0.0f;
    *pct = p;
    return WOK_HAL_OK;
}
