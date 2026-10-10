/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Flight Stub (all subsystems except drill)
**
** Starting point for a hardware port. Every function is a no-op that
** returns WOK_HAL_ERROR and leaves its output arguments untouched, so
** an unported subsystem fails loudly (HAL error counts, app faults)
** rather than reporting fabricated values. Replace each body with the
** real driver call — see docs/WOK_HAL_Porting_Guide.md for the
** walkthrough. Selected with -DWOK_HAL_BACKEND=flight_stub (the
** default is linux_sim). The drill subsystem's stub lives in
** wok_hal_drill_flight_stub.c.
*/
#include "wok_hal.h"
#include "wok_hal_mobility.h"
#include "wok_hal_geo.h"
#include "wok_hal_seis.h"
#include "wok_hal_power.h"
#include "wok_hal_storage.h"
#include "wok_hal_watchdog.h"
int32_t WOK_HAL_Init(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Shutdown(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_Init(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_Stop(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_SetSpeed(float s)
{
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_DriveForward(float d,float s)
{
    (void)d;
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_Turn(float a,float s)
{
    (void)a;
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_GetDistanceTraveled(float*m)
{
    (void)m;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_GetHeadingChange(float*d)
{
    (void)d;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_IsMoving(uint8_t*m)
{
    (void)m;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_GetSpeed(float*s)
{
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_GetTilt(float*t)
{
    (void)t;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Mobility_GetMotorCurrent(float*c)
{
    (void)c;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_Init(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_Deploy(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_Stow(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_EmergencyStop(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_GetState(uint8_t*s)
{
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_GetEndstops(uint8_t*d,uint8_t*s)
{
    (void)d;
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_GetMotorCurrent(float*c)
{
    (void)c;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_PlacePod(uint8_t p,float x,float y)
{
    (void)p;
    (void)x;
    (void)y;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_ReseatPod(uint8_t p)
{
    (void)p;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_RetrievePod(uint8_t p)
{
    (void)p;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_GetPodState(uint8_t p,uint8_t*s)
{
    (void)p;
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Geo_GetPodSeating(uint8_t p,float*f,uint8_t*a)
{
    (void)p;
    (void)f;
    (void)a;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_Init(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_Configure(uint16_t f,uint8_t g,uint8_t c)
{
    (void)f;
    (void)g;
    (void)c;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_SetSite(float x,float y,float h)
{
    (void)x;
    (void)y;
    (void)h;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_SetPodGeometry(uint8_t p,float x,float y,uint8_t a)
{
    (void)p;
    (void)x;
    (void)y;
    (void)a;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_StartAcquisition(uint16_t d)
{
    (void)d;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_StopAcquisition(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_GetSamples(int32_t*b,uint32_t m,uint32_t*a)
{
    (void)b;
    (void)m;
    (void)a;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_GetStatus(uint8_t*a,uint32_t*s)
{
    (void)a;
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_GetStrikeInfo(uint32_t*i,uint8_t*v)
{
    (void)i;
    (void)v;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_GetQuicklook(float*r,float*p,float*d,uint8_t n)
{
    (void)r;
    (void)p;
    (void)d;
    (void)n;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Seis_Evaluate(uint8_t*f,float*c)
{
    (void)f;
    (void)c;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_Init(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_Configure(uint8_t m,float e,float f0,float f1,float d)
{
    (void)m;
    (void)e;
    (void)f0;
    (void)f1;
    (void)d;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_Arm(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_Fire(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_Reset(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_Abort(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_GetState(uint8_t*s)
{
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_GetBank(float*v,float*p)
{
    (void)v;
    (void)p;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_GetShotInfo(float*f,uint32_t*t)
{
    (void)f;
    (void)t;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_GetSweepProgress(float*f)
{
    (void)f;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Src_ReseatPlate(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Power_Init(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Power_GetBatteryV(float*v)
{
    (void)v;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Power_GetBatterySOC(float*s)
{
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Power_GetBoardTemp(float*t)
{
    (void)t;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Power_GetMotorTemp(float*t)
{
    (void)t;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Power_GetCPULoad(float*p)
{
    (void)p;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Storage_Init(void)
{
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Storage_WriteFile(const char*f,const void*d,uint32_t s)
{
    (void)f;
    (void)d;
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Storage_ReadFile(const char*f,void*b,uint32_t m,uint32_t*s)
{
    (void)f;
    (void)b;
    (void)m;
    (void)s;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Storage_GetFreeSpace(uint64_t*b)
{
    (void)b;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Watchdog_Init(uint32_t t)
{
    (void)t;
    return WOK_HAL_ERROR;
}
int32_t WOK_HAL_Watchdog_Pet(void)
{
    return WOK_HAL_ERROR;
}
