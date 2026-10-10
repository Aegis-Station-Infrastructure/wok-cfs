/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_GEO — Application Private Header
 ************************************************************************/
#ifndef WOK_GEO_APP_H
#define WOK_GEO_APP_H

#include "cfe.h"
#include "wok_msgids.h"
#include "wok_perfids.h"
#include "wok_mission_types.h"
#include "wok_platform_cfg.h"
#include "wok_geo_msg.h"
#include "wok_geo_events.h"
#include "wok_hal_geo.h"

#define WOK_GEO_PIPE_NAME   "WOK_GEO_PIPE"

/* GEO-specific fault bits */
#define WOK_GEO_FAULT_JAM         0x0100
#define WOK_GEO_FAULT_TIMEOUT     0x0200
#define WOK_GEO_FAULT_OVERCURRENT 0x0400

typedef struct
{
    CFE_SB_PipeId_t       CmdPipe;
    uint32                RunStatus;
    WOK_Geo_HkTlm_t      HkTlm;
    WOK_Geo_StatusTlm_t   StatusTlm;

    WOK_GeoMechState_t    MechState;
    uint8   EndstopDeployed;
    uint8   EndstopStowed;
    float   MotorCurrent_A;
    uint16  FaultMask;
    uint16  LastSeqId;
    uint16  TimeoutCounter;
    uint32  CmdStartMs;     /* time DEPLOY/STOW was issued, for timeout */

    /* Config (from table) */
    float   MotorCurrentLimit_A;
    uint16  DeployTimeoutMs;
    uint16  StowTimeoutMs;

    /* Per-pod placement (CP-3): the app owns the seat -> force-proxy ->
     * reseat loop; the sequencer only sees terminal pod states. */
    struct
    {
        uint8   State;           /* WOK_GEO_POD_* (app state)          */
        uint8   Reseats;         /* reseat attempts at this position   */
        uint8   SingleReseat;    /* current PLACING is a commanded
                                    single reseat (CP-4): no internal
                                    retry loop on a low force reading  */
        float   SeatForce_N;
        float   X_m, Y_m;
        uint32  CmdStartMs;      /* motion-timeout reference           */
    } Pod[WOK_HAL_GEO_MAX_PODS];
    uint8   ActivePod;           /* pod in motion; 0xFF = none         */

    uint16  CmdCounter;
    uint16  ErrCounter;
} WOK_Geo_AppData_t;

void  WOK_Geo_AppMain(void);
int32 WOK_Geo_AppInit(void);
void  WOK_Geo_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr);
void  WOK_Geo_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr);
void  WOK_Geo_ReportHousekeeping(void);
void  WOK_Geo_PublishStatus(void);
bool  WOK_Geo_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

#endif /* WOK_GEO_APP_H */
