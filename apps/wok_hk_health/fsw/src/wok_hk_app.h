/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_HK — Application Private Header
 ************************************************************************/
#ifndef WOK_HK_APP_H
#define WOK_HK_APP_H

#include "cfe.h"
#include "wok_msgids.h"
#include "wok_perfids.h"
#include "wok_mission_types.h"
#include "wok_platform_cfg.h"
#include "wok_hk_msg.h"
#include "wok_hk_events.h"

#define WOK_HK_PIPE_NAME   "WOK_HK_PIPE"

typedef struct
{
    CFE_SB_PipeId_t       CmdPipe;
    WOK_Hk_HealthTlm_t   HealthTlm;

    /* Raw sensor readings (populated from HAL or other apps' TLM) */
    float   BatteryV;
    float   BatterySOC_pct;
    float   TempBoard_C;
    float   TempMotor_C;
    float   TiltDeg;
    float   CPULoad_pct;

    /* Thresholds (table-driven) */
    float   BatteryMin_V;
    float   TempMax_C;
    float   TiltMax_deg;

    /* FDIR state */
    uint16  FaultMask;
    uint8   WatchdogOK;
    uint8   SafeTrigger;

    /* Cached sequencer state (from SEQ mode TLM) */
    uint8   SeqState;

    uint16  CmdCounter;
    uint16  ErrCounter;
} WOK_Hk_AppData_t;

void  WOK_Hk_AppMain(void);
int32 WOK_Hk_AppInit(void);
void  WOK_Hk_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr);
void  WOK_Hk_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr);
void  WOK_Hk_ReportHealth(void);
void  WOK_Hk_CheckFaults(void);
bool  WOK_Hk_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

#endif /* WOK_HK_APP_H */
