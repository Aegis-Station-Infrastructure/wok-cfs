/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_HK — Message Definitions (commands + telemetry)
 *
 * Combined health / housekeeping / FDIR-lite app.
 * Aggregates system health and provides watchdog + SAFE trigger logic.
 ************************************************************************/
#ifndef WOK_HK_MSG_H
#define WOK_HK_MSG_H

#include "cfe.h"
#include "wok_mission_types.h"

/* ================================================================== */
/*  Command codes for WOK_HK_CMD_MID                                  */
/* ================================================================== */
#define WOK_HK_NOOP_CC           0
#define WOK_HK_RESET_CTRS_CC     1
#define WOK_HK_SET_THRESHOLDS_CC 2   /* Update fault thresholds        */
#define WOK_HK_CLEAR_FAULTS_CC   3   /* Operator fault acknowledge     */

/* ================================================================== */
/*  Command structures                                                */
/* ================================================================== */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
} WOK_Hk_NoArgsCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    float  BatteryMin_V;
    float  TempMax_C;
    float  TiltMax_deg;
} WOK_Hk_SetThresholdsCmd_t;

/* ================================================================== */
/*  Combined health telemetry  (WOK_HK_TLM_MID)                      */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;

    /* Counters */
    uint16  CmdCounter;
    uint16  ErrCounter;

    /* System health */
    float   BatteryV;
    float   BatterySOC_pct;      /* state of charge, percent          */
    float   TempBoard_C;
    float   TempMotor_C;
    float   TiltDeg;
    float   CPULoad_pct;

    /* Aggregated fault state */
    uint16  FaultMask;           /* WOK_FAULT_* common bits           */
    uint8   WatchdogOK;          /* 1 = all apps responsive           */
    uint8   SafeTrigger;         /* 1 = FDIR wants SAFE mode          */

    /* Current mission state (echoed from SEQ) */
    uint8   SeqState;            /* WOK_State_t                       */
    uint8   Spare[3];
} WOK_Hk_HealthTlm_t;

/* Ground decoders unpack this at fixed offsets — catch layout drift at
 * compile time (size includes the 16-byte telemetry header). */
CompileTimeAssert(sizeof(WOK_Hk_HealthTlm_t) == 52, WOK_Hk_HealthTlm_t_WrongSize);

#endif /* WOK_HK_MSG_H */
