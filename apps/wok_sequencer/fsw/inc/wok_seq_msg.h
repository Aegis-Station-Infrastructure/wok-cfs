/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_SEQ — Message Definitions (commands + telemetry)
 *
 * PUBLIC header — other apps include this to understand SEQ messages.
 ************************************************************************/
#ifndef WOK_SEQ_MSG_H
#define WOK_SEQ_MSG_H

#include "cfe.h"
#include "wok_mission_types.h"

/* ================================================================== */
/*  Ground command codes (CC) for WOK_SEQ_CMD_MID                     */
/* ================================================================== */
#define WOK_SEQ_NOOP_CC          0   /* No-op                           */
#define WOK_SEQ_RESET_CTRS_CC    1   /* Reset command / error counters   */
#define WOK_SEQ_START_CC         2   /* Begin waypoint loop              */
#define WOK_SEQ_ABORT_CC         3   /* Abort to SAFE immediately        */
#define WOK_SEQ_RESUME_CC        4   /* Resume from SAFE if possible     */
#define WOK_SEQ_SET_WAYPOINT_CC  5   /* Override next waypoint index     */
#define WOK_SEQ_SET_MODE_CC      6   /* Drill-as-you-go vs survey-first  */
#define WOK_SEQ_LOAD_WAYPOINT_CC 7   /* Upload/replace one waypoint      */
#define WOK_SEQ_SET_WP_COUNT_CC  8   /* Set number of active waypoints   */
#define WOK_SEQ_SET_PLACEMENT_CC 9   /* Fixed array vs per-pod placement */
#define WOK_SEQ_RELEASE_CC       10  /* Release a recalibration HOLD     */

/* Mission modes (SET_MODE) */
#define WOK_SEQ_MODE_DRILL_INLINE   0  /* drill at flagged sites as found */
#define WOK_SEQ_MODE_SURVEY_FIRST   1  /* survey all, then drill argmax   */

/* Placement modes (SET_PLACEMENT, CP-3) */
#define WOK_SEQ_PLACEMENT_FIXED     0  /* legacy fixed-array deploy/stow  */
#define WOK_SEQ_PLACEMENT_PER_POD   1  /* rover places each pod (WOK-CP-001) */

/* ================================================================== */
/*  Command structures                                                */
/* ================================================================== */

/* Generic no-argument command (NOOP, RESET, ABORT, RESUME, START) */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
} WOK_Seq_NoArgsCmd_t;

/* Set waypoint index override */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16                  WaypointIdx;
    uint16                  Spare;
} WOK_Seq_SetWaypointCmd_t;

/* Set mission mode */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8                   Mode;       /* WOK_SEQ_MODE_*                */
    uint8                   Spare[3];
} WOK_Seq_SetModeCmd_t;

/* Upload/replace one waypoint (ground retargeting). SourceMode +
 * SourceEnergyJ let ground command the seismic source per site;
 * RecalFlag holds after MOVE for ground recalibration. Zeros select
 * the defaults (impulse at the app energy, no hold). */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16                  WaypointIdx;
    uint8                   DrillFlag;
    uint8                   SourceMode;    /* 0 = impulse, 1 = sweep   */
    float                   DeltaForwardM;
    float                   DeltaLateralM;
    float                   SourceEnergyJ; /* 0 = app default          */
    uint8                   RecalFlag;     /* 1 = HOLD after MOVE      */
    uint8                   SpareR[3];
} WOK_Seq_LoadWaypointCmd_t;

/* Set number of active waypoints */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16                  Count;
    uint16                  Spare;
} WOK_Seq_SetWpCountCmd_t;

/* Set placement mode (CP-3). Default FIXED preserves the legacy
 * single-mechanism DEPLOY/STOW exactly (command stream and SeqIds). */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8                   Placement;  /* WOK_SEQ_PLACEMENT_*          */
    uint8                   Spare[3];
} WOK_Seq_SetPlacementCmd_t;

/* ================================================================== */
/*  Housekeeping telemetry  (WOK_SEQ_TLM_MID)                        */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint16  CmdCounter;
    uint16  ErrCounter;
    uint8   CurrentState;       /* WOK_State_t  */
    uint8   PreviousState;
    uint16  CurrentWaypointIdx;
    uint16  FaultMask;          /* WOK_FAULT_* bits */
    uint16  SeqId;              /* Current monotonic sequence ID */
} WOK_Seq_HkTlm_t;

/* ================================================================== */
/*  Mode telemetry  (WOK_SEQ_MODE_TLM_MID)                           */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8   State;              /* WOK_State_t */
    uint8   LoopCount;          /* How many full cycles completed */
    uint16  TotalWaypoints;
    uint16  FaultMask;
    uint16  Spare;
    /* Targeting status (appended — prefix decoders keep working) */
    uint8   MissionMode;        /* WOK_SEQ_MODE_*                     */
    uint8   DrillsCompleted;    /* tie-point gate progress            */
    uint8   BestSiteIdx;        /* argmax-score waypoint (0xFF none)  */
    uint8   SpareT;
    float   BestSiteScore;      /* its EvalIceFrac                    */
} WOK_Seq_ModeTlm_t;

/* Ground decoders unpack these at fixed offsets — catch layout drift at
 * compile time (sizes include the 16-byte telemetry header). */
CompileTimeAssert(sizeof(WOK_Seq_HkTlm_t)   == 28, WOK_Seq_HkTlm_t_WrongSize);
CompileTimeAssert(sizeof(WOK_Seq_ModeTlm_t) == 32, WOK_Seq_ModeTlm_t_WrongSize);

#endif /* WOK_SEQ_MSG_H */
