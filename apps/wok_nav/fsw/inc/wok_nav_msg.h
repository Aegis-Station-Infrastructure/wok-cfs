/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_NAV — Message Definitions (commands + telemetry)
 ************************************************************************/
#ifndef WOK_NAV_MSG_H
#define WOK_NAV_MSG_H

#include "cfe.h"
#include "wok_mission_types.h"

/* ================================================================== */
/*  Command codes for WOK_NAV_CMD_MID                                 */
/* ================================================================== */
#define WOK_NAV_NOOP_CC          0
#define WOK_NAV_RESET_CTRS_CC    1
#define WOK_NAV_GOTO_CC          2   /* Drive to waypoint              */
#define WOK_NAV_STOP_CC          3   /* Immediate stop                 */
#define WOK_NAV_SET_SPEED_CC     4   /* Override max speed             */
#define WOK_NAV_SET_POSE_CC      5   /* Overwrite dead-reckoned pose   */
#define WOK_NAV_REALIGN_CC       6   /* Corrective turn + drift zero   */

/* ================================================================== */
/*  Command structures                                                */
/* ================================================================== */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
} WOK_Nav_NoArgsCmd_t;

/* GoTo: body-frame relative step (start-frame persistent axes) */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
    float   DeltaForwardM;       /* + = forward in start frame          */
    float   DeltaLateralM;       /* + = left in start frame             */
} WOK_Nav_GoToCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
    float  MaxSpeed_mps;
} WOK_Nav_SetSpeedCmd_t;

/* CC 5: SET_POSE — ground-supplied truth overwrites the dead-reckoned
 * start-frame pose. Bookkeeping only (no motion); accepted from any
 * non-driving state. */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
    float   PoseX_m;             /* start-frame forward                 */
    float   PoseY_m;             /* start-frame lateral (+ = left)      */
} WOK_Nav_SetPoseCmd_t;

/* CC 6: REALIGN — physical heading correction. HeadingErrorDeg is the
 * ground-measured error (actual heading − start-frame heading, + = the
 * rover points left of where it believes); NAV turns −error in place
 * through the normal turn machinery, then declares the current physical
 * heading to be the start-frame heading (drift accumulator zeroed).
 * Rejected unless the drive is IDLE/ARRIVED/STOPPED. */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
    float   HeadingErrorDeg;
} WOK_Nav_RealignCmd_t;

/* ================================================================== */
/*  Housekeeping telemetry  (WOK_NAV_TLM_MID)                        */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint16  CmdCounter;
    uint16  ErrCounter;
    uint8   DriveState;          /* WOK_NavDriveState_t */
    uint8   Spare;
    uint16  FaultMask;
} WOK_Nav_HkTlm_t;

/* ================================================================== */
/*  Nav status telemetry  (WOK_NAV_STATUS_TLM_MID)                   */
/*                                                                    */
/*  Pose is dead-reckoned in the mission start frame:                 */
/*    PoseX_m = accumulated forward distance (start-frame)            */
/*    PoseY_m = accumulated lateral distance (start-frame, + = left)  */
/*    HeadingDriftDeg = accumulated heading deviation from start      */
/*  These are informational only; arrival is gated on per-step        */
/*  encoder/heading deltas, not on global pose.                       */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    float   PoseX_m;             /* dead-reckoned, start-frame forward    */
    float   PoseY_m;             /* dead-reckoned, start-frame lateral    */
    float   HeadingDriftDeg;     /* heading - start_heading               */
    float   Speed_mps;
    float   TiltDeg;
    uint8   DriveState;          /* WOK_NavDriveState_t                   */
    uint8   ArrivedFlag;         /* 1 = current step complete             */
    uint16  FaultMask;           /* mobility faults                       */
    uint16  SeqId;               /* Echo of last cmd SeqId                */
    uint16  SpareSeq;
    /* Ground recalibration (appended — prefix decoders keep working).
     * HeadingDriftDeg above reads as "believed drift since the last
     * realign" once REALIGN has re-declared the start-frame heading. */
    uint16  RealignCount;        /* REALIGNs completed since boot         */
    uint16  SpareR;
} WOK_Nav_StatusTlm_t;

/* Ground decoders unpack these at fixed offsets — catch layout drift at
 * compile time (sizes include the 16-byte telemetry header). */
CompileTimeAssert(sizeof(WOK_Nav_HkTlm_t)     == 24, WOK_Nav_HkTlm_t_WrongSize);
CompileTimeAssert(sizeof(WOK_Nav_StatusTlm_t) == 48, WOK_Nav_StatusTlm_t_WrongSize);

#endif /* WOK_NAV_MSG_H */
