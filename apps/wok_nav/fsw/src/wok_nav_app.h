/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_NAV — Application Private Header (relative-step model)
 *
 * NAV executes one relative step at a time:
 *   GoTo (DeltaForwardM, DeltaLateralM)  in the persistent start frame
 *
 *   1. Capture target in start-frame.
 *   2. Compute polar (dist, bearing) and correct for accumulated heading
 *      drift to find the heading the rover must face.
 *   3. Sub-state machine:
 *        SUB_TURN_TO   → HAL Turn until heading delta matches
 *        SUB_DRIVE     → HAL DriveForward until distance matches
 *        SUB_TURN_BACK → HAL Turn back to start-frame heading
 *      → ARRIVED, with SeqId echoed.
 *   4. Fault detection on each sub-state:
 *        STUCK         → no encoder/heading progress over T seconds
 *        OVERCURRENT   → motor current above limit
 *        TILT          → tilt above limit (also caught by HK)
 *
 * Pose telemetry is dead-reckoned from completed sub-states. Used for
 * ground monitoring; not load-bearing for arrival decisions.
 ************************************************************************/
#ifndef WOK_NAV_APP_H
#define WOK_NAV_APP_H

#include "cfe.h"
#include "wok_msgids.h"
#include "wok_perfids.h"
#include "wok_mission_types.h"
#include "wok_platform_cfg.h"
#include "wok_nav_msg.h"
#include "wok_nav_events.h"

#define WOK_NAV_PIPE_NAME   "WOK_NAV_PIPE"

/* NAV-specific fault bits (upper byte; common bits 0x00FF live in the
 * mission-wide fault set). */
#define WOK_NAV_FAULT_STUCK       0x0100
#define WOK_NAV_FAULT_SLIP        0x0200
#define WOK_NAV_FAULT_OVERCURRENT 0x0400

/* NAV sub-state — only meaningful while DriveState == DRIVING. */
typedef enum {
    WOK_NAV_SUB_IDLE       = 0,
    WOK_NAV_SUB_TURN_TO    = 1,  /* turning toward step bearing       */
    WOK_NAV_SUB_DRIVE      = 2,  /* straight drive of polar distance  */
    WOK_NAV_SUB_TURN_BACK  = 3,  /* restoring start-frame heading     */
    WOK_NAV_SUB_REALIGN    = 4,  /* ground-commanded corrective turn  */
} WOK_NavSubState_t;

typedef struct
{
    CFE_SB_PipeId_t      CmdPipe;
    WOK_Nav_HkTlm_t      HkTlm;
    WOK_Nav_StatusTlm_t  StatusTlm;

    /* Drive state (top level + sub-state for DRIVING) */
    WOK_NavDriveState_t  DriveState;
    WOK_NavSubState_t    SubState;
    uint8                ArrivedFlag;

    /* Current step target (body frame, persistent start frame) */
    float  StepDeltaForwardM;
    float  StepDeltaLateralM;

    /* Decomposed polar plan for current step */
    float  PlanDistanceM;        /* sqrt(df^2 + dl^2)                  */
    float  PlanBearingDeg;       /* atan2(dl, df), start-frame bearing */
    float  PlanTurnDeg;          /* turn from current heading to bearing */

    /* HAL progress snapshots, per sub-state */
    float  SubTurnTargetDeg;     /* signed: + = turn left              */
    float  SubDriveTargetM;
    float  HalDistanceM;
    float  HalHeadingDeg;
    uint8  HalMoving;

    /* Continuous sensor reads */
    float  Speed_mps;
    float  TiltDeg;
    float  MotorCurrent_A;

    /* Dead-reckoned pose in start frame (informational telemetry) */
    float  PoseX_m;              /* + = forward in start frame         */
    float  PoseY_m;              /* + = left    in start frame         */
    float  HeadingDriftDeg;      /* believed drift since last realign  */

    /* Ground recalibration */
    uint16 RealignCount;         /* REALIGNs completed since boot      */

    /* Stuck detection */
    float    LastProgress;       /* distance/heading at last check    */
    uint32_t StuckLastTickMs;    /* timestamp of last progress sample */
    uint32_t SubStateStartMs;    /* timestamp sub-state was entered   */

    /* Config (could go to TBL later — for now from wok_platform_cfg.h) */
    float  MaxSpeed_mps;
    float  ArrivalTolerance_m;
    float  TiltFaultThreshold_deg;
    float  CurrentFaultThreshold_A;
    uint16 StuckTimeoutMs;       /* no-progress window                 */

    /* SeqId echo */
    uint16 LastSeqId;

    /* Fault */
    uint16 FaultMask;

    /* Counters */
    uint16 CmdCounter;
    uint16 ErrCounter;
} WOK_Nav_AppData_t;

void  WOK_Nav_AppMain(void);
int32 WOK_Nav_AppInit(void);
void  WOK_Nav_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr);
void  WOK_Nav_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr);
void  WOK_Nav_ReportHousekeeping(void);
void  WOK_Nav_PublishStatus(void);
bool  WOK_Nav_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

#endif /* WOK_NAV_APP_H */
