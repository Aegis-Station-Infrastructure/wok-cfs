/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_NAV — Mobility (relative-step model)
 *
 * Each GoTo command is a body-frame (DeltaForwardM, DeltaLateralM) step in
 * the persistent start frame captured at app init. NAV decomposes it into
 * a polar (turn, drive, turn-back) sequence using HAL primitives, and runs
 * the resulting sub-state machine on the 1 Hz HK tick.
 *
 * Pose telemetry (PoseX_m, PoseY_m, HeadingDriftDeg) is dead-reckoned in
 * the start frame from completed sub-states. It's informational only —
 * arrival is gated on per-step encoder/heading deltas, not on global pose.
 *
 * Faults (per-tick during DRIVING):
 *   STUCK       — no progress for StuckTimeoutMs in the active sub-state
 *   OVERCURRENT — motor current above CurrentFaultThreshold_A
 *   TILT        — tilt above TiltFaultThreshold_deg
 *
 * Each step always ends with the rover facing the start-frame forward axis
 * (TURN_BACK restores heading to 0). This keeps per-step math trivial and
 * gives the operator a predictable inter-step orientation.
 ************************************************************************/
#include "wok_nav_app.h"
#include "wok_hal_mobility.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Below this turn magnitude, skip the turn entirely — saves a sub-state
 * transition on near-pure-forward steps and avoids spurious stuck timeouts
 * on motions the HAL would reject (Turn(0)). */
#define WOK_NAV_TURN_EPSILON_DEG  1.0f
#define WOK_NAV_DIST_EPSILON_M    0.01f

static WOK_Nav_AppData_t WOK_Nav_AppData;

static void     WOK_Nav_UpdateSensors(void);
static void     WOK_Nav_RunSubStateMachine(void);
static void     WOK_Nav_RunFaultDetection(float active_progress);
static void     WOK_Nav_PlanStep(float df, float dl);
static void     WOK_Nav_BeginSubTurnTo(void);
static void     WOK_Nav_BeginSubDrive(void);
static void     WOK_Nav_BeginSubTurnBack(void);
static void     WOK_Nav_FinishStep(void);
static void     WOK_Nav_FinishRealign(void);
static void     WOK_Nav_EnterFault(uint16 bit, const char *reason);
static uint32_t WOK_Nav_GetTimeMs(void);

/* ================================================================== */
void WOK_Nav_AppMain(void)
{
    int32            Status;
    CFE_SB_Buffer_t *BufPtr = NULL;

    CFE_ES_PerfLogEntry(WOK_NAV_PERF_ID);
    Status = WOK_Nav_AppInit();
    if (Status != CFE_SUCCESS) return;

    while (true)
    {
        CFE_ES_PerfLogExit(WOK_NAV_PERF_ID);
        Status = CFE_SB_ReceiveBuffer(&BufPtr, WOK_Nav_AppData.CmdPipe,
                                       CFE_SB_PEND_FOREVER);
        CFE_ES_PerfLogEntry(WOK_NAV_PERF_ID);
        if (Status == CFE_SUCCESS)
            WOK_Nav_ProcessCommandPacket(BufPtr);
        else
            break;
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
}

/* ================================================================== */
int32 WOK_Nav_AppInit(void)
{
    int32 Status;
    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&WOK_Nav_AppData.CmdPipe,
                                WOK_NAV_PIPE_DEPTH, WOK_NAV_PIPE_NAME);
    if (Status != CFE_SUCCESS) return Status;

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_NAV_CMD_MID),     WOK_Nav_AppData.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_NAV_SEND_HK_MID), WOK_Nav_AppData.CmdPipe);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Nav_AppData.HkTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_NAV_TLM_MID), sizeof(WOK_Nav_HkTlm_t));
    CFE_MSG_Init(CFE_MSG_PTR(WOK_Nav_AppData.StatusTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_NAV_STATUS_TLM_MID), sizeof(WOK_Nav_StatusTlm_t));

    WOK_Nav_AppData.DriveState              = WOK_NAV_IDLE;
    WOK_Nav_AppData.SubState                = WOK_NAV_SUB_IDLE;
    WOK_Nav_AppData.ArrivedFlag             = 0;
    WOK_Nav_AppData.FaultMask               = WOK_FAULT_NONE;

    WOK_Nav_AppData.MaxSpeed_mps            = WOK_NAV_DEFAULT_MAX_SPEED_MPS;
    WOK_Nav_AppData.ArrivalTolerance_m      = WOK_NAV_DEFAULT_ARRIVAL_TOL_M;
    WOK_Nav_AppData.TiltFaultThreshold_deg  = WOK_NAV_DEFAULT_TILT_LIMIT_DEG;
    WOK_Nav_AppData.CurrentFaultThreshold_A = 4.0f;   /* drive motor */
    WOK_Nav_AppData.StuckTimeoutMs          = 3000;   /* 3 s without progress */

    WOK_Nav_AppData.CmdCounter   = 0;
    WOK_Nav_AppData.ErrCounter   = 0;
    WOK_Nav_AppData.RealignCount = 0;

    WOK_HAL_Mobility_Init();

    CFE_EVS_SendEvent(WOK_NAV_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_NAV: App initialised (relative-step model)");
    return CFE_SUCCESS;
}

/* ================================================================== */
void WOK_Nav_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case WOK_NAV_CMD_MID:
            WOK_Nav_ProcessGroundCommand(BufPtr);
            break;
        case WOK_NAV_SEND_HK_MID:
            WOK_Nav_UpdateSensors();
            WOK_Nav_RunSubStateMachine();
            WOK_Nav_ReportHousekeeping();
            WOK_Nav_PublishStatus();
            break;
        default:
            WOK_Nav_AppData.ErrCounter++;
            break;
    }
}

/* ================================================================== */
/*  Sensors that are read every tick regardless of motion state       */
/* ================================================================== */
static void WOK_Nav_UpdateSensors(void)
{
    WOK_HAL_Mobility_GetSpeed(&WOK_Nav_AppData.Speed_mps);
    WOK_HAL_Mobility_GetTilt(&WOK_Nav_AppData.TiltDeg);
    WOK_HAL_Mobility_GetMotorCurrent(&WOK_Nav_AppData.MotorCurrent_A);
    WOK_HAL_Mobility_IsMoving(&WOK_Nav_AppData.HalMoving);
}

/* ================================================================== */
/*  Ground command dispatcher                                         */
/* ================================================================== */
void WOK_Nav_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t CmdCode = 0;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &CmdCode);

    switch (CmdCode)
    {
        case WOK_NAV_NOOP_CC:
            if (WOK_Nav_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Nav_NoArgsCmd_t)))
            {
                WOK_Nav_AppData.LastSeqId = ((WOK_Nav_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Nav_AppData.CmdCounter++;
                CFE_EVS_SendEvent(WOK_NAV_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_NAV: NOOP");
            }
            break;

        case WOK_NAV_RESET_CTRS_CC:
            if (WOK_Nav_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Nav_NoArgsCmd_t)))
            {
                WOK_Nav_AppData.LastSeqId = ((WOK_Nav_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Nav_AppData.CmdCounter = 0;
                WOK_Nav_AppData.ErrCounter = 0;
            }
            break;

        case WOK_NAV_GOTO_CC:
            if (WOK_Nav_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Nav_GoToCmd_t)))
            {
                WOK_Nav_GoToCmd_t *Cmd = (WOK_Nav_GoToCmd_t *)BufPtr;
                WOK_Nav_AppData.LastSeqId = Cmd->SeqId;
                WOK_Nav_AppData.CmdCounter++;

                /* New step clears motion-related faults and arrival flag */
                WOK_Nav_AppData.FaultMask &=
                    ~(WOK_NAV_FAULT_STUCK | WOK_NAV_FAULT_OVERCURRENT | WOK_FAULT_TILT_EXCEED);
                WOK_Nav_AppData.ArrivedFlag = 0;
                WOK_Nav_AppData.DriveState  = WOK_NAV_DRIVING;

                WOK_Nav_PlanStep(Cmd->DeltaForwardM, Cmd->DeltaLateralM);

                CFE_EVS_SendEvent(WOK_NAV_GOTO_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_NAV: GoTo (df=%.2fm, dl=%.2fm) bearing %.1f° dist %.2fm",
                                  (double)Cmd->DeltaForwardM, (double)Cmd->DeltaLateralM,
                                  (double)WOK_Nav_AppData.PlanBearingDeg,
                                  (double)WOK_Nav_AppData.PlanDistanceM);
            }
            break;

        case WOK_NAV_STOP_CC:
            if (WOK_Nav_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Nav_NoArgsCmd_t)))
            {
                WOK_Nav_AppData.LastSeqId = ((WOK_Nav_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Nav_AppData.CmdCounter++;
                WOK_Nav_AppData.DriveState = WOK_NAV_STOPPED;
                WOK_Nav_AppData.SubState   = WOK_NAV_SUB_IDLE;
                WOK_HAL_Mobility_Stop();
                CFE_EVS_SendEvent(WOK_NAV_STOP_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_NAV: Stop");
            }
            break;

        case WOK_NAV_SET_SPEED_CC:
            if (WOK_Nav_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Nav_SetSpeedCmd_t)))
            {
                WOK_Nav_SetSpeedCmd_t *Cmd = (WOK_Nav_SetSpeedCmd_t *)BufPtr;
                WOK_Nav_AppData.LastSeqId = Cmd->SeqId;
                WOK_Nav_AppData.CmdCounter++;
                WOK_Nav_AppData.MaxSpeed_mps = Cmd->MaxSpeed_mps;
                WOK_HAL_Mobility_SetSpeed(Cmd->MaxSpeed_mps);
            }
            break;

        case WOK_NAV_SET_POSE_CC:
            if (WOK_Nav_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Nav_SetPoseCmd_t)))
            {
                WOK_Nav_SetPoseCmd_t *Cmd = (WOK_Nav_SetPoseCmd_t *)BufPtr;
                WOK_Nav_AppData.LastSeqId = Cmd->SeqId;
                if (WOK_Nav_AppData.DriveState == WOK_NAV_DRIVING)
                {
                    WOK_Nav_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_NAV_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_NAV: SET_POSE rejected — driving");
                }
                else
                {
                    WOK_Nav_AppData.CmdCounter++;
                    CFE_EVS_SendEvent(WOK_NAV_SETPOSE_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_NAV: Pose set (%.2f, %.2f) → (%.2f, %.2f) [ground truth]",
                                      (double)WOK_Nav_AppData.PoseX_m,
                                      (double)WOK_Nav_AppData.PoseY_m,
                                      (double)Cmd->PoseX_m, (double)Cmd->PoseY_m);
                    WOK_Nav_AppData.PoseX_m = Cmd->PoseX_m;
                    WOK_Nav_AppData.PoseY_m = Cmd->PoseY_m;
                }
            }
            break;

        case WOK_NAV_REALIGN_CC:
            if (WOK_Nav_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Nav_RealignCmd_t)))
            {
                WOK_Nav_RealignCmd_t *Cmd = (WOK_Nav_RealignCmd_t *)BufPtr;
                WOK_Nav_AppData.LastSeqId = Cmd->SeqId;
                if (WOK_Nav_AppData.DriveState != WOK_NAV_IDLE &&
                    WOK_Nav_AppData.DriveState != WOK_NAV_ARRIVED &&
                    WOK_Nav_AppData.DriveState != WOK_NAV_STOPPED)
                {
                    WOK_Nav_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_NAV_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_NAV: REALIGN rejected — drive state %u",
                                      (unsigned)WOK_Nav_AppData.DriveState);
                }
                else if (fabsf(Cmd->HeadingErrorDeg) < WOK_NAV_TURN_EPSILON_DEG)
                {
                    /* Below the turn threshold: declare and move on —
                     * same epsilon step planning uses. */
                    WOK_Nav_AppData.CmdCounter++;
                    WOK_Nav_AppData.HeadingDriftDeg = 0.0f;
                    WOK_Nav_AppData.RealignCount++;
                    CFE_EVS_SendEvent(WOK_NAV_REALIGN_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_NAV: Realign #%u — error %.2f° below threshold, no turn; heading declared start-frame",
                                      WOK_Nav_AppData.RealignCount,
                                      (double)Cmd->HeadingErrorDeg);
                }
                else
                {
                    WOK_Nav_AppData.CmdCounter++;
                    /* Commanded motion: clear motion faults + arrival
                     * flag exactly as a new GoTo step does. */
                    WOK_Nav_AppData.FaultMask &=
                        ~(WOK_NAV_FAULT_STUCK | WOK_NAV_FAULT_OVERCURRENT | WOK_FAULT_TILT_EXCEED);
                    WOK_Nav_AppData.ArrivedFlag      = 0;
                    WOK_Nav_AppData.DriveState       = WOK_NAV_DRIVING;
                    WOK_Nav_AppData.SubState         = WOK_NAV_SUB_REALIGN;
                    WOK_Nav_AppData.SubTurnTargetDeg = -Cmd->HeadingErrorDeg;
                    WOK_Nav_AppData.LastProgress     = 0.0f;
                    WOK_Nav_AppData.SubStateStartMs  = WOK_Nav_GetTimeMs();
                    WOK_Nav_AppData.StuckLastTickMs  = WOK_Nav_AppData.SubStateStartMs;
                    WOK_HAL_Mobility_Turn(WOK_Nav_AppData.SubTurnTargetDeg, 0.0f);
                    CFE_EVS_SendEvent(WOK_NAV_REALIGN_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_NAV: Realign — turning %+.2f° to null ground-measured heading error",
                                      (double)WOK_Nav_AppData.SubTurnTargetDeg);
                }
            }
            break;

        default:
            WOK_Nav_AppData.ErrCounter++;
            CFE_EVS_SendEvent(WOK_NAV_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                              "WOK_NAV: Unknown CC %u", CmdCode);
            break;
    }
}

/* ================================================================== */
/*  Decompose (df, dl) into (turn, drive, turn-back) for one step.    */
/*  Each step starts with HeadingDriftDeg == 0 (rover facing start    */
/*  forward) since the previous step's TURN_BACK restores it.         */
/* ================================================================== */
static void WOK_Nav_PlanStep(float df, float dl)
{
    WOK_Nav_AppData.StepDeltaForwardM = df;
    WOK_Nav_AppData.StepDeltaLateralM = dl;

    WOK_Nav_AppData.PlanDistanceM = sqrtf(df * df + dl * dl);
    WOK_Nav_AppData.PlanBearingDeg = (WOK_Nav_AppData.PlanDistanceM > WOK_NAV_DIST_EPSILON_M)
        ? (float)(atan2f(dl, df) * 180.0 / M_PI)
        : 0.0f;
    WOK_Nav_AppData.PlanTurnDeg = WOK_Nav_AppData.PlanBearingDeg;

    /* Trivial step (no motion at all) — arrive immediately */
    if (WOK_Nav_AppData.PlanDistanceM < WOK_NAV_DIST_EPSILON_M)
    {
        WOK_Nav_FinishStep();
        return;
    }

    /* Skip TURN_TO if the bearing is already (approximately) forward */
    if (fabsf(WOK_Nav_AppData.PlanTurnDeg) < WOK_NAV_TURN_EPSILON_DEG)
        WOK_Nav_BeginSubDrive();
    else
        WOK_Nav_BeginSubTurnTo();
}

/* ================================================================== */
static void WOK_Nav_BeginSubTurnTo(void)
{
    WOK_Nav_AppData.SubState         = WOK_NAV_SUB_TURN_TO;
    WOK_Nav_AppData.SubTurnTargetDeg = WOK_Nav_AppData.PlanTurnDeg;
    WOK_Nav_AppData.LastProgress     = 0.0f;
    WOK_Nav_AppData.SubStateStartMs  = WOK_Nav_GetTimeMs();
    WOK_Nav_AppData.StuckLastTickMs  = WOK_Nav_AppData.SubStateStartMs;
    WOK_HAL_Mobility_Turn(WOK_Nav_AppData.SubTurnTargetDeg, 0.0f /* default */);
}

static void WOK_Nav_BeginSubDrive(void)
{
    WOK_Nav_AppData.SubState         = WOK_NAV_SUB_DRIVE;
    WOK_Nav_AppData.SubDriveTargetM  = WOK_Nav_AppData.PlanDistanceM;
    WOK_Nav_AppData.LastProgress     = 0.0f;
    WOK_Nav_AppData.SubStateStartMs  = WOK_Nav_GetTimeMs();
    WOK_Nav_AppData.StuckLastTickMs  = WOK_Nav_AppData.SubStateStartMs;
    WOK_HAL_Mobility_DriveForward(WOK_Nav_AppData.SubDriveTargetM,
                                  WOK_Nav_AppData.MaxSpeed_mps);
}

static void WOK_Nav_BeginSubTurnBack(void)
{
    /* If TURN_TO was skipped, there's nothing to restore */
    if (fabsf(WOK_Nav_AppData.PlanTurnDeg) < WOK_NAV_TURN_EPSILON_DEG)
    {
        WOK_Nav_FinishStep();
        return;
    }
    WOK_Nav_AppData.SubState         = WOK_NAV_SUB_TURN_BACK;
    WOK_Nav_AppData.SubTurnTargetDeg = -WOK_Nav_AppData.PlanTurnDeg;
    WOK_Nav_AppData.LastProgress     = 0.0f;
    WOK_Nav_AppData.SubStateStartMs  = WOK_Nav_GetTimeMs();
    WOK_Nav_AppData.StuckLastTickMs  = WOK_Nav_AppData.SubStateStartMs;
    WOK_HAL_Mobility_Turn(WOK_Nav_AppData.SubTurnTargetDeg, 0.0f);
}

static void WOK_Nav_FinishStep(void)
{
    /* Update dead-reckoned start-frame pose using the planned bearing */
    float bearing_rad = WOK_Nav_AppData.PlanBearingDeg * (float)(M_PI / 180.0);
    WOK_Nav_AppData.PoseX_m += WOK_Nav_AppData.PlanDistanceM * cosf(bearing_rad);
    WOK_Nav_AppData.PoseY_m += WOK_Nav_AppData.PlanDistanceM * sinf(bearing_rad);
    WOK_Nav_AppData.HeadingDriftDeg = 0.0f;  /* TURN_BACK restored heading */

    WOK_Nav_AppData.DriveState  = WOK_NAV_ARRIVED;
    WOK_Nav_AppData.SubState    = WOK_NAV_SUB_IDLE;
    WOK_Nav_AppData.ArrivedFlag = 1;

    CFE_EVS_SendEvent(WOK_NAV_ARRIVED_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_NAV: Arrived (start-frame pose %.2fm, %.2fm)",
                      (double)WOK_Nav_AppData.PoseX_m,
                      (double)WOK_Nav_AppData.PoseY_m);
}

/* Ground-commanded realign complete: the rover's current physical
 * heading IS the start-frame heading from here on — zero the believed
 * drift and count the correction. Pose is unchanged (rotation in place). */
static void WOK_Nav_FinishRealign(void)
{
    WOK_Nav_AppData.HeadingDriftDeg = 0.0f;
    WOK_Nav_AppData.RealignCount++;
    WOK_Nav_AppData.DriveState  = WOK_NAV_ARRIVED;
    WOK_Nav_AppData.SubState    = WOK_NAV_SUB_IDLE;
    WOK_Nav_AppData.ArrivedFlag = 1;

    CFE_EVS_SendEvent(WOK_NAV_REALIGN_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_NAV: Realign #%u complete — turned %+.2f°, heading declared start-frame",
                      WOK_Nav_AppData.RealignCount,
                      (double)WOK_Nav_AppData.SubTurnTargetDeg);
}

/* ================================================================== */
/*  Sub-state machine — runs each HK tick while DriveState==DRIVING   */
/* ================================================================== */
static void WOK_Nav_RunSubStateMachine(void)
{
    if (WOK_Nav_AppData.DriveState != WOK_NAV_DRIVING) return;
    if (WOK_Nav_AppData.SubState   == WOK_NAV_SUB_IDLE) return;

    /* Read the active accumulator + report progress for fault detection */
    float distance_done = 0.0f, heading_done = 0.0f;
    WOK_HAL_Mobility_GetDistanceTraveled(&distance_done);
    WOK_HAL_Mobility_GetHeadingChange(&heading_done);
    WOK_Nav_AppData.HalDistanceM = distance_done;
    WOK_Nav_AppData.HalHeadingDeg = heading_done;

    float active_progress = 0.0f;

    switch (WOK_Nav_AppData.SubState)
    {
        case WOK_NAV_SUB_TURN_TO:
            active_progress = fabsf(heading_done);
            if (active_progress + 0.5f >= fabsf(WOK_Nav_AppData.SubTurnTargetDeg))
            {
                WOK_Nav_AppData.HeadingDriftDeg = WOK_Nav_AppData.PlanBearingDeg;
                WOK_Nav_BeginSubDrive();
                return;
            }
            break;

        case WOK_NAV_SUB_DRIVE:
            active_progress = distance_done;
            if (active_progress + WOK_Nav_AppData.ArrivalTolerance_m
                >= WOK_Nav_AppData.SubDriveTargetM)
            {
                WOK_Nav_BeginSubTurnBack();
                return;
            }
            break;

        case WOK_NAV_SUB_TURN_BACK:
            active_progress = fabsf(heading_done);
            if (active_progress + 0.5f >= fabsf(WOK_Nav_AppData.SubTurnTargetDeg))
            {
                WOK_Nav_FinishStep();
                return;
            }
            break;

        case WOK_NAV_SUB_REALIGN:
            active_progress = fabsf(heading_done);
            if (active_progress + 0.5f >= fabsf(WOK_Nav_AppData.SubTurnTargetDeg))
            {
                WOK_Nav_FinishRealign();
                return;
            }
            break;

        default: return;
    }

    WOK_Nav_RunFaultDetection(active_progress);
}

/* ================================================================== */
/*  Fault detection during DRIVING                                    */
/* ================================================================== */
static void WOK_Nav_RunFaultDetection(float active_progress)
{
    /* Tilt — also caught at HK app level, but trip locally so the rover
     * stops mid-step rather than drifting until the next 1Hz HK cycle. */
    if (WOK_Nav_AppData.TiltDeg > WOK_Nav_AppData.TiltFaultThreshold_deg)
    {
        WOK_Nav_EnterFault(WOK_FAULT_TILT_EXCEED, "tilt exceeded");
        return;
    }

    /* Overcurrent (drive motor) */
    if (WOK_Nav_AppData.MotorCurrent_A > WOK_Nav_AppData.CurrentFaultThreshold_A)
    {
        WOK_Nav_EnterFault(WOK_NAV_FAULT_OVERCURRENT, "motor overcurrent");
        return;
    }

    /* Stuck — no progress in StuckTimeoutMs, and HAL is supposedly moving */
    uint32_t now = WOK_Nav_GetTimeMs();
    if (active_progress > WOK_Nav_AppData.LastProgress + 0.001f)
    {
        WOK_Nav_AppData.LastProgress    = active_progress;
        WOK_Nav_AppData.StuckLastTickMs = now;
    }
    else if ((now - WOK_Nav_AppData.StuckLastTickMs) > WOK_Nav_AppData.StuckTimeoutMs)
    {
        WOK_Nav_EnterFault(WOK_NAV_FAULT_STUCK, "no encoder/heading progress");
    }
}

static void WOK_Nav_EnterFault(uint16 bit, const char *reason)
{
    WOK_Nav_AppData.FaultMask  |= bit;
    WOK_Nav_AppData.DriveState  = WOK_NAV_FAULT;
    WOK_Nav_AppData.SubState    = WOK_NAV_SUB_IDLE;
    WOK_Nav_AppData.ArrivedFlag = 0;
    WOK_HAL_Mobility_Stop();

    uint16 eid = WOK_NAV_FAULT_STUCK_EID;
    if      (bit == WOK_NAV_FAULT_OVERCURRENT) eid = WOK_NAV_FAULT_OVERCUR_EID;
    else if (bit == WOK_FAULT_TILT_EXCEED)     eid = WOK_NAV_FAULT_TILT_EID;

    CFE_EVS_SendEvent(eid, CFE_EVS_EventType_ERROR,
                      "WOK_NAV: FAULT — %s (sub %u, mask 0x%04X)",
                      reason, (unsigned)WOK_Nav_AppData.SubState,
                      WOK_Nav_AppData.FaultMask);
}

/* ================================================================== */
void WOK_Nav_ReportHousekeeping(void)
{
    WOK_Nav_AppData.HkTlm.CmdCounter = WOK_Nav_AppData.CmdCounter;
    WOK_Nav_AppData.HkTlm.ErrCounter = WOK_Nav_AppData.ErrCounter;
    WOK_Nav_AppData.HkTlm.DriveState = (uint8)WOK_Nav_AppData.DriveState;
    WOK_Nav_AppData.HkTlm.FaultMask  = WOK_Nav_AppData.FaultMask;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Nav_AppData.HkTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Nav_AppData.HkTlm.TlmHeader), true);
}

void WOK_Nav_PublishStatus(void)
{
    WOK_Nav_AppData.StatusTlm.PoseX_m          = WOK_Nav_AppData.PoseX_m;
    WOK_Nav_AppData.StatusTlm.PoseY_m          = WOK_Nav_AppData.PoseY_m;
    WOK_Nav_AppData.StatusTlm.HeadingDriftDeg  = WOK_Nav_AppData.HeadingDriftDeg;
    WOK_Nav_AppData.StatusTlm.Speed_mps        = WOK_Nav_AppData.Speed_mps;
    WOK_Nav_AppData.StatusTlm.TiltDeg          = WOK_Nav_AppData.TiltDeg;
    WOK_Nav_AppData.StatusTlm.DriveState       = (uint8)WOK_Nav_AppData.DriveState;
    WOK_Nav_AppData.StatusTlm.ArrivedFlag      = WOK_Nav_AppData.ArrivedFlag;
    WOK_Nav_AppData.StatusTlm.FaultMask        = WOK_Nav_AppData.FaultMask;
    WOK_Nav_AppData.StatusTlm.SeqId            = WOK_Nav_AppData.LastSeqId;
    WOK_Nav_AppData.StatusTlm.RealignCount     = WOK_Nav_AppData.RealignCount;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Nav_AppData.StatusTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Nav_AppData.StatusTlm.TlmHeader), true);
}

bool WOK_Nav_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    size_t ActualLength = 0;
    CFE_MSG_GetSize(MsgPtr, &ActualLength);
    if (ActualLength != ExpectedLength)
    {
        WOK_Nav_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_NAV_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_NAV: Len err: exp %lu got %lu",
                          (unsigned long)ExpectedLength, (unsigned long)ActualLength);
        return false;
    }
    return true;
}

/* ================================================================== */
/*  Helpers                                                           */
/* ================================================================== */

/* uint64 multiply prevents overflow before final uint32 truncation;
 * delta arithmetic is mod-2^32 correct. */
static uint32_t WOK_Nav_GetTimeMs(void)
{
    CFE_TIME_SysTime_t t = CFE_TIME_GetTime();
    return (uint32_t)((uint64_t)t.Seconds * 1000ull + t.Subseconds / 4294967u);
}

