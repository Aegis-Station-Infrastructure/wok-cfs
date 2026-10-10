/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_GEO — Geophone Mechanism Controller (HAL-integrated)
 *
 * Calls WOK_HAL_Geo_* to deploy, stow, read endstops/current.
 ************************************************************************/
#include "wok_geo_app.h"
#include "wok_hal_geo.h"

static WOK_Geo_AppData_t WOK_Geo_AppData;

static void     WOK_Geo_UpdateFromHAL(void);
static void     WOK_Geo_RunFaultDetection(void);
static void     WOK_Geo_RunPodMachine(void);
static uint32_t WOK_Geo_GetTimeMs(void);

void WOK_Geo_AppMain(void)
{
    int32 Status; CFE_SB_Buffer_t *BufPtr = NULL;
    CFE_ES_PerfLogEntry(WOK_GEO_PERF_ID);
    Status = WOK_Geo_AppInit();
    if (Status != CFE_SUCCESS) return;
    while (true) {
        CFE_ES_PerfLogExit(WOK_GEO_PERF_ID);
        Status = CFE_SB_ReceiveBuffer(&BufPtr, WOK_Geo_AppData.CmdPipe, CFE_SB_PEND_FOREVER);
        CFE_ES_PerfLogEntry(WOK_GEO_PERF_ID);
        if (Status == CFE_SUCCESS) WOK_Geo_ProcessCommandPacket(BufPtr);
        else break;
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
}

int32 WOK_Geo_AppInit(void)
{
    int32 Status;
    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;
    Status = CFE_SB_CreatePipe(&WOK_Geo_AppData.CmdPipe, WOK_GEO_PIPE_DEPTH, WOK_GEO_PIPE_NAME);
    if (Status != CFE_SUCCESS) return Status;

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_GEO_CMD_MID),     WOK_Geo_AppData.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_GEO_SEND_HK_MID), WOK_Geo_AppData.CmdPipe);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Geo_AppData.HkTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_GEO_TLM_MID), sizeof(WOK_Geo_HkTlm_t));
    CFE_MSG_Init(CFE_MSG_PTR(WOK_Geo_AppData.StatusTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_GEO_STATUS_TLM_MID), sizeof(WOK_Geo_StatusTlm_t));

    WOK_Geo_AppData.MechState = WOK_GEO_STOWED;
    WOK_Geo_AppData.FaultMask = WOK_FAULT_NONE;
    WOK_Geo_AppData.CmdCounter = 0;
    WOK_Geo_AppData.ErrCounter = 0;

    WOK_Geo_AppData.MotorCurrentLimit_A = WOK_GEO_DEFAULT_CURRENT_LIMIT_A;
    WOK_Geo_AppData.DeployTimeoutMs     = WOK_GEO_DEFAULT_DEPLOY_TIMEOUT_MS;
    WOK_Geo_AppData.StowTimeoutMs       = WOK_GEO_DEFAULT_STOW_TIMEOUT_MS;

    for (int p = 0; p < WOK_HAL_GEO_MAX_PODS; p++)
    {
        WOK_Geo_AppData.Pod[p].State       = WOK_GEO_POD_CARRIED;
        WOK_Geo_AppData.Pod[p].Reseats     = 0;
        WOK_Geo_AppData.Pod[p].SeatForce_N = 0.0f;
    }
    WOK_Geo_AppData.ActivePod = 0xFF;

    WOK_HAL_Geo_Init();

    CFE_EVS_SendEvent(WOK_GEO_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_GEO: App initialised (v2.0.0)");
    return CFE_SUCCESS;
}

/* Read HAL state on every HK cycle */
static void WOK_Geo_UpdateFromHAL(void)
{
    uint8_t halState = 0;
    uint8_t depEnd = 0, stowEnd = 0;

    WOK_HAL_Geo_GetState(&halState);
    WOK_HAL_Geo_GetEndstops(&depEnd, &stowEnd);
    WOK_HAL_Geo_GetMotorCurrent(&WOK_Geo_AppData.MotorCurrent_A);

    /* FAULT is sticky in app state — don't let HAL clobber it. The latch
     * clears when ground re-issues DEPLOY/STOW. */
    if (WOK_Geo_AppData.MechState != WOK_GEO_FAULT)
        WOK_Geo_AppData.MechState = (WOK_GeoMechState_t)halState;
    WOK_Geo_AppData.EndstopDeployed = depEnd ? 1 : 0;
    WOK_Geo_AppData.EndstopStowed   = stowEnd ? 1 : 0;
}

/* Millisecond timestamp from cFE TIME. uint64 multiply prevents overflow
 * before final uint32 truncation; delta arithmetic is mod-2^32 correct. */
static uint32_t WOK_Geo_GetTimeMs(void)
{
    CFE_TIME_SysTime_t t = CFE_TIME_GetTime();
    return (uint32_t)((uint64_t)t.Seconds * 1000ull + t.Subseconds / 4294967u);
}

/* Elapsed ms since *start, robust to cFE TIME stepping backwards (the
 * early-boot FLYWHEEL correction): a negative delta would wrap mod 2^32
 * and trip an instant false timeout, so re-baseline *start instead. */
static uint32_t WOK_Geo_ElapsedMs(uint32_t *start)
{
    uint32_t delta = WOK_Geo_GetTimeMs() - *start;
    if (delta > 0x80000000u)
    {
        *start += delta;  /* i.e. *start = now */
        return 0;
    }
    return delta;
}

/* Fault detection — runs every HK cycle. Implements arch-doc 4.3:
 * jam = current spike with no endstop; timeout = motion didn't complete. */
static void WOK_Geo_RunFaultDetection(void)
{
    if (WOK_Geo_AppData.MechState != WOK_GEO_DEPLOYING &&
        WOK_Geo_AppData.MechState != WOK_GEO_STOWING)
        return;

    bool deploying     = (WOK_Geo_AppData.MechState == WOK_GEO_DEPLOYING);
    uint32_t elapsed   = WOK_Geo_ElapsedMs(&WOK_Geo_AppData.CmdStartMs);
    uint16_t timeout   = deploying ? WOK_Geo_AppData.DeployTimeoutMs
                                   : WOK_Geo_AppData.StowTimeoutMs;
    uint8_t  endstop   = deploying ? WOK_Geo_AppData.EndstopDeployed
                                   : WOK_Geo_AppData.EndstopStowed;

    if (elapsed > timeout && !endstop)
    {
        WOK_Geo_AppData.FaultMask |= WOK_GEO_FAULT_TIMEOUT;
        WOK_Geo_AppData.TimeoutCounter++;
        WOK_Geo_AppData.MechState = WOK_GEO_FAULT;
        WOK_HAL_Geo_EmergencyStop();
        CFE_EVS_SendEvent(WOK_GEO_FAULT_TIMEOUT_EID, CFE_EVS_EventType_ERROR,
                          "WOK_GEO: FAULT — %s timeout (%lu ms > %u limit)",
                          deploying ? "DEPLOY" : "STOW",
                          (unsigned long)elapsed, timeout);
        return;
    }

    if (WOK_Geo_AppData.MotorCurrent_A > WOK_Geo_AppData.MotorCurrentLimit_A &&
        !endstop)
    {
        WOK_Geo_AppData.FaultMask |= WOK_GEO_FAULT_JAM;
        WOK_Geo_AppData.MechState = WOK_GEO_FAULT;
        WOK_HAL_Geo_EmergencyStop();
        CFE_EVS_SendEvent(WOK_GEO_FAULT_JAM_EID, CFE_EVS_EventType_ERROR,
                          "WOK_GEO: FAULT — jam (%.2fA > %.2fA, no endstop)",
                          (double)WOK_Geo_AppData.MotorCurrent_A,
                          (double)WOK_Geo_AppData.MotorCurrentLimit_A);
    }
}

/* ================================================================== */
/*  Per-pod placement machine (CP-3) — runs every HK cycle.           */
/*                                                                    */
/*  Owns the seat -> force-proxy -> reseat loop for the pod in motion */
/*  (one at a time; single arm). Terminal outcomes: PLACED_OK, or     */
/*  PLACED_MARGINAL once the WOK_GEO_MAX_RESEAT budget is spent —     */
/*  marginal is a flag for the sequencer/test-shot, not a fault. A    */
/*  motion that never completes IS a fault (arm timeout).             */
/* ================================================================== */
static void WOK_Geo_RunPodMachine(void)
{
    uint8 p = WOK_Geo_AppData.ActivePod;
    if (p >= WOK_HAL_GEO_MAX_PODS) return;

    uint8_t  hal = 0;
    uint32_t elapsed = WOK_Geo_ElapsedMs(&WOK_Geo_AppData.Pod[p].CmdStartMs);
    WOK_HAL_Geo_GetPodState(p, &hal);

    if (WOK_Geo_AppData.Pod[p].State == WOK_GEO_POD_PLACING)
    {
        if (hal == WOK_HAL_GEO_POD_PLACED)
        {
            float   force = 0.0f;
            uint8_t att   = 0;
            WOK_HAL_Geo_GetPodSeating(p, &force, &att);
            WOK_Geo_AppData.Pod[p].SeatForce_N = force;

            if (force >= WOK_GEO_SEAT_FORCE_MIN_N)
            {
                WOK_Geo_AppData.Pod[p].State = WOK_GEO_POD_PLACED_OK;
                WOK_Geo_AppData.ActivePod    = 0xFF;
                CFE_EVS_SendEvent(WOK_GEO_POD_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_GEO: Pod %u seated OK at (%.2f,%.2f) — %.1fN, %u reseat(s)",
                                  p, (double)WOK_Geo_AppData.Pod[p].X_m,
                                  (double)WOK_Geo_AppData.Pod[p].Y_m,
                                  (double)force, WOK_Geo_AppData.Pod[p].Reseats);
            }
            else if (!WOK_Geo_AppData.Pod[p].SingleReseat &&
                     WOK_Geo_AppData.Pod[p].Reseats < WOK_GEO_MAX_RESEAT)
            {
                WOK_Geo_AppData.Pod[p].Reseats++;
                WOK_HAL_Geo_ReseatPod(p);
                WOK_Geo_AppData.Pod[p].CmdStartMs = WOK_Geo_GetTimeMs();
                CFE_EVS_SendEvent(WOK_GEO_POD_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_GEO: Pod %u seat force %.1fN < %.1fN — reseat %u/%u",
                                  p, (double)force, (double)WOK_GEO_SEAT_FORCE_MIN_N,
                                  WOK_Geo_AppData.Pod[p].Reseats, WOK_GEO_MAX_RESEAT);
            }
            else
            {
                WOK_Geo_AppData.Pod[p].State = WOK_GEO_POD_PLACED_MARGINAL;
                WOK_Geo_AppData.ActivePod    = 0xFF;
                CFE_EVS_SendEvent(WOK_GEO_POD_EID, CFE_EVS_EventType_ERROR,
                                  "WOK_GEO: Pod %u MARGINAL after %u reseats (%.1fN)",
                                  p, WOK_Geo_AppData.Pod[p].Reseats, (double)force);
            }
        }
        else if (elapsed > WOK_GEO_POD_TIMEOUT_MS)
        {
            WOK_Geo_AppData.FaultMask |= WOK_GEO_FAULT_TIMEOUT;
            WOK_Geo_AppData.TimeoutCounter++;
            WOK_Geo_AppData.ActivePod  = 0xFF;
            WOK_HAL_Geo_EmergencyStop();
            CFE_EVS_SendEvent(WOK_GEO_FAULT_TIMEOUT_EID, CFE_EVS_EventType_ERROR,
                              "WOK_GEO: FAULT — pod %u placement timeout (%lu ms)",
                              p, (unsigned long)elapsed);
        }
    }
    else if (WOK_Geo_AppData.Pod[p].State == WOK_GEO_POD_RETRIEVING)
    {
        if (hal == WOK_HAL_GEO_POD_STOWED)
        {
            WOK_Geo_AppData.Pod[p].State = WOK_GEO_POD_CARRIED;
            WOK_Geo_AppData.ActivePod    = 0xFF;
            CFE_EVS_SendEvent(WOK_GEO_POD_EID, CFE_EVS_EventType_INFORMATION,
                              "WOK_GEO: Pod %u recaptured and stowed", p);
        }
        else if (elapsed > WOK_GEO_POD_TIMEOUT_MS)
        {
            WOK_Geo_AppData.FaultMask |= WOK_GEO_FAULT_TIMEOUT;
            WOK_Geo_AppData.TimeoutCounter++;
            WOK_Geo_AppData.ActivePod  = 0xFF;
            WOK_HAL_Geo_EmergencyStop();
            CFE_EVS_SendEvent(WOK_GEO_FAULT_TIMEOUT_EID, CFE_EVS_EventType_ERROR,
                              "WOK_GEO: FAULT — pod %u retrieval timeout (%lu ms)",
                              p, (unsigned long)elapsed);
        }
    }
}

void WOK_Geo_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
    switch (CFE_SB_MsgIdToValue(MsgId)) {
        case WOK_GEO_CMD_MID:
            WOK_Geo_ProcessGroundCommand(BufPtr); break;
        case WOK_GEO_SEND_HK_MID:
            WOK_Geo_UpdateFromHAL();
            WOK_Geo_RunFaultDetection();
            WOK_Geo_RunPodMachine();
            WOK_Geo_ReportHousekeeping();
            WOK_Geo_PublishStatus();
            break;
        default: WOK_Geo_AppData.ErrCounter++; break;
    }
}

void WOK_Geo_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t CmdCode = 0;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &CmdCode);
    switch (CmdCode) {
        case WOK_GEO_NOOP_CC:
            if (WOK_Geo_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Geo_NoArgsCmd_t))) {
                WOK_Geo_AppData.LastSeqId = ((WOK_Geo_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Geo_AppData.CmdCounter++;
                CFE_EVS_SendEvent(WOK_GEO_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_GEO: NOOP (v2.0.0)");
            } break;
        case WOK_GEO_RESET_CTRS_CC:
            if (WOK_Geo_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Geo_NoArgsCmd_t))) {
                WOK_Geo_AppData.LastSeqId = ((WOK_Geo_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Geo_AppData.CmdCounter = 0; WOK_Geo_AppData.ErrCounter = 0;
            } break;
        case WOK_GEO_DEPLOY_CC:
            if (WOK_Geo_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Geo_NoArgsCmd_t))) {
                WOK_Geo_AppData.LastSeqId = ((WOK_Geo_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Geo_AppData.CmdCounter++;
                WOK_Geo_AppData.MechState   = WOK_GEO_DEPLOYING;
                WOK_Geo_AppData.CmdStartMs  = WOK_Geo_GetTimeMs();
                WOK_Geo_AppData.FaultMask  &= ~(WOK_GEO_FAULT_JAM | WOK_GEO_FAULT_TIMEOUT);
                WOK_HAL_Geo_Deploy();
                CFE_EVS_SendEvent(WOK_GEO_DEPLOY_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_GEO: Deploy commanded");
            } break;
        case WOK_GEO_STOW_CC:
            if (WOK_Geo_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Geo_NoArgsCmd_t))) {
                WOK_Geo_AppData.LastSeqId = ((WOK_Geo_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Geo_AppData.CmdCounter++;
                WOK_Geo_AppData.MechState   = WOK_GEO_STOWING;
                WOK_Geo_AppData.CmdStartMs  = WOK_Geo_GetTimeMs();
                WOK_Geo_AppData.FaultMask  &= ~(WOK_GEO_FAULT_JAM | WOK_GEO_FAULT_TIMEOUT);
                WOK_HAL_Geo_Stow();
                CFE_EVS_SendEvent(WOK_GEO_STOW_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_GEO: Stow commanded");
            } break;
        case WOK_GEO_ABORT_CC:
            if (WOK_Geo_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Geo_NoArgsCmd_t))) {
                WOK_Geo_AppData.LastSeqId = ((WOK_Geo_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Geo_AppData.CmdCounter++;
                WOK_Geo_AppData.MechState = WOK_GEO_FAULT;
                WOK_HAL_Geo_EmergencyStop();
                /* Resolve in-motion pods to where the e-stop left them
                 * (HAL: interrupted place retracts with the pod, an
                 * interrupted retrieval leaves it on the ground) */
                for (uint8 p = 0; p < WOK_HAL_GEO_MAX_PODS; p++)
                {
                    uint8_t hal = 0;
                    if (WOK_Geo_AppData.Pod[p].State != WOK_GEO_POD_PLACING &&
                        WOK_Geo_AppData.Pod[p].State != WOK_GEO_POD_RETRIEVING)
                        continue;
                    WOK_HAL_Geo_GetPodState(p, &hal);
                    WOK_Geo_AppData.Pod[p].State =
                        (hal == WOK_HAL_GEO_POD_STOWED) ? WOK_GEO_POD_CARRIED
                                                        : WOK_GEO_POD_PLACED_MARGINAL;
                }
                WOK_Geo_AppData.ActivePod = 0xFF;
                CFE_EVS_SendEvent(WOK_GEO_ABORT_EID, CFE_EVS_EventType_CRITICAL,
                                  "WOK_GEO: Mechanism ABORT");
            } break;
        case WOK_GEO_PLACE_POD_CC:
            if (WOK_Geo_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Geo_PlacePodCmd_t))) {
                WOK_Geo_PlacePodCmd_t *Cmd = (WOK_Geo_PlacePodCmd_t *)BufPtr;
                WOK_Geo_AppData.LastSeqId = Cmd->SeqId;
                if (Cmd->Pod >= WOK_HAL_GEO_MAX_PODS ||
                    WOK_Geo_AppData.ActivePod != 0xFF ||
                    WOK_Geo_AppData.Pod[Cmd->Pod].State != WOK_GEO_POD_CARRIED ||
                    WOK_HAL_Geo_PlacePod(Cmd->Pod, Cmd->X_m, Cmd->Y_m) != WOK_HAL_OK)
                {
                    WOK_Geo_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_GEO_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_GEO: PLACE_POD %u rejected (state %u, active %u)",
                                      Cmd->Pod,
                                      Cmd->Pod < WOK_HAL_GEO_MAX_PODS
                                          ? WOK_Geo_AppData.Pod[Cmd->Pod].State : 0xFF,
                                      WOK_Geo_AppData.ActivePod);
                }
                else
                {
                    WOK_Geo_AppData.CmdCounter++;
                    WOK_Geo_AppData.Pod[Cmd->Pod].State        = WOK_GEO_POD_PLACING;
                    WOK_Geo_AppData.Pod[Cmd->Pod].Reseats      = 0;
                    WOK_Geo_AppData.Pod[Cmd->Pod].SingleReseat = 0;
                    WOK_Geo_AppData.Pod[Cmd->Pod].SeatForce_N  = 0.0f;
                    WOK_Geo_AppData.Pod[Cmd->Pod].X_m          = Cmd->X_m;
                    WOK_Geo_AppData.Pod[Cmd->Pod].Y_m          = Cmd->Y_m;
                    WOK_Geo_AppData.Pod[Cmd->Pod].CmdStartMs   = WOK_Geo_GetTimeMs();
                    WOK_Geo_AppData.ActivePod                  = Cmd->Pod;
                    CFE_EVS_SendEvent(WOK_GEO_POD_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_GEO: Placing pod %u at (%.2f, %.2f)",
                                      Cmd->Pod, (double)Cmd->X_m, (double)Cmd->Y_m);
                }
            } break;
        case WOK_GEO_RESEAT_POD_CC:
            if (WOK_Geo_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Geo_RetrievePodCmd_t))) {
                WOK_Geo_RetrievePodCmd_t *Cmd = (WOK_Geo_RetrievePodCmd_t *)BufPtr;
                WOK_Geo_AppData.LastSeqId = Cmd->SeqId;
                if (Cmd->Pod >= WOK_HAL_GEO_MAX_PODS ||
                    WOK_Geo_AppData.ActivePod != 0xFF ||
                    (WOK_Geo_AppData.Pod[Cmd->Pod].State != WOK_GEO_POD_PLACED_OK &&
                     WOK_Geo_AppData.Pod[Cmd->Pod].State != WOK_GEO_POD_PLACED_MARGINAL) ||
                    WOK_HAL_Geo_ReseatPod(Cmd->Pod) != WOK_HAL_OK)
                {
                    WOK_Geo_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_GEO_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_GEO: RESEAT_POD %u rejected", Cmd->Pod);
                }
                else
                {
                    WOK_Geo_AppData.CmdCounter++;
                    WOK_Geo_AppData.Pod[Cmd->Pod].State        = WOK_GEO_POD_PLACING;
                    WOK_Geo_AppData.Pod[Cmd->Pod].Reseats++;
                    WOK_Geo_AppData.Pod[Cmd->Pod].SingleReseat = 1;
                    WOK_Geo_AppData.Pod[Cmd->Pod].CmdStartMs   = WOK_Geo_GetTimeMs();
                    WOK_Geo_AppData.ActivePod                  = Cmd->Pod;
                    CFE_EVS_SendEvent(WOK_GEO_POD_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_GEO: Commanded reseat of pod %u (attempt %u)",
                                      Cmd->Pod, WOK_Geo_AppData.Pod[Cmd->Pod].Reseats);
                }
            } break;
        case WOK_GEO_RETRIEVE_POD_CC:
            if (WOK_Geo_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Geo_RetrievePodCmd_t))) {
                WOK_Geo_RetrievePodCmd_t *Cmd = (WOK_Geo_RetrievePodCmd_t *)BufPtr;
                WOK_Geo_AppData.LastSeqId = Cmd->SeqId;
                if (Cmd->Pod >= WOK_HAL_GEO_MAX_PODS ||
                    WOK_Geo_AppData.ActivePod != 0xFF ||
                    (WOK_Geo_AppData.Pod[Cmd->Pod].State != WOK_GEO_POD_PLACED_OK &&
                     WOK_Geo_AppData.Pod[Cmd->Pod].State != WOK_GEO_POD_PLACED_MARGINAL) ||
                    WOK_HAL_Geo_RetrievePod(Cmd->Pod) != WOK_HAL_OK)
                {
                    WOK_Geo_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_GEO_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_GEO: RETRIEVE_POD %u rejected", Cmd->Pod);
                }
                else
                {
                    WOK_Geo_AppData.CmdCounter++;
                    WOK_Geo_AppData.Pod[Cmd->Pod].State      = WOK_GEO_POD_RETRIEVING;
                    WOK_Geo_AppData.Pod[Cmd->Pod].CmdStartMs = WOK_Geo_GetTimeMs();
                    WOK_Geo_AppData.ActivePod                = Cmd->Pod;
                    CFE_EVS_SendEvent(WOK_GEO_POD_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_GEO: Retrieving pod %u", Cmd->Pod);
                }
            } break;
        default:
            WOK_Geo_AppData.ErrCounter++;
            break;
    }
}

void WOK_Geo_ReportHousekeeping(void)
{
    WOK_Geo_AppData.HkTlm.CmdCounter = WOK_Geo_AppData.CmdCounter;
    WOK_Geo_AppData.HkTlm.ErrCounter = WOK_Geo_AppData.ErrCounter;
    WOK_Geo_AppData.HkTlm.MechState  = (uint8)WOK_Geo_AppData.MechState;
    WOK_Geo_AppData.HkTlm.FaultMask  = WOK_Geo_AppData.FaultMask;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Geo_AppData.HkTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Geo_AppData.HkTlm.TlmHeader), true);
}

void WOK_Geo_PublishStatus(void)
{
    WOK_Geo_AppData.StatusTlm.MechState       = (uint8)WOK_Geo_AppData.MechState;
    WOK_Geo_AppData.StatusTlm.EndstopDeployed = WOK_Geo_AppData.EndstopDeployed;
    WOK_Geo_AppData.StatusTlm.EndstopStowed   = WOK_Geo_AppData.EndstopStowed;
    WOK_Geo_AppData.StatusTlm.MotorCurrent_A  = WOK_Geo_AppData.MotorCurrent_A;
    WOK_Geo_AppData.StatusTlm.FaultMask       = WOK_Geo_AppData.FaultMask;
    WOK_Geo_AppData.StatusTlm.TimeoutCounter  = WOK_Geo_AppData.TimeoutCounter;
    WOK_Geo_AppData.StatusTlm.SeqId           = WOK_Geo_AppData.LastSeqId;
    /* Per-pod placement (CP-3) */
    {
        uint8 placed = 0, marginal = 0;
        for (uint8 p = 0; p < WOK_HAL_GEO_MAX_PODS; p++)
        {
            uint8 st = WOK_Geo_AppData.Pod[p].State;
            WOK_Geo_AppData.StatusTlm.PodState[p]       = st;
            WOK_Geo_AppData.StatusTlm.PodReseats[p]     = WOK_Geo_AppData.Pod[p].Reseats;
            WOK_Geo_AppData.StatusTlm.PodSeatForce_N[p] = WOK_Geo_AppData.Pod[p].SeatForce_N;
            if (st == WOK_GEO_POD_PLACED_OK || st == WOK_GEO_POD_PLACED_MARGINAL)
                placed |= (uint8)(1u << p);
            if (st == WOK_GEO_POD_PLACED_MARGINAL)
                marginal |= (uint8)(1u << p);
        }
        WOK_Geo_AppData.StatusTlm.PlacedMask   = placed;
        WOK_Geo_AppData.StatusTlm.MarginalMask = marginal;
        WOK_Geo_AppData.StatusTlm.ActivePod    = WOK_Geo_AppData.ActivePod;
    }
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Geo_AppData.StatusTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Geo_AppData.StatusTlm.TlmHeader), true);
}

bool WOK_Geo_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    size_t ActualLength = 0;
    CFE_MSG_GetSize(MsgPtr, &ActualLength);
    if (ActualLength != ExpectedLength) {
        WOK_Geo_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_GEO_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_GEO: Len err: exp %lu got %lu",
                          (unsigned long)ExpectedLength, (unsigned long)ActualLength);
        return false;
    }
    return true;
}
