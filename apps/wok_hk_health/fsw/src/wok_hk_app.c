/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_HK — Health / Housekeeping / FDIR-lite (HAL-integrated)
 *
 * Calls WOK_HAL_Power_* and WOK_HAL_Watchdog_* on every HK cycle.
 * Real sensor values drive fault detection.
 ************************************************************************/
#include "wok_hk_app.h"
#include "wok_seq_msg.h"
#include "wok_hal.h"
#include "wok_hal_power.h"
#include "wok_hal_watchdog.h"
#include "wok_hal_mobility.h"

static void WOK_Hk_UpdateFromHAL(void);
static WOK_Hk_AppData_t WOK_Hk_AppData;

void WOK_Hk_AppMain(void)
{
    int32 Status; CFE_SB_Buffer_t *BufPtr = NULL;
    CFE_ES_PerfLogEntry(WOK_HK_PERF_ID);
    Status = WOK_Hk_AppInit();
    if (Status != CFE_SUCCESS) return;
    while (true) {
        CFE_ES_PerfLogExit(WOK_HK_PERF_ID);
        Status = CFE_SB_ReceiveBuffer(&BufPtr, WOK_Hk_AppData.CmdPipe, CFE_SB_PEND_FOREVER);
        CFE_ES_PerfLogEntry(WOK_HK_PERF_ID);
        if (Status == CFE_SUCCESS) WOK_Hk_ProcessCommandPacket(BufPtr);
        else break;
    }
    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
}

int32 WOK_Hk_AppInit(void)
{
    int32 Status;
    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;
    Status = CFE_SB_CreatePipe(&WOK_Hk_AppData.CmdPipe, WOK_HK_PIPE_DEPTH, WOK_HK_PIPE_NAME);
    if (Status != CFE_SUCCESS) return Status;

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_HK_CMD_MID),      WOK_Hk_AppData.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_HK_SEND_HK_MID),  WOK_Hk_AppData.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_SEQ_MODE_TLM_MID), WOK_Hk_AppData.CmdPipe);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Hk_AppData.HealthTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_HK_TLM_MID), sizeof(WOK_Hk_HealthTlm_t));

    WOK_Hk_AppData.BatteryMin_V  = WOK_HK_DEFAULT_BATT_MIN_V;
    WOK_Hk_AppData.TempMax_C     = WOK_HK_DEFAULT_TEMP_MAX_C;
    WOK_Hk_AppData.TiltMax_deg   = WOK_HK_DEFAULT_TILT_MAX_DEG;
    WOK_Hk_AppData.FaultMask     = WOK_FAULT_NONE;
    WOK_Hk_AppData.WatchdogOK    = 1;
    WOK_Hk_AppData.SafeTrigger   = 0;
    WOK_Hk_AppData.SeqState      = (uint8)WOK_STATE_IDLE;
    WOK_Hk_AppData.CmdCounter    = 0;
    WOK_Hk_AppData.ErrCounter    = 0;

    /* Init HAL subsystems */
    WOK_HAL_Init();
    WOK_HAL_Power_Init();
    WOK_HAL_Watchdog_Init(WOK_HK_WATCHDOG_TIMEOUT_MS);

    CFE_EVS_SendEvent(WOK_HK_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_HK: App initialised (v2.0.0)");
    return CFE_SUCCESS;
}

/* Read HAL sensors on every HK cycle */
static void WOK_Hk_UpdateFromHAL(void)
{
    WOK_HAL_Power_GetBatteryV(&WOK_Hk_AppData.BatteryV);
    WOK_HAL_Power_GetBatterySOC(&WOK_Hk_AppData.BatterySOC_pct);
    WOK_HAL_Power_GetBoardTemp(&WOK_Hk_AppData.TempBoard_C);
    WOK_HAL_Power_GetMotorTemp(&WOK_Hk_AppData.TempMotor_C);

    /* Read tilt from mobility HAL (shared sensor) */
    WOK_HAL_Mobility_GetTilt(&WOK_Hk_AppData.TiltDeg);

    /* CPU load via HAL (sim reads host load average; flight stub returns ERROR) */
    if (WOK_HAL_Power_GetCPULoad(&WOK_Hk_AppData.CPULoad_pct) != WOK_HAL_OK)
        WOK_Hk_AppData.CPULoad_pct = 0.0f;

    /* Pet the watchdog and reflect the HAL return code in HK telemetry.
     * The fault-mask bit is set in WOK_Hk_CheckFaults, which is the only
     * place that owns FaultMask. */
    WOK_Hk_AppData.WatchdogOK =
        (WOK_HAL_Watchdog_Pet() == WOK_HAL_OK) ? 1 : 0;
}

void WOK_Hk_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
    switch (CFE_SB_MsgIdToValue(MsgId)) {
        case WOK_HK_CMD_MID:
            WOK_Hk_ProcessGroundCommand(BufPtr); break;
        case WOK_HK_SEND_HK_MID:
            WOK_Hk_UpdateFromHAL();
            WOK_Hk_CheckFaults();
            WOK_Hk_ReportHealth();
            break;
        case WOK_SEQ_MODE_TLM_MID: {
            WOK_Seq_ModeTlm_t *ModeTlm = (WOK_Seq_ModeTlm_t *)BufPtr;
            WOK_Hk_AppData.SeqState = ModeTlm->State;
            break;
        }
        default: WOK_Hk_AppData.ErrCounter++; break;
    }
}

void WOK_Hk_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t CmdCode = 0;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &CmdCode);
    switch (CmdCode) {
        case WOK_HK_NOOP_CC:
            if (WOK_Hk_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Hk_NoArgsCmd_t))) {
                WOK_Hk_AppData.CmdCounter++;
                CFE_EVS_SendEvent(WOK_HK_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_HK: NOOP (v2.0.0)");
            } break;
        case WOK_HK_RESET_CTRS_CC:
            if (WOK_Hk_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Hk_NoArgsCmd_t))) {
                WOK_Hk_AppData.CmdCounter = 0; WOK_Hk_AppData.ErrCounter = 0;
            } break;
        case WOK_HK_SET_THRESHOLDS_CC:
            if (WOK_Hk_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Hk_SetThresholdsCmd_t))) {
                WOK_Hk_SetThresholdsCmd_t *Cmd = (WOK_Hk_SetThresholdsCmd_t *)BufPtr;
                WOK_Hk_AppData.CmdCounter++;
                WOK_Hk_AppData.BatteryMin_V = Cmd->BatteryMin_V;
                WOK_Hk_AppData.TempMax_C    = Cmd->TempMax_C;
                WOK_Hk_AppData.TiltMax_deg  = Cmd->TiltMax_deg;
                CFE_EVS_SendEvent(WOK_HK_THRESHOLDS_SET_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_HK: Thresholds: batt>%.1fV temp<%.1fC tilt<%.1fdeg",
                                  (double)Cmd->BatteryMin_V, (double)Cmd->TempMax_C,
                                  (double)Cmd->TiltMax_deg);
            } break;
        case WOK_HK_CLEAR_FAULTS_CC:
            if (WOK_Hk_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Hk_NoArgsCmd_t))) {
                WOK_Hk_AppData.CmdCounter++;
                WOK_Hk_AppData.FaultMask   = WOK_FAULT_NONE;
                WOK_Hk_AppData.SafeTrigger = 0;
                CFE_EVS_SendEvent(WOK_HK_FAULTS_CLEARED_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_HK: Faults cleared");
            } break;
        default:
            WOK_Hk_AppData.ErrCounter++;
            break;
    }
}

void WOK_Hk_CheckFaults(void)
{
    uint16 PrevMask = WOK_Hk_AppData.FaultMask;
    WOK_Hk_AppData.FaultMask = WOK_FAULT_NONE;

    if (WOK_Hk_AppData.BatteryV < WOK_Hk_AppData.BatteryMin_V)
    {
        WOK_Hk_AppData.FaultMask |= WOK_FAULT_BATTERY_LOW;
        if (!(PrevMask & WOK_FAULT_BATTERY_LOW))
            CFE_EVS_SendEvent(WOK_HK_BATTERY_LOW_EID, CFE_EVS_EventType_CRITICAL,
                              "WOK_HK: Battery low (%.1fV < %.1fV)",
                              (double)WOK_Hk_AppData.BatteryV,
                              (double)WOK_Hk_AppData.BatteryMin_V);
    }

    if (WOK_Hk_AppData.TempBoard_C > WOK_Hk_AppData.TempMax_C)
    {
        WOK_Hk_AppData.FaultMask |= WOK_FAULT_TEMP_HIGH;
        if (!(PrevMask & WOK_FAULT_TEMP_HIGH))
            CFE_EVS_SendEvent(WOK_HK_TEMP_HIGH_EID, CFE_EVS_EventType_CRITICAL,
                              "WOK_HK: Temp high (%.1fC > %.1fC)",
                              (double)WOK_Hk_AppData.TempBoard_C,
                              (double)WOK_Hk_AppData.TempMax_C);
    }

    if (WOK_Hk_AppData.TiltDeg > WOK_Hk_AppData.TiltMax_deg)
    {
        WOK_Hk_AppData.FaultMask |= WOK_FAULT_TILT_EXCEED;
        if (!(PrevMask & WOK_FAULT_TILT_EXCEED))
            CFE_EVS_SendEvent(WOK_HK_TILT_EXCEED_EID, CFE_EVS_EventType_CRITICAL,
                              "WOK_HK: Tilt (%.1f > %.1f deg)",
                              (double)WOK_Hk_AppData.TiltDeg,
                              (double)WOK_Hk_AppData.TiltMax_deg);
    }

    if (!WOK_Hk_AppData.WatchdogOK)
    {
        WOK_Hk_AppData.FaultMask |= WOK_FAULT_WATCHDOG;
        if (!(PrevMask & WOK_FAULT_WATCHDOG))
            CFE_EVS_SendEvent(WOK_HK_WATCHDOG_FAIL_EID, CFE_EVS_EventType_CRITICAL,
                              "WOK_HK: Watchdog HAL Pet failed");
    }

    if (WOK_Hk_AppData.FaultMask != WOK_FAULT_NONE && !WOK_Hk_AppData.SafeTrigger)
    {
        WOK_Hk_AppData.SafeTrigger = 1;
        CFE_EVS_SendEvent(WOK_HK_SAFE_TRIGGER_EID, CFE_EVS_EventType_CRITICAL,
                          "WOK_HK: SAFE trigger (faults 0x%04X)",
                          WOK_Hk_AppData.FaultMask);
    }
    else if (WOK_Hk_AppData.FaultMask == WOK_FAULT_NONE)
    {
        WOK_Hk_AppData.SafeTrigger = 0;
    }
}

void WOK_Hk_ReportHealth(void)
{
    WOK_Hk_AppData.HealthTlm.CmdCounter    = WOK_Hk_AppData.CmdCounter;
    WOK_Hk_AppData.HealthTlm.ErrCounter    = WOK_Hk_AppData.ErrCounter;
    WOK_Hk_AppData.HealthTlm.BatteryV      = WOK_Hk_AppData.BatteryV;
    WOK_Hk_AppData.HealthTlm.BatterySOC_pct = WOK_Hk_AppData.BatterySOC_pct;
    WOK_Hk_AppData.HealthTlm.TempBoard_C   = WOK_Hk_AppData.TempBoard_C;
    WOK_Hk_AppData.HealthTlm.TempMotor_C   = WOK_Hk_AppData.TempMotor_C;
    WOK_Hk_AppData.HealthTlm.TiltDeg       = WOK_Hk_AppData.TiltDeg;
    WOK_Hk_AppData.HealthTlm.CPULoad_pct   = WOK_Hk_AppData.CPULoad_pct;
    WOK_Hk_AppData.HealthTlm.FaultMask     = WOK_Hk_AppData.FaultMask;
    WOK_Hk_AppData.HealthTlm.WatchdogOK    = WOK_Hk_AppData.WatchdogOK;
    WOK_Hk_AppData.HealthTlm.SafeTrigger   = WOK_Hk_AppData.SafeTrigger;
    WOK_Hk_AppData.HealthTlm.SeqState      = WOK_Hk_AppData.SeqState;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Hk_AppData.HealthTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Hk_AppData.HealthTlm.TlmHeader), true);
}

bool WOK_Hk_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    size_t ActualLength = 0;
    CFE_MSG_GetSize(MsgPtr, &ActualLength);
    if (ActualLength != ExpectedLength) {
        WOK_Hk_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_HK_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_HK: Len err: exp %lu got %lu",
                          (unsigned long)ExpectedLength, (unsigned long)ActualLength);
        return false;
    }
    return true;
}
