/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK_DRILL — Drill / Ice Prospector Application
** File: wok_drill/fsw/src/wok_drill_app.c
**
** cFS app for managing the drill arm, regolith sampling,
** thermal vaporization, and volatile analysis.
**
** Follows the same pattern as wok_geo / wok_nav:
**   - Subscribe to CMD + SEND_HK on SB
**   - Dispatch commands by command code
**   - Publish HK + STATUS telemetry on SEND_HK wakeup
**   - Run fault detection on every HK cycle
*/

#include "wok_drill_app.h"
#include "wok_hal_storage.h"
#include <string.h>
#include <stdio.h>

/* ── Module globals ──────────────────────────────────────────────── */
static WOK_Drill_AppData_t g_drill;

/* Minimum spacing between drill-log rows (decouples the log rate from
 * the platform-dependent SEND_HK tick rate) */
#define WOK_DRILL_LOG_ROW_MS  500

/* ── Forward declarations ────────────────────────────────────────── */
static CFE_Status_t WOK_Drill_Init(void);
static void         WOK_Drill_ProcessCmd(const CFE_SB_Buffer_t *BufPtr);
static void         WOK_Drill_ProcessHkRequest(void);
static void         WOK_Drill_RunFaultDetection(void);
static void         WOK_Drill_UpdateStatus(void);
static void         WOK_Drill_AppendLogRow(void);
static void         WOK_Drill_WriteLogFile(void);
static uint32_t     WOK_Drill_GetTimeMs(void);

/* ════════════════════════════════════════════════════════════════════
** Entry Point
** ════════════════════════════════════════════════════════════════════ */

void WOK_Drill_AppMain(void)
{
    CFE_Status_t      status;
    CFE_SB_Buffer_t  *BufPtr;
    CFE_SB_MsgId_t    MsgId;

    /* Register with ES */

    /* Initialize */
    status = WOK_Drill_Init();
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(0, CFE_EVS_EventType_ERROR,
            "WOK_DRILL: Init failed (0x%08lX)", (unsigned long)status);
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }

    /* Main loop */
    while (CFE_ES_RunLoop(NULL))
    {
        status = CFE_SB_ReceiveBuffer(&BufPtr, g_drill.CmdPipe, CFE_SB_PEND_FOREVER);

        if (status == CFE_SUCCESS)
        {
            CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

            switch (CFE_SB_MsgIdToValue(MsgId))
            {
                case WOK_DRILL_CMD_MID:
                    WOK_Drill_ProcessCmd(BufPtr);
                    break;

                case WOK_DRILL_SEND_HK_MID:
                    WOK_Drill_ProcessHkRequest();
                    break;

                default:
                    CFE_EVS_SendEvent(0, CFE_EVS_EventType_ERROR,
                        "WOK_DRILL: Unknown MsgId 0x%04X",
                        (unsigned int)CFE_SB_MsgIdToValue(MsgId));
                    g_drill.HkTlm.ErrCounter++;
                    break;
            }
        }
    }

    CFE_ES_ExitApp(CFE_ES_RunStatus_APP_EXIT);
}

/* ════════════════════════════════════════════════════════════════════
** Initialization
** ════════════════════════════════════════════════════════════════════ */

static CFE_Status_t WOK_Drill_Init(void)
{
    CFE_Status_t status;

    memset(&g_drill, 0, sizeof(g_drill));

    /* Set defaults from configuration */
    g_drill.MaxDepthM     = WOK_DRILL_DEFAULT_MAX_DEPTH_M;
    g_drill.MaxRPM        = WOK_DRILL_DEFAULT_MAX_RPM;
    g_drill.TorqueLimitNm = WOK_DRILL_DEFAULT_TORQUE_LIMIT_NM;
    g_drill.CurrentLimitA = WOK_DRILL_DEFAULT_CURRENT_LIMIT_A;
    g_drill.ArmTimeoutMs  = WOK_DRILL_DEFAULT_ARM_TIMEOUT_MS;
    g_drill.MaxCellTempC  = WOK_DRILL_DEFAULT_MAX_CELL_TEMP_C;
    g_drill.AppState      = WOK_DRILL_STATE_IDLE;

    /* Register events */
    status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS) return status;

    /* Create command pipe */
    status = CFE_SB_CreatePipe(&g_drill.CmdPipe,
                               WOK_DRILL_PIPE_DEPTH,
                               WOK_DRILL_PIPE_NAME);
    if (status != CFE_SUCCESS) return status;

    /* Subscribe to command MID */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_DRILL_CMD_MID),
                              g_drill.CmdPipe);
    if (status != CFE_SUCCESS) return status;

    /* Subscribe to SEND_HK MID */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_DRILL_SEND_HK_MID),
                              g_drill.CmdPipe);
    if (status != CFE_SUCCESS) return status;

    /* Initialize HK telemetry header */
    CFE_MSG_Init(&g_drill.HkTlm.TelemetryHeader.Msg,
                 CFE_SB_ValueToMsgId(WOK_DRILL_HK_TLM_MID),
                 sizeof(WOK_Drill_HkTlm_t));

    /* Initialize Status telemetry header */
    CFE_MSG_Init(&g_drill.StatusTlm.TelemetryHeader.Msg,
                 CFE_SB_ValueToMsgId(WOK_DRILL_STATUS_TLM_MID),
                 sizeof(WOK_Drill_StatusTlm_t));

    /* Initialize HAL drill subsystem */
    if (WOK_HAL_Drill_Init() != WOK_HAL_OK)
    {
        CFE_EVS_SendEvent(0, CFE_EVS_EventType_ERROR,
            "WOK_DRILL: HAL Drill Init failed");
        /* Non-fatal: continue, HAL calls will return errors */
    }

    /* Storage for drill log files (each app owns its HAL lib copy) */
    WOK_HAL_Storage_Init();

    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION,
        "WOK_DRILL: App initialised (v1.1.0) — stiffness log + evolved-gas");

    return CFE_SUCCESS;
}

/* ════════════════════════════════════════════════════════════════════
** Drill cycle log — penetration + evolved-gas rows
** ════════════════════════════════════════════════════════════════════ */

static void WOK_Drill_AppendLogRow(void)
{
    uint32_t now = WOK_Drill_GetTimeMs();

    if (g_drill.LogRowCount >= WOK_DRILL_LOG_MAX_ROWS) return;
    if (g_drill.LogRowCount > 0 &&
        (now - g_drill.LogLastRowMs) < WOK_DRILL_LOG_ROW_MS) return;

    WOK_Drill_LogRow_t *row = &g_drill.LogFile.Rows[g_drill.LogRowCount];
    memset(row, 0, sizeof(*row));
    row->T_ms            = now - g_drill.LogStartMs;
    row->Phase           = g_drill.AppState;
    row->DepthM          = g_drill.StatusTlm.DrillDepthM;
    row->WOB_N           = g_drill.StatusTlm.WOB_N;
    row->TorqueNm        = g_drill.StatusTlm.DrillTorqueNm;
    row->CurrentAmps     = g_drill.StatusTlm.DrillCurrentAmps;
    row->CellTempC       = g_drill.StatusTlm.CellTempC;
    row->VaporPressurePa = g_drill.StatusTlm.VaporPressurePa;
    row->VolatileConcPpm = g_drill.StatusTlm.VolatileConcPpm;

    g_drill.LogRowCount++;
    g_drill.LogLastRowMs = now;
}

static void WOK_Drill_WriteLogFile(void)
{
    char name[32];

    if (g_drill.LogRowCount == 0 || g_drill.LogWritten) return;

    g_drill.DrillIndex++;

    WOK_Drill_LogFileHeader_t *hdr = &g_drill.LogFile.Hdr;
    memset(hdr, 0, sizeof(*hdr));
    hdr->Magic        = WOK_DRILL_FILE_MAGIC;
    hdr->Version      = WOK_DRILL_FILE_VERSION;
    hdr->RowSize      = sizeof(WOK_Drill_LogRow_t);
    hdr->RowCount     = g_drill.LogRowCount;
    hdr->SeqId        = g_drill.LastSeqId;
    hdr->SiteX_m      = g_drill.SiteX_m;
    hdr->SiteY_m      = g_drill.SiteY_m;
    hdr->TargetDepthM = g_drill.StatusTlm.DrillDepthM;
    hdr->RPM          = g_drill.StatusTlm.DrillRPM;
    hdr->HeatSec      = WOK_DRILL_DEFAULT_HEAT_SEC;
    hdr->CoolSec      = WOK_DRILL_DEFAULT_COOL_SEC;
    hdr->DrillIndex   = g_drill.DrillIndex;

    snprintf(name, sizeof(name), "drill_%04lu.wok",
             (unsigned long)g_drill.DrillIndex);

    uint32_t size = (uint32_t)(sizeof(*hdr)
                  + (size_t)g_drill.LogRowCount * sizeof(WOK_Drill_LogRow_t));

    if (WOK_HAL_Storage_WriteFile(name, &g_drill.LogFile, size) == WOK_HAL_OK)
    {
        g_drill.LogWritten = 1;
        CFE_EVS_SendEvent(14, CFE_EVS_EventType_INFORMATION,
            "WOK_DRILL: %s — %u rows, site (%.1f,%.1f), final %.1f ppm",
            name, g_drill.LogRowCount,
            (double)g_drill.SiteX_m, (double)g_drill.SiteY_m,
            (double)g_drill.StatusTlm.VolatileConcPpm);
    }
    else
    {
        g_drill.HkTlm.ErrCounter++;
        CFE_EVS_SendEvent(14, CFE_EVS_EventType_ERROR,
            "WOK_DRILL: Log file write FAILED (%s)", name);
    }
}

/* ════════════════════════════════════════════════════════════════════
** Command Dispatch
** ════════════════════════════════════════════════════════════════════ */

static void WOK_Drill_ProcessCmd(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t CmdCode;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &CmdCode);

    switch (CmdCode)
    {
        /* ── CC 0: NOOP ──────────────────────────────────────────── */
        case WOK_DRILL_NOOP_CC:
            g_drill.LastSeqId = ((const WOK_Drill_NoopCmd_t *)BufPtr)->SeqId;
            g_drill.HkTlm.CmdCounter++;
            CFE_EVS_SendEvent(2, CFE_EVS_EventType_INFORMATION,
                "WOK_DRILL: NOOP received");
            break;

        /* ── CC 1: RESET COUNTERS ────────────────────────────────── */
        case WOK_DRILL_RESET_CTRS_CC:
            g_drill.LastSeqId = ((const WOK_Drill_NoopCmd_t *)BufPtr)->SeqId;
            g_drill.HkTlm.CmdCounter = 0;
            g_drill.HkTlm.ErrCounter = 0;
            CFE_EVS_SendEvent(3, CFE_EVS_EventType_INFORMATION,
                "WOK_DRILL: Counters reset");
            break;

        /* ── CC 2: DEPLOY ARM ────────────────────────────────────── */
        case WOK_DRILL_DEPLOY_ARM_CC:
        {
            const WOK_Drill_DeployArmCmd_t *cmd =
                (const WOK_Drill_DeployArmCmd_t *)BufPtr;
            g_drill.LastSeqId = cmd->SeqId;

            if (g_drill.AppState != WOK_DRILL_STATE_IDLE &&
                g_drill.AppState != WOK_DRILL_STATE_FAULT)
            {
                g_drill.HkTlm.ErrCounter++;
                CFE_EVS_SendEvent(0, CFE_EVS_EventType_ERROR,
                    "WOK_DRILL: DEPLOY_ARM rejected — state %d", g_drill.AppState);
                break;
            }
            float angle = cmd->TargetAngleDeg;
            if (angle <= 0.0f) angle = WOK_DRILL_DEFAULT_ARM_ANGLE_DEG;

            if (WOK_HAL_Drill_DeployArm(angle) == WOK_HAL_OK)
            {
                g_drill.PrevState     = g_drill.AppState;
                g_drill.AppState      = WOK_DRILL_STATE_DEPLOYING;
                g_drill.ArmCmdStartMs = WOK_Drill_GetTimeMs();
                g_drill.FaultBitmask &= ~WOK_DRILL_FAULT_ARM_JAM;
                g_drill.FaultBitmask &= ~WOK_DRILL_FAULT_ARM_TIMEOUT;
                g_drill.HkTlm.CmdCounter++;
                CFE_EVS_SendEvent(4, CFE_EVS_EventType_INFORMATION,
                    "WOK_DRILL: Deploying arm to %.1f deg", (double)angle);
            }
            else
            {
                g_drill.HkTlm.ErrCounter++;
                CFE_EVS_SendEvent(0, CFE_EVS_EventType_ERROR,
                    "WOK_DRILL: HAL DeployArm failed");
            }
            break;
        }

        /* ── CC 3: STOW ARM ──────────────────────────────────────── */
        case WOK_DRILL_STOW_ARM_CC:
        {
            g_drill.LastSeqId = ((const WOK_Drill_NoopCmd_t *)BufPtr)->SeqId;
            if (WOK_HAL_Drill_StowArm() == WOK_HAL_OK)
            {
                g_drill.PrevState     = g_drill.AppState;
                g_drill.AppState      = WOK_DRILL_STATE_STOWING;
                g_drill.ArmCmdStartMs = WOK_Drill_GetTimeMs();
                g_drill.HkTlm.CmdCounter++;
                CFE_EVS_SendEvent(5, CFE_EVS_EventType_INFORMATION,
                    "WOK_DRILL: Stowing arm");
            }
            else
            {
                g_drill.HkTlm.ErrCounter++;
            }
            break;
        }

        /* ── CC 4: START DRILL ───────────────────────────────────── */
        case WOK_DRILL_START_DRILL_CC:
        {
            const WOK_Drill_StartDrillCmd_t *cmd =
                (const WOK_Drill_StartDrillCmd_t *)BufPtr;
            g_drill.LastSeqId = cmd->SeqId;

            if (g_drill.AppState != WOK_DRILL_STATE_ARM_READY)
            {
                g_drill.HkTlm.ErrCounter++;
                CFE_EVS_SendEvent(0, CFE_EVS_EventType_ERROR,
                    "WOK_DRILL: START_DRILL rejected — arm not ready (state %d)",
                    g_drill.AppState);
                break;
            }
            float depth = cmd->DepthM;
            float rpm   = cmd->RPM;

            /* Clamp to limits */
            if (depth > g_drill.MaxDepthM) depth = g_drill.MaxDepthM;
            if (rpm   > g_drill.MaxRPM)    rpm   = g_drill.MaxRPM;
            if (depth <= 0.0f) depth = 1.0f;
            if (rpm   <= 0.0f) rpm   = 200.0f;

            /* Site context (rover pose) — stamps this drill cycle's data
             * products so ground can tie them to the seismic survey. */
            g_drill.SiteX_m = cmd->SiteX_m;
            g_drill.SiteY_m = cmd->SiteY_m;
            WOK_HAL_Drill_SetSite(cmd->SiteX_m, cmd->SiteY_m);

            /* Open a fresh drill-cycle log */
            g_drill.LogRowCount  = 0;
            g_drill.LogWritten   = 0;
            g_drill.LogLastRowMs = 0;
            g_drill.LogStartMs   = WOK_Drill_GetTimeMs();
            g_drill.ROP_mps      = 0.0f;
            g_drill.PrevDepthM   = 0.0f;
            g_drill.PrevDepthMs  = 0;

            if (WOK_HAL_Drill_StartDrill(depth, rpm) == WOK_HAL_OK)
            {
                g_drill.PrevState = g_drill.AppState;
                g_drill.AppState  = WOK_DRILL_STATE_DRILLING;
                g_drill.FaultBitmask &= ~WOK_DRILL_FAULT_DRILL_STALL;
                g_drill.FaultBitmask &= ~WOK_DRILL_FAULT_OVER_DEPTH;
                g_drill.FaultBitmask &= ~WOK_DRILL_FAULT_OVERCURRENT;
                g_drill.HkTlm.CmdCounter++;
                CFE_EVS_SendEvent(6, CFE_EVS_EventType_INFORMATION,
                    "WOK_DRILL: Drilling to %.2fm at %.0f RPM",
                    (double)depth, (double)rpm);
            }
            else
            {
                g_drill.HkTlm.ErrCounter++;
            }
            break;
        }

        /* ── CC 5: STOP DRILL ────────────────────────────────────── */
        case WOK_DRILL_STOP_DRILL_CC:
            g_drill.LastSeqId = ((const WOK_Drill_NoopCmd_t *)BufPtr)->SeqId;
            WOK_HAL_Drill_StopDrill();
            if (g_drill.AppState == WOK_DRILL_STATE_DRILLING)
            {
                g_drill.PrevState = g_drill.AppState;
                g_drill.AppState  = WOK_DRILL_STATE_DRILL_DONE;
            }
            g_drill.HkTlm.CmdCounter++;
            CFE_EVS_SendEvent(7, CFE_EVS_EventType_INFORMATION,
                "WOK_DRILL: Drill stopped");
            break;

        /* ── CC 6: START SAMPLE ──────────────────────────────────── */
        case WOK_DRILL_START_SAMPLE_CC:
        {
            const WOK_Drill_StartSampleCmd_t *cmd =
                (const WOK_Drill_StartSampleCmd_t *)BufPtr;
            g_drill.LastSeqId = cmd->SeqId;

            if (g_drill.AppState != WOK_DRILL_STATE_DRILL_DONE)
            {
                g_drill.HkTlm.ErrCounter++;
                CFE_EVS_SendEvent(0, CFE_EVS_EventType_ERROR,
                    "WOK_DRILL: START_SAMPLE rejected — drill not done (state %d)",
                    g_drill.AppState);
                break;
            }
            uint16_t heat = cmd->HeatSec;
            uint16_t cool = cmd->CoolSec;
            if (heat == 0) heat = WOK_DRILL_DEFAULT_HEAT_SEC;
            if (cool == 0) cool = WOK_DRILL_DEFAULT_COOL_SEC;

            if (WOK_HAL_Drill_StartSample(heat, cool) == WOK_HAL_OK)
            {
                g_drill.PrevState = g_drill.AppState;
                g_drill.AppState  = WOK_DRILL_STATE_SAMPLING;
                g_drill.FaultBitmask &= ~WOK_DRILL_FAULT_THERMAL_OT;
                g_drill.HkTlm.CmdCounter++;
                CFE_EVS_SendEvent(8, CFE_EVS_EventType_INFORMATION,
                    "WOK_DRILL: Sampling — heat %us, cool %us", heat, cool);
            }
            else
            {
                g_drill.HkTlm.ErrCounter++;
            }
            break;
        }

        /* ── CC 7: STOP SAMPLE ───────────────────────────────────── */
        case WOK_DRILL_STOP_SAMPLE_CC:
            g_drill.LastSeqId = ((const WOK_Drill_NoopCmd_t *)BufPtr)->SeqId;
            WOK_HAL_Drill_StopSample();
            if (g_drill.AppState == WOK_DRILL_STATE_SAMPLING)
            {
                g_drill.PrevState = g_drill.AppState;
                g_drill.AppState  = WOK_DRILL_STATE_SAMPLE_DONE;
            }
            g_drill.HkTlm.CmdCounter++;
            break;

        /* ── CC 8: EMERGENCY STOP ────────────────────────────────── */
        case WOK_DRILL_ESTOP_CC:
            g_drill.LastSeqId = ((const WOK_Drill_NoopCmd_t *)BufPtr)->SeqId;
            WOK_HAL_Drill_EmergencyStop();
            g_drill.PrevState = g_drill.AppState;
            g_drill.AppState  = WOK_DRILL_STATE_FAULT;
            g_drill.HkTlm.CmdCounter++;
            CFE_EVS_SendEvent(9, CFE_EVS_EventType_CRITICAL,
                "WOK_DRILL: EMERGENCY STOP");
            break;

        default:
            g_drill.HkTlm.ErrCounter++;
            CFE_EVS_SendEvent(0, CFE_EVS_EventType_ERROR,
                "WOK_DRILL: Unknown command code %d", CmdCode);
            break;
    }
}

/* ════════════════════════════════════════════════════════════════════
** HK / Status Telemetry
** ════════════════════════════════════════════════════════════════════ */

static void WOK_Drill_ProcessHkRequest(void)
{
    /* Update status from HAL */
    WOK_Drill_UpdateStatus();

    /* Run fault detection */
    WOK_Drill_RunFaultDetection();

    /* ── Pack HK telemetry ───────────────────────────────────────── */
    g_drill.HkTlm.AppState     = g_drill.AppState;
    g_drill.HkTlm.PrevState    = g_drill.PrevState;
    g_drill.HkTlm.FaultBitmask = g_drill.FaultBitmask;
    g_drill.HkTlm.SeqId        = g_drill.LastSeqId;
    g_drill.HkTlm.ArmAngleDeg  = g_drill.StatusTlm.ArmAngleDeg;
    g_drill.HkTlm.DrillDepthM  = g_drill.StatusTlm.DrillDepthM;

    /* Timestamp and send HK */
    CFE_SB_TimeStampMsg(&g_drill.HkTlm.TelemetryHeader.Msg);
    CFE_SB_TransmitMsg(&g_drill.HkTlm.TelemetryHeader.Msg, true);

    /* ── Send Status telemetry if active ─────────────────────────── */
    if (g_drill.AppState != WOK_DRILL_STATE_IDLE)
    {
        CFE_SB_TimeStampMsg(&g_drill.StatusTlm.TelemetryHeader.Msg);
        CFE_SB_TransmitMsg(&g_drill.StatusTlm.TelemetryHeader.Msg, true);
    }
}

/* ════════════════════════════════════════════════════════════════════
** Update status from HAL — called every HK cycle
** ════════════════════════════════════════════════════════════════════ */

static void WOK_Drill_UpdateStatus(void)
{
    WOK_Drill_StatusTlm_t *s = &g_drill.StatusTlm;

    /* Arm */
    WOK_HAL_Drill_GetArmState(&s->ArmHalState);
    WOK_HAL_Drill_GetArmAngle(&s->ArmAngleDeg);
    WOK_HAL_Drill_GetArmCurrent(&s->ArmCurrentAmps);

    /* Drill motor */
    WOK_HAL_Drill_GetDrillMotorState(&s->DrillHalState);
    WOK_HAL_Drill_GetDepth(&s->DrillDepthM);
    WOK_HAL_Drill_GetRPM(&s->DrillRPM);
    WOK_HAL_Drill_GetTorque(&s->DrillTorqueNm);
    WOK_HAL_Drill_GetMotorCurrent(&s->DrillCurrentAmps);
    WOK_HAL_Drill_GetWOB(&s->WOB_N);

    /* ROP: smoothed depth rate while drilling */
    if (g_drill.AppState == WOK_DRILL_STATE_DRILLING)
    {
        uint32_t now  = WOK_Drill_GetTimeMs();
        uint32_t dt   = now - g_drill.PrevDepthMs;
        if (g_drill.PrevDepthMs != 0 && dt > 0)
        {
            float inst = (s->DrillDepthM - g_drill.PrevDepthM) * 1000.0f / (float)dt;
            if (inst < 0.0f) inst = 0.0f;
            g_drill.ROP_mps = 0.7f * g_drill.ROP_mps + 0.3f * inst;
        }
        g_drill.PrevDepthM  = s->DrillDepthM;
        g_drill.PrevDepthMs = now;
    }
    else if (g_drill.AppState != WOK_DRILL_STATE_DRILL_DONE)
    {
        g_drill.ROP_mps = 0.0f;
    }
    s->ROP_mps = g_drill.ROP_mps;

    /* Thermal / sampling */
    WOK_HAL_Drill_GetSampleState(&s->SampleHalState);
    WOK_HAL_Drill_GetCellTemp(&s->CellTempC);
    WOK_HAL_Drill_GetVaporPressure(&s->VaporPressurePa);
    WOK_HAL_Drill_GetVolatileConc(&s->VolatileConcPpm);

    /* Log a row through the active phases (rate-limited) */
    if (g_drill.AppState == WOK_DRILL_STATE_DRILLING ||
        g_drill.AppState == WOK_DRILL_STATE_SAMPLING)
    {
        WOK_Drill_AppendLogRow();
    }

    /* ── Advance app state based on HAL state ────────────────────── */

    /* Arm deploy complete? */
    if (g_drill.AppState == WOK_DRILL_STATE_DEPLOYING &&
        s->ArmHalState == WOK_HAL_DRILL_ARM_READY)
    {
        g_drill.PrevState = g_drill.AppState;
        g_drill.AppState  = WOK_DRILL_STATE_ARM_READY;
        CFE_EVS_SendEvent(10, CFE_EVS_EventType_INFORMATION,
            "WOK_DRILL: Arm deployed — ready");
    }

    /* Drill complete? */
    if (g_drill.AppState == WOK_DRILL_STATE_DRILLING &&
        s->DrillHalState == WOK_HAL_DRILL_MOTOR_DONE)
    {
        g_drill.PrevState = g_drill.AppState;
        g_drill.AppState  = WOK_DRILL_STATE_DRILL_DONE;
        CFE_EVS_SendEvent(11, CFE_EVS_EventType_INFORMATION,
            "WOK_DRILL: Drill complete — depth %.2fm", (double)s->DrillDepthM);
    }

    /* Sample complete? */
    if (g_drill.AppState == WOK_DRILL_STATE_SAMPLING &&
        s->SampleHalState == WOK_HAL_DRILL_SAMPLE_DONE)
    {
        g_drill.PrevState = g_drill.AppState;
        g_drill.AppState  = WOK_DRILL_STATE_SAMPLE_DONE;

        /* Determine sample quality */
        if (s->VolatileConcPpm > 50.0f)
            s->SampleQuality = 1; /* good */
        else if (s->VolatileConcPpm > 10.0f)
            s->SampleQuality = 2; /* marginal */
        else
            s->SampleQuality = 3; /* bad / no detection */

        CFE_EVS_SendEvent(12, CFE_EVS_EventType_INFORMATION,
            "WOK_DRILL: Sample complete — %.1f ppm H2O, quality %d",
            (double)s->VolatileConcPpm, s->SampleQuality);

        /* Archive the drill cycle log (penetration + evolved-gas) */
        WOK_Drill_WriteLogFile();
    }

    /* Stow complete? */
    if (g_drill.AppState == WOK_DRILL_STATE_STOWING &&
        s->ArmHalState == WOK_HAL_DRILL_ARM_STOWED)
    {
        g_drill.PrevState = g_drill.AppState;
        g_drill.AppState  = WOK_DRILL_STATE_IDLE;
        CFE_EVS_SendEvent(13, CFE_EVS_EventType_INFORMATION,
            "WOK_DRILL: Arm stowed — IDLE");

        /* Drill-without-sample cycles still get their log archived */
        WOK_Drill_WriteLogFile();
    }
}

/* ════════════════════════════════════════════════════════════════════
** Fault Detection — runs every HK cycle
** ════════════════════════════════════════════════════════════════════ */

static void WOK_Drill_RunFaultDetection(void)
{
    WOK_Drill_StatusTlm_t *s = &g_drill.StatusTlm;
    uint16_t new_faults = 0;

    /* ── Arm timeout ─────────────────────────────────────────────── */
    if (g_drill.AppState == WOK_DRILL_STATE_DEPLOYING ||
        g_drill.AppState == WOK_DRILL_STATE_STOWING)
    {
        uint32_t elapsed = WOK_Drill_GetTimeMs() - g_drill.ArmCmdStartMs;
        if (elapsed > g_drill.ArmTimeoutMs)
        {
            new_faults |= WOK_DRILL_FAULT_ARM_TIMEOUT;
            CFE_EVS_SendEvent(20, CFE_EVS_EventType_ERROR,
                "WOK_DRILL: FAULT — arm timeout (%lu ms)", (unsigned long)elapsed);
        }
    }

    /* ── Drill overcurrent ───────────────────────────────────────── */
    if (g_drill.AppState == WOK_DRILL_STATE_DRILLING)
    {
        if (s->DrillCurrentAmps > g_drill.CurrentLimitA)
        {
            new_faults |= WOK_DRILL_FAULT_OVERCURRENT;
            CFE_EVS_SendEvent(21, CFE_EVS_EventType_ERROR,
                "WOK_DRILL: FAULT — drill overcurrent %.2fA (limit %.2fA)",
                (double)s->DrillCurrentAmps, (double)g_drill.CurrentLimitA);
        }

        /* Torque limit (stall detection) */
        if (s->DrillTorqueNm > g_drill.TorqueLimitNm)
        {
            new_faults |= WOK_DRILL_FAULT_DRILL_STALL;
            CFE_EVS_SendEvent(22, CFE_EVS_EventType_ERROR,
                "WOK_DRILL: FAULT — drill stall torque %.2f Nm (limit %.2f)",
                (double)s->DrillTorqueNm, (double)g_drill.TorqueLimitNm);
        }

        /* Over-depth */
        if (s->DrillDepthM > g_drill.MaxDepthM)
        {
            new_faults |= WOK_DRILL_FAULT_OVER_DEPTH;
            WOK_HAL_Drill_StopDrill();
            CFE_EVS_SendEvent(23, CFE_EVS_EventType_ERROR,
                "WOK_DRILL: FAULT — over-depth %.2fm (limit %.2f)",
                (double)s->DrillDepthM, (double)g_drill.MaxDepthM);
        }
    }

    /* ── Thermal overtemp ────────────────────────────────────────── */
    if (g_drill.AppState == WOK_DRILL_STATE_SAMPLING)
    {
        if (s->CellTempC > g_drill.MaxCellTempC)
        {
            new_faults |= WOK_DRILL_FAULT_THERMAL_OT;
            WOK_HAL_Drill_StopSample();
            CFE_EVS_SendEvent(24, CFE_EVS_EventType_ERROR,
                "WOK_DRILL: FAULT — thermal overtemp %.1fC (limit %.1f)",
                (double)s->CellTempC, (double)g_drill.MaxCellTempC);
        }
    }

    /* ── Apply faults ────────────────────────────────────────────── */
    if (new_faults)
    {
        g_drill.FaultBitmask |= new_faults;

        /* Transition to FAULT state */
        if (g_drill.AppState != WOK_DRILL_STATE_FAULT)
        {
            WOK_HAL_Drill_EmergencyStop();
            g_drill.PrevState = g_drill.AppState;
            g_drill.AppState  = WOK_DRILL_STATE_FAULT;
            CFE_EVS_SendEvent(25, CFE_EVS_EventType_CRITICAL,
                "WOK_DRILL: Entering FAULT — bitmask 0x%04X",
                g_drill.FaultBitmask);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════
** Utility: millisecond timestamp from cFE TIME
** ════════════════════════════════════════════════════════════════════ */

static uint32_t WOK_Drill_GetTimeMs(void)
{
    CFE_TIME_SysTime_t t = CFE_TIME_GetTime();
    /* Promote the *1000 to uint64 so it doesn't overflow at ~49d uptime
     * before the final uint32 truncation. uint32 wrap of the result is
     * fine — callers only use delta = end - start (modular arithmetic). */
    return (uint32_t)((uint64_t)t.Seconds * 1000ull + t.Subseconds / 4294967u);
}
