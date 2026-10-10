/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
* WOK_SEQ — Mission Sequencer / State Machine
*
* Drives the per-waypoint survey loop by commanding the child apps over
* the SB and gating each transition on their SeqId-echoed status TLM.
* The state machine runs only on SEND_HK ticks; child TLM just refreshes
* the cached status. All timeouts are wall-clock (CFE_TIME).
*
* Ops loop (VERIFY only in placement mode; HOLD only at recal-flagged
* waypoints):
*   IDLE → MOVE → [HOLD] → DEPLOY → [VERIFY] →
*          THUMP (start acq + arm + fire) → RECORD (wait acq complete)
*          ×stack → EVALUATE → [DRILL if flagged + battery OK] →
*          STOW → (next WP) → … → DONE
************************************************************************/
#include "wok_seq_app.h"
#include "wok_hal_seis.h"
#include "wok_hal_geo.h"

static WOK_Seq_AppData_t WOK_Seq_AppData;

/* Default waypoint table — body-frame deltas in the persistent start
 * frame (forward+, lateral+ = left). Demonstrates a closed-loop 2x3
 * snake-grid with three drill sites and a return-to-origin step.
 * SourceMode 0 / energy 0 = impulse at the app default everywhere. */
static const WOK_Waypoint_t DefaultWaypoints[WOK_SEQ_NUM_WAYPOINTS] = {
    { 0.0f,  0.0f, 0, 0, 0, 0, 0.0f},   /* start, no-motion baseline     */
    { 3.0f,  0.0f, 1, 0, 0, 0, 0.0f},   /* forward 3m, drill             */
    { 3.0f,  0.0f, 0, 0, 0, 0, 0.0f},   /* forward 3m                    */
    { 0.0f,  3.0f, 1, 0, 0, 0, 0.0f},   /* sideways 3m, drill (next row) */
    {-3.0f,  0.0f, 0, 0, 0, 0, 0.0f},   /* back 3m (start-frame reverse) */
    {-3.0f,  0.0f, 1, 0, 0, 0, 0.0f},   /* back 3m more, drill           */
    { 0.0f, -3.0f, 0, 0, 0, 0, 0.0f},   /* return to origin              */
};

static const char *SEQ_STATE_NAMES[] = {
    "IDLE", "MOVE", "DEPLOY", "THUMP", "RECORD", "EVALUATE",
    "DRILL", "STOW", "SAFE", "DONE", "VERIFY", "HOLD"
};

/* ================================================================== */
void WOK_Seq_AppMain(void)
{
    int32            Status;
    CFE_SB_Buffer_t *BufPtr = NULL;

    CFE_ES_PerfLogEntry(WOK_SEQ_PERF_ID);
    WOK_Seq_AppData.RunStatus = CFE_ES_RunStatus_APP_RUN;
    Status = WOK_Seq_AppInit();
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("WOK_SEQ: Init failed (0x%08lX)\n",
                             (unsigned long)Status);
        WOK_Seq_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&WOK_Seq_AppData.RunStatus) == true)
    {
        CFE_ES_PerfLogExit(WOK_SEQ_PERF_ID);
        Status = CFE_SB_ReceiveBuffer(&BufPtr, WOK_Seq_AppData.CmdPipe,
                                       CFE_SB_PEND_FOREVER);
        CFE_ES_PerfLogEntry(WOK_SEQ_PERF_ID);

        if (Status == CFE_SUCCESS)
            WOK_Seq_ProcessCommandPacket(BufPtr);
        else
        {
            CFE_EVS_SendEvent(WOK_SEQ_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "WOK_SEQ: SB pipe error (0x%08lX), app will exit",
                              (unsigned long)Status);
            WOK_Seq_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }
    CFE_ES_PerfLogExit(WOK_SEQ_PERF_ID);
    CFE_ES_ExitApp(WOK_Seq_AppData.RunStatus);
}

/* ================================================================== */
int32 WOK_Seq_AppInit(void)
{
    int32 Status;

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&WOK_Seq_AppData.CmdPipe,
                                WOK_SEQ_PIPE_DEPTH, WOK_SEQ_PIPE_NAME);
    if (Status != CFE_SUCCESS) return Status;

    /* Subscribe: own command + wakeup, then the five child TLM streams */
    {
        static const struct { uint32 Mid; const char *Name; } Subs[] = {
            { WOK_SEQ_CMD_MID,         "CMD"         },
            { WOK_SEQ_SEND_HK_MID,     "SEND_HK"     },
            { WOK_NAV_STATUS_TLM_MID,  "NAV status"  },
            { WOK_GEO_STATUS_TLM_MID,  "GEO status"  },
            { WOK_SEIS_STATUS_TLM_MID, "SEIS status" },
            { WOK_HK_TLM_MID,          "HK health"   },
            { WOK_DRILL_HK_TLM_MID,    "DRILL HK"    },
        };
        for (size_t i = 0; i < sizeof(Subs) / sizeof(Subs[0]); i++)
        {
            Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(Subs[i].Mid),
                                      WOK_Seq_AppData.CmdPipe);
            if (Status != CFE_SUCCESS)
            {
                CFE_EVS_SendEvent(WOK_SEQ_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "WOK_SEQ: Subscribe to %s failed (0x%08lX)",
                                  Subs[i].Name, (unsigned long)Status);
                return Status;
            }
        }
    }

    /* Init TLM headers */
    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.HkTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_SEQ_TLM_MID),
                 sizeof(WOK_Seq_HkTlm_t));
    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.ModeTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_SEQ_MODE_TLM_MID),
                 sizeof(WOK_Seq_ModeTlm_t));

    /* Init child command packets */
    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.NavGoToCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_NAV_CMD_MID), sizeof(WOK_Nav_GoToCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.NavGoToCmd.CmdHeader), WOK_NAV_GOTO_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.NavStopCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_NAV_CMD_MID), sizeof(WOK_Nav_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.NavStopCmd.CmdHeader), WOK_NAV_STOP_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.GeoDeployCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_GEO_CMD_MID), sizeof(WOK_Geo_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.GeoDeployCmd.CmdHeader), WOK_GEO_DEPLOY_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.GeoStowCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_GEO_CMD_MID), sizeof(WOK_Geo_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.GeoStowCmd.CmdHeader), WOK_GEO_STOW_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.GeoPlacePodCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_GEO_CMD_MID), sizeof(WOK_Geo_PlacePodCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.GeoPlacePodCmd.CmdHeader), WOK_GEO_PLACE_POD_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.GeoRetrievePodCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_GEO_CMD_MID), sizeof(WOK_Geo_RetrievePodCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.GeoRetrievePodCmd.CmdHeader), WOK_GEO_RETRIEVE_POD_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.GeoReseatPodCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_GEO_CMD_MID), sizeof(WOK_Geo_RetrievePodCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.GeoReseatPodCmd.CmdHeader), WOK_GEO_RESEAT_POD_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisPodGeomCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_PodGeomCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisPodGeomCmd.CmdHeader), WOK_SEIS_SET_POD_GEOM_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisStartCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_StartAcqCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisStartCmd.CmdHeader), WOK_SEIS_START_ACQ_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisStopCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisStopCmd.CmdHeader), WOK_SEIS_STOP_ACQ_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpArmCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpArmCmd.CmdHeader), WOK_SEIS_THUMP_ARM_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpFireCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpFireCmd.CmdHeader), WOK_SEIS_THUMP_FIRE_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpResetCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpResetCmd.CmdHeader), WOK_SEIS_THUMP_RESET_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisEvalCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisEvalCmd.CmdHeader), WOK_SEIS_EVALUATE_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisPlateReseatCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_NoArgsCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisPlateReseatCmd.CmdHeader), WOK_SEIS_PLATE_RESEAT_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.SeisCfgSrcCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID), sizeof(WOK_Seis_ConfigSourceCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.SeisCfgSrcCmd.CmdHeader), WOK_SEIS_CONFIG_SOURCE_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.DrillDeployCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_DRILL_CMD_MID), sizeof(WOK_Drill_DeployArmCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.DrillDeployCmd.CmdHeader), WOK_DRILL_DEPLOY_ARM_CC);
    WOK_Seq_AppData.DrillDeployCmd.TargetAngleDeg = WOK_DRILL_DEFAULT_ARM_ANGLE_DEG;

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.DrillStowCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_DRILL_CMD_MID), sizeof(WOK_Drill_NoopCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.DrillStowCmd.CmdHeader), WOK_DRILL_STOW_ARM_CC);

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.DrillStartCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_DRILL_CMD_MID), sizeof(WOK_Drill_StartDrillCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.DrillStartCmd.CmdHeader), WOK_DRILL_START_DRILL_CC);
    WOK_Seq_AppData.DrillStartCmd.DepthM = WOK_DRILL_DEFAULT_DEPTH_M;
    WOK_Seq_AppData.DrillStartCmd.RPM    = WOK_DRILL_DEFAULT_RPM;

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.DrillSampleCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_DRILL_CMD_MID), sizeof(WOK_Drill_StartSampleCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.DrillSampleCmd.CmdHeader), WOK_DRILL_START_SAMPLE_CC);
    WOK_Seq_AppData.DrillSampleCmd.HeatSec = WOK_DRILL_DEFAULT_HEAT_SEC;
    WOK_Seq_AppData.DrillSampleCmd.CoolSec = WOK_DRILL_DEFAULT_COOL_SEC;

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seq_AppData.DrillEstopCmd.CmdHeader),
                 CFE_SB_ValueToMsgId(WOK_DRILL_CMD_MID), sizeof(WOK_Drill_NoopCmd_t));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(WOK_Seq_AppData.DrillEstopCmd.CmdHeader), WOK_DRILL_ESTOP_CC);

    /* Default state */
    WOK_Seq_AppData.State            = WOK_STATE_IDLE;
    WOK_Seq_AppData.PrevState        = WOK_STATE_IDLE;
    WOK_Seq_AppData.EnteredState     = true;
    WOK_Seq_AppData.WaypointIdx      = 0;
    WOK_Seq_AppData.LoopCount        = 0;
    WOK_Seq_AppData.FaultMask        = WOK_FAULT_NONE;
    WOK_Seq_AppData.CmdCounter       = 0;
    WOK_Seq_AppData.ErrCounter       = 0;
    WOK_Seq_AppData.ThumpTickCount   = 0;
    WOK_Seq_AppData.EvalFlag         = 0;
    WOK_Seq_AppData.EvalConfidence   = 0.0f;
    WOK_Seq_AppData.DrillSubState    = WOK_DRILL_SUB_DEPLOY;
    WOK_Seq_AppData.SeqId            = 0;
    WOK_Seq_AppData.PendingSeqId     = 0;
    WOK_Seq_AppData.SiteX_m          = 0.0f;
    WOK_Seq_AppData.SiteY_m          = 0.0f;
    WOK_Seq_AppData.SiteHeadingDeg   = 0.0f;
    WOK_Seq_AppData.StrikesDone      = 0;
    WOK_Seq_AppData.PlateReseatsSite = 0;
    WOK_Seq_AppData.AcceptDegraded   = 0;
    for (int i = 0; i < WOK_SEQ_MAX_WAYPOINTS; i++)
        WOK_Seq_AppData.SiteDegraded[i] = 0;
    WOK_Seq_AppData.PlacementMode    = WOK_SEQ_PLACEMENT_FIXED;
    WOK_Seq_AppData.PlaceSubState    = WOK_PLACE_SUB_MOVE;
    WOK_Seq_AppData.PlacePodIdx      = 0;
    WOK_Seq_AppData.HoldRelease      = 0;
    WOK_Seq_AppData.MissionMode      = WOK_SEQ_MODE_DRILL_INLINE;
    WOK_Seq_AppData.DrillsCompleted  = 0;
    WOK_Seq_AppData.ReturnPhase      = 0;
    WOK_Seq_AppData.ReturnTargetIdx  = 0;
    for (int i = 0; i < WOK_SEQ_MAX_WAYPOINTS; i++)
        WOK_Seq_AppData.SiteScore[i] = -1.0f;
    WOK_Seq_AppData.NavTlmLenErrReported   = 0;
    WOK_Seq_AppData.GeoTlmLenErrReported   = 0;
    WOK_Seq_AppData.SeisTlmLenErrReported  = 0;
    WOK_Seq_AppData.HkTlmLenErrReported    = 0;
    WOK_Seq_AppData.DrillTlmLenErrReported = 0;

    /* Load waypoint table */
    WOK_Seq_AppData.TotalWaypoints = WOK_SEQ_NUM_WAYPOINTS;
    for (int i = 0; i < WOK_SEQ_NUM_WAYPOINTS; i++)
        WOK_Seq_AppData.Waypoints[i] = DefaultWaypoints[i];

    /* Safety thresholds — track HK app's defaults from wok_platform_cfg.h */
    WOK_Seq_AppData.BatteryMinV        = WOK_HK_DEFAULT_BATT_MIN_V;
    WOK_Seq_AppData.TiltMaxDeg         = WOK_HK_DEFAULT_TILT_MAX_DEG;
    WOK_Seq_AppData.TempMaxC           = WOK_HK_DEFAULT_TEMP_MAX_C;
    WOK_Seq_AppData.RecordDurationSec  = WOK_SEIS_DEFAULT_DURATION_SEC;

    /* Default SEIS config */
    WOK_Seq_AppData.SeisStartCmd.Duration_sec  = WOK_SEIS_DEFAULT_DURATION_SEC;
    WOK_Seq_AppData.SeisStartCmd.SampleRate_Hz = WOK_SEIS_DEFAULT_FS_HZ;
    WOK_Seq_AppData.SeisStartCmd.Gain          = WOK_SEIS_DEFAULT_GAIN;
    WOK_Seq_AppData.SeisStartCmd.ChannelMask   = 0x0F;

    CFE_EVS_SendEvent(WOK_SEQ_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_SEQ: App initialised (v" WOK_VERSION_STRING ") — %u waypoints loaded",
                      WOK_Seq_AppData.TotalWaypoints);

    return CFE_SUCCESS;
}

/* ================================================================== */
void WOK_Seq_ChangeState(WOK_State_t NewState)
{
    WOK_Seq_AppData.PrevState    = WOK_Seq_AppData.State;
    WOK_Seq_AppData.State        = NewState;
    WOK_Seq_AppData.EnteredState = true;
    CFE_EVS_SendEvent(WOK_SEQ_STATE_CHANGE_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_SEQ: %s → %s (WP %u/%u)",
                      SEQ_STATE_NAMES[WOK_Seq_AppData.PrevState],
                      SEQ_STATE_NAMES[NewState],
                      WOK_Seq_AppData.WaypointIdx,
                      WOK_Seq_AppData.TotalWaypoints);
}

/* ================================================================== */
/*  SeqId — monotonic sequence counter for stale-telemetry gating     */
/* ================================================================== */
static uint16 WOK_Seq_NextSeqId(void)
{
    WOK_Seq_AppData.SeqId++;
    if (WOK_Seq_AppData.SeqId == 0)
        WOK_Seq_AppData.SeqId = 1;  /* 0 reserved for ground/don't-care */
    WOK_Seq_AppData.PendingSeqId = WOK_Seq_AppData.SeqId;
    return WOK_Seq_AppData.SeqId;
}

/* ================================================================== */
/*  SB command senders                                                */
/* ================================================================== */
void WOK_Seq_SendNavGoTo(float df, float dl)
{
    WOK_Seq_AppData.NavGoToCmd.SeqId         = WOK_Seq_NextSeqId();
    WOK_Seq_AppData.NavGoToCmd.DeltaForwardM = df;
    WOK_Seq_AppData.NavGoToCmd.DeltaLateralM = dl;
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.NavGoToCmd.CmdHeader), true);
}
void WOK_Seq_SendNavStop(void)       { WOK_Seq_AppData.NavStopCmd.SeqId       = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.NavStopCmd.CmdHeader), true); }
void WOK_Seq_SendGeoDeploy(void)     { WOK_Seq_AppData.GeoDeployCmd.SeqId     = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.GeoDeployCmd.CmdHeader), true); }
void WOK_Seq_SendGeoStow(void)       { WOK_Seq_AppData.GeoStowCmd.SeqId       = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.GeoStowCmd.CmdHeader), true); }
void WOK_Seq_SendSeisStart(bool test_shot)
{
    /* Stamp the surveyed site (NAV pose at DEPLOY) into the acquisition.
     * Test shots (CP-4) get a short window and the TEST flag — SEIS
     * scores the pods and keeps the record out of the stack. */
    WOK_Seq_AppData.SeisStartCmd.SiteX_m        = WOK_Seq_AppData.SiteX_m;
    WOK_Seq_AppData.SeisStartCmd.SiteY_m        = WOK_Seq_AppData.SiteY_m;
    WOK_Seq_AppData.SeisStartCmd.SiteHeadingDeg = WOK_Seq_AppData.SiteHeadingDeg;
    WOK_Seq_AppData.SeisStartCmd.StackIdx       = WOK_Seq_AppData.StrikesDone;
    WOK_Seq_AppData.SeisStartCmd.StackTarget    = WOK_SEIS_STACK_TARGET;
    WOK_Seq_AppData.SeisStartCmd.Duration_sec   = test_shot
                                                ? WOK_SEQ_TEST_DUR_SEC
                                                : WOK_Seq_AppData.RecordDurationSec;
    WOK_Seq_AppData.SeisStartCmd.Flags          =
        (WOK_Seq_AppData.AcceptDegraded ? WOK_SEIS_ACQFLAG_ACCEPT_DEGRADED : 0)
      | (test_shot ? WOK_SEIS_ACQFLAG_TEST_SHOT : 0);
    WOK_Seq_AppData.SeisStartCmd.SeqId          = WOK_Seq_NextSeqId();
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisStartCmd.CmdHeader), true);
}
void WOK_Seq_SendSeisStop(void)      { WOK_Seq_AppData.SeisStopCmd.SeqId      = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisStopCmd.CmdHeader), true); }
void WOK_Seq_SendSeisConfigSource(uint8 mode, float energy_j)
{
    /* Per-site source select. Sweep pilot parameters (f0/f1/duration)
     * are SEIS-side configuration — zeros here mean "keep current". */
    WOK_Seq_AppData.SeisCfgSrcCmd.SourceMode = mode;
    WOK_Seq_AppData.SeisCfgSrcCmd.EnergyJ    = energy_j;
    WOK_Seq_AppData.SeisCfgSrcCmd.SweepF0_Hz = 0.0f;
    WOK_Seq_AppData.SeisCfgSrcCmd.SweepF1_Hz = 0.0f;
    WOK_Seq_AppData.SeisCfgSrcCmd.SweepDur_s = 0.0f;
    WOK_Seq_AppData.SeisCfgSrcCmd.SeqId      = WOK_Seq_NextSeqId();
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisCfgSrcCmd.CmdHeader), true);
}
void WOK_Seq_SendThumpArm(void)      { WOK_Seq_AppData.SeisThumpArmCmd.SeqId  = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpArmCmd.CmdHeader), true); }
void WOK_Seq_SendThumpFire(void)     { WOK_Seq_AppData.SeisThumpFireCmd.SeqId = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpFireCmd.CmdHeader), true); }
void WOK_Seq_SendThumpReset(void)    { WOK_Seq_AppData.SeisThumpResetCmd.SeqId = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisThumpResetCmd.CmdHeader), true); }
void WOK_Seq_SendSeisEvaluate(void)  { WOK_Seq_AppData.SeisEvalCmd.SeqId      = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisEvalCmd.CmdHeader), true); }
void WOK_Seq_SendSeisPlateReseat(void) { WOK_Seq_AppData.SeisPlateReseatCmd.SeqId = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisPlateReseatCmd.CmdHeader), true); }
void WOK_Seq_SendGeoPlacePod(uint8 pod, float x, float y)
{
    WOK_Seq_AppData.GeoPlacePodCmd.Pod   = pod;
    WOK_Seq_AppData.GeoPlacePodCmd.X_m   = x;
    WOK_Seq_AppData.GeoPlacePodCmd.Y_m   = y;
    WOK_Seq_AppData.GeoPlacePodCmd.SeqId = WOK_Seq_NextSeqId();
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.GeoPlacePodCmd.CmdHeader), true);
}
void WOK_Seq_SendGeoRetrievePod(uint8 pod)
{
    WOK_Seq_AppData.GeoRetrievePodCmd.Pod   = pod;
    WOK_Seq_AppData.GeoRetrievePodCmd.SeqId = WOK_Seq_NextSeqId();
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.GeoRetrievePodCmd.CmdHeader), true);
}
void WOK_Seq_SendGeoReseatPod(uint8 pod)
{
    WOK_Seq_AppData.GeoReseatPodCmd.Pod   = pod;
    WOK_Seq_AppData.GeoReseatPodCmd.SeqId = WOK_Seq_NextSeqId();
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.GeoReseatPodCmd.CmdHeader), true);
}
void WOK_Seq_SendSeisPodGeometry(void)
{
    /* Actual placed geometry + seating metadata for the site: placed
     * poses recorded during the placement loop, seating results from
     * the cached GEO status. */
    for (uint8 p = 0; p < WOK_HAL_GEO_MAX_PODS; p++)
    {
        float dx = WOK_Seq_AppData.PodPlacedX[p] - WOK_Seq_AppData.SiteX_m;
        float dy = WOK_Seq_AppData.PodPlacedY[p] - WOK_Seq_AppData.SiteY_m;
        WOK_Seq_AppData.SeisPodGeomCmd.PodX_m[p]      = WOK_Seq_AppData.PodPlacedX[p];
        WOK_Seq_AppData.SeisPodGeomCmd.PodY_m[p]      = WOK_Seq_AppData.PodPlacedY[p];
        WOK_Seq_AppData.SeisPodGeomCmd.PodOffset_m[p] = sqrtf(dx * dx + dy * dy);
        WOK_Seq_AppData.SeisPodGeomCmd.PodAttempt[p]  = WOK_Seq_AppData.GeoStatus.PodReseats[p];
    }
    WOK_Seq_AppData.SeisPodGeomCmd.PodMarginalMask = WOK_Seq_AppData.GeoStatus.MarginalMask;
    WOK_Seq_AppData.SeisPodGeomCmd.SeqId           = WOK_Seq_NextSeqId();
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.SeisPodGeomCmd.CmdHeader), true);
}

/* World-frame target for pod `pod` at the current site: the configured
 * offset laid along the site heading. */
static void WOK_Seq_PodTarget(uint8 pod, float *tx, float *ty)
{
    static const float off[WOK_HAL_GEO_MAX_PODS] = WOK_SEIS_POD_OFFSETS_M_INIT;
    float h = WOK_Seq_AppData.SiteHeadingDeg * 3.14159265f / 180.0f;
    *tx = WOK_Seq_AppData.SiteX_m + off[pod] * cosf(h);
    *ty = WOK_Seq_AppData.SiteY_m + off[pod] * sinf(h);
}

/* Drive to a world-frame target: relative step from the current pose */
static void WOK_Seq_NavGoToWorld(float tx, float ty)
{
    WOK_Seq_SendNavGoTo(tx - WOK_Seq_AppData.NavStatus.PoseX_m,
                        ty - WOK_Seq_AppData.NavStatus.PoseY_m);
}

/* NAV arrival for the pending SeqId */
static bool WOK_Seq_NavArrived(void)
{
    return WOK_Seq_AppData.NavStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
           WOK_Seq_AppData.NavStatus.ArrivedFlag;
}
void WOK_Seq_SendDrillDeploy(void)   { WOK_Seq_AppData.DrillDeployCmd.SeqId   = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.DrillDeployCmd.CmdHeader), true); }
void WOK_Seq_SendDrillStow(void)     { WOK_Seq_AppData.DrillStowCmd.SeqId     = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.DrillStowCmd.CmdHeader), true); }
void WOK_Seq_SendDrillStart(void)
{
    /* Same site stamp as the seismic record — the tie-point linkage */
    WOK_Seq_AppData.DrillStartCmd.SiteX_m = WOK_Seq_AppData.SiteX_m;
    WOK_Seq_AppData.DrillStartCmd.SiteY_m = WOK_Seq_AppData.SiteY_m;
    WOK_Seq_AppData.DrillStartCmd.SeqId   = WOK_Seq_NextSeqId();
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.DrillStartCmd.CmdHeader), true);
}
void WOK_Seq_SendDrillSample(void)   { WOK_Seq_AppData.DrillSampleCmd.SeqId   = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.DrillSampleCmd.CmdHeader), true); }
void WOK_Seq_SendDrillEstop(void)    { WOK_Seq_AppData.DrillEstopCmd.SeqId    = WOK_Seq_NextSeqId(); CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.DrillEstopCmd.CmdHeader), true); }

/* ================================================================== */
/*  Packet dispatcher                                                 */
/*  The state machine runs ONLY on SEND_HK — on each SEND_HK tick     */
/*  (~25 Hz with the stock SCH_LAB table; platform-dependent, which   */
/*  is why every timeout below is wall-clock). Child TLM updates the  */
/*  cached data but does NOT run the state machine.                   */
/* ================================================================== */
void WOK_Seq_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case WOK_SEQ_CMD_MID:
            WOK_Seq_ProcessGroundCommand(BufPtr);
            break;

        case WOK_SEQ_SEND_HK_MID:
            WOK_Seq_ReportHousekeeping();
            WOK_Seq_RunStateMachine();   /* ← ONLY place SM runs */
            break;

        /* Child TLM — length-checked, then cached; do NOT run the state
         * machine. A short/long packet is dropped (never partially copied). */
        case WOK_NAV_STATUS_TLM_MID:
            if (WOK_Seq_VerifyTlmLength(BufPtr, sizeof(WOK_Nav_StatusTlm_t),
                                        &WOK_Seq_AppData.NavTlmLenErrReported))
                memcpy(&WOK_Seq_AppData.NavStatus, BufPtr, sizeof(WOK_Nav_StatusTlm_t));
            break;
        case WOK_GEO_STATUS_TLM_MID:
            if (WOK_Seq_VerifyTlmLength(BufPtr, sizeof(WOK_Geo_StatusTlm_t),
                                        &WOK_Seq_AppData.GeoTlmLenErrReported))
                memcpy(&WOK_Seq_AppData.GeoStatus, BufPtr, sizeof(WOK_Geo_StatusTlm_t));
            break;
        case WOK_SEIS_STATUS_TLM_MID:
            if (WOK_Seq_VerifyTlmLength(BufPtr, sizeof(WOK_Seis_StatusTlm_t),
                                        &WOK_Seq_AppData.SeisTlmLenErrReported))
                memcpy(&WOK_Seq_AppData.SeisStatus, BufPtr, sizeof(WOK_Seis_StatusTlm_t));
            break;
        case WOK_HK_TLM_MID:
            if (WOK_Seq_VerifyTlmLength(BufPtr, sizeof(WOK_Hk_HealthTlm_t),
                                        &WOK_Seq_AppData.HkTlmLenErrReported))
                memcpy(&WOK_Seq_AppData.HkHealth, BufPtr, sizeof(WOK_Hk_HealthTlm_t));
            break;
        case WOK_DRILL_HK_TLM_MID:
            if (WOK_Seq_VerifyTlmLength(BufPtr, sizeof(WOK_Drill_HkTlm_t),
                                        &WOK_Seq_AppData.DrillTlmLenErrReported))
                memcpy(&WOK_Seq_AppData.DrillHk, BufPtr, sizeof(WOK_Drill_HkTlm_t));
            break;

        default:
            WOK_Seq_AppData.ErrCounter++;
            break;
    }
}

/* ================================================================== */
/*  Ground command handler                                            */
/* ================================================================== */
void WOK_Seq_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t CmdCode = 0;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &CmdCode);

    switch (CmdCode)
    {
        case WOK_SEQ_NOOP_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_NoArgsCmd_t)))
            {
                WOK_Seq_AppData.CmdCounter++;
                CFE_EVS_SendEvent(WOK_SEQ_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEQ: NOOP (v" WOK_VERSION_STRING ")");
            }
            break;

        case WOK_SEQ_RESET_CTRS_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_NoArgsCmd_t)))
            {
                WOK_Seq_AppData.CmdCounter = 0;
                WOK_Seq_AppData.ErrCounter = 0;
            }
            break;

        case WOK_SEQ_START_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_NoArgsCmd_t)))
            {
                WOK_Seq_AppData.CmdCounter++;
                WOK_Seq_AppData.WaypointIdx  = 0;
                WOK_Seq_AppData.LoopCount    = 0;
                WOK_Seq_AppData.FaultMask    = WOK_FAULT_NONE;
                WOK_Seq_AppData.EvalFlag     = 0;
                WOK_Seq_AppData.DrillsCompleted = 0;
                WOK_Seq_AppData.ReturnPhase     = 0;
                WOK_Seq_AppData.PlateReseatsSite = 0;
                WOK_Seq_AppData.AcceptDegraded   = 0;
                WOK_Seq_AppData.HoldRelease      = 0;
                for (int i = 0; i < WOK_SEQ_MAX_WAYPOINTS; i++)
                {
                    WOK_Seq_AppData.SiteScore[i]    = -1.0f;
                    WOK_Seq_AppData.SiteDegraded[i] = 0;
                }
                /* Clear cached status to avoid stale data */
                memset(&WOK_Seq_AppData.NavStatus, 0, sizeof(WOK_Seq_AppData.NavStatus));
                memset(&WOK_Seq_AppData.GeoStatus, 0, sizeof(WOK_Seq_AppData.GeoStatus));
                memset(&WOK_Seq_AppData.SeisStatus, 0, sizeof(WOK_Seq_AppData.SeisStatus));
                WOK_Seq_ChangeState(WOK_STATE_MOVE);
                CFE_EVS_SendEvent(WOK_SEQ_START_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEQ: Sequence started — %u waypoints",
                                  WOK_Seq_AppData.TotalWaypoints);
            }
            break;

        case WOK_SEQ_ABORT_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_NoArgsCmd_t)))
            {
                WOK_Seq_AppData.CmdCounter++;
                WOK_Seq_SendNavStop();
                WOK_Seq_SendGeoStow();
                WOK_Seq_SendSeisStop();
                WOK_Seq_SendDrillEstop();
                WOK_Seq_ChangeState(WOK_STATE_SAFE);
                CFE_EVS_SendEvent(WOK_SEQ_ABORT_EID, CFE_EVS_EventType_CRITICAL,
                                  "WOK_SEQ: ABORT — entering SAFE");
            }
            break;

        case WOK_SEQ_RESUME_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_NoArgsCmd_t)))
            {
                if (WOK_Seq_AppData.State == WOK_STATE_SAFE)
                {
                    WOK_Seq_AppData.CmdCounter++;
                    WOK_Seq_AppData.FaultMask  = WOK_FAULT_NONE;
                    /* Drop cached child-app statuses so stale FaultMask bits
                     * from before the abort don't immediately re-trigger SAFE
                     * before child apps publish fresh telemetry. */
                    memset(&WOK_Seq_AppData.NavStatus,  0, sizeof(WOK_Seq_AppData.NavStatus));
                    memset(&WOK_Seq_AppData.GeoStatus,  0, sizeof(WOK_Seq_AppData.GeoStatus));
                    memset(&WOK_Seq_AppData.SeisStatus, 0, sizeof(WOK_Seq_AppData.SeisStatus));
                    WOK_Seq_ChangeState(WOK_STATE_MOVE);
                    CFE_EVS_SendEvent(WOK_SEQ_RESUME_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEQ: Resumed at WP %u",
                                      WOK_Seq_AppData.WaypointIdx);
                }
                else
                {
                    WOK_Seq_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_SEQ_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEQ: RESUME rejected — not in SAFE");
                }
            }
            break;

        case WOK_SEQ_SET_WAYPOINT_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_SetWaypointCmd_t)))
            {
                WOK_Seq_SetWaypointCmd_t *Cmd = (WOK_Seq_SetWaypointCmd_t *)BufPtr;
                if (Cmd->WaypointIdx < WOK_Seq_AppData.TotalWaypoints)
                {
                    WOK_Seq_AppData.CmdCounter++;
                    WOK_Seq_AppData.WaypointIdx = Cmd->WaypointIdx;
                }
                else
                {
                    WOK_Seq_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_SEQ_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEQ: SET_WAYPOINT %u rejected — %u waypoints active",
                                      Cmd->WaypointIdx, WOK_Seq_AppData.TotalWaypoints);
                }
            }
            break;

        case WOK_SEQ_SET_MODE_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_SetModeCmd_t)))
            {
                WOK_Seq_SetModeCmd_t *Cmd = (WOK_Seq_SetModeCmd_t *)BufPtr;
                if (Cmd->Mode <= WOK_SEQ_MODE_SURVEY_FIRST)
                {
                    WOK_Seq_AppData.CmdCounter++;
                    WOK_Seq_AppData.MissionMode = Cmd->Mode;
                    CFE_EVS_SendEvent(WOK_SEQ_MODE_SET_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEQ: Mission mode = %s",
                                      Cmd->Mode == WOK_SEQ_MODE_SURVEY_FIRST
                                          ? "SURVEY_FIRST" : "DRILL_INLINE");
                }
                else
                {
                    WOK_Seq_AppData.ErrCounter++;
                }
            }
            break;

        case WOK_SEQ_LOAD_WAYPOINT_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_LoadWaypointCmd_t)))
            {
                WOK_Seq_LoadWaypointCmd_t *Cmd = (WOK_Seq_LoadWaypointCmd_t *)BufPtr;
                if (Cmd->WaypointIdx < WOK_SEQ_MAX_WAYPOINTS &&
                    Cmd->SourceMode <= 1 && Cmd->RecalFlag <= 1)
                {
                    WOK_Seq_AppData.CmdCounter++;
                    WOK_Waypoint_t *wp = &WOK_Seq_AppData.Waypoints[Cmd->WaypointIdx];
                    wp->DeltaForwardM = Cmd->DeltaForwardM;
                    wp->DeltaLateralM = Cmd->DeltaLateralM;
                    wp->DrillFlag     = Cmd->DrillFlag;
                    wp->SourceMode    = Cmd->SourceMode;
                    wp->SourceEnergyJ = Cmd->SourceEnergyJ;
                    wp->RecalFlag     = Cmd->RecalFlag;
                    CFE_EVS_SendEvent(WOK_SEQ_WP_LOADED_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEQ: WP %u loaded — (%.1f, %.1f) drill=%u src=%s E=%.0fJ recal=%u",
                                      Cmd->WaypointIdx,
                                      (double)Cmd->DeltaForwardM,
                                      (double)Cmd->DeltaLateralM, Cmd->DrillFlag,
                                      Cmd->SourceMode ? "SWEEP" : "IMPULSE",
                                      (double)Cmd->SourceEnergyJ, Cmd->RecalFlag);
                }
                else
                {
                    WOK_Seq_AppData.ErrCounter++;
                }
            }
            break;

        case WOK_SEQ_SET_PLACEMENT_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_SetPlacementCmd_t)))
            {
                WOK_Seq_SetPlacementCmd_t *Cmd = (WOK_Seq_SetPlacementCmd_t *)BufPtr;
                if (Cmd->Placement <= WOK_SEQ_PLACEMENT_PER_POD)
                {
                    WOK_Seq_AppData.CmdCounter++;
                    WOK_Seq_AppData.PlacementMode = Cmd->Placement;
                    CFE_EVS_SendEvent(WOK_SEQ_PLACEMENT_SET_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEQ: Placement mode = %s",
                                      Cmd->Placement == WOK_SEQ_PLACEMENT_PER_POD
                                          ? "PER_POD" : "FIXED_ARRAY");
                }
                else
                {
                    WOK_Seq_AppData.ErrCounter++;
                }
            }
            break;

        case WOK_SEQ_RELEASE_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_NoArgsCmd_t)))
            {
                if (WOK_Seq_AppData.State == WOK_STATE_HOLD)
                {
                    WOK_Seq_AppData.CmdCounter++;
                    WOK_Seq_AppData.HoldRelease = 1;
                    CFE_EVS_SendEvent(WOK_SEQ_RELEASE_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEQ: Recalibration hold released by ground");
                }
                else
                {
                    WOK_Seq_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_SEQ_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEQ: RELEASE rejected — not in HOLD");
                }
            }
            break;

        case WOK_SEQ_SET_WP_COUNT_CC:
            if (WOK_Seq_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seq_SetWpCountCmd_t)))
            {
                WOK_Seq_SetWpCountCmd_t *Cmd = (WOK_Seq_SetWpCountCmd_t *)BufPtr;
                if (Cmd->Count > 0 && Cmd->Count <= WOK_SEQ_MAX_WAYPOINTS)
                {
                    WOK_Seq_AppData.CmdCounter++;
                    WOK_Seq_AppData.TotalWaypoints = Cmd->Count;
                    CFE_EVS_SendEvent(WOK_SEQ_WP_COUNT_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEQ: Waypoint count = %u", Cmd->Count);
                }
                else
                {
                    WOK_Seq_AppData.ErrCounter++;
                }
            }
            break;

        default:
            WOK_Seq_AppData.ErrCounter++;
            CFE_EVS_SendEvent(WOK_SEQ_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                              "WOK_SEQ: Unknown CC %u", CmdCode);
            break;
    }
}

/* ================================================================== */
void WOK_Seq_ReportHousekeeping(void)
{
    WOK_Seq_AppData.HkTlm.CmdCounter         = WOK_Seq_AppData.CmdCounter;
    WOK_Seq_AppData.HkTlm.ErrCounter         = WOK_Seq_AppData.ErrCounter;
    WOK_Seq_AppData.HkTlm.CurrentState       = (uint8)WOK_Seq_AppData.State;
    WOK_Seq_AppData.HkTlm.PreviousState      = (uint8)WOK_Seq_AppData.PrevState;
    WOK_Seq_AppData.HkTlm.CurrentWaypointIdx = WOK_Seq_AppData.WaypointIdx;
    WOK_Seq_AppData.HkTlm.FaultMask          = WOK_Seq_AppData.FaultMask;
    WOK_Seq_AppData.HkTlm.SeqId              = WOK_Seq_AppData.SeqId;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Seq_AppData.HkTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.HkTlm.TlmHeader), true);

    WOK_Seq_AppData.ModeTlm.State          = (uint8)WOK_Seq_AppData.State;
    WOK_Seq_AppData.ModeTlm.LoopCount      = WOK_Seq_AppData.LoopCount;
    WOK_Seq_AppData.ModeTlm.TotalWaypoints = WOK_Seq_AppData.TotalWaypoints;
    WOK_Seq_AppData.ModeTlm.FaultMask      = WOK_Seq_AppData.FaultMask;
    WOK_Seq_AppData.ModeTlm.MissionMode    = WOK_Seq_AppData.MissionMode;
    WOK_Seq_AppData.ModeTlm.DrillsCompleted = WOK_Seq_AppData.DrillsCompleted;
    {
        int best = -1;
        float bs = -1.0f;
        for (uint16 i = 0; i < WOK_Seq_AppData.TotalWaypoints; i++)
            if (WOK_Seq_AppData.SiteScore[i] > bs)
            {
                bs = WOK_Seq_AppData.SiteScore[i];
                best = i;
            }
        WOK_Seq_AppData.ModeTlm.BestSiteIdx   = (best >= 0) ? (uint8)best : 0xFF;
        WOK_Seq_AppData.ModeTlm.BestSiteScore = (best >= 0) ? bs : 0.0f;
    }

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Seq_AppData.ModeTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seq_AppData.ModeTlm.TlmHeader), true);
}

/* Fire a low-energy coupling test shot (CP-4): configure the source at
 * the test energy, pre-trigger the (short) record, arm — the VERIFY
 * SHOT sub-state fires on ARMED, exactly like THUMP. */
static void WOK_Seq_StartTestShot(void)
{
    WOK_Seq_SendSeisConfigSource(WOK_HAL_SRC_MODE_IMPULSE, WOK_SEQ_TEST_ENERGY_J);
    WOK_Seq_AppData.SeisStatus.Acquiring         = 1;
    WOK_Seq_AppData.SeisStatus.SamplesAcquired_k = 0;
    WOK_Seq_SendSeisStart(true);
    WOK_Seq_SendThumpReset();
    WOK_Seq_SendThumpArm();
    WOK_Seq_AppData.ThumpEntryTime = CFE_TIME_GetTime();
    WOK_Seq_AppData.VerifySub      = WOK_VERIFY_SUB_SHOT;
}

/* Site finished (array stowed/recaptured): advance the waypoint loop */
static void WOK_Seq_AdvanceWaypoint(void)
{
    WOK_Seq_AppData.WaypointIdx++;
    if (WOK_Seq_AppData.WaypointIdx >= WOK_Seq_AppData.TotalWaypoints)
    {
        WOK_Seq_AppData.LoopCount++;
        CFE_EVS_SendEvent(WOK_SEQ_CYCLE_COMPLETE_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "WOK_SEQ: All %u waypoints complete (loop %u)",
                          WOK_Seq_AppData.TotalWaypoints,
                          WOK_Seq_AppData.LoopCount);
        WOK_Seq_CheckMissionEnd();
    }
    else
    {
        WOK_Seq_ChangeState(WOK_STATE_MOVE);
    }
}

/* ================================================================== */
/*  THE STATE MACHINE — runs on each SEND_HK tick (~25 Hz with the     */
/*  stock SCH_LAB table; platform-dependent). Nothing here counts     */
/*  ticks as time: every timeout is wall-clock via CFE_TIME.          */
/* ================================================================== */
void WOK_Seq_RunStateMachine(void)
{
    WOK_Seq_CheckSafetyInhibits();
    if (WOK_Seq_AppData.FaultMask != WOK_FAULT_NONE &&
        WOK_Seq_AppData.State != WOK_STATE_SAFE &&
        WOK_Seq_AppData.State != WOK_STATE_IDLE)
    {
        WOK_Seq_SendNavStop();
        WOK_Seq_SendGeoStow();
        WOK_Seq_SendSeisStop();
        WOK_Seq_SendDrillEstop();
        WOK_Seq_ChangeState(WOK_STATE_SAFE);
        CFE_EVS_SendEvent(WOK_SEQ_SAFE_ENTRY_EID, CFE_EVS_EventType_CRITICAL,
                          "WOK_SEQ: Fault 0x%04X — SAFE", WOK_Seq_AppData.FaultMask);
        return;
    }

    switch (WOK_Seq_AppData.State)
    {
        case WOK_STATE_IDLE:
            break;

        /* ── MOVE ─────────────────────────────────────────────────── */
        case WOK_STATE_MOVE:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState = false;
                if (WOK_Seq_AppData.WaypointIdx < WOK_Seq_AppData.TotalWaypoints)
                {
                    WOK_Waypoint_t *wp = &WOK_Seq_AppData.Waypoints[WOK_Seq_AppData.WaypointIdx];
                    WOK_Seq_SendNavGoTo(wp->DeltaForwardM, wp->DeltaLateralM);
                }
            }
            if (WOK_Seq_AppData.NavStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                WOK_Seq_AppData.NavStatus.ArrivedFlag)
            {
                if (WOK_Seq_AppData.ReturnPhase == 1)
                {
                    /* Arrived back at the selected site — drill directly
                     * (no geophone deploy needed), stamped with the
                     * site's recorded pose */
                    uint16 ti = WOK_Seq_AppData.ReturnTargetIdx;
                    WOK_Seq_AppData.SiteX_m     = WOK_Seq_AppData.SiteRecX[ti];
                    WOK_Seq_AppData.SiteY_m     = WOK_Seq_AppData.SiteRecY[ti];
                    WOK_Seq_AppData.ReturnPhase = 2;
                    WOK_Seq_ChangeState(WOK_STATE_DRILL);
                }
                else if (WOK_Seq_AppData.WaypointIdx < WOK_Seq_AppData.TotalWaypoints &&
                         WOK_Seq_AppData.Waypoints[WOK_Seq_AppData.WaypointIdx].RecalFlag)
                {
                    /* Recal-flagged waypoint: hold for ground-in-the-loop
                     * correction (SET_POSE/REALIGN) before the site work */
                    WOK_Seq_ChangeState(WOK_STATE_HOLD);
                }
                else
                {
                    WOK_Seq_ChangeState(WOK_STATE_DEPLOY);
                }
            }
            break;

        /* ── HOLD — ground-recalibration hold (recal-flagged waypoint) */
        /* Parked at the waypoint before DEPLOY: ground measures the    */
        /* pose/heading error externally and corrects it with NAV       */
        /* SET_POSE / REALIGN, then releases. The wall-clock timeout    */
        /* guarantees an unattended mission proceeds on its own.       ── */
        case WOK_STATE_HOLD:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState  = false;
                WOK_Seq_AppData.HoldRelease   = 0;
                WOK_Seq_AppData.HoldEntryTime = CFE_TIME_GetTime();
                CFE_EVS_SendEvent(WOK_SEQ_HOLD_ENTRY_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEQ: Holding at WP %u for ground recalibration "
                                  "(RELEASE to proceed; auto-proceed in %us)",
                                  WOK_Seq_AppData.WaypointIdx,
                                  WOK_SEQ_HOLD_TIMEOUT_SEC);
            }
            if (WOK_Seq_AppData.HoldRelease)
            {
                WOK_Seq_AppData.HoldRelease = 0;
                CFE_EVS_SendEvent(WOK_SEQ_HOLD_RELEASED_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEQ: Hold released at WP %u — proceeding",
                                  WOK_Seq_AppData.WaypointIdx);
                WOK_Seq_ChangeState(WOK_STATE_DEPLOY);
            }
            else
            {
                CFE_TIME_SysTime_t held =
                    CFE_TIME_Subtract(CFE_TIME_GetTime(),
                                      WOK_Seq_AppData.HoldEntryTime);
                if (held.Seconds > WOK_SEQ_HOLD_TIMEOUT_SEC)
                {
                    CFE_EVS_SendEvent(WOK_SEQ_HOLD_TIMEOUT_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEQ: Hold timeout at WP %u (%us) — proceeding unattended",
                                      WOK_Seq_AppData.WaypointIdx,
                                      WOK_SEQ_HOLD_TIMEOUT_SEC);
                    WOK_Seq_ChangeState(WOK_STATE_DEPLOY);
                }
            }
            break;

        /* ── DEPLOY ───────────────────────────────────────────────── */
        /* Fixed-array mode: one GEO deploy command (legacy).           */
        /* Per-pod mode (CP-3): the placement sub-sequence — drive to   */
        /* each pod position (far pod first), place it (GEO owns the    */
        /* seat/proxy/reseat loop internally), record the placed pose,  */
        /* then return to the shot point and hand SEIS the actual       */
        /* geometry before the first shot.                              */
        case WOK_STATE_DEPLOY:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState = false;
                /* Rover is parked at the site — snapshot NAV pose as the
                 * survey geometry for this site's seismic + drill records */
                WOK_Seq_AppData.SiteX_m        = WOK_Seq_AppData.NavStatus.PoseX_m;
                WOK_Seq_AppData.SiteY_m        = WOK_Seq_AppData.NavStatus.PoseY_m;
                WOK_Seq_AppData.SiteHeadingDeg = WOK_Seq_AppData.NavStatus.HeadingDriftDeg;
                WOK_Seq_AppData.StrikesDone    = 0;
                /* Fresh site = fresh plate seating campaign */
                WOK_Seq_AppData.PlateReseatsSite = 0;
                WOK_Seq_AppData.AcceptDegraded   = 0;
                CFE_EVS_SendEvent(WOK_SEQ_SITE_GEOM_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEQ: Site %u geometry — (%.2f, %.2f) hdg %.1f",
                                  WOK_Seq_AppData.WaypointIdx,
                                  (double)WOK_Seq_AppData.SiteX_m,
                                  (double)WOK_Seq_AppData.SiteY_m,
                                  (double)WOK_Seq_AppData.SiteHeadingDeg);
                if (WOK_Seq_AppData.PlacementMode == WOK_SEQ_PLACEMENT_PER_POD)
                {
                    float tx, ty;
                    WOK_Seq_AppData.PlacePodIdx   = WOK_HAL_GEO_MAX_PODS - 1;
                    WOK_Seq_AppData.PlaceSubState = WOK_PLACE_SUB_MOVE;
                    WOK_Seq_PodTarget((uint8)WOK_Seq_AppData.PlacePodIdx, &tx, &ty);
                    WOK_Seq_NavGoToWorld(tx, ty);
                }
                else
                {
                    WOK_Seq_SendGeoDeploy();
                }
            }
            if (WOK_Seq_AppData.PlacementMode == WOK_SEQ_PLACEMENT_PER_POD)
            {
                uint8 p = (uint8)WOK_Seq_AppData.PlacePodIdx;
                switch (WOK_Seq_AppData.PlaceSubState)
                {
                    case WOK_PLACE_SUB_MOVE:
                        if (WOK_Seq_NavArrived())
                        {
                            WOK_Seq_AppData.PodPlacedX[p] = WOK_Seq_AppData.NavStatus.PoseX_m;
                            WOK_Seq_AppData.PodPlacedY[p] = WOK_Seq_AppData.NavStatus.PoseY_m;
                            WOK_Seq_SendGeoPlacePod(p,
                                                    WOK_Seq_AppData.PodPlacedX[p],
                                                    WOK_Seq_AppData.PodPlacedY[p]);
                            WOK_Seq_AppData.PlaceSubState = WOK_PLACE_SUB_POD;
                        }
                        break;
                    case WOK_PLACE_SUB_POD:
                        if (WOK_Seq_AppData.GeoStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                            (WOK_Seq_AppData.GeoStatus.PodState[p] == WOK_GEO_POD_PLACED_OK ||
                             WOK_Seq_AppData.GeoStatus.PodState[p] == WOK_GEO_POD_PLACED_MARGINAL))
                        {
                            if (p > 0)
                            {
                                float tx, ty;
                                WOK_Seq_AppData.PlacePodIdx--;
                                WOK_Seq_AppData.PlaceSubState = WOK_PLACE_SUB_MOVE;
                                WOK_Seq_PodTarget((uint8)WOK_Seq_AppData.PlacePodIdx, &tx, &ty);
                                WOK_Seq_NavGoToWorld(tx, ty);
                            }
                            else
                            {
                                WOK_Seq_AppData.PlaceSubState = WOK_PLACE_SUB_RETURN;
                                WOK_Seq_NavGoToWorld(WOK_Seq_AppData.SiteX_m,
                                                     WOK_Seq_AppData.SiteY_m);
                            }
                        }
                        break;
                    case WOK_PLACE_SUB_RETURN:
                        if (WOK_Seq_NavArrived())
                        {
                            WOK_Seq_SendSeisPodGeometry();
                            {
                                unsigned marg = 0;
                                for (uint8 b = 0; b < WOK_HAL_GEO_MAX_PODS; b++)
                                    if (WOK_Seq_AppData.GeoStatus.MarginalMask & (1u << b))
                                        marg++;
                                CFE_EVS_SendEvent(WOK_SEQ_ARRAY_PLACED_EID,
                                                  CFE_EVS_EventType_INFORMATION,
                                                  "WOK_SEQ: Array placed at WP %u — %u marginal pod(s)",
                                                  WOK_Seq_AppData.WaypointIdx, marg);
                            }
                            WOK_Seq_ChangeState(WOK_STATE_VERIFY);
                        }
                        break;
                    default: break;
                }
            }
            else if (WOK_Seq_AppData.GeoStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                     WOK_Seq_AppData.GeoStatus.MechState == WOK_GEO_DEPLOYED)
            {
                WOK_Seq_ChangeState(WOK_STATE_THUMP);
            }
            break;

        /* ── THUMP — [config source] → start acq → reset → arm →    */
        /*    fire (the shot lands INSIDE the record window, giving  */
        /*    a valid t0). In sweep mode the source reports FIRED    */
        /*    only after the pilot completes.                       ── */
        case WOK_STATE_THUMP:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState   = false;
                WOK_Seq_AppData.ThumpTickCount = 0;
                WOK_Seq_AppData.ThumpEntryTime = CFE_TIME_GetTime();
                /* Per-site source select — sent only when the waypoint
                 * deviates from the impulse defaults, so a default
                 * mission's command/SeqId stream is untouched. */
                if (WOK_Seq_AppData.WaypointIdx < WOK_Seq_AppData.TotalWaypoints)
                {
                    WOK_Waypoint_t *wp =
                        &WOK_Seq_AppData.Waypoints[WOK_Seq_AppData.WaypointIdx];
                    if (wp->SourceMode != 0 || wp->SourceEnergyJ > 0.0f)
                        WOK_Seq_SendSeisConfigSource(wp->SourceMode,
                                                     wp->SourceEnergyJ);
                }
                /* Pre-trigger: recording must be running when the source
                 * fires, or the shot never reaches the trace. */
                WOK_Seq_AppData.SeisStatus.Acquiring = 1;
                WOK_Seq_AppData.SeisStatus.SamplesAcquired_k = 0;
                WOK_Seq_SendSeisStart(false);
                /* Reset source first to ensure clean IDLE state */
                WOK_Seq_SendThumpReset();
                WOK_Seq_SendThumpArm();
            }

            WOK_Seq_AppData.ThumpTickCount++;

            if (WOK_Seq_AppData.SeisStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                WOK_Seq_AppData.SeisStatus.SourceState == WOK_HAL_SRC_ARMED)
            {
                WOK_Seq_SendThumpFire();
            }
            else if (WOK_Seq_AppData.SeisStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                     WOK_Seq_AppData.SeisStatus.SourceState == WOK_HAL_SRC_FIRED)
            {
                CFE_EVS_SendEvent(WOK_SEQ_SOURCE_FIRED_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEQ: Source fired — %.1fN at WP %u",
                                  (double)WOK_Seq_AppData.SeisStatus.SourceForce_N,
                                  WOK_Seq_AppData.WaypointIdx);
                WOK_Seq_SendThumpReset();
                WOK_Seq_ChangeState(WOK_STATE_RECORD);
            }
            else
            {
                /* Wall-clock timeout: SEND_HK tick rate is platform-
                 * dependent, so ticks cannot be used as seconds. The
                 * budget must cover the worst-case bank charge plus a
                 * full sweep (WOK_SEQ_SOURCE_TIMEOUT_SEC) — a bank that
                 * cannot charge (sagging bus) parks here, times out,
                 * and the site records without a source. */
                CFE_TIME_SysTime_t elapsed =
                    CFE_TIME_Subtract(CFE_TIME_GetTime(),
                                      WOK_Seq_AppData.ThumpEntryTime);
                if (elapsed.Seconds > WOK_SEQ_SOURCE_TIMEOUT_SEC)
                {
                    CFE_EVS_SendEvent(WOK_SEQ_SOURCE_TIMEOUT_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEQ: Source timeout (%lus, %u ticks) — recording without source",
                                      (unsigned long)elapsed.Seconds,
                                      WOK_Seq_AppData.ThumpTickCount);
                    WOK_Seq_SendThumpReset();
                    WOK_Seq_ChangeState(WOK_STATE_RECORD);
                }
            }
            break;

        /* ── RECORD — acq started on THUMP entry; wait for the rest  */
        /*    of the window to complete after the strike             ── */
        case WOK_STATE_RECORD:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState = false;
            }
            if (WOK_Seq_AppData.SeisStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                !WOK_Seq_AppData.SeisStatus.Acquiring &&
                WOK_Seq_AppData.SeisStatus.SamplesAcquired_k > 0)
            {
                /* Plate verification (CP-2): SEIS kept a failing record
                 * out of the stack — reseat the plate and re-shoot the
                 * same StackIdx, bounded per site; on exhaustion accept
                 * degraded so the site proceeds flagged, not stalled.
                 * (PlateOk is always 1 for sweep shots and unevaluated
                 * records, so this branch is impulse-only by design.) */
                if (WOK_Seq_AppData.SeisStatus.PlateOk == 0 &&
                    !WOK_Seq_AppData.AcceptDegraded)
                {
                    /* CP-4 disambiguation: the plate ratio's reference IS
                     * the near pod. If the test shot already showed that
                     * pod's coupling is bad, the low ratio is pod-side —
                     * plate reseats can't fix it, so don't burn them. */
                    if (WOK_Seq_AppData.PlacementMode == WOK_SEQ_PLACEMENT_PER_POD &&
                        WOK_Seq_AppData.SeisStatus.PodScoreValid &&
                        !(WOK_Seq_AppData.SeisStatus.PodScoreOkMask & 0x01))
                    {
                        WOK_Seq_AppData.AcceptDegraded = 1;
                        WOK_Seq_AppData.SiteDegraded[WOK_Seq_AppData.WaypointIdx] = 1;
                        CFE_EVS_SendEvent(WOK_SEQ_PLATE_PODSIDE_EID, CFE_EVS_EventType_ERROR,
                                          "WOK_SEQ: Plate ratio low at WP %u but near pod scored %.2f — pod-side, accepting degraded without plate reseats",
                                          WOK_Seq_AppData.WaypointIdx,
                                          (double)WOK_Seq_AppData.SeisStatus.PodScore[0]);
                        WOK_Seq_ChangeState(WOK_STATE_THUMP);
                        break;
                    }
                    if (WOK_Seq_AppData.PlateReseatsSite < WOK_SEIS_MAX_PLATE_RESEAT)
                    {
                        WOK_Seq_AppData.PlateReseatsSite++;
                        WOK_Seq_SendSeisPlateReseat();
                        CFE_EVS_SendEvent(WOK_SEQ_PLATE_RESEAT_EID,
                                          CFE_EVS_EventType_INFORMATION,
                                          "WOK_SEQ: Plate coupling low at WP %u (ratio %.1f) — reseat %u/%u, re-shot",
                                          WOK_Seq_AppData.WaypointIdx,
                                          (double)WOK_Seq_AppData.SeisStatus.PlateRatio,
                                          WOK_Seq_AppData.PlateReseatsSite,
                                          WOK_SEIS_MAX_PLATE_RESEAT);
                    }
                    else
                    {
                        WOK_Seq_AppData.AcceptDegraded = 1;
                        WOK_Seq_AppData.SiteDegraded[WOK_Seq_AppData.WaypointIdx] = 1;
                        CFE_EVS_SendEvent(WOK_SEQ_PLATE_EXHAUSTED_EID, CFE_EVS_EventType_ERROR,
                                          "WOK_SEQ: Plate reseat budget exhausted at WP %u — proceeding DEGRADED",
                                          WOK_Seq_AppData.WaypointIdx);
                    }
                    /* Re-shoot the same strike (StrikesDone unchanged) */
                    WOK_Seq_ChangeState(WOK_STATE_THUMP);
                    break;
                }

                WOK_Seq_AppData.StrikesDone++;
                CFE_EVS_SendEvent(WOK_SEQ_RECORD_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEQ: Record %u/%u at WP %u (%uk samples, strike@%lu%s)",
                                  WOK_Seq_AppData.StrikesDone, WOK_SEIS_STACK_TARGET,
                                  WOK_Seq_AppData.WaypointIdx,
                                  WOK_Seq_AppData.SeisStatus.SamplesAcquired_k,
                                  (unsigned long)WOK_Seq_AppData.SeisStatus.StrikeSampleIdx,
                                  WOK_Seq_AppData.SeisStatus.StrikeValid ? "" : " MISSING");
                if (WOK_Seq_AppData.StrikesDone < WOK_SEIS_STACK_TARGET)
                {
                    /* More strikes to stack at this site */
                    WOK_Seq_ChangeState(WOK_STATE_THUMP);
                }
                else
                {
                    WOK_Seq_ChangeState(WOK_STATE_EVALUATE);
                }
            }
            break;

        /* ── EVALUATE ─────────────────────────────────────────────── */
        case WOK_STATE_EVALUATE:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState   = false;
                WOK_Seq_AppData.EvalFlag       = 0;
                WOK_Seq_AppData.EvalConfidence = 0.0f;
                /* Clear stale eval data before requesting */
                WOK_Seq_AppData.SeisStatus.EvalFlag = 0;
                WOK_Seq_AppData.SeisStatus.EvalConfidence = 0.0f;
                WOK_Seq_SendSeisEvaluate();
            }
            /* Wait for SEIS to populate eval results */
            if (WOK_Seq_AppData.SeisStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                WOK_Seq_AppData.SeisStatus.EvalConfidence > 0.01f)
            {
                WOK_Seq_AppData.EvalFlag       = WOK_Seq_AppData.SeisStatus.EvalFlag;
                WOK_Seq_AppData.EvalConfidence = WOK_Seq_AppData.SeisStatus.EvalConfidence;

                /* Survey record: score (ice posterior when the pick-based
                 * evaluate ran, confidence otherwise) + absolute site pose.
                 * This is what score-then-drill and the tie-point gate
                 * consume. */
                {
                    uint16 wi = WOK_Seq_AppData.WaypointIdx;
                    float score = WOK_Seq_AppData.SeisStatus.EvalIceFrac;
                    if (score < 0.0f) score = WOK_Seq_AppData.EvalConfidence;
                    WOK_Seq_AppData.SiteScore[wi] = score;
                    WOK_Seq_AppData.SiteRecX[wi]  = WOK_Seq_AppData.SiteX_m;
                    WOK_Seq_AppData.SiteRecY[wi]  = WOK_Seq_AppData.SiteY_m;
                }

                WOK_Waypoint_t *wp = &WOK_Seq_AppData.Waypoints[WOK_Seq_AppData.WaypointIdx];

                if (WOK_Seq_AppData.MissionMode == WOK_SEQ_MODE_SURVEY_FIRST)
                {
                    /* Score now, drill the best site after the loop */
                    WOK_Seq_ChangeState(WOK_STATE_STOW);
                }
                else if (WOK_Seq_AppData.EvalFlag && wp->DrillFlag)
                {
                    if (WOK_Seq_AppData.HkHealth.BatteryV > WOK_DRILL_MIN_BATTERY_V)
                    {
                        CFE_EVS_SendEvent(WOK_SEQ_ICE_DRILL_EID, CFE_EVS_EventType_INFORMATION,
                                          "WOK_SEQ: Ice signature (%.0f%% conf) — DRILLING at WP %u",
                                          (double)(WOK_Seq_AppData.EvalConfidence * 100.0f),
                                          WOK_Seq_AppData.WaypointIdx);
                        WOK_Seq_ChangeState(WOK_STATE_DRILL);
                    }
                    else
                    {
                        CFE_EVS_SendEvent(WOK_SEQ_DRILL_BATT_SKIP_EID, CFE_EVS_EventType_ERROR,
                                          "WOK_SEQ: Ice flagged but battery %.1fV < %.1fV — skip drill",
                                          (double)WOK_Seq_AppData.HkHealth.BatteryV,
                                          (double)WOK_DRILL_MIN_BATTERY_V);
                        WOK_Seq_ChangeState(WOK_STATE_STOW);
                    }
                }
                else
                {
                    if (WOK_Seq_AppData.EvalFlag && !wp->DrillFlag)
                    {
                        CFE_EVS_SendEvent(WOK_SEQ_ICE_NO_DRILL_EID, CFE_EVS_EventType_INFORMATION,
                                          "WOK_SEQ: Ice (%.0f%%) at WP %u — not drill-flagged",
                                          (double)(WOK_Seq_AppData.EvalConfidence * 100.0f),
                                          WOK_Seq_AppData.WaypointIdx);
                    }
                    WOK_Seq_ChangeState(WOK_STATE_STOW);
                }
            }
            break;

        /* ── DRILL ────────────────────────────────────────────────── */
        case WOK_STATE_DRILL:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState  = false;
                WOK_Seq_AppData.DrillSubState = WOK_DRILL_SUB_DEPLOY;
                WOK_Seq_SendDrillDeploy();
            }
            if (WOK_Seq_AppData.DrillHk.FaultBitmask != 0)
            {
                CFE_EVS_SendEvent(WOK_SEQ_DRILL_FAULT_EID, CFE_EVS_EventType_ERROR,
                                  "WOK_SEQ: Drill fault 0x%04X — e-stop + SAFE",
                                  WOK_Seq_AppData.DrillHk.FaultBitmask);
                WOK_Seq_SendDrillEstop();
                WOK_Seq_ChangeState(WOK_STATE_SAFE);
                break;
            }
            switch (WOK_Seq_AppData.DrillSubState)
            {
                case WOK_DRILL_SUB_DEPLOY:
                    if (WOK_Seq_AppData.DrillHk.SeqId == WOK_Seq_AppData.PendingSeqId &&
                        WOK_Seq_AppData.DrillHk.AppState == WOK_DRILL_STATE_ARM_READY)
                    {
                        WOK_Seq_AppData.DrillSubState = WOK_DRILL_SUB_DRILLING;
                        WOK_Seq_SendDrillStart();
                    }
                    break;
                case WOK_DRILL_SUB_DRILLING:
                    if (WOK_Seq_AppData.DrillHk.SeqId == WOK_Seq_AppData.PendingSeqId &&
                        WOK_Seq_AppData.DrillHk.AppState == WOK_DRILL_STATE_DRILL_DONE)
                    {
                        WOK_Seq_AppData.DrillSubState = WOK_DRILL_SUB_SAMPLING;
                        WOK_Seq_SendDrillSample();
                    }
                    break;
                case WOK_DRILL_SUB_SAMPLING:
                    if (WOK_Seq_AppData.DrillHk.SeqId == WOK_Seq_AppData.PendingSeqId &&
                        WOK_Seq_AppData.DrillHk.AppState == WOK_DRILL_STATE_SAMPLE_DONE)
                    {
                        WOK_Seq_AppData.DrillSubState = WOK_DRILL_SUB_STOW;
                        WOK_Seq_SendDrillStow();
                    }
                    break;
                case WOK_DRILL_SUB_STOW:
                    if (WOK_Seq_AppData.DrillHk.SeqId == WOK_Seq_AppData.PendingSeqId &&
                        WOK_Seq_AppData.DrillHk.AppState == WOK_DRILL_STATE_IDLE)
                    {
                        WOK_Seq_AppData.DrillsCompleted++;
                        CFE_EVS_SendEvent(WOK_SEQ_DRILL_DONE_EID, CFE_EVS_EventType_INFORMATION,
                                          "WOK_SEQ: Drill cycle complete at WP %u (%u total)",
                                          WOK_Seq_AppData.ReturnPhase == 2
                                              ? WOK_Seq_AppData.ReturnTargetIdx
                                              : WOK_Seq_AppData.WaypointIdx,
                                          WOK_Seq_AppData.DrillsCompleted);
                        if (WOK_Seq_AppData.ReturnPhase == 2)
                        {
                            /* Targeted return-drill done — mission complete */
                            WOK_Seq_AppData.ReturnPhase = 0;
                            CFE_EVS_SendEvent(WOK_SEQ_MISSION_COMPLETE_EID,
                                              CFE_EVS_EventType_INFORMATION,
                                              "WOK_SEQ: Tie-point gate satisfied — mission complete");
                            WOK_Seq_ChangeState(WOK_STATE_DONE);
                        }
                        else
                        {
                            WOK_Seq_ChangeState(WOK_STATE_STOW);
                        }
                    }
                    break;
                default: break;
            }
            break;

        /* ── VERIFY (CP-4, placement mode) ────────────────────────── */
        /* One low-energy test shot scores every pod at once (the       */
        /* plate factor is common-mode and cancels — this is the pure   */
        /* pod-side measurement the seating-force proxy and the plate   */
        /* metric can't give). Failing pods get a targeted drive-back   */
        /* reseat and a re-verification shot, on a bounded budget;      */
        /* then the partial-array policy applies and the stack shots    */
        /* proceed at the restored source configuration.                */
        case WOK_STATE_VERIFY:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState      = false;
                WOK_Seq_AppData.VerifyReseatsUsed = 0;
                WOK_Seq_AppData.VerifyTriedMask   = 0;
                WOK_Seq_StartTestShot();
            }
            switch (WOK_Seq_AppData.VerifySub)
            {
                case WOK_VERIFY_SUB_SHOT:
                    if (WOK_Seq_AppData.SeisStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                        WOK_Seq_AppData.SeisStatus.SourceState == WOK_HAL_SRC_ARMED)
                    {
                        WOK_Seq_SendThumpFire();
                    }
                    else if (WOK_Seq_AppData.SeisStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                             WOK_Seq_AppData.SeisStatus.SourceState == WOK_HAL_SRC_FIRED)
                    {
                        WOK_Seq_SendThumpReset();
                        WOK_Seq_AppData.VerifySub = WOK_VERIFY_SUB_WAIT;
                    }
                    else
                    {
                        CFE_TIME_SysTime_t el =
                            CFE_TIME_Subtract(CFE_TIME_GetTime(),
                                              WOK_Seq_AppData.ThumpEntryTime);
                        if (el.Seconds > WOK_SEQ_SOURCE_TIMEOUT_SEC)
                        {
                            CFE_EVS_SendEvent(WOK_SEQ_TEST_SHOT_TIMEOUT_EID, CFE_EVS_EventType_ERROR,
                                              "WOK_SEQ: Test shot timeout — skipping verification at WP %u",
                                              WOK_Seq_AppData.WaypointIdx);
                            WOK_Seq_SendThumpReset();
                            WOK_Seq_SendSeisConfigSource(WOK_HAL_SRC_MODE_IMPULSE,
                                                         WOK_SRC_DEFAULT_ENERGY_J);
                            WOK_Seq_ChangeState(WOK_STATE_THUMP);
                        }
                    }
                    break;

                case WOK_VERIFY_SUB_WAIT:
                    if (WOK_Seq_AppData.SeisStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                        !WOK_Seq_AppData.SeisStatus.Acquiring &&
                        WOK_Seq_AppData.SeisStatus.SamplesAcquired_k > 0 &&
                        WOK_Seq_AppData.SeisStatus.PodScoreValid)
                    {
                        uint8 failing = (uint8)(~WOK_Seq_AppData.SeisStatus.PodScoreOkMask) & 0x0F;
                        uint8 cand    = failing & (uint8)~WOK_Seq_AppData.VerifyTriedMask;

                        if (cand != 0 &&
                            WOK_Seq_AppData.VerifyReseatsUsed < WOK_SEQ_VERIFY_RESEAT_BUDGET)
                        {
                            /* Target the worst-scoring untried pod */
                            uint8 p = 0xFF;
                            float worst = 1e30f;
                            for (uint8 b = 0; b < WOK_HAL_GEO_MAX_PODS; b++)
                                if ((cand & (1u << b)) &&
                                    WOK_Seq_AppData.SeisStatus.PodScore[b] < worst)
                                {
                                    worst = WOK_Seq_AppData.SeisStatus.PodScore[b];
                                    p = b;
                                }
                            WOK_Seq_AppData.VerifyPod         = p;
                            WOK_Seq_AppData.VerifyTriedMask  |= (uint8)(1u << p);
                            WOK_Seq_AppData.VerifyReseatsUsed++;
                            CFE_EVS_SendEvent(WOK_SEQ_POD_RESEAT_EID,
                                              CFE_EVS_EventType_INFORMATION,
                                              "WOK_SEQ: Pod %u score %.2f below %.2f — targeted reseat (%u/%u)",
                                              p, (double)worst,
                                              (double)WOK_SEIS_POD_SCORE_MIN,
                                              WOK_Seq_AppData.VerifyReseatsUsed,
                                              WOK_SEQ_VERIFY_RESEAT_BUDGET);
                            WOK_Seq_AppData.VerifySub = WOK_VERIFY_SUB_MOVE;
                            WOK_Seq_NavGoToWorld(WOK_Seq_AppData.PodPlacedX[p],
                                                 WOK_Seq_AppData.PodPlacedY[p]);
                        }
                        else
                        {
                            uint8 verified = 0;
                            for (uint8 b = 0; b < WOK_HAL_GEO_MAX_PODS; b++)
                                if (WOK_Seq_AppData.SeisStatus.PodScoreOkMask & (1u << b))
                                    verified++;
                            if (verified < WOK_HAL_GEO_MAX_PODS)
                                WOK_Seq_AppData.SiteDegraded[WOK_Seq_AppData.WaypointIdx] = 1;
                            if (verified < WOK_SEQ_MIN_VERIFIED_PODS)
                                CFE_EVS_SendEvent(WOK_SEQ_PARTIAL_ARRAY_EID,
                                                  CFE_EVS_EventType_CRITICAL,
                                                  "WOK_SEQ: Only %u/%u pods verified at WP %u — below partial-array minimum, proceeding FLAGGED",
                                                  verified, WOK_HAL_GEO_MAX_PODS,
                                                  WOK_Seq_AppData.WaypointIdx);
                            else
                                CFE_EVS_SendEvent(WOK_SEQ_ARRAY_VERIFIED_EID,
                                                  CFE_EVS_EventType_INFORMATION,
                                                  "WOK_SEQ: Array verified %u/%u at WP %u%s",
                                                  verified, WOK_HAL_GEO_MAX_PODS,
                                                  WOK_Seq_AppData.WaypointIdx,
                                                  verified < WOK_HAL_GEO_MAX_PODS
                                                      ? " (degraded)" : "");
                            /* Restore the stack-shot source config (the
                             * waypoint's own override, if any, re-sends
                             * at THUMP entry) */
                            WOK_Seq_SendSeisConfigSource(WOK_HAL_SRC_MODE_IMPULSE,
                                                         WOK_SRC_DEFAULT_ENERGY_J);
                            WOK_Seq_ChangeState(WOK_STATE_THUMP);
                        }
                    }
                    break;

                case WOK_VERIFY_SUB_MOVE:
                    if (WOK_Seq_NavArrived())
                    {
                        WOK_Seq_SendGeoReseatPod(WOK_Seq_AppData.VerifyPod);
                        WOK_Seq_AppData.VerifySub = WOK_VERIFY_SUB_POD;
                    }
                    break;

                case WOK_VERIFY_SUB_POD:
                    if (WOK_Seq_AppData.GeoStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                        (WOK_Seq_AppData.GeoStatus.PodState[WOK_Seq_AppData.VerifyPod]
                             == WOK_GEO_POD_PLACED_OK ||
                         WOK_Seq_AppData.GeoStatus.PodState[WOK_Seq_AppData.VerifyPod]
                             == WOK_GEO_POD_PLACED_MARGINAL))
                    {
                        WOK_Seq_AppData.VerifySub = WOK_VERIFY_SUB_RETURN;
                        WOK_Seq_NavGoToWorld(WOK_Seq_AppData.SiteX_m,
                                             WOK_Seq_AppData.SiteY_m);
                    }
                    break;

                case WOK_VERIFY_SUB_RETURN:
                    if (WOK_Seq_NavArrived())
                    {
                        /* Seat attempts changed — refresh SEIS geometry,
                         * then re-verify with another test shot */
                        WOK_Seq_SendSeisPodGeometry();
                        WOK_Seq_StartTestShot();
                    }
                    break;

                default: break;
            }
            break;

        /* ── STOW ─────────────────────────────────────────────────── */
        /* Fixed-array mode: one GEO stow command (legacy). Per-pod     */
        /* mode: the retrieval mirror — drive to each placed pod (near  */
        /* first), recapture it, and only advance the waypoint once     */
        /* every pod reads CARRIED again and the rover is back at the   */
        /* shot point (CP-6: recapture confirmed before departure).     */
        case WOK_STATE_STOW:
            if (WOK_Seq_AppData.EnteredState)
            {
                WOK_Seq_AppData.EnteredState = false;
                if (WOK_Seq_AppData.PlacementMode == WOK_SEQ_PLACEMENT_PER_POD)
                {
                    WOK_Seq_AppData.PlacePodIdx   = 0;
                    WOK_Seq_AppData.PlaceSubState = WOK_PLACE_SUB_MOVE;
                    WOK_Seq_NavGoToWorld(WOK_Seq_AppData.PodPlacedX[0],
                                         WOK_Seq_AppData.PodPlacedY[0]);
                }
                else
                {
                    WOK_Seq_SendGeoStow();
                }
            }
            if (WOK_Seq_AppData.PlacementMode == WOK_SEQ_PLACEMENT_PER_POD)
            {
                uint8 p = (uint8)WOK_Seq_AppData.PlacePodIdx;
                switch (WOK_Seq_AppData.PlaceSubState)
                {
                    case WOK_PLACE_SUB_MOVE:
                        if (WOK_Seq_NavArrived())
                        {
                            WOK_Seq_SendGeoRetrievePod(p);
                            WOK_Seq_AppData.PlaceSubState = WOK_PLACE_SUB_POD;
                        }
                        break;
                    case WOK_PLACE_SUB_POD:
                        if (WOK_Seq_AppData.GeoStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                            WOK_Seq_AppData.GeoStatus.PodState[p] == WOK_GEO_POD_CARRIED)
                        {
                            if (p + 1 < WOK_HAL_GEO_MAX_PODS)
                            {
                                WOK_Seq_AppData.PlacePodIdx++;
                                WOK_Seq_AppData.PlaceSubState = WOK_PLACE_SUB_MOVE;
                                WOK_Seq_NavGoToWorld(
                                    WOK_Seq_AppData.PodPlacedX[WOK_Seq_AppData.PlacePodIdx],
                                    WOK_Seq_AppData.PodPlacedY[WOK_Seq_AppData.PlacePodIdx]);
                            }
                            else
                            {
                                WOK_Seq_AppData.PlaceSubState = WOK_PLACE_SUB_RETURN;
                                WOK_Seq_NavGoToWorld(WOK_Seq_AppData.SiteX_m,
                                                     WOK_Seq_AppData.SiteY_m);
                            }
                        }
                        break;
                    case WOK_PLACE_SUB_RETURN:
                        if (WOK_Seq_NavArrived())
                        {
                            CFE_EVS_SendEvent(WOK_SEQ_ARRAY_RECAPTURED_EID,
                                              CFE_EVS_EventType_INFORMATION,
                                              "WOK_SEQ: Array recaptured at WP %u",
                                              WOK_Seq_AppData.WaypointIdx);
                            WOK_Seq_AdvanceWaypoint();
                        }
                        break;
                    default: break;
                }
            }
            else if (WOK_Seq_AppData.GeoStatus.SeqId == WOK_Seq_AppData.PendingSeqId &&
                     WOK_Seq_AppData.GeoStatus.MechState == WOK_GEO_STOWED)
            {
                WOK_Seq_AdvanceWaypoint();
            }
            break;

        case WOK_STATE_SAFE:
            break;

        case WOK_STATE_DONE:
            break;

        default:
            break;
    }
}

/* ================================================================== */
/*  Mission-end decision — the tie-point mission-success gate          */
/*                                                                    */
/*  The whole prospecting case rests on at least one drill tie inside */
/*  the seismic footprint (it calibrates the Vp→ice transfer          */
/*  function; even a dry tie anchors its dry end). After the last     */
/*  waypoint:                                                         */
/*    - SURVEY_FIRST mode: return to the argmax-score site and drill. */
/*    - Any mode with zero drills completed: return to the argmax     */
/*      site and drill regardless of score.                           */
/*    - Otherwise: DONE.                                              */
/* ================================================================== */
void WOK_Seq_CheckMissionEnd(void)
{
    /* Argmax over the survey record */
    int    best = -1;
    float  best_score = -1.0f;
    for (uint16 i = 0; i < WOK_Seq_AppData.TotalWaypoints; i++)
    {
        if (WOK_Seq_AppData.SiteScore[i] > best_score)
        {
            best_score = WOK_Seq_AppData.SiteScore[i];
            best = i;
        }
    }

    bool want_drill =
        (WOK_Seq_AppData.MissionMode == WOK_SEQ_MODE_SURVEY_FIRST ||
         WOK_Seq_AppData.DrillsCompleted == 0);

    if (!want_drill || best < 0)
    {
        WOK_Seq_ChangeState(WOK_STATE_DONE);
        return;
    }

    if (WOK_Seq_AppData.HkHealth.BatteryV <= WOK_DRILL_MIN_BATTERY_V)
    {
        CFE_EVS_SendEvent(WOK_SEQ_TIEPOINT_UNMET_EID, CFE_EVS_EventType_CRITICAL,
                          "WOK_SEQ: Tie-point gate UNMET — battery %.1fV too low to drill",
                          (double)WOK_Seq_AppData.HkHealth.BatteryV);
        WOK_Seq_ChangeState(WOK_STATE_DONE);
        return;
    }

    /* Return to the selected site: relative step from current pose */
    float df = WOK_Seq_AppData.SiteRecX[best] - WOK_Seq_AppData.NavStatus.PoseX_m;
    float dl = WOK_Seq_AppData.SiteRecY[best] - WOK_Seq_AppData.NavStatus.PoseY_m;
    WOK_Seq_AppData.ReturnTargetIdx = (uint8)best;
    WOK_Seq_AppData.ReturnPhase     = 1;
    CFE_EVS_SendEvent(WOK_SEQ_RETURN_TARGET_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_SEQ: Targeting WP %d (score %.2f) — returning (%.1f, %.1f) to drill",
                      best, (double)best_score, (double)df, (double)dl);
    WOK_Seq_SendNavGoTo(df, dl);
    WOK_Seq_ChangeState(WOK_STATE_MOVE);
}

/* ================================================================== */
void WOK_Seq_CheckSafetyInhibits(void)
{
    WOK_Seq_AppData.FaultMask = WOK_FAULT_NONE;

    if (WOK_Seq_AppData.HkHealth.BatteryV > 0.1f &&
        WOK_Seq_AppData.HkHealth.BatteryV < WOK_Seq_AppData.BatteryMinV)
        WOK_Seq_AppData.FaultMask |= WOK_FAULT_BATTERY_LOW;

    if (WOK_Seq_AppData.HkHealth.TiltDeg > WOK_Seq_AppData.TiltMaxDeg)
        WOK_Seq_AppData.FaultMask |= WOK_FAULT_TILT_EXCEED;

    if (WOK_Seq_AppData.HkHealth.TempBoard_C > WOK_Seq_AppData.TempMaxC)
        WOK_Seq_AppData.FaultMask |= WOK_FAULT_TEMP_HIGH;

    if (WOK_Seq_AppData.HkHealth.SafeTrigger)
        WOK_Seq_AppData.FaultMask |= WOK_Seq_AppData.HkHealth.FaultMask;

    /* Aggregate child-app faults so they drive SAFE entry. NAV/GEO/SEIS
     * own per-app fault detection; SEQ only needs to react to it. The
     * DRILL state-machine already inspects DrillHk.FaultBitmask itself. */
    WOK_Seq_AppData.FaultMask |= WOK_Seq_AppData.NavStatus.FaultMask;
    WOK_Seq_AppData.FaultMask |= WOK_Seq_AppData.GeoStatus.FaultMask;
    WOK_Seq_AppData.FaultMask |= WOK_Seq_AppData.SeisStatus.FaultMask;
}

/* ================================================================== */
/*  Length check for a child app's telemetry before it is cached.     */
/*  Child TLM arrives every SEND_HK tick, so a mismatch is evented    */
/*  once (per *Reported flag) and counted thereafter.                 */
/* ================================================================== */
bool WOK_Seq_VerifyTlmLength(const CFE_SB_Buffer_t *BufPtr, size_t ExpectedLength,
                             uint8 *Reported)
{
    size_t ActualLength = 0;
    CFE_MSG_GetSize(&BufPtr->Msg, &ActualLength);
    if (ActualLength != ExpectedLength)
    {
        WOK_Seq_AppData.ErrCounter++;
        if (!*Reported)
        {
            CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;
            CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
            *Reported = 1;
            CFE_EVS_SendEvent(WOK_SEQ_TLM_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                              "WOK_SEQ: TLM len err MID 0x%04X: exp %lu got %lu (further errors counted only)",
                              (unsigned int)CFE_SB_MsgIdToValue(MsgId),
                              (unsigned long)ExpectedLength,
                              (unsigned long)ActualLength);
        }
        return false;
    }
    return true;
}

/* ================================================================== */
bool WOK_Seq_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    size_t ActualLength = 0;
    CFE_MSG_GetSize(MsgPtr, &ActualLength);
    if (ActualLength != ExpectedLength)
    {
        WOK_Seq_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_SEQ_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEQ: Len err: exp %lu got %lu",
                          (unsigned long)ExpectedLength,
                          (unsigned long)ActualLength);
        return false;
    }
    return true;
}
