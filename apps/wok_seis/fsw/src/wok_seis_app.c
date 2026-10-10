/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
* WOK_SEIS — Seismic Acquisition + EM Source + Evaluate (HAL-integrated)
*
* Drives the geophone ADC (WOK_HAL_Seis_*) and the dual-mode EM source
* (WOK_HAL_Src_*): records per-shot traces with an electrically latched
* strike t0, cross-correlates sweep records against the regenerated
* pilot (chunked across HK ticks), stacks strikes per site, picks first
* breaks, and runs the onboard two-layer refraction EVALUATE. Products
* (seis_/corr_/stack_/coup_NNNN.wok) are archived via WOK_HAL_Storage_*
* and downlinkable in chunks.
************************************************************************/
#include "wok_seis_app.h"
#include "wok_hal_seis.h"
#include "wok_hal_storage.h"
#include <math.h>

static WOK_Seis_AppData_t WOK_Seis_AppData;

/* Pod offsets — fixed deployment geometry, recorded in every trace file */
static const float WOK_Seis_PodOffsetsM[WOK_SEIS_MAX_CHANNELS] =
    WOK_SEIS_POD_OFFSETS_M_INIT;

/* STA/LTA picker tuning (samples at the configured fs) */
#define WOK_SEIS_PICK_STA_SAMPS      40     /* 10 ms @ 4 kHz            */
#define WOK_SEIS_PICK_TRIGGER_RATIO  4.0f   /* STA/LTA to declare onset */

/* Chunks streamed per HK tick — bounds SB/TO_LAB burst size */
#define WOK_SEIS_CHUNKS_PER_TICK  4

static void WOK_Seis_UpdateFromHAL(void);
static void WOK_Seis_DrainSamples(void);
static void WOK_Seis_ComputeQuicklook(void);
static void WOK_Seis_FillHeaderCommon(WOK_Seis_FileHeader_t *hdr, uint32 spc,
                                      uint32 strike_idx, uint8 strike_valid);
static void WOK_Seis_WriteTraceFile(void);
static void WOK_Seis_ComputePlateMetric(void);
static void WOK_Seis_ComputePodScores(void);
static void WOK_Seis_SyncCheck(void);
static void WOK_Seis_WriteCoupFile(void);
static void WOK_Seis_AccumulateStack(const int32 *trace, uint32 count,
                                     uint32 strike);
static void WOK_Seis_FinishSiteIfDone(void);
static void WOK_Seis_StartCorrelation(void);
static void WOK_Seis_PumpCorrelation(void);
static void WOK_Seis_WriteCorrFile(void);
static void WOK_Seis_RunPicker(void);
static void WOK_Seis_WriteStackFile(void);
static void WOK_Seis_StartDownlink(const WOK_Seis_DownlinkFileCmd_t *Cmd);
static void WOK_Seis_PumpDownlink(void);
static bool WOK_Seis_EvaluateFromPicks(uint8 *flag, float *confidence);

void WOK_Seis_AppMain(void)
{
    int32 Status; CFE_SB_Buffer_t *BufPtr = NULL;
    CFE_ES_PerfLogEntry(WOK_SEIS_PERF_ID);
    WOK_Seis_AppData.RunStatus = CFE_ES_RunStatus_APP_RUN;
    Status = WOK_Seis_AppInit();
    if (Status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("WOK_SEIS: Init failed (0x%08lX)\n", (unsigned long)Status);
        WOK_Seis_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }
    while (CFE_ES_RunLoop(&WOK_Seis_AppData.RunStatus) == true) {
        CFE_ES_PerfLogExit(WOK_SEIS_PERF_ID);
        Status = CFE_SB_ReceiveBuffer(&BufPtr, WOK_Seis_AppData.CmdPipe, CFE_SB_PEND_FOREVER);
        CFE_ES_PerfLogEntry(WOK_SEIS_PERF_ID);
        if (Status == CFE_SUCCESS) WOK_Seis_ProcessCommandPacket(BufPtr);
        else
        {
            CFE_EVS_SendEvent(WOK_SEIS_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "WOK_SEIS: SB pipe error (0x%08lX), app will exit",
                              (unsigned long)Status);
            WOK_Seis_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }
    CFE_ES_PerfLogExit(WOK_SEIS_PERF_ID);
    CFE_ES_ExitApp(WOK_Seis_AppData.RunStatus);
}

int32 WOK_Seis_AppInit(void)
{
    int32 Status;
    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;
    Status = CFE_SB_CreatePipe(&WOK_Seis_AppData.CmdPipe, WOK_SEIS_PIPE_DEPTH, WOK_SEIS_PIPE_NAME);
    if (Status != CFE_SUCCESS) return Status;

    /* Command subscription with a raised per-MsgId in-flight limit:
     * the sequencer legitimately bursts up to five SEIS commands in
     * one tick (SET_POD_GEOMETRY + CONFIG_SOURCE + START_ACQ + RESET +
     * ARM at a verification shot); the SB default limit of 4 silently
     * drops the tail of that burst ("Msg Limit Err"). */
    Status = CFE_SB_SubscribeEx(CFE_SB_ValueToMsgId(WOK_SEIS_CMD_MID),
                                WOK_Seis_AppData.CmdPipe, CFE_SB_DEFAULT_QOS, 8);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(WOK_SEIS_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Subscribe to CMD failed (0x%08lX)", (unsigned long)Status);
        return Status;
    }
    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(WOK_SEIS_SEND_HK_MID), WOK_Seis_AppData.CmdPipe);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(WOK_SEIS_SUB_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Subscribe to SEND_HK failed (0x%08lX)", (unsigned long)Status);
        return Status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seis_AppData.HkTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_TLM_MID), sizeof(WOK_Seis_HkTlm_t));
    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seis_AppData.StatusTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_STATUS_TLM_MID), sizeof(WOK_Seis_StatusTlm_t));
    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seis_AppData.QuicklookTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_QUICKLOOK_TLM_MID), sizeof(WOK_Seis_QuicklookTlm_t));
    CFE_MSG_Init(CFE_MSG_PTR(WOK_Seis_AppData.ChunkTlm.TlmHeader),
                 CFE_SB_ValueToMsgId(WOK_SEIS_FILECHUNK_TLM_MID), sizeof(WOK_Seis_FileChunkTlm_t));

    WOK_Seis_AppData.Acquiring         = 0;
    WOK_Seis_AppData.PrevAcquiring     = 0;
    WOK_Seis_AppData.Duration_sec      = WOK_SEIS_DEFAULT_DURATION_SEC;
    WOK_Seis_AppData.SampleRate_Hz     = WOK_SEIS_DEFAULT_FS_HZ;
    WOK_Seis_AppData.Gain              = WOK_SEIS_DEFAULT_GAIN;
    WOK_Seis_AppData.ChannelMask       = 0x0F;
    WOK_Seis_AppData.SamplesAcquired   = 0;
    WOK_Seis_AppData.FaultMask         = WOK_FAULT_NONE;
    WOK_Seis_AppData.CmdCounter        = 0;
    WOK_Seis_AppData.ErrCounter        = 0;
    WOK_Seis_AppData.SourceState      = 0;
    WOK_Seis_AppData.SourceForce_N    = 0.0f;
    WOK_Seis_AppData.SourceTimestamp_ms = 0;
    WOK_Seis_AppData.EvalFlag          = 0;
    WOK_Seis_AppData.EvalConfidence    = 0.0f;
    WOK_Seis_AppData.SiteX_m           = 0.0f;
    WOK_Seis_AppData.SiteY_m           = 0.0f;
    WOK_Seis_AppData.SiteHeadingDeg    = 0.0f;
    WOK_Seis_AppData.TraceCount        = 0;
    WOK_Seis_AppData.TraceNumChans     = 0;
    WOK_Seis_AppData.TraceTruncated    = 0;
    WOK_Seis_AppData.AcqIndex          = 0;
    WOK_Seis_AppData.StrikeSampleIdx   = 0;
    WOK_Seis_AppData.StrikeValid       = 0;
    WOK_Seis_AppData.StackIdx          = 0;
    WOK_Seis_AppData.StackTarget       = 1;
    WOK_Seis_AppData.StackFold         = 0;
    WOK_Seis_AppData.StackCoherence    = 0.0f;
    WOK_Seis_AppData.StackFileIndex    = 0;
    WOK_Seis_AppData.EvalV1_mps        = 0.0f;
    WOK_Seis_AppData.EvalV2_mps        = 0.0f;
    WOK_Seis_AppData.EvalDepthM        = 0.0f;
    WOK_Seis_AppData.EvalIceFrac       = -1.0f;
    for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
    {
        WOK_Seis_AppData.FirstBreakMs[ch] = -1.0f;
        WOK_Seis_AppData.PickQuality[ch]  = 0;
        WOK_Seis_AppData.ClipCount[ch]    = 0;
    }

    /* Source config defaults — preserve the drop-weight (pre-5.0)
     * source's behavior exactly */
    WOK_Seis_AppData.SourceMode    = WOK_SRC_DEFAULT_MODE;
    WOK_Seis_AppData.SourceEnergyJ = WOK_SRC_DEFAULT_ENERGY_J;
    WOK_Seis_AppData.SweepF0_Hz    = WOK_SRC_DEFAULT_SWEEP_F0_HZ;
    WOK_Seis_AppData.SweepF1_Hz    = WOK_SRC_DEFAULT_SWEEP_F1_HZ;
    WOK_Seis_AppData.SweepDur_s    = WOK_SRC_DEFAULT_SWEEP_DUR_S;
    WOK_Seis_AppData.ShotMode      = WOK_SRC_DEFAULT_MODE;
    WOK_Seis_AppData.ShotEnergyJ   = WOK_SRC_DEFAULT_ENERGY_J;
    WOK_Seis_AppData.ShotF0_Hz     = 0.0f;
    WOK_Seis_AppData.ShotF1_Hz     = 0.0f;
    WOK_Seis_AppData.ShotDur_s     = 0.0f;
    WOK_Seis_AppData.BankV         = 0.0f;
    WOK_Seis_AppData.BankSOC_pct   = 0.0f;
    WOK_Seis_AppData.SweepProgress = 0.0f;
    WOK_Seis_AppData.CorrActive    = 0;
    WOK_Seis_AppData.CorrPos       = 0;

    /* Pod geometry defaults (CP-3): static configured offsets */
    memcpy(WOK_Seis_AppData.PodOffsets, WOK_Seis_PodOffsetsM,
           sizeof(WOK_Seis_AppData.PodOffsets));
    memset(WOK_Seis_AppData.PodAttempts, 0, sizeof(WOK_Seis_AppData.PodAttempts));
    WOK_Seis_AppData.PodMarginalMask = 0;

    /* Test-shot coupling verification (CP-4) */
    WOK_Seis_AppData.TestShot       = 0;
    WOK_Seis_AppData.PodScoreOkMask = 0;
    WOK_Seis_AppData.PodScoreValid  = 0;
    WOK_Seis_AppData.TestShotCount  = 0;
    for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
        WOK_Seis_AppData.PodScore[ch] = -1.0f;

    /* Plate coupling metric (CP-2) */
    WOK_Seis_AppData.PlateRatio        = -1.0f;
    WOK_Seis_AppData.PlateBaseline     = 0.0f;
    WOK_Seis_AppData.PlateOk           = 1;
    WOK_Seis_AppData.AcceptDegraded    = 0;
    WOK_Seis_AppData.PlateDegradedSite = 0;
    WOK_Seis_AppData.PlateReseatsSite  = 0;
    WOK_Seis_AppData.PlateReseatTotal  = 0;
    WOK_Seis_AppData.AcqSiteValid      = 0;

    /* Init HAL subsystems */
    WOK_HAL_Seis_Init();
    WOK_HAL_Src_Init();
    WOK_HAL_Storage_Init();

    CFE_EVS_SendEvent(WOK_SEIS_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_SEIS: App initialised (v" WOK_VERSION_STRING ") — dual-mode EM source");
    return CFE_SUCCESS;
}

/* ================================================================== */
/*  Quicklook + QC stats from the real trace tail                     */
/* ================================================================== */
static void WOK_Seis_ComputeQuicklook(void)
{
    uint8  nch = WOK_Seis_AppData.TraceNumChans;
    uint32 n   = WOK_Seis_AppData.TraceCount;
    uint32 win = (n < 1024) ? n : 1024;
    uint32 s0  = n - win;

    if (nch == 0 || win == 0) return;

    for (uint8 slot = 0; slot < nch && slot < WOK_SEIS_MAX_CHANNELS; slot++)
    {
        double sum = 0.0, sumsq = 0.0;
        float  pk = 0.0f;
        for (uint32 s = s0; s < n; s++)
        {
            float v = (float)WOK_Seis_AppData.TraceFile.Samples[s * nch + slot];
            sum   += v;
            sumsq += (double)v * v;
            float a = v < 0.0f ? -v : v;
            if (a > pk) pk = a;
        }
        WOK_Seis_AppData.QuicklookTlm.RMS[slot]       = sqrtf((float)(sumsq / win));
        WOK_Seis_AppData.QuicklookTlm.DC_Offset[slot] = (float)(sum / win);
        WOK_Seis_AppData.QuicklookTlm.Peak[slot]      = pk;
    }
    for (uint8 slot = nch; slot < WOK_SEIS_MAX_CHANNELS; slot++)
    {
        WOK_Seis_AppData.QuicklookTlm.RMS[slot]       = 0.0f;
        WOK_Seis_AppData.QuicklookTlm.DC_Offset[slot] = 0.0f;
        WOK_Seis_AppData.QuicklookTlm.Peak[slot]      = 0.0f;
    }
    WOK_Seis_AppData.QuicklookTlm.SampleRate_Hz = WOK_Seis_AppData.SampleRate_Hz;
}

/* ================================================================== */
/*  Drain HAL samples into the per-strike trace buffer                */
/* ================================================================== */
static void WOK_Seis_DrainSamples(void)
{
    /* static: 16 KB would overflow the app's 16 KB stack */
    static int32 chunk[4096];
    uint32  actual = 0;
    uint8   nch = WOK_Seis_AppData.TraceNumChans;

    if (nch == 0) return;

    while (WOK_HAL_Seis_GetSamples(chunk,
                                   sizeof(chunk) / sizeof(chunk[0]),
                                   &actual) == WOK_HAL_OK && actual > 0)
    {
        uint32 n_samps = actual / nch;   /* samples per channel this chunk */
        uint32 room    = WOK_SEIS_TRACE_MAX_SAMPS_PER_CH - WOK_Seis_AppData.TraceCount;
        uint32 keep    = (n_samps < room) ? n_samps : room;

        if (keep > 0)
        {
            memcpy(&WOK_Seis_AppData.TraceFile.Samples[WOK_Seis_AppData.TraceCount * nch],
                   chunk, (size_t)keep * nch * sizeof(int32));
            WOK_Seis_AppData.TraceCount += keep;

            /* QC: count clipped samples per channel as they stream in */
            for (uint32 s = 0; s < keep * nch; s++)
            {
                int32 v = chunk[s];
                if (v > (int32)WOK_SEIS_QC_CLIP_THRESHOLD ||
                    v < -(int32)WOK_SEIS_QC_CLIP_THRESHOLD)
                    WOK_Seis_AppData.ClipCount[s % nch]++;
            }
        }
        if (keep < n_samps && !WOK_Seis_AppData.TraceTruncated)
        {
            WOK_Seis_AppData.TraceTruncated = 1;
            CFE_EVS_SendEvent(WOK_SEIS_TRACE_FULL_EID, CFE_EVS_EventType_ERROR,
                              "WOK_SEIS: Trace buffer full at %lu samps/chan — truncating",
                              (unsigned long)WOK_Seis_AppData.TraceCount);
        }
    }
}

/* ================================================================== */
/*  File-header fields common to raw and stacked products             */
/* ================================================================== */
static void WOK_Seis_FillHeaderCommon(WOK_Seis_FileHeader_t *hdr, uint32 spc,
                                      uint32 strike_idx, uint8 strike_valid)
{
    memset(hdr, 0, sizeof(*hdr));
    hdr->Magic           = WOK_SEIS_FILE_MAGIC;
    hdr->Version         = WOK_SEIS_FILE_VERSION;
    hdr->NumChannels     = WOK_Seis_AppData.TraceNumChans;
    hdr->SampleRate_Hz   = WOK_Seis_AppData.SampleRate_Hz;
    hdr->Gain            = WOK_Seis_AppData.Gain;
    hdr->SamplesPerChan  = spc;
    hdr->StrikeSampleIdx = strike_idx;
    hdr->StrikeValid     = strike_valid;
    hdr->ChannelMask     = WOK_Seis_AppData.ChannelMask;
    hdr->SeqId           = WOK_Seis_AppData.LastSeqId;
    hdr->SourceForce_N  = WOK_Seis_AppData.SourceForce_N;
    hdr->SiteX_m         = WOK_Seis_AppData.SiteX_m;
    hdr->SiteY_m         = WOK_Seis_AppData.SiteY_m;
    hdr->SiteHeadingDeg  = WOK_Seis_AppData.SiteHeadingDeg;
    memcpy(hdr->PodOffsets_m, WOK_Seis_AppData.PodOffsets, sizeof(hdr->PodOffsets_m));
    hdr->AcqIndex        = WOK_Seis_AppData.AcqIndex;
    for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
        hdr->FirstBreakMs[ch] = -1.0f;
    /* v3: shot context (latched at FIRE) + bank state at archive time */
    hdr->SourceMode    = WOK_Seis_AppData.ShotMode;
    hdr->SourceEnergyJ = WOK_Seis_AppData.ShotEnergyJ;
    hdr->SweepF0_Hz    = WOK_Seis_AppData.ShotF0_Hz;
    hdr->SweepF1_Hz    = WOK_Seis_AppData.ShotF1_Hz;
    hdr->SweepDur_s    = WOK_Seis_AppData.ShotDur_s;
    hdr->BankV         = WOK_Seis_AppData.BankV;
    hdr->BankSOC_pct   = WOK_Seis_AppData.BankSOC_pct;
    /* CP-2: plate verification record. Raw files: this record's verdict
     * (WriteStackFile overrides with the site-level outcome). */
    hdr->PlateDegraded = WOK_Seis_AppData.PlateOk ? 0 : 1;
    hdr->PlateReseats  = WOK_Seis_AppData.PlateReseatsSite;
}

/* ================================================================== */
/*  Write the per-strike raw trace file                               */
/* ================================================================== */
static void WOK_Seis_WriteTraceFile(void)
{
    char  name[32];
    uint8 nch = WOK_Seis_AppData.TraceNumChans;

    if (nch == 0 || WOK_Seis_AppData.TraceCount == 0) return;

    WOK_Seis_AppData.AcqIndex++;

    WOK_Seis_FillHeaderCommon(&WOK_Seis_AppData.TraceFile.Hdr,
                              WOK_Seis_AppData.TraceCount,
                              WOK_Seis_AppData.StrikeSampleIdx,
                              WOK_Seis_AppData.StrikeValid);
    WOK_Seis_AppData.TraceFile.Hdr.StackFold = 1;

    snprintf(name, sizeof(name), "seis_%04lu.wok",
             (unsigned long)WOK_Seis_AppData.AcqIndex);

    uint32 size = (uint32)(sizeof(WOK_Seis_FileHeader_t)
                + (size_t)WOK_Seis_AppData.TraceCount * nch * sizeof(int32));

    if (WOK_HAL_Storage_WriteFile(name, &WOK_Seis_AppData.TraceFile, size) == WOK_HAL_OK)
    {
        CFE_EVS_SendEvent(WOK_SEIS_TRACE_WRITTEN_EID, CFE_EVS_EventType_INFORMATION,
                          "WOK_SEIS: %s — %luk samps x%u ch, strike@%lu%s, site (%.1f,%.1f)",
                          name,
                          (unsigned long)(WOK_Seis_AppData.TraceCount / 1000),
                          nch,
                          (unsigned long)WOK_Seis_AppData.StrikeSampleIdx,
                          WOK_Seis_AppData.StrikeValid ? "" : " (NO STRIKE)",
                          (double)WOK_Seis_AppData.SiteX_m,
                          (double)WOK_Seis_AppData.SiteY_m);
    }
    else
    {
        WOK_Seis_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_SEIS_DATA_WRITE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Trace file write FAILED (%s, %lu bytes)",
                          name, (unsigned long)size);
    }
}

/* ================================================================== */
/*  Plate coupling metric (WOK-CP-001)                                */
/*                                                                    */
/*  ratio = (near-pod signal RMS over the post-t0 window) / (plate    */
/*  peak force) — force-normalized, so commanded energy cancels and   */
/*  only transfer into the ground moves it. Verdict is against a      */
/*  running site-to-site EMA baseline seeded by the first evaluated   */
/*  shot (a bad FIRST seat therefore poisons the seed — baseline      */
/*  maintenance is an open question, a testbed item). Sweep shots    */
/*  are not evaluated: pre-correlation arrivals sit at the noise      */
/*  floor. Only accepted shots update the baseline.                   */
/* ================================================================== */
static void WOK_Seis_ComputePlateMetric(void)
{
    WOK_Seis_AppData.PlateRatio = -1.0f;
    WOK_Seis_AppData.PlateOk    = 1;

    if (WOK_Seis_AppData.ShotMode != WOK_HAL_SRC_MODE_IMPULSE) return;
    if (!WOK_Seis_AppData.StrikeValid ||
        WOK_Seis_AppData.SourceForce_N <= 0.0f) return;

    uint8  nch = WOK_Seis_AppData.TraceNumChans;
    uint32 s0  = WOK_Seis_AppData.StrikeSampleIdx;
    uint32 s1  = s0 + WOK_SEIS_PLATE_WIN_SAMPS;
    if (nch == 0) return;
    if (s1 > WOK_Seis_AppData.TraceCount) s1 = WOK_Seis_AppData.TraceCount;
    if (s1 <= s0) return;

    double acc = 0.0;
    for (uint32 s = s0; s < s1; s++)
    {
        double v = (double)WOK_Seis_AppData.TraceFile.Samples[s * nch + 0];
        acc += v * v;
    }
    float rms = sqrtf((float)(acc / (double)(s1 - s0)));
    WOK_Seis_AppData.PlateRatio = rms / WOK_Seis_AppData.SourceForce_N;

    if (WOK_Seis_AppData.PlateBaseline <= 0.0f)
    {
        WOK_Seis_AppData.PlateBaseline = WOK_Seis_AppData.PlateRatio;
    }
    else if (WOK_Seis_AppData.PlateRatio >=
             WOK_SEIS_PLATE_COUP_FRAC * WOK_Seis_AppData.PlateBaseline)
    {
        WOK_Seis_AppData.PlateBaseline =
            (1.0f - WOK_SEIS_PLATE_EMA_ALPHA) * WOK_Seis_AppData.PlateBaseline
            + WOK_SEIS_PLATE_EMA_ALPHA * WOK_Seis_AppData.PlateRatio;
    }
    else
    {
        WOK_Seis_AppData.PlateOk = 0;
        if (WOK_Seis_AppData.AcceptDegraded)
            WOK_Seis_AppData.PlateDegradedSite = 1;
        CFE_EVS_SendEvent(WOK_SEIS_PLATE_LOW_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Plate coupling LOW — ratio %.1f vs baseline %.1f (%.0f%%)%s",
                          (double)WOK_Seis_AppData.PlateRatio,
                          (double)WOK_Seis_AppData.PlateBaseline,
                          (double)(100.0f * WOK_Seis_AppData.PlateRatio
                                   / WOK_Seis_AppData.PlateBaseline),
                          WOK_Seis_AppData.AcceptDegraded
                              ? " — accepted DEGRADED" : "");
    }
}

/* ================================================================== */
/*  Per-pod coupling scores from a test shot (CP-4, WOK-CP-001)       */
/*                                                                    */
/*  score_p = (RMS_p x (x_p^2 + 1)) / median over the array — signal  */
/*  RMS over the post-t0 window, normalized by geometric spreading    */
/*  at the pod's ACTUAL offset, ratioed to the array median. The      */
/*  plate factor is common-mode and cancels: this is a pure pod-side  */
/*  measurement, the plate metric's missing half. The spectral-      */
/*  coherence term is deferred until the coupling model has spectral  */
/*  structure (Sim Model Spec §11.8; bench item).                     */
/* ================================================================== */
static void WOK_Seis_ComputePodScores(void)
{
    uint8  nch = WOK_Seis_AppData.TraceNumChans;
    uint32 s0  = WOK_Seis_AppData.StrikeSampleIdx;
    uint32 s1  = s0 + WOK_SEIS_PLATE_WIN_SAMPS;

    if (nch == 0 || !WOK_Seis_AppData.StrikeValid) return;
    if (s1 > WOK_Seis_AppData.TraceCount) s1 = WOK_Seis_AppData.TraceCount;
    if (s1 <= s0) return;

    float norm[WOK_SEIS_MAX_CHANNELS] = {0};
    for (uint8 slot = 0; slot < nch; slot++)
    {
        double acc = 0.0;
        for (uint32 s = s0; s < s1; s++)
        {
            double v = (double)WOK_Seis_AppData.TraceFile.Samples[s * nch + slot];
            acc += v * v;
        }
        float off = WOK_Seis_AppData.PodOffsets[slot];
        norm[slot] = sqrtf((float)(acc / (double)(s1 - s0)))
                   * (off * off + 1.0f);
    }

    /* Array median (nch <= 4: sort a copy, average the middle pair) */
    float sorted[WOK_SEIS_MAX_CHANNELS];
    memcpy(sorted, norm, sizeof(sorted));
    for (uint8 i = 0; i < nch; i++)
        for (uint8 j = i + 1; j < nch; j++)
            if (sorted[j] < sorted[i])
            { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
    float median = (nch & 1) ? sorted[nch / 2]
                             : 0.5f * (sorted[nch / 2 - 1] + sorted[nch / 2]);
    if (median <= 0.0f) return;

    WOK_Seis_AppData.PodScoreOkMask = 0;
    for (uint8 slot = 0; slot < nch; slot++)
    {
        WOK_Seis_AppData.PodScore[slot] = norm[slot] / median;
        if (WOK_Seis_AppData.PodScore[slot] >= WOK_SEIS_POD_SCORE_MIN)
            WOK_Seis_AppData.PodScoreOkMask |= (uint8)(1u << slot);
    }
    WOK_Seis_AppData.PodScoreValid = 1;
    WOK_Seis_AppData.TestShotCount++;

    CFE_EVS_SendEvent(WOK_SEIS_POD_SCORES_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_SEIS: Pod scores %.2f/%.2f/%.2f/%.2f ok=0x%X (test shot %u)",
                      (double)WOK_Seis_AppData.PodScore[0],
                      (double)WOK_Seis_AppData.PodScore[1],
                      (double)WOK_Seis_AppData.PodScore[2],
                      (double)WOK_Seis_AppData.PodScore[3],
                      WOK_Seis_AppData.PodScoreOkMask,
                      WOK_Seis_AppData.TestShotCount);
}

/* ================================================================== */
/*  Sync/pick-consistency check (CP-5, WOK-CP-001)                    */
/*                                                                    */
/*  The test shot is a known-t0 event observed at every pod, so it    */
/*  checks sync AND coupling: pick each pod's first break on the raw  */
/*  test record, fit the same two-layer moveout the evaluate uses,    */
/*  and take per-pod residuals. A residual beyond the pick spec means */
/*  that pod's timebase (or pick) is off the array's family. Reported */
/*  and archived, not acted on — no mechanism fixes a clock; ground   */
/*  deweights and investigates.                                       */
/* ================================================================== */
static void WOK_Seis_PickTrace(float fb_ms[WOK_SEIS_MAX_CHANNELS])
{
    uint8  nch    = WOK_Seis_AppData.TraceNumChans;
    float  fs     = (float)WOK_Seis_AppData.SampleRate_Hz;
    const int32 *tr = WOK_Seis_AppData.TraceFile.Samples;
    uint32 strike = WOK_Seis_AppData.StrikeSampleIdx;

    for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++) fb_ms[ch] = -1.0f;
    if (nch == 0 || !WOK_Seis_AppData.StrikeValid || fs <= 0.0f) return;
    if (strike < WOK_SEIS_PICK_STA_SAMPS + 100) return;

    uint32 s_end = strike + WOK_SEIS_PLATE_WIN_SAMPS;
    if (s_end > WOK_Seis_AppData.TraceCount)
        s_end = WOK_Seis_AppData.TraceCount;

    for (uint8 slot = 0; slot < nch; slot++)
    {
        double noise_sq = 0.0;
        uint32 n_noise  = strike - WOK_SEIS_PICK_STA_SAMPS;
        for (uint32 s = 0; s < n_noise; s++)
        {
            double v = (double)tr[s * nch + slot];
            noise_sq += v * v;
        }
        noise_sq /= (double)n_noise;
        if (noise_sq < 1.0) noise_sq = 1.0;
        float noise_rms = sqrtf((float)noise_sq);

        double sta_sq = 0.0;
        for (uint32 s = strike - WOK_SEIS_PICK_STA_SAMPS; s < strike; s++)
        {
            double v = (double)tr[s * nch + slot];
            sta_sq += v * v;
        }
        for (uint32 s = strike; s < s_end; s++)
        {
            double v_in  = (double)tr[s * nch + slot];
            double v_out = (double)tr[(s - WOK_SEIS_PICK_STA_SAMPS) * nch + slot];
            sta_sq += v_in * v_in - v_out * v_out;
            float ratio = (float)sqrt((sta_sq / WOK_SEIS_PICK_STA_SAMPS) / noise_sq);
            if (ratio > WOK_SEIS_PICK_TRIGGER_RATIO)
            {
                uint32 onset = s;
                for (uint32 b = s - WOK_SEIS_PICK_STA_SAMPS + 1; b <= s; b++)
                {
                    int32 av = tr[b * nch + slot];
                    if (av < 0) av = -av;
                    if ((float)av > 4.0f * noise_rms) { onset = b; break; }
                }
                fb_ms[slot] = (float)(onset - strike) * 1000.0f / fs;
                break;
            }
        }
    }
}

static void WOK_Seis_SyncCheck(void)
{
    float fb[WOK_SEIS_MAX_CHANNELS];
    const float *x = WOK_Seis_AppData.PodOffsets;

    for (uint8 p = 0; p < WOK_SEIS_MAX_CHANNELS; p++)
        WOK_Seis_AppData.SyncResidualMs[p] = 0.0f;
    WOK_Seis_AppData.SyncSuspectMask = 0;

    WOK_Seis_PickTrace(fb);
    for (uint8 p = 0; p < WOK_SEIS_MAX_CHANNELS; p++)
        if (fb[p] < 0.0f) return;             /* incomplete — skip     */
    if (fb[1] <= fb[0]) return;               /* non-monotonic near    */

    float v1 = (x[1] - x[0]) / ((fb[1] - fb[0]) * 1e-3f);
    if (v1 <= 0.0f) return;

    /* Best-fit (v2, h1) over the evaluate grid; unweighted SSE */
    float best_sse = 1e30f, best_v2 = 0.0f, best_h1 = 0.0f;
    for (float v2 = WOK_SEIS_EVAL_V2_MIN_MPS; v2 <= WOK_SEIS_EVAL_V2_MAX_MPS;
         v2 += WOK_SEIS_EVAL_V2_STEP_MPS)
    {
        if (v2 <= v1) continue;
        float sin_ic = v1 / v2;
        float cos_ic = sqrtf(1.0f - sin_ic * sin_ic);
        for (float h1 = WOK_SEIS_EVAL_H1_MIN_M; h1 <= WOK_SEIS_EVAL_H1_MAX_M;
             h1 += WOK_SEIS_EVAL_H1_STEP_M)
        {
            float ti = 2.0f * h1 * cos_ic / v1 * 1e3f;
            float sse = 0.0f;
            for (uint8 p = 0; p < WOK_SEIS_MAX_CHANNELS; p++)
            {
                float direct = x[p] / v1 * 1e3f;
                float head   = x[p] / v2 * 1e3f + ti;
                float t_pred = (head < direct) ? head : direct;
                float r = fb[p] - t_pred;
                sse += r * r;
            }
            if (sse < best_sse) { best_sse = sse; best_v2 = v2; best_h1 = h1; }
        }
    }
    if (best_v2 <= 0.0f) return;

    float sin_ic = v1 / best_v2;
    float cos_ic = sqrtf(1.0f - sin_ic * sin_ic);
    float ti     = 2.0f * best_h1 * cos_ic / v1 * 1e3f;
    for (uint8 p = 0; p < WOK_SEIS_MAX_CHANNELS; p++)
    {
        float direct = x[p] / v1 * 1e3f;
        float head   = x[p] / best_v2 * 1e3f + ti;
        float t_pred = (head < direct) ? head : direct;
        WOK_Seis_AppData.SyncResidualMs[p] = fb[p] - t_pred;
        if (fabsf(WOK_Seis_AppData.SyncResidualMs[p]) > WOK_SEIS_SYNC_RESID_MAX_MS)
            WOK_Seis_AppData.SyncSuspectMask |= (uint8)(1u << p);
    }

    CFE_EVS_SendEvent(WOK_SEIS_SYNC_EID,
                      WOK_Seis_AppData.SyncSuspectMask
                          ? CFE_EVS_EventType_ERROR
                          : CFE_EVS_EventType_INFORMATION,
                      "WOK_SEIS: Sync residuals %+.2f/%+.2f/%+.2f/%+.2f ms — suspect 0x%X",
                      (double)WOK_Seis_AppData.SyncResidualMs[0],
                      (double)WOK_Seis_AppData.SyncResidualMs[1],
                      (double)WOK_Seis_AppData.SyncResidualMs[2],
                      (double)WOK_Seis_AppData.SyncResidualMs[3],
                      WOK_Seis_AppData.SyncSuspectMask);
}

/* ================================================================== */
/*  Coupling record (coup_NNNN.wok) — written after each test shot;   */
/*  NNNN matches the site's upcoming stack file.                      */
/* ================================================================== */
static void WOK_Seis_WriteCoupFile(void)
{
    WOK_Coup_FileHeader_t hdr;
    char name[32];

    memset(&hdr, 0, sizeof(hdr));
    hdr.Magic          = WOK_COUP_FILE_MAGIC;
    hdr.Version        = WOK_COUP_FILE_VERSION;
    hdr.SeqId          = WOK_Seis_AppData.LastSeqId;
    hdr.SiteX_m        = WOK_Seis_AppData.SiteX_m;
    hdr.SiteY_m        = WOK_Seis_AppData.SiteY_m;
    hdr.SiteHeadingDeg = WOK_Seis_AppData.SiteHeadingDeg;
    memcpy(hdr.PodOffsets_m, WOK_Seis_AppData.PodOffsets, sizeof(hdr.PodOffsets_m));
    memcpy(hdr.PodScore,     WOK_Seis_AppData.PodScore,   sizeof(hdr.PodScore));
    hdr.PodScoreOkMask  = WOK_Seis_AppData.PodScoreOkMask;
    hdr.PodMarginalMask = WOK_Seis_AppData.PodMarginalMask;
    memcpy(hdr.PodAttempts, WOK_Seis_AppData.PodAttempts, sizeof(hdr.PodAttempts));
    for (uint8 p = 0; p < WOK_SEIS_MAX_CHANNELS; p++)
        if (WOK_Seis_AppData.PodScoreOkMask & (1u << p))
            hdr.VerifiedCount++;
    hdr.TestShots     = WOK_Seis_AppData.TestShotCount;
    hdr.PlateDegraded = WOK_Seis_AppData.PlateDegradedSite;
    hdr.PlateReseats  = WOK_Seis_AppData.PlateReseatsSite;
    hdr.PlateRatio    = WOK_Seis_AppData.PlateRatio;
    hdr.PlateBaseline = WOK_Seis_AppData.PlateBaseline;
    memcpy(hdr.SyncResidualMs, WOK_Seis_AppData.SyncResidualMs,
           sizeof(hdr.SyncResidualMs));
    hdr.SyncSuspectMask = WOK_Seis_AppData.SyncSuspectMask;

    snprintf(name, sizeof(name), "coup_%04lu.wok",
             (unsigned long)(WOK_Seis_AppData.StackFileIndex + 1));

    if (WOK_HAL_Storage_WriteFile(name, &hdr, sizeof(hdr)) == WOK_HAL_OK)
        CFE_EVS_SendEvent(WOK_SEIS_COUP_WRITTEN_EID, CFE_EVS_EventType_INFORMATION,
                          "WOK_SEIS: %s — verified %u/4, site (%.1f,%.1f)",
                          name, hdr.VerifiedCount,
                          (double)hdr.SiteX_m, (double)hdr.SiteY_m);
    else
    {
        WOK_Seis_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_SEIS_DATA_WRITE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Coupling record write FAILED (%s)", name);
    }
}

/* ================================================================== */
/*  Fold the just-completed record into the site stack                */
/*                                                                    */
/*  Alignment: each record contributes the [-PRE,+POST] window around */
/*  its own strike t0. Coherence = zero-lag normalized correlation of */
/*  the new segment against the existing stack, averaged over         */
/*  channels, running-averaged over additions.                        */
/*                                                                    */
/*  The source buffer is the raw trace (impulse) or the correlated    */
/*  equivalent-impulse trace (sweep) — sweep-mode stacks are stacks   */
/*  of correlated traces.                                             */
/* ================================================================== */
static void WOK_Seis_AccumulateStack(const int32 *trace, uint32 count,
                                     uint32 strike)
{
    uint8  nch    = WOK_Seis_AppData.TraceNumChans;
    int32 *stack  = WOK_Seis_AppData.StackFile.Samples;

    if (nch == 0 || !WOK_Seis_AppData.StrikeValid) return;
    if (strike < WOK_SEIS_STACK_PRE_SAMPS) return;   /* window underflows */
    if (strike + WOK_SEIS_STACK_POST_SAMPS > count)
        return;                                       /* window overflows  */

    uint32 t0 = strike - WOK_SEIS_STACK_PRE_SAMPS;   /* trace index of window start */

    /* Coherence vs the existing stack (only meaningful from fold 2 on) */
    if (WOK_Seis_AppData.StackFold > 0)
    {
        double coh_sum = 0.0;
        for (uint8 slot = 0; slot < nch; slot++)
        {
            double dot = 0.0, na = 0.0, nb = 0.0;
            for (uint32 s = 0; s < WOK_SEIS_STACK_WIN_SAMPS; s++)
            {
                double a = (double)stack[s * nch + slot];
                double b = (double)trace[(t0 + s) * nch + slot];
                dot += a * b;
                na  += a * a;
                nb  += b * b;
            }
            if (na > 0.0 && nb > 0.0)
                coh_sum += dot / (sqrt(na) * sqrt(nb));
        }
        float coh = (float)(coh_sum / nch);
        if (coh < 0.0f) coh = 0.0f;
        /* Running mean over the fold-2..N additions */
        float k = (float)WOK_Seis_AppData.StackFold;   /* additions so far with data */
        WOK_Seis_AppData.StackCoherence =
            (WOK_Seis_AppData.StackCoherence * (k - 1.0f) + coh) / k;
    }

    for (uint32 s = 0; s < WOK_SEIS_STACK_WIN_SAMPS * nch; s++)
        stack[s] += trace[t0 * nch + s];

    WOK_Seis_AppData.StackFold++;
}

/* ================================================================== */
/*  After a record has been folded into the stack: if it was the      */
/*  site's final shot, pick first breaks and archive the stack.       */
/* ================================================================== */
static void WOK_Seis_FinishSiteIfDone(void)
{
    if ((uint16)(WOK_Seis_AppData.StackIdx + 1) >= WOK_Seis_AppData.StackTarget)
    {
        WOK_Seis_RunPicker();
        WOK_Seis_WriteStackFile();
    }
}

/* ================================================================== */
/*  Onboard sweep correlation                                         */
/*                                                                    */
/*  The recorded sweep is cross-correlated against the analytically   */
/*  regenerated pilot (WOK_HAL_Src_PilotSample — the source contract  */
/*  waveform) to produce the equivalent-impulse trace over the stack  */
/*  window [-PRE,+POST) around t0:                                    */
/*                                                                    */
/*    corr[j] = GAIN/Ep * sum_n trace[t0-PRE+j-LEAD+n] * pilot[n]     */
/*                                                                    */
/*  LEAD delays the output by CORR_LEAD_FRAC/(f1-f0): the correlation */
/*  (Klauder) wavelet is zero-phase and leads its center by           */
/*  ~1/bandwidth, which the onset picker would read as an early       */
/*  arrival; the delay re-aligns equivalent-impulse onsets with true  */
/*  impulse onsets (harness-tuned, see wok_platform_cfg.h).           */
/*                                                                    */
/*  The output is computed WOK_SEIS_CORR_SAMPS_PER_TICK samples/chan  */
/*  per HK tick so the app main loop never blocks longer than one     */
/*  tick; Acquiring stays asserted in TLM until the correlated trace  */
/*  is stacked, so the sequencer's RECORD state waits naturally.      */
/* ================================================================== */
static void WOK_Seis_StartCorrelation(void)
{
    float fs  = (float)WOK_Seis_AppData.SampleRate_Hz;
    float f0  = WOK_Seis_AppData.ShotF0_Hz;
    float f1  = WOK_Seis_AppData.ShotF1_Hz;
    float dur = WOK_Seis_AppData.ShotDur_s;

    if (fs <= 0.0f || f1 <= f0 || dur <= 0.0f)
    {
        CFE_EVS_SendEvent(WOK_SEIS_CORR_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Correlation skipped — bad pilot (%.0f-%.0f Hz %.1fs)",
                          (double)f0, (double)f1, (double)dur);
        return;
    }

    uint32 np = (uint32)(dur * fs);
    if (np > WOK_SEIS_PILOT_MAX_SAMPS)
    {
        np = WOK_SEIS_PILOT_MAX_SAMPS;
        CFE_EVS_SendEvent(WOK_SEIS_CORR_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Pilot clamped to %u samps (dur*fs too long)",
                          (unsigned int)np);
    }

    double ep = 0.0;
    for (uint32 n = 0; n < np; n++)
    {
        float p = WOK_HAL_Src_PilotSample((float)n / fs, f0, f1, dur);
        WOK_Seis_AppData.Pilot[n] = p;
        ep += (double)p * p;
    }
    if (ep <= 0.0) ep = 1.0;

    WOK_Seis_AppData.PilotSamps    = np;
    WOK_Seis_AppData.PilotEnergy   = ep;
    WOK_Seis_AppData.CorrLeadSamps =
        (int32)(WOK_SEIS_CORR_LEAD_FRAC / (f1 - f0) * fs + 0.5f);
    WOK_Seis_AppData.CorrPos       = 0;
    WOK_Seis_AppData.CorrActive    = 1;
    memset(WOK_Seis_AppData.CorrFile.Samples, 0,
           sizeof(WOK_Seis_AppData.CorrFile.Samples));
}

static void WOK_Seis_PumpCorrelation(void)
{
    if (!WOK_Seis_AppData.CorrActive) return;

    uint8         nch   = WOK_Seis_AppData.TraceNumChans;
    const int32  *trace = WOK_Seis_AppData.TraceFile.Samples;
    const float  *pilot = WOK_Seis_AppData.Pilot;
    uint32        np    = WOK_Seis_AppData.PilotSamps;
    uint32        count = WOK_Seis_AppData.TraceCount;
    uint32        strike = WOK_Seis_AppData.StrikeSampleIdx;
    double        ep    = WOK_Seis_AppData.PilotEnergy;

    uint32 end = WOK_Seis_AppData.CorrPos + WOK_SEIS_CORR_SAMPS_PER_TICK;
    if (end > WOK_SEIS_STACK_WIN_SAMPS) end = WOK_SEIS_STACK_WIN_SAMPS;

    for (uint32 j = WOK_Seis_AppData.CorrPos; j < end; j++)
    {
        int64 base = (int64)strike - WOK_SEIS_STACK_PRE_SAMPS
                   + (int64)j - WOK_Seis_AppData.CorrLeadSamps;
        /* Clamp the pilot overlap to the recorded samples */
        uint32 n0 = (base < 0) ? (uint32)(-base) : 0;
        uint32 n1 = np;
        if (base + (int64)np > (int64)count)
            n1 = (base >= (int64)count) ? 0 : (uint32)((int64)count - base);

        for (uint8 slot = 0; slot < nch; slot++)
        {
            double acc = 0.0;
            for (uint32 n = n0; n < n1; n++)
                acc += (double)trace[((uint32)(base + n)) * nch + slot]
                     * pilot[n];
            WOK_Seis_AppData.CorrFile.Samples[j * nch + slot] =
                (int32)(acc / ep * WOK_SEIS_CORR_GAIN);
        }
    }
    WOK_Seis_AppData.CorrPos = end;

    if (WOK_Seis_AppData.CorrPos >= WOK_SEIS_STACK_WIN_SAMPS)
    {
        WOK_Seis_AppData.CorrActive = 0;
        WOK_Seis_WriteCorrFile();
        /* The correlated trace covers exactly the stack window; its t0
         * sits at index PRE by construction. */
        WOK_Seis_AccumulateStack(WOK_Seis_AppData.CorrFile.Samples,
                                 WOK_SEIS_STACK_WIN_SAMPS,
                                 WOK_SEIS_STACK_PRE_SAMPS);
        WOK_Seis_FinishSiteIfDone();
        CFE_EVS_SendEvent(WOK_SEIS_CORR_EID, CFE_EVS_EventType_INFORMATION,
                          "WOK_SEIS: Correlation complete — fold %u at site (%.1f,%.1f)",
                          WOK_Seis_AppData.StackFold,
                          (double)WOK_Seis_AppData.SiteX_m,
                          (double)WOK_Seis_AppData.SiteY_m);
    }
}

/* ================================================================== */
/*  Write the correlated (equivalent-impulse) trace product           */
/* ================================================================== */
static void WOK_Seis_WriteCorrFile(void)
{
    char  name[32];
    uint8 nch = WOK_Seis_AppData.TraceNumChans;

    if (nch == 0) return;

    WOK_Seis_FileHeader_t *hdr = &WOK_Seis_AppData.CorrFile.Hdr;
    WOK_Seis_FillHeaderCommon(hdr, WOK_SEIS_STACK_WIN_SAMPS,
                              WOK_SEIS_STACK_PRE_SAMPS, 1);
    hdr->StackFold = 1;

    /* Same NNNN as the raw sweep record it derives from */
    snprintf(name, sizeof(name), "corr_%04lu.wok",
             (unsigned long)WOK_Seis_AppData.AcqIndex);

    uint32 size = (uint32)(sizeof(*hdr)
                + (size_t)WOK_SEIS_STACK_WIN_SAMPS * nch * sizeof(int32));

    if (WOK_HAL_Storage_WriteFile(name, &WOK_Seis_AppData.CorrFile, size) == WOK_HAL_OK)
    {
        CFE_EVS_SendEvent(WOK_SEIS_CORR_WRITTEN_EID, CFE_EVS_EventType_INFORMATION,
                          "WOK_SEIS: %s — equivalent-impulse trace, site (%.1f,%.1f)",
                          name,
                          (double)WOK_Seis_AppData.SiteX_m,
                          (double)WOK_Seis_AppData.SiteY_m);
    }
    else
    {
        WOK_Seis_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_SEIS_DATA_WRITE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Corr file write FAILED (%s)", name);
    }
}

/* ================================================================== */
/*  STA/LTA first-break picker on the stacked window                  */
/*                                                                    */
/*  LTA is frozen from the pre-strike noise (standard for triggered   */
/*  records); trigger at STA/LTA > ratio, then the onset is refined   */
/*  back to the first amplitude excursion above 4x noise RMS.         */
/* ================================================================== */
static void WOK_Seis_RunPicker(void)
{
    uint8  nch = WOK_Seis_AppData.TraceNumChans;
    int32 *stack = WOK_Seis_AppData.StackFile.Samples;
    float  fs = (float)WOK_Seis_AppData.SampleRate_Hz;

    for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
    {
        WOK_Seis_AppData.FirstBreakMs[ch] = -1.0f;
        WOK_Seis_AppData.PickQuality[ch]  = 0;
    }
    if (nch == 0 || WOK_Seis_AppData.StackFold == 0 || fs <= 0.0f) return;

    for (uint8 slot = 0; slot < nch; slot++)
    {
        /* Frozen LTA: noise power over the pre-strike window */
        double noise_sq = 0.0;
        uint32 n_noise  = WOK_SEIS_STACK_PRE_SAMPS - WOK_SEIS_PICK_STA_SAMPS;
        for (uint32 s = 0; s < n_noise; s++)
        {
            double v = (double)stack[s * nch + slot];
            noise_sq += v * v;
        }
        noise_sq /= (double)n_noise;
        if (noise_sq < 1.0) noise_sq = 1.0;
        float noise_rms = sqrtf((float)noise_sq);

        /* Rolling STA power from t0 forward */
        double sta_sq = 0.0;
        for (uint32 s = WOK_SEIS_STACK_PRE_SAMPS - WOK_SEIS_PICK_STA_SAMPS;
             s < WOK_SEIS_STACK_PRE_SAMPS; s++)
        {
            double v = (double)stack[s * nch + slot];
            sta_sq += v * v;
        }

        for (uint32 s = WOK_SEIS_STACK_PRE_SAMPS; s < WOK_SEIS_STACK_WIN_SAMPS; s++)
        {
            double v_in  = (double)stack[s * nch + slot];
            double v_out = (double)stack[(s - WOK_SEIS_PICK_STA_SAMPS) * nch + slot];
            sta_sq += v_in * v_in - v_out * v_out;

            float ratio = (float)sqrt((sta_sq / WOK_SEIS_PICK_STA_SAMPS) / noise_sq);
            if (ratio > WOK_SEIS_PICK_TRIGGER_RATIO)
            {
                /* Refine: first |sample| above 4x noise inside the STA win */
                uint32 onset = s;
                for (uint32 b = s - WOK_SEIS_PICK_STA_SAMPS + 1; b <= s; b++)
                {
                    int32 av = stack[b * nch + slot];
                    if (av < 0) av = -av;
                    if ((float)av > 4.0f * noise_rms) { onset = b; break; }
                }
                WOK_Seis_AppData.FirstBreakMs[slot] =
                    (float)(onset - WOK_SEIS_STACK_PRE_SAMPS) * 1000.0f / fs;
                float q = ratio * 10.0f;
                WOK_Seis_AppData.PickQuality[slot] =
                    (uint8)(q > 100.0f ? 100 : (q < 0.0f ? 0 : q));
                break;
            }
        }
    }

    CFE_EVS_SendEvent(WOK_SEIS_PICKS_EID, CFE_EVS_EventType_INFORMATION,
                      "WOK_SEIS: Picks (fold %u, coh %.0f%%) — %.2f/%.2f/%.2f/%.2f ms",
                      WOK_Seis_AppData.StackFold,
                      (double)(WOK_Seis_AppData.StackCoherence * 100.0f),
                      (double)WOK_Seis_AppData.FirstBreakMs[0],
                      (double)WOK_Seis_AppData.FirstBreakMs[1],
                      (double)WOK_Seis_AppData.FirstBreakMs[2],
                      (double)WOK_Seis_AppData.FirstBreakMs[3]);
}

/* ================================================================== */
/*  Write the stacked site product                                    */
/* ================================================================== */
static void WOK_Seis_WriteStackFile(void)
{
    char  name[32];
    uint8 nch = WOK_Seis_AppData.TraceNumChans;

    if (nch == 0 || WOK_Seis_AppData.StackFold == 0) return;

    WOK_Seis_AppData.StackFileIndex++;

    WOK_Seis_FileHeader_t *hdr = &WOK_Seis_AppData.StackFile.Hdr;
    WOK_Seis_FillHeaderCommon(hdr, WOK_SEIS_STACK_WIN_SAMPS,
                              WOK_SEIS_STACK_PRE_SAMPS, 1);
    hdr->StackFold    = WOK_Seis_AppData.StackFold;
    hdr->PlateDegraded = WOK_Seis_AppData.PlateDegradedSite;
    float cp = WOK_Seis_AppData.StackCoherence * 100.0f;
    hdr->CoherencePct = (uint8)(cp > 100.0f ? 100 : (cp < 0.0f ? 0 : cp));
    memcpy(hdr->FirstBreakMs, WOK_Seis_AppData.FirstBreakMs, sizeof(hdr->FirstBreakMs));
    memcpy(hdr->PickQuality,  WOK_Seis_AppData.PickQuality,  sizeof(hdr->PickQuality));

    snprintf(name, sizeof(name), "stack_%04lu.wok",
             (unsigned long)WOK_Seis_AppData.StackFileIndex);

    uint32 size = (uint32)(sizeof(*hdr)
                + (size_t)WOK_SEIS_STACK_WIN_SAMPS * nch * sizeof(int32));

    if (WOK_HAL_Storage_WriteFile(name, &WOK_Seis_AppData.StackFile, size) == WOK_HAL_OK)
    {
        CFE_EVS_SendEvent(WOK_SEIS_STACK_WRITTEN_EID, CFE_EVS_EventType_INFORMATION,
                          "WOK_SEIS: %s — fold %u, coh %u%%, site (%.1f,%.1f)",
                          name, hdr->StackFold, hdr->CoherencePct,
                          (double)WOK_Seis_AppData.SiteX_m,
                          (double)WOK_Seis_AppData.SiteY_m);
    }
    else
    {
        WOK_Seis_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_SEIS_DATA_WRITE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Stack file write FAILED (%s)", name);
    }
}

/* ================================================================== */
/*  Read HAL status on every HK cycle                                 */
/* ================================================================== */
static void WOK_Seis_UpdateFromHAL(void)
{
    uint8 acq = 0;
    uint32 samp = 0;

    WOK_HAL_Seis_GetStatus(&acq, &samp);
    WOK_Seis_AppData.Acquiring       = acq;
    WOK_Seis_AppData.SamplesAcquired = samp;

    /* Drain new samples into the trace buffer while acquiring, and once
     * more on the completion edge to catch the tail of the record. */
    if (acq || WOK_Seis_AppData.PrevAcquiring)
        WOK_Seis_DrainSamples();

    /* Acquisition-complete edge: latch strike t0, archive the raw
     * record, then fold it into the site stack — directly for impulse
     * shots, or via the onboard pilot correlation for sweeps (the stack
     * accumulates the correlated trace once the tick-chunked
     * correlation finishes). Pick + write the stack after the site's
     * final shot. */
    if (WOK_Seis_AppData.PrevAcquiring && !acq)
    {
        WOK_HAL_Seis_GetStrikeInfo(&WOK_Seis_AppData.StrikeSampleIdx,
                                   &WOK_Seis_AppData.StrikeValid);
        /* Plate verification before archive/stack: the raw file header
         * carries the verdict, and a failing record stays out of the
         * stack unless this StartAcq carried the AcceptDegraded flag
         * (SEQ re-shoots after commanding a plate reseat). */
        WOK_Seis_ComputePlateMetric();
        WOK_Seis_WriteTraceFile();
        if (WOK_Seis_AppData.TestShot)
        {
            /* Coupling test shot (CP-4/5): score the pods, run the
             * sync/pick-consistency check, archive the coupling
             * record, keep the record out of the site stack. */
            WOK_Seis_ComputePodScores();
            WOK_Seis_SyncCheck();
            WOK_Seis_WriteCoupFile();
        }
        else if (WOK_Seis_AppData.ShotMode == WOK_HAL_SRC_MODE_SWEEP &&
                 WOK_Seis_AppData.StrikeValid)
        {
            WOK_Seis_StartCorrelation();
        }
        else if (WOK_Seis_AppData.PlateOk || WOK_Seis_AppData.AcceptDegraded)
        {
            WOK_Seis_AccumulateStack(WOK_Seis_AppData.TraceFile.Samples,
                                     WOK_Seis_AppData.TraceCount,
                                     WOK_Seis_AppData.StrikeSampleIdx);
            WOK_Seis_FinishSiteIfDone();
        }
        else
        {
            CFE_EVS_SendEvent(WOK_SEIS_STACK_SKIP_EID, CFE_EVS_EventType_INFORMATION,
                              "WOK_SEIS: Record kept out of stack (plate low) — fold stays %u",
                              WOK_Seis_AppData.StackFold);
        }
    }
    WOK_Seis_AppData.PrevAcquiring = acq;

    /* Quicklook + QC from the real trace (clips counted during drain) */
    if (acq || WOK_Seis_AppData.TraceCount > 0)
    {
        WOK_Seis_ComputeQuicklook();

        WOK_Seis_AppData.QC_ClipFlag     = 0;
        WOK_Seis_AppData.QC_DeadChanMask = 0;
        uint8 slot = 0;
        for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
        {
            if (!(WOK_Seis_AppData.ChannelMask & (1u << ch))) continue;
            if (WOK_Seis_AppData.ClipCount[slot] > 0)
                WOK_Seis_AppData.QC_ClipFlag = 1;
            if (WOK_Seis_AppData.QuicklookTlm.RMS[slot] < WOK_SEIS_QC_DEAD_THRESHOLD)
                WOK_Seis_AppData.QC_DeadChanMask |= (uint8)(1u << ch);
            slot++;
        }
        /* Dropout: sample count didn't advance since last HK cycle */
        if (acq && samp > 0 && samp == WOK_Seis_AppData.LastSampleCount)
            WOK_Seis_AppData.QC_DropoutFlag = 1;
    }
    WOK_Seis_AppData.LastSampleCount = samp;

    /* Update source state from HAL */
    WOK_HAL_Src_GetState(&WOK_Seis_AppData.SourceState);
    WOK_HAL_Src_GetBank(&WOK_Seis_AppData.BankV,
                        &WOK_Seis_AppData.BankSOC_pct);
    if (WOK_Seis_AppData.SourceState == WOK_HAL_SRC_SWEEPING ||
        WOK_Seis_AppData.SourceState == WOK_HAL_SRC_FIRED)
    {
        WOK_HAL_Src_GetSweepProgress(&WOK_Seis_AppData.SweepProgress);
    }

    /* Read last shot info once delivered */
    if (WOK_Seis_AppData.SourceState == WOK_HAL_SRC_FIRED)
    {
        WOK_HAL_Src_GetShotInfo(&WOK_Seis_AppData.SourceForce_N,
                                &WOK_Seis_AppData.SourceTimestamp_ms);
    }
}

/* ================================================================== */
void WOK_Seis_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
    switch (CFE_SB_MsgIdToValue(MsgId)) {
        case WOK_SEIS_CMD_MID:
            WOK_Seis_ProcessGroundCommand(BufPtr); break;
        case WOK_SEIS_SEND_HK_MID:
            WOK_Seis_UpdateFromHAL();
            WOK_Seis_PumpCorrelation();
            WOK_Seis_ReportHousekeeping();
            WOK_Seis_PublishStatus();
            if (WOK_Seis_AppData.Acquiring)
                WOK_Seis_PublishQuicklook();
            WOK_Seis_PumpDownlink();
            break;
        default: WOK_Seis_AppData.ErrCounter++; break;
    }
}

/* ================================================================== */
void WOK_Seis_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t CmdCode = 0;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &CmdCode);
    switch (CmdCode) {

        /* ── CC 0: NOOP ──────────────────────────────────────────── */
        case WOK_SEIS_NOOP_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_NoArgsCmd_t))) {
                WOK_Seis_AppData.LastSeqId = ((WOK_Seis_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                CFE_EVS_SendEvent(WOK_SEIS_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEIS: NOOP (v" WOK_VERSION_STRING ")");
            } break;

        /* ── CC 1: RESET COUNTERS ────────────────────────────────── */
        case WOK_SEIS_RESET_CTRS_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_NoArgsCmd_t))) {
                WOK_Seis_AppData.LastSeqId = ((WOK_Seis_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Seis_AppData.CmdCounter = 0;
                WOK_Seis_AppData.ErrCounter = 0;
            } break;

        /* ── CC 2: START ACQUISITION ─────────────────────────────── */
        case WOK_SEIS_START_ACQ_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_StartAcqCmd_t))) {
                WOK_Seis_StartAcqCmd_t *Cmd = (WOK_Seis_StartAcqCmd_t *)BufPtr;
                WOK_Seis_AppData.LastSeqId = Cmd->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                WOK_Seis_AppData.Duration_sec    = Cmd->Duration_sec;
                WOK_Seis_AppData.SampleRate_Hz   = Cmd->SampleRate_Hz;
                WOK_Seis_AppData.Gain            = Cmd->Gain;
                WOK_Seis_AppData.ChannelMask     = Cmd->ChannelMask;
                WOK_Seis_AppData.SiteX_m         = Cmd->SiteX_m;
                WOK_Seis_AppData.SiteY_m         = Cmd->SiteY_m;
                WOK_Seis_AppData.SiteHeadingDeg  = Cmd->SiteHeadingDeg;
                WOK_Seis_AppData.Acquiring       = 1;
                WOK_Seis_AppData.PrevAcquiring   = 1;
                WOK_Seis_AppData.SamplesAcquired = 0;
                WOK_Seis_AppData.LastSampleCount = 0;
                WOK_Seis_AppData.QC_ClipFlag     = 0;
                WOK_Seis_AppData.QC_DeadChanMask = 0;
                WOK_Seis_AppData.QC_DropoutFlag  = 0;
                WOK_Seis_AppData.EvalFlag        = 0;
                WOK_Seis_AppData.EvalConfidence  = 0.0f;
                WOK_Seis_AppData.TraceCount      = 0;
                WOK_Seis_AppData.TraceTruncated  = 0;
                WOK_Seis_AppData.StrikeSampleIdx = 0;
                WOK_Seis_AppData.StrikeValid     = 0;
                WOK_Seis_AppData.TraceNumChans   = 0;
                for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
                {
                    WOK_Seis_AppData.ClipCount[ch] = 0;
                    if (Cmd->ChannelMask & (1u << ch))
                        WOK_Seis_AppData.TraceNumChans++;
                }
                /* A new acquisition supersedes any unfinished correlation */
                WOK_Seis_AppData.CorrActive    = 0;
                WOK_Seis_AppData.SweepProgress = 0.0f;
                /* Plate verification context: per-record verdict resets;
                 * per-site reseat/degraded counters reset when the site
                 * pose changes (NOT on StackIdx==0 — a post-reseat
                 * re-shot of the site's first strike is still the same
                 * seating campaign). */
                WOK_Seis_AppData.PlateRatio = -1.0f;
                WOK_Seis_AppData.PlateOk    = 1;
                WOK_Seis_AppData.AcceptDegraded =
                    (Cmd->Flags & WOK_SEIS_ACQFLAG_ACCEPT_DEGRADED) ? 1 : 0;
                WOK_Seis_AppData.TestShot =
                    (Cmd->Flags & WOK_SEIS_ACQFLAG_TEST_SHOT) ? 1 : 0;
                if (!WOK_Seis_AppData.AcqSiteValid ||
                    Cmd->SiteX_m != WOK_Seis_AppData.PrevAcqSiteX_m ||
                    Cmd->SiteY_m != WOK_Seis_AppData.PrevAcqSiteY_m)
                {
                    WOK_Seis_AppData.PlateReseatsSite  = 0;
                    WOK_Seis_AppData.PlateDegradedSite = 0;
                    WOK_Seis_AppData.PodScoreOkMask    = 0;
                    WOK_Seis_AppData.PodScoreValid     = 0;
                    WOK_Seis_AppData.TestShotCount     = 0;
                    WOK_Seis_AppData.SyncSuspectMask   = 0;
                    for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
                    {
                        WOK_Seis_AppData.PodScore[ch]       = -1.0f;
                        WOK_Seis_AppData.SyncResidualMs[ch] = 0.0f;
                    }
                    WOK_Seis_AppData.PrevAcqSiteX_m    = Cmd->SiteX_m;
                    WOK_Seis_AppData.PrevAcqSiteY_m    = Cmd->SiteY_m;
                    WOK_Seis_AppData.AcqSiteValid      = 1;
                }
                /* Seed the shot context from the config so headers are
                 * defined even if no shot lands in this window; FIRE
                 * re-latches it. */
                WOK_Seis_AppData.ShotMode    = WOK_Seis_AppData.SourceMode;
                WOK_Seis_AppData.ShotEnergyJ = WOK_Seis_AppData.SourceEnergyJ;
                if (WOK_Seis_AppData.SourceMode == WOK_HAL_SRC_MODE_SWEEP)
                {
                    WOK_Seis_AppData.ShotF0_Hz = WOK_Seis_AppData.SweepF0_Hz;
                    WOK_Seis_AppData.ShotF1_Hz = WOK_Seis_AppData.SweepF1_Hz;
                    WOK_Seis_AppData.ShotDur_s = WOK_Seis_AppData.SweepDur_s;
                    /* Sweep record window = sweep duration + listen time:
                     * the commanded duration is the listen time, extended
                     * here by the sweep so the pilot AND the arrivals it
                     * keeps exciting fit in the record. */
                    WOK_Seis_AppData.Duration_sec = (uint16)
                        (Cmd->Duration_sec
                         + (uint16)ceilf(WOK_Seis_AppData.SweepDur_s));
                }
                else
                {
                    WOK_Seis_AppData.ShotF0_Hz = 0.0f;
                    WOK_Seis_AppData.ShotF1_Hz = 0.0f;
                    WOK_Seis_AppData.ShotDur_s = 0.0f;
                }
                /* Stack control: StackIdx 0 opens a new site stack */
                WOK_Seis_AppData.StackIdx    = Cmd->StackIdx;
                WOK_Seis_AppData.StackTarget = (Cmd->StackTarget == 0) ? 1
                                             : Cmd->StackTarget;
                if (Cmd->StackIdx == 0)
                {
                    memset(WOK_Seis_AppData.StackFile.Samples, 0,
                           sizeof(WOK_Seis_AppData.StackFile.Samples));
                    WOK_Seis_AppData.StackFold      = 0;
                    WOK_Seis_AppData.StackCoherence = 0.0f;
                    for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
                    {
                        WOK_Seis_AppData.FirstBreakMs[ch] = -1.0f;
                        WOK_Seis_AppData.PickQuality[ch]  = 0;
                    }
                    WOK_Seis_AppData.EvalV1_mps  = 0.0f;
                    WOK_Seis_AppData.EvalV2_mps  = 0.0f;
                    WOK_Seis_AppData.EvalDepthM  = 0.0f;
                    WOK_Seis_AppData.EvalIceFrac = -1.0f;
                }
                WOK_HAL_Seis_SetSite(Cmd->SiteX_m, Cmd->SiteY_m, Cmd->SiteHeadingDeg);
                WOK_HAL_Seis_Configure(Cmd->SampleRate_Hz, Cmd->Gain, Cmd->ChannelMask);
                WOK_HAL_Seis_StartAcquisition(WOK_Seis_AppData.Duration_sec);
                CFE_EVS_SendEvent(WOK_SEIS_ACQ_START_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEIS: Acq %us @ %uHz gain=%u ch=0x%02X site=(%.1f,%.1f)",
                                  WOK_Seis_AppData.Duration_sec, Cmd->SampleRate_Hz,
                                  Cmd->Gain, Cmd->ChannelMask,
                                  (double)Cmd->SiteX_m, (double)Cmd->SiteY_m);
            } break;

        /* ── CC 3: STOP ACQUISITION ──────────────────────────────── */
        case WOK_SEIS_STOP_ACQ_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_NoArgsCmd_t))) {
                WOK_Seis_AppData.LastSeqId = ((WOK_Seis_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                WOK_Seis_AppData.Acquiring = 0;
                WOK_HAL_Seis_StopAcquisition();
                CFE_EVS_SendEvent(WOK_SEIS_ACQ_STOP_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEIS: Acq stopped at %lu samples",
                                  (unsigned long)WOK_Seis_AppData.SamplesAcquired);
            } break;

        /* ── CC 4: THUMP ARM — configure the shot, charge the bank ── */
        case WOK_SEIS_THUMP_ARM_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_NoArgsCmd_t))) {
                WOK_Seis_AppData.LastSeqId = ((WOK_Seis_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                if (WOK_HAL_Src_Configure(WOK_Seis_AppData.SourceMode,
                                          WOK_Seis_AppData.SourceEnergyJ,
                                          WOK_Seis_AppData.SweepF0_Hz,
                                          WOK_Seis_AppData.SweepF1_Hz,
                                          WOK_Seis_AppData.SweepDur_s) != WOK_HAL_OK) {
                    WOK_Seis_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_SEIS_SRC_CFG_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEIS: Source configure rejected — arming with previous shot config");
                }
                if (WOK_HAL_Src_Arm() == WOK_HAL_OK) {
                    CFE_EVS_SendEvent(WOK_SEIS_SRC_ARM_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEIS: Source arming (bank charging)");
                } else {
                    WOK_Seis_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_SEIS_SRC_ARM_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEIS: Source arm failed (not IDLE?)");
                }
            } break;

        /* ── CC 5: THUMP FIRE — discharge (impulse) / start sweep ─── */
        case WOK_SEIS_THUMP_FIRE_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_NoArgsCmd_t))) {
                WOK_Seis_AppData.LastSeqId = ((WOK_Seis_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                if (WOK_HAL_Src_Fire() == WOK_HAL_OK) {
                    /* Latch the shot context that shaped this record —
                     * this is what file headers carry and what selects
                     * the correlation path on acquisition complete. */
                    WOK_Seis_AppData.ShotMode    = WOK_Seis_AppData.SourceMode;
                    WOK_Seis_AppData.ShotEnergyJ = WOK_Seis_AppData.SourceEnergyJ;
                    WOK_Seis_AppData.ShotF0_Hz   = WOK_Seis_AppData.SweepF0_Hz;
                    WOK_Seis_AppData.ShotF1_Hz   = WOK_Seis_AppData.SweepF1_Hz;
                    WOK_Seis_AppData.ShotDur_s   = WOK_Seis_AppData.SweepDur_s;
                    WOK_HAL_Src_GetShotInfo(&WOK_Seis_AppData.SourceForce_N,
                                            &WOK_Seis_AppData.SourceTimestamp_ms);
                    if (WOK_Seis_AppData.SourceMode == WOK_HAL_SRC_MODE_SWEEP) {
                        CFE_EVS_SendEvent(WOK_SEIS_SRC_FIRED_EID, CFE_EVS_EventType_INFORMATION,
                                          "WOK_SEIS: Source FIRED — sweep %.0f-%.0f Hz %.1fs",
                                          (double)WOK_Seis_AppData.ShotF0_Hz,
                                          (double)WOK_Seis_AppData.ShotF1_Hz,
                                          (double)WOK_Seis_AppData.ShotDur_s);
                    } else {
                        CFE_EVS_SendEvent(WOK_SEIS_SRC_FIRED_EID, CFE_EVS_EventType_INFORMATION,
                                          "WOK_SEIS: Source FIRED — %.1fN",
                                          (double)WOK_Seis_AppData.SourceForce_N);
                    }
                } else {
                    WOK_Seis_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_SEIS_SRC_FIRE_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEIS: Source fire failed (not ARMED?)");
                }
            } break;

        /* ── CC 6: THUMP RESET — safe the source back to IDLE ─────── */
        case WOK_SEIS_THUMP_RESET_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_NoArgsCmd_t))) {
                WOK_Seis_AppData.LastSeqId = ((WOK_Seis_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                /* Nominal post-shot reset; from any other state (charging,
                 * armed, mid-sweep) safe the bank through the dump path. */
                if (WOK_HAL_Src_Reset() != WOK_HAL_OK)
                    WOK_HAL_Src_Abort();
                CFE_EVS_SendEvent(WOK_SEIS_SRC_RESET_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEIS: Source reset");
            } break;

        /* ── CC 7: EVALUATE ──────────────────────────────────────── */
        case WOK_SEIS_EVALUATE_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_NoArgsCmd_t))) {
                WOK_Seis_AppData.LastSeqId = ((WOK_Seis_NoArgsCmd_t *)BufPtr)->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                uint8 flag = 0;
                float conf = 0.0f;
                if (WOK_Seis_EvaluateFromPicks(&flag, &conf))
                {
                    /* Verdict derived from the recorded data */
                    CFE_EVS_SendEvent(WOK_SEIS_EVAL_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEIS: Evaluate — flag=%u conf=%.0f%% "
                                      "(v1=%.0f v2=%.0f h1=%.1fm icefrac=%.2f)",
                                      flag, (double)(conf * 100.0f),
                                      (double)WOK_Seis_AppData.EvalV1_mps,
                                      (double)WOK_Seis_AppData.EvalV2_mps,
                                      (double)WOK_Seis_AppData.EvalDepthM,
                                      (double)WOK_Seis_AppData.EvalIceFrac);
                }
                else
                {
                    /* No usable picks — fall back to the HAL evaluate */
                    WOK_HAL_Seis_Evaluate(&flag, &conf);
                    CFE_EVS_SendEvent(WOK_SEIS_EVAL_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEIS: Evaluate (HAL fallback) — flag=%u confidence=%.0f%%",
                                      flag, (double)(conf * 100.0f));
                }
                WOK_Seis_AppData.EvalFlag       = flag;
                WOK_Seis_AppData.EvalConfidence = conf;
            } break;

        /* ── CC 8: DOWNLINK FILE ─────────────────────────────────── */
        case WOK_SEIS_DOWNLINK_FILE_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_DownlinkFileCmd_t))) {
                const WOK_Seis_DownlinkFileCmd_t *Cmd =
                    (const WOK_Seis_DownlinkFileCmd_t *)BufPtr;
                WOK_Seis_AppData.LastSeqId = Cmd->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                WOK_Seis_StartDownlink(Cmd);
            } break;

        /* ── CC 9: CONFIG SOURCE — mode + shot parameters ─────────── */
        case WOK_SEIS_CONFIG_SOURCE_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_ConfigSourceCmd_t))) {
                const WOK_Seis_ConfigSourceCmd_t *Cmd =
                    (const WOK_Seis_ConfigSourceCmd_t *)BufPtr;
                WOK_Seis_AppData.LastSeqId = Cmd->SeqId;
                if (Cmd->SourceMode > WOK_HAL_SRC_MODE_SWEEP)
                {
                    WOK_Seis_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_SEIS_SRC_CFG_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEIS: Bad source mode %u", Cmd->SourceMode);
                    break;
                }
                WOK_Seis_AppData.CmdCounter++;
                WOK_Seis_AppData.SourceMode = Cmd->SourceMode;
                /* Zero-valued parameters keep the current setting */
                if (Cmd->EnergyJ    > 0.0f) WOK_Seis_AppData.SourceEnergyJ = Cmd->EnergyJ;
                if (Cmd->SweepF0_Hz > 0.0f) WOK_Seis_AppData.SweepF0_Hz    = Cmd->SweepF0_Hz;
                if (Cmd->SweepF1_Hz > 0.0f) WOK_Seis_AppData.SweepF1_Hz    = Cmd->SweepF1_Hz;
                if (Cmd->SweepDur_s > 0.0f) WOK_Seis_AppData.SweepDur_s    = Cmd->SweepDur_s;
                if (WOK_Seis_AppData.SweepDur_s > (float)WOK_SRC_SWEEP_MAX_DUR_SEC)
                    WOK_Seis_AppData.SweepDur_s = (float)WOK_SRC_SWEEP_MAX_DUR_SEC;
                CFE_EVS_SendEvent(WOK_SEIS_SRC_CFG_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEIS: Source config — %s, %.0fJ, sweep %.0f-%.0f Hz %.1fs",
                                  WOK_Seis_AppData.SourceMode == WOK_HAL_SRC_MODE_SWEEP
                                      ? "SWEEP" : "IMPULSE",
                                  (double)WOK_Seis_AppData.SourceEnergyJ,
                                  (double)WOK_Seis_AppData.SweepF0_Hz,
                                  (double)WOK_Seis_AppData.SweepF1_Hz,
                                  (double)WOK_Seis_AppData.SweepDur_s);
            } break;

        /* ── CC 10: PLATE RESEAT — lift and re-press the coupling plate ── */
        case WOK_SEIS_PLATE_RESEAT_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_NoArgsCmd_t))) {
                WOK_Seis_AppData.LastSeqId = ((WOK_Seis_NoArgsCmd_t *)BufPtr)->SeqId;
                if (WOK_HAL_Src_ReseatPlate() == WOK_HAL_OK)
                {
                    WOK_Seis_AppData.CmdCounter++;
                    WOK_Seis_AppData.PlateReseatsSite++;
                    WOK_Seis_AppData.PlateReseatTotal++;
                    CFE_EVS_SendEvent(WOK_SEIS_PLATE_RESEAT_EID, CFE_EVS_EventType_INFORMATION,
                                      "WOK_SEIS: Plate reseat %u at this site (%u total)",
                                      WOK_Seis_AppData.PlateReseatsSite,
                                      WOK_Seis_AppData.PlateReseatTotal);
                }
                else
                {
                    WOK_Seis_AppData.ErrCounter++;
                    CFE_EVS_SendEvent(WOK_SEIS_PLATE_RESEAT_ERR_EID, CFE_EVS_EventType_ERROR,
                                      "WOK_SEIS: Plate reseat REJECTED (source busy)");
                }
            } break;

        /* ── CC 11: SET_POD_GEOMETRY — actual placed geometry (CP-3) ── */
        case WOK_SEIS_SET_POD_GEOM_CC:
            if (WOK_Seis_VerifyCmdLength(&BufPtr->Msg, sizeof(WOK_Seis_PodGeomCmd_t))) {
                WOK_Seis_PodGeomCmd_t *Cmd = (WOK_Seis_PodGeomCmd_t *)BufPtr;
                WOK_Seis_AppData.LastSeqId = Cmd->SeqId;
                WOK_Seis_AppData.CmdCounter++;
                for (uint8 p = 0; p < WOK_SEIS_MAX_CHANNELS; p++)
                {
                    WOK_Seis_AppData.PodOffsets[p]  = Cmd->PodOffset_m[p];
                    WOK_Seis_AppData.PodAttempts[p] = Cmd->PodAttempt[p];
                    WOK_HAL_Seis_SetPodGeometry(p, Cmd->PodX_m[p], Cmd->PodY_m[p],
                                                Cmd->PodAttempt[p]);
                }
                WOK_Seis_AppData.PodMarginalMask = Cmd->PodMarginalMask;
                CFE_EVS_SendEvent(WOK_SEIS_POD_GEOM_EID, CFE_EVS_EventType_INFORMATION,
                                  "WOK_SEIS: Pod geometry — offsets %.2f/%.2f/%.2f/%.2f m, "
                                  "reseats %u/%u/%u/%u, marginal 0x%X",
                                  (double)Cmd->PodOffset_m[0], (double)Cmd->PodOffset_m[1],
                                  (double)Cmd->PodOffset_m[2], (double)Cmd->PodOffset_m[3],
                                  Cmd->PodAttempt[0], Cmd->PodAttempt[1],
                                  Cmd->PodAttempt[2], Cmd->PodAttempt[3],
                                  Cmd->PodMarginalMask);
            } break;

        default:
            WOK_Seis_AppData.ErrCounter++;
            break;
    }
}

/* ================================================================== */
/*  Pick-based evaluate: two-layer refraction grid inversion          */
/*                                                                    */
/*  v1 comes from the near pod pair (always on the direct branch at   */
/*  this array geometry). A grid over (v2, h1) predicts each pod's    */
/*  arrival as min(direct, head wave) — branch assignment falls out   */
/*  implicitly. The verdict is the fraction of near-optimal cells     */
/*  whose refractor is ice-fast, so an underdetermined solution ridge */
/*  (e.g. one pod on the refractor) reads honestly as mid confidence  */
/*  instead of a hard yes/no.                                         */
/*                                                                    */
/*  Returns false if the picks can't support a verdict (caller falls  */
/*  back to the HAL evaluate).                                        */
/* ================================================================== */
static bool WOK_Seis_EvaluateFromPicks(uint8 *flag, float *confidence)
{
    const float *x = WOK_Seis_AppData.PodOffsets;
    const float *t = WOK_Seis_AppData.FirstBreakMs;   /* ms after t0 */

    WOK_Seis_AppData.EvalV1_mps  = 0.0f;
    WOK_Seis_AppData.EvalV2_mps  = 0.0f;
    WOK_Seis_AppData.EvalDepthM  = 0.0f;
    WOK_Seis_AppData.EvalIceFrac = -1.0f;

    if (WOK_Seis_AppData.StackFold == 0) return false;
    for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
        if (t[ch] < 0.0f || WOK_Seis_AppData.PickQuality[ch] == 0)
            return false;

    /* Direct-wave velocity from the near pair */
    if (t[1] <= t[0]) return false;
    float v1 = (x[1] - x[0]) / ((t[1] - t[0]) * 0.001f);
    if (v1 < 100.0f || v1 > 600.0f) return false;

    /* Grid search over (v2, h1): find the best fit and, in the same
     * pass, likelihood-weight every cell (w = exp(-SSE/2σ²)) so the
     * ice fraction is the posterior mass on ice-fast refractors — far
     * sharper than a hard misfit cutoff, which admits fast/thick
     * combos that only barely fit. Weights are computed relative to a
     * first-pass minimum to keep exp() in range. */
    float  sigma = WOK_SEIS_EVAL_PICK_SIGMA_MS;
    float  best_sse = 1.0e30f, best_v2 = 0.0f, best_h1 = 0.0f;
    double w_total = 0.0, w_ice = 0.0;

    for (int pass = 0; pass < 2; pass++)
    {
        for (float v2 = WOK_SEIS_EVAL_V2_MIN_MPS; v2 <= WOK_SEIS_EVAL_V2_MAX_MPS;
             v2 += WOK_SEIS_EVAL_V2_STEP_MPS)
        {
            if (v2 <= v1) continue;
            float sin_ic = v1 / v2;
            float cos_ic = sqrtf(1.0f - sin_ic * sin_ic);
            for (float h1 = WOK_SEIS_EVAL_H1_MIN_M; h1 <= WOK_SEIS_EVAL_H1_MAX_M;
                 h1 += WOK_SEIS_EVAL_H1_STEP_M)
            {
                float ti_ms = 2.0f * h1 * cos_ic / v1 * 1000.0f;
                float sse = 0.0f;
                for (uint8 ch = 0; ch < WOK_SEIS_MAX_CHANNELS; ch++)
                {
                    float t_dir  = x[ch] / v1 * 1000.0f;
                    float t_head = x[ch] / v2 * 1000.0f + ti_ms;
                    float t_pred = (t_head < t_dir) ? t_head : t_dir;
                    float r = t[ch] - t_pred;
                    sse += r * r;
                }
                if (pass == 0)
                {
                    if (sse < best_sse)
                    {
                        best_sse = sse;
                        best_v2  = v2;
                        best_h1  = h1;
                    }
                }
                else
                {
                    /* Likelihood relative to the pass-0 minimum (keeps
                     * expf in range) times a uniform-in-slowness prior:
                     * high-v2 cells are nearly indistinguishable
                     * (t_head → intercept), so a uniform-in-v2 grid
                     * would let sheer volume masquerade as ice evidence */
                    float w = expf(-(sse - best_sse) / (2.0f * sigma * sigma))
                            * (1000.0f / v2) * (1000.0f / v2);
                    w_total += w;
                    if (v2 > WOK_SEIS_EVAL_V2_ICE_MPS) w_ice += w;
                }
            }
        }
    }
    if (best_v2 == 0.0f || w_total <= 0.0) return false;

    float frac = (float)(w_ice / w_total);
    /* Light modulation by stack quality: poor coherence caps confidence */
    float coh = WOK_Seis_AppData.StackCoherence;
    if (coh > 1.0f) coh = 1.0f;
    float conf = frac * (0.7f + 0.3f * coh);
    if (conf < 0.05f) conf = 0.05f;
    if (conf > 0.98f) conf = 0.98f;

    WOK_Seis_AppData.EvalV1_mps  = v1;
    WOK_Seis_AppData.EvalV2_mps  = best_v2;
    WOK_Seis_AppData.EvalDepthM  = best_h1;
    WOK_Seis_AppData.EvalIceFrac = frac;

    *flag = (frac >= WOK_SEIS_EVAL_ICE_FRAC_MIN) ? 1 : 0;
    *confidence = conf;
    return true;
}

/* ================================================================== */
/*  File downlink — chunked streaming over the SB, paced per HK tick  */
/* ================================================================== */
static void WOK_Seis_SendChunk(uint32 offset)
{
    uint32 remain = WOK_Seis_AppData.DownlinkSize - offset;
    uint16 len = (remain > WOK_SEIS_CHUNK_DATA_LEN)
               ? WOK_SEIS_CHUNK_DATA_LEN : (uint16)remain;

    WOK_Seis_AppData.ChunkTlm.FileSize = WOK_Seis_AppData.DownlinkSize;
    WOK_Seis_AppData.ChunkTlm.Offset   = offset;
    WOK_Seis_AppData.ChunkTlm.DataLen  = len;
    memcpy(WOK_Seis_AppData.ChunkTlm.Data,
           &WOK_Seis_AppData.DownlinkBuf[offset], len);

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Seis_AppData.ChunkTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seis_AppData.ChunkTlm.TlmHeader), true);
}

static void WOK_Seis_StartDownlink(const WOK_Seis_DownlinkFileCmd_t *Cmd)
{
    char name[WOK_SEIS_FILENAME_LEN + 1];

    memcpy(name, Cmd->FileName, WOK_SEIS_FILENAME_LEN);
    name[WOK_SEIS_FILENAME_LEN] = '\0';

    /* (Re)load unless this file is already in the downlink buffer —
     * chunk-by-chunk requests must not re-read per chunk */
    if (strncmp(name, WOK_Seis_AppData.ChunkTlm.FileName,
                WOK_SEIS_FILENAME_LEN) != 0 ||
        WOK_Seis_AppData.DownlinkSize == 0)
    {
        uint32 size = 0;
        if (WOK_HAL_Storage_ReadFile(name, WOK_Seis_AppData.DownlinkBuf,
                                     sizeof(WOK_Seis_AppData.DownlinkBuf),
                                     &size) != WOK_HAL_OK || size == 0)
        {
            WOK_Seis_AppData.ErrCounter++;
            CFE_EVS_SendEvent(WOK_SEIS_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                              "WOK_SEIS: Downlink — cannot read '%s'", name);
            return;
        }
        memset(WOK_Seis_AppData.ChunkTlm.FileName, 0,
               sizeof(WOK_Seis_AppData.ChunkTlm.FileName));
        memcpy(WOK_Seis_AppData.ChunkTlm.FileName, name,
               strlen(name) < WOK_SEIS_FILENAME_LEN ? strlen(name)
                                                    : WOK_SEIS_FILENAME_LEN);
        WOK_Seis_AppData.DownlinkSize = size;
        CFE_EVS_SendEvent(WOK_SEIS_DOWNLINK_EID, CFE_EVS_EventType_INFORMATION,
                          "WOK_SEIS: Downlinking %s (%lu bytes)",
                          name, (unsigned long)WOK_Seis_AppData.DownlinkSize);
    }

    if (Cmd->ChunkIdx == WOK_SEIS_CHUNK_STREAM_ALL)
    {
        /* Fire-and-forget stream, paced by PumpDownlink */
        WOK_Seis_AppData.DownlinkOffset = 0;
        WOK_Seis_AppData.DownlinkActive = 1;
    }
    else
    {
        /* Request/response: exactly this chunk, right now */
        uint32 offset = Cmd->ChunkIdx * WOK_SEIS_CHUNK_DATA_LEN;
        if (offset >= WOK_Seis_AppData.DownlinkSize)
        {
            WOK_Seis_AppData.ErrCounter++;
            return;
        }
        WOK_Seis_SendChunk(offset);
    }
}

static void WOK_Seis_PumpDownlink(void)
{
    if (!WOK_Seis_AppData.DownlinkActive) return;

    for (int i = 0; i < WOK_SEIS_CHUNKS_PER_TICK &&
                    WOK_Seis_AppData.DownlinkOffset < WOK_Seis_AppData.DownlinkSize; i++)
    {
        uint32 remain = WOK_Seis_AppData.DownlinkSize - WOK_Seis_AppData.DownlinkOffset;
        uint16 len = (remain > WOK_SEIS_CHUNK_DATA_LEN)
                   ? WOK_SEIS_CHUNK_DATA_LEN : (uint16)remain;
        WOK_Seis_SendChunk(WOK_Seis_AppData.DownlinkOffset);
        WOK_Seis_AppData.DownlinkOffset += len;
    }

    if (WOK_Seis_AppData.DownlinkOffset >= WOK_Seis_AppData.DownlinkSize)
    {
        WOK_Seis_AppData.DownlinkActive = 0;
        CFE_EVS_SendEvent(WOK_SEIS_DOWNLINK_DONE_EID, CFE_EVS_EventType_INFORMATION,
                          "WOK_SEIS: Downlink complete — %s (%lu bytes)",
                          WOK_Seis_AppData.ChunkTlm.FileName,
                          (unsigned long)WOK_Seis_AppData.DownlinkSize);
    }
}

/* ================================================================== */
void WOK_Seis_ReportHousekeeping(void)
{
    WOK_Seis_AppData.HkTlm.CmdCounter    = WOK_Seis_AppData.CmdCounter;
    WOK_Seis_AppData.HkTlm.ErrCounter    = WOK_Seis_AppData.ErrCounter;
    /* Busy while recording OR correlating: the record isn't a usable
     * product until the correlated trace has been stacked, so the
     * sequencer's RECORD state must keep waiting. */
    WOK_Seis_AppData.HkTlm.Acquiring     = (uint8)(WOK_Seis_AppData.Acquiring
                                          || WOK_Seis_AppData.CorrActive);
    WOK_Seis_AppData.HkTlm.SourceState  = WOK_Seis_AppData.SourceState;
    WOK_Seis_AppData.HkTlm.FaultMask     = WOK_Seis_AppData.FaultMask;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Seis_AppData.HkTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seis_AppData.HkTlm.TlmHeader), true);
}

void WOK_Seis_PublishStatus(void)
{
    WOK_Seis_AppData.StatusTlm.Acquiring         = (uint8)(WOK_Seis_AppData.Acquiring
                                                  || WOK_Seis_AppData.CorrActive);
    WOK_Seis_AppData.StatusTlm.QC_ClipFlag       = WOK_Seis_AppData.QC_ClipFlag;
    WOK_Seis_AppData.StatusTlm.QC_DeadChanMask   = WOK_Seis_AppData.QC_DeadChanMask;
    WOK_Seis_AppData.StatusTlm.QC_DropoutFlag    = WOK_Seis_AppData.QC_DropoutFlag;
    WOK_Seis_AppData.StatusTlm.SamplesAcquired_k = (uint16)(WOK_Seis_AppData.SamplesAcquired / 1000);
    WOK_Seis_AppData.StatusTlm.Duration_sec      = WOK_Seis_AppData.Duration_sec;
    WOK_Seis_AppData.StatusTlm.SampleRate_Hz     = WOK_Seis_AppData.SampleRate_Hz;
    WOK_Seis_AppData.StatusTlm.FaultMask         = WOK_Seis_AppData.FaultMask;
    WOK_Seis_AppData.StatusTlm.SourceState      = WOK_Seis_AppData.SourceState;
    WOK_Seis_AppData.StatusTlm.EvalFlag          = WOK_Seis_AppData.EvalFlag;
    WOK_Seis_AppData.StatusTlm.SeqId             = WOK_Seis_AppData.LastSeqId;
    WOK_Seis_AppData.StatusTlm.EvalConfidence    = WOK_Seis_AppData.EvalConfidence;
    WOK_Seis_AppData.StatusTlm.SourceForce_N    = WOK_Seis_AppData.SourceForce_N;
    WOK_Seis_AppData.StatusTlm.StrikeSampleIdx   = WOK_Seis_AppData.StrikeSampleIdx;
    WOK_Seis_AppData.StatusTlm.StrikeValid       = WOK_Seis_AppData.StrikeValid;
    WOK_Seis_AppData.StatusTlm.TraceSamples      = WOK_Seis_AppData.TraceCount;
    memcpy(WOK_Seis_AppData.StatusTlm.FirstBreakMs, WOK_Seis_AppData.FirstBreakMs,
           sizeof(WOK_Seis_AppData.StatusTlm.FirstBreakMs));
    memcpy(WOK_Seis_AppData.StatusTlm.PickQuality, WOK_Seis_AppData.PickQuality,
           sizeof(WOK_Seis_AppData.StatusTlm.PickQuality));
    WOK_Seis_AppData.StatusTlm.StackFold = WOK_Seis_AppData.StackFold;
    float cohp = WOK_Seis_AppData.StackCoherence * 100.0f;
    WOK_Seis_AppData.StatusTlm.CoherencePct =
        (uint8)(cohp > 100.0f ? 100 : (cohp < 0.0f ? 0 : cohp));
    WOK_Seis_AppData.StatusTlm.EvalV1_mps  = WOK_Seis_AppData.EvalV1_mps;
    WOK_Seis_AppData.StatusTlm.EvalV2_mps  = WOK_Seis_AppData.EvalV2_mps;
    WOK_Seis_AppData.StatusTlm.EvalDepthM  = WOK_Seis_AppData.EvalDepthM;
    WOK_Seis_AppData.StatusTlm.EvalIceFrac = WOK_Seis_AppData.EvalIceFrac;
    /* EM source status */
    WOK_Seis_AppData.StatusTlm.SourceMode  = WOK_Seis_AppData.SourceMode;
    {
        float soc = WOK_Seis_AppData.BankSOC_pct;
        WOK_Seis_AppData.StatusTlm.BankSOC_pct =
            (uint8)(soc > 100.0f ? 100 : (soc < 0.0f ? 0 : soc));
        float sp = WOK_Seis_AppData.SweepProgress * 100.0f;
        WOK_Seis_AppData.StatusTlm.SweepProgressPct =
            (uint8)(sp > 100.0f ? 100 : (sp < 0.0f ? 0 : sp));
        WOK_Seis_AppData.StatusTlm.CorrProgressPct =
            WOK_Seis_AppData.CorrActive
                ? (uint8)(WOK_Seis_AppData.CorrPos * 100u
                          / WOK_SEIS_STACK_WIN_SAMPS)
                : 100;
    }
    WOK_Seis_AppData.StatusTlm.BankV       = WOK_Seis_AppData.BankV;
    /* Plate coupling metric (CP-2) */
    WOK_Seis_AppData.StatusTlm.PlateRatio       = WOK_Seis_AppData.PlateRatio;
    WOK_Seis_AppData.StatusTlm.PlateBaseline    = WOK_Seis_AppData.PlateBaseline;
    WOK_Seis_AppData.StatusTlm.PlateOk          = WOK_Seis_AppData.PlateOk;
    WOK_Seis_AppData.StatusTlm.PlateReseatCount = WOK_Seis_AppData.PlateReseatTotal;
    WOK_Seis_AppData.StatusTlm.PlateDegraded    = WOK_Seis_AppData.PlateDegradedSite;
    /* Test-shot coupling scores (CP-4) */
    memcpy(WOK_Seis_AppData.StatusTlm.PodScore, WOK_Seis_AppData.PodScore,
           sizeof(WOK_Seis_AppData.StatusTlm.PodScore));
    WOK_Seis_AppData.StatusTlm.PodScoreOkMask = WOK_Seis_AppData.PodScoreOkMask;
    WOK_Seis_AppData.StatusTlm.PodScoreValid  = WOK_Seis_AppData.PodScoreValid;
    WOK_Seis_AppData.StatusTlm.TestShotCount  = WOK_Seis_AppData.TestShotCount;
    /* Sync consistency (CP-5) */
    memcpy(WOK_Seis_AppData.StatusTlm.SyncResidualMs, WOK_Seis_AppData.SyncResidualMs,
           sizeof(WOK_Seis_AppData.StatusTlm.SyncResidualMs));
    WOK_Seis_AppData.StatusTlm.SyncSuspectMask = WOK_Seis_AppData.SyncSuspectMask;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Seis_AppData.StatusTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seis_AppData.StatusTlm.TlmHeader), true);
}

void WOK_Seis_PublishQuicklook(void)
{
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(WOK_Seis_AppData.QuicklookTlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(WOK_Seis_AppData.QuicklookTlm.TlmHeader), true);
}

bool WOK_Seis_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    size_t ActualLength = 0;
    CFE_MSG_GetSize(MsgPtr, &ActualLength);
    if (ActualLength != ExpectedLength) {
        WOK_Seis_AppData.ErrCounter++;
        CFE_EVS_SendEvent(WOK_SEIS_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "WOK_SEIS: Len err: exp %lu got %lu",
                          (unsigned long)ExpectedLength, (unsigned long)ActualLength);
        return false;
    }
    return true;
}
