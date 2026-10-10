/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
* WOK_SEIS — Application Private Header
*
* v5.0 — Dual-mode EM source: source config, sweep records, onboard
*        pilot correlation (tick-chunked), correlated-trace product.
* v2.1 — Added thumper control and seismic evaluation
************************************************************************/
#ifndef WOK_SEIS_APP_H
#define WOK_SEIS_APP_H

#include "cfe.h"
#include "wok_msgids.h"
#include "wok_perfids.h"
#include "wok_mission_types.h"
#include "wok_platform_cfg.h"
#include "wok_seis_msg.h"
#include "wok_seis_events.h"

#define WOK_SEIS_PIPE_NAME   "WOK_SEIS_PIPE"
#define WOK_SEIS_PIPE_DEPTH  16

/* Regenerated-pilot buffer: WOK_SRC_SWEEP_MAX_DUR_SEC at the default
 * 4 kHz (64 KB as floats). Longer dur*fs products are clamped (evented). */
#define WOK_SEIS_PILOT_MAX_SAMPS  16384

typedef struct
{
    CFE_SB_PipeId_t          CmdPipe;
    WOK_Seis_HkTlm_t        HkTlm;
    WOK_Seis_StatusTlm_t    StatusTlm;
    WOK_Seis_QuicklookTlm_t QuicklookTlm;

    /* Acquisition state */
    uint8   Acquiring;
    uint8   PrevAcquiring;      /* previous HK cycle, for completion edge */
    uint16  Duration_sec;
    uint16  SampleRate_Hz;
    uint8   Gain;
    uint8   ChannelMask;
    uint32  SamplesAcquired;

    /* Survey-site context (from StartAcq cmd; SEQ fills from NAV pose) */
    float   SiteX_m;
    float   SiteY_m;
    float   SiteHeadingDeg;

    /* Source-receiver offsets used by the picker, the onboard
     * inversion, and archived headers. Static config defaults;
     * SET_POD_GEOMETRY (CP-3 placement missions) overwrites them with
     * the actual placed geometry, plus seating metadata for TLM. */
    float   PodOffsets[WOK_SEIS_MAX_CHANNELS];
    uint8   PodAttempts[WOK_SEIS_MAX_CHANNELS];
    uint8   PodMarginalMask;

    /* Test-shot coupling verification (CP-4). Scores are SITE-scoped:
     * they persist through the site's stack shots (the sequencer's
     * plate-branch disambiguation reads them) and reset with the
     * per-site counters when the site pose changes. */
    uint8   TestShot;            /* current record is a test shot      */
    float   PodScore[WOK_SEIS_MAX_CHANNELS];  /* <0 = not computed     */
    uint8   PodScoreOkMask;
    uint8   PodScoreValid;
    uint8   TestShotCount;       /* test shots at this site            */

    /* Sync/pick-consistency check (CP-5, site-scoped like the scores) */
    float   SyncResidualMs[WOK_SEIS_MAX_CHANNELS];
    uint8   SyncSuspectMask;

    /* Per-strike trace record: header + interleaved int32 samples,
     * written contiguously to storage on acquisition complete. */
    struct
    {
        WOK_Seis_FileHeader_t Hdr;
        int32 Samples[WOK_SEIS_MAX_CHANNELS * WOK_SEIS_TRACE_MAX_SAMPS_PER_CH];
    } TraceFile;
    uint32  TraceCount;         /* samples/chan stored so far            */
    uint8   TraceNumChans;      /* active channels this acquisition      */
    uint8   TraceTruncated;     /* buffer filled before acq end          */
    uint32  AcqIndex;           /* monotonic acquisition/file counter    */

    /* Strike-to-trace sync (from HAL after acquisition) */
    uint32  StrikeSampleIdx;
    uint8   StrikeValid;

    /* QC from the real trace (counted during drain) */
    uint32  ClipCount[WOK_SEIS_MAX_CHANNELS];

    /* Plate coupling metric (CP-2, WOK-CP-001): per impulse shot,
     * near-pod signal RMS / plate peak force, vs a running site-to-site
     * EMA baseline. Records that fail are kept out of the stack unless
     * the AcceptDegraded flag was set on their StartAcq. */
    float   PlateRatio;         /* last shot; <0 = not evaluated         */
    float   PlateBaseline;      /* cross-site EMA; 0 = unseeded          */
    uint8   PlateOk;            /* last shot's verdict (init/default 1)  */
    uint8   AcceptDegraded;     /* from StartAcq flags, per record       */
    uint8   PlateDegradedSite;  /* site accepted a failing plate seat    */
    uint8   PlateReseatsSite;   /* reseats commanded at this site        */
    uint8   PlateReseatTotal;   /* cumulative this boot (TLM)            */
    uint8   AcqSiteValid;       /* PrevAcqSite* hold a real site         */
    float   PrevAcqSiteX_m;     /* site of the previous StartAcq — the   */
    float   PrevAcqSiteY_m;     /*   per-site counters reset on change   */

    /* Source configuration (CONFIG_SOURCE cmd; defaults from platform
     * cfg preserve v4.0 impulse behavior). Applied to the HAL at ARM. */
    uint8   SourceMode;         /* WOK_HAL_SRC_MODE_*                    */
    float   SourceEnergyJ;      /* impulse bank target                   */
    float   SweepF0_Hz;
    float   SweepF1_Hz;
    float   SweepDur_s;

    /* Shot context latched at FIRE (what actually shaped this record) —
     * stamped into file headers. Re-seeded from the config at StartAcq
     * so headers are defined even for shotless records. */
    uint8   ShotMode;
    float   ShotEnergyJ;
    float   ShotF0_Hz, ShotF1_Hz, ShotDur_s;

    /* Bank + sweep telemetry (from HAL each HK tick) */
    float   BankV;
    float   BankSOC_pct;
    float   SweepProgress;      /* 0-1 */

    /* Onboard sweep correlation: the raw record is cross-correlated
     * against the regenerated pilot into CorrFile, chunked at
     * WOK_SEIS_CORR_SAMPS_PER_TICK output samples per HK tick so the
     * main loop never blocks longer than one tick. The correlated
     * (equivalent-impulse) trace is archived as corr_NNNN.wok and is
     * what feeds the site stack in sweep mode. */
    struct
    {
        WOK_Seis_FileHeader_t Hdr;
        int32 Samples[WOK_SEIS_MAX_CHANNELS * WOK_SEIS_STACK_WIN_SAMPS];
    } CorrFile;
    uint8   CorrActive;
    uint32  CorrPos;            /* output samples/chan completed         */
    uint32  PilotSamps;
    int32   CorrLeadSamps;      /* zero-phase -> causal alignment delay  */
    double  PilotEnergy;        /* sum p^2 (correlation normalizer)      */
    float   Pilot[WOK_SEIS_PILOT_MAX_SAMPS];

    /* Site stack: strike-aligned sum over [-PRE,+POST] around t0.
     * Written as stack_NNNN.wok after the site's last strike. In sweep
     * mode the stack accumulates correlated traces. */
    struct
    {
        WOK_Seis_FileHeader_t Hdr;
        int32 Samples[WOK_SEIS_MAX_CHANNELS * WOK_SEIS_STACK_WIN_SAMPS];
    } StackFile;
    uint16  StackIdx;           /* which strike of the site (from cmd)   */
    uint16  StackTarget;        /* strikes planned for the site          */
    uint8   StackFold;          /* strikes actually accumulated          */
    float   StackCoherence;     /* running mean zero-lag coherence 0-1   */
    uint32  StackFileIndex;     /* monotonic stack-file counter          */

    /* First-break picks (STA/LTA on the stack, after final strike) */
    float   FirstBreakMs[WOK_SEIS_MAX_CHANNELS];
    uint8   PickQuality[WOK_SEIS_MAX_CHANNELS];

    /* Onboard refraction inversion results (pick-based evaluate) */
    float   EvalV1_mps;
    float   EvalV2_mps;
    float   EvalDepthM;
    float   EvalIceFrac;        /* <0 = not run (HAL fallback used)     */

    /* File downlink: chunks of DownlinkBuf streamed a few per HK tick.
     * Sized for the largest product: a max-length raw sweep record
     * (112 B header + 40960 x 4 ch x 4 B = 656 KB). */
    WOK_Seis_FileChunkTlm_t ChunkTlm;
    uint8   DownlinkActive;
    uint32  DownlinkSize;
    uint32  DownlinkOffset;
    uint8   DownlinkBuf[768 * 1024];

    /* QC flags */
    uint8   QC_ClipFlag;
    uint8   QC_DeadChanMask;
    uint8   QC_DropoutFlag;
    uint32  LastSampleCount;    /* prev cycle's sample count, for dropout */

    /* Source state (from HAL; field names kept from the thumper era —
     * they are baked into TLM decoders) */
    uint8   ThumperState;       /* WOK_HAL_SRC_* */
    float   ThumperForce_N;     /* Last shot's peak plate force */
    uint32  ThumperTimestamp_ms;

    /* SeqId echo */
    uint16  LastSeqId;

    /* Evaluate results */
    uint8   EvalFlag;           /* 1 = ice signature detected */
    float   EvalConfidence;     /* 0.0–1.0 */

    uint16  FaultMask;
    uint16  CmdCounter;
    uint16  ErrCounter;
} WOK_Seis_AppData_t;

void  WOK_Seis_AppMain(void);
int32 WOK_Seis_AppInit(void);
void  WOK_Seis_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr);
void  WOK_Seis_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr);
void  WOK_Seis_ReportHousekeeping(void);
void  WOK_Seis_PublishStatus(void);
void  WOK_Seis_PublishQuicklook(void);
bool  WOK_Seis_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

#endif /* WOK_SEIS_APP_H */
