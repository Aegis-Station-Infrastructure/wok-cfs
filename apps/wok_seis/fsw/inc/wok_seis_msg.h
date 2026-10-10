/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_SEIS — Message Definitions (commands + telemetry)
 *
 * PUBLIC header: command codes and structures, HK / status / quicklook /
 * file-chunk telemetry, and the on-disk trace and coupling-record file
 * headers that the ground tools decode byte-for-byte.
 ************************************************************************/
#ifndef WOK_SEIS_MSG_H
#define WOK_SEIS_MSG_H

#include "cfe.h"
#include "wok_platform_cfg.h"   /* WOK_SEIS_MAX_CHANNELS */

/* ================================================================== */
/*  Command codes for WOK_SEIS_CMD_MID                                */
/*                                                                    */
/*  CCs 4-6 kept their names and external behavior across the source  */
/*  change (drop-weight -> EM source) for SEQ compatibility:          */
/*  ARM now charges the capacitor bank, FIRE discharges it (impulse)  */
/*  or starts the pilot sweep, RESET safes the bank.                  */
/* ================================================================== */
#define WOK_SEIS_NOOP_CC          0
#define WOK_SEIS_RESET_CTRS_CC    1
#define WOK_SEIS_START_ACQ_CC     2   /* Begin seismic acquisition     */
#define WOK_SEIS_STOP_ACQ_CC      3   /* Abort acquisition early       */
#define WOK_SEIS_THUMP_ARM_CC     4   /* Arm source (charge bank)      */
#define WOK_SEIS_THUMP_FIRE_CC    5   /* Fire source (strike / sweep)  */
#define WOK_SEIS_THUMP_RESET_CC   6   /* Safe source back to idle      */
#define WOK_SEIS_EVALUATE_CC      7   /* Evaluate recorded data        */
#define WOK_SEIS_DOWNLINK_FILE_CC 8   /* Stream a stored data product  */
#define WOK_SEIS_CONFIG_SOURCE_CC 9   /* Set source mode + shot params */
#define WOK_SEIS_PLATE_RESEAT_CC  10  /* Reseat the source coupling plate */
#define WOK_SEIS_SET_POD_GEOM_CC  11  /* Per-pod placed geometry (CP-3)   */

/* ================================================================== */
/*  Command structures                                                */
/* ================================================================== */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
} WOK_Seis_NoArgsCmd_t;

/* StartAcq with configurable parameters */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
    uint16  Duration_sec;        /* Record length in seconds          */
    uint16  SampleRate_Hz;       /* Fs — samples per second per chan  */
    uint8   Gain;                /* ADC gain setting (0-3 typical)    */
    uint8   ChannelMask;         /* Bit mask of active channels       */
    uint16  Flags;               /* WOK_SEIS_ACQFLAG_*                */
    /* Survey-site context (rover pose in the mission start frame) —
     * stamped into the trace file so every record carries its own
     * source-receiver geometry. SEQ fills from NAV pose; 0 from ground. */
    float   SiteX_m;
    float   SiteY_m;
    float   SiteHeadingDeg;
    /* Stacking: which strike of the site this record is (0-based) and
     * how many strikes the site will get. StackIdx==0 clears the stack;
     * the record at StackIdx==StackTarget-1 triggers pick + stack file.
     * (0,1) or (0,0) = unstacked single record. */
    uint16  StackIdx;
    uint16  StackTarget;
} WOK_Seis_StartAcqCmd_t;

/* StartAcq Flags: accept this record into the stack even if it fails
 * plate-coupling verification (SEQ sets it after the per-site reseat
 * budget is exhausted — the site proceeds flagged degraded). */
#define WOK_SEIS_ACQFLAG_ACCEPT_DEGRADED  0x0001
/* This record is a low-energy coupling TEST SHOT (CP-4, WOK-CP-001):
 * compute per-pod coupling scores from it, archive the coupling record,
 * and keep it out of the site stack. */
#define WOK_SEIS_ACQFLAG_TEST_SHOT        0x0002

/* CC 11: SET_POD_GEOMETRY — actual per-pod placement for the current
 * site (CP-3, placement missions only): each pod's placed position
 * [start frame], its source-receiver offset from the shot point, the
 * seat attempt it finally seated at (= reseat count), and which pods
 * seated marginal. Overrides the configured static offsets for the
 * picker, the onboard inversion, and every archived header until the
 * next SET_POD_GEOMETRY. SEQ sends it once per site after placement
 * completes, before the first shot. */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;
    uint16  SpareSeq;
    float   PodX_m[4];
    float   PodY_m[4];
    float   PodOffset_m[4];
    uint8   PodAttempt[4];
    uint8   PodMarginalMask;     /* bit per pod: seat-force gate spent  */
    uint8   SpareG[3];
} WOK_Seis_PodGeomCmd_t;

/* CC 9: CONFIG_SOURCE — select source mode and shot parameters for
 * subsequent ARM/FIRE cycles. Persistent until reconfigured; applied to
 * the hardware at the next ARM. Zero-valued parameters keep the current
 * setting, so ground can e.g. switch mode without respecifying the
 * sweep band. Defaults (platform cfg) preserve the drop-weight (pre-5.0)
 * source's behavior exactly: impulse mode at the 50 J drop-weight-
 * equivalent energy. */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
    uint8   SourceMode;          /* WOK_HAL_SRC_MODE_* (0=impulse,1=sweep) */
    uint8   SpareM[3];
    float   EnergyJ;             /* impulse bank energy; 0 = keep       */
    float   SweepF0_Hz;          /* pilot start frequency; 0 = keep     */
    float   SweepF1_Hz;          /* pilot end frequency;   0 = keep     */
    float   SweepDur_s;          /* pilot duration;        0 = keep     */
} WOK_Seis_ConfigSourceCmd_t;

/* CC 8: DOWNLINK_FILE — send chunks of a stored product (seis_/stack_/
 * corr_/drill_ file) as WOK_SEIS_FILECHUNK_TLM_MID packets.
 *
 * ChunkIdx selects the transfer mode:
 *   WOK_SEIS_CHUNK_STREAM_ALL — fire-and-forget stream, paced per HK
 *     tick. Fast but lossy through a congested TO path.
 *   N — request/response: transmit exactly chunk N immediately. Ground
 *     drives a stop-and-wait loop with retries (robust on lossy links;
 *     the TO_LAB pipe is shallow and drops under load). */
#define WOK_SEIS_FILENAME_LEN   24
#define WOK_SEIS_CHUNK_STREAM_ALL 0xFFFFFFFFu

typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
    char    FileName[WOK_SEIS_FILENAME_LEN];  /* flat name, no path    */
    uint32  ChunkIdx;            /* chunk to send, or STREAM_ALL        */
} WOK_Seis_DownlinkFileCmd_t;

/* ================================================================== */
/*  File-chunk telemetry  (WOK_SEIS_FILECHUNK_TLM_MID)                */
/*  Chunks are paced a few per HK tick; ground reassembles by Offset. */
/* ================================================================== */
#define WOK_SEIS_CHUNK_DATA_LEN 1024

typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    char    FileName[WOK_SEIS_FILENAME_LEN];
    uint32  FileSize;            /* total bytes in the file           */
    uint32  Offset;              /* byte offset of this chunk         */
    uint16  DataLen;             /* valid bytes in Data[]             */
    uint16  Spare;
    uint8   Data[WOK_SEIS_CHUNK_DATA_LEN];
} WOK_Seis_FileChunkTlm_t;

/* ================================================================== */
/*  Housekeeping telemetry  (WOK_SEIS_TLM_MID)                       */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint16  CmdCounter;
    uint16  ErrCounter;
    uint8   Acquiring;           /* 1 = acquisition in progress       */
    uint8   SourceState;        /* WOK_HAL_SrcState_t: 0=IDLE,1=CHARGING,
                                    2=ARMED,3=FIRED,4=SWEEPING         */
    uint16  FaultMask;
} WOK_Seis_HkTlm_t;

/* ================================================================== */
/*  Acquisition status telemetry  (WOK_SEIS_STATUS_TLM_MID)          */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8   Acquiring;           /* 1 = in progress                   */
    uint8   QC_ClipFlag;         /* 1 = clipping detected             */
    uint8   QC_DeadChanMask;     /* bit per channel: 1 = dead         */
    uint8   QC_DropoutFlag;      /* 1 = dropout detected              */
    uint16  SamplesAcquired_k;   /* thousands of samples so far       */
    uint16  Duration_sec;        /* configured duration                */
    uint16  SampleRate_Hz;       /* configured Fs                     */
    uint16  FaultMask;
    /* Source + evaluate status */
    uint8   SourceState;        /* WOK_HAL_SRC_* state                */
    uint8   EvalFlag;            /* 1 = ice signature detected        */
    uint16  SeqId;               /* Echo of last cmd SeqId            */
    float   EvalConfidence;      /* 0.0–1.0 confidence of detection   */
    float   SourceForce_N;      /* Last impulse magnitude            */
    /* Strike-to-trace synchronization (appended — decoders that only
     * unpack the prefix above keep working) */
    uint32  StrikeSampleIdx;     /* t0 as per-channel sample index    */
    uint8   StrikeValid;         /* 1 = strike landed in acq window   */
    uint8   SpareS[3];
    uint32  TraceSamples;        /* samples/chan retained in buffer   */
    /* Stack + first-break picks (appended) */
    float   FirstBreakMs[4];     /* per pod, ms after strike t0; <0 = no pick */
    uint8   PickQuality[4];      /* 0-100 (STA/LTA trigger strength)  */
    uint8   StackFold;           /* strikes accumulated at this site  */
    uint8   CoherencePct;        /* 0-100 mean stack coherence        */
    uint16  SpareP;
    /* Onboard refraction inversion (appended) — the evidence
     * behind EvalFlag/EvalConfidence when picks drive the verdict */
    float   EvalV1_mps;          /* direct-wave (layer 1) velocity    */
    float   EvalV2_mps;          /* best-fit refractor velocity       */
    float   EvalDepthM;          /* best-fit layer-1 thickness        */
    float   EvalIceFrac;         /* share of near-optimal cells that
                                    are ice-fast (0-1); <0 = pick-based
                                    evaluate not run                   */
    /* EM source status (appended) — decoders that only unpack the
     * prefix above keep working */
    uint8   SourceMode;          /* WOK_HAL_SRC_MODE_* configured      */
    uint8   BankSOC_pct;         /* bank charge, % of shot target      */
    uint8   SweepProgressPct;    /* 0-100 while sweeping               */
    uint8   CorrProgressPct;     /* onboard correlation progress; 100
                                    when idle/complete                 */
    float   BankV;               /* capacitor bank voltage             */
    /* Plate coupling metric (appended CP-2, WOK-CP-001) — decoders
     * that only unpack the prefix above keep working */
    float   PlateRatio;          /* last impulse shot: near-pod RMS /
                                    plate force; <0 = not evaluated    */
    float   PlateBaseline;       /* running site-to-site baseline;
                                    0 = not yet seeded                 */
    uint8   PlateOk;             /* 1 = last shot at/above family      */
    uint8   PlateReseatCount;    /* cumulative reseats this boot       */
    uint8   PlateDegraded;       /* 1 = current site accepted degraded */
    uint8   SpareC;
    /* Per-pod coupling scores from the site's latest test shot
     * (appended CP-4) — site-scoped: persist through the stack shots,
     * reset when the site pose changes */
    float   PodScore[4];         /* vs array median; <0 = not computed */
    uint8   PodScoreOkMask;      /* bit per pod: score >= threshold    */
    uint8   PodScoreValid;       /* 1 = a test shot scored this site   */
    uint8   TestShotCount;       /* test shots fired at this site      */
    uint8   SpareT;
    /* Sync/pick-consistency check from the test shot (appended CP-5):
     * per-pod residual against the best-fit two-layer moveout — the
     * known-t0 event observed at every pod checks sync AND coupling */
    float   SyncResidualMs[4];
    uint8   SyncSuspectMask;     /* |residual| > threshold             */
    uint8   SpareY[3];
} WOK_Seis_StatusTlm_t;

/* Ground decoders unpack these at fixed offsets — catch layout drift at
 * compile time (TLM sizes include the 16-byte telemetry header). */
CompileTimeAssert(sizeof(WOK_Seis_HkTlm_t)        == 24,   WOK_Seis_HkTlm_t_WrongSize);
CompileTimeAssert(sizeof(WOK_Seis_StatusTlm_t)    == 152,  WOK_Seis_StatusTlm_t_WrongSize);
CompileTimeAssert(sizeof(WOK_Seis_FileChunkTlm_t) == 1076, WOK_Seis_FileChunkTlm_t_WrongSize);

/* ================================================================== */
/*  Coupling record (coup_NNNN.wok) — CP-4, WOK-CP-001                */
/*                                                                    */
/*  Header-only product, written/overwritten after each coupling test */
/*  shot at a site; NNNN matches the site's stack_NNNN.wok. Carries   */
/*  everything ground processing needs to weight/deweight channels:   */
/*  per-pod test-shot scores, seat attempts, the placement force-gate */
/*  marginal mask, and the site's plate metric state.                 */
/* ================================================================== */
#define WOK_COUP_FILE_MAGIC    0x434F5550u   /* 'COUP' */
#define WOK_COUP_FILE_VERSION  2             /* v2: + sync residuals   */

typedef struct
{
    uint32  Magic;
    uint16  Version;
    uint16  SeqId;
    float   SiteX_m;
    float   SiteY_m;
    float   SiteHeadingDeg;
    float   PodOffsets_m[4];     /* actual source-receiver offsets     */
    float   PodScore[4];         /* vs array median; <0 = not computed */
    uint8   PodScoreOkMask;
    uint8   PodMarginalMask;     /* placement seating-force gate       */
    uint8   PodAttempts[4];      /* final seat attempt per pod         */
    uint8   VerifiedCount;       /* pods at/above the score threshold  */
    uint8   TestShots;           /* test shots fired at this site      */
    /* Plate metric state at archive time (CP-2) */
    uint8   PlateDegraded;
    uint8   PlateReseats;
    uint8   SpareCp[2];          /* keep the floats 4-byte aligned     */
    float   PlateRatio;
    float   PlateBaseline;
    /* v2 (CP-5): sync/pick-consistency check from the test shot */
    float   SyncResidualMs[4];   /* per-pod moveout-fit residual       */
    uint8   SyncSuspectMask;     /* |residual| > threshold             */
    uint8   SpareS2[3];
} WOK_Coup_FileHeader_t;         /* v1: 72 bytes; v2: 92 — little-endian */

CompileTimeAssert(sizeof(WOK_Coup_FileHeader_t) == 92, WOK_Coup_FileHeader_t_WrongSize);

/* ================================================================== */
/*  Quicklook telemetry  (WOK_SEIS_QUICKLOOK_TLM_MID)               */
/*  Down-sampled stats for ground monitoring during acquisition       */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    float   RMS[WOK_SEIS_MAX_CHANNELS];       /* RMS amplitude per channel */
    float   Peak[WOK_SEIS_MAX_CHANNELS];      /* Peak amplitude per channel*/
    float   DC_Offset[WOK_SEIS_MAX_CHANNELS]; /* Mean / DC offset          */
    uint16  SampleRate_Hz;
    uint16  Spare;
} WOK_Seis_QuicklookTlm_t;

CompileTimeAssert(sizeof(WOK_Seis_QuicklookTlm_t) == 68, WOK_Seis_QuicklookTlm_t_WrongSize);

/* ================================================================== */
/*  Trace file (raw per-shot archive, correlated product, AND stacked */
/*  site product)                                                     */
/*                                                                    */
/*  Written to WOK_DATA_DIR:                                          */
/*    seis_NNNN.wok  — after each acquisition: the raw record         */
/*                     (impulse strike, or the full sweep record).    */
/*                     StackFold=1, picks zeroed.                     */
/*    corr_NNNN.wok  — sweep shots only: the equivalent-impulse trace */
/*                     from onboard cross-correlation against the     */
/*                     regenerated pilot. Same NNNN as its raw        */
/*                     seis_NNNN.wok. Covers the stack window;        */
/*                     StrikeSampleIdx is t0's index WITHIN it.       */
/*    stack_NNNN.wok — after the final shot of a site: StackFold=N,   */
/*                     picks from the STA/LTA picker. In sweep mode   */
/*                     the stack is a stack of correlated traces.     */
/*  Layout:                                                           */
/*    [WOK_Seis_FileHeader_t][int32 samples, interleaved by active    */
/*     channel: sample s of active-channel slot c at (s*NumChannels   */
/*     + c)]                                                          */
/*  In corr/stack files the samples are the stack window and          */
/*  StrikeSampleIdx is t0's index WITHIN that window.                 */
/*  All header fields little-endian, 4-byte aligned. v2 = 84 bytes;   */
/*  v3 appends the EM-source shot context (112 bytes total).          */
/* ================================================================== */
#define WOK_SEIS_FILE_VERSION   3

typedef struct
{
    uint32  Magic;              /* WOK_SEIS_FILE_MAGIC ('SEIS')        */
    uint16  Version;            /* WOK_SEIS_FILE_VERSION               */
    uint16  NumChannels;        /* active channels (interleave stride) */
    uint16  SampleRate_Hz;
    uint16  Gain;
    uint32  SamplesPerChan;     /* samples stored per channel          */
    uint32  StrikeSampleIdx;    /* t0 as per-channel sample index      */
    uint8   StrikeValid;        /* 1 = strike landed in this window    */
    uint8   ChannelMask;        /* which physical pods are active      */
    uint16  SeqId;              /* sequencer correlation at StartAcq   */
    float   SourceForce_N;     /* strike impulse magnitude            */
    float   SiteX_m;            /* rover pose, mission start frame     */
    float   SiteY_m;
    float   SiteHeadingDeg;
    float   PodOffsets_m[WOK_SEIS_MAX_CHANNELS]; /* source-receiver     */
    uint32  AcqIndex;           /* monotonic acquisition counter       */
    /* v2: stack + picks */
    uint8   StackFold;          /* strikes summed (1 = raw record)     */
    uint8   CoherencePct;       /* 0-100 mean stack coherence          */
    /* CP-2 (in former SpareV2 — offsets and size unchanged): plate
     * coupling verification record. Raw files: PlateDegraded=1 means
     * THIS record failed verification (and was kept out of the stack
     * unless accepted). Stack files: the site proceeded degraded. */
    uint8   PlateDegraded;
    uint8   PlateReseats;       /* reseat attempts at this site        */
    float   FirstBreakMs[WOK_SEIS_MAX_CHANNELS]; /* <0 = no pick        */
    uint8   PickQuality[WOK_SEIS_MAX_CHANNELS];  /* 0-100               */
    /* v3: EM source shot context. SourceForce_N above carries the
     * shot's peak plate force in both modes. */
    uint8   SourceMode;         /* WOK_HAL_SRC_MODE_* at fire          */
    uint8   SpareV3[3];
    float   SourceEnergyJ;      /* commanded bank energy (impulse)     */
    float   SweepF0_Hz;         /* pilot parameters (sweep; else 0)    */
    float   SweepF1_Hz;
    float   SweepDur_s;
    float   BankV;              /* bank voltage at archive time        */
    float   BankSOC_pct;        /* bank charge % at archive time       */
} WOK_Seis_FileHeader_t;

CompileTimeAssert(sizeof(WOK_Seis_FileHeader_t) == 112, WOK_Seis_FileHeader_t_WrongSize);

#endif /* WOK_SEIS_MSG_H */
