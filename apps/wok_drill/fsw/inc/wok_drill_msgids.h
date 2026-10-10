/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK Drill Message IDs and Types
** Follows the mission pattern: CMD at 0x18Dx, TLM at 0x08Dx
** (base MIDs live in wok_common/msgs/inc/wok_msgids.h)
*/

#ifndef WOK_DRILL_MSGIDS_H
#define WOK_DRILL_MSGIDS_H

#include "cfe_sb.h"

/* ── Command MIDs ────────────────────────────────────────────────── */
#define WOK_DRILL_CMD_MID         0x18DA   /* Ground/SEQ → DRILL commands  */
#define WOK_DRILL_SEND_HK_MID    0x18DB   /* SCH_LAB → DRILL HK wakeup   */

/* ── Telemetry MIDs ──────────────────────────────────────────────── */
#define WOK_DRILL_HK_TLM_MID     0x08DA   /* DRILL HK telemetry (28 bytes) */
#define WOK_DRILL_STATUS_TLM_MID 0x08DB   /* DRILL status telemetry (48 bytes) */

/* ── Command Codes ───────────────────────────────────────────────── */
#define WOK_DRILL_NOOP_CC          0
#define WOK_DRILL_RESET_CTRS_CC    1
#define WOK_DRILL_DEPLOY_ARM_CC    2
#define WOK_DRILL_STOW_ARM_CC      3
#define WOK_DRILL_START_DRILL_CC   4
#define WOK_DRILL_STOP_DRILL_CC    5
#define WOK_DRILL_START_SAMPLE_CC  6
#define WOK_DRILL_STOP_SAMPLE_CC   7
#define WOK_DRILL_ESTOP_CC         8

/* ── Drill App States (high-level, reported in HK) ───────────────── */
#define WOK_DRILL_STATE_IDLE           0
#define WOK_DRILL_STATE_DEPLOYING      1
#define WOK_DRILL_STATE_ARM_READY      2
#define WOK_DRILL_STATE_DRILLING       3
#define WOK_DRILL_STATE_DRILL_DONE     4
#define WOK_DRILL_STATE_SAMPLING       5
#define WOK_DRILL_STATE_SAMPLE_DONE    6
#define WOK_DRILL_STATE_STOWING        7
#define WOK_DRILL_STATE_FAULT          8

/* ── Drill Fault Bitmask ─────────────────────────────────────────── */
#define WOK_DRILL_FAULT_ARM_JAM        0x0001
#define WOK_DRILL_FAULT_DRILL_STALL    0x0002
#define WOK_DRILL_FAULT_OVER_DEPTH     0x0004
#define WOK_DRILL_FAULT_THERMAL_OT     0x0008
#define WOK_DRILL_FAULT_ARM_TIMEOUT    0x0010
#define WOK_DRILL_FAULT_OVERCURRENT    0x0020

/* ── Telemetry Structures ────────────────────────────────────────── */

/*
** DRILL HK Telemetry (MID 0x08DA, 28 bytes payload after 16-byte CCSDS header)
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;

    uint16_t CmdCounter;
    uint16_t ErrCounter;
    uint8_t  AppState;        /* WOK_DRILL_STATE_* */
    uint8_t  PrevState;
    uint16_t FaultBitmask;
    uint16_t SeqId;              /* Echo of last cmd SeqId */
    uint16_t SpareSeq;
    float    ArmAngleDeg;
    float    DrillDepthM;

} WOK_Drill_HkTlm_t;

/*
** DRILL Status Telemetry (MID 0x08DB, 48 bytes payload after 16-byte CCSDS header)
*/
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;

    /* Arm */
    uint8_t  ArmHalState;     /* WOK_HAL_DRILL_ARM_* */
    uint8_t  Spare1[3];
    float    ArmAngleDeg;
    float    ArmCurrentAmps;

    /* Drill motor */
    uint8_t  DrillHalState;   /* WOK_HAL_DRILL_MOTOR_* */
    uint8_t  Spare2[3];
    float    DrillDepthM;
    float    DrillRPM;
    float    DrillTorqueNm;
    float    DrillCurrentAmps;

    /* Thermal / sampling */
    uint8_t  SampleHalState;  /* WOK_HAL_DRILL_SAMPLE_* */
    uint8_t  SampleQuality;   /* 0=no data, 1=good, 2=marginal, 3=bad */
    uint8_t  Spare3[2];
    float    CellTempC;
    float    VaporPressurePa;
    float    VolatileConcPpm;

    /* Stiffness-log channels (appended v1.1 — prefix decoders unaffected) */
    float    WOB_N;           /* weight-on-bit, Newtons               */
    float    ROP_mps;         /* smoothed rate of penetration, m/s    */

} WOK_Drill_StatusTlm_t;

/* ================================================================== */
/*  Drill log file (per drill cycle: penetration + evolved-gas record) */
/*                                                                    */
/*  Written to the data dir as drill_NNNN.wok after sampling:         */
/*    [WOK_Drill_LogFileHeader_t][WOK_Drill_LogRow_t x RowCount]      */
/*  Rows are sampled at >=500 ms spacing through DRILLING and         */
/*  SAMPLING. The (CellTempC, VolatilePpm) columns during heating form */
/*  the evolved-gas curve — the binding-classification measurement.   */
/*  The (DepthM, WOB, Torque, ROP-by-differencing) columns form the   */
/*  regolith stiffness log. All little-endian, 4-byte aligned.        */
/* ================================================================== */
#define WOK_DRILL_FILE_MAGIC    0x4452494C   /* 'DRIL' */
#define WOK_DRILL_FILE_VERSION  1
#define WOK_DRILL_LOG_MAX_ROWS  1024

typedef struct
{
    uint32_t Magic;
    uint16_t Version;
    uint16_t RowSize;         /* sizeof(WOK_Drill_LogRow_t)           */
    uint16_t RowCount;
    uint16_t SeqId;           /* correlation at START_DRILL           */
    float    SiteX_m;         /* rover pose, mission start frame      */
    float    SiteY_m;
    float    TargetDepthM;
    float    RPM;
    uint16_t HeatSec;
    uint16_t CoolSec;
    uint32_t DrillIndex;      /* monotonic drill-cycle counter        */
    uint32_t Spare;
} WOK_Drill_LogFileHeader_t;

typedef struct
{
    uint32_t T_ms;            /* ms since drill cycle start           */
    uint8_t  Phase;           /* WOK_DRILL_STATE_* at sample time     */
    uint8_t  Spare[3];
    float    DepthM;
    float    WOB_N;
    float    TorqueNm;
    float    CurrentAmps;
    float    CellTempC;
    float    VaporPressurePa;
    float    VolatileConcPpm;
} WOK_Drill_LogRow_t;

/* ── Command Structures ──────────────────────────────────────────── */

/* CC 0: NOOP — no payload */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16_t SeqId;              /* Sequencer correlation ID (0=ground) */
    uint16_t SpareSeq;
} WOK_Drill_NoopCmd_t;

/* CC 1: RESET_CTRS — no payload */
typedef WOK_Drill_NoopCmd_t WOK_Drill_ResetCmd_t;

/* CC 2: DEPLOY_ARM */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16_t SeqId;              /* Sequencer correlation ID (0=ground) */
    uint16_t SpareSeq;
    float TargetAngleDeg;
} WOK_Drill_DeployArmCmd_t;

/* CC 3: STOW_ARM — no payload */
typedef WOK_Drill_NoopCmd_t WOK_Drill_StowArmCmd_t;

/* CC 4: START_DRILL */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16_t SeqId;              /* Sequencer correlation ID (0=ground) */
    uint16_t SpareSeq;
    float DepthM;
    float RPM;
    float SiteX_m;               /* Rover pose at site (start frame) —   */
    float SiteY_m;               /*   stamps drill products for tie-point */
} WOK_Drill_StartDrillCmd_t;

/* CC 5: STOP_DRILL — no payload */
typedef WOK_Drill_NoopCmd_t WOK_Drill_StopDrillCmd_t;

/* CC 6: START_SAMPLE */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16_t SeqId;              /* Sequencer correlation ID (0=ground) */
    uint16_t SpareSeq;
    uint16_t HeatSec;
    uint16_t CoolSec;
} WOK_Drill_StartSampleCmd_t;

/* CC 7: STOP_SAMPLE — no payload */
typedef WOK_Drill_NoopCmd_t WOK_Drill_StopSampleCmd_t;

/* CC 8: EMERGENCY_STOP — no payload */
typedef WOK_Drill_NoopCmd_t WOK_Drill_EstopCmd_t;

#endif /* WOK_DRILL_MSGIDS_H */
