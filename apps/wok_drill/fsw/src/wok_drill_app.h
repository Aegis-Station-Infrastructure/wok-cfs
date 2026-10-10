/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK_DRILL — Drill / Ice Prospector Application
** File: wok_drill/fsw/src/wok_drill_app.h
*/

#ifndef WOK_DRILL_APP_H
#define WOK_DRILL_APP_H

#include "cfe.h"
#include "wok_drill_msgids.h"
#include "wok_hal_drill.h"

/* ── App configuration defaults (from wok_platform_cfg.h) ────────── */
#ifndef WOK_DRILL_DEFAULT_MAX_DEPTH_M
#define WOK_DRILL_DEFAULT_MAX_DEPTH_M      5.0f
#endif
#ifndef WOK_DRILL_DEFAULT_MAX_RPM
#define WOK_DRILL_DEFAULT_MAX_RPM          300.0f
#endif
#ifndef WOK_DRILL_DEFAULT_TORQUE_LIMIT_NM
#define WOK_DRILL_DEFAULT_TORQUE_LIMIT_NM  3.0f
#endif
#ifndef WOK_DRILL_DEFAULT_CURRENT_LIMIT_A
#define WOK_DRILL_DEFAULT_CURRENT_LIMIT_A  5.0f
#endif
#ifndef WOK_DRILL_DEFAULT_ARM_TIMEOUT_MS
#define WOK_DRILL_DEFAULT_ARM_TIMEOUT_MS   8000
#endif
#ifndef WOK_DRILL_DEFAULT_HEAT_SEC
#define WOK_DRILL_DEFAULT_HEAT_SEC         60
#endif
#ifndef WOK_DRILL_DEFAULT_COOL_SEC
#define WOK_DRILL_DEFAULT_COOL_SEC         30
#endif
#ifndef WOK_DRILL_DEFAULT_MAX_CELL_TEMP_C
#define WOK_DRILL_DEFAULT_MAX_CELL_TEMP_C  350.0f
#endif
#ifndef WOK_DRILL_DEFAULT_ARM_ANGLE_DEG
#define WOK_DRILL_DEFAULT_ARM_ANGLE_DEG    90.0f
#endif

/* ── Pipe configuration ──────────────────────────────────────────── */
#define WOK_DRILL_PIPE_DEPTH  16
#define WOK_DRILL_PIPE_NAME   "WOK_DRILL_PIPE"

/* ── App data ────────────────────────────────────────────────────── */
typedef struct
{
    CFE_SB_PipeId_t  CmdPipe;

    /* HK telemetry packet */
    WOK_Drill_HkTlm_t      HkTlm;

    /* Status telemetry packet */
    WOK_Drill_StatusTlm_t   StatusTlm;

    /* App state tracking */
    uint8_t  AppState;
    uint8_t  PrevState;
    uint16_t FaultBitmask;

    /* SeqId echo */
    uint16_t LastSeqId;

    /* Fault detection timers */
    uint32_t ArmCmdStartMs;

    /* Site context from START_DRILL (stamps the drill log file) */
    float    SiteX_m;
    float    SiteY_m;

    /* ROP: smoothed depth rate during DRILLING */
    float    ROP_mps;
    float    PrevDepthM;
    uint32_t PrevDepthMs;

    /* Drill cycle log: rows sampled through DRILLING + SAMPLING,
     * archived as drill_NNNN.wok. */
    struct
    {
        WOK_Drill_LogFileHeader_t Hdr;
        WOK_Drill_LogRow_t        Rows[WOK_DRILL_LOG_MAX_ROWS];
    } LogFile;
    uint16_t LogRowCount;
    uint8_t  LogWritten;      /* file already archived this cycle     */
    uint32_t LogLastRowMs;
    uint32_t LogStartMs;      /* drill-cycle t=0                      */
    uint32_t DrillIndex;      /* monotonic drill-cycle counter        */

    /* Configuration (table-driven) */
    float    MaxDepthM;
    float    MaxRPM;
    float    TorqueLimitNm;
    float    CurrentLimitA;
    uint32_t ArmTimeoutMs;
    float    MaxCellTempC;

} WOK_Drill_AppData_t;

/* ── Entry point (registered in cfe_es_startup.scr) ──────────────── */
void WOK_Drill_AppMain(void);

#endif /* WOK_DRILL_APP_H */
