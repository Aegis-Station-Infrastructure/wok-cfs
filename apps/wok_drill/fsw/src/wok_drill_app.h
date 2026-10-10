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
#include "wok_msgids.h"
#include "wok_perfids.h"
#include "wok_platform_cfg.h"
#include "wok_drill_platform_cfg.h"
#include "wok_drill_msgids.h"
#include "wok_drill_events.h"
#include "wok_hal_drill.h"

/* ── Pipe configuration (depth in wok_platform_cfg.h) ────────────── */
#define WOK_DRILL_PIPE_NAME   "WOK_DRILL_PIPE"

/* ── App data ────────────────────────────────────────────────────── */
typedef struct
{
    CFE_SB_PipeId_t  CmdPipe;
    uint32           RunStatus;

    /* HK telemetry packet */
    WOK_Drill_HkTlm_t      HkTlm;

    /* Status telemetry packet */
    WOK_Drill_StatusTlm_t   StatusTlm;

    /* App state tracking */
    uint8  AppState;
    uint8  PrevState;
    uint16 FaultBitmask;

    /* SeqId echo */
    uint16 LastSeqId;

    /* Fault detection timers */
    uint32 ArmCmdStartMs;

    /* Site context from START_DRILL (stamps the drill log file) */
    float    SiteX_m;
    float    SiteY_m;

    /* ROP: smoothed depth rate during DRILLING */
    float    ROP_mps;
    float    PrevDepthM;
    uint32 PrevDepthMs;

    /* Drill cycle log: rows sampled through DRILLING + SAMPLING,
     * archived as drill_NNNN.wok. */
    struct
    {
        WOK_Drill_LogFileHeader_t Hdr;
        WOK_Drill_LogRow_t        Rows[WOK_DRILL_LOG_MAX_ROWS];
    } LogFile;
    uint16 LogRowCount;
    uint8  LogWritten;      /* file already archived this cycle     */
    uint32 LogLastRowMs;
    uint32 LogStartMs;      /* drill-cycle t=0                      */
    uint32 DrillIndex;      /* monotonic drill-cycle counter        */

    /* Configuration (table-driven) */
    float    MaxDepthM;
    float    MaxRPM;
    float    TorqueLimitNm;
    float    CurrentLimitA;
    uint32 ArmTimeoutMs;
    float    MaxCellTempC;

} WOK_Drill_AppData_t;

/* ── Entry point (registered in cfe_es_startup.scr) ──────────────── */
void WOK_Drill_AppMain(void);
bool WOK_Drill_VerifyCmdLength(const CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

#endif /* WOK_DRILL_APP_H */
