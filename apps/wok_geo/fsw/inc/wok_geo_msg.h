/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_GEO — Message Definitions (commands + telemetry)
 ************************************************************************/
#ifndef WOK_GEO_MSG_H
#define WOK_GEO_MSG_H

#include "cfe.h"
#include "wok_mission_types.h"

/* ================================================================== */
/*  Command codes for WOK_GEO_CMD_MID                                 */
/* ================================================================== */
#define WOK_GEO_NOOP_CC          0
#define WOK_GEO_RESET_CTRS_CC    1
#define WOK_GEO_DEPLOY_CC        2
#define WOK_GEO_STOW_CC          3
#define WOK_GEO_ABORT_CC         4   /* Emergency stop mechanism */
#define WOK_GEO_PLACE_POD_CC     5   /* Place one pod at (x,y) (CP-3)  */
#define WOK_GEO_RETRIEVE_POD_CC  6   /* Recapture one placed pod (CP-3)*/
#define WOK_GEO_RESEAT_POD_CC    7   /* One commanded reseat of a placed
                                        pod (CP-4: test-shot-triggered;
                                        no internal retry — the test
                                        shot is the arbiter). Same
                                        struct as RETRIEVE_POD. */

/* Per-pod app states (CP-3). PLACING covers the whole internal
 * place -> proxy-check -> [reseat]* loop; PLACED_OK / PLACED_MARGINAL
 * are its terminal outcomes (marginal = seating-force gate exhausted
 * its WOK_GEO_MAX_RESEAT budget — flagged, arbitrated by the CP-4
 * test shot, never fatal on its own). */
#define WOK_GEO_POD_CARRIED          0
#define WOK_GEO_POD_PLACING          1
#define WOK_GEO_POD_PLACED_OK        2
#define WOK_GEO_POD_PLACED_MARGINAL  3
#define WOK_GEO_POD_RETRIEVING       4

/* ================================================================== */
/*  Command structures                                                */
/* ================================================================== */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;               /* Sequencer correlation ID (0=ground) */
    uint16  SpareSeq;
} WOK_Geo_NoArgsCmd_t;

/* CC 5: PLACE_POD — place pod `Pod` at surface position (X, Y) in the
 * mission start frame (the rover's pose at the placement stop; SEQ
 * fills from NAV). GEO runs the seat -> force-proxy -> reseat loop
 * internally and lands in PLACED_OK or PLACED_MARGINAL. */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;
    uint16  SpareSeq;
    uint8   Pod;
    uint8   SpareP[3];
    float   X_m;
    float   Y_m;
} WOK_Geo_PlacePodCmd_t;

/* CC 6: RETRIEVE_POD — recapture pod `Pod`; CARRIED again = recapture
 * confirmed (CP-6). */
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  SeqId;
    uint16  SpareSeq;
    uint8   Pod;
    uint8   SpareP[3];
} WOK_Geo_RetrievePodCmd_t;

/* ================================================================== */
/*  Housekeeping telemetry  (WOK_GEO_TLM_MID)                        */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint16  CmdCounter;
    uint16  ErrCounter;
    uint8   MechState;           /* WOK_GeoMechState_t */
    uint8   Spare;
    uint16  FaultMask;
} WOK_Geo_HkTlm_t;

/* ================================================================== */
/*  Mechanism status telemetry  (WOK_GEO_STATUS_TLM_MID)             */
/* ================================================================== */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8   MechState;           /* WOK_GeoMechState_t */
    uint8   EndstopDeployed;     /* 1 = deploy endstop hit  */
    uint8   EndstopStowed;       /* 1 = stow endstop hit    */
    uint8   Spare;
    float   MotorCurrent_A;
    uint16  FaultMask;
    uint16  TimeoutCounter;      /* increments on jam/timeout */
    uint16  SeqId;               /* Echo of last cmd SeqId */
    uint16  SpareSeq;
    /* Per-pod placement (appended CP-3) — decoders that only unpack
     * the prefix above keep working */
    uint8   PodState[4];         /* WOK_GEO_POD_* app states           */
    uint8   PodReseats[4];       /* seat attempts beyond the first     */
    float   PodSeatForce_N[4];   /* latest seating-force reading       */
    uint8   PlacedMask;          /* bit per pod on the ground          */
    uint8   MarginalMask;        /* bit per pod placed marginal        */
    uint8   ActivePod;           /* pod in motion; 0xFF = none         */
    uint8   SpareP;
} WOK_Geo_StatusTlm_t;

#endif /* WOK_GEO_MSG_H */
