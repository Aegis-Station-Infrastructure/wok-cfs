/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK Mission — Shared type definitions
 *
 * Enums and structs that multiple WOK apps need to agree on.
 *
 * Ops loop:
 *   MOVE → [HOLD] → DEPLOY → [VERIFY] → THUMP → RECORD → EVALUATE →
 *   [DRILL] → STOW
 ************************************************************************/
#ifndef WOK_MISSION_TYPES_H
#define WOK_MISSION_TYPES_H

#include "cfe.h"

/* ------------------------------------------------------------------ */
/*  Sequencer states  (the mission-level state machine)               */
/*                                                                    */
/*  IDLE → MOVE → [HOLD] → DEPLOY → [VERIFY] → THUMP → RECORD →      */
/*  EVALUATE → [DRILL if flagged + battery OK] → STOW → MOVE → …     */
/*  → DONE                                                            */
/* ------------------------------------------------------------------ */
typedef enum
{
    WOK_STATE_IDLE      = 0,
    WOK_STATE_MOVE      = 1,
    WOK_STATE_DEPLOY    = 2,   /* Deploy geophone array              */
    WOK_STATE_THUMP     = 3,   /* Arm + fire the seismic source      */
    WOK_STATE_RECORD    = 4,   /* Seismic acquisition running        */
    WOK_STATE_EVALUATE  = 5,   /* Check data against threshold       */
    WOK_STATE_DRILL     = 6,   /* Drill arm deploy→drill→sample→stow */
    WOK_STATE_STOW      = 7,   /* Stow geophone array                */
    WOK_STATE_SAFE      = 8,   /* Fault or abort — awaiting ground   */
    WOK_STATE_DONE      = 9,   /* All waypoints complete             */
    WOK_STATE_VERIFY    = 10,  /* Coupling test shot + targeted pod
                                  reseats (CP-4; placement mode only.
                                  Appended after DONE so existing
                                  state numbering/decoders hold)      */
    WOK_STATE_HOLD      = 11,  /* Ground-recalibration hold after MOVE
                                  at a recal-flagged waypoint: awaits
                                  RELEASE or the hold timeout          */
    WOK_STATE_COUNT
} WOK_State_t;

/* ------------------------------------------------------------------ */
/*  Common fault bit-mask (app-specific bits live in each app header)  */
/* ------------------------------------------------------------------ */
#define WOK_FAULT_NONE          0x0000
#define WOK_FAULT_BATTERY_LOW   0x0001
#define WOK_FAULT_TEMP_HIGH     0x0002
#define WOK_FAULT_TILT_EXCEED   0x0004
#define WOK_FAULT_WATCHDOG      0x0008
#define WOK_FAULT_COMM_LOSS     0x0010

/* ------------------------------------------------------------------ */
/*  Geophone mechanism states                                         */
/* ------------------------------------------------------------------ */
typedef enum
{
    WOK_GEO_STOWED    = 0,
    WOK_GEO_DEPLOYING = 1,
    WOK_GEO_DEPLOYED  = 2,
    WOK_GEO_STOWING   = 3,
    WOK_GEO_FAULT     = 4
} WOK_GeoMechState_t;

/* ------------------------------------------------------------------ */
/*  NAV arrival / mobility status                                     */
/* ------------------------------------------------------------------ */
typedef enum
{
    WOK_NAV_IDLE     = 0,
    WOK_NAV_DRIVING  = 1,
    WOK_NAV_ARRIVED  = 2,
    WOK_NAV_STOPPED  = 3,
    WOK_NAV_FAULT    = 4
} WOK_NavDriveState_t;

/* ------------------------------------------------------------------ */
/*  Body-frame waypoint (relative to mission-start "persistent" frame) */
/*                                                                    */
/*  Each waypoint is an offset from the previous site, expressed in   */
/*  the rover's start-frame body axes:                                */
/*    DeltaForwardM = forward (start-heading direction), + = forward  */
/*    DeltaLateralM = sideways from forward axis,        + = left     */
/*                                                                    */
/*  NAV captures the rover's heading at SEQ START as the start frame  */
/*  origin, then dead-reckons heading drift. Each step is corrected   */
/*  for accumulated drift before being decomposed into                */
/*    (turn-to-bearing, drive-distance, turn-to-restore).             */
/* ------------------------------------------------------------------ */
typedef struct
{
    float   DeltaForwardM;   /* metres along start-frame forward axis */
    float   DeltaLateralM;   /* metres along start-frame lateral axis */
    uint8   DrillFlag;       /* 1 = auto-drill if evaluate flags site */
    uint8   SourceMode;      /* seismic source mode at this site:
                                0 = impulse (default), 1 = sweep      */
    uint8   RecalFlag;       /* 1 = hold after MOVE for ground
                                recalibration                         */
    uint8   Spare;
    float   SourceEnergyJ;   /* impulse bank energy; 0 = app default  */
} WOK_Waypoint_t;

#endif /* WOK_MISSION_TYPES_H */
