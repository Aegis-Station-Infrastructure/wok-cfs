/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
* WOK_SEQ — Application Private Header
*
* App data for the mission state machine: waypoint table, per-site
* survey record, cached child-app status, and pre-built child commands.
************************************************************************/
#ifndef WOK_SEQ_APP_H
#define WOK_SEQ_APP_H

#include "cfe.h"
#include "wok_msgids.h"
#include "wok_perfids.h"
#include "wok_mission_types.h"
#include "wok_platform_cfg.h"
#include "wok_seq_msg.h"
#include "wok_seq_events.h"
#include "wok_nav_msg.h"
#include "wok_geo_msg.h"
#include "wok_seis_msg.h"
#include "wok_hk_msg.h"
#include "wok_drill_msgids.h"
#include "wok_drill_platform_cfg.h"   /* drill battery gate, heat/cool, arm angle */

#define WOK_SEQ_PIPE_NAME   "WOK_SEQ_PIPE"

/* Default waypoint table; ground-uploadable via LOAD_WAYPOINT */
#define WOK_SEQ_NUM_WAYPOINTS  7

/* Sequencer drill targets. Depth must punch through the ~1.5-2 m dry
 * surface layer into the ice-bearing zone, or the vapor cell only ever
 * sees dry regolith and no seismic tie point is possible. */
#define WOK_DRILL_DEFAULT_DEPTH_M   2.5f
#define WOK_DRILL_DEFAULT_RPM       200.0f

/* ------------------------------------------------------------------ */
/*  Drill sub-states (within the SEQ DRILL state)                     */
/* ------------------------------------------------------------------ */
typedef enum
{
    WOK_DRILL_SUB_DEPLOY    = 0,  /* Waiting for drill arm deploy     */
    WOK_DRILL_SUB_DRILLING  = 1,  /* Waiting for drill complete       */
    WOK_DRILL_SUB_SAMPLING  = 2,  /* Waiting for sample complete      */
    WOK_DRILL_SUB_STOW      = 3,  /* Waiting for drill arm stow      */
} WOK_DrillSubState_t;

/* Placement sub-states (CP-3, DEPLOY/STOW in per-pod placement mode) */
#define WOK_PLACE_SUB_MOVE    0   /* driving to the pod position       */
#define WOK_PLACE_SUB_POD     1   /* GEO place/retrieve in progress    */
#define WOK_PLACE_SUB_RETURN  2   /* driving back to the shot point    */

/* Verification sub-states (CP-4, WOK_STATE_VERIFY) */
#define WOK_VERIFY_SUB_SHOT    0  /* test shot: arm + fire             */
#define WOK_VERIFY_SUB_WAIT    1  /* record completing → read scores   */
#define WOK_VERIFY_SUB_MOVE    2  /* driving to a failing pod          */
#define WOK_VERIFY_SUB_POD     3  /* commanded reseat in progress      */
#define WOK_VERIFY_SUB_RETURN  4  /* driving back to the shot point    */

/* ------------------------------------------------------------------ */
/*  Application data                                                  */
/* ------------------------------------------------------------------ */
typedef struct
{
    CFE_SB_PipeId_t  CmdPipe;
    uint32           RunStatus;

    /* Telemetry packets */
    WOK_Seq_HkTlm_t    HkTlm;
    WOK_Seq_ModeTlm_t  ModeTlm;

    /* State machine */
    WOK_State_t  State;
    WOK_State_t  PrevState;
    uint16       WaypointIdx;
    uint16       TotalWaypoints;
    uint8        LoopCount;
    uint16       FaultMask;
    bool         EnteredState;     /* true on first cycle of new state */

    /* Drill sub-state (only valid when State == WOK_STATE_DRILL) */
    WOK_DrillSubState_t DrillSubState;

    /* Per-pod placement choreography (CP-3; valid in DEPLOY/STOW when
     * PlacementMode == PER_POD). The sequencer owns the MOVE/PLACE
     * interleave: drive to each pod position (far pod first on deploy,
     * near first on retrieve), command GEO, wait for the pod's
     * terminal state, and finish with a return leg to the shot point. */
    uint8   PlacementMode;      /* WOK_SEQ_PLACEMENT_*                  */
    uint8   PlaceSubState;      /* WOK_PLACE_SUB_*                      */
    int8    PlacePodIdx;        /* pod currently being handled          */
    float   PodPlacedX[4];      /* NAV pose at each pod's placement     */
    float   PodPlacedY[4];

    /* Coupling verification (CP-4, WOK_STATE_VERIFY) */
    uint8   VerifySub;          /* WOK_VERIFY_SUB_*                     */
    uint8   VerifyPod;          /* pod being reseated                   */
    uint8   VerifyReseatsUsed;  /* targeted reseats this site           */
    uint8   VerifyTriedMask;    /* pods already given a targeted reseat */

    /* Ground-recalibration hold (WOK_STATE_HOLD) */
    uint8   HoldRelease;        /* RELEASE received while holding       */
    CFE_TIME_SysTime_t HoldEntryTime;  /* wall-clock timeout reference  */

    /* Evaluate results from SEIS */
    uint8        EvalFlag;         /* 1 = ice signature detected       */
    float        EvalConfidence;   /* 0.0–1.0                          */

    /* Survey-site geometry: NAV pose snapshotted on DEPLOY entry (the
     * rover has arrived and parked). Stamped into SEIS StartAcq and
     * DRILL StartDrill so both data products carry the same site —
     * the basis of the drill-to-seismic tie point. */
    float        SiteX_m;
    float        SiteY_m;
    float        SiteHeadingDeg;

    /* Stacking: strikes completed at the current site. THUMP→RECORD
     * repeats until StrikesDone reaches WOK_SEIS_STACK_TARGET. */
    uint16       StrikesDone;

    /* Plate coupling verification (CP-2, WOK-CP-001): a record
     * that fails SEIS's plate metric is re-shot after a commanded plate
     * reseat, up to WOK_SEIS_MAX_PLATE_RESEAT per site; exhaustion sets
     * AcceptDegraded so the site proceeds flagged rather than stalling. */
    uint8        PlateReseatsSite;
    uint8        AcceptDegraded;
    uint8        SiteDegraded[WOK_SEQ_MAX_WAYPOINTS]; /* per-site record */

    /* Site survey record — the basis of score-then-drill targeting and
     * the tie-point mission-success gate. Indexed by waypoint. */
    float        SiteScore[WOK_SEQ_MAX_WAYPOINTS];   /* EvalIceFrac; <0 = unsurveyed */
    float        SiteRecX[WOK_SEQ_MAX_WAYPOINTS];    /* absolute pose (start frame)  */
    float        SiteRecY[WOK_SEQ_MAX_WAYPOINTS];
    uint8        MissionMode;      /* WOK_SEQ_MODE_*                     */
    uint8        DrillsCompleted;  /* completed drill cycles this run    */
    uint8        ReturnPhase;      /* 0 none, 1 driving back, 2 drilling */
    uint8        ReturnTargetIdx;  /* waypoint being returned to         */

    /* Safety thresholds */
    float        BatteryMinV;
    float        TiltMaxDeg;
    float        TempMaxC;

    /* SeqId enforcement */
    uint16       SeqId;           /* Monotonic counter (0 reserved for ground) */
    uint16       PendingSeqId;    /* SeqId we're waiting to see echoed back    */

    /* Timing */
    uint16       RecordDurationSec;
    uint32       ThumpTickCount;   /* counts ticks in THUMP state      */
    CFE_TIME_SysTime_t ThumpEntryTime; /* wall-clock timeout reference —
                                          SEND_HK tick rate is platform-
                                          dependent, ticks are not seconds */

    /* Waypoint table */
    WOK_Waypoint_t Waypoints[WOK_SEQ_MAX_WAYPOINTS];

    /* Cached child-app status (updated on every status TLM receipt) */
    WOK_Nav_StatusTlm_t   NavStatus;
    WOK_Geo_StatusTlm_t   GeoStatus;
    WOK_Seis_StatusTlm_t  SeisStatus;
    WOK_Hk_HealthTlm_t    HkHealth;
    WOK_Drill_HkTlm_t     DrillHk;
    /* One TLM-length-error event per child stream per boot; counted after */
    uint8                 NavTlmLenErrReported;
    uint8                 GeoTlmLenErrReported;
    uint8                 SeisTlmLenErrReported;
    uint8                 HkTlmLenErrReported;
    uint8                 DrillTlmLenErrReported;

    /* Pre-built command packets for child apps */
    WOK_Nav_GoToCmd_t       NavGoToCmd;
    WOK_Nav_NoArgsCmd_t     NavStopCmd;
    WOK_Geo_NoArgsCmd_t     GeoDeployCmd;
    WOK_Geo_NoArgsCmd_t     GeoStowCmd;
    WOK_Geo_PlacePodCmd_t   GeoPlacePodCmd;
    WOK_Geo_RetrievePodCmd_t GeoRetrievePodCmd;
    WOK_Geo_RetrievePodCmd_t GeoReseatPodCmd;   /* same struct, CC 7   */
    WOK_Seis_PodGeomCmd_t   SeisPodGeomCmd;
    WOK_Seis_StartAcqCmd_t  SeisStartCmd;
    WOK_Seis_NoArgsCmd_t    SeisStopCmd;
    WOK_Seis_NoArgsCmd_t    SeisThumpArmCmd;
    WOK_Seis_NoArgsCmd_t    SeisThumpFireCmd;
    WOK_Seis_NoArgsCmd_t    SeisThumpResetCmd;
    WOK_Seis_NoArgsCmd_t    SeisEvalCmd;
    WOK_Seis_NoArgsCmd_t    SeisPlateReseatCmd;
    WOK_Seis_ConfigSourceCmd_t SeisCfgSrcCmd;

    /* Drill commands (built and sent dynamically) */
    WOK_Drill_DeployArmCmd_t   DrillDeployCmd;
    WOK_Drill_NoopCmd_t        DrillStowCmd;
    WOK_Drill_StartDrillCmd_t  DrillStartCmd;
    WOK_Drill_StartSampleCmd_t DrillSampleCmd;
    WOK_Drill_NoopCmd_t        DrillEstopCmd;

    /* Counters */
    uint16       CmdCounter;
    uint16       ErrCounter;

} WOK_Seq_AppData_t;

/* ------------------------------------------------------------------ */
/*  Prototypes                                                        */
/* ------------------------------------------------------------------ */
void  WOK_Seq_AppMain(void);
int32 WOK_Seq_AppInit(void);
void  WOK_Seq_ProcessCommandPacket(CFE_SB_Buffer_t *BufPtr);
void  WOK_Seq_ProcessGroundCommand(CFE_SB_Buffer_t *BufPtr);
void  WOK_Seq_ReportHousekeeping(void);
void  WOK_Seq_RunStateMachine(void);
void  WOK_Seq_CheckSafetyInhibits(void);
void  WOK_Seq_CheckMissionEnd(void);
void  WOK_Seq_ChangeState(WOK_State_t NewState);

/* Child app command senders */
void  WOK_Seq_SendNavGoTo(float df, float dl);
void  WOK_Seq_SendNavStop(void);
void  WOK_Seq_SendGeoDeploy(void);
void  WOK_Seq_SendGeoStow(void);
void  WOK_Seq_SendSeisStart(bool test_shot);
void  WOK_Seq_SendSeisStop(void);
void  WOK_Seq_SendSeisConfigSource(uint8 mode, float energy_j);
void  WOK_Seq_SendThumpArm(void);
void  WOK_Seq_SendThumpFire(void);
void  WOK_Seq_SendThumpReset(void);
void  WOK_Seq_SendSeisEvaluate(void);
void  WOK_Seq_SendSeisPlateReseat(void);
void  WOK_Seq_SendGeoPlacePod(uint8 pod, float x, float y);
void  WOK_Seq_SendGeoRetrievePod(uint8 pod);
void  WOK_Seq_SendGeoReseatPod(uint8 pod);
void  WOK_Seq_SendSeisPodGeometry(void);
void  WOK_Seq_SendDrillDeploy(void);
void  WOK_Seq_SendDrillStow(void);
void  WOK_Seq_SendDrillStart(void);
void  WOK_Seq_SendDrillSample(void);
void  WOK_Seq_SendDrillEstop(void);

bool  WOK_Seq_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);
bool  WOK_Seq_VerifyTlmLength(const CFE_SB_Buffer_t *BufPtr, size_t ExpectedLength,
                              uint8 *Reported);

#endif /* WOK_SEQ_APP_H */
