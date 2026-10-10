/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
* WOK Mission — Software Bus Message IDs
*
* Allocated at topic offset 0xD0+ to avoid collisions with
* stock Draco apps (TO_LAB=0x80, SAMPLE_APP=0x82, CI_LAB=0x84).
*
* CMD base: 0x18D0  (0x1800 | 0xD0)
* TLM base: 0x08D0  (0x0800 | 0xD0)
************************************************************************/
#ifndef WOK_MSGIDS_H
#define WOK_MSGIDS_H

/* WOK_SEQ (Sequencer) */
#define WOK_SEQ_CMD_MID          0x18D0
#define WOK_SEQ_SEND_HK_MID     0x18D1
#define WOK_SEQ_TLM_MID         0x08D0
#define WOK_SEQ_MODE_TLM_MID    0x08D1

/* WOK_NAV (Navigation) */
#define WOK_NAV_CMD_MID          0x18D2
#define WOK_NAV_SEND_HK_MID     0x18D3
#define WOK_NAV_TLM_MID         0x08D2
#define WOK_NAV_STATUS_TLM_MID  0x08D3

/* WOK_GEO (Geophone) */
#define WOK_GEO_CMD_MID          0x18D4
#define WOK_GEO_SEND_HK_MID     0x18D5
#define WOK_GEO_TLM_MID         0x08D4
#define WOK_GEO_STATUS_TLM_MID  0x08D5

/* WOK_SEIS (Seismic + EM Source) */
#define WOK_SEIS_CMD_MID         0x18D6
#define WOK_SEIS_SEND_HK_MID    0x18D7
#define WOK_SEIS_TLM_MID        0x08D6
#define WOK_SEIS_STATUS_TLM_MID 0x08D7
#define WOK_SEIS_QUICKLOOK_TLM_MID 0x08D8
#define WOK_SEIS_FILECHUNK_TLM_MID 0x08DC  /* bulk file downlink stream */

/* WOK_HK (Health / FDIR) */
#define WOK_HK_CMD_MID           0x18D8
#define WOK_HK_SEND_HK_MID      0x18D9
#define WOK_HK_TLM_MID          0x08D9

/* WOK_DRILL (Drill / Ice Prospector) */
#define WOK_DRILL_CMD_MID        0x18DA
#define WOK_DRILL_SEND_HK_MID   0x18DB
#define WOK_DRILL_HK_TLM_MID    0x08DA
#define WOK_DRILL_STATUS_TLM_MID 0x08DB

#endif /* WOK_MSGIDS_H */
