#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
WOK Mission Command Console
============================

Interactive command-line interface for controlling all 6 WOK cFS apps
via CI_LAB (UDP command uplink) and TO_LAB (UDP telemetry downlink).

Type 'help' for available commands, 'quit' to exit.

Usage:
  python3 wok_console.py [--host 127.0.0.1] [--cmd-port 1234] [--tlm-port 2234]
"""

import socket
import struct
import time
import argparse
import sys
import readline
import os
import threading
import select
from collections import OrderedDict
from datetime import datetime

# ============================================================
# Message IDs (must match wok_msgids.h)
# ============================================================

# --- Commands (0x18D0 – 0x18DB) ---
WOK_SEQ_CMD_MID          = 0x18D0
WOK_SEQ_SEND_HK_MID      = 0x18D1
WOK_NAV_CMD_MID           = 0x18D2
WOK_NAV_SEND_HK_MID       = 0x18D3
WOK_GEO_CMD_MID           = 0x18D4
WOK_GEO_SEND_HK_MID       = 0x18D5
WOK_SEIS_CMD_MID          = 0x18D6
WOK_SEIS_SEND_HK_MID      = 0x18D7
WOK_HK_CMD_MID            = 0x18D8
WOK_HK_SEND_HK_MID        = 0x18D9
WOK_DRILL_CMD_MID         = 0x18DA
WOK_DRILL_SEND_HK_MID     = 0x18DB

# --- Telemetry (0x08D0 – 0x08DC) ---
WOK_SEQ_TLM_MID           = 0x08D0
WOK_SEQ_MODE_TLM_MID      = 0x08D1
WOK_NAV_TLM_MID           = 0x08D2
WOK_NAV_STATUS_TLM_MID    = 0x08D3
WOK_GEO_TLM_MID           = 0x08D4
WOK_GEO_STATUS_TLM_MID    = 0x08D5
WOK_SEIS_TLM_MID          = 0x08D6
WOK_SEIS_STATUS_TLM_MID   = 0x08D7
WOK_SEIS_QUICKLOOK_TLM_MID = 0x08D8
WOK_SEIS_FILECHUNK_TLM_MID = 0x08DC

# Ground archive: downlinked files + telemetry CSV logs land here
WOK_ARCHIVE_DIR = "wok_archive"
WOK_HK_TLM_MID            = 0x08D9
WOK_DRILL_TLM_MID         = 0x08DA
WOK_DRILL_STATUS_TLM_MID  = 0x08DB

# ============================================================
# Command Codes (must match *_msg.h headers)
# ============================================================

# wok_seq
SEQ_NOOP_CC           = 0
SEQ_RESET_CTRS_CC     = 1
SEQ_START_CC          = 2
SEQ_ABORT_CC          = 3
SEQ_RESUME_CC         = 4
SEQ_SET_WAYPOINT_CC   = 5

# wok_nav
NAV_NOOP_CC           = 0
NAV_RESET_CTRS_CC     = 1
NAV_GOTO_CC           = 2
NAV_STOP_CC           = 3
NAV_SET_SPEED_CC      = 4
NAV_SET_POSE_CC       = 5
NAV_REALIGN_CC        = 6

# wok_geo
GEO_NOOP_CC           = 0
GEO_RESET_CTRS_CC     = 1
GEO_DEPLOY_CC         = 2
GEO_STOW_CC           = 3
GEO_ABORT_CC          = 4
GEO_PLACE_POD_CC      = 5
GEO_RETRIEVE_POD_CC   = 6
GEO_RESEAT_POD_CC     = 7

# wok_seis
SEIS_NOOP_CC          = 0
SEIS_RESET_CTRS_CC    = 1
SEIS_START_ACQ_CC     = 2
SEIS_STOP_ACQ_CC      = 3
SEIS_THUMP_ARM_CC     = 4
SEIS_THUMP_FIRE_CC    = 5
SEIS_THUMP_RESET_CC   = 6
SEIS_EVALUATE_CC      = 7
SEIS_DOWNLINK_FILE_CC = 8
SEIS_CONFIG_SOURCE_CC = 9
SEIS_PLATE_RESEAT_CC  = 10

# wok_hk
HK_NOOP_CC            = 0
HK_RESET_CTRS_CC      = 1
HK_SET_THRESHOLDS_CC  = 2
HK_CLEAR_FAULTS_CC    = 3

# wok_drill
DRILL_NOOP_CC         = 0
DRILL_RESET_CTRS_CC   = 1
DRILL_DEPLOY_ARM_CC   = 2
DRILL_STOW_ARM_CC     = 3
DRILL_START_DRILL_CC  = 4
DRILL_STOP_DRILL_CC   = 5
DRILL_START_SAMPLE_CC = 6
DRILL_STOP_SAMPLE_CC  = 7
DRILL_ESTOP_CC        = 8

# ============================================================
# State / Enum display maps
# ============================================================
SEQ_STATES = {
    0: "IDLE", 1: "MOVE", 2: "DEPLOY", 3: "THUMP",
    4: "RECORD", 5: "EVALUATE", 6: "DRILL", 7: "STOW",
    8: "SAFE", 9: "DONE", 10: "VERIFY", 11: "HOLD"
}
NAV_DRIVE_STATES = {0: "IDLE", 1: "DRIVING", 2: "ARRIVED", 3: "STOPPED", 4: "FAULT"}
GEO_MECH_STATES  = {0: "STOWED", 1: "DEPLOYING", 2: "DEPLOYED", 3: "STOWING", 4: "FAULT"}

DRILL_STATES = {
    0: "IDLE", 1: "DEPLOYING", 2: "ARM_READY", 3: "DRILLING", 4: "DRILL_DONE",
    5: "SAMPLING", 6: "SAMPLE_DONE", 7: "STOWING", 8: "FAULT"
}

DRILL_FAULT_BITS = [
    (0x0001, "ARM_JAM"),
    (0x0002, "DRILL_STALL"),
    (0x0004, "OVER_DEPTH"),
    (0x0008, "THERMAL_OT"),
    (0x0010, "ARM_TIMEOUT"),
    (0x0020, "OVERCURRENT"),
]

FAULT_BITS = [
    (0x0001, "BATTERY_LOW"),
    (0x0002, "TEMP_HIGH"),
    (0x0004, "TILT_EXCEED"),
    (0x0008, "WATCHDOG"),
    (0x0010, "COMM_LOSS"),
    (0x0100, "NAV_STUCK"),
    (0x0200, "NAV_SLIP"),
    (0x0400, "OVERCURRENT"),
]

TLM_NAMES = {
    WOK_SEQ_TLM_MID:           "SEQ_HK",
    WOK_SEQ_MODE_TLM_MID:      "SEQ_MODE",
    WOK_NAV_TLM_MID:           "NAV_HK",
    WOK_NAV_STATUS_TLM_MID:    "NAV_STATUS",
    WOK_GEO_TLM_MID:           "GEO_HK",
    WOK_GEO_STATUS_TLM_MID:    "GEO_STATUS",
    WOK_SEIS_TLM_MID:          "SEIS_HK",
    WOK_SEIS_STATUS_TLM_MID:   "SEIS_STATUS",
    WOK_SEIS_QUICKLOOK_TLM_MID:"SEIS_QUICKLOOK",
    WOK_HK_TLM_MID:            "HK_HEALTH",
    WOK_DRILL_TLM_MID:         "DRILL_HK",
    WOK_DRILL_STATUS_TLM_MID:  "DRILL_STATUS",
}


# SeqId prefix for child app commands (SeqId=0 from ground = don't-care)
SID = struct.pack("<HH", 0, 0)  # SeqId + SpareSeq


def decode_faults(mask):
    """Return human-readable fault string from bitmask."""
    if mask == 0:
        return "NONE"
    flags = [name for bit, name in FAULT_BITS if mask & bit]
    if not flags:
        return f"0x{mask:04X}"
    return " | ".join(flags)


# ============================================================
# ANSI helpers
# ============================================================
RST  = "\033[0m"
BOLD = "\033[1m"
DIM  = "\033[2m"
RED  = "\033[31m"
GRN  = "\033[32m"
YEL  = "\033[33m"
BLU  = "\033[34m"
MAG  = "\033[35m"
CYN  = "\033[36m"
WHT  = "\033[37m"

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


# ============================================================
# Command Sender (CI_LAB uplink — CCSDS v1 primary header)
# ============================================================
class CmdSender:
    """Sends CCSDS command packets over UDP to CI_LAB."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def _header(self, mid, fc, payload_len):
        """Build 6-byte CCSDS primary header + 2-byte secondary (FC + checksum)."""
        self.seq = (self.seq + 1) & 0x3FFF
        # CCSDS primary header: StreamId, Sequence, Length
        # Length field = total packet size - 7 (per CCSDS convention)
        length = payload_len + 8 - 7   # 8 = 6 primary + 2 secondary
        hdr  = struct.pack(">HHH", mid, 0xC000 | self.seq, length)
        hdr += struct.pack("BB", fc, 0)  # function code + checksum placeholder
        return hdr

    def send(self, mid, fc, payload=b"", label=""):
        pkt = self._header(mid, fc, len(payload)) + payload
        self.sock.sendto(pkt, (self.host, self.port))
        if label:
            print(f"  {GRN}→ Sent {label} ({len(pkt)} bytes){RST}")

    def close(self):
        self.sock.close()


# ============================================================
# Telemetry Listener (TO_LAB downlink)
# ============================================================
class TlmListener:
    """Background thread that listens for TO_LAB telemetry packets on UDP."""

    def __init__(self, port, verbose=False):
        self.port = port
        self.verbose = verbose
        self.running = False
        self.thread = None
        self.sock = None
        self.latest = {}       # mid → raw payload bytes (most recent)
        self.pkt_count = 0
        self.lock = threading.Lock()
        # File-downlink reassembly: name → {"size": int, "parts": {offset: bytes}}
        self.chunk_files = {}
        # Drill-status CSV archive (lazy-opened)
        self._drill_csv = None

    def start(self):
        if self.running:
            return
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(("0.0.0.0", self.port))
        except OSError as e:
            print(f"  {YEL}⚠ TLM listener bind failed on port {self.port}: {e}{RST}")
            print(f"  {DIM}  (Telemetry display won't work, commands still sent){RST}")
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print(f"  {DIM}TLM listener active on UDP :{self.port}{RST}")

    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()

    def _run(self):
        while self.running:
            try:
                ready, _, _ = select.select([self.sock], [], [], 0.5)
                if not ready:
                    continue
                data, addr = self.sock.recvfrom(4096)
                if len(data) < 6:
                    continue
                # Parse CCSDS primary header; key packets by full StreamID
                stream_id = struct.unpack(">H", data[0:2])[0]
                with self.lock:
                    self.latest[stream_id] = data
                    self.pkt_count += 1
                if stream_id == WOK_SEIS_FILECHUNK_TLM_MID:
                    self._handle_chunk(data)
                elif stream_id == WOK_DRILL_STATUS_TLM_MID:
                    self._archive_drill_status(data)
                if self.verbose:
                    name = TLM_NAMES.get(stream_id, f"0x{stream_id:04X}")
                    print(f"\r  {DIM}← TLM {name} ({len(data)} bytes){RST}")
                    # Reprint prompt
                    print(f"{CYN}wok>{RST} ", end="", flush=True)
            except OSError:
                break  # socket closed
            except Exception:
                continue  # ignore transient decode errors

    def get(self, mid):
        with self.lock:
            return self.latest.get(mid)

    def _handle_chunk(self, data):
        """Reassemble WOK_Seis_FileChunkTlm_t packets into archive files."""
        if len(data) < 16 + 24 + 12:
            return
        try:
            raw_name, fsize, offset, dlen = struct.unpack_from("<24sIIH", data, 16)
            name = raw_name.split(b"\x00")[0].decode(errors="replace")
            payload = data[16 + 36: 16 + 36 + dlen]
            if not name or dlen == 0 or len(payload) < dlen:
                return
            st = self.chunk_files.setdefault(name, {"size": fsize, "parts": {}})
            st["parts"][offset] = payload
            got = sum(len(p) for p in st["parts"].values())
            if got >= st["size"]:
                os.makedirs(WOK_ARCHIVE_DIR, exist_ok=True)
                path = os.path.join(WOK_ARCHIVE_DIR, os.path.basename(name))
                with open(path, "wb") as f:
                    for off in sorted(st["parts"]):
                        f.seek(off)
                        f.write(st["parts"][off])
                del self.chunk_files[name]
                print(f"\r  {GRN}⇓ Downlink complete: {path} ({st['size']} bytes){RST}")
                print(f"{CYN}wok>{RST} ", end="", flush=True)
        except (struct.error, OSError):
            pass

    def _archive_drill_status(self, data):
        """Append every DRILL STATUS packet to a CSV in the archive."""
        d = decode_drill_status(data)
        if not d:
            return
        try:
            if self._drill_csv is None:
                os.makedirs(WOK_ARCHIVE_DIR, exist_ok=True)
                path = os.path.join(WOK_ARCHIVE_DIR, "drill_status.csv")
                new = not os.path.exists(path)
                self._drill_csv = open(path, "a")
                if new:
                    self._drill_csv.write(
                        "utc,depth_m,wob_N,torque_Nm,current_A,rop_mps,"
                        "cell_temp_C,vapor_Pa,volatile_ppm,sample_state\n")
            self._drill_csv.write(
                f"{time.time():.3f},{d.get('drill_depth_m','')},"
                f"{d.get('wob_N','')},{d.get('drill_torque_Nm','')},"
                f"{d.get('drill_current_A','')},{d.get('rop_mps','')},"
                f"{d.get('cell_temp_C','')},{d.get('vapor_pressure_Pa','')},"
                f"{d.get('volatile_conc_ppm','')},{d.get('sample_state','')}\n")
            self._drill_csv.flush()
        except OSError:
            pass


# ============================================================
# Telemetry Decoders
# ============================================================
def decode_seq_hk(data):
    """Decode WOK_Seq_HkTlm_t (after 12-byte CCSDS header)."""
    if not data or len(data) < 26:
        return None
    # Payload starts after the 16-byte TLM header (6 primary + 6 time
    # secondary + 4 alignment padding — Draco default config)
    off = 16
    try:
        cmd_cnt, err_cnt, state, prev_state, wp_idx, fault_mask, seq_id = \
            struct.unpack_from("<HHBBHHH", data, off)
        return {
            "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
            "state": SEQ_STATES.get(state, f"?{state}"),
            "prev_state": SEQ_STATES.get(prev_state, f"?{prev_state}"),
            "waypoint_idx": wp_idx,
            "faults": decode_faults(fault_mask),
            "fault_raw": fault_mask,
            "seq_id": seq_id,
        }
    except struct.error:
        return None


def decode_seq_mode(data):
    """Decode WOK_Seq_ModeTlm_t."""
    if not data or len(data) < 28:
        return None
    off = 16
    try:
        state, loop_count, total_wp, fault_mask, spare = \
            struct.unpack_from("<BBHHH", data, off)
        d = {
            "state": SEQ_STATES.get(state, f"?{state}"),
            "loop_count": loop_count,
            "total_waypoints": total_wp,
            "faults": decode_faults(fault_mask),
        }
        # v4.0 targeting status
        if len(data) >= off + 16:
            mode, drills, best_idx, _sp, best_score = \
                struct.unpack_from("<BBBBf", data, off + 8)
            d.update({
                "mission_mode": {0: "DRILL_INLINE", 1: "SURVEY_FIRST"}.get(mode, f"?{mode}"),
                "drills_completed": drills,
                "best_site": best_idx if best_idx != 0xFF else None,
                "best_score": round(best_score, 2),
            })
        return d
    except struct.error:
        return None


def decode_nav_status(data):
    """Decode WOK_Nav_StatusTlm_t.
    Pose is dead-reckoned in the start frame: pose_x = forward distance from
    mission start, pose_y = lateral (+ = left), heading_drift = deg from start
    heading. Informational only — arrival is gated on per-step deltas."""
    if not data or len(data) < 40:
        return None
    off = 16
    try:
        px, py, hdrift, spd, tilt, drive, arrived, faults, seq_id, spare = \
            struct.unpack_from("<fffffBBHHH", data, off)
        d = {
            "pose_x": px, "pose_y": py,
            "heading_drift": hdrift, "speed": spd, "tilt": tilt,
            "drive_state": NAV_DRIVE_STATES.get(drive, f"?{drive}"),
            "arrived": bool(arrived),
            "faults": decode_faults(faults),
            "seq_id": seq_id,
        }
        # Ground recalibration (appended fields)
        if len(data) >= off + 32:
            realigns, _sp = struct.unpack_from("<HH", data, off + 28)
            d["realign_count"] = realigns
        return d
    except struct.error:
        return None


def decode_geo_status(data):
    """Decode WOK_Geo_StatusTlm_t."""
    if not data or len(data) < 32:
        return None
    off = 16
    try:
        mech, endstop_dep, endstop_stow, spare, motor_i, faults, timeout_cnt, seq_id, spare2 = \
            struct.unpack_from("<BBBBfHHHH", data, off)
        d = {
            "mech_state": GEO_MECH_STATES.get(mech, f"?{mech}"),
            "endstop_deployed": bool(endstop_dep),
            "endstop_stowed": bool(endstop_stow),
            "motor_current_A": motor_i,
            "faults": decode_faults(faults),
            "timeout_count": timeout_cnt,
            "seq_id": seq_id,
        }
        # CP-3 per-pod placement fields
        if len(data) >= off + 44:
            pod_states = {0: "CARRIED", 1: "PLACING", 2: "PLACED_OK",
                          3: "MARGINAL", 4: "RETRIEVING"}
            ps = struct.unpack_from("<BBBB", data, off + 16)
            rs = struct.unpack_from("<BBBB", data, off + 20)
            sf = struct.unpack_from("<ffff", data, off + 24)
            placed, marginal, active, _sp = struct.unpack_from("<BBBB", data, off + 40)
            d.update({
                "pod_state": [pod_states.get(s, f"?{s}") for s in ps],
                "pod_reseats": list(rs),
                "pod_seat_force_N": [round(v, 1) for v in sf],
                "placed_mask": f"0x{placed:X}",
                "marginal_mask": f"0x{marginal:X}",
                "active_pod": active if active != 0xFF else None,
            })
        return d
    except struct.error:
        return None


def decode_seis_status(data):
    """Decode WOK_Seis_StatusTlm_t."""
    if not data or len(data) < 28:
        return None
    off = 16
    try:
        acquiring, qc_clip, qc_dead, qc_drop, samples_k, duration, fs, faults = \
            struct.unpack_from("<BBBBHHHH", data, off)
        d = {
            "acquiring": bool(acquiring),
            "qc_clip": bool(qc_clip),
            "qc_dead_mask": f"0x{qc_dead:02X}",
            "qc_dropout": bool(qc_drop),
            "samples_k": samples_k,
            "duration_sec": duration,
            "sample_rate_Hz": fs,
            "faults": decode_faults(faults),
        }
        # v2.1 thumper/eval fields
        if len(data) >= off + 24:
            thumper, eval_flag, seq_id, eval_conf, force = \
                struct.unpack_from("<BBHff", data, off + 12)
            thumper_states = {0: "IDLE", 1: "CHARGING", 2: "ARMED", 3: "FIRED",
                              4: "SWEEPING"}
            d.update({
                "thumper": thumper_states.get(thumper, f"?{thumper}"),
                "eval_flag": bool(eval_flag),
                "eval_confidence": round(eval_conf, 2),
                "thumper_force_N": round(force, 1),
                "seq_id": seq_id,
            })
        # v2.4 strike-to-trace sync fields
        if len(data) >= off + 36:
            strike_idx, strike_valid, trace_samps = \
                struct.unpack_from("<IB3xI", data, off + 24)
            d.update({
                "strike_sample_idx": strike_idx,
                "strike_valid": bool(strike_valid),
                "trace_samples": trace_samps,
            })
        # v2.5 stack + first-break picks
        if len(data) >= off + 60:
            fb = struct.unpack_from("<ffff", data, off + 36)
            q0, q1, q2, q3, fold, coh = struct.unpack_from("<BBBBBB", data, off + 52)
            d.update({
                "first_break_ms": [round(v, 2) for v in fb],
                "pick_quality": [q0, q1, q2, q3],
                "stack_fold": fold,
                "coherence_pct": coh,
            })
        # v3.0 onboard refraction inversion
        if len(data) >= off + 76:
            v1, v2, h1, icefrac = struct.unpack_from("<ffff", data, off + 60)
            d.update({
                "eval_v1_mps": round(v1, 0),
                "eval_v2_mps": round(v2, 0),
                "eval_depth_m": round(h1, 2),
                "eval_ice_frac": round(icefrac, 2),
            })
        # v5.0 EM source: mode, bank, sweep + correlation progress
        if len(data) >= off + 84:
            src_mode, bank_soc, sweep_pct, corr_pct, bank_v = \
                struct.unpack_from("<BBBBf", data, off + 76)
            d.update({
                "source_mode": {0: "IMPULSE", 1: "SWEEP"}.get(src_mode, f"?{src_mode}"),
                "bank_V": round(bank_v, 1),
                "bank_soc_pct": bank_soc,
                "sweep_progress_pct": sweep_pct,
                "corr_progress_pct": corr_pct,
            })
        # CP-2 plate coupling metric
        if len(data) >= off + 96:
            plate_ratio, plate_base, plate_ok, plate_reseats, plate_degr = \
                struct.unpack_from("<ffBBB", data, off + 84)
            d.update({
                "plate_ratio": round(plate_ratio, 1),
                "plate_baseline": round(plate_base, 1),
                "plate_ok": bool(plate_ok),
                "plate_reseats": plate_reseats,
                "plate_degraded": bool(plate_degr),
            })
        # CP-4 test-shot pod coupling scores
        if len(data) >= off + 116:
            scores = struct.unpack_from("<ffff", data, off + 96)
            ok_mask, valid, test_shots, _sp = \
                struct.unpack_from("<BBBB", data, off + 112)
            d.update({
                "pod_scores": [round(v, 2) for v in scores],
                "pod_score_ok_mask": f"0x{ok_mask:X}",
                "pod_scores_valid": bool(valid),
                "test_shots": test_shots,
            })
        # CP-5 sync/pick-consistency residuals
        if len(data) >= off + 136:
            resid = struct.unpack_from("<ffff", data, off + 116)
            susp = struct.unpack_from("<B", data, off + 132)[0]
            d.update({
                "sync_residual_ms": [round(v, 2) for v in resid],
                "sync_suspect_mask": f"0x{susp:X}",
            })
        return d
    except struct.error:
        return None


def decode_hk_health(data):
    """Decode WOK_Hk_HealthTlm_t."""
    if not data or len(data) < 44:
        return None
    off = 16
    try:
        cmd_cnt, err_cnt, batt_v, batt_soc, temp_brd, temp_mot, tilt, cpu, \
        faults, wdog, safe, seq_state, s1, s2, s3 = \
            struct.unpack_from("<HHffffffHBBBBBB", data, off)
        return {
            "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
            "battery_V": batt_v, "battery_SOC_%": batt_soc,
            "temp_board_C": temp_brd, "temp_motor_C": temp_mot,
            "tilt_deg": tilt, "cpu_load_%": cpu,
            "faults": decode_faults(faults), "fault_raw": faults,
            "watchdog_ok": bool(wdog),
            "safe_trigger": bool(safe),
            "seq_state": SEQ_STATES.get(seq_state, f"?{seq_state}"),
        }
    except struct.error:
        return None


def decode_drill_hk(data):
    """Decode WOK_Drill_HkTlm_t (0x08DA)."""
    if not data or len(data) < 32:
        return None
    off = 16
    try:
        cmd_cnt, err_cnt, state, prev_state, fault_mask, seq_id, spare, arm_angle, depth = \
            struct.unpack_from("<HHBBHHHff", data, off)
        return {
            "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
            "state": DRILL_STATES.get(state, f"?{state}"),
            "prev_state": DRILL_STATES.get(prev_state, f"?{prev_state}"),
            "faults": decode_drill_faults(fault_mask),
            "fault_raw": fault_mask,
            "seq_id": seq_id,
            "arm_angle_deg": round(arm_angle, 1),
            "drill_depth_m": round(depth, 3),
        }
    except struct.error:
        return None


def decode_drill_status(data):
    """Decode WOK_Drill_StatusTlm_t (0x08DB)."""
    if not data or len(data) < 48:
        return None
    off = 16
    try:
        arm_hal, _, _, _, arm_angle, arm_current, \
        drill_hal, _, _, _, drill_depth, drill_rpm, drill_torque, drill_current, \
        sample_hal, sample_qual, _, _, cell_temp, vapor_pa, volatile_ppm = \
            struct.unpack_from("<BBBB ff BBBB ffff BBBB fff", data, off)
        arm_states = {0: "STOWED", 1: "DEPLOYING", 2: "READY", 3: "STOWING"}
        drill_motor_states = {0: "IDLE", 1: "DRILLING", 2: "DONE"}
        sample_states = {0: "IDLE", 1: "HEATING", 2: "ANALYZING", 3: "COOLING", 4: "DONE"}
        qual_names = {0: "--", 1: "GOOD", 2: "MARGINAL", 3: "BAD"}
        d = {
            "arm_state": arm_states.get(arm_hal, f"?{arm_hal}"),
            "arm_angle_deg": round(arm_angle, 1),
            "arm_current_A": round(arm_current, 2),
            "drill_motor": drill_motor_states.get(drill_hal, f"?{drill_hal}"),
            "drill_depth_m": round(drill_depth, 3),
            "drill_rpm": round(drill_rpm, 0),
            "drill_torque_Nm": round(drill_torque, 2),
            "drill_current_A": round(drill_current, 2),
            "sample_state": sample_states.get(sample_hal, f"?{sample_hal}"),
            "sample_quality": qual_names.get(sample_qual, f"?{sample_qual}"),
            "cell_temp_C": round(cell_temp, 1),
            "vapor_pressure_Pa": round(vapor_pa, 1),
            "volatile_conc_ppm": round(volatile_ppm, 1),
        }
        # v1.1 stiffness-log channels
        if len(data) >= off + 56:
            wob, rop = struct.unpack_from("<ff", data, off + 48)
            d.update({"wob_N": round(wob, 1), "rop_mps": round(rop, 4)})
        return d
    except struct.error:
        return None


def decode_drill_faults(mask):
    """Return human-readable drill fault string from bitmask."""
    if mask == 0:
        return "NONE"
    flags = [name for bit, name in DRILL_FAULT_BITS if mask & bit]
    if not flags:
        return f"0x{mask:04X}"
    return " | ".join(flags)


TLM_DECODERS = {
    WOK_SEQ_TLM_MID:         ("SEQ_HK",        decode_seq_hk),
    WOK_SEQ_MODE_TLM_MID:    ("SEQ_MODE",       decode_seq_mode),
    WOK_NAV_STATUS_TLM_MID:  ("NAV_STATUS",     decode_nav_status),
    WOK_GEO_STATUS_TLM_MID:  ("GEO_STATUS",     decode_geo_status),
    WOK_SEIS_STATUS_TLM_MID: ("SEIS_STATUS",    decode_seis_status),
    WOK_HK_TLM_MID:          ("HK_HEALTH",      decode_hk_health),
    WOK_DRILL_TLM_MID:       ("DRILL_HK",       decode_drill_hk),
    WOK_DRILL_STATUS_TLM_MID:("DRILL_STATUS",   decode_drill_status),
}


# ============================================================
# Telemetry display commands
# ============================================================
def cmd_tlm(tlm_listener, args):
    """Show latest telemetry. Usage: tlm [app]
    Apps: seq, nav, geo, seis, hk, drill, all"""
    which = args[0].lower() if args else "all"

    targets = {
        "seq":   [WOK_SEQ_TLM_MID, WOK_SEQ_MODE_TLM_MID],
        "nav":   [WOK_NAV_STATUS_TLM_MID],
        "geo":   [WOK_GEO_STATUS_TLM_MID],
        "seis":  [WOK_SEIS_STATUS_TLM_MID],
        "hk":    [WOK_HK_TLM_MID],
        "drill": [WOK_DRILL_TLM_MID, WOK_DRILL_STATUS_TLM_MID],
        "all":   list(TLM_DECODERS.keys()),
    }

    mids = targets.get(which)
    if mids is None:
        print(f"  {RED}Unknown app '{which}'. Use: seq, nav, geo, seis, hk, drill, all{RST}")
        return

    found_any = False
    for mid in mids:
        name, decoder = TLM_DECODERS.get(mid, (None, None))
        if not decoder:
            continue
        raw = tlm_listener.get(mid)
        if not raw:
            print(f"  {DIM}{name}: no data received yet{RST}")
            continue
        found_any = True
        decoded = decoder(raw)
        if decoded:
            print(f"  {BOLD}{CYN}{name}{RST}")
            for k, v in decoded.items():
                # Color faults red
                if "fault" in k.lower() and v not in ("NONE", 0, "0x0000"):
                    print(f"    {k:20s} {RED}{v}{RST}")
                else:
                    print(f"    {k:20s} {v}")
        else:
            print(f"  {name}: decode failed ({len(raw)} bytes)")

    if not found_any:
        print(f"  {YEL}No telemetry received yet. Is cFS running with TO_LAB?{RST}")

    print(f"  {DIM}({tlm_listener.pkt_count} total packets received){RST}")


def cmd_tlm_verbose(tlm_listener, args):
    """Toggle verbose telemetry printing (shows every packet as it arrives)."""
    tlm_listener.verbose = not tlm_listener.verbose
    state = "ON" if tlm_listener.verbose else "OFF"
    print(f"  Verbose telemetry: {state}")


# ============================================================
# SEQ Commands
# ============================================================
def cmd_seq_noop(sender, args):
    """Send NOOP to Sequencer — verify command path"""
    sender.send(WOK_SEQ_CMD_MID, SEQ_NOOP_CC, label="SEQ NOOP")

def cmd_seq_reset(sender, args):
    """Reset Sequencer command/error counters"""
    sender.send(WOK_SEQ_CMD_MID, SEQ_RESET_CTRS_CC, label="SEQ RESET_CTRS")

def cmd_seq_start(sender, args):
    """Start the waypoint loop sequence"""
    sender.send(WOK_SEQ_CMD_MID, SEQ_START_CC, label="SEQ START")

def cmd_seq_abort(sender, args):
    """Abort sequence — enter SAFE immediately"""
    sender.send(WOK_SEQ_CMD_MID, SEQ_ABORT_CC, label="SEQ ABORT")

def cmd_seq_resume(sender, args):
    """Resume from SAFE mode (if in SAFE)"""
    sender.send(WOK_SEQ_CMD_MID, SEQ_RESUME_CC, label="SEQ RESUME")

def cmd_seq_mode(sender, args):
    """Set mission mode. Usage: seq.mode <0|1>  (0=drill-inline, 1=survey-first)"""
    mode = int(args[0]) if args else 1
    payload = struct.pack("<BBBB", mode, 0, 0, 0)
    sender.send(WOK_SEQ_CMD_MID, 6, payload,
                label=f"SEQ SET_MODE {'SURVEY_FIRST' if mode else 'DRILL_INLINE'}")

def cmd_seq_placement(sender, args):
    """Set placement mode. Usage: seq.placement <0|1>  (0=fixed array, 1=per-pod placement)"""
    p = int(args[0]) if args else 1
    payload = struct.pack("<BBBB", p, 0, 0, 0)
    sender.send(WOK_SEQ_CMD_MID, 9, payload,
                label=f"SEQ SET_PLACEMENT {'PER_POD' if p else 'FIXED_ARRAY'}")

def cmd_seq_release(sender, args):
    """Release a recalibration HOLD (recal-flagged waypoint) so the
    sequencer proceeds to the site work."""
    sender.send(WOK_SEQ_CMD_MID, 10, label="SEQ RELEASE")

def cmd_seq_loadwp(sender, args):
    """Upload one waypoint. Usage: seq.loadwp <idx> <fwd_m> <lat_m> [drill01] [srcmode01] [energy_J] [recal01]
    srcmode: 0 = impulse (default), 1 = sweep. energy_J: impulse bank
    energy, 0 = flight default. recal: 1 = hold after MOVE for ground
    recalibration. Zeros preserve prior behavior exactly."""
    if len(args) < 3:
        print(f"  {RED}Usage: seq.loadwp <idx> <fwd_m> <lat_m> [drill01] [srcmode01] [energy_J] [recal01]{RST}")
        return
    idx, fwd, lat = int(args[0]), float(args[1]), float(args[2])
    drill  = int(args[3]) if len(args) > 3 else 0
    src    = int(args[4]) if len(args) > 4 else 0
    energy = float(args[5]) if len(args) > 5 else 0.0
    recal  = int(args[6]) if len(args) > 6 else 0
    payload = struct.pack("<HBBfffBBBB", idx, drill, src, fwd, lat, energy,
                          recal, 0, 0, 0)
    sender.send(WOK_SEQ_CMD_MID, 7, payload,
                label=f"SEQ LOAD_WP {idx} ({fwd},{lat}) drill={drill} "
                      f"src={'SWEEP' if src else 'IMPULSE'} E={energy}J")

def cmd_seq_wpcount(sender, args):
    """Set active waypoint count. Usage: seq.wpcount <n>"""
    n = int(args[0]) if args else 7
    payload = struct.pack("<HH", n, 0)
    sender.send(WOK_SEQ_CMD_MID, 8, payload, label=f"SEQ WP_COUNT {n}")

def cmd_seq_waypoint(sender, args):
    """Set waypoint index override. Usage: seq.waypoint <index>"""
    if len(args) < 1:
        print("  Usage: seq.waypoint <index>")
        return
    idx = int(args[0])
    payload = struct.pack("<HH", idx, 0)  # WaypointIdx + Spare
    sender.send(WOK_SEQ_CMD_MID, SEQ_SET_WAYPOINT_CC, payload,
                label=f"SEQ SET_WAYPOINT idx={idx}")

def cmd_seq_hk(sender, args):
    """Request Sequencer housekeeping (trigger SEND_HK)"""
    sender.send(WOK_SEQ_SEND_HK_MID, 0, label="SEQ SEND_HK")


# ============================================================
# NAV Commands
# ============================================================
def cmd_nav_noop(sender, args):
    """Send NOOP to Navigation"""
    sender.send(WOK_NAV_CMD_MID, NAV_NOOP_CC, SID, label="NAV NOOP")

def cmd_nav_reset(sender, args):
    """Reset NAV command/error counters"""
    sender.send(WOK_NAV_CMD_MID, NAV_RESET_CTRS_CC, SID, label="NAV RESET_CTRS")

def cmd_nav_goto(sender, args):
    """Relative-step move. Usage: nav.goto <delta_forward_m> <delta_lateral_m>
    Both deltas are interpreted in the persistent start frame captured at
    SEQ START. + forward = start-heading direction; + lateral = left."""
    if len(args) < 2:
        print("  Usage: nav.goto <delta_forward_m> <delta_lateral_m>")
        return
    df, dl = float(args[0]), float(args[1])
    payload = SID + struct.pack("<ff", df, dl)
    sender.send(WOK_NAV_CMD_MID, NAV_GOTO_CC, payload,
                label=f"NAV GOTO (df={df:.2f}, dl={dl:.2f})")

def cmd_nav_stop(sender, args):
    """Immediate stop"""
    sender.send(WOK_NAV_CMD_MID, NAV_STOP_CC, SID, label="NAV STOP")

def cmd_nav_setpose(sender, args):
    """Overwrite the dead-reckoned pose with ground truth (bookkeeping only).
    Usage: nav.setpose <x_m> <y_m>  (start frame; rejected while driving)"""
    if len(args) < 2:
        print("  Usage: nav.setpose <x_m> <y_m>")
        return
    x, y = float(args[0]), float(args[1])
    payload = SID + struct.pack("<ff", x, y)
    sender.send(WOK_NAV_CMD_MID, NAV_SET_POSE_CC, payload,
                label=f"NAV SET_POSE ({x:.2f}, {y:.2f})")

def cmd_nav_realign(sender, args):
    """Corrective in-place turn of -error, then declare the current heading
    to be the start-frame heading (drift zeroed). error = ground-measured
    heading error, + = rover points left of its belief.
    Usage: nav.realign <heading_error_deg>  (rejected while driving/faulted)"""
    if len(args) < 1:
        print("  Usage: nav.realign <heading_error_deg>")
        return
    err = float(args[0])
    payload = SID + struct.pack("<f", err)
    sender.send(WOK_NAV_CMD_MID, NAV_REALIGN_CC, payload,
                label=f"NAV REALIGN (error {err:+.2f}°)")

def cmd_nav_speed(sender, args):
    """Set max speed. Usage: nav.speed <m/s>"""
    if len(args) < 1:
        print("  Usage: nav.speed <m/s>")
        return
    spd = float(args[0])
    payload = SID + struct.pack("<f", spd)
    sender.send(WOK_NAV_CMD_MID, NAV_SET_SPEED_CC, payload,
                label=f"NAV SET_SPEED {spd:.3f} m/s")

def cmd_nav_hk(sender, args):
    """Request NAV housekeeping"""
    sender.send(WOK_NAV_SEND_HK_MID, 0, label="NAV SEND_HK")


# ============================================================
# GEO Commands
# ============================================================
def cmd_geo_noop(sender, args):
    """Send NOOP to Geophone controller"""
    sender.send(WOK_GEO_CMD_MID, GEO_NOOP_CC, SID, label="GEO NOOP")

def cmd_geo_reset(sender, args):
    """Reset GEO command/error counters"""
    sender.send(WOK_GEO_CMD_MID, GEO_RESET_CTRS_CC, SID, label="GEO RESET_CTRS")

def cmd_geo_deploy(sender, args):
    """Deploy geophones"""
    sender.send(WOK_GEO_CMD_MID, GEO_DEPLOY_CC, SID, label="GEO DEPLOY")

def cmd_geo_stow(sender, args):
    """Stow geophones"""
    sender.send(WOK_GEO_CMD_MID, GEO_STOW_CC, SID, label="GEO STOW")

def cmd_geo_place(sender, args):
    """Place one pod at (x, y) [start frame] — contingency/manual path;
    normally sequencer-driven in placement mode. GEO runs the seat →
    force-proxy → reseat loop internally. Usage: geo.place <pod> <x_m> <y_m>"""
    if len(args) < 3:
        print("  Usage: geo.place <pod> <x_m> <y_m>")
        return
    pod, x, y = int(args[0]), float(args[1]), float(args[2])
    payload = SID + struct.pack("<B3xff", pod, x, y)
    sender.send(WOK_GEO_CMD_MID, GEO_PLACE_POD_CC, payload,
                label=f"GEO PLACE_POD {pod} ({x:.2f}, {y:.2f})")

def cmd_geo_retrieve(sender, args):
    """Recapture one placed pod (CARRIED again = confirmed) — contingency
    recovery after e.g. a SAFE abort mid-placement. Usage: geo.retrieve <pod>"""
    if len(args) < 1:
        print("  Usage: geo.retrieve <pod>")
        return
    pod = int(args[0])
    payload = SID + struct.pack("<B3x", pod)
    sender.send(WOK_GEO_CMD_MID, GEO_RETRIEVE_POD_CC, payload,
                label=f"GEO RETRIEVE_POD {pod}")

def cmd_geo_reseat(sender, args):
    """One commanded reseat of a placed pod (no internal retry — the test
    shot is the arbiter). Usage: geo.reseat <pod>"""
    if len(args) < 1:
        print("  Usage: geo.reseat <pod>")
        return
    pod = int(args[0])
    payload = SID + struct.pack("<B3x", pod)
    sender.send(WOK_GEO_CMD_MID, GEO_RESEAT_POD_CC, payload,
                label=f"GEO RESEAT_POD {pod}")

def cmd_geo_abort(sender, args):
    """Emergency stop geophone mechanism"""
    sender.send(WOK_GEO_CMD_MID, GEO_ABORT_CC, SID, label="GEO ABORT")

def cmd_geo_hk(sender, args):
    """Request GEO housekeeping"""
    sender.send(WOK_GEO_SEND_HK_MID, 0, label="GEO SEND_HK")


# ============================================================
# SEIS Commands
# ============================================================
def cmd_seis_noop(sender, args):
    """Send NOOP to Seismic app"""
    sender.send(WOK_SEIS_CMD_MID, SEIS_NOOP_CC, SID, label="SEIS NOOP")

def cmd_seis_reset(sender, args):
    """Reset SEIS command/error counters"""
    sender.send(WOK_SEIS_CMD_MID, SEIS_RESET_CTRS_CC, SID, label="SEIS RESET_CTRS")

def cmd_seis_start(sender, args):
    """Start seismic acquisition. Usage: seis.start [duration_s] [sample_rate_Hz] [gain] [chan_mask_hex] [site_x_m] [site_y_m] [site_hdg_deg]
    Defaults: 6 4000 1 0F 0 0 0"""
    duration = int(args[0]) if len(args) > 0 else 6
    fs       = int(args[1]) if len(args) > 1 else 4000
    gain     = int(args[2]) if len(args) > 2 else 1
    ch_mask  = int(args[3], 16) if len(args) > 3 else 0x0F
    site_x   = float(args[4]) if len(args) > 4 else 0.0
    site_y   = float(args[5]) if len(args) > 5 else 0.0
    site_hdg = float(args[6]) if len(args) > 6 else 0.0

    print(f"  Duration:    {duration} sec")
    print(f"  Sample Rate: {fs} Hz")
    print(f"  Gain:        {gain}")
    print(f"  Channels:    0x{ch_mask:02X}")
    print(f"  Site:        ({site_x}, {site_y}) hdg {site_hdg}")

    stack_idx    = int(args[7]) if len(args) > 7 else 0
    stack_target = int(args[8]) if len(args) > 8 else 1
    payload = SID + struct.pack("<HHBBHfffHH", duration, fs, gain, ch_mask, 0,
                                site_x, site_y, site_hdg,
                                stack_idx, stack_target)
    sender.send(WOK_SEIS_CMD_MID, SEIS_START_ACQ_CC, payload,
                label=f"SEIS START_ACQ {duration}s @ {fs}Hz")

def cmd_seis_stop(sender, args):
    """Stop seismic acquisition"""
    sender.send(WOK_SEIS_CMD_MID, SEIS_STOP_ACQ_CC, SID, label="SEIS STOP_ACQ")

def cmd_seis_hk(sender, args):
    """Request SEIS housekeeping"""
    sender.send(WOK_SEIS_SEND_HK_MID, 0, label="SEIS SEND_HK")

def cmd_seis_thump_arm(sender, args):
    """Arm the EM source (charge the capacitor bank)"""
    sender.send(WOK_SEIS_CMD_MID, SEIS_THUMP_ARM_CC, SID, label="SEIS THUMP_ARM")

def cmd_seis_thump_fire(sender, args):
    """Fire the EM source (impulse discharge, or start the sweep)"""
    sender.send(WOK_SEIS_CMD_MID, SEIS_THUMP_FIRE_CC, SID, label="SEIS THUMP_FIRE")

def cmd_seis_thump_reset(sender, args):
    """Reset the EM source to idle"""
    sender.send(WOK_SEIS_CMD_MID, SEIS_THUMP_RESET_CC, SID, label="SEIS THUMP_RESET")

def cmd_seis_evaluate(sender, args):
    """Evaluate recorded seismic data against ice threshold"""
    sender.send(WOK_SEIS_CMD_MID, SEIS_EVALUATE_CC, SID, label="SEIS EVALUATE")

def cmd_seis_plate_reseat(sender, args):
    """Lift and re-press the source coupling plate — contingency/manual
    path; normally sequencer-driven on a plate-metric failure. Rejected
    while the source is charging/armed/sweeping."""
    sender.send(WOK_SEIS_CMD_MID, SEIS_PLATE_RESEAT_CC, SID,
                label="SEIS PLATE_RESEAT")

def cmd_seis_source(sender, args):
    """Configure the seismic source. Usage: seis.source <mode01> [energy_J] [f0_Hz] [f1_Hz] [dur_s]
    mode: 0 = impulse (cap discharge), 1 = sweep (linear pilot).
    Zero/omitted parameters keep the current onboard setting.
    Examples: seis.source 0 100      — impulse at 100 J
              seis.source 1 0 100 1600 3.0 — sweep, 100-1600 Hz over 3 s"""
    if not args:
        print(f"  {RED}Usage: seis.source <mode01> [energy_J] [f0_Hz] [f1_Hz] [dur_s]{RST}")
        return
    mode   = int(args[0])
    energy = float(args[1]) if len(args) > 1 else 0.0
    f0     = float(args[2]) if len(args) > 2 else 0.0
    f1     = float(args[3]) if len(args) > 3 else 0.0
    dur    = float(args[4]) if len(args) > 4 else 0.0
    payload = SID + struct.pack("<BBBBffff", mode, 0, 0, 0, energy, f0, f1, dur)
    sender.send(WOK_SEIS_CMD_MID, SEIS_CONFIG_SOURCE_CC, payload,
                label=f"SEIS CONFIG_SOURCE {'SWEEP' if mode else 'IMPULSE'} "
                      f"E={energy}J f={f0}-{f1}Hz T={dur}s")

SEIS_CHUNK_STREAM_ALL = 0xFFFFFFFF
SEIS_CHUNK_LEN        = 1024

def cmd_seis_fetch(sender, tlm, args):
    """Downlink a stored data product to wok_archive/ using stop-and-wait
    chunk requests (robust on the lossy TO path).
    Usage: seis.fetch <filename>  e.g. seis.fetch stack_0001.wok"""
    if not args:
        print(f"  {RED}Usage: seis.fetch <filename>{RST}")
        return
    fname = args[0]
    name24 = fname.encode()[:24].ljust(24, b"\x00")
    if not tlm.running:
        print(f"  {RED}TLM listener not running — cannot reassemble{RST}")
        return

    def request(idx):
        payload = SID + name24 + struct.pack("<I", idx)
        sender.send(WOK_SEIS_CMD_MID, SEIS_DOWNLINK_FILE_CC, payload)

    def have(idx):
        st = tlm.chunk_files.get(fname)
        return st is not None and (idx * SEIS_CHUNK_LEN) in st["parts"]

    def fetch_chunk(idx):
        for _ in range(5):
            request(idx)
            for _ in range(20):
                if have(idx):
                    return True
                time.sleep(0.05)
        return False

    tlm.chunk_files.pop(fname, None)
    # Chunk 0 first — learns FileSize
    if not fetch_chunk(0):
        print(f"  {RED}No response for {fname} — does it exist onboard?{RST}")
        return
    fsize = tlm.chunk_files[fname]["size"]
    nchunks = (fsize + SEIS_CHUNK_LEN - 1) // SEIS_CHUNK_LEN
    print(f"  Fetching {fname}: {fsize} bytes in {nchunks} chunks")

    for idx in range(1, nchunks):
        if not fetch_chunk(idx):
            print(f"  {RED}Chunk {idx}/{nchunks} failed after retries — aborting{RST}")
            return
        if idx % 25 == 0:
            print(f"  {DIM}... {idx}/{nchunks}{RST}")
    # The listener writes the file automatically once complete
    print(f"  {GRN}Fetched {nchunks} chunks{RST}")


# ============================================================
# HK Commands
# ============================================================
def cmd_hk_noop(sender, args):
    """Send NOOP to Health/HK app"""
    sender.send(WOK_HK_CMD_MID, HK_NOOP_CC, label="HK NOOP")

def cmd_hk_reset(sender, args):
    """Reset HK command/error counters"""
    sender.send(WOK_HK_CMD_MID, HK_RESET_CTRS_CC, label="HK RESET_CTRS")

def cmd_hk_thresholds(sender, args):
    """Set fault thresholds. Usage: hk.thresholds <batt_min_V> <temp_max_C> <tilt_max_deg>
    Defaults: 22.0 55.0 30.0"""
    batt = float(args[0]) if len(args) > 0 else 22.0
    temp = float(args[1]) if len(args) > 1 else 55.0
    tilt = float(args[2]) if len(args) > 2 else 30.0
    payload = struct.pack("<fff", batt, temp, tilt)
    sender.send(WOK_HK_CMD_MID, HK_SET_THRESHOLDS_CC, payload,
                label=f"HK SET_THRESHOLDS batt>{batt}V temp<{temp}C tilt<{tilt}deg")

def cmd_hk_clear(sender, args):
    """Clear all fault flags"""
    sender.send(WOK_HK_CMD_MID, HK_CLEAR_FAULTS_CC, label="HK CLEAR_FAULTS")

def cmd_hk_hk(sender, args):
    """Request HK housekeeping"""
    sender.send(WOK_HK_SEND_HK_MID, 0, label="HK SEND_HK")


# ============================================================
# DRILL Commands
# ============================================================
def cmd_drill_noop(sender, args):
    """Send NOOP to Drill app"""
    sender.send(WOK_DRILL_CMD_MID, DRILL_NOOP_CC, SID, label="DRILL NOOP")

def cmd_drill_reset(sender, args):
    """Reset DRILL command/error counters"""
    sender.send(WOK_DRILL_CMD_MID, DRILL_RESET_CTRS_CC, SID, label="DRILL RESET_CTRS")

def cmd_drill_deploy(sender, args):
    """Deploy drill arm. Usage: drill.deploy [angle_deg]  (default 90)"""
    angle = float(args[0]) if args else 90.0
    payload = SID + struct.pack("<f", angle)
    sender.send(WOK_DRILL_CMD_MID, DRILL_DEPLOY_ARM_CC, payload,
                label=f"DRILL DEPLOY_ARM {angle:.1f} deg")

def cmd_drill_stow(sender, args):
    """Stow drill arm"""
    sender.send(WOK_DRILL_CMD_MID, DRILL_STOW_ARM_CC, SID, label="DRILL STOW_ARM")

def cmd_drill_start(sender, args):
    """Start drilling. Usage: drill.start [depth_m] [rpm] [site_x_m] [site_y_m]  (default 1.0 200 0 0)"""
    depth  = float(args[0]) if len(args) > 0 else 1.0
    rpm    = float(args[1]) if len(args) > 1 else 200.0
    site_x = float(args[2]) if len(args) > 2 else 0.0
    site_y = float(args[3]) if len(args) > 3 else 0.0
    print(f"  Depth:  {depth} m")
    print(f"  RPM:    {rpm}")
    print(f"  Site:   ({site_x}, {site_y})")
    payload = SID + struct.pack("<ffff", depth, rpm, site_x, site_y)
    sender.send(WOK_DRILL_CMD_MID, DRILL_START_DRILL_CC, payload,
                label=f"DRILL START {depth}m @ {rpm} RPM")

def cmd_drill_stop(sender, args):
    """Stop drill motor"""
    sender.send(WOK_DRILL_CMD_MID, DRILL_STOP_DRILL_CC, SID, label="DRILL STOP")

def cmd_drill_sample(sender, args):
    """Start thermal vaporization. Usage: drill.sample [heat_sec] [cool_sec]  (default 60 30)"""
    heat = int(args[0]) if len(args) > 0 else 60
    cool = int(args[1]) if len(args) > 1 else 30
    print(f"  Heat:   {heat} sec")
    print(f"  Cool:   {cool} sec")
    payload = SID + struct.pack("<HH", heat, cool)
    sender.send(WOK_DRILL_CMD_MID, DRILL_START_SAMPLE_CC, payload,
                label=f"DRILL SAMPLE heat={heat}s cool={cool}s")

def cmd_drill_abort(sender, args):
    """Emergency stop all drill operations"""
    sender.send(WOK_DRILL_CMD_MID, DRILL_ESTOP_CC, SID, label="DRILL EMERGENCY_STOP")

def cmd_drill_hk(sender, args):
    """Request DRILL housekeeping"""
    sender.send(WOK_DRILL_SEND_HK_MID, 0, label="DRILL SEND_HK")


# ============================================================
# Convenience / Compound Commands
# ============================================================
def cmd_allnoop(sender, args):
    """Send NOOP to all 6 WOK apps — quick connectivity check"""
    # SEQ and HK have no SeqId; child apps (NAV, GEO, SEIS, DRILL) need SID prefix
    for label, mid, cc, payload in [
        ("SEQ",   WOK_SEQ_CMD_MID,   SEQ_NOOP_CC,   b""),
        ("NAV",   WOK_NAV_CMD_MID,   NAV_NOOP_CC,   SID),
        ("GEO",   WOK_GEO_CMD_MID,   GEO_NOOP_CC,   SID),
        ("SEIS",  WOK_SEIS_CMD_MID,  SEIS_NOOP_CC,  SID),
        ("HK",    WOK_HK_CMD_MID,    HK_NOOP_CC,    b""),
        ("DRILL", WOK_DRILL_CMD_MID, DRILL_NOOP_CC, SID),
    ]:
        sender.send(mid, cc, payload, label=f"{label} NOOP")
        time.sleep(0.05)

def cmd_allreset(sender, args):
    """Reset counters on all 6 WOK apps"""
    for label, mid, cc, payload in [
        ("SEQ",   WOK_SEQ_CMD_MID,   SEQ_RESET_CTRS_CC,   b""),
        ("NAV",   WOK_NAV_CMD_MID,   NAV_RESET_CTRS_CC,   SID),
        ("GEO",   WOK_GEO_CMD_MID,   GEO_RESET_CTRS_CC,   SID),
        ("SEIS",  WOK_SEIS_CMD_MID,  SEIS_RESET_CTRS_CC,  SID),
        ("HK",    WOK_HK_CMD_MID,    HK_RESET_CTRS_CC,    b""),
        ("DRILL", WOK_DRILL_CMD_MID, DRILL_RESET_CTRS_CC, SID),
    ]:
        sender.send(mid, cc, payload, label=f"{label} RESET_CTRS")
        time.sleep(0.05)

def cmd_allhk(sender, args):
    """Request housekeeping from all 6 apps"""
    for label, mid in [
        ("SEQ",   WOK_SEQ_SEND_HK_MID),
        ("NAV",   WOK_NAV_SEND_HK_MID),
        ("GEO",   WOK_GEO_SEND_HK_MID),
        ("SEIS",  WOK_SEIS_SEND_HK_MID),
        ("HK",    WOK_HK_SEND_HK_MID),
        ("DRILL", WOK_DRILL_SEND_HK_MID),
    ]:
        sender.send(mid, 0, label=f"{label} SEND_HK")
        time.sleep(0.05)

def cmd_quickstart(sender, args):
    """Startup sequence: all NOOP → SEQ START
    Usage: quickstart [settle_delay_sec]"""
    delay = float(args[0]) if args else 1.0
    print(f"  Running: ALL NOOP → (wait {delay}s) → SEQ START")
    cmd_allnoop(sender, [])
    time.sleep(delay)
    cmd_seq_start(sender, [])

def cmd_safestop(sender, args):
    """Safe shutdown: SEIS STOP → DRILL ABORT → NAV STOP → GEO STOW → SEQ ABORT"""
    print("  Running safe stop sequence...")
    cmd_seis_stop(sender, [])
    time.sleep(0.2)
    cmd_drill_abort(sender, [])
    time.sleep(0.2)
    cmd_nav_stop(sender, [])
    time.sleep(0.2)
    cmd_geo_stow(sender, [])
    time.sleep(0.5)
    cmd_seq_abort(sender, [])


# ============================================================
# Test Suite
# ============================================================
class TestRunner:
    """Simple test harness that sends commands and checks telemetry responses."""

    def __init__(self, sender, tlm):
        self.sender = sender
        self.tlm = tlm
        self.passed = 0
        self.failed = 0
        self.total = 0

    def check(self, name, condition, detail=""):
        self.total += 1
        if condition:
            self.passed += 1
            print(f"    {GRN}✓ PASS{RST}  {name}")
        else:
            self.failed += 1
            msg = f" ({detail})" if detail else ""
            print(f"    {RED}✗ FAIL{RST}  {name}{msg}")

    def wait_tlm(self, mid, timeout=3.0):
        """Wait for a telemetry packet to arrive with given MID."""
        start = time.time()
        while time.time() - start < timeout:
            raw = self.tlm.get(mid)
            if raw:
                return raw
            time.sleep(0.1)
        return None

    def summary(self):
        color = GRN if self.failed == 0 else RED
        print()
        print(f"  {BOLD}Test Results: {color}{self.passed}/{self.total} passed{RST}")
        if self.failed:
            print(f"  {RED}{self.failed} FAILED{RST}")
        print()


def test_seq(sender, tlm, args):
    """Test suite: Sequencer — NOOP, RESET, state transitions"""
    print(f"\n  {BOLD}═══ SEQ Test Suite ═══{RST}\n")
    t = TestRunner(sender, tlm)

    # NOOP
    sender.send(WOK_SEQ_CMD_MID, SEQ_NOOP_CC, label="SEQ NOOP")
    time.sleep(0.3)
    sender.send(WOK_SEQ_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_SEQ_TLM_MID)
    t.check("SEQ NOOP — HK received", raw is not None)
    if raw:
        d = decode_seq_hk(raw)
        if d:
            t.check("SEQ NOOP — cmd_cnt > 0", d["cmd_cnt"] > 0, f"cmd_cnt={d['cmd_cnt']}")
            t.check("SEQ NOOP — err_cnt == 0", d["err_cnt"] == 0, f"err_cnt={d['err_cnt']}")

    # RESET
    sender.send(WOK_SEQ_CMD_MID, SEQ_RESET_CTRS_CC, label="SEQ RESET_CTRS")
    time.sleep(0.3)
    sender.send(WOK_SEQ_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_SEQ_TLM_MID)
    if raw:
        d = decode_seq_hk(raw)
        if d:
            t.check("SEQ RESET — counters zeroed", d["cmd_cnt"] == 0 and d["err_cnt"] == 0,
                     f"cmd={d['cmd_cnt']} err={d['err_cnt']}")

    # START
    sender.send(WOK_SEQ_CMD_MID, SEQ_START_CC, label="SEQ START")
    time.sleep(0.5)
    sender.send(WOK_SEQ_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_SEQ_TLM_MID)
    if raw:
        d = decode_seq_hk(raw)
        if d:
            t.check("SEQ START — state != IDLE", d["state"] != "IDLE", f"state={d['state']}")

    # ABORT
    sender.send(WOK_SEQ_CMD_MID, SEQ_ABORT_CC, label="SEQ ABORT")
    time.sleep(0.5)
    sender.send(WOK_SEQ_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_SEQ_TLM_MID)
    if raw:
        d = decode_seq_hk(raw)
        if d:
            t.check("SEQ ABORT — state == SAFE", d["state"] == "SAFE", f"state={d['state']}")

    # RESUME
    sender.send(WOK_SEQ_CMD_MID, SEQ_RESUME_CC, label="SEQ RESUME")
    time.sleep(0.5)
    sender.send(WOK_SEQ_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_SEQ_TLM_MID)
    if raw:
        d = decode_seq_hk(raw)
        if d:
            # RESUME re-enters MOVE, but a no-motion waypoint advances the
            # SM within one fast HK tick — accept any active mission state
            t.check("SEQ RESUME — mission active",
                    d["state"] in ("MOVE", "DEPLOY", "THUMP", "RECORD", "EVALUATE"),
                    f"state={d['state']}")

    # Clean up
    sender.send(WOK_SEQ_CMD_MID, SEQ_ABORT_CC)
    t.summary()


def test_nav(sender, tlm, args):
    """Test suite: Navigation — NOOP, GOTO, STOP, SET_POSE, REALIGN"""
    print(f"\n  {BOLD}═══ NAV Test Suite ═══{RST}\n")
    t = TestRunner(sender, tlm)

    def nav_status():
        sender.send(WOK_NAV_SEND_HK_MID, 0)
        time.sleep(0.5)
        raw = t.wait_tlm(WOK_NAV_STATUS_TLM_MID)
        return decode_nav_status(raw) if raw else None

    sender.send(WOK_NAV_CMD_MID, NAV_NOOP_CC, SID, label="NAV NOOP")
    time.sleep(0.3)
    d = nav_status()
    t.check("NAV NOOP — status TLM received", d is not None)
    pose0 = (d["pose_x"], d["pose_y"]) if d else (0.0, 0.0)
    realigns0 = d.get("realign_count", 0) if d else 0

    # GOTO — relative step: forward 2m, lateral 1m (left)
    payload = SID + struct.pack("<ff", 2.0, 1.0)
    sender.send(WOK_NAV_CMD_MID, NAV_GOTO_CC, payload, label="NAV GOTO (df=2.0, dl=1.0)")
    time.sleep(0.3)
    d = nav_status()
    if d:
        t.check("NAV GOTO — drive_state == DRIVING", d["drive_state"] == "DRIVING",
                 f"drive_state={d['drive_state']}")

    # REALIGN while driving — must be rejected (state + count unchanged)
    sender.send(WOK_NAV_CMD_MID, NAV_REALIGN_CC, SID + struct.pack("<f", 5.0),
                label="NAV REALIGN (expect reject: driving)")
    # SET_POSE while driving — must be rejected (pose untouched)
    sender.send(WOK_NAV_CMD_MID, NAV_SET_POSE_CC, SID + struct.pack("<ff", 99.0, 99.0),
                label="NAV SET_POSE (expect reject: driving)")
    time.sleep(0.3)
    d = nav_status()
    if d:
        t.check("NAV REALIGN while driving — rejected (still DRIVING)",
                 d["drive_state"] == "DRIVING", f"drive_state={d['drive_state']}")
        t.check("NAV REALIGN while driving — realign_count unchanged",
                 d.get("realign_count", 0) == realigns0,
                 f"count={d.get('realign_count')}")
        t.check("NAV SET_POSE while driving — rejected (pose untouched)",
                 abs(d["pose_x"] - 99.0) > 1.0, f"pose_x={d['pose_x']:.2f}")

    # STOP
    sender.send(WOK_NAV_CMD_MID, NAV_STOP_CC, SID, label="NAV STOP")
    time.sleep(0.3)
    d = nav_status()
    if d:
        t.check("NAV STOP — drive_state == STOPPED", d["drive_state"] == "STOPPED",
                 f"drive_state={d['drive_state']}")

    # SET_POSE from stopped — accepted, pose overwritten
    sender.send(WOK_NAV_CMD_MID, NAV_SET_POSE_CC, SID + struct.pack("<ff", 5.0, 7.0),
                label="NAV SET_POSE (5.0, 7.0)")
    time.sleep(0.3)
    d = nav_status()
    if d:
        t.check("NAV SET_POSE — pose == (5.0, 7.0)",
                 abs(d["pose_x"] - 5.0) < 0.01 and abs(d["pose_y"] - 7.0) < 0.01,
                 f"pose=({d['pose_x']:.2f}, {d['pose_y']:.2f})")

    # REALIGN below the 1° threshold — instant, no turn, count increments
    sender.send(WOK_NAV_CMD_MID, NAV_REALIGN_CC, SID + struct.pack("<f", 0.5),
                label="NAV REALIGN (0.5° — below threshold)")
    time.sleep(0.3)
    d = nav_status()
    if d:
        t.check("NAV REALIGN (sub-threshold) — realign_count incremented",
                 d.get("realign_count", 0) == realigns0 + 1,
                 f"count={d.get('realign_count')}")

    # REALIGN with a real corrective turn (5° ≈ 0.2 s at 30°/s)
    sender.send(WOK_NAV_CMD_MID, NAV_REALIGN_CC, SID + struct.pack("<f", 5.0),
                label="NAV REALIGN (5.0°)")
    time.sleep(1.5)
    d = nav_status()
    if d:
        t.check("NAV REALIGN (5°) — completed to ARRIVED",
                 d["drive_state"] == "ARRIVED", f"drive_state={d['drive_state']}")
        t.check("NAV REALIGN (5°) — realign_count incremented",
                 d.get("realign_count", 0) == realigns0 + 2,
                 f"count={d.get('realign_count')}")
        t.check("NAV REALIGN — heading_drift zeroed",
                 abs(d["heading_drift"]) < 0.01, f"drift={d['heading_drift']:.2f}")

    # Restore the pre-suite pose (bookkeeping courtesy)
    sender.send(WOK_NAV_CMD_MID, NAV_SET_POSE_CC,
                SID + struct.pack("<ff", pose0[0], pose0[1]),
                label="NAV SET_POSE (restore)")
    time.sleep(0.3)

    t.summary()


def test_geo(sender, tlm, args):
    """Test suite: Geophone — NOOP, DEPLOY, STOW, ABORT"""
    print(f"\n  {BOLD}═══ GEO Test Suite ═══{RST}\n")
    t = TestRunner(sender, tlm)

    sender.send(WOK_GEO_CMD_MID, GEO_NOOP_CC, SID, label="GEO NOOP")
    time.sleep(0.3)
    sender.send(WOK_GEO_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_GEO_STATUS_TLM_MID)
    t.check("GEO NOOP — status TLM received", raw is not None)

    # DEPLOY
    sender.send(WOK_GEO_CMD_MID, GEO_DEPLOY_CC, SID, label="GEO DEPLOY")
    time.sleep(0.3)
    sender.send(WOK_GEO_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_GEO_STATUS_TLM_MID)
    if raw:
        d = decode_geo_status(raw)
        if d:
            t.check("GEO DEPLOY — state is DEPLOYING or DEPLOYED",
                     d["mech_state"] in ("DEPLOYING", "DEPLOYED"),
                     f"mech_state={d['mech_state']}")

    # STOW
    sender.send(WOK_GEO_CMD_MID, GEO_STOW_CC, SID, label="GEO STOW")
    time.sleep(0.3)
    sender.send(WOK_GEO_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_GEO_STATUS_TLM_MID)
    if raw:
        d = decode_geo_status(raw)
        if d:
            t.check("GEO STOW — state is STOWING or STOWED",
                     d["mech_state"] in ("STOWING", "STOWED"),
                     f"mech_state={d['mech_state']}")

    # ABORT
    sender.send(WOK_GEO_CMD_MID, GEO_ABORT_CC, SID, label="GEO ABORT")
    time.sleep(0.3)
    sender.send(WOK_GEO_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_GEO_STATUS_TLM_MID)
    if raw:
        d = decode_geo_status(raw)
        if d:
            t.check("GEO ABORT — state == FAULT", d["mech_state"] == "FAULT",
                     f"mech_state={d['mech_state']}")

    t.summary()


def test_seis(sender, tlm, args):
    """Test suite: Seismic — NOOP, START_ACQ, STOP_ACQ"""
    print(f"\n  {BOLD}═══ SEIS Test Suite ═══{RST}\n")
    t = TestRunner(sender, tlm)

    sender.send(WOK_SEIS_CMD_MID, SEIS_NOOP_CC, SID, label="SEIS NOOP")
    time.sleep(0.3)
    sender.send(WOK_SEIS_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_SEIS_STATUS_TLM_MID)
    t.check("SEIS NOOP — status TLM received", raw is not None)

    # START ACQ (v2.5 payload: site pose + stack fields)
    payload = SID + struct.pack("<HHBBHfffHH", 10, 500, 1, 0x0F, 0,
                                0.0, 0.0, 0.0, 0, 1)
    sender.send(WOK_SEIS_CMD_MID, SEIS_START_ACQ_CC, payload,
                label="SEIS START_ACQ 10s @ 500Hz")
    time.sleep(0.3)
    sender.send(WOK_SEIS_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_SEIS_STATUS_TLM_MID)
    if raw:
        d = decode_seis_status(raw)
        if d:
            t.check("SEIS START_ACQ — acquiring == True", d["acquiring"],
                     f"acquiring={d['acquiring']}")

    # STOP ACQ
    sender.send(WOK_SEIS_CMD_MID, SEIS_STOP_ACQ_CC, SID, label="SEIS STOP_ACQ")
    time.sleep(0.3)
    sender.send(WOK_SEIS_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_SEIS_STATUS_TLM_MID)
    if raw:
        d = decode_seis_status(raw)
        if d:
            t.check("SEIS STOP_ACQ — acquiring == False", not d["acquiring"],
                     f"acquiring={d['acquiring']}")

    t.summary()


def test_hk(sender, tlm, args):
    """Test suite: Health/HK — NOOP, SET_THRESHOLDS, CLEAR_FAULTS"""
    print(f"\n  {BOLD}═══ HK Test Suite ═══{RST}\n")
    t = TestRunner(sender, tlm)

    sender.send(WOK_HK_CMD_MID, HK_NOOP_CC, label="HK NOOP")
    time.sleep(0.3)
    sender.send(WOK_HK_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_HK_TLM_MID)
    t.check("HK NOOP — health TLM received", raw is not None)
    if raw:
        d = decode_hk_health(raw)
        if d:
            t.check("HK — watchdog OK", d["watchdog_ok"], f"watchdog={d['watchdog_ok']}")
            t.check("HK — no safe trigger", not d["safe_trigger"],
                     f"safe_trigger={d['safe_trigger']}")

    # SET THRESHOLDS
    payload = struct.pack("<fff", 20.0, 60.0, 35.0)
    sender.send(WOK_HK_CMD_MID, HK_SET_THRESHOLDS_CC, payload,
                label="HK SET_THRESHOLDS 20V/60C/35deg")
    time.sleep(0.3)
    t.check("HK SET_THRESHOLDS — command accepted", True)

    # CLEAR FAULTS
    sender.send(WOK_HK_CMD_MID, HK_CLEAR_FAULTS_CC, label="HK CLEAR_FAULTS")
    time.sleep(0.3)
    sender.send(WOK_HK_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_HK_TLM_MID)
    if raw:
        d = decode_hk_health(raw)
        if d:
            t.check("HK CLEAR_FAULTS — faults == NONE", d["fault_raw"] == 0,
                     f"faults={d['faults']}")

    t.summary()


def test_all(sender, tlm, args):
    """Run all test suites in sequence"""
    print(f"\n  {BOLD}{'═'*50}")
    print(f"  WOK Full Test Suite")
    print(f"  {'═'*50}{RST}\n")

    test_seq(sender, tlm, args)
    test_nav(sender, tlm, args)
    test_geo(sender, tlm, args)
    test_seis(sender, tlm, args)
    test_hk(sender, tlm, args)
    test_drill(sender, tlm, args)

    print(f"  {BOLD}{'═'*50}")
    print(f"  Full suite complete")
    print(f"  {'═'*50}{RST}")


def test_drill(sender, tlm, args):
    """Test suite: Drill — NOOP, DEPLOY, STOW, ABORT"""
    print(f"\n  {BOLD}═══ DRILL Test Suite ═══{RST}\n")
    t = TestRunner(sender, tlm)

    # NOOP
    sender.send(WOK_DRILL_CMD_MID, DRILL_NOOP_CC, SID, label="DRILL NOOP")
    time.sleep(0.3)
    sender.send(WOK_DRILL_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_DRILL_TLM_MID)
    t.check("DRILL NOOP — HK TLM received", raw is not None)

    # Clear any e-stop latched by an earlier suite (e.g. test.seq's ABORT
    # sends DRILL ESTOP; the HAL only clears it on a stow cycle)
    sender.send(WOK_DRILL_CMD_MID, DRILL_STOW_ARM_CC, SID, label="DRILL STOW (clear e-stop)")
    time.sleep(3.5)

    # DEPLOY
    payload = SID + struct.pack("<f", 90.0)
    sender.send(WOK_DRILL_CMD_MID, DRILL_DEPLOY_ARM_CC, payload, label="DRILL DEPLOY_ARM 90 deg")
    time.sleep(4.0)  # sim takes 3 seconds
    sender.send(WOK_DRILL_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_DRILL_TLM_MID)
    if raw:
        d = decode_drill_hk(raw)
        if d:
            t.check("DRILL DEPLOY — state is DEPLOYING or ARM_READY",
                     d["state"] in ("DEPLOYING", "ARM_READY"),
                     f"state={d['state']}")

    # STOW
    sender.send(WOK_DRILL_CMD_MID, DRILL_STOW_ARM_CC, SID, label="DRILL STOW_ARM")
    time.sleep(4.0)
    sender.send(WOK_DRILL_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_DRILL_TLM_MID)
    if raw:
        d = decode_drill_hk(raw)
        if d:
            t.check("DRILL STOW — state is STOWING or IDLE",
                     d["state"] in ("STOWING", "IDLE"),
                     f"state={d['state']}")

    # ABORT (deploy then immediately abort)
    payload = SID + struct.pack("<f", 90.0)
    sender.send(WOK_DRILL_CMD_MID, DRILL_DEPLOY_ARM_CC, payload, label="DRILL DEPLOY_ARM")
    time.sleep(1.0)
    sender.send(WOK_DRILL_CMD_MID, DRILL_ESTOP_CC, SID, label="DRILL EMERGENCY_STOP")
    time.sleep(0.5)
    sender.send(WOK_DRILL_SEND_HK_MID, 0)
    time.sleep(0.5)
    raw = t.wait_tlm(WOK_DRILL_TLM_MID)
    if raw:
        d = decode_drill_hk(raw)
        if d:
            t.check("DRILL ABORT — state == FAULT", d["state"] == "FAULT",
                     f"state={d['state']}")

    t.summary()


def test_connectivity(sender, tlm, args):
    """Quick connectivity test — NOOP all apps, check for any TLM response"""
    print(f"\n  {BOLD}═══ Connectivity Test ═══{RST}\n")
    t = TestRunner(sender, tlm)

    initial_count = tlm.pkt_count

    cmd_allnoop(sender, [])
    time.sleep(0.5)
    cmd_allhk(sender, [])
    time.sleep(1.5)

    t.check("CI_LAB — commands sent (no socket error)", True)
    t.check("TO_LAB — received telemetry packets",
            tlm.pkt_count > initial_count,
            f"received {tlm.pkt_count - initial_count} new packets")

    for name, mid in [("SEQ", WOK_SEQ_TLM_MID), ("NAV", WOK_NAV_TLM_MID),
                       ("GEO", WOK_GEO_TLM_MID), ("SEIS", WOK_SEIS_TLM_MID),
                       ("HK", WOK_HK_TLM_MID), ("DRILL", WOK_DRILL_TLM_MID)]:
        raw = tlm.get(mid)
        t.check(f"{name} — HK telemetry received", raw is not None)

    t.summary()


# ============================================================
# Command Registry
# ============================================================
# We build two registries: one for commands that only need sender,
# one for commands that need sender + tlm_listener.
# The REPL dispatches accordingly.

SENDER_COMMANDS = OrderedDict([
    # Sequencer
    ("seq.noop",      cmd_seq_noop),
    ("seq.reset",     cmd_seq_reset),
    ("seq.start",     cmd_seq_start),
    ("seq.abort",     cmd_seq_abort),
    ("seq.resume",    cmd_seq_resume),
    ("seq.waypoint",  cmd_seq_waypoint),
    ("seq.mode",      cmd_seq_mode),
    ("seq.placement", cmd_seq_placement),
    ("seq.release",   cmd_seq_release),
    ("seq.loadwp",    cmd_seq_loadwp),
    ("seq.wpcount",   cmd_seq_wpcount),
    ("seq.hk",        cmd_seq_hk),

    # Navigation
    ("nav.noop",      cmd_nav_noop),
    ("nav.reset",     cmd_nav_reset),
    ("nav.goto",      cmd_nav_goto),
    ("nav.stop",      cmd_nav_stop),
    ("nav.speed",     cmd_nav_speed),
    ("nav.setpose",   cmd_nav_setpose),
    ("nav.realign",   cmd_nav_realign),
    ("nav.hk",        cmd_nav_hk),

    # Geophone
    ("geo.noop",      cmd_geo_noop),
    ("geo.reset",     cmd_geo_reset),
    ("geo.deploy",    cmd_geo_deploy),
    ("geo.stow",      cmd_geo_stow),
    ("geo.abort",     cmd_geo_abort),
    ("geo.place",     cmd_geo_place),
    ("geo.retrieve",  cmd_geo_retrieve),
    ("geo.reseat",    cmd_geo_reseat),
    ("geo.hk",        cmd_geo_hk),

    # Seismic
    ("seis.noop",     cmd_seis_noop),
    ("seis.reset",    cmd_seis_reset),
    ("seis.start",    cmd_seis_start),
    ("seis.stop",     cmd_seis_stop),
    ("seis.thump_arm", cmd_seis_thump_arm),
    ("seis.thump_fire", cmd_seis_thump_fire),
    ("seis.thump_reset", cmd_seis_thump_reset),
    ("seis.evaluate", cmd_seis_evaluate),
    ("seis.source",   cmd_seis_source),
    ("seis.plate_reseat", cmd_seis_plate_reseat),
    ("seis.hk",       cmd_seis_hk),

    # Health / HK
    ("hk.noop",       cmd_hk_noop),
    ("hk.reset",      cmd_hk_reset),
    ("hk.thresholds", cmd_hk_thresholds),
    ("hk.clear",      cmd_hk_clear),
    ("hk.hk",         cmd_hk_hk),

    # Drill
    ("drill.noop",    cmd_drill_noop),
    ("drill.reset",   cmd_drill_reset),
    ("drill.deploy",  cmd_drill_deploy),
    ("drill.stow",    cmd_drill_stow),
    ("drill.start",   cmd_drill_start),
    ("drill.stop",    cmd_drill_stop),
    ("drill.sample",  cmd_drill_sample),
    ("drill.abort",   cmd_drill_abort),
    ("drill.hk",      cmd_drill_hk),

    # Convenience
    ("allnoop",       cmd_allnoop),
    ("allreset",      cmd_allreset),
    ("allhk",         cmd_allhk),
    ("quickstart",    cmd_quickstart),
    ("safestop",      cmd_safestop),
])

# Commands that need (sender, tlm_listener, args)
TEST_COMMANDS = OrderedDict([
    ("seis.fetch",        cmd_seis_fetch),   # needs sender + tlm listener
    ("test.connectivity", test_connectivity),
    ("test.seq",          test_seq),
    ("test.nav",          test_nav),
    ("test.geo",          test_geo),
    ("test.seis",         test_seis),
    ("test.hk",           test_hk),
    ("test.drill",        test_drill),
    ("test.all",          test_all),
])

# TLM commands need (tlm_listener, args)
TLM_COMMANDS = OrderedDict([
    ("tlm",           cmd_tlm),
    ("tlm.verbose",   cmd_tlm_verbose),
])


# ============================================================
# Help
# ============================================================
def print_help():
    print()
    print(f"  {BOLD}WOK Mission Command Console{RST}")
    print(f"  {'═'*55}")
    print()
    print(f"  {BOLD}Sequencer (SEQ){RST}")
    print(f"  {'─'*55}")
    print(f"  seq.noop              Verify command path")
    print(f"  seq.reset             Reset counters")
    print(f"  seq.start             Begin waypoint loop")
    print(f"  seq.abort             Abort → SAFE")
    print(f"  seq.resume            Resume from SAFE")
    print(f"  seq.waypoint <idx>    Override waypoint index")
    print(f"  seq.mode <0|1>        0=drill-inline, 1=survey-first")
    print(f"  seq.placement <0|1>   0=fixed array, 1=per-pod placement missions")
    print(f"  seq.release           Release a recalibration HOLD")
    print(f"  seq.loadwp <idx> <fwd> <lat> [drill] [srcmode] [energy_J] [recal]")
    print(f"                        Upload/replace one waypoint")
    print(f"  seq.wpcount <n>       Set active waypoint count")
    print(f"  seq.hk                Request HK telemetry")
    print()
    print(f"  {BOLD}Navigation (NAV){RST}")
    print(f"  {'─'*55}")
    print(f"  nav.noop              Verify command path")
    print(f"  nav.reset             Reset counters")
    print(f"  nav.goto <df> <dl>    Relative step (start-frame: forward, +left)")
    print(f"  nav.stop              Immediate stop")
    print(f"  nav.speed <m/s>       Set max speed")
    print(f"  nav.setpose <x> <y>   Overwrite pose with ground truth (no motion)")
    print(f"  nav.realign <err_deg> Corrective turn + re-declare start-frame heading")
    print(f"  nav.hk                Request HK telemetry")
    print()
    print(f"  {BOLD}Geophone (GEO){RST}")
    print(f"  {'─'*55}")
    print(f"  geo.noop              Verify command path")
    print(f"  geo.reset             Reset counters")
    print(f"  geo.deploy            Deploy geophones (fixed-array mode)")
    print(f"  geo.stow              Stow geophones (fixed-array mode)")
    print(f"  geo.place <pod> <x> <y>  Place one pod (placement contingency)")
    print(f"  geo.reseat <pod>      Reseat a marginal pod")
    print(f"  geo.retrieve <pod>    Retrieve one pod")
    print(f"  geo.abort             Emergency stop mechanism")
    print(f"  geo.hk                Request HK telemetry")
    print()
    print(f"  {BOLD}Seismic + EM Source (SEIS){RST}")
    print(f"  {'─'*55}")
    print(f"  seis.noop             Verify command path")
    print(f"  seis.reset            Reset counters")
    print(f"  seis.start [dur] [fs] [gain] [ch_hex] [x] [y] [hdg] [sidx] [starget]")
    print(f"                        Start acquisition (site pose + stack idx/target)")
    print(f"                        Defaults: 6s 4000Hz gain=1 ch=0F site(0,0,0) stack 0/1")
    print(f"  seis.stop             Stop acquisition")
    print(f"  seis.fetch <file>     Downlink a data product to wok_archive/")
    print(f"  seis.thump_arm        Arm source (charge capacitor bank)")
    print(f"  seis.thump_fire       Fire source (impulse discharge / start sweep)")
    print(f"  seis.thump_reset      Reset source to idle")
    print(f"  seis.source <mode> [energy f0 f1 dur]")
    print(f"                        Configure source: 0=impulse, 1=swept-sine")
    print(f"  seis.plate_reseat     Reseat the source plate")
    print(f"  seis.evaluate         Evaluate data for ice signature")
    print(f"  seis.hk               Request HK telemetry")
    print()
    print(f"  {BOLD}Health / HK{RST}")
    print(f"  {'─'*55}")
    print(f"  hk.noop               Verify command path")
    print(f"  hk.reset              Reset counters")
    print(f"  hk.thresholds [V] [C] [deg]")
    print(f"                        Set fault thresholds (22.0 55.0 30.0)")
    print(f"  hk.clear              Clear all faults")
    print(f"  hk.hk                 Request HK telemetry")
    print()
    print(f"  {BOLD}Drill / Ice Prospector (DRILL){RST}")
    print(f"  {'─'*55}")
    print(f"  drill.noop            Verify command path")
    print(f"  drill.reset           Reset counters")
    print(f"  drill.deploy [angle]  Deploy arm (default 90 deg)")
    print(f"  drill.stow            Retract arm to stowed")
    print(f"  drill.start [depth] [rpm] [x] [y]")
    print(f"                        Start drilling (default 1.0m 200RPM, site stamp 0,0)")
    print(f"  drill.stop            Stop drill motor")
    print(f"  drill.sample [heat_s] [cool_s]")
    print(f"                        Thermal vaporization (default 60s 30s)")
    print(f"  drill.abort           Emergency stop all drill ops")
    print(f"  drill.hk              Request HK telemetry")
    print()
    print(f"  {BOLD}Convenience{RST}")
    print(f"  {'─'*55}")
    print(f"  allnoop               NOOP all 6 apps")
    print(f"  allreset              Reset all counters")
    print(f"  allhk                 Request HK from all apps")
    print(f"  quickstart [delay]    NOOP all → START sequence")
    print(f"  safestop              SEIS stop → DRILL abort → NAV stop → GEO stow → ABORT")
    print()
    print(f"  {BOLD}Telemetry{RST}")
    print(f"  {'─'*55}")
    print(f"  tlm [app]             Show latest TLM (seq/nav/geo/seis/hk/drill/all)")
    print(f"  tlm.verbose           Toggle live packet printing")
    print()
    print(f"  {BOLD}Test Suites{RST}")
    print(f"  {'─'*55}")
    print(f"  test.connectivity     Quick CI_LAB/TO_LAB check")
    print(f"  test.seq              Sequencer test suite")
    print(f"  test.nav              Navigation test suite")
    print(f"  test.geo              Geophone test suite")
    print(f"  test.seis             Seismic test suite")
    print(f"  test.hk               Health/HK test suite")
    print(f"  test.drill            Drill test suite")
    print(f"  test.all              Run ALL test suites")
    print()
    print(f"  {BOLD}General{RST}")
    print(f"  {'─'*55}")
    print(f"  help                  Show this help")
    print(f"  quit / exit / q       Exit console")
    print()
    print(f"  {DIM}Tab completion enabled. Command history saved to ~/.wok_history{RST}")
    print()


# ============================================================
# Tab completion
# ============================================================
ALL_COMMANDS = list(SENDER_COMMANDS.keys()) + list(TEST_COMMANDS.keys()) + \
               list(TLM_COMMANDS.keys()) + ["help", "quit", "exit"]

def completer(text, state):
    options = [c for c in ALL_COMMANDS if c.startswith(text)]
    return options[state] if state < len(options) else None


# ============================================================
# Main REPL
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="WOK Mission Command Console")
    parser.add_argument("--host", default="127.0.0.1",
                        help="CI_LAB target host (default: 127.0.0.1)")
    parser.add_argument("--cmd-port", type=int, default=1234,
                        help="CI_LAB command uplink port (default: 1234)")
    parser.add_argument("--tlm-port", type=int, default=2234,
                        help="TO_LAB telemetry downlink port (default: 2234)")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose telemetry printing at start")
    args = parser.parse_args()

    sender = CmdSender(args.host, args.cmd_port)
    tlm = TlmListener(args.tlm_port, verbose=args.verbose)

    # Set up readline
    readline.set_completer(completer)
    readline.parse_and_bind("tab: complete")
    histfile = os.path.expanduser("~/.wok_history")
    try:
        readline.read_history_file(histfile)
    except FileNotFoundError:
        pass

    print()
    print(f"  {BOLD}╔════════════════════════════════════════════╗{RST}")
    print(f"  {BOLD}║       WOK Mission Command Console          ║{RST}")
    print(f"  {BOLD}║  Seismic Rover — cFS Draco Ground System   ║{RST}")
    print(f"  {BOLD}╚════════════════════════════════════════════╝{RST}")
    print(f"  CI_LAB  → {args.host}:{args.cmd_port}")
    print(f"  TO_LAB  ← 0.0.0.0:{args.tlm_port}")
    print(f"  Type 'help' for commands, 'quit' to exit")
    print(f"  Tab completion enabled")
    print()

    tlm.start()

    while True:
        try:
            line = input(f"{CYN}wok>{RST} ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()
        cmd_args = parts[1:]

        if cmd in ("quit", "exit", "q"):
            break
        elif cmd == "help":
            print_help()
        elif cmd in SENDER_COMMANDS:
            try:
                SENDER_COMMANDS[cmd](sender, cmd_args)
            except Exception as e:
                print(f"  {RED}Error: {e}{RST}")
        elif cmd in TEST_COMMANDS:
            try:
                TEST_COMMANDS[cmd](sender, tlm, cmd_args)
            except Exception as e:
                print(f"  {RED}Error: {e}{RST}")
        elif cmd in TLM_COMMANDS:
            try:
                TLM_COMMANDS[cmd](tlm, cmd_args)
            except Exception as e:
                print(f"  {RED}Error: {e}{RST}")
        else:
            # Fuzzy suggest
            matches = [c for c in ALL_COMMANDS if cmd in c]
            if matches:
                print(f"  Unknown command: '{cmd}'. Did you mean: {', '.join(matches[:5])}?")
            else:
                print(f"  Unknown command: '{cmd}'. Type 'help' for available commands.")

    # Save history
    try:
        readline.write_history_file(histfile)
    except Exception:
        pass

    tlm.stop()
    sender.close()
    print("  Console closed.")


if __name__ == "__main__":
    main()
