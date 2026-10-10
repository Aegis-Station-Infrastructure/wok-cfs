#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
WOK Ground Control — WebSocket ↔ UDP Bridge
Bridges browser WebSocket clients to cFS CI_LAB (cmd) and TO_LAB (tlm) UDP ports.

Usage:
    python3 wok_bridge.py [--cfs-host 127.0.0.1] [--cmd-port 1234] [--tlm-port 2234]
                          [--ws-port 8765] [--ws-host 127.0.0.1]

Requires Python 3.10+ and the 'websockets' package (see ground/requirements.txt).

SECURITY: there is NO authentication on the WebSocket. Any client that can
reach it can command the flight software. The server therefore listens on
127.0.0.1 only by default; passing --ws-host 0.0.0.0 (so a dashboard on
another machine can connect) is an explicit opt-in that must only be used
on a trusted LAN. The telemetry UDP socket is bound to the address TO_LAB
is told to send to (127.0.0.1 for a local cFS), never to all interfaces.
"""

import argparse
import asyncio
import json
import socket
import struct
import time

try:
    import websockets
except ImportError:
    import sys
    print("ERROR: the 'websockets' package is required.")
    print("  Install it with:  pip install websockets")
    print("  (or your distro's python3-websockets package)")
    sys.exit(1)


# ─── cFS / CCSDS Constants ─────────────────────────────────────────────
# Draco CCSDS v1 primary headers (default cFS v7.0.0 build, no extended
# header). All app payloads are little-endian, 4-byte aligned, and start
# right after the header:
# CMD: 8-byte header  (6 primary + 2 secondary: function code + checksum)
# TLM: 16-byte header (6 primary + 6 time secondary + 4 alignment padding)

class WokMid:
    """WOK Message IDs — must match wok_msgids.h"""
    # Command MIDs (0x18D0+)
    SEQ_CMD      = 0x18D0
    SEQ_SEND_HK  = 0x18D1
    NAV_CMD      = 0x18D2
    NAV_SEND_HK  = 0x18D3
    GEO_CMD      = 0x18D4
    GEO_SEND_HK  = 0x18D5
    SEIS_CMD     = 0x18D6
    SEIS_SEND_HK = 0x18D7
    HK_CMD       = 0x18D8
    HK_SEND_HK   = 0x18D9
    DRILL_CMD     = 0x18DA
    DRILL_SEND_HK = 0x18DB

    # Telemetry MIDs (0x08D0+)
    SEQ_TLM       = 0x08D0
    SEQ_MODE_TLM  = 0x08D1
    NAV_TLM       = 0x08D2
    NAV_STATUS_TLM = 0x08D3
    GEO_TLM       = 0x08D4
    GEO_STATUS_TLM = 0x08D5
    SEIS_TLM      = 0x08D6
    SEIS_STATUS_TLM = 0x08D7
    SEIS_QUICKLOOK = 0x08D8
    HK_TLM        = 0x08D9
    DRILL_TLM     = 0x08DA
    DRILL_STATUS_TLM = 0x08DB
    SEIS_FILECHUNK = 0x08DC

    # TO_LAB
    TO_LAB_CMD    = 0x1880


# ─── CCSDS Packet Builder ──────────────────────────────────────────────

_cmd_seq_count = 0

def build_ccsds_cmd(mid, cc=0, payload=b""):
    """Build a CCSDS v1 command packet for CI_LAB (Draco format).
    Matches wok_console.py CmdSender._header() exactly."""
    global _cmd_seq_count
    _cmd_seq_count = (_cmd_seq_count + 1) & 0x3FFF
    # Primary header: StreamID (2) + Sequence (2) + Length (2)
    stream_id = mid
    sequence = 0xC000 | _cmd_seq_count
    # Length = total_bytes - 7 (CCSDS convention)
    total = 8 + len(payload)  # 6 primary + 2 secondary + payload
    length = total - 7

    hdr = struct.pack(">HHH", stream_id, sequence, length)
    # Secondary header: function code (1 byte) + checksum placeholder (1 byte)
    sec = struct.pack("BB", cc, 0)
    pkt = hdr + sec + payload
    return pkt


def build_to_lab_enable(ip_str="127.0.0.1"):
    """Build TO_LAB enable command (CC=6, payload = char dest_IP[16]).
    The payload is the bare IP string — TO_LAB echoes it verbatim into
    OS_SocketAddrFromString, so any prefix breaks the downlink."""
    payload = ip_str.encode("ascii").ljust(16, b"\x00")
    return build_ccsds_cmd(WokMid.TO_LAB_CMD, cc=6, payload=payload)


def local_ip_toward(host):
    """The IP address of THIS machine as seen from `host` — i.e. where
    TO_LAB must send telemetry so the bridge receives it. For a local
    cFS this is 127.0.0.1; for a remote cFS it is the local address of
    the route toward it (no packets are actually sent)."""
    import socket as _socket
    try:
        if _socket.gethostbyname(host).startswith("127."):
            return "127.0.0.1"
        s = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        s.connect((host, 9))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


# ─── CCSDS Telemetry Decoder ───────────────────────────────────────────

TLM_DECODERS = {}

# WOK_HAL_SrcState_t — EM source state, shared by SEIS HK and STATUS
SOURCE_STATES = ["IDLE", "CHARGING", "ARMED", "FIRED", "SWEEPING"]

def decode_tlm_header(data):
    """Decode CCSDS primary header, return (mid, seq_count, length, payload_offset)."""
    if len(data) < 16:
        return None
    stream_id, sequence, length = struct.unpack_from(">HHH", data, 0)
    mid = stream_id  # full MID including the 0x0800 telemetry bit
    seq_count = sequence & 0x3FFF
    # Payload starts at byte 16 (after full TLM header)
    return mid, seq_count, length, 16


def decode_seq_tlm(data):
    """Decode SEQ HK telemetry (0x08D0) — matches WOK_Seq_HkTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<HHBBHHH"):
        return {"error": "short payload"}
    # Real struct: uint16 cmd_cnt, uint16 err_cnt, uint8 state, uint8 prev_state,
    #              uint16 wp_idx, uint16 fault_mask, uint16 seq_id
    cmd_cnt, err_cnt, state, prev_state, wp_idx, fault_raw, seq_id = struct.unpack_from(
        "<HHBBHHH", payload, 0
    )
    states = ["IDLE", "MOVE", "DEPLOY", "THUMP", "RECORD", "EVALUATE", "DRILL", "STOW", "SAFE", "DONE", "VERIFY", "HOLD"]
    faults = decode_fault_bits(fault_raw)
    return {
        "app": "SEQ", "type": "HK",
        "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
        "state": states[state] if state < len(states) else f"UNK({state})",
        "prev_state": states[prev_state] if prev_state < len(states) else f"UNK({prev_state})",
        "waypoint_idx": wp_idx,
        "fault_raw": fault_raw,
        "faults": faults,
        "seq_id": seq_id,
    }

def decode_seq_mode_tlm(data):
    """Decode SEQ MODE telemetry (0x08D1) — matches WOK_Seq_ModeTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<BBHHH"):
        return {"error": "short payload"}
    # Real struct: uint8 state, uint8 loop_count, uint16 total_wp, uint16 fault_mask, uint16 spare
    state, loop_cnt, total_wp, fault_mask, spare = struct.unpack_from("<BBHHH", payload, 0)
    states = ["IDLE", "MOVE", "DEPLOY", "THUMP", "RECORD", "EVALUATE", "DRILL", "STOW", "SAFE", "DONE", "VERIFY", "HOLD"]
    d = {
        "app": "SEQ", "type": "MODE",
        "state": states[state] if state < len(states) else f"UNK({state})",
        "loop_count": loop_cnt,
        "total_waypoints": total_wp,
        "faults": decode_fault_bits(fault_mask),
    }
    # Targeting status (appended fields)
    if len(payload) >= 16:
        mode, drills, best_idx, _sp, best_score = struct.unpack_from("<BBBBf", payload, 8)
        d.update({
            "mission_mode": {0: "DRILL_INLINE", 1: "SURVEY_FIRST"}.get(mode, f"UNK({mode})"),
            "drills_completed": drills,
            "best_site": best_idx if best_idx != 0xFF else None,
            "best_score": round(best_score, 2),
        })
    return d

def decode_nav_tlm(data):
    """Decode NAV HK telemetry (0x08D2) — matches WOK_Nav_HkTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<HHBBH"):
        return {"error": "short payload"}
    # Real struct: uint16 cmd_cnt, uint16 err_cnt, uint8 drive_state, uint8 spare,
    #              uint16 fault_mask
    cmd_cnt, err_cnt, drive_state, _spare, fault_raw = struct.unpack_from("<HHBBH", payload, 0)
    drive_states = ["IDLE", "DRIVING", "ARRIVED", "STOPPED", "FAULT"]
    return {
        "app": "NAV", "type": "HK",
        "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
        "drive_state": drive_states[drive_state] if drive_state < len(drive_states) else f"UNK({drive_state})",
        "fault_raw": fault_raw,
    }

def decode_nav_status_tlm(data):
    """Decode NAV STATUS telemetry (0x08D3) — matches WOK_Nav_StatusTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<fffffBBHHH"):
        return {"error": "short payload"}
    # Real struct: float px, py, hdg, spd, tilt, uint8 drive_state, uint8 arrived,
    #              uint16 faults, uint16 seq_id, uint16 spare
    x, y, hdg, spd, tilt, drive, arrived, fault_raw, seq_id, spare = struct.unpack_from(
        "<fffffBBHHH", payload, 0
    )
    drive_states = ["IDLE", "DRIVING", "ARRIVED", "STOPPED", "FAULT"]
    d = {
        "app": "NAV", "type": "STATUS",
        "x": round(x, 3), "y": round(y, 3),
        "heading": round(hdg, 1),
        "speed": round(spd, 3),
        "tilt": round(tilt, 1),
        "drive_state": drive_states[drive] if drive < len(drive_states) else f"UNK({drive})",
        "arrived": bool(arrived),
        "faults": decode_fault_bits(fault_raw),
        "seq_id": seq_id,
    }
    # Ground recalibration (appended fields)
    if len(payload) >= 32:
        realigns, _sp = struct.unpack_from("<HH", payload, 28)
        d["realign_count"] = realigns
    return d

def decode_geo_tlm(data):
    """Decode GEO HK telemetry (0x08D4) — matches WOK_Geo_HkTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<HHBBH"):
        return {"error": "short payload"}
    # Real struct: uint16 cmd_cnt, uint16 err_cnt, uint8 mech_state, uint8 spare,
    #              uint16 fault_mask
    cmd_cnt, err_cnt, mech_state, _spare, fault_raw = struct.unpack_from("<HHBBH", payload, 0)
    mech_states = ["STOWED", "DEPLOYING", "DEPLOYED", "STOWING", "FAULT"]
    return {
        "app": "GEO", "type": "HK",
        "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
        "mech_state": mech_states[mech_state] if mech_state < len(mech_states) else f"UNK({mech_state})",
        "fault_raw": fault_raw,
    }

def decode_geo_status_tlm(data):
    """Decode GEO STATUS telemetry (0x08D5) — matches WOK_Geo_StatusTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<BBBBfHHHH"):
        return {"error": "short payload"}
    # Real struct: uint8 mech_state, uint8 endstop_dep, uint8 endstop_stow, uint8 spare,
    #              float motor_i, uint16 faults, uint16 timeout_cnt, uint16 seq_id, uint16 spare
    mech, endstop_dep, endstop_stow, spare, motor_amps, fault_raw, timeout_cnt, seq_id, spare2 = struct.unpack_from(
        "<BBBBfHHHH", payload, 0
    )
    mech_states = ["STOWED", "DEPLOYING", "DEPLOYED", "STOWING", "FAULT"]
    d = {
        "app": "GEO", "type": "STATUS",
        "mech_state": mech_states[mech] if mech < len(mech_states) else f"UNK({mech})",
        "endstop_deployed": bool(endstop_dep),
        "endstop_stowed": bool(endstop_stow),
        "motor_current_a": round(motor_amps, 3),
        "faults": decode_fault_bits(fault_raw),
        "timeout_count": timeout_cnt,
        "seq_id": seq_id,
    }
    # CP-3 per-pod placement (appended fields)
    if len(payload) >= 44:
        pod_states = ["CARRIED", "PLACING", "PLACED_OK", "MARGINAL", "RETRIEVING"]
        ps = struct.unpack_from("<BBBB", payload, 16)
        rs = struct.unpack_from("<BBBB", payload, 20)
        sf = struct.unpack_from("<ffff", payload, 24)
        placed, marginal, active, _sp = struct.unpack_from("<BBBB", payload, 40)
        d.update({
            "pod_state": [pod_states[s] if s < len(pod_states) else f"UNK({s})"
                          for s in ps],
            "pod_reseats": list(rs),
            "pod_seat_force_n": [round(v, 1) for v in sf],
            "placed_mask": placed,
            "marginal_mask": marginal,
            "active_pod": active if active != 0xFF else None,
        })
    return d

def decode_seis_tlm(data):
    """Decode SEIS HK telemetry (0x08D6) — matches WOK_Seis_HkTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<HHBBH"):
        return {"error": "short payload"}
    # Real struct: uint16 cmd_cnt, uint16 err_cnt, uint8 acquiring,
    #              uint8 source_state, uint16 fault_mask
    cmd_cnt, err_cnt, acquiring, source_state, fault_raw = struct.unpack_from("<HHBBH", payload, 0)
    return {
        "app": "SEIS", "type": "HK",
        "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
        "acquiring": bool(acquiring),
        "source_state": SOURCE_STATES[source_state] if source_state < len(SOURCE_STATES) else f"UNK({source_state})",
        "fault_raw": fault_raw,
        "faults": decode_fault_bits(fault_raw),
    }

def decode_seis_status_tlm(data):
    """Decode SEIS STATUS telemetry (0x08D7) — matches WOK_Seis_StatusTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<BBBBHHHH"):
        return {"error": "short payload"}
    # Struct prefix: uint8 acquiring, qc_clip, qc_dead, qc_drop,
    #                uint16 samples_k, duration, fs, faults,
    #                uint8 source_state, eval_flag, uint16 seq_id,
    #                float eval_confidence, float source_force_n
    acquiring, qc_clip, qc_dead, qc_drop, samples_k, duration, fs, fault_raw = struct.unpack_from(
        "<BBBBHHHH", payload, 0
    )
    # Try to decode extended fields if payload is large enough
    source_state = 0
    eval_flag = 0
    seq_id = 0
    eval_confidence = 0.0
    source_force = 0.0
    if len(payload) >= 24:
        source_state, eval_flag, seq_id, eval_confidence, source_force = struct.unpack_from(
            "<BBHff", payload, 12
        )
    # Appended fields: strike-to-trace sync + trace buffer fill
    strike_sample_idx = 0
    strike_valid = False
    trace_samples = 0
    if len(payload) >= 36:
        strike_sample_idx, strike_valid_u8, trace_samples = struct.unpack_from(
            "<IB3xI", payload, 24
        )
        strike_valid = bool(strike_valid_u8)
    # Stack + first-break picks (appended)
    first_break_ms = []
    pick_quality = []
    stack_fold = 0
    coherence_pct = 0
    if len(payload) >= 60:
        first_break_ms = [round(v, 2) for v in struct.unpack_from("<ffff", payload, 36)]
        q = struct.unpack_from("<BBBBBB", payload, 52)
        pick_quality = list(q[:4])
        stack_fold, coherence_pct = q[4], q[5]
    # Onboard refraction inversion (appended)
    eval_inv = {}
    if len(payload) >= 76:
        v1, v2, h1, icefrac = struct.unpack_from("<ffff", payload, 60)
        eval_inv = {"eval_v1_mps": round(v1, 0), "eval_v2_mps": round(v2, 0),
                    "eval_depth_m": round(h1, 2), "eval_ice_frac": round(icefrac, 2)}
    # EM source: mode, bank, sweep + correlation progress (appended)
    src = {}
    if len(payload) >= 84:
        src_mode, bank_soc, sweep_pct, corr_pct, bank_v = \
            struct.unpack_from("<BBBBf", payload, 76)
        src = {"source_mode": "SWEEP" if src_mode == 1 else "IMPULSE",
               "bank_v": round(bank_v, 1), "bank_soc_pct": bank_soc,
               "sweep_progress_pct": sweep_pct, "corr_progress_pct": corr_pct}
    # CP-2 plate coupling metric
    if len(payload) >= 96:
        plate_ratio, plate_base, plate_ok, plate_reseats, plate_degr = \
            struct.unpack_from("<ffBBB", payload, 84)
        src.update({
            "plate_ratio": round(plate_ratio, 1),
            "plate_baseline": round(plate_base, 1),
            "plate_ok": bool(plate_ok),
            "plate_reseats": plate_reseats,
            "plate_degraded": bool(plate_degr),
        })
    # CP-4 test-shot pod coupling scores
    if len(payload) >= 116:
        scores = struct.unpack_from("<ffff", payload, 96)
        ok_mask, score_valid, test_shots, _sp = struct.unpack_from("<BBBB", payload, 112)
        src.update({
            "pod_scores": [round(v, 2) for v in scores],
            "pod_score_ok_mask": ok_mask,
            "pod_scores_valid": bool(score_valid),
            "test_shots": test_shots,
        })
    # CP-5 sync/pick-consistency residuals
    if len(payload) >= 136:
        resid = struct.unpack_from("<ffff", payload, 116)
        susp = struct.unpack_from("<B", payload, 132)[0]
        src.update({
            "sync_residual_ms": [round(v, 2) for v in resid],
            "sync_suspect_mask": susp,
        })
    return {
        "app": "SEIS", "type": "STATUS",
        "acquiring": bool(acquiring),
        "qc_clip": bool(qc_clip),
        "qc_dead_mask": f"0x{qc_dead:02X}",
        "qc_dropout": bool(qc_drop),
        "qc_flags": qc_clip | (qc_dead << 1) | (qc_drop << 5),
        "samples_collected": samples_k * 1000,
        "duration_sec": duration,
        "sample_rate_hz": fs,
        "faults": decode_fault_bits(fault_raw),
        "source_state": SOURCE_STATES[source_state] if source_state < len(SOURCE_STATES) else f"UNK({source_state})",
        "eval_flag": bool(eval_flag),
        "eval_confidence": round(eval_confidence, 2),
        "source_force_n": round(source_force, 1),
        "seq_id": seq_id,
        "strike_sample_idx": strike_sample_idx,
        "strike_valid": strike_valid,
        "trace_samples": trace_samples,
        "first_break_ms": first_break_ms,
        "pick_quality": pick_quality,
        "stack_fold": stack_fold,
        "coherence_pct": coherence_pct,
        **eval_inv,
        **src,
    }

def decode_seis_quicklook(data):
    """Decode SEIS QUICKLOOK telemetry (0x08D8) — matches WOK_Seis_QuicklookTlm_t.
    Planar layout: float RMS[4], float Peak[4], float DC_Offset[4],
    uint16 SampleRate_Hz, uint16 Spare."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<4f4f4fHH"):
        return {"error": "short payload"}
    rms = struct.unpack_from("<4f", payload, 0)
    peak = struct.unpack_from("<4f", payload, 16)
    dc = struct.unpack_from("<4f", payload, 32)
    fs, _spare = struct.unpack_from("<HH", payload, 48)
    channels = [{
        "channel": i,
        "rms": round(rms[i], 2),
        "peak": round(peak[i], 2),
        "dc_offset": round(dc[i], 2),
    } for i in range(4)]
    return {
        "app": "SEIS", "type": "QUICKLOOK",
        "channels": channels,
        "sample_rate_hz": fs,
    }

def decode_hk_tlm(data):
    """Decode HK_HEALTH telemetry (0x08D9) — matches WOK_Hk_HealthTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<HHffffffHBBBBBB"):
        return {"error": "short payload"}
    # Real struct: uint16 cmd_cnt, uint16 err_cnt,
    #              float batt_v, batt_soc, temp_brd, temp_mot, tilt, cpu,
    #              uint16 faults, uint8 wdog, uint8 safe, uint8 seq_state, uint8 s1, s2, s3
    cmd_cnt, err_cnt, batt_v, batt_soc, board_temp, motor_temp, tilt, cpu_load, \
        fault_raw, watchdog_ok, safe_trigger, seq_state, s1, s2, s3 = struct.unpack_from(
        "<HHffffffHBBBBBB", payload, 0
    )
    faults = decode_fault_bits(fault_raw)
    return {
        "app": "HK", "type": "HEALTH",
        "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
        "battery_v": round(batt_v, 2),
        "battery_soc": round(batt_soc, 1),
        "board_temp_c": round(board_temp, 1),
        "motor_temp_c": round(motor_temp, 1),
        "tilt_deg": round(tilt, 1),
        "cpu_load_pct": round(cpu_load, 1),
        "fault_raw": fault_raw,
        "faults": faults,
        "watchdog_ok": bool(watchdog_ok),
        "safe_trigger": bool(safe_trigger),
        "seq_state": seq_state,
    }

def decode_drill_tlm(data):
    """Decode DRILL HK telemetry (0x08DA) — matches WOK_Drill_HkTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<HHBBHHHff"):
        return {"error": "short payload"}
    cmd_cnt, err_cnt, state, prev_state, fault_mask, seq_id, spare, arm_angle, depth = struct.unpack_from(
        "<HHBBHHHff", payload, 0
    )
    drill_states = ["IDLE", "DEPLOYING", "ARM_READY", "DRILLING", "DRILL_DONE",
                    "SAMPLING", "SAMPLE_DONE", "STOWING", "FAULT"]
    drill_faults = decode_drill_fault_bits(fault_mask)
    return {
        "app": "DRILL", "type": "HK",
        "cmd_cnt": cmd_cnt, "err_cnt": err_cnt,
        "state": drill_states[state] if state < len(drill_states) else f"UNK({state})",
        "prev_state": drill_states[prev_state] if prev_state < len(drill_states) else f"UNK({prev_state})",
        "fault_raw": fault_mask,
        "faults": drill_faults,
        "seq_id": seq_id,
        "arm_angle": round(arm_angle, 1),
        "drill_depth": round(depth, 3),
    }

def decode_drill_status_tlm(data):
    """Decode DRILL STATUS telemetry (0x08DB) — matches WOK_Drill_StatusTlm_t."""
    payload = data[16:]
    if len(payload) < struct.calcsize("<BBBB ff BBBB ffff BBBB fff"):
        return {"error": "short payload"}
    (arm_hal, _, _, _,
     arm_angle, arm_current,
     drill_hal, _, _, _,
     drill_depth, drill_rpm, drill_torque, drill_current,
     sample_hal, sample_quality, _, _,
     cell_temp, vapor_pressure, volatile_conc) = struct.unpack_from(
        "<BBBB ff BBBB ffff BBBB fff", payload, 0
    )
    arm_states = ["STOWED", "DEPLOYING", "READY", "STOWING"]
    drill_motor_states = ["IDLE", "DRILLING", "DONE"]
    sample_states = ["IDLE", "HEATING", "ANALYZING", "COOLING", "DONE"]
    d = {
        "app": "DRILL", "type": "STATUS",
        "arm_state": arm_states[arm_hal] if arm_hal < len(arm_states) else f"UNK({arm_hal})",
        "arm_angle": round(arm_angle, 1),
        "arm_current": round(arm_current, 2),
        "drill_motor_state": drill_motor_states[drill_hal] if drill_hal < len(drill_motor_states) else f"UNK({drill_hal})",
        "drill_depth": round(drill_depth, 3),
        "drill_rpm": round(drill_rpm, 0),
        "drill_torque": round(drill_torque, 2),
        "drill_current": round(drill_current, 2),
        "sample_state": sample_states[sample_hal] if sample_hal < len(sample_states) else f"UNK({sample_hal})",
        "sample_quality": sample_quality,
        "cell_temp": round(cell_temp, 1),
        "vapor_pressure": round(vapor_pressure, 1),
        "volatile_conc": round(volatile_conc, 1),
    }
    # Stiffness-log channels (appended)
    if len(payload) >= 56:
        wob, rop = struct.unpack_from("<ff", payload, 48)
        d.update({"wob_N": round(wob, 1), "rop_mps": round(rop, 4)})
    return d

def decode_drill_fault_bits(raw):
    """Decode drill-specific fault bitmask."""
    faults = []
    bits = {
        0x0001: "ARM_JAM",
        0x0002: "DRILL_STALL",
        0x0004: "OVER_DEPTH",
        0x0008: "THERMAL_OT",
        0x0010: "ARM_TIMEOUT",
        0x0020: "OVERCURRENT",
    }
    for mask, name in bits.items():
        if raw & mask:
            faults.append(name)
    return faults if faults else ["NONE"]

def decode_fault_bits(raw):
    """Decode fault bitmask to list of fault names."""
    faults = []
    bits = {
        0x0001: "BATTERY_LOW",
        0x0002: "TEMP_HIGH",
        0x0004: "TILT_EXCEED",
        0x0008: "WATCHDOG",
        0x0010: "COMM_LOSS",
        0x0100: "NAV_STUCK",
        0x0200: "NAV_SLIP",
        0x0400: "OVERCURRENT",
    }
    for mask, name in bits.items():
        if raw & mask:
            faults.append(name)
    return faults if faults else ["NONE"]


# Register decoders
TLM_DECODERS[WokMid.SEQ_TLM]        = decode_seq_tlm
TLM_DECODERS[WokMid.SEQ_MODE_TLM]   = decode_seq_mode_tlm
TLM_DECODERS[WokMid.NAV_TLM]        = decode_nav_tlm
TLM_DECODERS[WokMid.NAV_STATUS_TLM] = decode_nav_status_tlm
TLM_DECODERS[WokMid.GEO_TLM]        = decode_geo_tlm
TLM_DECODERS[WokMid.GEO_STATUS_TLM] = decode_geo_status_tlm
def decode_seis_filechunk(data):
    """Decode SEIS FILECHUNK (0x08DC) — metadata only for the web UI;
    reassembly is the console's job (seis.fetch)."""
    if len(data) < 16 + struct.calcsize("<24sIIHH"):
        return {"error": "short packet"}
    raw_name, fsize, offset, dlen = struct.unpack_from("<24sIIH", data, 16)
    return {
        "app": "SEIS", "type": "FILECHUNK",
        "file": raw_name.split(b"\x00")[0].decode(errors="replace"),
        "file_size": fsize,
        "offset": offset,
        "data_len": dlen,
        "progress_pct": round(100.0 * min(offset + dlen, fsize) / fsize, 1) if fsize else 0,
    }

TLM_DECODERS[WokMid.SEIS_TLM]       = decode_seis_tlm
TLM_DECODERS[WokMid.SEIS_STATUS_TLM]= decode_seis_status_tlm
TLM_DECODERS[WokMid.SEIS_QUICKLOOK] = decode_seis_quicklook
TLM_DECODERS[WokMid.SEIS_FILECHUNK] = decode_seis_filechunk
TLM_DECODERS[WokMid.HK_TLM]         = decode_hk_tlm
TLM_DECODERS[WokMid.DRILL_TLM]      = decode_drill_tlm
TLM_DECODERS[WokMid.DRILL_STATUS_TLM] = decode_drill_status_tlm


# ─── Command Builders ──────────────────────────────────────────────────

def build_command(app, action, params=None):
    """Build a cFS command from app name + action string."""
    params = params or {}

    # SeqId prefix for child app commands (SeqId=0 from ground = don't-care)
    SID = struct.pack("<HH", 0, 0)  # SeqId + SpareSeq

    cmds = {
        # SEQ (no SeqId — sequencer receives ground commands only)
        ("seq", "noop"):     (WokMid.SEQ_CMD, 0, b""),
        ("seq", "reset"):    (WokMid.SEQ_CMD, 1, b""),
        ("seq", "start"):    (WokMid.SEQ_CMD, 2, b""),
        ("seq", "abort"):    (WokMid.SEQ_CMD, 3, b""),
        ("seq", "resume"):   (WokMid.SEQ_CMD, 4, b""),
        ("seq", "waypoint"): (WokMid.SEQ_CMD, 5, struct.pack("<HH", params.get("idx", 0), 0)),
        ("seq", "mode"):     (WokMid.SEQ_CMD, 6, struct.pack("<BBBB",
                                params.get("mode", 1), 0, 0, 0)),
        ("seq", "loadwp"):   (WokMid.SEQ_CMD, 7, struct.pack("<HBBfffBBBB",
                                params.get("idx", 0), params.get("drill", 0),
                                params.get("src_mode", 0),
                                params.get("fwd", 0.0), params.get("lat", 0.0),
                                params.get("energy_j", 0.0),
                                params.get("recal", 0), 0, 0, 0)),
        ("seq", "wpcount"):  (WokMid.SEQ_CMD, 8, struct.pack("<HH",
                                params.get("count", 7), 0)),
        ("seq", "placement"): (WokMid.SEQ_CMD, 9, struct.pack("<BBBB",
                                params.get("placement", 0), 0, 0, 0)),
        ("seq", "release"):  (WokMid.SEQ_CMD, 10, b""),
        ("seq", "send_hk"):  (WokMid.SEQ_SEND_HK, 0, b""),
        # NAV
        ("nav", "noop"):     (WokMid.NAV_CMD, 0, SID),
        ("nav", "reset"):    (WokMid.NAV_CMD, 1, SID),
        ("nav", "goto"):     (WokMid.NAV_CMD, 2, SID + struct.pack("<ff", params.get("x", 0.0), params.get("y", 0.0))),
        ("nav", "stop"):     (WokMid.NAV_CMD, 3, SID),
        ("nav", "speed"):    (WokMid.NAV_CMD, 4, SID + struct.pack("<f", params.get("speed", 0.5))),
        # Recalibration (nav recal epoch): setpose = bookkeeping-only
        # ground-truth overwrite; realign = corrective in-place turn
        ("nav", "setpose"):  (WokMid.NAV_CMD, 5, SID + struct.pack("<ff",
                                params.get("x", 0.0), params.get("y", 0.0))),
        ("nav", "realign"):  (WokMid.NAV_CMD, 6, SID + struct.pack("<f",
                                params.get("error_deg", 0.0))),
        ("nav", "send_hk"):  (WokMid.NAV_SEND_HK, 0, b""),
        # GEO
        ("geo", "noop"):     (WokMid.GEO_CMD, 0, SID),
        ("geo", "reset"):    (WokMid.GEO_CMD, 1, SID),
        ("geo", "deploy"):   (WokMid.GEO_CMD, 2, SID),
        ("geo", "stow"):     (WokMid.GEO_CMD, 3, SID),
        ("geo", "abort"):    (WokMid.GEO_CMD, 4, SID),
        # Per-pod placement (CP-3) — contingency/manual paths
        ("geo", "place"):    (WokMid.GEO_CMD, 5, SID + struct.pack("<B3xff",
                                params.get("pod", 0),
                                params.get("x", 0.0), params.get("y", 0.0))),
        ("geo", "retrieve"): (WokMid.GEO_CMD, 6, SID + struct.pack("<B3x",
                                params.get("pod", 0))),
        ("geo", "reseat"):   (WokMid.GEO_CMD, 7, SID + struct.pack("<B3x",
                                params.get("pod", 0))),
        ("geo", "send_hk"):  (WokMid.GEO_SEND_HK, 0, b""),
        # SEIS
        ("seis", "noop"):    (WokMid.SEIS_CMD, 0, SID),
        ("seis", "reset"):   (WokMid.SEIS_CMD, 1, SID),
        ("seis", "start"):   (WokMid.SEIS_CMD, 2, SID + struct.pack("<HHBBHfffHH",
                                params.get("duration", 6),
                                params.get("fs", 4000),
                                params.get("gain", 1),
                                params.get("ch_mask", 0x0F),
                                0,  # spare
                                params.get("site_x", 0.0),
                                params.get("site_y", 0.0),
                                params.get("site_heading", 0.0),
                                params.get("stack_idx", 0),
                                params.get("stack_target", 1))),
        ("seis", "stop"):    (WokMid.SEIS_CMD, 3, SID),
        ("seis", "thump_arm"):  (WokMid.SEIS_CMD, 4, SID),
        ("seis", "thump_fire"): (WokMid.SEIS_CMD, 5, SID),
        ("seis", "thump_reset"):(WokMid.SEIS_CMD, 6, SID),
        ("seis", "evaluate"):   (WokMid.SEIS_CMD, 7, SID),
        # Dual-mode EM source config: zero params keep onboard
        # settings; mode 0 = impulse, 1 = sweep
        ("seis", "source"):  (WokMid.SEIS_CMD, 9, SID + struct.pack("<BBBBffff",
                                params.get("mode", 0), 0, 0, 0,
                                params.get("energy_j", 0.0),
                                params.get("f0_hz", 0.0),
                                params.get("f1_hz", 0.0),
                                params.get("dur_s", 0.0))),
        ("seis", "plate_reseat"): (WokMid.SEIS_CMD, 10, SID),
        ("seis", "fetch"):   (WokMid.SEIS_CMD, 8, SID
                               + str(params.get("filename", "")).encode()[:24].ljust(24, b"\x00")
                               + struct.pack("<I", params.get("chunk_idx", 0xFFFFFFFF))),
        ("seis", "send_hk"): (WokMid.SEIS_SEND_HK, 0, b""),
        # HK (no SeqId — not sequenced)
        ("hk", "noop"):      (WokMid.HK_CMD, 0, b""),
        ("hk", "reset"):     (WokMid.HK_CMD, 1, b""),
        ("hk", "thresholds"):(WokMid.HK_CMD, 2, struct.pack("<fff",
                                params.get("batt_v", 22.0),
                                params.get("temp_c", 55.0),
                                params.get("tilt_deg", 30.0))),
        ("hk", "clear"):     (WokMid.HK_CMD, 3, b""),
        ("hk", "send_hk"):   (WokMid.HK_SEND_HK, 0, b""),
        # DRILL
        ("drill", "noop"):    (WokMid.DRILL_CMD, 0, SID),
        ("drill", "reset"):   (WokMid.DRILL_CMD, 1, SID),
        ("drill", "deploy"):  (WokMid.DRILL_CMD, 2, SID + struct.pack("<f", params.get("angle", 90.0))),
        ("drill", "stow"):    (WokMid.DRILL_CMD, 3, SID),
        ("drill", "start"):   (WokMid.DRILL_CMD, 4, SID + struct.pack("<ffff",
                                 params.get("depth", 1.0), params.get("rpm", 200.0),
                                 params.get("site_x", 0.0), params.get("site_y", 0.0))),
        ("drill", "stop"):    (WokMid.DRILL_CMD, 5, SID),
        ("drill", "sample"):  (WokMid.DRILL_CMD, 6, SID + struct.pack("<HH",
                                 params.get("heat_sec", 60), params.get("cool_sec", 30))),
        ("drill", "stop_sample"): (WokMid.DRILL_CMD, 7, SID),
        ("drill", "abort"):   (WokMid.DRILL_CMD, 8, SID),
        ("drill", "send_hk"): (WokMid.DRILL_SEND_HK, 0, b""),
    }

    key = (app.lower(), action.lower())
    if key not in cmds:
        return None
    mid, cc, payload = cmds[key]
    return build_ccsds_cmd(mid, cc, payload)


# ─── Bridge Server ──────────────────────────────────────────────────────

class WokBridge:
    """Pure relay: every telemetry value originates from the FSW (HAL-
    backed) via TO_LAB; every command goes to CI_LAB. Nothing is
    fabricated here — to run without hardware, run cFS with the
    linux_sim HAL."""

    def __init__(self, cfs_host, cmd_port, tlm_port, ws_port, ws_host="127.0.0.1"):
        self.cfs_host = cfs_host
        self.cmd_port = cmd_port
        self.tlm_port = tlm_port
        self.ws_port = ws_port
        self.ws_host = ws_host
        self.clients = set()
        self.latest_tlm = {}
        self.cmd_sock = None
        self.tlm_sock = None
        self.running = False

    async def start(self):
        self.running = True

        # Set up UDP sockets. The telemetry socket is bound to the address
        # TO_LAB is told to send to (127.0.0.1 for a local cFS, this host's
        # LAN address for a remote one) rather than to all interfaces.
        tlm_bind = local_ip_toward(self.cfs_host)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tlm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tlm_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tlm_sock.bind((tlm_bind, self.tlm_port))
        self.tlm_sock.setblocking(False)

        # Start WebSocket server
        print(f"[WOK Bridge] WebSocket server on ws://{self.ws_host}:{self.ws_port}")
        if self.ws_host not in ("127.0.0.1", "localhost", "::1"):
            print("[WOK Bridge] WARNING: WebSocket exposed beyond localhost with NO"
                  " authentication — trusted LAN only")
        print(f"[WOK Bridge] cFS command target: {self.cfs_host}:{self.cmd_port}")
        print(f"[WOK Bridge] Telemetry listen: {tlm_bind}:{self.tlm_port}")

        async with websockets.serve(self.ws_handler, self.ws_host, self.ws_port):
            await self.tlm_recv_loop()

    async def ws_handler(self, websocket):
        self.clients.add(websocket)
        client_addr = websocket.remote_address
        print(f"[WOK Bridge] Client connected: {client_addr}")
        tlm_dest = local_ip_toward(self.cfs_host)
        pkt = build_to_lab_enable(tlm_dest)
        self.cmd_sock.sendto(pkt, (self.cfs_host, self.cmd_port))
        print(f"[WOK Bridge] TO_LAB enable sent (telemetry -> {tlm_dest})")

        # Send current telemetry state
        if self.latest_tlm:
            try:
                await websocket.send(json.dumps({
                    "type": "tlm_snapshot",
                    "data": self.latest_tlm,
                    "timestamp": time.time(),
                }))
            except websockets.exceptions.ConnectionClosed:
                pass

        try:
            async for message in websocket:
                await self.handle_ws_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            print(f"[WOK Bridge] Client disconnected: {client_addr}")

    async def handle_ws_message(self, websocket, message):
        """Handle incoming WebSocket message (command from browser)."""
        try:
            msg = json.loads(message)
        except json.JSONDecodeError:
            await websocket.send(json.dumps({"type": "error", "msg": "invalid JSON"}))
            return

        msg_type = msg.get("type")

        if msg_type == "command":
            app = msg.get("app", "")
            action = msg.get("action", "")
            params = msg.get("params", {})

            pkt = build_command(app, action, params)
            if pkt is None:
                await websocket.send(json.dumps({
                    "type": "cmd_ack",
                    "status": "error",
                    "msg": f"Unknown command: {app}.{action}",
                }))
                return

            try:
                self.cmd_sock.sendto(pkt, (self.cfs_host, self.cmd_port))
                ack = {"type": "cmd_ack", "status": "ok", "app": app, "action": action, "size": len(pkt)}
            except Exception as e:
                ack = {"type": "cmd_ack", "status": "error", "msg": str(e)}

            await websocket.send(json.dumps(ack))

        elif msg_type == "allnoop":
            for app in ["seq", "nav", "geo", "seis", "hk", "drill"]:
                pkt = build_command(app, "noop")
                if pkt:
                    self.cmd_sock.sendto(pkt, (self.cfs_host, self.cmd_port))
            await websocket.send(json.dumps({"type": "cmd_ack", "status": "ok", "action": "allnoop"}))

        elif msg_type == "allhk":
            for app in ["seq", "nav", "geo", "seis", "hk", "drill"]:
                pkt = build_command(app, "send_hk")
                if pkt:
                    self.cmd_sock.sendto(pkt, (self.cfs_host, self.cmd_port))
            await websocket.send(json.dumps({"type": "cmd_ack", "status": "ok", "action": "allhk"}))

        elif msg_type == "safestop":
            for app, action in [("seis", "stop"), ("drill", "abort"), ("nav", "stop"), ("geo", "stow"), ("seq", "abort")]:
                pkt = build_command(app, action)
                if pkt:
                    self.cmd_sock.sendto(pkt, (self.cfs_host, self.cmd_port))
            await websocket.send(json.dumps({"type": "cmd_ack", "status": "ok", "action": "safestop"}))

        elif msg_type == "ping":
            await websocket.send(json.dumps({"type": "pong", "timestamp": time.time()}))

    async def tlm_recv_loop(self):
        """Receive UDP telemetry and broadcast to WebSocket clients."""
        loop = asyncio.get_running_loop()
        while self.running:
            try:
                # Event-loop-native receive on the non-blocking socket:
                # no executor thread, no busy-poll
                data = await loop.sock_recv(self.tlm_sock, 4096)
                if len(data) < 6:
                    continue

                hdr = decode_tlm_header(data)
                if hdr is None:
                    continue
                mid, seq_count, length, offset = hdr

                decoder = TLM_DECODERS.get(mid)
                if decoder:
                    decoded = decoder(data)
                    decoded["mid"] = f"0x{mid:04X}"
                    decoded["seq"] = seq_count
                    decoded["raw_size"] = len(data)
                    self.latest_tlm[f"0x{mid:04X}"] = decoded

                    msg = json.dumps({
                        "type": "telemetry",
                        "mid": f"0x{mid:04X}",
                        "data": decoded,
                        "timestamp": time.time(),
                    })
                    self.broadcast(msg)
                else:
                    # Unknown MID — broadcast raw hex for debugging
                    msg = json.dumps({
                        "type": "raw_tlm",
                        "mid": f"0x{mid:04X}",
                        "hex": data.hex(),
                        "size": len(data),
                        "timestamp": time.time(),
                    })
                    self.broadcast(msg)

            except Exception as e:
                if self.running:
                    print(f"[WOK Bridge] TLM error: {e}")
                    await asyncio.sleep(0.1)

    # ─── Client Broadcast ───────────────────────────────────────────

    def broadcast(self, message):
        """Send message to all connected WebSocket clients. Closed
        connections are skipped by websockets.broadcast and removed from
        self.clients by their own ws_handler on exit."""
        if self.clients:
            websockets.broadcast(self.clients, message)


# ─── Main ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="WOK Ground Control — WebSocket Bridge")
    parser.add_argument("--cfs-host", default="127.0.0.1", help="cFS host IP")
    parser.add_argument("--cmd-port", type=int, default=1234, help="CI_LAB UDP port")
    parser.add_argument("--tlm-port", type=int, default=2234, help="TO_LAB UDP port")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket server port")
    parser.add_argument("--ws-host", default="127.0.0.1",
                        help="WebSocket listen address (default 127.0.0.1; "
                             "0.0.0.0 exposes the UNAUTHENTICATED bridge to the LAN)")
    args = parser.parse_args()

    bridge = WokBridge(args.cfs_host, args.cmd_port, args.tlm_port, args.ws_port, args.ws_host)
    try:
        asyncio.run(bridge.start())
    except KeyboardInterrupt:
        print("\n[WOK Bridge] Shutting down.")

if __name__ == "__main__":
    main()
