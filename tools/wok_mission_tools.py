#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""Mission-run toolkit for WOK verification (verification tooling).

Usage: wok_mission_tools.py [--host H] [--cmd-port P] [--tlm-port P] <subcommand> ...

Targets 127.0.0.1:1234 (CI_LAB) / 2234 (TO_LAB) by default. To drive a
second WOK instance either pass the ports explicitly or set
AEGIS_PORT_OFFSET=<n> (the same offset the core was started with): the
ports become 1234+n / 2234+n.

Subcommands:
  enable_tlm                       — TO_LAB enable (bare-IP payload, once per boot)
  start | abort | mode <m> | wpcount <n> | placement <0|1>
  loadwp <idx> <fwd> <lat> <drill> <src> <energy> [recal]
  setpose <x> <y> | realign <err_deg> | release — nav recalibration + hold release
  source <mode> [energy f0 f1 dur] — SEIS CONFIG_SOURCE
  fetch <file> <outdir>            — stop-and-wait chunk fetch to outdir
  validate <dir>                   — parse stack files, compare picks vs geology truth
  events <log>                     — normalized decisive-event stream from a core log
"""
import socket, struct, sys, time, math, os, glob, re

PORT_OFFSET = int(os.environ.get("AEGIS_PORT_OFFSET", "0") or 0)
CMD_ADDR = ("127.0.0.1", 1234 + PORT_OFFSET)
TLM_PORT = 2234 + PORT_OFFSET

MID = {"SEQ": 0x18D0, "NAV": 0x18D2, "GEO": 0x18D4, "SEIS": 0x18D6,
       "HK": 0x18D8, "DRILL": 0x18DA, "TO_LAB": 0x1880}
SID = struct.pack("<HH", 0, 0)
seq_counter = [0]

def send(mid, cc, payload=b""):
    seq_counter[0] += 1
    total = 8 + len(payload)
    pkt = struct.pack(">HHH", mid, 0xC000 | (seq_counter[0] & 0x3FFF),
                      total - 7) + struct.pack("BB", cc, 0) + payload
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(pkt, CMD_ADDR)
    s.close()

def enable_tlm():
    send(MID["TO_LAB"], 6, b"127.0.0.1".ljust(16, b"\x00"))
    print("TO_LAB enabled -> 127.0.0.1")

# ── Geology truth (port of wok_hal_sim_geology.c) ─────────────────────
PODS = [1.5, 3.0, 4.5, 6.0]

def truth_arrival_ms(x, y, off):
    h1 = 1.5 + 0.5 * math.sin(0.4 * x) * math.cos(0.3 * y)
    ice = 0.003 + 5.0 * math.exp(-((x - 6.0) ** 2 + (y - 3.0) ** 2)
                                 / (2 * 2.5 ** 2))
    v1, v2 = 250.0, 800.0 + 350.0 * ice
    t_dir = off / v1
    t_head = t_dir
    if v2 > v1:
        s = v1 / v2
        t_head = off / v2 + 2 * h1 * math.sqrt(1 - s * s) / v1
    return min(t_dir, t_head) * 1000.0

# ── .wok parsing ───────────────────────────────────────────────────────
SEIS_HDR = "<IHHHHIIBBHffffffffIBBHffffBBBB"
V3EXT = "<B3xffffff"

def parse_seis(path):
    raw = open(path, "rb").read()
    h = struct.unpack_from(SEIS_HDR, raw, 0)
    d = dict(zip(["magic", "ver", "nch", "fs", "gain", "spc", "strike",
                  "svalid", "chmask", "seqid", "force", "sx", "sy", "shdg",
                  "p0", "p1", "p2", "p3", "acqidx", "fold", "coh", "sp",
                  "fb0", "fb1", "fb2", "fb3", "q0", "q1", "q2", "q3"], h))
    d["hdr_size"] = 84
    if d["ver"] >= 3:
        (d["src_mode"], d["energy"], d["f0"], d["f1"], d["sdur"],
         d["bankv"], d["banksoc"]) = struct.unpack_from(V3EXT, raw, 84)
        d["hdr_size"] = 112
    d["nsamp_bytes"] = len(raw) - d["hdr_size"]
    d["file"] = os.path.basename(path)
    return d

def validate(dirpath):
    ok = True
    for p in sorted(glob.glob(os.path.join(dirpath, "stack_*.wok"))):
        d = parse_seis(p)
        picks = [d["fb0"], d["fb1"], d["fb2"], d["fb3"]]
        mode = "sweep" if d.get("src_mode") == 1 else "impulse"
        errs = []
        for ch, pk in enumerate(picks):
            t = truth_arrival_ms(d["sx"], d["sy"], PODS[ch])
            errs.append(pk - t if pk >= 0 else float("nan"))
        worst = max((abs(e) for e in errs if not math.isnan(e)), default=99)
        stat = "OK" if worst <= 0.6 else "CHECK"
        if stat != "OK":
            ok = False
        print(f"{d['file']} v{d['ver']} {mode:7s} site=({d['sx']:.1f},{d['sy']:.1f}) "
              f"fold={d['fold']} coh={d['coh']} picks={['%.2f' % p for p in picks]} "
              f"err_ms={['%+.2f' % e for e in errs]} worst={worst:.2f} {stat}")
    return ok

# ── Chunk fetch (stop-and-wait, mirrors console seis.fetch) ───────────
def fetch(fname, outdir):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", TLM_PORT))
    sock.settimeout(0.2)
    name24 = fname.encode()[:24].ljust(24, b"\x00")
    parts, fsize = {}, None

    def request(idx):
        send(MID["SEIS"], 8, SID + name24 + struct.pack("<I", idx))

    def wait_chunk(idx, tries=8):
        nonlocal fsize
        for _ in range(tries):
            request(idx)
            t_end = time.time() + 0.6
            while time.time() < t_end:
                try:
                    data, _ = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                if len(data) < 6 or struct.unpack(">H", data[:2])[0] != 0x08DC:
                    continue
                raw_name, fs_, off, dlen = struct.unpack_from("<24sIIH", data, 16)
                nm = raw_name.split(b"\x00")[0].decode(errors="replace")
                if nm != fname:
                    continue
                fsize = fs_
                parts[off] = data[16 + 36:16 + 36 + dlen]
                if off == idx * 1024:
                    return True
        return False

    if not wait_chunk(0):
        print(f"FETCH FAIL {fname}: no response")
        return False
    n = (fsize + 1023) // 1024
    for i in range(1, n):
        if not wait_chunk(i):
            print(f"FETCH FAIL {fname}: chunk {i}/{n}")
            return False
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, fname), "wb") as f:
        for off in sorted(parts):
            f.seek(off)
            f.write(parts[off])
    print(f"FETCHED {fname} ({fsize} bytes, {n} chunks) -> {outdir}")
    sock.close()
    return True

# ── Event normalization (regression compare) ──────────────────────────
DECISIVE = [
    r"WOK_SEQ: (\w+) → (\w+)",              # state transitions
    r"Ice signature .*DRILLING at WP \d+",
    r"not drill-flagged",
    r"skip drill",
    r"Drill cycle complete at WP \d+",
    r"Tie-point gate",
    r"Targeting WP",
    r"All \d+ waypoints complete",
    r"Evaluate — flag=\d+",
]

def events(logpath):
    pat = re.compile("|".join(f"({p})" for p in DECISIVE))
    for line in open(logpath, errors="replace"):
        if "WOK_SEQ" not in line and "WOK_SEIS: Evaluate" not in line:
            continue
        m = pat.search(line)
        if m:
            # strip EVS prefix/timestamps: keep from 'WOK_'
            i = line.find("WOK_")
            txt = line[i:].strip()
            # normalize numbers that legitimately drift (confidence %)
            txt = re.sub(r"\d+% conf", "N% conf", txt)
            txt = re.sub(r"conf=\d+%", "conf=N%", txt)
            print(txt)

if __name__ == "__main__":
    a = sys.argv[1:]
    # Leading --host/--cmd-port/--tlm-port options (override AEGIS_PORT_OFFSET)
    while len(a) >= 2 and a[0] in ("--host", "--cmd-port", "--tlm-port"):
        opt, val = a[0], a[1]
        if opt == "--host":
            CMD_ADDR = (val, CMD_ADDR[1])
        elif opt == "--cmd-port":
            CMD_ADDR = (CMD_ADDR[0], int(val))
        else:
            TLM_PORT = int(val)
        a = a[2:]
    if not a:
        print(__doc__); sys.exit(2)
    c = a[0]
    if c == "enable_tlm": enable_tlm()
    elif c == "start":   send(MID["SEQ"], 2)
    elif c == "abort":   send(MID["SEQ"], 3)
    elif c == "mode":    send(MID["SEQ"], 6, struct.pack("<BBBB", int(a[1]), 0, 0, 0))
    elif c == "placement": send(MID["SEQ"], 9, struct.pack("<BBBB", int(a[1]), 0, 0, 0))
    elif c == "wpcount": send(MID["SEQ"], 8, struct.pack("<HH", int(a[1]), 0))
    elif c == "loadwp":
        recal = int(a[7]) if len(a) > 7 else 0
        send(MID["SEQ"], 7, struct.pack("<HBBfffBBBB", int(a[1]), int(a[4]),
             int(a[5]), float(a[2]), float(a[3]), float(a[6]), recal, 0, 0, 0))
    elif c == "release": send(MID["SEQ"], 10)
    elif c == "setpose":
        send(MID["NAV"], 5, SID + struct.pack("<ff", float(a[1]), float(a[2])))
    elif c == "realign":
        send(MID["NAV"], 6, SID + struct.pack("<f", float(a[1])))
    elif c == "source":
        e = float(a[2]) if len(a) > 2 else 0.0
        f0 = float(a[3]) if len(a) > 3 else 0.0
        f1 = float(a[4]) if len(a) > 4 else 0.0
        du = float(a[5]) if len(a) > 5 else 0.0
        send(MID["SEIS"], 9, SID + struct.pack("<BBBBffff", int(a[1]), 0, 0, 0, e, f0, f1, du))
    elif c == "fetch":   sys.exit(0 if fetch(a[1], a[2]) else 1)
    elif c == "validate": sys.exit(0 if validate(a[1]) else 1)
    elif c == "events":  events(a[1])
    else: print(__doc__); sys.exit(2)
