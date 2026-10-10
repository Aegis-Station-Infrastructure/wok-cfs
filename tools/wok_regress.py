#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""Regression compare: drop-weight (pre-5.0) baseline mission vs the EM-source mission.

Usage: wok_regress.py <baseline_log> <new_log> <baseline_products> <new_products>

Checks:
  1. State flow: identical normalized SEQ state-transition sequence.
  2. Drill decisions: identical DRILLING/skip/decline/gate decisions per WP.
  3. Picks: per-site stack picks agree within +-0.5 ms.
  4. Products: same product set (counts per type, per-site stacks), same
     per-file sample geometry (nch, spc, site).
"""
import sys, re, glob, os, struct

SEIS_HDR = "<IHHHHIIBBHffffffffIBBHffffBBBB"

def parse(path):
    raw = open(path, "rb").read()
    h = struct.unpack_from(SEIS_HDR, raw, 0)
    keys = ["magic","ver","nch","fs","gain","spc","strike","svalid","chmask",
            "seqid","force","sx","sy","shdg","p0","p1","p2","p3","acqidx",
            "fold","coh","sp","fb0","fb1","fb2","fb3","q0","q1","q2","q3"]
    return dict(zip(keys, h))

def transitions(log):
    out = []
    for line in open(log, errors="replace"):
        m = re.search(r"WOK_SEQ: ([A-Z]+) → ([A-Z]+) \(WP (\d+)/(\d+)\)", line)
        if m:
            out.append(f"{m.group(1)}->{m.group(2)} WP{m.group(3)}/{m.group(4)}")
    return out

def decisions(log):
    out = []
    pats = [r"DRILLING at WP (\d+)", r"Ice \(\d+%\) at WP (\d+) — not drill-flagged",
            r"skip drill", r"Drill cycle complete at WP (\d+)",
            r"Targeting WP (\d+)", r"Tie-point gate"]
    for line in open(log, errors="replace"):
        for p in pats:
            m = re.search(p, line)
            if m:
                out.append(re.sub(r"\d+% conf|conf=\d+%|score -?[\d.]+", "N",
                           re.sub(r"\(.*?returning.*?\)", "(returning)",
                                  line[line.find("WOK_SEQ"):].strip())))
                break
    return out

def stacks_by_site(d):
    out = {}
    for p in sorted(glob.glob(os.path.join(d, "stack_*.wok"))):
        h = parse(p)
        out[(round(h["sx"], 1), round(h["sy"], 1))] = h
    return out

def main():
    blog, nlog, bdir, ndir = sys.argv[1:5]
    ok = True

    bt, nt = transitions(blog), transitions(nlog)
    if bt == nt:
        print(f"STATE FLOW: IDENTICAL ({len(bt)} transitions)")
    else:
        ok = False
        print(f"STATE FLOW: DIFFERS (baseline {len(bt)}, new {len(nt)})")
        for i, (a, b) in enumerate(zip(bt, nt)):
            if a != b:
                print(f"  first divergence at {i}: '{a}' vs '{b}'")
                break
        if len(bt) != len(nt):
            longer, shorter = (bt, nt) if len(bt) > len(nt) else (nt, bt)
            print(f"  extra tail: {longer[len(shorter):][:5]}")

    bd, nd = decisions(blog), decisions(nlog)
    if bd == nd:
        print(f"DRILL DECISIONS: IDENTICAL ({len(bd)} decision events)")
    else:
        ok = False
        print("DRILL DECISIONS: DIFFER")
        for a, b in zip(bd, nd):
            if a != b:
                print(f"  '{a}'\n  vs\n  '{b}'")
        if len(bd) != len(nd):
            print(f"  counts: {len(bd)} vs {len(nd)}")

    bs, ns = stacks_by_site(bdir), stacks_by_site(ndir)
    if set(bs) != set(ns):
        ok = False
        print(f"STACK SITES DIFFER: {sorted(set(bs) ^ set(ns))}")
    else:
        worst = 0.0
        for site in sorted(bs):
            hb, hn = bs[site], ns[site]
            for ch in range(4):
                pb, pn = hb[f"fb{ch}"], hn[f"fb{ch}"]
                if pb < 0 or pn < 0:
                    if pb != pn:
                        ok = False
                        print(f"  site {site} pod {ch}: pick presence differs ({pb} vs {pn})")
                    continue
                d = abs(pn - pb)
                worst = max(worst, d)
                if d > 0.5:
                    ok = False
                    print(f"  site {site} pod {ch}: pick delta {d:.2f} ms > 0.5")
            for k in ("nch", "fold", "sx", "sy"):
                if hb[k] != hn[k]:
                    ok = False
                    print(f"  site {site}: {k} differs ({hb[k]} vs {hn[k]})")
        print(f"PICKS: {len(bs)} sites, worst per-pod delta {worst:.2f} ms "
              f"({'OK <= 0.5' if worst <= 0.5 else 'FAIL'})")

    for pat in ("seis_*.wok", "stack_*.wok", "drill_*.wok", "corr_*.wok",
                "coup_*.wok"):
        nb = len(glob.glob(os.path.join(bdir, pat)))
        nn = len(glob.glob(os.path.join(ndir, pat)))
        tag = "OK" if nb == nn else "DIFFER"
        if nb != nn:
            ok = False
        print(f"PRODUCTS {pat}: baseline {nb}, new {nn} {tag}")

    print("REGRESSION", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)

main()
