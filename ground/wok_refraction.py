#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
WOK ground processing — refraction inversion + volatile analysis.

Consumes the onboard data products (stack_NNNN.wok, drill_NNNN.wok — fetched
with `seis.fetch` into wok_archive/, or read straight from the sim data dir)
and produces the per-site prospecting quantities:

  Seismic (per stack file):
    - v1 (direct-wave / layer-1 velocity) from the near pod pair
    - v2 (refractor velocity) and h1 (layer-1 thickness) from a two-layer
      grid inversion over the onboard first-break picks, WITH uncertainty:
      the [lo, hi] ranges span every (v2, h1) cell whose misfit is within
      the pick-noise tolerance of the optimum
    - ice fraction: share of near-optimal cells with an ice-fast refractor

  Drill (per log file):
    - regolith competence log (torque vs depth) and the drill-measured
      layer interface (first sustained torque step)
    - evolved-gas decomposition: the (cell temp, cumulative ppm) heating
      curve is fit as a non-negative mix of three release modes (pore ice
      ~100 C, adsorbed film ~160 C, hydrated mineral ~320 C). The modes
      are the simulator's assumed values (wok_hal_sim_geology.c); replace
      them with TGA/EGA lab calibration before use on real samples.
      This recovers the TOTAL volatile abundance (including what stayed
      bound above the 300 C cell limit) and the binding fractions that
      drive extraction cost.

  Tie points (sites with both products):
    - drill-measured interface depth vs seismic h1
    - measured wt% vs seismic v2 — the calibration pair the Vp→ice
      transfer function (Phase 4) is built from

Usage:
    python3 wok_refraction.py [data_dir] [--csv out.csv]

Pure stdlib — no numpy/scipy required.
"""

import struct, glob, math, os, sys, csv

# ── File formats (must match wok_seis_msg.h / wok_drill_msgids.h) ──────
SEIS_HDR = "<IHHHHIIBBHffffffffIBBBBffffBBBB"     # 84 bytes, v2 base
SEIS_HDR_SIZE = struct.calcsize(SEIS_HDR)
# v3 appends the EM-source shot context (dual-mode source, 112 bytes
# total): mode, energy, sweep pilot params, bank state. The inversion
# is unchanged for sweep-mode stacks — they are stacks of onboard-
# correlated equivalent-impulse traces, so picks/geometry read the same.
SEIS_HDR_V3EXT = "<B3xffffff"                      # +28 bytes
DRILL_HDR = "<IHHHHffffHHII"                       # 40 bytes
DRILL_ROW = "<IB3xfffffff"                         # 36 bytes
SEIS_MAGIC, DRILL_MAGIC = 0x53454953, 0x4452494C

# ── Inversion tuning (mirror the onboard evaluate) ─────────────────────
V2_MIN, V2_MAX, V2_STEP = 300.0, 4050.0, 25.0      # finer than onboard
H1_MIN, H1_MAX, H1_STEP = 0.5, 3.0, 0.05
PICK_SIGMA_MS = 0.35
V2_ICE_MPS = 1200.0

# ── Evolved-gas release modes (deg C) — sim-assumed, mirror the values in
#    wok_hal_sim_geology.c; replace with lab TGA/EGA calibration ─────────
RELEASE_MODES = [("pore_ice", 100.0, 20.0),
                 ("adsorbed", 160.0, 30.0),
                 ("hydrated", 320.0, 30.0)]


def release_cdf(t, mu, sig):
    return 1.0 / (1.0 + math.exp(-(t - mu) / (0.6 * sig)))


# ── Seismic inversion ───────────────────────────────────────────────────
COUP_MAGIC = 0x434F5550   # 'COUP'
COUP_HDR = "<IHH11f2B4B2B2B2xff"
COUP_V2EXT = "<4fB3x"      # sync residuals + suspect mask (CP-5)


def load_coupling(data_dir):
    """Coupling records (coup_NNNN.wok, CP-4/5) keyed by rounded site.
    Returns {site: {"scores": [...], "ok_mask", "verified", "attempts",
    "marginal_mask", "test_shots", "sync_residual_ms", "sync_suspect"}}."""
    out = {}
    for p in sorted(glob.glob(f"{data_dir}/coup_*.wok")):
        raw = open(p, "rb").read()
        if len(raw) < struct.calcsize(COUP_HDR):
            continue
        h = struct.unpack_from(COUP_HDR, raw, 0)
        if h[0] != COUP_MAGIC:
            continue
        (magic, ver, seqid, sx, sy, shdg,
         o0, o1, o2, o3, s0, s1, s2, s3,
         ok_mask, marg_mask, a0, a1, a2, a3, verified, shots,
         pdeg, pres, pratio, pbase) = h
        rec = {
            "file": os.path.basename(p), "scores": [s0, s1, s2, s3],
            "ok_mask": ok_mask, "marginal_mask": marg_mask,
            "attempts": [a0, a1, a2, a3], "verified": verified,
            "test_shots": shots, "plate_degraded": pdeg,
            "sync_residual_ms": None, "sync_suspect": 0,
        }
        if ver >= 2 and len(raw) >= struct.calcsize(COUP_HDR) + struct.calcsize(COUP_V2EXT):
            r0, r1, r2, r3, susp = struct.unpack_from(
                COUP_V2EXT, raw, struct.calcsize(COUP_HDR))
            rec["sync_residual_ms"] = [r0, r1, r2, r3]
            rec["sync_suspect"] = susp
        out[(round(sx, 1), round(sy, 1))] = rec
    return out


def pod_weights(coup):
    """Channel weights for the pick misfit: verified pods weigh 1;
    low-coupling pods deweight by score^2 (pick timing variance rises
    as SNR drops; amplitude ratio ~ score), floored at 0.05 so a bad
    pod still constrains, weakly. Deweight only — never boost."""
    if not coup:
        return [1.0, 1.0, 1.0, 1.0]
    w = []
    for s in coup["scores"]:
        if s < 0:
            w.append(1.0)
        else:
            w.append(max(0.05, min(1.0, s * s)))
    return w


def invert_stack(path, coup=None):
    raw = open(path, "rb").read()
    h = struct.unpack_from(SEIS_HDR, raw, 0)
    (magic, ver, nch, fs, gain, spc, strike, svalid, chmask, seqid,
     force, sx, sy, shdg, p0, p1, p2, p3, acqidx,
     fold, coh, plate_degr, plate_reseats, fb0, fb1, fb2, fb3,
     q0, q1, q2, q3) = h
    if magic != SEIS_MAGIC:
        return None
    weights = pod_weights(coup)
    src_mode = 0
    if ver >= 3:
        src_mode, energy, f0hz, f1hz, sdur, bankv, banksoc = \
            struct.unpack_from(SEIS_HDR_V3EXT, raw, SEIS_HDR_SIZE)
    pods, picks, quals = [p0, p1, p2, p3], [fb0, fb1, fb2, fb3], [q0, q1, q2, q3]
    if any(p < 0 for p in picks) or any(q == 0 for q in quals):
        return {"file": os.path.basename(path), "site": (sx, sy),
                "error": "incomplete picks"}
    if picks[1] <= picks[0]:
        return {"file": os.path.basename(path), "site": (sx, sy),
                "error": "non-monotonic near picks"}

    v1 = (pods[1] - pods[0]) / ((picks[1] - picks[0]) * 1e-3)

    def sse_at(v2, h1):
        sin_ic = v1 / v2
        cos_ic = math.sqrt(1.0 - sin_ic * sin_ic)
        ti = 2.0 * h1 * cos_ic / v1 * 1e3
        s = 0.0
        for x, t, w in zip(pods, picks, weights):
            t_pred = min(x / v1 * 1e3, x / v2 * 1e3 + ti)
            s += w * (t - t_pred) ** 2
        return s

    best = (1e30, None, None)
    v2 = V2_MIN
    while v2 <= V2_MAX:
        if v2 > v1:
            h1 = H1_MIN
            while h1 <= H1_MAX:
                s = sse_at(v2, h1)
                if s < best[0]:
                    best = (s, v2, h1)
                h1 += H1_STEP
        v2 += V2_STEP
    sse_min, v2_best, h1_best = best

    # Likelihood over the grid: L = exp(-ΔSSE/2σ²). ice_frac is the
    # posterior mass (likelihood × prior) on ice-fast refractors; the
    # [lo, hi] ranges span cells within the ~3σ contour of the LIKELIHOOD
    # alone, so the prior does not narrow or shift the quoted ranges.
    two_sig2 = 2 * PICK_SIGMA_MS ** 2
    range_cut = math.exp(-4.5)          # 3-sigma likelihood ratio
    v2_lo = v2_hi = v2_best
    h1_lo = h1_hi = h1_best
    w_total = w_ice = 0.0
    v2 = V2_MIN
    while v2 <= V2_MAX:
        if v2 > v1:
            h1 = H1_MIN
            while h1 <= H1_MAX:
                lik = math.exp(-(sse_at(v2, h1) - sse_min) / two_sig2)
                # Uniform-in-slowness prior: high-v2 cells are nearly
                # indistinguishable (t_head → intercept), so uniform-in-v2
                # would let sheer grid volume masquerade as ice evidence.
                w = lik * (1000.0 / v2) ** 2
                w_total += w
                if v2 > V2_ICE_MPS:
                    w_ice += w
                if lik >= range_cut:
                    v2_lo, v2_hi = min(v2_lo, v2), max(v2_hi, v2)
                    h1_lo, h1_hi = min(h1_lo, h1), max(h1_hi, h1)
                h1 += H1_STEP
        v2 += V2_STEP

    return {"file": os.path.basename(path), "site": (sx, sy), "fold": fold,
            "source": "sweep" if src_mode == 1 else "impulse",
            "plate_degraded": bool(plate_degr), "plate_reseats": plate_reseats,
            "weights": weights,
            "coherence": coh, "picks_ms": picks, "v1": v1,
            "v2": v2_best, "v2_lo": v2_lo, "v2_hi": v2_hi,
            "h1": h1_best, "h1_lo": h1_lo, "h1_hi": h1_hi,
            "ice_frac": w_ice / w_total if w_total else 0.0,
            "rms_ms": math.sqrt(sse_min / max(sum(weights), 1e-6))}


# ── Drill log analysis ──────────────────────────────────────────────────
def solve_release_weights(temps, ppms):
    """Non-negative least squares (3 unknowns) via normal equations with
    an active-set clamp: fit cumulative ppm(T) = sum(w_i * cdf_i(T))."""
    basis = [[release_cdf(t, mu, sig) for (_, mu, sig) in RELEASE_MODES]
             for t in temps]
    active = [0, 1, 2]
    while True:
        n = len(active)
        A = [[sum(basis[r][active[i]] * basis[r][active[j]]
                  for r in range(len(temps))) for j in range(n)]
             for i in range(n)]
        b = [sum(basis[r][active[i]] * ppms[r] for r in range(len(temps)))
             for i in range(n)]
        # Gaussian elimination
        M = [row[:] + [b[i]] for i, row in enumerate(A)]
        for c in range(n):
            piv = max(range(c, n), key=lambda r: abs(M[r][c]))
            if abs(M[piv][c]) < 1e-12:
                return [0.0, 0.0, 0.0]
            M[c], M[piv] = M[piv], M[c]
            for r in range(n):
                if r != c:
                    f = M[r][c] / M[c][c]
                    for k in range(c, n + 1):
                        M[r][k] -= f * M[c][k]
        w_active = [M[i][n] / M[i][i] for i in range(n)]
        if all(w >= 0 for w in w_active) or n == 1:
            w = [0.0, 0.0, 0.0]
            for i, idx in enumerate(active):
                w[idx] = max(0.0, w_active[i])
            return w
        active = [idx for i, idx in enumerate(active) if w_active[i] > 0] or [1]


def analyze_drill(path):
    raw = open(path, "rb").read()
    (magic, ver, rowsz, rowcnt, seqid, sx, sy, tgt, rpm,
     heat, cool, didx, _sp) = struct.unpack_from(DRILL_HDR, raw, 0)
    if magic != DRILL_MAGIC:
        return None
    rows = [struct.unpack_from(DRILL_ROW, raw, 40 + i * rowsz)
            for i in range(rowcnt)]
    # row: t_ms, phase, depth, wob, torque, current, temp, vapor, ppm
    drill_rows = [r for r in rows if r[1] == 3]

    # Competence log + interface: first sustained torque step above 1.4x
    # the shallow (top 0.5 m) baseline. Median baseline — bit-bind spikes
    # (~5% of rows) inflate a mean enough to mask light cementation.
    interface = None
    shallow = sorted(r[4] for r in drill_rows if 0.05 < r[2] < 0.5)
    if shallow:
        base = shallow[len(shallow) // 2]
        streak, streak_start = 0, None
        for r in drill_rows:
            if r[2] > 0.5 and r[4] > 1.4 * base:
                if streak == 0:
                    streak_start = r[2]
                streak += 1
                if streak >= 3:
                    interface = streak_start
                    break
            else:
                streak = 0

    # Evolved-gas heating ramp only (up to the temperature peak)
    samp = [r for r in rows if r[1] == 5]
    result = {"file": os.path.basename(path), "site": (sx, sy),
              "target_m": tgt, "interface_m": interface,
              "rows": rowcnt}
    if samp:
        peak_i = max(range(len(samp)), key=lambda i: samp[i][6])
        ramp = [(r[6], r[8]) for r in samp[:peak_i + 1]]
        temps = [t for t, _ in ramp]
        ppms = [p for _, p in ramp]
        w = solve_release_weights(temps, ppms)
        total = sum(w)
        measured = max(ppms) if ppms else 0.0
        result.update({
            "measured_ppm": measured,
            "total_ppm": total,
            "wt_pct": total / 1e4,
            "binding": {RELEASE_MODES[i][0]: (w[i] / total if total else 0.0)
                        for i in range(3)},
            "recovery": measured / total if total else 0.0,
        })
    return result


# ── Fusion: transfer function, EDI, prospect ranking ────────────────────
def fuse(stacks, drills, pairs):
    """The fusion layer: calibrate Vp→ice from the drill tie points,
    extrapolate across every surveyed site, score and rank prospects.

    The transfer function is fit to MEASURED v2 (not truth), so systematic
    inversion bias at this array geometry is absorbed by the calibration —
    exactly why the tie point is the mission-success gate.
    """
    print("\n=== Fusion (Vp → ice transfer + prospect ranking) ===")
    if not pairs:
        print("  No calibration pairs — drill at least one surveyed site "
              "to anchor the transfer function.")
        return []

    # Fit wt% = a * (v2 - b). One pair: anchor b at the driest surveyed v2.
    if len(pairs) >= 2:
        n = len(pairs)
        sx = sum(p[0] for p in pairs); sy = sum(p[1] for p in pairs)
        sxx = sum(p[0]**2 for p in pairs); sxy = sum(p[0]*p[1] for p in pairs)
        denom = n * sxx - sx * sx
        a = (n * sxy - sx * sy) / denom if denom else 0.0
        b = (sx - sy / a) / n if a else 0.0
    else:
        v2_dry = min((s["v2"] for s in stacks if "error" not in s),
                     default=pairs[0][0])
        a = pairs[0][1] / max(pairs[0][0] - v2_dry, 1.0)
        b = v2_dry
    print(f"  transfer function: wt% = {a*1000:.2f}e-3 × (v2 − {b:.0f} m/s)"
          f"   [{len(pairs)} tie point{'s' if len(pairs) != 1 else ''}]")

    def est_wt(v2):
        return max(0.0, a * (v2 - b))

    # Binding-derived extraction ease from the nearest drill (pore ice is
    # cheap to extract, hydrated minerals expensive); default mid if none.
    def extraction_ease(site):
        best, ease = 1e9, 0.6
        for d in drills:
            if "binding" not in d:
                continue
            r = math.hypot(site[0]-d["site"][0], site[1]-d["site"][1])
            if r < best:
                best = r
                bind = d["binding"]
                ease = (1.0 * bind["pore_ice"] + 0.7 * bind["adsorbed"]
                        + 0.25 * bind["hydrated"])
        return ease

    drilled_sites = {(round(d["site"][0], 1), round(d["site"][1], 1))
                     for d in drills}
    fused = []
    for s in stacks:
        if "error" in s:
            continue
        site = s["site"]
        wt, wt_lo, wt_hi = est_wt(s["v2"]), est_wt(s["v2_lo"]), est_wt(s["v2_hi"])
        confirmed = (round(site[0], 1), round(site[1], 1)) in drilled_sites
        ease = extraction_ease(site)
        # Shallower ice tops and easier binding extract cheaper; deeper
        # overburden costs. EDI in [0,1], higher = easier.
        depth_factor = max(0.2, 1.0 - 0.15 * s["h1"])
        edi = ease * depth_factor
        score = wt * s["ice_frac"] * edi
        fused.append({
            "site": site, "v2": s["v2"], "wt": wt, "wt_lo": wt_lo,
            "wt_hi": wt_hi, "ice_frac": s["ice_frac"], "edi": edi,
            "score": score,
            "conf": "CONFIRMED (drilled)" if confirmed else "inferred (seismic)",
        })

    fused.sort(key=lambda f: -f["score"])
    print("\n  Prospect queue (score = wt% × P(ice) × EDI)")
    print("  EDI = extraction-difficulty index: binding ease × depth factor,"
          " 0-1, higher = easier to extract")
    for i, f in enumerate(fused, 1):
        print(f"   {i}. site ({f['site'][0]:5.1f},{f['site'][1]:5.1f})  "
              f"est {f['wt']:.2f} wt% [{f['wt_lo']:.2f}-{f['wt_hi']:.2f}]  "
              f"P(ice)={f['ice_frac']:.2f}  EDI={f['edi']:.2f}  "
              f"score={f['score']:.2f}  {f['conf']}")
    return fused


# ── Report ──────────────────────────────────────────────────────────────
def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    data_dir = args[0] if args else \
        ("wok_archive" if os.path.isdir("wok_archive") else "/tmp/wok_data")
    csv_out = None
    if "--csv" in sys.argv:
        csv_out = sys.argv[sys.argv.index("--csv") + 1]

    coups = load_coupling(data_dir)

    def coup_for(path):
        raw = open(path, "rb").read()
        h = struct.unpack_from(SEIS_HDR, raw, 0)
        return coups.get((round(h[11], 1), round(h[12], 1)))

    stacks = [invert_stack(p, coup_for(p))
              for p in sorted(glob.glob(f"{data_dir}/stack_*.wok"))]
    drills = [analyze_drill(p) for p in sorted(glob.glob(f"{data_dir}/drill_*.wok"))]
    stacks = [s for s in stacks if s]
    drills = [d for d in drills if d]

    print(f"\n=== WOK site survey — {data_dir} ===\n")
    if coups:
        print("Coupling records (test-shot verification, CP-4):")
        for site, c in sorted(coups.items()):
            sc = "/".join(f"{v:.2f}" if v >= 0 else "-" for v in c["scores"])
            sync = ""
            if c.get("sync_residual_ms") is not None:
                sync = ("  sync " +
                        "/".join(f"{r:+.2f}" for r in c["sync_residual_ms"]) +
                        (f"  SYNC-SUSPECT 0x{c['sync_suspect']:X}"
                         if c["sync_suspect"] else ""))
            print(f"  {c['file']} site ({site[0]:5.1f},{site[1]:5.1f}): "
                  f"scores {sc}  verified {c['verified']}/4  "
                  f"attempts {'/'.join(str(a) for a in c['attempts'])}  "
                  f"test shots {c['test_shots']}"
                  f"{'  PLATE-DEGRADED' if c['plate_degraded'] else ''}{sync}")
        print()
    print("Seismic (two-layer refraction inversion):")
    for s in stacks:
        if "error" in s:
            print(f"  {s['file']} site {s['site']}: {s['error']}")
            continue
        wtag = ""
        if any(w < 1.0 for w in s.get("weights", [])):
            wtag = "  wts=" + "/".join(f"{w:.2f}" for w in s["weights"])
        if s.get("plate_degraded"):
            wtag += f"  PLATE-DEGRADED ({s['plate_reseats']} reseats)"
        print(f"  {s['file']} site ({s['site'][0]:5.1f},{s['site'][1]:5.1f}) "
              f"[{s.get('source', 'impulse'):7s}]: "
              f"v1={s['v1']:4.0f}  v2={s['v2']:4.0f} [{s['v2_lo']:.0f}-{s['v2_hi']:.0f}] m/s  "
              f"h1={s['h1']:.2f} [{s['h1_lo']:.2f}-{s['h1_hi']:.2f}] m  "
              f"ice-frac={s['ice_frac']:.2f}  rms={s['rms_ms']:.2f} ms{wtag}")

    print("\nDrill (competence + evolved-gas decomposition):")
    for d in drills:
        line = (f"  {d['file']} site ({d['site'][0]:5.1f},{d['site'][1]:5.1f}): "
                f"interface={d['interface_m']:.2f} m  " if d.get("interface_m")
                else f"  {d['file']} site {d['site']}: interface=?  ")
        if "total_ppm" in d:
            b = d["binding"]
            line += (f"total={d['total_ppm']:.0f} ppm ({d['wt_pct']:.2f} wt%)  "
                     f"recovered={d['recovery']*100:.0f}%  "
                     f"[ads {b['adsorbed']*100:.0f}% / pore {b['pore_ice']*100:.0f}%"
                     f" / hyd {b['hydrated']*100:.0f}%]")
        print(line)

    print("\nTie points (co-located seismic + drill):")
    pairs = []          # (v2_measured, wt_pct, drill_result)
    for d in drills:
        near = [s for s in stacks if "error" not in s and
                abs(s["site"][0] - d["site"][0]) < 0.5 and
                abs(s["site"][1] - d["site"][1]) < 0.5]
        for s in near:
            iface = d.get("interface_m")
            di = f"{iface:.2f}" if iface else "?"
            print(f"  site ({d['site'][0]:.1f},{d['site'][1]:.1f}): "
                  f"seismic h1={s['h1']:.2f} m vs drill interface={di} m | "
                  f"calibration pair: v2={s['v2']:.0f} m/s ↔ "
                  f"{d.get('wt_pct', 0):.2f} wt% water-equiv")
            if "wt_pct" in d:
                pairs.append((s["v2"], d["wt_pct"], d))

    fused = fuse(stacks, drills, pairs)

    if csv_out:
        with open(csv_out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["type", "file", "x", "y", "v1", "v2", "v2_lo", "v2_hi",
                        "h1", "h1_lo", "h1_hi", "ice_frac", "rms_ms",
                        "interface_m", "total_ppm", "wt_pct",
                        "f_adsorbed", "f_pore", "f_hydrated", "recovery"])
            for s in stacks:
                if "error" in s:
                    continue
                w.writerow(["seis", s["file"], s["site"][0], s["site"][1],
                            f"{s['v1']:.0f}", f"{s['v2']:.0f}",
                            f"{s['v2_lo']:.0f}", f"{s['v2_hi']:.0f}",
                            f"{s['h1']:.2f}", f"{s['h1_lo']:.2f}",
                            f"{s['h1_hi']:.2f}", f"{s['ice_frac']:.2f}",
                            f"{s['rms_ms']:.2f}", "", "", "", "", "", "", ""])
            for d in drills:
                b = d.get("binding", {})
                w.writerow(["drill", d["file"], d["site"][0], d["site"][1],
                            "", "", "", "", "", "", "", "", "",
                            f"{d.get('interface_m') or 0:.2f}",
                            f"{d.get('total_ppm', 0):.0f}",
                            f"{d.get('wt_pct', 0):.3f}",
                            f"{b.get('adsorbed', 0):.2f}",
                            f"{b.get('pore_ice', 0):.2f}",
                            f"{b.get('hydrated', 0):.2f}",
                            f"{d.get('recovery', 0):.2f}"])
            w.writerow([])
            w.writerow(["rank", "x", "y", "est_wt_pct", "wt_lo", "wt_hi",
                        "p_ice", "edi", "score", "confidence"])
            for i, fz in enumerate(fused, 1):
                w.writerow([i, fz["site"][0], fz["site"][1],
                            f"{fz['wt']:.2f}", f"{fz['wt_lo']:.2f}",
                            f"{fz['wt_hi']:.2f}", f"{fz['ice_frac']:.2f}",
                            f"{fz['edi']:.2f}", f"{fz['score']:.2f}",
                            fz["conf"]])
        print(f"\nWrote {csv_out}")


if __name__ == "__main__":
    main()
