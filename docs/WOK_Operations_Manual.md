# WOK Operations Manual

**Document:** WOK-OM-001  
**Version:** 1.1 (FSW 5.0)  
**Date:** October 2026  
**Audience:** mission operators — the person planning waypoint tables,
watching a run, responding at holds, recovering from SAFE, and turning
downlinked products into decisions.

This manual is procedures and interpretation. It does not repeat the
reference material — for those, see:

| Question | Document |
|----------|----------|
| How do I build/install/launch the stack? | [Build & Install Guide](WOK_Build_Install_Guide.md) |
| What does every command/TLM field mean? | [Ground Console Guide](WOK_Ground_Console_Guide.md) |
| How does the FSW work inside? | [Architecture & Design](WOK_Architecture_Design.md) |
| What physics does the sim implement? | [Sim Model Spec](WOK_Sim_Model_Spec.md) |

---

## 1. The mission in one page

WOK surveys a line/grid of sites for near-surface water ice. At each
site it fires a dual-mode EM seismic source into a geophone array,
picks first breaks onboard, inverts them for refractor velocity, and
decides (or you decide) where to drill. **The single product the
mission cannot end without is one drill-to-seismic tie point** — the
sequencer enforces this: a mission with zero drills returns to its
best-scoring site and drills it before DONE.

The operational loop, annotated with what you own at each step:

```
IDLE → MOVE → [HOLD]* → DEPLOY → [VERIFY]** → [THUMP → RECORD] ×3
     → EVALUATE → [DRILL] → STOW → MOVE → … → DONE
                                    ↓ (any critical fault)
                                   SAFE  ← you recover from here
```

- `*` HOLD only at waypoints you flagged `recal=1` — your
  ground-in-the-loop navigation correction pass (§5).
- `**` VERIFY only in per-pod placement mode — autonomous coupling
  verification; you read its results, you rarely intervene (§6).

Everything the rover decides, it also telemeters and archives. Your
leverage points are: the waypoint table before START, the HOLD windows
during the run, and RESUME/contingency commands after a fault.

## 2. Ground segment bring-up

1. `tools/wok_sim.sh up` — starts core-cpu1, the WebSocket bridge, and
   the dashboard HTTP server, enables TO_LAB, prints the dashboard URL.
   Heed its warning about `fs.mqueue.msg_max` (must be ≥ 64 or the
   downlink starves — details in the Build & Install Guide).
2. Console: `python3 ground/wok_console.py`. First commands:
   `allnoop` (command path), `test.connectivity`.
3. Sim runs on real wall-clock: **the battery drains in real time**
   (~0.001 V/s from 25.2 V — the 23.5 V drill inhibit arrives after
   roughly 25–30 min of run time, and the 22.0 V SAFE threshold after
   roughly 45–55 min depending on load). Cycle `wok_sim.sh down` / `up`
   between missions; never reuse a long-idle core for a fresh run.
4. Sim environment toggles (set before `wok_sim.sh up`; all default off
   and preserve regression behavior exactly):
   - `WOK_SIM_COUPLING=1` — ground-coupling truth model (bad seats
     exist, reseats matter).
   - `WOK_SIM_SYNC_ERR_MS=<ms>` — per-pod clock bias for wireless-pod
     sync studies.
   - `WOK_SIM_TURN_BIAS_DEG=<deg>` — unsensed drivetrain turn bias
     (dead reckoning diverges from truth; the recalibration exercise).

## 3. Mission planning

### 3.1 Waypoint table

Each waypoint is a relative step in the start frame plus per-site
options. Upload before START:

```
seq.loadwp <idx> <fwd_m> <lat_m> [drill] [srcmode] [energy_J] [recal]
seq.wpcount <n>
```

| Field | Meaning | Planning guidance |
|-------|---------|-------------------|
| fwd/lat | start-frame deltas from the previous site | Pure-forward legs accrue no heading drift (no turns); lateral/backward legs cost two turns each — schedule recal holds after several turning legs |
| drill | 1 = drill here if EVALUATE flags ice | Battery: drilling requires > 23.5 V at decision time |
| srcmode | 0 impulse (default) / 1 sweep | Sweep for long-offset/force-limited work; costs a longer record window + onboard correlation time |
| energy_J | impulse bank energy (0 = 50 J default) | Bank charges ~1 s per 50 J; ceiling 200 J |
| recal | 1 = HOLD after MOVE for a ground pass | Place where accumulated drift matters and a ground contact is expected; the hold self-releases after 120 s |

### 3.2 Mission mode and placement mode

- `seq.mode 0` (drill-inline, default): drill at flagged sites as
  found. `seq.mode 1` (survey-first): score every site, then return to
  the argmax and drill it. Either way the tie-point gate guarantees ≥1
  drill.
- `seq.placement 1` enables per-pod placement: DEPLOY
  becomes the placement sub-sequence, VERIFY runs the coupling test
  shot, STOW retrieves pod-by-pod. Default 0 keeps the legacy fixed
  array.

### 3.3 Time and battery budget (sim rates)

| Item | Cost |
|------|------|
| Drive | 0.5 m/s (+ ~3 s per 90° turn) |
| Basic site (3 stacked shots) | ~30–45 s |
| Drill site | + ~4–5 min (2.5 m drill + ~55 s heat ramp, 10 s hold, ~90 s cool) |
| Placement mode, per site | + ~2–3 min (out-and-back legs, 8 pod ops at 3–4 s each, 1–2 test shots, reseat drives if pods fail) |
| Recal hold | up to 120 s if unattended |
| Battery | 25.2 V at boot; SAFE below 22.0 V; drill inhibited below 23.5 V; bank charge inhibited below 23.0 V (reads as a source arm timeout, not a fault) |

A default 7-waypoint mission with two drills runs ≈ 12 min; add ~50%
for placement mode. Plan to be DONE with battery margin above 23.5 V
if late waypoints carry drill flags.

## 4. Executing and monitoring a run

1. Load the table (`seq.loadwp` ×N, `seq.wpcount`), set modes, `seq.start`.
2. Watch the dashboard (state, NAV pose/minimap, per-app panels) or
   grep the core log. The decisive event stream is compact — these are
   the lines that matter:

| Event (grep) | Meaning |
|--------------|---------|
| `Site N geometry` | Pose stamped for the site — this is what all products inherit |
| `Holding at WP` | Recal hold open — your window (§5) |
| `Pod scores` / `Array verified` | Coupling verification result (§6) |
| `Plate coupling LOW` / `DEGRADED` | Plate metric fail / degraded acceptance (§6) |
| `Sync residuals … suspect` | Pod timing off the array family (§6) |
| `Ice signature … DRILLING` / `not drill-flagged` / `skip drill` | The drill decision |
| `Tie-point gate` / `Targeting WP` | End-of-mission return-drill logic |
| `FAULT` / `→ SAFE` | You're on (§7) |

3. `tools/wok_mission_tools.py events <log>` prints the normalized
   decisive-event stream from a core log after (or during) a run.

## 5. Procedure: ground recalibration pass (HOLD)

Dead reckoning cannot see its own drift — every step's turn-back
restores the *believed* heading. You measure the error externally and
correct it. At a recal-flagged waypoint the sequencer parks **before**
the site-pose snapshot, so your correction lands in every product from
that site onward.

On `Holding at WP n for ground recalibration (RELEASE to proceed; auto-proceed in 120s)`:

1. Measure the rover's true pose and heading against an external
   reference (in sim: computed from the known turn bias; in flight:
   imagery/landmarks/sun sensor).
2. `nav.realign <heading_error_deg>` — sign: + means the rover points
   left of its belief; NAV turns −error and re-declares the start-frame
   heading. Sub-1° errors are accepted without a physical turn.
3. `nav.setpose <x> <y>` — overwrite the accumulated position error
   with truth. **Send both**: realign stops further growth, setpose
   clears what already accrued.
4. `seq.release` — the mission proceeds to DEPLOY.

Expectations: `realign_count` increments in NAV TLM;
`heading_drift` reads as drift-since-this-realign from here on. The
corrective turn rides the same drivetrain as every turn — a systematic
per-turn error leaves exactly a one-turn residual, so don't chase
perfection with repeated realigns (they converge to that floor, no
further). If you miss the window, the hold times out with an ERROR
event and the mission continues uncorrected — plan another flagged
waypoint downstream rather than aborting.

## 6. Reading the coupling system (placement missions)

The coupling chain is autonomous; your job is interpretation:

- **Seating force gate (placement)**: `Pod n seat force <26.0N —
  reseat k/2` then either `seated OK` or `MARGINAL`. MARGINAL is a
  flag, not a failure — the test shot arbitrates.
- **VERIFY test shot**: `Pod scores a/b/c/d ok=0xM` — scores are
  amplitude ratios to the array median (1.0 = family; the plate factor
  cancels, so a low score is pod-side). Failing pods get one targeted
  drive-back reseat each within a per-site budget, then:
  `Array verified V/4` — V<4 flags the site degraded; V<3 raises a
  CRITICAL partial-array event but the site still proceeds (data is
  weighted on the ground, never discarded silently).
- **Plate metric (every impulse shot)**: `Plate coupling LOW — ratio R
  vs baseline B` triggers plate reseats; if the near pod's score is
  bad the FSW knows it's pod-side and accepts degraded without burning
  plate reseats. `plate_degraded` in TLM / stack headers marks it.
- **Sync residuals**: `suspect 0xM` means a pod's test-shot pick sits
  >0.5 ms off the array's moveout fit — a clock (or bad-pick) flag.
  Nothing onboard acts on it (no mechanism fixes a clock): downlink
  the coupling record, deweight, investigate. Note the check false-
  alarms on low-coupling pods; read it next to the scores.

Everything above is archived per site in `coup_NNNN.wok` (scores, seat
attempts, marginal mask, plate state, sync residuals) and consumed by
`ground/wok_refraction.py`, which deweights low-coupling channels
automatically.

Manual/contingency forms exist for every mechanism primitive:
`geo.place`, `geo.reseat`, `geo.retrieve`, `seis.plate_reseat`.

## 7. Contingencies

### 7.1 SAFE entry

Any critical fault stops NAV, stows GEO (legacy arm), stops SEIS,
e-stops DRILL, and parks in SAFE awaiting ground. Diagnose from the
fault mask in HK/status TLM (the console decodes names): battery low
(< 22.0 V), tilt (> 30°), temp, NAV stuck/overcurrent, GEO jam/timeout,
DRILL faults. Then:

1. Clear the cause (in sim, usually battery — restart the stack; a
   fresh core resets the battery model).
2. `seq.resume` — re-enters MOVE at the current waypoint with fresh
   child status (stale fault bits are dropped deliberately).

### 7.2 SAFE mid-placement (pods on the ground)

The pod inventory is honest across an abort: check GEO status TLM
(`pod_state` per pod — PLACED_OK/MARGINAL means it is physically on
the ground). Before resuming or restarting a placement mission:

1. `tlm geo` → note which pods read placed.
2. Drive to each placed pod's position if needed (`nav.goto`), then
   `geo.retrieve <pod>`; confirm `recaptured and stowed` / CARRIED.
3. Then `seq.resume` (or restart the mission).

An interrupted placement resolves conservatively: a fresh place
retracts with the pod (CARRIED); an interrupted reseat or retrieval
leaves the pod on the ground (MARGINAL) — trust the TLM, not memory.

### 7.3 Source won't arm

A sagging bus (< 23.0 V) silently stalls bank charging; the sequencer's
30 s source timeout fires and the site records without a shot (or a
test shot is skipped with `skipping verification`). This is a battery
condition, not a source fault — check bus voltage before suspecting
hardware.

### 7.4 Downlink stalls

Chunk fetches that stall around 10 chunks are the kernel mqueue clamp
(`fs.mqueue.msg_max` < 64), not a link problem. Fix the sysctl and
re-fetch; transfers are stop-and-wait and resume cleanly.

## 8. Data products and ground processing

| Product | When | Contents |
|---------|------|----------|
| `seis_NNNN.wok` | every shot | raw record + full shot context header |
| `corr_NNNN.wok` | sweep shots | correlated equivalent-impulse trace |
| `stack_NNNN.wok` | per site | aligned stack, picks, QC, plate/degraded flags |
| `coup_NNNN.wok` | per site (placement) | pod scores, seat attempts, plate metric, sync residuals |
| `drill_NNNN.wok` | per drill | drilling-response log + evolved-gas curve |

Procedure after (or during) a mission:

1. Fetch what you need: `seis.fetch <name>` (reassembles into
   `wok_archive/`). Products are also readable directly from
   `/tmp/wok_data` in sim.
2. Sanity: `tools/wok_mission_tools.py validate <dir>` (picks vs the
   sim's closed-form truth — sim only).
3. Science: `python3 ground/wok_refraction.py <dir>` — per-site Vp
   profiles with uncertainty, coupling summary + channel weights,
   competence logs, evolved-gas binding decomposition, tie points, the
   fitted Vp→ice transfer function, and the ranked prospect queue.
4. Read the prospect queue with the confidence labels: **CONFIRMED**
   (drilled) vs **inferred** (seismic-only, extrapolated through the
   transfer function). Degraded/sync-flagged sites carry wider bounds —
   that is the flags doing their job, not a defect.

Queue columns: `est wt%` is the transfer-function estimate with its
3σ range; `P(ice)` the onboard posterior that the refractor is
ice-cemented; `EDI` the extraction-difficulty index (binding ease from
the nearest drill's evolved-gas fit × a depth factor; higher is easier);
`score = wt% × P(ice) × EDI`.

## 9. Operator quick reference

**Thresholds that drive behavior:**

| Value | Meaning |
|------:|---------|
| 22.0 V | SAFE (battery low) |
| 23.0 V | bank charge inhibit (arm timeout symptom) |
| 23.5 V | drill inhibit |
| 30° | tilt fault |
| 26 N | pod seating-force gate (2 reseats, then MARGINAL) |
| 0.70 | pod coupling score threshold (test shot) |
| 0.70 × baseline | plate transfer ratio threshold (2 reseats, then degraded) |
| 3 / 4 | partial-array minimum (below → CRITICAL, proceed flagged) |
| 0.5 ms | sync-suspect residual threshold |
| ~1° | realign / step-planning turn epsilon |
| 30 s | source (THUMP/test-shot) timeout |
| 120 s | recal hold auto-proceed |

**Command cheat sheet** (full syntax in the Console Guide):
mission — `seq.loadwp` `seq.wpcount` `seq.mode` `seq.placement`
`seq.start` `seq.abort` `seq.resume` `seq.release`;
recalibration — `nav.realign` `nav.setpose`;
contingency — `nav.goto` `nav.stop` `geo.retrieve` `geo.place`
`geo.reseat` `seis.plate_reseat` `drill.abort`;
data — `seis.fetch`, `wok_mission_tools.py validate|events`;
health — `allnoop` `test.all`.
