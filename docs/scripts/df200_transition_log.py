#!/usr/bin/env python3
"""DF-200 / QuadPlane transition log triage from ArduPilot .BIN.

Reusable analyzer for the DF-200 SITL campaign. Prints:
  1) key Q_*/AIRSPEED parms
  2) MSG / MODE timeline
  3) milestone checklist (MSG only — not the failure clock)
  4) ONSET detectors (Alt drop / surfaces / attitude) — PRIMARY failure clock
  5) dense ATT/QTUN/TILT/AETR table
  6) detail dump centered on FIRST onset (not late |att|>30)

Important (lessons from 096 mis-read):
  - Do NOT treat MSG "airspeed reached" / TIMER as the failure time.
    Tilt can still be rising in AIRSPEED_WAIT while Alt/surfaces already bad.
  - Do NOT use |Roll|/|Pitch|>30 as onset — that is the late crash, seconds
    after Elev/Ail and altitude already went wrong.
  - Primary clock: first sustained Alt drop + first large Ail/Elev, with Tilt
    still climbing. Attitude blowup is a consequence.

Usage:
  python3 docs/scripts/df200_transition_log.py logs/00000096.BIN
  python3 docs/scripts/df200_transition_log.py logs/00000096.BIN --csv docs/scripts/out/096.csv
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

from pymavlink import mavutil

KEY_PARMS = [
    "ARMING_CHECK",
    "Q_TILT_MAX", "Q_TILT_RATE_UP", "Q_TRANSITION_MS", "Q_OPTIONS", "Q_TF_YAW_ERR",
    "Q_TILT_CRD_EN", "Q_TILT_CRD_MRG",
    "Q_TILT_CRD_T1", "Q_TILT_CRD_V1", "Q_TILT_CRD_T2", "Q_TILT_CRD_V2",
    "Q_TILT_CRD_T3", "Q_TILT_CRD_V3", "Q_TILT_CRD_T4", "Q_TILT_CRD_V4",
    "Q_A_RAT_RLL_P", "Q_A_RAT_RLL_I", "Q_A_RAT_RLL_D",
    "Q_A_RAT_PIT_P", "Q_A_RAT_PIT_I", "Q_A_RAT_PIT_D",
    "Q_A_RAT_YAW_P", "Q_A_RAT_YAW_I", "Q_A_RAT_YAW_D", "Q_A_ANG_YAW_P",
    "Q_A_RATE_Y_MAX", "Q_A_ACCEL_Y_MAX", "Q_M_YAW_HEADROOM",
    "Q_M_THST_HOVER", "Q_ANGLE_MAX", "Q_A_INPUT_TC",
    "Q_WP_SPEED_UP", "Q_WP_ACCEL_Z",
    "AIRSPEED_MIN", "AIRSPEED_CRUISE", "Q_ASSIST_SPEED", "Q_TILT_MASK",
]

EVENT_HINTS = (
    "align", "Transition", "airspeed", "FW done", "Hit ground",
    "lane switch", "yaw reset", "Mission:", "VTOL",
)


def nearest(arr, t, tol=0.3):
    if not arr:
        return None
    best = min(arr, key=lambda a: abs(a[0] - t))
    return best if abs(best[0] - t) <= tol else None


def load(path: str):
    m = mavutil.mavlink_connection(path)
    ATT, RATE, QTUN, TILT, MOTT, ARSP, AETR, CTUN, TECS = [], [], [], [], [], [], [], [], []
    MSG, MODE, PARM = [], [], {}
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        try:
            ts = msg.TimeUS / 1e6
        except Exception:
            continue
        if t == "ATT":
            ATT.append((ts, msg.Roll, msg.Pitch, msg.Yaw,
                        msg.DesRoll, msg.DesPitch, msg.DesYaw))
        elif t == "RATE":
            RATE.append((ts, msg.R, msg.RDes, msg.ROut,
                         msg.P, msg.PDes, msg.POut,
                         msg.Y, msg.YDes, msg.YOut))
        elif t == "QTUN":
            d = msg.to_dict(); d.pop("mavpackettype", None)
            QTUN.append((ts, d))
        elif t == "TILT":
            d = msg.to_dict(); d.pop("mavpackettype", None)
            TILT.append((ts, d))
        elif t == "MOTT":
            MOTT.append((ts, [msg.Mot1, msg.Mot2, msg.Mot3, msg.Mot4,
                              msg.Mot5, msg.Mot6, msg.Mot7, msg.Mot8]))
        elif t == "ARSP":
            ARSP.append((ts, float(msg.Airspeed)))
        elif t == "AETR":
            d = msg.to_dict(); d.pop("mavpackettype", None)
            AETR.append((ts, d))
        elif t == "CTUN":
            d = msg.to_dict(); d.pop("mavpackettype", None)
            CTUN.append((ts, d))
        elif t == "TECS":
            d = msg.to_dict(); d.pop("mavpackettype", None)
            TECS.append((ts, d))
        elif t == "MSG":
            MSG.append((ts, msg.Message))
        elif t == "MODE":
            MODE.append((ts, msg.Mode))
        elif t == "PARM":
            PARM[msg.Name] = msg.Value
    return dict(ATT=ATT, RATE=RATE, QTUN=QTUN, TILT=TILT, MOTT=MOTT,
                ARSP=ARSP, AETR=AETR, CTUN=CTUN, TECS=TECS, MSG=MSG, MODE=MODE, PARM=PARM)


def find_window(data, t0=None, t1=None):
    msgs = data["MSG"]
    if not msgs:
        att = data["ATT"]
        return (att[0][0], att[-1][0]) if att else (0.0, 0.0)

    start = None
    end = None
    for ts, text in msgs:
        low = text.lower()
        if start is None and ("transition started" in low or "aligning yaw" in low):
            start = ts
        if "hit ground" in low or "crash" in low:
            end = ts
    if start is None:
        start = msgs[0][0]
    if end is None:
        end = msgs[-1][0]
        if data["ATT"]:
            end = max(end, data["ATT"][-1][0])
    if t0 is not None:
        start = t0
    else:
        start = max(0.0, start - 5.0)
    if t1 is not None:
        end = t1
    return start, end


def phase_flags(msgs, t):
    """MSG phase label only — NOT the failure clock."""
    label = "pre"
    for ts, text in msgs:
        if ts > t:
            break
        low = text.lower()
        if "transition started" in low:
            label = "AIRSPEED_WAIT"
        elif "airspeed reached" in low:
            label = "TIMER"
        elif "fw done" in low:
            label = "FW"
        elif "aligning yaw" in low:
            label = "YAW_ALIGN"
        elif "yaw aligned" in low:
            label = "YAW_OK"
        elif "hit ground" in low:
            label = "CRASH"
    return label


def aspeed_at(data, t):
    ct = nearest(data["CTUN"], t)
    if ct and ct[1].get("As") is not None:
        return ct[1]["As"]
    ar = nearest(data["ARSP"], t)
    return ar[1] if ar else float("nan")


def tilt_at(data, t):
    ti = nearest(data["TILT"], t)
    return ti[1].get("Tilt") if ti else float("nan")


def tecs_at(data, t, key):
    """Fetch a TECS log field by nearest timestamp ('hdem','spdem','th',...)."""
    te = nearest(data.get("TECS", []), t)
    return te[1].get(key) if te else float("nan")


def find_trans_start(msgs):
    for ts, text in msgs:
        if "transition started" in text.lower():
            return ts
    for ts, text in msgs:
        if "aligning yaw" in text.lower() or "yaw aligned" in text.lower():
            return ts
    return None


def detect_onsets(data):
    """Primary failure clock: surfaces / altitude BEFORE large attitude.

    Returns ordered list of (name, t, detail_dict) and the recommended focus_t.
    """
    t_trans = find_trans_start(data["MSG"])
    if t_trans is None:
        t_trans = data["ATT"][0][0] if data["ATT"] else 0.0

    onsets = []

    # Peak altitude after transition start, then first clear drop
    peak_alt = None
    peak_t = None
    for ts, d in data["QTUN"]:
        if ts < t_trans:
            continue
        alt = d.get("Alt")
        if alt is None:
            continue
        if peak_alt is None or alt > peak_alt:
            peak_alt = alt
            peak_t = ts
    if peak_alt is not None:
        for ts, d in data["QTUN"]:
            if ts < peak_t:
                continue
            alt = d.get("Alt")
            crt = d.get("CRt", 0.0) or 0.0
            if alt is not None and alt < peak_alt - 0.25 and crt < -0.05:
                onsets.append(("alt_drop", ts, {
                    "alt": alt, "peak_alt": peak_alt, "peak_t": peak_t,
                    "CRt": crt, "Tilt": tilt_at(data, ts), "AS": aspeed_at(data, ts),
                    "phase": phase_flags(data["MSG"], ts),
                }))
                break

    # Surface growth (AETR). Elev often leads; Ail follows.
    seen = set()
    thresholds = [
        ("elev_200", "Elev", 200),
        ("elev_500", "Elev", 500),
        ("ail_100", "Ail", 100),
        ("ail_300", "Ail", 300),
        ("rudd_100", "Rudd", 100),
    ]
    for ts, d in data["AETR"]:
        if ts < t_trans:
            continue
        for name, key, thr in thresholds:
            if name in seen:
                continue
            val = d.get(key)
            if val is not None and abs(val) >= thr:
                seen.add(name)
                onsets.append((name, ts, {
                    key: val, "Tilt": tilt_at(data, ts), "AS": aspeed_at(data, ts),
                    "phase": phase_flags(data["MSG"], ts),
                }))

    # Attitude — recorded but ranked AFTER surfaces/alt for focus
    att_seen = set()
    for ts, roll, pit, yaw, droll, dpit, dyaw in data["ATT"]:
        if ts < t_trans:
            continue
        checks = [
            ("pit_5", abs(pit) >= 5, pit, dpit),
            ("roll_5", abs(roll) >= 5, roll, droll),
            ("pit_15", abs(pit) >= 15, pit, dpit),
            ("roll_15", abs(roll) >= 15, roll, droll),
            ("att_30", abs(roll) >= 30 or abs(pit) >= 30, roll if abs(roll) >= abs(pit) else pit,
             droll if abs(roll) >= abs(pit) else dpit),
        ]
        for name, ok, val, des in checks:
            if ok and name not in att_seen:
                att_seen.add(name)
                onsets.append((name, ts, {
                    "att": val, "des": des,
                    "Tilt": tilt_at(data, ts), "AS": aspeed_at(data, ts),
                    "phase": phase_flags(data["MSG"], ts),
                }))

    # MotSpan high after surfaces start (ignore immediate post-align noise)
    for ts, mots in data["MOTT"]:
        if ts < t_trans + 2.0:
            continue
        span = max(mots) - min(mots)
        if span >= 0.85:
            onsets.append(("motspan_085", ts, {
                "span": span, "Tilt": tilt_at(data, ts), "AS": aspeed_at(data, ts),
                "phase": phase_flags(data["MSG"], ts),
            }))
            break

    onsets.sort(key=lambda x: x[1])

    # Focus = earliest of alt_drop / elev_200 / ail_100 (not att_30)
    focus_names = {"alt_drop", "elev_200", "ail_100", "elev_500", "ail_300"}
    focus_t = None
    for name, ts, _ in onsets:
        if name in focus_names:
            focus_t = ts
            break
    if focus_t is None:
        for name, ts, _ in onsets:
            if name.startswith("pit_") or name.startswith("roll_"):
                focus_t = ts
                break
    return onsets, focus_t, t_trans


def diagnose(data, onsets, focus_t):
    print("\n=== ONSET (primary failure clock) ===")
    print("  Rank by TIME. Surfaces/Alt lead; |att|>30 is late consequence.")
    if not onsets:
        print("  (no onset markers found)")
    for name, ts, det in onsets:
        tilt = det.get("Tilt", float("nan"))
        aspd = det.get("AS", float("nan"))
        ph = det.get("phase", "?")
        extra = {k: v for k, v in det.items() if k not in ("Tilt", "AS", "phase")}
        print(f"  {ts:8.2f}  [{ph:12s}] tilt={tilt:5.1f} AS={aspd:5.1f}  {name:12s}  {extra}")

    print("\n=== Quick diagnosis ===")
    # MSG milestones are context only
    joined = " | ".join(t for _, t in data["MSG"])
    has_align = "yaw aligned" in joined.lower()
    has_start = "Transition started" in joined
    has_aspd = "airspeed reached" in joined.lower()
    has_fw = "FW done" in joined

    if not has_align:
        print("  - never yaw-aligned → yaw authority / bit23 / Q_TF_YAW_ERR")
        return
    if not has_start:
        print("  - aligned but no Transition started → mission path")
        return

    # Prefer onset-based diagnosis
    elev_t = next((ts for n, ts, _ in onsets if n == "elev_200"), None)
    ail_t = next((ts for n, ts, _ in onsets if n == "ail_100"), None)
    alt_t = next((ts for n, ts, _ in onsets if n == "alt_drop"), None)
    att_t = next((ts for n, ts, _ in onsets if n in ("pit_5", "roll_5", "att_30")), None)
    aspd_msg_t = next((ts for ts, t in data["MSG"] if "airspeed reached" in t.lower()), None)

    if focus_t is not None:
        ph = phase_flags(data["MSG"], focus_t)
        tilt = tilt_at(data, focus_t)
        print(f"  - FIRST useful onset @ {focus_t:.2f}s  phase={ph}  tilt≈{tilt:.1f}°"
              f"  (tilt still rising? check table)")
        if elev_t and (ail_t is None or elev_t <= ail_t + 0.05):
            print(f"  - Elev grew first (@{elev_t:.2f}) → pitch/surface coupling before roll blowup")
        if ail_t:
            print(f"  - Ail grew (@{ail_t:.2f}) while still WAIT → FW surfaces active before TIMER")
        if alt_t:
            print(f"  - Alt drop (@{alt_t:.2f}) before/with attitude → energy/lift already wrong")
        if att_t and focus_t is not None and att_t > focus_t + 0.3:
            print(f"  - Large attitude (@{att_t:.2f}) is LATE vs surfaces/alt — do not call this the root time")
        if aspd_msg_t and focus_t < aspd_msg_t:
            print(f"  - Onset BEFORE 'airspeed reached' (@{aspd_msg_t:.2f}) → NOT a TIMER/FW-done bug")
            print("  - Still AIRSPEED_WAIT: tilting up, surfaces already fighting attitude/altitude")
        elif aspd_msg_t and focus_t >= aspd_msg_t:
            print(f"  - Onset at/after airspeed reached (@{aspd_msg_t:.2f}) → TIMER-entry band")
        if not has_fw:
            print("  - never reached FW done")
        else:
            print("  - reached FW done → also inspect post-FW handoff")
    else:
        if not has_aspd:
            print("  - no clear onset markers; died before airspeed reached (WAIT)")
        elif not has_fw:
            print("  - no clear onset markers; died after airspeed / before FW done")
        else:
            print("  - reached FW done; inspect post-FW")


def summarize(data, t0, t1, step=0.5, csv_path=None):
    PARM = data["PARM"]
    print("=== Key PARM ===")
    for k in KEY_PARMS:
        if k in PARM:
            print(f"  {k:22s} = {PARM[k]}")

    print("\n=== MSG (filtered) ===")
    for ts, text in data["MSG"]:
        if any(h.lower() in text.lower() for h in EVENT_HINTS):
            print(f"  {ts:8.2f}  {text}")

    print("\n=== MODE ===")
    for ts, mode in data["MODE"]:
        print(f"  {ts:8.2f}  mode={mode}")

    joined = " | ".join(t for _, t in data["MSG"])
    checks = [
        ("yaw aligned", "yaw aligned" in joined.lower()),
        ("Transition started", "Transition started" in joined),
        ("airspeed reached", "airspeed reached" in joined.lower()),
        ("FW done", "FW done" in joined),
        ("Hit ground", "Hit ground" in joined),
    ]
    print("\n=== Milestone checklist (MSG only — not failure clock) ===")
    for name, ok in checks:
        print(f"  [{'x' if ok else ' '}] {name}")

    onsets, focus_t, t_trans = detect_onsets(data)
    diagnose(data, onsets, focus_t)

    print(f"\n=== Dense timeline {t0:.1f}–{t1:.1f}s (step={step}) ===")
    hdr = (f"{'t':>6} {'ph':>12} {'Roll':>6}{'DRl':>6} {'Pit':>6}{'DPi':>6} "
           f"{'ThO':>5} {'Alt':>6}{'CRt':>6} {'AS':>5} {'Tilt':>5} "
           f"{'hdem':>6}{'spdem':>6}{'TECSth':>6} "
           f"{'Ail':>7}{'Elev':>7} {'Span':>5} {'TMix':>5}")
    print(hdr)

    rows = []
    t = t0
    while t <= t1 + 1e-9:
        a = nearest(data["ATT"], t)
        r = nearest(data["RATE"], t)
        q = nearest(data["QTUN"], t)
        ti = nearest(data["TILT"], t)
        mo = nearest(data["MOTT"], t)
        ae = nearest(data["AETR"], t)
        if a is None:
            t += step
            continue
        tho = q[1].get("ThO") if q else float("nan")
        alt = q[1].get("Alt") if q else float("nan")
        crt = q[1].get("CRt") if q else float("nan")
        tmix = q[1].get("TMix") if q else float("nan")
        tilt = ti[1].get("Tilt") if ti else float("nan")
        aspd = aspeed_at(data, t)
        ail = ae[1].get("Ail") if ae else float("nan")
        elev = ae[1].get("Elev") if ae else float("nan")
        hdem = tecs_at(data, t, "hdem")
        spdem = tecs_at(data, t, "spdem")
        tecs_th = tecs_at(data, t, "th")
        span = float("nan")
        mots = None
        if mo:
            mots = mo[1]
            span = max(mots) - min(mots)
        ph = phase_flags(data["MSG"], t)
        mark = ""
        if focus_t is not None and abs(t - focus_t) < step:
            mark = " <-- ONSET"
        print(f"{t:6.1f} {ph:>12} {a[1]:6.1f}{a[4]:6.1f} {a[2]:6.1f}{a[5]:6.1f} "
              f"{tho:5.2f} {alt:6.1f}{crt:6.2f} {aspd:5.1f} {tilt:5.1f} "
              f"{hdem:6.1f}{spdem:6.1f}{tecs_th:6.2f} "
              f"{ail:7.1f}{elev:7.1f} {span:5.2f} {tmix:5.2f}{mark}")
        rows.append({
            "t": t, "phase": ph,
            "Roll": a[1], "DesRoll": a[4], "Pitch": a[2], "DesPitch": a[5],
            "Yaw": a[3], "DesYaw": a[6],
            "R": r[1] if r else None, "RDes": r[2] if r else None, "ROut": r[3] if r else None,
            "P": r[4] if r else None, "PDes": r[5] if r else None, "POut": r[6] if r else None,
            "Y": r[7] if r else None, "YDes": r[8] if r else None, "YOut": r[9] if r else None,
            "ThO": tho, "Alt": alt, "CRt": crt, "AS": aspd, "Tilt": tilt,
            "hdem": hdem, "spdem": spdem, "TECSth": tecs_th,
            "Ail": ail, "Elev": elev, "MotSpan": span, "TMix": tmix,
            **({f"Mot{i+1}": mots[i] for i in range(8)} if mots else {}),
        })
        t += step

    # Detail centered on FIRST onset (surfaces/alt), not late att_30
    focus = focus_t if focus_t is not None else (t0 + t1) / 2
    print(f"\n=== Detail around FIRST onset t={focus:.2f}s (−4s … +2s) ===")
    print(f"{'t':>6} {'Tilt':>5} {'AS':>5} {'Alt':>6}{'CRt':>6} "
          f"{'Roll':>6}{'Pit':>6} {'Ail':>8}{'Elev':>8} {'Span':>5} {'ThO':>5} Mot1..8")
    t = focus - 4.0
    while t <= focus + 2.0:
        a = nearest(data["ATT"], t)
        q = nearest(data["QTUN"], t)
        mo = nearest(data["MOTT"], t)
        ae = nearest(data["AETR"], t)
        ti = nearest(data["TILT"], t)
        if a and q:
            span = (max(mo[1]) - min(mo[1])) if mo else float("nan")
            mots = " ".join(f"{x:4.2f}" for x in mo[1]) if mo else ""
            print(f"{t:6.2f} {ti[1].get('Tilt', float('nan')) if ti else float('nan'):5.1f} "
                  f"{aspeed_at(data, t):5.1f} "
                  f"{q[1].get('Alt', float('nan')):6.2f}{q[1].get('CRt', float('nan')):6.2f} "
                  f"{a[1]:6.1f}{a[2]:6.1f} "
                  f"{ae[1].get('Ail', float('nan')) if ae else float('nan'):8.1f}"
                  f"{ae[1].get('Elev', float('nan')) if ae else float('nan'):8.1f} "
                  f"{span:5.2f} {q[1].get('ThO', float('nan')):5.2f} {mots}")
        t += 0.25

    if csv_path and rows:
        path = Path(csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        preferred = ["t", "phase", "Roll", "DesRoll", "Pitch", "DesPitch", "Yaw", "DesYaw",
                     "R", "RDes", "ROut", "P", "PDes", "POut", "Y", "YDes", "YOut",
                     "ThO", "Alt", "CRt", "AS", "Tilt", "hdem", "spdem", "TECSth",
                     "Ail", "Elev", "MotSpan", "TMix",
                     "Mot1", "Mot2", "Mot3", "Mot4", "Mot5", "Mot6", "Mot7", "Mot8"]
        keys = sorted({k for row in rows for k in row})
        fieldnames = [k for k in preferred if k in keys] + [k for k in keys if k not in preferred]
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(rows)
        print(f"\nCSV written: {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bin", help="path to .BIN log")
    ap.add_argument("--t0", type=float, default=None, help="window start (s)")
    ap.add_argument("--t1", type=float, default=None, help="window end (s)")
    ap.add_argument("--step", type=float, default=0.5, help="dense table step (s)")
    ap.add_argument("--csv", default=None, help="optional CSV dump path")
    args = ap.parse_args()

    path = Path(args.bin)
    if not path.exists():
        sys.exit(f"missing log: {path}")

    print(f"Loading {path} ...")
    data = load(str(path))
    t0, t1 = find_window(data, args.t0, args.t1)
    print(f"Window: {t0:.2f} → {t1:.2f}s")
    summarize(data, t0, t1, step=args.step, csv_path=args.csv)


if __name__ == "__main__":
    main()
