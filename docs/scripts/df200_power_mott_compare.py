#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DF-200 transition power compare from DataFlash MOTT (no rotor.log required).

Same method as the 203-vs-206 hand analysis:
  - Outer MOT_5..8: prop60_power(cmd * 3181)          [static table]
  - Inner MOT_1..4: prop40_power(cmd * 4200, velin)
    velin ≈ AS * sin(tilt_deg)   # proxy; true SITL uses body axial velin

Absolute kW may differ from df200_power_analysis.py (which needs /tmp/df200_rotor.log).
Relative compare across BINs with this script is consistent.

Usage:
  python3 docs/scripts/df200_power_mott_compare.py \\
      logs/00000193.BIN logs/00000203.BIN logs/00000206.BIN

  python3 docs/scripts/df200_power_mott_compare.py logs/00000203.BIN logs/00000206.BIN \\
      --labels "203_EN0,206_EN1" --csv docs/scripts/out/power_mott_compare.csv
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import sys
from bisect import bisect_left
from pathlib import Path

import numpy as np
from pymavlink import mavutil

# Reuse motor tables from df200_power_analysis.py
_PA = Path(__file__).resolve().parent / "df200_power_analysis.py"
_spec = importlib.util.spec_from_file_location("df200_power_analysis", _PA)
pa = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pa)


def _nearest(arr, t):
    if not arr:
        return None
    times = [a[0] for a in arr]
    i = bisect_left(times, t)
    cands = []
    if i < len(arr):
        cands.append(arr[i])
    if i > 0:
        cands.append(arr[i - 1])
    return min(cands, key=lambda a: abs(a[0] - t))


def load_bin(path: str):
    m = mavutil.mavlink_connection(path)
    mott, arsp, tilt, msg = [], [], [], []
    parms = {}
    while True:
        x = m.recv_match(blocking=False)
        if x is None:
            break
        typ = x.get_type()
        if typ == "PARM":
            try:
                parms[x.Name] = float(x.Value)
            except Exception:
                pass
            continue
        try:
            ts = x.TimeUS / 1e6
        except Exception:
            continue
        if typ == "MOTT":
            d = x.to_dict()
            cmds = [float(d.get(f"Mot{i}", 0.0) or 0.0) for i in range(1, 9)]
            mott.append((ts, cmds))
        elif typ == "ARSP":
            arsp.append((ts, float(getattr(x, "Airspeed", 0.0) or 0.0)))
        elif typ == "TILT":
            d = x.to_dict()
            tilt.append(
                (
                    ts,
                    float(d.get("Tilt") or 0.0),
                    d.get("Crd"),
                )
            )
        elif typ == "MSG":
            msg.append((ts, getattr(x, "Message", "")))
    return mott, arsp, tilt, msg, parms


def phase_edges(msg):
    st = next((ts for ts, me in msg if "Transition started" in me), None)
    asr = next((ts for ts, me in msg if "airspeed reached" in me), None)
    fw = next((ts for ts, me in msg if "FW done" in me), None)
    return st, asr, fw


def series_from_mott(mott, arsp, tilt, t_lo, t_hi):
    ts, P, Pin, Pout, Tilt, AS = [], [], [], [], [], []
    for t, cmds in mott:
        if t < t_lo or t > t_hi:
            continue
        a = _nearest(arsp, t)
        ti = _nearest(tilt, t)
        aspd = a[1] if a else 0.0
        tilt_deg = ti[1] if ti else 0.0
        velin = aspd * np.sin(np.radians(max(tilt_deg, 0.0)))
        pin = sum(pa.prop40_power(max(c, 0.0) * 4200.0, max(velin, 0.0)) for c in cmds[:4])
        pout = sum(pa.prop60_power(max(c, 0.0) * 3181.0) for c in cmds[4:8])
        ts.append(t)
        Pin.append(pin / 1000.0)
        Pout.append(pout / 1000.0)
        P.append((pin + pout) / 1000.0)
        Tilt.append(tilt_deg)
        AS.append(aspd)
    return {
        "ts": np.asarray(ts),
        "P": np.asarray(P),
        "Pin": np.asarray(Pin),
        "Pout": np.asarray(Pout),
        "Tilt": np.asarray(Tilt),
        "AS": np.asarray(AS),
    }


def _stats(ts, P, Pin, Pout, mask, name):
    if mask is None or not np.any(mask) or mask.sum() < 2:
        return {
            "name": name,
            "n": 0,
            "dur_s": 0.0,
            "mean_kW": float("nan"),
            "p95_kW": float("nan"),
            "peak_kW": float("nan"),
            "Pin_mean": float("nan"),
            "Pout_mean": float("nan"),
            "Wh": float("nan"),
        }
    d = P[mask]
    tt = ts[mask]
    return {
        "name": name,
        "n": int(mask.sum()),
        "dur_s": float(tt[-1] - tt[0]),
        "mean_kW": float(d.mean()),
        "p95_kW": float(np.percentile(d, 95)),
        "peak_kW": float(d.max()),
        "Pin_mean": float(Pin[mask].mean()),
        "Pout_mean": float(Pout[mask].mean()),
        "Wh": float(np.trapz(d, tt) / 3600.0 * 1000.0),
    }


def analyze_one(path: str, label: str | None = None):
    mott, arsp, tilt, msg, parms = load_bin(path)
    st, asr, fw = phase_edges(msg)
    if st is None:
        raise SystemExit(f"{path}: no 'Transition started' MSG")
    lab = label or Path(path).stem
    t_hi = (fw + 20.0) if fw is not None else (asr + 30.0 if asr else st + 90.0)
    ser = series_from_mott(mott, arsp, tilt, st - 25.0, t_hi)
    ts, P, Pin, Pout, Tilt = ser["ts"], ser["P"], ser["Pin"], ser["Pout"], ser["Tilt"]
    AS = ser["AS"]

    climb = (ts >= st - 25.0) & (ts < st)
    wait = (ts >= st) & ((asr is None) | (ts < asr))
    timer = (asr is not None) & (ts >= asr) & ((fw is None) | (ts < fw))
    fw20 = (fw is not None) & (ts >= fw) & (ts < fw + 20.0)
    whole = (ts >= st) & ((fw is None) | (ts < fw))

    phases = [
        _stats(ts, P, Pin, Pout, climb, "vtol_pre25"),
        _stats(ts, P, Pin, Pout, wait, "WAIT"),
        _stats(ts, P, Pin, Pout, timer, "TIMER"),
        _stats(ts, P, Pin, Pout, fw20, "FW_20s"),
        _stats(ts, P, Pin, Pout, whole, "whole_trans"),
    ]

    tilt_bins = []
    for lo, hi in [(0, 10), (10, 20), (20, 30), (30, 45), (45, 46)]:
        if hi < 46:
            m = wait & (Tilt >= lo) & (Tilt < hi)
        else:
            m = wait & (Tilt >= lo)
        stt = _stats(ts, P, Pin, Pout, m, f"WAIT_tilt[{lo},{hi})")
        if stt["n"] >= 5:
            stt["AS_mean"] = float(AS[m].mean())
            tilt_bins.append(stt)

    crd = {
        "Q_TILT_CRD_EN": parms.get("Q_TILT_CRD_EN"),
        "AIRSPEED_MIN": parms.get("AIRSPEED_MIN"),
        "Q_TILT_MAX": parms.get("Q_TILT_MAX"),
    }
    return {
        "label": lab,
        "path": path,
        "st": st,
        "asr": asr,
        "fw": fw,
        "wait_s": (asr - st) if asr else None,
        "timer_s": (fw - asr) if (fw and asr) else None,
        "parms": crd,
        "phases": phases,
        "tilt_bins": tilt_bins,
    }


def _fmt_row(st):
    if st["n"] == 0:
        return f"  {st['name']:18s}  (no samples)"
    return (
        f"  {st['name']:18s} n={st['n']:4d}  dur={st['dur_s']:5.1f}s  "
        f"mean={st['mean_kW']:5.1f}  p95={st['p95_kW']:5.1f}  peak={st['peak_kW']:5.1f} kW  "
        f"Pin={st['Pin_mean']:4.1f} Pout={st['Pout_mean']:4.1f}  E≈{st['Wh']:6.1f} Wh"
    )


def print_report(results):
    for r in results:
        print(f"\n=== {r['label']}  {r['path']} ===")
        p = r["parms"]
        print(
            f"  CRD_EN={p.get('Q_TILT_CRD_EN')}  AIRSPEED_MIN={p.get('AIRSPEED_MIN')}  "
            f"Q_TILT_MAX={p.get('Q_TILT_MAX')}"
        )
        print(
            f"  Transition {r['st']:.2f} → AS {r['asr']} → FW {r['fw']}  "
            f"(WAIT {r['wait_s']:.1f}s, TIMER {r['timer_s']})"
            if r["wait_s"] is not None
            else f"  Transition {r['st']:.2f} (no airspeed reached)"
        )
        for ph in r["phases"]:
            print(_fmt_row(ph))
        if r["tilt_bins"]:
            print("  WAIT by tilt bin:")
            for b in r["tilt_bins"]:
                extra = f" AS={b.get('AS_mean', float('nan')):4.1f}"
                print(_fmt_row(b) + extra)

    if len(results) < 2:
        return

    print("\n=== WAIT delta vs first BIN ===")
    base = results[0]
    bw = next(p for p in base["phases"] if p["name"] == "WAIT")
    for r in results[1:]:
        w = next(p for p in r["phases"] if p["name"] == "WAIT")
        if bw["n"] < 2 or w["n"] < 2:
            continue
        dE = w["Wh"] - bw["Wh"]
        dMean = w["mean_kW"] - bw["mean_kW"]
        pct = (w["Wh"] / bw["Wh"] - 1.0) * 100.0 if bw["Wh"] else float("nan")
        print(
            f"  {r['label']} − {base['label']}:  "
            f"E {bw['Wh']:.1f}→{w['Wh']:.1f} Wh (Δ{dE:+.1f}, {pct:+.0f}%)  "
            f"mean {bw['mean_kW']:.1f}→{w['mean_kW']:.1f} kW (Δ{dMean:+.1f})  "
            f"peak {bw['peak_kW']:.1f}→{w['peak_kW']:.1f}  "
            f"dur {bw['dur_s']:.1f}→{w['dur_s']:.1f}s"
        )

    print(
        "\nNote: MOTT+AS*sin(tilt) proxy — not identical to rotor.log velin replay. "
        "FW MOTT≈0 → FW_20s power shows ~0 (use ESC/BAT or rotor.log for cruise)."
    )


def write_csv(results, path: str):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for r in results:
        for ph in r["phases"] + r["tilt_bins"]:
            rows.append(
                {
                    "label": r["label"],
                    "bin": r["path"],
                    "CRD_EN": r["parms"].get("Q_TILT_CRD_EN"),
                    "AIRSPEED_MIN": r["parms"].get("AIRSPEED_MIN"),
                    "phase": ph["name"],
                    "n": ph["n"],
                    "dur_s": ph["dur_s"],
                    "mean_kW": ph["mean_kW"],
                    "p95_kW": ph["p95_kW"],
                    "peak_kW": ph["peak_kW"],
                    "Pin_mean_kW": ph["Pin_mean"],
                    "Pout_mean_kW": ph["Pout_mean"],
                    "Wh": ph["Wh"],
                    "AS_mean": ph.get("AS_mean", ""),
                    "wait_s": r["wait_s"],
                    "timer_s": r["timer_s"],
                }
            )
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nWrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bins", nargs="+", help="DataFlash BIN paths")
    ap.add_argument("--labels", default=None, help="Comma labels matching bins order")
    ap.add_argument("--csv", default=None, help="Write summary CSV")
    args = ap.parse_args()
    labels = None
    if args.labels:
        labels = [s.strip() for s in args.labels.split(",")]
        if len(labels) != len(args.bins):
            sys.exit("--labels count must match bins")
    results = []
    for i, b in enumerate(args.bins):
        lab = labels[i] if labels else None
        results.append(analyze_one(b, lab))
    print_report(results)
    if args.csv:
        write_csv(results, args.csv)


if __name__ == "__main__":
    main()
