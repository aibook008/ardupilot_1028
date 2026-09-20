#!/usr/bin/env python3
"""JT-20 transition power analysis from ArduPilot .BIN logs.

Supports:
  - New logs with MOTT (Mot1..Mot8 = mix+compensate thrust 0..1 before PWM)
  - Old logs without MOTT: recover u_i from RCOU via Q_M_SPIN_*/THST_EXPO

Power model (SITL disk, matches Tools/autotest/models/jt20.json Frame::init):
  v_out = velocity_max * sqrt((1-expo)*u + expo*u^2)   # Vscale=1 assumed
  T     = 0.5 * rho * A_eff * v_out^2                  # vin=0
  P     = power_factor * T                             # W, from hover P/T

Writes CSV summary and prints phase table. Used by docs/jt_power_anaylisis.md.
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from pymavlink import mavutil

# Defaults from current jt20.json + Frame::init (sea-level rho)
DEFAULTS = dict(
    mass=10.255,
    refCurrent=45.0,
    refVoltage=25.2,
    hoverThrOut=0.35,
    propExpo=0.65,
    num_motors=8,
    maxVoltage=25.2,
    rho=1.225,
    g=9.80665,
    pwm_min=1000.0,
    pwm_max=2000.0,
    spin_min=0.15,
    spin_max=0.95,
)


def disk_params(cfg):
    hover_thrust = cfg["mass"] * cfg["g"]
    hover_power = cfg["refCurrent"] * cfg["refVoltage"]
    hover_velocity = 2.0 * hover_power / hover_thrust
    velocity_max = hover_velocity / math.sqrt(cfg["hoverThrOut"])
    effective_disc = hover_thrust / (0.5 * cfg["rho"] * hover_velocity ** 2)
    A_eff = effective_disc / cfg["num_motors"]
    power_factor = hover_power / hover_thrust
    return velocity_max, A_eff, power_factor


def pwm_to_thrust(pwm, cfg):
    """Invert AP_Motors thr_lin to recover thrust 0..1 from PWM."""
    act = (pwm - cfg["pwm_min"]) / (cfg["pwm_max"] - cfg["pwm_min"])
    act = max(0.0, min(1.0, act))
    if act <= cfg["spin_min"]:
        return 0.0
    if act >= cfg["spin_max"]:
        return 1.0
    y = (act - cfg["spin_min"]) / (cfg["spin_max"] - cfg["spin_min"])
    e = cfg["propExpo"]
    if abs(e) < 1e-6:
        return y
    disc = (1 - e) ** 2 + 4 * e * y
    u = (-(1 - e) + math.sqrt(disc)) / (2 * e)
    return float(max(0.0, min(1.0, u)))


def thrust_power(u, velocity_max, A_eff, power_factor, rho, expo):
    u = max(0.0, min(1.0, float(u)))
    v_out = velocity_max * math.sqrt((1 - expo) * u + expo * u * u)
    T = 0.5 * rho * A_eff * (v_out * v_out)
    P = power_factor * T
    return T, P, v_out


def load_log(path, cfg, velocity_max, A_eff, power_factor):
    log = mavutil.mavlink_connection(str(path), notimestamps=True, robust_parsing=True)
    mott, rcou, qtun, tilt, baro, tecs, msgs = [], [], [], [], [], [], []
    parms = {}
    want = {
        "Q_M_PWM_MIN",
        "Q_M_PWM_MAX",
        "Q_M_SPIN_MIN",
        "Q_M_SPIN_MAX",
        "Q_M_THST_EXPO",
        "Q_TILT_MAX",
        "Q_TRANSITION_MS",
        "AIRSPEED_MIN",
        "Q_TRAN_PIT_MAX",
    }
    while True:
        m = log.recv_match(blocking=False)
        if m is None:
            break
        t = m.get_type()
        if t == "MOTT":
            mott.append(
                [
                    m.TimeUS / 1e6,
                    m.Mot1,
                    m.Mot2,
                    m.Mot3,
                    m.Mot4,
                    m.Mot5,
                    m.Mot6,
                    m.Mot7,
                    m.Mot8,
                ]
            )
        elif t == "RCOU":
            rcou.append([m.TimeUS / 1e6] + [getattr(m, f"C{i}") for i in range(5, 13)])
        elif t == "QTUN":
            qtun.append(
                (
                    m.TimeUS / 1e6,
                    m.ThI,
                    m.ThO,
                    m.ThH,
                    int(getattr(m, "Trn", -1)),
                    m.Alt,
                    m.DAlt,
                )
            )
        elif t == "TILT":
            tilt.append((m.TimeUS / 1e6, m.Tilt))
        elif t == "BARO":
            baro.append((m.TimeUS / 1e6, m.Alt))
        elif t == "TECS":
            tecs.append((m.TimeUS / 1e6, getattr(m, "th", np.nan), getattr(m, "h", np.nan)))
        elif t == "PARM" and m.Name in want:
            parms[m.Name] = m.Value
        elif t == "MSG":
            s = str(m.Message)
            if any(k in s for k in ("Transition", "align", "Takeoff", "Mission")):
                msgs.append((m.TimeUS / 1e6, s))

    # override motor linearization from log if present
    if "Q_M_PWM_MIN" in parms:
        cfg["pwm_min"] = parms["Q_M_PWM_MIN"]
    if "Q_M_PWM_MAX" in parms:
        cfg["pwm_max"] = parms["Q_M_PWM_MAX"]
    if "Q_M_SPIN_MIN" in parms:
        cfg["spin_min"] = parms["Q_M_SPIN_MIN"]
    if "Q_M_SPIN_MAX" in parms:
        cfg["spin_max"] = parms["Q_M_SPIN_MAX"]
    if "Q_M_THST_EXPO" in parms:
        cfg["propExpo"] = parms["Q_M_THST_EXPO"]

    mott = np.array(mott) if mott else None
    rcou = np.array(rcou) if rcou else None
    qtun = np.array(qtun) if qtun else None
    tilt = np.array(tilt) if tilt else None
    baro = np.array(baro) if baro else None
    tecs = np.array(tecs) if tecs else None

    use_mott = mott is not None and len(mott) > 0
    if use_mott:
        tgrid = mott[:, 0]
        U = mott[:, 1:9]
        source = "MOTT"
    else:
        if rcou is None or len(rcou) == 0:
            raise SystemExit(f"{path}: no MOTT and no RCOU")
        tgrid = rcou[:, 0]
        U = np.zeros((len(rcou), 8))
        for i in range(len(rcou)):
            for j in range(8):
                U[i, j] = pwm_to_thrust(rcou[i, j + 1], cfg)
        source = "RCOU_inv"

    # nearest helpers
    def near(arr, t0):
        if arr is None or len(arr) == 0:
            return None
        return arr[np.argmin(np.abs(arr[:, 0] - t0))]

    rows = []
    for i, t0 in enumerate(tgrid):
        us = U[i]
        Ts, Ps, Vs = [], [], []
        for u in us:
            T, P, v = thrust_power(u, velocity_max, A_eff, power_factor, cfg["rho"], cfg["propExpo"])
            Ts.append(T)
            Ps.append(P)
            Vs.append(v)
        q = near(qtun, t0)
        ti = near(tilt, t0)
        b = near(baro, t0)
        te = near(tecs, t0)
        trn = int(q[4]) if q is not None else -1
        thi = float(q[1]) if q is not None else float("nan")
        tho = float(q[2]) if q is not None else float("nan")
        tilt_deg = float(ti[1]) if ti is not None else float("nan")
        baro_alt = float(b[1]) if b is not None else float("nan")
        tecs_th = float(te[1]) if te is not None else float("nan")
        rows.append(
            dict(
                t=float(t0),
                source=source,
                Trn=trn,
                ThI=thi,
                ThO=tho,
                Tilt=tilt_deg,
                BARO=baro_alt,
                Tecs_th=tecs_th,
                **{f"u{j+1}": float(us[j]) for j in range(8)},
                **{f"T{j+1}": float(Ts[j]) for j in range(8)},
                **{f"P{j+1}": float(Ps[j]) for j in range(8)},
                u_in=float(np.mean(us[:4])),
                u_out=float(np.mean(us[4:])),
                T_in=float(np.sum(Ts[:4])),
                T_out=float(np.sum(Ts[4:])),
                T_tot=float(np.sum(Ts)),
                P_in=float(np.sum(Ps[:4])),
                P_out=float(np.sum(Ps[4:])),
                P_tot=float(np.sum(Ps)),
            )
        )
    return rows, msgs, parms, source


def phase_of(row):
    # Prefer Trn when QTUN present; also use tilt
    trn, tilt = row["Trn"], row["Tilt"]
    if trn == 2 or (isinstance(tilt, float) and tilt >= 85):
        return "DONE/FW"
    if trn == 1 or (isinstance(tilt, float) and 45 < tilt < 85):
        return "TIMER"
    if isinstance(tilt, float) and tilt > 1:
        return "WAIT_tilt"
    if trn == 0:
        return "WAIT/VTOL"
    return "OTHER"


def summarize(rows):
    by = defaultdict(list)
    for r in rows:
        by[phase_of(r)].append(r)
    summary = []
    for phase in ["WAIT/VTOL", "WAIT_tilt", "TIMER", "DONE/FW", "OTHER"]:
        rs = by.get(phase)
        if not rs:
            continue
        summary.append(
            dict(
                phase=phase,
                n=len(rs),
                t0=rs[0]["t"],
                t1=rs[-1]["t"],
                ThI_mean=float(np.nanmean([r["ThI"] for r in rs])),
                u_in_mean=float(np.mean([r["u_in"] for r in rs])),
                u_out_mean=float(np.mean([r["u_out"] for r in rs])),
                T_tot_mean=float(np.mean([r["T_tot"] for r in rs])),
                T_in_mean=float(np.mean([r["T_in"] for r in rs])),
                T_out_mean=float(np.mean([r["T_out"] for r in rs])),
                P_tot_mean=float(np.mean([r["P_tot"] for r in rs])),
                P_in_mean=float(np.mean([r["P_in"] for r in rs])),
                P_out_mean=float(np.mean([r["P_out"] for r in rs])),
                P_tot_max=float(np.max([r["P_tot"] for r in rs])),
                Tilt_mean=float(np.nanmean([r["Tilt"] for r in rs])),
            )
        )
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", help=".BIN log paths")
    ap.add_argument("--out-dir", default="docs/scripts/out", help="CSV output directory")
    args = ap.parse_args()

    cfg = dict(DEFAULTS)
    global velocity_max, A_eff, power_factor
    velocity_max, A_eff, power_factor = disk_params(cfg)
    print(
        f"disk model: velocity_max={velocity_max:.2f} m/s  A_eff={A_eff:.4f} m^2  "
        f"power_factor={power_factor:.3f} W/N"
    )

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    all_summaries = {}
    for logpath in args.logs:
        path = Path(logpath)
        rows, msgs, parms, source = load_log(path, cfg, velocity_max, A_eff, power_factor)
        summary = summarize(rows)
        all_summaries[path.name] = (summary, source, msgs, parms)

        csv_path = out_dir / f"{path.stem}_power_timeseries.csv"
        if rows:
            with csv_path.open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
        print(f"\n=== {path.name}  source={source}  samples={len(rows)} ===")
        print(f"wrote {csv_path}")
        print(
            f"{'phase':<10} {'n':>5} {'ThI':>6} {'u_in':>6} {'u_out':>6} "
            f"{'T_tot':>7} {'P_tot':>8} {'P_in':>8} {'P_out':>8} {'Pmax':>8} {'Tilt':>6}"
        )
        for s in summary:
            print(
                f"{s['phase']:<10} {s['n']:5d} {s['ThI_mean']:6.2f} {s['u_in_mean']:6.2f} "
                f"{s['u_out_mean']:6.2f} {s['T_tot_mean']:7.1f} {s['P_tot_mean']:8.0f} "
                f"{s['P_in_mean']:8.0f} {s['P_out_mean']:8.0f} {s['P_tot_max']:8.0f} {s['Tilt_mean']:6.1f}"
            )

    # comparison CSV
    cmp_path = out_dir / "jt20_power_phase_compare.csv"
    with cmp_path.open("w", newline="") as f:
        fields = [
            "log",
            "source",
            "phase",
            "n",
            "ThI_mean",
            "u_in_mean",
            "u_out_mean",
            "T_tot_mean",
            "P_tot_mean",
            "P_in_mean",
            "P_out_mean",
            "P_tot_max",
            "Tilt_mean",
        ]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for name, (summary, source, _, _) in all_summaries.items():
            for s in summary:
                w.writerow(dict(log=name, source=source, **{k: s[k] for k in fields if k in s}))
    print(f"\nwrote {cmp_path}")


if __name__ == "__main__":
    main()
