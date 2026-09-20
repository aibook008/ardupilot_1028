#!/usr/bin/env python3
"""DF-200 force/moment budget analyzer.

Reusable tool for the DF-200 transition campaign. Reads the three SITL debug
logs produced by this branch and the DataFlash BIN, then prints a force/moment
budget aligned in time so onset causes can be separated from consequences.

Inputs (all optional, used if present):
  /tmp/df200_aero.log   - aerodynamic F/M breakdown      (SIM_Plane.cpp)
  /tmp/df200_rotor.log  - rotor F/M + per-motor Mxi/cmd   (SIM_Frame.cpp)
  /tmp/df200_mix.log    - AP_MotorsMatrix mixer internals (AP_MotorsMatrix.cpp)
  <BIN>                 - DataFlash: ATT/RATE/MOTT/AETR/TILT/QTUN/ARSP/MSG

Physics:
  - body frame NED: Z down, so thrust Fz is negative (up). Lift reported by
    aero F.z is also negative (up). Gravity = mass*g positive (down).
  - mass: read from df200.json * 1.5 (QuadPlane scales), or override --mass.
  - Mgeom = sum(r x F); Mrotor = sum rotor reaction; Mtot = Mgeom + Mrotor.
  - Per-motor Mxi (geom torque X) shows which motor makes a net roll moment.

Usage:
  python3 docs/scripts/df200_force_budget.py logs/00000158.BIN
  python3 docs/scripts/df200_force_budget.py logs/00000158.BIN --t0 138 --t1 140
  python3 docs/scripts/df200_force_budget.py logs/00000158.BIN --mass 350
"""
from __future__ import annotations
import argparse, re, os
from pathlib import Path
import numpy as np
from pymavlink import mavutil

AERO = "/tmp/df200_aero.log"
ROTOR = "/tmp/df200_rotor.log"
MIX = "/tmp/df200_mix.log"
JSON = "Tools/autotest/models/df200.json"


def _grab(line, tag):
    m = re.search(rf'{tag}=([^ ]+)', line)
    return m.group(1) if m else None


def _vec(s):
    return np.fromstring(s, sep=',')


def load_rotor(path):
    """Return list of dicts: t, AS, F, Mtot, Mgeom, Mrotor, pqr, amp,
    cmd(8,), Mxi(8,), Tzi(8,), Txi(8,)."""
    rows = []
    if not Path(path).exists():
        return rows
    for L in Path(path).read_text().splitlines():
        t = _grab(L, 't')
        if not t:
            continue
        AS = _grab(L, 'AS'); F = _grab(L, 'F'); Mtot = _grab(L, 'Mtot')
        Mgeom = _grab(L, 'Mgeom'); Mrotor = _grab(L, 'Mrotor'); pqr = _grab(L, 'pqr')
        if not all([t, AS, F, Mtot, Mgeom, Mrotor, pqr]):
            continue
        p, q, r = map(float, pqr.split(','))
        d = dict(t=float(t), AS=float(AS), F=_vec(F), Mtot=_vec(Mtot),
                 Mgeom=_vec(Mgeom), Mrotor=_vec(Mrotor),
                 pqr=np.array([p, q, r]), amp=max(abs(p), abs(q), abs(r)))
        for tag, key in (('cmd', 'cmd'), ('Mxi', 'Mxi'), ('Tzi', 'Tzi'), ('Txi', 'Txi')):
            v = _grab(L, tag)
            if v:
                try:
                    d[key] = _vec(v)
                except Exception:
                    pass
        rows.append(d)
    return rows


def load_aero(path):
    """t, AS, F(3,), M(3,), pqr, amp, plus La_da/Ma_de/Na_dr if present."""
    rows = []
    if not Path(path).exists():
        return rows
    for L in Path(path).read_text().splitlines():
        t = _grab(L, 't')
        if not t:
            continue
        AS = _grab(L, 'AS'); M = _grab(L, 'M'); pqr = _grab(L, 'pqr'); F = _grab(L, 'F')
        if not all([t, AS, M, pqr, F]):
            continue
        p, q, r = map(float, pqr.split(','))
        d = dict(t=float(t), AS=float(AS), F=_vec(F), M=_vec(M),
                 pqr=np.array([p, q, r]), amp=max(abs(p), abs(q), abs(r)))
        for tag in ('La_da', 'Ma_de', 'Na_dr', 'La_p', 'Na_r', 'Ma_a', 'Ma_cg'):
            v = _grab(L, tag)
            if v:
                try:
                    d[tag] = float(v)
                except Exception:
                    pass
        rows.append(d)
    return rows


def load_mix(path):
    """t, thr_in, thr_best, thr_adj, roll_t, pitch_t, yaw_t, yaw_allow,
    rpy_scale, rpy_lo, rpy_hi, lim, out(8,)."""
    rows = []
    if not Path(path).exists():
        return rows
    for L in Path(path).read_text().splitlines():
        t = _grab(L, 't')
        if not t:
            continue
        d = dict(t=float(t))
        for k in ['thr_in', 'thr_best', 'thr_adj', 'roll_t', 'pitch_t',
                  'yaw_t', 'yaw_allow', 'rpy_scale', 'rpy_lo', 'rpy_hi']:
            v = _grab(L, k)
            if v:
                try:
                    d[k] = float(v)
                except Exception:
                    pass
        lim = _grab(L, 'lim')
        if lim:
            d['lim'] = lim
        out = _grab(L, 'out')
        if out:
            try:
                d['out'] = _vec(out)
            except Exception:
                pass
        rows.append(d)
    return rows


def nearest(rows, t, tol=0.06):
    if not rows:
        return None
    best = min(rows, key=lambda r: abs(r['t'] - t))
    return best if abs(best['t'] - t) <= tol else None


def read_mass():
    """Flight mass = json mass * 1.5 (QuadPlane scaling)."""
    p = Path(JSON)
    if not p.exists():
        return 350.0
    m = re.search(r'"mass"\s*:\s*([0-9.]+)', p.read_text())
    if not m:
        return 350.0
    return float(m.group(1)) * 1.5


def bin_window(binpath, t0, t1):
    """ATT/RATE/MOTT/AETR/TILT/QTUN/ARSP samples in [t0,t1]."""
    m = mavutil.mavlink_connection(binpath)
    att = rate = mott = arsp = aetr = tilt = qtun = None
    out = []
    while True:
        msg = m.recv_match(type=['ATT', 'RATE', 'MOTT', 'ARSP', 'AETR', 'TILT', 'QTUN'], blocking=False)
        if msg is None:
            break
        ts = msg.TimeUS / 1e6
        if ts < t0 or ts > t1:
            continue
        t = msg.get_type()
        if t == 'ATT': att = msg
        elif t == 'RATE': rate = msg
        elif t == 'MOTT': mott = msg
        elif t == 'ARSP': arsp = msg
        elif t == 'AETR': aetr = msg
        elif t == 'TILT': tilt = msg
        elif t == 'QTUN': qtun = msg
        if t != 'RATE' or not all([att, rate, mott, arsp, aetr, tilt, qtun]):
            continue
        vs = [getattr(mott, f'Mot{i}') for i in range(1, 9)]
        out.append(dict(t=ts, AS=arsp.Airspeed, tilt=tilt.Tilt,
                        roll=att.Roll, desroll=att.DesRoll, pitch=att.Pitch, despitch=att.DesPitch,
                        r=rate.R, p=rate.P, y=rate.Y,
                        ThO=qtun.ThO, Alt=qtun.Alt, CRt=qtun.CRt,
                        Thr=aetr.Thr, Ail=aetr.Ail, Elev=aetr.Elev, Rudd=aetr.Rudd,
                        mot=vs, span=max(vs) - min(vs)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bin')
    ap.add_argument('--t0', type=float, default=None, help='window start (auto: onset-3)')
    ap.add_argument('--t1', type=float, default=None, help='window end (auto: onset+3)')
    ap.add_argument('--mass', type=float, default=None, help='override flight mass kg')
    ap.add_argument('--step', type=float, default=0.1, help='downsample interval s')
    args = ap.parse_args()

    rotor = load_rotor(ROTOR)
    aero = load_aero(AERO)
    mix = load_mix(MIX)
    mass = args.mass or read_mass()
    g = mass * 9.81
    print(f"# mass={mass:.1f} kg  gravity={g:.0f} N  (rotor/aero/mix rows: "
          f"{len(rotor)}/{len(aero)}/{len(mix)})")

    # auto window: first |pqr|>0.1 in rotor log after AS>30
    if args.t0 is None or args.t1 is None:
        onset = None
        for r in rotor:
            if r['AS'] > 30 and r['amp'] > 0.1:
                onset = r['t']; break
        if onset is None:
            onset = rotor[-1]['t'] if rotor else 100.0
        t0 = args.t0 if args.t0 is not None else onset - 3
        t1 = args.t1 if args.t1 is not None else onset + 3
    else:
        t0, t1 = args.t0, args.t1

    print(f"# window {t0:.2f} -> {t1:.2f}  (downsample {args.step}s)\n")
    hdr = (f"{'t':>7} {'AS':>5} {'tilt':>5} {'rotFz':>6} {'aeroFz':>7} {'sum_up':>7} "
           f"{'vs_g':>7} {'Mx':>7} {'My':>7} {'Mz':>6} {'roll_t':>7} {'rpy_sc':>6} {'lim':>4} "
           f"{'Roll':>6} {'r':>6} {'ThO':>4} {'Alt':>5} {'CRt':>5}")
    print(hdr)
    print('-' * len(hdr))

    last = -1e9
    for r in rotor:
        t = r['t']
        if t < t0 or t > t1:
            continue
        if t - last < args.step:
            continue
        last = t
        a = nearest(aero, t)
        mx = nearest(mix, t)
        rot_fz = -r['F'][2]                 # up positive
        aero_fz = -a['F'][2] if a else 0.0  # up positive
        up = rot_fz + aero_fz
        M = r['Mtot']
        roll_t = mx['roll_t'] if mx and 'roll_t' in mx else float('nan')
        rpy_sc = mx['rpy_scale'] if mx and 'rpy_scale' in mx else float('nan')
        lim = mx['lim'] if mx and 'lim' in mx else '  '
        # BIN attitude
        binrow = None
        # cheap: pull a few BIN samples around t (skip for speed if many)
        b = bin_window(args.bin, t - 0.05, t + 0.05)
        if b:
            binrow = min(b, key=lambda x: abs(x['t'] - t))
        roll = binrow['roll'] if binrow else float('nan')
        rr = binrow['r'] if binrow else float('nan')
        tho = binrow['ThO'] if binrow else float('nan')
        alt = binrow['Alt'] if binrow else float('nan')
        crt = binrow['CRt'] if binrow else float('nan')
        print(f"{t:7.2f} {r['AS']:5.1f} "
              f"{(binrow['tilt'] if binrow else 0):5.1f} "
              f"{rot_fz:6.0f} {aero_fz:7.0f} {up:7.0f} {up-g:+7.0f} "
              f"{M[0]:7.0f} {M[1]:7.0f} {M[2]:6.0f} "
              f"{roll_t:7.2f} {rpy_sc:6.2f} {lim:>4} "
              f"{roll:6.1f} {rr:6.1f} {tho:4.2f} {alt:5.1f} {crt:5.2f}")

    print("\n# columns: rotFz/aeroFz/sum_up in N (up+); vs_g = sum_up - gravity; "
          "Mx/My/Mz rotor total Nm; roll_t = mixer roll thrust request; "
          "rpy_sc<1 & lim=RPYT = mixer saturated.")


if __name__ == "__main__":
    main()
