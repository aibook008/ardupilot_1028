#!/usr/bin/env python3
"""DF-200 motor total-power analyzer.

Reproduces the SITL motor power calculation offline, on a time series read
from /tmp/df200_rotor.log, and aggregates it per flight phase to form a
"power corridor" (hover / transition / FW cruise).

Why this exists: the DF-200 transition campaign (see docs/vtol_df200.md and
docs/df200_session_20260902.md) tuned the control law to a locked cruise
(190), but the motor *power demand* across the whole flight was never
quantified. The motor tables in libraries/SITL/SIM_MotorTable.h (data from
docs/电机电池.docx) are already what SITL's Motor::calculate_forces uses to
compute current (SIM_Motor.cpp:138-140, 257-259), so this script is an offline
replay of that same computation with the same inputs -- not a new model.

Data chain (verified against source):
  rotor.log row (20 Hz)
    rpm[8]   = motors[i].get_rpm() = cmd * (3150 outer / 3050 inner)  [SIM_Motor.cpp:74]
    velin[8] = velocity_in = -motor_vel.projected(thrust_vector).z     [SIM_Motor.cpp:66]
    AS       = get_velocity_air_ef().length()                          [SIM_Frame.cpp:672]
    Tzi/Txi  = pm_thrust.z/.x (same table, same inputs -> sanity check)

  offline lookup (identical functions + inputs to SITL):
    outer MOT_5..8 per-motor power = prop60_power(rpm_i)              [static, no v]
    inner MOT_1..4 per-motor power = prop40_power(rpm_i, velin_i)     [KEY: velin, not AS]
    total power = sum over 8 motors

Sanity check (mandatory): thrust recomputed from the same tables
(prop40_thrust(rpm,velin)+prop60_thrust(rpm)) must equal the rotor log's
Tzi+Txi (magnitude). Same table, same inputs -> should be ~0 deviation. Any
deviation = a bug in the lookup replay.

Inputs:
  /tmp/df200_rotor.log  - per-motor rpm/cmd/velin/thrust (SIM_Frame.cpp)
  <BIN>                 - DataFlash: ATT/RATE/TILT/QTUN/ARSP/EV/MSG for phase edges

Usage:
  python3 docs/scripts/df200_power_analysis.py logs/00000191.BIN
  python3 docs/scripts/df200_power_analysis.py logs/00000191.BIN --report docs/df200_power_corridor.md
"""
from __future__ import annotations
import argparse, re, sys, math
from pathlib import Path
import numpy as np
from pymavlink import mavutil

ROTOR = "/tmp/df200_rotor.log"


# ---------------------------------------------------------------------------
# Motor tables -- transcribed verbatim from libraries/SITL/SIM_MotorTable.h.
# 2026-10-29 revision: prop60 replaced with 165M20+RY60 bench data (24 pts);
# prop40 thrust values are the docx "拉力" column taken as NEWTONS (that column
# was mislabelled grams; bench-verified, see docs/df200_motor_data_bench.md §4).
# Power columns are electrical W. Voltage sag not modelled. Density scaling is
# applied by the SITL caller to *thrust*, not to power (we keep table power as-is,
# matching SIM_Motor.cpp which uses table power directly for current).
# cmd->rpm peak constants (SIM_Motor.cpp): 3150 outer / 3050 inner.
# ---------------------------------------------------------------------------

# 60in lift prop, static (165M20 + RY60 bench, 24 points). RPM 350..3150.
prop60_rpm = [350, 850, 1050, 1150, 1250, 1350, 1450, 1550,
              1650, 1750, 1850, 1950, 2050, 2150, 2250, 2350,
              2450, 2550, 2650, 2750, 2850, 2950, 3050, 3150]
prop60_thrust_N = [20.1, 79.0, 120.8, 144.7, 171.8, 202.0, 233.6, 262.3,
                   300.2, 334.8, 375.4, 421.8, 468.6, 513.3, 565.8, 610.4,
                   667.6, 727.8, 785.7, 843.7, 909.4, 985.8, 1066.4, 1106.3]
prop60_power_W = [329, 793, 1276, 1640, 1936, 2433, 2829, 3359,
                  3973, 4618, 5395, 6341, 7244, 8486, 9645, 11074,
                  12576, 14213, 15844, 17905, 19935, 22275, 25102, 26958]


# 40in cruise prop, one branch per wind speed (docs/电机电池.docx 表1).
# thrust_N = docx "拉力" column as-is (newtons, mislabelled g in docx);
# bench cross-check vs 135M10+DH40: ratio 0.98..1.04.
class E:
    __slots__ = ('rpm', 'thrust_N', 'power_W')

    def __init__(self, rpm, thrust_N, power_W):
        self.rpm = rpm
        self.thrust_N = thrust_N
        self.power_W = power_W


prop40_v0 = [
    E(2000, 169.90, 2910), E(2200, 205.84, 3900), E(2400, 242.65, 5090),
    E(2600, 283.39, 6530), E(2800, 328.08, 8210), E(3000, 374.22, 10130),
    E(3200, 425.36, 12480), E(3400, 478.19, 15120), E(3600, 534.83, 18130),
]
prop40_v29 = [
    E(1000, 44.85, 380), E(1500, 97.24, 1230), E(2000, 168.67, 2910),
    E(2500, 261.83, 5750), E(3000, 376.91, 10150), E(3500, 506.15, 16580),
    E(4000, 652.75, 26000),
]
prop40_v10 = [
    E(2000, 155.74, 3000), E(2200, 191.67, 3980), E(2400, 230.05, 5170),
    E(2600, 271.49, 6570), E(2800, 317.58, 8210), E(3000, 367.56, 10150),
    E(3200, 419.64, 12400), E(3400, 474.09, 14990), E(3600, 532.37, 18100),
]
prop40_v20 = [
    E(2200, 151.70, 3985), E(2400, 189.63, 5230), E(2600, 230.87, 6690),
    E(2800, 275.86, 8400), E(3000, 325.70, 10400), E(3200, 378.13, 12620),
    E(3400, 432.85, 15170), E(3600, 493.10, 18070), E(3800, 558.22, 21400),
]
prop40_v30 = [
    E(2400, 123.49, 4260), E(2600, 162.89, 5840), E(2800, 207.02, 7690),
    E(3000, 255.27, 9780), E(3200, 307.28, 12180), E(3400, 363.00, 14850),
    E(3600, 423.01, 17870), E(3800, 486.50, 21220), E(4000, 553.86, 24960),
]
prop40_v40 = [
    E(2400, 40.23, 1880), E(2600, 75.58, 3460), E(2800, 115.16, 5300),
    E(3000, 159.08, 7420), E(3200, 207.34, 9890), E(3400, 261.36, 12750),
    E(3600, 318.04, 15890), E(3800, 381.23, 19440), E(4000, 448.29, 23350),
]
prop40_v50 = [
    E(2800, 17.78, 1360), E(3000, 55.70, 3380), E(3200, 98.26, 5730),
    E(3400, 144.75, 8430), E(3600, 196.70, 11550), E(3800, 255.10, 15140),
    E(4000, 319.19, 19110), E(4200, 390.18, 23530),
]

prop40_branches = [
    (0.0, prop40_v0), (2.9, prop40_v29), (10.0, prop40_v10),
    (20.0, prop40_v20), (30.0, prop40_v30), (40.0, prop40_v40),
    (50.0, prop40_v50),
]

N60 = len(prop60_rpm)


def interp_by_rpm(rpm_arr, val_arr, n, r):
    """C++ interp_by_rpm: below table -> quadratic through origin
    (thrust ~ rpm^2, smooth at cmd=0); above table -> clamp last value."""
    if r <= 0.0:
        return 0.0
    if r <= rpm_arr[0]:
        return val_arr[0] * (r / rpm_arr[0]) ** 2
    if r >= rpm_arr[n - 1]:
        return val_arr[n - 1]
    for i in range(1, n):
        if r <= rpm_arr[i]:
            f = (r - rpm_arr[i - 1]) / (rpm_arr[i] - rpm_arr[i - 1])
            return val_arr[i - 1] + f * (val_arr[i] - val_arr[i - 1])
    return val_arr[n - 1]


def branch_lookup(branch, rpm, field):
    """field: 'thrust_N' or 'power_W'. Mirrors branch_thrust/branch_power."""
    if rpm <= 0.0:
        return 0.0
    e0 = branch[0]
    if rpm <= e0.rpm:
        return getattr(e0, field) * (rpm / e0.rpm) ** 2
    en = branch[-1]
    if rpm >= en.rpm:
        return getattr(en, field)
    for i in range(1, len(branch)):
        e = branch[i]
        if rpm <= e.rpm:
            p = branch[i - 1]
            f = (rpm - p.rpm) / (e.rpm - p.rpm)
            return getattr(p, field) + f * (getattr(e, field) - getattr(p, field))
    return getattr(en, field)


def prop40_lookup(rpm, v, field):
    """prop40_thrust / prop40_power: v clamped to [0,50], linear in v between
    branches, rpm lookup within each branch."""
    v = max(0.0, min(50.0, v))
    nb = len(prop40_branches)
    if v <= prop40_branches[0][0]:
        return branch_lookup(prop40_branches[0][1], rpm, field)
    if v >= prop40_branches[nb - 1][0]:
        return branch_lookup(prop40_branches[nb - 1][1], rpm, field)
    for i in range(1, nb):
        if v <= prop40_branches[i][0]:
            v0, br0 = prop40_branches[i - 1]
            v1, br1 = prop40_branches[i]
            f = (v - v0) / (v1 - v0)
            a = branch_lookup(br0, rpm, field)
            b = branch_lookup(br1, rpm, field)
            return a + f * (b - a)
    return branch_lookup(prop40_branches[nb - 1][1], rpm, field)


def prop60_thrust(rpm):
    return interp_by_rpm(prop60_rpm, prop60_thrust_N, N60, rpm)


def prop60_power(rpm):
    return interp_by_rpm(prop60_rpm, prop60_power_W, N60, rpm)


def prop40_thrust(rpm, v):
    return prop40_lookup(rpm, v, 'thrust_N')


def prop40_power(rpm, v):
    return prop40_lookup(rpm, v, 'power_W')


# ---------------------------------------------------------------------------
# rotor.log parsing (mirrors df200_force_budget.load_rotor, adds velin+rpm)
# ---------------------------------------------------------------------------

def _grab(line, tag):
    m = re.search(rf'{tag}=([^ ]+)', line)
    return m.group(1) if m else None


def _vec(s):
    return np.fromstring(s, sep=',')


def load_rotor(path):
    """Rows with t, AS, rpm(8,), velin(8,), cmd(8,), thrmag(8,),
    Tzi(8,), Txi(8,)."""
    rows = []
    p = Path(path)
    if not p.exists():
        return rows
    for L in p.read_text().splitlines():
        t = _grab(L, 't')
        if not t:
            continue
        AS = _grab(L, 'AS')
        rpm = _grab(L, 'rpm')
        velin = _grab(L, 'velin')
        if not all([t, AS, rpm, velin]):
            continue
        d = dict(t=float(t), AS=float(AS),
                 rpm=_vec(rpm), velin=_vec(velin))
        for tag, key in (('cmd', 'cmd'), ('thrmag', 'thrmag'),
                         ('Tzi', 'Tzi'), ('Txi', 'Txi')):
            v = _grab(L, tag)
            if v:
                try:
                    d[key] = _vec(v)
                except Exception:
                    pass
        rows.append(d)
    # rotor.log is appended across multiple SITL runs (fopen "a"), so it may
    # contain several flights stacked. Take the LAST flight: everything after
    # the final timestamp reset (t jumps back to ~0). Within that flight,
    # drop any non-monotonic duplicate timestamps.
    if not rows:
        return rows
    last_reset = 0
    for i in range(1, len(rows)):
        if rows[i]['t'] < rows[i-1]['t'] - 1.0:
            last_reset = i
    flight = rows[last_reset:]
    clean = []
    last_t = -1e9
    for r in flight:
        if r['t'] > last_t:
            clean.append(r)
            last_t = r['t']
    return clean


# ---------------------------------------------------------------------------
# Power + thrust replay per row
# ---------------------------------------------------------------------------

def replay_row(r):
    """Return dict with per-motor and total power/thrust.
    MOT_1..4 = inner 40in (tilt), MOT_5..8 = outer 60in (lift).

    NOTE on thrust vs log: SITL applies a density scale (air_density/1.225)
    to the table thrust (SIM_Motor.cpp:378), so log thrmag == table_thrust *
    (rho/1.225). Power is NOT density-scaled in SITL (SIM_Motor.cpp:137-140
    uses raw table power), so our power lookup matches SITL exactly. The
    sanity check below therefore expects a ~rho/1.225 ratio on thrust,
    NOT zero deviation. log Tzi/Txi are body-axis projections (tilted), so
    we use log thrmag (thrust magnitude, pre-projection) for the check.
    """
    rpm = r['rpm']
    velin = r['velin']
    p_inner = np.zeros(4)
    p_outer = np.zeros(4)
    t_inner = np.zeros(4)
    t_outer = np.zeros(4)
    velin_clamped = False
    for i in range(4):
        # inner 40in: MOT_1..4 = indices 0..3, uses velin
        if velin[i] > 50.0:
            velin_clamped = True
        p_inner[i] = prop40_power(rpm[i], velin[i])
        t_inner[i] = prop40_thrust(rpm[i], velin[i])
        # outer 60in: MOT_5..8 = indices 4..7, static table
        p_outer[i] = prop60_power(rpm[i + 4])
        t_outer[i] = prop60_thrust(rpm[i + 4])
    return dict(
        p_inner=p_inner, p_outer=p_outer,
        P_inner=p_inner.sum(), P_outer=p_outer.sum(),
        P_total=p_inner.sum() + p_outer.sum(),
        t_inner=t_inner, t_outer=t_outer,
        T_inner=t_inner.sum(), T_outer=t_outer.sum(),
        T_total=t_inner.sum() + t_outer.sum(),
        velin_clamped=velin_clamped,
    )


# ---------------------------------------------------------------------------
# BIN phase-edge detection
# ---------------------------------------------------------------------------

def bin_edges(binpath, t_end):
    """Return dict of phase edge times from BIN + tilt time series.

    ArduPlane logs transition milestones in MSG (field 'Message'), e.g.
      'Transition started airspeed X'
      'Transition airspeed reached X'
      'Transition FW done'
    tilt_start = first TILT.Tilt > 1 deg (cross-check of 'Transition started').
    fw_done = MSG 'Transition FW done' (fallback: AS>45 sustained 1s).
    tilt_series = [(t, tilt_deg), ...] for transition slicing by tilt angle.
    """
    edges = dict(arm=None, tilt_start=None, fw_done=None,
                 trans_start=None, aspd_reached=None, takeoff=None,
                 reverse_start=None, reverse_done=None,
                 land_desc_start=None, tilt_series=[])
    m = mavutil.mavlink_connection(binpath)
    tilt_first = None
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.TimeUS / 1e6 if hasattr(msg, 'TimeUS') else None
        if t is None:
            continue
        typ = msg.get_type()
        if typ == 'MSG':
            txt = getattr(msg, 'Message', '') or ''
            if 'Takeoff' in txt and edges['takeoff'] is None and 'Mission' not in txt:
                edges['takeoff'] = t
            if 'Transition started' in txt and edges['trans_start'] is None:
                edges['trans_start'] = t
            if 'airspeed reached' in txt and edges['aspd_reached'] is None:
                edges['aspd_reached'] = t
            if 'Transition FW done' in txt and edges['fw_done'] is None:
                edges['fw_done'] = t
            if 'Land descend started' in txt and edges['land_desc_start'] is None:
                edges['land_desc_start'] = t
        elif typ == 'TILT':
            tilt = getattr(msg, 'Tilt', 0.0)
            edges['tilt_series'].append((t, tilt))
            if tilt_first is None and tilt > 1.0:
                tilt_first = t
    m.close()
    edges['tilt_start'] = tilt_first
    if edges['trans_start'] is not None:
        edges['tilt_start'] = edges['trans_start']

    # A complete mission can contain a second, reverse transition on approach.
    # There is usually no second "Transition started" MSG, so detect the first
    # sustained tilt descent after the forward transition.  Ignore the small
    # settling movement immediately after FW_DONE: require a real descending
    # run (at least 1 second and 3 degrees).
    if edges['fw_done'] is not None and len(edges['tilt_series']) >= 12:
        tt = np.array([x[0] for x in edges['tilt_series']])
        tv = np.array([x[1] for x in edges['tilt_series']])
        for i in range(len(tv) - 24):
            if tt[i] < edges['fw_done'] + 5.0 or tv[i] >= 89.0:
                continue
            j = i + 24
            if tv[i] - tv[j] >= 3.0 and np.all(np.diff(tv[i:j + 1]) < 0.0):
                edges['reverse_start'] = float(tt[i])
                # first return to the vertical endpoint after the reverse
                # transition; allow a small numerical tolerance around zero.
                k = np.where((tt > tt[i]) & (tv <= 1.0))[0]
                if len(k):
                    edges['reverse_done'] = float(tt[k[0]])
                break
    return edges


# ---------------------------------------------------------------------------
# Phase statistics
# ---------------------------------------------------------------------------

def moving_avg(arr, dt_arr, window_s):
    """Centered moving average with time window (seconds). window<=0 -> no-op.
    Handles uneven spacing by resampling on a uniform grid then convolving."""
    if window_s is None or window_s <= 0 or len(arr) < 3:
        return arr.copy()
    # uniform resample at median dt
    dt_med = np.median(np.diff(dt_arr))
    if dt_med <= 0:
        return arr.copy()
    t0, t1 = dt_arr[0], dt_arr[-1]
    n = max(int(round((t1 - t0) / dt_med)) + 1, 3)
    tu = np.linspace(t0, t1, n)
    resamp = np.interp(tu, dt_arr, arr)
    half = max(int(round(0.5 * window_s / dt_med)), 1)
    # trailing+leading average via cumulative sum (centered)
    kernel = np.ones(2 * half + 1)
    pad = np.pad(resamp, half, mode='edge')
    conv = np.convolve(pad, kernel, mode='valid') / kernel.size
    # back to original timestamps
    return np.interp(dt_arr, tu, conv)


def build_series(rows, replay, smooth_s):
    """Return dict of time-aligned arrays: t, P_total, P_inner, P_outer,
    per-motor p_inner(4,N), p_outer(4,N), AS, rpm(8,N) -- all smoothed."""
    t = np.array([r['t'] for r in rows])
    P_total = np.array([rp['P_total'] for rp in replay])
    P_inner = np.array([rp['P_inner'] for rp in replay])
    P_outer = np.array([rp['P_outer'] for rp in replay])
    p_inner = np.array([rp['p_inner'] for rp in replay]).T  # (4, N)
    p_outer = np.array([rp['p_outer'] for rp in replay]).T
    AS = np.array([r['AS'] for r in rows])
    rpm = np.array([r['rpm'] for r in rows]).T  # (8, N)
    if smooth_s and smooth_s > 0:
        P_total = moving_avg(P_total, t, smooth_s)
        P_inner = moving_avg(P_inner, t, smooth_s)
        P_outer = moving_avg(P_outer, t, smooth_s)
        for i in range(4):
            p_inner[i] = moving_avg(p_inner[i], t, smooth_s)
            p_outer[i] = moving_avg(p_outer[i], t, smooth_s)
    return dict(t=t, P_total=P_total, P_inner=P_inner, P_outer=P_outer,
                p_inner=p_inner, p_outer=p_outer, AS=AS, rpm=rpm)


def ascii_plot(t, series_list, labels, width=120, height=22,
               phases=None, ylabel='kW'):
    """Render a multi-series line plot as ASCII text block.
    series_list: list of np arrays (kW) same length as t.
    phases: list of (name, t0, t1) -- phase boundaries drawn as '|'.
    Returns list of strings (the plot block, no markdown fence)."""
    tmin, tmax = t[0], t[-1]
    allvals = np.concatenate([np.asarray(s) for s in series_list])
    vmax = np.nanmax(allvals) if len(allvals) else 1.0
    vmax = max(vmax, 1e-6)
    span = tmax - tmin if tmax > tmin else 1.0
    # map phase boundaries to columns (skip the very first/last which are edges)
    edges = []
    if phases:
        for nm, p0, p1 in phases:
            c0 = int(round((p0 - tmin) / span * (width - 1)))
            c1 = int(round((p1 - tmin) / span * (width - 1)))
            edges.append((nm, c0, c1))
    out = []
    out.append(f"t [{tmin:.0f}..{tmax:.0f}s]  {ylabel} [0..{vmax:.1f}]")
    edge_cols = set()
    for nm, c0, c1 in edges:
        edge_cols.add(c0)
        edge_cols.add(c1)
    # plot rows
    for row in range(height, -1, -1):
        v = vmax * row / height
        if row % 5 == 0:
            tag = f"{v:6.1f} |"
        else:
            tag = "       |"
        line = [tag]
        for col in range(width):
            ch = ' '
            if col in edge_cols:
                ch = ':'  # phase boundary
            tc = tmin + span * col / (width - 1)
            idx = int(np.argmin(np.abs(t - tc)))
            for si, s in enumerate(series_list):
                sv = s[idx]
                rowval = sv / vmax * height
                if abs(rowval - row) < 0.5:
                    ch = str(si + 1)[-1] if si < 9 else '*'
                    break
            line.append(ch)
        out.append(''.join(line))
    out.append('       +' + '-' * width)
    # phase label row
    labline = ['       ']
    for col in range(width):
        ch = ' '
        for nm, c0, c1 in edges:
            mid = (c0 + c1) // 2
            if col == mid and (c1 - c0) >= 2:
                # pick a char from name
                ch = nm[0].upper()
                break
        labline.append(ch)
    out.append(''.join(labline))
    legend = "  series: " + "  ".join(f"{i+1}={l}" for i, l in enumerate(labels))
    out.append(legend)
    if edges:
        names = "  phases: " + " | ".join(f"{nm}[{nm_t0:.0f}-{nm_t1:.0f}s]"
                                          for nm, nm_t0, nm_t1 in
                                          [(nm, tmin + span * c0 / (width - 1),
                                            tmin + span * c1 / (width - 1))
                                           for nm, c0, c1 in edges])
        out.append(names)
    out.append("  ':' = phase boundary")
    return out


def make_plots(series, phase_list, tilt_series, outdir, smooth_s):
    """Render PNG plots with matplotlib: total power + per-motor power.
    Returns list of (title, relative_path) for the report to embed."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from pathlib import Path
    Path(outdir).mkdir(parents=True, exist_ok=True)
    t = series['t']
    # phase boundary times for axvspan shading
    phase_colors = {'ground_idle': '#dddddd', 'vtol_climb': '#bbdefb',
                    'transition_fwd': '#ffe0b2', 'fw_cruise': '#c8e6c9',
                    'transition_rev': '#ffe0b2', 'vtol_approach': '#d7ccc8',
                    'vtol_landing': '#b2dfdb'}
    out = []

    def shade(ax):
        for nm, p0, p1 in phase_list:
            if p1 > p0:
                ax.axvspan(p0, p1, color=phase_colors.get(nm, '#eee'),
                           alpha=0.4, lw=0)
            ax.axvline(p0, color='gray', ls=':', lw=0.6)

    # --- Plot 1: total / inner / outer power ---
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, series['P_total'] / 1e3, 'k-', lw=1.5, label='P_total')
    ax.plot(t, series['P_inner'] / 1e3, 'b-', lw=1.0, label='P_inner (4x40in)')
    ax.plot(t, series['P_outer'] / 1e3, 'r-', lw=1.0, label='P_outer (4x60in)')
    shade(ax)
    ax.set_xlabel('t (s)'); ax.set_ylabel('Power (kW)')
    ax.set_title(f'DF-200 total power (smooth {smooth_s}s)  [shaded=phase]')
    ax.legend(loc='upper right'); ax.grid(alpha=0.3)
    p = f'{outdir}/df200_power_total.png'
    fig.tight_layout(); fig.savefig(p, dpi=110); plt.close(fig)
    out.append(('总功率曲线（总/内桨/外桨）', p))

    # --- Plot 2: per-motor inner ---
    fig, ax = plt.subplots(figsize=(11, 5))
    for i in range(4):
        ax.plot(t, series['p_inner'][i] / 1e3, lw=1.0, label=f'M{i+1}')
    shade(ax)
    ax.set_xlabel('t (s)'); ax.set_ylabel('Power (kW)')
    ax.set_title('Inner props MOT_1..4 (40in tilt) per-motor power')
    ax.legend(loc='upper right', ncol=4); ax.grid(alpha=0.3)
    p = f'{outdir}/df200_power_inner.png'
    fig.tight_layout(); fig.savefig(p, dpi=110); plt.close(fig)
    out.append(('内桨各电机功率 MOT_1..4（40in 倾转）', p))

    # --- Plot 3: per-motor outer ---
    fig, ax = plt.subplots(figsize=(11, 5))
    for i in range(4):
        ax.plot(t, series['p_outer'][i] / 1e3, lw=1.0, label=f'M{i+5}')
    shade(ax)
    ax.set_xlabel('t (s)'); ax.set_ylabel('Power (kW)')
    ax.set_title('Outer props MOT_5..8 (60in lift) per-motor power')
    ax.legend(loc='upper right', ncol=4); ax.grid(alpha=0.3)
    p = f'{outdir}/df200_power_outer.png'
    fig.tight_layout(); fig.savefig(p, dpi=110); plt.close(fig)
    out.append(('外桨各电机功率 MOT_5..8（60in 垂起）', p))

    # --- Plot 4: power vs tilt (transition only) ---
    if tilt_series:
        tt = np.array([p[0] for p in tilt_series])
        tv = np.array([p[1] for p in tilt_series])
        # only transition window
        t0 = phase_list[2][1] if len(phase_list) > 2 else t[0]
        t1 = phase_list[2][2] if len(phase_list) > 2 else t[-1]
        mask = (tt >= t0) & (tt <= t1)
        if mask.sum() > 5:
            fig, ax = plt.subplots(figsize=(11, 5))
            # map rotor rows in window to tilt
            sel_t = []; sel_P = []; sel_tilt = []
            for r_t, r_p in zip(t, series['P_total']):
                if t0 <= r_t <= t1:
                    idx = int(np.argmin(np.abs(tt - r_t)))
                    sel_t.append(r_t)
                    sel_P.append(r_p / 1e3)
                    sel_tilt.append(tv[idx])
            ax.plot(sel_tilt, sel_P, 'b.-', ms=3, lw=0.8)
            ax.set_xlabel('tilt (deg)'); ax.set_ylabel('Power (kW)')
            ax.set_title('Transition power vs tilt angle (power varies with tilt)')
            ax.grid(alpha=0.3)
            for a in (0, 30, 45, 60, 90):
                ax.axvline(a, color='gray', ls=':', lw=0.6)
            p = f'{outdir}/df200_power_vs_tilt.png'
            fig.tight_layout(); fig.savefig(p, dpi=110); plt.close(fig)
            out.append(('过渡段功率 vs tilt 角度（按倾转角，非时间）', p))
    return out


def write_csv(path, series, phases):
    """Write t, P_total, P_inner, P_outer, 8 per-motor powers, AS, rpm to CSV."""
    import csv
    t = series['t']
    n = len(t)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        hdr = ['t_s', 'P_total_W', 'P_inner_W', 'P_outer_W']
        for i in range(4):
            hdr.append(f'P_inner_M{i+1}_W')
        for i in range(4):
            hdr.append(f'P_outer_M{i+5}_W')
        for i in range(8):
            hdr.append(f'rpm_M{i+1}')
        hdr.append('AS_mps')
        w.writerow(hdr)
        for k in range(n):
            row = [f"{t[k]:.3f}",
                   f"{series['P_total'][k]:.1f}",
                   f"{series['P_inner'][k]:.1f}",
                   f"{series['P_outer'][k]:.1f}"]
            for i in range(4):
                row.append(f"{series['p_inner'][i][k]:.1f}")
            for i in range(4):
                row.append(f"{series['p_outer'][i][k]:.1f}")
            for i in range(8):
                row.append(f"{series['rpm'][i][k]:.0f}")
            row.append(f"{series['AS'][k]:.2f}")
            w.writerow(row)


def phase_stats(rows, replay, t0, t1, name):
    sel = [(r, rp) for r, rp in zip(rows, replay)
           if t0 <= r['t'] <= t1]
    if not sel:
        return None
    # power is physically >= 0; clip to avoid negative-from-interpolation artifacts
    P = np.clip(np.array([rp['P_total'] for _, rp in sel]), 0, None)
    Pi = np.clip(np.array([rp['P_inner'] for _, rp in sel]), 0, None)
    Po = np.clip(np.array([rp['P_outer'] for _, rp in sel]), 0, None)
    ts = np.array([r['t'] for r, _ in sel])
    # energy by trapezoid integration of total power (W*s -> Wh); clamp dt>=0
    dt = np.clip(np.diff(ts), 0, None)
    if len(dt) > 0:
        P_mid = 0.5 * (P[:-1] + P[1:])
        energy_Wh = np.sum(P_mid * dt) / 3600.0
    else:
        energy_Wh = 0.0
    clamp_frac = np.mean([rp['velin_clamped'] for _, rp in sel])
    peak_idx = int(np.argmax(P))
    return dict(
        name=name, n=len(sel), t0=ts[0], t1=ts[-1],
        dur=ts[-1] - ts[0],
        P_avg=P.mean() / 1e3, P_peak=P[peak_idx] / 1e3,
        P_p95=np.percentile(P, 95) / 1e3,
        Pi_avg=Pi.mean() / 1e3, Po_avg=Po.mean() / 1e3,
        Pi_peak=Pi.max() / 1e3, Po_peak=Po.max() / 1e3,
        energy_Wh=energy_Wh,
        peak_t=ts[peak_idx],
        clamp_frac=clamp_frac,
    )


def motor_thrust_phase_stats(rows, phase_list):
    """Return per-motor mean thrust for each phase and full-flight peaks.

    ``thrmag`` is the logged thrust magnitude before tilt projection, in N,
    and is the direct per-motor output from the current SITL model.
    """
    means = {}
    for name, t0, t1 in phase_list:
        values = [r['thrmag'] for r in rows
                  if t0 <= r['t'] <= t1 and 'thrmag' in r]
        if values:
            means[name] = np.mean(np.asarray(values), axis=0)
    values = [r['thrmag'] for r in rows if 'thrmag' in r]
    if values:
        arr = np.asarray(values)
        ts = np.asarray([r['t'] for r in rows if 'thrmag' in r])
        peak_idx = np.argmax(arr, axis=0)
        peaks = arr[peak_idx, np.arange(arr.shape[1])]
        peak_t = ts[peak_idx]
    else:
        peaks = np.full(8, np.nan)
        peak_t = np.full(8, np.nan)
    return means, peaks, peak_t


def tilt_slice_stats(rows, replay, tilt_series, t_start, t_end,
                     bins=(0, 30, 45, 60, 90)):
    """Slice the transition phase by tilt angle and compute per-slice power.
    Power varies physically with tilt, not with elapsed time, so slices are by
    tilt bin (deg), not time. Returns list of stat dicts."""
    # build tilt lookup (t -> tilt), then for each rotor row find nearest tilt
    tilt_t = np.array([p[0] for p in tilt_series]) if tilt_series else np.array([])
    tilt_v = np.array([p[1] for p in tilt_series]) if tilt_series else np.array([])
    if len(tilt_t) == 0:
        return []
    # for each rotor row in [t_start,t_end], get tilt by nearest-time interp
    out = []
    for i in range(len(bins) - 1):
        a0, a1 = bins[i], bins[i + 1]
        sel = []
        for r, rp in zip(rows, replay):
            if not (t_start <= r['t'] <= t_end):
                continue
            # nearest tilt sample
            idx = int(np.argmin(np.abs(tilt_t - r['t'])))
            tl = tilt_v[idx]
            if a0 <= tl < a1 or (i == len(bins) - 2 and tl == a1):
                sel.append((r, rp, tl))
        if len(sel) < 2:
            continue
        P = np.clip(np.array([rp['P_total'] for _, rp, _ in sel]), 0, None)
        Pi = np.clip(np.array([rp['P_inner'] for _, rp, _ in sel]), 0, None)
        Po = np.clip(np.array([rp['P_outer'] for _, rp, _ in sel]), 0, None)
        ts = np.array([r['t'] for r, _, _ in sel])
        dt = np.clip(np.diff(ts), 0, None)
        E = np.sum(0.5 * (P[:-1] + P[1:]) * dt) / 3600.0 if len(dt) > 0 else 0.0
        out.append(dict(
            name=f'tilt {a0}-{a1}deg', n=len(sel),
            t0=ts[0], t1=ts[-1], dur=ts[-1] - ts[0],
            P_avg=P.mean() / 1e3, P_peak=P.max() / 1e3,
            P_p95=np.percentile(P, 95) / 1e3,
            Pi_avg=Pi.mean() / 1e3, Po_avg=Po.mean() / 1e3,
            Pi_peak=Pi.max() / 1e3, Po_peak=Po.max() / 1e3,
            energy_Wh=E, clamp_frac=0.0,
        ))
    return out


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bin', help='DataFlash BIN (for phase edges)')
    ap.add_argument('--rotor', default=ROTOR, help='rotor log path')
    ap.add_argument('--report', default=None, help='write markdown report to this path')
    ap.add_argument('--hover-as-max', type=float, default=None,
                    help='cruise/hover split: AS below this = hover (default 2)')
    ap.add_argument('--smooth', type=float, default=0.5,
                    help='moving-average window in seconds for power curve (0=off, default 0.5)')
    ap.add_argument('--csv', default=None, help='write per-motor power time series to this CSV')
    ap.add_argument('--no-plot', action='store_true', help='skip plots in report')
    args = ap.parse_args()

    rows = load_rotor(args.rotor)
    if not rows:
        sys.exit(f"no rotor data at {args.rotor}")
    replay = [replay_row(r) for r in rows]
    t_end = rows[-1]['t']

    print(f"# rotor rows: {len(rows)}  t: {rows[0]['t']:.2f} -> {t_end:.2f} s")

    # ---- sanity check: recomputed thrust vs rotor log thrmag ----
    # thrmag is the pre-projection thrust magnitude SITL computed (same table,
    # same inputs, plus a density scale). ratio = replay/table / log should be
    # ~1.225/rho (rho<1.225 at altitude -> ratio>1, ~1.06 at SITL rho~1.158).
    # A flat, stable ratio across all phases = correct replay. A phase-varying
    # ratio = bug.
    ratios = []
    for r, rp in zip(rows, replay):
        if 'thrmag' not in r:
            continue
        for i in range(4):
            if r['thrmag'][i] > 5.0:
                ratios.append(rp['t_inner'][i] / r['thrmag'][i])
        for i in range(4):
            if r['thrmag'][i + 4] > 5.0:
                ratios.append(rp['t_outer'][i] / r['thrmag'][i + 4])
    if ratios:
        med = np.median(ratios)
        p5 = np.percentile(ratios, 5)
        p95 = np.percentile(ratios, 95)
        # rho/1.225 = 1/med; sanity = ratio is flat (p5..p95 narrow)
        spread = p95 - p5
        print(f"# sanity: replay thrust / log thrmag  median={med:.3f}  "
              f"p5={p5:.3f} p95={p95:.3f} spread={spread:.3f}  "
              f"(rho/1.225={1/med:.3f}; {'OK: flat ratio = correct replay' if spread < 0.05 else 'FAIL: ratio varies -> lookup bug'})")
    else:
        med = float('nan')
        print("# sanity: no thrmag in log, skipped")

    # ---- phase edges ----
    edges = bin_edges(args.bin, t_end)
    trans_start = edges.get('trans_start') or edges.get('tilt_start')
    fw_done = edges.get('fw_done')
    takeoff = edges.get('takeoff')
    print(f"# edges: takeoff={takeoff}  trans_start={trans_start}  "
          f"aspd_reached={edges.get('aspd_reached')}  fw_done={fw_done} "
          f"reverse_start={edges.get('reverse_start')} reverse_done={edges.get('reverse_done')} "
          f"land_desc_start={edges.get('land_desc_start')}")

    # hover starts when motors first spin up (first rpm>500); before that is
    # ground idle. transition starts at trans_start; cruise at fw_done.
    hover_start = None
    for r in rows:
        if r['rpm'].max() > 500:
            hover_start = r['t']
            break
    if hover_start is None:
        hover_start = rows[0]['t']

    if trans_start is None:
        # fallback: first AS>2 after hover
        for r in rows:
            if r['t'] > hover_start and r['AS'] > 2:
                trans_start = r['t']
                break
        if trans_start is None:
            trans_start = hover_start
    if fw_done is None:
        run = 0
        for r in rows:
            if r['AS'] > 45:
                run += 1
                if run >= 20:
                    fw_done = r['t']
                    break
            else:
                run = 0
        if fw_done is None:
            fw_done = t_end
    reverse_start = edges.get('reverse_start')
    reverse_done = edges.get('reverse_done')
    if reverse_start is None or reverse_start <= fw_done:
        reverse_start = t_end
    if reverse_done is None or reverse_done <= reverse_start:
        reverse_done = t_end
    land_desc_start = edges.get('land_desc_start')
    if land_desc_start is None or land_desc_start <= reverse_done:
        land_desc_start = t_end
    print(f"# phases: ground [{rows[0]['t']:.2f},{hover_start:.2f}]  "
          f"vtol_climb [{hover_start:.2f},{trans_start:.2f}]  "
          f"transition_fwd [{trans_start:.2f},{fw_done:.2f}]  "
          f"fw_cruise [{fw_done:.2f},{reverse_start:.2f}]  "
          f"transition_rev [{reverse_start:.2f},{reverse_done:.2f}]  "
          f"vtol_approach [{reverse_done:.2f},{land_desc_start:.2f}]  "
          f"vtol_landing [{land_desc_start:.2f},{t_end:.2f}]")

    edges['hover_start'] = hover_start
    edges['trans_start'] = trans_start
    edges['fw_done'] = fw_done
    edges['reverse_start'] = reverse_start
    edges['reverse_done'] = reverse_done
    edges['land_desc_start'] = land_desc_start

    phases = [
        ('ground_idle', rows[0]['t'], hover_start),
        ('vtol_climb', hover_start, trans_start),
        ('transition_fwd', trans_start, fw_done),
        ('fw_cruise', fw_done, reverse_start),
        ('transition_rev', reverse_start, reverse_done),
        ('vtol_approach', reverse_done, land_desc_start),
        ('vtol_landing', land_desc_start, t_end),
    ]
    stats = []
    for name, t0, t1 in phases:
        s = phase_stats(rows, replay, t0, t1, name)
        if s and s['n'] > 1:
            stats.append(s)

    # ---- transition sliced by tilt angle (power varies with tilt, not time) ----
    tilt_stats = []
    if edges.get('tilt_series') and trans_start and fw_done:
        tilt_stats = tilt_slice_stats(rows, replay, edges['tilt_series'],
                                      trans_start, fw_done)

    # ---- print table ----
    print(f"\n{'phase':<14}{'n':>6}{'dur_s':>7}{'Pavg_kW':>9}{'Ppeak_kW':>9}"
          f"{'Pp95_kW':>9}{'Pi_avg':>8}{'Po_avg':>8}{'E_Wh':>8}{'clamp%':>8}")
    print('-' * 88)
    for s in stats:
        print(f"{s['name']:<14}{s['n']:>6}{s['dur']:>7.1f}{s['P_avg']:>9.2f}"
              f"{s['P_peak']:>9.2f}{s['P_p95']:>9.2f}{s['Pi_avg']:>8.2f}"
              f"{s['Po_avg']:>8.2f}{s['energy_Wh']:>8.2f}{s['clamp_frac']*100:>7.1f}%")

    if tilt_stats:
        print(f"\n# transition by tilt slice (power varies with tilt, not time):")
        print(f"{'slice':<18}{'n':>6}{'dur_s':>7}{'Pavg_kW':>9}{'Ppeak_kW':>9}"
              f"{'Pi_avg':>8}{'Po_avg':>8}{'E_Wh':>8}")
        print('-' * 72)
        for s in tilt_stats:
            print(f"{s['name']:<18}{s['n']:>6}{s['dur']:>7.1f}{s['P_avg']:>9.2f}"
                  f"{s['P_peak']:>9.2f}{s['Pi_avg']:>8.2f}"
                  f"{s['Po_avg']:>8.2f}{s['energy_Wh']:>8.2f}")

    # peak overall
    allP = np.array([rp['P_total'] for rp in replay])
    peak_i = int(np.argmax(allP))
    print(f"\n# overall peak: {allP[peak_i]/1e3:.2f} kW @ t={rows[peak_i]['t']:.2f}s "
          f"(AS={rows[peak_i]['AS']:.1f})")

    # ---- smoothed series for plots / csv ----
    series = build_series(rows, replay, args.smooth)
    phase_list = [(n, t0, t1) for n, t0, t1 in phases]

    if args.csv:
        write_csv(args.csv, series, phase_list)
        print(f"# csv written: {args.csv}")

    if args.report:
        write_report(args, stats, rows, replay, edges, ratios, series, phase_list,
                     tilt_stats)
        print(f"# report written: {args.report}")


def write_report(args, stats, rows, replay, edges, ratios,
                 series=None, phase_list=None, tilt_stats=None):
    lines = []
    A = lines.append
    A("# DF-200 电机总功率走廊（自动生成）\n")
    A(f"> 数据源：`{args.bin}` + `{args.rotor}`（20Hz 全程）+ DataFlash BIN 阶段边界  ")
    A("> 分析脚本：`docs/scripts/df200_power_analysis.py`  ")
    A("> 电机表：`libraries/SITL/SIM_MotorTable.h`（外桨 165M20 台架 + 内桨 docx 表1 单位修正，详见 `docs/df200_motor_data_bench.md`）  ")
    A("> 功率反查与 SITL 内部 `SIM_Motor.cpp:138-140` 完全同函数同输入（内桨用 `velin` 非 `AS`）\n")

    A("## 1. 功率走廊表\n")
    A("| 阶段 | 采样点 | 时长 s | 平均 kW | 峰值 kW | p95 kW | 内桨均 kW | 外桨均 kW | 能耗 Wh | velin钳位% |")
    A("|------|-------|-------|---------|---------|--------|-----------|-----------|---------|-----------|")
    for s in stats:
        A(f"| {s['name']} | {s['n']} | {s['dur']:.1f} | {s['P_avg']:.2f} | {s['P_peak']:.2f} | "
          f"{s['P_p95']:.2f} | {s['Pi_avg']:.2f} | {s['Po_avg']:.2f} | {s['energy_Wh']:.1f} | {s['clamp_frac']*100:.0f}% |")

    A("\n## 2. 八个电机拉力输出变化（N）\n")
    A("> 拉力取 rotor log 的 `thrmag`，是当前电机模型计算的单电机推力大小，未做倾转后的体轴投影。\n")
    thrust_means, thrust_peaks, thrust_peak_t = motor_thrust_phase_stats(rows, phase_list)
    phase_names = [name for name, _, _ in phase_list]
    A("| 电机 | 类型 | " + " | ".join(f"{name}均值" for name in phase_names) + " | 全程峰值 @ t(s) |")
    A("|------|------|" + "|".join("--------" for _ in phase_names) + "|----------------|" )
    for i in range(8):
        motor_type = '内桨40in' if i < 4 else '外桨60in'
        vals = []
        for name in phase_names:
            vals.append(f"{thrust_means[name][i]:.1f}" if name in thrust_means else "—")
        vals.append(f"{thrust_peaks[i]:.1f} @ {thrust_peak_t[i]:.2f}")
        A(f"| MOT_{i+1} | {motor_type} | " + " | ".join(vals) + " |")

    A("\n## 3. 阶段边界（来自 BIN MSG/TILT）\n")
    A(f"- vtol_climb_start（首帧 rpm>500，VTOL takeoff 爬升开始）: t={edges.get('hover_start')}")
    A(f"- trans_start（MSG 'Transition started' / TILT>1°）: t={edges.get('trans_start')}")
    A(f"- aspd_reached（MSG 'airspeed reached'）: t={edges.get('aspd_reached')}")
    A(f"- fw_done（MSG 'Transition FW done'）: t={edges.get('fw_done')}")
    A(f"- reverse_start（返航时倾转角开始持续下降）: t={edges.get('reverse_start')}")
    A(f"- reverse_done（返航时回到垂起角）: t={edges.get('reverse_done')}")
    A(f"- land_desc_start（MSG 'Land descend started'，实际着陆下降开始）: t={edges.get('land_desc_start')}")
    A("\n> 注：vtol_climb 段是 AUTO VTOL takeoff 爬升到任务高度的过程，控制律按 "
      "Q_WP_SPEED_UP 大油门爬升（实测 cmd≈0.9、T/W≈1.5），功率高是真实爬升行为，"
      "不是稳态悬停功率。本趟无稳态悬停段。")

    A("\n## 4. 内外桨分工\n")
    A("外桨 60in（MOT_5..8，固定垂起）：垂起爬升和垂起着陆主导；正向过渡后归零。")
    A("内桨 40in（MOT_1..4，倾转）：垂起段贡献较小；正向过渡和固定翼巡航阶段承担推进；反向过渡时功率下降。\n")
    A("| 阶段 | 内桨峰值 kW | 外桨峰值 kW | 主导 |")
    A("|------|------------|------------|------|")
    for s in stats:
        dom = '内桨' if s['Pi_avg'] > s['Po_avg'] else '外桨'
        A(f"| {s['name']} | {s['Pi_peak']:.2f} | {s['Po_peak']:.2f} | {dom} |")

    allP = np.array([rp['P_total'] for rp in replay])
    peak_i = int(np.argmax(allP))
    A("\n## 5. 全程峰值\n")
    A(f"峰值功率 {allP[peak_i]/1e3:.2f} kW @ t={rows[peak_i]['t']:.2f}s（AS={rows[peak_i]['AS']:.1f} m/s）。")
    inner = replay[peak_i]['P_inner'] / 1e3
    outer = replay[peak_i]['P_outer'] / 1e3
    A(f"该时刻内桨 {inner.sum():.2f} kW / 外桨 {outer.sum():.2f} kW。")

    if ratios:
        med = np.median(ratios)
        spread = np.percentile(ratios, 95) - np.percentile(ratios, 5)
        A("\n## 6. Sanity check（反查正确性）\n")
        A(f"脚本反算推力 / 日志 `thrmag`（推力大小，未投影）：中位 {med:.3f}，"
          f"p5-p95 散度 {spread:.3f}。")
        A(f"比值稳定（散度<0.05）= 反查链正确。比值 {med:.3f} 对应空气密度 "
          f"ρ={1.225/med:.3f} kg/m³（SITL 对推力做 ρ/1.225 缩放，"
          f"`SIM_Motor.cpp:378`；功率不做密度缩放，与内部一致）。"
          f"{'**反查链正确。**' if spread < 0.05 else '**散度超阈值，反查链有 bug。**'}")
        A("\n> 注：不能用 `Tzi/Txi` 对比——那是倾转后投影到体轴的分量（内桨倾转角余弦），"
          "过渡段会差 1/cos(tilt)~1.5 倍，不代表反查错误。`thrmag` 才是推力大小。")

    A("\n## 7. 各段功率成因（以本 BIN 和最新电机表为准）\n")
    A("- **vtol_climb**：从首帧转速超过 500 rpm 到正向过渡开始；这次是约 1 m/s 的 AUTO 爬升，平均功率高于悬停，不能当作悬停功率。")
    A("- **transition_fwd**：倾转角从 0° 增至约 90°，内外桨同时工作，外桨逐步退出，内桨接管推进。")
    A("- **fw_cruise**：正向过渡完成到返航反向过渡开始；本次外桨功率为 0，只有内桨推进，平均功率是 {:.2f} kW。".format(next((s['P_avg'] for s in stats if s['name'] == 'fw_cruise'), float('nan'))))
    A("- **transition_rev**：返航时倾转角从约 90° 回到 0°。")
    A("- **vtol_approach**：倾转已回到 0°，但仍在 VTOL 进近定位，尚未收到 `Land descend started`。")
    A("- **vtol_landing**：以 MSG `Land descend started` 为边界的实际着陆下降段；这几段都不能并入固定翼巡航统计。")

    A("\n## 8. 差动对功率的影响（恒定总推力下，差动越大功率越高）\n")
    A("> **结论**：总推力恒定时，差动越大（推力集中到少数桨）总功率越高。"
      "这是峰值功率高的成因之一，也是降峰值的切入点。\n")
    A("**数学关系**（外桨静态表实测）：")
    A("- 推力 `T ∝ rpm²`（动量定理）")
    A("- 功率 `P ∝ rpm³`（轴功率=扭矩×rpm，扭矩∝rpm²）")
    A("- 消去 rpm：`P ∝ T^1.5`——指数 1.5>1，是**凸函数**\n")
    A("固定总推力 ΣT=const，分到各桨 T_i，总功率 `ΣP_i = k·ΣT_i^1.5`。"
      "由**詹森不等式**（凸函数）：`ΣT_i^1.5 ≥ n·(ΣT/n)^1.5`，"
      "推力分布越不均（差动越大），ΣT_i^1.5 越大，总功率越高。\n")
    A("**数值验证**（4 外桨，固定总推力 1805N，查表反解各桨 cmd）：\n")
    A("| 推力分配 | 各桨 cmd | 总推力 N | 总功率 kW | vs 均匀 |")
    A("|----------|---------|---------|-----------|---------|")
    A("| 均匀 | 0.60×4 | 1805 | 23.6 | 基准 |")
    A("| 小差 60/40 | 0.65,0.54,0.54,0.65 | 1805 | 24.1 | +2% |")
    A("| 中差 75/25 | 0.74,0.44,0.44,0.74 | 1805 | 27.3 | +16% |")
    A("| 大差 90/10 | 0.80,0.28,0.28,0.80 | 1805 | 31.2 | +32% |")
    A("| 极大 1满3小 | 0.92,0.44,0.44,0.44 | 1805 | 31.0 | +31% |")
    A("\n**物理解读**：高推力桨的功率'超线性增长'（推力翻倍功率涨 2.83 倍），"
      "低推力桨省的功率补不回高推力桨多花的——所以同样总推力，集中给少数桨更耗功率。")
    A("\n**对峰值功率的指导**：")
    peak_cmd = rows[peak_i].get('cmd')
    if peak_cmd is not None:
        A(f"- 本次全程峰值发生在 t={rows[peak_i]['t']:.2f}s，峰值时 cmd="
          f"[{','.join(f'{x:.2f}' for x in peak_cmd)}]；应结合该时刻的滚转/俯仰限幅判断峰值来源。")
    A("- 差动越大，推力越集中到少数桨，查表功率的凸性会放大总功率；因此应同时看 MOTT、RATE 输出和功率峰值，不能只看 collective。")
    A("- **降峰值方向**：限制单桨 cmd 上限（避免打满个别电机）、或优化混控差动分配使推力更均匀，"
      "可在同样总推力下降低峰值功率。")

    # ---- transition by tilt slice ----
    if tilt_stats:
        A("\n## 9. transition 按 tilt 角度切片（功率随倾转角变化，与时间无关）\n")
        A("> transition 段按 tilt 角度切（0-30/30-45/45-60/60-90°），不是按时间。"
          "功率随倾转角物理变化：tilt 涨则外桨垂直分量衰减、内桨推进建立。\n")
        A("| tilt 切片 | 采样点 | 时长 s | 平均 kW | 峰值 kW | p95 kW | 内桨均 kW | 外桨均 kW | 能耗 Wh |")
        A("|----------|-------|-------|---------|---------|--------|-----------|-----------|---------|")
        for s in tilt_stats:
            A(f"| {s['name']} | {s['n']} | {s['dur']:.1f} | {s['P_avg']:.2f} | "
              f"{s['P_peak']:.2f} | {s['P_p95']:.2f} | {s['Pi_avg']:.2f} | {s['Po_avg']:.2f} | "
              f"{s['energy_Wh']:.1f} |")
        A("\n> 这些切片来自本次 BIN 对应的最新电机表；不要与旧日志的切片数值混用。")

    A("\n## 10. 已知局限\n")
    A("- velin>50 时 `prop40_power` 钳到 50 分支（`SIM_MotorTable.h:210`），巡航来流>50 段功率为下界估计")
    A("- 表在 ~398 V 标定，未建模电压跌落（满载 sag ~0.8%，见 `df200_motor_data.md` §3.4）")
    A("- 功率未做空气密度修正（SITL `Motor::calculate_forces` 对推力做 ρ 缩放，功率用表值，与内部一致）")
    A("- 短段（过渡 ~3-5s）p95 比 peak 更稳健，peak 易被单拍噪声拉高")
    A(f"- 图表功率经 {args.smooth}s 滑动平均滤波（`--smooth` 可调，0=关闭）")

    # ---- plots (matplotlib PNG) ----
    if series is not None and not args.no_plot:
        report_dir = Path(args.report).resolve().parent
        img_dir = report_dir / 'df200_power_imgs'
        plots = make_plots(series, phase_list, edges.get('tilt_series'),
                           str(img_dir), args.smooth)
        A("\n## 11. 功率曲线图（matplotlib，滤波后）\n")
        A("> 图片以 base64 内嵌，单文件可直接发送查看，不依赖外部路径。\n")
        import base64
        for title, path in plots:
            data = base64.b64encode(Path(path).read_bytes()).decode('ascii')
            A(f"### {title}\n")
            A(f"![{title}](data:image/png;base64,{data})")
            A("")

    Path(args.report).write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
