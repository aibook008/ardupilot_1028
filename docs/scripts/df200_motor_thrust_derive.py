#!/usr/bin/env python3
"""DF-200 SITL motor thrust parameter derivation tool.

Derives the SITL json motor parameters (refCurrent, velocity_max, etc.) from
real motor test data (docs/电机电池.docx + docs/数据库_v2.xlsx 动力参数).

SITL calc_thrust formula (SIM_Motor.cpp:321):
    Vout = voltage_scale * velocity_max * sqrt((1-expo)*cmd + expo*cmd^2)
    T   = 0.5 * rho * A_eff * (Vout^2 - Vin^2)

SITL derivation (SIM_Frame.cpp:531-536):
    hover_thrust     = mass * g                    (total, N)
    hover_power      = refCurrent * refVoltage     (total, W)
    hover_velocity_out = 2 * hover_power / hover_thrust
    velocity_max     = hover_velocity_out / sqrt(hoverThrOut)
    effective_disc_area = hover_thrust / (0.5 * rho * hover_velocity_out^2)
    effective_prop_area = effective_disc_area / num_motors

Key check: Vout at hover cmd must be > cruise airspeed, else thrust goes
negative at speed (Vout^2 - Vin^2 < 0).

Usage:
    python3 docs/scripts/df200_motor_thrust_derive.py
    python3 docs/scripts/df200_motor_thrust_derive.py --refcurrent 258
    python3 docs/scripts/df200_motor_thrust_derive.py --target-vout 60
"""
from __future__ import annotations
import argparse, math, json, re
from pathlib import Path

# --- constants ---
G = 9.81
RHO = 1.225  # sea level air density kg/m3
JSON_PATH = "Tools/autotest/models/df200.json"


def read_json():
    """Read df200.json (with comments) and return dict."""
    t = Path(JSON_PATH).read_text()
    t = re.sub(r'#.*', '', t)
    t = re.sub(r',\s*}', '}', t)
    t = re.sub(r',\s*]', ']', t)
    return json.loads(t)


def sitl_derive(mass, ref_current, ref_voltage, hover_thr_out, rho=RHO):
    """Replicate SIM_Frame.cpp:531-536 derivation."""
    hover_thrust = mass * G
    hover_power = ref_current * ref_voltage
    hover_vout = 2 * hover_power / hover_thrust
    vel_max = hover_vout / math.sqrt(hover_thr_out)
    eff_disc = hover_thrust / (0.5 * rho * hover_vout**2)
    return dict(hover_thrust=hover_thrust, hover_power=hover_power,
                hover_vout=hover_vout, vel_max=vel_max, eff_disc=eff_disc)


def calc_thrust(cmd, vin, vel_max, expo, rho, eff_prop_area, voltage_scale=1.0):
    """Replicate SIM_Motor.cpp:321 calc_thrust."""
    vout = voltage_scale * vel_max * math.sqrt((1 - expo) * cmd + expo * cmd**2)
    t = 0.5 * rho * eff_prop_area * (vout**2 - vin**2)
    return vout, t


def reverse_refcurrent(target_vout_hover, mass, ref_voltage, hover_thr_out):
    """Find refCurrent that gives desired Vout at hover cmd."""
    # Vout(hover) = vel_max * sqrt(hover_thr_out)
    # vel_max = hover_vout / sqrt(hover_thr_out)
    # hover_vout = 2 * hover_power / hover_thrust
    # hover_power = ref_current * ref_voltage
    hover_thrust = mass * G
    needed_hover_vout = target_vout_hover / math.sqrt(hover_thr_out)
    needed_power = needed_hover_vout * hover_thrust / 2
    ref_current = needed_power / ref_voltage
    return ref_current, needed_power


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--refcurrent', type=float, default=None,
                    help='override refCurrent to test')
    ap.add_argument('--target-vout', type=float, default=60,
                    help='target Vout at hover cmd (must > cruise AS)')
    ap.add_argument('--cruise-as', type=float, default=50,
                    help='cruise airspeed for thrust-sign check')
    args = ap.parse_args()

    j = read_json()
    mass = j['mass'] * 1.5  # QuadPlane mass *= 1.5
    ref_current = args.refcurrent if args.refcurrent else j['refCurrent']
    ref_voltage = j['refVoltage']
    hover_thr = j['hoverThrOut']
    expo = j['propExpo']
    disc_area = j['disc_area']
    num_motors = j['num_motors']

    print(f"=== df200.json current values ===")
    print(f"  mass(json)={j['mass']}, flight mass={mass:.1f} kg")
    print(f"  refCurrent={j['refCurrent']}, refVoltage={ref_voltage}")
    print(f"  hoverThrOut={hover_thr}, propExpo={expo}")
    print(f"  disc_area={disc_area}, num_motors={num_motors}")

    # current derivation
    d = sitl_derive(mass, ref_current, ref_voltage, hover_thr)
    eff_prop = d['eff_disc'] / num_motors

    print(f"\n=== SITL derivation (refCurrent={ref_current}) ===")
    print(f"  hover_thrust = {mass:.1f}×{G} = {d['hover_thrust']:.0f} N")
    print(f"  hover_power = {ref_current}×{ref_voltage} = {d['hover_power']:.0f} W")
    print(f"  hover_velocity_out = 2×{d['hover_power']:.0f}/{d['hover_thrust']:.0f} = {d['hover_vout']:.1f} m/s")
    print(f"  velocity_max = {d['hover_vout']:.1f}/sqrt({hover_thr}) = {d['vel_max']:.1f} m/s")
    print(f"  effective_disc_area = {d['eff_disc']:.3f} m² (json {disc_area})")
    print(f"  effective_prop_area = {eff_prop:.4f} m²/motor")

    # thrust at various conditions
    print(f"\n=== Thrust at various cmd/Vin (refCurrent={ref_current}) ===")
    print(f"  {'cmd':>5} {'Vin':>5} {'Vout':>6} {'T/motor':>8} {'T_total':>8} {'sign':>5}")
    for cmd in [hover_thr, 0.15, 0.5, 1.0]:
        for vin in [0, args.cruise_as]:
            vout, t = calc_thrust(cmd, vin, d['vel_max'], expo, RHO, eff_prop)
            sign = '✓' if t > 0 else '✗ NEG'
            print(f"  {cmd:5.2f} {vin:5.0f} {vout:6.1f} {t:8.1f} {t*num_motors:8.0f} {sign:>5}")

    # check: Vout at hover vs cruise AS
    vout_hover = d['vel_max'] * math.sqrt(hover_thr)
    print(f"\n=== Critical check ===")
    print(f"  Vout(hover cmd={hover_thr}) = {vout_hover:.1f} m/s")
    print(f"  Cruise AS = {args.cruise_as} m/s")
    if vout_hover > args.cruise_as:
        print(f"  Vout > AS → thrust positive at cruise ✓")
    else:
        print(f"  Vout < AS → thrust NEGATIVE at cruise ✗  ← NEEDS FIX")
        rc, pw = reverse_refcurrent(args.target_vout, mass, ref_voltage, hover_thr)
        print(f"\n  To get Vout(hover)={args.target_vout:.0f} > {args.cruise_as}:")
        print(f"    refCurrent = {rc:.0f} (current {ref_current:.0f})")
        print(f"    hover_power = {pw:.0f} W")
        # verify
        d2 = sitl_derive(mass, rc, ref_voltage, hover_thr)
        vout2 = d2['vel_max'] * math.sqrt(hover_thr)
        print(f"    → velocity_max = {d2['vel_max']:.1f}, Vout(hover) = {vout2:.1f}")
        eff2 = d2['eff_disc'] / num_motors
        v, t = calc_thrust(hover_thr, args.cruise_as, d2['vel_max'], expo, RHO, eff2)
        print(f"    → T at AS={args.cruise_as}: {t:.1f} N/motor ({'positive ✓' if t>0 else 'negative'})")

    # amplification: dT per 1 m/s Vin difference
    v1, t1 = calc_thrust(hover_thr, args.cruise_as, d['vel_max'], expo, RHO, eff_prop)
    v2, t2 = calc_thrust(hover_thr, args.cruise_as + 1, d['vel_max'], expo, RHO, eff_prop)
    print(f"\n=== Vin difference amplification ===")
    print(f"  dT per 1 m/s Vin diff = {t1 - t2:.1f} N (at AS={args.cruise_as}, cmd={hover_thr})")
    print(f"  (lower is better - reduces position%gyro feedback gain)")


if __name__ == '__main__':
    main()
