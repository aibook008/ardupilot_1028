#!/usr/bin/env python3
"""Verify motor allocation matrix correspondence: Lua ↔ JSON ↔ SIM_Frame.

For DF-200 / JT-20 tiltrotors the mixer factors and SITL plant must agree:

  Lua (control mixer)          JSON (SITL plant)           SIM_Frame.cpp
  -------------------------    ------------------------    -------------------
  roll  = cos(angle+90)   ←→   atan2(Y,X) from position ←→ Motor(..., angle, ...)
  pitch = cos(angle)      ←→   same angle               ←→ same
  yaw   = ±1 (sign)       ←→   sign(motorN_yaw)         ←→ YAW_FACTOR_CW/CCW
                               (magnitude scales SITL
                                reaction torque only)

AP_MotorsMatrix convention (add_motor):
  roll_fac  = cos(radians(angle_deg + 90)) = -sin(radians(angle_deg))
  pitch_fac = cos(radians(angle_deg))
  yaw_fac   = prop direction (±1), NOT geometry

Usage:
  python3 docs/scripts/verify_motor_matrix.py
  python3 docs/scripts/verify_motor_matrix.py --vehicle df200
  python3 docs/scripts/verify_motor_matrix.py --vehicle jt20 --tol 1e-3
"""
from __future__ import annotations

import argparse
import ast
import json
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def load_json_with_comments(path: Path) -> dict:
    """SITL model files are JSONC (// and # comments allowed)."""
    text = path.read_text(encoding="utf-8")
    lines = []
    for line in text.splitlines():
        # strip // comments and # comments outside strings (good enough here)
        cut = len(line)
        in_str = False
        i = 0
        while i < len(line):
            c = line[i]
            if c == '"' and (i == 0 or line[i - 1] != "\\"):
                in_str = not in_str
            elif not in_str and c in ("#", "/"):
                if c == "#" or (c == "/" and i + 1 < len(line) and line[i + 1] == "/"):
                    cut = i
                    break
            i += 1
        lines.append(line[:cut])
    cleaned = "\n".join(lines)
    # trailing commas before } or ]
    cleaned = re.sub(r",\s*([}\]])", r"\1", cleaned)
    return json.loads(cleaned)


def parse_df200_lua(path: Path) -> list[dict]:
    """Parse BASE = { {roll,pitch,yaw,order}, ... } from df200_motor_matrix.lua."""
    text = path.read_text(encoding="utf-8")
    m = re.search(r"local\s+BASE\s*=\s*\{(.*?)\n\}", text, re.S)
    if not m:
        raise ValueError(f"BASE table not found in {path}")
    body = m.group(1)
    rows = []
    for line in body.splitlines():
        line = line.split("--", 1)[0].strip().rstrip(",")
        if not line.startswith("{"):
            continue
        # Lua table {...} → Python list; ast would otherwise make a set.
        vals = ast.literal_eval("[" + line[1:-1] + "]")
        rows.append(
            {
                "roll": float(vals[0]),
                "pitch": float(vals[1]),
                "yaw": float(vals[2]),
                "order": int(vals[3]),
            }
        )
    if len(rows) != 8:
        raise ValueError(f"expected 8 BASE rows, got {len(rows)} in {path}")
    return rows


def parse_jt20_lua(path: Path) -> list[dict]:
    """Parse M[i] = { roll=..., pitch=..., yaw=..., order=... } from jt20 lua."""
    text = path.read_text(encoding="utf-8")
    rows = [None] * 8
    for m in re.finditer(
        r"M\[(\d+)\]\s*=\s*\{\s*roll\s*=\s*([^,]+),\s*pitch\s*=\s*([^,]+),\s*yaw\s*=\s*([^,]+),\s*order\s*=\s*(\d+)",
        text,
    ):
        i = int(m.group(1))
        rows[i] = {
            "roll": float(m.group(2)),
            "pitch": float(m.group(3)),
            "yaw": float(m.group(4)),
            "order": int(m.group(5)),
        }
    if any(r is None for r in rows):
        raise ValueError(f"failed to parse all M[0..7] in {path}")
    return rows  # type: ignore[return-value]


def parse_sim_frame_motors(path: Path, array_name: str, expect_n: int | None = None) -> list[dict]:
    """Parse static Motor name[] = { Motor(...), ... } from SIM_Frame.cpp."""
    text = path.read_text(encoding="utf-8")
    m = re.search(
        rf"static\s+Motor\s+{re.escape(array_name)}\s*\[\s*\]\s*=\s*\{{(.*?)\n\}};",
        text,
        re.S,
    )
    if not m:
        raise ValueError(f"{array_name}[] not found in {path}")
    body = m.group(1)
    motors = []
    # Motor(AP_MOTORS_MOT_N, angle, YAW_FACTOR_XXX|0|float, order [, tilt args...])
    for mm in re.finditer(
        r"Motor\(\s*AP_MOTORS_MOT_(\d+)\s*,\s*([-\d.]+)f?\s*,\s*"
        r"(AP_MOTORS_MATRIX_YAW_FACTOR_(CW|CCW)|[-\d.]+f?)\s*,\s*(\d+)",
        body,
    ):
        yaw_tok = mm.group(3)
        if yaw_tok.endswith("CCW"):
            yaw = 1.0
        elif yaw_tok.endswith("CW"):
            yaw = -1.0
        else:
            yaw = float(yaw_tok.rstrip("f"))
        motors.append(
            {
                "mot": int(mm.group(1)),
                "angle_deg": float(mm.group(2)),
                "yaw": yaw,  # CW=-1, CCW=+1
                "order": int(mm.group(5)),
            }
        )
    if expect_n is not None and len(motors) != expect_n:
        raise ValueError(f"expected {expect_n} motors in {array_name}, got {len(motors)}")
    return motors


def parse_setup_quad_matrix(path: Path) -> dict[str, list[dict]]:
    """Parse AP_MotorsMatrix::setup_quad_matrix MotorDef tables.

    Returns map frame_type_string -> list of {angle_deg, yaw, order} in MOT_1..N order.
    """
    text = path.read_text(encoding="utf-8")
    m = re.search(
        r"bool\s+AP_MotorsMatrix::setup_quad_matrix\s*\(motor_frame_type frame_type\)\s*\{(.*?)\n\}",
        text,
        re.S,
    )
    if not m:
        raise ValueError("setup_quad_matrix not found")
    body = m.group(1)
    frames: dict[str, list[dict]] = {}
    # each case MOTOR_FRAME_TYPE_XXX: { ... MotorDef motors[] { {ang, yaw, order}, ...}; add_motors...}
    for cm in re.finditer(
        r"case\s+MOTOR_FRAME_TYPE_([A-Z0-9_]+)\s*:\s*\{(.*?)\n\s*\}",
        body,
        re.S,
    ):
        ftype = cm.group(1)
        case_body = cm.group(2)
        # skip VTAIL / ATAIL style that uses add_motor() calls with separate roll/pitch
        if "add_motor(" in case_body and "MotorDef" not in case_body:
            continue
        motors = []
        for rm in re.finditer(
            r"\{\s*([-\d.]+)f?\s*,\s*(AP_MOTORS_MATRIX_YAW_FACTOR_(CW|CCW)|[-\d.]+f?)\s*,\s*(\d+)\s*\}",
            case_body,
        ):
            yaw_tok = rm.group(2)
            if yaw_tok.endswith("CCW"):
                yaw = 1.0
            elif yaw_tok.endswith("CW"):
                yaw = -1.0
            else:
                yaw = float(yaw_tok.rstrip("f"))
            motors.append(
                {
                    "angle_deg": float(rm.group(1)),
                    "yaw": yaw,
                    "order": int(rm.group(4)),
                }
            )
        if motors:
            frames[ftype] = motors
    return frames


def factors_from_angle(angle_deg: float) -> tuple[float, float]:
    """AP_MotorsMatrix::add_motor roll/pitch from angle."""
    roll = math.cos(math.radians(angle_deg + 90.0))
    pitch = math.cos(math.radians(angle_deg))
    return roll, pitch


def angle_from_xy(x: float, y: float) -> float:
    return math.degrees(math.atan2(y, x))


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def sgn(v: float) -> int:
    if v > 0:
        return 1
    if v < 0:
        return -1
    return 0


VEHICLES = {
    "df200": {
        "json": REPO / "Tools/autotest/models/df200.json",
        "lua": REPO / "scripts/df200_motor_matrix.lua",
        "lua_parser": parse_df200_lua,
        "sim_array": "df200_motors",
        "labels": [
            "M1 FR tilt ①",
            "M2 RL tilt ⑥",
            "M3 FL tilt ⑤",
            "M4 RR tilt ②",
            "M5 FR vert ③",
            "M6 RL vert ⑧",
            "M7 FL vert ⑦",
            "M8 RR vert ④",
        ],
        # Mixer factors should match JSON geometry (no CG shift in df200.json).
        "lua_vs_json_geometry": True,
        # JT20 CG-shifted JSON: Lua was built from SIM_Frame/SDF angles, not JSON atan2.
        "lua_vs_sim_angle": True,
        # Historical "yaw flip" comments mean vs physical prop naming; currently
        # Lua sign == JSON motorN_yaw sign == SIM_Frame CW/CCW macro value.
        "expect_sum_pitch_zero": True,
    },
    "jt20": {
        "json": REPO / "Tools/autotest/models/jt20.json",
        "lua": REPO / "scripts/jt20_motor_matrix.lua.off",
        "lua_parser": parse_jt20_lua,
        "sim_array": "jt20_motors",
        "labels": [
            "M1 FR tilt",
            "M2 RL tilt",
            "M3 FL tilt",
            "M4 RR tilt",
            "M5 FR vert",
            "M6 RL vert",
            "M7 FL vert",
            "M8 RR vert",
        ],
        # JSON positions are CG-shifted (+0.104 x); atan2(JSON) ≠ SIM/Lua angles.
        "lua_vs_json_geometry": False,
        "lua_vs_sim_angle": True,
        # Front/rear SDF angles are asymmetric → Σ pitch ≠ 0 is expected.
        "expect_sum_pitch_zero": False,
    },
}


def verify(vehicle: str, tol: float, angle_tol_deg: float) -> int:
    cfg = VEHICLES[vehicle]
    model = load_json_with_comments(cfg["json"])
    lua = cfg["lua_parser"](cfg["lua"])
    sim = parse_sim_frame_motors(
        REPO / "libraries/SITL/SIM_Frame.cpp", cfg["sim_array"], expect_n=8
    )
    # normalize key used below
    for S in sim:
        S["yaw_sitl_nominal"] = S["yaw"]

    print(f"=== {vehicle.upper()} motor matrix correspondence ===")
    print(f"JSON : {cfg['json'].relative_to(REPO)}")
    print(f"Lua  : {cfg['lua'].relative_to(REPO)}")
    print(f"SIM  : libraries/SITL/SIM_Frame.cpp::{cfg['sim_array']}[]")
    print()
    print(
        f"{'M':<3} {'label':<14} {'J°':>7} {'SIM°':>7} {'ref°':>7} "
        f"{'R_lua':>8} {'R_ref':>8} {'P_lua':>8} {'P_ref':>8} "
        f"{'Y_lua':>6} {'Y_json':>7} {'Y_sim':>6}  notes"
    )
    print("-" * 118)

    fails: list[str] = []
    for i in range(8):
        n = i + 1
        pos = model[f"motor{n}_position"]
        yaw_json = float(model[f"motor{n}_yaw"])
        x, y = float(pos[0]), float(pos[1])
        ang_json = angle_from_xy(x, y)

        L = lua[i]
        S = sim[i]
        label = cfg["labels"][i]

        # Reference angle for Lua roll/pitch check:
        # - DF200: JSON position (no CG shift) ≈ SIM_Frame
        # - JT20: SIM_Frame/SDF angles (JSON is CG-shifted; atan2 differs)
        if cfg["lua_vs_json_geometry"]:
            ang_ref = ang_json
            ref_src = "JSON"
        else:
            ang_ref = S["angle_deg"]
            ref_src = "SIM"
        r_ref, p_ref = factors_from_angle(ang_ref)

        d_ang_json_sim = wrap180(ang_json - S["angle_deg"])
        d_roll = L["roll"] - r_ref
        d_pitch = L["pitch"] - p_ref

        notes = []

        # 1) JSON position angle vs SIM_Frame (informational for JT20 CG shift)
        if abs(d_ang_json_sim) > angle_tol_deg:
            if cfg["lua_vs_json_geometry"]:
                notes.append(f"J≠SIM({d_ang_json_sim:+.2f}°)")
                fails.append(
                    f"M{n}: JSON atan2 {ang_json:.2f} vs SIM_Frame {S['angle_deg']:.2f}"
                )
            else:
                notes.append(f"J≠SIM({d_ang_json_sim:+.2f}° CG-shift OK)")

        # 2) Lua roll/pitch vs reference geometry
        if abs(d_roll) > tol:
            notes.append(f"RΔ={d_roll:+.4f}")
            fails.append(f"M{n}: Lua roll {L['roll']} vs {ref_src} geo {r_ref:.4f}")
        if abs(d_pitch) > tol:
            notes.append(f"PΔ={d_pitch:+.4f}")
            fails.append(f"M{n}: Lua pitch {L['pitch']} vs {ref_src} geo {p_ref:.4f}")

        # 3) Lua yaw sign == JSON motorN_yaw sign (plant reaction torque sign)
        if sgn(L["yaw"]) != sgn(yaw_json):
            notes.append("Y_lua≠Y_json")
            fails.append(
                f"M{n}: Lua yaw sign {sgn(L['yaw'])} vs JSON motor{n}_yaw "
                f"sign {sgn(yaw_json)} (val={yaw_json})"
            )

        # 4) Lua yaw sign == SIM_Frame CW/CCW macro value (CW=-1, CCW=+1)
        if sgn(L["yaw"]) != sgn(S["yaw_sitl_nominal"]):
            notes.append("Y_lua≠Y_sim")
            fails.append(
                f"M{n}: Lua yaw {sgn(L['yaw'])} vs SIM_Frame "
                f"{sgn(S['yaw_sitl_nominal'])}"
            )

        # 5) JSON yaw magnitude note (DF200 |9| = SITL torque scale, not mixer ±1)
        if abs(abs(yaw_json) - 1.0) > 1e-6:
            notes.append(f"|Yj|={abs(yaw_json):g} scale")

        # 6) motor index vs MOT_N in SIM_Frame
        if S["mot"] != n:
            notes.append(f"SIM_MOT={S['mot']}")
            fails.append(f"M{n}: SIM_Frame MOT index {S['mot']}")

        ok = "OK" if not any(
            t for t in notes if "CG-shift OK" not in t and "scale" not in t
        ) else "; ".join(notes)
        # keep informational notes visible even on otherwise-OK rows
        if ok == "OK" and notes:
            ok = "OK (" + "; ".join(notes) + ")"

        print(
            f"M{n:<2} {label:<14} {ang_json:7.2f} {S['angle_deg']:7.2f} {ang_ref:7.2f} "
            f"{L['roll']:8.4f} {r_ref:8.4f} {L['pitch']:8.4f} {p_ref:8.4f} "
            f"{L['yaw']:6.0f} {yaw_json:7.1f} {S['yaw_sitl_nominal']:6.0f}  {ok}"
        )

    # Allocation matrix printout (Lua BASE, hover / scale=1)
    print()
    print("Allocation matrix A (Lua BASE, columns = roll/pitch/yaw/throttle factors):")
    print(f"{'':>4} {'roll':>10} {'pitch':>10} {'yaw':>10} {'thr':>10}")
    for i, L in enumerate(lua):
        print(f"M{i+1:<3} {L['roll']:10.4f} {L['pitch']:10.4f} {L['yaw']:10.1f} {1.0:10.1f}")

    # Authority sanity
    print()
    print("Authority sanity (sum of factors; hover collective cancellation):")
    sr = sum(L["roll"] for L in lua)
    sp = sum(L["pitch"] for L in lua)
    sy = sum(L["yaw"] for L in lua)
    print(f"  Σ roll_fac  = {sr:+.4f}  (want ~0)")
    want_p = "want ~0" if cfg["expect_sum_pitch_zero"] else "JT20 asymmetric OK"
    print(f"  Σ pitch_fac = {sp:+.4f}  ({want_p})")
    print(f"  Σ yaw_fac   = {sy:+.1f}  (want ~0)")
    if abs(sr) > 0.05:
        fails.append(f"Σ roll not near zero: {sr}")
    if cfg["expect_sum_pitch_zero"] and abs(sp) > 0.05:
        fails.append(f"Σ pitch not near zero: {sp}")
    if abs(sy) > 0.05:
        fails.append(f"Σ yaw not near zero: {sy}")

    # Effective SITL yaw torque scale vs mixer
    print()
    print("Correspondence rules used:")
    print("  roll/pitch: Lua ↔ cos(angle±) from", "JSON pos" if cfg["lua_vs_json_geometry"] else "SIM_Frame angle")
    print("  yaw sign:   Lua == sign(JSON motorN_yaw) == SIM_Frame CW/CCW macro")
    print("  SITL |motorN_yaw| only scales reaction torque magnitude (not mixer).")
    scales = [abs(float(model[f"motor{i}_yaw"])) for i in range(1, 9)]
    print(f"  JSON |motorN_yaw| = {scales}")
    if vehicle == "df200" and any(abs(s - 9.0) < 1e-6 for s in scales):
        print("  DF200: |yaw|=9 → SITL reaction Mz ≈ 9× a ±1 plant.")
    if vehicle == "jt20":
        print("  JT20: JSON x shifted +0.104 for CG; mixer angles stay on SIM_Frame/SDF.")

    print()
    if fails:
        print(f"FAIL ({len(fails)}):")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("PASS: Lua ↔ JSON ↔ SIM_Frame correspondence OK.")
    return 0


# Official Quad: SITL frame name ↔ setup_quad_matrix case
# FRAME_CLASS=1 (QUAD) + FRAME_TYPE selects the mixer table.
OFFICIAL_QUADS = [
    # sitl_frame_name, sim_array, setup_quad case name, FRAME_TYPE enum value
    # enums from libraries/AP_Motors/AP_Motors_Class.h
    ("+", "quad_plus_motors", "PLUS", 0),          # MOTOR_FRAME_TYPE_PLUS
    ("x", "quad_x_motors", "X", 1),                # MOTOR_FRAME_TYPE_X
    ("bfx", "quad_bf_x_motors", "BF_X", 12),       # MOTOR_FRAME_TYPE_BF_X
    ("djix", "quad_dji_x_motors", "DJI_X", 13),    # MOTOR_FRAME_TYPE_DJI_X
    ("cwx", "quad_cw_x_motors", "CW_X", 14),       # MOTOR_FRAME_TYPE_CW_X
    ("bfxrev", "quad_bf_x_rev_motors", "BF_X_REV", 18),  # MOTOR_FRAME_TYPE_BF_X_REV
]


def verify_official_quads(tol: float = 1e-6) -> int:
    """Compare SITL SIM_Frame motor tables vs AP_MotorsMatrix::setup_quad_matrix."""
    sim_path = REPO / "libraries/SITL/SIM_Frame.cpp"
    mtx_path = REPO / "libraries/AP_Motors/AP_MotorsMatrix.cpp"
    setup = parse_setup_quad_matrix(mtx_path)

    print("=== Official QUAD: SITL SIM_Frame ↔ setup_quad_matrix ===")
    print(f"SITL : {sim_path.relative_to(REPO)}")
    print(f"Mixer: {mtx_path.relative_to(REPO)}::setup_quad_matrix")
    print()
    print(
        f"{'SITL':<8} {'FRAME_TYPE':<10} {'M':>2} "
        f"{'ang_sim':>8} {'ang_mix':>8} {'yaw_sim':>8} {'yaw_mix':>8} "
        f"{'R':>8} {'P':>8} {'ord_s':>5} {'ord_m':>5}  notes"
    )
    print("-" * 110)

    fails: list[str] = []
    for sitl_name, sim_array, case_name, ftype_val in OFFICIAL_QUADS:
        if case_name not in setup:
            fails.append(f"{sitl_name}: setup_quad case {case_name} not parsed")
            print(f"{sitl_name:<8} {case_name:<10}  FAIL: case not found in setup_quad_matrix")
            continue
        sim = parse_sim_frame_motors(sim_path, sim_array, expect_n=4)
        mix = setup[case_name]
        if len(mix) != 4:
            fails.append(f"{sitl_name}: mixer has {len(mix)} motors, want 4")
            continue

        for i in range(4):
            S = sim[i]
            M = mix[i]
            n = i + 1
            # MOT index in SITL should be i+1 and match array order used by add_motors(i,...)
            if S["mot"] != n:
                fails.append(f"{sitl_name} M{n}: SITL MOT index {S['mot']}")

            notes = []
            if abs(S["angle_deg"] - M["angle_deg"]) > 1e-3:
                notes.append(f"ANG Δ={S['angle_deg']-M['angle_deg']:+.2f}")
                fails.append(
                    f"{sitl_name} M{n}: angle SITL {S['angle_deg']} vs mixer {M['angle_deg']}"
                )
            if sgn(S["yaw"]) != sgn(M["yaw"]) or abs(S["yaw"] - M["yaw"]) > 1e-6:
                notes.append(f"YAW S={S['yaw']} M={M['yaw']}")
                fails.append(
                    f"{sitl_name} M{n}: yaw SITL {S['yaw']} vs mixer {M['yaw']}"
                )
            if S["order"] != M["order"]:
                notes.append(f"ORDER S={S['order']} M={M['order']}")
                fails.append(
                    f"{sitl_name} M{n}: testing_order SITL {S['order']} vs mixer {M['order']}"
                )

            r, p = factors_from_angle(M["angle_deg"])
            ok = "OK" if not notes else "; ".join(notes)
            print(
                f"{sitl_name:<8} {case_name:<10} M{n} "
                f"{S['angle_deg']:8.1f} {M['angle_deg']:8.1f} "
                f"{S['yaw']:8.1f} {M['yaw']:8.1f} "
                f"{r:8.4f} {p:8.4f} "
                f"{S['order']:5d} {M['order']:5d}  {ok}"
            )

        # print allocation matrix for this frame
        print(f"  → allocation A  (FRAME_CLASS=1, FRAME_TYPE={ftype_val} / {case_name}):")
        print(f"     {'':>4} {'roll':>10} {'pitch':>10} {'yaw':>10}")
        for i, M in enumerate(mix):
            r, p = factors_from_angle(M["angle_deg"])
            print(f"     M{i+1:<3} {r:10.4f} {p:10.4f} {M['yaw']:10.1f}")
        sr = sum(factors_from_angle(M["angle_deg"])[0] for M in mix)
        sp = sum(factors_from_angle(M["angle_deg"])[1] for M in mix)
        sy = sum(M["yaw"] for M in mix)
        print(f"     ΣR={sr:+.4f}  ΣP={sp:+.4f}  ΣY={sy:+.1f}")
        print()

    # H frame: same angles as X, yaw flipped — present in mixer, no dedicated SITL name
    if "H" in setup and "X" in setup:
        print("Note: MOTOR_FRAME_TYPE_H (mixer only) = X angles with yaw signs flipped:")
        for i, (Hx, Xx) in enumerate(zip(setup["H"], setup["X"])):
            same_ang = abs(Hx["angle_deg"] - Xx["angle_deg"]) < 1e-6
            yaw_flip = sgn(Hx["yaw"]) == -sgn(Xx["yaw"])
            print(
                f"  M{i+1}: ang {Hx['angle_deg']:>7.1f} (same_as_X={same_ang})  "
                f"yaw H={Hx['yaw']:+.0f} X={Xx['yaw']:+.0f} flipped={yaw_flip}"
            )
            if not same_ang or not yaw_flip:
                fails.append(f"H vs X correspondence broken at M{i+1}")
        print()

    if fails:
        print(f"FAIL ({len(fails)}):")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("PASS: official SITL quad frames match setup_quad_matrix tables.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "--vehicle",
        choices=["df200", "jt20", "official", "all"],
        default="all",
        help="df200/jt20 = Lua↔JSON↔SIM; official = SITL↔setup_quad_matrix; all = everything",
    )
    ap.add_argument("--tol", type=float, default=2e-3, help="abs tol for roll/pitch factor match")
    ap.add_argument("--angle-tol", type=float, default=0.5, help="deg tol JSON atan2 vs SIM_Frame angle")
    args = ap.parse_args()

    rc = 0
    if args.vehicle in ("official", "all"):
        rc |= verify_official_quads()
        print()
    vehicles = []
    if args.vehicle == "all":
        vehicles = ["df200", "jt20"]
    elif args.vehicle in ("df200", "jt20"):
        vehicles = [args.vehicle]
    for v in vehicles:
        rc |= verify(v, args.tol, args.angle_tol)
        print()
    return rc


if __name__ == "__main__":
    sys.exit(main())

