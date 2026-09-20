#!/usr/bin/env python3
"""Generate AP_Motors allocation matrix from motor positions.

Same formula as AP_MotorsMatrix::add_motor():
  angle = atan2(Y, X)          # deg, APM body: X fwd, Y right
  roll  = cos(radians(angle + 90)) = -sin(radians(angle))
  pitch = cos(radians(angle))
  yaw   = prop direction (±1), NOT from geometry

Yaw signs must be supplied (JSON motorN_yaw, CLI, or built-in vehicle).

Two output scalings:
  raw         — Lua / Motors_dynamic style (default): keep cos values, |yaw|=1
  normalised  — setup_quad_matrix style: scale each axis so max |factor| = 0.5
                (AP_MotorsMatrix::normalise_rpy_factors)

Usage:
  # from SITL model JSON
  python3 docs/scripts/gen_motor_matrix.py --json Tools/autotest/models/df200.json

  # built-in vehicle (reads JSON + known yaw convention)
  python3 docs/scripts/gen_motor_matrix.py --vehicle df200
  python3 docs/scripts/gen_motor_matrix.py --vehicle jt20 --angle-source sim

  # official quad X (no JSON; angles from setup_quad_matrix)
  python3 docs/scripts/gen_motor_matrix.py --preset x
  python3 docs/scripts/gen_motor_matrix.py --preset + --scale normalised

  # manual positions (m) + yaw signs
  python3 docs/scripts/gen_motor_matrix.py \\
      --pos 1.72,1.315,-1.72,-1.315,1.72,-1.315,-1.72,1.315 \\
      --yaw 1,1,-1,-1

  # emit Lua snippet
  python3 docs/scripts/gen_motor_matrix.py --vehicle df200 --format lua
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def load_json_with_comments(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    lines = []
    for line in text.splitlines():
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
    cleaned = re.sub(r",\s*([}\]])", r"\1", "\n".join(lines))
    return json.loads(cleaned)


def factors_from_angle(angle_deg: float) -> tuple[float, float]:
    roll = math.cos(math.radians(angle_deg + 90.0))
    pitch = math.cos(math.radians(angle_deg))
    return roll, pitch


def angle_from_xy(x: float, y: float) -> float:
    return math.degrees(math.atan2(y, x))


def sgn(v: float) -> int:
    if v > 0:
        return 1
    if v < 0:
        return -1
    return 0


def normalise_rpy(motors: list[dict]) -> list[dict]:
    """AP_MotorsMatrix::normalise_rpy_factors — max |axis| → 0.5."""
    rmax = max(abs(m["roll"]) for m in motors) or 1.0
    pmax = max(abs(m["pitch"]) for m in motors) or 1.0
    ymax = max(abs(m["yaw"]) for m in motors) or 1.0
    out = []
    for m in motors:
        out.append(
            {
                **m,
                "roll": 0.5 * m["roll"] / rmax,
                "pitch": 0.5 * m["pitch"] / pmax,
                "yaw": 0.5 * m["yaw"] / ymax,
            }
        )
    return out


def parse_sim_frame_angles(array_name: str) -> list[dict]:
    path = REPO / "libraries/SITL/SIM_Frame.cpp"
    text = path.read_text(encoding="utf-8")
    m = re.search(
        rf"static\s+Motor\s+{re.escape(array_name)}\s*\[\s*\]\s*=\s*\{{(.*?)\n\}};",
        text,
        re.S,
    )
    if not m:
        raise ValueError(f"{array_name}[] not found in SIM_Frame.cpp")
    body = m.group(1)
    motors = []
    for mm in re.finditer(
        r"Motor\(\s*AP_MOTORS_MOT_(\d+)\s*,\s*([-\d.]+)f?\s*,\s*"
        r"AP_MOTORS_MATRIX_YAW_FACTOR_(CW|CCW)\s*,\s*(\d+)",
        body,
    ):
        yaw = 1.0 if mm.group(3) == "CCW" else -1.0
        motors.append(
            {
                "mot": int(mm.group(1)),
                "angle_deg": float(mm.group(2)),
                "yaw": yaw,
                "order": int(mm.group(4)),
            }
        )
    return motors


# Official setup_quad_matrix presets (angle, yaw, order) — MOT_1..4
PRESETS = {
    "+": [
        (90, 1, 2),
        (-90, 1, 4),
        (0, -1, 1),
        (180, -1, 3),
    ],
    "x": [
        (45, 1, 1),
        (-135, 1, 3),
        (-45, -1, 4),
        (135, -1, 2),
    ],
    "bfx": [
        (135, -1, 2),
        (45, 1, 1),
        (-135, 1, 3),
        (-45, -1, 4),
    ],
    "djix": [
        (45, 1, 1),
        (-45, -1, 4),
        (-135, 1, 3),
        (135, -1, 2),
    ],
    "cwx": [
        (45, 1, 1),
        (135, -1, 2),
        (-135, 1, 3),
        (-45, -1, 4),
    ],
    "h": [  # X angles, yaw flipped
        (45, -1, 1),
        (-135, -1, 3),
        (-45, 1, 4),
        (135, 1, 2),
    ],
}

VEHICLES = {
    "df200": {
        "json": REPO / "Tools/autotest/models/df200.json",
        "sim_array": "df200_motors",
        # default: angles from JSON positions (no CG shift)
        "default_angle_source": "json",
        "labels": [
            "FR tilt ①",
            "RL tilt ⑥",
            "FL tilt ⑤",
            "RR tilt ②",
            "FR vert ③",
            "RL vert ⑧",
            "FL vert ⑦",
            "RR vert ④",
        ],
    },
    "jt20": {
        "json": REPO / "Tools/autotest/models/jt20.json",
        "sim_array": "jt20_motors",
        # JSON is CG-shifted; mixer angles should come from SIM_Frame
        "default_angle_source": "sim",
        "labels": [
            "FR tilt",
            "RL tilt",
            "FL tilt",
            "RR tilt",
            "FR vert",
            "RL vert",
            "FL vert",
            "RR vert",
        ],
    },
}


def motors_from_json(model: dict, yaw_override: list[float] | None = None) -> list[dict]:
    n = int(model.get("num_motors", 0)) or sum(
        1 for i in range(1, 13) if f"motor{i}_position" in model
    )
    motors = []
    for i in range(1, n + 1):
        pos = model[f"motor{i}_position"]
        x, y = float(pos[0]), float(pos[1])
        ang = angle_from_xy(x, y)
        roll, pitch = factors_from_angle(ang)
        if yaw_override is not None:
            yaw = float(yaw_override[i - 1])
        else:
            yaw = float(model.get(f"motor{i}_yaw", 0.0))
        motors.append(
            {
                "mot": i,
                "x": x,
                "y": y,
                "z": float(pos[2]) if len(pos) > 2 else 0.0,
                "angle_deg": ang,
                "roll": roll,
                "pitch": pitch,
                "yaw": float(sgn(yaw)),  # mixer uses sign only
                "yaw_json_raw": yaw,
                "order": i,
            }
        )
    return motors


def motors_from_sim(array_name: str) -> list[dict]:
    raw = parse_sim_frame_angles(array_name)
    out = []
    for s in raw:
        roll, pitch = factors_from_angle(s["angle_deg"])
        out.append(
            {
                "mot": s["mot"],
                "x": None,
                "y": None,
                "z": None,
                "angle_deg": s["angle_deg"],
                "roll": roll,
                "pitch": pitch,
                "yaw": s["yaw"],
                "yaw_json_raw": s["yaw"],
                "order": s["order"],
            }
        )
    return out


def motors_from_preset(name: str) -> list[dict]:
    rows = PRESETS[name]
    out = []
    for i, (ang, yaw, order) in enumerate(rows, start=1):
        roll, pitch = factors_from_angle(ang)
        out.append(
            {
                "mot": i,
                "x": math.cos(math.radians(ang)),
                "y": math.sin(math.radians(ang)),
                "z": 0.0,
                "angle_deg": float(ang),
                "roll": roll,
                "pitch": pitch,
                "yaw": float(yaw),
                "yaw_json_raw": float(yaw),
                "order": order,
            }
        )
    return out


def motors_from_pos_yaw(pos_xy: list[float], yaws: list[float]) -> list[dict]:
    if len(pos_xy) % 2:
        raise ValueError("--pos needs even count: x1,y1,x2,y2,...")
    n = len(pos_xy) // 2
    if len(yaws) != n:
        raise ValueError(f"--yaw needs {n} values, got {len(yaws)}")
    out = []
    for i in range(n):
        x, y = pos_xy[2 * i], pos_xy[2 * i + 1]
        ang = angle_from_xy(x, y)
        roll, pitch = factors_from_angle(ang)
        out.append(
            {
                "mot": i + 1,
                "x": x,
                "y": y,
                "z": 0.0,
                "angle_deg": ang,
                "roll": roll,
                "pitch": pitch,
                "yaw": float(sgn(yaws[i])),
                "yaw_json_raw": float(yaws[i]),
                "order": i + 1,
            }
        )
    return out


def format_table(motors: list[dict], labels: list[str] | None = None) -> str:
    lines = []
    lines.append(
        f"{'M':<3} {'label':<12} {'x':>8} {'y':>8} {'ang°':>8} "
        f"{'roll':>10} {'pitch':>10} {'yaw':>6} {'order':>5}"
    )
    lines.append("-" * 90)
    for i, m in enumerate(motors):
        lab = labels[i] if labels and i < len(labels) else ""
        xs = f"{m['x']:8.3f}" if m["x"] is not None else f"{'—':>8}"
        ys = f"{m['y']:8.3f}" if m["y"] is not None else f"{'—':>8}"
        yaw_s = f"{m['yaw']:6.0f}" if abs(m['yaw']) in (0.0, 1.0) else f"{m['yaw']:6.2f}"
        lines.append(
            f"M{m['mot']:<2} {lab:<12} {xs} {ys} {m['angle_deg']:8.2f} "
            f"{m['roll']:10.4f} {m['pitch']:10.4f} {yaw_s} {m['order']:5d}"
        )
    sr = sum(m["roll"] for m in motors)
    sp = sum(m["pitch"] for m in motors)
    sy = sum(m["yaw"] for m in motors)
    lines.append("")
    lines.append(f"Σ roll={sr:+.4f}  Σ pitch={sp:+.4f}  Σ yaw={sy:+.1f}")
    return "\n".join(lines)


def format_lua(motors: list[dict], name: str = "BASE") -> str:
    lines = [
        f"-- generated by docs/scripts/gen_motor_matrix.py",
        f"-- roll=cos(ang+90), pitch=cos(ang), yaw=sign",
        f"local {name} = {{",
        f"    --          roll      pitch     yaw    order",
    ]
    for m in motors:
        lines.append(
            f"    {{ {m['roll']:8.4f}, {m['pitch']:8.4f}, {m['yaw']:2.0f}, {m['order']} }}, "
            f"-- M{m['mot']}  ang={m['angle_deg']:.2f}°"
        )
    lines.append("}")
    return "\n".join(lines)


def format_cpp_motordef(motors: list[dict]) -> str:
    lines = [
        "// MotorDef { angle_degrees, yaw_factor, testing_order }",
        "// for AP_MotorsMatrix::add_motors() / setup_*_matrix",
        "static const AP_MotorsMatrix::MotorDef motors[] {",
    ]
    for m in motors:
        yaw_macro = (
            "AP_MOTORS_MATRIX_YAW_FACTOR_CCW"
            if m["yaw"] > 0
            else "AP_MOTORS_MATRIX_YAW_FACTOR_CW"
            if m["yaw"] < 0
            else "0"
        )
        ang = m["angle_deg"]
        ang_s = f"{ang:.2f}f" if abs(ang - round(ang)) > 1e-6 else f"{int(round(ang))}"
        lines.append(f"    {{ {ang_s:>9}, {yaw_macro}, {m['order']} }}, // M{m['mot']}")
    lines.append("};")
    return "\n".join(lines)


def format_csv(motors: list[dict]) -> str:
    lines = ["mot,x,y,z,angle_deg,roll,pitch,yaw,order"]
    for m in motors:
        lines.append(
            f"{m['mot']},{m['x']},{m['y']},{m['z']},"
            f"{m['angle_deg']:.6f},{m['roll']:.6f},{m['pitch']:.6f},"
            f"{m['yaw']:.1f},{m['order']}"
        )
    return "\n".join(lines)


def parse_float_list(s: str) -> list[float]:
    return [float(x) for x in re.split(r"[, ]+", s.strip()) if x]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--vehicle", choices=list(VEHICLES), help="built-in DF200/JT20")
    src.add_argument("--json", type=Path, help="SITL model JSON with motorN_position/yaw")
    src.add_argument("--preset", choices=list(PRESETS), help="official quad preset (+/x/bfx/...)")
    src.add_argument("--pos", help="flat x1,y1,x2,y2,... positions (m)")
    ap.add_argument("--yaw", help="yaw signs/values, comma-separated (required with --pos)")
    ap.add_argument(
        "--angle-source",
        choices=["json", "sim"],
        help="for --vehicle: angles from JSON atan2 or SIM_Frame array",
    )
    ap.add_argument(
        "--scale",
        choices=["raw", "normalised"],
        default="raw",
        help="raw=Lua style; normalised=max|factor|->0.5 like setup_quad_matrix",
    )
    ap.add_argument(
        "--format",
        choices=["table", "lua", "cpp", "csv", "all"],
        default="all",
    )
    args = ap.parse_args()

    labels = None
    motors: list[dict]

    if args.preset:
        motors = motors_from_preset(args.preset)
        title = f"preset {args.preset}"
    elif args.pos:
        if not args.yaw:
            ap.error("--pos requires --yaw")
        motors = motors_from_pos_yaw(parse_float_list(args.pos), parse_float_list(args.yaw))
        title = "manual positions"
    elif args.vehicle:
        cfg = VEHICLES[args.vehicle]
        labels = cfg["labels"]
        src_ang = args.angle_source or cfg["default_angle_source"]
        model = load_json_with_comments(cfg["json"])
        if src_ang == "sim":
            motors = motors_from_sim(cfg["sim_array"])
            # still attach JSON yaw raw magnitude for info
            for i, m in enumerate(motors):
                key = f"motor{i+1}_yaw"
                if key in model:
                    m["yaw_json_raw"] = float(model[key])
                    m["yaw"] = float(sgn(m["yaw_json_raw"]))
            title = f"{args.vehicle} (angles=SIM_Frame::{cfg['sim_array']}, yaw=sign JSON)"
        else:
            motors = motors_from_json(model)
            title = f"{args.vehicle} (angles=JSON atan2, yaw=sign JSON)"
    else:
        model = load_json_with_comments(args.json)
        motors = motors_from_json(model)
        title = f"JSON {args.json}"

    if args.scale == "normalised":
        motors = normalise_rpy(motors)
        title += " [normalised max|rpy|=0.5]"
    else:
        title += " [raw]"

    print(f"=== Allocation matrix: {title} ===")
    print("formula: roll=cos(ang+90), pitch=cos(ang), yaw=±1 (prop direction)")
    print()

    if args.format in ("table", "all"):
        print(format_table(motors, labels))
        print()
        # show JSON |yaw| if present and not ±1
        raws = [m.get("yaw_json_raw") for m in motors]
        if any(r is not None and abs(abs(r) - 1.0) > 1e-6 for r in raws):
            print("note: JSON motorN_yaw raw values (SITL torque scale, not mixer):")
            print(" ", raws)
            print()

    if args.format in ("lua", "all"):
        print(format_lua(motors))
        print()

    if args.format in ("cpp", "all"):
        print(format_cpp_motordef(motors))
        print()

    if args.format == "csv":
        print(format_csv(motors))

    return 0


if __name__ == "__main__":
    sys.exit(main())
