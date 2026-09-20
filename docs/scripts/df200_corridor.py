#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DF-200 倾转走廊（tilt corridor）分析脚本

定义一条 θ–V 包线走廊：
  - V_min(θ)：升力下限——外桨推力 + 内桨推力·cos(θ) + 机翼升力(L∝V²) 恰好托住重量
  - V_max(θ)：功率封顶——给定功率预算 P_cap 下，该 (θ,V) 所需总功率不超过 P_cap
  - 功率等高线 P(θ,V)：在该 (θ,V) 维持平飞所需电机总功率
并把 193 仿真实测轨迹 (θ,V,P) 叠加到走廊图上，验证仿真落在走廊内。

参数来源（与 SITL 仿真完全一致）：
  电机表：docs/df200_motor_data.md（源自 docs/电机电池.docx），落地 libraries/SITL/SIM_MotorTable.h
          - 外桨 60in 静态表 prop60_*（MOT_5..8，rpm×3181）
          - 内桨 40in 风洞表 prop40_*（MOT_1..4，rpm×4200，依赖来流 velin）
  气动  ：docs/数据库_v2.xlsx，落地 libraries/SITL/SIM_Plane.cpp:109-173（-df200 分支）
          - S=5.28846 b=7.5 c=0.70513  CL0=0.311 CLa=6.023  CD0=0.03195 oswald=0.9 α_stall=0.2443
  质量  ：MTOW=380kg（xlsx）— SITL json mass=233.333×1.5；对照 350kg（docx 口径）也画

复用 docs/scripts/df200_power_analysis.py 的：
  - 电机表插值（prop60/prop40，与 SIM_MotorTable.h 同函数同输入）
  - load_rotor（读 /tmp/df200_rotor.log，取最后一趟）
  - bin_edges（从 BIN 取 TILT 时序 + MSG 阶段边界）

用法：
  python3 docs/scripts/df200_corridor.py \\
      --bin logs/00000193.BIN --rotor /tmp/df200_rotor.log \\
      --out-md docs/df200_tilt_corridor.md --img-dir docs/df200_corridor_imgs

依赖：numpy, matplotlib, pymavlink
"""

import argparse
import math
import os
import re
import sys
from pathlib import Path

import numpy as np

# matplotlib Agg（无显示环境）
import matplotlib
matplotlib.use("Agg")
# 中文字体：优先用系统已有的 CJK 字体，找不到则回退英文标签
_CJK_FONTS = ["Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei",
              "WenQuanYi Micro Hei", "Source Han Sans SC", "AR PL UMing CN",
              "Droid Sans Fallback"]
import matplotlib.font_manager as fm
# 注册 Noto Sans CJK ttc（系统有但 matplotlib 默认不扫 ttc）
for _p in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
           "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"):
    if os.path.exists(_p):
        try:
            fm.fontManager.addfont(_p)
        except Exception:
            pass
_available = {f.name for f in fm.fontManager.ttflist}
_CJK_OK = next((f for f in _CJK_FONTS if f in _available), None)
if _CJK_OK:
    matplotlib.rcParams["font.sans-serif"] = [_CJK_OK, "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt  # noqa: E402

# ---------------------------------------------------------------------------
# 复用 df200_power_analysis.py 的电机表 + 日志解析（同函数同输入，保证一致）
# ---------------------------------------------------------------------------
_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))
import importlib.util  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "_df200_pa", _SCRIPT_DIR / "df200_power_analysis.py")
_pa = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_pa)

# 直接复用
prop60_thrust = _pa.prop60_thrust
prop60_power = _pa.prop60_power
prop40_thrust = _pa.prop40_thrust
prop40_power = _pa.prop40_power
load_rotor = _pa.load_rotor
bin_edges = _pa.bin_edges

# 转速系数（SIM_Motor.h calculate_forces :74 重载 :204：外桨 cmd×3181、内桨 cmd×4200）
RPM_MAX_OUTER = 3181.0
RPM_MAX_INNER = 4200.0

# ---------------------------------------------------------------------------
# 气动参数（与 SIM_Plane.cpp:109-173 -df200 分支逐项一致）
# ---------------------------------------------------------------------------
RHO = 1.152          # 空气密度 @584m（df200_session §3.7；SITL 推力按 ρ/1.225 缩放，走廊算在 584m）
RHO_SL = 1.225        # 海平面（电机表标定密度）
S = 5.28846           # 参考面积 m²
B = 7.5               # 翼展 m
C = 0.70513           # 平均气动弦长 m
CL0 = 0.311           # α≈0 升力系数
CLA = 6.023           # 1/rad，α=-2..8° 拟合
CD0 = 0.03195         # 最小阻力系数
OSWALD = 0.9
ALPHA_STALL = 0.2443  # rad ~14°
ALPHA_CRUISE = math.radians(2.0)   # 巡航工作迎角（与 §3.7 复算一致）

G = 9.80665

# 质量基准两套
MASS_CONFIGS = {
    "380_55": dict(label="380 kg / 55 m/s (SITL 实况)",
                   mass=380.0, v_cruise=55.0, note="xlsx MTOW + parm AIRSPEED_CRUISE"),
    "350_48": dict(label="350 kg / 48.6 m/s (xlsx/docx 原始)",
                   mass=350.0, v_cruise=48.6, note="docx 口径 + xlsx 基本信息"),
}


def wing_lift(V, alpha=ALPHA_CRUISE, rho=RHO):
    """机翼升力 L = 0.5·ρ·V²·S·CL，CL=CL0+CLa·α（线性段，未含失速）。"""
    cl = CL0 + CLA * alpha
    return 0.5 * rho * V * V * S * cl


def wing_drag(V, alpha=ALPHA_CRUISE, rho=RHO):
    """机翼阻力 D = 0.5·ρ·V²·S·(CD0 + CL²/(π·e·AR))。AR=b²/S。"""
    cl = CL0 + CLA * alpha
    ar = B * B / S
    cd = CD0 + cl * cl / (math.pi * OSWALD * ar)
    return 0.5 * rho * V * V * S * cd


# ---------------------------------------------------------------------------
# 走廊物理模型
# ---------------------------------------------------------------------------
def motor_thrust_outer(cmd):
    """外桨单桨推力（N），cmd∈[0,1] → rpm=cmd·3181 → 查静态表。
    注意：SITL calc_thrust_table 对推力做 ρ/1.225 缩放（SIM_Motor.cpp:378），
    走廊算在 584m，故乘 RHO/RHO_SL。"""
    rpm = cmd * RPM_MAX_OUTER
    return prop60_thrust(rpm) * (RHO / RHO_SL)


def motor_thrust_inner(cmd, V):
    """内桨单桨推力（N），rpm=cmd·4200，查风洞表（依赖来流 V=velin），
    同样做密度缩放。"""
    rpm = cmd * RPM_MAX_INNER
    return prop40_thrust(rpm, V) * (RHO / RHO_SL)


def motor_power_outer(cmd):
    """外桨单桨轴功率（W），查静态表（功率不做密度缩放，与 SIM_Motor 一致）。"""
    rpm = cmd * RPM_MAX_OUTER
    return prop60_power(rpm)


def motor_power_inner(cmd, V):
    """内桨单桨轴功率（W），查风洞表。"""
    rpm = cmd * RPM_MAX_INNER
    return prop40_power(rpm, V)


def vertical_balance(theta_deg, V, mass, f_outer=1.0, cmd_inner=1.0):
    """给定 (θ,V)，在 f_outer·外桨满推 + cmd_inner·内桨满推 下，垂向分量 vs 重量。

    外桨固定垂起：Fz_outer = 4·T_o·f_outer
    内桨倾转 θ：Fz_inner = 4·T_i·cos(θ)，T_i 依赖来流 V（倾转后内桨来流≈V）
    机翼升力：L = wing_lift(V)

    返回 (ΣFz, W, Fz_outer, Fz_inner, L_wing)。
    """
    th = math.radians(theta_deg)
    # 内桨来流：倾转后桨盘法向分量≈V·sin(θ)（近似，与 velin 同量级）；走廊用 V 近似
    T_o = motor_thrust_outer(1.0)            # 满推单桨
    T_i = motor_thrust_inner(1.0, V)         # 满推单桨 @来流 V
    Fz_o = 4.0 * T_o * f_outer
    Fz_i = 4.0 * T_i * math.cos(th) * cmd_inner
    L = wing_lift(V)
    W = mass * G
    return Fz_o + Fz_i + L, W, Fz_o, Fz_i, L


def v_min_for_theta(theta_deg, mass, f_outer=1.0, cmd_inner=1.0):
    """V_min(θ)：在给定旋翼策略下，内桨满推 (ci=1) 时垂直恰好托住重量的最小空速。

    垂直平衡（ci=1）：4·To·fo + 4·Ti_max(V)·cos(θ) + L(V) = W
      - fo=1（旋翼满推）：V=0 即可托住 → V_min=0（悬停）
      - fo=0（外桨退场）：靠机翼+内桨满推，V_min>0

    注：这是"垂直恰好平衡"的下限。推进是否够是独立约束——
    在该 V_min 处若 Fx<D（推不动），说明 fo=0 路径在此 θ 需更高 V，
    但 V_min 本身定义为垂直平衡点（走廊下边界的主语义）。
    """
    th = math.radians(theta_deg)
    W = mass * G
    T_o = motor_thrust_outer(1.0)
    # θ=0 且 fo<1：物理不可行——内桨全垂直(cos0=1)、外桨全垂直，无水平推进器
    # 克服阻力（sin0=0），即使数学上机翼能补到托住重量也飞不了（无推进）
    if theta_deg < 1.0 and f_outer < 0.999:
        return float("nan")
    # V=0 时旋翼能否托住
    fz0 = 4.0 * T_o * f_outer + 4.0 * motor_thrust_inner(1.0, 0.0) * math.cos(th)
    if fz0 >= W:
        return 0.0
    # 二分 V：fz(V) - W 由负转正
    lo, hi = 0.0, 80.0
    fz_hi = 4.0 * T_o * f_outer + 4.0 * motor_thrust_inner(1.0, hi) * math.cos(th) \
        + wing_lift(hi)
    if fz_hi < W:
        return float("nan")  # V=80 仍托不住（理论不应发生）
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        Ti = motor_thrust_inner(1.0, mid)
        fz = 4.0 * T_o * f_outer + 4.0 * Ti * math.cos(th) + wing_lift(mid)
        if fz < W:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def total_power_at(theta_deg, V, mass, f_outer=0.0):
    """给定 (theta,V) 与旋翼策略 f_outer，维持平飞所需电机总功率 (kW)。

    策略：外桨出力份额 f_outer in [0,1] 固定（1=满推垂起，0=外桨退场即 193 transition）。
    内桨油门 ci 自动取"恰好满足垂直平衡"的最小值；再检查水平推进是否够克服阻力。

    垂直平衡：4*To*fo + 4*Ti*ci*cos(theta) + L_wing = W
    水平推进：4*Ti*ci*sin(theta) >= D  （不足则该点不可平飞，返回 inf）

    返回 (P_total_kW, fo, ci, Fx_net, D, ok)。
    """
    W = mass * G
    th = math.radians(theta_deg)
    L = wing_lift(V)
    D = wing_drag(V)
    deficit = W - L  # 旋翼需补的垂向缺口

    T_i_max = motor_thrust_inner(1.0, V)   # 内桨满推单桨推力（依赖来流 V）
    T_o_max = motor_thrust_outer(1.0)      # 外桨满推单桨推力（静态）

    fo = max(0.0, min(1.0, f_outer))
    Fz_o = 4.0 * T_o_max * fo

    ci = 0.0
    ok = True
    cos_th = math.cos(th)
    sin_th = math.sin(th)

    need_vert = deficit - Fz_o  # 内桨需补的垂向
    if need_vert <= 1e-6:
        # 外桨已托住重量，内桨只管推进：4*Ti_max*sin*ci = D
        if sin_th < 1e-6:
            ci = 0.0
            ok = (D < 1.0)  # theta=0 垂起悬停 D=0 才行
        else:
            ci = D / (4.0 * T_i_max * sin_th) if T_i_max > 0 else 1.0
            ci = max(0.0, min(1.0, ci))
            # ci<1 说明内桨有富余；若算出的 ci>1 则该 V 下推不动，需更高 V
            ok = ci <= 1.0
    else:
        # 内桨补垂直：4*Ti_max*cos*ci >= need_vert
        if cos_th < 1e-6 or T_i_max <= 0:
            ok = False
            ci = 1.0
        else:
            ci = need_vert / (4.0 * T_i_max * cos_th)
            if ci > 1.01:
                # 内桨满推仍托不住垂直 -> fo=0 策略此 V 不行，需更高 V
                ok = False
                ci = 1.0
            else:
                # 垂直可平衡（ci<=1.01 视为满推边界）。推进：满推 Fx vs D
                # Fx<D 表示此 V 推不动（fo=0 推进下限），不是"不可平飞"，
                # ok=True（垂直能托住），Fx_net 标负。真正不可平飞是垂直托不住。
                ok = True
                if ci > 1.0:
                    ci = 1.0  # 钳到满推（边界点）
                # 边界点（ci 钳到 1.0 满推）下，推进够即 ok（不再因 ci>1 判 False）

    P_o = 4.0 * motor_power_outer(fo)
    P_i = 4.0 * motor_power_inner(ci, V)
    Fx_net = 4.0 * T_i_max * sin_th * ci - D
    if not ok:
        return float("inf"), fo, ci, Fx_net, D, False
    return (P_o + P_i) / 1000.0, fo, ci, Fx_net, D, True


def v_max_for_theta(theta_deg, mass, p_cap_kw, f_outer=0.0):
    """V_max(theta): 可行区上界——从 V_min 起向上扫，找最大的同时满足
    (ok=True 且 P<=P_cap) 的空速。

    物理含义：高速时阻力 D∝V² 升高，内桨水平分量 4·Ti·ci·sin(θ) 若不够
    克服 D（θ 小时 sin 小）则不可平飞；或推进功率超过 P_cap。两者取上界。
    """
    vmin = v_min_for_theta(theta_deg, mass, f_outer=f_outer)
    if math.isnan(vmin):
        return float("nan")
    # 从 V_min 向上扫描，记录最大的可行 V（允许中间窄 not-ok 带，取最大）
    last_ok = vmin
    for V in np.arange(vmin + 0.5, 80.0 + 0.5, 0.5):
        P, _, _, _, _, ok = total_power_at(theta_deg, V, mass, f_outer=f_outer)
        if ok and P <= p_cap_kw:
            last_ok = V
    # 二分精化 last_ok .. last_ok+0.5 之间
    lo, hi = last_ok, min(last_ok + 0.5, 80.0)
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        P, _, _, _, _, ok = total_power_at(theta_deg, mid, mass, f_outer=f_outer)
        if ok and P <= p_cap_kw:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ---------------------------------------------------------------------------
# 193 实测轨迹提取
# ---------------------------------------------------------------------------
def _bin_track(bin_path):
    """仅用 BIN 提取 (t, AS, tilt) 轨迹——rotor.log 缺失时的回退。

    注：MOTT.Mot1..8 在 DF-200 配置下记录的是统一集体油门（8 值相同），
    丢失差动信息，不能用于精确功率反查（差动使功率偏高 16-32%，
    见 df200_power_corridor.md §7）。故 BIN-only 模式只画 (θ,V) 轨迹，
    功率引用 df200_power_corridor.md 已有的 193 实测报告值。

    轨迹用 TILT 时间轴采样，CTUN.As 插值得空速。
    """
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(bin_path)
    as_list, tilt_list = [], []
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            break
        typ = msg.get_type()
        t = getattr(msg, "TimeUS", None)
        if t is None:
            continue
        t = t / 1e6
        if typ == "GPS":
            # GPS.Spd = 地速 (m/s)；SITL 无风时 ≈ TAS，与 rotor.log AS 同口径
            # （CTUN.As 是皮托 EAS，低速段失真，不用）
            as_list.append((t, getattr(msg, "Spd", 0.0)))
        elif typ == "TILT":
            tilt_list.append((t, getattr(msg, "Tilt", 0.0)))
    m.close()
    if not tilt_list:
        return None
    tt = np.array([x[0] for x in tilt_list])
    tilt_v = np.array([x[1] for x in tilt_list])
    as_t = np.array([x[0] for x in as_list]) if as_list else np.array([0.0])
    as_v = np.array([x[1] for x in as_list]) if as_list else np.array([0.0])
    as_at = np.interp(tt, as_t, as_v, left=0.0, right=as_v[-1] if len(as_v) else 0.0)

    edges = bin_edges(bin_path, tt[-1] if len(tt) else 100.0)
    rec = dict(t=tt, AS=as_at, tilt=tilt_v,
               P_total=np.full(len(tt), np.nan),   # 功率引用报告值，不在此反查
               P_inner=np.full(len(tt), np.nan),
               P_outer=np.full(len(tt), np.nan),
               Fz_rotor=np.full(len(tt), np.nan),
               L_wing=np.array([wing_lift(v) for v in as_at]),
               source="BIN-only (TILT+CTUN.As 轨迹；功率见 df200_power_corridor.md)")
    rec["edges"] = edges
    return rec


def extract_193_track(bin_path, rotor_path):
    """返回 dict：t, AS, tilt, P_total, P_inner, P_outer, Fz_rotor, L_wing。
    优先用 rotor.log（含 velin/thrmag 精确值）；缺失则回退 BIN-only（MOTT→cmd→rpm→power）。"""
    rows = load_rotor(rotor_path)
    if not rows:
        return _bin_track(bin_path)
    edges = bin_edges(bin_path, rows[-1]["t"])
    tilt_ts = edges["tilt_series"]
    tilt_t = np.array([x[0] for x in tilt_ts]) if tilt_ts else np.array([0.0])
    tilt_v = np.array([x[1] for x in tilt_ts]) if tilt_ts else np.array([0.0])

    tt = np.array([r["t"] for r in rows])
    tilt_at = np.interp(tt, tilt_t, tilt_v, left=0.0, right=90.0)

    rec = dict(t=tt, AS=np.array([r["AS"] for r in rows]),
               tilt=tilt_at,
               P_total=np.zeros(len(rows)),
               P_inner=np.zeros(len(rows)),
               P_outer=np.zeros(len(rows)),
               Fz_rotor=np.zeros(len(rows)),
               L_wing=np.zeros(len(rows)))
    for i, r in enumerate(rows):
        Pi = sum(prop40_power(r["rpm"][k], r["velin"][k]) for k in range(4))
        Po = sum(prop60_power(r["rpm"][k]) for k in range(4, 8))
        rec["P_inner"][i] = Pi / 1000.0
        rec["P_outer"][i] = Po / 1000.0
        rec["P_total"][i] = (Pi + Po) / 1000.0
        # 旋翼垂直推力（用 thrmag，已含密度缩放）+ 机翼升力
        if "thrmag" in r:
            To = sum(r["thrmag"][4:8])
            Ti_mag = sum(r["thrmag"][0:4])
            th = math.radians(tilt_at[i])
            rec["Fz_rotor"][i] = To + Ti_mag * math.cos(th)
        rec["L_wing"][i] = wing_lift(r["AS"])
    rec["edges"] = edges
    return rec


# ---------------------------------------------------------------------------
# 画图（简洁版：升力下限 + 功率封顶 + 193 轨迹）
# ---------------------------------------------------------------------------
# 193 功率报告值（docs/df200_power_corridor.md §8 tilt 切片），用于功率走廊图
# 外桨在 transition 全程有出力，FW done 后才 SHUTDOWN
POWER_193_TILT = [
    # (tilt_mid_deg, P_mean_kW, P_peak_kW, label)
    (15, 88.0, 107.1, "0-30"),
    (37.5, 102.8, 110.1, "30-45"),
    (52.5, 76.2, 112.7, "45-60"),
    (75, 47.8, 52.0, "60-90"),
    (90, 7.6, 15.4, "FW"),
]


def plot_corridor(cfg_key, mass_cfg, track, img_dir, p_cap_kw=110.0):
    """简洁 θ–V 走廊：一条 V_min（外桨全关下限）+ 功率封顶区 + 193 轨迹。

    图意：
      - 蓝线 V_min：外桨一旦关死，机翼+内桨至少要托住重量的空速下限
      - 红线：倾转段功率封顶 110 kW（电池规格，见 power_session §6）
      - 黑线：193 实测 (θ,V) —— 应落在蓝线之上
    """
    mass = mass_cfg["mass"]
    thetas = np.linspace(0, 90, 31)
    v_min = np.array([v_min_for_theta(th, mass, f_outer=0.0) for th in thetas])

    fig, ax = plt.subplots(figsize=(9, 6))

    # 允许区：V >= V_min（外桨可安全关死的区）；V_min 为 nan 的 θ=0 不填
    valid = ~np.isnan(v_min)
    ax.fill_between(thetas[valid], v_min[valid], 75,
                    color="#c6efce", alpha=0.55, label="允许区 (V >= V_min)")
    ax.fill_between(thetas[valid], 0, v_min[valid],
                    color="#ffc7ce", alpha=0.35, label="掉高区 (V < V_min, 关外桨会掉)")

    ax.plot(thetas[valid], v_min[valid], "b-", lw=2.5,
            label="V_min: 外桨关死后的升力下限")
    # 功率封顶作为水平参考（倾转段 ≤110 kW）；在 θ–V 图上用文字标注，
    # 真正的功率对比见 plot_power_corridor
    ax.axhline(55, color="gray", ls=":", lw=1.0, alpha=0.7,
               label="巡航目标 55 m/s")

    # 193 轨迹：只画 transition 窗口（trans_start → fw_done+5s），
    # 避免 rotor.log 多趟追加 / 长巡航段把 (θ,V) 连成伪影水平线
    if track is not None:
        edges = track["edges"]
        t0 = edges.get("trans_start")
        t1 = edges.get("fw_done")
        # 只画到 fw_done：其后 TILT 日志会跳回 0，但空速仍约巡航，
        # 会画出伪影水平线 (0°,58)↔(90°,58)。
        if t0 is not None and t1 is not None:
            mask = (track["t"] >= t0 - 1.0) & (track["t"] <= t1 + 0.2)
        else:
            mask = np.ones(len(track["t"]), dtype=bool)
        th = track["tilt"][mask]
        vv = track["AS"][mask]
        tt = track["t"][mask]
        order = np.argsort(tt)
        ax.plot(th[order], vv[order], "-",
                color="black", lw=2.0, zorder=5, label="193 transition 轨迹")
        for label, key, xytext in [
            ("trans_start", "trans_start", (8, -12)),
            ("aspd_reached", "aspd_reached", (8, 8)),
            ("FW done\n(外桨SHUTDOWN)", "fw_done", (-70, 10)),
        ]:
            t = edges.get(key)
            if t is None:
                continue
            idx = int(np.argmin(np.abs(track["t"] - t)))
            ax.plot(track["tilt"][idx], track["AS"][idx], "o",
                    color="black", ms=7, zorder=6)
            ax.annotate(label, (track["tilt"][idx], track["AS"][idx]),
                        textcoords="offset points", xytext=xytext,
                        fontsize=8, color="black")

    ax.set_xlim(0, 90)
    ax.set_ylim(0, 70)
    ax.set_xlabel("倾转角 θ (°)  0=垂直 / 90=水平")
    ax.set_ylabel("空速 V (m/s)")
    ax.set_title(f"DF-200 倾转走廊  [{mass_cfg['label']}]\n"
                 f"下限=升力托住重量；外桨在 FW done 才 SHUTDOWN")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)

    out = img_dir / f"corridor_{cfg_key}.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def plot_power_corridor(img_dir, p_cap_kw=110.0):
    """主图：功率 vs θ。封顶 = 垂起 90 kW / 倾转峰值 110 kW；叠 193 实测。"""
    fig, ax = plt.subplots(figsize=(9, 5.5))
    th = [r[0] for r in POWER_193_TILT]
    p_mean = [r[1] for r in POWER_193_TILT]
    p_peak = [r[2] for r in POWER_193_TILT]
    labels = [r[3] for r in POWER_193_TILT]

    # 功率包线：θ 小偏垂起 90kW，倾转中段允许 110kW，前飞回落
    # 用分段水平线表达「可持续 / 短时峰值」
    ax.fill_between([0, 90], 0, 90.0, color="#c6efce", alpha=0.35,
                    label="可持续区 ≤90 kW（垂起约 50 s）")
    ax.fill_between([0, 90], 90.0, p_cap_kw, color="#ffe699", alpha=0.40,
                    label=f"短时峰值区 90–{p_cap_kw:.0f} kW（累计 ≤10 s）")
    ax.axhline(90.0, color="orange", ls="-", lw=1.8,
               label="垂起封顶 90 kW / 50 s")
    ax.axhline(p_cap_kw, color="red", ls="--", lw=2.0,
               label=f"峰值封顶 {p_cap_kw:.0f} kW / ≤10 s")
    ax.axhline(25.0, color="gray", ls=":", lw=1.2,
               label="巡航预算 ~25 kW")

    ax.plot(th, p_mean, "b-o", lw=2.0, ms=7, label="193 均值")
    ax.plot(th, p_peak, "b--^", lw=1.2, ms=6, alpha=0.75, label="193 峰值")
    for x, y, lab in zip(th, p_peak, labels):
        ax.annotate(f"{lab}\n{y:.0f}", (x, y), textcoords="offset points",
                    xytext=(0, 8), ha="center", fontsize=8, color="navy")

    ax.set_xlim(0, 95)
    ax.set_ylim(0, 140)
    ax.set_xlabel("倾转角 θ (°)")
    ax.set_ylabel("电机总功率 (kW)")
    ax.set_title("DF-200 倾转走廊 · 主图：功率限制\n"
                 "90 kW/50 s · 110 kW/≤10 s · 193 轨迹验收")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    out = img_dir / "corridor_power.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def plot_compare(track, img_dir, p_cap_kw=110.0):
    """两套基准 V_min 对比 + 193 轨迹（简洁）。"""
    thetas = np.linspace(0, 90, 31)
    fig, ax = plt.subplots(figsize=(9, 6))
    styles = {"380_55": ("b-", "380kg/55m/s (SITL)"),
              "350_48": ("g-", "350kg/48.6m/s (原始)")}
    for key, cfg in MASS_CONFIGS.items():
        v_min = np.array([v_min_for_theta(th, cfg["mass"], f_outer=0.0) for th in thetas])
        st, lab = styles[key]
        valid = ~np.isnan(v_min)
        ax.plot(thetas[valid], v_min[valid], st, lw=2.2,
                label=f"V_min {lab}")
    if track is not None:
        edges = track["edges"]
        t0 = edges.get("trans_start")
        t1 = edges.get("fw_done")
        if t0 is not None and t1 is not None:
            mask = (track["t"] >= t0 - 1.0) & (track["t"] <= t1 + 0.2)
        else:
            mask = np.ones(len(track["t"]), dtype=bool)
        order = np.argsort(track["t"][mask])
        ax.plot(track["tilt"][mask][order], track["AS"][mask][order], "k-",
                lw=2.0, label="193 transition 轨迹")
    ax.set_xlim(0, 90)
    ax.set_ylim(0, 70)
    ax.set_xlabel("倾转角 θ (°)")
    ax.set_ylabel("空速 V (m/s)")
    ax.set_title("DF-200 V_min 两套基准对比 + 193 轨迹\n"
                 "（V_min = 外桨关死后的升力下限）")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    out = img_dir / "corridor_compare.png"
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 走廊数值表（写入 markdown）
# ---------------------------------------------------------------------------
def corridor_table(mass_cfg, p_cap_kw=110.0):
    """生成走廊数值表行（θ 0..90 步 10°），fo=0 主路径。
    P@V_min = 满推功率（ci=1，V_min 定义即满推恰好托住）。"""
    rows = []
    mass = mass_cfg["mass"]
    for th in range(0, 91, 10):
        vm = v_min_for_theta(th, mass, f_outer=0.0)
        vmax = v_max_for_theta(th, mass, p_cap_kw, f_outer=0.0)
        # V_min 处功率：fo=0 + 内桨满推 (ci=1) @ V_min
        if math.isnan(vm):
            P_min = float("nan")
            ci = 1.0
            L_frac = float("nan")
        else:
            P_min = 4.0 * motor_power_inner(1.0, vm) / 1000.0  # fo=0 外桨=0
            # 推进检查
            Ti = motor_thrust_inner(1.0, vm)
            Fx = 4.0 * Ti * math.sin(math.radians(th))
            D = wing_drag(vm)
            ci = 1.0
            # 若 Fx<D，标 P_min 仍为满推功率（推进不足是 V_min 的另一约束，这里报功率）
            L_frac = wing_lift(vm) / (mass * G)
        rows.append(dict(theta=th, v_min=vm, v_max=vmax,
                         P_min=P_min, f_outer=0.0, ci=ci,
                         L_frac=L_frac))
    return rows


def fmt(x, p=1):
    if math.isnan(x):
        return "—"
    return f"{x:.{p}f}"


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="DF-200 倾转走廊分析")
    ap.add_argument("--bin", default="logs/00000193.BIN")
    ap.add_argument("--rotor", default="/tmp/df200_rotor.log")
    ap.add_argument("--out-md", default="docs/df200_tilt_corridor.md")
    ap.add_argument("--img-dir", default="docs/df200_corridor_imgs")
    ap.add_argument("--p-cap", type=float, default=110.0,
                    help="功率封顶 kW（默认 110，对应早期走廊封顶）")
    args = ap.parse_args()

    img_dir = Path(args.img_dir)
    img_dir.mkdir(parents=True, exist_ok=True)

    # θ–V 轨迹强制用 BIN（TILT+GPS.Spd），与 --bin 指定的 193 飞次对齐。
    # rotor.log 是 fopen("a") 追加写，常叠多趟，时间轴易与 BIN 错配；
    # 功率已用 POWER_193_TILT 报告值，不必依赖 rotor 反查。
    track = _bin_track(args.bin)
    if track is None:
        print("[warn] BIN 轨迹提取失败，只画走廊不叠轨迹", file=sys.stderr)
    else:
        print(f"[info] 轨迹来源: {track.get('source', 'BIN')}  n={len(track['t'])}",
              file=sys.stderr)

    # 画简洁走廊图 + 功率走廊 + 对比图
    paths = []
    for key, cfg in MASS_CONFIGS.items():
        paths.append(plot_corridor(key, cfg, track, img_dir, args.p_cap))
    paths.append(plot_power_corridor(img_dir, args.p_cap))
    paths.append(plot_compare(track, img_dir))

    # 走廊数值表（两套）
    lines = []
    for key, cfg in MASS_CONFIGS.items():
        lines.append(f"\n### {cfg['label']}\n")
        lines.append("| θ° | V_min (m/s) | V_max (m/s) | P@V_min (kW) | 外桨份额 f_o | 翼升力/W |")
        lines.append("|----|-------------|-------------|-------------|-------------|----------|")
        for r in corridor_table(cfg, args.p_cap):
            lines.append(f"| {r['theta']} | {fmt(r['v_min'],1)} | {fmt(r['v_max'],1)} | "
                         f"{fmt(r['P_min'],1)} | {r['f_outer']:.2f} | {r['L_frac']:.2f} |")

    # 193 轨迹关键点（功率引用 df200_power_corridor.md 报告值，BIN-only 模式不反查）
    if track is not None:
        edges = track["edges"]
        # 时刻 -> (标签, 报告功率kW)。功率取自 df200_power_corridor.md 193 实测
        checkpoints = [
            (edges.get("trans_start", 87.1), "trans_start", 4.0),
            (92.0, "tilt=45 (AS_WAIT 卡点)", 7.3),
            (edges.get("aspd_reached", 108.5), "aspd_reached", 6.6),
            (edges.get("fw_done", 113.0), "FW done (tilt=90)", 7.1),
            (120.0, "FW 巡航", 6.0),
        ]
        lines.append("\n### 193 实测轨迹关键点\n")
        lines.append("| 时刻 | t (s) | AS (m/s) | tilt (°) | 报告功率 (kW) | 翼升力 (N) | 翼升力/W(380) |")
        lines.append("|------|-------|----------|---------|---------------|-----------|---------------|")
        W = 380.0 * G
        for t, label, P_rep in checkpoints:
            if t is None:
                continue
            idx = int(np.argmin(np.abs(track["t"] - t)))
            lw = track["L_wing"][idx]
            lines.append(f"| {label} | {track['t'][idx]:.1f} | {track['AS'][idx]:.1f} | "
                        f"{track['tilt'][idx]:.1f} | {P_rep:.1f} | {lw:.0f} | {lw/W:.2f} |")
        lines.append("\n> 功率来源：`df200_power_corridor.md` 193 实测（rotor.log 精确反查）。"
                     "BIN-only 模式（rotor.log 缺失时）只画 (θ,V) 轨迹，功率引用报告值。")

    print("\n".join(lines))
    print("\n图片：", ", ".join(str(p) for p in paths))
    print(f"Markdown 片段已打印（图片在 {img_dir}）")


if __name__ == "__main__":
    main()
