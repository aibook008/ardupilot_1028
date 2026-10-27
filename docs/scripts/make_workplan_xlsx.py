#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于 docs/飞控开发说明.md 生成 Excel 工作计划表（两个 sheet：全部节点 + 十月计划）"""
import openpyxl
from openpyxl.styles import Font, Alignment, PatternFill, Border, Side
from openpyxl.utils import get_column_letter

OUT = "/home/codyqin/Desktop/git/ardupilot/docs/飞控工作计划表.xlsx"

wb = openpyxl.Workbook()

# ---------- 通用样式 ----------
FONT_TITLE = Font(name="微软雅黑", size=14, bold=True)
FONT_HEADER = Font(name="微软雅黑", size=10.5, bold=True, color="FFFFFF")
FONT_BODY = Font(name="微软雅黑", size=10.5)
FONT_SECTION = Font(name="微软雅黑", size=10.5, bold=True)

FILL_HEADER = PatternFill("solid", fgColor="2F5496")   # 深蓝表头
FILL_SECTION = PatternFill("solid", fgColor="D9E2F3")  # 浅蓝分组行
FILL_DONE = PatternFill("solid", fgColor="E2EFDA")     # 绿：已完成
FILL_NOW = PatternFill("solid", fgColor="FFF2CC")      # 黄：当前进行
FILL_FOCUS = PatternFill("solid", fgColor="FCE4D6")    # 橙：月度重点

THIN = Side(style="thin", color="B0B0B0")
BORDER = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)

WRAP_TOP = Alignment(wrap_text=True, vertical="top", horizontal="left")
CENTER = Alignment(wrap_text=True, vertical="center", horizontal="center")


def style_row(ws, row, ncols, *, body=True, fill=None, font=None, align=None):
    for c in range(1, ncols + 1):
        cell = ws.cell(row=row, column=c)
        cell.border = BORDER
        if fill:
            cell.fill = fill
        if font:
            cell.font = font
        cell.alignment = align if align else (CENTER if c == 1 else WRAP_TOP)


# ============================================================
# Sheet 1：全部工作节点
# ============================================================
ws1 = wb.active
ws1.title = "全部工作节点"

COLS1 = ["序号", "节点", "工作项", "内容", "出口准则", "状态", "备注"]
widths1 = [6, 16, 26, 52, 36, 10, 22]
for i, w in enumerate(widths1, 1):
    ws1.column_dimensions[get_column_letter(i)].width = w

ws1.merge_cells("A1:G1")
c = ws1.cell(row=1, column=1, value="倾转旋翼验证机飞控系统 — 工作计划总表")
c.font = FONT_TITLE
c.alignment = CENTER
ws1.row_dimensions[1].height = 28

# 表头
for i, h in enumerate(COLS1, 1):
    cell = ws1.cell(row=2, column=i, value=h)
    cell.font = FONT_HEADER
    cell.fill = FILL_HEADER
    cell.alignment = CENTER
    cell.border = BORDER
ws1.row_dimensions[2].height = 22

# 数据行: (序号, 节点, 工作项, 内容, 出口准则, 状态, 备注)
rows1 = [
    ("1.1", "节点一\n仿真收尾与\n机载软件初版", "转换过程高度损失优化",
     "转换过程存在一定高度损失，属升力/推力交接的固有现象，当前仿真结果处于可控、合理的工程范围；定位主要贡献因素并持续优化",
     "优化后确认高度损失处于安全裕度内", "进行中", "P7"),
    ("1.2", "", "反向转换策略完善",
     "返航/着陆过程中的减速-收倾转控制策略需完善，当前存在转换瞬间推力分配不连续问题",
     "反向转换高度波动处于安全裕度内", "进行中", "P3"),
    ("1.3", "", "起飞功率峰值压降",
     "起飞瞬间功率尖峰超出电池规格，方案明确（差动限制 + 起飞油门策略调整）",
     "全阶段功率控制在电池规格包络内", "方案明确", "P8，垂起验证前置"),
    ("1.4", "", "鲁棒性拉偏仿真",
     "按协议要求完成升阻特性、舵效、重量重心、风场等参数拉偏仿真",
     "拉偏矩阵全部通过（不发散、可恢复）", "计划中", "docs/df200_bias_sim.md"),
    ("1.5", "", "仿真测试报告",
     "汇总全流程科目、拉偏结果、问题归零记录",
     "报告齐套，随机载软件初版交付", "计划中", ""),

    ("2.1", "节点二\n半物理仿真与\n地面站", "半物理仿真联调",
     "真飞控硬件 + 仿真模型闭环，验证真实时序、输出频率、传感器链路、CAN 通信",
     "半物理仿真完整复现仿真结果", "计划中", ""),
    ("2.2", "", "地面站定制",
     "多模式任务规划、实时曲线、航线编辑、参数管理",
     "α 版可支撑验证机地面与飞行试验", "计划中", ""),

    ("3.1", "节点三\n安全功能完善", "转换中断回退机制",
     "当前转换保护以超时降级为主，需增加主动中断回退：转换过程中空速不足或姿态异常时，主动退回旋翼模式",
     "半物理仿真注入验证：可安全回旋翼", "开发中", "P2"),
    ("3.2", "", "异常保护完善",
     "高度异常保护、失速保护独立模块",
     "触发与恢复功能验证通过", "计划中", ""),
    ("3.3", "", "故障注入自动化测试",
     "上述全部故障场景建立自动化测试用例，纳入版本回归",
     "每次软件更新自动回归", "计划中", ""),

    ("4.1", "节点四\n验证机地面试验", "桌面联调",
     "飞控 + 真实舵机/电调 + 仿真模型联调，全接口遍历；拷机 ≥1 小时（CPU/温度/日志无异常）",
     "桌面联调报告", "下周启动", "十月计划②"),
    ("4.2", "", "全机地面联试",
     "航电、数传、动力、电源全系统联试；传感器全标定（IMU 温漂、罗盘、空速零点）；链路丢失自动返航验证",
     "联试报告，零遗留问题", "计划中", ""),

    ("5.1", "节点五\n验证机试飞", "旋翼模态首飞",
     "系留点火 → 低空悬停 → 平移/偏航/定点 → 自动起降",
     "姿态/位置/起降精度满足协议性能指标", "计划中", "★ 十月重点准备"),
    ("5.2", "", "转换与固定翼首飞",
     "低空短时转换（小角度倾转→回退）→ 完整转换 → 固定翼航线 → 反向转换 + 降落",
     "转换过程姿态与高度控制满足协议性能指标", "计划中", "P2/P3 完成后开放"),
    ("5.3", "", "全模态自动飞行",
     "自动起飞 → 航线 → 转换 → 前飞 → 反向转换 → 降落",
     "全模态控制律冻结，参数包冻结", "计划中", ""),

    ("6.1", "节点六\n合规与交付", "二次开发支持",
     "中文 API 文档、用户手册、示例工程", "—", "计划中", ""),
    ("6.2", "", "交付文档齐套",
     "协议要求的全部文档随阶段产出，末期只做收口定稿", "—", "计划中", ""),
]

r = 3
for item in rows1:
    for i, v in enumerate(item, 1):
        ws1.cell(row=r, column=i, value=v)
    status = item[5]
    if "下周启动" in status:
        fill = FILL_NOW
    elif "重点" in item[6]:
        fill = FILL_FOCUS
    else:
        fill = None
    style_row(ws1, r, len(COLS1), fill=fill)
    ws1.row_dimensions[r].height = 46
    r += 1

ws1.freeze_panes = "A3"

# ============================================================
# Sheet 2：十月工作计划
# ============================================================
ws2 = wb.create_sheet("十月工作计划")

COLS2 = ["序号", "时间节点", "工作项", "内容", "出口准则", "状态", "前置依赖", "关联风险/文档"]
widths2 = [6, 14, 22, 52, 34, 12, 20, 20]
for i, w in enumerate(widths2, 1):
    ws2.column_dimensions[get_column_letter(i)].width = w

ws2.merge_cells("A1:H1")
c = ws2.cell(row=1, column=1, value="十月工作计划 — 主线：桌面联调（下周）→ 线束合模 → 实机垂起验证")
c.font = FONT_TITLE
c.alignment = CENTER
ws2.row_dimensions[1].height = 28

for i, h in enumerate(COLS2, 1):
    cell = ws2.cell(row=2, column=i, value=h)
    cell.font = FONT_HEADER
    cell.fill = FILL_HEADER
    cell.alignment = CENTER
    cell.border = BORDER
ws2.row_dimensions[2].height = 22

# --- 分组标题行：已完成 ---
r = 3
ws2.merge_cells(start_row=r, start_column=1, end_row=r, end_column=8)
c = ws2.cell(row=r, column=1, value="一、已完成（垂起验证前置条件全部就绪）")
c.font = FONT_SECTION
c.fill = FILL_SECTION
c.alignment = WRAP_TOP
for col in range(1, 9):
    ws2.cell(row=r, column=col).border = BORDER
    ws2.cell(row=r, column=col).fill = FILL_SECTION
r += 1

rows2_done = [
    ("1", "已完成", "通信驱动调通",
     "飞控与舵机/电调通信驱动联调通过，桌面联调前置条件就绪",
     "—", "✅ 已完成", "—", ""),
    ("2", "已完成", "垂起动力模型落地",
     "按电机台架实测数据建立电机表模型，动力仿真与实机对齐，具备悬停油门/功率对照基准",
     "动力模型与台架数据一致", "✅ 已完成", "—", "P10 修正基础"),
    ("3", "已完成", "旋翼模态控制链闭环",
     "垂起爬升/悬停串级控制参数整定完成，纵向控制问题修复闭环，垂起爬升不掉高",
     "垂起爬升高度控制收敛、不掉高", "✅ 已完成", "—", ""),
    ("4", "已完成", "转换航向对齐与掉高优化",
     "垂起转平飞航向对齐门控验证通过；转换掉高主要贡献因素定位并完成优化",
     "航向对齐稳定复现；掉高处于安全裕度内", "✅ 已完成", "—", "P7"),
    ("5", "已完成", "反向转换链路验证",
     "前飞减速 → 收倾转 → 多旋翼进近反向链路完成多轮仿真迭代验证，衔接平稳",
     "反向转换高度波动处于安全裕度内", "✅ 已完成", "—", "P3"),
    ("6", "已完成", "垂起段功率分析",
     "垂起/巡航分段功率量化并与电池规格对照；起飞功率尖峰成因定位，压降方案明确",
     "功率走廊量化完成；P8 压降方案明确", "✅ 已完成", "—", "P8"),
]

for item in rows2_done:
    for i, v in enumerate(item, 1):
        ws2.cell(row=r, column=i, value=v)
    style_row(ws2, r, len(COLS2), fill=FILL_DONE)
    ws2.row_dimensions[r].height = 40
    r += 1

# --- 分组标题行：十月安排 ---
ws2.merge_cells(start_row=r, start_column=1, end_row=r, end_column=8)
c = ws2.cell(row=r, column=1, value="二、十月工作安排（重心：支撑实机垂起验证落地，对应节点五 5.1）")
c.font = FONT_SECTION
c.alignment = WRAP_TOP
for col in range(1, 9):
    ws2.cell(row=r, column=col).border = BORDER
    ws2.cell(row=r, column=col).fill = FILL_SECTION
r += 1

rows2_todo = [
    ("7", "下周启动", "桌面联调（节点 4.1）",
     "飞控 + 真实舵机/电调 + 仿真模型闭环联调，全接口遍历；重点验证真实链路时序、输出频率与通信稳定性",
     "拷机 ≥1 小时（CPU/温度/日志无异常），桌面联调报告", "▶ 下周启动", "① 通信驱动调通", ""),
    ("8", "桌面联调后", "线束合模与整机接入",
     "配合整机线束合模，完成机载电气、传感器与通信链路接入检查",
     "整机通电自检通过", "○ 待启动", "⑦ 桌面联调", "P11 振动/EMI 预案"),
    ("9", "线束合模后", "实机垂起验证支撑（月度重点）",
     "① 旋翼模态控制参数自仿真基线迁移；② 传感器标定（IMU 温漂、罗盘、空速零点）与动力上电检查；③ 起飞功率尖峰压降方案实机落地（P8/节点 1.3）；④ 垂起验证大纲与试验判据编制",
     "具备系留点火条件；垂起验证大纲齐套", "★ 月度重点", "⑧ 线束合模；仿真侧准备①-⑥全部完成", "P8/P10；节点 5.1"),
]

for item in rows2_todo:
    for i, v in enumerate(item, 1):
        ws2.cell(row=r, column=i, value=v)
    fill = FILL_NOW if "下周启动" in item[5] else FILL_FOCUS
    style_row(ws2, r, len(COLS2), fill=fill)
    ws2.row_dimensions[r].height = 52
    r += 1

# 补充说明行
r += 1
note_font = Font(name="微软雅黑", size=9, italic=True, color="595959")
notes = [
    "说明：",
    "① 垂起验证的仿真侧准备工作（动力模型、旋翼模态控制链、转换衔接、反向转换、功率分析）已全部完成，实机垂起验证条件就绪。",
    "② 工作重心为支撑垂起验证落地（节点五 5.1 旋翼模态首飞）；十月末出口为具备系留点火条件。",
    "③ P8（起飞功率尖峰压降）为系留点火前置项，仿真方案已明确，须在实机落地闭环（对应节点 1.3）。",
    "④ 传感器标定（IMU 温漂、罗盘、空速零点）对应节点 4.2，系留点火前完成。",
]
for note in notes:
    ws2.merge_cells(start_row=r, start_column=1, end_row=r, end_column=8)
    cell = ws2.cell(row=r, column=1, value=note)
    cell.font = note_font
    cell.alignment = WRAP_TOP
    r += 1

ws2.freeze_panes = "A3"

wb.save(OUT)
print(f"saved: {OUT}")
