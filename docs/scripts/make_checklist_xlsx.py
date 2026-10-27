#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成低压联调清单与物料表 Excel（多 sheet，与 md 版对应）"""
import openpyxl
from openpyxl.styles import Font, Alignment, PatternFill, Border, Side
from openpyxl.utils import get_column_letter

OUT = "/home/codyqin/Desktop/git/ardupilot/docs/低压联调清单与物料表.xlsx"

wb = openpyxl.Workbook()

# ---------- 样式 ----------
FONT_TITLE = Font(name="微软雅黑", size=13, bold=True)
FONT_HEADER = Font(name="微软雅黑", size=10, bold=True, color="FFFFFF")
FONT_BODY = Font(name="微软雅黑", size=10)
FONT_SECTION = Font(name="微软雅黑", size=10.5, bold=True, color="2F5496")
FONT_NOTE = Font(name="微软雅黑", size=9, italic=True, color="595959")

FILL_HEADER = PatternFill("solid", fgColor="2F5496")
FILL_SECTION = PatternFill("solid", fgColor="D9E2F3")
FILL_KEY = PatternFill("solid", fgColor="FFF2CC")   # 重点日
FILL_DONE = PatternFill("solid", fgColor="E2EFDA")

THIN = Side(style="thin", color="B0B0B0")
BORDER = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)
CENTER = Alignment(wrap_text=True, vertical="center", horizontal="center")
LEFT_TOP = Alignment(wrap_text=True, vertical="top", horizontal="left")


# ============================================================
# Sheet 0：日更新计划（与 docs/低压联调日更新计划.md 对应）
# ============================================================
ws = wb.active
ws.title = "日更新计划"
ws.sheet_view.showGridLines = False


def section_header(ws, r, text, ncols):
    """浅蓝分组标题行"""
    ws.merge_cells(start_row=r, start_column=1, end_row=r, end_column=ncols)
    c = ws.cell(row=r, column=1, value=text)
    c.font = FONT_SECTION
    c.alignment = Alignment(vertical="center", horizontal="left")
    for col in range(1, ncols + 1):
        ws.cell(row=r, column=col).fill = FILL_SECTION
        ws.cell(row=r, column=col).border = BORDER
    return r + 1


def day_table(ws, r, title, items, ncols=3):
    """单日计划表：标题行 + 工作项表。items = [(工作项, 内容, 出口)]"""
    ws.merge_cells(start_row=r, start_column=1, end_row=r, end_column=ncols)
    c = ws.cell(row=r, column=1, value=title)
    c.font = Font(name="微软雅黑", size=10.5, bold=True)
    c.alignment = Alignment(vertical="center", horizontal="left")
    r += 1
    for i, h in enumerate(["工作项", "内容", "出口", "完成✔"], 1):
        cc = ws.cell(row=r, column=i, value=h)
        cc.font = FONT_HEADER
        cc.fill = FILL_HEADER
        cc.alignment = CENTER
        cc.border = BORDER
    r += 1
    for name, content, exit_ in items:
        for i, v in enumerate([name, content, exit_, ""], 1):
            cc = ws.cell(row=r, column=i, value=v)
            cc.font = FONT_BODY
            cc.border = BORDER
            cc.alignment = LEFT_TOP
        ws.row_dimensions[r].height = 30
        r += 1
    return r


widths = [22, 56, 26, 8]
for i, w in enumerate(widths, 1):
    ws.column_dimensions[get_column_letter(i)].width = w

ws.merge_cells("A1:D1")
c = ws.cell(row=1, column=1, value="低压联调日更新计划（10.09–10.16）— 目标：10.16 前完成桌面联调（节点 4.1）")
c.font = FONT_TITLE
c.alignment = Alignment(vertical="center", horizontal="left")
ws.row_dimensions[1].height = 26

r = 3
r = section_header(ws, r, "前提：通信驱动已调通｜范围：舵机、倾转舵机、电机（电调）、BMS 全链路遍历｜整机联调时间待总体协调，桌面联调先行不阻塞", 4)
r += 1

DAYS = [
    ("📅 10.09（今天，周五）— 方案确认与物料准备", [
        ("联调方案确认", "确认低压联调接口清单、链路分配（CAN/PWM/串口）、测试科目与日更新机制", "接口清单冻结"),
        ("物料清点", "舵机×N、倾转舵机×N、电机+电调×N、BMS、线束、电源、CAN 卡确认到位", "缺料清单发出"),
        ("上位机准备", "确认舵机/倾转舵机/BMS/电调各自上位机可用，无法获取的列为风险", "上位机到位确认表"),
        ("环境搭建", "桌面联调工位布置：飞控+调试器、CAN 分析仪、直流电源、上位机 PC", "工位就绪"),
    ]),
    ("📅 10.10（明天，周六）— BMS + 舵机（重点日）", [
        ("舵机刷固件", "全部舵机刷最新固件，配置 Servo ID（不重复、与安装位对应表登记）", "全部舵机固件+ID 完成并登记"),
        ("舵机上位机验证", "上位机逐个遍历舵机：行程、方向、堵转保护、回读", "单体全部通过"),
        ("BMS 协议联调", "BMS 上电，飞控/BMS 上位机对通：电压/电流/SOC/温度遥测逐项核对", "BMS 遥测帧全部读通"),
        ("BMS 告警联测", "过压/欠压/过流/过温告警阈值注入验证", "告警与恢复逻辑通过"),
        ("日结", "更新日计划：完成项打勾、问题清单、次日计划", "日更新发出"),
    ]),
    ("📅 10.11（周日）— 机动/缓冲日", [
        ("问题清零", "处理 10.10 遗留问题（协议不符、ID 冲突、告警误报等）", "遗留问题闭环或转风险"),
        ("电机电调预准备", "电机/电调固件刷新预研、CAN 节点 ID 分配表编制、正反转定义确认", "电调预准备完成"),
        ("倾转舵机预准备", "倾转舵机固件确认、多帧协议联调预案（命令 2 帧/反馈 4 帧、CRC 算例已闭环）", "倾转联调预案就绪"),
    ]),
    ("📅 10.12（周一）— 电机/电调联调（重点日）", [
        ("电调固件刷新", "全部电调刷最新固件，按分配表配置 CAN 节点 ID", "全部电调 ID 完成并登记"),
        ("电机单体测试", "上位机/飞控逐台测试：正反转、油门-转速响应、回读", "单体全部通过"),
        ("多电机同步", "多台电机油门同步性、响应一致性测试", "同步误差在指标内"),
        ("飞控侧接入", "飞控 CAN 口实际下发油门指令，电调回读验证", "飞控↔电调链路通"),
        ("日结", "更新日计划", "日更新发出"),
    ]),
    ("📅 10.13（周二）— 倾转舵机联调（重点日）", [
        ("倾转舵机单体", "上位机测试倾转角指令/角度反馈，CRC 校验双向闭环", "单体全部通过"),
        ("多帧收发", "命令 2 帧/反馈 4 帧多帧机制联调（协议已对齐厂商 v0）", "多帧收发稳定"),
        ("飞控侧接入", "飞控下发倾转角指令，倾转舵机反馈回读核对", "飞控↔倾转链路通"),
        ("日结", "更新日计划", "日更新发出"),
    ]),
    ("📅 10.14（周三）— 飞控集成遍历 + 联动测试", [
        ("全接口遍历", "飞控逐一驱动全部外设：舵面舵机×N、倾转舵机×N、电机×N、BMS", "接口遍历表全勾"),
        ("混合链路稳定性", "全外设同时挂线运行 ≥30min，观察总线负载、丢帧、重启", "无丢帧/无重启"),
        ("参数预置", "舵机行程/方向、倾转限位、电机序号、BMS 告警参数写入飞控", "参数表初版固化"),
        ("日结", "更新日计划", "日更新发出"),
    ]),
    ("📅 10.15（周四）— 拷机与联调报告", [
        ("拷机 ≥1 小时", "全系统（飞控+全部外设）连续运行 ≥1h：CPU/温度/日志检查，无异常", "拷机记录"),
        ("问题回归", "全部问题闭环确认，遗留项转风险清单", "问题清单归零或转风险"),
        ("桌面联调报告", "汇总接口遍历表、拷机记录、问题归零记录", "报告初版发出"),
    ]),
    ("📅 10.16（周五）— 报告定稿与状态评审", [
        ("联调报告定稿", "报告评审、修订、定稿", "报告齐套 ✅"),
        ("节点确认", "桌面联调（节点 4.1）确认完成", "节点关闭"),
        ("整机联调对齐", "向总体提交联调结论，配合整机联调时间协调", "整机联调窗口确认"),
    ]),
]

for title, items in DAYS:
    r = day_table(ws, r, title, items)
    r += 1

# 风险与预案
r = section_header(ws, r, "风险与预案", 4)
r += 1
for i, h in enumerate(["#", "风险", "预案", ""], 1):
    cc = ws.cell(row=r, column=i, value=h)
    cc.font = FONT_HEADER
    cc.fill = FILL_HEADER
    cc.alignment = CENTER
    cc.border = BORDER
r += 1
RISKS = [
    ("R1", "舵机/电调/BMS 上位机未到位或版本不符", "10.09 清点时确认，缺位当天升级风险并协调厂家远程支持"),
    ("R2", "厂家协议与飞控实现不符", "以 10.09 冻结的接口清单为准，当天内升级至厂家对齐"),
    ("R3", "电机多机同步不达标", "预留 10.11 机动日缓冲；必要时调整同步策略或替换个体"),
    ("R4", "硬件个体故障（坏件）", "备件周转，坏件隔离登记"),
    ("R5", "整机联调时间提前", "桌面联调按日计划推进不压缩；整机窗口确认后只做增量适配"),
]
for num, risk, plan in RISKS:
    for i, v in enumerate([num, risk, plan, ""], 1):
        cc = ws.cell(row=r, column=i, value=v)
        cc.font = FONT_BODY
        cc.border = BORDER
        cc.alignment = LEFT_TOP
    r += 1

r += 1
r = section_header(ws, r, "日更新机制", 4)
r += 1
MECH = [
    "每日下班前：更新本计划——完成项打勾、新增问题进风险表、编制次日计划并发出；",
    "问题记录格式：日期 / 对象 / 现象 / 归因 / 处置 / 状态（开放/闭环）；",
    "里程碑检查点：10.10（BMS+舵机通）、10.12（电机通）、10.13（倾转通）、10.14（全接口遍历）、10.15（拷机通过）、10.16（报告定稿，节点关闭）。",
]
for m in MECH:
    ws.merge_cells(start_row=r, start_column=1, end_row=r, end_column=4)
    c = ws.cell(row=r, column=1, value=m)
    c.font = FONT_BODY if not m.startswith("里程碑") else Font(name="微软雅黑", size=10, bold=True, color="C00000")
    c.alignment = LEFT_TOP
    r += 1

# ---------- 样式（见文件顶部统一定义） ----------


def write_table(ws, start_row, headers, rows, widths, header_fill=FILL_HEADER):
    """写一个标准表格，返回下一可用行号"""
    for i, w in enumerate(widths, 1):
        ws.column_dimensions[get_column_letter(i)].width = max(
            ws.column_dimensions[get_column_letter(i)].width or 0, w)
    for i, h in enumerate(headers, 1):
        c = ws.cell(row=start_row, column=i, value=h)
        c.font = FONT_HEADER
        c.fill = header_fill
        c.alignment = CENTER
        c.border = BORDER
    ws.row_dimensions[start_row].height = 20
    r = start_row + 1
    for row in rows:
        for i, v in enumerate(row, 1):
            c = ws.cell(row=r, column=i, value=v)
            c.font = FONT_BODY
            c.border = BORDER
            c.alignment = CENTER if (i == 1 and len(str(v)) <= 4) else LEFT_TOP
        r += 1
    return r


def add_title(ws, row, text, ncols):
    ws.merge_cells(start_row=row, start_column=1, end_row=row, end_column=ncols)
    c = ws.cell(row=row, column=1, value=text)
    c.font = FONT_SECTION
    c.alignment = Alignment(vertical="center", horizontal="left")
    return row + 1


# ============================================================
# Sheet 1：使用索引 + 物料清单
# ============================================================
ws = wb.create_sheet("物料清单")
ws.sheet_view.showGridLines = False

r = add_title(ws, 1, "低压联调 — 物料清单（10.09 清点，缺料当天发出清单）", 8)
r += 1

headers = ["#", "类别", "名称", "型号/规格", "数量", "用途", "到位", "备注"]
widths = [5, 8, 24, 16, 10, 20, 8, 24]
rows = [
    (1, "飞控", "飞控主机", "待填", 1, "联调主控", "", "含 SD 卡、底板"),
    (2, "飞控", "调试器（J-Link/ST-Link）", "待填", 1, "固件烧录/调试", "", ""),
    (3, "外设", "舵面舵机", "待填", "待填", "副翼/升降/方向", "", "数量按实机清单"),
    (4, "外设", "倾转舵机", "待填", "待填", "左右电机舱倾转", "", "厂商 v0 协议（已对齐）"),
    (5, "外设", "电机 + 电调", "待填", "待填", "动力组", "", "电调支持 DroneCAN"),
    (6, "外设", "BMS", "待填", 1, "电池管理遥测", "", "备用是否到位？"),
    (7, "通信", "CAN 分析仪 / USB-CAN 卡", "待填", 1, "总线抓包/调试", "", ""),
    (8, "通信", "CAN 终端电阻 120Ω", "待填", "≥2", "总线端接", "", ""),
    (9, "电源", "直流稳压电源", "待填", 1, "低压联调供电", "", "电压范围待确认"),
    (10, "电源", "供电线缆/连接器（XT60 等）", "待填", "若干", "动力/低压供电", "", ""),
    (11, "线束", "CAN 线束/分支线", "待填", "若干", "总线连接", "", "含屏蔽线"),
    (12, "线束", "舵机/传感器延长线、连接器", "待填", "若干", "外设接入", "", ""),
    (13, "工装", "上位机 PC", "待填", 2, "外设上位机+地面站", "", "一用一备"),
    (14, "仪表", "万用表", "待填", 1, "电气检查", "", ""),
    (15, "仪表", "示波器（可选）", "待填", 1, "信号质量/时序", "", ""),
    (16, "辅料", "标签纸、扎带、热缩管、紧固件", "—", "若干", "线束标识与固定", "", "ID 对应表贴签"),
    (17, "软件", "各外设上位机安装包+固件包", "版本待登记", "—", "10.09 确认", "", "版本登记入上位机表"),
    (18, "软件", "飞控固件（含联调分支）", "待填", "—", "联调基线", "", "版本号登记"),
]
r = write_table(ws, r, headers, rows, widths)
r += 1
notes = [
    "说明：① '到位'列 10.09 现场打勾；② 缺料项当天发出缺料清单并升级协调；③ 型号/数量按实机清单确认后冻结。",
]
for n in notes:
    ws.merge_cells(start_row=r, start_column=1, end_row=r, end_column=8)
    c = ws.cell(row=r, column=1, value=n)
    c.font = FONT_NOTE
    c.alignment = LEFT_TOP
    r += 1

# ============================================================
# Sheet 2：上位机到位确认 + 通信接口清单
# ============================================================
ws = wb.create_sheet("上位机与接口")
ws.sheet_view.showGridLines = False

r = add_title(ws, 1, "上位机到位确认表（10.09，风险 R1：缺位当天升级协调厂家）", 6)
r += 1
headers = ["对象", "上位机名称/版本", "来源", "已安装", "功能可用", "备注"]
widths = [14, 24, 18, 10, 12, 30]
rows = [
    ("舵面舵机", "", "", "", "", "需支持固件刷新 + Servo ID 配置"),
    ("倾转舵机", "", "", "", "", "需支持多帧收发、CRC 配置"),
    ("电机/电调", "", "", "", "", "需支持节点 ID 配置、正反转设置"),
    ("BMS", "", "", "", "", "需支持遥测读取、告警阈值配置"),
    ("飞控地面站", "Mission Planner / QGC", "开源", "", "", "参数写入、日志下载"),
]
r = write_table(ws, r, headers, rows, widths)

r += 2
r = add_title(ws, r, "通信接口清单（10.09 冻结，风险 R2 处置基准：厂家协议不符时以本表为准）", 8)
r += 1
headers = ["#", "对象", "飞控接口", "协议", "波特率", "设备数", "关键报文/信号", "备注"]
widths = [5, 12, 14, 16, 10, 8, 34, 26]
rows = [
    (1, "舵面舵机", "CAN", "DroneCAN", "", "", "指令/位置反馈", "Servo ID 见 ID 分配表"),
    (2, "倾转舵机", "CAN", "厂商 v0 自定义", "", "", "倾转角指令（2帧）/反馈（4帧）、CRC", "协议已对齐，CRC 算例已闭环"),
    (3, "电机/电调", "CAN", "DroneCAN", "", "", "油门指令/转速反馈", "节点 ID 见 ID 分配表"),
    (4, "BMS", "CAN", "待填（厂家协议）", "", "", "电压/电流/SOC/温度/告警", "遥测帧清单待对齐"),
]
r = write_table(ws, r, headers, rows, widths)

# ============================================================
# Sheet 3：ID 分配表（Servo ID + CAN 节点 ID）
# ============================================================
ws = wb.create_sheet("ID分配表")
ws.sheet_view.showGridLines = False

r = add_title(ws, 1, "Servo ID 分配登记表（10.10，刷固件后逐台登记，ID 不重复，贴签对应安装位）", 9)
r += 1
headers = ["舵机编号", "安装位置", "Servo ID", "固件版本", "行程 min/max", "方向（正/反）", "刷机日期", "执行人", "备注"]
widths = [10, 16, 10, 12, 14, 14, 12, 10, 16]
rows = [(f"S{i:02d}", "", "", "", "", "", "", "", "") for i in range(1, 7)]
r = write_table(ws, r, headers, rows, widths)

r += 2
r = add_title(ws, r, "CAN 节点 ID 分配表（10.11 编制 / 10.12 执行；建议分段：电调 10-19、舵机 20-29、倾转 30-39、BMS 40+）", 9)
r += 1
headers = ["设备类型", "设备编号", "安装位置", "CAN 节点 ID", "固件版本", "正/反转定义", "刷机日期", "执行人", "备注"]
rows = [("电调", f"M{i:02d}", "", "", "", "", "", "", "") for i in range(1, 7)]
rows += [("倾转舵机", "T01", "左电机舱", "", "", "—", "", "", "厂商 v0 协议"),
         ("倾转舵机", "T02", "右电机舱", "", "", "—", "", "", "厂商 v0 协议"),
         ("BMS", "B01", "—", "", "", "—", "", "", "")]
r = write_table(ws, r, headers, rows, widths)

# ============================================================
# Sheet 4：接口遍历表
# ============================================================
ws = wb.create_sheet("接口遍历表")
ws.sheet_view.showGridLines = False

r = add_title(ws, 1, "接口遍历表（10.14，飞控逐一驱动全部外设，任一格不过登记问题表）", 8)
r += 1
headers = ["设备", "指令下发", "反馈回读", "行程/限位", "方向确认", "异常注入（超时/断线）", "结果", "备注"]
widths = [10, 12, 12, 12, 12, 20, 10, 22]
rows = []
rows += [(f"舵机 S{i:02d}", "", "", "", "", "", "", "") for i in range(1, 7)]
rows += [("倾转 T01", "", "", "", "", "", "", "多帧+CRC"),
         ("倾转 T02", "", "", "", "", "", "", "多帧+CRC")]
rows += [(f"电机 M{i:02d}", "", "", "—", "", "", "", "含正反转") for i in range(1, 7)]
rows += [("BMS B01", "—（遥测）", "", "—", "—", "", "", "告警注入")]
r = write_table(ws, r, headers, rows, widths)
r += 1
ws.merge_cells(start_row=r, start_column=1, end_row=r, end_column=8)
c = ws.cell(row=r, column=1, value="说明：电机无行程/限位项（用正反转代替）；BMS 为遥测对象，无指令下发项，重点验证告警注入与恢复。")
c.font = FONT_NOTE
c.alignment = LEFT_TOP

# ============================================================
# Sheet 5：参数预置表
# ============================================================
ws = wb.create_sheet("参数预置表")
ws.sheet_view.showGridLines = False

r = add_title(ws, 1, "参数预置表（10.14，参数表初版固化）", 6)
r += 1
headers = ["参数名", "对象", "值", "来源", "写入确认", "备注"]
widths = [28, 16, 12, 18, 10, 30]
rows = [
    ("舵机行程 MIN/MAX", "各舵机", "", "手册", "", "对应 Servo ID 登记表行程值"),
    ("舵机方向 REV", "各舵机", "", "联调实测", "", ""),
    ("倾转限位", "倾转 T01/T02", "", "设计值", "", ""),
    ("电机序号映射", "电调 M01-M06", "", "实机布置", "", ""),
    ("BMS 告警阈值（过压/欠压/过流/过温）", "BMS", "", "手册/协议", "", ""),
    ("CAN 驱动使能/速率", "飞控", "", "接口清单", "", "与接口清单冻结值一致"),
]
r = write_table(ws, r, headers, rows, widths)

# ============================================================
# Sheet 6：问题记录 + 拷机记录
# ============================================================
ws = wb.create_sheet("问题与拷机记录")
ws.sheet_view.showGridLines = False

r = add_title(ws, 1, "问题记录表（每日更新；10.15 全部闭环或转风险清单）", 8)
r += 1
headers = ["编号", "日期", "对象", "现象", "归因", "处置", "状态", "闭环日期"]
widths = [8, 10, 12, 30, 24, 24, 8, 12]
rows = [(f"Q{i:02d}", "", "", "", "", "", "开放", "") for i in range(1, 11)]
r = write_table(ws, r, headers, rows, widths)

r += 2
r = add_title(ws, r, "10.14 混合链路稳定性记录（≥30min，全外设挂线）", 6)
r += 1
headers = ["检查项", "判据", "0min", "15min", "30min", "结果"]
widths = [22, 18, 10, 10, 10, 14]
rows = [
    ("CAN 总线负载", "在设计范围内", "", "", "", ""),
    ("丢帧/错误帧计数", "0", "", "", "", ""),
    ("设备重启/掉线次数", "0", "", "", "", ""),
    ("飞控状态", "无异常告警", "", "", "", ""),
]
r = write_table(ws, r, headers, rows, widths)

r += 2
r = add_title(ws, r, "10.15 拷机记录（≥1h，全系统）", 6)
r += 1
headers = ["检查项", "判据", "0min", "30min", "60min", "结果"]
rows = [
    ("CPU 占用", "无持续满载", "", "", "", ""),
    ("飞控温度", "在规格内", "", "", "", ""),
    ("日志异常条数", "0（记录文件名）", "", "", "", ""),
    ("通信链路", "全部在线", "", "", "", ""),
    ("重启/掉线", "0", "", "", "", ""),
]
r = write_table(ws, r, headers, rows, widths)

r += 2
ws.merge_cells(start_row=r, start_column=1, end_row=r, end_column=6)
c = ws.cell(row=r, column=1,
            value="拷机开始/结束时间：________～________；执行人：________；日志文件：________")
c.font = FONT_BODY
c.alignment = LEFT_TOP

wb.save(OUT)
print(f"saved: {OUT}")
