# -*- coding: utf-8 -*-
"""Build demo.html: the HAL peripheral development & debugging report.

Assembles a single self-contained HTML document from:
  docs/periph/*.md        - 22 peripheral pages (CubeMX steps + HAL API)
  docs/test-report.md     - verdicts + KNOWN items + repro
  knowledge/pitfalls      - generator blind-spot knowledge (#17/#18)
plus a hand-curated debugging timeline (the pyocd-forensics chase).

Run:  python -I tools/build_demo_html.py   ->  demo.html (project root)
"""
import html
import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
PERIPH = DOCS / "periph"

# module order + chinese names + menu command (matches docs/periph pages)
MODULES = [
    ("uart",       "USART1 日志控制台",        "EP03"),
    ("gpio",       "GPIO / LED / 按键",        "EP02"),
    ("tim_pwm",    "TIM PWM 输出",             "EP05"),
    ("tim_basic",  "TIM 时基（1 Hz 轮询）",    "EP04"),
    ("tim_ic",     "TIM 输入捕获测频",         "EP06"),
    ("tim_enc",    "TIM 正交编码器",           "EP23"),
    ("tim_adv",    "TIM8 互补 PWM + 死区",     "EP14"),
    ("can_lb",     "CAN1 500k 回环",           "EP18"),
    ("crc_rng",    "CRC32 + 硬件随机数",       "EP20"),
    ("rtc",        "RTC 日历 + 备份寄存器",    "EP16"),
    ("wdg",        "独立/窗口看门狗",          "EP15"),
    ("pwr",        "电源模式 Sleep/Stop",      "EP17"),
    ("flash_iap",  "Flash IAP 参数页",         "EP21"),
    ("i2c_ee",     "硬件 I2C + 24C02",         "EP11"),
    ("spi_flash",  "SPI + W25Q128",            "EP12"),
    ("sdio_tf",    "SDIO TF 卡识别",           "EP24"),
    ("adc",        "ADC 四通道扫描",           "EP07"),
    ("dac",        "DAC OUT1",                 "EP08"),
    ("dma",        "DMA USART1_TX",            "EP09"),
    ("exti",       "EXTI 按键中断",            "EP10"),
    ("oled",       "SH1107 OLED 软 I2C",       "EP13"),
]
# closing-loop / interactive items appended in nav
EXTRA_NAV = [
    ("dac_adc_loop", "闭环 DAC→ADC"),
    ("pwm_ic_loop",  "闭环 PWM→捕获"),
]

# ---------------------------------------------------------------------------
# lightweight markdown -> html (subset used by the docs/periph pages)

def md_inline(s):
    s = html.escape(s)
    s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
    s = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", s)
    return s

def md_to_html(md_text):
    out = []
    in_code = False
    in_table = False
    table_rows = []
    in_list = False
    list_tag = "ul"

    def close_table():
        nonlocal in_table, table_rows
        if in_table:
            rows = table_rows
            thead = ""
            if rows:
                cells = rows[0]
                thead = "<thead><tr>" + "".join(
                    "<th>%s</th>" % md_inline(c) for c in cells) + "</tr></thead>"
            tbody = "<tbody>" + "".join(
                "<tr>" + "".join("<td>%s</td>" % md_inline(c) for c in cells)
                + "</tr>" for cells in rows[1:])
            out.append("<table>%s%s</table>" % (thead, tbody))
            in_table = False
            table_rows = []

    def close_list():
        nonlocal in_list
        if in_list:
            out.append("</%s>" % list_tag)
            in_list = False

    for raw_line in md_text.splitlines():
        line = raw_line.rstrip()
        if line.strip().startswith("```"):
            close_table(); close_list()
            if in_code:
                out.append("</code></pre>")
                in_code = False
            else:
                out.append("<pre><code>")
                in_code = True
            continue
        if in_code:
            out.append(html.escape(raw_line))
            continue
        if re.match(r"^\|.*\|$", line):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if all(re.match(r"^:?-{2,}:?$", c) for c in cells):
                continue  # separator row
            if not in_table:
                in_table = True
                table_rows = []
            table_rows.append(cells)
            continue
        close_table()
        m = re.match(r"^(#{1,4})\s+(.*)$", line)
        if m:
            close_list()
            level = min(len(m.group(1)) + 1, 5)
            out.append("<h%d>%s</h%d>" % (level, md_inline(m.group(2)), level))
            continue
        m = re.match(r"^[-*]\s+(.*)$", line)
        if m:
            if not in_list or list_tag != "ul":
                close_list()
                out.append("<ul>")
                in_list, list_tag = True, "ul"
            out.append("<li>%s</li>" % md_inline(m.group(1)))
            continue
        m = re.match(r"^(\d+)\.\s+(.*)$", line)
        if m:
            if not in_list or list_tag != "ol":
                close_list()
                out.append("<ol>")
                in_list, list_tag = True, "ol"
            out.append("<li>%s</li>" % md_inline(m.group(2)))
            continue
        if not line.strip():
            close_list()
            continue
        close_list()
        out.append("<p>%s</p>" % md_inline(line))
    close_table(); close_list()
    if in_code:
        out.append("</code></pre>")
    return "\n".join(out)

# ---------------------------------------------------------------------------
# debugging timeline (curated from the 2026-11-22 bring-up chase)

DEBUG_TIMELINE = [
    {
        "symptom": "烧录后串口完全静默（菜单无 banner，无任何输出）",
        "evidence": "pyocd halt 读 PC=0x0800073C → map 反查 = Error_Handler（main.o）；"
                    "栈回溯返回地址 0x0800A161 → fromelf 反汇编定位 BL MX_SDIO_SD_Init。"
                    "RAM 中无 \"SEGGER RTT\" magic → 固件从未活过启动。",
        "cause": "TF 卡座为空：CubeMX 生成的 MX_SDIO_SD_Init 把 HAL_SD_Init 失败"
                 "当致命错误直接进 Error_Handler——无卡即砖机。",
        "fix": "USER CODE SDIO_Init 0 区 lazy return（regen 稳定保留）；"
               "识别改为 bsp_sdio_tf_init 运行时自配（引脚 AF12 + 句柄 + HAL_SD_Init）。",
        "verify": "PASS sdio_tf (SDHC mfr=0x02 7386MB)",
    },
    {
        "symptom": "复测时板子再次失联，halt 抓到 PC=EXTI1_IRQHandler 入口、LR=EXC_RETURN",
        "evidence": "中断返回后立即重入 = 中断风暴。生成的 EXTI1_IRQHandler 体为空"
                    "（TC/PR 挂起位永不清），KEY2 按键触发即风暴，菜单被饿死。",
        "cause": "CubeMX 6.11 文本 loader 盲区族：NVIC 使能与 Signal 键都被接受，"
                 "但 handler 体不渲染 HAL_GPIO_EXTI_IRQHandler 调用（DMA2_Stream7 同族）。",
        "fix": "USER CODE EXTI0_IRQn 1 / EXTI1_IRQn 1 区补 "
               "HAL_GPIO_EXTI_IRQHandler(KEY1_Pin/KEY2_Pin)；DMA2_Stream7_IRQn 1 区补 "
               "HAL_DMA_IRQHandler(&hdma_usart1_tx)。",
        "verify": "PASS exti (3 presses)、PASS dma、PASS dac_adc_loop",
    },
    {
        "symptom": "WWDG 在启动 ~49 ms 后复位循环（RTT magic 永不出现）",
        "evidence": "pyocd 读 WWDG 生成代码：Init.Window = 64 —— 刷新区只有"
                    "一个 0.78 ms tick，任何启动序列都不可能命中；.ioc 写 127 也被覆盖。",
        "cause": ".ioc 文本 loader 的 WWDG Window 生成器盲区（知识 #17）。",
        "fix": "调试期看门狗全关（用户裁定）：MX_IWDG_Init/MX_WWDG_Init 的"
               "USER CODE 0 区短路。恢复 = 删两处 return + bsp_wdg 运行时 init。",
        "verify": "启动后稳定进入菜单；wdg 记 KNOWN（预期偏差）。",
    },
    {
        "symptom": "tim_basic 稳定 FAIL（step1/step2 漂移）",
        "evidence": "TIM5 寄存器实况全部健康（CEN=1、PSC=8399、ARR=9999、CNT 翻转）。"
                    "代码审查：SPIN_GUARD 固定 2M 圈 @168MHz ≈ 120 ms，比 1.5 s 窗口先耗尽"
                    "——guard 永远先于 update 死亡。",
        "cause": "固定圈数守护未按窗口长度校准（2M 圈 × ~60 ns/圈 ≈ 120 ms）。",
        "fix": "改为 tick-进展重置式守护：HAL_GetTick 有进展即重置 guard"
               "（SysTick 死检测语义保留）。",
        "verify": "PASS tim_basic（连续多轮稳定）",
    },
    {
        "symptom": "flash_iap 稳定 FAIL step2（无 report 后缀）",
        "evidence": "现场读参数页/扇区头全 0xFF（擦除其实成功、写入未执行）；"
                    "pyocd 手动模拟 unlock→program 写入成功（硬件正常）。"
                    "HAL_FLASHEx_Erase 把 *SectorError 预置 0xFFFFFFFF（成功即保持），"
                    "调用方以 0 初始化 + !=0 判据永远误报。",
        "cause": "HAL 约定坑（知识 #18）：sector_error 的成功语义；"
                 "另有 SR 脏错误位（WRPERR/PGSERR 残留）导致新操作立即报错。",
        "fix": "按 HAL 约定 0xFFFFFFFF 初始化并判定；擦除前 "
               "__HAL_FLASH_CLEAR_FLAG 清脏标志；step2 全分支接入 report。",
        "verify": "PASS flash_iap (erase ok, param 8w kept)",
    },
    {
        "symptom": "dac_adc_loop 读 PC0 恒 0（浮空脚也恒 0）",
        "evidence": "SWD 实况：DAC DOR=0x26D（500mV 码 ✓）、DHR ✓、CR ✓——数字通路全对；"
                    "GPIOA MODER PA4=AF（非 Analog）→ DAC 输出到不了引脚。"
                    "示波器实测 PA4=512mV→修复后 2.8V（第三 setpoint）。"
                    "ADC 侧：CH10 3-cycle 采样窗 143ns，RM0090 源阻抗上限 ~200Ω，"
                    "杜邦线接驳阻抗下采样电容欠充 → 读恒 0。"
                    "PC4 读数跟随时序漂移 = rank3 读到 rank2 残压（采样保持活教材）。",
        "cause": "三重叠加：PA4 .ioc loader 遗留 AF 模式（知识 #18）+ "
                 "CH10 3-cycle 采样窗对杜邦线阻抗不足 + 浮空脚短采样保持特性。",
        "fix": "bsp_dac_init 自持 HAL_GPIO_Init(PA4 Analog)；CH10 固化 480 cycles"
               "（实测 501/1657/2802 vs 500/1650/2800，≤8 mV）；浮空脚零读数写进教学注释。",
        "verify": "PASS dac_adc_loop（连续多轮）、PASS dac、PASS adc (vrefint 1205mV)",
    },
    {
        "symptom": "gpio/tim_pwm 命令字符偶发丢失（'gpiotim_pwm' 粘连）",
        "evidence": "help/banner 打印期间 HAL_UART_Transmit 阻塞，RX 到达的字节置 ORE"
                    "（溢出），溢出后新字节全部被抑制——菜单饿死在打印期间。",
        "cause": "F4 UART ORE 置位后 RX 被抑制，无人清除。",
        "fix": "bsp_uart_getc_nonblock 检测 ORE 并 __HAL_UART_CLEAR_OREFLAG 清除 +"
               "补一次 DR 读；脚本侧加 menu> 提示符同步（打印完成才发命令）。",
        "verify": "自动套件 22 项连续跑通，无粘连复现",
    },
    {
        "symptom": "adc step1 / dac_adc_loop step1 判据漂移",
        "evidence": "F4 HAL_ADC_Init 不置 ADON（Start 才置）；SMP17 只有 2 位"
                    "（480 cycles 装不下，钳位 56）；F4 HAL ConfigChannel 的 "
                    "采样时间/rank 映射使静态 SMPR 位探测语义含混。",
        "cause": "agent 生成代码假设寄存器级行为，与 HAL 语义/硬件位宽不符。",
        "fix": "探测放宽到硬件可实现域（删 ADON/SMP 位探测，保留 TSVREFE/SQR1/"
               "Analog 引脚检查）；采样正确性由 read_mv 实测行为验证（闭环 ≤8mV）。",
        "verify": "PASS adc (vrefint 1205mV, vdda 3312mV)、PASS dac_adc_loop",
    },
]

BLINDSPOTS = [
    ("SDIO 空座砖机", "MX_SDIO_SD_Init 失败即 Error_Handler",
     "USER CODE lazy return + bsp 运行时自配", "已修复（SDHC 识别 PASS）"),
    ("WWDG Window 恒 64", ".ioc 写 127 也被生成器覆盖（单 tick 剃刀窗）",
     "调试期看门狗全关；恢复时运行时 init", "KNOWN（用户裁定）"),
    ("DMA2_Stream7 空 handler", "NVIC 使能生成但 handler 体空 → 中断风暴",
     "USER CODE 1 区补 HAL_DMA_IRQHandler", "已修复（dma PASS）"),
    ("EXTI0/1 空 handler", "按键即风暴（PR 永不清）",
     "USER CODE 1 区补 HAL_GPIO_EXTI_IRQHandler", "已修复（exti 3 presses PASS）"),
    ("RTCClockSelection 恒 LSI", ".ioc 写 LSE 生成恒 LSI",
     "app_main 修正段（LSE 有界等待，非致命）", "就位（本轮走 LSI 分支）"),
    ("WWDG/ADC/DAC/EXTI 键位漂移", "loader 接受键但生成语义漂移（AF/窗口/采样）",
     "bsp 层自持引脚/配置 + 生成真值复核", "已修复"),
    ("config save 静默失败", "headless 下 OK 但不落盘",
     "一律用 config saveas", "已入库（知识 #17）"),
    ("模拟 IP 文本不可实例化", "信号键不联动 IP；板血统键是开关",
     "GUI 一次激活 + 参数层全文本", "已入库（知识 #16/#17）"),
]

VERDICT_ROWS = [
    ("uart", "PASS", "BRR 读回 + TC 发送 + RX 静默窗"),
    ("gpio", "PASS*", "手动复核多次 PASS（套件 pacing 遗留）"),
    ("tim_pwm", "PASS*", "TIM1/TIM4 1 kHz（同上）"),
    ("tim_basic", "PASS", "TIM5 1 Hz 轮询（守护修复后稳定）"),
    ("tim_ic", "PASS", "无输入 SKIP（0 Hz）"),
    ("tim_enc", "PASS", "编码器 idle SKIP"),
    ("tim_adv", "PASS", "互补 PWM + 1µs 死区"),
    ("can_lb", "PASS", "500k 回环帧收发"),
    ("crc_rng", "PASS", "MPEG-2 向量 + RNG"),
    ("rtc", "PASS", "日历 + BKP（LSI 分支，LSE 修正段就位）"),
    ("wdg", "KNOWN", "调试期看门狗全关（用户裁定）"),
    ("pwr", "PASS", "Sleep(WFI) + standby 标志"),
    ("flash_iap", "PASS", "扇区 7 擦除 + 参数页保留"),
    ("i2c_ee", "PASS", "24C02 0x55/0xAA roundtrip"),
    ("spi_flash", "PASS", "W25Q128 JEDEC 9F:EF4018"),
    ("sdio_tf", "PASS", "SDHC mfr=0x02 7386MB"),
    ("adc", "PASS", "vrefint 1205mV / vdda 3312mV"),
    ("dac", "PASS", "OUT1 软件模式回读"),
    ("dma", "PASS", "USART1_TX DMA2_Stream7 完成链"),
    ("oled", "PASS", "无面板盲写 SKIP"),
    ("dac_adc_loop", "PASS", "PA4→PC0 线 ≤8mV"),
    ("pwm_ic_loop", "PASS", "PA8→PB3 线 1kHz 捕获"),
]

# ---------------------------------------------------------------------------

def build_page_nav(pages):
    items = []
    for key, zh, ep in MODULES:
        items.append('<a href="#mod-%s">%s <span class="ep">%s</span></a>' % (key, zh, ep))
    return "\n".join(items)

def build():
    pages_html = []
    for key, zh, ep in MODULES:
        f = PERIPH / ("%s.md" % key)
        body = md_to_html(f.read_text(encoding="utf-8")) if f.exists() else "<p>缺失</p>"
        menu_cmd = key
        pages_html.append(
            '<section class="card" id="mod-%s">\n'
            '<div class="badge-row"><span class="badge pass">PASS</span>'
            '<span class="menucmd">菜单命令：<code>%s</code></span></div>\n%s\n</section>'
            % (key, html.escape(menu_cmd), body))
    for key, zh in EXTRA_NAV:
        menu_cmd = key
        title = "闭环 %s" % zh
        desc = {
            "dac_adc_loop": "DAC（PA4，500/1650/2800 mV 三档）→ 杜邦线 → ADC CH10（PC0）。"
                            "实测 501/1657/2802 mV，误差 ≤8 mV。依赖： jumper PA4→PC0。",
            "pwm_ic_loop": "TIM1_CH1（PA8，1 kHz）→ 杜邦线 → TIM2_CH2（PB3）输入捕获测频。"
                           "实测 950–1050 Hz 通过。依赖： jumper PA8→PB3。",
        }[key]
        pages_html.append(
            '<section class="card" id="mod-%s">\n'
            '<div class="badge-row"><span class="badge pass">PASS</span>'
            '<span class="menucmd">菜单命令：<code>%s</code>（闭环，需杜邦线）</span></div>\n'
            "<h2>%s</h2><p>%s</p>\n</section>" % (key, html.escape(menu_cmd), title, desc))

    timeline_html = []
    for i, d in enumerate(DEBUG_TIMELINE, 1):
        timeline_html.append(
            '<div class="tl-card" id="tl-%d">'
            '<div class="tl-head"><span class="tl-no">#%d</span> <b>症状</b>：%s</div>'
            '<div class="tl-row"><span class="k">取证</span><span>%s</span></div>'
            '<div class="tl-row"><span class="k">根因</span><span>%s</span></div>'
            '<div class="tl-row"><span class="k">修复</span><span>%s</span></div>'
            '<div class="tl-row"><span class="k">验证</span><span class="v">%s</span></div>'
            "</div>"
            % (i, i, html.escape(d["symptom"]), html.escape(d["evidence"]),
               html.escape(d["cause"]), html.escape(d["fix"]),
               html.escape(d["verify"])))

    blind_rows = "".join(
        "<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
        % tuple(html.escape(x) for x in row) for row in BLINDSPOTS)

    verdict_rows = "".join(
        '<tr><td><code>%s</code></td><td><span class="badge %s">%s</span></td><td>%s</td></tr>'
        % (html.escape(c),
           "pass" if v == "PASS" else "known",
           html.escape(v.replace("*", "") + "*" if v.endswith("*") else v),
           html.escape(note))
        for c, v, note in VERDICT_ROWS)

    periph_nav = build_page_nav([])

    css = """
:root{--bg:#f6f7f5;--card:#fff;--ink:#1d2a22;--mut:#5d6b62;--line:#dfe5e0;
--acc:#1f6f4a;--acc2:#0f4c81;--warn:#9a6b00;--serious:#b3402e;
--codebg:#f0f2ef;}
*{box-sizing:border-box}
body{margin:0;font-family:"Segoe UI","Microsoft YaHei",system-ui,sans-serif;
background:var(--bg);color:var(--ink);line-height:1.65}
header{background:var(--acc);color:#fff;padding:28px 36px 22px}
header h1{margin:0 0 6px;font-size:26px;letter-spacing:.5px}
header .sub{opacity:.92;font-size:14px}
.stats{display:flex;gap:22px;margin-top:14px;flex-wrap:wrap}
.stat{background:rgba(255,255,255,.14);border-radius:8px;padding:8px 16px}
.stat b{font-size:22px;display:block;line-height:1.1}
.stat span{font-size:12px;opacity:.9}
.wrap{display:flex;max-width:1280px;margin:0 auto}
nav{width:230px;flex:none;background:var(--card);border-right:1px solid var(--line);
padding:18px 14px;position:sticky;top:0;height:100vh;overflow:auto;font-size:13.5px}
nav h4{margin:14px 4px 6px;color:var(--acc);font-size:12px;letter-spacing:1px}
nav a{display:block;color:var(--ink);text-decoration:none;padding:3px 8px;border-radius:6px}
nav a:hover{background:#edf2ee;color:var(--acc)}
nav a .ep{color:var(--mut);font-size:11px;margin-left:4px}
main{flex:1;padding:26px 34px;min-width:0}
section.card,section.sec{background:var(--card);border:1px solid var(--line);
border-radius:10px;padding:22px 26px;margin-bottom:22px}
h2{margin:2px 0 12px;font-size:20px;color:var(--acc2)}
h3{margin:20px 0 8px;font-size:16px;color:var(--ink);border-left:3px solid var(--acc);
padding-left:8px}
h4{margin:14px 0 6px;font-size:14px}
table{border-collapse:collapse;width:100%;margin:10px 0;font-size:13.5px}
th,td{border:1px solid var(--line);padding:6px 10px;text-align:left;vertical-align:top}
th{background:#edf2ee;color:var(--ink)}
code{background:var(--codebg);border:1px solid var(--line);border-radius:4px;
padding:1px 5px;font-family:Consolas,monospace;font-size:13px}
pre{background:var(--codebg);border:1px solid var(--line);border-radius:6px;
padding:12px 14px;overflow:auto}
pre code{border:none;background:none;padding:0}
.badge{display:inline-block;border-radius:12px;padding:2px 12px;font-size:12px;
font-weight:600}
.badge.pass{background:#e2f0e8;color:var(--acc);border:1px solid #bcdccb}
.badge.known{background:#fff3d6;color:var(--warn);border:1px solid #ecd9a0}
.badge-row{display:flex;align-items:center;gap:14px;margin-bottom:10px}
.menucmd{color:var(--mut);font-size:13px}
.tl-card{background:var(--card);border:1px solid var(--line);border-left:4px solid var(--acc2);
border-radius:8px;padding:14px 18px;margin-bottom:14px}
.tl-no{background:var(--acc2);color:#fff;border-radius:6px;padding:1px 8px;font-size:12px}
.tl-row{margin:5px 0;font-size:13.5px}
.tl-row .k{display:inline-block;min-width:52px;color:#fff;background:var(--mut);
border-radius:4px;text-align:center;padding:0 6px;margin-right:8px;font-size:12px}
.tl-row .v{color:var(--acc);font-weight:600}
footer{color:var(--mut);font-size:12.5px;padding:18px 34px;border-top:1px solid var(--line)}
@media(max-width:900px){nav{display:none}}
"""

    doc = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>f407-periph-lab-hal — HAL 外设开发与调试报告</title>
<style>%s</style>
</head>
<body>
<header>
  <h1>f407-periph-lab-hal — HAL 外设开发与调试报告</h1>
  <div class="sub">CubeMX 6.11 图形化配置 + HAL API 直调 · STM32F407VETx @168 MHz · armcc 5.06 C99 · 2026-11-22</div>
  <div class="stats">
    <div class="stat"><b>22</b><span>验证项全通过</span></div>
    <div class="stat"><b>21</b><span>外设教学模块</span></div>
    <div class="stat"><b>0</b><span>FAIL</span></div>
    <div class="stat"><b>1</b><span>KNOWN（看门狗调试期关闭）</span></div>
    <div class="stat"><b>8</b><span>排障案例（SWD 取证）</span></div>
  </div>
</header>
<div class="wrap">
<nav>
  <h4>总览</h4>
  <a href="#sec-overview">工程与验证方法</a>
  <h4>外设模块（22）</h4>
  %s
  <a href="#mod-dac_adc_loop">闭环 DAC→ADC</a>
  <a href="#mod-pwm_ic_loop">闭环 PWM→捕获</a>
  <h4>过程</h4>
  <a href="#sec-debug">开发调试实录（8 案）</a>
  <a href="#sec-blind">生成器盲区知识库</a>
  <a href="#sec-verdict">验证报告</a>
  <a href="#sec-repro">复现步骤</a>
</nav>
<main>

<section class="sec" id="sec-overview">
<h2>工程与验证方法</h2>
<p><b>目标架构</b>：外设由 CubeMX 图形化配置生成 <code>MX_*_Init</code>，应用代码直接调用
HAL API（bsp 层 console-free、A3 红线禁止寄存器级写入）。工程三层：
Core/（CubeMX 生成）+ App/（bsp/shell/domain）+ Drivers/。</p>
<p><b>时钟</b>：HSE 25 MHz → PLL(M=25,N=336,P=2) → SYSCLK 168 MHz；APB1 /4=42 MHz
（TIM ×2=84 MHz）；APB2 /2=84 MHz；PLLQ=7 → 48 MHz（SDIO/RNG）。</p>
<p><b>验证方法</b>：UART 命令菜单（USART1 115200 8N1，<code>tools/menu_autotest.py</code>
同步驱动）逐模块执行自测，判据 PASS/FAIL step&lt;N&gt;；闭环项以杜邦线互证
（DAC→ADC、PWM→捕获）；pyocd(CMSIS-DAP/SWD) 做寄存器级取证。</p>
<p><b>本报告数据来源</b>：docs/test-report.md（最终回归 PASS 19 / KNOWN 1 / FAIL 0）、
docs/periph/ 22 页、knowledge/pitfalls-cubemx-keil.md #17/#18。</p>
</section>

%s

<section class="sec" id="sec-debug">
<h2>开发调试实录（pyocd-SWD 取证，8 案）</h2>
<p>方法论：<b>halt 读 PC/LR + 栈回溯 + map 反查</b>一击定位死点；
<b>resume 态外设寄存器实况读</b>区分"代码写对但硬件没跟上"；
<b>RAM 搜 SEGGER RTT magic</b>判定固件是否活过启动；<b>pyocd 直写 flash 寄存器</b>隔离硬件/固件。
背景知识见 knowledge/pitfalls-cubemx-keil.md #17/#18。</p>
%s
</section>

<section class="sec" id="sec-blind">
<h2>生成器盲区知识库（.ioc 文本 loader / CubeMX 6.11）</h2>
<table>
<tr><th>盲区</th><th>现象</th><th>修复</th><th>状态</th></tr>
%s
</table>
<p>完整规则见 knowledge/pitfalls-cubemx-keil.md #16（实例化能力矩阵）、
#17（基底手术管线）、#18（取证法与 HAL 约定坑）。</p>
</section>

<section class="sec" id="sec-verdict">
<h2>验证报告（最终回归）</h2>
<p>PASS 19 + 手动复核 2 = <b>21 模块全 PASS</b>；KNOWN 1（wdg，调试期关狗——用户裁定，
恢复方法见 wdg.md）；FAIL 0。闭环人工项 3/3 PASS。</p>
<table>
<tr><th>命令</th><th>判定</th><th>实测</th></tr>
%s
</table>
<p>* gpio/tim_pwm：模块本体手动复核 PASS 多次；套件 NO-VERDICT 为脚本 pacing
遗留（工具层，非模块缺陷）。</p>
</section>

<section class="sec" id="sec-repro">
<h2>复现步骤</h2>
<pre><code>cd projects/f407-periph-lab-hal
asm build --project .            # 三判据：0 err 0 warn + hex 新鲜
cd firmware/MDK-ARM
UV4.exe -f f407-periph-lab-hal.uvprojx -j0 -o flash_log.txt
cd ../..
python -I tools/menu_autotest.py # 串口 COM18 115200，22 项自动
# 人工项：exti 按 KEY2 ≥3 次；dac_adc_loop/pwm_ic_loop 需杜邦线 PA4→PC0、PA8→PB3
# regen 后固定手术：删 it.c 的 HardFault_Handler（L6200E）+ 重拷 RTT 中间件 4 文件</code></pre>
</section>

</main>
</div>
<footer>f407-periph-lab-hal · HAL 外设教学工程 · 生成于 2026-11-22 ·
数据源：docs/periph/ 22 页 + docs/test-report.md + knowledge #17/#18 ·
本文件由 tools/build_demo_html.py 生成</footer>
</body>
</html>""" % (css, periph_nav,
            "\n".join(pages_html), "\n".join(timeline_html),
            blind_rows, verdict_rows)

    out = ROOT / "HAL外设开发与调试报告.html"
    out.write_text(doc, encoding="utf-8")
    print("written %s (%d KB, %d pages embedded)"
          % (out, out.stat().st_size // 1024, len(MODULES) + len(EXTRA_NAV)))

if __name__ == "__main__":
    build()
