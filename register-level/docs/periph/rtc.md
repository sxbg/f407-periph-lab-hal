# bsp_rtc — RTC 日历 + 备份域（EP16）

寄存器级 RTC：LSE 32.768 kHz 日历、Alarm A、20×BKPxR 备份寄存器。菜单命令 `rtc`
（带 last_report；LSE 超时=SKIP 语义）。RTC **不在** .ioc（备份域整链 PWR DBP →
BDCR → RTC 寄存器全归模块所有）。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| Clock Source | LSE（32.768 kHz，板焊晶振 PC14/PC15） | EP-facts：OSC32 在板上；RTCSEL=LSE |
| Calendar | 日历 + 1 Hz 走时 | INIT/INITF 初始化序列 + PRER=127:255 |
| Alarm A | 每分钟秒字段匹配 | ALRMAR MSK4-2 屏蔽时分秒高位 |
| Backup Registers | 20×32 位 BKPxR | CR1220（VBAT）供电，全复位存续 |
| Wakeup / Tamper | 未用（Tamper 脚 PC13 被 W25Q 占） | pin-budget 冲突 #9 |
| NVIC | RTC_Alarm_IRQn(41) off | 轮询（ISR.ALRAF 软件看） |
| PC14/PC15 | 模拟（晶振），无 GPIO 配置 | LSE 驱动 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PC14 | — | 模拟（OSC32_IN） | 32.768 kHz 晶振 |
| PC15 | — | 模拟（OSC32_OUT） | 同上 |
| VBAT | — | 电源 | CR1220 纽扣电池 |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | PWREN bit28 | PWR 时钟（DBP 前置） |
| PWR->CR | DBP bit8 | 备份域写保护解除 |
| RCC->BDCR | LSEON / LSERDY / RTCSEL=10(LSE) / RTCEN | 备份域时钟选择与使能（4 s 就绪窗） |
| RTC->WPR | 0xCA→0x53 | 写保护钥匙序列 |
| RTC->ISR | INIT / INITF / RSF / ALRAF | 初始化模式握手、影子同步、报警标志 |
| RTC->PRER | PREDIV_A=127 / PREDIV_S=255 | 32768/(128×256)=1 Hz |
| RTC->TR / DR | BCD 时分秒 / 年月日星期 | 影子寄存器读写（RSF 后才可信） |
| RTC->CR | ALRAE=1（ALRAIE=0） | 报警使能（轮询） |
| RTC->ALRMAR | MSK4/MSK3/MSK2=1 | 只比秒字段 |
| RTC->BKPxR（20 个） | 32 位自由 | "这次上电是谁让的"标记载体（EP05/EP15 联动） |

## 对应视频集数

EP16（RTC 实时时钟与备份域）。
