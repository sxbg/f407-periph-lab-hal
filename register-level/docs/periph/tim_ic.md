# bsp_tim_ic — TIM2 输入捕获频率计（EP13）

寄存器级输入捕获：TIM2_CH2=PB3（TI2 直连），32 位计数器 1 s 窗口测频。
菜单命令 `tim_ic`（带 last_report：无信号时 `PASS tim_ic (no signal, 0 Hz)` SKIP 语义）。
TIM2 在 .ioc（Input Capture2 from TI2 模式），生成代码只到基础 init，捕获寄存器本模块接管。

## 表 1 — CubeMX 等效配置（.ioc 真值 + 接管点）

| 字段 | 值（IOC） | 说明 |
|---|---|---|
| TIM2 Mode | Input Capture 2 direct on TI2 | IOC 路由 PB3；捕获寄存器本模块写 |
| Counter Period | 4294967295（32 位） | F4 只有 TIM2/TIM5 是 32 位通用定时器 |
| PSC | 0 | 最大分辨率 |
| Kernel Clock | APB1 ×2 = 84 MHz | RM ch.6 |
| PB3 告警 | PB3 默认 AF0 = JTDO | MODER=10 重路由；SWD 不受影响 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PB3 | AF1 | 输入（TIM2_CH2/TI2） | 板头排针信号输入 |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | TIM2EN bit0 | 内核时钟门控 |
| TIM2->PSC | 0 | 84 MHz 计数（32 位窗内不回绕饱和） |
| TIM2->ARR | 0xFFFFFFFF | 满 32 位窗口 |
| TIM2->CCMR1 | CC2S=10（TI2 直连）/ IC2F | 输入滤波：短于 N 个时钟的毛刺到不了 CCR2 |
| TIM2->CCER | CC2P=0（上升沿）/ CC2E | 捕获沿极性+使能 |
| TIM2->SR | CC2IF / CC2OF | 捕获发生（硬件清=读 CCR2）/ 过捕获告警 |
| TIM2->CCR2 | 捕获时刻值 | 频率=ΔCCR/窗口；跨 2^32 回绕补 N×2^32 |
| TIM2->CR1 | CEN=1 最后写 / CKD=00 / DIR=0 | f_DTS=f_CK_INT 决定滤波时钟基准 |

## 对应视频集数

EP13（输入捕获与频率测量）。
