# bsp_tim_ic — TIM2 输入捕获频率计（EP13）

本模块教：TIM2_CH2=PB3（TI2 直连）输入捕获测频——32 位自由计数 + 1 s 窗口内抓两个上升沿，freq = 84 MHz/period，溢出补偿保留为教学原则。CubeMX 配到基础 init，捕获通道形状由 `HAL_TIM_IC_*` 运行时写。寄存器版（CCMR1/CCER 手写）细节见 f407-periph-lab 工程同名文档。

## CubeMX GUI 配置步骤

1. 左侧 Categories → Timers → **TIM2**：Clock Source = Internal Clock；Channel2 = **Input Capture direct mode**（IC2 直连 TI2，引脚自动落 PB3）。
2. Parameter Settings（`MX_TIM2_Init` 真值）：Prescaler = **0**（最大分辨率）；Counter Mode = Up；Counter Period = **4294967295**（32 位自由计数，84 MHz 下 ~51.1 s 才回绕一次）；Clock Division = **DIV1**（f_DTS = f_CK_INT，滤波时钟基准）；auto-reload preload = Disable。
3. 引脚 PB3（TIM2 MspInit 生成）：AF1、**No pull**——上下拉会偏置浮空探针脚。
4. NVIC Settings：不开（轮询捕获；`HAL_TIM_IC_Start` 不碰 DIER）。
5. 时钟树：TIM2 挂 APB1（PCLK1 = 42 MHz ×2 = **84 MHz**，即 `BSP_TIM_IC_REF_CLOCK_HZ`）。
6. 生成代码只到 `HAL_TIM_Base_Init`（CCMR1/CCER 未配）；捕获形状（CC2S/IC2F/极性/分频）由 bsp 模块 `HAL_TIM_IC_ConfigChannel` 运行时写。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| PB3 | TIM2_CH2（TI2） | AF1 输入，无上下拉（板头排针信号输入） |

PB3 默认 AF0 = JTDO，AF1 复用即重路由；SWD（PA13/PA14）不受影响。

## HAL API 调用清单

bsp_tim_ic 对外 4 个函数 + 1 个时钟常量（`BSP_TIM_IC_REF_CLOCK_HZ` = 84000000）；console-free，自测返回 0=PASS、非零=失败步号，测量结果经 `bsp_tim_ic_last_report()` 交给菜单拼判据行（模块间零依赖：返回数据而非打印）。通道形状只经 `HAL_TIM_IC_ConfigChannel`/`HAL_TIM_IC_Start` 写（A3 红线；寄存器读仅作校验）。

| bsp 函数 | 对应 HAL 调用 | 一句作用 |
|---|---|---|
| `int bsp_tim_ic_init(void)` | `HAL_TIM_IC_Stop`（幂等）→ `HAL_TIM_IC_ConfigChannel`（RISING / DIRECTTI=CC2S 01 / DIV1 / Filter=3）→ `HAL_TIM_IC_Start`（CC2E + CEN） | 捕获通道武装：上升沿、每有效边捕获、~762 ns 毛刺拒绝（IC2F=0011，8 个 f_DTS/8 采样一致才认边） |
| `uint32_t bsp_tim_ic_measure_freq(void)` | `__HAL_TIM_GET_FLAG` + `__HAL_TIM_GET_COMPARE`（读 CCR2） | 共享 1 s 窗口抓两个上升沿，freq = 84M/period_ticks；窗口内无信号对返回 0（= "无信号"） |
| `const char *bsp_tim_ic_last_report(void)` | — | 判据行后缀：`(no signal, 0 Hz)` / `(measured N Hz)` |
| `int bsp_tim_ic_selftest(void)` | 上述组合 | 2 步自测（见下节） |

频率范围 ~1 Hz .. 84 MHz 可表；低于 ~2 Hz 时 1 s 窗口很难捕到两个边，慢信号同样报 0。1 tick 分辨率 = 11.9 ns 周期。周期公式（EP13 原理）：period_ticks = (CCR2@edge2 + N×2^32) − CCR2@edge1，N = 两边之间的 UIF 数（每次溢出 = 一次 2^32 回绕，64 位中间量防越界）。

典型调用：

```c
#include "bsp_tim_ic.h"

bsp_tim_ic_init();                          /* 直连 TI2、上升沿、滤波 3 */
uint32_t hz = bsp_tim_ic_measure_freq();    /* 0 = 窗口内无信号对 */
```

## 生成器盲区备注

- **EGR.UG 武装笔删除**：老版起测前敲 UG 归零 CNT——边到边测量与 CNT 相位无关，HAL 版改为每次测量开头统一清陈旧标志（`__HAL_TIM_CLEAR_FLAG`，UPDATE|CC2，rc_w0 拼法）。
- **CC2IF 无需手动清**：读 CCR2 即清（RM ch.18.4.14 硬件契约，同 UART RXNE 的"读即清"）；UIF（溢出计数用）才需要 `__HAL_TIM_CLEAR_FLAG`。

## 自测验证

菜单命令：`tim_ic`
判据：`PASS tim_ic (no signal, 0 Hz)`（空探针 = SKIP 语义，不是缺陷）或 `PASS tim_ic (measured N Hz)`。两步：① 捕获配置读回（CCMR1 CC2S/IC2F/IC2PSC、CCER CC2E/CC2P/CC2NP、PSC=0、ARR=0xFFFFFFFF）② 一次测量（无信号 PASS 并写报告后缀，有信号报告实测值）。
本轮实测：**PASS (no signal, 0 Hz)**——无输入 SKIP 语义。
闭环交叉验证：`pwm_ic_loop`——PA8→PB3 杜邦线，TIM1 1 kHz PWM → TIM2 捕获，实测 950–1050 Hz（两个独立定时器互测）。

## 对应视频集数

EP13（输入捕获与频率测量）。
