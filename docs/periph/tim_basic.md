# bsp_tim_basic — TIM5 基础时基（EP04）

本模块教：PSC/ARR/UIF 三件套的 HAL 拼法——TIM5 配成 1 Hz 轮询时基（`HAL_TIM_Base_Start` 只置 CEN，DIER.UIE 保持清零），update flag 排空计秒。HAL 版精简：老工程 TIM5-IRQ + TIM6/TIM7-轮询三件套坍缩为 TIM5 单定时器（TIM6/TIM7 不在本工程 IOC）。寄存器版细节见 f407-periph-lab 工程同名文档。

## CubeMX GUI 配置步骤

1. 左侧 Categories → Timers → **TIM5**：Activated；Clock Source = Internal Clock（纯内部时基，无引脚）。
2. Parameter Settings（`MX_TIM5_Init` 真值）：Prescaler = 0；Counter Mode = Up；Counter Period = **4294967295**（32 位——F4 只有 TIM2/TIM5 是 32 位通用定时器）；Clock Division = DIV1；auto-reload preload = Disable。
3. NVIC Settings：TIM5 更新中断**不开**（W4b 轮询版；W6 IRQ 版再开 NVIC 布线）。
4. TIM6/TIM7：不出现在本工程 IOC——老版三件套不再，set/clear/set 轮询契约改在 TIM5 自己的 flag 上教。
5. 时钟树：TIM5 挂 APB1（PCLK1 = 42 MHz，总线分频 → ×2 = **84 MHz**）。
6. 生成代码只到 `HAL_TIM_Base_Init`（CEN=0）；1 Hz 形状由 bsp 模块运行时武装。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| — | — | 无引脚，纯内部计数时基 |

## HAL API 调用清单

bsp_tim_basic 对外 4 个函数；console-free，自测返回 0=PASS、非零=失败步号，诊断文本经 `bsp_tim_basic_last_report()` 交给菜单拼判据行。时基形状只经 `__HAL_TIM_SET_*` 宏 + `HAL_TIM_Base_Start` 写（A3 红线；寄存器读仅作校验）。

| bsp 函数 | 对应 HAL 调用 | 一句作用 |
|---|---|---|
| `int bsp_tim_basic_init(void)` | `HAL_TIM_Base_Stop`（幂等）→ `__HAL_TIM_SET_PRESCALER(8399)`/`SET_AUTORELOAD(9999)`/`SET_COUNTER(0)` → `__HAL_TIM_CLEAR_FLAG`（清 MX_TIM5_Init 留下的 pending UIF）→ `HAL_TIM_Base_Start`（仅 CEN，故意不用 `_IT`） | TIM5 1 Hz 轮询武装；UIE 保持清零、不碰 NVIC |
| `uint32_t bsp_tim_basic_uptime_seconds(void)` | `__HAL_TIM_GET_FLAG`/`__HAL_TIM_CLEAR_FLAG` | 每次调用排空 update flag 计秒——每秒至少被调一次才保真（菜单/自测节奏满足） |
| `int bsp_tim_basic_selftest(void)` | 上述组合 | 3 步自测，共 ~3.6 s（见下节） |
| `const char *bsp_tim_basic_last_report(void)` | — | 判据行后缀（失败诊断），"" = 无诊断 |

1 Hz 推导（与 PWM 同一条两步分频律）：f_UPDATE = 84 MHz/(PSC+1)/(ARR+1) = 84M/8400/10000 = 1 Hz → PSC = 8399（84M→10 kHz）、ARR = 9999（10 kHz→1 Hz）。

典型调用：

```c
#include "bsp_tim_basic.h"

bsp_tim_basic_init();                          /* TIM5 1 Hz，中断不开 */
uint32_t up = bsp_tim_basic_uptime_seconds();  /* 排空 UIF 计秒 */
```

## 生成器盲区备注

- **PSC 影子 + EGR.UG 盲区**：PSC 在首个更新事件才装载，HAL 无 UG 拼写 → Start 后第一段 ~119 µs（10000 tick @84 MHz）后才进入 1 Hz 节拍；且 `MX_TIM5_Init` 的 UG 会留下一个 pending UIF，init 里 `__HAL_TIM_CLEAR_FLAG` 清掉（老版 `SR = ~UIF` 的 HAL 拼法，rc_w0 写 0 清除）。
- **轮询版 vs W6 IRQ 版**：本版 `HAL_TIM_Base_Start` 只置 CEN——"标志在没人打断时也存在"正是轮询课点；`HAL_TIM_Base_Start_IT` + `HAL_TIM_PeriodElapsedCallback` + `HAL_NVIC_SetPriority/EnableIRQ` 的 IRQ 版在 W6 NVIC 布线落地后回归（老版的 volatile ISR 计数器随之一并回来）。
- **看门狗**：自测跨三个 1.5 s 窗口，狗刷新由菜单主循环的 `bsp_wdg_refresh()` 负责（未 armed 时为无害写）；调试期 IWDG/WWDG 全关（用户裁定 2026-11-22，test-report §4.1）。

## 自测验证

菜单命令：`tim_basic`
判据：`PASS tim_basic`（失败时带 `bsp_tim_basic_last_report()` 后缀，如 `(step1: no update)`）。三步：① 1.2 s 窗口内更新计数 ≥1（窗口用 SysTick 计——独立时钟交叉验证被测定时器）② UIF set/clear/set 契约（置位→写 0 清→再置位=周期性）③ 排空后 1.2 s 窗口内恰一次更新（1 Hz 节拍跨窗保持）。
本轮实测：**PASS**——"TIM5 1 Hz 轮询版（守护语义修复后稳定）"。
调试器佐证：live 观察 `htim5.Instance->SR.UIF` 每秒翻一次；`DIER.UIE` 保持 0（无中断路由）；`bsp_tim_basic_uptime_seconds()` 在持续轮询下 1/s 前进。

## 对应视频集数

EP04（定时器基础与时基）。
