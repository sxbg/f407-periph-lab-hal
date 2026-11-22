# bsp_pwr — 电源管理：Sleep/Stop/Standby（EP17）

PWR 三低功耗模式教学：Sleep 100 ms（SysTick 中断唤醒，selftest 真跑）、Stop（KEY1=PA0 唤醒，菜单 demo `pwr_stop_demo`）、Standby（API-only，进入即掉电不返回）。HAL 版 PWR **无 .ioc 独立 init**——三种模式全部是运行时 `HAL_PWR_*` 调用；EXTI 唤醒线归 GPIO 配置节与 bsp_exti。教学内核不变：三模式 = 一条 WFI 指令 + 三个控制位（LPDS/PDDS/SLEEPDEEP）的排列，变的只是"谁写这些位"——HAL 写，bsp 只调 API。

## CubeMX GUI 配置步骤

PWR 模式没有 CubeMX 页面（.ioc 无此 IP，知识 #13）：`PWR->CR/CSR`、`SCB->SCR` 全由 `HAL_PWR_*` 运行时操作，bsp 层零寄存器写（A3 红线）。唤醒相关的按键 EXTI 在 GPIO 配置节完成：

1. Pinout 视图点 PA0 → GPIO Mode = **External Interrupt Mode with Rising edge trigger detection**（KEY1 按下=高，上升沿=按下）；标签 `KEY1`。
2. 点 PA1 → **Falling edge**（KEY2 按下=低）；标签 `KEY2`。
3. System Core → GPIO：PA0 **Pull-down**（空闲低）、PA1 **Pull-up**（空闲高）。
4. System Core → NVIC：勾选 **EXTI line0 interrupt** 与 **EXTI line1 interrupt**，优先级均 5（Stop 唤醒要求 EXTI 中断模式 + NVIC 向量使能——RM0090 停机唤醒条件，两条腿缺一不可）。
5. 生成。CubeMX 从此拥有 EXTI 线 0/1 布线与向量（`stm32f4xx_it.c` → `HAL_GPIO_EXTI_IRQHandler` 链）；本模块只拥有唤醒**计数**。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| PA0 | KEY1（WKUP1/EXTI0） | 输入 + 上升沿 EXTI + 下拉（Stop 唤醒源；Standby 后兼作 WKUP1） |
| PA1 | KEY2（EXTI1） | 输入 + 下降沿 EXTI + 上拉（归 exti.md，此处仅列引脚地图） |

## HAL API 调用清单

三模式的 HAL 语义映射（bsp 层绝不手写控制位，A3 红线）：

- Sleep：`HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI)`——内部清 SLEEPDEEP 后 WFI；唤醒后执行原地继续，任意使能的 NVIC 中断可醒。
- Stop：`HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI)`——内部置 LPDS=1（低耗调节器保 1.2 V 域）、PDDS=0、SLEEPDEEP=1，WFI，唤醒后再清 SLEEPDEEP。
- Standby：`HAL_PWR_EnterSTANDBYMode()`——PDDS=1 + 清 WUF + SLEEPDEEP=1 + WFI **一拍完成**；HAL 没有 arm-only 调用，武装+进入原子。

模块公开 API（bsp_pwr.h）：

- `int bsp_pwr_init(void)` — 开 PWR/SYSCFG 门控（`__HAL_RCC_PWR_CLK_ENABLE` 等），只读回读 LPDS/PDDS/SLEEPDEEP 静息态。0=PASS。
- `uint32_t bsp_pwr_sleep_ms(uint32_t ms)` — Sleep 毫秒延时：循环进 Sleep，每次 WFI 由 1 ms SysTick 中断唤醒并计一次 COUNTFLAG；TICKINT 若被关，`HAL_ResumeTick()` 临时武装、事后 `HAL_SuspendTick()` 还原（WFI 零中断会永久睡死）。SysTick 疑死返回 `BSP_PWR_SLEEP_DEAD_SYSTICK`（有界退出）。
- `void bsp_pwr_enter_stop(void)` — Stop 全流程：清残留 `__HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin)`（挂起未清 = Stop 进入被忽略，RM 表注）→ 进 Stop → 唤醒后 `SystemInit()` 重跑——Stop 退出系统时钟落回 HSI 16 MHz 且 PLL 关，老波特率会喷 ~10.9 kBd 垃圾（EP17 教训）；SystemInit 幂等，重跑即回 168 MHz。
- `void bsp_pwr_arm_standby(void)` — `__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU)` 后进 Standby。**函数不返回**：寄存器/SRAM 全丢，唤醒 = 复位级启动（硬件唤醒源：WKUP1 上升沿 = 本板 KEY1、RTC 闹钟/唤醒/入侵、NRST、IWDG）。
- `int bsp_pwr_standby_flag(void)` — `__HAL_PWR_GET_FLAG(PWR_FLAG_SB)`：1=上次启动来自 Standby 唤醒（`__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB)` 清除）。
- `void bsp_pwr_on_exti0(void)` / `uint32_t bsp_pwr_exti0_wake_count(void)` — 唤醒钩子（一次 volatile 自增）与读数。
- `int bsp_pwr_selftest(void)` / `const char *bsp_pwr_last_report(void)` — 自测与一行报告。

```c
uint32_t slept = bsp_pwr_sleep_ms(100);   /* Sleep: SysTick 逐毫秒唤醒 */
/* menu> pwr_stop_demo 后按 KEY1： */
if (bsp_pwr_exti0_wake_count() > 0u) { /* 唤醒确实走了 EXTI0 */ }
```

回调路由（A3 红线）：`HAL_GPIO_EXTI_Callback` 的 weak 覆盖**全工程只有一份**，由 bsp_exti.c 统一按 `GPIO_Pin` 分发——KEY1_Pin 分支调 `bsp_pwr_on_exti0()` 钩子计数，本模块不自建回调（两处强覆盖链接期冲突）。Stop 不进自测：`pwr_stop_demo` 从菜单 demos 触发，醒后 shell 打印 wake count 作为 EXTI 路径证据；睡眠期心跳停摆会被 logcheck 判"挂死"，属预期；唤醒失败救援 = SWD 复位。Standby 保持 API-only（进入即掉电）。

## 生成器盲区备注

本模块无映射盲区。EXTI0 空 handler 的"中断风暴"修复归 exti.md 主题，此处一句带过：Core 侧向量已补 `HAL_GPIO_EXTI_IRQHandler`，回调分发收敛在 bsp_exti.c。

## 自测验证

菜单命令 `pwr`（selftest 只碰 Sleep，绝不进 Stop/Standby——进前者搁浅控制台，进后者直接复位）：

- 步骤 1：门控 + 静息态回读（PWR/SYSCFG 时钟开，LPDS/PDDS/SLEEPDEEP 全清）。
- 步骤 2：`bsp_pwr_sleep_ms(100)` 实测须落 [100,130] ms（短于不可能——循环数 SysTick 圈；长于说明唤醒链停摆），且事后 SLEEPDEEP 仍清。
- 测量诚实注：100 ms 窗口由 SysTick 计时、也由 SysTick 唤醒——同一振荡器双角色，证明的是 WFI/唤醒**机制**而非晶振绝对精度。

本轮实测（2026-11-22 test-report）：**PASS pwr — Sleep(WFI) + standby 标志读回**。

## 对应视频集数

EP17（低功耗模式与唤醒）。
