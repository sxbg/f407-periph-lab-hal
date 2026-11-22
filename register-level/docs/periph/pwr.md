# bsp_pwr — 电源管理：Sleep/Stop/Standby（EP17）

PWR 三低功耗模式教学：Sleep 100 ms（SysTick 唤醒，selftest 真跑）、Stop
（KEY1=PA0 唤醒，demo 命令 `pwr_stop_demo`）、Standby（演示语义）。PWR **不在**
.ioc；PA0 引脚归 bsp_gpio，EXTI0 唤醒线只在进 Stop 时武装。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| PWR Mode 教学 | Sleep / Stop（LPDS, 1.2V 域保电）/ Standby（PDDS） | SCB SLEEPDEEP + PWR->CR 组合编码 |
| Wakeup | WKUP1 = PA0（KEY1，EWUP 臂位） | 上升沿=按下；Stop 前清 WUF/EXTI 挂起位 |
| EXTI0 | 仅 enter_stop 内开：RTSR bit0 + IMR bit0 + NVIC(6) | SYSCFGEN bit14 前置；唤醒后 SystemInit 重跑（168 MHz 回归） |
| Standby 后 | PWR->CSR SBF / RCC->CSR LPWRRSTF | 复位源判读（EP05 联动） |
| NVIC | EXTI0_IRQn 仅 Stop 唤醒期间使能 | 轮询纪律外的必要向量 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PA0 | — | 输入（WKUP1/EXTI0） | KEY1（按下→高）；引脚配置归 bsp_gpio |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | PWREN bit28 | PWR 总线时钟 |
| PWR->CR | LPDS / PDDS / CWUF / DBP | Stop 电压调节器模式 / Standby 选择 / 清唤醒旗 / 备份域解锁 |
| PWR->CSR | WUF / SBF / EWUP | 唤醒旗 / Standby 旗 / WKUP 臂位 |
| SCB->SCR | SLEEPDEEP bit2 | WFI 进深睡（0=Sleep 1=Stop/Standby，分界在 PDDS） |
| EXTI->IMR / RTSR / PR | bit0 | 线屏蔽 / 上升沿触发 / 挂起（rc_w1 清；挂起未清=Stop 进入被忽略） |
| RCC->APB2ENR | SYSCFGEN bit14 | EXTI 配置前置门 |
| NVIC | EXTI0_IRQn=6 | 唤醒向量 |

## 对应视频集数

EP17（低功耗模式与唤醒）。
