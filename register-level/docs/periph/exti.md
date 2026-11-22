# bsp_exti — EXTI1 ← KEY2 下降沿中断（EP10）

寄存器级 EXTI 链路教学：PA1(KEY2) → SYSCFG EXTICR → EXTI FTSR/IMR/PR → NVIC EXTI1_IRQn →
3 语句 ISR（读测 PR / rc_w1 清 / volatile 计数++）。菜单命令 `exti`（**交互式**：打印
"按 KEY2 三次（6 秒内）"，≥3 次 → `PASS exti (3 presses)`；超时 → `PASS exti (idle, 0 presses)`
SKIP 语义不 FAIL）。
SYSCFG/EXTI 不在 .ioc（知识 #13 adapter 全权）；PA1 脚配置（输入+上拉）归 bsp_gpio，
本模块只接管路由（bsp_pwr/EXTI0 先例）。

## 表 1 — CubeMX 等效配置（模块真值 + 接管点）

| 字段 | 值 | 说明 |
|---|---|---|
| SYSCFG/EXTI IP | 不在 .ioc | adapter 全权外设 |
| PA1（IOC 真值） | GPIO_Input + GPIO_PULLUP，label KEY2 | 上拉空闲=1，按下接地=0 → **按下即下降沿** |
| GPIO mode 等效 | GPIO_MODE_IT_FALLING | HAL 等价写的就是 EXTICR 同一 nibble |
| Trigger | 下降沿 only（FTSR1=1，RTSR1=0） | 松开（上升沿）不计次 |
| NVIC | EXTI1_IRQn=7，优先级 10 | ISR 3 语句纪律（EP10/EP15） |

## 表 2 — 引脚

| 脚 | 信号 | 方向 | 板载器件 |
|---|---|---|---|
| PA1 | KEY2 | 输入（上拉，低有效） | 板载按键（bsp_gpio 拥有脚配置） |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB2ENR | SYSCFGEN bit14 | EXTICR 可写前提 |
| RCC->AHB1ENR | GPIOAEN | 线路采样 PA1 焊盘（仅时钟，不动脚模式） |
| SYSCFG->EXTICR[0] | EXTI1 nibble [7:4]=0000（PA） | 每条 EXTI 线选一个端口的同号脚（0000=PA…） |
| EXTI->FTSR | TR1=1 | 下降沿锁存使能 |
| EXTI->RTSR | TR1=0 | 上升沿解除 |
| EXTI->IMR | MR1=1 | 不屏蔽=中断模式（屏蔽则 PR 置位但 NVIC 永远收不到——最安静的故障） |
| EXTI->PR | PR1 rc_w1 | 写 1 清挂起（与 TIM/ADC SR rc_w0 相反的对照点）；初始化清陈旧挂起 |
| NVIC EXTI1_IRQn | =7 使能 | startup 弱符号槽位；ISR 见 bsp_exti.c |

volatile 教学点：计数器不带 volatile 时主编译器可把轮询读提升出循环——ISR 在加计数而主线"看见"冻结。

## 对应视频集数

EP10（外部中断：引脚→SYSCFG→EXTI→NVIC 四级链路与 ISR 纪律）。
