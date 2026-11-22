# bsp_tim_adv — TIM8 互补 PWM + 死区（EP14）

寄存器级高级定时器：TIM8_CH1=PC6 独立 PWM + CH2=PC7/CH2N=PB14 互补对，
1 µs 死区（BDTR DTG 两段编码实算）。菜单命令 `tim_adv`。TIM8 在 .ioc
（PWM Gen1 CH1 + Gen2 CH2/CH2N，PSC=0 基础 init），通道/BDTR 寄存器本模块接管。

## 表 1 — CubeMX 等效配置（.ioc 真值 + 接管点）

| 字段 | 值（IOC） | 说明 |
|---|---|---|
| TIM8 Mode | PWM Generation1 CH1 + Generation2 CH2/CH2N（CCxE 使能） | IOC 路由三脚；通道寄存器本模块写 |
| Prescaler | 0（基础 init 默认） | 模块自选 1 µs 节拍（PSC=167，ARR=999 → 1 kHz 帧） |
| Pins | PC6=CH1、PC7=CH2、PB14=CH2N（均 AF3） | 板头钩子 |
| Kernel Clock | APB2 ×2 = 168 MHz | CKD=00 → t_DTS=5.952 ns，死区基准 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PC6 | AF3 | 输出（CH1） | 板头排针 |
| PC7 | AF3 | 输出（CH2） | 板头排针 |
| PB14 | AF3 | 输出（CH2N 互补相） | 板头排针 |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB2ENR | TIM8EN bit1 | 内核时钟门控 |
| TIM8->PSC / ARR | 167 / 999 | 1 MHz 计数节拍 + 1 kHz 帧 |
| TIM8->RCR | 0 | 无重复计数（每更新事件都生效） |
| TIM8->CCMR1 | OC1M/OC2M=110 + OC1PE/OC2PE | PWM 模式 1 + 比较预装载 |
| TIM8->CCER | CC1E / CC2E / CC2NE | 三路输出使能（互补对由 CC2E+CC2NE 联合定义） |
| TIM8->BDTR | MOE=1 / DTG=0x94 / OSSR/OSSI | 死区两段编码第二段：DTG[7]=1 时 DT=(64+DTG[5:0])×2×t_DTS=(64+20)×11.905ns=1000ns=1 µs 精确落位 |
| TIM8->EGR | UG | 装载影子 |
| TIM8->CCR1 / CCR2 | 50% 占空 | 预装载写法 |
| TIM8->CR1 | CEN 最后写 / ARPE=0 / DIR=0 / CKD=00 | 立即 ARR（IOC 无预装载键） |

## 对应视频集数

EP14（高级定时器：互补输出与死区）。
