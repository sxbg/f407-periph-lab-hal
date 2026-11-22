# bsp_tim_encoder — TIM3 正交编码器接口（EP23）

寄存器级编码器模式：TIM3_CH1/CH2=PB4/PB5，×4 计数（TI12），自由回绕测速。
菜单命令 `tim_enc`（带 last_report：CNT 回绕/rpm 对比 PPR）。TIM3 在 .ioc
（Encoder Interface，EncoderMode=TI12），生成代码只到基础 init，编码器寄存器本模块接管。

## 表 1 — CubeMX 等效配置（.ioc 真值 + 接管点）

| 字段 | 值（IOC） | 说明 |
|---|---|---|
| TIM3 Mode | Encoder Interface（EncoderMode=TI12） | IOC 记录意图；SMS=011 由模块写 |
| Counter Period | 65535（ARPE=ENABLE） | 满 16 位窗；自由回绕本身就是 EP23 教学点 |
| Pins | PB4=S_TIM3_CH1（ENC_A）/ PB5=S_TIM3_CH2（ENC_B） | AF2 |
| Kernel Clock | APB1 ×2 = 84 MHz | 只有输入滤波器吃这个时钟——计数器本体由编码器沿驱动 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PB4 | AF2 | 输入（TIM3_CH1，A 相） | 编码器 A（板头排针） |
| PB5 | AF2 | 输入（TIM3_CH2，B 相） | 编码器 B（板头排针） |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | TIM3EN bit1 | 内核时钟门控 |
| TIM3->SMCR | SMS=011（编码器×4）/ TS=00 | 从模式选 TI1+TI2 双边沿计数 |
| TIM3->CCMR1 | IC1/IC2 通道输入+滤波 | 两相数字滤波 |
| TIM3->CCER | CC1P/CC2P=0 | 相序极性（换向开关） |
| TIM3->PSC / ARR | 0 / 65535 | 编码器模式下 PSC 无意义（沿即时钟） |
| TIM3->CNT | 16 位计数 | 转角/转数读出；回绕即 EP23 的"signed delta"课 |
| TIM3->CR1 | CEN=1 / DIR（硬件） | 计数使能；DIR 位由硬件随转向翻转（只读判据） |

## 对应视频集数

EP23（定时器编码器接口与电机测速）。
