# bsp_tim_pwm — TIM1/TIM4 PWM 输出（EP06）

寄存器级 PWM：TIM1_CH1=PA8（APB2 ×2=168 MHz）+ TIM4_CH1=PD12（APB1 ×2=84 MHz），
1 kHz 帧占空可调。菜单命令 `tim_pwm`。TIM1/TIM4 在 .ioc（PWM Generation CH1 模式），
生成代码只到基础初始化（PSC=0 ARR=max CEN=0），通道寄存器由本模块接管。

## 表 1 — CubeMX 等效配置（.ioc 真值 + 模块接管点）

| 字段 | 值（IOC） | 说明 |
|---|---|---|
| TIM1 Mode | PWM Generation CH1 | IOC 路由 PA8；通道寄存器本模块写 |
| TIM4 Mode | PWM Generation CH1 | 同上（PD12） |
| Kernel Clock | TIM1=APB2×2=168 MHz；TIM4=APB1×2=84 MHz | RM ch.6：总线有分频时定时器时钟 ×2 |
| PSC/ARR（IOC 初值） | 0 / 65535 | 模块自选 1 kHz（PSC/ARR 按 168M 与 84M 各自实算） |
| NVIC | TIM1_UP_TIM10_IRQn 等不开 | 轮询教学基线 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PA8 | AF1 | 输出（TIM1_CH1） | 板头排针（EP06 波形教学点） |
| PD12 | AF2 | 输出（TIM4_CH1） | 板头排针 |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB2ENR / APB1ENR | TIM1EN bit0 / TIM4EN bit2 | 内核时钟门控（两总线两倍频） |
| TIMx->PSC | 实算分频 | 1 MHz 计数节拍 |
| TIMx->ARR | 999 | 1 kHz 帧 |
| TIMx->CCR1 | 占空值 | OC1PE 预装载下写影子寄存器 |
| TIMx->CCMR1 | OC1M=110（PWM1）/ OC1PE=1 | 输出比较模式+比较预装载 |
| TIMx->CCER | CC1E=1 | CH1 引脚输出使能 |
| TIMx->EGR | UG | 装载 PSC/ARR 影子 |
| TIMx->CR1 | CEN（最后写）/ ARPE | 计数使能 |
| TIM1->BDTR | MOE=1 | 高级定时器主输出使能（TIM1 特有，关=全引脚无效） |

## 对应视频集数

EP06（定时器 PWM 输出）。
