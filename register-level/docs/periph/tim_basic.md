# bsp_tim_basic — TIM5/TIM6/TIM7 基础时基（EP04）

TIM5 1 s 中断时基 + TIM6/TIM7 轮询时基：PSC/ARR/UIF 三件套教学。菜单命令 `tim_basic`。
TIM5 在 .ioc（仅基础初始化，NVIC 关）；TIM6/TIM7 **不在** .ioc（模块全权：RCC 门控、
PSC/ARR、NVIC、ISR）。

## 表 1 — CubeMX 等效配置

| 字段 | 值 | 说明 |
|---|---|---|
| TIM5（IOC） | 基础 init：PSC=0，ARR=max，CEN=0，NVIC off | 无引脚时基，中断由模块武装 |
| TIM6/TIM7 | 不在 IOC | 全部 CubeMX 等效配置=模块真值：Basic Timer，TIM6 可作 DAC 触发（EP09 链） |
| Kernel Clock | 均 APB1 ×2 = 84 MHz | RM ch.6 规则 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| — | — | 无引脚 | 纯内部计数时基 |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | TIM5EN bit3 / TIM6EN bit4 / TIM7EN bit5 | 内核时钟门控 |
| TIMx->PSC / ARR | 1 s 实算（84M 分频链） | 事件频率=定时器时钟/(PSC+1)/(ARR+1) |
| TIMx->EGR | UG | 立即装载影子寄存器 |
| TIMx->SR | UIF | 更新事件标志（轮询判据/TIM5 ISR 里清） |
| TIM5->DIER | UIE=1 | 更新中断使能（TIM6/7 纯轮询不开） |
| TIMx->CR1 | CEN（最后写）/ OPM=0 | 连续计数 |
| NVIC | TIM5_IRQn=50：ISER[1] bit18、IPR byte50 | CMSIS 层中断装配（教学：向量表→ISER→IPR） |

## 对应视频集数

EP04（定时器基础与时基）。
