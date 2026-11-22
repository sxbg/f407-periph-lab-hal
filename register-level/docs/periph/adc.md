# bsp_adc — ADC1 寄存器级单通道采集（EP08/EP09）

寄存器级 ADC1：SMPR 480 周期采样 / SQR1 L=0 单转换 / CR2 SWSTART 软件启动 + EOC 轮询读 DR。
四个通道：IN8=PB0（闭环测试脚）、IN10=PC0、IN14=PC4、Vrefint=IN17（内部基准，TSVREFE 唤醒）。
菜单命令 `adc`（带 last_report：`PASS adc (vrefint 1502mV, vdda 3298mV)`）。
ADC IP 不在 .ioc（知识 #13 adapter 全权），模拟脚在 .ioc 为 GPIO_Analog 占位，MODER 由本模块接管。

## 表 1 — CubeMX 等效配置（模块真值 + 接管点）

| 字段 | 值 | 说明 |
|---|---|---|
| ADC1 IP | 不在 .ioc | adapter 全权外设不出现在 .ioc |
| ADCCLK | CCR ADCPRE=复位默认 00 = PCLK2/2 = 42 MHz | **数据手册上限 36 MHz**——按默认保留，注释给出合规修法（ADCPRE=01 → 21 MHz），EP08 时钟教学点 |
| Sampling Time | 480 cycles（SMPx=111）× 4 通道 | Vrefint 高阻源要求 ≥~10 µs 采样（RM 13.10）；480/42M=11.4 µs 达标 |
| Sequence | 1 conversion（SQR1 L=0） | 单通道轮询教学；换通道=重写 SQR3 SQ1 |
| CR2 | ADON=1、CONT=0 单次、EXTEN=00、SWSTART 启动 | F4 语义：再写 ADON 不再启动转换（与 F1 不同），只有 SWSTART/JSWSTART 启动 |
| NVIC | EOCIE 关 | EOC 轮询（EP08 基线） |

## 表 2 — 引脚

| 脚 | 通道 | 方向 | 板载器件 |
|---|---|---|---|
| PB0 | ADC1_IN8 | 模拟输入 | 板头排针（dac_adc_loop 测试脚） |
| PC0 | ADC1_IN10 | 模拟输入 | 板头排针 |
| PC4 | ADC1_IN14 | 模拟输入 | 板头排针 |
| （内部） | ADC1_IN17 Vrefint | 内部基准 | 1.21 V 典型，反推 VDDA |

引脚规则：MODER=11 模拟（施密特/输入缓冲断开），PUPDR=00 无上下拉（拉电阻会与源阻抗分压）。

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB2ENR | ADC1EN bit8 | 数字接口时钟（寄存器可写前提） |
| ADC->CCR | TSVREFE bit23 = 1 | 唤醒 Vrefint（IN17，仅主 ADC1）；ADCPRE [17:16] 模拟时钟分频 |
| GPIOx->MODER | PB0/PC0/PC4 = 11 | 模拟模式 |
| ADC1->SMPR1/2 | SMP10/14/17/8 = 111 | 480 周期采样（SMPR1 管 10..18，SMPR2 管 0..9） |
| ADC1->SQR1 | L=0 | 规则序列长度 = 1 |
| ADC1->SQR3 | SQ1=通道号 | 每次读前重定向通道 |
| ADC1->CR1 | 0 | SCAN=0、RES=00（12 位） |
| ADC1->CR2 | ADON / SWSTART bit30 | 上电 / 软件启动（ADON=1 才生效，自清零） |
| ADC1->SR | EOC bit1（rc_w0） | 转换完成；读 DR 也清 EOC；OVR 未使能（轮询不会溢出） |
| ADC1->DR | DATA [11:0] 右对齐 | 12 位码 0..4095 |

换算：mv = raw×3300/4096（量化步距 0.806 mV）；vdda = 1210×4096/raw_vref（Vrefint 典型 1.21 V 反推电源轨）。

## 对应视频集数

EP08（ADC 采集原理：采样时间/单次转换/EOC 轮询）、EP09（DAC+ADC 模拟闭环，Vrefint 教学点）。
