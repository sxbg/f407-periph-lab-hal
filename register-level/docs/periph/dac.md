# bsp_dac — DAC 通道 1 软件模式输出（EP09）

寄存器级 DAC 通道 1（PA4=DAC_OUT1）：TEN1=0 软件模式（DHR12R1 写入 → 1 个 APB1 周期后自动进
DOR1）、BOFF1=0 输出缓冲开、DOR 回读自检。菜单命令 `dac`（0/1650/3300 mV 三点回读）。
DAC IP 不在 .ioc（知识 #13 adapter 全权），PA4 在 .ioc 为 GPIO_Analog 占位。

## 表 1 — CubeMX 等效配置（模块真值 + 接管点）

| 字段 | 值 | 说明 |
|---|---|---|
| DAC IP | 不在 .ioc | adapter 全权外设 |
| PA4 可用性 | SPI1 NSS 是软件 NSS（SSM=1，SSI=1） | pin-budget 矩阵已把 PA4 让给 DAC |
| OUT2=PA5 | 不用（W25Q128 CLK 占） | 单通道方案 |
| Trigger | NONE（TEN1=0） | 软件模式：写 DHR → 1 APB1 周期后自动进 DOR（RM 14.3.4） |
| Output Buffer | ENABLED（BOFF1=0） | 低阻驱动杜邦线+ADC 负载；代价是轨边余量 ~0.2 V（闭环测试点 500/1650/2800 mV 全在缓冲区内） |
| Wave Generation | NONE（WAVE1=00） | DC 设点 API；噪声/三角波是带触发才可用的 EP09 进阶 |
| NVIC | 关 | 无标志可轮询 |

## 表 2 — 引脚

| 脚 | 信号 | 方向 | 板载器件 |
|---|---|---|---|
| PA4 | DAC_OUT1 | 模拟输出（MODER=11） | 板头排针（dac_adc_loop 测试脚） |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | DACEN bit29 | 内核时钟门控 |
| GPIOA->MODER | MODER4 [9:8]=11 | 模拟模式（缓冲直接驱动焊盘） |
| DAC->CR | EN1=0 位、TEN1 bit2=0、BOFF1 bit1=0、WAVE1 [6:5]=00 | 通道使能（模拟宏单元启动 tWAKEUP）/软件模式/缓冲开/无波形 |
| DAC->DHR12R1 | DACC1DHR [11:0] | 12 位右对齐保持寄存器（DOR 不可直写） |
| DAC->DOR1 | DACC1DOR [11:0] | 转换寄存器——回读即自检（DOR 才是模拟级真正看到的值） |

换算：DACoutput = VREF+ × DOR/4096；set_mv：code = round(mv×4096/3300) 四舍五入，上限 4095（满量程即公式饱和）。量化步距 0.806 mV。

## 对应视频集数

EP09（DAC 输出与模拟闭环；缓冲/触发/波形生成三开关教学）。
