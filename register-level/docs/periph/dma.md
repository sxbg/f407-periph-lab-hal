# bsp_dma — DMA2 Stream 架构教学（EP07）

寄存器级 DMA2 双流：S6 mem-to-mem 64 字搬运（FIFO 模式强制）+ S7/Ch4 USART1_TX 一行日志
（直接模式，DMAT 借用后归还）。菜单命令 `dma`（串口可见 `DMA-TX-OK` 行 + PASS）。
DMA IP 不在 .ioc（知识 #13 adapter 全权），Stream 分配按项目分配表（S6/S7 空闲）。

## 表 1 — CubeMX 等效配置（模块真值 + 接管点）

| 字段 | 值 | 说明 |
|---|---|---|
| DMA IP | 不在 .ioc | adapter 全权外设 |
| S6 方向 | mem-to-mem（DIR[1:0]=10） | 仅 DMA2 能 M2M（RM 10.3.6）；EN=1 即启动（M2M 无请求） |
| S6 FIFO | **DMDIS=1，FTH=full** | RM 10.3.6 原文：M2M 禁止 Circular 与 direct 模式——教学陷阱点 |
| S6 数据宽 | PSIZE=MSIZE=word、PINC+MINC、单拍 | 无打包；优先级 PL=medium（仲裁教学） |
| S7/Ch4 方向 | mem-to-peripheral（DIR[1:0]=01） | Table 43：DMA2 Ch4 = USART1_TX |
| S7 FIFO | DMDIS=0 直接模式 | M2P 的教科书用例（与 S6 的强制 FIFO 对照） |
| S7 数据宽 | byte、MINC=1、PINC=0 | 逐字节走字符串；DR 固定地址 |
| USART1 CR3 | DMAT 置位 → 突发 → 清零复位 | bsp_uart 轮询所有权不受损（RM 10.3.17 关停顺序：先流后外设） |
| NVIC | TCIE/TEIE 关 | TCIF 轮询教学 |

## 表 2 — 引脚

无新增引脚（S6 纯内存搬运；S7 走既有 PA9=USART1_TX 日志口）。逻辑分析仪 PA9 可见
`DMA-TX-OK` 8N1 帧背靠背、无逐字节轮询间隙——与 bsp_uart_puts 的可见差异即 EP07 的论点。

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->AHB1ENR | DMA2EN bit22 | 控制器时钟门控 |
| DMA_SxCR | CHSEL[2:0]（S7=4）/ DIR[1:0] / PINC·MINC / PSIZE·MSIZE / PL / EN | 流引擎：通道=请求源（Table 43），方向 Table 45；**F4 无 MEM2MEM 位（F1 遗留），M2M 即 DIR=10** |
| DMA_SxPAR | S6=源地址 / S7=&USART1->DR | 外设端口（M2M 时是源） |
| DMA_SxM0AR | S6=目的地址 / S7=字符串 | 存储地址 0 |
| DMA_SxNDTR | 传输数（16 位） | 每拍递减；完成后归 0（自检第 ③ 步） |
| DMA_SxFCR | S6 DMDIS+FTH / S7 =0 | FIFO 使能与阈值（M2M 禁 direct 的出处 10.3.6） |
| DMA2->HISR | TCIF6 bit21 / TCIF7 bit27 | 流 4..7 用高半寄存器；TCIF=NDTR 归 0 |
| DMA2->HIFCR | Cxxx6/7 五连位 | rc_w1 写 1 清（与 TIM/ADC SR 的 rc_w0 相反，对照点） |

配置流程照 RM 10.3.17 十步（EN=0 → 等真 0 → 清旗标 → PAR/M0AR/NDTR → CHSEL → PL → FIFO → CR → EN）。

## 对应视频集数

EP07（DMA：流/通道/仲裁 EP07 讲解；mem-to-mem 与 UART DMA 发送演示）。
