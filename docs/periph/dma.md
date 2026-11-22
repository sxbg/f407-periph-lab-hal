# bsp_dma — DMA 直接存储器访问（EP07）

HAL 版 DMA 教学：CubeMX 只实例化一条流——DMA2 Stream7/Channel4 = USART1_TX
（Memory→Peripheral，byte，MINC，Normal，直连模式）；bsp 层经 `HAL_UART_Transmit_DMA`
走三拍完成链 + NDTR 计数器自证。菜单命令 `dma`（串口可见 `DMA-TX-OK` 行 + PASS）。

## CubeMX GUI 配置步骤

1. USART1 → DMA Settings → Add USART1_TX（请求映射自动落到 DMA2 Stream7 /
   DMA_CHANNEL_4——RM0090 Table 43：DMA2 Ch4 = USART1_TX）。
2. 流参数（DMA Request tab）：Direction=Memory To Peripheral、Memory Increment=MINC
   Enable（逐字节走字符串）、Peripheral Increment=PINC Disable（DR 固定地址）、
   Data Width=byte/byte、Mode=Normal（NDTR 跑到 0 后停）、FIFO=Disable（直接模式——
   M2P 允许；对照老工程 S6 mem-to-mem 强制 FIFO 的教学点。HAL 版 W2 裁定不实例化
   M2M 流：HAL 无一调用式 M2M 教学 API，本模块改证 USART1_TX DMA 路径）。
3. NVIC：DMA2 stream7 global interrupt 勾选（MX_DMA_Init 设优先级 5 并 EnableIRQ）。
4. 实例化位置：hdma_usart1_tx 定义在 Core/Src/usart.c，由 `__HAL_LINKDMA` 挂进
   huart1.hdmatx；dma.c 只开 DMA2 时钟 + 布 NVIC。

**空 handler 盲区（知识 #18，本模块必教）**：生成的 `DMA2_Stream7_IRQHandler` 是
空体——TCIF 永远不清 → 中断风暴饿死菜单（上板实测，test-report §5.3）。修法
（USER CODE 区补，regen 稳定保留）：`extern DMA_HandleTypeDef hdma_usart1_tx;`
（usart.h 不导出它）+ `HAL_DMA_IRQHandler(&hdma_usart1_tx)`。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| （无新增） | DMA 走既有 PA9=USART1_TX 日志口 | AF7 |

无新增引脚。与轮询 TX 的可见差异：`DMA-TX-OK` 8N1 帧背靠背、无逐字节轮询间隙——
谁在服务 DR（DMA 而非 CPU）即 EP07 的论点。

## HAL API 调用清单

- `int bsp_dma_init(void)` — 只读回读流形状（Instance/Channel4/M2P/PINC off/MINC on/
  byte/byte/Normal/FIFO off）+ LINKDMA 接线（huart1.hdmatx 指向本流）+ DMA 向量活；
  并补武装 USART1_IRQn（优先级 5）：HAL UART DMA 发送的第二阶段完成需要 USART1 中断
  服务（UART_DMATransmitCplt 清 DMAT 并 raise TC → HAL_UART_IRQHandler →
  UART_EndTransmit_IT 把 gState 送回 READY），否则一次突发后 gState 恒 BUSY_TX，
  之后每次轮询控制台写都返回 HAL_BUSY。0=PASS。
- `int bsp_dma_uart_tx(const char *s)` — 一行 DMA 日志：`HAL_UART_Transmit_DMA` 一次
  完成 PAR/M0AR/NDTR 装载 + 开流（老工程 RM 10.3.17 十步的一次调用等价），随后三拍
  完成链全部有界（每拍 100×1 ms 封顶）：①流排空（DMA state BUSY→READY）②UART TC
  置位（移位管线空——更强的"线排空"保证）③gState 回 READY（控制台所有权还给
  bsp_uart 轮询 TX——老工程"DMAT 借用后归还"的 HAL 翻译）。0=送达，-1=空串/超时。
- `int bsp_dma_selftest(void)` — 步 1 路径回读；步 2 发 `DMA-TX-OK\r\n`（载荷本体
  由 DMA 送达）；步 3 NDTR==0 + 流 READY（老工程 NDTR 归零 + EN 自清的 HAL 孪生）。

```c
#include "bsp_dma.h"
bsp_dma_uart_tx("DMA-TX-OK\r\n"); /* CPU never touches DR; DMA serves it */
```

寄存器概念 → HAL 调用映射：S7 CR（CHSEL4/DIR=M2P/MINC/byte/直连）→ CubeMX Init
结构（只读回读）；PAR/M0AR/NDTR + EN=1 → `HAL_UART_Transmit_DMA`；轮询 HISR TCIF7
→ `HAL_DMA_GetState(&hdma_usart1_tx) != BUSY`；轮询 USART SR TC →
`__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC)`。

## 生成器盲区备注（如有）

- DMA2_Stream7_IRQHandler 空体 → USER CODE 区补 HAL_DMA_IRQHandler（上文）。
- hdma_usart1_tx 定义在 usart.c 但 usart.h 不导出 → 消费方 extern（bsp_dma.c 与
  it.c 同一模式）。
- NVIC 只布 DMA 流向量、不布 USART1 → bsp_dma_init 补武装 USART1_IRQn；
  USART1_IRQHandler 放 bsp_dma.c（Core 不动，仅服务 DMA 完成的 TC）。

## 自测验证

菜单 `dma`：**PASS**（test-report §2，USART1_TX 经 DMA2_Stream7 完成链）——
`DMA-TX-OK` 行出现在命令回显与判词之间，线格式与轮询 TX 完全一致。

## 对应视频集数

EP07（DMA：流/通道/仲裁讲解；mem-to-mem 与 UART DMA 发送演示——HAL 版只保留 UART
DMA 演示，mem-to-mem 对照见寄存器版）。
