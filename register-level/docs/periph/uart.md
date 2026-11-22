# bsp_uart — USART1 日志控制台（EP03）

寄存器级 USART1：115200 8N1 轮询收发，菜单 (`test_menu`) 的承载口。
USART1 **在** .ioc 里（P2 生成，HAL 只做了引脚+时钟），CR1/BRR 等由本模块接管。

## 表 1 — CubeMX 等效配置（.ioc 真值）

| 字段 | 值（IOC） | 说明 |
|---|---|---|
| Mode | Asynchronous（VM_ASYNC） | 8N1 控制台，无外时钟脚 |
| Baud Rate | 115200 | 板载 USB-TTL 支持；84 MHz 下误差 0.023% |
| Word Length | 8 Bits（M=0） | 字节行协议 |
| Parity / Stop | NONE（PCE=0）/ 1（STOP=00） | 控制台 ASCII |
| Hw Flow Control | NONE | PA9/PA10 两线 |
| Oversampling | 16（OVER8=0） | BRR=0x2D9（fCK/baud = 84M/115200） |
| Kernel Clock | PCLK2 = 84 MHz | RCC.APB2Freq_Value（168/2） |
| NVIC | USART1_IRQn 不开 | 轮询教学基线 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PA9 | AF7 | 输出（TX） | 板载 USB-TTL / CH340 RX |
| PA10 | AF7 | 输入（RX） | 板载 USB-TTL / CH340 TX |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB2ENR | USART1EN bit14 | 内核时钟门控 |
| USART1->CR1 | UE bit13 / TE / RE / M=0 / PCE=0 | UE 最后写；OVER8=0 |
| USART1->CR2 | STOP[13:12]=00 | 1 停止位 |
| USART1->CR3 | 0 | 无 DMA/智能卡/半双工 |
| USART1->BRR | 0x2D9 | 过采样 16：fCK/baud 整数+小数部分 |
| USART1->SR / DR | TXE / TC / RXNE | 发送缓冲空/完全移出/收到（读 DR 清 RXNE） |

## 对应视频集数

EP03（USART 串口与日志口）。
