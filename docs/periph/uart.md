# bsp_uart — USART1 日志控制台（EP03）

本模块教：CubeMX 把串口配成什么（生成代码即真值）、应用层怎么用 `HAL_UART_*` 轮询收发、以及如何用读回校验确认配置未被破坏。USART1 是菜单 shell 的承载口；寄存器版（CR1/BRR 手写）细节见 f407-periph-lab 工程同名文档。

## CubeMX GUI 配置步骤

1. 时钟树（Clock Configuration，三串口共用）：HSE 25 MHz → PLL（M=25, N=336, P=2）→ SYSCLK 168 MHz；APB1 Prescaler = /4 → 42 MHz，APB2 Prescaler = /2 → 84 MHz（USART1 内核时钟 = PCLK2 = 84 MHz）。
2. 左侧 Categories → Connectivity → USART1：Mode 选 **Asynchronous**（异步 8N1，无外时钟脚）。
3. Parameter Settings（与生成代码 `MX_USART1_UART_Init` 一一对应）：
   - Baud Rate = **115200**（84 MHz 下实测误差 +0.023%，远小于 8N1 ~2% 预算）
   - Word Length = **8 Bits**（含校验位）；Parity = **NONE**；Stop Bits = **1**
   - Mode = **TX_RX**；Hardware Flow Control = **NONE**
   - Over Sampling = **16**（对应 BRR = 0x2D9：USARTDIV = 84M/16/115200 = 45.5729 → 尾数 45、小数 round(0.5729×16)=9）
4. DMA Settings：USART1_TX → DMA2 Stream7 / Channel4，Memory→Peripheral，Normal，Low 优先级（详见 dma.md；本模块不用 DMA）。
5. NVIC Settings：USART1 中断**不开**——轮询教学基线。
6. USART2（PA2/PA3）、USART3（PB10/PB11）重复步骤 2-3：同为 115200 8N1、TX_RX、无 DMA、无中断。

生成真值锚点（`Core/Src/usart.c`）：`BaudRate=115200; WordLength=UART_WORDLENGTH_8B; StopBits=UART_STOPBITS_1; Parity=UART_PARITY_NONE; Mode=UART_MODE_TX_RX; HwFlowCtl=UART_HWCONTROL_NONE; OverSampling=UART_OVERSAMPLING_16`。引脚由 `HAL_UART_MspInit` 配置：AF 推挽、无上下拉、Very High speed、AF7。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| PA9 | LOG_TX（USART1_TX） | AF7 推挽输出，板载 USB-TTL/CH340 |
| PA10 | LOG_RX（USART1_RX） | AF7 输入，无上下拉 |
| PA2 | LOG2_TX（USART2_TX） | AF7 推挽输出 |
| PA3 | LOG2_RX（USART2_RX） | AF7 输入 |
| PB10 | RS485_TX（USART3_TX） | AF7 推挽输出 |
| PB11 | RS485_RX（USART3_RX） | AF7 输入 |

## HAL API 调用清单

bsp_uart 对外 4 个函数；自测返回 0=PASS、非零=失败步号，判据行由 shell 层打印（uart 自身即控制台，失败诊断由它直接输出）。CubeMX 拥有外设，本模块只做读回校验 + HAL 收发（A3 红线：不写外设控制寄存器）。

| bsp 函数 | 对应 HAL 调用 | 一句作用 |
|---|---|---|
| `int bsp_uart_init(void)` | 读回 `huart1`（BRR=0x2D9 + UE/TE/RE） | 校验 CubeMX 配置完好（只读，-1=失配） |
| `void bsp_uart_puts(const char *s)` | `HAL_UART_Transmit`（32 字节分段，超时上限 5 ms/字节） | 阻塞发字符串；TX 卡死即返回，不挂菜单循环 |
| `int bsp_uart_getc_nonblock(void)` | `HAL_UART_Receive(&huart1, &b, 1, 0)`（0 tick 超时） | 非阻塞收一字节：0..255，无数据 = -1（HAL_TIMEOUT） |
| `int bsp_uart_selftest(void)` | 上述组合 | 3 步自测（见下节） |

典型调用：

```c
#include "bsp_uart.h"

bsp_uart_init();                        /* 0 = 生成配置完好 */
bsp_uart_puts("hello 115200 8N1\r\n");  /* HAL_UART_Transmit 轮询发送 */
int c = bsp_uart_getc_nonblock();       /* HAL_UART_Receive 零超时收键 */
```

## 自测验证

菜单命令：`uart`
判据：`PASS uart`。三步：① BRR/CR1 读回（0x2D9 + UE/TE/RE）② `HAL_UART_Transmit` 发送 `UART-SELFTEST-OK`（TC 门控，100 ms 上限）③ RX 静默窗：先清残余字节，再三个 10 ms 窗口必须全部超时（RXNE 保持清零）。
本轮实测：**PASS**——"BRR 读回 0x2D9 + TC 发送 + RX 静默窗"。备注：step3 偶发检出 USB CDC 桥/线噪声杂散字节，复跑即过（test-report §4.5）。

## 对应视频集数

EP03（USART 串口与日志口）。
