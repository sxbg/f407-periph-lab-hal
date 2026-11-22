# bsp_sdio_tf — SDIO TF 卡识别流程（EP24）

寄存器级 SDIO **卡识别协议链**：PC8-11=D0-D3 / PC12=CK / PD2=CMD（AF12），
识别期 CLKDIV=210 → 396.2 kHz（<400 kHz 规则），跑完 CMD0 → CMD8 → ACMD41 →
CMD2 → CMD3 → CMD9，捕获 OCR CCS / CID 制造商 / CSD 容量。菜单命令 `sdio_tf`。
**教学聚焦协议骨架**：FIFO/DMA 数据通路（CMD17/24、DTIMER/DLEN/DCTRL、32×32 位
FIFO）注明"数据读写教学留待扩展"——识别即可证明引脚/时钟/CPSM/响应解析全链路。
SDIO **不在** .ioc；六脚为 GPIO_Analog 占位（TF_D0..TF_CK/TF_CMD），AF12 由本模块写入。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| SDIO Mode | SD 4-bit wide bus | 识别期跑 1-bit（WIDBUS=00），宽总线属数据相位课 |
| Clock Divide Factor | CLKDIV=210 → 396.2 kHz | SDIO_CK = SDIOCLK/(CLKDIV+2) = 84 MHz/212；偶数 CLKDIV=50% 占空；RM 31.9.2 识别期必须 <400 kHz |
| SDIOCLK | PLLQ = 84 MHz（本板时钟树） | RM 名义 48 MHz PLLQ 未满足，CLKDIV 按实算 84 MHz 选定 |
| Hardware Flow Control | Disable | 仅命令路径，不需要 |
| DMA | off | FIFO 未用（数据相位 = 扩展练习） |
| NVIC | SDIO_IRQn off | 轮询教学基线 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PC8 | AF12 | 双向（D0） | TF 卡座 DAT0 |
| PC9 | AF12 | 双向（D1） | TF 卡座 DAT1 |
| PC10 | AF12 | 双向（D2） | TF 卡座 DAT2 |
| PC11 | AF12 | 双向（D3） | TF 卡座 DAT3（DAT3 高采样 = 卡进 SPI 模式的告警点，见模块头注释） |
| PC12 | AF12 | 输出（CK） | TF 卡座 CLK（与 ESP8266-EN 冲突 #1，SDIO 优先） |
| PD2 | AF12 | 双向（CMD） | TF 卡座 CMD |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB2ENR | SDIOEN bit11 | SDIO 挂 APB2；SDIOCLK 来自 PLLQ 分频输出 |
| SDIO->POWER | PWRCTRL[1:0]=11 | 卡时钟上电；两次写之间 ≥7 HCLK |
| SDIO->CLKCR | CLKDIV=210 / CLKEN / WIDBUS=00 | 识别时钟 396.2 kHz；偶数分频 50% 占空 |
| SDIO->ARG | 32 位参数 | 必须先于 CMD 写入 |
| SDIO->CMD | CMDINDEX[5:0] / WAITRESP[7:6] / CPSMEN | WAITRESP 00=无响应(CMDSENT)/01=短(R1,R3,R6,R7)/11=长(R2) |
| SDIO->RESPCMD | 只读 | 最近响应的命令号（R3 等无命令号响应里无意义） |
| SDIO->RESP1..4 | R2=127 位 CID/CSD 打包；R3=OCR；R6=RCA<<16+状态；R7=参数回显 | RESP1=位 127:96 … RESP4[0] 恒 0 |
| SDIO->STA | CMDSENT / CMDREND / CCRCFAIL / CTIMEOUT | 完成标志组；CTIMEOUT=硬件 64 卡时钟超时（无卡判据）；R3 无 CRC → CCRCFAIL 需软件忽略 |
| SDIO->ICR | CCRCFAILC 等 | rc_w1c：写 1 清对应 STA 静态位 |
| DTIMER/DLEN/DCTRL/FIFO | 未动 | 数据相位（扩展）：DTDIR/DTEN/DBLOCKSIZE + 32 字 FIFO |

R1/R6/R7 响应格式（RM 31.5）：R1=32 位卡状态；R6=RESP1[31:16]=卡自选 RCA、
[15:0]=状态（bit15=ERROR）；R7=参数回显（[11:8] 电压接受、[7:0] 校验图案 0xAA）。
容量公式：SDHC（CCS=1，CSD v2）capacity = (C_SIZE+1) × 512 KiB，C_SIZE=CSD[69:48]；
SDSC（CSD v1）C_SIZE=CSD[73:62]（乘数×READ_BL_LEN，另算）。

## 对应视频集数

EP24（SDIO 与 TF 卡）。
