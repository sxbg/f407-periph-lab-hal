# bsp_sdio_tf — SDIO TF 卡识别流程（EP24）

SDIO **卡识别协议链**教学：PC8-PC11=D0-D3 / PC12=CK / PD2=CMD（AF12），4-bit 宽总线。寄存器版手跑 CMD0→CMD8→ACMD41→CMD2→CMD3→CMD9 整个识别序列（CPSM 逐命令、RESP1-4 逐格式）；HAL 版由 `HAL_SD_Init` 在识别时跑完该序列，本模块用 `HAL_SD_GetCardState / GetCardInfo / GetCardCID` 讯问捕获结果 + 亲手解析原始 CSD 字（C_SIZE 位手术存活）。数据通路（FIFO/DMA、CMD17/24、DTIMER/DLEN/DCTRL、32×32 位 FIFO）注明"留待扩展"——识别即可证明引脚/时钟/响应解析全链路。

## CubeMX GUI 配置步骤

1. Connectivity → SDIO → Mode：**SD 4-bit Wide Bus**（PC8-PC12 + PD2 自动落位）。
2. Parameter Settings：Clock transfer edge **Rising**、Clock bypass / Power save **Disable**、Hardware flow control **Disable**、ClockDiv 默认（识别 <400 kHz 规则由 HAL 的 SDIO_INIT_CLK_DIV 兑现；SDIOCLK 走 PLLQ 48 MHz，config-spec §2）。
3. DMA / NVIC 不勾（识别链教学基线）。

**必须写下的生成器盲区（核心，固件侧修复已就位）**：生成包装函数 `MX_SDIO_SD_Init` 对 `HAL_SD_Init` 失败一律 `Error_Handler()`（`__disable_irq()` + 死循环）——TF 座通常空着，**无卡即砖死启动**（pyocd 取证定位的死点之一）。

固件侧修复（三件事）：

1. sdio.c 的 USER CODE `SDIO_Init 0` 区直接 lazy return（包装函数 neutered，`#pragma diag_suppress 111` 压掉不可达代码告警）。
2. 卡识别改由 `bsp_sdio_tf_init()` **运行时自配**（引脚 AF12 + 句柄字段 + `HAL_SD_Init` / `HAL_SD_ConfigWideBusOperation`），按需触发、失败返回 SKIP 语义。
3. 实测 SDHC 7386 MB 识别 PASS；regen 后该 USER CODE 段稳定保留。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| PC8 | TF_D0 / SDIO_D0 | AF12 双向，Very High，无上下拉 |
| PC9 | TF_D1 / SDIO_D1 | AF12 双向 |
| PC10 | TF_D2 / SDIO_D2 | AF12 双向 |
| PC11 | TF_D3 / SDIO_D3 | AF12 双向（DAT3 高采样 = 卡进 SPI 模式的告警点） |
| PC12 | TF_CK / SDIO_CK | AF12 输出 |
| PD2 | TF_CMD / SDIO_CMD | AF12 双向 |

## HAL API 调用清单

- `int bsp_sdio_tf_init(void)` — 首次调用自配六脚 AF12 + `hsd` 字段（BusWide 先 **1B**：识别期按 SD 规约跑 1-bit；ClockDiv = `SDIO_TRANSFER_CLK_DIV`；边沿上升/旁路与流控关）→ `HAL_SD_Init(&hsd)` 跑完整识别链 → `HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B)` 切 4-bit。返回 0 = 识别成功；**−3 = 无卡（SKIP 语义：空座 ACMD41 超时 / CMD6 同路径，是硬件自身判决）**；−1 配置缺失 / −2 卡带错误旗（CMD13）/ −4 卡死在识别态（在座但没上完电——弱接触/坏卡）/ −5 CID 失败 / −6 RCA=0 / −7 容量字段无效 / −8 卡型与 CSD 结构矛盾。
- `bsp_sdio_tf_type() / _manufacturer_id() / _rca() / _c_size() / _capacity_mb()` — 捕获字段访问器：卡型判决（OCR CCS：SDSC 字节寻址 CSD v1 / SDHC 块寻址 CSD v2）、CID[127:120] 制造商字节、卡自选 RCA、C_SIZE（v2 = CSD[69:48]，v1 = CSD[73:62]，对 `hsd.CSD[]` 原始字做位手术——lane 打包与寄存器版 RESP1..4 相同，解析教学原样存活）、v2 容量 MiB（BlockNbr×BlockSize 推导，v1 返回 0 = "原始字段无容量"老语义）。
- `int bsp_sdio_tf_selftest(void)` / `const char *bsp_sdio_tf_last_report(void)`。

```c
if (bsp_sdio_tf_init() == 0) {
    /* 报告行即数据："(SDHC mfr=0x02 7386MB)" */
}
```

识别时钟规则一句话（RM 31.9.2）：SDIO_CK = SDIOCLK/(CLKDIV+2)，识别期必须 <400 kHz（寄存器版 CLKDIV=210 → 396.2 kHz；HAL 版由 SDIO_INIT_CLK_DIV 兑现），偶数 CLKDIV = 50% 占空；CMD8 的 R7 回显分离 v1-only 卡、ACMD41 的 HCS/CCS 给出 SDSC/SDHC 判决、容量公式 (C_SIZE+1)×512 KiB（SDHC）——寄存器深讲指引见老工程 sdio_tf.md 表 3。

寄存器版 → HAL 版映射一览：6 脚 AF12 mux = SDIO_MspInit（本板改由 bsp 运行时自配）；POWER/CLKCR 识别时钟 = HAL_SD_InitCard；CMD0→…→CMD9 手跑序列 = HAL_SD_Init 一次跑完；RESP1..4 读 = `hsd.CSD[]/hsd.CID[]` 捕获字；ACMD41 在位轮询 = `HAL_SD_GetCardState`（CMD13）。教学存活位：C_SIZE 位手术、CCS/CSD 一致性检查、<400 kHz 规则。

## 生成器盲区备注

- **核心盲区**（如上）：空座 `HAL_SD_Init` 失败即 `Error_Handler` 砖机 → USER CODE `SDIO_Init 0` 区 lazy return（固件侧修复，知识 #17/#18 管线一部分）；识别改为 `bsp_sdio_tf_init` 运行时自配（引脚 AF12 + 句柄 + `HAL_SD_Init`），实测 SDHC 7386 MB。
- **sweep/识别语义**：无卡 = **SKIP 不是 FAIL**（TF 卡可移除是设计属性；−3 判 PASS 并出 `(no card)` 报告——与 24C02/W25Q"焊死在板上必须应答"的语义刻意相反）。

## 自测验证

菜单命令 `sdio_tf`：

- 步骤 1：识别查询（−3 走 SKIP-PASS；其余失败按返回码报步骤名，如 `(init: stuck in identification)`）。
- 步骤 2：捕获字段健全性（RCA≠0、C_SIZE≠0、SDHC 必带容量、CCS 与 CSD 结构一致——SDHC 对 v2 / SDSC 对 v1）。
- 失败语义对照：`(no card)` = 空座 SKIP-PASS；`(init: stuck in identification)` = 卡在座但没上完电（弱接触/坏卡，step-1 FAIL）；`RCA=0` / `card type/CSD unknown` = 识别数据损坏或不支持，step-1/2 FAIL。

本轮实测（2026-11-22）：**PASS sdio_tf (SDHC mfr=0x02 7386MB)**——板上实卡 4-bit 识别全链路。

## 对应视频集数

EP24（SDIO 与 TF 卡）。
