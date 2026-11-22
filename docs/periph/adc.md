# bsp_adc — ADC1 模数转换（EP08/EP09）

HAL 版 ADC1：CubeMX 生成的 4-rank 规则扫描（CH8/CH10/CH14 + Vrefint），软件触发单次
模式；bsp 层逐 rank 轮询 EOC 取数，mV 换算 + Vrefint 反推电源轨。菜单命令 `adc`。

## CubeMX GUI 配置步骤

1. Analog → ADC1 → 勾 IN8（PB0）激活 ADC1 IP（知识 #17：模拟 IP 的激活只有 GUI 一条
   真路，首个通道勾选产出引脚+参数全套；后续通道参数可文本补齐）。
2. 依次勾入 IN10（PC0）、IN14（PC4）、Vrefint（CH17；F4 上 Vrefint 就是普通 regular
   通道，无需 VP 键组）。
3. Parameter Settings：ScanConvMode=ENABLE、ContinuousConvMode=DISABLE、
   ExternalTrigConv=软件触发（ADC_SOFTWARE_START，触发边沿 NONE）、12bit 右对齐、
   NbrOfConversion=4、EOC Selection=Single Conversion（EOCS=1——顺带使能溢出检测，
   丢 rank 时 OVR 置位而不再静默错位）。
4. Rank/采样窗（NVIC 不用开，纯轮询）：

| Rank | 通道 | 引脚 | Sampling Time |
|---|---|---|---|
| 1 | CH8 | PB0 | 3 cycles |
| 2 | CH10 | PC0 | 3 cycles（生成值）→ **bsp_adc_init 运行时重编 480 cycles** |
| 3 | CH14 | PC4 | 3 cycles |
| 4 | Vrefint (CH17) | 内部 | 480 cycles |

5. Clock：ClockPrescaler=PCLK2/4（ADC_CLOCK_SYNC_PCLK_DIV4 = 21 MHz）。老工程寄存器版
   ADCPRE 复位默认 00=PCLK2/2=42 MHz 超数据手册 36 MHz 上限的"诚实告警点"，HAL 版
   用配置直接解决。

**采样窗教学点（知识 #18.3，必须看懂再动采样时间）**：3 cycles = 143 ns @21 MHz，
采样保持电容（RAIN cap ~200 Ω 量级）对杜邦线接驳阻抗欠充——CH10 接 DAC 闭环时读恒 0。
CH10 固化 480 cycles 后实测 501/1657/2802 mV vs DAC 设点 500/1650/2800（≤8 mV）。
Rank1/3 保持 3 cycles 是刻意的活教材：浮空脚读恒 0（保持电容无放电路径）+ rank3 会
读到 rank2 残压（实测 PC4 跟踪 PC0 的上一个设点）。F4 特性两条：HAL_ADC_Init 不置
ADON（HAL_ADC_Start 才置）；SMP17 只有 2 位——480 装不下，钳位到字段上限 56，所以
自检不精确探测 SMP17。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| PB0 | ADC1_IN8（ADC1_IN8_Pin） | Analog、无上下拉（MODER=11，MspInit 写） |
| PC0 | ADC1_IN10 | Analog、无上下拉 |
| PC4 | ADC1_IN14 | Analog、无上下拉 |
| （内部） | ADC1_IN17 Vrefint | CCR TSVREFE=1（ConfigChannel(VREFINT) 写入） |

引脚规则不变：模拟模式断开施密特/输入缓冲；PUPDR=00——拉电阻会与源阻抗分压。

## HAL API 调用清单

- `int bsp_adc_init(void)` — 只读回读 CubeMX 配置（handle 形状 4-rank 扫描/单次/
  12bit 右对齐、CCR TSVREFE、SQR1 L=3、SMPR 采样窗字段、PB0/PC0/PC4 MODER 模拟），
  并把 CH10 重编到 480 cycles。0=PASS，-1=不匹配。
- `uint32_t bsp_adc_read_raw(uint32_t ch)` — `HAL_ADC_Start(&hadc1)` 触发整轮 4-rank
  扫描，按序逐 rank `HAL_ADC_PollForConversion` + `HAL_ADC_GetValue`，取 ch 对应 rank
  的 12bit 码 0..4095；ch 用 `BSP_ADC_CH_IN8_PB0/_CH_IN10_PC0/_CH_IN14_PC4/_CH_VREFINT`
  选择器（值即通道号）。丢 rank（扫描跑赢轮询）→ OVR 置位 → 返回 0（EOC 时序纪律：
  rank 必须落地即取）。
- `uint32_t bsp_adc_read_mv(uint32_t ch)` — mv = raw×3300/4096（量化步距 0.806 mV；
  F4 无出厂 VREFINT_CAL，电源轨保持 3.3 V 假设，由 Vrefint 窗把守）。
- `const char *bsp_adc_last_report(void)` — 上次自测数据串（console-free，rule #7）。
- `int bsp_adc_selftest(void)` — 步 1 配置回读；步 2 全扫描 + Vrefint ∈ [1150,1280] mV。

```c
#include "bsp_adc.h"
uint32_t mv = bsp_adc_read_mv(BSP_ADC_CH_IN10_PC0); /* PC0 上的电压 (mV) */
```

寄存器概念 → HAL 调用映射：SQR3 SQ1 每读重指 → 固定 4-rank 扫描取指定 rank；
CR2 SWSTART=1 → `HAL_ADC_Start`；轮询 SR EOC → `HAL_ADC_PollForConversion`；
读 DR（清 EOC）→ `HAL_ADC_GetValue`；TSVREFE → ConfigChannel(VREFINT)。

## 生成器盲区备注（如有）

- ADC IP 激活必须 GUI 一次（勾 IN8 + File→Save Project 菜单保存）；通道参数/Vrefint
  之后全部文本补齐，roundtrip(saveas) + regen 生成判据（知识 #17）。
- HAL_ADC_Init 不置 ADON（知识 #18.1）——Start 才上电；自检回读的是 TSVREFE/SQR/
  SMPR/MODER，不含 ADON。
- CH10 480 cycles 修正在 bsp_adc_init 运行时重编（不在 Core——regen 会冲掉 Core 侧
  手改），regen 稳定；.ioc 里的生成值保持 3 cycles 教学值。

## 自测验证

菜单 `adc`：**PASS (vrefint 1205mV, vdda 3312mV)**（test-report §2）——4-rank 扫描
CH8/CH10/CH14+Vrefint；Vrefint 窗判 PASS，报告回填 vrefint/vdda 两个数。
闭环佐证：`dac_adc_loop` **PASS**（PA4→PC0 线，501/1657/2802 vs 500/1650/2800，≤8 mV）。

## 对应视频集数

EP08（ADC 采集原理：采样时间/单次转换/EOC 轮询）、EP09（DAC+ADC 模拟闭环，Vrefint 教学点）。
