# bsp_dac — DAC 通道 1 输出（EP09）

HAL 版 DAC OUT1（PA4=DAC_OUT1）：CubeMX 生成软件模式 + 输出缓冲的通道配置；bsp 层
mV 设点 API + DOR 回读自检。菜单命令 `dac`（0/1650/3300 mV 三点 DOR 回读）。

## CubeMX GUI 配置步骤

1. Analog → DAC → 勾 OUT1（PA4）激活 DAC IP（知识 #17：DAC 的信号键 + IP 项在
   custom 文本上同样被拒，激活只有 GUI 一条真路；勾完 File→Save Project 菜单保存，
   Ctrl+S 会被 SWT 焦点吞掉）。
2. OUT1 参数：Output Buffer=ENABLE（低阻驱动杜邦线+ADC 负载；代价是轨边 ~0.2 V
   余量——闭环测试点 500/1650/2800 mV 全在缓冲区内）、Trigger=None（软件模式：
   写 DHR → 1 个 APB1 周期后自动进 DOR，无需 SWTRIG，RM0090 14.3.4）。
3. Wave generation 不用（DC 设点 API；噪声/三角波是带触发才可用的进阶）。
4. NVIC 不用开（无标志可轮询）。
5. OUT2=PA5 不用（W25Q128 CLK 占用；PA4 空闲的前提是 SPI1 用软件 NSS）。

**PA4 引脚模式盲区（知识 #18.4，本模块必教）**：.ioc loader 给 PA4 遗留的是 AF 模式
而非 Analog——DAC 模拟输出到不了引脚（引脚电平纹丝不动，闭环读恒 0）。修复不在
Core：`bsp_dac_init` 自持 `HAL_GPIO_Init(GPIOA, GPIO_MODE_ANALOG)`（HAL 调用，
A3 干净），每次 init 施加、regen 稳定。实测佐证：示波器 PA4=2.8 V（第三 setpoint，
闭环运行时波形实时跟随序列）。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| PA4 | DAC_OUT1（DAC_OUT1_Pin） | Analog（生成器给 AF → bsp_dac_init 自配 Analog，见盲区） |

## HAL API 调用清单

- `int bsp_dac_init(void)` — 只读回读通道 CR 形状（TEN1/TSEL1/WAVE1/MAMP1/BOFF1
  全 0：软件触发、缓冲开、无波形；HAL_DAC_ConfigChannel 刻意留 EN1=0）+ 自配 PA4
  Analog + `HAL_DAC_Start`（置 EN1 唤醒模拟宏单元，1 ms 盖过 us 级 tWAKEUP；DHR
  复位默认 0 → PA4 空闲 0 V）。0=PASS，-1=不匹配/Start 失败。
- `uint32_t bsp_dac_set_mv(uint32_t mv)` — code = round(mv×4096/3300)（即
  (mv×4096+1650)/3300），钳位 4095（RM 公式满量程饱和即此路径）；经
  `HAL_DAC_SetValue(..., DAC_ALIGN_12B_R, code)` + `HAL_DAC_Start` 装载，返回实际
  装载的码。
- `int bsp_dac_selftest(void)` — 步 1 CR+引脚回读；步 2 三点 set→DOR 回读：
  0/1650/3300 mV → DOR1 = 0/2048/4095（端点证明轨道映射与钳位路径；DOR 才是模拟级
  真正看到的值，DHR 只是保持寄存器）。

```c
#include "bsp_dac.h"
bsp_dac_set_mv(1650u); /* PA4 = mid-rail */
```

寄存器概念 → HAL 调用映射：CR EN1=1 → `HAL_DAC_Start`（通道唤醒）；DHR12R1 写 →
`HAL_DAC_SetValue`；DOR1 回读 → `HAL_DAC_GetValue`；MODER 模拟写 → 生成器 MspInit
（只读回读）+ bsp_dac_init 运行时补写（盲区）。

## 生成器盲区备注（如有）

PA4 AF 模式遗留（上文，知识 #18.4）——修在 bsp 层 HAL_GPIO_Init(Analog)，regen
稳定。其余无：通道配置本体（Trigger=None / OutputBuffer=Enable）由生成器正确渲染。

## 自测验证

菜单 `dac`：**PASS**（test-report §2，DAC OUT1 软件模式回读）。闭环佐证：
`dac_adc_loop` **PASS**（PA4→PC0 杜邦线，DAC 500/1650/2800 mV vs ADC
501/1657/2802 mV，≤8 mV——同 VDDA，残差=量化+噪声）；示波器 PA4=2.8 V
（第三 setpoint）。

## 对应视频集数

EP09（DAC 输出与模拟闭环；缓冲/触发/波形生成三开关教学）。
