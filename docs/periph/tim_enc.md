# bsp_tim_encoder — TIM3 正交编码器接口（EP23）

TIM3_CH1/CH2=PB4/PB5 正交解码（TI12 ×4 计数）、回绕安全测速与自测。寄存器级
深讲（SMCR/CCMR1/CCER 逐位）看老工程 `docs/periph/tim_enc.md` 表 3，本页只讲
HAL 映射与生成真值。

## CubeMX GUI 配置步骤

1. TIM3 → Mode → **Encoder Interface**，Encoder Mode=**TI12**（.ioc 真值
   `TIM3.EncoderMode=TIM_ENCODERMODE_TI12`；注意生成代码只到
   `HAL_TIM_Base_Init` 基础时基，编码器"形状"由本模块
   `HAL_TIM_Encoder_Init` 接管，见盲区备注）。
2. Parameter Settings：Counter Period=**65535**（满 16 位窗，自由回绕本身就是
   EP23 教学点）；Prescaler=**0**（编码器模式下 PSC 无意义——沿即时钟）；
   AutoReloadPreload=**Enable**。
3. GPIO Setting：PB4=S_TIM3_CH1（标签 **ENC_A**）、PB5=S_TIM3_CH2（**ENC_B**），
   AF2，No pull（模块 init 里补上拉，见 API 清单）。
4. Kernel clock：APB1 ×2 = 84 MHz——只有输入滤波器吃这个时钟，计数器本体由
   编码器沿驱动。
5. 生成代码真值（tim.c）：`htim3.Init.Prescaler=0 / Period=65535 /
   AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE`，仅基础 init。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| PB4 | TIM3_CH1 / ENC_A | 输入（AF2，编码器 A 相；init 后补 GPIO_PULLUP） |
| PB5 | TIM3_CH2 / ENC_B | 输入（AF2，编码器 B 相；同上） |

## HAL API 调用清单

- `int bsp_tim_encoder_init(void)` — 编码器形状 init：`HAL_TIM_Encoder_Stop`
  （幂等重入，BUSY 上 Start 是 HAL_ERROR）→ `HAL_GPIO_Init` 补 ENC_A/ENC_B
  上拉（IOC 留 No pull，悬空线在阈值附近晃会出幻边）→
  `HAL_TIM_Encoder_Init`（TI12、DIRECTTI、IC 滤波=3≈762 ns 去抖、DIV1、
  RISING）→ `HAL_TIM_Encoder_Start(TIM_CHANNEL_ALL)`（含 CC1E/CC2E/CEN，
  无中断——轮询课）。返回 0=PASS（SMCR/CCMR1/CCER/ARR 只读回读核验，
  A3 豁免），-1=回读不符。

典型用法：

```c
bsp_tim_encoder_init();
uint32_t cnt = bsp_tim_encoder_read_count();   /* 位置（模 16 位一圈） */
int32_t  rpm = bsp_tim_encoder_speed_rpm(200); /* 200 ms 窗测速 */
/* rpm = delta * 60000 / (4 * PPR * win_ms)，delta 为 (int16_t) 回绕折叠：
   反转=负 rpm；窗内计数勿超 |32767|，否则 16 位回绕混叠符号（EP23 走查）。 */
```

- `uint32_t bsp_tim_encoder_read_count(void)` — 原始 16 位计数
  （`__HAL_TIM_GET_COUNTER`），0<->65535 自由回绕，即转角模一圈读数。
- `int bsp_tim_encoder_set_ppr(uint32_t ppr)` — rpm 换算用每转脉冲数
  （默认 1024），合法 1..65535；0 拒绝返回 -1 不改旧值。
- `int32_t bsp_tim_encoder_speed_rpm(uint32_t win_ms)` — 采样-`HAL_Delay`-
  采样，回绕安全 signed delta 测速（win_ms≥1）；无信号返回 0。
- `const char *bsp_tim_encoder_last_report(void)` — 上次自测的一行报告
  （"(idle)" 或 "(moved, N cts/300ms)"），菜单拼在判定行后（返回数据而非
  打印，模块间零依赖）。
- `int bsp_tim_encoder_selftest(void)` — 返回 0=PASS，否则失败步号：
  1=配置回读（SMS=011/双通道映射+滤波/CCER 使能/ARR/CEN）
  2=静止 200 ms 必须 delta=0（上拉+滤波下幻边计数即布线/噪声问题）
  3=转动探测 300 ms（有信号报计数、无信号报 idle，都 PASS）。

## 生成器盲区备注

.ioc 记录 Encoder Interface/TI12 意图，但生成代码只输出基础时基 init
（`MX_TIM3_Init` 内无 `HAL_TIM_Encoder_Init`）——编码器形状归 bsp 模块
接管，属本工程的**有意分工**（与老工程"模块接管"同构），非缺陷；复现/恢复
直接调 `bsp_tim_encoder_init()` 即可。

## 自测验证

菜单命令：`tim_enc`；本轮实测：**PASS (idle)**（编码器静止，SKIP 语义——
未接编码器不是缺陷）。接上编码器后步 2 静止判据仍过，手转轴报
"(moved, N cts/300ms)"，`speed_rpm(200)` 给带符号 rpm。

## 对应视频集数

EP23（定时器编码器接口与电机测速）。
