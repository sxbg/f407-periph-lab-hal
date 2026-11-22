# f407-periph-lab 验证测试报告

> 日期：2026-11-21 · 环境：F407VE dev board v5.5 / 板载 CMSIS-DAP / COM18 115200 8N1
> 固件：16 bsp+engine 模块全量构建（0 err 0 warn，hex 91.9KB），Keil UV4 -f 烧录 Verify OK
> 自动化：`tools/menu_autotest.py`（pyserial 驱动串口菜单，逐命令 PASS/FAIL 判据）
> 结论：**21 模块全部 PASS（含 2 项杜邦线闭环实测）**；spi_flash 最终定位为两处
> 寄存器移植缺陷（引脚模式值），修复后 `PASS spi_flash 9F:EF4018 page ok`——芯片
> 与硬件自始正常，详见 §3.3

## 1. 结果总表

| 命令 | 模块 | 结果 | 实测证据 |
|---|---|---|---|
| uart | USART1 | **PASS** | BRR 回读 0x2D9（115226 bps，+0.023%） |
| gpio | GPIO | **PASS** | 寄存器回读 + KEY 静止电平 + LED 目视 |
| tim_pwm | TIM1/TIM4 | **PASS** | PSC/ARR/CCR/MOE 回读；PA8/PD12 = 1kHz |
| tim_basic | TIM5/6/7 | **PASS** | 1s 中断计数 + UIF 轮询置位/清除 |
| tim_ic | TIM2 | **PASS** | 配置回读（无信号 SKIP 语义） |
| tim_enc | TIM3 编码器 | **PASS** | SMS=011 回读；idle 200ms ΔCNT=0 |
| tim_adv | TIM8 互补+死区 | **PASS** | DTG=0x94、MOE、CC2NE 回读 |
| can_lb | CAN1 bxCAN | **PASS** | INAK 握手 + 回环帧 ID=0x123 比对 + TSR 标志链 |
| crc_rng | CRC+RNG | **PASS** | "12345678"→0x49E3C2FB 硬软对拍；RNG 8 连读 |
| rtc | RTC 备份域 | **PASS** | LSE ok；秒走 1；BKP0R=0xA5A55A5A 保留 |
| wdg | IWDG+WWDG | **PASS** | 装载 2s→改 8.19s 装载；复位标志快照/清零 |
| pwr | PWR | **PASS** | Sleep 100ms 实测（SysTick 唤醒） |
| flash_iap | Flash 接口 | **PASS** | 末扇区擦除+参数页 8 字写读回+保留 |
| i2c_ee | I2C1 软引擎+24C02 | **PASS** | 0x55/0xAA 读写回环（软 I2C 引擎，见 §3.2） |
| spi_flash | SPI1+W25Q128 | **PASS** | JEDEC=EF 40 18 + 擦除→全 FF→编程→回读闭环；终局根因 = PA6 未进 AF（引擎恒读 0）+ PC13 写成 AF（CS 未跟随 ODR），详见 §3.1/§3.3 |
| sdio_tf | SDIO | **PASS (no card)** | 识别链就绪；插卡即出容量 |
| adc | ADC1 | **PASS** | Vrefint 1202mV（反推 VDDA 3317mV） |
| dac | DAC | **PASS** | DOR 三点回读（0/2048/4095 含钳位） |
| dma | DMA2 | **PASS** | S6 mem-to-mem 64 字 + S7 串口 DMA-TX-OK |
| exti | EXTI1←KEY2 | **PASS (4 presses)** | 累计计数制（用户实按 3 次 + 1 边沿）；教训：EXTI 初始化在首次 `exti` 运行时才生效——"先跑一次配置，再按键，再跑一次收证据"两段式（已写入 docs/periph/exti.md） |
| dac_adc_loop | DAC↔ADC 闭环 | **PASS** | 502/1654/2806 mV，最大偏差 6mV（线 PA4→PB0） |
| pwm_ic_loop | PWM↔IC 闭环 | **PASS** | 实测 1000Hz @目标 1000（线 PA8→PB3） |

## 2. 杜邦线闭环实测数据（三角验证之"板上测到什么"）

**PA4→PB0（DAC→ADC）**：VDDA 以 Vrefint 反推 3317mV

| DAC 设定 | ADC 实读 | 偏差 |
|---|---|---|
| 500 mV | 502 mV | 2 mV |
| 1650 mV | 1654 mV | 4 mV |
| 2800 mV | 2806 mV | 6 mV |

**PA8→PB3（PWM→输入捕获）**：目标 1000 Hz，实测 1000 Hz（1s 窗口，±50Hz 判据）。

## 3. 验证过程中发现并修复的缺陷（全部留痕）

### 3.1 软件缺陷（已修复，修复后复测 PASS）
1. **wdg 装载模式不可能**：未启动的 IWDG 上 PVU/RVU 永不清零、PR/RLR 读回旧值——
   改为"配置→启动 0xCCCC→同步等待→回读"，菜单循环喂狗（真实产品模式）；
   RLR 增大到 4095（8.19s 标称）覆盖 flash 擦除的 2.6s 静默期（PRIMASK 不挡 LSI）。
2. **I2C 等待语义错误**：`ee_wait_sr1` 相等判据使 ACK 成功路径（仅 ADDR 置位）
   被误判为 10ms 停摆——改为等待掩码任一置位，调用方自行区分。
3. **SPI 片选反相**：`sf_cs_low/high` 两函数 BSRR 半区写反（W25Q 从未被选中，
   JEDEC 读 000000）——交换后时序正确。
4. **DMA M2M 通道模式**：RM0090 10.3.6 明文 mem-to-mem 禁止 direct/circular——
   S6 用 FIFO 模式（任务书笔误按手册修正）。
5. **WWDG EWIF 语义**：停靠态（T=0）下 EWIF 必然置位，不作失败判据（init 清一次，
   上板期间重置位属预期）。
6. **SPI MISO 未进 AF 模式（spi_flash 死寂主因）**：PA6 写 `MODER=00`（普通输入），
   RM0090 §8.3.2 要求外设输入脚 `MODER=10`（AF）——SPI1_MISO 输入未路由，引擎
   恒读内部 0（引脚实际被板上上拉为高仍读 000000）。改为 AF 后 JEDEC 立即返回。
7. **CS 引脚模式值写错（CMSIS 字段常量陷阱）**：PC13 写 `GPIO_MODER_MODER13_1`
   （字段 bit1 = `0b10` = AF）而非 `_0`（`0b01` = 输出），注释却写"01 output"——
   CS 从未跟随 ODR，驱高后 IDR 仍为低（新判据 `csh:00` 现形）。修复后芯片正常选中。
   附带修复：`s_report[32]` 无界追加溢出 → 192 字节 + 全边界检查；软位拍探针未切
   MODER（AF 下 BSRR 拨不动引脚）→ 切 GPIO 输出模式，A/B 才成立；init 读回新增
   **引脚模式断言**（PC13=01、PA5/6/7=10），该类错误今后在 step1 直接 FAIL。

### 3.2 架构级变更（硬件 I2C → 软 I2C，用户"借鉴 Radio"决策）
硬件 I2C 随机读在真板上非确定停摆（100k/50k 均复现，跨开机漂移，5 轮修复未根治）。
按 Radio 实证方案移植软 I2C 引擎（`i2c_soft_bus.c` 逐行照抄 Radio soft_i2c.c：
开漏位拍、SCL 高期间采样 ACK、9 时钟恢复、DWT 半位延时），上板一次通过。
**教学价值**：这正是 EP11"硬件 I2C vs 软 I2C"选型的真实案例——同一块板、同一颗
EEPROM、两种引擎、软的赢了。

### 3.3 spi_flash 终局定位（软件误判 → 真因修复，硬件自始正常）

用户全程坚持"硬件没有问题"——正确。定位链：逻辑分析仪证明 MCU 发码完美
（CLK + `9F 00 00 00`），按数据手册补全 0xAB 深度掉电唤醒、mode3、0x90、
软位拍五路探测后，软位拍三轮返回 `171717/8B8B8B/C5C5C5`——**同轮内四类命令
同值、跨轮随机**，是浮空线串扰噪声（真芯片 9F 应答必为三个不同字节）。
新增 `csh:`（驱高后 IDR）判据当场现形：`csh:00` = CS 驱高失败。最终两处
根因（详见 §3.1 #6/#7）：PA6 未进 AF 模式（引擎恒读 0）+ PC13 误写 AF 模式
（CMSIS `_1` 常量陷阱）。修复后一次通过：`PASS spi_flash 9F:EF4018 page ok`。
遗留处置项：无。

### 3.4 ADCCLK 告警（非阻塞）

ADCCLK=PCLK2/2=42MHz 超数据手册 36MHz 上限（采样仍可用，精度略降）——
EP08 教学告警点，合规修法 ADCCPR=01→21MHz 一行即改。

## 4. 待完成项

无。exti 已于 2026-11-21 21:40 完成交互验证（**PASS exti (4 presses)**，累计
计数制）；spi_flash 同日收官。**终态：21/21 PASS，含 2 项杜邦线闭环实测。**

## 5. 构建记录

| 轮 | 结果 | 备注 |
|---|---|---|
| 首建 | 0 err 0 warn | 骨架 + HardFault 手术 |
| P3a-P3d 五波次 | 每轮 0 err 0 warn | 16 模块递增 |
| P3e/P3f/P3g | 每轮 0 err 0 warn | +4 模块 + OLED + 软 I2C 引擎 |
| 终版（本报告） | 0 err 0 warn，Code=29346 | hex 91.9KB，UV4 Verify OK |
| spi 诊断增强（0xAB/mode3/90 + soft 修正） | 0 err 0 warn | runs/build_20261121_225420.log，UV4 Verify OK |
| **spi 终局修复（PA6→AF、PC13→输出、引脚断言）** | 0 err 0 warn | runs/build_20261121_230741.log，UV4 Verify OK；`PASS spi_flash 9F:EF4018 page ok` |
