# docs/periph — 每外设一页索引

每页三表：CubeMX 等效配置 / 引脚 / 寄存器清单，加对应视频集数。
配置真值来源：.ioc 里有的 IP 抄 .ioc；adapter 全权外设（知识 #13，.ioc 无此 IP）
抄各 `firmware/App/bsp/<module>.c` 文件头的 CUBEMX CONFIG 注释。

时钟树（全工程共用）：SYSCLK 168 MHz / APB1 42 MHz（定时器 ×2=84）/ APB2 84 MHz
（定时器 ×2=168）/ PLLQ 84 MHz。菜单入口：`firmware/App/shell/test_menu.c`。

## 已实现 21 模块

| 菜单命令 | 模块 | 一句话 | 硬件/器件 | 集数 |
|---|---|---|---|---|
| uart | bsp_uart | USART1 115200 8N1 控制台（BRR/TC/RXNE） | 板载 USB-TTL | EP03 |
| gpio | bsp_gpio | PC5/PB2 LED + PA0/PA1 按键 + PA15 背光（MODER/BSRR/IDR） | LED×2、KEY×2 | EP02 |
| tim_pwm | bsp_tim_pwm | TIM1_CH1=PA8 + TIM4_CH1=PD12 1 kHz PWM | 板头排针 | EP06 |
| tim_basic | bsp_tim_basic | TIM5 1s 中断 + TIM6/TIM7 轮询时基 | 无引脚 | EP04 |
| tim_ic | bsp_tim_ic | TIM2_CH2=PB3 输入捕获频率计（32 位 1 s 窗） | 板头排针 | EP13 |
| tim_enc | bsp_tim_encoder | TIM3 编码器 ×4（PB4/PB5，CNT 回绕 rpm） | 板头排针 | EP23 |
| tim_adv | bsp_tim_adv | TIM8 CH1/CH2/CH2N 互补 PWM + 1 µs 死区（DTG/MOE） | 板头排针 | EP14 |
| can_lb | bsp_can_loopback | CAN1 PB8/PB9 500k 回环（INAK 握手/帧收发/过滤器） | TJA1050 | EP18 |
| crc_rng | bsp_crc_rng | CRC32 MPEG-2 向量 + RNG DRDY/FIPS（CECS 告警点） | 无引脚 | EP20 |
| rtc | bsp_rtc | RTC LSE 日历 + Alarm A + 20×BKPxR | 32.768k 晶振 + CR1220 | EP16 |
| wdg | bsp_wdg | IWDG ~2s + WWDG 窗口 + 复位源旗标 | LSI 内部 | EP15 |
| pwr | bsp_pwr | Sleep/Stop/Standby（Stop=KEY1 唤醒 demo） | KEY1=PA0 | EP17 |
| flash_iap | bsp_flash_iap | 解锁+擦扇区 7+参数页 0x0807FC00 | 片内 Flash | EP21 |
| i2c_ee | bsp_i2c_eeprom | I2C1 100k PB6/PB7 + 24C02@0x50 字节写读 | 24C02（焊死） | EP11 |
| spi_flash | bsp_spi_flash | SPI1 模式0 5.25 MHz + W25Q128 JEDEC/擦/写/读 | W25Q128 16MB（字库区红线） | EP12 |
| sdio_tf | bsp_sdio_tf | SDIO 识别链 CMD0/8/41/2/3/9（识别 396.2 kHz） | TF 卡座（可拔） | EP24 |
| adc | bsp_adc | ADC1 SWSTART/EOC 轮询 PB0/PC0/PC4+Vrefint（SMPR/SQR/CR2，42 MHz 告警点） | 板头排针 + 内部基准 | EP08/EP09 |
| dac | bsp_dac | DAC1 OUT1=PA4 软件模式（DHR12R1→DOR 三点回读） | 板头排针 | EP09 |
| dma | bsp_dma | DMA2 S6 mem2mem + S7/Ch4 USART1_TX（TCIF/NDTR/DMAT 借还） | 无引脚（走 PA9） | EP07 |
| exti | bsp_exti | EXTI1=KEY2 PA1 下降沿计数（交互式 6 s 窗，EXTICR/IMR/FTSR/PR） | KEY2=PA1 | EP10 |
| oled | bsp_oled | SH1107 128x64 软 I2C PD9/PE15（GPIO 位拍 400k 盲写 + font5x7 帧缓冲） | 4 针 OLED 模块（外接） | EP12 |

### 杜邦线闭环实测（shell 层组合命令，需先接线）

| 菜单命令 | 接线 | 流程 | 判据 | 集数 |
|---|---|---|---|---|
| dac_adc_loop | **PA4→PB0** | DAC 500/1650/2800 mV 三步 → ADC 逐步回读 | 每步 \|Δ\|≤60 mV（同 VDDA，残差=量化+噪声） | EP08/EP09 |
| pwm_ic_loop | **PA8→PB3** | TIM1 1 kHz 50% PWM → TIM2 输入捕获测频 | 950..1050 Hz | EP06+EP13 |

## 文档化未实现外设（教程集数映射，无 bsp 模块）

| 外设 | 原因（pin-budget/无硬件） | 集数 |
|---|---|---|
| USB OTG FS/HS | 双 USB 座在板，IOC 未收（冲突/教学排期） | EP19 |
| FSMC | LCD 34 针座在板，地址线被 SDIO/按键占面广，文档化 | EP27 |
| I2S | 无音频 CODEC，I2S2_CK 与 24C02 冲突 | EP28 |
| ETH / DCMI | F407 专属，但无 PHY/摄像头，且引脚 100% 被占 → 排除 | EP31 |

## 姊妹文档

- `docs/docs/设计.md` — 外设×引脚×CubeMX 配置总表（P1 pin-budget 汇总）
- `docs/docs/bringup-log.md` — 带机日志
- `WorkManger/tasks/0005-f407-periph-lab/pin-budget.md` — 80/82 脚利用矩阵 + 14 冲突裁决
