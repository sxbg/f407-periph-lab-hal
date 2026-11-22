# docs/periph — HAL 版外设教学文档索引

每页固定结构：CubeMX GUI 配置步骤 / 引脚 / HAL API 调用清单 / 生成器盲区备注 /
自测验证 / 对应视频集数。配置真值：`firmware/Core/Src/<periph>.c`（生成器产物）+
`firmware/App/bsp/<module>.c`。本文档形态为「CubeMX GUI 配置步骤 + HAL API 调用清单 +
生成器盲区备注」，寄存器级深度讲解见 f407-periph-lab（寄存器版）同名文档。
生成器盲区汇总见 knowledge/pitfalls-cubemx-keil.md #17/#18。

时钟树（全工程共用）：HSE 25 MHz → SYSCLK 168 MHz / APB1 42 MHz（定时器 ×2=84）/
APB2 84 MHz（定时器 ×2=168）/ PLLQ 48 MHz。菜单入口：`firmware/App/shell/test_menu.c`。

## 21 个外设教学模块

| 文件 | 外设 | EP 集数 | 菜单命令 |
|---|---|---|---|
| uart.md | bsp_uart — USART1 115200 8N1 控制台 | EP03 | uart |
| gpio.md | bsp_gpio — LED×2 + KEY×2 + 背光 | EP02 | gpio |
| tim_pwm.md | bsp_tim_pwm — TIM1_CH1/TIM4_CH1 1 kHz PWM | EP06 | tim_pwm |
| tim_basic.md | bsp_tim_basic — TIM5 1s 时基 | EP04 | tim_basic |
| tim_ic.md | bsp_tim_ic — TIM2_CH2 输入捕获频率计 | EP13 | tim_ic |
| tim_enc.md | bsp_tim_encoder — TIM3 编码器 ×4 | EP23 | tim_enc |
| tim_adv.md | bsp_tim_adv — TIM8 互补 PWM + 死区 | EP14 | tim_adv |
| can_lb.md | bsp_can_loopback — CAN1 500k 回环 | EP18 | can_lb |
| crc_rng.md | bsp_crc_rng — CRC32 MPEG-2 + RNG | EP20 | crc_rng |
| rtc.md | bsp_rtc — RTC 日历 + Alarm + BKPxR | EP16 | rtc |
| wdg.md | bsp_wdg — IWDG + WWDG + 复位源旗标 | EP15 | wdg |
| pwr.md | bsp_pwr — Sleep/Stop/Standby（KEY1 唤醒） | EP17 | pwr |
| flash_iap.md | bsp_flash_iap — 解锁+擦扇区+参数页 | EP21 | flash_iap |
| i2c_ee.md | bsp_i2c_eeprom — I2C1 + 24C02@0x50 | EP11 | i2c_ee |
| spi_flash.md | bsp_spi_flash — SPI1 + W25Q128 | EP12 | spi_flash |
| sdio_tf.md | bsp_sdio_tf — SDIO TF 识别链 | EP24 | sdio_tf |
| adc.md | bsp_adc — ADC1 4-rank 扫描 + Vrefint | EP08/EP09 | adc |
| dac.md | bsp_dac — DAC OUT1=PA4 软件模式 | EP09 | dac |
| dma.md | bsp_dma — DMA2 Stream7/Ch4 = USART1_TX | EP07 | dma |
| exti.md | bsp_exti — EXTI0/EXTI1 双键中断 | EP10 | exti |
| oled.md | bsp_oled — SH1107 软 I2C PD9/PE15 | EP12 | oled |

## 杜邦线闭环命令（shell 层组合命令，需先接线）

| 菜单命令 | 接线 | 判据 | 集数 |
|---|---|---|---|
| dac_adc_loop | PA4→PC0 | DAC 500/1650/2800 mV vs ADC ≤8 mV | EP08/EP09 |
| pwm_ic_loop | PA8→PB3 | 实测 950..1050 Hz | EP06+EP13 |

注：oled.md 的位拍引擎 i2c_soft_bus 为 A3 豁免的教学主体（bsp/ 共享文件，非独立页）。
上板判定全文见 `docs/test-report.md`（20 PASS + wdg KNOWN + 闭环 3 项）。
