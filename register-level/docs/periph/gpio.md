# bsp_gpio — GPIO：LED/按键/背光（EP02）

寄存器级 GPIO 读写：PC5/PB2 LED、PA0/PA1 按键、PA15 LCD 背光。菜单命令 `gpio`。
这些脚都在 .ioc 里（HAL MX_GPIO_Init 已按 IOC 配置），本模块做寄存器级接管+回读教学。

## 表 1 — CubeMX 等效配置（.ioc 真值）

| 字段 | 值（IOC） | 说明 |
|---|---|---|
| PC5 LED_GREEN | GPIO_Output，PP，Low speed，初值 RESET | 菜单心跳灯（app_config.h 引用） |
| PB2 LED_BLUE | GPIO_Output，PP，Low speed，初值 SET | 第二状态灯（开机即亮） |
| PA0 KEY1 | GPIO_Input，Pull-down | 按下接 VDD → 高有效 |
| PA1 KEY2 | GPIO_Input，Pull-up | 按下接 GND → 低有效 |
| PA15 LCD_BL | GPIO_Output，PP，Low speed，初值 RESET | LCD 背光使能 |
| 端口时钟 | GPIOA/B/C = AHB1ENR（HAL 已做） | 本模块再武装（幂等） |
| PA15 告警 | PA15 默认 AF0 = JTAG JTDI | MODER=01 即重路由；SWD（PA13/14）不受影响 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PC5 | — | 输出 | LED_GREEN（绿灯） |
| PB2 | — | 输出 | LED_BLUE（蓝灯） |
| PA0 | — | 输入（下拉） | KEY1（KEY/WKUP 共用） |
| PA1 | — | 输入（上拉） | KEY2 |
| PA15 | — | 输出 | LCD 背光 BL |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->AHB1ENR | GPIOAEN/GPIOBEN/GPIOCEN | AHB1 端口时钟门控 |
| GPIOx->MODER | 输出 01 / 输入 00 | 方向选择（PA15/PA15 类 JTAG 重路由点） |
| GPIOx->OTYPER | 0 = 推挽 | LED/BL 均推挽 |
| GPIOx->OSPEEDR | Low speed | 电平类信号无需高速度 |
| GPIOx->PUPDR | PA0=10 下拉 / PA1=01 上拉 | 按键基准电平 |
| GPIOx->IDR | bit 读 | 按键采样 |
| GPIOx->ODR / BSRR | 读改写 / 原子置清 | ODR 读参与翻转（ISR 安全）；BSRR 单写生效 |

## 对应视频集数

EP02（GPIO 输入输出与按键）。
