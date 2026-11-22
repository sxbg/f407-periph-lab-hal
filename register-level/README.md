# f407-periph-lab — STM32F407VE 全外设教学工程

《STM32 外设一次讲透》31 集系列（`VideoPipeline\Project\AutoSTM32`）的代码与
配置支持工程。每个外设一个独立 `firmware/App/bsp/<periph>.c/.h`：
寄存器级驱动 + Doxygen 内嵌 CubeMX 配置 + 自测试函数 + UART 菜单验证。

## 构建

```
cd AutoSTM32 根目录
PYTHONPATH=src python -m autostm32.cli build --project projects/f407-periph-lab
```

- 三判据：exit≤1 + `0 Error(s), 0 Warning(s)` + hex mtime 新鲜
- `firmware/Drivers/`（HAL+CMSIS）不入库：clone 后先跑
  `cubegen regen --sync-only`（其 -q 脚本重定向 MDK-ARM 并重建 Drivers）
- **regen 后手术清单见 `docs/设计.md` §4**（HardFault 归属等）

## 上板验证

烧录后串口（板载 USB-TTL，115200 8N1）敲 `help` 列菜单。自测命令逐模块 PASS/FAIL；
`wdg_test`/`pwr_stop_demo` 为复位/低功耗演示命令（上板前读警示打印）。
示波器/逻辑分析仪验证点：PA8+PD12 1kHz PWM、PC6/PC7/PB14 互补+1µs 死区、
PB4/PB5 编码器、PB8/PB9 CAN 500k 回环。

## 文档

- `docs/设计.md` — 82 脚全地图 + 架构 + regen 手术清单
- `docs/periph/<模块>.md` — 每外设 CubeMX 等效配置表/引脚表/寄存器表/集数映射（17 页）
- `docs/bringup-log.md` — 环境 + 踩坑记录（fixed format）
- 系列 31 集状态：`tutorials/PLAN.md`（本工程每模块映射一集，见 docs/periph/README.md）

## 范围

IOC 收录 13 IP（Clock/GPIO/TIM×7/USART×3 + SysTick）；SPI/I2C/CAN/SDIO/FSMC/ADC/
DAC/DMA/RTC/看门狗等 18 项为 **adapter 全权外设**（寄存器级 bsp 模块 + 文档化配置，
CubeMX 等效表在 docs/periph/；决策依据见 AutoSTM32 knowledge #10/#13）。
排除：ETH/DCMI/OTG_HS/UART4/5/FSMC NAND（板无硬件或引脚冲突，docs/periph/README.md）。
