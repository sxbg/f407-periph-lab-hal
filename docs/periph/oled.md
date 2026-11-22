# bsp_oled — SH1107 OLED 软 I2C 显示（PD9=SCL / PE15=SDA）（EP12）

HAL 版 OLED：**无 CubeMX 外设**——GPIO 开漏位拍软 I2C（~400 kHz，DWT CYCCNT 半位
定时）+ SH1107 命令流 + font5x7 帧缓冲 + 4 行×16 字符文本 API。位拍引擎
i2c_soft_bus（Radio 实证代码 1:1 保留）是 A3 豁免的教学主体；菜单命令 `oled`。

## CubeMX GUI 配置步骤

**无外设可配**：软 I2C 与 SH1107 都不在 .ioc（知识 #13：adapter 全权外设不入
IOC）。仅有的 .ioc 痕迹是两个引脚的占位：

1. PD9、PE15 在 .ioc 中保持 GPIO_Analog（标签 LCD_D14/LCD_D12——LCD 34 针座的
   防冲突占位；4 针 OLED 变体插同一排针轨）。MX_GPIO_Init 对它们不写任何有效配置。
2. 引脚的真实配置由 `i2c_soft_bus_init()` 运行时施加：Output Open-Drain + Pull-up +
   Speed High（HAL 等价 `GPIO_MODE_OUTPUT_OD + GPIO_PULLUP + GPIO_SPEED_FREQ_HIGH`，
   引擎内为寄存器形态）+ 两线释放空闲高。每次 bsp_oled_init 幂等施加，CubeMX
   生成/再生成永不冲突。
3. 总线速率 ~400 kHz：半位 = SystemCoreClock/(400×2000) = 210 cycles @168 MHz，
   DWT CYCCNT 定时（1 字节 8 数据+1 ACK = 22.5 µs）。位定时不能给 SysTick——
   COUNTFLAG 粒度 1 ms 是四百个位周期；SysTick 只管 DC-DC 100 ms 稳压窗。
4. NVIC/DMA：无（轮询位拍；一页 129 字节一事务）。
5. 协议要点：面板地址 0x3C（备选 0x3D），线上字节 = addr<<1 = 0x78/0x7A；控制字节
   bit7 Co=0、bit6 D/C#：0x00=命令流 / 0x40=数据流（地址段之后的"语法"是器件自己的）。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| PD9 | SCL（.ioc 标签 LCD_D14） | 运行时配：Output Open-Drain + Pull-up + High |
| PE15 | SDA（.ioc 标签 LCD_D12） | 同上 |

开漏线与物理：写 1=释放（上拉回高）、写 0=拉低，主机永不"推高"——SDA 冲突、时钟
拉伸物理上不成立（wired-AND）；IDR 读真实线电平（START 检查与 ACK 槽建立在这一点
上）。上拉以模块自带 4.7–10 kΩ 为主，内部 ~40 kΩ 仅兜底。

## HAL API 调用清单

**i2c_soft_bus（引擎，A3 豁免的教学主体——直接 BSRR/IDR 位拍保边沿精度，逐字节不变
保留自寄存器版；bsp_oled.c 本体保持 HAL 干净：HAL 时钟宏 + HAL_GPIO_ReadPin 回读）**：

- `i2c_soft_bus_init(bus, scl_port, scl_pin, sda_port, sda_pin, khz)` — 配脚 +
  推半位周期 + 释放两线空闲高（调用方先开 GPIO 端口时钟）。
- `i2c_soft_bus_write(bus, addr7, data, len)` — START,(addr7<<1)|W,len 字节,STOP；
  每字节采 ACK，NACK 即停载荷、STOP 照发（总线不悬在事务中间）。
- `i2c_soft_bus_probe(bus, addr7)` — 地址探测（器件在位检测）。
- `i2c_soft_bus_recover(bus)` — 9 脉冲 + STOP 经典解缠（目标卡在字节中段时）。
- `i2c_soft_bus_delay_ms(ms)` — SysTick COUNTFLAG 毫秒等待（死 tick 有界退出）。

**bsp_oled（应用层）**：

- `int bsp_oled_init(void)` — 引擎配脚 + 配置回读（MODER/OTYPER/PUPDR + 两线空闲高，
  HAL_GPIO_ReadPin 读 IDR）+ 0x3C/0x3D 探测（每地址一次解缠重试）+ SH1107 初始化
  序列（Radio 实证表，仅 MUX 行 0x7F→0x3F 适配 64 行；0xAD 0x8B DC-DC 后等 100 ms）
  + 清屏帧先刷 + 0xAF 开屏。0=序列已发（**无面板不是错误**——盲写语义）；-1=回读
  不符/线不空闲；-2=线被拉死（写只读总线唯一能看见的真故障）。
- `void bsp_oled_clear(void)` — 帧缓冲（8 页×128 列 RAM 镜像）全零 + 全页标脏。
- `void bsp_oled_puts(uint8_t line, const char *str)` — line 0..3 每行 16 字符
  （行距 16 px：字形页 + 空白页；6 px 步进；小写折叠大写、越界打 '?'；font bit0=
  顶像素与页字节取向一致，免位旋转）。
- `const char *bsp_oled_last_report(void)` / `int bsp_oled_selftest(void)` — 判词串 /
  三步自测（init / 文本路径+fb 校验+全帧刷出 / 等待窗纪律）。

```c
#include "bsp_oled.h"
bsp_oled_puts(0u, "F407-PERIPH-LAB"); /* line 0 */
bsp_oled_puts(1u, "I2C OLED OK");     /* line 1; flush inside selftest */
```

两种 I2C 器件形态（EP11 对照，本模块教学核心）：存储器件（24C02，EP11）事务携带
**器件内地址**（`S 0xA0 字地址 数据 P`，随机读=先设指针再取数）；显示控制器
（SH1107）事务携带的是**流**——控制字节只说"后面是命令/数据"，像素落点由先发的
命令（页/列光标）决定，控制器自推进光标、只画不读。同一根电气总线，地址段之后的
"语法"各是各的。

## 生成器盲区备注（如有）

无（软总线豁免即主题）。唯一所有权重申：PD9/PE15 的引脚配置永远属于
i2c_soft_bus_init（.ioc 保持 Analog 占位），CubeMX 任何生成不会与之冲突。

## 自测验证

菜单 `oled`：**PASS (no display: cmds blind-sent)**（test-report §2，无面板 SKIP
语义）——地址字节照发被 NACK（引擎跳过载荷、STOP 照发），命令流不报错；示波器在
裸总线上仍能看到完整初始化波形（START、0x78、0x00、逐字节）。接线正确后重跑：
`PASS oled (ack@0x3C, text on lines 0-1)` + 面板肉眼对照（行 0 "F407-PERIPH-LAB"、
行 1 "I2C OLED OK"）；帧缓冲侧由软件校验（页 0/2 有墨、其余页全零、脏位清零）。
线被拉死（含一次 9 脉冲解缠重试）才 FAIL step1。

## 对应视频集数

EP12（SPI/外设波次——4 针软 I2C OLED 变体在 W25Q 接线时发现）；与 EP11
（I2C EEPROM，存储器件形态）对照。
