# bsp_oled — SH1107 128x64 OLED 软 I2C（PD9=SCL / PE15=SDA）（EP12）

寄存器级 **GPIO 位拍软 I2C**（无 I2C IP，~400 kHz，DWT CYCCNT 半位定时）+ SH1107
命令流 + font5x7 帧缓冲 + 4 行×16 字符文本 API。菜单命令 `oled`。
借鉴 Radio 工程 `adapters/sh1107` + `adapters/soft_i2c` 的实证代码（时序、控制字节、
初始化序列、取模方式），拍平进单模块；`font5x7.h` 逐字节复制自 Radio。
软 I2C 与 SH1107 均不在 .ioc（知识 #13：adapter 全权外设），PD9/PE15 在 .ioc 里只是
GPIO_Analog 占位（标签 LCD_D14/LCD_D12，防冲突），输出开漏配置由本模块写入。

## 表 1 — CubeMX 等效配置（无此 IP；模块为真值源）

| 字段 | 值 | 说明 |
|---|---|---|
| I2C IP | 无（GPIO 位拍模拟） | 显示控制器只需写流，教学取"看得见每一根线"的形态 |
| PD9 模式 | Output Open-Drain + Pull-up + High | MODER[19:18]=01 / OTYPER bit9=1 / PUPDR[19:18]=01 / OSPEEDR[19:18]=11 |
| PE15 模式 | Output Open-Drain + Pull-up + High | MODER[31:30]=01 / OTYPER bit15=1 / PUPDR[31:30]=01 / OSPEEDR[31:30]=11 |
| 线速 | ~400 kHz（半位 210 cycles @168 MHz） | half = SystemCoreClock/(400×2000)；1 字节（8 数据+1 ACK）=22.5 µs |
| 位定时源 | DWT CYCCNT（DEMCR TRCENA + CTRL CYCCNTENA） | SysTick 粒度 1 ms，管不了 µs 级线拍；ms 级等待（DC-DC 稳压窗）仍用 SysTick COUNTFLAG+guard |
| ACK 策略 | 探测时读 ACK，其余盲写 | 屏可能没插；NACK 不判 FAIL（SKIP 语义 `(no display)`） |
| NVIC / DMA | 无 | 轮询位拍；一页 129 字节一事务 |
| Radio HAL 等价 | `GPIO_MODE_OUTPUT_OD + GPIO_PULLUP + GPIO_SPEED_FREQ_HIGH` | Radio `sh1107_port.c` 即此调用，本模块展开为寄存器写 |

## 表 2 — 引脚

| 脚 | 方向 | 板载去向 | 说明 |
|---|---|---|---|
| PD9 | 开漏输出/输入（SCL） | LCD 34 针座 D14 位（FMC_D14 位线） | 4 针 OLED 变体插同排针轨（EP12 现场发现）；标签 LCD_D14 防冲突 |
| PE15 | 开漏输出/输入（SDA） | LCD 34 针座 D12 位（FMC_D12 位线） | 同上，标签 LCD_D12 |

开漏要点：写 1 = 释放（上拉回高），写 0 = 拉低；主机永远不"推高"，这是 I2C
线与物理（wired-AND）——SDA 冲突、时钟拉伸在物理上不成立。上拉以模块自带
4.7–10 kΩ 为主，内部 ~40 kΩ 仅兜底（同 24C02 结论）。

## 表 3 — 寄存器 / SH1107 命令清单

GPIO 侧：`RCC->AHB1ENR` bit3/4（GPIOD/EN）；PD9、PE15 各自 `MODER/OTYPER/OSPEEDR/
PUPDR`；`BSRR`（置 1 释放、位 16+ 置 0 拉低，原子）；`IDR`（读真实线电平——起停
检测与 ACK 槽的"听"）。位定时：`CoreDebug->DEMCR` TRCENA + `DWT->CTRL` CYCCNTENA +
`DWT->CYCCNT`（210 cycles 半位，死 DWT 有 spin 上限）。

I2C 帧结构：`S 0x78(或0x7A) [控制字节] [字节...] P`。控制字节 bit7 Co=0（后续同类）、
bit6 D/C#：**0x00=命令流 / 0x40=数据流**（照抄 Radio 约定）。

SH1107 初始化序列（照搬 Radio `sh1107.c` 实证表；仅 MUX 一行 0x7F→0x3F 适配 64 行）：

| 命令 | 参数 | 作用 | 注 |
|---|---|---|---|
| 0xAE | — | 睡眠 | 配置先在黑屏下做 |
| 0xD5 | 0x80 | 内部振荡/分频 | 复位默认 |
| 0xA8 | 0x3F | MUX=64 COM 行 | **本工程适配**（Radio 128 行=0x7F） |
| 0xD3 | 0x00 | 显示偏移 0 | |
| 0x40 | — | RAM 起始行 0 | |
| 0xAD | 0x8B | 内部 DC-DC 开 | SH1107 专属（SSD1306 是 0x8D 0x14）；后等 100 ms |
| 0xDC | 0x00 | 显示起始列 0 | SH1107 专属命令 |
| 0xA1 | — | 列重映射 | 与 0xC8 成对（Radio 取向对；倒像换 0xA0/0xC0） |
| 0xC8 | — | COM 反向扫描 | 同上 |
| 0xDA | 0x12 | COM 引脚交替配置 | |
| 0x81 | 0xCF | 对比度默认 | Radio 默认值 |
| 0x20 | 0x00 | 页寻址模式 | 页光标 + 列光标各走各的 |
| 0xD9 | 0x22 | 预充电 1/2 DCLK | |
| 0xDB | 0x35 | VCOMH≈0.77·Vcc | |
| 0xA4 | — | 显示 RAM 内容 | 0xA5 是全亮测试 |
| 0xA6 | — | 正常（非反显） | |
| 0xAF | — | 显示开 | 清屏帧推完后最后发（Radio 顺序） |

刷屏：页寻址 `0xB0|页` → `0x00|列低4位` → `0x10|列高4位` → 0x40+128 字节
（一页一个事务，整帧 8 页 ≈24 ms）。列偏移旋钮 `OLED_COL_OFFSET`（个别 128 宽屏从列 2 起）。

## 两种 I2C 器件形态（EP11 vs EP12 对照，本模块教学核心）

- **存储器件**（24C02，EP11）：事务里携带**器件内地址**——`S 0xA0 字地址 数据 P`，
  随机读 = "先设指针再取数"；读写的是存储单元。
- **显示控制器**（SH1107，本模块）：事务携带的是**流**——控制字节只说"后面是命令/
  数据"，像素落在哪由**先发的命令**（页/列光标）决定，控制器自己推进光标；
  只画不读。同一根电气总线，地址段之后的"语法"各是各的。

## 自测试（menu> oled）

| 步 | 内容 | 判据 |
|---|---|---|
| 1 | 引脚配置回读 + 探测 0x3C/0x3D + 初始化序列 + 0xAF | 线被拉死（含一次 9 脉冲解缠重试）才 FAIL；NACK 不 FAIL（盲写语义） |
| 2 | 清屏 → line0 "F407-PERIPH-LAB"、line1 "I2C OLED OK" → 帧缓冲软件校验 → 全帧刷出 | 页 0/2 有墨、其余页全零、脏位清零 |
| 3 | 超时窗纪律：全部 SysTick 等待（DC-DC 100 ms 窗）未触窗 | 触窗 = FAIL（SysTick 死？） |

PASS 判词：`PASS oled (ack@0x3C, text on lines 0-1)`（有屏，肉眼对照）；无屏也
PASS：`PASS oled (no display: cmds blind-sent)`（命令照发不报错，接线正确后重跑
肉眼验证）。示波器在裸总线上仍能看到完整初始化波形（START、0x78、0x00、逐字节）。

## 对应视频集数

EP12（SPI/外设波次——4 针软 I2C OLED 变体在 W25Q 接线时发现）；与 EP11
（I2C EEPROM，存储器件形态）对照。
