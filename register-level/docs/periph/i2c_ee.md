# bsp_i2c_eeprom — I2C1 主机 + 板载 24C02 EEPROM（EP11）

寄存器级 I2C1 主机：PB6=SCL / PB7=SDA（AF4，开漏+上拉），100 kHz 标准模式，
对板载 24C02（7 位地址 0x50）做单字节写/随机读。菜单命令 `i2c_ee`。
I2C1 **不在** .ioc（知识 #13：adapter 全权外设），PB6/PB7 在 .ioc 里只是
GPIO_Analog 占位（标签 EE_SCL/EE_SDA），AF4 复用由本模块写入。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| I2C1 Mode | I2C（Standard Mode，仅 Master） | 24C02 无 FM 需求，Sm 是教学基线 |
| Clock Speed | 100 kHz（CCR=210） | f_SCL = PCLK1/(2×CCR) = 42 MHz/420，整除零误差 |
| Address Size | 7-bit | 目标地址 0b1010_000 = 0x50（A2/A1/A0 接地） |
| Own Address (OAR1) | 复位值 0x0000 | 纯主机实验，无人寻址 MCU |
| Rise Time (TRISE) | 43 | 1000 ns ÷ 23.81 ns + 1（RM 27.6.9 Sm 公式） |
| Clock No Stretch / DMA / NVIC | off / off / 关中断 | 轮询教学基线 |
| PB6/PB7 | GPIO_AF4 开漏 + 内部上拉 | 板上另有外部上拉电阻（内部 ~40 kΩ 不足以单独撑 100 kHz） |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PB6 | AF4 | 开漏输出/输入（SCL） | 24C02 SCL（U12，A2/A1/A0=0） |
| PB7 | AF4 | 开漏输出/输入（SDA） | 24C02 SDA |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | I2C1EN bit21 | I2C1 内核时钟门控（PCLK1=42 MHz 直连，非定时器无 ×2） |
| I2C1->CR2 | FREQ[5:0]=42 | 告知 I2C 自身输入时钟（MHz），CCR/TRISE 的单位锚点 |
| I2C1->CCR | CCR[11:0]=210, F/S=0 | Sm 模式 T_high=T_low=CCR×T_PCLK1 → 100.0 kHz 整 |
| I2C1->TRISE | TRISE[5:0]=43 | SCL 上升沿反馈回路最大时限（拉伸检测） |
| I2C1->OAR1 | 0x0000 | 自身地址（主机模式无用，列出补全寄存器地图） |
| I2C1->CR1 | PE / START / STOP / ACK / SWRST | 使能最后写；START(BUSY=1)即重复起始；ACK 在 EV6 前清 |
| I2C1->SR1 | SB / ADDR / TxE / BTF / RxNE / AF | EV5(EV 清除对)、EV6(SR1+SR2 读对)、EV8_2(BTF 时写 STOP)、AF=NACK(rc_w0) |
| I2C1->SR2 | MSL / BUSY / TRA | 主机锁存、总线忙（Stop 清）、收发方向；读 SR2 才清 ADDR |
| I2C1->DR | 0xA0/0xA1 + 字节 | 地址字节与数据字节的出入口 |

24C02 协议要点：页 = 8 字节，页内地址低 3 位回绕（本模块只做单字节写，永不跨页）；
每次写后有 ≤5 ms 内部写周期（tWR），期间器件 NACK —— `write_byte` 以 ACK 轮询收尾
（START+地址直到 ACK，上限 ~1.2 s），写读可背靠背。

## 对应视频集数

EP11（I2C 总线与 EEPROM）。
