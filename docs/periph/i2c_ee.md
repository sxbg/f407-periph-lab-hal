# bsp_i2c_eeprom — I2C1 主机 + 板载 24C02 EEPROM（EP11）

硬件 I2C1 主机（HAL 版教学主线）：PB6=SCL / PB7=SDA（AF4 开漏），100 kHz 标准模式，对板载 24C02（7 位地址 0x50）做单字节写/随机读。CubeMX 全权拥有引脚与 I2C1 寄存器文件；bsp 层只做生成配置回读 + HAL 调用（A3 红线，寄存器触碰仅只读探针）。24C02 命令层不变，变的是线束引擎：老版软开漏位拍总线（`i2c_soft_bus`）退役，`HAL_I2C_Mem_Write/Read/IsDeviceReady` 的"一调用=一事务"是行业标准形状。

## CubeMX GUI 配置步骤

1. Connectivity → I2C1 → I2C Mode：勾选 **I2C**（Standard Mode，仅 Master；400 kHz 是对照实验项）。
2. Parameter Settings：**Clock Speed = 100000**、**Addressing Mode = 7-bit**、Own Address1 = 0（纯主机，无人寻址 MCU）、DutyCycle = 2、No Stretch / General Call = Disable。
3. 引脚自动落 **PB6 = I2C1_SCL / PB7 = I2C1_SDA**：GPIO Mode **AF Open-Drain**、Pull **No pull**（上拉靠板上外部电阻——内部 ~40 kΩ 撑不住 100 kHz）、Speed Very High、AF4。
4. NVIC / DMA 不勾（轮询教学基线）。
5. 生成 `MX_I2C1_Init()` + `HAL_I2C_MspInit()`（main.c 开机调用）；CCR/TRISE 数学（CCR=210、TRISE=43）进 `HAL_I2C_Init` 内部，不再是手写项。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| PB6 | EE_SCL / I2C1_SCL | AF4 开漏，无内部上拉（板上外部上拉生效，空闲高） |
| PB7 | EE_SDA / I2C1_SDA | AF4 开漏，无内部上拉 |

## HAL API 调用清单

- `int bsp_i2c_eeprom_init(void)` — 回读生成配置：`hi2c1.Init.ClockSpeed == 100000`、CR1 PE 已使能（`HAL_I2C_Init` 最后一步开外设，PE 清 = 生成 init 未完成）、两线空闲均高（`HAL_GPIO_ReadPin` 只读探针；空闲有低 = 总线卡死，HAL 版只报告不做恢复脉冲）。
- `int bsp_i2c_eeprom_write_byte(uint8_t addr, uint8_t data)` — `HAL_I2C_Mem_Write(&hi2c1, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 20)`：一调用 = 一事务（START→0xA0→字地址→数据→STOP），HAL 负责 START/STOP/ACK/超时（`HAL_GetTick` 兜底，拒绝永久等待）；随后 tWR ACK 轮询（`HAL_I2C_IsDeviceReady` 每 1 ms 一探、12 次窗口 > 2× 最坏 5 ms 写周期）→ 写读可背靠背。返回 −1=NACK（双线仍高，器件真拒绝）/ −2=总线卡死（某线被拉低）。
- `int bsp_i2c_eeprom_read_byte(uint8_t addr, uint8_t *data)` — `HAL_I2C_Mem_Read`：指针写 + 内部重复起始（Sr）读，一调用完成随机读；老版为绕停摆用两段干净事务，那形状已成 HAL 内部序列。
- `int bsp_i2c_eeprom_selftest(void)` / `const char *bsp_i2c_eeprom_last_report(void)`。

```c
bsp_i2c_eeprom_write_byte(0x10u, 0x55u);   /* 内含 tWR 轮询 */
uint8_t v;
bsp_i2c_eeprom_read_byte(0x10u, &v);       /* v == 0x55 */
```

协议要点（24C02）：

- 7 位地址 0b1010_000 = 0x50（A2/A1/A0 接地）；HAL 全部调用要 8 位形 `dev7 << 1` = `0xA0/0xA1`（每事务 HAL 超时 20 ms ≈ 100× 余量，卡死总线快速失败）。
- 错误分类判据：失败后 `HAL_GPIO_ReadPin` 读双线——双高 = 总线健康、是器件 NACK（−1）；有线拉低 = 总线卡死（−2）。分类逻辑与软引擎时代一致，只换了裁判。
- 256 B 存储阵列，字地址单字节；**页 = 8 B 且页内低 3 位回绕**——跨页写会回卷覆盖页首而非顺延（本模块只做单字节写永不跨页；分块页写是 EP11 练习）。
- 每次写后 ≤5 ms 内部写周期（tWR），期间器件对一切 NACK——ACK 轮询是 EEPROM 标准等法；跳过轮询 = "写后立刻读回旧值"的经典 EEPROM 教训。
- 自测先扫 0x50..0x57（整个 1010+A2A1A0 块）精化地址。
- 寄存器深讲（CCR=210/TRISE=43/EV5–EV8_2 清除对/SR1+SR2 读对）压缩一句指引：见老工程 i2c_ee.md 表 3——时序与事件机已由 HAL 包办。
- 引擎迁移一览：软总线引脚电气 = MspInit 生成；半位时序公式 = `HAL_I2C_Init` 数学；ACK 槽探针 = `HAL_I2C_IsDeviceReady`；9 时钟总线恢复 = HAL 错误路径复位外设（重跑 init 即恢复，留作扩展练习）。

## 生成器盲区备注

本模块无映射盲区。已知边界（如实记录）：硬件 I2C 随机读曾在老工程真板非确定停摆（2026-11-21，旧工程 §3.2，100k/50k 复现、五轮修复未愈后转软总线）——**HAL 版以硬件引擎为教学主线**；若 W6 上板复现同一停摆，按知识库"已知板级边界"记录在案，**不得**在模块内复活软总线。

## 自测验证

菜单命令 `i2c_ee`：

- 步骤 1：配置回读（ClockSpeed/PE/双线空闲，无总线流量）。
- 步骤 2：扫描 + 写 0x55 @0x10 → 随机读比对（NACK 是硬件发现，如实报告——24C02 焊死在板上必须应答）。
- 步骤 3：写 0xAA @0x11（证明地址指针不卡死——字地址相坏会一直答 0x10 的旧值）。
- 步骤 4：总线空闲双高（最后一次 STOP 后开漏释放、上拉获胜；有低 = 目标卡死半字节）。

本轮实测：**PASS i2c_ee (0x55/0xAA roundtrip ok)**——硬件 I2C1 + 板载 24C02 实板通过。

## 对应视频集数

EP11（I2C 总线与 EEPROM）。
