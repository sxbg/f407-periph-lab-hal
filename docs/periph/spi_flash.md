# bsp_spi_flash — SPI1 主机 + 板载 W25Q128 16MB Flash（EP12）

SPI1 全双工主机（HAL 版）：模式 0、8 位 MSB-first、软 NSS、84M/16 = 5.25 MHz，PA5/6/7（AF5）+ PC13 软片选，对板载 W25Q128 做 JEDEC 识别 / 4KB 扇区擦 / 页编程 / 读取。**分区红线**：0x00000000–0x00A00000-1 = 自由区；0x00A00000–0x00FFFFFF = 字库区（出厂字库 HZK），擦/写一律拒绝（先查地址后上总线，WREN 不外泄）。SPI 没有"只发不收"——每次发送同时移入一个接收字节，读器件靠 dummy 0x00/0xFF 时钟。

## CubeMX GUI 配置步骤

1. Connectivity → SPI1 → Mode：**Full-Duplex Master**（Direction 2 Lines）。
2. Parameter Settings：**Prescaler = 16**（→ 5.25 MHz，保守值让 0x03 读远离 50 MHz 上限）、CPOL **Low** / CPHA **1 Edge**（模式 0；W25Q 支持 0/3，教学取 0）、**8 Bits**、**MSB First**、**NSS = Software**（SSM=SSI=1；真 NSS 脚 PA4 被 KEY3 占用）。
3. 引脚 **PA5 = SPI1_SCK / PA6 = SPI1_MISO / PA7 = SPI1_MOSI**：GPIO AF Push-Pull、Pull 无、Speed **Very High**（OSPEEDR 档位由 CubeMX 生成——老工程知识 #14 漏配致 5.25 MHz SCK 静默失效的盲区，现为生成器保证）。
4. PC13 配 **GPIO_Output**、初始电平 **High**、标签 `FLASH_CS`（MX_GPIO_Init 开机即拉高 deselect；片选永远不是 SPI 外设脚，RM0090 28.3.3）。
5. NVIC 不勾（轮询基线）。生成 `MX_SPI1_Init()`（main.c 开机调用）。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| PA5 | FLASH_SCK / SPI1_SCK | AF5 推挽，Very High |
| PA6 | FLASH_MISO / SPI1_MISO | AF5（外设输入脚也必须 AF，MODER=10——知识 #15 教训现为生成器保证） |
| PA7 | FLASH_MOSI / SPI1_MOSI | AF5 推挽，Very High |
| PC13 | FLASH_CS | GPIO 输出初始高，软片选（低有效：RESET = 选中） |

## HAL API 调用清单

- `int bsp_spi_flash_init(void)` — 回读 `hspi1.Init` 八字段（master/2-lines/8-bit/模式 0/软 NSS//16/MSB）+ CR1（MSTR|SSM|SSI 置位，CPOL/CPHA/LSB/DFF 复位零；SPE **不**要求——HAL 每次传输内部开关引擎）+ CS 拉高 + 发一帧 0xAB 深度掉电释放（每 init 一帧，把"芯片睡着"从嫌疑名单永久剔除）。
- `uint8_t bsp_spi_flash_xfer8(uint8_t tx)` — `HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10)`：HAL 等完 TXE/RXNE **含尾部 BSY** 才返回——老版 CS-vs-BSY 纪律（RXNE 只证明最后采样沿发生，BSY=0 才证明整帧离开移位寄存器，RM 28.3.7）收敛为"CS 低→HAL 帧→CS 高"一个形状。HAL 超时返回 0x00，失败由命令层结果裁决。
- `int bsp_spi_flash_jedec_id(uint8_t id[3])` — 0x9F 帧 + 3 个哑交换，期望 EF 40 18（无 WREN 无 BUSY，永远合法的第一探针）。
- `int bsp_spi_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)` — 0x03 + 24 位大端地址 + `HAL_SPI_Receive`，数据流到 CS 升为止；len ≤ 0xFFFF = HAL Size 上限。
- `int bsp_spi_flash_page_program(uint32_t addr, const uint8_t *buf, uint32_t len)` — 页对齐 + len ≤ 256 + 整段在同一页内（跨页回绕覆盖页首，API 拒绝而非信任调用方）+ 红线检查 → WREN(0x06，WEL 自清) → 0x02 帧 → RDSR1 BUSY 轮询（写典型 0.6 ms / 最大 3 ms，窗 50 ms）。−3 = 字库区违规（总线零流量）。
- `int bsp_spi_flash_sector_erase4k(uint32_t addr)` — 4 KiB 对齐（0x20 擦的是包含地址的整扇区）+ 红线 → WREN → 0x20 → BUSY 轮询（擦典型 45 ms / 最大 400 ms，窗 500 ms）。
- `int bsp_spi_flash_selftest(void)` / `const char *bsp_spi_flash_last_report(void)`。

```c
uint8_t id[3];
bsp_spi_flash_jedec_id(id);                     /* EF 40 18 */
bsp_spi_flash_sector_erase4k(0x00010000u);
bsp_spi_flash_page_program(0x00010000u, pat, 16u);
```

W25Q 命令层一句话总表（寄存器深讲指引见老工程 spi_flash.md 表 3）：0x9F 识别、0x06 WREN、0x05 RDSR1 BUSY（BUSY=1 期间除 RDSR1 全聋——轮询即等待）、0x20 4KB 擦（擦后 NOR 读 FF：编程只能清位，所以擦必先于写）、0x02 页编程、0x03 读（≤50 MHz，本板 5.25 MHz 免等待）、0xAB 深度掉电释放（tRES ~3 µs）。

HAL 帧与超时常量：单字节交换超时 10 ms（5.25 MHz 下一字节 ~1.5 µs，100× 余量）；擦 BUSY 窗 500 ms、写 BUSY 窗 50 ms、读 API 上限 0xFFFF 字节（`HAL_SPI_Transmit/Receive` 的 16 位 Size）。

## 生成器盲区备注

- 老寄存器版的 `scope_demo` / `pin_probe_demo`（无限循环台架、手工重 mux 引脚）与**位拍软探针 / 模式 3 慢速后备引擎**（`sf_set_spi_mode` 等，寄存器写）不迁移（A3：CubeMX 拥有引脚后禁止手写 MODER/AFR/CR1）。
- **DPD 唤醒 0xAB 保留**（knowledge #13/#14：0xB9 深度掉电后的器件只应答 0xAB，0x9F 静默与坏芯片无法区分）；自测失败路径带 `9F:`（首读）/`ab:`（释放后重试）/`p6:`（MISO 原始电平，CS 释放时 DO 为 Hi-Z）三判据留证据。
- OSPEEDR 漏配（知识 #14）与 MISO 未进 AF / CS 字段值写错（知识 #15）两桩老工程硬板教训，HAL 版由 CubeMX 生成保证，init 只做回读断言防回归。

## 自测验证

菜单命令 `spi_flash`：

- 步骤 1：配置回读（Init 字段 + CR1 + CS 空闲高）。
- 步骤 2：JEDEC 必须 EF 40 18（静默/垃圾 → 0xAB 释放 + 重试一次，焊死的 W25Q 必须应答）。
- 步骤 3：自由区 0x00010000 页操练（红线下 1 MiB 余量）：4KB 擦 → 256 B 全 FF → 16 B 图案写入 → 比对（分相断言，报告点名失败相；每轮重擦，幂等）。

本轮实测：**PASS spi_flash 9F:EF4018 page ok**。

## 对应视频集数

EP12（SPI 总线与 SPI Flash）。
