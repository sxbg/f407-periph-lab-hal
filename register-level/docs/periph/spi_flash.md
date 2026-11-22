# bsp_spi_flash — SPI1 主机 + 板载 W25Q128 16MB Flash（EP12）

寄存器级 SPI1 全双工主机：PA5=SCK / PA6=MISO / PA7=MOSI（AF5）+ PC13 软片选，
模式 0、8 位、5.25 MHz，对板载 W25Q128 做 JEDEC 识别/扇区擦除/页编程/读取。
菜单命令 `spi_flash`。SPI1 **不在** .ioc；PA5/6/7 为 GPIO_Analog 占位
（FLASH_SCK/MISO/MOSI），AF5 由本模块写入；PC13 在 .ioc 是 GPIO_Output 高
（FLASH_CS，HAL 开机即拉高 deselect），init 后归本模块所有。

**分区红线**：0x00000000–0x00A00000-1 = 自由区；0x00A00000–0x00FFFFFF =
字库区（出厂字库），擦/写一律拒绝（先查地址，后上总线）。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| SPI1 Mode | Full-Duplex Master | W25Q 单线命令流 |
| Hardware NSS | Disable（SSM=1, SSI=1） | 真 NSS 脚 PA4 = KEY3 被占；CS 用 PC13 GPIO |
| Frame Format / Data Size | Motorola / 8 Bits | W25Q 命令/地址/ID 均为 MSB-first 字节流 |
| First Bit | MSB First | 同上 |
| CPOL / CPHA | 0 / 0（模式 0） | W25Q 支持模式 0/3，教学取 0 |
| Prescaler (BR) | /16 → 5.25 MHz | 保守值；84M/8=10.5 MHz 也可，/16 让 0x03 读远离 50 MHz 上限 |
| PC13 | GPIO_Output，初始高 | CS 空闲 deselect |
| NVIC | SPI1_IRQn off | 轮询教学基线 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PA5 | AF5 | 推挽输出（SCK） | W25Q128 CLK |
| PA6 | AF5 | 输入，**MODER 必须 =10（AF）** | W25Q128 DO |
| PA7 | AF5 | 推挽输出（MOSI） | W25Q128 DI |
| PC13 | GPIO 输出 | 软片选 CS（**MODER 必须 =01**） | W25Q128 CS#（WP#/HOLD# 板上拉 3.3V） |

> 实测教训（RM0090 §8.3.2）：**外设输入脚（MISO）也必须进 AF 模式**——配成
> 普通输入（MODER=00）时 SPI1 恒读 0x00，与引脚真实电平无关；CS 写
> `GPIO_MODER_MODER13_1`（字段 bit1=0b10=AF）而非 `_0`（0b01=输出）时
> CS 不跟随 ODR。两者叠加 = 命令完美但芯片"完全沉默"。init 读回已加引脚
> 模式断言防回归。

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB2ENR | SPI1EN bit12 | SPI1 挂 APB2，分频基准 = PCLK2 = 84 MHz |
| RCC->AHB1ENR | GPIOAEN/GPIOCEN | 引脚时钟 |
| SPI1->CR1 | MSTR / SPE / BR=100 / CPOL=0 / CPHA=0 / SSM+SSI / DFF=0 / LSBFIRST=0 | 主机+使能最后写；BR 指数分频 2^4=/16；SSI=1 防 SPE 后掉从机 |
| SPI1->CR2 | 0 | TXEIE/RXNEIE 关（轮询） |
| SPI1->SR | TXE / RXNE / BSY | TXE 可装 DR；RXNE 有收字节（读 DR 清）；BSY=0 才许拉高 CS |
| SPI1->DR | 8 位字节访问 | 全双工：写即收，读出 MISO 同 8 拍内容 |
| GPIOC->BSRR | bit13 / bit13+16 | CS 拉低/拉高（原子） |

**CS 时序纪律（EP12 教学点）**：CS 低 → 命令 → 数据 → 等 BSY=0 → CS 高。
RXNE 只证明最后采样沿发生，BSY=0 才证明整帧离开移位寄存器；提前拉高 = 命令
被截断，过晚不拉 = 下一字节被解析为新命令。

W25Q 命令层：0x9F JEDEC（期望 EF 40 18）、0x06 WREN（WEL 自清）、0x05 RDSR1
（BUSY bit0 轮询，擦典型 45 ms/最大 400 ms，写典型 0.6 ms）、0x20 4KB 扇区擦、
0x02 页编程（256B 页对齐 + 不越页，跨页会回绕覆盖页首）、0x03 读（≤50 MHz）。

## 对应视频集数

EP12（SPI 总线与 SPI Flash）。
