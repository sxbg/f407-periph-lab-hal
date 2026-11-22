# bsp_crc_rng — CRC32 硬件单元 + RNG 真随机（EP20）

寄存器级 CRC32（AHB1）与 RNG（AHB2）：MPEG-2 语义字算 CRC + DRDY 门控随机数。
菜单命令 `crc_rng`。两者**均不在** .ioc（知识 #13），模块全权所有；EP20 教学对比
zlib crc32（反射+异或终值）与 F4 硬件（MSB-first 无终值、32 位字粒度）三差异。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| CRC | 基础 CRC（F4 代）：多项式/种子固定，无 POLY/INIT/REV_IN/REV_OUT | RM ch.4；F0/F7 代才有可配置项 |
| CRC Clock | AHB1（HCLK=168 MHz），CRCEN bit12 | 字-only 访问，写 DR 总线停等 32 次移位 |
| RNG | CR:RNGEN=1，IE=0 | 模拟熵源+LFSR+健康监测；FIPS 140-2 流程（首字丢弃） |
| RNG Clock | AHB2，RNGEN bit6；模拟部分走 RNG_CLK 域（PLLQ） | 本板 PLLQ=84 MHz（RM 期望 <48 MHz）——CECS 置位即上报点，非软件绕过 |
| NVIC | CRC/RNG 中断均 off | 轮询教学基线 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| — | — | 无引脚 | 纯计算/熵源外设 |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->AHB1ENR | CRCEN bit12 | CRC 总线时钟 |
| RCC->AHB2ENR | RNGEN bit6 | RNG 总线时钟（模拟部分另走 PLLQ 域） |
| CRC->CR | RESET bit0（自清） | 状态复位到 0xFFFFFFFF；本单元唯一控制位 |
| CRC->DR | 字写=喂字 / 读=状态 | 每次写停等 32 次移位完成；MSB-first 大端字 |
| CRC->IDR | 8 位自由存储 | 不受 RESET 影响 |
| RNG->CR | RNGEN bit2 / IE bit3=0 | 熵源电源；轮询课 |
| RNG->SR | DRDY / CECS / SECS | DR 读清 DRDY；时钟/种子错误状态（本板 PLLQ=84M 告警点） |
| RNG->DR | 32 位随机字 | 读清 DRDY（下一次等待重新计数） |

## 对应视频集数

EP20（CRC 校验与随机数）。
