# bsp_flash_iap — 片内 Flash IAP 参数页（EP21）

FLASH 钥匙解锁 + 擦除最后扇区 + 编程参数页 0x0807FC00（~2 s）。菜单命令
`flash_iap`（带 last_report）。FLASH 接口无 CubeMX 页面（.ioc 无此 IP）；F407VE
512 KB 密度：4×16K+64K+3×128K，最后扇区 = 7（"Sector 11" 是 1 MB 密度件的说法）。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| Density | STM32F407VETx = 512 KB | RM Table 5 扇区表：0-3=16K，4=64K，5-7=128K |
| Target sector | 7（0x0806 0000-0x0807 FFFF） | 参数页用扇区尾 1 KB：0x0807FC00..0x0807FFFF |
| PSIZE | x32（word 编程） | CR PSIZE[8:7] 与擦写粒度匹配 |
| Readout protect | RDP level 0（关） | 选项字节教学指针 |
| Key sequence | KEY1=0x45670123 → KEY2=0xCDEF89AB | CR LOCK 解锁 |
| 中断纪律 | 进 IAP 前 save/restore（非盲开） | 擦写期间总线停等，ISR 不可取指 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| — | — | 无引脚 | 片内 Flash 控制器 |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| FLASH->KEYR | 两把钥匙 | 解锁 CR.LOCK（写错=总线错误） |
| FLASH->CR | LOCK / PSIZE / PG / PER / SNB / STRT | 擦=PER+SNB+STRT；写=PG+半字/字写入 |
| FLASH->SR | BSY / EOP / PGSERR / WRPERR | 忙等 + 完成旗 + 错误双星（先查后清） |
| FLASH->ACR | LATENCY | 等待周期（168 MHz=5WS，只读判据） |
| FLASH->OPTCR | RDP / WRP | 选项字节（只读教学指针） |

## 对应视频集数

EP21（片内 Flash 与参数存储）。
