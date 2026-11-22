# bsp_flash_iap — 片内 Flash IAP 参数页（EP21）

FLASH 接口钥匙解锁 + 扇区擦除 + 字编程，全部流量硬限制在末扇区参数页 0x0807FC00（1 KB）。HAL 版无 .ioc init（CubeMX 没有 FLASH IP 页面，知识 #13）——`HAL_FLASH_*` 是纯运行时 API；擦/写窗口用 PRIMASK 短屏蔽（保存/恢复，非盲开）。安全契约：CPU 正在从这块 Flash 取指执行，擦写代码扇区 = 取指停等/损坏 → 无恢复；所以擦除函数无参（只能扇区 7）、编程函数页外地址一律拒绝——**拒绝优于文档告警**。

## CubeMX GUI 配置步骤

无任何 .ioc/GUI 步骤（纯运行时外设，同 PWR）。需要知道的硬件真值：

1. 密度：STM32F407VETx = **512 KB**（RM0090 Table 5：扇区 0-3=16K、4=64K、5-7=128K）——**末扇区是 7**（0x0806 0000–0x0807 FFFF）；HAL 例程里的 "Sector 11" 是 1 MB 密度件的说法，在本件上是非法 SNB。
2. 并行宽度：VoltageRange 3（2.7–3.6 V）= x32 字编程，CR PSIZE 与之匹配（擦除与编程同宽）。
3. Readout protect RDP level 0（关）——选项字节只是教学指针，本模块不碰（OPTKEYR/选项字节教学保留在寄存器版材料）。
4. 中断纪律：进 IAP 前 `__get_PRIMASK()` 保存 + `__disable_irq()`，操作后原值恢复——擦写期间总线停等，ISR 不可取指；128 KB 扇区擦典型 1.3 s / 最大 2.6 s，字编程 ~35 µs/字，心跳暂停是文档化成本。
5. HAL 时序兜底：HAL 内部 BSY 轮询自带 FLASH_TIMEOUT_VALUE（50 s），覆盖上述全部物理窗口。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| — | — | 无引脚（片内 Flash 控制器，内存映射直读不需要解锁） |

## HAL API 调用清单

HAL 调用底层语义（寄存器深讲指引：老工程 flash_iap.md 表 3）：

- `HAL_FLASH_Unlock()/HAL_FLASH_Lock()` — KEYR 双钥匙序列（0x45670123 → 0xCDEF89AB，顺序错=总线错误）清 CR.LOCK；HAL 钥匙后复查 LOCK，被拒即返回错误。每次 Unlock 必配对 Lock。
- `HAL_FLASHEx_Erase(&erase, &sector_error)` — 组 CR = SER + SNB=扇区 + PSIZE（=VoltageRange 实参）+ STRT，随后有界 BSY 轮询；错误位（WRPERR/PGAERR/PGPERR/PGSERR）进 ErrorCode 输出并使调用返回 HAL_ERROR——寄存器版的 rc_w1 审计由 HAL 代劳。
- `HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data)` — CR = PG + PSIZE=word，32 位存储即编程命令，逐字 BSY 轮询；NOR 位只能 1→0，0→1 只能靠擦。

模块公开 API（bsp_flash_iap.h）：

- `int bsp_flash_iap_init(void)` — 只读体检：ACR latency 非零（168 MHz = 5 WS，0 WS 回读 = 时钟树回退告警）+ CR LOCK 现状进报告，不动任何东西。
- `int bsp_flash_iap_erase_last_sector(void)` — 无参设计（代码扇区 0-6 物理不可达）：Unlock → `__HAL_FLASH_CLEAR_FLAG(...)` 清脏标志 → `HAL_FLASHEx_Erase`（`FLASH_TYPEERASE_SECTORS` / Sector 7 / NbSectors 1 / `FLASH_VOLTAGE_RANGE_3`）→ Lock。返回 0 / −1（HAL 错误或 SNB 错误）。
- `int bsp_flash_iap_program_words(uint32_t addr, const uint32_t *src, uint32_t n)` — 字对齐 + 参数页界检查（越界先拒绝 −2，不上总线），逐字 Program；页必须先擦。
- `int bsp_flash_iap_read_words(uint32_t addr, uint32_t *dst, uint32_t n)` — 纯内存读（教学点：读不需要解锁，CPU 取指就在读这块存储）；范围限 512 KB 全片。
- `int bsp_flash_iap_selftest(void)` / `const char *bsp_flash_iap_last_report(void)`。

```c
/* 参数页先擦后写（教学 8 字特征图案）: */
uint32_t words[2] = { 0x53454737u, 0x50415241u }; /* "SEG7" "PARA" */
bsp_flash_iap_erase_last_sector();                /* 只能是扇区 7 */
bsp_flash_iap_program_words(0x0807FC00u, words, 2u);
```

## 生成器盲区备注

- **`HAL_FLASHEx_Erase` 的 `*SectorError` 被 HAL 预置 0xFFFFFFFF**（成功时保持全一，只在出错扇区写 SNB）——调用方按 C 直觉 0 初始化 + `!=0` 判据会**永远误报**；必须预置 `0xFFFFFFFF` 后判 `!= 0xFFFFFFFF`（本轮实测踩中后修复，知识 #18）。
- **擦除前清脏标志**：`__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|OPERR|WRPERR|PGAERR|PGPERR|PGSERR)`——上次失败残留的错误位会让新操作立即 PGSERR（实测开机首擦前 SR=0xC0）。
- PRIMASK 窗口的诚实副作用：屏蔽期间 `HAL_GetTick()` 冻结，HAL 内部 50 s 超时退化为纯 BSY 轮询——BSY 下降才是真实出口（数据手册窗口才是物理上限）。

## 自测验证

菜单命令 `flash_iap`（带 last_report）：

- 步骤 1：unlock → LOCK 读 0 → lock → LOCK 读 1（整个钥匙契约双向证实）。
- 步骤 2：擦扇区 7 → 全 128 KB（32768 字）扫 FF → 参数页写 8 字特征图案（"SEG7 PARA 1234 5678…"）→ 读回比对 → **故意留在片上**（断电保持演示：断电重启后 Keil memory window 看 0x0807FC00 仍在，下轮重擦重写）。
- 步骤 3：再锁（模块绝不把可写状态留在菜单命令身后）。

本轮实测：**PASS flash_iap (erase ok, param 8w kept)**。

## 对应视频集数

EP21（片内 Flash 与参数存储）。
