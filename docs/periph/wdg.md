# bsp_wdg — IWDG/WWDG 看门狗（EP15）

IWDG 配置回读 + WWDG 窗口法 + RCC->CSR 复位源旗标解码。EP15 教学点：LSI
17–47 kHz 展开带来的 IWDG 超时散布（本 build 8.192 s 标称 → 实际 5.6–15.4 s）
与 WWDG"早喂即复位"窗口法。寄存器级深讲（KR 三钥匙、PVU/RVU 影子同步）看
老工程 `docs/periph/wdg.md` 表 3。**本轮调试期两只狗全关（用户裁定）**——
语义见盲区备注。

## CubeMX GUI 配置步骤

1. System Core → **IWDG** → Activated（VP_IWDG_VS_IWDG）：Prescaler=**/64**、
   Reload=**4095** → 4096 tick ÷ 32 kHz 标称 = 8.192 s（生成真值 iwdg.c：
   `IWDG_PRESCALER_64` + `Reload=4095`；`HAL_IWDG_Init` 内部做 KR 钥匙
   序列 + KR=0xCCCC 启动——armed 后不可关，只能整段跳过或复位）。
2. System Core → **WWDG** → Activated（VP_WWDG_VS_WWDG）：Prescaler=**/8**、
   Counter=**127**、Window=**127**（.ioc 真值 `WWDG.WindowValue=0x7F`，
   config-spec §5 意图）、EWI 关。生成真值 wwdg.c 是 /8 + Counter=127 +
   **Window=64**——生成器恒输出 64（知识 #17 单 tick 剃刀窗，见盲区备注；
   42 MHz/4096/8 → tick=780.2 µs，最大 64 tick≈49.9 ms）。
3. Option bytes：WDG_SW=软件启动（CubeMX 默认；硬件自启不用）。
4. NVIC：WWDG_IRQn 不开（EWI 未用，轮询基线）。
5. 无引脚——LSI 为内部 RC 振荡器（IWDG 在 VDD 域独立计数，停机模式仍走）。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| — | — | 无引脚 |

## HAL API 调用清单

- `int bsp_wdg_init(void)` — 核验 boot-armed 计划（handle 字段 + PR/RLR/
  CFR 只读回读，A3 豁免；WDGA=1 证明狗**在跑**——与老工程"停狗实证"
  反演）。IWDG PR/RLR 之所以可信恰因狗已 armed（老工程 2026-11-21 实证：
  未 armed 的 IWDG 影子同步永不完成、回读陈值——HAL 版 boot 启动恰好是
  让验证成为可能的设计）。0=PASS，-1=不符。

典型用法（shell 主循环喂狗纪律）：

```c
/* 菜单循环每圈: */
bsp_wdg_refresh();                  /* IWDG 无条件喂 + WWDG 窗口守卫喂 */
bsp_wdg_iwdg_feed();                /* 只喂 IWDG（HAL_IWDG_Refresh=0xAAAA） */
uint32_t f = bsp_wdg_reset_flags(); /* CSR 旗标快照, 可自行再解码 */
```

- `void bsp_wdg_refresh(void)` — 菜单循环喂狗助手：IWDG 无条件（只需 stay
  < 5.6 s 快沿）；WWDG **窗口守卫**——先只读探 CR T[6:0]，落入合法条带
  （T≤W）才发 `HAL_WWDG_Refresh`（T>W 时喂=瞬间复位，RM 22.4.2；生成
  W=64 时条带单 tick，守卫几乎不触发）。
- `void bsp_wdg_reset_demo(void)` — 安全演示（**绝不在自测里调**；shell 另
  一注册表、先打印警告）：最后喂一次 IWDG 后死循环断粮，NEVER RETURNS；
  复位落在 5.6–15.4 s 后（EP15 边际课），下电 boot_diag 发
  EV_WATCHDOG_RESET，`wdg` 命令看旗标解码。（当前 WWDG 剃刀窗未修则 WWDG
  先在 ~49 ms 触发、旗标为 wwdg——IWDG 课被掩盖而非破坏。）
- `uint32_t bsp_wdg_reset_flags(void)` — CSR bits 25..31 七旗标快照原样
  返回（只读；RMVF 清旗写被 A3 禁止——app_main 在 boot 已
  `__HAL_RCC_CLEAR_RESET_FLAGS`）。
- `const char *bsp_wdg_last_report(void)` — "(iwdg ok, wwdg razor win)" /
  "(rst: iwdg)" 等一行报告。
- `int bsp_wdg_selftest(void)` — 返回 0=PASS，否则失败步号：
  1=IWDG 计划+PR/RLR 回读 2=CSR 快照+解码入报告（RMVF 清旗循环删除：
  A3 禁写 CSR）3=WWDG CFR 回读+WDGA 置位（razor 窗只报告不判 FAIL——
  那是 .ioc 的真值）。

## 生成器盲区备注

**最重盲区（知识 #17）**：`MX_IWDG_Init`/`MX_WWDG_Init` 在 USER CODE 0 区
短路（`return;` + `#pragma diag_suppress 111`）——调试期看门狗全关
（**用户裁定 2026-11-22**；IWDG armed 后无法解锁，只能整段跳过）。
②生成 WWDG Window 恒 64（.ioc 写 127 也被覆盖）→ 单 tick（780.2 µs）剃刀
刷新窗，任何 boot 序列都喂不进 → 不短路则每轮上电 ~49 ms 复位循环。
**恢复方法**：删两处 `return;` + 接上 bsp_wdg 运行时 init（模块已具备
`HAL_IWDG_Init`/`HAL_WWDG_Init` 能力）；菜单主循环的 `bsp_wdg_refresh()`
调用已就位（未 armed 时为无害写）。恢复后 .ioc 的 Window 仍需放宽
（老工程 W=0x70 给 48 tick≈37.4 ms 条带）或删 WWDG。

## 自测验证

菜单命令：`wdg`；本轮实测：**KNOWN FAIL step1 "iwdg plan mismatch"**——狗
关了而模块按 armed 狗验证，预期偏差（test-report §4.1，用户裁定）；两只狗
恢复后即测（运行时 init 语义）。

## 对应视频集数

EP15（看门狗与复位管理）。
