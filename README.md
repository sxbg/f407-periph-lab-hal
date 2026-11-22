# f407-periph-lab-hal — STM32F407 全外设教学工程（HAL 版）

CubeMX 6.11 图形化配置生成 `MX_*_Init`，应用代码直接调用 HAL API。
STM32F407VETx @168 MHz（HSE 25 MHz），armcc 5.06 C99，MDK-ARM + CMSIS-DAP。

## 两个版本，按需选用

| 目录 | 版本 | 适用场景 |
|---|---|---|
| `firmware/`（本目录根） | **HAL 版** | 生产主线：CubeMX 配置可重生成（`asm cubegen regen`），`MX_*_Init` + `HAL_*` 直调，bsp 层 console-free、A3 红线（禁止寄存器级写入） |
| `register-level/` | **寄存器版** | 教学深挖 / 需要寄存器级控制的客户：同一块板、同一批外设，BSP 直接写寄存器（含 BRR 推导、EXTICR/IMR 手工链、SDIO 命令流等寄存器讲解） |

两版菜单命令名、模块划分、教学路线完全对齐，便于对照学习与客户切换。
寄存器版细节见 `register-level/docs/periph/*.md`（同名文档，寄存器深讲）；
HAL 版见 `docs/periph/*.md`（GUI 配置步骤 + HAL API 清单 + 生成器盲区备注）。

## 快速开始

```bash
# 构建（三判据：0 err 0 warn + hex 新鲜）
asm build --project .

# 烧录（CMSIS-DAP）
cd firmware/MDK-ARM && UV4.exe -f f407-periph-lab-hal.uvprojx -j0 -o flash_log.txt

# 上板验证（22 项，串口 COM18 115200）
python -I tools/menu_autotest.py
```

- 菜单交互：任意串口终端连 COM18，敲 `help` 看全部命令
- 闭环验证需两根杜邦线：PA4→PC0、PA8→PB3
- 报告：`docs/test-report.md`（最终回归 PASS 19 / KNOWN 1 / FAIL 0）
- 单文件版开发+调试报告：`demo.html`（浏览器直接打开）

## 已知边界（详见 docs/test-report.md §4 与 knowledge #17/#18）

- 看门狗调试期全关（用户裁定；恢复方法见 `docs/periph/wdg.md`）
- RTC 生成器输出恒 LSI（app_main 有 LSE 修正段，非致命）
- CubeMX 6.11 文本 loader 的生成器盲区汇总：`knowledge/pitfalls-cubemx-keil.md` #16-#18
