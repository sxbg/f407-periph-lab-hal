# bsp_wdg — IWDG/WWDG 看门狗（EP15）

IWDG 2 s 配置回读 + WWDG 窗口 + RCC->CSR 复位源旗标。菜单命令 `wdg`（带
last_report）；喂狗死等演示走 demo 命令 `wdg_test`/`wdg_reset_demo`（另一注册表，
先打印警告）。IWDG/WWDG **不在** .ioc（知识 #13）；LSI 17–47 kHz 展开带来的
超时散布（1.4–3.8 s）是教学点。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| IWDG | LSI 32 kHz 标称（17–47 kHz），PR/RLR 实算 ~2 s | KR 钥匙序列：0x5555 解锁/0xAAAA 喂/0xCCCC 启动 |
| WWDG | PCLK1=42 MHz，WDGTB + W[6:0] 窗口 | 窗口期外喂=复位；EWI 未用 |
| Option bytes | WDG_SW = 软件看门狗 | 硬件自启不用 |
| NVIC | WWDG_IRQn off | 轮询教学基线 |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| — | — | 无引脚 | LSI 为内部 RC 振荡器 |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | WWDGEN bit11 | WWDG 总线时钟（计数时钟也是 PCLK1 分频） |
| IWDG->KR | 0x5555/0xAAAA/0xCCCC | 解锁/喂狗/启动三把钥匙 |
| IWDG->PR / RLR | 分频/重装值 | 超时=TI_LSI×4^PR×(RLR+1)，LSI 散布→超时散布 |
| IWDG->SR | RVU / PVU | 影子更新进行中（写后需等） |
| WWDG->CFR | WDGTB / W[6:0] / EWI | 窗口下界（早喂=复位） |
| WWDG->CR | T[6:0] / WDGA | WDGA=1 后不可关；T 递减到 0x3F 前必须喂 |
| RCC->CSR | RMVF / IWDGRSTF / WWDGRSTF 等 | 复位源旗标（RMVF 置 1 清旗）；EP05 复位源教学 |

## 对应视频集数

EP15（看门狗与复位管理）。
