# bsp_exti — 外部中断 EXTI（EP10）

HAL 版 EXTI 双线：PA0=KEY1 上升沿（EXTI0）、PA1=KEY2 下降沿（EXTI1）；引脚电配置由
CubeMX 生成，bsp 层经 HAL_GPIO_Init 重编路由 + 统一 weak 回调分发 + 交互式计数自测。
菜单命令 `exti`（**交互式**：绿 LED 闪 6 秒窗，按 KEY2 ≥3 次 → `PASS exti (3 presses)`；
超时 → `PASS exti (idle, N presses)`，SKIP 语义不 FAIL）。

## CubeMX GUI 配置步骤

1. PA0 → GPIO_EXTI0：Mode=External Interrupt Mode with Rising edge trigger
   detection，Pull-down（KEY1 空闲 0、按下接 VDD → 按下即上升沿）。
2. PA1 → GPIO_EXTI1：Falling edge trigger detection，Pull-up（KEY2 空闲 1、按下接地
   → 按下即下降沿；松开的上升沿不计次）。
3. NVIC：EXTI line0/line1 interrupt，优先级 5。注意生成物形态：MX_GPIO_Init 只
   SetPriority、**不 EnableIRQ**——使能是 bsp_exti_init 的编路职责。
4. 生成的电配置（gpio.c）：KEY1=IT_RISING+PULLDOWN、KEY2=IT_FALLING+PULLUP，
   与 IOC 真值一致。

一条 `HAL_GPIO_Init(GPIO_MODE_IT_*)` 就写完老工程的整条链：SYSCFG EXTICR nibble
（线←端口）+ FTSR/RTSR 触发锁存 + IMR 不屏蔽。**空 handler 盲区（知识 #18，本模块
必教）**：生成的 EXTI0/EXTI1_IRQHandler 是空体——PR（rc_w1 语义）永远清不掉，按键
即中断风暴。修法（USER CODE 区补，regen 稳定保留）：
`HAL_GPIO_EXTI_IRQHandler(KEYn_Pin)`。当前文件状态（固件 map 佐证）：EXTI1 已补
KEY2_Pin（6 字节，`PASS exti (3 presses)` 即它的功劳）；EXTI0 仍为空体（2 字节）——
KEY1 按下尚有风暴隐患，KEY1 的回调链（bsp_pwr 唤醒计数）依赖同一单行补丁待落。

## 引脚

| 引脚 | 信号/标签 | 模式 |
|---|---|---|
| PA0 | KEY1（KEY1_Pin）= EXTI0 | IT_RISING + Pull-down |
| PA1 | KEY2（KEY2_Pin）= EXTI1 | IT_FALLING + Pull-up |

## HAL API 调用清单

- `int bsp_exti_init(void)` — HAL_GPIO_Init 重编两线（幂等修复生成电配置偏差）+
  清陈旧挂起（`__HAL_GPIO_EXTI_CLEAR_IT`——PR rc_w1 的 HAL 拼写；挂起不清会让后续
  WFI/Stop 入睡被忽略，bsp_pwr 教训）+ 使能 EXTI0/EXTI1 NVIC（优先级 5，与 .ioc
  NVIC 节一致）+ 全链回读（IMR 0/1 不屏蔽、RTSR/FTSR 触发锁存、EXTICR nibble=0000
  端口 A、MODER 输入、PUPDR 10/01、NVIC 使能）。0=PASS。
- `uint32_t bsp_exti_key2_count(void)` — ISR 侧 volatile 计数主线读（volatile 教学
  点：不带限定符主编译器可把轮询读提升出循环，计数"冻结"）。
- `const char *bsp_exti_last_report(void)` — 上次自测报告串（console-free，rule #7；
  老工程的 UART 提示改为绿 LED 闪——看板，不看终端）。
- `int bsp_exti_selftest(void)` — 步 1 路由回读；步 2 交互 6 s 窗（LED 每 150 ms
  翻转作提示；IWDG 每 500 ms 喂——窗长 6 s 逼近 ~8.2 s 狗，喂狗强制非装饰）；
  计数为跨开机累计语义，≥3 → `(N presses)`，超时 → `(idle, N presses)`，两态皆 PASS。
- 回调分发（bsp_exti.c）：`HAL_GPIO_EXTI_Callback` 是整个端口唯一的 weak override
  （两模块都 override 会链接冲突——单回调所有权规则），按 GPIO_Pin 过滤：KEY2 计数；
  KEY1 → `bsp_pwr_on_exti0()`（pwr 模块的唤醒计数钩子）。

```c
#include "bsp_exti.h"
uint32_t n = bsp_exti_key2_count(); /* presses counted by the ISR */
```

寄存器概念 → HAL 调用映射：EXTICR nibble / FTSR/RTSR / IMR → `HAL_GPIO_Init`
(IT_RISING / IT_FALLING) 一次写完；PR rc_w1 清挂起 → `__HAL_GPIO_EXTI_CLEAR_IT`；
NVIC SetPriority/EnableIRQ → `HAL_NVIC_*`；ISR 三语句纪律（读测 PR / rc_w1 清 /
计数）→ Core 向量分发进 weak 回调，ISR 侧只做"记笔记"。

## 生成器盲区备注（如有）

- EXTI0/EXTI1_IRQHandler 空体 → USER CODE 区补 HAL_GPIO_EXTI_IRQHandler（上文；
  EXTI1 已落 KEY2_Pin，EXTI0 待补——KEY1 中断路由未接通，按键即风暴）。
- MX_GPIO_Init 只 SetPriority 不 EnableIRQ → 向量使能归 bsp_exti_init。
- main.h 曾无 KEY 标签（bsp_exti.c 旧注释按字面量命名）；现行 main.h 已有
  KEY1_Pin/KEY2_Pin 宏，it.c 的 KEY2_Pin 即用此宏。

## 自测验证

菜单 `exti`：**PASS (3 presses)**（test-report §3，KEY2 人工按键 3 次被 EXTI1+ISR
精确计数——修复空 handler 中断风暴后）。无按键窗 → `(idle, N presses)`，SKIP 语义
（硬件缺席不 FAIL）。

## 对应视频集数

EP10（外部中断：引脚→SYSCFG→EXTI→NVIC 四级链路与 ISR 纪律）。
