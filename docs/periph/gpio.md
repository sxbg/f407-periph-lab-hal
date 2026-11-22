# bsp_gpio — GPIO：LED/按键/背光（EP02）

本模块教 CubeMX 引脚配置（输出初值/上下拉/EXTI 模式）与 `HAL_GPIO_*` 写读回环：PC5/PB2 LED、PA0/PA1 按键、PA15 LCD 背光。寄存器版（MODER/BSRR 手写）细节见 f407-periph-lab 工程同名文档。

## CubeMX GUI 配置步骤

1. 左侧 Categories → System Core → GPIO（或在芯片视图直接点引脚选模式）：
   - **PC5** → GPIO_Output，标签 `LED_GREEN`；Output Level = Low，Speed = Low，Pull = No pull（生成初值 `GPIO_PIN_RESET`，绿灯灭）
   - **PB2** → GPIO_Output，标签 `LED_BLUE`；Output Level = High（开机即亮），Speed = Low
   - **PA15** → GPIO_Output，标签 `LCD_BL`；Output Level = Low，Speed = Low（背光灭）
   - **PA0** → External Interrupt Mode with **Rising** edge trigger detection，标签 `KEY1`，Pull-down（按下接 VDD → 高有效）
   - **PA1** → External Interrupt Mode with **Falling** edge trigger detection，标签 `KEY2`，Pull-up（按下接 GND → 低有效）
2. GPIO 模式页确认：三个输出脚 = Output Push Pull；两个按键脚 = External Interrupt Mode。
3. NVIC Settings（System Core → NVIC）：EXTI0/EXTI1 优先级 = 5。生成代码只 `HAL_NVIC_SetPriority`，`EnableIRQ` 与 handler 在 USER CODE 补（EXTI 细节归 exti.md）。
4. 时钟：GPIOA/B/C/D/E/H 端口时钟由 `MX_GPIO_Init` 统一使能（AHB1），无需手动。

生成真值锚点（`Core/Src/gpio.c`）：输出脚 `GPIO_MODE_OUTPUT_PP` + `GPIO_SPEED_FREQ_LOW` + `GPIO_NOPULL`；KEY1 `GPIO_MODE_IT_RISING` + `GPIO_PULLDOWN`；KEY2 `GPIO_MODE_IT_FALLING` + `GPIO_PULLUP`；初值经 `HAL_GPIO_WritePin` 落地。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| PC5 | LED_GREEN | 输出推挽，Low speed，初值低（绿灯灭） |
| PB2 | LED_BLUE | 输出推挽，Low speed，初值高（蓝灯亮） |
| PA0 | KEY1 | EXTI 上升沿，下拉输入；按下=1，空闲=0 |
| PA1 | KEY2 | EXTI 下降沿，上拉输入；按下=0，空闲=1 |
| PA15 | LCD_BL | 输出推挽，Low speed，初值低（背光灭） |

PA15 默认 AF0 = JTAG JTDI，配置为输出即重路由；SWD（PA13/PA14）不受影响。

## HAL API 调用清单

bsp_gpio 对外 7 个函数；console-free，自测返回 0=PASS、非零=失败步号。引脚配置本身归 `MX_GPIO_Init`（CubeMX 拥有），本模块只用 `HAL_GPIO_WritePin`/`HAL_GPIO_ReadPin` 驱动。

| bsp 函数 | 对应 HAL 调用 | 一句作用 |
|---|---|---|
| `int bsp_gpio_init(void)` | `HAL_GPIO_ReadPin` ×5 | 读回启动态/按键空闲电平（只读，-1=失配） |
| `void bsp_gpio_led_green(uint32_t on)` | `HAL_GPIO_WritePin(PC5)` | 绿灯开关（PC5，启动态低） |
| `void bsp_gpio_led_blue(uint32_t on)` | `HAL_GPIO_WritePin(PB2)` | 蓝灯开关（PB2，启动态高） |
| `void bsp_gpio_lcd_backlight(uint32_t on)` | `HAL_GPIO_WritePin(PA15)` | 背光开关（启动态低） |
| `uint32_t bsp_gpio_key1_pressed(void)` | `HAL_GPIO_ReadPin(PA0)` | 1=按下（高有效） |
| `uint32_t bsp_gpio_key2_pressed(void)` | `HAL_GPIO_ReadPin(PA1)` | 1=按下（低有效） |
| `int bsp_gpio_selftest(void)` | 上述组合 | 3 步自测（见下节） |

典型调用：

```c
#include "bsp_gpio.h"

bsp_gpio_led_green(1);                /* HAL_GPIO_WritePin 置高 */
if (bsp_gpio_key1_pressed()) {        /* HAL_GPIO_ReadPin，高有效 */
    bsp_gpio_led_blue(0);             /* 按住 KEY1 灭蓝灯 */
}
bsp_gpio_lcd_backlight(0);            /* PA15 拉低关背光 */
```

## 生成器盲区备注

- KEY1/KEY2 标签曾在 .ioc 剥键事故中丢失（引脚在 Mcu.Pin 列表但 Mode/Label 键被剥）；恢复后 `main.h` 才有 `KEY1_Pin`/`KEY2_Pin` 宏——标签丢失时引脚宏退化为裸 `GPIO_PIN_0`，bsp 代码无处引用。
- 两键生成为 EXTI 模式：生成代码只含引脚配置 + NVIC 优先级，IRQ handler 留空会中断风暴，必须在 USER CODE 补 `HAL_GPIO_EXTI_IRQHandler`——按键引脚到此为止，EXTI 完整链路见 exti.md。

## 自测验证

菜单命令：`gpio`
判据：`PASS gpio`。三步：① 启动态 + 按键空闲电平，3 次采样跨 ~2 ms（跑之前松开按键）② 输出回环：HAL 写 → IDR 验证跟随 → 恢复启动态 ③ 可见闪烁：绿/蓝各两闪、间隔 200 ms（给人眼看的）。
本轮实测：**PASS\***——PC5/PB2 LED、PA0/PA1 键、PA15 BL；模块本体手动复核多次 PASS（自动套件字符 pacing 粘连为脚本层遗留，非模块缺陷，test-report §4.2）。

## 对应视频集数

EP02（GPIO 输入输出与按键）。
