# bsp_tim_pwm — TIM1/TIM4 PWM 输出（EP06）

本模块教：CubeMX 只生成定时器基础 init（PSC=0/ARR=max/CEN=0），通道形状（PWM 模式/PSC/ARR/占空）由 `HAL_TIM_PWM_*` 在运行时写；TIM1（APB2 ×2=168 MHz，高级）与 TIM4（APB1 ×2=84 MHz，通用）同播 1 kHz。寄存器版（CCMR1/EGR.UG 手写）细节见 f407-periph-lab 工程同名文档。

## CubeMX GUI 配置步骤

1. 左侧 Categories → Timers → **TIM1**：Clock Source = Internal Clock；Channel1 = **PWM Generation CH1**（引脚自动落 PA8）。
2. Parameter Settings（`MX_TIM1_Init` 真值）：Prescaler = 0；Counter Mode = Up；Counter Period = 65535；auto-reload preload = **Disable**（ARR 写入立即生效）；Clock Division = DIV1；RepetitionCounter = 0。
3. Timers → **TIM4**：Channel1 = **PWM Generation CH1**（PD12）；参数同上（无 RepetitionCounter）。
4. 时钟树：TIM1 挂 APB2（PCLK2 = 84 MHz，总线有分频 → 定时器时钟 ×2 = **168 MHz**）；TIM4 挂 APB1（PCLK1 = 42 MHz ×2 = **84 MHz**）——RM ch.6 规则。
5. NVIC Settings：两个定时器中断都不开（轮询教学基线）。
6. 生成代码只到 `HAL_TIM_Base_Init`（CEN=0，通道未配）；1 kHz 形状由 bsp 模块武装——这是 HAL 版与寄存器版的分工边界。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| PA8 | TIM1_CH1 | AF1 推挽输出，Low speed（板头排针，波形教学点） |
| PD12 | TIM4_CH1 | AF2 推挽输出，Low speed（板头排针） |

## HAL API 调用清单

bsp_tim_pwm 对外 3 个函数 + 2 个通道选择宏（`BSP_TIM_PWM_CH_TIM1_CH1` / `BSP_TIM_PWM_CH_TIM4_CH1`）；console-free，自测返回 0=PASS、非零=失败步号。通道形状只经 `HAL_TIM_PWM_ConfigChannel`/`HAL_TIM_PWM_Start` 和 `__HAL_TIM_SET_*` 宏写（A3 红线；寄存器读仅作校验）。

| bsp 函数 | 对应 HAL 调用 | 一句作用 |
|---|---|---|
| `int bsp_tim_pwm_init(void)` | `HAL_TIM_PWM_Stop` → `HAL_TIM_PWM_ConfigChannel`（PWM1、Pulse=500 预装载）→ `__HAL_TIM_SET_PRESCALER/AUTORELOAD/COUNTER` → `HAL_TIM_PWM_Start`（CC1E + TIM1 MOE + CEN 最后置） | 双通道武装为 1 kHz / 50%（幂等） |
| `int bsp_tim_pwm_set_duty(uint32_t ch, uint32_t per_mille)` | `__HAL_TIM_SET_COMPARE` | 千分比占空（1000=常高，>1000 拒绝返回 -1）；OC1PE 预装载 → 下一帧边界生效（≤1 ms，无毛刺） |
| `int bsp_tim_pwm_selftest(void)` | 读回 PSC/ARR/CCR/CCER/MOE/CEN | 3 步自测（见下节） |

`HAL_TIM_PWM_ConfigChannel` 从结构体写 CCMR1/CCR1，全字段真值：`OCMode=TIM_OCMODE_PWM1`（CNT<CCR 时有效）、`Pulse=500`（50% 预装载）、`OCPolarity=TIM_OCPOLARITY_HIGH`、`OCFastMode=TIM_OCFAST_DISABLE`、`OCIdleState=TIM_OCIDLESTATE_RESET`（MOE 清掉后引脚静止在低）。

频率两步分频（两个定时器共用同一公式）：f_tick = f_tim/(PSC+1)，f_frame = f_tick/(ARR+1)。TIM1：168M/(167+1) = 1 MHz tick，/(999+1) = 1 kHz；TIM4：84M/(83+1) = 1 MHz。内核时钟差 2:1，终值刻意同为 1 MHz/1 kHz——占空数学共享。

典型调用：

```c
#include "bsp_tim_pwm.h"

bsp_tim_pwm_init();                                  /* 双通道 1 kHz 50% */
bsp_tim_pwm_set_duty(BSP_TIM_PWM_CH_TIM1_CH1, 250);  /* PA8 → 25% 占空 */
bsp_tim_pwm_set_duty(BSP_TIM_PWM_CH_TIM4_CH1, 0);    /* PD12 → 常低 */
```

## 生成器盲区备注

- **PSC 影子寄存器 + EGR.UG 盲区**：PSC 写入在下一个更新事件才锁进影子，而 HAL 没有 EGR.UG 立即装载的拼写——Start 后第一帧按旧 PSC=0 跑（TIM1 ~6 µs / TIM4 ~12 µs），之后 1 kHz 节拍锁定。教学上接受，代码写点处有注释。
- **TIM1 BDTR MOE**：由 `HAL_TIM_PWM_Start` 内部置位；少了它 CCER 全开 PA8 也是平线——高级定时器第一坑，自测步 3 专查。
- **幂等重配**：HAL 通道状态机 BUSY 时再 Start 是 HAL_ERROR，所以 init 先 `HAL_TIM_PWM_Stop` 恢复 READY 并掉 CC1E/MOE/CEN（寄存器版直接写 CR1=0 的 HAL 等价物）。
- ARPE = Disable（IOC 真值）：ARR 写入立即生效，无影子延迟。

## 自测验证

菜单命令：`tim_pwm`
判据：`PASS tim_pwm`。三步：① PSC/ARR 读回（TIM1 167/999、TIM4 83/999）② `set_duty(500)` 后 CCR 读回落在 ARR/2 ±1 ③ 使能位读回：两路 CC1E、TIM1 BDTR MOE、两路 CR1 CEN（示波器不在时永远回到这三个位）。
本轮实测：**PASS\***——TIM1_CH1=PA8 + TIM4_CH1=PD12 1 kHz；手动复核多次 PASS（套件 pacing 遗留，test-report §4.2）。
闭环交叉验证：`pwm_ic_loop`——PA8→PB3 杜邦线，TIM1 1 kHz PWM → TIM2 捕获实测 950–1050 Hz。

## 对应视频集数

EP06（定时器 PWM 输出）。
