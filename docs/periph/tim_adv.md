# bsp_tim_adv — TIM8 互补 PWM + 死区（EP14）

高级定时器教学：TIM8_CH1=PC6 独立 PWM、CH2=PC7 / CH2N=PB14 互补对 + 1 µs
死区（BDTR DTG 两段编码实算）。寄存器级深讲（RCR/BDTR/EGR 影子装载）看老
工程 `docs/periph/tim_adv.md` 表 3，本页只讲 HAL 映射与生成真值。

## CubeMX GUI 配置步骤

1. TIM8 → Mode → **PWM Generation CH1** + **PWM Generation CH2 with CH2N**
   （.ioc 真值：Channel-PWM Generation1 CH1 / Generation2 CH2 / CH2N 三键；
   路由 PC6/PC7/PB14 三脚，通道寄存器本模块写）。
2. Parameter Settings（生成真值 tim.c）：Prescaler=**0**、Counter Period=
   **65535**、RepetitionCounter=**0**、ClockDivision=**DIV1**（→ t_DTS=
   5.952 ns，死区基准）、AutoReloadPreload=Disable——基础 init；1 kHz 帧
   与占空由模块运行时接管（PSC=167、ARR=999 → 1 MHz 节拍 / 1 kHz 帧）。
3. Break And Dead-time：本板 PB12 无 break 源，**Break 故意不配置**（BKIN
   悬空时武装只会给 MOE 异步清零添路）；死区 DTG 由模块
   `HAL_TIMEx_ConfigBreakDeadTime` 写 **0x94**（EP14 实算：1000 ns / 5.952
   ns = 168 tick > 127 → 第二段 DTG=10_010100b=0x94，实际
   (64+20)×2×5.952=999.9 ns）。
4. GPIO：PC6/PC7/PB14 均 AF3、No pull、Low speed（PB14 走 MspPostInit）。
5. MOE 红线：高级定时器陷阱——`HAL_TIM_PWM_Start`/`HAL_TIMEx_PWMN_Start`
   内部 `__HAL_TIM_MOE_ENABLE`；没有 MOE，三路 CCER 使能全是死脚。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| PC6 | TIM8_CH1 | 输出（AF3，独立 PWM，1 kHz / 50%） |
| PC7 | TIM8_CH2 | 输出（AF3，互补对正相） |
| PB14 | TIM8_CH2N | 输出（AF3，互补对反相，含 1 µs 死区间隙） |

## HAL API 调用清单

- `int bsp_tim_adv_init(void)` — 三路输出 init：`HAL_TIM_PWM_Stop` ×2 +
  `HAL_TIMEx_PWMN_Stop`（幂等重入）→ `HAL_TIM_PWM_ConfigChannel` ×2
  （PWM1 模式、Pulse=500 即 50%、active high、idle LOW）→ 死区 0x94 →
  `__HAL_TIM_SET_PRESCALER/AUTORELOAD`（167/999）→ Start ×2 +
  `HAL_TIMEx_PWMN_Start`。返回 0=PASS（CCER/BDTR/PSC/ARR 只读回读，
  A3 豁免），-1=HAL 失败或回读不符。

典型用法：

```c
bsp_tim_adv_init();                            /* PC6 独立 + PC7/PB14 互补 */
bsp_tim_adv_set_deadtime_ns(500);              /* 旋钮: 500 ns 就地重写 */
uint32_t got = bsp_tim_adv_get_deadtime_ns();  /* 只读解码 DTG 回 ns */
```

- `int bsp_tim_adv_set_deadtime_ns(uint32_t ns)` — ns→DTG 四段编码走查
  （≤756 ns 直编码；≤1512 用 2×t_DTS 步进/0x80 偏置；≤3024 ×8/0xC0；
  ≤6048 ×16/0xE0），经 `HAL_TIMEx_ConfigBreakDeadTime` 下写并重 arm MOE
  （该调用 AutomaticOutput=DISABLE 会清 MOE）。0 或 >6048 ns 拒绝返回 -1，
  无写入。
- `uint32_t bsp_tim_adv_get_deadtime_ns(void)` — 只读解码当前 BDTR DTG
  字段（HAL 无 DTG getter，A3 豁免读）；set 的精确逆变换，自测往返用。
- `int bsp_tim_adv_selftest(void)` — 返回 0=PASS，否则失败步号：
  1=DTG/CCER/MOE 回读（0x94、三使能、MOE、CEN）
  2=死区 ns 往返（1000→999.9、200→202.4，误差 ≤1 个量化步）
  3=互补极性（CC2P=CC2NP=0，对称 active-high，只有死区隔开两相）。

## 生成器盲区备注

无重大盲区。分工说明：IOC 只给基础时基（PSC=0/ARR=65535），1 kHz 帧与死区
是模块运行时 HAL 调用（`__HAL_TIM_SET_*` + `HAL_TIMEx_ConfigBreakDeadTime`）。
首帧小坑（非盲区）：PSC 影子——Start 后第一帧仍按旧 PSC=0 跑（约 6 µs），
随后 1 kHz 节拍才锁定。

## 自测验证

菜单命令：`tim_adv`；本轮实测：**PASS**（TIM8 互补 PWM + 1 µs 死区）。
示波器看点：PB14 是 PC7 的精确反相，唯每个翻转沿 ±1 µs 窗内两脚同为低
（死区，结构性无直通）；死区旋钮 `set_deadtime_ns(5000)` 落 5048 ns
（第 4 段步进 95 ns，5 µs 是离格目标）。

## 对应视频集数

EP14（高级定时器：互补输出与死区）。
