# bsp_rtc — RTC 日历 + 备份域（EP16）

LSE 32.768 kHz 日历（本 build 实跑 LSI，见盲区备注）、Alarm A 每分钟秒匹配、
20×BKPxR 备份寄存器（CR1220/VBAT 供电，全复位存续）。备份域深讲（PWR DBP →
BDCR RTCSEL → WPR 三层锁、PRER 两写法、影子寄存器撕裂）看老工程
`docs/periph/rtc.md` 表 3。HAL 版所有权变更：RTC 已进 .ioc，备份域整链归
MspInit/驱动，模块 verify-only + 日历/报警/BKP API。

## CubeMX GUI 配置步骤

1. RTC → **Activated**（+ Activate Clock Source），Calendar 勾选。
2. Calendar（生成真值 rtc.c）：HourFormat=**24h**、AsynchPrediv=**127**、
   SynchPrediv=**255**（32.768k/(128×256)=1 Hz 计划）、OutPut=Disable。
3. Clock Source：GUI/.ioc 意图 **LSE**（`RCC.RTCClockSelection=
   RTC_CLKSOURCE_LSE`；PC14/PC15 板焊晶振）——但生成代码恒 LSI，属生成器
   盲区（见下节与 app_main 修正段）。
4. Time/Date：生成代码每次上电无条件打 00:00:00 / Monday 2000-01-01
   （`HAL_RTC_SetTime/SetDate` BCD）——生成行为保留；"这次上电是谁让的"
   标记走 BKP 寄存器（EP05/EP15 联动）。
5. NVIC：RTC_Alarm 中断不开（轮询：ISR.ALRAF 软件看；alarm 走非 IT
   `HAL_RTC_SetAlarm`——stm32f4xx_it.c 无 RTC_Alarm_IRQHandler 可派发）。
6. 引脚：LSE 建立时 PC14/PC15 为晶振模拟脚；本 build LSI 分支下 RTC 代码
   不占用这两脚。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| PC14 | OSC32_IN | 模拟（32.768 kHz 晶振；LSE 建立时） |
| PC15 | OSC32_OUT | 模拟（同上） |
| VBAT | — | 电源（CR1220 纽扣电池，备份域/BKPxR 供电） |

## HAL API 调用清单

- `int bsp_rtc_init(void)` — **verify-only**（DBP→时钟源→PRER 整链已由
  MspInit/`HAL_RCCEx_PeriphCLKConfig`/`HAL_RTC_Init` 在 boot 完成）：核验
  127/255+24h 计划、活动时钟源为 LSE 或 LSI（`__HAL_RCC_GET_RTC_SOURCE`
  只读，A3 豁免）、PRER 寄存器回读一致。0=PASS（报告带活动源），-1=不符。

典型用法：

```c
bsp_rtc_set_time(12, 34, 56);        /* 十进制入参, HAL 内部转 BCD */
uint8_t h, m, s;
bsp_rtc_get_time(&h, &m, &s);        /* GetTime 后必须跟 GetDate（影子锁） */
bsp_rtc_set_alarm_seconds(30);       /* 每分钟 :30 触发一次（非 IT） */
if (bsp_rtc_alarm_fired()) { /* ALRAF 已读清 */ }
bsp_rtc_write_bkp(0u, 0xA5A55A5Au);  /* 备份寄存器 0..19 */
```

- `int bsp_rtc_set_time(uint8_t hh, uint8_t mm, uint8_t ss)` — 十进制入参
  校验（h<24,m<60,s<60）后 `HAL_RTC_SetTime(RTC_FORMAT_BIN)`（WPR/INIT/
  INITF/BCD 全在驱动内部）；越界或驱动失败 -1。日期不动。
- `int bsp_rtc_get_time(uint8_t *hh, uint8_t *mm, uint8_t *ss)` —
  `HAL_RTC_GetTime` + 尾随 `HAL_RTC_GetDate`（HAL 契约：不读日期则影子
  寄存器对下次读保持锁定，RM 26.3.16；老工程的双读一致性检查由驱动完成）；
  读后重验范围（撕裂防护保留）。
- `int bsp_rtc_set_alarm_seconds(uint8_t ss)` — Alarm A 屏蔽日期/时/分、
  只比秒字段（`RTC_ALARMMASK_*` + 亚秒全屏蔽），非 IT
  `HAL_RTC_SetAlarm`（驱动内部等 ALRAWF 再编程再使能）。-1=越界/ALRAWF
  超时。
- `int bsp_rtc_alarm_fired(void)` — 轮询 ALRAF（只读宏，A3 豁免），置位即
  `__HAL_RTC_ALARM_CLEAR_FLAG` 清旗并返 1（清旗是本路径唯一授权写）；老
  WPR 出错返回路径已不存在（HAL 内部管 WPR）。
- `int bsp_rtc_write_bkp(uint32_t idx, uint32_t val)` /
  `uint32_t bsp_rtc_read_bkp(uint32_t idx)` — `HAL_RTCEx_BKUPWrite/Read`，
  索引 0..19（坏索引 write 返 -1 / read 返 0）。
- `const char *bsp_rtc_last_report(void)` — "(rtc src lsi, bkp0 kept)" 等
  一行报告。
- `int bsp_rtc_selftest(void)` — 返回 0=PASS，否则失败步号（编号承老工程）：
  1=init 核验（**无 LSE 等待了**——老 -2 SKIP 语义随等待一起消失）
  2=日历真走时：秒字段 +1 必须落在 2.5 s HAL_GetTick 窗内（LSI 展宽：
  一"秒"=32768 周期/f_LSI=0.70..1.93 s；计时器是独立的 HAL tick，交叉检验
  纪律不变）3=BKP0=0xA5A55A5A、BKP19=其反码写入读回，**值留在原处**供
  下次断电演示 VBAT 保留。

## 生成器盲区备注

**RTCClockSelection 生成器盲区**：.ioc 写 LSE，生成恒 LSI（rtc.c
`RTCClockSelection = RCC_RTCCLKSOURCE_LSI`）。app_main 修正段（非致命）：
LSE 有界等待 2 s → 就绪则 `HAL_RTC_DeInit` → RTCClockSelection=LSE →
`HAL_RTC_Init` + RTT 标记；未就绪留 LSI、报告如实说明。本轮实测走 LSI
分支（"rtc src lsi"），LSE 起振条件待复核。bsp_rtc_init 读活动源动态报告，
未来 .ioc 修到 LSE 无需改代码。

## 自测验证

菜单命令：`rtc`；本轮实测：**PASS (rtc src lsi, bkp0 kept)**（日历 + 备份
寄存器；时钟源见 test-report §4.3）。断电保留演示：USB 断电（CR1220 在位）
再上电，BKP0R 仍应读到 0xA5A55A5A。

## 对应视频集数

EP16（RTC 实时时钟与备份域）。
