# bsp_can_loopback — CAN1 500k 回环（EP18）

bxCAN 单板自证教学：500 kbit/s 位时序 + 回环模式自发自收。寄存器级深讲
（INRQ/INAK 模式机、BTR off-by-one、过滤器组解剖）看老工程
`docs/periph/can_lb.md` 表 3，本页只讲 HAL 映射。HAL 版所有权变更：CAN1
已进 .ioc，本模块只持有 .ioc 表达不了的部分。

## CubeMX GUI 配置步骤

1. Connectivity → CAN1 → **Activated**；引脚 PB8=CAN1_RX / PB9=CAN1_TX
   （AF9，No pull，Very High speed；板载 TJA1050 收发器）。
2. Bit Timing（生成真值 can.c）：**Prescaler=7、BS1=8tq、BS2=3tq、SJW=1tq**
   → tq=166.7 ns，12 tq/bit，APB1 42 MHz / (7+1) / 12 = **500 kbit/s**，
   采样点 (1+8)/12=75%（替换老工程 BRP=5/BS1=11/BS2=2 的 85.7% 形状——
   同波特率、更早采样）。
3. Parameters：Mode=**NORMAL**（.ioc 值；回环由模块运行时切换，见盲区备注）；
   TimeTriggered / AutoBusOff / AutoWakeUp / AutoRetransmission / RFLM /
   TXFP 全 Disable。
4. NVIC：CAN1_RX0 中断**不开**（轮询教学基线）。
5. 过滤器组 .ioc 表达不了——由模块 `HAL_CAN_ConfigFilter` 配（bank 0、
   32 位 mask、id=mask=0 全收 → FIFO0）。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| PB8 | CAN1_RX | 输入（AF9，TJA1050 RXD；回环模式内部回环、无视 CANRX 引脚） |
| PB9 | CAN1_TX | 输出（AF9，TJA1050 TXD；回环模式下仍输出真实帧波形可观测） |

## HAL API 调用清单

- `int bsp_can_loopback_init(void)` — 回环切换走 A3 红线内的 sanctioned
  HAL 模式（改 handle 字段 + 重跑驱动，不碰 MCR/BTR）：
  `hcan1.Init.Mode=CAN_MODE_LOOPBACK` → `HAL_CAN_Init`（INRQ→INAK 握手在
  驱动内部）→ `HAL_CAN_ConfigFilter`（全收→FIFO0）→ `HAL_CAN_Start`（等
  INAK 清）。BTR 只读回读证明"生成时序+LBKM"真到了硬件，随后清 FIFO0
  陈帧保再跑确定性。返回 0=PASS；-1=配置回读不符；-2=`HAL_CAN_Init` 失败
  （老 enter-init 超时位型）；-3=`HAL_CAN_Start` 失败（老 INAK 清除超时）。

典型用法（一发一收）：

```c
uint8_t tx[4] = {0xA1, 0xB2, 0xC3, 0xD4}, rx[8];
uint32_t id, dlc;
bsp_can_loopback_send(0x123, tx, 4);       /* 选最低空闲邮箱并置 TXRQ */
bsp_can_loopback_tx_done(10);              /* 等该邮箱完成 + TXOK 判定 */
bsp_can_loopback_recv(&id, rx, &dlc, 10);  /* FIFO0 弹出并比对 */
```

- `int bsp_can_loopback_send(uint32_t std_id, const uint8_t *data, uint32_t dlc)`
  — 标准 ID 数据帧；本 vintage 的 `HAL_CAN_AddTxMessage`（即新版 HAL 的
  `HAL_CAN_AddMessageToTxFifoQ`），扫 TSR 选最低空闲邮箱。0=受理；-1=无
  空闲邮箱/未启动（老 TME0=0 位型）；-2=id>0x7FF 或 dlc>8。
- `int bsp_can_loopback_tx_done(uint32_t timeout_ms)` — 等上次发送的邮箱
  完成（HAL_GetTick 有界 + SysTick 死守护，绝不挂死）。0=完成且 TXOK=1；
  -1=SysTick 死；-2=窗口耗尽；-3=完成但 TXOK=0（仲裁丢失/TERR；TSR 只读
  探针，A3 豁免）。
- `int bsp_can_loopback_recv(uint32_t *std_id, uint8_t *data, uint32_t *dlc,
  uint32_t timeout_ms)` — FIFO0 填充级等待 + `HAL_CAN_GetRxMessage`；
  data 需 ≥8 字节（HAL 拷全部 8 路 payload，小端 lane 次序驱动已吸收）。
  0=收到，-1=超时/参数错。
- `const char *bsp_can_loopback_last_report(void)` — "(frame OK)" /
  "(data mismatch)"。
- `int bsp_can_loopback_selftest(void)` — 返回 0=PASS，否则失败步号：
  1=握手（回环重 init+过滤器+start+BTR 回读）
  2=回环往返（ID=0x123、DLC=4 逐字节比对）
  3=第二帧完成旗标（RQCP+TXOK 同置——老工程的 rc_w0 写 1 清除实证
  **有意删除**：A3 禁写 TSR）。

## 生成器盲区备注

无重大盲区。教学点写明：**回环模式由 bsp_init 里
`hcan1.Init.Mode = CAN_MODE_LOOPBACK` + `HAL_CAN_Init` 切换**（.ioc 保持
NORMAL）——老工程的 INRQ/INAK 手工握手现在收进这三个 HAL 调用里。双板
总线实验 = 跳过回环切换、加载真实过滤器，留作扩展。

## 自测验证

菜单命令：`can_lb`；本轮实测：**PASS (frame OK)**（CAN1 500k 回环，帧收发
验证）。空板可跑：回环模式 bxCAN 内部 Tx→Rx 回环（RM 32.5.2），PB9 仍能
看到真实帧波形。

## 对应视频集数

EP18（CAN 总线与 bxCAN）。
