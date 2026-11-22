# bsp_can_loopback — CAN1 500k 回环（EP18）

寄存器级 bxCAN：PB8/PB9（TJA1050 板载收发器），500 kbit/s，回环模式自发自收。
菜单命令 `can_lb`（带 last_report `(frame OK)`）。CAN1 **不在** .ioc；PB8/PB9 为
GPIO_Analog 占位（CAN_RX/CAN_TX），AF9 由本模块写入；回环模式下 bxCAN 内部回环、
无视 CANRX 引脚（RM 32.5.2），空板也能跑。

## 表 1 — CubeMX 等效配置（模块为真值源；.ioc 中无此 IP）

| 字段 | 值 | 说明 |
|---|---|---|
| Mode | Loop back（LBKM=1） | 单板自证；双板总线才接 TJA1050 |
| Bit Timing | BRP=5, BS1=11, BS2=2, SJW=0 | 42 MHz/6/14=500 kbit/s，采样点 85.7%（BTR=0x401A0005 含 LBKM） |
| Kernel Clock | PCLK1 = 42 MHz 直连 | CAN 非定时器，无 APB ×2 |
| Filter | Bank 0，32 位 mask，全收 | FIFO 0；FINIT 解锁/上锁 |
| NVIC | CAN1_RX0_IRQn off | 轮询教学基线 |
| PB8/PB9 | GPIO_AF9，无上下拉 | CAN_RX/CAN_TX |

## 表 2 — 引脚

| 脚 | AF | 方向 | 板载器件 |
|---|---|---|---|
| PB8 | AF9 | 输入（CAN1_RX） | TJA1050 RXD |
| PB9 | AF9 | 输出（CAN1_TX） | TJA1050 TXD（回环模式下也输出真实帧波形可观测） |

## 表 3 — 寄存器清单

| 寄存器 | 关键位/值 | 一句话 |
|---|---|---|
| RCC->APB1ENR | CAN1EN bit25 | 内核时钟门控 |
| CAN1->MCR | INRQ / SLEEP | 初始化请求模式机：INRQ→INAK→配置→INRQ=0 |
| CAN1->MSR | INAK / SLAK | 模式握手回执（SLAK=0 防吃 TXRQ） |
| CAN1->BTR | BRP/TS1/TS2/SJW/LBKM | 位时序实算（TS1/TS2 是减一字段，经典 off-by-one 陷阱） |
| CAN1->FMR / FM1R / FS1R / FFA1R / FA1R | FINIT / FBM0=0 / FSC0=1 / FFA0=0 / FACT0=1 | 过滤器组 0：掩码模式、32 位、路由 FIFO0、激活 |
| CAN1->sFilterRegister[0].FR1/FR2 | 0 / 0 | 全 0 掩码 = 全帧接收 |
| CAN1->sTxMailBox[0].TIR/TDTR/TDLR/TDHR | STID<<21 / DLC / 小端双字 | TX 描述符+载荷；TIR|=TXRQ 启动 |
| CAN1->TSR | RQCP0 / TXOK0 / TME0 | 完成链（rc_w0 写 1 清）；TXOK 区分成败 |
| CAN1->RF0R | FMP0 / RFOM0 | FIFO0 计数 + rc_w0 弹出 |
| CAN1->sFIFOMailBox[0].RIR/RDTR/RDLR/RDHR | ID>>21 / DLC[3:0] / 小端 | RX 描述符+载荷（读无副作用） |

## 对应视频集数

EP18（CAN 总线与 bxCAN）。
