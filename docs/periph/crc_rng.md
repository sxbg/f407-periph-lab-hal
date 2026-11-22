# bsp_crc_rng — CRC32 硬件单元 + RNG 真随机（EP20）

F4"基本型"CRC（MPEG-2 语义、32 位字粒度）与 AHB2 RNG 真随机源。EP20 教学
对比三差异：zlib crc32（反射 + 异或终值 + 字节粒度）vs F4 硬件（MSB-first
无终值、字粒度）。寄存器级深讲看老工程 `docs/periph/crc_rng.md` 表 3。
HAL 版所有权变更：CRC/RNG 均已进 .ioc，模块只验生成结果 + HAL 驱动外设。

## CubeMX GUI 配置步骤

1. Computing → **CRC** → Activated：F4 单元无任何可配置项（多项式 0x4C11DB7/
   种子 0xFFFFFFFF 固化在硅里，无 POLY/INIT/REV 寄存器——F0/F7 代才有）；
   生成代码即裸 `HAL_CRC_Init`（crc.c 真值：`hcrc.Instance=CRC` 一行）。
2. Computing → **RNG** → Activated：默认参数、中断不开（IE=0，轮询教学
   基线）；`HAL_RNG_Init` 内部完成 CR:RNGEN 上电（模拟熵源+LFSR+健康监测）。
3. RNG 时钟：PLLQ=7 @ HSE 25 MHz → **48 MHz**，落在 RNG_CLK 法线内
   （config-spec 硬约束"RNG 必须 ≤48 MHz"；老工程 PLLQ=84 MHz 的 CECS
   告警点在本 build 不存在）。
4. NVIC：两者中断均不开（轮询基线）。
5. 无引脚——纯计算/熵源外设。

## 引脚

| 引脚 | 信号/标签 | 方向/模式 |
|---|---|---|
| — | — | 无引脚 |

## HAL API 调用清单

- `int bsp_crc_rng_init(void)` — **verify-only**（MspInit 在 boot 已开
  AHB1 CRCEN / AHB2 RNGEN 时钟门，`HAL_RNG_Init` 已给 RNG 上电）：只读
  核验两 handle 活、两时钟门开（`__HAL_RCC_*_IS_CLK_ENABLED`，A3 豁免）、
  RNG 状态 READY。0=PASS，-1=不符。幂等。
- `uint32_t bsp_crc_rng_crc32(const uint8_t *data, uint32_t len)` — 字节流
  过硬件：首块 `HAL_CRC_Calculate`（= CR:RESET + 喂字 + 读 DR，即老工程
  crc32 序列原话），续块 `HAL_CRC_Accumulate`。单元吃**恰好 32 位大端字**，
  尾部零填充——**填充字节是 CRC 输入的一部分**（EP20 课 #3）。返回裸 DR
  状态（MPEG-2 语义，无反射无 xorout）；len=0 或 data=NULL → 0xFFFFFFFF。

典型用法：

```c
static const uint8_t vec[9] = "123456789";
uint32_t crc = bsp_crc_rng_crc32(vec, 9u);  /* 0xAE24E09D（含 1 个补零字） */
uint32_t rnd;
if (bsp_crc_rng_random(10, &rnd) == 0) { /* rnd = 新随机字 */ }
```

- `int bsp_crc_rng_random(uint32_t timeout_ms, uint32_t *out)` — 单个 32 位
  随机字：`HAL_RNG_GenerateRandomNumber` 内部轮询 SR:DRDY、DR 读清 DRDY。
  注意 timeout_ms 仅为接口对齐保留，实际 DRDY 窗是 HAL 内部
  RNG_TIMEOUT_VALUE（2 ms，tick 基）；DRDY 通常两个 RNG_CLK 周期内置位，
  实践不可见。0=交付，-1=DRDY 超时/参数错。
- `int bsp_crc_rng_selftest(void)` — 返回 0=PASS，否则失败步号：
  1=CRC 三方契约（硬件=软件位反照=离线常量："12345678"→0x49E3C2FB、
  "123456789"+3 补零→0xAE24E09D；目录值 0x0376E6E7 属未填充流，**不**
  断言——EP20 课 #3）2=RNG 首字 FIPS 丢弃后连读 8 字无相邻重复（连续
  随机数检测：重复=健康失败，p=2^-32/对）3=CECS/SECS 健康位全清（只读
  旗标，A3 豁免）。

## 生成器盲区备注

无。CRC/RNG 在 HAL 版就是"无参 IP"：GUI 只有 Activated 开关，生成代码
裸 init，无 IOC↔代码漂移面。RNG 时钟硬约束（PLLQ≤48 MHz）由时钟树保证，
不落在本 IP 的配置项里。

## 自测验证

菜单命令：`crc_rng`；本轮实测：**PASS**（CRC32 MPEG-2 向量 + RNG 连读）。
离线对照：`python -c "import zlib; print(hex(zlib.crc32(b'123456789')))"`
→ 0xcbf43926（zlib 孪生值，故意不同的算法形状——EP20 对比课的锚点）。

## 对应视频集数

EP20（CRC 校验与随机数）。
