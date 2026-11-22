/**
 ******************************************************************************
 * @file    bsp_can_loopback.c
 * @brief   CAN1 register-level bxCAN (PB8/PB9, 500 kbit/s loop-back) --
 *          teaching module with init handshake and self-test.
 ******************************************************************************
 * @addtogroup BSP_CAN_LOOPBACK
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | CAN1 in IOC       | NOT PRESENT                          | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | PB8/PB9 in IOC    | GPIO_Analog, labels CAN_RX/CAN_TX    | pin-map
 * |                   |                                      | placeholders
 * |                   |                                      | (anti-collision);
 * |                   |                                      | the AF9 mux is
 * |                   |                                      | written HERE
 * | Mode              | Loop back (LBKM=1)                   | single-board
 * |                   |                                      | self-proof; the
 * |                   |                                      | TJA1050 on
 * |                   |                                      | PB8/PB9 is for
 * |                   |                                      | the optional
 * |                   |                                      | dual-board bus
 * | Bit rate          | 500 kbit/s                           | EP18 worked
 * |                   |                                      | example (below)
 * | Filter            | Bank 0, 32-bit mask, accept-all      | FIFO 0, lab =
 * |                   |                                      | receive
 * |                   |                                      | everything
 * | NVIC              | CAN1_RX0_IRQn off                    | polling lesson
 * == REGISTER WALKTHROUGH (RM0090 ch.32, bxCAN) ============================
 * Kernel clock: RCC->APB1ENR bit25 (CAN1EN). CAN1 ticks at PCLK1 = 42 MHz
 *   DIRECTLY -- the "APB timer x2" rule from bsp_tim_* does NOT apply to
 *   CAN (it is not a timer; its APB bridge has no multiplier).
 * Pin mux: PB8 -> AF9 (CAN1_RX), PB9 -> AF9 (CAN1_TX): MODER [17:16]/
 *   [19:18] = 10, AFR[1] nibbles 0/1 = 1001. In loop-back mode the bxCAN
 *   feeds Tx back to Rx internally and DISREGARDS the CANRX pin (RM0090
 *   32.5.2) -- so this lab runs even with nothing wired to the pins.
 * Mode machine (RM0090 32.4.2) -- the handshake every bxCAN driver needs:
 *   1. MCR: INRQ=1, SLEEP=0 -> hardware leaves sleep and, once 11
 *      recessive bits are seen, syncs to the bus and answers INAK=1.
 *   2. NOW the config registers unlock: MCR (NART/RFLM/AWUM/ABOM/TTCM all
 *      left 0 = automatic retransmission, FIFO overrun overwrites oldest
 *      frame, manual wake/bus-off, no time-triggered mode), BTR, filters.
 *   3. INRQ=0 -> hardware resyncs and clears INAK: normal mode.
 *   Every wait is bounded by a SysTick window (spec #5).
 * BIT TIMING -- the EP18 exercise at 500 kbit/s:
 *   f_bit = f_PCLK1 / (BRP+1) / t_q_per_bit.
 *   Choose BRP=5 -> tq clock = 42 MHz / 6 = 7 MHz -> tq = 142.86 ns.
 *   500 kbit/s needs 7000/500k = 14 tq per bit: SYNC=1 + BS1=11 + BS2=2.
 *   BTR fields: BRP[5:0]=5, TS1[3:0]=BS1-1=10, TS2[2:0]=BS2-1=1, SJW=0
 *   (resync jump 1 tq). Whole value = 0x401A0005 (with LBKM bit30).
 *   Sample point = (1+11)/14 = 85.7% -- late sampling, the safe choice on
 *   a lab bus with modest length (CiA recommends ~87.5%).
 *   Quirks: BRP=0 would give 21 tq/bit (42 MHz/1) -- legal but wasteful;
 *   TS1/TS2 are register-minus-one fields, a classic off-by-one trap.
 * TX path -- mailbox 0 (three mailboxes exist, 0..2; TSR tracks each):
 *   TIR = STID<<21 (11-bit ID in STID[28:18]), IDE=0 standard, RTR=0 data
 *   frame; TDTR = DLC (0..8, DLC[3:0]); TDLR/TDHR carry the payload in
 *   LITTLE-ENDIAN lanes: byte0 -> TDLR[7:0] .. byte7 -> TDHR[31:24].
 *   TIR |= TXRQ starts the MAC. Completion chain in TSR: RQCP0 (request
 *   completed, rc_w0: write-1 to clear), TXOK0 (frame sent clean),
 *   ALST0/TERR0 the failure twins. TME0=1 says the mailbox is free again.
 * RX path -- FIFO 0 (two FIFOs, 3-deep each; RF0R tracks FIFO 0):
 *   FMP0[1:0] = pending count. Read RIR (ID>>21), RDTR (DLC[3:0]),
 *   RDLR/RDHR (same little-endian lanes as TX), then RF0R = RFOM0
 *   (rc_w0) to release the output mailbox and shift the next frame up.
 * Filters (RM0090 32.7) -- 28 banks shared between CAN1/CAN2 (CAN2SB
 *   splits them; CAN2 unused here):
 *   FMR: FINIT=1 to unlock FM1R/FS1R/FFA1R/FA1R/filter registers.
 *   Bank 0: FM1R FBM0=0 (MASK mode), FS1R FSC0=1 (single 32-bit scale),
 *   FFA1R FFA0=0 (route matches to FIFO 0), FA1R FACT0=1 (activate),
 *   FR1=0 (identifier compare) + FR2=0 (mask) = every bit "don't care"
 *   -> accept all frames. FINIT=0 locks it back.
 * CAN2.0B note: this lab speaks base frames only (IDE=0, 11-bit ID);
 *   extended frames would set IDE=1 and use EXID[28:0] -- the FS1R 32-bit
 *   scale already covers both. Arbitration lesson: the ID doubles as the
 *   bus priority (lower ID wins the wire-AND), which is why 0x123 beats
 *   any higher ID it may collide with on the optional dual-board bus.
 * == HOW TO VERIFY =========================================================
 * menu> can_lb
 *   "PASS can_lb (frame OK)" -- the controller shipped ID=0x123 and
 *   received its own bytes back through the internal Tx->Rx loop.
 *   All three steps are self-contained: no transceiver wiring needed.
 * Scope check (optional): PB9 (CANTX) shows the real frame waveform even
 *   in loop-back mode ("The transmitted messages can be monitored on the
 *   CANTX pin", RM0090 32.5.2): 1 dominant start bit, ID field, ~2 us/bit.
 * Dual-board extension: jumper CANH<->CANH, CANL<->CANL across boards,
 *   one 120R terminator per end, clear LBKM in BTR, and both boards'
 *   accept-all filters do the rest.
 */
#include "bsp_can_loopback.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* CAN_BTR field positions come from the CMSIS device header (RM0090
 * ch.32.8.4 table): BRP [5:0] = tq prescaler-1, TS1 [3:0] = BS1-1,
 * TS2 [2:0] = BS2-1, SJW [1:0] = resync jump width-1, LBKM bit30. */

/* 500 kbit/s at 42 MHz: BRP=5, BS1=11 (TS1=10), BS2=2 (TS2=1), SJW=0,
 * LBKM=1 -- the full EP18 derivation lives in the walkthrough. */
#define BSP_CAN_BTR_VALUE \
    ((5u << CAN_BTR_BRP_Pos) | (10u << CAN_BTR_TS1_Pos) | \
     (1u << CAN_BTR_TS2_Pos) | (0u << CAN_BTR_SJW_Pos) | \
     CAN_BTR_LBKM) /* = 0x401A0005 */

/* Timeout windows (SysTick COUNTFLAG granularity, 1 ms). INAK needs a few
 * bit times; a 500 kbit/s 4-byte frame takes ~110 us; 10 ms is generous
 * for all three. */
#define BSP_CAN_TO_MS 10u

/* Self-test step numbers (the value bsp_can_loopback_selftest returns). */
#define BSP_CAN_ST1_HANDSHAKE 1u /* INAK handshake + config read-back */
#define BSP_CAN_ST2_FRAME     2u /* loop-back TX -> RX round trip */
#define BSP_CAN_ST3_FLAGS     3u /* TXOK0/RQCP0 flag chain + rc_w0 clear */

/* Spin cap between SysTick wraps: SysTick-dead guard (spec #5). */
#define BSP_CAN_SYSTICK_GUARD 2000000u

/* ---- module state ----------------------------------------------------------- */

/* Report of the last selftest run; the menu reads it after the run. */
static char s_report[24];

/* ---- private helpers -------------------------------------------------------- */

/* Copy a fixed string into s_report (tiny, no libc). */
static void can_set_report(const char *txt)
{
    char *p = s_report;
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

/* Spin until (reg & mask) == want or timeout_ms elapses (SysTick COUNTFLAG
 * polling, 1 ms per wrap). Returns 0 = matched, -1 = timeout, -2 = SysTick
 * dead (bounded exit, never hang). */
static int can_wait_bits(volatile uint32_t *reg, uint32_t mask,
                         uint32_t want, uint32_t timeout_ms)
{
    uint32_t guard = BSP_CAN_SYSTICK_GUARD;
    for (;;) {
        if (((*reg) & mask) == want) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
                guard = BSP_CAN_SYSTICK_GUARD; /* alive: refill the cap */
            } else {
                return -1; /* window exhausted */
            }
        }
        if (--guard == 0u) {
            return -2; /* SysTick dead: bounded exit */
        }
    }
}

/* Spin until FIFO 0 holds a pending frame (FMP0 != 0). Returns 0 = frame
 * pending, -1 = timeout, -2 = SysTick dead. (can_wait_bits can't express
 * "wait for nonzero": its match test is an equality, so a 0-want matches
 * instantly -- the pending flag needs its own inverted loop.) */
static int can_wait_pending(uint32_t timeout_ms)
{
    uint32_t guard = BSP_CAN_SYSTICK_GUARD;
    for (;;) {
        if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0u) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
                guard = BSP_CAN_SYSTICK_GUARD;
            } else {
                return -1; /* window exhausted, no frame */
            }
        }
        if (--guard == 0u) {
            return -2; /* SysTick dead: bounded exit */
        }
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_can_loopback_init(void)
{
    /* Kernel clock: CAN1EN = APB1ENR bit25. Idempotent re-arm. */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    (void)RCC->APB1ENR; /* read-back stalls until the clock is live */

    /* Pin mux: PB8 -> AF9 (CAN1_RX), PB9 -> AF9 (CAN1_TX). MODER [17:16]/
     * [19:18] = 10 (alternate), AFR[1] nibbles 0/1 = 1001. No pulls: the
     * TJA1050 drives RX when wired, and loop-back mode ignores CANRX
     * anyway (RM0090 32.5.2). HAL equivalent:
     *   GPIO_InitStruct.Alternate = GPIO_AF9_CAN1; HAL_GPIO_Init(GPIOB,...). */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER8_Msk) |
                   GPIO_MODER_MODER8_1; /* [17:16]=10 AF mode */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER9_Msk) |
                   GPIO_MODER_MODER9_1; /* [19:18]=10 AF mode */
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk) |
                    GPIO_AFRH_AFSEL8_0 | GPIO_AFRH_AFSEL8_3; /* AF9 RX */
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL9_Msk) |
                    GPIO_AFRH_AFSEL9_0 | GPIO_AFRH_AFSEL9_3; /* AF9 TX */
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk; /* no pull on CAN_RX */
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk; /* no pull on CAN_TX */

    /* Mode machine step 1: request init mode. One write sets INRQ and
     * clears SLEEP (hardware refuses INRQ while SLEEP=1). */
    CAN1->MCR = CAN_MCR_INRQ;
    if (can_wait_bits(&CAN1->MSR, CAN_MSR_INAK, CAN_MSR_INAK,
                      BSP_CAN_TO_MS) != 0) {
        return -2; /* no INAK: controller never synced to init mode */
    }

    /* Step 2 (now unlocked): MCR defaults -- every policy bit 0 (see
     * walkthrough for each one's meaning); BTR = the EP18 500 kbit/s
     * shape + loop-back; then the accept-all filter bank 0. */
    CAN1->MCR = CAN_MCR_INRQ; /* NART/RFLM/AWUM/ABOM/TTCM/DBF = 0 */
    CAN1->BTR = BSP_CAN_BTR_VALUE;

    CAN1->FMR |= CAN_FMR_FINIT; /* unlock the filter banks */
    CAN1->FM1R &= ~CAN_FM1R_FBM0; /* bank 0: list->MASK mode (bit 0 = 0) */
    CAN1->FS1R |= CAN_FS1R_FSC0;  /* bank 0: single 32-bit scale */
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0; /* bank 0 matches -> FIFO 0 */
    CAN1->sFilterRegister[0].FR1 = 0u; /* 32-bit scale: FR1 = identifier,
                                        * FR2 = mask; all-zero mask =
                                        * every bit "don't care" */
    CAN1->sFilterRegister[0].FR2 = 0u; /* -> accept ALL frames */
    CAN1->FA1R |= CAN_FA1R_FACT0;  /* bank 0 live */
    CAN1->FMR &= ~CAN_FMR_FINIT;   /* re-lock the filter machinery */

    /* Mode machine step 3: leave init, hardware resyncs (INAK clears). */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    if (can_wait_bits(&CAN1->MSR, CAN_MSR_INAK, 0u, BSP_CAN_TO_MS) != 0) {
        return -3; /* stuck in init mode */
    }

    /* Read-back proof: the timing shape stuck and no sleep/init flag is
     * left set. SLAK=0 matters: a controller left asleep silently eats
     * TXRQs and every later step would time out. */
    if ((CAN1->BTR != BSP_CAN_BTR_VALUE) ||
        ((CAN1->MSR & (CAN_MSR_INAK | CAN_MSR_SLAK)) != 0u)) {
        return -1;
    }
    return 0;
}

int bsp_can_loopback_send(uint32_t std_id, const uint8_t *data,
                          uint32_t dlc)
{
    uint32_t tdlr = 0u;
    uint32_t tdhr = 0u;
    uint32_t i;

    if ((std_id > 0x7FFu) || (dlc > 8u) || (data == 0)) {
        return -2; /* bad arguments: no register touched */
    }
    /* Mailbox free? TME0=1 (TSR bit26). With three mailboxes a driver
     * could round-robin 0/1/2 -- the lab keeps one, the flag-chain story
     * stays single-threaded. */
    if ((CAN1->TSR & CAN_TSR_TME0) == 0u) {
        return -1; /* mailbox 0 still busy with the previous frame */
    }

    /* Payload into the little-endian lanes: byte0 -> TDLR[7:0] ... */
    for (i = 0u; i < dlc; i++) {
        if (i < 4u) {
            tdlr |= ((uint32_t)data[i]) << (8u * i);
        } else {
            tdhr |= ((uint32_t)data[i - 4u]) << (8u * (i - 4u));
        }
    }
    CAN1->sTxMailBox[0].TDTR = dlc; /* DLC[3:0]; TGT=0 (no timestamp) */
    CAN1->sTxMailBox[0].TDLR = tdlr;
    CAN1->sTxMailBox[0].TDHR = tdhr;
    /* Identifier last + the TXRQ in the SAME write: STID[28:18] |= bit0.
     * IDE=0 (standard frame), RTR=0 (data frame). */
    CAN1->sTxMailBox[0].TIR = (std_id << 21) | CAN_TI0R_TXRQ;
    return 0;
}

int bsp_can_loopback_tx_done(uint32_t timeout_ms)
{
    /* Completion chain: RQCP0 (rc_w0) is raised for ANY mailbox-0
     * completion; TXOK0 distinguishes success from ALST0/TERR0 failures.
     * First wait the completion, then clear it by writing the bit back. */
    {
        const int rc = can_wait_bits(&CAN1->TSR, CAN_TSR_RQCP0,
                                     CAN_TSR_RQCP0, timeout_ms);
        if (rc != 0) {
            return (rc == -1) ? -2 : -1; /* -2 timeout, -1 SysTick dead */
        }
    }
    if ((CAN1->TSR & CAN_TSR_TXOK0) == 0u) {
        CAN1->TSR = CAN_TSR_RQCP0; /* clear the stale completion flag */
        return -3; /* completed but NOT ok: arbitration lost / TERR */
    }
    CAN1->TSR = CAN_TSR_RQCP0; /* rc_w0 clear: write-1. Only the bits
                                * written 1 are cleared -- TXOK0 (also
                                * rc_w0) is written 0 here and survives. */
    return 0;
}

int bsp_can_loopback_recv(uint32_t *std_id, uint8_t *data, uint32_t *dlc,
                          uint32_t timeout_ms)
{
    uint32_t i;

    if ((std_id == 0) || (data == 0) || (dlc == 0)) {
        return -1;
    }
    /* FMP0 [1:0] = frames pending in FIFO 0. 0 pending after the window
     * = nothing came back (in loop-back mode that means the TX itself
     * failed -- check tx_done first). */
    if (can_wait_pending(timeout_ms) != 0) {
        return -1;
    }

    /* Read the output mailbox: ID, DLC, payload (same little-endian
     * lanes as TX). Reading the registers is side-effect free -- only
     * RFOM0 shifts the FIFO. */
    *std_id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FFu;
    *dlc = CAN1->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk;
    {
        const uint32_t l = CAN1->sFIFOMailBox[0].RDLR;
        const uint32_t h = CAN1->sFIFOMailBox[0].RDHR;
        for (i = 0u; i < 8u; i++) {
            if (i < 4u) {
                data[i] = (uint8_t)((l >> (8u * i)) & 0xFFu);
            } else {
                data[i] = (uint8_t)((h >> (8u * (i - 4u))) & 0xFFu);
            }
        }
    }
    CAN1->RF0R = CAN_RF0R_RFOM0; /* rc_w0: release the output mailbox */
    return 0;
}

const char *bsp_can_loopback_last_report(void)
{
    return s_report;
}

int bsp_can_loopback_selftest(void)
{
    static const uint8_t payload[BSP_CAN_LOOPBACK_SELFTEST_DLC] = {
        0xA1u, 0xB2u, 0xC3u, 0xD4u
    };
    uint8_t rx[8];
    uint32_t id = 0u;
    uint32_t dlc = 0u;
    uint32_t i;

    /* Step 1: the init handshake (INRQ -> INAK -> configure -> INAK
     * cleared) plus the BTR read-back: exactly the 500 kbit/s value the
     * walkthrough derives. A -2/-3 here means the controller never
     * reached/left init mode -- check APB1 clock + pins first. */
    if (bsp_can_loopback_init() != 0) {
        return (int)BSP_CAN_ST1_HANDSHAKE;
    }

    /* Step 2: loop-back round trip. Send ID=0x123 with 4 known bytes,
     * wait for the mailbox to confirm (TXOK0), then pull the frame out
     * of FIFO 0 and compare every field. */
    if (bsp_can_loopback_send(BSP_CAN_LOOPBACK_SELFTEST_ID, payload,
                              BSP_CAN_LOOPBACK_SELFTEST_DLC) != 0) {
        return (int)BSP_CAN_ST2_FRAME;
    }
    if (bsp_can_loopback_tx_done(BSP_CAN_TO_MS) != 0) {
        return (int)BSP_CAN_ST2_FRAME;
    }
    if (bsp_can_loopback_recv(&id, rx, &dlc, BSP_CAN_TO_MS) != 0) {
        return (int)BSP_CAN_ST2_FRAME; /* nothing came back: LBKM off? */
    }
    if ((id != BSP_CAN_LOOPBACK_SELFTEST_ID) ||
        (dlc != BSP_CAN_LOOPBACK_SELFTEST_DLC)) {
        can_set_report("(data mismatch)");
        return (int)BSP_CAN_ST2_FRAME;
    }
    for (i = 0u; i < dlc; i++) {
        if (rx[i] != payload[i]) {
            can_set_report("(data mismatch)");
            return (int)BSP_CAN_ST2_FRAME;
        }
    }
    can_set_report("(frame OK)");

    /* Step 3: TSR flag chain, observed raw on a second frame. First
     * clear BOTH rc_w0 flags from the step-2 completion so what follows
     * is provably fresh: RQCP0 AND TXOK0 must then be re-raised by the
     * new frame (completion + success), and RQCP0 must clear on its
     * write-1 (TXOK0 survives that write -- each rc_w0 bit clears on its
     * own 1, they do not shadow each other). */
    CAN1->TSR = CAN_TSR_RQCP0 | CAN_TSR_TXOK0;
    if (bsp_can_loopback_send(BSP_CAN_LOOPBACK_SELFTEST_ID, payload,
                              BSP_CAN_LOOPBACK_SELFTEST_DLC) != 0) {
        return (int)BSP_CAN_ST3_FLAGS;
    }
    if (can_wait_bits(&CAN1->TSR, CAN_TSR_RQCP0 | CAN_TSR_TXOK0,
                      CAN_TSR_RQCP0 | CAN_TSR_TXOK0,
                      BSP_CAN_TO_MS) != 0) {
        return (int)BSP_CAN_ST3_FLAGS;
    }
    CAN1->TSR = CAN_TSR_RQCP0; /* write-1-to-clear, exact semantics */
    if ((CAN1->TSR & CAN_TSR_RQCP0) != 0u) {
        return (int)BSP_CAN_ST3_FLAGS; /* did not clear: rc_w0 broken */
    }
    /* Drain the second frame from FIFO 0 so the lab stays tidy for a
     * re-run (ignore its content -- step 2 already proved it). */
    (void)bsp_can_loopback_recv(&id, rx, &dlc, BSP_CAN_TO_MS);
    return 0; /* all steps passed */
}

/** @} */
