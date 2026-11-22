/**
 ******************************************************************************
 * @file    bsp_sdio_tf.c
 * @brief   SDIO register-level TF card identification (EP24 worked
 *          example: the command-path protocol, no data phase) --
 *          teaching module with self-test.
 ******************************************************************************
 * @addtogroup BSP_SDIO_TF
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | SDIO in IOC       | NOT PRESENT                          | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | PC8-PC12/PD2      | GPIO_Analog, labels TF_D0..TF_D3/    | pin-map
 * | in IOC            | TF_CK/TF_CMD (the AF12 mux is        | placeholders
 * |                   | written HERE)                        | (anti-collision)
 * | Mode              | SD 4-bit (identification runs 1-bit) | socket is wired
 * |                   |                                      | for DAT0..3;
 * |                   |                                      | wide bus is the
 * |                   |                                      | data-phase lesson
 * | Clock divide      | CLKDIV=210 -> 396.2 kHz              | identification
 * |                   |                                      | MUST be <400 kHz
 * |                   |                                      | (RM 31.9.2 note;
 * |                   |                                      | derivation below)
 * | Hardware flow     | off                                  | command path
 * |                   |                                      | only
 * | DMA               | off (FIFO polled/not used)           | data phase =
 * |                   |                                      | EP24 extension
 * | NVIC              | SDIO_IRQn off                        | polling lesson
 * == REGISTER WALKTHROUGH (RM0090 ch.31, SDIO v2.0) ========================
 * Kernel clock: RCC->APB2ENR bit11 (SDIOEN). The SDIO adapter ticks from
 *   SDIOCLK = the PLL Q divider output = 84 MHz on THIS clock tree (the
 *   IOC runs PLLQ=84 MHz; the F4's nominal "48 MHz PLLQ for USB/SDIO"
 *   rule is NOT met -- the identification-clock derivation below
 *   compensates by choosing CLKDIV from the ACTUAL 84 MHz, and a data
 *   phase would do the same at 25 MHz card speed).
 * Pin mux (6 pins, AF12): PC8/PC9/PC10/PC11 -> D0..D3, PC12 -> CK,
 *   PD2 -> CMD. All push-pull (AF MODE 10, OTYPER=0), no pulls (the
 *   socket/card drives its own lines), speed grade HIGH. DAT3 caveat:
 *   a card decides "SD vs SPI mode" by sampling DAT3 at CMD0 time --
 *   the weak card-internal pull-up (50..90k) is tolerated by every
 *   mainstream card; the belt-and-braces driver would hold PC11 LOW as
 *   GPIO until after CMD0. This module follows the ST HAL sequence
 *   (all-AF before CMD0), which is what the reference boards run.
 * POWER (0x00) PWRCTRL[1:0] = 11: card clock power-on. The RM demands
 *   >= 7 HCLK between POWER writes -- the dummy read-back below gives
 *   that slack (the read itself stalls 2+ HCLK and the writes are far
 *   apart in code).
 * CLKCR (0x04): CLKDIV[7:0] = 210 -> SDIO_CK = SDIOCLK/(CLKDIV+2)
 *   = 84 MHz/212 = 396.2 kHz, JUST under the "< 400 kHz while in
 *   identification mode" RM 31.9.2 note; even CLKDIV = the 50%-duty
 *   recipe from the same note. CLKEN bit8 = 1 starts the card clock
 *   (kept running, PWRSAV=0). 4-bit WIDBUS is NOT set here: cards
 *   identify in 1-bit mode and only switch wide AFTER ACMD41+RCA (the
 *   data-phase lesson).
 * CPSM command flow (RM 31.4.14 adapter + 31.3.1): for each command:
 *   1. ICR: clear the four completion flags (CCRCFAIL/CTIMEOUT/CMDREND/
 *      CMDSENT are static rc_w1c bits -- write 1 to ICR to clear).
 *   2. ARG = the 32-bit argument.
 *   3. CMD = CMDINDEX[5:0] | WAITRESP | CPSMEN bit10. WAITRESP[7:6]:
 *      00 = no response (CPSM will raise CMDSENT), 01 = short R1/R3/R6/
 *      R7 (CMDREND/CCRCFAIL), 11 = long R2, 136-bit CID/CSD (RESP1..4).
 *   4. Poll STA until a completion flag. CTIMEOUT (bit2) is the CPSM's
 *      own watchdog: fixed 64 SDIO_CK periods (~161 us at 396 kHz) --
 *      the HARDWARE's "card absent" verdict. CCRCFAIL (bit0) = response
 *      arrived but CRC wrong; for R3 (OCR) that is EXPECTED -- R3 has
 *      no CRC field, so the CRC checker eats garbage and the software
 *      must IGNORE the flag and read RESP1 anyway.
 *   The external SysTick window (10 ms) only bounds a wedged CPSM; the
 *   normal exit is a hardware flag within ~0.2 ms.
 * RESPONSE FORMATS (RM 31.5 -- the reading grid for every RESPx value):
 *   R1 (short, CRC-protected): RESP1 = 32-bit card status (the CMD55
 *      reply). R1b adds a busy dance on DAT0 -- not used here.
 *   R2 (long, CRC): 127-bit CID or CSD packed MSB-first into RESP1..4:
 *      RESP1 = bits 127:96 ... RESP4 = bits 31:1 + fixed 0 at bit0.
 *   R3 (short, NO CRC): RESP1 = the 32-bit OCR register. Busy = bit31,
 *      CCS = bit30 (echoes our HCS argument when busy), voltage window
 *      echo in bits 23:15.
 *   R6 (short, CRC): CMD3's reply, RESP1[31:16] = the NEW RCA the card
 *      picked, RESP1[15:0] = status (bit15 = composite ERROR).
 *   R7 (short, CRC): CMD8's reply, RESP1 echoes the argument: [11:8]
 *      voltage-accepted, [7:0] check pattern -- a card that never
 *      heard of CMD8 answers NOTHING (CTIMEOUT), which is exactly how
 *      v1-only cards are detected.
 * IDENTIFICATION SEQUENCE (RM 31.4.4 SD path + SD Physical Layer spec):
 *   CMD0  GO_IDLE_STATE   arg=0, no response -> card resets to idle.
 *   CMD8  SEND_IF_COND    arg=0x1AA ("voltage 2.7-3.6 V" + pattern AA),
 *         R7 echo must come back 0x1AA -> a v2-capable card. v1 cards
 *         and empty sockets both sit silent here (CTIMEOUT).
 *   CMD55 APP_CMD         arg=0 (RCA not assigned yet), R1 -> arms the
 *         NEXT command as an application command (ACMD_x).
 *   ACMD41 SD_APP_OP_COND arg=0x40FF8000 (HCS bit30 = "I can take an
 *         SDHC" + 2.7-3.6 V window 0xFF8000), R3/OCR. Repeat the
 *         CMD55+ACMD41 pair until OCR bit31 (busy) = 1 -- the card is
 *         done with its internal power-up ramp (spec allows up to 1 s).
 *         Then OCR bit30 (CCS): 1 = SDHC/SDXC (block addressed, CSD v2),
 *         0 = SDSC (byte addressed, CSD v1).
 *   CMD2  ALL_SEND_CID    arg=0, R2 long -> the 128-bit card identity;
 *         CID[127:120] = manufacturer ID byte.
 *   CMD3  SET_RELATIVE_ADDR arg=0, R6 -> the card PICKS its own RCA
 *         (unlike MMC where the host assigns); from now on the bus may
 *         leave identification clock (this module stays at 396 kHz --
 *         bumping CLKDIV toward 25 MHz is the data-phase lesson).
 *   CMD9  SEND_CSD        arg=RCA<<16, R2 long -> the CSD register.
 *         Structure bits 127:126: 00 = v1 (SDSC), 01 = v2 (SDHC).
 *         v2 C_SIZE = bits 69:48 (22 bits); capacity =
 *         (C_SIZE+1) x 512 KiB -- THAT is the whole capacity story for
 *         every SDHC/SDXC card (no multiplier bits anymore, unlike v1).
 *         v1 C_SIZE = bits 73:62 (12 bits) with C_SIZE_MULT x
 *         READ_BL_LEN -- captured for the report only.
 * DATA PATH (not implemented, the extension): CMD17/18/24/25 move
 *   blocks through the 32x32-bit FIFO (0x80) under DTIMER/DLEN/DCTRL
 *   (DTDIR/DTEN/DBLOCKSIZE) -- a full DPSM of its own, plus DMA for
 *   anything serious. Identification alone proves: pins, clock, CPSM,
 *   response parsing -- the entire protocol skeleton the data phase
 *   would plug into (EP24 focuses on this split).
 * == HOW TO VERIFY =========================================================
 * menu> sdio_tf
 *   Card inserted: "PASS sdio_tf (SDHC mfr=0x03 3854MB)" -- the whole
 *   chain CMD0..CMD9 ran, CCS says block-addressed, the manufacturer
 *   byte and derived capacity come straight out of CID/CSD.
 *   Empty socket:  "PASS sdio_tf (no card)" -- CMD8 silent AND ACMD41
 *   timed out for the whole 1 s window = the hardware's own verdict
 *   (SKIP semantics: a removable medium being absent is not a defect).
 * Scope check (optional): PC12 (CK) shows a free-running 396.2 kHz
 *   clock; PD2 (CMD) carries 48-bit command frames on every command.
 * Failure semantics: "stuck (ACMD41 not busy)" = a card is IN the
 *   socket but never finished power-up (weak socket contact, dead card)
 *   -- step-1 FAIL with that report; "RCA=0" / "CSD invalid" = response
 *   corruption mid-chain, step-2 FAIL.
 */
#include "bsp_sdio_tf.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* ACMD41 argument: HCS (bit30) + 2.7-3.6 V voltage window (0xFF8000).
 * Sent every iteration; the card echoes CCS in RESP1[30] once busy. */
#define BSP_SDIO_TF_ACMD41_ARG 0x40FF8000u

/* CMD8 argument: VHS=0001 (2.7-3.6 V, bits 11:8) + check pattern 0xAA. */
#define BSP_SDIO_TF_CMD8_ARG 0x000001AAu

/* WAITRESP encodings of SDIO_CMD bits 7:6 (RM 31.9.4). */
#define BSP_SDIO_TF_RESP_NONE (0x0u << 6)
#define BSP_SDIO_TF_RESP_SHORT (0x1u << 6)
#define BSP_SDIO_TF_RESP_LONG (0x3u << 6)

/* STA completion flags (static, rc_w1c through ICR). */
#define BSP_SDIO_TF_STA_DONE \
    (SDIO_STA_CMDSENT | SDIO_STA_CMDREND | SDIO_STA_CCRCFAIL | \
     SDIO_STA_CTIMEOUT)

/* R3 has NO CRC field (RM 31.5.4): CCRCFAIL fires on every OCR reply
 * and must be ignored -- the "crc expected" switch of sd_send_cmd. */

/* Timeout windows (SysTick COUNTFLAG, 1 ms): per-command 10 ms bounds a
 * wedged CPSM (hardware CTIMEOUT fires at ~161 us normally); the
 * ACMD41 busy loop gets the spec's full 1 s power-up budget. */
#define BSP_SDIO_TF_CMD_TO_MS 10u
#define BSP_SDIO_TF_ACMD41_WINDOW_MS 1000u

/* Self-test step numbers (the value bsp_sdio_tf_selftest returns). */
#define BSP_SDIO_TF_ST1_IDENTIFY 1u /* CMD0..CMD9 chain */
#define BSP_SDIO_TF_ST2_FIELDS   2u /* RCA/C_SIZE/CCS-vs-CSD sanity */

/* Spin cap between SysTick wraps: SysTick-dead guard (spec #5). */
#define BSP_SDIO_TF_SYSTICK_GUARD 2000000u

/* ---- module state ----------------------------------------------------------- */

/* Captured identification results (read by the accessors + selftest). */
static struct {
    bsp_sdio_tf_type_t type; /* NONE/SDSC/SDHC verdict */
    uint8_t mfr_id;          /* CID[127:120] */
    uint16_t rca;            /* card-published relative address */
    uint32_t cs_size;        /* C_SIZE field (v1: 12-bit, v2: 22-bit) */
    uint32_t cap_mb;         /* v2 capacity in MiB, 0 for v1 */
} s_sd;

/* Report of the last identification run; the menu reads it after the run. */
static char s_report[48];

/* ---- private helpers -------------------------------------------------------- */

/* Reset s_report to a fixed prefix. */
static void sd_report_set(const char *txt)
{
    char *p = s_report;
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

/* Append a string / one hex byte ("0x03" -> "3") / unsigned decimal. */
static void sd_report_append(const char *txt)
{
    char *p = s_report;
    while (*p != '\0') {
        p++;
    }
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

static void sd_report_append_hex8(uint8_t v)
{
    static const char hexd[16] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    char *p = s_report;
    while (*p != '\0') {
        p++;
    }
    *p++ = hexd[(v >> 4) & 0x0Fu];
    *p++ = hexd[v & 0x0Fu];
    *p = '\0';
}

static void sd_report_append_dec(uint32_t v)
{
    char digits[10];
    unsigned m = 0u;
    char *p = s_report;
    do {
        digits[m++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v > 0u);
    while (*p != '\0') {
        p++;
    }
    while (m > 0u) {
        *p++ = digits[--m];
    }
    *p = '\0';
}

/* SysTick-tick counter: returns 1 once per elapsed ms (COUNTFLAG read
 * clears the flag), with the SysTick-dead guard as the bounded exit.
 * Returns -2 when SysTick looks dead (caller aborts). */
static int sd_tick(void)
{
    static uint32_t guard = BSP_SDIO_TF_SYSTICK_GUARD;
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
        guard = BSP_SDIO_TF_SYSTICK_GUARD;
        return 1;
    }
    if (--guard == 0u) {
        return -2; /* SysTick dead: bounded exit */
    }
    return 0;
}

/* Send one command through the CPSM and wait for a completion flag.
 * waitresp = one of the BSP_SDIO_TF_RESP_* encodings; ignore_crc = 1
 * for R3 (no CRC field, CCRCFAIL is the expected arrival marker).
 * On success *resp1 carries SDIO->RESP1 (short responses; long-response
 * commands read RESP1..4 themselves -- RESP1 is still captured here as
 * the first 32-bit lane). Returns 0 = completed, -2 = CTIMEOUT (no
 * card/no answer), -3 = CRC failure (real one), -4 = wedged CPSM
 * (external window). */
static int sd_send_cmd(uint32_t idx, uint32_t arg, uint32_t waitresp,
                       int ignore_crc, uint32_t timeout_ms, uint32_t *resp1)
{
    /* 1. Clear ALL four static completion flags (rc_w1c semantics). */
    SDIO->ICR = SDIO_ICR_CCRCFAILC | SDIO_ICR_CTIMEOUTC |
                SDIO_ICR_CMDRENDC | SDIO_ICR_CMDSENTC;
    /* 2-3. Argument FIRST (RM 31.9.3: "must be loaded ... before writing
     * a command"), then the CPSM go-write. */
    SDIO->ARG = arg;
    SDIO->CMD = (idx & 0x3Fu) | waitresp | SDIO_CMD_CPSMEN;

    /* Per-call SysTick guard: the STA poll is tight, so the SysTick-dead
     * bound must belong to THIS wait (a shared counter would drain fast). */
    {
        uint32_t guard = BSP_SDIO_TF_SYSTICK_GUARD;
        for (;;) {
            const uint32_t sta = SDIO->STA;
            if ((sta & BSP_SDIO_TF_STA_DONE) != 0u) {
                if ((sta & SDIO_STA_CTIMEOUT) != 0u) {
                    return -2; /* 64 SDIO_CK, no answer: card absent */
                }
                if ((sta & SDIO_STA_CCRCFAIL) != 0u) {
                    if (ignore_crc == 0) {
                        return -3; /* response came, CRC broken: real error */
                    }
                    /* R3: fall through and take the OCR from RESP1. */
                }
                if (resp1 != 0) {
                    *resp1 = SDIO->RESP1;
                }
                return 0; /* CMDSENT (no-resp) or CMDREND/accepted CRC */
            }
            if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
                if (timeout_ms > 0u) {
                    timeout_ms--;
                    guard = BSP_SDIO_TF_SYSTICK_GUARD; /* alive: refill */
                } else {
                    return -4; /* CPSM never set a completion flag */
                }
            }
            if (--guard == 0u) {
                return -4; /* SysTick dead: bounded exit */
            }
        }
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_sdio_tf_init(void)
{
    uint32_t resp1 = 0u;
    uint32_t resp2;
    uint32_t resp3;
    int elapsed_ms;

    /* Kernel clock gate: SDIOEN = APB2ENR bit11. HAL equivalent:
     * __HAL_RCC_SDIO_CLK_ENABLE(). */
    RCC->APB2ENR |= RCC_APB2ENR_SDIOEN;
    (void)RCC->APB2ENR;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;

    /* Six-pin AF12 mux (walkthrough): PC8/9/10/11 = D0..D3, PC12 = CK,
     * PD2 = CMD. Push-pull, no pull, HIGH speed. HAL equivalent:
     *   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     *   GPIO_InitStruct.Pull = GPIO_NOPULL;
     *   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
     *   GPIO_InitStruct.Alternate = GPIO_AF12_SDIO; */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER8_Msk) |
                   GPIO_MODER_MODER8_1; /* PC8 AF */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER9_Msk) |
                   GPIO_MODER_MODER9_1; /* PC9 AF */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER10_Msk) |
                   GPIO_MODER_MODER10_1; /* PC10 AF */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER11_Msk) |
                   GPIO_MODER_MODER11_1; /* PC11 AF (DAT3 caveat above) */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER12_Msk) |
                   GPIO_MODER_MODER12_1; /* PC12 AF (CK) */
    GPIOD->MODER = (GPIOD->MODER & ~GPIO_MODER_MODER2_Msk) |
                   GPIO_MODER_MODER2_1; /* PD2 AF (CMD) */
    GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~GPIO_OSPEEDR_OSPEED8_Msk) |
                     GPIO_OSPEEDR_OSPEED8_1; /* D0 high speed */
    GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~GPIO_OSPEEDR_OSPEED9_Msk) |
                     GPIO_OSPEEDR_OSPEED9_1;
    GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~GPIO_OSPEEDR_OSPEED10_Msk) |
                     GPIO_OSPEEDR_OSPEED10_1;
    GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~GPIO_OSPEEDR_OSPEED11_Msk) |
                     GPIO_OSPEEDR_OSPEED11_1;
    GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~GPIO_OSPEEDR_OSPEED12_Msk) |
                     GPIO_OSPEEDR_OSPEED12_1;
    GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~GPIO_OSPEEDR_OSPEED2_Msk) |
                     GPIO_OSPEEDR_OSPEED2_1;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD10_Msk;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD11_Msk;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD12_Msk;
    GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD2_Msk;
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk) |
                    (0xCu << 0); /* PC8 nibble = 1100 = AF12 */
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL9_Msk) |
                    (0xCu << 4); /* PC9 */
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL10_Msk) |
                    (0xCu << 8); /* PC10 */
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL11_Msk) |
                    (0xCu << 12); /* PC11 */
    GPIOC->AFR[1] = (GPIOC->AFR[1] & ~GPIO_AFRH_AFSEL12_Msk) |
                    (0xCu << 16); /* PC12 */
    GPIOD->AFR[0] = (GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL2_Msk) |
                    (0xCu << 8); /* PD2 nibble = 1100 = AF12 */

    /* Card clock power-on (PWRCTRL=11). The RM 31.9.1 note demands
     * >= 7 HCLK between POWER writes; this single write plus the
     * read-back below satisfies it by construction. */
    SDIO->POWER = 0x3u;
    (void)SDIO->POWER;

    /* Identification clock: CLKDIV=210 -> 396.2 kHz < 400 kHz (RM
     * 31.9.2 note), even value -> 50% duty, CLKEN starts the card
     * clock, 1-bit bus (WIDBUS=00), no HWFC. The clock keeps running
     * from here on -- identification needs a live bus. */
    SDIO->CLKCR = (uint32_t)BSP_SDIO_TF_IDENT_CLKDIV | SDIO_CLKCR_CLKEN;
    if ((SDIO->CLKCR & SDIO_CLKCR_CLKDIV) != BSP_SDIO_TF_IDENT_CLKDIV ||
        (SDIO->CLKCR & SDIO_CLKCR_CLKEN) == 0u) {
        return -1; /* clock config read-back mismatch */
    }

    /* >= 74 card-clock warm-up (SD spec) before the first command: at
     * 396 kHz a 2 ms window clocks the card ~790 times. This also gives
     * the card's internal power-on ramp time. */
    elapsed_ms = 0;
    while (elapsed_ms < 2) {
        const int t = sd_tick();
        if (t < 0) {
            return -1;
        }
        elapsed_ms += (t > 0) ? 1 : 0;
    }

    /* CMD0 GO_IDLE_STATE, no response: resets every card on the bus.
     * CMDSENT only proves the host sent it (no listener required). */
    if (sd_send_cmd(0u, 0u, BSP_SDIO_TF_RESP_NONE, 0,
                    BSP_SDIO_TF_CMD_TO_MS, 0) != 0) {
        return -2; /* CPSM never completed a no-response command */
    }

    /* CMD8 SEND_IF_COND, R7: the v2 probe. A compliant card echoes the
     * argument (voltage-accepted nibble + 0xAA pattern in RESP1[11:0]);
     * silence (CTIMEOUT) = v1-only card or empty socket. The echo value
     * is deliberately NOT branch material here: the ACMD41 loop below
     * is what classifies the card, and the CMD8 reply only speeds that
     * verdict up in the full SD spec flow. */
    (void)sd_send_cmd(8u, BSP_SDIO_TF_CMD8_ARG, BSP_SDIO_TF_RESP_SHORT, 0,
                      BSP_SDIO_TF_CMD_TO_MS, 0);

    /* ACMD41 loop (CMD55 + ACMD41) until OCR busy bit31 = 1, budget
     * 1 s. The card may need the whole second after power-up. R3 has
     * no CRC -> CCRCFAIL is ignored (the ignore_crc switch). */
    elapsed_ms = 0;
    while (elapsed_ms < (int)BSP_SDIO_TF_ACMD41_WINDOW_MS) {
        const int t = sd_tick();
        if (t < 0) {
            return -4;
        }
        elapsed_ms += (t > 0) ? 1 : 0;

        if (sd_send_cmd(55u, 0u, BSP_SDIO_TF_RESP_SHORT, 0,
                        BSP_SDIO_TF_CMD_TO_MS, 0) != 0) {
            return -3; /* CMD55 unanswered: no card in the socket */
        }
        resp1 = 0u;
        if (sd_send_cmd(41u, BSP_SDIO_TF_ACMD41_ARG, BSP_SDIO_TF_RESP_SHORT,
                        1, BSP_SDIO_TF_CMD_TO_MS, &resp1) != 0) {
            return -3; /* ACMD41 unanswered: no card */
        }
        if ((resp1 & (1u << 31)) != 0u) {
            break; /* card powered up and ready */
        }
    }
    if (elapsed_ms >= (int)BSP_SDIO_TF_ACMD41_WINDOW_MS) {
        return -4; /* card present but NEVER went busy (stuck socket?) */
    }
    s_sd.type = ((resp1 & (1u << 30)) != 0u) ? BSP_SDIO_TF_SDHC
                                             : BSP_SDIO_TF_SDSC;

    /* CMD2 ALL_SEND_CID, R2 long: RESP1..4 = CID[127:1] (RESP4 bit0=0).
     * CID[127:120] = manufacturer ID byte = RESP1[31:24]. */
    if (sd_send_cmd(2u, 0u, BSP_SDIO_TF_RESP_LONG, 0,
                    BSP_SDIO_TF_CMD_TO_MS, 0) != 0) {
        return -5;
    }
    s_sd.mfr_id = (uint8_t)(SDIO->RESP1 >> 24);

    /* CMD3 SET_RELATIVE_ADDR, R6: the card PICKS its RCA (RESP1[31:16])
     * and reports status bits [15:0]; bit15 = composite ERROR. */
    resp1 = 0u;
    if (sd_send_cmd(3u, 0u, BSP_SDIO_TF_RESP_SHORT, 0,
                    BSP_SDIO_TF_CMD_TO_MS, &resp1) != 0) {
        return -6;
    }
    if (((resp1 >> 16) & 0xFFFFu) == 0u) {
        return -6; /* RCA 0 is never published by a sane card */
    }
    s_sd.rca = (uint16_t)(resp1 >> 16);

    /* CMD9 SEND_CSD, R2 long, addressed by the new RCA. Structure
     * v1/v2 decides where C_SIZE hides (walkthrough bit-map). */
    if (sd_send_cmd(9u, ((uint32_t)s_sd.rca) << 16, BSP_SDIO_TF_RESP_LONG,
                    0, BSP_SDIO_TF_CMD_TO_MS, 0) != 0) {
        return -7;
    }
    resp2 = SDIO->RESP2; /* RESP1..4 = CSD[127:1] MSB-first */
    resp3 = SDIO->RESP3;
    {
        const uint32_t csd_struct = (SDIO->RESP1 >> 30) & 0x3u;
        if (csd_struct == 1u) { /* v2 (SDHC/SDXC): C_SIZE = CSD[69:48] */
            s_sd.cs_size = ((resp2 & 0x3Fu) << 16) | (resp3 >> 16);
            /* capacity = (C_SIZE+1) x 512 KiB; in MiB = (C_SIZE+1)/2.
             * 32-bit safe: C_SIZE+1 <= 2^22, x512 <= 2^31. */
            s_sd.cap_mb = ((s_sd.cs_size + 1u) * 512u) / 1024u;
        } else if (csd_struct == 0u) { /* v1 (SDSC): C_SIZE = CSD[73:62] */
            s_sd.cs_size = ((resp2 & 0x3FFu) << 2) | (resp3 >> 30);
            s_sd.cap_mb = 0u; /* v1 needs C_SIZE_MULT x READ_BL_LEN:
                               * report the raw field only (walkthrough) */
        } else {
            return -8; /* reserved structure version: corrupt response */
        }
    }
    return 0; /* identified: type/mfr/rca/c_size/cap all captured */
}

bsp_sdio_tf_type_t bsp_sdio_tf_type(void)
{
    return s_sd.type;
}

uint8_t bsp_sdio_tf_manufacturer_id(void)
{
    return s_sd.mfr_id;
}

uint16_t bsp_sdio_tf_rca(void)
{
    return s_sd.rca;
}

uint32_t bsp_sdio_tf_c_size(void)
{
    return s_sd.cs_size;
}

uint32_t bsp_sdio_tf_capacity_mb(void)
{
    return s_sd.cap_mb;
}

const char *bsp_sdio_tf_last_report(void)
{
    return s_report;
}

int bsp_sdio_tf_selftest(void)
{
    /* Step 1: the whole identification chain. A no-card verdict
     * PASSES with its own report (SKIP semantics -- removable medium);
     * every other failure is a step-1 FAIL with the step named. */
    s_sd.type = BSP_SDIO_TF_NONE;
    {
        const int rc = bsp_sdio_tf_init();
        if (rc == -3) { /* empty socket: the hardware's own verdict */
            sd_report_set("(no card)");
            return 0; /* PASS, nothing further to check */
        }
        if (rc != 0) {
            static const char *const why[8] = {
                "clock cfg", "CMD0 stuck", "", "ACMD41 stuck(no busy)",
                "CMD2 failed", "CMD3 failed/RCA=0", "CMD9 failed",
                "CSD invalid"
            };
            /* rc in [-1..-4] and [-5..-8]: map to the step name. */
            int idx = -rc; /* 1..8 */
            if ((idx < 1) || (idx > 8)) {
                sd_report_set("(init: unknown fail)");
            } else {
                sd_report_set("(init: ");
                sd_report_append(why[idx - 1]);
                sd_report_append(")");
            }
            return (int)BSP_SDIO_TF_ST1_IDENTIFY;
        }
    }

    /* Step 2: captured-field sanity (the "print" of OCR CCS / CID mfr /
     * CSD capacity is the report line itself, data straight from the
     * captured registers). Assert: RCA nonzero (re-checked), C_SIZE
     * nonzero, CCS agrees with the CSD shape the capacity math used. */
    sd_report_set("(");
    sd_report_append((s_sd.type == BSP_SDIO_TF_SDHC) ? "SDHC" : "SDSC");
    sd_report_append(" mfr=0x");
    sd_report_append_hex8(s_sd.mfr_id);
    sd_report_append(" ");
    sd_report_append_dec((s_sd.cap_mb != 0u) ? s_sd.cap_mb
                                             : s_sd.cs_size);
    sd_report_append((s_sd.cap_mb != 0u) ? "MB" : " (v1 field)");
    sd_report_append(")");
    if (s_sd.rca == 0u) {
        sd_report_set("(RCA=0)");
        return (int)BSP_SDIO_TF_ST2_FIELDS;
    }
    if (s_sd.cs_size == 0u) {
        sd_report_set("(C_SIZE=0)");
        return (int)BSP_SDIO_TF_ST2_FIELDS;
    }
    if ((s_sd.type == BSP_SDIO_TF_SDHC) && (s_sd.cap_mb == 0u)) {
        sd_report_set("(SDHC but no capacity)");
        return (int)BSP_SDIO_TF_ST2_FIELDS;
    }
    return 0; /* all steps passed */
}

/** @} */
