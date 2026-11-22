/**
 ******************************************************************************
 * @file    bsp_spi_flash.c
 * @brief   SPI1 register-level host + W25Q128 16 MiB NOR flash (EP12
 *          worked example) -- teaching module with self-test.
 ******************************************************************************
 * @addtogroup BSP_SPI_FLASH
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | SPI1 in IOC       | NOT PRESENT                          | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | PA5/PA6/PA7       | GPIO_Analog, labels FLASH_SCK/       | pin-map
 * | in IOC            | FLASH_MISO/FLASH_MOSI (placeholders; | placeholders;
 * |                   | the AF5 mux is written HERE)         | anti-collision
 * | PC13 in IOC       | GPIO_Output, label FLASH_CS, initial | CS idle HIGH =
 * |                   | HIGH (MX_GPIO_Init de-asserts CS)    | flash dormant;
 * |                   |                                      | owned HERE after
 * | Mode              | Full-Duplex Master                   | W25Q128 quad not
 * |                   |                                      | wired on this
 * |                   |                                      | board (D2/D3 are
 * |                   |                                      | WP/HOLD)
 * | Hardware NSS      | DISABLED (SSM=1, SSI=1)              | PA4 = the real
 * |                   |                                      | NSS pin -- it is
 * |                   |                                      | KEY3 here, so CS
 * |                   |                                      | is a soft GPIO
 * | Frame / format    | 8-bit, MSB first, Motorola           | W25Q128 default
 * | CPOL / CPHA       | 0 / 0 (SPI mode 0)                   | W25Q supports 0
 * |                   |                                      | and 3; lab = 0
 * | Prescaler BR      | /16 -> 5.25 MHz                      | conservative:
 * |                   |                                      | 84M/8=10.5 MHz
 * |                   |                                      | would work, /16
 * |                   |                                      | keeps all 0x03
 * |                   |                                      | reads far under
 * |                   |                                      | the 50 MHz part
 * |                   |                                      | limit
 * | NVIC              | SPI1_IRQn off                        | polling lesson
 * == REGISTER WALKTHROUGH (RM0090 ch.28, SPI v2.1) =========================
 * Kernel clock: RCC->APB2ENR bit12 (SPI1EN). SPI1 lives on APB2, so the
 *   baud-rate divider divides PCLK2 = 84 MHz DIRECTLY (SPI2/3 would sit
 *   on APB1/42 MHz -- the datasheet "SPI1 is the fast one" fact).
 * Pin mux: PA5 -> AF5 (SPI1_SCK), PA6 -> AF5 (SPI1_MISO), PA7 -> AF5
 *   (SPI1_MOSI): MODER [11:10]/[13:12]/[15:14] = 10, AFRL nibbles
 *   5/6/7 = 0101. SCK/MOSI are host-driven push-pull (OTYPER=0, the
 *   reset default); MISO is INPUT (MODER=00) and FLOATING (PUPDR=00) --
 *   the W25Q128 DO pin drives it, a pull would only argue with it.
 * CS = PC13: plain GPIO OUTPUT, never an SPI peripheral pin (the RM's
 *   own advice, 28.3.3 note: "When a master is communicating with SPI
 *   slaves which need to be de-selected between transmissions, ... a
 *   GPIO must be used and toggled by software"). Idle HIGH (deselected),
 *   driven low for the duration of a command frame.
 * CR1 -- the whole mode lives in ONE register, set while SPE=0:
 *   bit0  CPHA = 0: sample on FIRST (leading) clock edge.
 *   bit1  CPOL = 0: clock idles LOW. Together = SPI mode 0.
 *   bit2  MSTR = 1: master (SCK is an OUTPUT).
 *   bit5:3 BR   = 100 (4): f_SCK = PCLK2 / 2^BR = 84 MHz/2^4 = 5.25 MHz.
 *         (The divider is exponential: BR=3 would be /8=10.5 MHz, also
 *         legal for the W25Q128 -- the lab deliberately picks /16.)
 *   bit6  SPE  = 1: enable, written LAST.
 *   bit7  LSBFIRST = 0: MSB first (W25Q128 commands/addresses/IDs are
 *         big-endian bit streams).
 *   bit8  SSI = 1 + bit9 SSM = 1: NSS in software mode. SSI is the
 *         "virtual NSS pin" the master samples in place of PA4; SSI=1
 *         keeps the master alive (SSI=0 would drop it to slave as soon
 *         as SPE sets -- the classic first-day SPI trap when the real
 *         NSS pin is unavailable).
 *   bit11 DFF = 0: 8-bit frames.
 * SR -- the polling flags (RM 28.3.7):
 *   TXE (bit1): TX buffer empty -> safe to load the next DR byte.
 *   RXNE(bit0): a fully received byte sits in DR -> read it (the read
 *               clears the flag).
 *   BSY (bit7): transfer still in flight. THE CS-TIMING FLAG: RXNE of
 *               the last byte only proves the last SAMPLING edge
 *               happened; BSY=0 is what proves the whole frame left the
 *               shift register. Raise CS only after BSY reads 0 --
 *               earlier and the tail bits of the command are truncated
 *               mid-air.
 * CS TIMING DISCIPLINE (the EP12 lesson this module teaches):
 *     CS LOW -> command byte(s) -> address bytes (MSB first) -> data
 *     bytes -> wait BSY=0 -> CS HIGH.
 *   W25Q128 samples CS-LOW as the frame opener; every byte in between
 *   is one 8-clock exchange (the host "reads" whatever MISO carries --
 *   for reads these are the payload, for commands they are don't-care).
 *   Releasing CS too early ABORTS the command mid-frame (0x9F would
 *   deliver a truncated ID; 0x02 with a truncated address writes
 *   garbage); leaving CS low too long runs command frames together --
 *   the device interprets whatever byte arrives after the frame as the
 *   NEXT command. Both directions are bugs; the wait_busy + CS bookends
 *   here are the contract.
 * W25Q128 COMMAND LAYER (W25Q128JV datasheet -- 16 MiB = 0x1000000):
 *   0x9F JEDEC ID: returns EF (Winbond manufacturer), 40 (SPI capacity
 *        family), 18 (128 Mbit / 16 MiB). No WREN, no busy -- always
 *        legal, the first probe any driver sends.
 *   0xAB RELEASE POWER-DOWN: the ONLY instruction a deep-power-down
 *        (0xB9) part still hears -- everything else, 0x9F included,
 *        is ignored until the release. With 3 dummy bytes it doubles
 *        as a device-ID read (0x17). init() sends the bare AB + a
 *        1 ms tRES wait once, so a sleeping part is awake before the
 *        first real command (datasheet sec. 8.2.24).
 *   0x90 MANUFACTURER/DEVICE ID: address 000000h, then EFh + 17h.
 *        A different decode path than 0x9F -- the selftest's fifth
 *        attempt, separating "chip dead" from "9F path broken".
 *   0x06 WREN: sets SR1 WEL (bit1). WEL SELF-CLEARS at the end of a
 *        program/erase cycle, so every write operation is the pair
 *        WREN + command. (0x04 WRDI clears it early -- not used here.)
 *   0x05 RDSR1: returns SR1. BUSY (bit0) = 1 while an erase/program
 *        cycle runs internally (typ 45 ms for a 4 KiB sector erase,
 *        0.6 ms for a page program). WEL (bit1) echoes the latch.
 *        Polling BUSY is HOW the driver waits: the device ignores every
 *        command except RDSR1 (and 0x3B...) while BUSY=1.
 *   0x20 4 KiB SECTOR ERASE: 24-bit address, sector = address & ~0xFFF.
 *        Erased NOR reads as 0xFF (programming can only clear bits --
 *        the reason erase precedes every program). Typ 45 ms, max
 *        400 ms -> timeout window 500 ms.
 *   0x02 PAGE PROGRAM: up to 256 bytes, ALL INSIDE ONE PAGE (the upper
 *        16 address bits + A8 may not change mid-frame). Crossing the
 *        boundary WRAPS to the page start -- bytes land on top of what
 *        you wrote first, silently. The API enforces alignment + size
 *        instead of trusting callers. Typ 0.6 ms, max 3 ms -> 50 ms.
 *   0x03 READ: 24-bit address, then clocks data out forever (until CS
 *        HIGH). 50 MHz max -- 5.25 MHz here, no wait states needed.
 * PARTITION RED LINE (board manual "storage space partition": first
 *   10 MB free, 6 MB font area): 0x00A00000..0x00FFFFFF holds
 *   factory-burned HZK glyph libraries. Erase/program APIs check the
 *   whole target range against BSP_SPI_FLASH_FREE_LIMIT BEFORE any bus
 *   traffic (no WREN slips out) and refuse with -3.
 * == HOW TO VERIFY =========================================================
 * menu> spi_flash
 *   "PASS spi_flash 9F:EF4018 page ok" -- JEDEC ID matched the datasheet
 *   (verified 2026-11-21 on real hardware) and a 16-byte pattern survived
 *   erase -> program -> read at 0x00010000 (free area). The selftest
 *   erases that sector every run, so re-runs are idempotent.
 * Scope check (optional): PA5 shows 8-clock bursts per byte (~1.52 us
 *   per byte at 5.25 MHz), CS (PC13) framing each command.
 * Failure semantics: the step-2 FAIL report lists EVERY attempt's raw
 *   bytes (engine 9F@5.25M, slow 9F@328k, mode-3 9F, 0x90 mfr/dev, and
 *   the soft bit-bang block 9F-fast/9F-repeat/9F-slow + 0xAB id +
 *   RDSR1) plus the pin/register dump -- e.g.
 *   "9F:000000 slow:000000 m3:000000 90:0000 soft:XXXXXXXX/...
 *    ab:XX rsr:XX cs:1 csh:1 p6:10 pa:22 afr:55 sr:02 cr1:64".
 *   soft bytes that are IDENTICAL within a run but change run-to-run
 *   (8B8B8B, then C5C5C5, then 171717...) are floating-line noise: a
 *   real 9F answer is THREE DIFFERENT bytes (EF 40 17/18) every run.
 *   p6:10 = PA6 floats under both internal pulls (MISO net open or the
 *   chip unpowered); csh:1 = CS can be driven high (net not shorted).
 *   Interpretation: identical bytes across the soft repeats = a REAL
 *   deterministic answer (chip alive on the slow link); changing bytes
 *   = floating-line noise. ab:17 = the chip answered its own 0xAB ID
 *   (definitive chip-alive proof). All-zero everywhere = the part never
 *   drove MISO (DPD ruled out by the 0xAB preamble; suspect power/CS/
 *   HOLD/WP wiring or the die); EF-anything-else = wrong silicon. If a
 *   fallback engine attempt answers ("!slow" / "!mode3"), the selftest
 *   ADOPTS that datapath and step 3 runs on it.
 */
#include "bsp_spi_flash.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* JEDEC ID of the soldered part (W25Q128JV): EF=Winbond, 40=SPI 3V3
 * family, 18=128 Mbit. The selftest asserts all three bytes. */
#define BSP_SPI_FLASH_JEDEC_MFR 0xEFu
#define BSP_SPI_FLASH_JEDEC_TYPE 0x40u
#define BSP_SPI_FLASH_JEDEC_SIZE 0x18u

/* W25Q128 instruction codes (datasheet table). */
#define BSP_SPI_FLASH_CMD_WREN 0x06u
#define BSP_SPI_FLASH_CMD_RDSR1 0x05u
#define BSP_SPI_FLASH_CMD_SECTOR_ERASE 0x20u
#define BSP_SPI_FLASH_CMD_PAGE_PROGRAM 0x02u
#define BSP_SPI_FLASH_CMD_READ 0x03u
#define BSP_SPI_FLASH_CMD_JEDEC 0x9Fu
#define BSP_SPI_FLASH_CMD_MFR_DEV_ID 0x90u  /* manufacturer + device ID */
#define BSP_SPI_FLASH_CMD_RELEASE_PD 0xABu  /* release deep power-down */

/* SR1 bits of interest: BUSY bit0, WEL bit1. */
#define BSP_SPI_FLASH_SR1_BUSY 0x01u

/* Timeout windows (SysTick COUNTFLAG, 1 ms). Erase typ 45 max 400 ms ->
 * 500 ms; program typ 0.6 max 3 ms -> 50 ms; single-byte exchange/flag
 * waits 10 ms (~100x headroom). */
#define BSP_SPI_FLASH_BUSY_ERASE_TO_MS 500u
#define BSP_SPI_FLASH_BUSY_PROG_TO_MS 50u
#define BSP_SPI_FLASH_FLAG_TO_MS 10u

/* Self-test exercise target: first 64 KiB block of the FREE area (4 KiB
 * aligned), ~1 MiB under the font red line. */
#define BSP_SPI_FLASH_SELFTEST_ADDR 0x00010000u
#define BSP_SPI_FLASH_SELFTEST_LEN 16u

/* Self-test step numbers (the value bsp_spi_flash_selftest returns). */
#define BSP_SPI_FLASH_ST1_CONFIG 1u /* CR1/SR mode read-back */
#define BSP_SPI_FLASH_ST2_JEDEC  2u /* 0x9F == EF 40 18 */
#define BSP_SPI_FLASH_ST3_PAGE   3u /* erase -> FF -> program -> compare */

/* Spin cap between SysTick wraps: SysTick-dead guard (spec #5). */
#define BSP_SPI_FLASH_SYSTICK_GUARD 2000000u

/* ---- module state ----------------------------------------------------------- */

/* Report of the last selftest run; the menu reads it after the run.
 * Sized for the FULL diagnostic chain (per-attempt IDs + pin dumps);
 * every append below is bounds-checked -- the earlier 32-byte buffer
 * with unchecked appends overflowed static memory on the fail path. */
#define SF_REPORT_MAX 192u
static char s_report[SF_REPORT_MAX];

/* ---- private helpers -------------------------------------------------------- */

/* Current used length of s_report. */
static unsigned sf_report_len(void)
{
    unsigned n = 0u;
    while ((n < (SF_REPORT_MAX - 1u)) && (s_report[n] != '\0')) {
        n++;
    }
    return n;
}

/* Overwrite s_report with a fixed string (bounded, no libc). */
static void sf_set_report(const char *txt)
{
    unsigned i = 0u;
    while ((*txt != '\0') && (i < (SF_REPORT_MAX - 1u))) {
        s_report[i] = *txt++;
        i++;
    }
    s_report[i] = '\0';
}

/* Append a string to s_report (bounded, no libc). */
static void sf_report_append(const char *txt)
{
    unsigned i = sf_report_len();
    while ((*txt != '\0') && (i < (SF_REPORT_MAX - 1u))) {
        s_report[i] = *txt++;
        i++;
    }
    s_report[i] = '\0';
}

/* Append one byte as two hex digits ("0xEF" -> "EF"; bounded). */
static void sf_report_hex_byte(uint8_t v)
{
    static const char hexd[16] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    unsigned i = sf_report_len();
    if (i < (SF_REPORT_MAX - 2u)) {
        s_report[i++] = hexd[(v >> 4) & 0x0Fu];
        s_report[i++] = hexd[v & 0x0Fu];
        s_report[i] = '\0';
    }
}

/* CS bookends (walkthrough CS discipline): PC13 BSRR, output, idle HIGH. */
static void sf_cs_low(void)
{
    /* ACTIVE-LOW chip select: BSRR[31:16] RESETs the bit -> PC13 LOW.
     * (Fixed: the original pair drove PC13 HIGH here, leaving the
     * W25Q deselected -- JEDEC read 000000 on real hardware.) */
    GPIOC->BSRR = (1u << (13 + 16));
}

static void sf_cs_high(void)
{
    GPIOC->BSRR = (1u << 13); /* BSRR[15:0] SET -> PC13 HIGH (deselected) */
}

/* Wait until (SPI1->SR & mask) == want or timeout_ms elapses. Returns
 * 0 = matched, -1 = timeout, -2 = SysTick dead (bounded exit). */
static int sf_wait_sr(uint32_t mask, uint32_t want, uint32_t timeout_ms)
{
    uint32_t guard = BSP_SPI_FLASH_SYSTICK_GUARD;
    for (;;) {
        if ((SPI1->SR & mask) == want) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
                guard = BSP_SPI_FLASH_SYSTICK_GUARD; /* alive: refill */
            } else {
                return -1; /* window exhausted */
            }
        }
        if (--guard == 0u) {
            return -2; /* SysTick dead: bounded exit */
        }
    }
}

/* One full CS-framed frame with N data exchanges after 'cmd' (address
 * bytes included in the data[] tail by the callers): CS low, command,
 * the exchanges (rx[] optionally captured), BSY-wait, CS high. This is
 * the CS-vs-BSY discipline in ONE place so every command gets it right. */
static int sf_frame(uint8_t cmd, const uint8_t *tx, uint8_t *rx,
                    uint32_t n)
{
    uint32_t i;

    sf_cs_low();
    /* ELECTRICAL PROOF: IDR reads the PIN, not the register -- if PC13
     * is hijacked (RTC/tamper) or shorted, this catches it. */
    if (((GPIOC->IDR >> 13) & 1u) != 0u) {
        sf_cs_high(); /* pin never went LOW: hijacked or shorted */
        return -3;    /* caller maps -3 to a distinct report suffix */
    }
    (void)bsp_spi_flash_xfer8(cmd);
    for (i = 0u; i < n; i++) {
        const uint8_t got = bsp_spi_flash_xfer8((tx != 0) ? tx[i] : 0x00u);
        if (rx != 0) {
            rx[i] = got;
        }
        if (sf_wait_sr(SPI_SR_BSY, 0u, BSP_SPI_FLASH_FLAG_TO_MS) != 0) {
            sf_cs_high(); /* abort the frame before reporting failure */
            return -1;
        }
    }
    /* The walkthrough's cardinal rule: BSY=0 BEFORE the CS rising edge.
     * (Checked per byte above so a stalled SCK is caught mid-frame;
     * this final wait covers the no-data-command case too.) */
    if (sf_wait_sr(SPI_SR_BSY, 0u, BSP_SPI_FLASH_FLAG_TO_MS) != 0) {
        sf_cs_high();
        return -1;
    }
    sf_cs_high();
    return 0;
}

/* Poll RDSR1 until BUSY clears or the per-op timeout expires (the ONLY
 * way to wait on a W25Q128 -- the part ignores commands while BUSY=1).
 * Returns 0 = idle again, -1 = still busy at the window end. */
static int sf_wait_busy(uint32_t timeout_ms)
{
    uint8_t sr1 = 0u;

    for (;;) {
        /* RDSR1 frame: command + one dummy exchange clocks SR1 out. */
        if (sf_frame(BSP_SPI_FLASH_CMD_RDSR1, 0, &sr1, 1u) != 0) {
            return -1;
        }
        if ((sr1 & BSP_SPI_FLASH_SR1_BUSY) == 0u) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
            } else {
                return -1; /* still programming/erasing at window end */
            }
        }
    }
}

/* Partition guard for EVERY mutating operation: the whole byte range
 * [addr, addr+len) must sit inside the free area (< 0x00A00000). Called
 * BEFORE any bus traffic (a WREN must not leak out on a refused op). */
static int sf_range_in_free(uint32_t addr, uint32_t len)
{
    if (len == 0u) {
        return -1;
    }
    if ((addr >= BSP_SPI_FLASH_FREE_LIMIT) ||
        ((addr + len) > BSP_SPI_FLASH_FREE_LIMIT) ||
        ((addr + len) > BSP_SPI_FLASH_TOTAL_SIZE)) {
        return -3; /* font red line (or out of the 16 MiB array) */
    }
    return 0;
}

/* 1 ms delay via the HAL SysTick COUNTFLAG (same bounded pattern the
 * other bsp modules use; no libc). Shared by the DPD-release wait and
 * the scope/pin-probe demos. */
static void sf_delay_ms(unsigned ms)
{
    while (ms-- > 0u) {
        uint32_t guard = BSP_SPI_FLASH_SYSTICK_GUARD;
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u) {
            if (--guard == 0u) {
                return; /* SysTick dead: bounded exit (frames stop) */
            }
        }
    }
}

/* Wake the part from DEEP POWER-DOWN (W25Q128JV datasheet sec. 8.2.24):
 * after 0xB9 the device ignores EVERY instruction except 0xAB -- a DPD
 * part answers 0x9F with a silent MISO, byte-for-byte identical to a
 * broken chip. 0xAB with CS returned HIGH releases DPD after tRES
 * (~3 us typ, well under the 1 ms wait). A fresh part is never in DPD,
 * but a hand-me-down, a previously mis-driven board, or a factory test
 * pattern can leave it there; one AB frame up front costs 8 clocks and
 * removes "chip asleep" from the suspect list for good. */
static void sf_release_power_down(void)
{
    (void)sf_frame(BSP_SPI_FLASH_CMD_RELEASE_PD, 0, 0, 0);
    sf_delay_ms(1u);
}

/* Read Manufacturer/Device ID via 0x90 (datasheet sec. 8.2.23): command
 * + 3 address bytes (000000h), then EFh (Winbond) + 17h (W25Q128 device
 * ID), repeating. A DIFFERENT decode path inside the chip than 0x9F --
 * if 0x9F is silent but 0x90 answers, the chip is alive and the 9F path
 * (or the driver's 9F framing) is the suspect, not the silicon. */
static int sf_read_mfr_dev_id(uint8_t id[2])
{
    uint32_t i;

    sf_cs_low();
    if (((GPIOC->IDR >> 13) & 1u) != 0u) {
        sf_cs_high();
        return -3; /* CS never went low: hijacked or shorted */
    }
    (void)bsp_spi_flash_xfer8(BSP_SPI_FLASH_CMD_MFR_DEV_ID);
    for (i = 0u; i < 3u; i++) {
        (void)bsp_spi_flash_xfer8(0x00u); /* address 000000h */
    }
    for (i = 0u; i < 2u; i++) {
        id[i] = bsp_spi_flash_xfer8(0x00u); /* MFR, then DEV ID */
    }
    if (sf_wait_sr(SPI_SR_BSY, 0u, BSP_SPI_FLASH_FLAG_TO_MS) != 0) {
        sf_cs_high();
        return -1;
    }
    sf_cs_high();
    return 0;
}

/* Re-arm the engine with a different SPI mode / baud divider (SPE must
 * drop for a CR1 write, RM 28.3.3). W25Q128 explicitly supports SPI
 * modes 0 AND 3 (datasheet AC table: SCK idle low or high); if mode 0
 * is silent, mode 3 proves whether the failure is sample-edge related.
 * br: 4 = /16 (5.25 MHz), 7 = /256 (328 kHz). */
static void sf_set_spi_mode(uint32_t cpol, uint32_t cpha, uint32_t br)
{
    uint32_t cr1 = SPI1->CR1 & ~SPI_CR1_SPE;

    cr1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR);
    if (cpol != 0u) {
        cr1 |= SPI_CR1_CPOL;
    }
    if (cpha != 0u) {
        cr1 |= SPI_CR1_CPHA;
    }
    cr1 |= (br << SPI_CR1_BR_Pos);
    SPI1->CR1 = cr1;
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* Idle-level pull test on PA6 (MISO). With CS high the W25Q's DO is
 * Hi-Z, so the pin shows either the board's external tie (level holds
 * against BOTH internal pulls = the MISO net physically exists) or it
 * follows our pull (true floating pin = the DO trace is open, or the
 * chip is unpowered -- a dead chip's Hi-Z DO looks the same).
 * Returns bit1 = level with pull-up, bit0 = level with pull-down:
 *   10 = follows both pulls -> floating;  11/00 = externally tied. */
static uint8_t sf_miso_pull_probe(void)
{
    uint8_t up;
    uint8_t dn;
    const uint32_t pupd_save = GPIOA->PUPDR & GPIO_PUPDR_PUPD6_Msk;

    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD6_Msk) |
                   GPIO_PUPDR_PUPD6_0; /* 01: pull-up */
    sf_delay_ms(2);
    up = (uint8_t)((GPIOA->IDR >> 6) & 1u);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD6_Msk) |
                   GPIO_PUPDR_PUPD6_1; /* 10: pull-down */
    sf_delay_ms(2);
    dn = (uint8_t)((GPIOA->IDR >> 6) & 1u);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD6_Msk) | pupd_save;
    return (uint8_t)((up << 1) | dn);
}

/* ---- public API ------------------------------------------------------------ */

int bsp_spi_flash_init(void)
{
    /* Kernel clock gate: SPI1EN = APB2ENR bit12. HAL equivalent:
     * __HAL_RCC_SPI1_CLK_ENABLE(). */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR; /* read-back stalls until the clock is live */

    /* GPIOA for PA5/PA6/PA7 (SCK/MISO/MOSI), GPIOC for the CS pin. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    /* Pin mux PA5/PA6/PA7 -> AF5 (walkthrough). HAL equivalent:
     *   GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
     *   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;   (SCK/MOSI push-pull)
     *   GPIO_InitStruct.Pull = GPIO_NOPULL;        (MISO floats)
     *   GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
     *   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
     * MISO (PA6) MUST BE MODER=10 (AF) TOO -- RM0090 sec. 8.3.2:
     * "For other peripherals: Configure the desired I/O as an alternate
     * function in the GPIOx_MODER register ... connect the I/O to the
     * desired AFx". A plain input (00) does NOT route the pad to
     * SPI1_MISO; the engine reads a constant 0x00 no matter what the
     * pin carries (board finding: PA6 idled HIGH thanks to the board's
     * DO pull-up, yet every engine ID read returned 000000 -- the
     * "pa:22" register dump = PA5 AF, PA6 INPUT, PA7 AF gave it away).
     * PUPDR=00 keeps the pin floating (the chip's DO drives it). */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER5_Msk) |
                   GPIO_MODER_MODER5_1; /* PA5 [11:10]=10 AF (SCK) */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER6_Msk) |
                   GPIO_MODER_MODER6_1; /* PA6 [13:12]=10 AF (MISO) */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER7_Msk) |
                   GPIO_MODER_MODER7_1; /* PA7 [15:14]=10 AF (MOSI) */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5_Msk; /* SCK no pull */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6_Msk; /* MISO floating (walkthrough) */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD7_Msk; /* MOSI no pull */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5; /* SCK push-pull */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_7; /* MOSI push-pull */
    /* OSPEEDR VERY-HIGH (11): SCK runs 5.25 MHz -- the reset default
     * OSPEEDR=00 (LOW, 2 MHz class) slew-limits the edges far beyond
     * spec and the W25Q cannot sample its CLK (board finding: JEDEC
     * read 000000 until this was set; HAL/CubeMX SPI init always asks
     * GPIO_SPEED_FREQ_HIGH -- the register-form port missed it). */
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR &
                      ~(GPIO_OSPEEDR_OSPEED5_Msk | GPIO_OSPEEDR_OSPEED7_Msk)) |
                     GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED7; /* 11 */
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6_Msk); /* MISO is an input */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL5_Msk) |
                    GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2; /* 0101=AF5 */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk) |
                    GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL6_2; /* 0101=AF5 */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL7_Msk) |
                    GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_2; /* 0101=AF5 */

    /* CS pin PC13: output, push-pull, de-asserted HIGH before the first
     * frame (the IOC already asks MX_GPIO_Init for exactly this state --
     * init re-asserts ownership per the header note). */
    /* MODER13 = 01 = GENERAL-PURPOSE OUTPUT. The earlier code wrote
     * GPIO_MODER_MODER13_1 (the field's bit1) = 0b10 = ALTERNATE
     * FUNCTION -- a comment-vs-code trap: PC13 sat in AF0, never
     * followed ODR, IDR read back 0 even while ODR=1 (the "csh:00"
     * report field), and the W25Q was never actually selected by the
     * GPIO. _0 is the field's bit0: 0b01. */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER13_Msk) |
                   GPIO_MODER_MODER13_0; /* [27:26]=01 output */
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_13; /* push-pull */
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD13_Msk; /* no pull needed */
    sf_cs_high();

    /* Mode 0, master, /16, 8-bit, MSB first, software NSS (walkthrough
     * CR1 table). SPE=0 during the whole write (config-while-disabled,
     * RM 28.3.3 procedure), then enabled last. HAL equivalent:
     *   hspi.Init.Mode = SPI_MODE_MASTER;
     *   hspi.Init.Direction = SPI_DIRECTION_2LINES;
     *   hspi.Init.DataSize = SPI_DATASIZE_8BIT;
     *   hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
     *   hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
     *   hspi.Init.NSS = SPI_NSS_SOFT;
     *   hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
     *   hspi.Init.FirstBit = SPI_FIRSTBIT_MSB; */
    SPI1->CR1 = SPI_CR1_MSTR |       /* bit2: master */
                (4u << SPI_CR1_BR_Pos) | /* BR=100: 84 MHz/16 = 5.25 MHz */
                SPI_CR1_SSM |        /* bit9: NSS by software ... */
                SPI_CR1_SSI;         /* bit8: ... pinned HIGH (master) */
    /* CPOL=0/CPHA=0/DFF=0/LSBFIRST=0 are all reset-value zeros -- left
     * unmentioned in the write above ON PURPOSE: an OR-set would silently
     * inherit a previous run's bits, an assignment documents each one. */
    SPI1->CR2 = 0x0u; /* TXEIE/RXNEIE off: polling lesson */
    SPI1->CR1 |= SPI_CR1_SPE; /* enable LAST */

    /* Read-back proof: master + enabled + /16 + soft NSS, no stray bits. */
    if ((SPI1->CR1 & (SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM |
                      SPI_CR1_SSI)) !=
            (SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI) ||
        (SPI1->CR1 & SPI_CR1_BR) != (4u << SPI_CR1_BR_Pos) ||
        (SPI1->CR1 & (SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_DFF |
                      SPI_CR1_LSBFIRST)) != 0u) {
        return -1;
    }
    /* Pin-mode proof -- the two board-founding bugs must never pass
     * silently again: CS is a plain GPIO OUTPUT (01), and SCK/MISO/
     * MOSI are all routed as AF (10; a plain input on MISO leaves the
     * SPI engine reading a hardwired 0x00). */
    if (((GPIOC->MODER & GPIO_MODER_MODER13_Msk) != GPIO_MODER_MODER13_0) ||
        ((GPIOA->MODER & (GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER6_Msk |
                          GPIO_MODER_MODER7_Msk)) !=
         (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 |
          GPIO_MODER_MODER7_1))) {
        return -2;
    }

    /* Datasheet preamble: if the part was left in deep power-down, this
     * AB frame is the ONLY thing it will hear -- do it once per init,
     * before any caller sends a real command. */
    sf_release_power_down();
    return 0;
}

uint8_t bsp_spi_flash_xfer8(uint8_t tx)
{
    /* Full-duplex discipline: TXE says the buffer is free (RM 28.3.3
     * transmit sequence), loading DR starts the 8 clocks immediately;
     * RXNE says a received byte is parked in DR; reading DR clears it. */
    if (sf_wait_sr(SPI_SR_TXE, SPI_SR_TXE, BSP_SPI_FLASH_FLAG_TO_MS) != 0) {
        return 0u; /* TX never drained: caller's frame wait will report */
    }
    *(volatile uint8_t *)&SPI1->DR = tx; /* 8-bit frame: BYTE access */
    if (sf_wait_sr(SPI_SR_RXNE, SPI_SR_RXNE, BSP_SPI_FLASH_FLAG_TO_MS) != 0) {
        return 0u;
    }
    return *(volatile uint8_t *)&SPI1->DR;
}

int bsp_spi_flash_jedec_id(uint8_t id[3])
{
    if (id == 0) {
        return -1;
    }
    /* 0x9F + three dummy exchanges clock the ID out (walkthrough). */
    return sf_frame(BSP_SPI_FLASH_CMD_JEDEC, 0, id, 3u);
}

int bsp_spi_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4];
    uint32_t i;

    if ((buf == 0) || (len == 0u) || (addr >= BSP_SPI_FLASH_TOTAL_SIZE) ||
        ((addr + len) > BSP_SPI_FLASH_TOTAL_SIZE)) {
        return -1;
    }
    /* 0x03 + big-endian 24-bit address (walkthrough: MSB first). */
    cmd[0] = BSP_SPI_FLASH_CMD_READ;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)(addr >> 0);
    sf_cs_low();
    for (i = 0u; i < 4u; i++) {
        (void)bsp_spi_flash_xfer8(cmd[i]);
    }
    for (i = 0u; i < len; i++) {
        buf[i] = bsp_spi_flash_xfer8(0x00u); /* dummy = read clocks */
    }
    if (sf_wait_sr(SPI_SR_BSY, 0u, BSP_SPI_FLASH_FLAG_TO_MS) != 0) {
        sf_cs_high();
        return -2; /* SCK stalled mid-read */
    }
    sf_cs_high(); /* ends the 0x03 stream (device stops driving MISO) */
    return 0;
}

int bsp_spi_flash_sector_erase4k(uint32_t addr)
{
    int rc;

    /* Sector granularity: 0x20 erases the 4 KiB sector CONTAINING the
     * address -- a non-aligned address here is a caller bug we catch. */
    if ((addr >= BSP_SPI_FLASH_TOTAL_SIZE) ||
        ((addr % BSP_SPI_FLASH_SECTOR_SIZE) != 0u)) {
        return -1;
    }
    rc = sf_range_in_free(addr, BSP_SPI_FLASH_SECTOR_SIZE);
    if (rc != 0) {
        return rc; /* -3 = font red line, BEFORE any bus traffic */
    }

    /* WREN + 0x20 frame, then poll BUSY (typ 45 ms / max 400 ms). */
    if (sf_frame(BSP_SPI_FLASH_CMD_WREN, 0, 0, 0) != 0) {
        return -2;
    }
    {
        const uint8_t a2 = (uint8_t)(addr >> 16);
        const uint8_t a1 = (uint8_t)(addr >> 8);
        const uint8_t a0 = (uint8_t)(addr >> 0);
        const uint8_t arg[3] = { a2, a1, a0 };
        if (sf_frame(BSP_SPI_FLASH_CMD_SECTOR_ERASE, arg, 0, 3u) != 0) {
            return -2;
        }
    }
    return sf_wait_busy(BSP_SPI_FLASH_BUSY_ERASE_TO_MS);
}

int bsp_spi_flash_page_program(uint32_t addr, const uint8_t *buf,
                               uint32_t len)
{
    uint8_t arg[3];
    uint32_t i;
    int rc;

    /* Page contract (walkthrough): aligned start + len <= page size +
     * whole range inside the ONE page => (addr | (len-1)) keeps the
     * upper bits, i.e. no boundary is crossed. NOR semantics: program
     * can only clear 1-bits, so the page must be erased beforehand. */
    if ((buf == 0) || (len == 0u) || (len > BSP_SPI_FLASH_PAGE_SIZE) ||
        ((addr % BSP_SPI_FLASH_PAGE_SIZE) != 0u)) {
        return -1;
    }
    rc = sf_range_in_free(addr, len);
    if (rc != 0) {
        return rc; /* -3 = font red line, BEFORE any bus traffic */
    }

    /* WREN first (WEL self-clears at cycle end -- walkthrough). */
    if (sf_frame(BSP_SPI_FLASH_CMD_WREN, 0, 0, 0) != 0) {
        return -2;
    }
    arg[0] = (uint8_t)(addr >> 16);
    arg[1] = (uint8_t)(addr >> 8);
    arg[2] = (uint8_t)(addr >> 0);
    sf_cs_low();
    (void)bsp_spi_flash_xfer8(BSP_SPI_FLASH_CMD_PAGE_PROGRAM);
    (void)bsp_spi_flash_xfer8(arg[0]);
    (void)bsp_spi_flash_xfer8(arg[1]);
    (void)bsp_spi_flash_xfer8(arg[2]);
    for (i = 0u; i < len; i++) {
        (void)bsp_spi_flash_xfer8(buf[i]);
    }
    if (sf_wait_sr(SPI_SR_BSY, 0u, BSP_SPI_FLASH_FLAG_TO_MS) != 0) {
        sf_cs_high(); /* CS release terminates the frame on failure */
        return -2;
    }
    sf_cs_high(); /* frame end starts the internal program cycle */
    return sf_wait_busy(BSP_SPI_FLASH_BUSY_PROG_TO_MS);
}

const char *bsp_spi_flash_last_report(void)
{
    return s_report;
}


/* ---- SOFT bit-bang JEDEC probe (decisive A/B test) -------------------
 * Same four pins, driven by GPIO toggles at ~100 kHz instead of the
 * SPI1 engine. If THIS gets an ID but the engine does not, the chip is
 * alive and the engine path is at fault; if THIS is also 0, the chip /
 * wiring is dead regardless of software. PA5=SCK out, PA7=MOSI out,
 * PA6=MISO in, PC13=CS out (already clock/mux configured). */
#define SF_SOFT_SCK_HIGH()  (GPIOA->BSRR = (1u << 5))
#define SF_SOFT_SCK_LOW()   (GPIOA->BSRR = (1u << (5 + 16)))
#define SF_SOFT_MOSI(x)     ((x) ? (GPIOA->BSRR = (1u << 7))                                    : (GPIOA->BSRR = (1u << (7 + 16))))
#define SF_SOFT_MISO()      (((GPIOA->IDR >> 6) & 1u))

/* Every raw capture of one soft-probe run (all on the bit-bang path):
 * a repeated/slow 9F pair separates a REAL deterministic answer from
 * floating-line noise, and 0xAB/RDSR1 give the chip two more chances
 * to prove its DO pin is alive. */
typedef struct {
    uint8_t jedec_a[3];    /* 9F @ ~500 kHz, frame A          */
    uint8_t jedec_b[3];    /* 9F identical repeat: stability  */
    uint8_t jedec_slow[3]; /* 9F at ~1/5 clock: rate check    */
    uint8_t ab_id;         /* 0xAB + 3 dummy -> device ID     */
    uint8_t sr1;           /* 0x05 RDSR1 -> status byte       */
} sf_soft_result_t;

/* Half-bit spin count; 40 ~= 500 kHz, 200 ~= 100 kHz. */
static unsigned s_soft_spin = 40u;

static void sf_soft_spin(unsigned cycles)
{
    while (cycles-- > 0u) {
        __asm volatile ("nop");
        __asm volatile ("nop");
        __asm volatile ("nop");
    }
}

static uint8_t sf_soft_xfer8(uint8_t tx)
{
    uint8_t rx = 0u;
    unsigned b;
    for (b = 0u; b < 8u; b++) {
        SF_SOFT_SCK_LOW();
        SF_SOFT_MOSI((tx & (0x80u >> b)) != 0u); /* setup before edge */
        sf_soft_spin(s_soft_spin);               /* half-bit delay */
        SF_SOFT_SCK_HIGH();                      /* mode 0: sample now */
        rx = (uint8_t)((rx << 1) | SF_SOFT_MISO());
        sf_soft_spin(s_soft_spin);
    }
    SF_SOFT_SCK_LOW();
    return rx;
}

static void sf_soft_probe(sf_soft_result_t *r)
{
    unsigned i;
    const uint32_t moder_save = GPIOA->MODER &
        (GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk);

    /* THE MODE SWITCH IS THE TEST (board finding): with PA5/PA7 still
     * muxed to AF5 the SPI engine owns the pins and BSRR writes are
     * ignored -- the earlier "soft probe" measured nothing but a
     * shorted CS frame. GPIO OUTPUT mode hands the pins back to ODR so
     * the bit-bang actually reaches the bus; SPE drops meanwhile so the
     * engine never fights the toggles. */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    GPIOA->MODER = (GPIOA->MODER &
                    ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk)) |
                   GPIO_MODER_MODER5_0 | GPIO_MODER_MODER7_0; /* 01 out */
    SF_SOFT_SCK_LOW();
    SF_SOFT_MOSI(0u);
    sf_soft_spin(100); /* settle (idle levels) before the first edge */
    s_soft_spin = 40u;

    /* D first: 0xAB + 3 dummy -> the device's own ID byte (0x17 =
     * W25Q128), and the frame doubles as the DPD release. A sleeping
     * chip is awake for every probe below. */
    sf_cs_low();
    (void)sf_soft_xfer8(BSP_SPI_FLASH_CMD_RELEASE_PD);
    (void)sf_soft_xfer8(0x00u);
    (void)sf_soft_xfer8(0x00u);
    (void)sf_soft_xfer8(0x00u);
    r->ab_id = sf_soft_xfer8(0x00u);
    sf_cs_high();
    sf_delay_ms(1u); /* tRES: stay out of the bus after the release */

    /* A: 9F @ ~500 kHz -- the frame the engine cannot hear. */
    sf_cs_low();
    (void)sf_soft_xfer8(BSP_SPI_FLASH_CMD_JEDEC);
    for (i = 0u; i < 3u; i++) {
        r->jedec_a[i] = sf_soft_xfer8(0x00u);
    }
    sf_cs_high();

    /* B: identical repeat. Same bytes twice = a real answer; different
     * bytes = the "ID" is noise on a floating MISO line. */
    sf_cs_low();
    (void)sf_soft_xfer8(BSP_SPI_FLASH_CMD_JEDEC);
    for (i = 0u; i < 3u; i++) {
        r->jedec_b[i] = sf_soft_xfer8(0x00u);
    }
    sf_cs_high();

    /* C: 9F at ~1/5 clock (~100 kHz) -- rate sensitivity. */
    s_soft_spin = 200u;
    sf_cs_low();
    (void)sf_soft_xfer8(BSP_SPI_FLASH_CMD_JEDEC);
    for (i = 0u; i < 3u; i++) {
        r->jedec_slow[i] = sf_soft_xfer8(0x00u);
    }
    sf_cs_high();
    s_soft_spin = 40u;

    /* E: RDSR1 -- any answer proves the chip's DO is driving. */
    sf_cs_low();
    (void)sf_soft_xfer8(BSP_SPI_FLASH_CMD_RDSR1);
    r->sr1 = sf_soft_xfer8(0x00u);
    sf_cs_high();

    /* Hand the pins back to AF5 + re-enable the engine. */
    GPIOA->MODER = (GPIOA->MODER &
                    ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk)) |
                   moder_save;
    SPI1->CR1 |= SPI_CR1_SPE;
}

int bsp_spi_flash_selftest(void)
{
    /* Step 1: init + CR1 read-back (mode 0/master//16/soft NSS). */
    if (bsp_spi_flash_init() != 0) {
        sf_set_report("(cfg readback mismatch)");
        return (int)BSP_SPI_FLASH_ST1_CONFIG;
    }

    /* Step 2: JEDEC ID. Attempt chain -- each attempt names itself in
     * the report, so the transcript shows WHICH datapath went silent:
     *   1. soft bit-bang @ ~500 kHz   (engine-independent A/B; init
     *      already sent 0xAB, so even a DPD part is awake by now)
     *   2. SPI1 engine, mode 0, 5.25 MHz (the normal path)
     *   3. SPI1 engine, mode 0, 328 kHz  (hot-edge / wiring hypothesis)
     *   4. SPI1 engine, mode 3, 328 kHz  (W25Q128 supports mode 0 AND 3)
     *   5. 0x90 manufacturer/device ID   (alternate ID decode path)
     * The soldered W25Q128 is EXPECTED to answer EF 40 18; silence is a
     * step-2 FAIL whose report carries every attempt's raw bytes. */
    {
        uint8_t id[3];
        sf_soft_result_t soft;
        uint32_t i;

        sf_soft_probe(&soft);
        if (bsp_spi_flash_jedec_id(id) != 0) {
            sf_set_report("(id frame timeout)");
            return (int)BSP_SPI_FLASH_ST2_JEDEC;
        }
        sf_set_report("9F:");
        for (i = 0u; i < 3u; i++) {
            sf_report_hex_byte(id[i]);
        }
        if ((id[0] == 0x00u) && (id[1] == 0x00u) && (id[2] == 0x00u)) {
            uint8_t slow_id[3] = { 0xFFu, 0xFFu, 0xFFu };
            uint8_t m3_id[3] = { 0xFFu, 0xFFu, 0xFFu };
            uint8_t id90[2] = { 0xFFu, 0xFFu };
            int works = 0; /* 0 none, 1 slow/mode0, 2 mode3 */

            sf_set_spi_mode(0u, 0u, 7u); /* /256 = 328 kHz */
            (void)bsp_spi_flash_jedec_id(slow_id);
            sf_set_spi_mode(1u, 1u, 7u); /* mode 3 */
            (void)bsp_spi_flash_jedec_id(m3_id);
            sf_set_spi_mode(0u, 0u, 7u); /* back to mode 0 */
            (void)sf_read_mfr_dev_id(id90);

            sf_report_append(" slow:");
            for (i = 0u; i < 3u; i++) {
                sf_report_hex_byte(slow_id[i]);
            }
            sf_report_append(" m3:");
            for (i = 0u; i < 3u; i++) {
                sf_report_hex_byte(m3_id[i]);
            }
            sf_report_append(" 90:");
            for (i = 0u; i < 2u; i++) {
                sf_report_hex_byte(id90[i]);
            }
            /* A working fallback mode IS the answer: adopt it so step 3
             * (erase/program) runs on the datapath that responded. */
            if ((m3_id[0] == BSP_SPI_FLASH_JEDEC_MFR) &&
                (m3_id[1] == BSP_SPI_FLASH_JEDEC_TYPE) &&
                (m3_id[2] == BSP_SPI_FLASH_JEDEC_SIZE)) {
                id[0] = m3_id[0];
                id[1] = m3_id[1];
                id[2] = m3_id[2];
                works = 2;
                sf_report_append(" !mode3");
            } else if ((slow_id[0] == BSP_SPI_FLASH_JEDEC_MFR) &&
                       (slow_id[1] == BSP_SPI_FLASH_JEDEC_TYPE) &&
                       (slow_id[2] == BSP_SPI_FLASH_JEDEC_SIZE)) {
                id[0] = slow_id[0];
                id[1] = slow_id[1];
                id[2] = slow_id[2];
                works = 1;
                sf_report_append(" !slow");
            }
            if (works == 0) {
                sf_set_spi_mode(0u, 0u, 4u); /* none worked: lab defaults */
            }
        }
        if ((id[0] != BSP_SPI_FLASH_JEDEC_MFR) ||
            (id[1] != BSP_SPI_FLASH_JEDEC_TYPE) ||
            (id[2] != BSP_SPI_FLASH_JEDEC_SIZE)) {
            /* Soft-path evidence: 9F twice (stability), 9F slow (rate),
             * 0xAB id, RDSR1. "ab:" 0x17 or a stable "soft:" block means
             * the chip IS answering on the bit-bang link. */
            sf_report_append(" soft:");
            for (i = 0u; i < 3u; i++) {
                sf_report_hex_byte(soft.jedec_a[i]);
            }
            sf_report_append("/");
            for (i = 0u; i < 3u; i++) {
                sf_report_hex_byte(soft.jedec_b[i]);
            }
            sf_report_append("/");
            for (i = 0u; i < 3u; i++) {
                sf_report_hex_byte(soft.jedec_slow[i]);
            }
            sf_report_append(" ab:");
            sf_report_hex_byte(soft.ab_id);
            sf_report_append(" rsr:");
            sf_report_hex_byte(soft.sr1);
            sf_report_append(" cs:");
            sf_report_hex_byte((uint8_t)((GPIOC->ODR >> 13) & 0x1u));
            /* CS must be electrically HIGH when released: a 0 here means
             * the PC13 net is shorted low (chip permanently selected). */
            sf_cs_high();
            sf_report_append(" csh:");
            sf_report_hex_byte((uint8_t)((GPIOC->IDR >> 13) & 0x1u));
            /* PA6 idle pull test: 10 = the MISO pin floats (DO trace
             * open / chip unpowered); 11 or 00 = an external tie holds
             * the net -- the wire reaches something. */
            sf_report_append(" p6:");
            sf_report_hex_byte(sf_miso_pull_probe());
            sf_report_append(" pa:");
            sf_report_hex_byte((uint8_t)((GPIOA->MODER >> 10) & 0x3Fu));
            sf_report_append(" afr:");
            sf_report_hex_byte((uint8_t)((GPIOA->AFR[0] >> 20) & 0xFFu));
            sf_report_append(" sr:");
            sf_report_hex_byte((uint8_t)(SPI1->SR & 0xFFu));
            sf_report_append(" cr1:");
            sf_report_hex_byte((uint8_t)(SPI1->CR1 & 0xFFu));
            return (int)BSP_SPI_FLASH_ST2_JEDEC;
        }
    }

    /* Step 3: free-area page exercise at 0x00010000 (red-line safe:
     * 64 KiB into a 10 MiB free area). NOR flow = erase -> verify FF ->
     * program -> verify, each phase asserted separately so the report
     * names the failing one. */
    {
        static const uint8_t pattern[BSP_SPI_FLASH_SELFTEST_LEN] = {
            0xA5u, 0x5Au, 0x00u, 0xFFu, 0x12u, 0x34u, 0x56u, 0x78u,
            0x9Au, 0xBCu, 0xDEu, 0xF0u, 0x0Fu, 0x1Eu, 0x2Du, 0x3Cu
        };
        uint8_t buf[BSP_SPI_FLASH_PAGE_SIZE];
        uint32_t i;

        if (bsp_spi_flash_sector_erase4k(BSP_SPI_FLASH_SELFTEST_ADDR) != 0) {
            sf_report_append(" erase/busy timeout");
            return (int)BSP_SPI_FLASH_ST3_PAGE;
        }
        if (bsp_spi_flash_read(BSP_SPI_FLASH_SELFTEST_ADDR, buf,
                               BSP_SPI_FLASH_PAGE_SIZE) != 0) {
            sf_report_append(" read-after-erase failed");
            return (int)BSP_SPI_FLASH_ST3_PAGE;
        }
        for (i = 0u; i < BSP_SPI_FLASH_PAGE_SIZE; i++) {
            if (buf[i] != 0xFFu) {
                sf_report_append(" erased != 0xFF");
                return (int)BSP_SPI_FLASH_ST3_PAGE;
            }
        }
        if (bsp_spi_flash_page_program(BSP_SPI_FLASH_SELFTEST_ADDR, pattern,
                                       BSP_SPI_FLASH_SELFTEST_LEN) != 0) {
            sf_report_append(" program failed/busy");
            return (int)BSP_SPI_FLASH_ST3_PAGE;
        }
        if (bsp_spi_flash_read(BSP_SPI_FLASH_SELFTEST_ADDR, buf,
                               BSP_SPI_FLASH_SELFTEST_LEN) != 0) {
            sf_report_append(" read-after-program failed");
            return (int)BSP_SPI_FLASH_ST3_PAGE;
        }
        for (i = 0u; i < BSP_SPI_FLASH_SELFTEST_LEN; i++) {
            if (buf[i] != pattern[i]) {
                sf_report_append(" program mismatch");
                return (int)BSP_SPI_FLASH_ST3_PAGE;
            }
        }
    }
    sf_report_append(" page ok");
    return 0; /* all steps passed */
}

/** @} */

/* ---- scope demo ------------------------------------------------------------ */

/* Continuous JEDEC read frames, 100 ms period, forever. Built for the
 * bench: scope CH1 on PC13 (CS) and CH2 on PA5 (SCK) shows the CS
 * pulse + 32-clock 0x9F burst every 100 ms. Exit by NRST. */
void bsp_spi_flash_scope_demo(void)
{
    uint8_t id[3];
    /* ROOT-CAUSE FIX (board finding 2026-11-21): the demo previously
     * NEVER called init -- SPI1 clock was un-gated and PA5/6/7 stayed
     * in the IOC's GPIO_Analog mode, so PA5 showed a flat line and
     * every xfer8 timed out on RXNE. Init FIRST: it gates the SPI1
     * clock, muxes PA5/PA7 to AF5, arms the engine and sends the 0xAB
     * deep-power-down release (datasheet preamble). */
    (void)bsp_spi_flash_init();

    /* REPEATED READ every 100 ms, forever -- one hardware JEDEC frame
     * (CS low -> 0x9F -> 3 ID bytes -> CS high) followed by the soft
     * bit-bang sequence (0xAB id, 9F x3 at two speeds, RDSR1). The LA
     * sees BOTH datapaths per period: if DO moves only on the soft
     * frames, the chip is alive and the engine path is deaf; if DO is
     * flat throughout, the chip itself is silent. NRST exits. */
    for (;;) {
        sf_soft_result_t soft;
        (void)bsp_spi_flash_jedec_id(id);
        sf_soft_probe(&soft);
        sf_delay_ms(100);
    }
}

/* ---- pin-probe demo (answers "is PA5 alive at all?") ---------------- */

/* Alternates forever:
 *   Phase A (5 s): PA5/PA7 as plain GPIO toggling ~1 kHz -- any scope
 *     or multimeter must see this; proves pin+probe+trace are alive.
 *   Phase B (2 s): PA5/PA7 back to AF5, SPI /256 (328 kHz), continuous
 *     JEDEC frames -- if the chip answers now, the earlier failure was
 *     a measurement artifact; if MISO stays 0x00, the wire/chip is the
 *     suspect. Exit by NRST. */
void bsp_spi_flash_pin_probe_demo(void)
{
    unsigned phase = 0u;
    for (;;) {
        if ((phase & 1u) == 0u) {
            /* Phase A: plain GPIO square wave on PA5 + PA7. */
            GPIOA->MODER = (GPIOA->MODER &
                            ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk)) |
                           GPIO_MODER_MODER5_0 | GPIO_MODER_MODER7_0; /* 01 out */
            {
                unsigned ms = 5000u;
                while (ms-- > 0u) {
                    GPIOA->ODR ^= (GPIO_ODR_OD5 | GPIO_ODR_OD7);
                    {
                        uint32_t guard = BSP_SPI_FLASH_SYSTICK_GUARD;
                        while ((SysTick->CTRL &
                                SysTick_CTRL_COUNTFLAG_Msk) == 0u) {
                            if (--guard == 0u) {
                                return; /* SysTick dead: visible stop */
                            }
                        }
                    }
                }
            }
        } else {
            /* Phase B: AF5 + slow continuous JEDEC frames. Init FIRST
             * (gates SPI1 clock + engine arm) -- same root-cause fix as
             * the scope demo. */
            (void)bsp_spi_flash_init();
            GPIOA->MODER = (GPIOA->MODER &
                            ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk)) |
                           GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1; /* 10 AF */
            GPIOA->AFR[0] = (GPIOA->AFR[0] &
                             ~(GPIO_AFRL_AFSEL5_Msk | GPIO_AFRL_AFSEL7_Msk)) |
                            GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2 |
                            GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_2; /* AF5 */
            SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) |
                        (7u << SPI_CR1_BR_Pos); /* /256 = 328 kHz */
            SPI1->CR1 &= ~SPI_CR1_SPE;
            SPI1->CR1 |= SPI_CR1_SPE;
            {
                unsigned ms = 2000u;
                while (ms-- > 0u) {
                    uint8_t id3[3];
                    (void)bsp_spi_flash_jedec_id(id3);
                    sf_delay_ms(20);
                    if (ms == 0u) {
                        break;
                    }
                }
            }
        }
        phase++;
    }
}
