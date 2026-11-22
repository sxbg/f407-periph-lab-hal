/**
 ******************************************************************************
 * @file    bsp_spi_flash.c
 * @brief   SPI1 host (HAL edition) + W25Q128 16 MiB NOR flash -- teaching
 *          module with self-test.
 ******************************************************************************
 * @addtogroup BSP_SPI_FLASH
 * @{
 *
 * == REGISTER -> HAL MAPPING ===============================================
 *   register-edition concept               HAL edition call
 *   -------------------------------------  --------------------------------
 *   RCC APB2ENR SPI1EN + PA5/6/7 AF5 mux   MX_SPI1_Init + SPI1_MspInit
 *   PC13 MODER/OTYPER + BSRR toggles       FLASH_CS label GPIO, driven by
 *                                          HAL_GPIO_WritePin (CS bookends)
 *   CR1 write (MSTR/BR/SSM/SSI/DFF...)     hspi1.Init fields (written by
 *                                          CubeMX, read back here)
 *   TXE/RXNE/BSY flag polling              HAL_SPI_TransmitReceive /
 *                                          Transmit / Receive -- the HAL
 *                                          waits every flag INCLUDING the
 *                                          trailing BSY before CS rises,
 *                                          so the CS-vs-BSY discipline is
 *                                          HAL's job now (RM 28.3.7: BSY=0
 *                                          is what proves the whole frame
 *                                          left the shift register)
 *   0xAB deep-power-down release           sf_release_power_down (kept --
 *                                          knowledge #13/#14: a DPD part
 *                                          answers ONLY 0xAB)
 *   0x05 RDSR1 BUSY polling                sf_wait_busy (kept, HAL frames)
 *   0x90 mfr/dev id + mode-3/slow-baud     dropped (register-level bench
 *   fallbacks + GPIO bit-bang probe        diagnostics, A3-forbidden now)
 * == W25Q128 COMMAND LAYER (W25Q128JV datasheet) ===========================
 *   0x9F JEDEC ID: EF (Winbond), 40 (SPI 3V3 family), 18 (128 Mbit). No
 *        WREN, no busy -- always legal, the first probe any driver sends.
 *   0xAB RELEASE POWER-DOWN: the ONLY instruction a deep-power-down
 *        (0xB9) part still hears -- everything else, 0x9F included, is
 *        ignored until the release. init() sends the bare AB + a 1 ms
 *        tRES wait once; the step-2 fallback re-sends it before the
 *        one retry (datasheet sec. 8.2.24).
 *   0x06 WREN: sets SR1 WEL (bit1); WEL SELF-CLEARS at the end of a
 *        program/erase cycle, so every write operation is the pair
 *        WREN + command.
 *   0x05 RDSR1: SR1. BUSY (bit0) = 1 while an erase/program cycle runs
 *        (typ 45 ms for a 4 KiB sector erase, 0.6 ms for a page
 *        program). Polling BUSY is HOW the driver waits -- the device
 *        ignores every command except RDSR1 while BUSY=1.
 *   0x20 4 KiB SECTOR ERASE: 24-bit address, sector = addr & ~0xFFF.
 *        Erased NOR reads as 0xFF (programming can only clear bits --
 *        the reason erase precedes every program). Typ 45 ms, max
 *        400 ms -> timeout window 500 ms.
 *   0x02 PAGE PROGRAM: up to 256 bytes, ALL INSIDE ONE PAGE (crossing
 *        the boundary WRAPS to the page start -- the API enforces
 *        alignment + size instead of trusting callers). Typ 0.6 ms,
 *        max 3 ms -> 50 ms.
 *   0x03 READ: 24-bit address, then data out forever until CS HIGH.
 *        50 MHz max -- 5.25 MHz here, no wait states needed.
 * PARTITION RED LINE (board manual "storage space partition": first
 *   10 MB free, 6 MB font area): 0x00A00000..0x00FFFFFF holds
 *   factory-burned HZK glyph libraries. Erase/program APIs check the
 *   whole target range against BSP_SPI_FLASH_FREE_LIMIT BEFORE any bus
 *   traffic (no WREN slips out) and refuse with -3.
 * == HOW TO VERIFY =========================================================
 * menu> spi_flash
 *   "PASS spi_flash 9F:EF4018 page ok" -- JEDEC ID matched the
 *   datasheet (register edition: verified 2026-11-21 on real hardware)
 *   and a 16-byte pattern survived erase -> program -> read at
 *   0x00010000 (free area). The selftest erases that sector every run,
 *   so re-runs are idempotent.
 *   Step-2 failure report carries the evidence: the first ID ("9F:"),
 *   the post-release retry ("ab:"), and the MISO raw level ("p6:") --
 *   0x17-class retry bytes = the chip was asleep and woke; identical
 *   all-zero everywhere = the part never drove MISO.
 */
#include "bsp_spi_flash.h"

#include "main.h" /* FLASH_CS / FLASH_MISO pin labels from CubeMX */
#include "stm32f4xx_hal.h"
#include "spi.h" /* hspi1 from MX_SPI1_Init (CubeMX-generated) */

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
#define BSP_SPI_FLASH_CMD_RELEASE_PD 0xABu /* release deep power-down */

/* SR1 bits of interest: BUSY bit0 (WEL bit1 not used here). */
#define BSP_SPI_FLASH_SR1_BUSY 0x01u

/* HAL per-call timeout: a byte at 5.25 MHz takes ~1.5 us; the HAL polls
 * flags with tick granularity -- 10 ms is ~100x headroom. Busy windows:
 * erase typ 45 max 400 ms -> 500 ms; program typ 0.6 max 3 ms -> 50 ms. */
#define BSP_SPI_FLASH_FLAG_TO_MS 10u
#define BSP_SPI_FLASH_BUSY_ERASE_TO_MS 500u
#define BSP_SPI_FLASH_BUSY_PROG_TO_MS 50u

/* HAL_SPI_Transmit/Receive carry a 16-bit Size -- the read API cap. */
#define BSP_SPI_FLASH_MAX_READ_LEN 0xFFFFu

/* Self-test exercise target: first 64 KiB block of the FREE area (4 KiB
 * aligned), ~1 MiB under the font red line. */
#define BSP_SPI_FLASH_SELFTEST_ADDR 0x00010000u
#define BSP_SPI_FLASH_SELFTEST_LEN 16u

/* Self-test step numbers (the value bsp_spi_flash_selftest returns). */
#define BSP_SPI_FLASH_ST1_CONFIG 1u /* HAL config read-back */
#define BSP_SPI_FLASH_ST2_JEDEC  2u /* 0x9F == EF 40 18 */
#define BSP_SPI_FLASH_ST3_PAGE   3u /* erase -> FF -> program -> compare */

/* ---- module state ----------------------------------------------------------- */

/* Report of the last selftest run; the menu reads it after the run.
 * Every append below is bounds-checked -- the register edition's
 * earlier 32-byte buffer with unchecked appends overflowed static
 * memory on the fail path. */
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

/* CS bookends (the walkthrough CS discipline, HAL-shaped): FLASH_CS is
 * a plain GPIO output, idle HIGH (deselected). ACTIVE-LOW: RESET =
 * PC13 LOW = W25Q selected. The register edition's BSRR pairs (and the
 * original PC13-polarity bug it documents) are one HAL_GPIO_WritePin
 * call each now. */
static void sf_cs_low(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static void sf_cs_high(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

/* One full CS-framed frame with N data exchanges after 'cmd': CS low,
 * command, the exchanges (rx[] optionally captured), CS high. The
 * register edition waited BSY before the CS rising edge here -- the
 * HAL does that wait inside bsp_spi_flash_xfer8, so the discipline is
 * one place, fewer lines, same contract. Returns 0 (per-byte HAL
 * failures surface as 0x00 bytes; the command-level result judges). */
static int sf_frame(uint8_t cmd, const uint8_t *tx, uint8_t *rx,
                    uint32_t n)
{
    uint32_t i;
    uint8_t got;

    sf_cs_low();
    (void)bsp_spi_flash_xfer8(cmd);
    for (i = 0u; i < n; i++) {
        got = bsp_spi_flash_xfer8((tx != 0) ? tx[i] : 0x00u);
        if (rx != 0) {
            rx[i] = got;
        }
    }
    sf_cs_high();
    return 0;
}

/* Poll RDSR1 until BUSY clears or the per-op timeout expires (the ONLY
 * way to wait on a W25Q128 -- the part ignores commands while BUSY=1).
 * Returns 0 = idle again, -1 = still busy at the window end. */
static int sf_wait_busy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t sr1 = 0u;

    for (;;) {
        /* RDSR1 frame: command + one dummy exchange clocks SR1 out. */
        if (sf_frame(BSP_SPI_FLASH_CMD_RDSR1, 0, &sr1, 1u) != 0) {
            return -1;
        }
        if ((sr1 & BSP_SPI_FLASH_SR1_BUSY) == 0u) {
            return 0;
        }
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return -1; /* still programming/erasing at the window end */
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

/* Wake the part from DEEP POWER-DOWN (W25Q128JV datasheet sec. 8.2.24):
 * after 0xB9 the device ignores EVERY instruction except 0xAB -- a DPD
 * part answers 0x9F with a silent MISO, byte-for-byte identical to a
 * broken chip (knowledge #13/#14). 0xAB with CS returned HIGH releases
 * DPD after tRES (~3 us typ, well under the 1 ms wait). A fresh part
 * is never in DPD, but one 0xAB frame per init costs 8 clocks and
 * removes "chip asleep" from the suspect list for good. */
static void sf_release_power_down(void)
{
    (void)sf_frame(BSP_SPI_FLASH_CMD_RELEASE_PD, 0, 0, 0);
    HAL_Delay(1u);
}

/* read-only (A3 exemption): MISO (PA6) raw level sampled with CS
 * released (the W25Q's DO is Hi-Z then). Replaces the register
 * edition's pull-toggle probe -- CubeMX owns PUPDR now, so the HAL
 * edition can only LOOK at the pin and record it. */
static uint8_t sf_miso_level(void)
{
    return (HAL_GPIO_ReadPin(FLASH_MISO_GPIO_Port, FLASH_MISO_Pin) ==
            GPIO_PIN_SET) ? 1u : 0u;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_spi_flash_init(void)
{
    /* Config read-back only (read-only, A3 exemption): CubeMX owns the
     * GPIO mux (SPI1_MspInit: PA5/PA6/PA7 AF5) and SPI1's CR1
     * (HAL_SPI_Init from hspi1.Init). The register edition wrote
     * MODER/AFRL/OSPEEDR by hand and re-read them -- those are the
     * generated code's guarantees now (the OSPEEDR-slew lesson,
     * knowledge #14, and the MISO-must-be-AF lesson are CubeMX's to
     * keep), verified here, never written. */
    if (hspi1.Instance == NULL) {
        return -1; /* MX_SPI1_Init never ran */
    }
    if ((hspi1.Init.Mode != SPI_MODE_MASTER) ||
        (hspi1.Init.Direction != SPI_DIRECTION_2LINES) ||
        (hspi1.Init.DataSize != SPI_DATASIZE_8BIT) ||
        (hspi1.Init.CLKPolarity != SPI_POLARITY_LOW) ||
        (hspi1.Init.CLKPhase != SPI_PHASE_1EDGE) ||
        (hspi1.Init.NSS != SPI_NSS_SOFT) ||
        (hspi1.Init.BaudRatePrescaler != SPI_BAUDRATEPRESCALER_16) ||
        (hspi1.Init.FirstBit != SPI_FIRSTBIT_MSB)) {
        return -1;
    }
    /* CR1 sanity: master + software NSS pinned HIGH; CPOL/CPHA/LSB/
     * DFF all reset-value zeros. SPE is deliberately NOT required
     * here -- the HAL leaves the engine disabled between transfer
     * calls and enables it inside each HAL_SPI_* call. */
    {
        const uint32_t cr1 = hspi1.Instance->CR1;
        if (((cr1 & (SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI)) !=
             (SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI)) ||
            ((cr1 & (SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST |
                     SPI_CR1_DFF)) != 0u)) {
            return -1;
        }
    }
    /* CS bookend: idle HIGH before the first frame (MX_GPIO_Init does
     * this at boot; init re-asserts ownership). */
    sf_cs_high();

    /* Datasheet preamble: if the part was left in deep power-down,
     * this AB frame is the ONLY thing it will hear -- do it once per
     * init, before any caller sends a real command. */
    sf_release_power_down();
    return 0;
}

uint8_t bsp_spi_flash_xfer8(uint8_t tx)
{
    uint8_t rx = 0u;

    /* Full-duplex discipline, HAL-shaped: HAL_SPI_TransmitReceive
     * waits TXE, loads DR, waits RXNE, reads DR, then waits BSY clear
     * before returning -- the CS-vs-BSY tail the register edition had
     * to poll by hand (RM 28.3.7). */
    if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1u,
                                BSP_SPI_FLASH_FLAG_TO_MS) != HAL_OK) {
        return 0u; /* caller's command-level result carries the failure */
    }
    return rx;
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
    uint8_t head[4];

    if ((buf == 0) || (len == 0u) || (len > BSP_SPI_FLASH_MAX_READ_LEN) ||
        (addr >= BSP_SPI_FLASH_TOTAL_SIZE) ||
        ((addr + len) > BSP_SPI_FLASH_TOTAL_SIZE)) {
        return -1;
    }
    /* 0x03 + big-endian 24-bit address (walkthrough: MSB first), then
     * the device clocks data out until CS rises. */
    head[0] = BSP_SPI_FLASH_CMD_READ;
    head[1] = (uint8_t)(addr >> 16);
    head[2] = (uint8_t)(addr >> 8);
    head[3] = (uint8_t)(addr >> 0);
    sf_cs_low();
    if ((HAL_SPI_Transmit(&hspi1, head, 4u, BSP_SPI_FLASH_FLAG_TO_MS) !=
         HAL_OK) ||
        (HAL_SPI_Receive(&hspi1, buf, (uint16_t)len,
                         BSP_SPI_FLASH_FLAG_TO_MS) != HAL_OK)) {
        sf_cs_high();
        return -2; /* HAL transfer failed (frame aborted) */
    }
    sf_cs_high(); /* ends the 0x03 stream (device stops driving MISO) */
    return 0;
}

int bsp_spi_flash_sector_erase4k(uint32_t addr)
{
    uint8_t head[4];
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
    head[0] = BSP_SPI_FLASH_CMD_SECTOR_ERASE;
    head[1] = (uint8_t)(addr >> 16);
    head[2] = (uint8_t)(addr >> 8);
    head[3] = (uint8_t)(addr >> 0);
    sf_cs_low();
    if (HAL_SPI_Transmit(&hspi1, head, 4u, BSP_SPI_FLASH_FLAG_TO_MS) !=
        HAL_OK) {
        sf_cs_high();
        return -2;
    }
    sf_cs_high();
    return sf_wait_busy(BSP_SPI_FLASH_BUSY_ERASE_TO_MS);
}

int bsp_spi_flash_page_program(uint32_t addr, const uint8_t *buf,
                               uint32_t len)
{
    uint8_t head[4];
    int rc;

    /* Page contract (walkthrough): aligned start + len <= page size +
     * whole range inside the ONE page => no boundary is crossed. NOR
     * semantics: program can only clear 1-bits, so the page must be
     * erased beforehand. */
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
    head[0] = BSP_SPI_FLASH_CMD_PAGE_PROGRAM;
    head[1] = (uint8_t)(addr >> 16);
    head[2] = (uint8_t)(addr >> 8);
    head[3] = (uint8_t)(addr >> 0);
    sf_cs_low();
    if ((HAL_SPI_Transmit(&hspi1, head, 4u, BSP_SPI_FLASH_FLAG_TO_MS) !=
         HAL_OK) ||
        (HAL_SPI_Transmit(&hspi1, buf, (uint16_t)len,
                          BSP_SPI_FLASH_FLAG_TO_MS) != HAL_OK)) {
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

/* Is 'id' not the expected EF 40 18? (readability helper for step 2). */
static int sf_jedec_mismatch(const uint8_t id[3])
{
    return ((id[0] != BSP_SPI_FLASH_JEDEC_MFR) ||
            (id[1] != BSP_SPI_FLASH_JEDEC_TYPE) ||
            (id[2] != BSP_SPI_FLASH_JEDEC_SIZE)) ? 1 : 0;
}

int bsp_spi_flash_selftest(void)
{
    /* Step 1: init + config read-back (HAL Init fields + CR1 + CS
     * idle). */
    if (bsp_spi_flash_init() != 0) {
        sf_set_report("(cfg readback mismatch)");
        return (int)BSP_SPI_FLASH_ST1_CONFIG;
    }

    /* Step 2: JEDEC ID. The soldered W25Q128 is EXPECTED to answer
     * EF 40 18. On a silent/garbage answer: ONE software 0xAB release
     * frame (knowledge #13/#14 -- a deep-power-down part answers ONLY
     * the release command; if the flash answered garbage, the AB frame
     * is what wakes it) + retry, then the raw IDs and the MISO level
     * into the report. The register edition's mode-3/slow-baud/bit-
     * bang fallback engines are gone with their register writes. */
    {
        uint8_t id[3];
        uint32_t i;

        if (bsp_spi_flash_jedec_id(id) != 0) {
            sf_set_report("(id frame timeout)");
            return (int)BSP_SPI_FLASH_ST2_JEDEC;
        }
        sf_set_report("9F:");
        for (i = 0u; i < 3u; i++) {
            sf_report_hex_byte(id[i]);
        }
        if (sf_jedec_mismatch(id) != 0) {
            sf_release_power_down();
            if (bsp_spi_flash_jedec_id(id) != 0) {
                sf_report_append(" ab:--"); /* retry frame failed */
            } else {
                sf_report_append(" ab:");
                for (i = 0u; i < 3u; i++) {
                    sf_report_hex_byte(id[i]);
                }
            }
            /* MISO raw level with CS released (Hi-Z DO): diagnostic
             * evidence only -- 0 with the net floating open, 1 with an
             * external tie. */
            sf_report_append(" p6:");
            sf_report_hex_byte(sf_miso_level());
            if (sf_jedec_mismatch(id) != 0) {
                return (int)BSP_SPI_FLASH_ST2_JEDEC;
            }
            /* The release+retry found the real ID: keep going (the
             * report still shows the first answer for the transcript). */
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
