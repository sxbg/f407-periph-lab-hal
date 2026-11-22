/**
 ******************************************************************************
 * @file    bsp_flash_iap.c
 * @brief   Internal flash IAP -- teaching module: KEY1/KEY2 unlock, sector
 *          erase, word programming, option-byte pointer -- with every
 *          mutating operation hard-limited to the last-sector param page.
 ******************************************************************************
 * @addtogroup BSP_FLASH_IAP
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | FLASH interface   | NOT PRESENT in the IOC               | knowledge #13:
 * |                   | (CubeMX has no page for it)          | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | Device density    | STM32F407VETx = 512 KB: 4x16K +      | RM0090 Table 5
 * |                   | 64K + 3x128K, last sector = 7        | ("Sector 11" is
 * |                   | (0x0806 0000-0x0807 FFFF)            | 1 MB-dense parts)
 * | Readout protect   | off (RDP level 0)                    | option bytes,
 * |                   |                                      | teaching pointer
 * | Parameter page    | 0x0807FC00..0x0807FFFF (1 KB)        | last-sector tail
 * == REGISTER WALKTHROUGH (RM0090 ch.3 flash interface) ====================
 * The flash controller is the ONE peripheral whose subject is the memory
 * the CPU is EXECUTING FROM -- every rule below exists because of that.
 *   KEYR (ch.3.6.1): write KEY1 = 0x45670123 then KEY2 = 0xCDEF89AB in
 *     that exact order -> CR.LOCK clears. Reading KEYR is illegal
 *     (write-only); the lock STATE is CR.LOCK bit31 (1 = locked, the
 *     reset value).
 *   ACR (ch.3.4): LATENCY[2:0] wait states for the read speed (168 MHz
 *     needs 5 WS), PRFTEN prefetch, ICEN/DCEN caches. Owned by the boot
 *     clock init; read here for provenance only.
 *   CR (ch.3.7.3): PG bit0 program-mode, SER bit1 sector-erase-mode,
 *     MER bit2 mass-erase, SNB[3:0] bits6:3 target sector, PSIZE[1:0]
 *     bits9:8 parallel size (00 byte / 01 halfword / 10 word / 11
 *     doubleword -- must match the data width written; it also selects
 *     the datasheet's erase-time class), STRT bit16 kick off, EOPIE/
 *     ERRIE interrupt enables (off: polling lesson), LOCK bit31.
 *   SR (ch.3.7.4): BSY bit16 -- the ONLY "operation done" oracle (EOP
 *     bit0 is optional); error bits OPERR1 WRPERR4 PGAERR5 PGPERR6
 *     PGSERR7, all rc_w1 (cleared by WRITING 1).
 * Sector erase sequence (ch.3.6.2), exactly as implemented below:
 *   1. check SR.BSY == 0
 *   2. unlock (keys above)
 *   3. SER + SNB=7 + PSIZE=word (PG/MER masked off)
 *   4. STRT; 5. wait BSY falling (typ 1.3 s / max 2.6 s for a 128 KB
 *      sector at word PSIZE per the datasheet tables -- the wait bound
 *      lives in the header);
 *   6. audit + clear error bits; 7. SER off, re-lock.
 * Word program sequence (ch.3.6.3): unlock, PG=1 + PSIZE=word, write the
 *   full 32-bit word to the address (the write IS the programming
 *   command), wait BSY per word, PG off, error audit, lock.
 *   Flash physics: a bit goes 1->0 by programming, 0->1 ONLY by erase --
 *   that is why the parameter page must be erased between different
 *   contents, and why "program once, keep forever" is the natural
 *   teaching use of a parameter store.
 * THE WARNING COMMENT (task sheet, restated as policy): erasing or
 *   programming a sector the code runs from stops the fetch mid-
 *   instruction -- the CPU stalls on the bus or executes garbage; there
 *   is no recovery. Policy: the last sector / param page ONLY, enforced
 *   by the API shape itself (erase takes no argument; program refuses
 *   any address outside the page) -- refusal is better than doc text.
 * INTERRUPT WINDOW: each mutating operation runs with PRIMASK raised
 *   (save/restore, not a blind enable). Why mask at all: (a) no ISR can
 *   be forced to re-enter flash mid-erase and stall the handler; (b) the
 *   register sequence stays atomic against any other user of the
 *   controller. Cost: SysTick-driven services (HAL tick, RTT heartbeat)
 *   pause for the operation duration (~1.3-2.6 s erase) -- visible,
 *   documented, harmless for a lab. (COUNTFLAG keeps setting while
 *   PRIMASK is up, so the BSY waits stay bounded -- PM0056 B3.3.)
 * Option bytes, one sentence (ch.3.8): OPTKEYR with 0x08192346/0x08201358
 *   opens OPTCR for RDP/BOR/WDG_SW/nRST_* config -- one-way choices with
 *   an option-byte load cycle; NOT touched by this module.
 * == HOW TO VERIFY =========================================================
 * menu> flash_iap
 *   PASS flash_iap (param 8w kept) -- the erase takes ~1-3 s (console
 *   pauses: that is the PRIMASK window, by design), FF sweep over the
 *   full 128 KB, 8 words written at 0x0807FC00, compared, left in place.
 * Power-cycle the board: the 8 words SURVIVE (that is the retention
 * lesson) and are erased+rewritten by the next run -- to see them, look
 * at 0x0807FC00 in the Keil memory window BEFORE re-running the command
 * (pattern: 53454737 50415241 31323334 35363738 01020304 05060708
 * A5A55A5A 5A5AA5A5).
 */
#include "bsp_flash_iap.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Unlock keys (RM0090 ch.3.6.1; the numeric truth lives here, not in a
 * HAL header, so this module stays on the CMSIS device header only). */
#define FLASH_IAP_KEY1 0x45670123u
#define FLASH_IAP_KEY2 0xCDEF89ABu

/* PSIZE = word programming (bits [9:8] = 10): matches the 32-bit writes
 * below, and sets the datasheet's word-PSIZE erase-time class. */
#define FLASH_IAP_PSIZE_WORD (FLASH_CR_PSIZE_1)

/* SR error bits (rc_w1: write 1 to clear) + EOP -- the write-mask used
 * after each operation. BSY (read-only) is excluded. */
#define FLASH_IAP_ERRMASK (FLASH_SR_EOP | FLASH_SR_OPERR | \
                           FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                           FLASH_SR_PGPERR | FLASH_SR_PGSERR)

/* The 8 teaching words programmed into the parameter page (step 2): an
 * ASCII-stamped header pair + a stride pattern, easy to recognize in the
 * Keil memory window (walkthrough). */
static const uint32_t param_test_words[8] = {
    0x53454737u, /* "SEG7"  -- written in the last sector          */
    0x50415241u, /* "PARA"  -- the parameter page marker           */
    0x31323334u, 0x35363738u, /* "1234" "5678"                    */
    0x01020304u, 0x05060708u, /* stride                           */
    0xA5A55A5Au, 0x5A5AA5A5u  /* the RTC-module magic + inverse   */
};

/* Self-test step numbers (the value bsp_flash_iap_selftest returns). */
#define BSP_FL_ST1_UNLOCK 1u /* LOCK 1->0->1 cycle proven          */
#define BSP_FL_ST2_ERASE  2u /* erase -> FF sweep -> 8 words kept  */
#define BSP_FL_ST3_LOCK   3u /* re-lock readback                   */

#define BSP_FL_SYSTICK_GUARD 2000000u

/* Full erase-verify sweep length: 128 KB = 32768 words. */
#define BSP_FL_SECTOR_WORDS (BSP_FLASH_LAST_SIZE / 4u)

/* ---- private state ---------------------------------------------------------- */

static char s_report[40];
static int s_report_valid = 0;

static void report_set(const char *msg)
{
    unsigned i = 0u;
    while ((msg[i] != '\0') && (i < (sizeof(s_report) - 1u))) {
        s_report[i] = msg[i];
        i++;
    }
    s_report[i] = '\0';
    s_report_valid = 1;
}

/* ---- private helpers -------------------------------------------------------- */

/* Poll SR.BSY falling within timeout_ms (COUNTFLAG-paced, PRIMASK-proof:
 * the COUNTFLAG sets on the wrap whether or not the interrupt is taken --
 * PM0056 B3.3). Returns 0 = idle, -1 = timeout / SysTick dead. */
static int fl_wait_bsy(uint32_t timeout_ms)
{
    uint32_t guard = BSP_FL_SYSTICK_GUARD;
    for (;;) {
        if ((FLASH->SR & FLASH_SR_BSY) == 0u) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
                guard = BSP_FL_SYSTICK_GUARD;
            } else {
                return -1;
            }
        }
        if (--guard == 0u) {
            return -1;
        }
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_flash_iap_init(void)
{
    /* Touch nothing: a read of ACR proves the controller answers, and the
     * observed LOCK state goes into the report (teaching: know the lock
     * and the boot-set latency before ever mutating). */
    const uint32_t acr = FLASH->ACR;
    const uint32_t locked = (FLASH->CR & FLASH_CR_LOCK) ? 1u : 0u;

    if ((acr & FLASH_ACR_LATENCY) == 0u) {
        report_set("(warn: 0 WS latency?)"); /* 168 MHz needs 5 WS -- a
                                              * 0-WS read-back would mean
                                              * the clock tree fell back  */
    } else if (locked != 0u) {
        report_set("(locked, ready)");
    } else {
        report_set("(UNLOCKED at boot: odd)");
    }
    return 0;
}

int bsp_flash_iap_erase_last_sector(void)
{
    uint32_t primask;
    int rc = 0;

    /* No sector argument on purpose (header contract): the ONLY sector
     * this function can ever touch is BSP_FLASH_LAST_SECTOR. */

    if ((FLASH->SR & FLASH_SR_BSY) != 0u) {
        return -1; /* ch.3.6.2 step 1: controller must be idle */
    }

    primask = __get_PRIMASK(); /* interrupt window OPEN: shortest possible */
    __disable_irq();

    FLASH->KEYR = FLASH_IAP_KEY1; /* ch.3.6.1: two keys, exact order */
    FLASH->KEYR = FLASH_IAP_KEY2;
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
        __set_PRIMASK(primask);
        return -1; /* keys rejected (wrong order / RDP interference) */
    }

    /* SER + SNB + PSIZE, then STRT (ch.3.6.2 steps 3-4). SNB[3:0]=7 is
     * valid ONLY on this 512 KB part (the 1 MB tables' sectors 8..11 do
     * not exist here -- walkthrough). PG/MER masked off for hygiene. */
    FLASH->CR = (FLASH->CR & ~(FLASH_CR_SNB | FLASH_CR_PSIZE |
                               FLASH_CR_PG | FLASH_CR_MER)) |
                FLASH_CR_SER | (BSP_FLASH_LAST_SECTOR << 3) |
                FLASH_IAP_PSIZE_WORD;
    FLASH->CR |= FLASH_CR_STRT;

    if (fl_wait_bsy(BSP_FLASH_ERASE_TIMEOUT_MS) != 0) {
        rc = -1; /* > 6 s for a 128 KB sector: controller misbehaving */
    }
    /* Error audit (rc_w1 clear even on success -- hygiene). */
    if ((FLASH->SR & FLASH_IAP_ERRMASK) != 0u) {
        FLASH->SR = FLASH_IAP_ERRMASK; /* write-1-to-clear */
        rc = -1;
    }

    FLASH->CR &= ~FLASH_CR_SER; /* leave erase mode before re-locking */
    FLASH->CR |= FLASH_CR_LOCK; /* re-protect: window CLOSED */
    __set_PRIMASK(primask);     /* interrupt window CLOSED */
    return rc;
}

int bsp_flash_iap_program_words(uint32_t addr, const uint32_t *src,
                                uint32_t n)
{
    uint32_t primask;
    uint32_t i;
    int rc = 0;

    /* The SAFETY GUARD (header contract): param page, word-aligned, in
     * range. Everything else is refused BEFORE any flash register move. */
    if (src == 0) {
        return -2;
    }
    if ((n == 0u) || ((addr & 3u) != 0u) ||
        (addr < BSP_FLASH_PARAM_ADDR) ||
        ((addr + (n * 4u)) > (BSP_FLASH_PARAM_ADDR + BSP_FLASH_PARAM_SIZE))) {
        return -2;
    }
    if ((FLASH->SR & FLASH_SR_BSY) != 0u) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    FLASH->KEYR = FLASH_IAP_KEY1;
    FLASH->KEYR = FLASH_IAP_KEY2;
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
        __set_PRIMASK(primask);
        return -1;
    }

    /* PG + PSIZE=word (ch.3.6.3): each 32-bit write latches one word. */
    FLASH->CR = (FLASH->CR & ~(FLASH_CR_SER | FLASH_CR_MER |
                               FLASH_CR_PSIZE)) |
                FLASH_CR_PG | FLASH_IAP_PSIZE_WORD;

    for (i = 0u; i < n; i++) {
        /* The write IS the programming command (walkthrough). */
        *(volatile uint32_t *)(addr + (i * 4u)) = src[i];
        if (fl_wait_bsy(BSP_FLASH_PROG_TIMEOUT_MS) != 0) {
            rc = -1;
            break;
        }
        if ((FLASH->SR & FLASH_IAP_ERRMASK) != 0u) {
            rc = -1; /* e.g. PGAERR: the 1->0-only rule was violated */
            break;
        }
    }

    FLASH->CR &= ~FLASH_CR_PG;     /* leave program mode first */
    FLASH->SR = FLASH_IAP_ERRMASK; /* rc_w1 clear (walkthrough) */
    FLASH->CR |= FLASH_CR_LOCK;
    __set_PRIMASK(primask);
    return rc;
}

int bsp_flash_iap_read_words(uint32_t addr, uint32_t *dst, uint32_t n)
{
    uint32_t i;

    if ((dst == 0) || (n == 0u) || ((addr & 3u) != 0u) ||
        (addr < 0x08000000u) ||
        ((addr + (n * 4u)) > (0x08000000u + (512u * 1024u)))) {
        return -1;
    }
    /* No unlock, no registers: flash is memory-mapped (the teaching
     * point -- the CPU is reading it right now to fetch this code). */
    for (i = 0u; i < n; i++) {
        dst[i] = *(volatile uint32_t *)(addr + (i * 4u));
    }
    return 0;
}

const char *bsp_flash_iap_last_report(void)
{
    return (s_report_valid != 0) ? s_report : "";
}

int bsp_flash_iap_selftest(void)
{
    /* Step 1: unlock mechanics -- keys in, LOCK reads 0, lock again,
     * LOCK reads 1. The whole ch.3.6.1 contract in four register ops. */
    {
        uint32_t primask = __get_PRIMASK();
        int bad = 0;
        __disable_irq();
        FLASH->KEYR = FLASH_IAP_KEY1;
        FLASH->KEYR = FLASH_IAP_KEY2;
        if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
            bad = 1; /* unlock rejected */
        }
        FLASH->CR |= FLASH_CR_LOCK;
        if ((FLASH->CR & FLASH_CR_LOCK) == 0u) {
            bad = 1; /* re-lock rejected */
        }
        __set_PRIMASK(primask);
        if (bad != 0) {
            return (int)BSP_FL_ST1_UNLOCK;
        }
    }

    /* Step 2: erase -> full-sector FF sweep -> program 8 words -> compare
     * -> KEEP them (power-off retention demo, header). The erase holds
     * interrupts off for ~1.3-2.6 s: the console/heartbeat pause is the
     * documented cost (walkthrough). */
    if (bsp_flash_iap_erase_last_sector() != 0) {
        return (int)BSP_FL_ST2_ERASE;
    }
    {
        /* Full 128 KB sweep: every one of the 32768 words must read
         * 0xFFFFFFFF. Reads-only (no PRIMASK needed), ~2 ms at 168 MHz,
         * bounded loop. */
        const volatile uint32_t *p =
            (const volatile uint32_t *)BSP_FLASH_LAST_BASE;
        uint32_t remaining = BSP_FL_SECTOR_WORDS;
        while (remaining > 0u) {
            if ((*p++) != 0xFFFFFFFFu) {
                return (int)BSP_FL_ST2_ERASE; /* erase incomplete */
            }
            remaining--;
        }
    }
    if (bsp_flash_iap_program_words(BSP_FLASH_PARAM_ADDR,
                                    param_test_words, 8u) != 0) {
        return (int)BSP_FL_ST2_ERASE;
    }
    {
        uint32_t back[8];
        uint32_t k;
        if (bsp_flash_iap_read_words(BSP_FLASH_PARAM_ADDR, back, 8u) != 0) {
            return (int)BSP_FL_ST2_ERASE;
        }
        for (k = 0u; k < 8u; k++) {
            if (back[k] != param_test_words[k]) {
                return (int)BSP_FL_ST2_ERASE; /* compare mismatch */
            }
        }
        report_set("(erase ok, param 8w kept)");
    }

    /* Step 3: final lock proof (and hygiene: the module NEVER leaves the
     * flash writable behind a menu command). */
    if ((FLASH->CR & FLASH_CR_LOCK) == 0u) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        FLASH->CR |= FLASH_CR_LOCK;
        __set_PRIMASK(primask);
        if ((FLASH->CR & FLASH_CR_LOCK) == 0u) {
            return (int)BSP_FL_ST3_LOCK;
        }
    }
    return 0; /* all steps passed */
}

/** @} */
