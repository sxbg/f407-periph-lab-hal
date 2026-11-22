/**
 ******************************************************************************
 * @file    bsp_flash_iap.c
 * @brief   Internal flash IAP -- teaching module (HAL edition of
 *          projects/f407-periph-lab bsp_flash_iap): HAL_FLASH_Unlock/
 *          Ex_Erase/Program/Lock -- with every mutating operation
 *          hard-limited to the last-sector param page.
 ******************************************************************************
 * @addtogroup BSP_FLASH_IAP
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab-hal.ioc -- by design) ======
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
 * == HAL WALKTHROUGH (what each HAL_FLASH_* call does under the hood) ======
 * The flash controller is the ONE peripheral whose subject is the memory
 * the CPU is EXECUTING FROM -- every rule below exists because of that.
 * The A3 rule says this module drives it ONLY through the HAL API; what
 * HAL does on the registers (RM0090 ch.3) is the teaching payload:
 *   Unlock/Lock (HAL_FLASH_Unlock/Lock): the KEY1 = 0x45670123 / KEY2 =
 *     0xCDEF89AB sequence into KEYR, in that exact order, clears CR.LOCK
 *     (bit31, 1 = locked, the reset value). HAL re-checks LOCK after the
 *     keys and returns HAL_ERROR if the unlock was rejected. Locking is
 *     one CR.LOCK write -- always pair every Unlock with a Lock.
 *   Sector erase (HAL_FLASHEx_Erase): builds CR = SER + SNB[3:0] = sector
 *     + PSIZE[1:0] (the VoltageRange argument: range 3 = 2.7-3.6 V = word
 *     parallelism), kicks STRT, then polls SR.BSY with the HAL's own
 *     timeout (FLASH_TIMEOUT_VALUE = 50 s). The datasheet numbers behind
 *     that wait: 128 KB sector erase typ 1.3 s / max 2.6 s at word PSIZE
 *     (the header constants document the physics). Error bits (WRPERR/
 *     PGAERR/PGPERR/PGSERR) land in the HAL's ErrorCode output and make
 *     the call return HAL_ERROR -- the register edition's rc_w1 audit,
 *     done for us.
 *   Word program (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data)):
 *     CR = PG + PSIZE=word, then the full 32-bit store to the address
 *     (the write IS the programming command), then the per-word BSY poll.
 *     Flash physics: a bit goes 1->0 by programming, 0->1 ONLY by erase
 *     -- that is why the parameter page must be erased between different
 *     contents, and why "program once, keep forever" is the natural
 *     teaching use of a parameter store.
 *   ACR (latency/caches): owned by the boot clock init; READ here for
 *     provenance only (A3 read-only exemption -- 168 MHz needs 5 WS).
 * THE WARNING COMMENT (register edition, restated as policy): erasing or
 *   programming a sector the code runs from stops the fetch mid-
 *   instruction -- the CPU stalls on the bus or executes garbage; there
 *   is no recovery. Policy: the last sector / param page ONLY, enforced
 *   by the API shape itself (erase takes no argument; program refuses
 *   any address outside the page) -- refusal is better than doc text.
 * INTERRUPT WINDOW (kept from the register edition): each mutating
 *   operation runs with PRIMASK raised (save/restore, not a blind
 *   enable). Why mask at all: (a) no ISR can be forced to re-enter flash
 *   mid-erase and stall the handler; (b) the HAL's internally generated
 *   register sequence stays atomic against any other user of the
 *   controller. Cost: SysTick-driven services (HAL tick, log heartbeat)
 *   pause for the operation duration (~1.3-2.6 s erase) -- visible,
 *   documented, harmless for a lab. HONEST SIDE EFFECT of masking: the
 *   HAL's tick-based wait inside FLASH_WaitForLastOperation sees a FROZEN
 *   HAL_GetTick(), so its FLASH_TIMEOUT_VALUE degenerates to a pure BSY
 *   poll; BSY falling is the real (datasheet-bounded) exit. __disable_irq
 *   is a CMSIS CPU instruction, not a peripheral register write (A3).
 * Option bytes, one sentence (ch.3.8): OPTKEYR with 0x08192346/0x08201358
 *   opens OPTCR for RDP/BOR/WDG_SW/nRST_* config -- one-way choices with
 *   an option-byte load cycle; NOT touched by this module (and NOT in the
 *   HAL API -- that teaching stays register-edition material).
 * == HOW TO VERIFY =========================================================
 * menu> flash_iap (menu wiring re-added by the integrator)
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

#include "stm32f4xx_hal.h"

/* ---- module constants ----------------------------------------------------- */

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

/* ---- public API ------------------------------------------------------------ */

int bsp_flash_iap_init(void)
{
    /* Touch nothing: a read of ACR proves the controller answers, and the
     * observed LOCK state goes into the report (teaching: know the lock
     * and the boot-set latency before ever mutating). */
    const uint32_t acr = FLASH->ACR;     /* read-only (A3 exemption) */
    const uint32_t locked = (FLASH->CR & FLASH_CR_LOCK) ? 1u : 0u; /* read-only (A3 exemption) */

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
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0xFFFFFFFFu; /* HAL convention: preset to
        all-ones, stays that way on success (knowledge #17) */
    uint32_t primask;
    int rc = 0;

    /* No sector argument on purpose (header contract): the ONLY sector
     * this function can ever touch is BSP_FLASH_LAST_SECTOR. */

    primask = __get_PRIMASK(); /* interrupt window OPEN: shortest possible */
    __disable_irq();

    /* The KEYR key sequence + LOCK re-check (ch.3.6.1 steps, HAL form). */
    if (HAL_FLASH_Unlock() != HAL_OK) {
        __set_PRIMASK(primask);
        report_set("(unlock rejected)");
        return -1; /* keys rejected (wrong order / RDP interference) */
    }

    /* SER + SNB=last sector + PSIZE=word + STRT + bounded BSY poll
     * (ch.3.6.2 steps 3-5, HAL form). SNB=7 is valid ONLY on this 512 KB
     * part (the 1 MB tables' sectors 8..11 do not exist here -- header
     * note). Voltage range 3 = word parallelism, matching the program
     * width below. */
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_BANK_1; /* single-bank part (VE) */
    erase.Sector       = BSP_FLASH_LAST_SECTOR;
    erase.NbSectors    = 1u;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    /* Stale error bits from an earlier failed operation make every new
     * operation set PGSERR immediately (observed live: SR=0xC0 before
     * the first erase of a boot). Clear them before issuing STRT --
     * sanctioned HAL macro, A3-clean (flash controller, not a peripheral
     * the teaching module owns). */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        rc = -1; /* >50 s for a 128 KB sector: controller misbehaving */
        {
            /* report the raw codes: "(erase rc HAL err=0x... snberr=0x...)" */
            char buf[40];
            const char *pre = "(erase err snb=";
            unsigned i = 0u, k = 0u;
            unsigned v = sector_error;
            while (pre[i] != 0) { buf[i] = pre[i]; i++; }
            do { /* sector_error nibble hex */
                unsigned nib = (v >> 28) & 0xFu;
                unsigned sh = 28u;
                while (1) {
                    buf[i++] = (nib < 10u) ? (char)('0' + nib)
                                           : (char)('A' + nib - 10u);
                    if (sh == 0u) break;
                    sh -= 4u;
                    nib = (v >> sh) & 0xFu;
                }
                break;
            } while (0);
            buf[i++] = ')';
            buf[i] = 0;
            (void)k;
            report_set(buf);
        }
    }
    if (sector_error != 0xFFFFFFFFu) {
        report_set("(erase snb error)");
        rc = -1; /* SR error bits audited by HAL (WRP/PGA/PGP/PGS...) */
    }

    HAL_FLASH_Lock();       /* re-protect: window CLOSED */
    __set_PRIMASK(primask); /* interrupt window CLOSED */
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

    primask = __get_PRIMASK();
    __disable_irq();

    if (HAL_FLASH_Unlock() != HAL_OK) {
        __set_PRIMASK(primask);
        return -1;
    }

    /* PG + PSIZE=word + the 32-bit store + per-word BSY poll (ch.3.6.3,
     * HAL form): one HAL_FLASH_Program call per word. */
    for (i = 0u; i < n; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (i * 4u),
                              (uint64_t)src[i]) != HAL_OK) {
            rc = -1; /* e.g. PGAERR: the 1->0-only rule was violated */
            break;
        }
    }

    HAL_FLASH_Lock();       /* leave program mode + re-protect */
    __set_PRIMASK(primask); /* interrupt window CLOSED */
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
    /* No unlock, no HAL calls: flash is memory-mapped (the teaching
     * point -- the CPU is reading it right now to fetch this code).
     * Allowed read, no A3 concern. */
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
    /* Step 1: unlock mechanics -- HAL key sequence in, LOCK reads 0,
     * HAL lock again, LOCK reads 1. The whole ch.3.6.1 contract through
     * the HAL API, proven twice over. */
    {
        uint32_t primask = __get_PRIMASK();
        int bad = 0;
        __disable_irq();
        if (HAL_FLASH_Unlock() != HAL_OK) {
            bad = 1; /* unlock rejected */
        } else {
            if ((FLASH->CR & FLASH_CR_LOCK) != 0u) { /* read-only (A3 exemption) */
                bad = 1; /* LOCK failed to clear */
            }
            HAL_FLASH_Lock();
            if ((FLASH->CR & FLASH_CR_LOCK) == 0u) { /* read-only (A3 exemption) */
                bad = 1; /* re-lock rejected */
            }
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
        /* the erase function already reported its own failure detail */
        if (bsp_flash_iap_last_report()[0] == 0) {
            report_set("(erase call failed)");
        }
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
                report_set("(sweep: not FF)");
                return (int)BSP_FL_ST2_ERASE; /* erase incomplete */
            }
            remaining--;
        }
    }
    if (bsp_flash_iap_program_words(BSP_FLASH_PARAM_ADDR,
                                    param_test_words, 8u) != 0) {
        report_set("(program failed)");
        return (int)BSP_FL_ST2_ERASE;
    }
    {
        uint32_t back[8];
        uint32_t k;
        if (bsp_flash_iap_read_words(BSP_FLASH_PARAM_ADDR, back, 8u) != 0) {
            report_set("(readback failed)");
            return (int)BSP_FL_ST2_ERASE;
        }
        for (k = 0u; k < 8u; k++) {
            if (back[k] != param_test_words[k]) {
                report_set("(compare mismatch)");
                return (int)BSP_FL_ST2_ERASE; /* compare mismatch */
            }
        }
        report_set("(erase ok, param 8w kept)");
    }

    /* Step 3: final lock proof (and hygiene: the module NEVER leaves the
     * flash writable behind a menu command). HAL_FLASH_Lock is one CR
     * write -- no interrupt window needed for it. */
    if ((FLASH->CR & FLASH_CR_LOCK) == 0u) { /* read-only (A3 exemption) */
        HAL_FLASH_Lock();
        if ((FLASH->CR & FLASH_CR_LOCK) == 0u) { /* read-only (A3 exemption) */
            return (int)BSP_FL_ST3_LOCK;
        }
    }
    return 0; /* all steps passed */
}

/** @} */
