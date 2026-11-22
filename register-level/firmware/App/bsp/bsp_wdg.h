/**
 * @file bsp_wdg.h
 * @brief IWDG + WWDG + reset-source flags -- teaching module: both watchdogs
 *        configured at register level, CSR reset-cause decode, and the
 *        no-feed reset-loop demo kept OUT of the safe selftest.
 *
 * Teaching module (P3c wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough / verification recipe live in the
 * bsp_wdg.c file header.
 *
 * Ownership rule: neither watchdog is in the IOC (knowledge #13: adapter-
 * owned peripherals stay out). This module is the FULL owner of RCC->CSR
 * LSION/LSIRDY + RMVF flag ops, IWDG->KR/PR/RLR/SR, WWDG->CR/CFR/SR and its
 * APB1 clock gate. No CubeMX/HAL watchdog code can ever conflict.
 *
 * SAFETY CONTRACT (read before calling anything):
 *  - bsp_wdg_init() and bsp_wdg_selftest() NEVER start either watchdog.
 *    IWDG starts only via KR=0xCCCC (one-way, runs until reset) and WWDG
 *    only via CR.WDGA (one-way). Both "go live" paths live in
 *    bsp_wdg_reset_demo() ONLY -- a menu demo command with a printed
 *    warning, never inside a selftest step.
 */
#ifndef BSP_WDG_H
#define BSP_WDG_H

#include <stdint.h>

/* IWDG timeout plan (see the .c walkthrough for the full derivation):
 *   tick = divider(PRR) / f_LSI,  timeout = (RLR+1) x tick
 *   PR=4 -> divider 64; nominal LSI 32 kHz -> 2.000 ms per tick;
 *   RLR=999 -> 1000 ticks = 8.19 s nominal (5.6-15.4 s over LSI tolerance) nominal. LSI RC tolerance 17..47 kHz
 *   (EP15 lesson) -> real reset lands 1.36 s .. 3.77 s. Any feed loop
 *   must therefore be comfortably FASTER than 1.36 s. */
#define BSP_WDG_IWDG_TIMEOUT_MS 2000u
#define BSP_WDG_IWDG_PR         4u   /* div /64 */
#define BSP_WDG_IWDG_RLR        999u /* 1000 ticks */

/* WWDG plan: PCLK1 = 42 MHz, WDGTB=3 -> /8 -> tick = 4096*8/42 MHz =
 * 780.2 us. Counter loaded at 0x7F: reset after (127-63) ticks = 50.0 ms;
 * window W=0x70: refresh is LEGAL only while T < 0x70, i.e. in the
 * 48-tick (37.4 ms) strip ending at T=0x40 -- refresh too early resets. */
#define BSP_WDG_WWDG_W     0x70u /* window value (CRF W[6:0]) */
#define BSP_WDG_WWDG_WDGTB 3u    /* prescaler bits: 4096 * 2^3 */

/* Register-level config of both watchdogs WITHOUT starting either:
 *   IWDG: LSI on + LSIRDY wait, KR=0x5555 unlock, PR/RLR programmed with
 *         the PVU/RVU sync waits, KR=0xFFFF re-protect (counter idle).
 *   WWDG: APB1 WWDGEN clock gate, CFR window+prescaler written (WDGA
 *         stays 0 -- the counter does not run).
 * Returns 0 = PASS (PR/RLR/CFR read-back matches), -1 = mismatch or an
 * LSI/LSIRDY timeout. */
int bsp_wdg_init(void);

/* Feed the IWDG (KR=0xAAAA -- reloads the counter from RLR). Only legal
 * after bsp_wdg_reset_demo()/iwdg_start has armed the dog; feeding while
 * idle is harmless (a write to KR is never an error). */
void bsp_wdg_iwdg_feed(void);

/* SAFETY DEMO (never call from a selftest): configures + STARTS the IWDG
 * with the ~2 s plan above, one final feed, then waits for the reset with
 * the feed path deliberately dead. The board resets in 1.4..3.8 s; on the
 * next boot app_main turns RCC->CSR IWDGRSTF into the RTT
 * EV_WATCHDOG_RESET event and this module's 'wdg' command shows the
 * flag/clear mechanics. NEVER RETURNS. */
void bsp_wdg_reset_demo(void);

/* Snapshot of the reset-cause flags in RCC->CSR (bits 25..31: BOR/PIN/
 * POR/SFT/IWDG/WWDG/LPWR), decoded into bsp_wdg_last_report() and RETURNED
 * raw so a caller can re-decode. Writing RCC->CSR RMVF=1 clears them all
 * (done by the selftest AFTER the snapshot). */
uint32_t bsp_wdg_reset_flags(void);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line (the module itself has no console dependency). Examples:
 * "(iwdg PR/RLR ok)" / "(rst: pin+por, flags cleared)" / "(rst: none)". */
const char *bsp_wdg_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. SAFE by construction: no watchdog is ever started here.
 * Steps:
 *   1 = IWDG config write + read-back (PR=4 / RLR=999 through the
 *       unlock-PVU/RVU-wait sequence, then KR=0xFFFF re-protect)
 *   2 = reset-source mechanics: RCC->CSR flag snapshot (goes into the
 *       report), RMVF clear write, then ALL seven *RSTF bits must read 0
 *   3 = WWDG config write + read-back (CFR W=0x70 / WDGTB=/8, WDGA left
 *       0, EWI left 0) */
int bsp_wdg_selftest(void);

#endif /* BSP_WDG_H */
