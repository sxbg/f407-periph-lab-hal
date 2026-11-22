/**
 * @file bsp_wdg.h
 * @brief IWDG + WWDG + reset-source flags (HAL edition of the register-
 *        level bsp_wdg) -- verify-only init, feed discipline, the
 *        no-feed reset demo, CSR reset-cause decode.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_wdg; the IWDG/WWDG
 * contrast walkthrough -- KR keys, PVU/RVU shadow sync, the W-window
 * law, the EP15 LSI-tolerance margin lesson -- stays in the old
 * project's file header as the companion reference).
 *
 * SAFETY CONTRACT (INVERTED vs the register edition): both watchdogs
 * ARE in the .ioc and MX_IWDG_Init / MX_WWDG_Init (Core/Src, boot order
 * in main.c) START them before app_main runs. Nothing in this module
 * can start or stop a dog -- IWDG start is one-way from the moment the
 * driver writes KR=0xCCCC, WWDG activation is the one-way WDGA bit.
 * The contract is therefore: bsp_wdg_init() and bsp_wdg_selftest()
 * only VERIFY the generated plans; the module supplies the FEED APIs;
 * the SHELL owns the discipline.
 *
 * FEEDING DISCIPLINE (read before wiring the menu loop):
 *  - IWDG plan (generated): prescaler /64, reload 4095 -> 4096 LSI
 *    ticks = 8.192 s nominal; across the LSI RC 17..47 kHz spread the
 *    real deadline is 5.6 s (fast chip, EP15 design edge) to 15.4 s
 *    (slow chip). The shell must call bsp_wdg_refresh() every loop
 *    pass, and every selftest must finish well inside the 5.6 s fast
 *    edge (all current ones do: can ~30 ms, crc_rng ~50 ms, rtc ~2.6 s
 *    worst case, wdg ~ms).
 *  - WWDG plan (generated): prescaler /8, counter 127 (0x7F), window
 *    64 (0x40), EWI off. RAZOR-WINDOW FINDING (deferred to the .ioc
 *    owner): the refresh-legal strip is counter <= window, but the
 *    reset fires when the counter rolls to 0x3F -- with W = 0x40 the
 *    strip is the SINGLE tick T == 0x40 (780.2 us at 42 MHz/4096/8).
 *    Consequences: (a) no realistic loop can feed it, so the board
 *    resets ~49 ms after every boot until the .ioc relaxes the window
 *    (the register edition's W = 0x70 gives a 48-tick / 37.4 ms strip)
 *    or drops the WWDG; (b) refreshing while the counter runs ABOVE W
 *    resets INSTANTLY (RM0090 ch.22.4.2) -- which is why
 *    bsp_wdg_refresh() feeds the WWDG window-GUARDED. Until the .ioc
 *    is fixed, keep bsp_wdg_refresh()'s WWDG half semantics in mind:
 *    the IWDG half keeps working, the WWDG half mostly stands down.
 */

#ifndef BSP_WDG_H
#define BSP_WDG_H

#include <stdint.h>

/* IWDG timeout plan -- the GENERATED values (Core/Src/iwdg.c), replacing
 * the register edition's PR=4/RLR=999 two-second plan:
 *   tick = divider / f_LSI,  timeout = (RLR+1) x tick
 *   /64 at nominal 32 kHz -> 2.000 ms per tick, 4096 ticks = 8.192 s;
 *   LSI 17..47 kHz -> real reset lands 5.58 s .. 15.42 s. */
#define BSP_WDG_IWDG_TIMEOUT_MS 8192u
#define BSP_WDG_IWDG_PR         4u    /* /64 (IWDG_PRESCALER_64 encodes 4) */
#define BSP_WDG_IWDG_RLR        4095u /* 4096 ticks */

/* WWDG plan -- the GENERATED values (Core/Src/wwdg.c): PCLK1 = 42 MHz,
 * WDGTB=3 -> /8 -> tick = 4096 x 8 / 42 MHz = 780.2 us; counter loaded
 * at 0x7F, reset when it rolls to 0x3F -> 64 ticks = 49.9 ms max;
 * window 0x40 leaves the single-tick legal strip (razor, file header). */
#define BSP_WDG_WWDG_W       64u   /* window value (CFR W[6:0])       */
#define BSP_WDG_WWDG_WDGTB   3u    /* prescaler exponent: 4096 * 2^3  */
#define BSP_WDG_WWDG_COUNTER 127u  /* reload value (CR T[6:0] | WDGA) */

/* Verify the boot-armed plans (handle fields + register read-backs,
 * all read-only): 0 = PASS, -1 = mismatch. Idempotent for menu reruns. */
int bsp_wdg_init(void);

/* Feed the IWDG only (HAL_IWDG_Refresh -- the old KR=0xAAAA write, now
 * inside the driver). Harmless any time: the dog is armed from boot. */
void bsp_wdg_iwdg_feed(void);

/* Feed BOTH dogs, the menu-loop helper for the shell:
 *  - IWDG: unconditional refresh (no window law, just stay < 5.6 s);
 *  - WWDG: window-GUARDED refresh -- a read-only CR probe first, and
 *    the refresh is issued only once the counter has dropped into the
 *    legal strip (T <= W). Refreshing while T > W resets instantly
 *    (RM0090 ch.22.4.2); the guard makes this helper safe at any loop
 *    rate and with any window value. With the generated W = 0x40 the
 *    guard almost never fires (single-tick strip) -- the razor window
 *    itself is an .ioc decision, see the file header. */
void bsp_wdg_refresh(void);

/* SAFETY DEMO (never call from a selftest): the no-feed reset loop.
 * The IWDG is armed at boot; this feeds it ONE last time (full 8.192 s
 * nominal window from NOW), then dies with the feed path deliberately
 * dead. NEVER RETURNS. Reset lands 5.6..15.4 s later; the next boot's
 * boot_diag emits the RTT EV_WATCHDOG_RESET event and 'wdg' shows the
 * flag mechanics. NOTE until the razor window is fixed in the .ioc,
 * the WWDG fires first (~49 ms after boot) and its flag will be the
 * observed reset cause -- the IWDG lesson is masked, not broken. */
void bsp_wdg_reset_demo(void);

/* Snapshot of the reset-cause flags in RCC->CSR (bits 25..31: BOR/PIN/
 * POR/SFT/IWDG/WWDG/LPWR), decoded into bsp_wdg_last_report() and
 * RETURNED raw so a caller can re-decode. Read-only probe (A3
 * exemption). INTENTIONAL CHANGE: no RMVF clear write here (A3 forbids
 * control-register writes; app_main.c already clears the flags at boot
 * via __HAL_RCC_CLEAR_RESET_FLAGS before the menu runs). */
uint32_t bsp_wdg_reset_flags(void);

/* One-line report of the LAST selftest run, for the menu to append to
 * its verdict line (no console dependency). Examples:
 * "(iwdg ok, wwdg razor win)" / "(rst: none)" / "(rst: iwdg)". */
const char *bsp_wdg_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. SAFE by construction: nothing here can start or stop a dog
 * (both run from boot; only reset stops them). Steps (numbering kept):
 *   1 = IWDG plan verify: handle fields + PR/RLR register read-back
 *       (readable reliably precisely BECAUSE the dog is armed -- the
 *       register edition's 2026-11-21 unarmed-stale-read finding)
 *   2 = reset-source mechanics: CSR flag snapshot + decode into the
 *       report ("rst: ..." / "rst: none"). The RMVF clear cycle is
 *       dropped (A3; app_main owns the boot-time clear)
 *   3 = WWDG plan verify: CFR read-back (prescaler/window) + WDGA set
 *       (INVERTED from the register edition's parked-dog proof: the
 *       HAL edition proves the dog RUNNING) + razor-window note */
int bsp_wdg_selftest(void);

#endif /* BSP_WDG_H */
