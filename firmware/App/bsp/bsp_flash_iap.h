/**
 * @file bsp_flash_iap.h
 * @brief Internal flash IAP (in-application programming) -- teaching module:
 *        HAL unlock, sector erase, word programming, with the erase/program
 *        window HARD-LIMITED to the last-sector parameter page. (HAL edition
 *        of projects/f407-periph-lab bsp_flash_iap.)
 *
 * HAL MAPPING NOTE (register edition -> HAL edition):
 *   register edition              | HAL edition
 *   ------------------------------+-----------------------------------------
 *   FLASH->KEYR key sequence      | HAL_FLASH_Unlock()/HAL_FLASH_Lock()
 *   CR SER+SNB+PSIZE+STRT, BSY poll| HAL_FLASHEx_Erase() with
 *                                 | FLASH_EraseInitTypeDef{TypeErase =
 *                                 | FLASH_TYPEERASE_SECTORS, Sector =
 *                                 | BSP_FLASH_LAST_SECTOR, NbSectors = 1,
 *                                 | VoltageRange = FLASH_VOLTAGE_RANGE_3}
 *   CR PG+PSIZE=word, word write, | HAL_FLASH_Program(FLASH_TYPEPROGRAM_
 *     BSY poll per word           |   WORD, addr, data)
 *   SR error-bit audit            | the HAL call's HAL_OK/ErrorCode result
 *   direct memory reads           | unchanged (allowed read: flash is
 *                                 | memory-mapped; reading needs no unlock)
 *
 * Teaching module (P3c wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / walkthrough / verification recipe live in the
 * bsp_flash_iap.c file header.
 *
 * Ownership rule: the flash interface is NOT in the IOC (knowledge #13:
 * adapter-owned peripherals stay out -- there is no CubeMX page for it at
 * all). This module is the FULL owner of the flash controller through the
 * HAL_FLASH_* API only (A3 rule: no KEYR/CR/SR writes here; CR/SR/ACR are
 * only READ, with exemption comments). ACR (latency/caches) belongs to the
 * boot clock init and is only READ here.
 *
 * SECTOR NUMBER NOTE (this part is 512 KB): the last sector on THIS
 * STM32F407VE is SECTOR 7 (0x0806 0000-0x0807 FFFF, 128 KB). The "sector
 * 11" name from HAL examples is the 1 MB-density table (sectors 8..11 do
 * not exist here); passing 11 to HAL_FLASHEx_Erase on this part would be
 * an invalid SNB. The constants below are the hardware truth.
 *
 * SAFETY CONTRACT (read before calling anything): the CPU executes from
 * this same flash. Erasing/programming a sector the code runs from stalls
 * or corrupts the fetch -> instant crash. Therefore:
 *  - bsp_flash_iap_erase_sector() accepts ONLY the last sector (7 on this
 *    512 KB F407VE: 0x08060000..0x0807FFFF) -- every other sector number
 *    is REFUSED, -2. No selftest step, no API call can touch code sectors.
 *  - bsp_flash_iap_program_words() accepts ONLY addresses inside the
 *    1 KB parameter page at the very tail (BSP_FLASH_PARAM_ADDR..+1KB).
 *  - Every flash operation runs with interrupts masked (PRIMASK save/
 *    restore) for the shortest possible window -- see the walkthrough.
 */
#ifndef BSP_FLASH_IAP_H
#define BSP_FLASH_IAP_H

#include <stdint.h>

/* F407VE flash map (RM0090 Table 5, 512 KB density -- NOT the 1 MB table):
 * 4 x 16 KB (sectors 0..3) + 1 x 64 KB (sector 4) + 3 x 128 KB (sectors
 * 5..7). The last sector on THIS part is SECTOR 7 (0x0806 0000-0x0807
 * FFFF); the "Sector 11" name from the 1 MB tables does not exist here. */
#define BSP_FLASH_SECTOR_COUNT 8u
#define BSP_FLASH_LAST_SECTOR  7u
#define BSP_FLASH_LAST_BASE    0x08060000u
#define BSP_FLASH_LAST_SIZE    (128u * 1024u)

/* The teaching parameter page: the LAST 1 KB of the last sector. All
 * program/erase traffic of this module is confined to it. */
#define BSP_FLASH_PARAM_ADDR 0x0807FC00u
#define BSP_FLASH_PARAM_SIZE 1024u
#define BSP_FLASH_PARAM_WORDS (BSP_FLASH_PARAM_SIZE / 4u)

/* Datasheet erase/program windows (kept from the register edition). The
 * HAL's internal BSY wait is bounded by FLASH_TIMEOUT_VALUE (50 s), which
 * covers both with margin; these constants document the physics the wait
 * must tolerate: 128 KB sector erase typ 1.3 s max 2.6 s at word PSIZE
 * (voltage range 3), ~35 us per word program. */
#define BSP_FLASH_ERASE_TIMEOUT_MS 6000u
#define BSP_FLASH_PROG_TIMEOUT_MS 100u

/* Check-only init: reports the current LOCK state + ACR latency in the
 * last-report buffer, changes nothing. Returns 0 when the flash
 * controller answers (ACR/CR readable), -1 otherwise. */
int bsp_flash_iap_init(void);

/* Erase the last sector (BSP_FLASH_LAST_SECTOR) so its whole 128 KB reads
 * 0xFFFFFFFF afterwards. Any other sector number is refused (-2) -- the
 * code lives in sectors 0..6, see the safety contract. Bounded wait
 * inside HAL_FLASHEx_Erase. Returns 0 = erased, -1 = HAL error /
 * timeout / sector error flag, -2 = refused. */
int bsp_flash_iap_erase_last_sector(void);

/* Program n 32-bit words at 'addr'. addr must be word-aligned and INSIDE
 * the parameter page (refused otherwise, -2). Each word may only flip 1->0
 * (flash physics); the page must have been erased first.
 * Returns 0 = programmed, -1 = HAL error / timeout / alignment,
 * -2 = refused (outside the page). */
int bsp_flash_iap_program_words(uint32_t addr, const uint32_t *src,
                                uint32_t n);

/* Read n words from 'addr' into dst (plain memory reads -- flash is
 * memory-mapped; the teaching point is that reading needs NO unlock).
 * Returns 0, or -1 on alignment/range error. */
int bsp_flash_iap_read_words(uint32_t addr, uint32_t *dst, uint32_t n);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line (the module itself has no console dependency). Examples:
 * "(lock ok)" / "(erase ok, FF verified)" / "(param 8w kept)". */
const char *bsp_flash_iap_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps (all confined to the last sector / parameter page):
 *   1 = HAL_FLASH_Unlock -> LOCK reads 0 -> HAL_FLASH_Lock -> LOCK
 *       reads 1
 *   2 = erase last sector -> the FULL 128 KB reads FF -> program 8 words
 *       into the parameter page -> read back and compare -> the 8 words
 *       are deliberately LEFT programmed (power-off retention demo for
 *       the next boot)
 *   3 = re-lock -> LOCK reads 1
 * WARNING (walkthrough): interrupt-driven services are stalled during
 *   each erase/program window (PRIMASK up, seconds for the erase) -- the
 *   log heartbeat pauses visibly; that is the documented cost. */
int bsp_flash_iap_selftest(void);

#endif /* BSP_FLASH_IAP_H */
