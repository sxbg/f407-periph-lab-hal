/**
 * @file bsp_crc_rng.h
 * @brief CRC32 hardware unit (AHB1) + true random number generator (AHB2)
 *        (HAL edition of the register-level bsp_crc_rng) -- verify-only
 *        init, CRC/RNG APIs, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_crc_rng; the EP20
 * walkthrough -- the F4 "basic" CRC vs zlib contrast, the word-only APB
 * access law, the RNG FIPS flow -- stays in the old project's file
 * header as the companion reference).
 *
 * OWNERSHIP CHANGE vs the register edition: CRC and RNG ARE in the .ioc
 * now. MX_CRC_Init / MX_RNG_Init (Core/Src/crc.c / rng.c) run at boot:
 * the MspInit hooks own the AHB1 CRCEN / AHB2 RNGEN clock gates, and
 * HAL_RNG_Init owns the CR:RNGEN power bit (interrupt IE stays off --
 * the polling lesson survives). This module only VERIFIES the generated
 * bring-up (read-only probes, A3 exemption) and drives the peripherals
 * through HAL_CRC_* / HAL_RNG_*.
 *
 * CRC API semantics (why the vectors are UNCHANGED from the register
 * edition): the F4 CRC unit eats EXACT 32-bit words, big-endian, fixed
 * MPEG-2 datapath (poly 0x4C11DB7, init 0xFFFFFFFF, no reflection, no
 * xorout). HAL_CRC_Calculate() = CR:RESET + word writes + DR read --
 * the register edition's bsp_crc_rng_crc32() sequence verbatim; the
 * module packs the byte stream into big-endian words with a zero-padded
 * tail exactly as before. Expected values therefore carry over:
 *   "12345678"            -> 0x49E3C2FB (word-aligned, exact MPEG-2)
 *   "123456789" + 3 pads  -> 0xAE24E09D (padded stream; the unpadded
 *                            catalog value 0x0376E6E7 stays unreachable
 *                            word-aligned -- EP20 lesson #3)
 */

#ifndef BSP_CRC_RNG_H
#define BSP_CRC_RNG_H

#include <stdint.h>

/* Verify the CubeMX-generated bring-up: both handles alive, both clock
 * gates on (read-only RCC probes, A3 exemption), RNG state READY.
 * Returns 0 = PASS, -1 = mismatch. Idempotent for menu re-runs. */
int bsp_crc_rng_init(void);

/* CRC-32 over a byte stream through the F407 CRC unit via
 * HAL_CRC_Calculate (reset + first chunk) followed by HAL_CRC_Accumulate
 * (continuation chunks). The unit consumes 32-bit words, big-endian, so
 * an unaligned tail is ZERO-PADDED to a word boundary -- the padding is
 * part of the CRC input (EP20 lesson #3). Returns the raw DR state:
 * MSB-first register state (no reflection, no xorout -- CRC-32/MPEG-2
 * semantics for word-aligned input). len = 0 (or data = NULL) yields
 * 0xFFFFFFFF, the reset state. */
uint32_t bsp_crc_rng_crc32(const uint8_t *data, uint32_t len);

/* One random 32-bit word: HAL_RNG_GenerateRandomNumber polls SR:DRDY
 * internally and returns HAL_ERROR on its timeout. INTENTIONAL NOTE:
 * timeout_ms is kept in the signature for interface parity with the
 * register edition, but the effective DRDY bound is the HAL-internal
 * RNG_TIMEOUT_VALUE (2 ms, tick based) -- DRDY normally sets within two
 * RNG_CLK cycles, so this is invisible in practice.
 * Returns 0 = value delivered, -1 = DRDY never set in time / bad arg. */
int bsp_crc_rng_random(uint32_t timeout_ms, uint32_t *out);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = CRC vectors: hardware DR must equal BOTH the software bitwise
 *       mirror (second computation, determinism criterion) and the
 *       offline constants for "12345678" (0x49E3C2FB) and
 *       "123456789"+pad (0xAE24E09D) -- the SAME vectors as the
 *       register edition, per the HAL_CRC_Calculate mapping above
 *   2 = RNG: 8 consecutive reads (first FIPS discard before them),
 *       no two consecutive values equal (continuous random number
 *       test -- a repeat is a health failure, p = 2^-32 per pair)
 *   3 = RNG health: CECS (clock) and SECS (seed) status bits clear
 *       (read-only flag probes, A3 exemption). On a board whose PLLQ
 *       violates the < 48 MHz RNG_CLK domain, a set CECS here is the
 *       clock-tree finding to escalate, not a driver bug (this build
 *       runs PLLQ = 48 MHz, inside the law). */
int bsp_crc_rng_selftest(void);

#endif /* BSP_CRC_RNG_H */
