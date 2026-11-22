/**
 * @file bsp_crc_rng.h
 * @brief CRC32 hardware unit (AHB1) + true random number generator (AHB2)
 *        -- register-level init, CRC/RNG APIs, self-test.
 *
 * Teaching module (P3b wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough (EP20 zlib-contrast lesson) /
 * verification recipe live in the bsp_crc_rng.c file header.
 *
 * Ownership rule: neither CRC nor RNG is in the IOC (knowledge #13:
 * adapter-owned peripherals stay out) -- this module is their FULL owner:
 * RCC AHB1ENR CRCEN / AHB2ENR RNGEN gates, CRC->CR/DR and RNG->CR/SR/DR.
 * No CubeMX code will ever conflict.
 */
#ifndef BSP_CRC_RNG_H
#define BSP_CRC_RNG_H

#include <stdint.h>

/* Register-level init: AHB1 CRCEN + AHB2 RNGEN clock gates, RNG enabled
 * (CR RNGEN=1, interrupt IE=0 -- polling lesson), CRC left at reset
 * (each bsp_crc_rng_crc32() call resets it anyway).
 * Returns 0 = PASS (gate read-back), -1 = mismatch. */
int bsp_crc_rng_init(void);

/* CRC-32 over a byte stream using the F407 CRC unit. The unit consumes
 * EXACT 32-bit words (word-only APB access, RM0090 ch.4), big-endian, so
 * the tail of an unaligned stream is ZERO-PADDED to a word boundary --
 * the padding is part of the CRC input (see the walkthrough for why
 * "123456789" therefore does NOT give the catalog value). Returns the raw
 * DR readback: the MSB-first register state (no reflection, no xorout --
 * CRC-32/MPEG-2 semantics for word-aligned input). */
uint32_t bsp_crc_rng_crc32(const uint8_t *data, uint32_t len);

/* One random 32-bit word: wait SR DRDY=1 (bounded by timeout_ms), then
 * read DR (a DR read clears DRDY -- the next read waits again). Returns
 * 0 = value delivered, -1 = DRDY never set in time. */
int bsp_crc_rng_random(uint32_t timeout_ms, uint32_t *out);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = CRC vectors: hardware DR must equal BOTH the software bitwise
 *       mirror (second computation, determinism criterion) and the
 *       offline-computed constants for
 *       "12345678"   (word-aligned -> exact MPEG-2 0x49E3C2FB) and
 *       "123456789"+pad (-> 0xAE24E09D; the unpadded catalog value
 *       0x0376E6E7 is NOT reachable word-aligned -- walkthrough)
 *   2 = RNG: 8 consecutive DR reads (first FIPS discard before them),
 *       each after a DRDY wait, no two consecutive values equal
 *   3 = RNG health: CECS (clock) and SECS (seed) error bits clear */
int bsp_crc_rng_selftest(void);

#endif /* BSP_CRC_RNG_H */
