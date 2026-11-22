/**
 ******************************************************************************
 * @file    bsp_crc_rng.c
 * @brief   CRC32 hardware unit + RNG BSP (HAL edition) -- see
 *          bsp_crc_rng.h for the contract and the HAL mapping.
 ******************************************************************************
 * @addtogroup BSP_CRC_RNG
 * @{
 *
 * == HAL EDITION MAP (register edition -> here) ===========================
 * RCC->AHB1ENR CRCEN / RCC->AHB2ENR RNGEN    -> MspInit hooks (Core/Src/
 *                                               crc.c / rng.c), boot time.
 * RNG->CR = RNG_CR_RNGEN (IE=0)              -> HAL_RNG_Init (same CR write
 *                                               inside the driver).
 * CRC->CR = CRC_CR_RESET; CRC->DR = word     -> HAL_CRC_Calculate(&hcrc,
 *   ...; DR readback                            words, n) -- reset + word
 *                                               feeds + DR read.
 * (continuation without a second RESET)      -> HAL_CRC_Accumulate.
 * RNG SR DRDY poll + DR read                 -> HAL_RNG_GenerateRandomNumber
 *                                               (DRDY poll + DR read inside).
 * RNG SR CECS/SECS health read               -> __HAL_RNG_GET_FLAG probes
 *                                               (read-only, A3 exemption).
 *
 * == CUBEMX CONFIG (in f407-periph-lab-hal.ioc now) ========================
 * | Field       | Value                        | Note
 * |-------------|------------------------------|---------------------------
 * | CRC         | default (fixed MPEG-2        | the F4 unit has no POLY/INIT
 * |             | datapath, nothing to config) | registers to configure
 * | RNG         | default, no interrupt        | IE=0 polling lesson
 * | RNG clock   | PLLQ = 7 @ 25 MHz HSE        | 48 MHz, inside the
 * |             | -> 48 MHz                    | < 48 MHz RM law
 * | NVIC        | both interrupts off          | polling lesson
 * == HOW TO VERIFY =========================================================
 * menu> crc_rng
 *   PASS crc_rng -- CRC matched the mirror + constants on both vectors,
 *   8 RNG words arrived with no consecutive repeats, no error bits.
 * Cross-check offline: python -c "import zlib;
 * print(hex(zlib.crc32(b'123456789')))" -> 0xcbf43926 -- the zlib twin,
 * deliberately a DIFFERENT algorithm shape (reflection + xorout +
 * byte granularity; walkthrough in the register edition).
 */
#include "bsp_crc_rng.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "crc.h" /* hcrc from MX_CRC_Init (CubeMX-generated) */
#include "rng.h" /* hrng from MX_RNG_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* Fixed datapath constant of the F4 CRC unit (no POLY register exists
 * to hold it -- it lives in the silicon; kept for the software mirror). */
#define BSP_CRC_POLY 0x04C11DB7u

/* Test vectors: '1'..'8' = 0x31..0x38, '9' = 0x39. Constants computed
 * offline with the bitwise MPEG-2 algorithm; UNCHANGED from the register
 * edition because HAL_CRC_Calculate performs the same reset + big-endian
 * word feed the register code did (header note). */
#define BSP_CRC_VEC8_EXPECT    0x49E3C2FBu /* MPEG-2 of "12345678"       */
#define BSP_CRC_VEC9_PAD_EXPECT 0xAE24E09Du /* MPEG-2 of 9B + 3 zero pads */

/* Word-chunk size for the streaming CRC (stack buffer, no malloc). */
#define BSP_CRC_CHUNK_WORDS 8u

/* Timeout passed to bsp_crc_rng_random: interface parity with the
 * register edition (the effective bound is the HAL-internal one). */
#define BSP_CRC_DRDY_TO_MS 10u

/* Self-test step numbers (the value bsp_crc_rng_selftest returns). */
#define BSP_CRC_ST1_VECTORS 1u /* CRC hw vs mirror vs constants */
#define BSP_CRC_ST2_RNG_SEQ 2u /* 8 reads, no consecutive repeat */
#define BSP_CRC_ST3_RNG_ERR 3u /* CECS / SECS clear */

/* ---- private helpers -------------------------------------------------------- */

/* ONE 32-bit word through the hardware's exact shift sequence: MSB
 * first, feedback = top bit xor data bit, no reflection. This is the
 * software mirror the selftest compares against (second computation). */
static uint32_t crc32_word(uint32_t state, uint32_t word)
{
    int b;
    for (b = 31; b >= 0; b--) {
        const uint32_t fb = ((state >> 31) ^ (word >> b)) & 1u;
        state = (state << 1) ^ ((fb != 0u) ? BSP_CRC_POLY : 0u);
    }
    return state;
}

/* The same padded stream the hardware consumes (big-endian words,
 * zero-padded tail), pushed through the mirror. len = 0 -> 0xFFFFFFFF
 * (the reset state, no words fed). */
static uint32_t crc_sw_mirror(const uint8_t *data, uint32_t len)
{
    uint32_t state = 0xFFFFFFFFu;
    uint32_t i = 0u;
    while (i < len) {
        uint32_t word = 0u;
        uint32_t k;
        for (k = 0u; k < 4u; k++) {
            word <<= 8; /* pad byte (0) shifts in once the stream ends */
            if (i < len) {
                word |= data[i];
                i++;
            }
        }
        state = crc32_word(state, word);
    }
    return state;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_crc_rng_init(void)
{
    /* Read-only verification (A3 exemption): MX_CRC_Init / MX_RNG_Init
     * already enabled the clocks and powered the RNG at boot -- this
     * module proves the bring-up instead of redoing it. */
    if ((hcrc.Instance == NULL) || (hrng.Instance == NULL)) {
        return -1; /* CubeMX init never ran */
    }
    if (__HAL_RCC_CRC_IS_CLK_ENABLED() == 0u) {
        return -1; /* AHB1 CRCEN gone */
    }
    if (__HAL_RCC_RNG_IS_CLK_ENABLED() == 0u) {
        return -1; /* AHB2 RNGEN gone */
    }
    if (hrng.State != HAL_RNG_STATE_READY) {
        return -1; /* driver not idle (HAL_RNG_Init failed at boot?) */
    }
    return 0;
}

uint32_t bsp_crc_rng_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t words[BSP_CRC_CHUNK_WORDS];
    uint32_t i = 0u;
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t first = 1u; /* first chunk resets the unit, rest continue */

    if (data == 0) {
        len = 0u; /* new guard vs the register edition: NULL stays safe */
    }
    while ((i < len) || (first != 0u)) {
        uint32_t n = 0u;
        while ((n < BSP_CRC_CHUNK_WORDS) && (i < len)) {
            uint32_t word = 0u;
            uint32_t k;
            /* Pack 4 bytes into one BIG-ENDIAN word ('1' = 0x31 lands in
             * the MSB lane); the tail pads with zeros -- pad bytes are
             * PART of the CRC input (EP20 lesson #3). */
            for (k = 0u; k < 4u; k++) {
                word <<= 8;
                if (i < len) {
                    word |= data[i];
                    i++;
                }
            }
            words[n] = word;
            n++;
        }
        if (first != 0u) {
            /* HAL_CRC_Calculate = CR:RESET + word feeds + DR read: the
             * register edition's sequence verbatim (driver-internal
             * register writes are the HAL's business, not ours). */
            crc = HAL_CRC_Calculate(&hcrc, words, n);
            first = 0u;
        } else {
            crc = HAL_CRC_Accumulate(&hcrc, words, n); /* continue stream */
        }
    }
    return crc; /* raw DR state: MPEG-2 semantics, no xorout */
}

int bsp_crc_rng_random(uint32_t timeout_ms, uint32_t *out)
{
    if (out == 0) {
        return -1;
    }
    /* Interface-parity note (header): the effective DRDY window is the
     * HAL's internal RNG_TIMEOUT_VALUE, not this parameter. */
    (void)timeout_ms;
    if (HAL_RNG_GenerateRandomNumber(&hrng, out) != HAL_OK) {
        return -1; /* DRDY never set inside the HAL window */
    }
    return 0; /* *out holds a fresh word; its DR read rearmed DRDY */
}

int bsp_crc_rng_selftest(void)
{
    /* Fresh init verification first (idempotent, menu-driven pattern). */
    if (bsp_crc_rng_init() != 0) {
        return (int)BSP_CRC_ST1_VECTORS;
    }

    /* Step 1: CRC vectors -- the three-way contract. Vector A ("12345678",
     * 8 bytes, word-aligned): hardware must equal the software mirror AND
     * the offline MPEG-2 constant 0x49E3C2FB (exact, no padding). */
    {
        static const uint8_t vec8[8] = {
            0x31u, 0x32u, 0x33u, 0x34u, 0x35u, 0x36u, 0x37u, 0x38u
        };
        const uint32_t hw = bsp_crc_rng_crc32(vec8, 8u);
        if ((hw != crc_sw_mirror(vec8, 8u)) ||
            (hw != BSP_CRC_VEC8_EXPECT)) {
            return (int)BSP_CRC_ST1_VECTORS;
        }
    }
    /* Vector B ("123456789", 9 bytes -> one zero-padded tail word): hw
     * must equal the mirror of the SAME padded stream AND 0xAE24E09D.
     * The catalog value 0x0376E6E7 is deliberately NOT asserted -- it
     * belongs to the unpadded stream (walkthrough table). This vector
     * also exercises the chunk boundary: 9 bytes = 2 words + 1 pad word,
     * i.e. Calculate (reset) + Accumulate (continuation). */
    {
        static const uint8_t vec9[9] = {
            0x31u, 0x32u, 0x33u, 0x34u, 0x35u, 0x36u, 0x37u, 0x38u, 0x39u
        };
        const uint32_t hw = bsp_crc_rng_crc32(vec9, 9u);
        if ((hw != crc_sw_mirror(vec9, 9u)) ||
            (hw != BSP_CRC_VEC9_PAD_EXPECT)) {
            return (int)BSP_CRC_ST1_VECTORS;
        }
    }

    /* Step 2: RNG sequence. FIPS 140-2 preamble: discard the first word
     * after RNGEN (it seeds the continuous comparison only). Then 8
     * reads, each must differ from its predecessor (continuous random
     * number test -- a repeat is a health failure, not "random chance":
     * p = 2^-32 per pair). */
    {
        uint32_t prev = 0u;
        uint32_t val = 0u;
        int i;
        if (bsp_crc_rng_random(BSP_CRC_DRDY_TO_MS, &prev) != 0) {
            return (int)BSP_CRC_ST2_RNG_SEQ; /* discarded preamble */
        }
        for (i = 0; i < 8; i++) {
            if (bsp_crc_rng_random(BSP_CRC_DRDY_TO_MS, &val) != 0) {
                return (int)BSP_CRC_ST2_RNG_SEQ; /* DRDY timeout */
            }
            if (val == prev) {
                return (int)BSP_CRC_ST2_RNG_SEQ; /* FIPS continuous test */
            }
            prev = val;
        }
    }

    /* Step 3: RNG health bits -- CECS (clock error) and SECS (seed
     * error) must both read 0. Read-only flag probes (A3 exemption):
     * the RNG flags are documented read-only in the HAL header. */
    if ((__HAL_RNG_GET_FLAG(&hrng, RNG_FLAG_CECS) != RESET) ||
        (__HAL_RNG_GET_FLAG(&hrng, RNG_FLAG_SECS) != RESET)) {
        return (int)BSP_CRC_ST3_RNG_ERR;
    }
    return 0; /* all steps passed */
}

/** @} */
