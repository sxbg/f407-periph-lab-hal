/**
 ******************************************************************************
 * @file    bsp_crc_rng.c
 * @brief   CRC32 hardware unit + RNG -- teaching module with register-level
 *          init and self-test (EP20 worked example).
 ******************************************************************************
 * @addtogroup BSP_CRC_RNG
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | CRC / RNG in IOC  | NOT PRESENT                          | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | CRC clock         | AHB1 (HCLK = 168 MHz), CRCEN bit12   | RM0090 ch.4
 * | RNG clock         | AHB2, RNGEN bit6; analog part runs   | RM0090 ch.24
 * |                   | from the RNG_CLK domain (PLLQ)       |
 * | NVIC              | both interrupts off                  | polling lesson
 * == REGISTER WALKTHROUGH ==================================================
 * CRC (RM0090 ch.4 -- the F4 "basic" CRC, NOT the F0/F7 configurable one):
 *   RCC->AHB1ENR bit12 (CRCEN): clock gate.
 *   CR bit0 (RESET): sets the internal state to 0xFFFFFFFF and
 *     self-clears -- the only control bit that exists here.
 *   What the F4 unit does NOT have (the teaching contrast): no POLY
 *     register (polynomial is FIXED 0x4C11DB7, x^32+x^26+x^23+...+x+1),
 *     no INIT register (seed fixed 0xFFFFFFFF), no REV_IN/REV_OUT bits
 *     (no reflection control) -- those appeared with the F0/F3/F7
 *     generation. On this silicon the datapath is hardwired.
 *   DR (0x00): write = feed the next EXACTLY 32-bit word (the APB access
 *     is word-only: "The CRC registers have to be accessed by words",
 *     RM0090 4.4); read = the register state. The write stalls the bus
 *     until the 32 shifts finish, so back-to-back writes are safe.
 *   IDR (0x04): an 8-byte free storage cell, unaffected by RESET.
 *   Algorithm the hardware implements, per word W fed from state C:
 *     for bit b = 31..0: fb = C[31] xor W[b];
 *                        C = (C << 1) xor (fb ? 0x04C11DB7 : 0)
 *   That is CRC-32/MPEG-2 EXACTLY (poly 04C11DB7, init FFFFFFFF, MSB-
 *   first, no reflection, no final xor) -- for WORD-ALIGNED input.
 *   THE EP20 LESSON -- vs zlib's crc32() (CRC-32/ISO-HDLC), three deltas:
 *     1. reflection: zlib reflects input and output bits (refin=refout=
 *        true); the hardware shifts MSB-first, untouched bits.
 *     2. final xor: zlib xors the output with 0xFFFFFFFF; the hardware
 *        leaves the raw state in DR.
 *     3. granularity: zlib eats a BYTE stream; the F4 unit eats 32-bit
 *        words, big-endian. An unaligned tail must be zero-padded, and
 *        the padding is part of the CRC input.
 *     Consequence for the classic vector "123456789" (9 bytes):
 *       catalog MPEG-2      = 0x0376E6E7  (9 bytes, unreachable here:
 *                                         word-only access forces 12)
 *       this module (pad 0) = 0xAE24E09D  (12 bytes incl. 3 zero pads)
 *       zlib crc32          = 0xCBF43926  (reflection + xorout: a
 *                                         DIFFERENT algorithm shape)
 *       For an 8-byte vector ("12345678", word-aligned, no padding) the
 *       hardware reproduces the MPEG-2 catalog value 0x49E3C2FB exactly.
 *   Self-test criterion: hardware DR must match a SECOND, bitwise
 *     computation of the same stream (crc32_word/crc_sw_mirror below)
 *     AND the offline constants. Determinism is the contract; if a
 *     silicon revision ever surfaced a reflected readback, step 1 would
 *     FAIL and the recorded DR value is the finding to document.
 * RNG (RM0090 ch.24):
 *   RCC->AHB2ENR bit6 (RNGEN): bus clock gate for the digital part.
 *   CR bit2 (RNGEN): powers the analog entropy source + LFSR + health
 *     detector; bit3 (IE) left 0 -- polling lesson.
 *   SR: DRDY bit0 = a fresh word sits in DR (a DR READ clears it).
 *       CECS bit1 / SECS bit2 = current clock/seed error status (the
 *       sticky interrupt-style twins are CEIS/SEIS). FIPS 140-2 flow:
 *       read only while DRDY=1 and no error bits; DISCARD the first
 *       number after RNGEN=1 (continuous-test preamble).
 *   Caveat documented for this board: the IOC's PLLQ = 84 MHz, while the
 *   RM requires RNG_CLK < 48 MHz (the "48 MHz clocks" domain). The
 *   selftest FAILs step 3 if CECS asserts -- on this board config that
 *   is the finding to escalate (fix = PLLQ <= 48 / 6 ... i.e. a proper
 *   48 MHz PLLQ), not a software workaround.
 * == HOW TO VERIFY =========================================================
 * menu> crc_rng
 *   PASS crc_rng -- CRC matched the mirror + constants on both vectors,
 *   8 RNG words arrived with no consecutive repeats, no error bits.
 * Cross-check offline: python -c "import zlib;
 * print(hex(zlib.crc32(b'123456789')))" -> 0xcbf43926 (the zlib twin,
 * deliberately different for the three reasons above).
 */
#include "bsp_crc_rng.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Fixed datapath constants of the F4 CRC unit (no POLY/INIT registers
 * exist to hold them -- they live in the silicon). */
#define BSP_CRC_POLY 0x04C11DB7u

/* Test vectors: '1'..'8' = 0x31..0x38, '9' = 0x39. Constants computed
 * offline with the bitwise MPEG-2 algorithm (see walkthrough table). */
#define BSP_CRC_VEC8_1 0x31323334u /* "1234" big-endian word */
#define BSP_CRC_VEC8_2 0x35363738u /* "5678" big-endian word */
#define BSP_CRC_VEC8_EXPECT 0x49E3C2FBu /* MPEG-2 of "12345678" */
#define BSP_CRC_VEC9_3 0x39000000u /* "9" + 3 zero-pad bytes */
#define BSP_CRC_VEC9_PAD_EXPECT 0xAE24E09Du /* MPEG-2 of 9B + 3 pads */

/* Self-test step numbers (the value bsp_crc_rng_selftest returns). */
#define BSP_CRC_ST1_VECTORS 1u /* CRC hw vs mirror vs constants */
#define BSP_CRC_ST2_RNG_SEQ 2u /* 8 DRDY-gated reads, no repeats */
#define BSP_CRC_ST3_RNG_ERR 3u /* CECS / SECS clear */

/* Timeout window for every DRDY wait (SysTick COUNTFLAG, 1 ms). DRDY
 * sets after ~2 RNG_CLK periods; 10 ms is orders of magnitude generous. */
#define BSP_CRC_DRDY_TO_MS 10u

/* Spin cap between SysTick wraps: SysTick-dead guard (spec #5). */
#define BSP_CRC_SYSTICK_GUARD 2000000u

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

/* Same stream the hardware sees (word-only access, big-endian words,
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

/* Wait RNG SR DRDY=1 within timeout_ms (SysTick COUNTFLAG polling).
 * Returns 0 = DRDY set, -1 = timeout, -2 = SysTick dead (bounded exit). */
static int rng_wait_drdy(uint32_t timeout_ms)
{
    uint32_t guard = BSP_CRC_SYSTICK_GUARD;
    for (;;) {
        if ((RNG->SR & RNG_SR_DRDY) != 0u) {
            return 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            if (timeout_ms > 0u) {
                timeout_ms--;
                guard = BSP_CRC_SYSTICK_GUARD; /* alive: refill the cap */
            } else {
                return -1; /* DRDY never came */
            }
        }
        if (--guard == 0u) {
            return -2; /* SysTick dead: bounded exit */
        }
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_crc_rng_init(void)
{
    /* CRC lives on AHB1 (CRCEN bit12), RNG on AHB2 (RNGEN bit6) -- two
     * different bus clock gates for two different domains (walkthrough).
     * Read-backs stall the core until each clock is live. */
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    (void)RCC->AHB1ENR;
    (void)RCC->AHB2ENR;

    /* RNG analog part on, interrupt off (polling lesson). HAL equivalent:
     * HAL_RNG_Init(&hrng) -- which also just sets CR:RNGEN for this IP. */
    RNG->CR = RNG_CR_RNGEN; /* IE=0 */

    /* CRC needs no setup beyond the gate: every bsp_crc_rng_crc32() call
     * strokes RESET first, so no state can leak between runs. */
    if (((RCC->AHB1ENR & RCC_AHB1ENR_CRCEN) == 0u) ||
        ((RCC->AHB2ENR & RCC_AHB2ENR_RNGEN) == 0u) ||
        ((RNG->CR & RNG_CR_RNGEN) == 0u)) {
        return -1;
    }
    return 0;
}

uint32_t bsp_crc_rng_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t i = 0u;

    CRC->CR = CRC_CR_RESET; /* state := 0xFFFFFFFF, bit self-clears */
    while (i < len) {
        uint32_t word = 0u;
        uint32_t k;
        /* Pack 4 bytes into one BIG-ENDIAN word ('1'='0x31' ends up in
         * the MSB lane). The tail pads with zeros -- the pad bytes are
         * PART of the CRC input (walkthrough, EP20 lesson #3). The write
         * stalls the bus until the unit finishes the 32 shifts. */
        for (k = 0u; k < 4u; k++) {
            word <<= 8;
            if (i < len) {
                word |= data[i];
                i++;
            }
        }
        CRC->DR = word;
    }
    return CRC->DR; /* raw register state: MPEG-2 semantics, no xorout */
}

int bsp_crc_rng_random(uint32_t timeout_ms, uint32_t *out)
{
    if (out == 0) {
        return -1;
    }
    if (rng_wait_drdy(timeout_ms) != 0) {
        return -1; /* no fresh word in time */
    }
    *out = RNG->DR; /* the read itself clears DRDY for the next round */
    return 0;
}

int bsp_crc_rng_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
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
     * belongs to the unpadded stream (walkthrough table). */
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
     * reads, each DRDY-gated, each must differ from its predecessor
     * (continuous random number test -- a repeat is a health failure,
     * not "random chance": p = 2^-32 per pair). */
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
     * error) must both read 0. On THIS board note the PLLQ=84 MHz
     * caveat (walkthrough): a set CECS here is a clock-tree finding,
     * not a driver bug. */
    if ((RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0u) {
        return (int)BSP_CRC_ST3_RNG_ERR;
    }
    return 0; /* all steps passed */
}

/** @} */
