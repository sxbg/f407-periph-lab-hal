/**
 ******************************************************************************
 * @file    bsp_dma.c
 * @brief   DMA2 teaching module: Stream6 memory-to-memory + Stream7/Ch4
 *          USART1_TX -- register-level, RM0090 10.3.17 procedure spelled
 *          out step by step (EP07).
 ******************************************************************************
 * @addtogroup BSP_DMA
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | DMA in IOC        | NOT PRESENT as an IP                 | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | peripherals stay
 * |                   |                                      | out of the .ioc
 * | Stream allocation | DMA2_S6 = mem-to-mem demo (idle      | project DMA
 * |                   | otherwise), DMA2_S7 + Channel 4 =    | allocation table
 * |                   | USART1_TX (idle otherwise)           | (knowledge #13)
 * | DMA2 only for M2M | RM 10.3.6: "Only the DMA2 controller | DMA1 cannot
 * |                   | is able to perform memory-to-memory" | M2M at all
 * | NVIC              | TCIE/TEIE off on both streams        | flags polled --
 * |                   |                                      | polling lesson
 * | (USRT1_TX note)   | bsp_uart OWNS USART1 (polling); this | DMAT set -> send
 * |                   | module only arms DMAT for one burst  | -> clear DMAT,
 * |                   | and restores it                      | ownership intact
 *
 * == REGISTER WALKTHROUGH (RM0090 ch.10 "DMA controller") ==================
 * Clock gate: DMA1/DMA2 hang on AHB1 -- RCC->AHB1ENR bit22 (DMA2EN).
 * Stream architecture (10.3.5): 8 streams x 8 channels per controller.
 *   A CHANNEL (CHSEL[2:0] in CR) names the REQUEST SOURCE (Table 43, e.g.
 *   DMA2 Ch4 = USART1_TX); a STREAM is the engine that serves it. Two
 *   streams can map the same channel -- only one may run at a time
 *   (request collision), which is why the allocation table pins S7.
 * Direction (10.3.6, CR DIR[1:0], Table 45): 00 = P2M (PAR source), 01 =
 *   M2P (M0AR source), 10 = MEMORY-TO-MEMORY (PAR source, M0AR dest).
 *   NOTE vs the F1 generation: the F4 CR has NO MEM2MEM control bit --
 *   M2M is purely DIR=10. In M2M no peripheral request is involved at
 *   all: EN=1 starts the burst immediately (the "stream enable = the
 *   request" trick), and only DMA2 can do it (RM quote above).
 * THE CONFIG PROCEDURE (10.3.17) -- this module is its step-by-step demo:
 *   1. EN=0, then READ EN until 0 (a disable only lands after the current
 *      transfer; writing EN=0 is not immediately effective).
 *   2. clear the stream's event flags (LISR/HISR twins are write-1-clear
 *      in LIFCR/HIFCR) -- stale TCIF would fake an instant "done".
 *   3. PAR = peripheral (or M2M source) address.
 *   4. M0AR = memory address. 5. NDTR = number of transfers (16-bit;
 *      decremented per beat).
 *   6. CHSEL. 7. PL[1:0] priority (the arbiter of 10.3.4 -- EP07's
 *      "who wins when two streams want the bus" lesson).
 *   8. FIFO usage: FCR DMDIS. 9. CR: DIR, PSIZE/MSIZE, PINC/MINC,
 *      bursts. 10. EN=1.
 *   And the shutdown warning (10.3.17): to detach a peripheral, FIRST
 *   stop the stream and wait EN=0, only THEN touch the peripheral --
 *   bsp_dma_uart_tx follows exactly that order.
 * FIFO vs DIRECT (10.3.6/10.3.12, FCR): DMDIS=0 = direct mode (each
 *   beat passes straight through the 4-word FIFO). DMDIS=1 = FIFO mode
 *   with FTH[1:0] threshold. THE TRAP THIS MODULE TEACHES: RM 10.3.6
 *   says "When memory-to-memory mode is used, the Circular and direct
 *   modes are not allowed" -- so S6 runs FIFO mode (DMDIS=1, FTH=full:
 *   4 word beats drain the FIFO in one stroke), while S7's M2P is the
 *   textbook direct-mode case (DMDIS=0, no threshold).
 * Flags (10.3.13): streams 0-3 report in LISR/LIFCR, streams 4-7 in
 *   HISR/HIFCR -- S6/S7 live in the HIGH half (TCIF6 = HISR bit21,
 *   TCIF7 = HISR bit27). TCIF = NDTR reached 0 (transfer complete);
 *   TEIF = bus error; FEIF = FIFO error (e.g. bad threshold/burst
 *   combination). Status bits are cleared by writing 1 to the matching
 *   C...F bit in HIFCR (rc_w1 -- the opposite convention of TIM/ADC SR,
 *   worth spelling out for the students).
 * EN auto-clear: a non-circular stream that finishes (NDTR=0) drops EN
 *   by hardware -- the selftest reads it back as the "done, on my own"
 *   proof, together with NDTR==0.
 * == HOW TO VERIFY =========================================================
 * menu> dma
 *   DMA-TX-OK            <- the S7 burst arriving on the console mid-test
 *   PASS dma             (steps: 1 S6 mem2mem 64 words + EN auto-clear,
 *                         2 S7 UART DMA line, 3 NDTR==0 on both)
 *   FAIL dma step<N> maps to the steps above.
 * Scope/logic analyzer: 'asm la' on PA9 shows the "DMA-TX-OK" frames
 *   back-to-back with no per-byte polling gaps -- the visible difference
 *   between this path and bsp_uart_puts (EP07's point: DMA frees the
 *   CPU; the wire format is identical).
 */
#include "bsp_dma.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* NDTR is a 16-bit transfer counter (RM0090 10.3.17 step 4). */
#define BSP_DMA_NDTR_MAX 65535u

/* Transfer wait: iteration cap only (spec #5). 64 word beats drain in
 * microseconds at AHB speed; the cap is orders of magnitude longer, so a
 * timeout means the stream stalled (TEIF territory), not slowness. */
#define BSP_DMA_TC_GUARD 2000000u

/* Spin cap while waiting for EN to actually clear (10.3.17 step 1). */
#define BSP_DMA_EN_GUARD 2000000u

/* Self-test step numbers (the value bsp_dma_selftest returns). */
#define BSP_DMA_ST1_MEM2MEM 1u /* 64-word move + EN auto-clear */
#define BSP_DMA_ST2_UART_TX 2u /* S7/Ch4 log line via DMA       */
#define BSP_DMA_ST3_NDTR    3u /* NDTR==0 on S6 and S7          */

/* Self-test payload: 64 words with a recognizable (non-trivial, non-zero)
 * pattern -- index spread across the bits so a stale/shifted word cannot
 * pass by luck. Static buffers: no dynamic allocation (armcc rules). */
#define BSP_DMA_TEST_WORDS 64u

/* The S7 demo line (sent on the log UART by DMA). */
static const char s_dma_tx_msg[] = "DMA-TX-OK\r\n";

/* ---- private helpers -------------------------------------------------------- */

/* 10.3.17 steps 1-2 for one high-half stream: disable, wait EN really
 * clears, write 1s to clear the stream's five event flags in HIFCR.
 * Returns 0 = stream ready, -1 = EN never cleared. */
static int dma_stream_prepare(DMA_Stream_TypeDef *s, uint32_t ifcr_mask)
{
    uint32_t guard = BSP_DMA_EN_GUARD;
    s->CR &= ~DMA_SxCR_EN; /* step 1a: request the disable */
    while ((s->CR & DMA_SxCR_EN) != 0u) { /* step 1b: wait it to land */
        if (--guard == 0u) {
            return -1;
        }
    }
    DMA2->HIFCR = ifcr_mask; /* step 2: rc_w1 clear of the 5 event flags */
    return 0;
}

/* Bounded wait for one HISR flag. Returns 0 = seen, -1 = timeout. */
static int dma_wait_tcif(uint32_t tcif_mask)
{
    uint32_t guard = BSP_DMA_TC_GUARD;
    while ((DMA2->HISR & tcif_mask) == 0u) {
        if (--guard == 0u) {
            return -1;
        }
    }
    return 0;
}

/* Word-compare (no libc): 0 = identical. */
static uint32_t dma_words_differ(const uint32_t *a, const uint32_t *b,
                                 uint32_t n)
{
    uint32_t i;
    for (i = 0u; i < n; i++) {
        if (a[i] != b[i]) {
            return 1u;
        }
    }
    return 0u;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_dma_init(void)
{
    /* Clock gate (RM0090 ch.6): DMA2EN = AHB1ENR bit22. */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR; /* read-back stalls the core until clocks are live */

    /* Both owned streams start parked: disabled, FIFO/direct defaults in
     * place per-stream (each API configures FCR itself), event flags
     * cleared so no stale TCIF survives from a previous run. */
    if (dma_stream_prepare(DMA2_Stream6,
                           DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 |
                           DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6 |
                           DMA_HIFCR_CTCIF6) != 0) {
        return -1;
    }
    if (dma_stream_prepare(DMA2_Stream7,
                           DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 |
                           DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 |
                           DMA_HIFCR_CTCIF7) != 0) {
        return -1;
    }

    /* Read-back proof: gate on, both streams disabled. */
    if (((RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN) == 0u) ||
        ((DMA2_Stream6->CR & DMA_SxCR_EN) != 0u) ||
        ((DMA2_Stream7->CR & DMA_SxCR_EN) != 0u)) {
        return -1;
    }
    return 0;
}

int bsp_dma_mem2mem(uint32_t *dst, const uint32_t *src, uint32_t words)
{
    if ((dst == 0) || (src == 0) || (words == 0u) ||
        (words > BSP_DMA_NDTR_MAX)) {
        return -1;
    }

    /* Steps 1-2 (10.3.17): park the stream, clear stale flags. */
    if (dma_stream_prepare(DMA2_Stream6,
                           DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 |
                           DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6 |
                           DMA_HIFCR_CTCIF6) != 0) {
        return -1;
    }

    /* Steps 3-5: M2M uses PAR as the SOURCE and M0AR as the destination
     * (Table 45, DIR=10), NDTR = word count. */
    DMA2_Stream6->PAR  = (uint32_t)src;
    DMA2_Stream6->M0AR = (uint32_t)dst;
    DMA2_Stream6->NDTR = words;

    /* Steps 6-9 in ONE CR write:
     *   CHSEL=000    -- don't care: M2M never waits for a request.
     *   PL=medium    -- the arbiter knob (10.3.4); alone on the bus here.
     *   PSIZE=MSIZE=word (10b) -- one beat = one word, no packing.
     *   PINC | MINC  -- both pointers step by 4 bytes per beat.
     *   DIR=10       -- MEMORY-TO-MEMORY (Table 45). On F4 this IS the
     *                  M2M select; there is no separate MEM2MEM bit
     *                  (that spelling belongs to the F1 DMA_CCR).
     *   single beats -- PBURST/MBURST=00.
     *   No TCIE      -- flags polled (walkthrough).
     * HAL equivalent: HAL_DMA_Start(&hdma, src, dst, words) with
     *   Init.Direction = DMA_MEMORY_TO_MEMORY, MemOrPeriph inc enabled. */
    DMA2_Stream6->CR = DMA_SxCR_CHSEL_0 /* any: M2M ignores requests; a
                                         * NONZERO CHSEL also proves the
                                         * point in the read-back */
                       | DMA_SxCR_DIR_1            /* DIR[1:0]=10: M2M  */
                       | DMA_SxCR_MINC | DMA_SxCR_PINC
                       | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 /* word    */
                       | DMA_SxCR_PL_1;            /* medium priority  */

    /* Step 8: FIFO mode REQUIRED for M2M -- RM 10.3.6 forbids direct mode
     * here (see walkthrough). FTH=11 (full): the 4-word FIFO drains in a
     * single word-beat burst. */
    DMA2_Stream6->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH;

    /* Step 10: EN=1 -- in M2M this IS the start trigger (no request
     * needed): the burst runs to NDTR=0 and EN auto-clears. */
    DMA2_Stream6->CR |= DMA_SxCR_EN;

    /* Completion: TCIF6 bounded; TEIF/FEIF means a bus/FIFO error. Then
     * a bounded wait for the hardware EN auto-clear (same event, but the
     * read-back must not race the completion -- the selftest asserts
     * EN==0 as its "finished by itself" proof). */
    if (dma_wait_tcif(DMA_HISR_TCIF6) != 0) {
        return -1;
    }
    if ((DMA2->HISR & (DMA_HISR_TEIF6 | DMA_HISR_FEIF6)) != 0u) {
        return -1;
    }
    {
        uint32_t guard = BSP_DMA_EN_GUARD;
        while ((DMA2_Stream6->CR & DMA_SxCR_EN) != 0u) {
            if (--guard == 0u) {
                return -1; /* EN never dropped: stream did not finish */
            }
        }
    }
    return 0;
}

int bsp_dma_uart_tx(const char *s)
{
    uint32_t len = 0u;

    if ((s == 0) || (s[0] == '\0')) {
        return -1;
    }
    while ((s[len] != '\0') && (len < BSP_DMA_NDTR_MAX)) {
        len++;
    }

    /* Steps 1-2 for S7 (10.3.17). */
    if (dma_stream_prepare(DMA2_Stream7,
                           DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 |
                           DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 |
                           DMA_HIFCR_CTCIF7) != 0) {
        return -1;
    }

    /* Steps 3-5: M2P direction (DIR=01, Table 45) reads the SOURCE from
     * M0AR (the string) and writes the DESTINATION PAR (the TX data
     * register -- writes to DR go to the shift pipeline). */
    DMA2_Stream7->PAR  = (uint32_t)&USART1->DR;
    DMA2_Stream7->M0AR = (uint32_t)s;
    DMA2_Stream7->NDTR = len;

    /* Steps 6-9 in ONE CR write: CHSEL=4 = USART1_TX on DMA2 (Table 43),
     * DIR=01 M2P, MSIZE=PSIZE=byte, MINC=1 (walk the string), PINC=0
     * (DR sits at one address), direct mode below, no TCIE. HAL
     * equivalent: HAL_UART_Transmit_DMA(&huart1, ...). */
    DMA2_Stream7->CR = (4u << DMA_SxCR_CHSEL_Pos)  /* Channel 4 = USART1_TX */
                       | DMA_SxCR_DIR_0            /* DIR[1:0]=01: M2P     */
                       | DMA_SxCR_MINC             /* memory walks        */
                       | DMA_SxCR_PL_1;            /* medium priority     */
    /* PSIZE/MSIZE default 00 = byte; PINC=0; PBURST/MBURST=00 = single. */

    /* Direct mode is ALLOWED for M2P (the M2M-only restriction does not
     * apply): DMDIS=0 -- each beat passes through the FIFO straight to
     * the DR (the teaching contrast to S6's forced FIFO mode). */
    DMA2_Stream7->FCR = 0u;

    /* Arm the USART's DMA request line FIRST (CR3 bit7 DMAT: a set TXE
     * raises request 4/7 instead of just waiting for software), then
     * enable the stream (step 10). From here every TXE drains one byte
     * from the string until NDTR hits 0 -- the CPU never touches DR. */
    USART1->CR3 |= USART_CR3_DMAT;
    DMA2_Stream7->CR |= DMA_SxCR_EN;

    /* TCIF7 = all bytes handed to the USART. Then TC on the USART = the
     * shift register drained (the stronger guarantee -- bsp_uart's puts
     * closes on TC for the same reason). Both bounded (spec #5). */
    if (dma_wait_tcif(DMA_HISR_TCIF7) != 0) {
        return -1;
    }
    {
        uint32_t guard = BSP_DMA_TC_GUARD;
        while ((USART1->SR & USART_SR_TC) == 0u) {
            if (--guard == 0u) {
                return -1;
            }
        }
    }

    /* Shutdown order per the 10.3.17 warning: stream first (EN already
     * auto-cleared after NDTR=0), wait it confirmed, THEN release the
     * peripheral request line -- DMAT back to 0 so bsp_uart's polling
     * ownership resumes exactly as it was. */
    {
        uint32_t guard = BSP_DMA_EN_GUARD;
        while ((DMA2_Stream7->CR & DMA_SxCR_EN) != 0u) {
            if (--guard == 0u) {
                break; /* bounded: DMAT cleared below regardless */
            }
        }
    }
    USART1->CR3 &= ~USART_CR3_DMAT;
    return 0;
}

int bsp_dma_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_dma_init() != 0) {
        return (int)BSP_DMA_ST1_MEM2MEM;
    }

    /* Step 1: 64-word mem-to-memory through S6. Pattern src, zeroed dst:
     * the word-compare is the data proof; EN reading back 0 after TCIF is
     * the hardware proof the stream finished by itself. */
    {
        static uint32_t s_src[BSP_DMA_TEST_WORDS]; /* ZI: no dynamic mem */
        static uint32_t s_dst[BSP_DMA_TEST_WORDS];
        uint32_t i;
        for (i = 0u; i < BSP_DMA_TEST_WORDS; i++) {
            s_src[i] = (i << 24u) ^ (i * 0x9E3779B9u) ^ 0xA5A50000u;
            s_dst[i] = 0u;
        }
        if (bsp_dma_mem2mem(s_dst, s_src, BSP_DMA_TEST_WORDS) != 0) {
            return (int)BSP_DMA_ST1_MEM2MEM;
        }
        if (dma_words_differ(s_dst, s_src, BSP_DMA_TEST_WORDS) != 0u) {
            return (int)BSP_DMA_ST1_MEM2MEM;
        }
        if ((DMA2_Stream6->CR & DMA_SxCR_EN) != 0u) {
            return (int)BSP_DMA_ST1_MEM2MEM; /* EN should have auto-cleared */
        }
    }

    /* Step 2: the S7/Ch4 path -- one log line delivered by DMA. Visible
     * on the console between the command echo and this verdict line. */
    if (bsp_dma_uart_tx(s_dma_tx_msg) != 0) {
        return (int)BSP_DMA_ST2_UART_TX;
    }

    /* Step 3: both transfer counters spent themselves to 0 -- the NDTR
     * proof that every beat was served (a partially-completed transfer
     * would still count down the unsent remainder). */
    if ((DMA2_Stream6->NDTR != 0u) || (DMA2_Stream7->NDTR != 0u)) {
        return (int)BSP_DMA_ST3_NDTR;
    }
    return 0; /* all steps passed */
}

/** @} */
