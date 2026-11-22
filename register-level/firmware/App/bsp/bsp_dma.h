/**
 * @file bsp_dma.h
 * @brief DMA2 streams (S6 memory-to-memory demo + S7/Ch4 USART1_TX) --
 *        register-level init, mem2mem API, DMA-driven log line, self-test.
 *
 * Teaching module (P3e wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough / verification recipe live
 * in the bsp_dma.c file header.
 *
 * Ownership rule: the DMA IP is NOT in the IOC (knowledge #13: adapter-owned
 * peripherals stay out). The two streams claimed here are idle by design
 * (allocation table in the project knowledge base: DMA2_S6 = mem-to-mem
 * teaching, DMA2_S7/Ch4 = USART1_TX). After bsp_dma_init() this module
 * OWNS DMA2_Stream6 and DMA2_Stream7 registers (CR/NDTR/PAR/M0AR/FCR).
 * bsp_dma_uart_tx() touches USART1 CR3 DMAT at the HARDWARE level -- set,
 * burst, restore to clear, so bsp_uart's polling ownership never sees a
 * permanent change (no bsp_uart include; rule #7).
 */
#ifndef BSP_DMA_H
#define BSP_DMA_H

#include <stdint.h>

/* Register-level init: AHB1 DMA2EN clock gate; both owned streams left
 * disabled and flag-clean. Returns 0 = PASS (gate read-back), -1 = mismatch. */
int bsp_dma_init(void);

/* One memory-to-memory burst on DMA2_Stream6: 'words' 32-bit words from
 * 'src' to 'dst' (word PSIZE=MSIZE, both pointers incrementing). Follows
 * the RM0090 10.3.17 procedure (disable, wait EN=0, clear flags, load
 * PAR/M0AR/NDTR, CR, enable), polls TCIF6 bounded. mem-to-mem needs FIFO
 * mode (DMDIS=1 -- direct mode is FORBIDDEN here, RM 10.3.6 note).
 * 'words' must be 1..65535 (NDTR is 16-bit). Returns 0 = transferred
 * (EN auto-cleared by hardware), -1 = parameter/timeout/error-flag. */
int bsp_dma_mem2mem(uint32_t *dst, const uint32_t *src, uint32_t words);

/* One DMA-driven log line on DMA2_Stream7 / Channel 4 = USART1_TX
 * (RM0090 Table 43 request mapping): bytes -> USART1->DR as TXE requests
 * arrive, TCIF7 marks the delivery, then the USART TC flag proves the
 * shift register drained before DMAT is cleared again. PRECONDITION: the
 * console must be initialized (the menu runs bsp_uart_init at boot).
 * Returns 0 = line delivered, -1 = empty string or timeout. */
int bsp_dma_uart_tx(const char *s);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = S6 mem2mem of 64 words (pattern src -> zeroed dst): word-compare
 *       must find 0 mismatches, and EN must have AUTO-CLEARED (hardware
 *       drops the stream when NDTR hits 0)
 *   2 = S7/Ch4 DMA sends "DMA-TX-OK\r\n" on the log UART: TCIF7 sets
 *       bounded, and the line is visible on the console
 *   3 = after both transfers NDTR reads 0 on S6 and S7 (the counter
 *       proof that every beat was served) */
int bsp_dma_selftest(void);

#endif /* BSP_DMA_H */
