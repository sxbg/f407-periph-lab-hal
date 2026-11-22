/**
 * @file bsp_dma.h
 * @brief DMA via HAL UART TX (DMA2_Stream7/Ch4 = USART1_TX) -- HAL
 *        edition: config verify, DMA-driven log line, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_dma).
 *
 * SCOPE DECISION (W2): the register edition proved two DMA facts --
 * Stream6 memory-to-memory and Stream7/Ch4 USART1_TX wiring. The
 * HAL-edition .ioc instantiates ONLY the USART1_TX stream
 * (hdma_usart1_tx, DMA2_Stream7/DMA_CHANNEL_4, configured and linked
 * to huart1 by MX_USART1_UART_Init in Core/Src/usart.c; MX_DMA_Init
 * arms its NVIC vector). There is no mem2mem stream, and HAL offers no
 * one-call M2M teaching API in this project, so bsp_dma_mem2mem() has
 * no HAL edition -- THIS MODULE PROVES THE USART1_TX DMA PATH instead.
 *
 * Register-concept -> HAL-call mapping (vs the register edition):
 *   S7 CR write (CHSEL4/DIR=M2P/MINC/byte/direct)
 *                                -> CubeMX Init struct (verified
 *                                   read-only via the handle fields)
 *   PAR/M0AR/NDTR load + EN=1    -> HAL_UART_Transmit_DMA(&huart1,...)
 *   arm DMAT, send, restore DMAT -> HAL arms DMAT, UART_DMATransmitCplt
 *                                   clears it; the UART-side completion
 *                                   needs USART1_IRQHandler serviced --
 *                                   this module arms the vector (see
 *                                   bsp_dma_init) so the console returns
 *                                   to polled ownership, the HAL-era
 *                                   translation of the old "restore
 *                                   DMAT" courtesy
 *   poll HISR TCIF7 (bounded)    -> poll HAL_DMA_GetState(&hdma_usart1_tx)
 *                                   != BUSY (read-only state checks,
 *                                   iteration-capped loops)
 *   poll USART SR TC             -> poll UART_FLAG_TC via the
 *                                   __HAL_UART_GET_FLAG macro
 *   NDTR==0 + EN auto-clear      -> NDTR read-back + DMA state READY
 *
 * Console discipline: the module does not print (rule #7) -- the
 * "DMA-TX-OK" line appearing between echo and verdict IS the test
 * payload, delivered by DMA.
 */
#ifndef BSP_DMA_H
#define BSP_DMA_H

#include <stdint.h>

/* Verify the CubeMX-generated DMA pathway with read-only probes: the
 * stream is DMA2_Stream7/Ch4 M2P, byte/byte, memory-increment only,
 * normal mode, direct (FIFO disabled); it is the one linked into
 * huart1.hdmatx; the DMA completion vector (MX_DMA_Init) is live. Also
 * arms USART1_IRQn (priority 5, matching the DMA stream) if not armed:
 * the HAL UART DMA-tx finish path raises the UART TC interrupt, and
 * without its handler huart1.gState would stay BUSY_TX forever -- every
 * later polled HAL_UART_Transmit (the console) would return HAL_BUSY.
 * Returns 0 = PASS, -1 = mismatch. */
int bsp_dma_init(void);

/* One DMA-driven log line on DMA2_Stream7/Ch4 = USART1_TX: hands 's' to
 * HAL_UART_Transmit_DMA, then follows the three-beat completion chain,
 * every beat bounded (ms-granularity iteration caps):
 *   1. DMA stream drains the string (state BUSY -> READY)
 *   2. UART TC flag sets (the shift pipeline is empty -- the stronger
 *      guarantee the register edition closed on)
 *   3. huart1.gState returns READY (USART1 IRQ serviced the TC: console
 *      ownership back to bsp_uart's polled TX)
 * PRECONDITION: the console must be initialized (the menu runs
 * bsp_uart_init at boot). Returns 0 = line delivered, -1 = empty string
 * or a completion beat timed out. */
int bsp_dma_uart_tx(const char *s);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = DMA-path config read-back (see bsp_dma_init): stream shape,
 *       LINKDMA wiring, DMA + USART1 vectors live
 *   2 = "DMA-TX-OK\r\n" delivered through the three-beat chain above
 *       (visible on the console between the command echo and this
 *       verdict line)
 *   3 = counter proof, the HAL twin of the register edition's NDTR==0 +
 *       EN auto-clear: hdma_usart1_tx NDTR reads 0 and the stream state
 *       is READY after the transfer */
int bsp_dma_selftest(void);

#endif /* BSP_DMA_H */
