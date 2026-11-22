/**
 * @file    bsp_dma.c
 * @brief   DMA via HAL UART TX (DMA2_Stream7/Ch4 = USART1_TX) -- HAL
 *          edition -- see bsp_dma.h for the contract, the W2 scope
 *          decision (no mem2mem stream), and the register->HAL map.
 ******************************************************************************
 * The one non-obvious piece of HAL wiring (worth teaching): a HAL
 * UART DMA transmit finishes in TWO stages. HAL_UART_Transmit_DMA arms
 * the stream and sets huart1.gState = BUSY_TX. When the stream drains
 * into USART1->DR, the DMA2_Stream7 interrupt runs UART_DMATransmitCplt,
 * which clears DMAT and raises the UART TC interrupt. Only the USART1
 * interrupt (HAL_UART_IRQHandler -> UART_EndTransmit_IT) puts gState
 * back to READY. The .ioc arms the DMA stream vector but NOT
 * USART1_IRQn, so this module arms it in bsp_dma_init and provides the
 * handler here (bsp module file, Core stays untouched) -- without it,
 * gState stays BUSY_TX after one DMA burst and every later polled
 * console write returns HAL_BUSY.
 */
#include "bsp_dma.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "usart.h" /* huart1 from MX_USART1_UART_Init (CubeMX-generated) */

/* hdma_usart1_tx is DEFINED by the CubeMX-generated Core/Src/usart.c
 * (linked into huart1.hdmatx by __HAL_LINKDMA) but usart.h does not
 * re-export it, so the consumer declares it -- the same pattern
 * stm32f4xx_it.c uses. */
extern DMA_HandleTypeDef hdma_usart1_tx;

/* ---- module constants ----------------------------------------------------- */

/* String length cap: the DMA transfer counter (NDTR) is 16-bit (RM0090
 * 10.3.17 step 4) -- HAL_UART_Transmit_DMA takes uint16_t anyway. */
#define BSP_DMA_LEN_MAX 65535u

/* Completion-beat waits: ms-granularity passes, each capped. 11 bytes
 * at 115200 need ~1 ms; 100 passes = 100 ms worst case per beat, so a
 * timeout means the chain really broke, not that the link was slow
 * (spec #5: every wait bounded). */
#define BSP_DMA_TX_PASSES 100u

/* Self-test step numbers (the value bsp_dma_selftest returns). */
#define BSP_DMA_ST1_PATH     1u /* DMA-path config read-back  */
#define BSP_DMA_ST2_UART_TX  2u /* DMA log line (3-beat chain)*/
#define BSP_DMA_ST3_COUNTER  3u /* NDTR==0 + stream READY     */

/* The demo line (sent on the log UART by DMA). Static const: no dynamic
 * allocation (armcc rules). */
static const char s_dma_tx_msg[] = "DMA-TX-OK\r\n";

/* ---- private helpers -------------------------------------------------------- */

/* Bounded ms-granularity wait for a predicate, 'passes' iterations of
 * 1 ms each. Returns 0 = predicate seen, -1 = cap expired. */
static int dma_wait_passes(int (*pred)(void))
{
    uint32_t passes = BSP_DMA_TX_PASSES;
    while (pred() == 0) {
        if (passes == 0u) {
            return -1;
        }
        passes--;
        HAL_Delay(1);
    }
    return 0;
}

/* Predicates for the three completion beats (each a read-only probe:
 * HAL state getters / SR flag macro -- no register writes, A3-safe). */
static int dma_stream_ready(void)
{
    return (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY) ? 1 : 0;
}

static int dma_uart_tc(void)
{
    return (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == SET) ? 1 : 0;
}

static int dma_uart_ownership_back(void)
{
    return (huart1.gState == HAL_UART_STATE_READY) ? 1 : 0;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_dma_init(void)
{
    /* The stream the .ioc instantiated (W2 scope: no mem2mem stream) --
     * shape verified read-only against the handle fields the CubeMX
     * generator wrote (A3 read-only exemption). */
    if (hdma_usart1_tx.Instance != DMA2_Stream7) {
        return -1;
    }
    if (hdma_usart1_tx.Init.Channel != DMA_CHANNEL_4) {
        return -1; /* Ch4 = USART1_TX (RM0090 Table 43) */
    }
    if (hdma_usart1_tx.Init.Direction != DMA_MEMORY_TO_PERIPH) {
        return -1; /* M2P: the old S7 DIR=01 case */
    }
    if (hdma_usart1_tx.Init.PeriphInc != DMA_PINC_DISABLE) {
        return -1; /* DR sits at one address */
    }
    if (hdma_usart1_tx.Init.MemInc != DMA_MINC_ENABLE) {
        return -1; /* walk the string */
    }
    if ((hdma_usart1_tx.Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE) ||
        (hdma_usart1_tx.Init.MemDataAlignment != DMA_MDATAALIGN_BYTE)) {
        return -1; /* byte beats */
    }
    if (hdma_usart1_tx.Init.Mode != DMA_NORMAL) {
        return -1; /* non-circular: NDTR runs to 0 */
    }
    if (hdma_usart1_tx.Init.FIFOMode != DMA_FIFOMODE_DISABLE) {
        return -1; /* direct mode: allowed for M2P (the old S6-vs-S7
                    * FIFO teaching contrast) */
    }
    /* LINKDMA wiring: the UART hands its TX requests to THIS stream. */
    if (huart1.hdmatx != &hdma_usart1_tx) {
        return -1;
    }
    /* The DMA completion vector must be live (MX_DMA_Init armed it). */
    if (NVIC_GetEnableIRQ(DMA2_Stream7_IRQn) == 0u) {
        return -1;
    }
    /* Arm the UART-side completion vector (idempotent). Priority 5 to
     * match the DMA stream. See the file header for why this is
     * required: without it gState never returns to READY and the polled
     * console dies after the first DMA burst. */
    HAL_NVIC_SetPriority(USART1_IRQn, 5u, 0u);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (NVIC_GetEnableIRQ(USART1_IRQn) == 0u) {
        return -1;
    }
    return 0;
}

int bsp_dma_uart_tx(const char *s)
{
    uint32_t len = 0u;

    if ((s == NULL) || (s[0] == '\0')) {
        return -1;
    }
    while ((s[len] != '\0') && (len < BSP_DMA_LEN_MAX)) {
        len++;
    }

    /* Hand the burst to the HAL: this loads PAR (DR), M0AR (string) and
     * NDTR (len), enables the stream, sets DMAT, and takes the UART
     * gState to BUSY_TX -- the old edition's 10.3.17 steps 3-10 in one
     * call. */
    if (HAL_UART_Transmit_DMA(&huart1, (const uint8_t *)s,
                              (uint16_t)len) != HAL_OK) {
        return -1;
    }

    /* Beat 1: stream drains string -> DR (TXE-served beats); the
     * DMA2_Stream7 IRQ marks the handle READY. */
    if (dma_wait_passes(dma_stream_ready) != 0) {
        return -1;
    }
    /* Beat 2: TCIE now set; the last byte is still in the shift
     * pipeline. TC flag = the wire drained (the stronger guarantee). */
    if (dma_wait_passes(dma_uart_tc) != 0) {
        return -1;
    }
    /* Beat 3: the USART1 IRQ serviced TC and ended the HAL Tx process:
     * gState READY = console ownership back to bsp_uart's polled TX
     * (old edition: DMAT restored). */
    if (dma_wait_passes(dma_uart_ownership_back) != 0) {
        return -1;
    }
    return 0;
}

int bsp_dma_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_dma_init() != 0) {
        return (int)BSP_DMA_ST1_PATH;
    }

    /* Step 2: the DMA path -- one log line delivered by DMA. Visible on
     * the console between the command echo and this verdict line (the
     * wire format is identical to polled TX; the difference is who
     * serves the bytes -- DMA does, the CPU never touches DR). */
    if (bsp_dma_uart_tx(s_dma_tx_msg) != 0) {
        return (int)BSP_DMA_ST2_UART_TX;
    }

    /* Step 3: the counter proof, HAL twins of the register edition's
     * NDTR==0 + EN auto-clear checks: the transfer counter spent itself
     * to 0 (every beat served; a partially-completed transfer would
     * still count down the unsent remainder) and the stream dropped
     * itself to READY. The NDTR read is a read-only probe (A3
     * read-only exemption); the state check goes through HAL. */
    if (hdma_usart1_tx.Instance->NDTR != 0u) {
        return (int)BSP_DMA_ST3_COUNTER;
    }
    if (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY) {
        return (int)BSP_DMA_ST3_COUNTER;
    }
    return 0; /* all steps passed */
}

/* ---- USART1 vector (HAL completion path; Core stays untouched) ------------- */

/* Standard HAL pattern (the same one CubeMX emits into it.c when the
 * USART1 global interrupt is ticked): route every USART1 interrupt into
 * the HAL handler. This exists only to service the TC raised by the DMA
 * transmit finish path -- RX interrupts stay off (polled RX), so this
 * vector is quiet outside a DMA burst. */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
