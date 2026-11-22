/**
 * @file bsp_uart.c
 * @brief USART1 console BSP (HAL edition) -- see bsp_uart.h for the contract.
 */
#include "bsp_uart.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "usart.h" /* huart1 from MX_USART1_UART_Init (CubeMX-generated) */

/* Per-byte TX timeout in ticks: at 115200 8N1 one byte needs ~87 us; the
 * HAL polls TXE with tick granularity, so one tick is plenty per byte. */
#define BSP_UART_TX_TIMEOUT_MS 5u

int bsp_uart_init(void)
{
    /* Read-back only (A3 read-only exemption): the module verifies the
     * CubeMX configuration instead of rewriting registers. */
    if (huart1.Instance == NULL) {
        return -1;
    }
    if (huart1.Instance->BRR != BSP_UART_BRR_EXPECTED) {
        return -1;
    }
    {
        const uint32_t cr1 = huart1.Instance->CR1;
        if (((cr1 & USART_CR1_UE) == 0u) ||
            ((cr1 & USART_CR1_TE) == 0u) ||
            ((cr1 & USART_CR1_RE) == 0u)) {
            return -1;
        }
    }
    return 0;
}

void bsp_uart_puts(const char *s)
{
    /* Walk the string, chunking into HAL_UART_Transmit calls with bounded
     * timeouts; on HAL_TIMEOUT bail out early (watchdog discipline). */
    while (*s != '\0') {
        const char *chunk = s;
        unsigned n = 0u;
        while ((*s != '\0') && (n < 32u)) {
            s++;
            n++;
        }
        if (HAL_UART_Transmit(&huart1, (uint8_t *)(uintptr_t)chunk, n,
                              BSP_UART_TX_TIMEOUT_MS * n) != HAL_OK) {
            return; /* TX stuck (e.g. line down): never hang the loop */
        }
    }
}

int bsp_uart_getc_nonblock(void)
{
    uint8_t b = 0u;
    if (HAL_UART_Receive(&huart1, &b, 1u, 0u) == HAL_OK) {
        return (int)b;
    }
    /* ORE purge: bytes that arrived while the menu was blocked inside a
     * HAL_UART_Transmit print set ORE; once set, every new RXNE is
     * suppressed and the menu starves on paste bursts. Clear it so the
     * stream resumes (knowledge #18 pacing fix, board-verified). */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        (void)HAL_UART_Receive(&huart1, &b, 1u, 0u); /* final DR read */
    }
    return -1; /* HAL_TIMEOUT: RXNE clear */
}

int bsp_uart_selftest(void)
{
    /* Step 1: BRR + CR1 read-back (see bsp_uart_init). */
    if (bsp_uart_init() != 0) {
        bsp_uart_puts("  uart step1: BRR/CR1 readback mismatch\r\n");
        return 1;
    }

    /* Step 2: TC-gated TX through HAL_UART_Transmit. */
    {
        const char msg[] = "UART-SELFTEST-OK";
        if (HAL_UART_Transmit(&huart1, (const uint8_t *)msg,
                              (uint16_t)(sizeof(msg) - 1u),
                              100u) != HAL_OK) {
            bsp_uart_puts("  uart step2: HAL_UART_Transmit failed\r\n");
            return 2;
        }
        bsp_uart_puts("\r\n");
    }

    /* Step 3: quiet-window RX timeout -- three 10 ms windows must all
     * time out (RXNE clear). Any echo tail of the command itself is
     * consumed first (one byte may still sit in DR from the \r). */
    {
        uint8_t b = 0u;
        while (HAL_UART_Receive(&huart1, &b, 1u, 0u) == HAL_OK) {
            /* drain residual bytes until RXNE stays clear */
        }
        int w;
        for (w = 0; w < 3; w++) {
            if (HAL_UART_Receive(&huart1, &b, 1u, 10u) == HAL_OK) {
                bsp_uart_puts("  uart step3: stray byte in quiet window\r\n");
                return 3;
            }
        }
    }
    return 0;
}
