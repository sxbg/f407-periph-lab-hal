/**
 ******************************************************************************
 * @file    bsp_uart.c
 * @brief   USART1 log console -- teaching module with register-level init
 *          and self-test (P3 DL-A5 single-module pilot).
 ******************************************************************************
 * @addtogroup BSP_UART
 * @{
 *
 * == CUBEMX CONFIG (from f407-periph-lab.ioc) ==============================
 * | Field            | Value (IOC truth)                    | Why
 * |------------------|--------------------------------------|-------------------
 * | Mode             | Asynchronous (USART1.VirtualMode=    | 8N1 console, no
 * |                  | VM_ASYNC)                            | external clock pin
 * | Baud Rate        | 115200 (USART1.BaudRate)             | board USB-TTL
 * |                  |                                      | supports it; 0.023%
 * |                  |                                      | error at 84 MHz
 * | Word Length      | 8 Bits, M=0                          | byte line protocol
 * | Parity           | NONE (PCE=0)                         | console ASCII
 * | Stop Bits        | 1 (CR2 STOP[1:0]=00)                 | default, max rate
 * | Hw Flow Control  | NONE                                 | 2-wire PA9/PA10
 * | Oversampling     | 16 (OVER8=0)                         | best accuracy;
 * |                  |                                      | BRR=0x2D9, see below
 * | Pins             | PA9=USART1_TX, PA10=USART1_RX, AF7   | board USB-TTL wired
 * | Kernel Clock     | PCLK2 = 84 MHz (RCC.APB2Freq_Value,  | BRR reference fCK
 * |                  | APB2 = SYSCLK/2 = 168/2)             |
 * | NVIC             | USART1_IRQn NOT enabled              | polling-first
 * |                  |                                      | teaching base
 *
 * == REGISTER WALKTHROUGH (what this module actually touches) ==============
 * RCC->APB2ENR bit 14 (USART1EN): kernel clock gate. A peripheral MUST be
 *   clocked before its registers are written (RM0090 ch.8/30); HAL's
 *   MspInit already set this, the module re-arms it idempotently so init
 *   stays self-sufficient.
 * BRR (RM0090 ch.30, "Fractional baud rate generation"):
 *   USARTDIV = fCK/(16*baud) = 84000000/(16*115200) = 45.5729
 *   DIV_Mantissa[15:4] = 45 (0x2D), DIV_Fraction[3:0] = round(0.5729*16)=9
 *   -> BRR = 45*16+9 = 729 = 0x2D9
 *   -> actual baud = 84000000/(16*45.5625) = 115226, error +0.023%
 *      (typical UART tolerance budget 2%; anything >2% at 8N1 sheds bits).
 *   RM0090 requires UE=0 while writing BRR -- hence the disable-first
 *   order below.
 * CR1 (RM0090 ch.30, control register 1):
 *   bit13 UE  usart enable -- set LAST, after BRR/CR2/CR3 are final
 *   bit12 M   word length: 0 = 8 data bits (IOC WordLength=8B)
 *   bit10 PCE parity control: 0 = none (IOC Parity=NONE)
 *   bit15 OVER8 oversampling: 0 = by 16 (IOC OverSampling=16)
 *   bit3  TE  transmitter enable (drives PA9 into AF7 push-pull)
 *   bit2  RE  receiver enable (PA10 sampled through AF7)
 * CR2: STOP[13:12]=00 -> 1 stop bit. CR3: no DMA/no smartcard/no half-duplex.
 * SR flags used (RM0090 ch.30, status register):
 *   bit7 TXE "TX data register empty" -- DR can take the next byte
 *   bit6 TC  "transmission complete" -- shift register fully drained;
 *        TC is the STRONGER guarantee (TXE still lets the last frame be
 *        in flight) -- used to close puts() and before NVIC_SystemReset
 *   bit5 RXNE "read data register not empty" -- DR holds a received byte;
 *        cleared BY THE DR READ (peripheral contract, not by software)
 *   bit3 ORE overrun -- set when a new byte lands while RXNE=1; cleared by
 *        the SR-then-DR read sequence, which is why getc always reads SR
 *        FIRST then DR (also the required FE/NE clearing order).
 *
 * == HOW TO VERIFY =========================================================
 * On the menu console (115200 8N1, any terminal):
 *   menu> uart
 *   [uart] step1: BRR=2D9 CR1=200C (UE|TE|RE)
 *   [uart] step2: TX (watch for the marker line)
 *   UART-SELFTEST-OK
 *   [uart] step3: RX idle window ok (no data in window)
 *   PASS uart
 * Failure surfaces as "FAIL uart step<N>"; N maps to the steps above.
 * Extra: scope PA9 -- 8N1 frames, 8.68 us per bit at 115200.
 */
#include "bsp_uart.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Bounded-spin budget for all flag waits (spec #5: never dead-wait). Same
 * magnitude as LOG_POLL_LIMIT: ~tens of ms at 168 MHz, far longer than one
 * 115200 frame (~87 us), so a timeout here means real breakage, not latency. */
#define BSP_UART_POLL_LIMIT 2000000u

/* Self-test step numbers (the value bsp_uart_selftest returns on failure). */
#define BSP_UART_ST1_READBACK 1u /* BRR/CR1 register read-back */
#define BSP_UART_ST2_TX_TC    2u /* TC-gated transmit */
#define BSP_UART_ST3_RX_TO    3u /* RX idle-timeout path */

/* ---- flag helpers ---------------------------------------------------------- */

/* Spin until (SR & mask) or the iteration cap expires. 0 = flag seen,
 * -1 = timeout. All TX waits funnel through here so there is exactly one
 * place with the bounded-wait discipline. */
static int uart_wait_flag(uint32_t mask)
{
    uint32_t guard = BSP_UART_POLL_LIMIT;
    while ((USART1->SR & mask) == 0u) {
        if (--guard == 0u) {
            return -1; /* bounded exit: never hang (spec #5) */
        }
    }
    return 0;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_uart_init(void)
{
    /* Kernel clock on (RCC->APB2ENR bit14). Idempotent re-arm of the bit
     * HAL's MspInit already set: registers are write-safe only when clocked. */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR; /* read-back stalls the core until the clock is live */

    /* Disable first: RM0090 says BRR must be written with UE=0. Clearing UE
     * also aborts any frame the HAL left mid-flight -- full ownership. */
    USART1->CR1 &= ~USART_CR1_UE;

    /* BRR = 0x2D9: 84 MHz / 16 / 115200, mantissa 45 fraction 9 (see header
     * derivation; +0.023% baud error). HAL equivalent:
     *   huart1.Init.BaudRate = 115200; HAL_UART_Init(&huart1); */
    USART1->BRR = BSP_UART_BRR_EXPECTED;

    /* CR2/CR3 to IOC truth: 1 stop bit (STOP[1:0]=00), no DMA/flow/HD. The
     * HAL left these already correct; writing them makes this init the
     * single owner of the full CR set rather than a partial patcher. */
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;

    /* CR1 in one write: M=0 (8 bits), PCE=0 (no parity), OVER8=0 (16x),
     * UE|TE|RE on. UE is set only now -- after BRR/CR2/CR3 are final.
     * HAL equivalent: huart1.Init.{Mode=UART_MODE_TX_RX, ...}; HAL_UART_Init. */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* Read-back proof: the peripheral must have accepted the config. */
    if (USART1->BRR != BSP_UART_BRR_EXPECTED) {
        return -1;
    }
    if ((USART1->CR1 & (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE)) !=
        (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE)) {
        return -1;
    }
    return 0;
}

void bsp_uart_puts(const char *s)
{
    if (s == 0) {
        return;
    }
    while (*s != '\0') {
        /* TXE: previous DR content has moved to the shift register. */
        if (uart_wait_flag(USART_SR_TXE) != 0) {
            return; /* bounded exit on a dead peripheral */
        }
        /* DR write (RM0090 ch.30): byte is queued, moved to the shift
         * register when TXE clears again. HAL equivalent: HAL_UART_Transmit. */
        USART1->DR = (uint32_t)(uint8_t)(*s++);
    }
    /* TC: shift register drained -- the string is fully on the wire, not
     * merely queued. Bounded: on timeout the bytes are already in flight. */
    (void)uart_wait_flag(USART_SR_TC);
}

int bsp_uart_getc_nonblock(void)
{
    /* Read SR FIRST, then DR: this exact sequence also clears RXNE and any
     * ORE/FE/NE error flags (RM0090 clearing contracts). */
    if ((USART1->SR & USART_SR_RXNE) == 0u) {
        return -1; /* nothing received: non-blocking exit */
    }
    return (int)(uint8_t)USART1->DR;
}

int bsp_uart_selftest(void)
{
    /* Step 1: register read-back -- proves init took effect and stays put. */
    bsp_uart_puts("[uart] step1: BRR/CR1 read-back\r\n");
    if (USART1->BRR != BSP_UART_BRR_EXPECTED) {
        bsp_uart_puts("[uart] step1: BRR mismatch\r\n");
        return (int)BSP_UART_ST1_READBACK;
    }
    {
        const uint32_t cr1 = USART1->CR1;
        const uint32_t want =
            USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
        if ((cr1 & want) != want) {
            bsp_uart_puts("[uart] step1: CR1 UE/TE/RE mismatch\r\n");
            return (int)BSP_UART_ST1_READBACK;
        }
        /* Detail line: BRR and CR1 in hex so the numbers can be cross-checked
         * against the walkthrough table above. */
        bsp_uart_puts("[uart] step1: BRR=2D9 CR1=");
        {
            /* Tiny local hex printer: keeps this module independent of the
             * platform log layer (spec #7: no cross-module includes). */
            static const char digits[16] = {
                '0', '1', '2', '3', '4', '5', '6', '7',
                '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
            };
            char buf[9];
            unsigned n = 0u;
            uint32_t v = cr1;
            do {
                buf[n++] = digits[v & 0xFu];
                v >>= 4;
            } while (v > 0u && n < 8u);
            while (n > 0u) {
                (void)uart_wait_flag(USART_SR_TXE);
                USART1->DR = (uint32_t)(uint8_t)buf[--n];
            }
        }
        bsp_uart_puts(" (UE|TE|RE)\r\n");
    }

    /* Step 2: transmit the marker through the TC-gated path. A logic
     * analyzer on PA9 sees 8N1 frames at 8.68 us/bit. */
    bsp_uart_puts("[uart] step2: TX marker next line\r\n");
    bsp_uart_puts("UART-SELFTEST-OK\r\n");

    /* Step 3: RX idle-timeout path. A quiet line must time out (that IS the
     * pass); a stray keystroke must drain and RXNE must clear on the DR
     * read -- both branches prove the RX path follows the RM contract. */
    bsp_uart_puts("[uart] step3: RX idle window\r\n");
    {
        uint32_t guard = BSP_UART_POLL_LIMIT;
        for (;;) {
            const uint32_t sr = USART1->SR; /* SR first (clear-order rule) */
            if ((sr & USART_SR_RXNE) != 0u) {
                (void)USART1->DR; /* then DR: consumes the byte */
                if ((USART1->SR & USART_SR_RXNE) != 0u) {
                    bsp_uart_puts("[uart] step3: RXNE stuck after DR read\r\n");
                    return (int)BSP_UART_ST3_RX_TO;
                }
                continue; /* drain everything the window delivers */
            }
            if ((sr & USART_SR_ORE) != 0u) {
                /* ORE set with no RXNE should not survive an SR read; if it
                 * does the clearing contract is broken. */
                bsp_uart_puts("[uart] step3: ORE stuck\r\n");
                return (int)BSP_UART_ST3_RX_TO;
            }
            if (--guard == 0u) {
                break; /* window expired with no data: expected quiet line */
            }
        }
    }
    bsp_uart_puts("[uart] step3: RX idle timeout ok\r\n");
    return 0; /* all steps passed */
}

/** @} */
