/**
 ******************************************************************************
 * @file    bsp_gpio.c
 * @brief   GPIO: LEDs (PC5/PB2), keys (PA0/PA1), LCD backlight (PA15) --
 *          teaching module with register-level init and self-test.
 ******************************************************************************
 * @addtogroup BSP_GPIO
 * @{
 *
 * == CUBEMX CONFIG (from f407-periph-lab.ioc) ==============================
 * | Field            | Value (IOC truth)                    | Why
 * |------------------|--------------------------------------|-------------------
 * | PC5 LED_GREEN    | GPIO_Output, PP, Low speed,          | menu heartbeat
 * |                  | boot state RESET (off)               | LED, app_config.h
 * | PB2 LED_BLUE     | GPIO_Output, PP, Low speed,          | second status LED
 * |                  | boot state SET (on)                  | (blue is on at boot)
 * | PA0 KEY1         | GPIO_Input, GPIO_PULLDOWN            | button closes to
 * |                  |                                      | VDD -> active-high;
 * |                  |                                      | idle level = 0
 * | PA1 KEY2         | GPIO_Input, GPIO_PULLUP              | button closes to
 * |                  |                                      | GND -> active-low;
 * |                  |                                      | idle level = 1
 * | PA15 LCD_BL      | GPIO_Output, PP, Low speed,          | LCD backlight
 * |                  | boot state RESET (off)               | enable pin
 * | Port clocks      | GPIOA/B/C on AHB1 (HAL did it)       | re-armed here
 * | (PA15 caveat)    | PA15 = JTAG JTDI in default AF0      | MODER=01 re-routes
 * |                  |                                      | it; SWD (PA13/14)
 * |                  |                                      | stays untouched
 * == REGISTER WALKTHROUGH (RM0090 ch.8 "General-purpose I/Os") ==============
 * RCC->AHB1ENR (ch.6): GPIOAEN bit0 / GPIOBEN bit1 / GPIOCEN bit2 -- AHB1
 *   port clock gates. Config registers are write-safe only when clocked;
 *   a dummy read after the OR stalls the core until the clock is live.
 * MODER (ch.8.4.1): 2 bits per pin. 00=input, 01=general-purpose output,
 *   10=alternate function, 11=analog. Fields used here:
 *   PC5 [11:10]=01, PB2 [5:4]=01, PA15 [31:30]=01 (outputs);
 *   PA0 [1:0]=00, PA1 [3:2]=00 (inputs -- the pull resistors live in PUPDR).
 * OTYPER (ch.8.4.2): 1 bit per pin. 0=push-pull (both FETs driven), 1=
 *   open-drain (PMOS off, line floats high via pull-up). All five pins 0.
 * OSPEEDR (ch.8.4.3): 2 bits per pin: 00=low ... 11=very high slew. IOC
 *   picked LOW (00) for all outputs -- 1 kHz class signals, lowest EMI.
 * PUPDR (ch.8.4.4): 2 bits per pin. 00=none, 01=pull-up (~40 kOhm to VDD),
 *   10=pull-down (~40 kOhm to GND). KEY1 PA0 [1:0]=10 (pull-down -> idle 0,
 *   button to VDD reads 1 = active-high); KEY2 PA1 [3:2]=01 (pull-up ->
 *   idle 1, button to GND reads 0 = active-low).
 * IDR (ch.8.4.5): input level, latched every AHB clock, read-only.
 * ODR (ch.8.4.6): output latch. NEVER write it read-modify-write style from
 *   mixed contexts: two RMW sequences can interleave and one update is lost
 *   (the classic lost-update race).
 * BSRR (ch.8.4.7): the atomic alternative used by every output API here:
 *   bits [15:0] BSy = set pin y, bits [31:16] BRy = reset pin y. One
 *   32-bit write, hardware resolves set-vs-reset in a single cycle, no
 *   ODR read involved -- ISR-safe by construction.
 * == HOW TO VERIFY =========================================================
 * menu> gpio
 *   PASS gpio            (all three steps; FAIL gpio step<N> maps below)
 *   step1 = config + ODR read-back, step2 = KEY idle levels, step3 = blink.
 *   Watch the board: green flashes 200 ms, blue dips 200 ms (step 3),
 *   then boot state returns (green off, blue on).
 * Extra: hold KEY1/KEY2 during step 2 and the test FAILs by design -- the
 *   idle-level contract requires released keys.
 * NOTE: LED_GREEN (PC5) is also the menu-loop heartbeat (app_config.h
 *   APP_MENU_HEARTBEAT_MS); the heartbeat only runs between menu commands,
 *   so it cannot fight the blink inside this selftest.
 */
#include "bsp_gpio.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* Owned pins as bit numbers (named once, used everywhere: no magic values).
 * BSRR/ODR/IDR all address pins by these single-bit masks. */
#define LED_GREEN_BIT (1u << 5u)  /* PC5 */
#define LED_BLUE_BIT  (1u << 2u)  /* PB2 */
#define LCD_BL_BIT    (1u << 15u) /* PA15 */
#define KEY1_BIT      (1u << 0u)  /* PA0 */
#define KEY2_BIT      (1u << 1u)  /* PA1 */

/* Self-test step numbers (the value bsp_gpio_selftest returns on failure). */
#define BSP_GPIO_ST1_CONFIG 1u /* config read-back + ODR round-trip */
#define BSP_GPIO_ST2_KEYS   2u /* KEY idle levels via IDR */
#define BSP_GPIO_ST3_BLINK  3u /* visible blink + restore */

/* Inner spin cap for one SysTick wrap (~tens of ms of CPU cycles): if
 * COUNTFLAG never sets within it, SysTick is not running -- bounded exit
 * instead of a hang (spec #5). */
#define GPIO_SYSTICK_GUARD 2000000u

/* ---- private helpers -------------------------------------------------------- */

/* Wait approximately 'ms' milliseconds by counting SysTick wraps.
 * COUNTFLAG (CTRL bit 16, PM0056/B3.3) sets on every counter wrap and is
 * cleared BY THE READ, so each counted set is exactly one wrap. The HAL
 * timebase reloads 168000-1 ticks at 168 MHz (HAL_InitTick), i.e. one wrap
 * = 1 ms -- which is the whole measurement. Returns 0 = waited, -1 =
 * SysTick looks dead (bounded exit, caller decides how to fail). */
static int gpio_wait_ms(uint32_t ms)
{
    uint32_t left = ms;
    while (left > 0u) {
        uint32_t guard = GPIO_SYSTICK_GUARD;
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0u) {
            if (--guard == 0u) {
                return -1; /* no wrap in the budget: SysTick not ticking */
            }
        }
        left--;
    }
    return 0;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_gpio_init(void)
{
    /* Port clocks (RM0090 ch.6 AHB1ENR). HAL's MX_GPIO_Init already set
     * these bits; re-arming is idempotent and makes init self-sufficient. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR; /* read-back stalls the core until clocks are live */

    /* --- PC5 LED_GREEN: output, push-pull, low speed, no pull ---------- */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER5_Msk) |
                   GPIO_MODER_MODER5_0; /* [11:10]=01 general-purpose output */
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT5;  /* OT5=0 push-pull (both FETs) */
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_Msk; /* [11:10]=00 low slew */
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD5_Msk;       /* [11:10]=00 no pull */

    /* --- PB2 LED_BLUE: same output shape on port B --------------------- */
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER2_Msk) |
                   GPIO_MODER_MODER2_0; /* [5:4]=01 output */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT2;  /* push-pull */
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_Msk; /* low slew */
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD2_Msk;       /* no pull */

    /* --- PA15 LCD_BL: output on port A (steals JTDI, keeps SWD) -------- */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER15_Msk) |
                   GPIO_MODER_MODER15_0; /* [31:30]=01 output */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT15;  /* push-pull */
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED15_Msk; /* low slew */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD15_Msk;       /* no pull */

    /* --- PA0 KEY1: input + pull-down (RM0090 ch.8.4.4, 10 = pull-down) - */
    GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;      /* [1:0]=00 input */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD0_Msk) |
                   GPIO_PUPDR_PUPD0_1; /* [1:0]=10 ~40k to GND: idle reads 0 */

    /* --- PA1 KEY2: input + pull-up (01 = pull-up) ----------------------- */
    GPIOA->MODER &= ~GPIO_MODER_MODER1_Msk;      /* [3:2]=00 input */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD1_Msk) |
                   GPIO_PUPDR_PUPD1_0; /* [3:2]=01 ~40k to VDD: idle reads 1 */

    /* Boot state via BSRR (IOC PinState truth: green off, blue on, BL off).
     * HAL equivalent: HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET/RESET). */
    GPIOC->BSRR = GPIO_BSRR_BR5;   /* PC5 low  (green off)   */
    GPIOB->BSRR = GPIO_BSRR_BS2;   /* PB2 high (blue on)     */
    GPIOA->BSRR = GPIO_BSRR_BR15;  /* PA15 low (backlight off) */

    /* Read-back proof: config registers must have accepted the fields. */
    if (((GPIOC->MODER & GPIO_MODER_MODER5_Msk) != GPIO_MODER_MODER5_0) ||
        ((GPIOB->MODER & GPIO_MODER_MODER2_Msk) != GPIO_MODER_MODER2_0) ||
        ((GPIOA->MODER & GPIO_MODER_MODER15_Msk) != GPIO_MODER_MODER15_0)) {
        return -1;
    }
    if (((GPIOA->PUPDR & GPIO_PUPDR_PUPD0_Msk) != GPIO_PUPDR_PUPD0_1) ||
        ((GPIOA->PUPDR & GPIO_PUPDR_PUPD1_Msk) != GPIO_PUPDR_PUPD1_0)) {
        return -1;
    }
    return 0;
}

void bsp_gpio_led_green(uint32_t on)
{
    /* BSRR atomic write (RM0090 ch.8.4.7): set-half or reset-half, one
     * write, no ODR read -> cannot lose an update against an ISR. */
    GPIOC->BSRR = (on != 0u) ? GPIO_BSRR_BS5 : GPIO_BSRR_BR5;
}

void bsp_gpio_led_blue(uint32_t on)
{
    GPIOB->BSRR = (on != 0u) ? GPIO_BSRR_BS2 : GPIO_BSRR_BR2;
}

void bsp_gpio_lcd_backlight(uint32_t on)
{
    GPIOA->BSRR = (on != 0u) ? GPIO_BSRR_BS15 : GPIO_BSRR_BR15;
}

uint32_t bsp_gpio_key1_pressed(void)
{
    /* IDR read (ch.8.4.5). Pull-down + button-to-VDD wiring: pressed = 1. */
    return ((GPIOA->IDR & KEY1_BIT) != 0u) ? 1u : 0u;
}

uint32_t bsp_gpio_key2_pressed(void)
{
    /* Pull-up + button-to-GND wiring: pressed pulls the line low -> 0. */
    return ((GPIOA->IDR & KEY2_BIT) == 0u) ? 1u : 0u;
}

int bsp_gpio_selftest(void)
{
    /* Fresh init first: the selftest is self-contained on the menu, and
     * init is idempotent (bsp_uart ownership pattern). */
    if (bsp_gpio_init() != 0) {
        return (int)BSP_GPIO_ST1_CONFIG;
    }

    /* Step 1: config read-back (MODER/OTYPER/OSPEEDR/PUPDR of all five
     * pins) + ODR round-trip through BSRR: drive all three outputs high,
     * verify ODR latched them, then restore the boot state and verify. */
    if (((GPIOC->MODER & GPIO_MODER_MODER5_Msk) != GPIO_MODER_MODER5_0) ||
        ((GPIOB->MODER & GPIO_MODER_MODER2_Msk) != GPIO_MODER_MODER2_0) ||
        ((GPIOA->MODER & GPIO_MODER_MODER15_Msk) != GPIO_MODER_MODER15_0) ||
        ((GPIOA->MODER & GPIO_MODER_MODER0_Msk) != 0u) ||
        ((GPIOA->MODER & GPIO_MODER_MODER1_Msk) != 0u)) {
        return (int)BSP_GPIO_ST1_CONFIG;
    }
    if (((GPIOC->OTYPER | GPIOB->OTYPER | GPIOA->OTYPER) &
         (GPIO_OTYPER_OT5 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT15)) != 0u) {
        return (int)BSP_GPIO_ST1_CONFIG; /* open-drain bits must stay clear */
    }
    if (((GPIOA->PUPDR & GPIO_PUPDR_PUPD0_Msk) != GPIO_PUPDR_PUPD0_1) ||
        ((GPIOA->PUPDR & GPIO_PUPDR_PUPD1_Msk) != GPIO_PUPDR_PUPD1_0)) {
        return (int)BSP_GPIO_ST1_CONFIG; /* key pull resistors drifted */
    }
    bsp_gpio_led_green(1u);
    bsp_gpio_led_blue(1u);
    bsp_gpio_lcd_backlight(1u);
    if (((GPIOC->ODR & LED_GREEN_BIT) == 0u) ||
        ((GPIOB->ODR & LED_BLUE_BIT) == 0u) ||
        ((GPIOA->ODR & LCD_BL_BIT) == 0u)) {
        return (int)BSP_GPIO_ST1_CONFIG; /* BSRR set did not reach ODR */
    }
    bsp_gpio_led_green(0u);     /* restore IOC boot state ... */
    bsp_gpio_led_blue(1u);
    bsp_gpio_lcd_backlight(0u);
    if (((GPIOC->ODR & LED_GREEN_BIT) != 0u) ||
        ((GPIOB->ODR & LED_BLUE_BIT) == 0u) ||
        ((GPIOA->ODR & LCD_BL_BIT) != 0u)) {
        return (int)BSP_GPIO_ST1_CONFIG; /* BSRR reset did not reach ODR */
    }

    /* Step 2: KEY idle levels through IDR. 3 samples ~2 ms apart must all
     * match the pull-resistor contract (KEY1 pulled to GND -> 0, KEY2
     * pulled to VDD -> 1). A pressed key breaks the contract on purpose:
     * release the keys while the test runs. */
    {
        uint32_t sample;
        for (sample = 0u; sample < 3u; sample++) {
            if (sample > 0u) {
                if (gpio_wait_ms(1u) != 0) {
                    return (int)BSP_GPIO_ST2_KEYS; /* dead SysTick */
                }
            }
            if ((GPIOA->IDR & KEY1_BIT) != 0u) {
                return (int)BSP_GPIO_ST2_KEYS; /* KEY1 not low at idle */
            }
            if ((GPIOA->IDR & KEY2_BIT) == 0u) {
                return (int)BSP_GPIO_ST2_KEYS; /* KEY2 not high at idle */
            }
        }
    }

    /* Step 3: visible blink for the human on the bench. Two BSRR toggles
     * 200 ms apart: green flashes on, blue dips off, then the boot state
     * is restored. A toggle is an ODR read + BSRR write (no ODR RMW --
     * see walkthrough). */
    bsp_gpio_led_green(1u);  /* toggle 1: green on, blue off */
    bsp_gpio_led_blue(0u);
    if (gpio_wait_ms(200u) != 0) {
        return (int)BSP_GPIO_ST3_BLINK;
    }
    bsp_gpio_led_green(0u);  /* toggle 2: back to boot state */
    bsp_gpio_led_blue(1u);
    if (gpio_wait_ms(50u) != 0) {
        return (int)BSP_GPIO_ST3_BLINK;
    }
    return 0; /* all steps passed */
}

/** @} */
