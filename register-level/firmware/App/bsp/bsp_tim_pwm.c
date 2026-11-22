/**
 ******************************************************************************
 * @file    bsp_tim_pwm.c
 * @brief   TIM1_CH1 (PA8) + TIM4_CH1 (PD12) PWM at 1 kHz -- teaching module
 *          with register-level init and self-test.
 ******************************************************************************
 * @addtogroup BSP_TIM_PWM
 * @{
 *
 * == CUBEMX CONFIG (from f407-periph-lab.ioc) ==============================
 * | Field            | Value (IOC truth)                    | Why
 * |------------------|--------------------------------------|-------------------
 * | TIM1 mode        | PWM Generation CH1 (PA8, AF1)        | IOC routed the
 * |                  |                                      | pin; the generated
 * |                  |                                      | code stays at base
 * |                  |                                      | init (PSC=0 ARR=max
 * |                  |                                      | CEN=0) -- channel
 * |                  |                                      | regs owned here
 * | TIM4 mode        | PWM Generation CH1 (PD12, AF2)       | same split
 * | Kernel clocks    | TIM1 on APB2 x2 = 168 MHz            | RM0090 ch.6:
 * |                  | TIM4 on APB1 x2 = 84 MHz             | timer clk = 2x PCLK
 * |                  |                                      | when the bus divides
 * | PSC/ARR in IOC   | 0 / 65535 (base init defaults)       | module picks its
 * |                  |                                      | own values below
 * == REGISTER WALKTHROUGH (TIM1: RM0090 ch.17, TIM4: ch.18) =================
 * RCC->APB2ENR bit0 (TIM1EN) / RCC->APB1ENR bit2 (TIM4EN): kernel clock
 *   gates; re-armed idempotently (HAL MspInit already set them).
 * Frequency (the two-step divider every STM32 timer shares):
 *   f_CK_CNT = f_tim / (PSC + 1)        -- PSC is BUFFERED: a new value is
 *      latched into the shadow register only at the next update event, so
 *      init follows the write with EGR.UG (update generation, ch.17.4.5/
 *      ch.18.4.5) to load PSC/ARR and reset CNT in one stroke.
 *   f_frame  = f_CK_CNT / (ARR + 1)     -- ARR is the counter's reload tap:
 *      on reaching ARR the counter wraps to 0 and (up-counter) raises UIF.
 *   TIM1: 168000000 / (167+1) = 1 MHz tick;   1 MHz / (999+1) = 1000 Hz.
 *   TIM4:  84000000 / (83+1)  = 1 MHz tick;   1 MHz / (999+1) = 1000 Hz.
 *   Both channels end at identical 1 MHz/1 kHz -- deliberately, so duty
 *   math (below) is shared while the kernel clocks differ 2:1.
 * CCMR1 (output compare mode, ch.17.4.7/ch.18.4.7):
 *   CC1S [1:0]   = 00 -> channel 1 configured as OUTPUT (the same bits act
 *                  as CC2S input-select when a channel is an input -- see
 *                  bsp_tim_ic for the input-mode twin).
 *   OC1M [6:4]   = 110 -> PWM mode 1: output ACTIVE while CNT < CCR
 *                  (mode 2 is the inverted logic).
 *   OC1PE bit3   = 1  -> compare preload: CCR1 writes land in a preload
 *                  register and transfer to the active (shadow) compare at
 *                  the next update event -- duty changes are frame-aligned
 *                  and glitch-free instead of mid-frame racy.
 * CCER (ch.17.4.9/ch.18.4.9):
 *   CC1E bit0    = 1  -> output 1 driven to the pin.
 *   CC1P bit1    = 0  -> active HIGH (mode 1 active level = high).
 *   CC1NP bit3   = 0  -> complementary output N unused.
 * BDTR (TIM1 ONLY, ch.17.4.18): MOE bit15 = 1 -> main output enable. This
 *   is THE advanced-timer trap: TIM1/TIM8 route outputs through a break/
 *   dead-time stage; with MOE=0 every CCER enable is dead and PA8 stays
 *   flat. TIM4 has no BDTR -- nothing extra to set.
 * RCR (TIM1 ONLY, ch.17.4.16): repetition counter 0 -> update on every
 *   overflow (RCR>1 would emit update events only every RCR+1 overflows).
 * CR1 (ch.17.4.1/ch.18.4.1): CEN bit0 = 1 counter enable (written LAST,
 *   after the whole channel is armed); ARPE bit7 = 0 (ARR without preload,
 *   IOC AutoReloadPreload=DISABLE: an ARR write takes effect immediately);
 *   DIR bit4 = 0 up-counter; CMS [6:5] = 00 edge-aligned; CKD [9:8] = 00
 *   (f_DTS = f_CK_INT, matters only for the input filters).
 * Duty (the API math): mode 1 active while CNT < CCR, so the active slice
 *   is CCR ticks of the (ARR+1)-tick frame:
 *   CCR = round((ARR+1) * per_mille / 1000). per_mille=500 -> CCR=500 of
 *   1000 ticks = 50% duty; 1000 -> CCR=1000 > ARR=999 -> always active
 *   (100%); 0 -> never active (0%).
 * == HOW TO VERIFY =========================================================
 * menu> tim_pwm
 *   PASS tim_pwm         (steps: 1 PSC/ARR, 2 CCR@500, 3 CCxE/MOE)
 * Scope checks: PA8 and PD12 each show a 1 kHz square wave, 3.3 V swing,
 *   500 us high / 500 us low at the default 50% duty. Drive duty from the
 *   menu (later wave): set_duty(100) -> 100 us high; set_duty(0) -> flat
 *   low; set_duty(1000) -> flat high.
 * Cross-check: jumper PD12 -> PB3 and run menu 'tim_ic' -- the input
 *   capture meter should report ~1000 Hz (two independent timer modules
 *   measuring each other).
 */
#include "bsp_tim_pwm.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* 1 kHz frame derivation (see walkthrough): both timers tick at 1 MHz and
 * wrap every 1000 ticks. Named once, checked by read-back in selftest. */
#define BSP_TIM_PWM_TIM1_PSC 167u /* 168 MHz / (167+1)  = 1 MHz */
#define BSP_TIM_PWM_TIM4_PSC 83u  /*  84 MHz / (83+1)   = 1 MHz */
#define BSP_TIM_PWM_ARR      999u /* (999+1) ticks      = 1 kHz */

/* Self-test step numbers (the value bsp_tim_pwm_selftest returns). */
#define BSP_TIM_PWM_ST1_PSC_ARR 1u /* PSC/ARR read-back */
#define BSP_TIM_PWM_ST2_CCR     2u /* duty round-trip @ 500 per-mille */
#define BSP_TIM_PWM_ST3_ENABLE  3u /* CCxE / MOE read-back */

/* ---- public API ------------------------------------------------------------ */

int bsp_tim_pwm_init(void)
{
    /* Kernel clocks: TIM1EN = APB2ENR bit0, TIM4EN = APB1ENR bit2
     * (RM0090 ch.6). Idempotent re-arm of the HAL MspInit bits. */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    (void)RCC->APB2ENR; /* read-back stalls the core until clocks are live */
    (void)RCC->APB1ENR;

    /* Stop both counters before touching channel regs (CEN=0). HAL left
     * them stopped (base init never starts the counter); the explicit
     * clear makes the module own the full CR1 rather than inherit it. */
    TIM1->CR1 = 0u;
    TIM4->CR1 = 0u;

    /* --- pin mux, register level -----------------------------------------
     * PA8 -> AF1 (TIM1_CH1): MODER8 [17:16]=10 (alternate function),
     * AFR[1] nibble 0 = 0001 (AF1). PD12 -> AF2 (TIM4_CH1): MODER12
     * [25:24]=10, AFR[1] nibble 4 = 0010 (AF2). Push-pull, low speed, no
     * pull (IOC truth). HAL equivalent in MspInit:
     *   GPIO_InitStruct.Alternate = GPIO_AF1_TIM1 / GPIO_AF2_TIM4;
     *   HAL_GPIO_Init(...). */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER8_Msk) |
                   GPIO_MODER_MODER8_1; /* [17:16]=10 AF mode */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL8_Msk) |
                    GPIO_AFRH_AFSEL8_0; /* AFSEL8[3:0]=0001 -> AF1 TIM1 */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT8;   /* push-pull */
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8_Msk; /* low slew, per IOC */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk;       /* no pull */

    GPIOD->MODER = (GPIOD->MODER & ~GPIO_MODER_MODER12_Msk) |
                   GPIO_MODER_MODER12_1; /* [25:24]=10 AF mode */
    GPIOD->AFR[1] = (GPIOD->AFR[1] & ~GPIO_AFRH_AFSEL12_Msk) |
                    GPIO_AFRH_AFSEL12_1; /* AFSEL12[3:0]=0010 -> AF2 TIM4 */
    GPIOD->OTYPER &= ~GPIO_OTYPER_OT12;   /* push-pull */
    GPIOD->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED12_Msk; /* low slew */
    GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD12_Msk;       /* no pull */

    /* --- timebase: PSC/ARR + one UG stroke ------------------------------- */
    /* PSC buffered (RM0090 ch.17.4.10/ch.18.4.10): takes effect at the next
     * update event -- hence the EGR.UG right after, which also zeroes CNT.
     * HAL equivalent: htimX.Init.Prescaler/Period; HAL_TIM_Base_Init. */
    TIM1->PSC = BSP_TIM_PWM_TIM1_PSC;
    TIM1->ARR = BSP_TIM_PWM_ARR;
    TIM1->RCR = 0u; /* repetition counter 0: update on EVERY overflow */
    TIM1->EGR = TIM_EGR_UG;  /* latch PSC/ARR shadows, CNT=0 (sets UIF!) */
    TIM1->SR = ~TIM_SR_UIF;  /* UG raised UIF; clear it (SR is rc_w0) */
    TIM4->PSC = BSP_TIM_PWM_TIM4_PSC;
    TIM4->ARR = BSP_TIM_PWM_ARR;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = ~TIM_SR_UIF;

    /* --- channel 1 shape: PWM mode 1, preloaded compare ------------------ */
    /* CC1S=00 output, OC1M=110 PWM1 (active while CNT<CCR), OC1PE=1
     * frame-aligned duty updates. HAL equivalent:
     *   TIM_OC_InitTypeDef oc = { .OCMode = TIM_OCMODE_PWM1,
     *                             .Pulse = 500, .OCPolarity = HIGH,
     *                             .OCFastMode = DISABLE };
     *   HAL_TIM_PWM_ConfigChannel(&htimX, &oc, TIM_CHANNEL_1); */
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM4->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;

    /* --- output enables: CC1E, active-high, then TIM1 main output -------- */
    TIM1->CCER = TIM_CCER_CC1E; /* CC1E=1, CC1P/CC1NP=0: active high */
    TIM4->CCER = TIM_CCER_CC1E;
    TIM1->BDTR = TIM_BDTR_MOE;  /* TIM1 ONLY (ch.17.4.18): without MOE the
                                 * break stage keeps PA8 silent forever --
                                 * the classic advanced-timer trap. */
    TIM1->CCR1 = 500u;          /* preload 50% duty (transfers at first
                                 * update event); HAL: oc.Pulse = 500. */
    TIM4->CCR1 = 500u;

    /* --- start, CR1 in one write ------------------------------------------ */
    /* CEN=1 last; ARPE=0 (IOC preload disable), DIR=0 up, CMS=00 edge,
     * CKD=00. HAL equivalent: HAL_TIM_PWM_Start(&htimX, TIM_CHANNEL_1). */
    TIM1->CR1 = TIM_CR1_CEN;
    TIM4->CR1 = TIM_CR1_CEN;

    /* Read-back proof: the timebase and the enables must have stuck. */
    if ((TIM1->PSC != BSP_TIM_PWM_TIM1_PSC) ||
        (TIM4->PSC != BSP_TIM_PWM_TIM4_PSC) ||
        (TIM1->ARR != BSP_TIM_PWM_ARR) ||
        (TIM4->ARR != BSP_TIM_PWM_ARR)) {
        return -1;
    }
    if (((TIM1->CCER & TIM_CCER_CC1E) == 0u) ||
        ((TIM4->CCER & TIM_CCER_CC1E) == 0u) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) == 0u)) {
        return -1;
    }
    return 0;
}

int bsp_tim_pwm_set_duty(uint32_t ch, uint32_t per_mille)
{
    /* Reject out-of-range duty without touching registers: 1001 would
     * silently wrap the rounding math, so fail loudly instead. */
    if (per_mille > 1000u) {
        return -1;
    }
    /* CCR = round((ARR+1) * per_mille / 1000), +500 for round-half-up.
     * Max operand 1000*1000 = 1e6 fits uint32_t comfortably. Writes land
     * in the CCR1 preload register (OC1PE=1): the active compare updates
     * at the next frame boundary -- 1 ms worst case, never mid-frame. */
    {
        const uint32_t ccr = ((BSP_TIM_PWM_ARR + 1u) * per_mille + 500u) /
                             1000u;
        if (ch == BSP_TIM_PWM_CH_TIM1_CH1) {
            TIM1->CCR1 = ccr;
        } else if (ch == BSP_TIM_PWM_CH_TIM4_CH1) {
            TIM4->CCR1 = ccr;
        } else {
            return -1; /* unknown channel selector */
        }
    }
    return 0;
}

int bsp_tim_pwm_selftest(void)
{
    /* Fresh init first: menu-driven selftest is self-contained, init is
     * idempotent (bsp_uart ownership pattern). */
    if (bsp_tim_pwm_init() != 0) {
        return (int)BSP_TIM_PWM_ST1_PSC_ARR;
    }

    /* Step 1: timebase read-back -- the exact 1 kHz derivation values. */
    if ((TIM1->PSC != BSP_TIM_PWM_TIM1_PSC) ||
        (TIM4->PSC != BSP_TIM_PWM_TIM4_PSC) ||
        (TIM1->ARR != BSP_TIM_PWM_ARR) ||
        (TIM4->ARR != BSP_TIM_PWM_ARR)) {
        return (int)BSP_TIM_PWM_ST1_PSC_ARR;
    }

    /* Step 2: duty round-trip. set_duty(500) must write CCR=500 on both
     * timers (1 MHz tick, 1000-tick frame): |500 - ARR/2| = |500-499.5|
     * <= 1, the spec tolerance. Reading CCRx reads the preload register
     * (RM0090 ch.17.4.13/ch.18.4.13), so no wait for the update event is
     * needed. */
    if (bsp_tim_pwm_set_duty(BSP_TIM_PWM_CH_TIM1_CH1, 500u) != 0) {
        return (int)BSP_TIM_PWM_ST2_CCR;
    }
    if (bsp_tim_pwm_set_duty(BSP_TIM_PWM_CH_TIM4_CH1, 500u) != 0) {
        return (int)BSP_TIM_PWM_ST2_CCR;
    }
    {
        /* ARR/2 = 499.5: accept 499..500 on both channels. */
        const uint32_t ccr1 = TIM1->CCR1;
        const uint32_t ccr4 = TIM4->CCR1;
        if ((ccr1 < 499u) || (ccr1 > 500u) ||
            (ccr4 < 499u) || (ccr4 > 500u)) {
            return (int)BSP_TIM_PWM_ST2_CCR;
        }
    }

    /* Step 3: enable bits -- CCER CC1E on both timers plus TIM1 BDTR MOE.
     * This is the pair a scope outage always comes back to: CC1E without
     * MOE (TIM1) still outputs nothing. */
    if (((TIM1->CCER & TIM_CCER_CC1E) == 0u) ||
        ((TIM4->CCER & TIM_CCER_CC1E) == 0u) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) == 0u) ||
        ((TIM1->CR1 & TIM_CR1_CEN) == 0u) ||
        ((TIM4->CR1 & TIM_CR1_CEN) == 0u)) {
        return (int)BSP_TIM_PWM_ST3_ENABLE;
    }
    return 0; /* all steps passed */
}

/** @} */
