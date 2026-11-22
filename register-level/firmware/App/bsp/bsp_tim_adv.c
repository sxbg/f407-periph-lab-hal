/**
 ******************************************************************************
 * @file    bsp_tim_adv.c
 * @brief   TIM8 complementary PWM + dead-time generator -- teaching module
 *          with register-level init and self-test (EP14 worked example).
 ******************************************************************************
 * @addtogroup BSP_TIM_ADV
 * @{
 *
 * == CUBEMX CONFIG (from f407-periph-lab.ioc) ==============================
 * | Field             | Value (IOC truth)                    | Why
 * |-------------------|--------------------------------------|-------------------
 * | TIM8 mode         | PWM Generation1 CH1 + PWM Generation2| IOC routed the
 * |                   | CH2, CH2N=CCxE enable                | pins; generated
 * |                   |                                      | code stays at
 * |                   |                                      | base init --
 * |                   |                                      | channel regs
 * |                   |                                      | owned here
 * | TIM8 Prescaler    | 0 (base init default)                | module picks
 * |                   |                                      | its own below
 * | Pins              | PC6=TIM8_CH1, PC7=TIM8_CH2 (AF3),    | board header
 * |                   | PB14=TIM8_CH2N (AF3)                 | hooks
 * | Kernel clock      | APB2 x2 = 168 MHz                    | RM0090 ch.6
 * == REGISTER WALKTHROUGH (RM0090 ch.17, TIM1/TIM8 advanced timers) ========
 * RCC->APB2ENR bit1 (TIM8EN): kernel clock gate, re-armed idempotently.
 * Pin mux: PC6/PC7 -> AF3 (TIM8_CH1/CH2): MODER [17:16]/[15:14] = 10,
 *   AFRL nibbles 6/7 = 0011. PB14 -> AF3 (TIM8_CH2N): MODER [29:28] = 10,
 *   AFRH nibble 6 = 0011. Push-pull, low slew, no pull (IOC truth). HAL
 *   equivalent in MspInit: GPIO_AF3_TIM8 + HAL_GPIO_Init.
 * Frame (same two-step divider as every timer):
 *   TIM8: 168 MHz / (167+1) = 1 MHz tick; 1 MHz / (999+1) = 1 kHz frame.
 * CCMR1 -- BOTH channels in one register: low byte = channel 1, high byte
 *   = channel 2. Per channel: CCxS=00 output, OCxM=110 (PWM mode 1:
 *   active while CNT < CCR), OCxPE=1 (preloaded compare, glitch-free).
 * CCER (ch.17.4.9) -- the complementary map, one bit per output:
 *   CC1E bit0 = 1  -> CH1 to PC6 (plain, no N-channel for CH1 in this
 *                     IOC: CH1N never enabled).
 *   CC2E bit4 = 1  -> CH2 (OC2) to PC7.
 *   CC2NE bit6 = 1 -> CH2N (OC2N) to PB14 -- the complement of OC2.
 *   CC1P/CC2P/CC2NP = 0 -> all outputs active HIGH (both phases of the
 *   pair share the polarity; the dead time is what keeps them apart).
 * BDTR (ch.17.4.18) -- the advanced-timer's own control deck:
 *   DTG [7:0] = 0x94 -> 1 us dead time (derivation below, EP14 worked
 *              example).
 *   MOE bit15 = 1 -> main output enable. Without MOE every CCER enable is
 *              dead and PC6/PC7/PB14 stay flat -- the classic trap (same
 *              bit TIM1 needed in bsp_tim_pwm).
 *   OSSI/OSSR = 0, LOCK = 0, BKIN stage NOT configured: this lab board has
 *   no break source wired to PB12; the break/reset path is documented as
 *   deliberately untouched (setting BKIN inputs with nothing driving them
 *   would only add a way for MOE to be cleared asynchronously).
 * *DEAD-TIME DERIVATION (the EP14 exercise, step by step)*
 *   CKD[9:8] = 00 in CR1 -> f_DTS = f_CK_INT = 168 MHz
 *                          -> t_DTS = 1/168 MHz = 5.952 ns.
 *   Target dead time: 1 us = 1000 ns -> ticks = 1000 / 5.952 = 168.0.
 *   RM0090 DTG encoding table:
 *     DTG[7:5]=0xx -> DT = DTG[7:0]        * t_DTS  (max 127  = 756 ns)
 *     DTG[7:6]=10  -> DT = (64 + DTG[5:0]) * 2*t_DTS (128..254 ticks)
 *     DTG[7:5]=110 -> DT = (32 + DTG[4:0]) * 8*t_DTS (256..504 ticks)
 *     DTG[7:5]=111 -> DT = (32 + DTG[4:0]) * 16*t_DTS(256..1008 ticks)
 *   168 ticks > 127: range 1 does not reach -> range 2:
 *     DTG[5:0] = ticks/2 - 64 = 84 - 64 = 20 = 0x14
 *     DTG byte = 10_010100b = 0x94.
 *   Realized DT = (64+20) * 2 * 5.952 ns = 168 * 5.952 = 999.9 ns --
 *   quantization step in range 2 = 2*t_DTS = 11.9 ns, error -0.07 ns.
 * RCR = 0 -> update on every overflow. EGR.UG latches PSC/ARR/RCR
 *   shadows and zeroes CNT (raises UIF; SR is rc_w0, cleared right after).
 * CR1: CEN=1 LAST; ARPE=0 (IOC has no preload key: immediate ARR), DIR=0
 *   up, CMS=00 edge-aligned, CKD=00 (f_DTS = f_CK_INT -- the dead-time
 *   generator's clock source per the derivation above).
 * == HOW TO VERIFY =========================================================
 * menu> tim_adv
 *   PASS tim_adv (steps: 1 DTG/CCER/MOE, 2 ns round-trip, 3 polarities)
 * Scope checks: PC6 = 1 kHz 50% PWM. PC7/PB14 = the complementary pair:
 *   both 1 kHz, 50%, active-high, and PB14 is the exact inverse of PC7
 *   EXCEPT the 1 us window around each transition where BOTH are low (the
 *   dead time -- zoom in: no shoot-through overlap is possible by
 *   construction). Dead-time knob: set_deadtime_ns(5000) -> 4.98 us gaps.
 * Bench cross-check: if PB14 doubles as a board LED, the dead-time gaps
 *   are invisible; a scope or the logic-analyzer menu (la) on PC7+PB14 is
 *   the honest meter.
 */
#include "bsp_tim_adv.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

/* ---- module constants ----------------------------------------------------- */

/* 1 kHz frame derivation (see walkthrough): 168 MHz/(167+1) = 1 MHz tick,
 * (999+1) ticks = 1 kHz. Checked by read-back in selftest. */
#define BSP_TIM_ADV_PSC 167u
#define BSP_TIM_ADV_ARR 999u

/* Self-test step numbers (the value bsp_tim_adv_selftest returns). */
#define BSP_TIM_ADV_ST1_CONFIG  1u /* DTG / CCER / MOE read-back */
#define BSP_TIM_ADV_ST2_DT_ROUND 2u /* ns round-trip within one step */
#define BSP_TIM_ADV_ST3_POL     3u /* CC2P / CC2NP polarity read-back */

/* Dead-time bounds accepted by bsp_tim_adv_set_deadtime_ns: 1 tick
 * (~6 ns) up to the range-4 ceiling (1008 ticks = 6048 ns). */
#define BSP_TIM_ADV_DT_MAX_TICKS 1008u
#define BSP_TIM_ADV_DT_MAX_NS \
    ((BSP_TIM_ADV_DT_MAX_TICKS * BSP_TIM_ADV_TDTS_NS_NUM) / \
     BSP_TIM_ADV_TDTS_NS_DEN) /* = 6048 ns */

/* ---- public API ------------------------------------------------------------ */

int bsp_tim_adv_init(void)
{
    /* Kernel clock: TIM8EN = APB2ENR bit1 (RM0090 ch.6). Idempotent
     * re-arm of the HAL MspInit bit. */
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    (void)RCC->APB2ENR; /* read-back stalls until the clock is live */

    /* Stop the counter; own the whole CR1 (CEN=0, CKD=00, ARPE=0). */
    TIM8->CR1 = 0u;

    /* --- pin mux, register level -----------------------------------------
     * PC6 -> AF3 (TIM8_CH1), PC7 -> AF3 (TIM8_CH2): both live in AFR[0]
     * (pins < 8), nibbles 6 and 7 = 0011. PB14 -> AF3 (TIM8_CH2N): AFR[1]
     * nibble 6 = 0011. Push-pull, low slew, no pull (IOC truth). */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER6_Msk) |
                   GPIO_MODER_MODER6_1; /* [13:12]=10 AF mode */
    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk) |
                    GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL6_1; /* AF3 TIM8 */
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT6; /* push-pull */
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6_Msk; /* low slew, per IOC */
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD6_Msk;       /* no pull */

    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER7_Msk) |
                   GPIO_MODER_MODER7_1; /* [15:14]=10 AF mode */
    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~GPIO_AFRL_AFSEL7_Msk) |
                    GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_1; /* AF3 TIM8 */
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT7;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7_Msk;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD7_Msk;

    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER14_Msk) |
                   GPIO_MODER_MODER14_1; /* [29:28]=10 AF mode */
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~GPIO_AFRH_AFSEL14_Msk) |
                    GPIO_AFRH_AFSEL14_0 | GPIO_AFRH_AFSEL14_1; /* AF3 CH2N */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED14_Msk;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD14_Msk;

    /* --- timebase: PSC/ARR/RCR + one UG stroke --------------------------- */
    TIM8->PSC = BSP_TIM_ADV_PSC;
    TIM8->ARR = BSP_TIM_ADV_ARR;
    TIM8->RCR = 0u;         /* update on EVERY overflow */
    TIM8->EGR = TIM_EGR_UG; /* latch PSC/ARR shadows, CNT=0 (sets UIF) */
    TIM8->SR = ~TIM_SR_UIF; /* clear the UG-raised UIF (rc_w0) */

    /* --- both channels: PWM mode 1, preloaded compare --------------------- */
    /* CCMR1 low byte = CH1, high byte = CH2: CCxS=00 output, OCxM=110
     * PWM1, OCxPE=1. HAL equivalent per channel:
     *   TIM_OC_InitTypeDef oc = { .OCMode = TIM_OCMODE_PWM1,
     *                             .Pulse = 500, .OCPolarity = HIGH };
     *   HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_x); */
    TIM8->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    TIM8->CCR1 = 500u; /* 50% duty on both (500 of 1000 ticks) */
    TIM8->CCR2 = 500u;

    /* --- output enables: CC1E + CC2E + CC2NE, active-high ----------------- */
    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE;
    /* CC1P/CC2P/CC2NP all 0 in that write: every phase active HIGH. */

    /* --- dead time: 1 us via the EP14 derivation -> DTG = 0x94 ------------ */
    TIM8->BDTR = TIM_BDTR_MOE | 0x94u; /* MOE + (64+20)*2*t_DTS = 999.9 ns.
                                        * Break stage deliberately NOT
                                        * configured (walkthrough note). */

    /* --- start ------------------------------------------------------------- */
    TIM8->CR1 = TIM_CR1_CEN; /* HAL: HAL_TIM_PWM_Start(all channels) */

    /* Read-back proof: enables, main output, DTG byte, frame. */
    if ((TIM8->PSC != BSP_TIM_ADV_PSC) || (TIM8->ARR != BSP_TIM_ADV_ARR)) {
        return -1;
    }
    if ((TIM8->CCER &
         (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE)) !=
        (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE)) {
        return -1;
    }
    if (((TIM8->BDTR & TIM_BDTR_MOE) == 0u) ||
        ((TIM8->BDTR & TIM_BDTR_DTG_Msk) != 0x94u)) {
        return -1;
    }
    return 0;
}

int bsp_tim_adv_set_deadtime_ns(uint32_t ns)
{
    /* Convert ns -> t_DTS ticks with round-half-up: ticks = ns / 5.952.
     * The 10000/1680 fraction is exact for 168 MHz (walkthrough). */
    const uint32_t ticks = ((ns * BSP_TIM_ADV_TDTS_NS_DEN) +
                            (BSP_TIM_ADV_TDTS_NS_NUM / 2u)) /
                           BSP_TIM_ADV_TDTS_NS_NUM;
    uint32_t dtg;

    if ((ns == 0u) || (ns > BSP_TIM_ADV_DT_MAX_NS)) {
        return -1; /* reject with no write: fail loudly, not silently */
    }

    /* Walk the four DTG ranges bottom-up (RM0090 BDTR table). */
    if (ticks <= 127u) {
        dtg = ticks; /* range 1: DT = DTG * t_DTS */
    } else if (ticks <= 254u) {
        /* range 2: DT = (64+DTG[5:0]) * 2*t_DTS, 2-tick steps -> round */
        dtg = 0x80u + ((ticks + 1u) / 2u) - 64u;
    } else if (ticks <= 504u) {
        /* range 3: DT = (32+DTG[4:0]) * 8*t_DTS, 8-tick steps */
        dtg = 0xC0u + ((ticks + 4u) / 8u) - 32u;
    } else {
        /* range 4: DT = (32+DTG[4:0]) * 16*t_DTS, 16-tick steps */
        dtg = 0xE0u + ((ticks + 8u) / 16u) - 32u;
    }

    TIM8->BDTR = (TIM8->BDTR & ~TIM_BDTR_DTG_Msk) |
                 (dtg & TIM_BDTR_DTG_Msk); /* keep MOE, swap DTG only */
    return 0;
}

uint32_t bsp_tim_adv_get_deadtime_ns(void)
{
    const uint32_t dtg = TIM8->BDTR & TIM_BDTR_DTG_Msk;
    uint32_t ticks;

    /* Exact inverse of the set() walk. */
    if ((dtg & 0x80u) == 0u) {
        ticks = dtg; /* DTG[7:5]=0xx: DT = DTG * t_DTS */
    } else if ((dtg & 0xC0u) == 0x80u) {
        ticks = (64u + (dtg & 0x3Fu)) * 2u;
    } else if ((dtg & 0xE0u) == 0xC0u) {
        ticks = (32u + (dtg & 0x1Fu)) * 8u;
    } else {
        ticks = (32u + (dtg & 0x1Fu)) * 16u;
    }
    /* ticks -> ns, round-half-up: ns = ticks * (10000/1680). */
    return ((ticks * BSP_TIM_ADV_TDTS_NS_NUM) +
            (BSP_TIM_ADV_TDTS_NS_DEN / 2u)) / BSP_TIM_ADV_TDTS_NS_DEN;
}

int bsp_tim_adv_selftest(void)
{
    /* Fresh init first (idempotent, menu-driven selftest pattern). */
    if (bsp_tim_adv_init() != 0) {
        return (int)BSP_TIM_ADV_ST1_CONFIG;
    }

    /* Step 1: DTG / CCER / MOE read-back -- the exact shape the init
     * derived from the 1 us target. */
    if (((TIM8->BDTR & TIM_BDTR_MOE) == 0u) ||
        ((TIM8->BDTR & TIM_BDTR_DTG_Msk) != 0x94u) ||
        ((TIM8->CCER &
          (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE)) !=
         (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE)) ||
        ((TIM8->CR1 & TIM_CR1_CEN) == 0u)) {
        return (int)BSP_TIM_ADV_ST1_CONFIG;
    }

    /* Step 2: dead-time round-trip through the register: 1000 ns in ->
     * DTG=0x94 in BDTR -> decode back must land within one quantization
     * step (11.9 ns; range-2 realized value is 999.9 ns, error -0.07 ns).
     * A second knob value (200 ns, range 1, DTG=34=0x22, exact 202 ns...
     * 34 ticks * 5.952 = 202.4 ns) proves the multi-range encoder. */
    if (bsp_tim_adv_set_deadtime_ns(1000u) != 0) {
        return (int)BSP_TIM_ADV_ST2_DT_ROUND;
    }
    {
        const uint32_t got = bsp_tim_adv_get_deadtime_ns();
        const uint32_t diff = (got > 1000u) ? (got - 1000u)
                                            : (1000u - got);
        if (diff > 12u) {
            return (int)BSP_TIM_ADV_ST2_DT_ROUND;
        }
    }
    if (bsp_tim_adv_set_deadtime_ns(200u) != 0) {
        return (int)BSP_TIM_ADV_ST2_DT_ROUND;
    }
    {
        /* 200 ns -> 34 ticks (range 1) -> 202.4 ns realized: still inside
         * one 5.952 ns step of the target. Restore the 1 us default
         * afterwards so the scope picture stays the init default. */
        const uint32_t got = bsp_tim_adv_get_deadtime_ns();
        const uint32_t diff = (got > 200u) ? (got - 200u)
                                           : (200u - got);
        if (diff > 6u) {
            return (int)BSP_TIM_ADV_ST2_DT_ROUND;
        }
        (void)bsp_tim_adv_set_deadtime_ns(1000u);
    }

    /* Step 3: complementary polarity -- CC2P=0 AND CC2NP=0. Both phases
     * active-high: the complementary pair is symmetric, only the dead
     * time separates OC2 from OC2N (asymmetric polarity would ALSO be
     * legal hardware -- CCxP/CCxNP exist for that -- but it is not what
     * this lab wires, and the bits prove the read-back discipline). */
    if ((TIM8->CCER & (TIM_CCER_CC2P | TIM_CCER_CC2NP)) != 0u) {
        return (int)BSP_TIM_ADV_ST3_POL;
    }
    return 0; /* all steps passed */
}

/** @} */
