/**
 * @file bsp_tim_pwm.c
 * @brief TIM1_CH1 (PA8) + TIM4_CH1 (PD12) PWM at 1 kHz -- teaching module
 *        (HAL edition of f407-periph-lab bsp_tim_pwm).
 *
 * == WHAT CUBEMX ALREADY DID (MX_TIM1_Init / MX_TIM4_Init + MspInit) ======
 * | Item          | Value (IOC truth)                | Why
 * |---------------|----------------------------------|--------------------
 * | TIM1 mode     | PWM Generation CH1 (PA8, AF1)    | pin routed by
 * |               |                                  | MspInit; generated
 * |               |                                  | code stays at base
 * |               |                                  | init (PSC=0,
 * |               |                                  | ARR=65535, CEN=0)
 * | TIM4 mode     | PWM Generation CH1 (PD12, AF2)   | same split
 * | Kernel clocks | TIM1 on APB2 x2 = 168 MHz        | RM0090 ch.6: timer
 * |               | TIM4 on APB1 x2 = 84 MHz         | clk = 2x PCLK when
 * |               |                                  | the bus divides
 * | ARPE          | DISABLE both (IOC)               | ARR write is
 * |               |                                  | immediate (no shadow)
 * == HAL MAPPING (register concept -> HAL call) ============================
 *   PSC write (+ the old EGR.UG stroke) __HAL_TIM_SET_PRESCALER. PSC is
 *       BUFFERED (latched into the shadow at the next update event) and
 *       the EGR.UG immediate-load stroke is not reachable through HAL
 *       calls, so the FIRST frame after start runs at the stale PSC=0:
 *       1000 ticks at f_tim = one ~6 us (TIM1) / ~12 us (TIM4) startup
 *       frame before the 1 kHz cadence locks in. Accepted for a teaching
 *       wave; noted at the write site.
 *   ARR write ......................... __HAL_TIM_SET_AUTORELOAD. ARPE=0
 *       (IOC preload disable): the write is effective immediately.
 *   CCMR1 OC1M=110 + OC1PE=1 .......... HAL_TIM_PWM_ConfigChannel
 *       (TIM_OCMODE_PWM1; the preload is what makes duty updates frame-
 *       aligned and glitch-free instead of mid-frame racy).
 *   CCER CC1E + BDTR MOE + CR1 CEN .... HAL_TIM_PWM_Start: sets CC1E, on
 *       the advanced TIM1 also __HAL_TIM_MOE_ENABLE, and enables the
 *       counter LAST. MOE is THE advanced-timer trap: without it every
 *       CCER enable is dead and PA8 stays flat.
 *   CCR1 duty ......................... __HAL_TIM_SET_COMPARE /
 *       __HAL_TIM_GET_COMPARE (reading CCR reads the preload register,
 *       RM0090 ch.17.4.13/ch.18.4.13 -- the selftest needs no wait for
 *       the update event).
 *   Idempotent re-init ................ HAL_TIM_PWM_Stop first: HAL keeps
 *       a per-channel BUSY/READY state machine and Start on a BUSY
 *       channel is a HAL_ERROR; Stop restores READY and drops
 *       CC1E/MOE/CEN (the register edition simply wrote CR1=0).
 * Frequency (the two-step divider every STM32 timer shares):
 *   f_CK_CNT = f_tim / (PSC + 1);  f_frame = f_CK_CNT / (ARR + 1)
 *   TIM1: 168000000 / (167+1) = 1 MHz tick; 1 MHz / (999+1) = 1000 Hz.
 *   TIM4:  84000000 / (83+1)  = 1 MHz tick; 1 MHz / (999+1) = 1000 Hz.
 *   Both channels end at identical 1 MHz/1 kHz -- deliberately, so the
 *   duty math is shared while the kernel clocks differ 2:1.
 * Duty (the API math): mode 1 active while CNT < CCR, so the active slice
 *   is CCR ticks of the (ARR+1)-tick frame:
 *   CCR = round((ARR+1) * per_mille / 1000). per_mille=500 -> 50% duty;
 *   1000 -> always active (100%); 0 -> never active (0%).
 * == HOW TO VERIFY =========================================================
 * menu> tim_pwm
 *   PASS tim_pwm         (steps: 1 PSC/ARR, 2 CCR@500, 3 CCxE/MOE)
 * Scope checks: PA8 and PD12 each show a 1 kHz square wave, 3.3 V swing,
 *   500 us high / 500 us low at the default 50% duty. Drive duty from the
 *   menu: set_duty(100) -> 100 us high; set_duty(0) -> flat low;
 *   set_duty(1000) -> flat high.
 * Cross-check: jumper PD12 -> PB3 and run menu 'tim_ic' -- the input
 *   capture meter should report ~1000 Hz (two independent timer modules
 *   measuring each other).
 */
#include "bsp_tim_pwm.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "tim.h" /* htim1/htim4 from MX_TIMx_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* 1 kHz frame derivation (see file header): both timers tick at 1 MHz and
 * wrap every 1000 ticks. Named once, checked by read-back in selftest. */
#define BSP_TIM_PWM_TIM1_PSC 167u /* 168 MHz / (167+1)  = 1 MHz */
#define BSP_TIM_PWM_TIM4_PSC 83u  /*  84 MHz / (83+1)   = 1 MHz */
#define BSP_TIM_PWM_ARR      999u /* (999+1) ticks      = 1 kHz */

/* Self-test step numbers (the value bsp_tim_pwm_selftest returns). */
#define BSP_TIM_PWM_ST1_PSC_ARR 1u /* PSC/ARR read-back */
#define BSP_TIM_PWM_ST2_CCR     2u /* duty round-trip @ 500 per-mille */
#define BSP_TIM_PWM_ST3_ENABLE  3u /* CCxE / MOE / CEN read-back */

/* ---- private helpers -------------------------------------------------------- */

/* Disarm + reconfigure + start one timer's channel 1 in HAL calls. */
static int pwm_arm_timer(TIM_HandleTypeDef *htim, uint32_t psc)
{
    TIM_OC_InitTypeDef oc;

    /* Idempotent re-init: Stop restores the HAL channel state machine to
     * READY and drops CC1E/MOE/CEN (a second Start on a BUSY channel is a
     * HAL_ERROR). Harmless on the first call (nothing running yet). */
    if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1) != HAL_OK) {
        return -1;
    }

    /* Channel shape: PWM mode 1 (active while CNT < CCR), 50% preload,
     * active high, no fast compare. HAL writes CCMR1 (OC1M/OC1PE) and
     * CCR1 from this struct. Idle states LOW so the pin rests low when
     * MOE is cleared -- same as the register edition (CR2 OISx at 0). */
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 500u; /* preload 50% duty (transfers at first update) */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_1) != HAL_OK) {
        return -1;
    }

    /* 1 kHz frame. PSC is shadowed (latches at the next update event) and
     * HAL offers no EGR.UG stroke, so the first frame after start runs at
     * the stale PSC=0: 1000 ticks at f_tim (~6 us TIM1 / ~12 us TIM4)
     * before the 1 kHz cadence locks in -- accepted, file header. */
    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, BSP_TIM_PWM_ARR);
    __HAL_TIM_SET_COUNTER(htim, 0u); /* deterministic phase */

    /* CC1E (+ TIM1 BDTR MOE), then CEN LAST -- all in one HAL call. */
    if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1) != HAL_OK) {
        return -1;
    }
    return 0;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_tim_pwm_init(void)
{
    /* CubeMX owns clocks/pins/base timebase; verify the handles are live. */
    if ((htim1.Instance == NULL) || (htim4.Instance == NULL)) {
        return -1;
    }
    if (pwm_arm_timer(&htim1, BSP_TIM_PWM_TIM1_PSC) != 0) {
        return -1;
    }
    if (pwm_arm_timer(&htim4, BSP_TIM_PWM_TIM4_PSC) != 0) {
        return -1;
    }

    /* Read-back proof: the timebase must have stuck. PSC has no HAL
     * getter; the Instance reads below are read-only probes. */
    if ((__HAL_TIM_GET_AUTORELOAD(&htim1) != BSP_TIM_PWM_ARR) ||
        (__HAL_TIM_GET_AUTORELOAD(&htim4) != BSP_TIM_PWM_ARR) ||
        (htim1.Instance->PSC != BSP_TIM_PWM_TIM1_PSC) || /* read-only (A3 exemption) */
        (htim4.Instance->PSC != BSP_TIM_PWM_TIM4_PSC)) { /* read-only (A3 exemption) */
        return -1;
    }
    if (((htim1.Instance->CCER & TIM_CCER_CC1E) == 0u) || /* read-only (A3 exemption) */
        ((htim4.Instance->CCER & TIM_CCER_CC1E) == 0u) || /* read-only (A3 exemption) */
        ((htim1.Instance->BDTR & TIM_BDTR_MOE) == 0u)) {  /* read-only (A3 exemption) */
        return -1;
    }
    return 0;
}

int bsp_tim_pwm_set_duty(uint32_t ch, uint32_t per_mille)
{
    /* Reject out-of-range duty without touching the compare: 1001 would
     * silently wrap the rounding math, so fail loudly instead. */
    if (per_mille > 1000u) {
        return -1;
    }
    /* CCR = round((ARR+1) * per_mille / 1000), +500 for round-half-up.
     * Max operand 1000*1000 = 1e6 fits uint32_t comfortably. The write
     * lands in the CCR1 preload register (OC1PE=1 from ConfigChannel):
     * the active compare updates at the next frame boundary -- 1 ms worst
     * case, never mid-frame. */
    {
        const uint32_t ccr = ((BSP_TIM_PWM_ARR + 1u) * per_mille + 500u) /
                             1000u;
        if (ch == BSP_TIM_PWM_CH_TIM1_CH1) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
        } else if (ch == BSP_TIM_PWM_CH_TIM4_CH1) {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
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
    if ((__HAL_TIM_GET_AUTORELOAD(&htim1) != BSP_TIM_PWM_ARR) ||
        (__HAL_TIM_GET_AUTORELOAD(&htim4) != BSP_TIM_PWM_ARR) ||
        (htim1.Instance->PSC != BSP_TIM_PWM_TIM1_PSC) || /* read-only (A3 exemption) */
        (htim4.Instance->PSC != BSP_TIM_PWM_TIM4_PSC)) { /* read-only (A3 exemption) */
        return (int)BSP_TIM_PWM_ST1_PSC_ARR;
    }

    /* Step 2: duty round-trip. set_duty(500) must write CCR=500 on both
     * timers (1 MHz tick, 1000-tick frame): |500 - ARR/2| = |500-499.5|
     * <= 1, the spec tolerance. __HAL_TIM_GET_COMPARE reads the preload
     * register, so no wait for the update event is needed. */
    if (bsp_tim_pwm_set_duty(BSP_TIM_PWM_CH_TIM1_CH1, 500u) != 0) {
        return (int)BSP_TIM_PWM_ST2_CCR;
    }
    if (bsp_tim_pwm_set_duty(BSP_TIM_PWM_CH_TIM4_CH1, 500u) != 0) {
        return (int)BSP_TIM_PWM_ST2_CCR;
    }
    {
        /* ARR/2 = 499.5: accept 499..500 on both channels. */
        const uint32_t ccr1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
        const uint32_t ccr4 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
        if ((ccr1 < 499u) || (ccr1 > 500u) ||
            (ccr4 < 499u) || (ccr4 > 500u)) {
            return (int)BSP_TIM_PWM_ST2_CCR;
        }
    }

    /* Step 3: enable bits -- CCER CC1E on both timers plus TIM1 BDTR MOE
     * and CR1 CEN on both. This is the trio a scope outage always comes
     * back to: CC1E without MOE (TIM1) still outputs nothing. */
    if (((htim1.Instance->CCER & TIM_CCER_CC1E) == 0u) || /* read-only (A3 exemption) */
        ((htim4.Instance->CCER & TIM_CCER_CC1E) == 0u) || /* read-only (A3 exemption) */
        ((htim1.Instance->BDTR & TIM_BDTR_MOE) == 0u) ||  /* read-only (A3 exemption) */
        ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0u) ||    /* read-only (A3 exemption) */
        ((htim4.Instance->CR1 & TIM_CR1_CEN) == 0u)) {    /* read-only (A3 exemption) */
        return (int)BSP_TIM_PWM_ST3_ENABLE;
    }
    return 0; /* all steps passed */
}
