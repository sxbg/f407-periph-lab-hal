/**
 * @file bsp_tim_adv.c
 * @brief TIM8 complementary PWM + dead-time generator -- teaching module
 *        (HAL edition of f407-periph-lab bsp_tim_adv; EP14 worked example).
 ******************************************************************************
 * == WHAT CUBEMX ALREADY DID (MX_TIM8_Init + MspInit/MspPostInit) ==========
 * | Item          | Value (IOC truth)                  | Why
 * |---------------|------------------------------------|-------------------
 * | TIM8 mode     | PWM Generation CH1 + CH2 with CH2N | pins routed by
 * |               | (base init only: PSC=0, ARR=65535) | MspInit/MspPostInit
 * | Pins          | PC6=TIM8_CH1, PC7=TIM8_CH2,        | AF3, no pull,
 * |               | PB14=TIM8_CH2N                     | low speed
 * | Kernel clock  | APB2 x2 = 168 MHz                  | RM0090 ch.6
 * | CKD           | ClockDivision=DIV1 -> t_DTS=f_tim  | dead-time tick
 * | Break         | not configured (BKIN floating)     | see BDTR note below
 * == HAL MAPPING (register concept -> HAL call) ============================
 *   CCMR1 OC1M/OC2M=110 + OCxPE=1 ..... HAL_TIM_PWM_ConfigChannel per
 *       channel (TIM_OCMODE_PWM1; the preloaded compare is what makes the
 *       complementary pair glitch-free).
 *   CCER CC1E|CC2E|CC2NE .............. HAL_TIM_PWM_Start (x2) +
 *       HAL_TIMEx_PWMN_Start (the complementary enable).
 *   BDTR DTG=0x94 (1 us dead time) .... HAL_TIMEx_ConfigBreakDeadTime
 *       (DeadTime=0x94). Break stage deliberately NOT configured -- this
 *       lab board has no break source wired to PB12, and arming BKIN with
 *       nothing driving it would only add a way for MOE to be cleared
 *       asynchronously (Break/Break2 DISABLE, AOE off).
 *   BDTR MOE .......................... set inside HAL_TIM_PWM_Start /
 *       HAL_TIMEx_PWMN_Start via __HAL_TIM_MOE_ENABLE. MOE is THE
 *       advanced-timer trap (same bit TIM1 needed in bsp_tim_pwm):
 *       without it every CCER enable is dead and PC6/PC7/PB14 stay flat.
 *   PSC/ARR ........................... __HAL_TIM_SET_PRESCALER /
 *       __HAL_TIM_SET_AUTORELOAD (RCR=0 from IOC base init, untouched).
 *       Same PSC-shadow caveat as bsp_tim_pwm: the first frame after
 *       start runs at stale PSC=0 (~6 us) before the 1 kHz cadence locks.
 *   DTG knob (set_deadtime_ns) ........ rebuild the BDTR config via
 *       HAL_TIMEx_ConfigBreakDeadTime with only DeadTime changed -- that
 *       call rewrites the whole BDTR and (AutomaticOutput=DISABLE) drops
 *       MOE, so the function re-arms it with __HAL_TIM_MOE_ENABLE to keep
 *       a live PWM running through the knob change (the register edition
 *       kept MOE in the same read-modify-write).
 *   DTG decode (get_deadtime_ns) ...... read-only BDTR probe (A3
 *       exemption) -- no HAL getter for the dead-time field.
 *   Idempotent re-init ................ Stop first for all three outputs
 *       (HAL channel state machines back to READY; Start on BUSY is a
 *       HAL_ERROR); the PWMN_Stop also drops MOE.
 * Frame + dead-time derivation (unchanged, the EP14 exercise):
 *   TIM8: 168 MHz / (167+1) = 1 MHz tick; 1 MHz / (999+1) = 1 kHz frame.
 *   CKD=00 -> f_DTS = 168 MHz -> t_DTS = 5.952 ns.
 *   Target 1 us = 1000 ns -> 1000/5.952 = 168.0 ticks > 127 (range 1
 *   ceiling) -> range 2: DTG[5:0] = ticks/2 - 64 = 20 = 0x14, DTG byte =
 *   10_010100b = 0x94. Realized DT = (64+20)*2*5.952 = 999.9 ns
 *   (quantization step 2*t_DTS = 11.9 ns, error -0.07 ns).
 * == HOW TO VERIFY =========================================================
 * menu> tim_adv
 *   PASS tim_adv (steps: 1 DTG/CCER/MOE, 2 ns round-trip, 3 polarities)
 * Scope checks: PC6 = 1 kHz 50% PWM. PC7/PB14 = the complementary pair:
 *   both 1 kHz, 50%, active-high, and PB14 is the exact inverse of PC7
 *   EXCEPT the 1 us window around each transition where BOTH are low (the
 *   dead time -- zoom in: no shoot-through overlap is possible by
 *   construction). Dead-time knob: set_deadtime_ns(5000) -> 5048 ns
 *   realized (range-4 step is 95 ns: 5000 ns is an off-grid target).
 * Bench cross-check: if PB14 doubles as a board LED, the dead-time gaps
 *   are invisible; a scope or the logic-analyzer menu on PC7+PB14 is the
 *   honest meter.
 */
#include "bsp_tim_adv.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "tim.h" /* htim8 from MX_TIM8_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* 1 kHz frame derivation (see file header): 168 MHz/(167+1) = 1 MHz tick,
 * (999+1) ticks = 1 kHz. Checked by read-back in selftest. */
#define BSP_TIM_ADV_PSC 167u
#define BSP_TIM_ADV_ARR 999u

/* Self-test step numbers (the value bsp_tim_adv_selftest returns). */
#define BSP_TIM_ADV_ST1_CONFIG   1u /* DTG / CCER / MOE read-back */
#define BSP_TIM_ADV_ST2_DT_ROUND 2u /* ns round-trip within one step */
#define BSP_TIM_ADV_ST3_POL      3u /* CC2P / CC2NP polarity read-back */

/* Dead-time bounds accepted by bsp_tim_adv_set_deadtime_ns: 1 tick
 * (~6 ns) up to the range-4 ceiling (1008 ticks = 6048 ns). */
#define BSP_TIM_ADV_DT_MAX_TICKS 1008u
#define BSP_TIM_ADV_DT_MAX_NS \
    ((BSP_TIM_ADV_DT_MAX_TICKS * BSP_TIM_ADV_TDTS_NS_NUM) / \
     BSP_TIM_ADV_TDTS_NS_DEN) /* = 6048 ns */

/* ---- private helpers -------------------------------------------------------- */

/* Re-write BDTR through HAL with the given DTG byte, keeping the
 * deliberately-unconfigured break stage off, then re-arm MOE (the config
 * call clears it when AutomaticOutput=DISABLE). */
static int adv_apply_deadtime(uint32_t dtg)
{
    /* Full zero init: the BreakFilter field (and any Break2 field in newer
     * HAL generations) must read as disabled/unfiltered, never as stack
     * garbage that HAL ORs straight into BDTR. */
    TIM_BreakDeadTimeConfigTypeDef bdt = {0};

    bdt.OffStateRunMode = TIM_OSSR_DISABLE;
    bdt.OffStateIDLEMode = TIM_OSSI_DISABLE;
    bdt.LockLevel = TIM_LOCKLEVEL_OFF;
    bdt.DeadTime = dtg & 0xFFu;
    bdt.BreakState = TIM_BREAK_DISABLE;
    bdt.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &bdt) != HAL_OK) {
        return -1;
    }
    __HAL_TIM_MOE_ENABLE(&htim8); /* keep a live PWM through the knob change */
    return 0;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_tim_adv_init(void)
{
    TIM_OC_InitTypeDef oc;

    /* CubeMX owns clocks/pins/base timebase; verify the handle is live. */
    if (htim8.Instance == NULL) {
        return -1;
    }

    /* Idempotent re-init: disarm all three outputs and restore the HAL
     * channel state machines to READY (Start on a BUSY channel is a
     * HAL_ERROR). Harmless on the first call. */
    if (HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1) != HAL_OK) {
        return -1;
    }
    if (HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }
    if (HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }

    /* Both channels: PWM mode 1 (active while CNT < CCR), 50% preload,
     * active high. Idle states LOW so the pins rest low while MOE is
     * cleared -- same as the register edition (CR2 OISx at 0). HAL writes
     * CCMR1 (OCxM/OCxPE) and CCR1/CCR2 from this struct. */
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 500u; /* 50% duty on both (500 of 1000 ticks) */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_1) != HAL_OK) {
        return -1;
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }

    /* Dead time: 1 us via the EP14 derivation -> DTG = 0x94 (file
     * header). Break stage deliberately NOT configured (header note). */
    if (adv_apply_deadtime(0x94u) != 0) {
        return -1;
    }

    /* 1 kHz frame (PSC shadow caveat: first frame ~6 us at stale PSC=0). */
    __HAL_TIM_SET_PRESCALER(&htim8, BSP_TIM_ADV_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim8, BSP_TIM_ADV_ARR);
    __HAL_TIM_SET_COUNTER(&htim8, 0u);

    /* Start all three outputs; the advanced-timer starts also set BDTR
     * MOE (without it CC1E/CC2E/CC2NE are dead -- the classic trap). */
    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) {
        return -1;
    }
    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }
    if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }

    /* Read-back proof: frame, enables, main output, DTG byte. CCER/BDTR/
     * PSC have no HAL getters; those Instance reads are read-only probes. */
    if ((__HAL_TIM_GET_AUTORELOAD(&htim8) != BSP_TIM_ADV_ARR) ||
        (htim8.Instance->PSC != BSP_TIM_ADV_PSC)) { /* read-only (A3 exemption) */
        return -1;
    }
    if ((htim8.Instance->CCER &                             /* read-only (A3 exemption) */
         (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE)) !=
        (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE)) {
        return -1;
    }
    if (((htim8.Instance->BDTR & TIM_BDTR_MOE) == 0u) ||    /* read-only (A3 exemption) */
        ((htim8.Instance->BDTR & TIM_BDTR_DTG_Msk) != 0x94u)) {
        return -1;
    }
    return 0;
}

int bsp_tim_adv_set_deadtime_ns(uint32_t ns)
{
    /* Convert ns -> t_DTS ticks with round-half-up: ticks = ns / 5.952.
     * The 10000/1680 fraction is exact for 168 MHz (file header). */
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

    /* Apply through HAL (keeps MOE armed inside the helper). */
    return adv_apply_deadtime(dtg);
}

uint32_t bsp_tim_adv_get_deadtime_ns(void)
{
    const uint32_t dtg =
        htim8.Instance->BDTR & TIM_BDTR_DTG_Msk; /* read-only (A3 exemption) */
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
    if (((htim8.Instance->BDTR & TIM_BDTR_MOE) == 0u) ||   /* read-only (A3 exemption) */
        ((htim8.Instance->BDTR & TIM_BDTR_DTG_Msk) != 0x94u) ||
        ((htim8.Instance->CCER &                            /* read-only (A3 exemption) */
          (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE)) !=
         (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2NE)) ||
        ((htim8.Instance->CR1 & TIM_CR1_CEN) == 0u)) {      /* read-only (A3 exemption) */
        return (int)BSP_TIM_ADV_ST1_CONFIG;
    }

    /* Step 2: dead-time round-trip through the register: 1000 ns in ->
     * DTG=0x94 in BDTR -> decode back must land within one quantization
     * step (11.9 ns; range-2 realized value is 999.9 ns, error -0.07 ns).
     * A second knob value (200 ns, range 1, DTG=34=0x22, realized
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
    if ((htim8.Instance->CCER &                             /* read-only (A3 exemption) */
         (TIM_CCER_CC2P | TIM_CCER_CC2NP)) != 0u) {
        return (int)BSP_TIM_ADV_ST3_POL;
    }
    return 0; /* all steps passed */
}
