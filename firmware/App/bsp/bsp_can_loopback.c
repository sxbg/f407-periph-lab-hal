/**
 ******************************************************************************
 * @file    bsp_can_loopback.c
 * @brief   CAN1 500 kbit/s loop-back BSP (HAL edition) -- see
 *          bsp_can_loopback.h for the contract and the HAL mapping.
 ******************************************************************************
 * @addtogroup BSP_CAN_LOOPBACK
 * @{
 *
 * == HAL EDITION MAP (register edition -> here) ===========================
 * RCC->APB1ENR CAN1EN + PB8/PB9 AF9 mux     -> MspInit (Core/Src/can.c),
 *                                              runs once at MX_CAN1_Init.
 * MCR INRQ->INAK enter, BTR write, INAK     -> HAL_CAN_Init after the mode
 *   clear (leave init)                        field change; the driver does
 *                                              the whole handshake with an
 *                                              internal bounded wait.
 * FMR FINIT + FM1R/FS1R/FFA1R/FA1R/FR1/FR2  -> HAL_CAN_ConfigFilter with a
 *                                              CAN_FilterTypeDef (bank 0,
 *                                              32-bit mask, id=mask=0 ->
 *                                              accept all, FIFO 0).
 * TIR/TDTR/TDLR/TDHR mailbox writes         -> HAL_CAN_AddTxMessage.
 * TSR RQCP0/TXOK0/TME0 flag chain           -> HAL_CAN_IsTxMessagePending
 *                                              (TME) + read-only TSR probes
 *                                              (RQCP/TXOK, A3 exemption).
 * RF0R FMP0 + RIR/RDTR/RDLR/RDHR + RFOM0    -> HAL_CAN_GetRxFifoFillLevel
 *                                              + HAL_CAN_GetRxMessage.
 *
 * == CUBEMX CONFIG (in f407-periph-lab-hal.ioc now) ========================
 * | Field       | Value                        | Note
 * |-------------|------------------------------|---------------------------
 * | CAN1        | prescaler 7, BS1 8tq,        | 500 kbit/s @ APB1 42 MHz,
 * |             | BS2 3tq, SJW 1tq, NORMAL     | sample point 75%
 * | PB8/PB9     | AF9 CAN1_RX/CAN1_TX, no pull | MspInit via HAL_GPIO_Init
 * | Mode        | NORMAL in .ioc; LOOPBACK     | this module re-inits with
 * |             | applied at init              | the handle-field pattern
 * | Filter      | not expressible in the .ioc  | configured HERE (accept all
 * |             |                              | -> FIFO 0)
 * | NVIC        | CAN1_RX0_IRQn off            | polling lesson
 * == HOW TO VERIFY =========================================================
 * menu> can_lb
 *   "PASS can_lb (frame OK)" -- the controller shipped ID=0x123 and
 *   received its own bytes back through the internal Tx->Rx loop.
 * Scope check (optional): PB9 (CANTX) shows the real frame waveform even
 *   in loop-back mode ("The transmitted messages can be monitored on the
 *   CANTX pin", RM0090 32.5.2). Dual-board bus exercise: stays an
 *   extension -- skip the LOOPBACK re-init and load real filters.
 */
#include "bsp_can_loopback.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "can.h" /* hcan1 from MX_CAN1_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* Timeout windows (ms, HAL_GetTick based). INAK needs a few bit times; a
 * 500 kbit/s 4-byte frame takes ~110 us; 10 ms is generous for all three. */
#define BSP_CAN_TO_MS 10u

/* Spin cap between HAL_GetTick() reads: SysTick-dead guard (spec #5). */
#define BSP_CAN_SYSTICK_GUARD 2000000u

/* Self-test step numbers (the value bsp_can_loopback_selftest returns). */
#define BSP_CAN_ST1_HANDSHAKE 1u /* loop-back re-init + filter + start */
#define BSP_CAN_ST2_FRAME     2u /* loop-back TX -> RX round trip      */
#define BSP_CAN_ST3_FLAGS     3u /* RQCP/TXOK completion flags          */

/* ---- module state ----------------------------------------------------------- */

/* Report of the last selftest run; the menu reads it after the run. */
static char s_report[24];

/* Mailbox INDEX (0..2) chosen by the most recent send; tx_done and the
 * step-3 flag probe watch exactly that mailbox. HAL returns a bitmask
 * (CAN_TX_MAILBOX0 = bit0 set); this stores the bit position. */
static uint32_t s_tx_mbox;

/* ---- private helpers -------------------------------------------------------- */

/* Copy a fixed string into s_report (tiny, no libc). */
static void can_set_report(const char *txt)
{
    char *p = s_report;
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

/* Wait conditions for can_wait(): 1 = satisfied, 0 = keep waiting.
 * can_tx_idle watches the mailbox the last send picked (1u << index is
 * the HAL bitmask: CAN_TX_MAILBOX0/1/2). */
static int can_tx_idle(void)
{
    return (HAL_CAN_IsTxMessagePending(&hcan1, (1u << s_tx_mbox)) == 0u) ? 1 : 0;
}

static int can_rx_pending(void)
{
    return (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0u) ? 1 : 0;
}

/* Spin until cond() is satisfied or timeout_ms elapses (HAL_GetTick
 * based). Returns 0 = satisfied, -2 = window exhausted, -1 = SysTick
 * dead (HAL_GetTick frozen -- bounded exit, never hang). */
static int can_wait(int (*cond)(void), uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    uint32_t last_tick = start;
    uint32_t guard = BSP_CAN_SYSTICK_GUARD;
    for (;;) {
        uint32_t now;
        if (cond() != 0) {
            return 0;
        }
        now = HAL_GetTick();
        if ((now - start) >= timeout_ms) {
            return -2; /* window exhausted */
        }
        if (now != last_tick) {
            last_tick = now;
            guard = BSP_CAN_SYSTICK_GUARD; /* tick alive: refill the cap */
        } else {
            guard--;
            if (guard == 0u) {
                return -1; /* SysTick dead: bounded exit */
            }
        }
    }
}

/* Mailbox index from the HAL bitmask (1<<idx). */
static uint32_t can_mbox_index(uint32_t mbox_mask)
{
    uint32_t idx = 0u;
    while (((1u << idx) != mbox_mask) && (idx < 3u)) {
        idx++;
    }
    return idx;
}

/* Read-only (A3 exemption): RQCP + TXOK pair of the given mailbox index,
 * shifted out of TSR. RQCP is bit0 of each mailbox nibble, TXOK bit1. */
static uint32_t can_mbox_status(uint32_t idx)
{
    return (CAN1->TSR >> (4u * idx)) & (CAN_TSR_RQCP0 | CAN_TSR_TXOK0);
}

/* Drain FIFO 0: stale frames from a previous menu run would be pulled by
 * the next recv and masquerade as the fresh round trip. */
static void can_drain_fifo0(void)
{
    CAN_RxHeaderTypeDef rxh;
    uint8_t buf[8];
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0u) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxh, buf) != HAL_OK) {
            return; /* driver refused: stop poking */
        }
    }
}

/* ---- public API ------------------------------------------------------------ */

int bsp_can_loopback_init(void)
{
    CAN_FilterTypeDef filt;
    uint32_t btr_expect;

    if (hcan1.Instance == NULL) {
        return -2; /* CubeMX init never ran */
    }

    /* Loop-back switch: mutate the handle's Init field, re-run the
     * driver. HAL_CAN_Init performs the INRQ->INAK handshake internally
     * (the old enter-init timeout surfaces here as -2). */
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        return -2;
    }

    /* Accept-all filter bank 0 -> FIFO 0. Mask 0 = every bit "don't
     * care"; id = 0 for tidiness. SlaveStartFilterBank = the CubeMX
     * default split (CAN2 unused in this lab). */
    filt.FilterBank = 0u;
    filt.FilterMode = CAN_FILTERMODE_IDMASK;
    filt.FilterScale = CAN_FILTERSCALE_32BIT;
    filt.FilterIdHigh = 0u;
    filt.FilterIdLow = 0u;
    filt.FilterMaskIdHigh = 0u;
    filt.FilterMaskIdLow = 0u;
    filt.FilterFIFOAssignment = CAN_RX_FIFO0;
    filt.FilterActivation = CAN_FILTER_ENABLE;
    filt.SlaveStartFilterBank = 14u;
    if (HAL_CAN_ConfigFilter(&hcan1, &filt) != HAL_OK) {
        return -1; /* filter never armed */
    }

    /* Leave init mode: the driver waits for INAK to clear (the old
     * leave-init timeout surfaces here as -3). */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return -3;
    }

    /* Read-back proof (read-only, A3 exemption): BTR must now carry the
     * generated timing shape composed with the loop-back bit -- i.e. the
     * re-init really reached the hardware. SLAK is the driver's job. */
    btr_expect = (hcan1.Init.Prescaler - 1u) |
                 hcan1.Init.TimeSeg1 |
                 hcan1.Init.TimeSeg2 |
                 hcan1.Init.SyncJumpWidth |
                 CAN_BTR_LBKM;
    if (CAN1->BTR != btr_expect) {
        (void)HAL_CAN_Stop(&hcan1); /* park: do not leave a half-proven state */
        return -1;
    }

    can_drain_fifo0(); /* re-runs stay deterministic */
    return 0;
}

int bsp_can_loopback_send(uint32_t std_id, const uint8_t *data,
                          uint32_t dlc)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t mbox_mask = 0u;

    if ((std_id > 0x7FFu) || (dlc > 8u) || (data == 0)) {
        return -2; /* bad arguments: nothing touched */
    }
    hdr.StdId = std_id;
    hdr.ExtId = 0u;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = (uint8_t)dlc;
    hdr.TransmitGlobalTime = DISABLE;
    /* HAL_CAN_AddTxMessage = this vintage's HAL_CAN_AddMessageToTxFifoQ:
     * it scans TSR for a free mailbox, copies the payload and raises
     * TXRQ. No free mailbox / not started -> HAL_ERROR (the old TME0=0
     * case). */
    if (HAL_CAN_AddTxMessage(&hcan1, &hdr, data, &mbox_mask) != HAL_OK) {
        return -1;
    }
    s_tx_mbox = can_mbox_index(mbox_mask);
    return 0;
}

int bsp_can_loopback_tx_done(uint32_t timeout_ms)
{
    int rc;

    /* TME (transmit-mailbox empty) going true = the request completed,
     * success or failure alike. Bounded by the HAL tick window. */
    rc = can_wait(can_tx_idle, timeout_ms);
    if (rc != 0) {
        return (rc == -1) ? -1 : -2; /* SysTick dead vs window exhausted */
    }
    /* Read-only (A3 exemption): TXOK0 distinguishes the clean ship from
     * the ALST0/TERR0 failure twins. */
    if ((can_mbox_status(s_tx_mbox) & CAN_TSR_TXOK0) == 0u) {
        return -3; /* completed but NOT ok: arbitration lost / TERR */
    }
    return 0;
}

int bsp_can_loopback_recv(uint32_t *std_id, uint8_t *data, uint32_t *dlc,
                          uint32_t timeout_ms)
{
    CAN_RxHeaderTypeDef rxh;
    uint8_t buf[8];
    uint32_t i;

    if ((std_id == 0) || (data == 0) || (dlc == 0)) {
        return -1;
    }
    /* FMP0 analog: fill level of FIFO 0. 0 pending after the window =
     * nothing came back (in loop-back mode that means the TX itself
     * failed -- check tx_done first). */
    if (can_wait(can_rx_pending, timeout_ms) != 0) {
        return -1;
    }
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxh, buf) != HAL_OK) {
        return -1;
    }
    *std_id = rxh.StdId;
    *dlc = rxh.DLC;
    for (i = 0u; i < 8u; i++) {
        data[i] = buf[i]; /* HAL copies all 8 payload lanes (little-endian
                           * DRx lane order absorbed by the driver) */
    }
    return 0;
}

const char *bsp_can_loopback_last_report(void)
{
    return s_report;
}

int bsp_can_loopback_selftest(void)
{
    static const uint8_t payload[BSP_CAN_LOOPBACK_SELFTEST_DLC] = {
        0xA1u, 0xB2u, 0xC3u, 0xD4u
    };
    uint8_t rx[8];
    uint32_t id = 0u;
    uint32_t dlc = 0u;
    uint32_t i;

    /* Step 1: the init handshake, now inside HAL_CAN_Init/ConfigFilter/
     * Start, plus the BTR read-back proof (generated 500 kbit/s shape +
     * LBKM). A -2/-3 here means the controller never reached/left init
     * mode -- check the APB1 clock and pins (MspInit) first. */
    if (bsp_can_loopback_init() != 0) {
        return (int)BSP_CAN_ST1_HANDSHAKE;
    }

    /* Step 2: loop-back round trip. Send ID=0x123 with 4 known bytes,
     * wait for the mailbox to confirm (TXOK), then pull the frame out
     * of FIFO 0 and compare every field. */
    if (bsp_can_loopback_send(BSP_CAN_LOOPBACK_SELFTEST_ID, payload,
                              BSP_CAN_LOOPBACK_SELFTEST_DLC) != 0) {
        return (int)BSP_CAN_ST2_FRAME;
    }
    if (bsp_can_loopback_tx_done(BSP_CAN_TO_MS) != 0) {
        return (int)BSP_CAN_ST2_FRAME;
    }
    if (bsp_can_loopback_recv(&id, rx, &dlc, BSP_CAN_TO_MS) != 0) {
        return (int)BSP_CAN_ST2_FRAME; /* nothing came back: LBKM off? */
    }
    if ((id != BSP_CAN_LOOPBACK_SELFTEST_ID) ||
        (dlc != BSP_CAN_LOOPBACK_SELFTEST_DLC)) {
        can_set_report("(data mismatch)");
        return (int)BSP_CAN_ST2_FRAME;
    }
    for (i = 0u; i < dlc; i++) {
        if (rx[i] != payload[i]) {
            can_set_report("(data mismatch)");
            return (int)BSP_CAN_ST2_FRAME;
        }
    }
    can_set_report("(frame OK)");

    /* Step 3: completion flags on a second frame. The register edition
     * first cleared RQCP0/TXOK0 (write-1-to-clear proof of the rc_w0
     * mechanics) -- INTENTIONALLY DROPPED under the A3 rule (TSR stays
     * read-only here). What remains provable: the second send completes
     * and BOTH flags sit set in TSR for its mailbox. */
    if (bsp_can_loopback_send(BSP_CAN_LOOPBACK_SELFTEST_ID, payload,
                              BSP_CAN_LOOPBACK_SELFTEST_DLC) != 0) {
        return (int)BSP_CAN_ST3_FLAGS;
    }
    if (bsp_can_loopback_tx_done(BSP_CAN_TO_MS) != 0) {
        return (int)BSP_CAN_ST3_FLAGS;
    }
    if (can_mbox_status(s_tx_mbox) != (CAN_TSR_RQCP0 | CAN_TSR_TXOK0)) {
        return (int)BSP_CAN_ST3_FLAGS; /* completion chain incomplete */
    }
    /* Drain the second frame from FIFO 0 so the lab stays tidy for a
     * re-run (ignore its content -- step 2 already proved it). */
    (void)bsp_can_loopback_recv(&id, rx, &dlc, BSP_CAN_TO_MS);
    return 0; /* all steps passed */
}

/** @} */
