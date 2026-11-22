/**
 * @file bsp_can_loopback.h
 * @brief CAN1 register-level bxCAN lab module (PB8=RX/PB9=TX, 500 kbit/s,
 *        loop-back mode) -- init handshake, one-mailbox TX/RX, self-test.
 *
 * Teaching module (P3b wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / register walkthrough / verification recipe (EP18 bit
 * timing worked example) live in the bsp_can_loopback.c file header.
 *
 * Ownership rule: CAN1 is NOT in the IOC at all -- PB8/PB9 sit there as
 * GPIO_Analog placeholders labeled CAN_RX/CAN_TX (knowledge pitfall #13:
 * adapter-owned peripherals stay out of the .ioc). This module is therefore
 * the FULL owner: RCC APB1ENR CAN1EN gate, the PB8/PB9 AF9 mux, and every
 * CAN register (MCR/MSR/TSR/RF0R/BTR/FMR/filter bank). CubeMX will never
 * generate conflicting code -- but never interleave HAL_CAN_* here either.
 *
 * Board fact: the TJA1050 transceiver sits on PB8(RX)/PB9(TX). This module
 * runs bxCAN in LOOP-BACK mode (LBKM=1): frames sent are received back
 * internally, the CANRX pin is ignored -- single-board self-proof. The
 * dual-board bus exercise over the TJA1050 ( jumper both boards' CANH/CANL
 * + a 120R terminator) stays an optional extension; nothing here changes
 * for it except clearing LBKM and loading the real filter set.
 */
#ifndef BSP_CAN_LOOPBACK_H
#define BSP_CAN_LOOPBACK_H

#include <stdint.h>

/* Frame geometry this module builds: standard (11-bit) ID, data frames
 * only, DLC 0..8. ID 0x123 is the selftest's known frame. */
#define BSP_CAN_LOOPBACK_SELFTEST_ID 0x123u
#define BSP_CAN_LOOPBACK_SELFTEST_DLC 4u

/* Register-level init: APB1 clock gate, PB8/PB9 -> AF9 (both), MCR init
 * mode handshake (INRQ -> INAK -> configure -> leave), 500 kbit/s bit
 * timing (BRP=5, BS1=11, BS2=2 at 42 MHz APB1), filter bank 0 in 32-bit
 * mask mode accepting everything into FIFO 0, loop-back enabled.
 * Returns 0 = PASS, -1 = BTR/filter read-back mismatch, -2 = INAK
 * (enter-init) timeout, -3 = INAK-clear (leave-init) timeout. */
int bsp_can_loopback_init(void);

/* Send one standard-ID data frame from mailbox 0. Copies up to 8 bytes
 * into TDLR/TDHR (little-endian, byte 0 in the LSB lane) and requests
 * transmission. Does NOT wait for completion -- read the verdict via
 * bsp_can_loopback_tx_done() (polls TSR) so senders control their own
 * timeouts. Returns 0 = accepted into the mailbox, -1 = mailbox not empty
 * (TME0=0), -2 = bad arguments (id > 0x7FF or dlc > 8). */
int bsp_can_loopback_send(uint32_t std_id, const uint8_t *data,
                          uint32_t dlc);

/* Poll the mailbox-0 completion chain: 0 = RQCP0 + TXOK0 both set within
 * timeout_ms (frame shipped clean), -1 = SysTick dead (bounded exit),
 * -2 = timeout_ms elapsed with no completion at all, -3 = RQCP0 set but
 * TXOK0 clear (arbitration lost / transmission error). Clears RQCP0 on
 * every completion (rc_w0). */
int bsp_can_loopback_tx_done(uint32_t timeout_ms);

/* Poll FIFO 0 for one received frame within timeout_ms. Outputs: std_id,
 * data (>= 8 bytes of scratch), dlc. Returns 0 = frame delivered and the
 * FIFO output mailbox released (RFOM0), -1 = no frame in time. */
int bsp_can_loopback_recv(uint32_t *std_id, uint8_t *data, uint32_t *dlc,
                          uint32_t timeout_ms);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line. Returns "(frame OK)" on the round-trip proof, "(data
 * mismatch)" on a payload error, "" before the first run. */
const char *bsp_can_loopback_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = init handshake: INRQ->INAK enter, BTR/filter read-back, INAK
 *       cleared on exit (the controller reached normal + loop-back mode)
 *   2 = loop-back round trip: send ID=0x123, 4 data bytes; the frame
 *       reappears in FIFO 0 with matching ID and payload
 *   3 = TSR flag chain: the TX must have raised TXOK0 and RQCP0, and
 *       RQCP0 must clear on its rc_w0 write-1 */
int bsp_can_loopback_selftest(void);

#endif /* BSP_CAN_LOOPBACK_H */
