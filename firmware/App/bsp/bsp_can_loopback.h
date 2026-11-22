/**
 * @file bsp_can_loopback.h
 * @brief CAN1 500 kbit/s loop-back lab module (HAL edition of the
 *        register-level bsp_can_loopback) -- PB8=RX/PB9=TX, one-mailbox
 *        TX/RX round trip, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_can_loopback; the
 * full register walkthrough -- bxCAN mode machine, EP18 bit-timing
 * derivation, filter-bank anatomy -- stays in the old project's file
 * header as the companion reference).
 *
 * OWNERSHIP CHANGE vs the register edition: CAN1 IS in the .ioc now.
 * CubeMX owns the kernel clock gate (CAN1EN via MspInit), the PB8/PB9
 * AF9 mux, and the bit timing: prescaler 7, BS1 8 tq, BS2 3 tq at
 * APB1 = 42 MHz -> tq = 166.7 ns, 12 tq/bit = 500 kbit/s, sample point
 * (1+8)/12 = 75% (replaces the register edition's BRP=5/BS1=11/BS2=2
 * shape; same bit rate, earlier sampling). This module owns only what
 * the .ioc cannot express: LOOP-BACK mode and the accept-all filter.
 *
 * Loop-back switch (A3 rule -- no MCR/BTR register writes): the
 * sanctioned HAL pattern mutates the handle and re-runs the driver --
 *   hcan1.Init.Mode = CAN_MODE_LOOPBACK;  HAL_CAN_Init(&hcan1);
 * then HAL_CAN_ConfigFilter + HAL_CAN_Start. The INRQ/INAK handshake
 * of the register edition now lives inside those three HAL calls.
 *
 * HAL API mapping note: this HAL driver vintage exposes
 * HAL_CAN_AddTxMessage() -- the same function the newer HAL names
 * HAL_CAN_AddMessageToTxFifoQ(); it picks the lowest free mailbox and
 * reports it through its out parameter.
 */

#ifndef BSP_CAN_LOOPBACK_H
#define BSP_CAN_LOOPBACK_H

#include <stdint.h>

/* Frame geometry this module builds: standard (11-bit) ID, data frames
 * only, DLC 0..8. ID 0x123 is the selftest's known frame. */
#define BSP_CAN_LOOPBACK_SELFTEST_ID 0x123u
#define BSP_CAN_LOOPBACK_SELFTEST_DLC 4u

/* Init: switch hcan1 to loop-back, arm the accept-all filter bank 0
 * (-> FIFO 0), start the controller, verify the BTR read-back, drain
 * any stale frames so re-runs stay deterministic.
 * Returns 0 = PASS,
 *        -1 = config proof failed (BTR read-back mismatch or filter
 *             configuration rejected),
 *        -2 = HAL_CAN_Init failed (the old INRQ->INAK enter-init
 *             timeout case),
 *        -3 = HAL_CAN_Start failed (the old leave-init INAK-clear
 *             timeout case). */
int bsp_can_loopback_init(void);

/* Send one standard-ID data frame. Copies up to 8 bytes into the HAL
 * TX path and requests transmission. Does NOT wait for completion --
 * read the verdict via bsp_can_loopback_tx_done(). Serialize calls:
 * the mailbox selected by this send is the one tx_done completes.
 * Returns 0 = accepted, -1 = no free mailbox / controller not started
 * (the old TME0=0 case), -2 = bad arguments (id > 0x7FF or dlc > 8). */
int bsp_can_loopback_send(uint32_t std_id, const uint8_t *data,
                          uint32_t dlc);

/* Wait for the last send's mailbox to complete within timeout_ms
 * (HAL_GetTick bounded). Returns 0 = completed and TXOK set (frame
 * shipped clean), -1 = SysTick dead (bounded exit, never hang),
 * -2 = timeout_ms elapsed with the mailbox still pending,
 * -3 = completed but TXOK clear (arbitration lost / TERR). The TXOK
 * probe is a read-only TSR access (A3 read-only exemption). */
int bsp_can_loopback_tx_done(uint32_t timeout_ms);

/* Poll FIFO 0 for one received frame within timeout_ms. Outputs:
 * std_id, data (>= 8 bytes of scratch, the HAL copies all 8 payload
 * lanes), dlc. Returns 0 = frame delivered, -1 = no frame in time or
 * bad arguments. */
int bsp_can_loopback_recv(uint32_t *std_id, uint8_t *data, uint32_t *dlc,
                          uint32_t timeout_ms);

/* One-line report of the LAST selftest run, for the menu to append to
 * its verdict line. "(frame OK)" on the round-trip proof, "(data
 * mismatch)" on a payload error, "" before the first run. */
const char *bsp_can_loopback_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed. Steps:
 *   1 = init handshake: loop-back re-init + filter + start, BTR
 *       read-back carries the generated timing + LBKM
 *   2 = loop-back round trip: send ID=0x123, 4 data bytes; the frame
 *       reappears in FIFO 0 with matching ID and payload
 *   3 = completion flags on a second frame: RQCP0 and TXOK0 both
 *       observed set in TSR (read-only). INTENTIONAL CHANGE vs the
 *       register edition: the write-1-to-clear (rc_w0) proof of RQCP0
 *       is dropped -- A3 forbids writing TSR here; the rc_w0 mechanics
 *       live on in the register edition's walkthrough. */
int bsp_can_loopback_selftest(void);

#endif /* BSP_CAN_LOOPBACK_H */
