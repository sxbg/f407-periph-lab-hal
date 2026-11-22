/**
 * @file bsp_sdio_tf.h
 * @brief SDIO register-level TF (microSD) card IDENTIFICATION (PC8..PC11 =
 *        D0..D3, PC12 = CK, PD2 = CMD) -- init/identify state machine,
 *        card-register capture, self-test.
 *
 * Teaching module (P3d wave, bsp-module-spec.md template -- see bsp_uart).
 * SCOPE: the CARD IDENTIFICATION PROTOCOL only (CMD0/CMD8/ACMD41/CMD2/
 * CMD3/CMD9, R1/R2/R3/R6/R7 response formats, the <=400 kHz identification
 * clock rule). Data transfer (CMD17/18/24, the 32-bit FIFO, DMA) is
 * deliberately NOT implemented -- identification alone proves the whole
 * command path; the FIFO/DMA data pipeline is the extension exercise
 * (EP24 data-phase episode), noted in the walkthrough.
 *
 * Full CubeMX config table / register walkthrough / verification recipe
 * live in the bsp_sdio_tf.c file header.
 *
 * Ownership rule: SDIO is NOT in the IOC (knowledge #13: adapter-owned
 * peripherals stay out) -- PC8..PC12/PD2 sit there as GPIO_Analog
 * placeholders (labels TF_D0..TF_D3/TF_CK/TF_CMD). This module is the
 * FULL owner of RCC->APB2ENR SDIOEN, the six-pin AF12 config and SDIO's
 * POWER/CLKCR/ARG/CMD/RESPCMD/RESP1..4/STA/ICR/MASK registers. No
 * CubeMX code will ever conflict. NOTE: PC12 doubles as the schematic's
 * ESP8266-EN candidate (pin-budget conflict #1, SDIO wins).
 */
#ifndef BSP_SDIO_TF_H
#define BSP_SDIO_TF_H

#include <stdint.h>

/* Identification clock: SDIO_CK = SDIOCLK/(CLKDIV+2) with SDIOCLK =
 * PLLQ = 84 MHz on this board -> CLKDIV=210 -> 396.2 kHz < 400 kHz
 * (RM 31.9.2 identification-mode note; even CLKDIV = 50% duty). */
#define BSP_SDIO_TF_IDENT_CLKDIV 210u

/* Card type verdict of the LAST identification run. */
typedef enum {
    BSP_SDIO_TF_NONE = 0,  /* no card answered (or not run yet) */
    BSP_SDIO_TF_SDSC,      /* CCS=0: byte-addressed, CSD v1 */
    BSP_SDIO_TF_SDHC       /* CCS=1: block-addressed, CSD v2 */
} bsp_sdio_tf_type_t;

/* Register-level bring-up + IDENTIFICATION: clock gate, six-pin AF12
 * mux, SDIO POWER on, CLKCR CLKDIV=210 (396.2 kHz), then the SD spec
 * sequence CMD0 -> CMD8 -> ACMD41 (HCS/CCS) -> CMD2 (CID) -> CMD3 (RCA)
 * -> CMD9 (CSD). On success the captured fields are readable via the
 * accessors below and SDIO_CK keeps running at the identification rate.
 * Returns 0 = card identified; negative = the step that failed:
 *   -1 = clock-gate/CLKCR read-back mismatch
 *   -2 = CMD0 completion flag never came (CPSM dead)
 *   -3 = NO CARD (CMD8 got no valid R7 AND the ACMD41 loop exhausted
 *        with timeouts/NACK-shaped responses -- the SKIP verdict)
 *   -4 = card answered ACMD41 but never went BUSY (stuck/unpowered card)
 *   -5 = CMD2 (CID, long R2) failed
 *   -6 = CMD3 (R6) failed or returned RCA=0
 *   -7 = CMD9 (CSD, long R2) failed
 *   -8 = CSD structure version invalid (neither 0=v1 nor 1=v2) */
int bsp_sdio_tf_init(void);

/* Accessors for the fields captured by the last bsp_sdio_tf_init():
 * card type (NONE/SDSC/SDHC from OCR CCS bit30), manufacturer ID byte
 * (CID[127:120]), published RCA (CMD3 RESP1[31:16]) and the CSD C_SIZE
 * field (v2: bits 69:48; v1: bits 73:62) with the derived capacity in
 * MiB ((C_SIZE+1) x 512 KiB, v2 only -- v1 returns 0). */
bsp_sdio_tf_type_t bsp_sdio_tf_type(void);
uint8_t bsp_sdio_tf_manufacturer_id(void);
uint16_t bsp_sdio_tf_rca(void);
uint32_t bsp_sdio_tf_c_size(void);
uint32_t bsp_sdio_tf_capacity_mb(void);

/* One-line report of the LAST identification run, for the menu to append
 * to its verdict line. Examples: "(no card)" / "(SDHC mfr=0x03 3854MB)". */
const char *bsp_sdio_tf_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed (bsp_sdio_tf_last_report carries the "why"). Steps:
 *   1 = identification: bsp_sdio_tf_init() == 0 -- EXCEPT the no-card
 *       verdict (-3), which PASSES with the "(no card)" report: an empty
 *       socket is an environment property (SKIP semantics), not a code
 *       defect. The 24C02/W25Q analog does NOT apply here: a TF card is
 *       removable BY DESIGN.
 *   2 = captured-field sanity: RCA != 0, C_SIZE != 0, and CCS agrees
 *       with the CSD version byte (SDHC -> CSD v2, SDSC -> CSD v1). */
int bsp_sdio_tf_selftest(void);

#endif /* BSP_SDIO_TF_H */
