/**
 * @file bsp_sdio_tf.h
 * @brief SDIO TF (microSD) card IDENTIFICATION (HAL edition: PC8..PC11 =
 *        D0..D3, PC12 = CK, PD2 = CMD, AF12, 4-bit wide bus) -- presence
 *        query, card-register capture, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_sdio_tf). SCOPE is
 * unchanged: the CARD IDENTIFICATION story only -- what CID/CSD say, the
 * SDSC-vs-SDHC (CCS) verdict, capacity math, and the < 400 kHz
 * identification-clock rule. The register edition walked the SD spec
 * sequence by hand (CMD0/CMD8/ACMD41/CMD2/CMD3/CMD9 through the CPSM,
 * R1/R2/R3/R6/R7 response formats); the HAL edition lets
 * MX_SDIO_SD_Init() (main.c: HAL_SD_Init + HAL_SD_ConfigWideBusOperation)
 * run that whole sequence once at boot, and this module INTERROGATES the
 * captured state through HAL_SD_GetCardState / HAL_SD_GetCardInfo /
 * HAL_SD_GetCardCID plus the raw CSD words the HAL stored in hsd.CSD[].
 * A3 rule: no peripheral control-register writes; the only register
 * touches are read-only probes (data words of the handle, marked with a
 * read-only (A3 exemption) comment).
 *
 * Ownership rule (inverted from the register edition): CubeMX owns
 * SDIO's whole register file and the six-pin AF12 mux (SDIO_MspInit).
 * Nothing is programmed here. The raw C_SIZE bit surgery is KEPT on the
 * HAL's captured words -- that parsing lesson survives the migration.
 *
 * Known removability semantics (old behavior, mirrored exactly): a TF
 * card is removable BY DESIGN, so an empty socket is an environment
 * property (SKIP verdict), not a code defect -- see bsp_sdio_tf_selftest.
 */
#ifndef BSP_SDIO_TF_H
#define BSP_SDIO_TF_H

#include <stdint.h>

/* Identification clock (teaching note, kept from the register edition):
 * SDIO_CK = SDIOCLK/(CLKDIV+2) and the SD spec demands < 400 kHz while
 * in identification mode (RM 31.9.2 note; even CLKDIV = 50% duty). The
 * register edition programmed CLKDIV=210 itself (84 MHz SDIOCLK ->
 * 396.2 kHz); the HAL edition delegates that rule to HAL_SD_InitCard
 * (SDIO_INIT_CLK_DIV on the 48 MHz SDIOCLK tree). Nothing programs this
 * constant any more -- it documents the constraint. */
#define BSP_SDIO_TF_IDENT_CLKDIV 210u

/* Card type verdict of the LAST identification run. */
typedef enum {
    BSP_SDIO_TF_NONE = 0,  /* no card answered (or not run yet) */
    BSP_SDIO_TF_SDSC,      /* CCS=0: byte-addressed, CSD v1 */
    BSP_SDIO_TF_SDHC       /* CCS=1: block-addressed, CSD v2 */
} bsp_sdio_tf_type_t;

/* Identification query over the HAL: MX_SDIO_SD_Init identified the
 * card once at boot; this function re-checks presence (CMD13 through
 * HAL_SD_GetCardState), re-reads the captured identification data
 * (HAL_SD_GetCardInfo / HAL_SD_GetCardCID) and re-parses the raw CSD
 * words (hsd.CSD[]). On success the captured fields are readable via
 * the accessors below. Returns 0 = card identified; negative = the
 * step that failed:
 *   -1 = handle/config read-back mismatch (MX_SDIO_SD_Init missing)
 *   -2 = CMD13 answered with an error flag (card wedged)
 *   -3 = NO CARD (CMD13 silent / card disconnected / never identified
 *        -- the SKIP verdict)
 *   -4 = card stuck in identification state
 *   -5 = CID capture failed
 *   -6 = RCA = 0
 *   -7 = capacity fields invalid (BlockNbr/BlockSize = 0)
 *   -8 = card type unknown (neither SDSC nor SDHC/SDXC, or the CCS vs
 *        CSD-structure pair disagrees) */
int bsp_sdio_tf_init(void);

/* Accessors for the fields captured by the last bsp_sdio_tf_init():
 * card type (NONE/SDSC/SDHC from the HAL's CardType / OCR CCS verdict),
 * manufacturer ID byte (CID[127:120]), published RCA, the C_SIZE field
 * re-parsed from the raw CSD words (v2: bits 69:48; v1: bits 73:62)
 * with the derived capacity in MiB (HAL BlockNbr/BlockSize math, v2
 * only -- v1 returns 0, same as the register edition). */
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
 *       verdict (-3), which PASSES with the "(no card)" report: an
 *       empty socket is an environment property (SKIP semantics), not
 *       a code defect. The 24C02/W25Q analog does NOT apply here: a TF
 *       card is removable BY DESIGN.
 *   2 = captured-field sanity: RCA != 0, C_SIZE != 0, CCS agrees with
 *       the CSD structure (SDHC -> CSD v2, SDSC -> CSD v1), and an
 *       SDHC verdict carries a nonzero capacity. */
int bsp_sdio_tf_selftest(void);

#endif /* BSP_SDIO_TF_H */
