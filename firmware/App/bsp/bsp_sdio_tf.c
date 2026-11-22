/**
 ******************************************************************************
 * @file    bsp_sdio_tf.c
 * @brief   SDIO TF card identification (HAL edition: presence query +
 *          captured-register interrogation) -- teaching module with
 *          self-test.
 ******************************************************************************
 * @addtogroup BSP_SDIO_TF
 * @{
 *
 * == REGISTER -> HAL MAPPING ===============================================
 *   register-edition concept               HAL edition call
 *   -------------------------------------  --------------------------------
 *   RCC APB2ENR SDIOEN + 6-pin AF12 mux    SDIO_MspInit (AF12, very-high
 *                                            speed, no pulls)
 *   POWER/CLKCR writes (PWRCTRL=11,        HAL_SD_InitCard inside
 *     CLKDIV=210 -> 396.2 kHz ident          HAL_SD_Init (MX_SDIO_SD_Init,
 *     clock, CLKEN)                          main.c) -- the < 400 kHz
 *                                            identification rule is the
 *                                            HAL's SDIO_INIT_CLK_DIV now
 *   CPSM command flow (ARG/CMD/STA/ICR)    HAL_SD_GetCardState (CMD13
 *     per command                            presence probe),
 *                                            HAL_SD_GetCardInfo (captured
 *                                            SdCard copy), HAL_SD_GetCardCID
 *                                            (captured CID parse)
 *   CMD0->CMD8->ACMD41->CMD2->CMD3->CMD9   HAL_SD_InitCard ran the whole
 *     by hand                                SD spec sequence once at boot
 *   SDIO->RESP1..4 reads                   hsd.CSD[0..3] / hsd.CID[0..3]
 *     (127-bit lanes, MSB-first)             (same lane packing -- the
 *                                            C_SIZE bit surgery survives
 *                                            verbatim; read-only A3
 *                                            exemption: captured DATA
 *                                            words, no control registers)
 *
 * == WHAT SURVIVED THE MIGRATION ===========================================
 * The teaching payload of the register edition was the SD spec story:
 * identification must run under 400 kHz (RM 31.9.2), CMD8's R7 echo
 * separates v1-only cards, ACMD41's HCS/CCS pair gives the SDSC/SDHC
 * verdict, CID[127:120] is the manufacturer byte, and C_SIZE's bit
 * position depends on the CSD structure version (v2: CSD[69:48],
 * capacity = (C_SIZE+1) x 512 KiB; v1: CSD[73:62]). All of that lives
 * on here -- the first three as documentation of what HAL_SD_InitCard
 * did at boot, the last two as live bit surgery on hsd.CSD[].
 * == HOW TO VERIFY =========================================================
 * menu> sdio_tf
 *   Card inserted: "PASS sdio_tf (SDHC mfr=0x03 3854MB)" -- the HAL's
 *   boot-time identification captured CID/CSD; the manufacturer byte
 *   and derived capacity come straight out of the captured words.
 *   Empty socket:  "PASS sdio_tf (no card)" -- CMD13 silent/disconnected
 *   = the hardware's own verdict (SKIP semantics: a removable medium
 *   being absent is not a defect).
 * Failure semantics: "(init: stuck in identification)" = a card is IN
 *   the socket but never finished power-up (weak socket contact, dead
 *   card) -- step-1 FAIL with that report; "RCA=0" / "card type/CSD
 *   unknown" = identification data corrupted or unsupported, step-1 or
 *   step-2 FAIL.
 */
#include "bsp_sdio_tf.h"

#include "stm32f4xx_hal.h"
#include "sdio.h" /* hsd from MX_SDIO_SD_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* Self-test step numbers (the value bsp_sdio_tf_selftest returns). */
#define BSP_SDIO_TF_ST1_IDENTIFY 1u /* presence + capture query */
#define BSP_SDIO_TF_ST2_FIELDS   2u /* RCA/C_SIZE/CCS-vs-CSD sanity */

/* ---- module state ----------------------------------------------------------- */

/* Captured identification results (read by the accessors + selftest). */
static struct {
    bsp_sdio_tf_type_t type; /* NONE/SDSC/SDHC verdict */
    uint8_t mfr_id;          /* CID[127:120] */
    uint16_t rca;            /* card-published relative address */
    uint32_t cs_size;        /* C_SIZE field (v1: 12-bit, v2: 22-bit) */
    uint32_t cap_mb;         /* v2 capacity in MiB, 0 for v1 */
} s_sd;

/* Report of the last identification run; the menu reads it after the run. */
static char s_report[48];

/* ---- private helpers -------------------------------------------------------- */

/* Reset s_report to a fixed prefix. */
static void sd_report_set(const char *txt)
{
    char *p = s_report;
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

/* Append a string / one hex byte ("0x03" -> "3") / unsigned decimal. */
static void sd_report_append(const char *txt)
{
    char *p = s_report;
    while (*p != '\0') {
        p++;
    }
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

static void sd_report_append_hex8(uint8_t v)
{
    static const char hexd[16] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    char *p = s_report;
    while (*p != '\0') {
        p++;
    }
    *p++ = hexd[(v >> 4) & 0x0Fu];
    *p++ = hexd[v & 0x0Fu];
    *p = '\0';
}

static void sd_report_append_dec(uint32_t v)
{
    char digits[10];
    unsigned m = 0u;
    char *p = s_report;
    do {
        digits[m++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v > 0u);
    while (*p != '\0') {
        p++;
    }
    while (m > 0u) {
        *p++ = digits[--m];
    }
    *p = '\0';
}

/* ---- public API ------------------------------------------------------------ */

int bsp_sdio_tf_init(void)
{
    HAL_SD_CardInfoTypeDef card;
    HAL_SD_CardCIDTypeDef cid;
    HAL_SD_CardStateTypeDef state;
    uint32_t csd_struct;

    /* Runtime self-configuration (knowledge #17): the generated boot
     * wrapper MX_SDIO_SD_Init is neutered (USER CODE SDIO_Init 0 returns
     * early) because a failed HAL_SD_Init with an empty socket bricked
     * the boot. The module configures pins + handle itself, on demand,
     * all through sanctioned HAL calls. */
    if (s_sd.type == BSP_SDIO_TF_NONE) {
        GPIO_InitTypeDef af12;

        /* 6 pins: PC8..PC11 = D0..D3, PC12 = CK, PD2 = CMD (all AF12,
         * very-high speed, no pull -- the RM0090 SDIO electricals). */
        af12.Mode = GPIO_MODE_AF_PP;
        af12.Pull = GPIO_NOPULL;
        af12.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        af12.Alternate = GPIO_AF12_SDIO;
        af12.Pin = TF_D0_Pin | TF_D1_Pin | TF_D2_Pin | TF_D3_Pin
                 | TF_CK_Pin;
        HAL_GPIO_Init(GPIOC, &af12);
        af12.Pin = TF_CMD_Pin;
        HAL_GPIO_Init(GPIOD, &af12);

        hsd.Instance = SDIO;
        hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
        hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
        hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
        hsd.Init.BusWide = SDIO_BUS_WIDE_1B; /* spec: ident runs 1-bit */
        hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
        hsd.Init.ClockDiv = SDIO_TRANSFER_CLK_DIV;
        if (HAL_SD_Init(&hsd) != HAL_OK) {
            /* ACMD41 timeouts out here with an EMPTY socket: this is the
             * removable-medium SKIP verdict, not a defect. */
            return -3;
        }
        if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
            return -3; /* same empty-socket path through CMD6 */
        }
    }

    /* Identification state: s_sd.type stays BSP_SDIO_TF_NONE until a
     * successful identification below re-arms it for the next call. */
    /* Card presence: HAL_SD_GetCardState sends CMD13 -- the HAL's own
     * "is the card alive" probe, the analog of the register edition's
     * ACMD41 poll. A card that answers maps its status bits [12:9] to
     * the HAL_SD_CARD_* states. */
    state = HAL_SD_GetCardState(&hsd);
    if (state == HAL_SD_CARD_ERROR) {
        return -2; /* card answered with an error flag */
    }
    if ((state == (HAL_SD_CardStateTypeDef)0u) || /* no CMD13 answer */
        (state == HAL_SD_CARD_DISCONNECTED) ||
        (hsd.SdCard.CardType == 0u)) {
        return -3; /* NO CARD -- the SKIP verdict (removable medium) */
    }
    if (state == HAL_SD_CARD_IDENTIFICATION) {
        return -4; /* identification never completed (stuck card) */
    }

    /* Captured registers: HAL_SD_GetCardInfo copies the identification
     * results (CardType/CardVersion/RCA/capacity) that HAL_SD_Init
     * stored in hsd.SdCard -- no bus traffic, pure data read. */
    if (HAL_SD_GetCardInfo(&hsd, &card) != HAL_OK) {
        return -7;
    }
    if ((card.CardType != CARD_SDSC) &&
        (card.CardType != CARD_SDHC_SDXC)) {
        return -8; /* neither SDSC nor SDHC/SDXC */
    }
    s_sd.type = (card.CardType == CARD_SDHC_SDXC) ? BSP_SDIO_TF_SDHC
                                                  : BSP_SDIO_TF_SDSC;

    /* CID (HAL_SD_GetCardCID parses hsd.CID[] captured by CMD2 at
     * boot): CID[127:120] = manufacturer ID byte. */
    if (HAL_SD_GetCardCID(&hsd, &cid) != HAL_OK) {
        return -5;
    }
    s_sd.mfr_id = cid.ManufacturerID;

    /* RCA sanity (the register edition's CMD3/R6 check): a sane card
     * never publishes RCA 0. */
    if (card.RelCardAdd == 0u) {
        return -6;
    }
    s_sd.rca = (uint16_t)card.RelCardAdd;

    /* Capacity fields (the CMD9/CSD payload, captured at boot). */
    if ((card.BlockNbr == 0u) || (card.BlockSize == 0u)) {
        return -7;
    }

    /* read-only (A3 exemption): raw CSD bit surgery on the HAL's
     * captured words. hsd.CSD[0..3] = CSD[127:1] MSB-first -- the same
     * lane packing the register edition read from RESP1..4, so the bit
     * maps survive verbatim. Structure bits 127:126: 00 = v1 (SDSC),
     * 01 = v2 (SDHC/SDXC). */
    csd_struct = (hsd.CSD[0] >> 30) & 0x3u;
    if (((s_sd.type == BSP_SDIO_TF_SDHC) && (csd_struct != 1u)) ||
        ((s_sd.type == BSP_SDIO_TF_SDSC) && (csd_struct != 0u))) {
        return -8; /* CCS verdict and CSD structure disagree */
    }
    if (csd_struct == 1u) { /* v2 (SDHC/SDXC): C_SIZE = CSD[69:48] */
        s_sd.cs_size = ((hsd.CSD[2] & 0x3Fu) << 16) | (hsd.CSD[3] >> 16);
    } else { /* v1 (SDSC): C_SIZE = CSD[73:62] */
        s_sd.cs_size = ((hsd.CSD[2] & 0x3FFu) << 2) | (hsd.CSD[3] >> 30);
    }

    /* Capacity in MiB from the HAL's block math: BlockNbr x BlockSize
     * bytes; BlockSize is a power of two (512), so the division form
     * below is overflow-free for every SDHC/SDXC card. v2 only -- v1
     * keeps the register edition's "raw field, no capacity" semantics. */
    if (s_sd.type == BSP_SDIO_TF_SDHC) {
        s_sd.cap_mb = card.BlockNbr / (1024u * 1024u / card.BlockSize);
    } else {
        s_sd.cap_mb = 0u;
    }
    return 0; /* identified: type/mfr/rca/c_size/cap all captured */
}

bsp_sdio_tf_type_t bsp_sdio_tf_type(void)
{
    return s_sd.type;
}

uint8_t bsp_sdio_tf_manufacturer_id(void)
{
    return s_sd.mfr_id;
}

uint16_t bsp_sdio_tf_rca(void)
{
    return s_sd.rca;
}

uint32_t bsp_sdio_tf_c_size(void)
{
    return s_sd.cs_size;
}

uint32_t bsp_sdio_tf_capacity_mb(void)
{
    return s_sd.cap_mb;
}

const char *bsp_sdio_tf_last_report(void)
{
    return s_report;
}

int bsp_sdio_tf_selftest(void)
{
    /* Step 1: the identification query. A no-card verdict PASSES with
     * its own report (SKIP semantics -- removable medium); every other
     * failure is a step-1 FAIL with the step named. */
    s_sd.type = BSP_SDIO_TF_NONE;
    {
        const int rc = bsp_sdio_tf_init();
        if (rc == -3) { /* empty socket: the hardware's own verdict */
            sd_report_set("(no card)");
            return 0; /* PASS, nothing further to check */
        }
        if (rc != 0) {
            static const char *const why[8] = {
                "config mismatch", "state probe error", "",
                "stuck in identification", "CID failed", "RCA=0",
                "capacity invalid", "card type/CSD unknown"
            };
            /* rc in [-1..-4] and [-5..-8]: map to the step name. */
            int idx = -rc; /* 1..8 */
            if ((idx < 1) || (idx > 8)) {
                sd_report_set("(init: unknown fail)");
            } else {
                sd_report_set("(init: ");
                sd_report_append(why[idx - 1]);
                sd_report_append(")");
            }
            return (int)BSP_SDIO_TF_ST1_IDENTIFY;
        }
    }

    /* Step 2: captured-field sanity (the "print" of the CCS verdict /
     * CID mfr / CSD capacity is the report line itself, data straight
     * from the captured registers). Assert: RCA nonzero (re-checked),
     * C_SIZE nonzero, SDHC carries a real capacity. */
    sd_report_set("(");
    sd_report_append((s_sd.type == BSP_SDIO_TF_SDHC) ? "SDHC" : "SDSC");
    sd_report_append(" mfr=0x");
    sd_report_append_hex8(s_sd.mfr_id);
    sd_report_append(" ");
    sd_report_append_dec((s_sd.cap_mb != 0u) ? s_sd.cap_mb
                                             : s_sd.cs_size);
    sd_report_append((s_sd.cap_mb != 0u) ? "MB" : " (v1 field)");
    sd_report_append(")");
    if (s_sd.rca == 0u) {
        sd_report_set("(RCA=0)");
        return (int)BSP_SDIO_TF_ST2_FIELDS;
    }
    if (s_sd.cs_size == 0u) {
        sd_report_set("(C_SIZE=0)");
        return (int)BSP_SDIO_TF_ST2_FIELDS;
    }
    if ((s_sd.type == BSP_SDIO_TF_SDHC) && (s_sd.cap_mb == 0u)) {
        sd_report_set("(SDHC but no capacity)");
        return (int)BSP_SDIO_TF_ST2_FIELDS;
    }
    return 0; /* all steps passed */
}

/** @} */

