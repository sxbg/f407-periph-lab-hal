/**
 ******************************************************************************
 * @file    bsp_i2c_eeprom.c
 * @brief   Hardware I2C1 host (HAL edition) + onboard 24C02 EEPROM --
 *          teaching module with self-test.
 ******************************************************************************
 * @addtogroup BSP_I2C_EEPROM
 * @{
 *
 * == ENGINE (HAL edition) ==================================================
 * The 24C02 command layer is unchanged from the register edition; the
 * WIRE ENGINE behind it changed from the soft open-drain bus (and,
 * before that, from register-level F4 hardware I2C1) to the hardware
 * I2C1 driven through the HAL. Register-edition concept -> HAL edition
 * call, one row each:
 *
 *   register-edition concept            HAL edition call
 *   ----------------------------------  -----------------------------------
 *   RCC->AHB1ENR GPIOBEN + PB6/PB7      MX_I2C1_Init + I2C1_MspInit
 *     MODER/OTYPER/PUPDR (soft bus        (PB6/PB7 AF4, open-drain,
 *     electricals, written by engine)     pull-up)
 *   half-bit timing from SystemCore     HAL_I2C_Init TIMINGR/CCR math
 *     Clock (khz -> cycles formula)       (ClockSpeed=100k, DutyCycle=2)
 *   9-clock bus recovery + STOP         none here -- HAL error paths
 *                                       reset the peripheral; re-running
 *                                       HAL_I2C_Init is the recovery
 *                                       (left as the extension exercise)
 *   i2c_soft_bus_write (ptr+data)       HAL_I2C_Mem_Write (1 byte)
 *   write then read (2 transactions)    HAL_I2C_Mem_Read (Sr inside)
 *   i2c_soft_bus_probe (ACK slot)       HAL_I2C_IsDeviceReady
 *   tWR ack-poll loop                   same loop, IsDeviceReady probes
 *   GPIOB->IDR idle-level read          HAL_GPIO_ReadPin on PB6/PB7
 *                                       (kept, read-only A3 exemption)
 *
 * == 24C02 PROTOCOL (AT24C02 datasheet + board manual note "chip address
 *   0b1010..., 0x50"): 7-bit device address 0b1010_000 = 0x50 (A2/A1/A0
 *   grounded). The HAL takes the 8-bit address (dev7 << 1 = 0xA0/0xA1)
 *   on every call. Byte operations:
 *     write: HAL_I2C_Mem_Write(dev, word_addr, 8BIT, data, 1) -- the
 *            S 0xA0 word_addr data P frame in one call.
 *     read:  HAL_I2C_Mem_Read(dev, word_addr, 8BIT, buf, 1) -- the HAL
 *            sets the pointer, then (repeated-Start) reads the byte;
 *            the register edition's two-transaction shape is now the
 *            HAL's internal sequence.
 *   Word address is ONE byte (256 bytes). PAGE WRITE discipline: the
 *   page counter is the LOW 3 bits of the word address and ROLLS OVER
 *   INSIDE the page -- a multi-byte page write crossing a boundary
 *   would wrap and overwrite the page START, not spill into the next
 *   page. A single-byte write can never wrap; this module keeps
 *   exposing byte writes only (the multi-byte page write via
 *   HAL_I2C_Mem_Write, chunked at page boundaries, is the EP11
 *   exercise). BSP_I2C_EEPROM_PAGE documents the boundary.
 *   tWR -- after every write the 24C02 starts an internal write cycle
 *   (up to 5 ms) during which it NACKs EVERYTHING (by design; ACK
 *   polling is the EEPROM-standard way to wait it out). The poll loop
 *   probes HAL_I2C_IsDeviceReady every 1 ms, bounded by
 *   BSP_I2C_EE_POLL_TRIES. Skipping the poll = the classic "read back
 *   right after write returns stale data" EEPROM lesson.
 * == ERROR CODES ===========================================================
 *   -1 = NACK: the transaction reached the wire and the target said
 *        "no" (HAL_I2C_ERROR_AF, or both lines still recessive-HIGH on
 *        the IDR read-back). On this board the 24C02 is SOLDERED on --
 *        a NACK is a hardware finding, reported via last_report; the
 *        module never invents a PASS without device feedback.
 *   -2 = bus fault: HAL timed out and/or a line reads LOW (short,
 *        swapped wires, or a target wedged mid-byte). The HAL refuses
 *        to wait forever; HAL_GetTick bounds every wait.
 * == KNOWN BOUNDARY (carried over, old-project section 3.2) ================
 * The register edition moved OFF hardware I2C1 because random reads
 * stalled non-deterministically on the real board (2026-11-21 finding,
 * reproduced at 100 kHz and 50 kHz, not cured by five repair rounds).
 * The HAL edition deliberately runs the hardware engine through HAL
 * calls -- that is the teaching main line for this project. If the
 * same stall shows up during W6 board bring-up, record it in
 * knowledge/ as a known board boundary with the transcript; do NOT
 * patch this module to switch engines.
 * == HOW TO VERIFY =========================================================
 * menu> i2c_ee
 *   "PASS i2c_ee (0x55/0xAA roundtrip ok)" -- two byte writes to word
 *   addresses 0x10/0x11 came back through random reads; the values are
 *   LEFT in the EEPROM (harmless: free page area, and re-running just
 *   rewrites the same bytes).
 *   Step 4 proof: scope/HAL_GPIO_ReadPin -- SCL=PB6 and SDA=PB7 both
 *   HIGH while idle (open-drain released, pull-ups win).
 */
#include "bsp_i2c_eeprom.h"

#include "main.h" /* EE_SCL/EE_SDA pin labels from CubeMX */
#include "stm32f4xx_hal.h"
#include "i2c.h" /* hi2c1 from MX_I2C1_Init (CubeMX-generated) */

/* ---- module constants ----------------------------------------------------- */

/* 24C02 7-bit device address -- scan-refined at selftest start (the scan
 * covers 0x50..0x57, the whole 1010+A2A1A0 block; this board straps 0x50).
 * The HAL wants the 8-bit form (dev7 << 1) = 0xA0/0xA1 on the wire. */
#define BSP_I2C_EE_DEV_ADDR 0x50u

/* Per-transaction HAL timeout: one 2-byte exchange at 100 kHz is
 * ~0.2 ms; the HAL polls flags with tick granularity -- 20 ms is
 * ~100x headroom and still fails fast on a wedged bus. */
#define BSP_I2C_EE_CMD_TO_MS 20u

/* Self-test step numbers (the value bsp_i2c_eeprom_selftest returns). */
#define BSP_I2C_EE_ST1_CONFIG 1u /* config read-back (HAL-owned cfg) */
#define BSP_I2C_EE_ST2_RW55   2u /* scan + 0x55 @0x10 write+readback */
#define BSP_I2C_EE_ST3_RWAA   3u /* 0xAA @0x11 write+readback */
#define BSP_I2C_EE_ST4_IDLE   4u /* SCL/SDA recessive HIGH */

/* tWR ack-poll window: the 24C02 internal write cycle is 5 ms typ/max.
 * Probes paced 1 ms apart give a ~12 ms window -- >2x the worst-case
 * tWR. */
#define BSP_I2C_EE_POLL_TRIES 12u

/* ---- module state ----------------------------------------------------------- */

/* The device address the scan refined, and the report of the last
 * selftest run (the menu reads it after the run). */
static uint8_t s_dev_addr = BSP_I2C_EE_DEV_ADDR; /* scan-refined */
static char s_report[32];

/* ---- private helpers -------------------------------------------------------- */

/* Copy a fixed string into s_report (tiny, no libc). */
static void ee_set_report(const char *txt)
{
    char *p = s_report;
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

/* The 8-bit (Receive/Transmit-form) address every HAL I2C call wants. */
static uint16_t ee_dev_addr8(void)
{
    return (uint16_t)(s_dev_addr << 1);
}

/* Classify a failed HAL transaction: if both lines read recessive-HIGH
 * the bus is healthy and the failure was the target's NACK (-1); a LOW
 * line means the bus is stuck (-2). Same classification as the soft
 * engine used -- only the judge changed. */
static int ee_wire_fault(void)
{
    /* read-only (A3 exemption): the pin levels are the REAL line
     * truth; HAL_GPIO_ReadPin just wraps the IDR read. */
    if ((HAL_GPIO_ReadPin(EE_SCL_GPIO_Port, EE_SCL_Pin) == GPIO_PIN_SET) &&
        (HAL_GPIO_ReadPin(EE_SDA_GPIO_Port, EE_SDA_Pin) == GPIO_PIN_SET)) {
        return -1; /* bus healthy: the device NACKed */
    }
    return -2; /* line held LOW: bus fault */
}

/* Probe: does a device ACK address byte (dev7 << 1 | W)? HAL_I2C_
 * IsDeviceReady sends the address and judges the ACK slot (the soft
 * engine's probe, HAL-shaped). Returns 0=ACK. */
static int ee_probe(uint8_t dev7)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(dev7 << 1), 1u,
                              BSP_I2C_EE_CMD_TO_MS) == HAL_OK) {
        return 0;
    }
    return -1;
}

/* Scan 0x50..0x57, return the first ACKed 7-bit address, else 0. */
static uint8_t ee_scan(void)
{
    uint8_t a;
    for (a = 0x50u; a <= 0x57u; a++) {
        if (ee_probe(a) == 0) {
            return a;
        }
    }
    return 0u;
}

/* tWR ack-poll: after a byte write the 24C02 is blind for up to 5 ms
 * and NACKs everything; probe until it ACKs. One HAL_Delay(1) pace
 * between probes spreads the tries over a window wider than the
 * worst-case write cycle. Returns 0 = device ready, -1 = NACKed
 * through the whole window. */
static int ee_ack_poll(void)
{
    int tries;

    for (tries = 0; tries < (int)BSP_I2C_EE_POLL_TRIES; tries++) {
        if (ee_probe(s_dev_addr) == 0) {
            return 0; /* ACK: write cycle done */
        }
        HAL_Delay(1u);
    }
    return -1;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_i2c_eeprom_init(void)
{
    /* Config read-back only (read-only, A3 exemption): CubeMX owns the
     * pins (I2C1_MspInit: PB6/PB7 AF4, open-drain, pull-up) and I2C1's
     * CR1/TIMINGR/CCR (HAL_I2C_Init from ClockSpeed/DutyCycle). The
     * register edition wrote and re-read MODER/OTYPER/PUPDR by hand --
     * those are now the generated code's guarantees, verified here,
     * never written. */
    if (hi2c1.Instance == NULL) {
        return -1; /* MX_I2C1_Init never ran */
    }
    if (hi2c1.Init.ClockSpeed != (uint32_t)(BSP_I2C_EE_BUS_KHZ * 1000u)) {
        return -1; /* not the 100 kHz standard-mode config we document */
    }
    /* HAL_I2C_Init enables the peripheral last (CR1 PE) -- a clear PE
     * means the generated init never completed. */
    if ((hi2c1.Instance->CR1 & I2C_CR1_PE) == 0u) {
        return -1;
    }
    /* Bus electricals (read-only, A3 exemption): both lines recessive
     * HIGH = open-drain released, pull-ups win. A LOW here at idle is
     * a wedged bus (init's recovery pulse was the register edition's
     * first-line cure; the HAL edition reports it instead). */
    HAL_Delay(1); /* let the last STOP settle: pull-up recharge takes a
                     fraction of a microsecond, but the HAL release path
                     (state machine unwind) spans a few ticks */
    if ((HAL_GPIO_ReadPin(EE_SCL_GPIO_Port, EE_SCL_Pin) != GPIO_PIN_SET) ||
        (HAL_GPIO_ReadPin(EE_SDA_GPIO_Port, EE_SDA_Pin) != GPIO_PIN_SET)) {
        return -1;
    }
    return 0;
}

int bsp_i2c_eeprom_write_byte(uint8_t addr, uint8_t data)
{
    if (addr >= BSP_I2C_EEPROM_SIZE) {
        return -1; /* out of the 256-byte array: refuse before the bus */
    }

    /* One HAL call = one transaction: START -> (dev<<1)|W -> word
     * address -> data -> STOP (the frame the register edition built by
     * hand, byte-checked at every ACK slot). */
    if (HAL_I2C_Mem_Write(&hi2c1, ee_dev_addr8(), (uint16_t)addr,
                          I2C_MEMADD_SIZE_8BIT, &data, 1u,
                          BSP_I2C_EE_CMD_TO_MS) != HAL_OK) {
        return ee_wire_fault(); /* -1 NACK / -2 line held LOW */
    }

    /* tWR ack-poll: the device is blind for up to 5 ms now; probing
     * until it ACKs makes write->read safe to back-to-back. */
    return ee_ack_poll();
}

int bsp_i2c_eeprom_read_byte(uint8_t addr, uint8_t *data)
{
    if ((addr >= BSP_I2C_EEPROM_SIZE) || (data == 0)) {
        return -1;
    }

    /* RANDOM READ (walkthrough): HAL_I2C_Mem_Read issues the pointer-
     * set write, then the read with its internal repeated-Start, in
     * ONE call. The register edition used two clean transactions to
     * dodge the hardware engine's Sr stall (the known boundary) -- HAL
     * owns the Sr now; if it stalls on W6 bring-up, that IS the
     * boundary reproducing, to be recorded, not patched around. */
    if (HAL_I2C_Mem_Read(&hi2c1, ee_dev_addr8(), (uint16_t)addr,
                         I2C_MEMADD_SIZE_8BIT, data, 1u,
                         BSP_I2C_EE_CMD_TO_MS) != HAL_OK) {
        return ee_wire_fault(); /* transaction failed */
    }
    return 0;
}

const char *bsp_i2c_eeprom_last_report(void)
{
    return s_report;
}

int bsp_i2c_eeprom_selftest(void)
{
    /* Step 1: init + config read-back (ClockSpeed/PE -- all of step-1
     * evidence is read-only probes, no bus traffic involved). */
    if (bsp_i2c_eeprom_init() != 0) {
        ee_set_report("(cfg readback mismatch)");
        return (int)BSP_I2C_EE_ST1_CONFIG;
    }

    /* Step 2: scan 0x50..0x57 with HAL_I2C_IsDeviceReady probes (7-bit
     * address block of the 24C02 family), refine the device address,
     * then 0x55 @0x10 -- write, then random-read compare. A NACK here
     * means the soldered 24C02 did NOT answer: report it as the
     * hardware finding it is (walkthrough "NACK story"). */
    {
        uint8_t rd = 0u;
        uint8_t found = ee_scan();
        if (found == 0u) {
            ee_set_report("(scan: no ack 0x50-0x57)");
            return (int)BSP_I2C_EE_ST2_RW55;
        }
        s_dev_addr = found;
        if (bsp_i2c_eeprom_write_byte(0x10u, 0x55u) != 0) {
            ee_set_report("(w 0x55@0x10 failed/NACK)");
            return (int)BSP_I2C_EE_ST2_RW55;
        }
        {
            const int rc = bsp_i2c_eeprom_read_byte(0x10u, &rd);
            if (rc != 0) {
                ee_set_report((rc == -1) ? "(r @0x10 NACK)"
                              : (rc == -2) ? "(r @0x10 stall)"
                                           : "(r @0x10 mismatch)");
                return (int)BSP_I2C_EE_ST2_RW55;
            }
        }
        if (rd != 0x55u) {
            ee_set_report("(0x10 readback mismatch)");
            return (int)BSP_I2C_EE_ST2_RW55;
        }
    }

    /* Step 3: 0xAA @0x11 -- second cell, proves the address pointer is
     * not stuck (a broken word-address phase would keep answering from
     * 0x10 and step 3 would catch the 0x55). */
    {
        uint8_t rd = 0u;
        if (bsp_i2c_eeprom_write_byte(0x11u, 0xAAu) != 0) {
            ee_set_report("(w 0xAA@0x11 failed/NACK)");
            return (int)BSP_I2C_EE_ST3_RWAA;
        }
        {
            const int rc = bsp_i2c_eeprom_read_byte(0x11u, &rd);
            if (rc != 0) {
                ee_set_report((rc == -1) ? "(r @0x11 NACK)"
                              : (rc == -2) ? "(r @0x11 stall)"
                                           : "(r @0x11 mismatch)");
                return (int)BSP_I2C_EE_ST3_RWAA;
            }
        }
        if (rd != 0xAAu) {
            ee_set_report("(0x11 readback mismatch)");
            return (int)BSP_I2C_EE_ST3_RWAA;
        }
    }
    ee_set_report("(0x55/0xAA roundtrip ok)");

    /* Step 4: bus idle = both lines HIGH (recessive). After the last
     * STOP both open-drain drivers release and the pull-ups win; a LOW
     * here means the bus is wedged (e.g. a target stuck mid-byte -- the
     * classic I2C lock-up). */
    HAL_Delay(1); /* let the last STOP settle: pull-up recharge takes a
                     fraction of a microsecond, but the HAL release path
                     (state machine unwind) spans a few ticks */
    if ((HAL_GPIO_ReadPin(EE_SCL_GPIO_Port, EE_SCL_Pin) != GPIO_PIN_SET) ||
        (HAL_GPIO_ReadPin(EE_SDA_GPIO_Port, EE_SDA_Pin) != GPIO_PIN_SET)) {
        ee_set_report("(bus not idle high)");
        return (int)BSP_I2C_EE_ST4_IDLE;
    }
    return 0; /* all steps passed */
}

/** @} */
