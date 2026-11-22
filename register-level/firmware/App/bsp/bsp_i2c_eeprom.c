/**
 ******************************************************************************
 * @file    bsp_i2c_eeprom.c
 * @brief   Soft-I2C bus host + onboard 24C02 EEPROM (EP11 worked example)
 *          -- teaching module with self-test.
 ******************************************************************************
 * @addtogroup BSP_I2C_EEPROM
 * @{
 *
 * == ENGINE (P3g, board finding 2026-11-21) ================================
 * The 24C02 command layer below is unchanged from the P3d module; the WIRE
 * ENGINE behind it changed from register-level F4 hardware I2C1 to the
 * soft open-drain bus (firmware/App/bsp/i2c_soft_bus.c -- a line-for-line
 * port of the Radio project's field-proven soft_i2c.c, the engine that
 * drives the Radio OLED). Why: the hardware engine stalled
 * NON-DETERMINISTICALLY on the real board -- scan/writes succeeded at
 * random, then reads wedged waiting for SB/ADDR that never came,
 * reproduced at 100 kHz AND 50 kHz, drifting across power cycles, not
 * cured by five repair rounds. The same physical bus bit-banged by hand
 * is deterministic: every edge has a software cause, every wait is a
 * bounded loop, and "no device" is just a sampled HIGH in the ACK slot
 * instead of a hung state machine. THIS FILE never touches I2C1's
 * CR1/SR1/DR -- those registers are simply off the bus now (I2C1EN is
 * never armed; the peripheral sleeps at reset).
 *
 * == PIN CONFIG (module truth, PB6=SCL / PB7=SDA) ==========================
 * | Field             | Value (module truth)                 | Why
 * |-------------------|--------------------------------------|-------------------
 * | I2C1 in IOC       | NOT PRESENT                          | knowledge #13:
 * |                   |                                      | adapter-owned
 * |                   |                                      | resources stay
 * |                   |                                      | out of the .ioc
 * | PB6/PB7 in IOC    | GPIO_Analog, labels EE_SCL/EE_SDA    | pin-map
 * |                   |                                      | placeholders
 * |                   |                                      | (anti-collision)
 * | MODER [13:12]     | 01 general output (via engine init)  | master-driven
 * |                   | [15:14] = 01                         | bus
 * | OTYPER bits 6/7   | 1 OPEN-DRAIN                         | I2C mandates
 * |                   |                                      | wired-AND: pull
 * |                   |                                      | LOW, release
 * |                   |                                      | HIGH only
 * | OSPEEDR [13:12]/  | 11 high slew                         | clean edges
 * |   [15:14]         |                                      | through the
 * |                   |                                      | pull-up
 * | PUPDR [13:12]/    | 01 pull-up                           | in-chip
 * |   [15:14]         |                                      | fallback; the
 * |                   |                                      | board also has
 * |                   |                                      | external
 * |                   |                                      | resistors per
 * |                   |                                      | the schematic
 * | Bus speed         | 100 kHz (BSP_I2C_EE_BUS_KHZ; half bit| Radio formula
 * |                   | = SystemCoreClock/(khz*2000) cycles  | khz -> cycles
 * | AFR               | untouched (MODER=GPIO output makes   | AF fields are
 * |                   | the nibbles dead)                    | moot now
 * | RCC               | AHB1ENR GPIOBEN armed HERE, idempotent| engine is
 * |                   |                                      | clock-agnostic
 * == 24C02 PROTOCOL (AT24C02 datasheet + board manual note "chip address
 *   0b1010..., 0x50"): 7-bit device address 0b1010_000 = 0x50 (A2/A1/A0
 *   grounded). Byte operations:
 *     write: S 0xA0(=0x50<<1, W) word_addr data P   (byte write)
 *     read:  S 0xA0 word_addr P  S 0xA1(=0x50<<1|1) data NA P
 *            (random read in TWO clean transactions: set the pointer,
 *            STOP, then a current-address read. The repeated-Start Sr
 *            variant is functionally identical; the two-transaction form
 *            is what this module standardized on during the hardware-
 *            engine days and is kept -- one shape, proven on this board.)
 *   Word address is ONE byte (256 bytes). PAGE WRITE discipline: the
 *   page counter is the LOW 3 bits of the word address and ROLLS OVER
 *   INSIDE the page -- a multi-byte page write crossing a boundary
 *   would wrap and overwrite the page START, not spill into the next
 *   page. A single-byte write can never wrap, which is why this
 *   module exposes byte writes only (multi-byte page writes are the
 *   EP11 exercise: chunk at page boundaries).
 *   tWR -- after every write the 24C02 starts an internal write cycle
 *   (up to 5 ms) during which it NACKs EVERYTHING (this NACK is by
 *   design: ACK polling is the EEPROM-standard way to wait it out).
 *   bsp_i2c_eeprom_write_byte() therefore ends with an ack-poll loop:
 *   probe START+address every ~1 ms until the device ACKs, bounded by
 *   BSP_I2C_EE_POLL_TRIES. Skipping the poll = the classic "read back
 *   right after write returns stale data" EEPROM lesson.
 * == ERROR CODES (the soft engine's fault taxonomy) ========================
 *   -1 = NACK: the transaction reached the wire and the target said "no"
 *        (absent/misaddressed device, or the ack-poll window closed while
 *        the write cycle was still running). On this board the 24C02 is
 *        SOLDERED on -- a NACK is a hardware finding, reported via
 *        last_report; the module never invents a PASS without device
 *        feedback.
 *   -2 = bus fault: a line read LOW at START (short, swapped wires, or a
 *        target wedged mid-byte). The engine refused to start rather than
 *        fight; init's 9-clock recovery is the first-line cure.
 * == HOW TO VERIFY =========================================================
 * menu> i2c_ee
 *   "PASS i2c_ee (0x55/0xAA roundtrip ok)" -- two byte writes to word
 *   addresses 0x10/0x11 came back through random reads; the values are
 *   LEFT in the EEPROM (harmless: free page area, and re-running just
 *   rewrites the same bytes).
 *   Step 4 proof: scope/IDR -- SCL=PB6 and SDA=PB7 both HIGH while idle
 *   (open-drain released, pull-ups win).
 */
#include "bsp_i2c_eeprom.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */

#include "i2c_soft_bus.h"

/* ---- module constants ----------------------------------------------------- */

/* 24C02 7-bit device address -- scan-refined at selftest start (the scan
 * covers 0x50..0x57, the whole 1010+A2A1A0 block; this board straps 0x50).
 * The engine shifts left internally: (addr7 << 1) = 0xA0/0xA1 on the wire. */
#define BSP_I2C_EE_DEV_ADDR 0x50u

/* Line masks on GPIOB (PB6=SCL, PB7=SDA). */
#define BSP_I2C_EE_SCL_PIN 0x0040u /* PB6 */
#define BSP_I2C_EE_SDA_PIN 0x0080u /* PB7 */

/* Soft bus speed: 100 kHz bit rate target -> half bit = 840 CPU cycles
 * at 168 MHz (engine formula). Deterministic by construction -- the old
 * hardware CCR/TRISE constants are gone with that engine. */
#define BSP_I2C_EE_BUS_KHZ 100u

/* Self-test step numbers (the value bsp_i2c_eeprom_selftest returns). */
#define BSP_I2C_EE_ST1_CONFIG 1u /* pin/direction read-back */
#define BSP_I2C_EE_ST2_RW55   2u /* scan + 0x55 @0x10 write+readback */
#define BSP_I2C_EE_ST3_RWAA   3u /* 0xAA @0x11 write+readback */
#define BSP_I2C_EE_ST4_IDLE   4u /* SCL/SDA recessive HIGH on IDR */

/* tWR ack-poll window: the 24C02 internal write cycle is 5 ms typ/max.
 * One probe transaction takes ~0.13 ms at 100 kHz; pacing 1 ms between
 * probes gives a ~13 ms window -- >2x the worst-case tWR. */
#define BSP_I2C_EE_POLL_TRIES 12u

/* ---- module state ----------------------------------------------------------- */

/* The soft bus instance (two pins + derived half-bit period) and the
 * device address the scan refined. Report of the last selftest run; the
 * menu reads it after the run. */
static i2c_soft_bus_t s_bus;
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

/* Classify a failed engine transaction (walkthrough "error codes"): if
 * both lines read HIGH on IDR the bus is healthy and the failure was the
 * target's NACK (-1); a LOW line means the engine refused to start on a
 * held bus (-2). GPIOB->IDR bits 6/7 are the REAL line levels -- the
 * same truth bus_start() judged. */
static int ee_wire_fault(void)
{
    if ((GPIOB->IDR & (BSP_I2C_EE_SCL_PIN | BSP_I2C_EE_SDA_PIN)) ==
        (BSP_I2C_EE_SCL_PIN | BSP_I2C_EE_SDA_PIN)) {
        return -1; /* bus healthy: the device NACKed */
    }
    return -2; /* line held LOW: bus fault */
}

/* Probe: does a device ACK address byte (dev7 << 1 | W)? Returns 0=ACK. */
static int ee_probe(uint8_t dev7)
{
    return i2c_soft_bus_probe(&s_bus, dev7) ? 0 : -1;
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

/* tWR ack-poll (walkthrough): after a byte write the 24C02 is blind for
 * up to 5 ms and NACKs everything; probe START+address until it ACKs.
 * One 1 ms pace between probes (engine's SysTick pacing) spreads the
 * tries over a window wider than the worst-case write cycle. Every probe
 * carries its own STOP, so the line is idle between attempts. Returns
 * 0 = device ready, -1 = NACKed through the whole window. */
static int ee_ack_poll(void)
{
    int tries;

    for (tries = 0; tries < (int)BSP_I2C_EE_POLL_TRIES; tries++) {
        if (i2c_soft_bus_probe(&s_bus, s_dev_addr)) {
            return 0; /* ACK: write cycle done */
        }
        i2c_soft_bus_delay_ms(1u);
    }
    return -1;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_i2c_eeprom_init(void)
{
    /* Port clock arm (caller contract of i2c_soft_bus_init): GPIOBEN =
     * AHB1ENR bit1. Idempotent re-arm; the read-back stalls the core
     * until the clock is live. HAL equivalent:
     * __HAL_RCC_GPIOB_CLK_ENABLE(). */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    /* Engine init: both pins -> GPIO output, open-drain, high slew,
     * pull-up (the Radio electricals, register-form) + half-bit timing
     * from SystemCoreClock and + lines released to idle-HIGH. */
    i2c_soft_bus_init(&s_bus, GPIOB, BSP_I2C_EE_SCL_PIN,
                      GPIOB, BSP_I2C_EE_SDA_PIN, BSP_I2C_EE_BUS_KHZ);

    /* One 9-clock recovery + STOP: the soft analogue of the old engine's
     * CR1 SWRST -- unwedges a bus left holding SDA LOW (a target caught
     * mid-byte by a host reboot). Bounded: 9 x two half periods + STOP. */
    i2c_soft_bus_recover(&s_bus);

    /* Read-back proof: the electricals stuck on both pins (MODER 01 =
     * output, OTYPER 1 = open-drain, PUPDR 01 = pull-up). */
    if (((GPIOB->MODER >> 12) & 0x3u) != 0x1u || /* PB6 output */
        ((GPIOB->MODER >> 14) & 0x3u) != 0x1u || /* PB7 output */
        ((GPIOB->OTYPER >> 6) & 0x1u) != 0x1u || /* PB6 open-drain */
        ((GPIOB->OTYPER >> 7) & 0x1u) != 0x1u || /* PB7 open-drain */
        ((GPIOB->PUPDR >> 12) & 0x3u) != 0x1u || /* PB6 pull-up */
        ((GPIOB->PUPDR >> 14) & 0x3u) != 0x1u) { /* PB7 pull-up */
        return -1;
    }
    return 0;
}

int bsp_i2c_eeprom_write_byte(uint8_t addr, uint8_t data)
{
    uint8_t frame[2];

    if (addr >= BSP_I2C_EEPROM_SIZE) {
        return -1; /* out of the 256-byte array: refuse before the bus */
    }

    /* One clean transaction: START -> (dev<<1)|W -> word address -> data
     * -> STOP. The engine ACK-checks every byte and aborts the payload on
     * the first NACK, but ALWAYS completes the STOP. */
    frame[0] = addr;
    frame[1] = data;
    if (!i2c_soft_bus_write(&s_bus, s_dev_addr, frame, 2u)) {
        return ee_wire_fault(); /* -1 NACK / -2 line held LOW */
    }

    /* tWR ack-poll (walkthrough): the device is blind for up to 5 ms
     * now; probing until it ACKs makes write->read safe to back-to-back. */
    return ee_ack_poll();
}

int bsp_i2c_eeprom_read_byte(uint8_t addr, uint8_t *data)
{
    if ((addr >= BSP_I2C_EEPROM_SIZE) || (data == 0)) {
        return -1;
    }

    /* RANDOM READ, two clean transactions (walkthrough): the pointer-set
     * write carries NO data byte, so it never triggers the 24C02's
     * internal write cycle (no tWR), and the current-address read then
     * fetches exactly one byte -- which the master NACKs (engine's
     * "ACK all but last" rule for len=1) so the target releases SDA for
     * the STOP. */
    if (!i2c_soft_bus_write(&s_bus, s_dev_addr, &addr, 1u)) {
        return ee_wire_fault(); /* transaction A failed */
    }
    if (!i2c_soft_bus_read(&s_bus, s_dev_addr, data, 1u)) {
        return ee_wire_fault(); /* transaction B failed */
    }
    return 0;
}

const char *bsp_i2c_eeprom_last_report(void)
{
    return s_report;
}

int bsp_i2c_eeprom_selftest(void)
{
    /* Step 1: init + config read-back (pin mode, open-drain, pull-up --
     * all of step-1 evidence is register level, no bus traffic involved).
     * The init's recovery pulse also leaves the bus idle. */
    if (bsp_i2c_eeprom_init() != 0) {
        ee_set_report("(cfg readback mismatch)");
        return (int)BSP_I2C_EE_ST1_CONFIG;
    }

    /* Step 2: scan 0x50..0x57 with engine probes (7-bit address block of
     * the 24C02 family), refine the device address, then 0x55 @0x10 --
     * write, then random-read compare. A NACK here (rc -1) means the
     * soldered 24C02 did NOT answer: report it as the hardware finding it
     * is (walkthrough "NACK story"); rc -2 means the bus never started. */
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
     * Stop both open-drain drivers release and the pull-ups win; IDR
     * reads must show bits 6/7 set. A LOW here means the bus is wedged
     * (e.g. a target stuck mid-byte -- the classic I2C lock-up). */
    if ((GPIOB->IDR & (GPIO_IDR_IDR_6 | GPIO_IDR_IDR_7)) !=
        (GPIO_IDR_IDR_6 | GPIO_IDR_IDR_7)) {
        ee_set_report("(bus not idle high)");
        return (int)BSP_I2C_EE_ST4_IDLE;
    }
    return 0; /* all steps passed */
}

/** @} */
