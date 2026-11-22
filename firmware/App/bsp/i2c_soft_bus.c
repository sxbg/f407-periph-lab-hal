/**
 ******************************************************************************
 * @file    i2c_soft_bus.c
 * @brief   Generic bit-banged open-drain I2C master -- Radio-proven soft
 *          engine, register-form port (bsp/i2c_soft_bus.h for the contract).
 ******************************************************************************
 * @addtogroup I2C_SOFT_BUS
 * @{
 *
 * == WHY THIS ENGINE EXISTS (board finding 2026-11-21) =====================
 * The F4 hardware I2C1 host (the previous bsp_i2c_eeprom engine) stalled
 * NON-DETERMINISTICALLY on the real board: scan/writes succeeded at random,
 * then reads wedged waiting for SB/ADDR that never came -- reproduced at
 * 100 kHz AND 50 kHz, drifting across power cycles, not cured by five
 * repair rounds. The Radio project (same MCU family, same physics) talks to
 * ALL its I2C devices over a soft, bit-banged open-drain bus and is field
 * proven (its OLED display IS this engine). Decision: keep the EEPROM
 * command layer, swap the protocol engine to this port. Nothing here is
 * invented -- the edge order, delays and ACK sampling below are Radio
 * soft_i2c.c line by line.
 *
 * == PIN ELECTRICALS (Radio's open-drain writing, register form) ===========
 * i2c_soft_bus_init() reconfigures both pins to GPIO output, OPEN-DRAIN,
 * HIGH SLEW, PULL-UP -- the register-form equivalent of Radio's
 *   init.Mode = GPIO_MODE_OUTPUT_OD; init.Pull = GPIO_PULLUP;
 *   init.Speed = GPIO_SPEED_FREQ_HIGH;  HAL_GPIO_Init(...)
 *   MODER [2p+1:2p] = 01  general output (the bus is master-driven)
 *   OTYPER bit p    = 1   open-drain (RM 10.3: a 1 written through BSRR
 *                         RELEASES the line -- the pull-up returns it HIGH;
 *                         a 0 drives it LOW; nothing ever actively drives
 *                         HIGH, which is what makes the wired-AND bus safe)
 *   OSPEEDR         = 11  high slew (fast enough edges through the
 *                         pull-up + bus capacitance)
 *   PUPDR [2p+1:2p] = 01  in-chip pull-up -- the fallback that makes a
 *                         BARE bus still read HIGH at release, which turns
 *                         "no device attached" into a clean NACK instead of
 *                         a dead bus
 * BSRR (not ODR read-modify-write) because it is atomic. IDR reads the
 * REAL line (every attached device + pull-up included), NOT our ODR intent:
 * bus_start() refuses when a line will not rise (wedged target / short /
 * swapped wires), and the ACK slot samples that truth on the 9th clock.
 *
 * == BIT TIMING (Radio's constants, DWT-timed) =============================
 * Arm the DWT cycle counter once (CoreDebug->DEMCR TRCENA, DWT->CTRL
 * CYCCNTENA, DWT->CYCCNT counts CPU cycles). One half bit period =
 *   half_cycles = SystemCoreClock / (khz * 2000)      [Radio soft_i2c_init]
 * 100 kHz at 168 MHz -> 840 cycles = 5.00 us per half bit, 10.00 us per
 * bit period. delay_cycles() stays bounded even if the DWT trace unit is
 * unavailable: at most 'cycles' spins, each worth a few cycles (Radio's
 * verbatim bound) -- a slower-but-terminating beat instead of a hang.
 * Why not SysTick for this delay: COUNTFLAG granularity is 1 ms -- two
 * hundred bit periods; useless at the wire (the bsp_oled.c finding). The
 * engine's ms-scale pacing API (i2c_soft_bus_delay_ms) IS SysTick-based,
 * matching every other bsp module.
 *
 * == PROTOCOL FRAMES (RM0090-free -- the wire defines everything) ==========
 * START  = SDA falling while SCL is HIGH. Generated only after BOTH lines
 *          verified HIGH; otherwise the call refuses (returns false) rather
 *          than fighting a held bus.
 * STOP   = SDA rising while SCL is HIGH. Always generated at the end of a
 *          transaction, including the NACK paths -- the bus is never left
 *          mid-frame.
 * Data   = 8 bits MSB first, each changed ONLY while SCL is LOW (SCL HIGH
 *          is the target's sampling window).
 * ACK    = the 9th clock: master releases SDA and SAMPLES it during SCL
 *          HIGH. The target pulling SDA LOW is ACK; the line staying HIGH
 *          through the pull-up is NACK -- the ONLY "no device" signal an
 *          I2C master ever gets.
 * Address byte = (addr7 << 1) | R/W -- Radio's `addr7 << 1` convention: the
 *          caller passes the 7-BIT address (0x50 for the 24C02), the shift
 *          happens here, in one place.
 * Multi-byte reads: master ACKs every byte except the LAST (NACK tells the
 *          target to stop driving SDA), then STOP.
 *
 * == BUS RECOVERY (the classic I2C unwedge) ================================
 * A target caught mid-byte when the host rebooted keeps SDA held LOW and
 * blocks every START (SDA can never fall while SCL is HIGH). The cure is
 * mechanical: up to 9 SCL pulses clock the stuck bit(s) out of the target's
 * shifter (I2C_RECOVER_PULSES = 9, Radio), then a STOP returns the bus to
 * idle. bsp_i2c_eeprom_init() runs this once at every init -- the soft
 * analogue of the old engine's CR1 SWRST reset.
 *
 * == HOW TO VERIFY =========================================================
 * Scope/logic analyzer on the bus: half-bit = 5.00 us at 100 kHz; SCL never
 * toggles during a refused START; the ACK slot shows the target's LOW pulse
 * on the 9th clock; STOP always present at transaction end. Menu> i2c_ee
 * exercises the engine through the EEPROM module's selftest.
 */
#include "i2c_soft_bus.h"

/* ---- module constants ------------------------------------------------------ */

/* Radio soft_i2c.c: I2C_RECOVER_PULSES (9u) -- the unwedge pulse count. */
#define I2C_SOFT_BUS_RECOVER_PULSES (9u)

/* Spin cap for the SysTick ms pacing: if COUNTFLAG never sets within it,
 * SysTick is not running -- bounded exit instead of a hang (house guard
 * constant, spec #5, same value as the other bsp modules). */
#define I2C_SOFT_BUS_SYSTICK_GUARD 2000000u

/* ---- wire-level primitives (Radio soft_i2c.c, register-form) --------------- */

/* scl_release/scl_drive_low/sda_release/sda_drive_low: Radio's four BSRR
 * helpers. BSRR[n] = release (line -> HIGH via pull-up), BSRR[n+16] =
 * drive LOW. Atomic -- no read-modify-write race. */
static void scl_release(const i2c_soft_bus_t *bus)
{
    bus->scl_port->BSRR = (uint32_t)bus->scl_pin;
}

static void scl_drive_low(const i2c_soft_bus_t *bus)
{
    bus->scl_port->BSRR = (uint32_t)bus->scl_pin << 16u;
}

static void sda_release(const i2c_soft_bus_t *bus)
{
    bus->sda_port->BSRR = (uint32_t)bus->sda_pin;
}

static void sda_drive_low(const i2c_soft_bus_t *bus)
{
    bus->sda_port->BSRR = (uint32_t)bus->sda_pin << 16u;
}

/* The bus listeners: IDR bits are the REAL line levels. Radio sda_level /
 * scl_level, verbatim. */
static bool sda_level(const i2c_soft_bus_t *bus)
{
    return (bus->sda_port->IDR & bus->sda_pin) != 0u;
}

static bool scl_level(const i2c_soft_bus_t *bus)
{
    return (bus->scl_port->IDR & bus->scl_pin) != 0u;
}

/* ---- timing ------------------------------------------------------------------ */

/* Arm the DWT cycle counter once (see walkthrough). Radio dwt_init. */
static bool s_dwt_ready;

static void dwt_init(void)
{
    if (s_dwt_ready)
    {
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* bit24: trace enable */
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* bit0: start counting */
    s_dwt_ready = true;
}

/* Busy-wait 'cycles' CPU cycles. Bounded even if the trace unit is
 * unavailable: one spin is worth a few cycles, so allowing 'cycles'
 * iterations keeps the same order of delay (Radio delay_cycles, verbatim
 * including the bound). */
static void delay_cycles(uint32_t cycles)
{
    uint32_t start;
    uint32_t spins = 0u;
    uint32_t max_spins;

    if (cycles == 0u)
    {
        return;
    }
    max_spins = cycles;
    start = DWT->CYCCNT;
    while (((DWT->CYCCNT - start) < cycles) && (spins < max_spins))
    {
        spins++;
    }
}

/* One half bit period (Radio bus_delay). */
static void bus_delay(const i2c_soft_bus_t *bus)
{
    delay_cycles(bus->half_cycles);
}

/* ---- protocol frames (Radio bus_start/bus_stop/bus_write_byte/
 * ---- bus_read_byte, line for line) ------------------------------------------ */

/* START: with both lines released, verify the bus is actually idle (both
 * HIGH on IDR), then SDA falls while SCL is HIGH. Returns false when a
 * line is held LOW -- the one fault a soft master can SEE before it even
 * starts (short, swapped wires, wedged target). */
static bool bus_start(i2c_soft_bus_t *bus)
{
    sda_release(bus);
    bus_delay(bus);
    scl_release(bus);
    bus_delay(bus);

    if (!sda_level(bus) || !scl_level(bus))
    {
        return false;
    }

    sda_drive_low(bus);
    bus_delay(bus); /* SDA falls, SCL still HIGH = START */
    scl_drive_low(bus);
    bus_delay(bus);

    return true;
}

/* STOP: SDA rises while SCL is HIGH. Ends EVERY framed sequence (Radio
 * bus_stop) so a NACK path still leaves the bus idle. */
static void bus_stop(i2c_soft_bus_t *bus)
{
    sda_drive_low(bus);
    bus_delay(bus);
    scl_release(bus);
    bus_delay(bus);
    sda_release(bus);
    bus_delay(bus);
}

/* One byte MSB-first + the 9th-bit ACK slot. Data changes only while SCL
 * is LOW; the target samples during SCL HIGH. ACK slot: master releases
 * SDA and samples DURING SCL HIGH -- target LOW = ACK, line stayed HIGH =
 * NACK (Radio bus_write_byte, verbatim). Returns true = ACKed. */
static bool bus_write_byte(i2c_soft_bus_t *bus, uint8_t byte)
{
    uint8_t mask;
    bool ack;

    for (mask = 0x80u; mask != 0u; mask >>= 1u)
    {
        if ((byte & mask) != 0u)
        {
            sda_release(bus);
        }
        else
        {
            sda_drive_low(bus);
        }
        bus_delay(bus);
        scl_release(bus); /* rising edge: target samples SDA now */
        bus_delay(bus);
        scl_drive_low(bus); /* falling edge: SDA may change again */
        bus_delay(bus);
    }

    /* 9th clock: master hands SDA to the target, then listens. */
    sda_release(bus);
    bus_delay(bus);
    scl_release(bus);
    bus_delay(bus);
    ack = !sda_level(bus); /* pulled LOW = ACK */
    scl_drive_low(bus);
    bus_delay(bus);

    return ack;
}

/* Receive one byte MSB-first, then the 9th-clock master-ACK/NACK slot:
 * 'ack' true = master pulls SDA LOW for one clock (more bytes coming),
 * false = master NACKs (last byte -- the target must release SDA for the
 * STOP). SDA is released through the whole byte; the target drives it
 * (Radio bus_read_byte, verbatim). */
static uint8_t bus_read_byte(i2c_soft_bus_t *bus, bool ack)
{
    uint8_t byte = 0u;
    uint8_t i;

    sda_release(bus);

    for (i = 0u; i < 8u; i++)
    {
        byte <<= 1u;
        bus_delay(bus);
        scl_release(bus);
        bus_delay(bus);
        if (sda_level(bus)) /* sample while SCL HIGH */
        {
            byte |= 1u;
        }
        scl_drive_low(bus);
        bus_delay(bus);
    }

    if (ack)
    {
        sda_drive_low(bus);
    }
    else
    {
        sda_release(bus);
    }
    bus_delay(bus);
    scl_release(bus);
    bus_delay(bus);
    scl_drive_low(bus);
    bus_delay(bus);
    sda_release(bus);

    return byte;
}

/* ---- public API ------------------------------------------------------------ */

void i2c_soft_bus_init(i2c_soft_bus_t *bus,
                       GPIO_TypeDef *scl_port, uint16_t scl_pin,
                       GPIO_TypeDef *sda_port, uint16_t sda_pin,
                       uint32_t khz)
{
    uint32_t pos;

    if ((bus == 0) || (khz == 0u))
    {
        return; /* Radio: NULL / khz==0 no-op */
    }

    bus->scl_port = scl_port;
    bus->scl_pin = scl_pin;
    bus->sda_port = sda_port;
    bus->sda_pin = sda_pin;

    /* kHz -> Hz, then split one bit period into two half cycles (Radio
     * verbatim): 100 kHz @ 168 MHz -> 840 cycles = 5.00 us. */
    bus->half_cycles = SystemCoreClock / (khz * 2000u);
    if (bus->half_cycles == 0u)
    {
        bus->half_cycles = 1u;
    }

    dwt_init();

    /* Pins -> GPIO output / open-drain / high slew / pull-up. Radio does
     * this via HAL_GPIO_Init(GPIO_MODE_OUTPUT_OD, GPIO_PULLUP,
     * GPIO_SPEED_FREQ_HIGH); register form (single-bit pins assumed --
     * the BSP passes PB6/PB7). The port clock must ALREADY be running
     * (caller contract, header note) or these writes are dead. */
    for (pos = 0u; pos < 16u; pos++)
    {
        uint32_t shift;
        if (((scl_pin >> pos) & 1u) != 0u)
        {
            shift = pos * 2u;
            scl_port->MODER = (scl_port->MODER & ~(3u << shift)) |
                              (1u << shift); /* 01: general output */
            scl_port->OTYPER |= scl_pin; /* open-drain (wired-AND) */
            scl_port->OSPEEDR = (scl_port->OSPEEDR & ~(3u << shift)) |
                                (3u << shift); /* 11: high slew */
            scl_port->PUPDR = (scl_port->PUPDR & ~(3u << shift)) |
                              (1u << shift); /* 01: pull-up */
        }
        if (((sda_pin >> pos) & 1u) != 0u)
        {
            shift = pos * 2u;
            sda_port->MODER = (sda_port->MODER & ~(3u << shift)) |
                              (1u << shift);
            sda_port->OTYPER |= sda_pin;
            sda_port->OSPEEDR = (sda_port->OSPEEDR & ~(3u << shift)) |
                                (3u << shift);
            sda_port->PUPDR = (sda_port->PUPDR & ~(3u << shift)) |
                              (1u << shift);
        }
    }

    /* Release both lines to idle-HIGH and let the pull-ups settle one
     * half period (Radio: scl_release + sda_release + bus_delay). */
    scl_release(bus);
    sda_release(bus);
    bus_delay(bus);
}

bool i2c_soft_bus_write(i2c_soft_bus_t *bus, uint8_t addr7,
                        const uint8_t *data, uint16_t len)
{
    uint16_t i;
    bool ok = true;

    if ((bus == 0) || (data == 0))
    {
        return false;
    }

    if (!bus_start(bus))
    {
        return false;
    }

    /* Address byte: (addr7 << 1) with R/W = 0 -- Radio's convention. */
    if (!bus_write_byte(bus, (uint8_t)(addr7 << 1u)))
    {
        ok = false;
    }

    for (i = 0u; ok && (i < len); i++)
    {
        if (!bus_write_byte(bus, data[i]))
        {
            ok = false; /* NACK: stop the payload ... */
        }
    }

    bus_stop(bus); /* ... but ALWAYS finish the frame */

    return ok;
}

bool i2c_soft_bus_read(i2c_soft_bus_t *bus, uint8_t addr7,
                       uint8_t *data, uint16_t len)
{
    uint16_t i;
    bool ok = true;

    if ((bus == 0) || (data == 0) || (len == 0u))
    {
        return false;
    }

    if (!bus_start(bus))
    {
        return false;
    }

    if (!bus_write_byte(bus, (uint8_t)((addr7 << 1u) | 1u)))
    {
        ok = false;
    }

    /* ACK every byte except the last (master NACKs it so the target
     * releases SDA for the STOP) -- Radio's (i + 1) < len predicate. */
    for (i = 0u; ok && (i < len); i++)
    {
        data[i] = bus_read_byte(bus, (uint16_t)(i + 1u) < len);
    }

    bus_stop(bus);

    return ok;
}

bool i2c_soft_bus_probe(i2c_soft_bus_t *bus, uint8_t addr7)
{
    bool ok;

    if (!bus_start(bus))
    {
        return false;
    }

    ok = bus_write_byte(bus, (uint8_t)(addr7 << 1u));
    bus_stop(bus);

    return ok;
}

void i2c_soft_bus_recover(i2c_soft_bus_t *bus)
{
    uint8_t i;

    if (bus == 0)
    {
        return;
    }

    sda_release(bus);
    for (i = 0u; i < I2C_SOFT_BUS_RECOVER_PULSES; i++)
    {
        scl_drive_low(bus);
        bus_delay(bus);
        scl_release(bus);
        bus_delay(bus);
    }
    bus_stop(bus);
}

void i2c_soft_bus_delay_ms(uint32_t ms)
{
    uint32_t guard = I2C_SOFT_BUS_SYSTICK_GUARD;

    while (ms > 0u)
    {
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u)
        {
            ms--;
            guard = I2C_SOFT_BUS_SYSTICK_GUARD; /* alive: refill */
        }
        else if (--guard == 0u)
        {
            return; /* SysTick dead: bounded exit */
        }
    }
}

/** @} */
