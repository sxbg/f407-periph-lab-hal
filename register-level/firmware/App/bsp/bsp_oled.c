/**
 ******************************************************************************
 * @file    bsp_oled.c
 * @brief   SH1107 128x64 OLED on a bit-banged soft I2C bus (PD9=SCL,
 *          PE15=SDA) -- teaching module with self-test.
 ******************************************************************************
 * @addtogroup BSP_OLED
 * @{
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab.ioc -- by design) ==========
 * | Field             | Value (module truth)                | Why
 * |-------------------|-------------------------------------|-------------------
 * | I2Cx / OLED IP    | NOT PRESENT in the IOC              | knowledge #13:
 * |                   | (no peripheral clock, no CR regs)   | adapter-owned
 * |                   |                                     | devices stay out;
 * |                   |                                     | the bus is GPIO
 * |                   |                                     | bit-bang, not I2C IP
 * | PD9 in IOC        | GPIO_Analog, label LCD_D14          | anti-collision
 * |                   |                                     | placeholder only;
 * |                   |                                     | output OD config
 * |                   |                                     | is written HERE
 * | PE15 in IOC       | GPIO_Analog, label LCD_D12          | same (LCD 34-pin
 * |                   |                                     | socket rows: a 4-pin
 * |                   |                                     | OLED plugs onto the
 * |                   |                                     | same rails, EP12)
 * | "Mode" equiv      | GPIO_Output + Output Open Drain     | Radio port mode
 * |                   |                                     | GPIO_MODE_OUTPUT_OD
 * | "Pull-up/Pull-down"| Pull-up (internal ~40k fallback)   | module usually has
 * |                   |                                     | 4.7-10k externals;
 * |                   |                                     | Radio sets
 * |                   |                                     | GPIO_PULLUP too
 * | "Maximum output
 * |  speed"           | High (OSPEEDR=11)                   | 400 kHz edges
 * | Bus speed         | ~400 kHz (half bit = 210 cycles     | Radio OLED_I2C_KHZ
 * |                   | @168 MHz, DWT-timed)                | = 400
 * | NVIC / DMA        | none (polling bit-bang, blind write)| simplest teaching
 * |                   |                                     | form of the bus
 * == REGISTER WALKTHROUGH ==================================================
 * Clock gates: RCC->AHB1ENR bit3 (GPIODEN) + bit4 (GPIOEEN), re-armed
 *   idempotently; the read-back stalls the core until the port clocks are
 *   live. HAL equivalent: __HAL_RCC_GPIOD_CLK_ENABLE() / __GPIOE...
 * Pin config (both pins, PD9 shown; PE15 mirrors at field 15):
 *   MODER [19:18] = 01 general-purpose OUTPUT (the bus is master-driven;
 *     input is sampled through IDR on the same pin, output buffers on).
 *   OTYPER bit9 = 1 OPEN-DRAIN (RM 10.3: "output control: push-pull vs
 *     open-drain"). THE wired-AND trick the whole I2C family leans on:
 *     writing 0 actively pulls the pin LOW; writing 1 only RELEASES it,
 *     and the line returns HIGH through a pull-up. A master can therefore
 *     never fight the panel's SDA (or a stretched SCL) -- arbitration and
 *     clock-stretching fall out of the physics for free. HAL equivalent
 *     (this is verbatim what the Radio port does):
 *       GPIO_InitTypeDef init = {0};
 *       init.Mode  = GPIO_MODE_OUTPUT_OD;
 *       init.Pull  = GPIO_PULLUP;
 *       init.Speed = GPIO_SPEED_FREQ_HIGH;
 *       init.Pin   = GPIO_PIN_9;  HAL_GPIO_Init(GPIOD, &init);  (SCL)
 *       init.Pin   = GPIO_PIN_15; HAL_GPIO_Init(GPIOE, &init);  (SDA)
 *   OSPEEDR [19:18] = 11 high slew (400 kHz edges need <25 ns rise into
 *     ~40 k + wire capacitance; the internal pull-up alone is the slow
 *     path, external 4.7-10 k resistors on the module do the real work).
 *   PUPDR [19:18] = 01 pull-up (the fallback that makes a BARE bus still
 *     read HIGH when released -- that is what makes "no panel attached"
 *     look like a clean NACK instead of a stuck line).
 *   ODR via BSRR: BSRR[n]=1 releases (line -> HIGH), BSRR[n+16]=1 drives
 *     low. BSRR is atomic -- no read-modify-write race with the ISR-less
 *     but still re-entrant init path. IDR[n] reads the REAL line level
 *     (not the ODR intent): this is the bus "listener" the start
 *     condition and the ACK slot are built on.
 * Bit timing: DWT cycle counter (CM4 debug core, RM Part B "Data Watchpoint
 *   and Trace"): CoreDebug->DEMCR bit24 TRCENA arms it, DWT->CTRL bit0
 *   CYCCNTENA starts DWT->CYCCNT counting CPU cycles. Half a bit at
 *   400 kHz = 168 MHz / (400k * 2) = 210 cycles = 1.25 us; one byte (8
 *   data + 1 ACK) takes 9 x 2.5 us = 22.5 us. The delay helper is bounded
 *   even if DWT is dead (Radio's bound: at most 'cycles' spins, each spin
 *   costing a few cycles). Why not SysTick for bit timing: SysTick's
 *   COUNTFLAG granularity is 1 ms -- four hundred bit periods; useless at
 *   the wire level. SysTick IS used for the millisecond windows (DC-DC
 *   settle) with the same COUNTFLAG+guard pattern as bsp_i2c_eeprom.
 * Bus protocol (every transition generated by hand):
 *   START = SDA falling while SCL is HIGH; STOP = SDA rising while SCL is
 *   HIGH. Data bits may only change while SCL is LOW (the window between
 *   the falling and the next rising edge). Bits go MSB first. The 9th
 *   clock is the ACK slot: master releases SDA; the TARGET pulling it LOW
 *   is "ACK", nobody pulling (line stays HIGH through the pull-up) is
 *   "NACK". On a write-only display bus the NACK carries exactly one bit
 *   of information: "no one at this address".
 * I2C address bytes: panel 7-bit address 0x3C (or 0x3D, the alt strap)
 *   -> on-wire 0x78 / 0x7A (addr<<1, R/W=0). Radio probes 0x3C then 0x3D
 *   because modules ship strapped either way.
 * SH1107 CONTROL BYTE (the step the I2C IP would hide from you): after
 *   the address byte every transaction opens with one control byte.
 *   bit7 Co = 0: ALL following bytes keep this type; bit6 D/C#: 0 =
 *   command, 1 = display RAM data. This module uses the Radio convention:
 *   0x00 opens a command transaction, 0x40 opens a data transaction.
 *   A command and its argument may ride together (Co=0) or as separate
 *   transactions -- Radio sends them separately, we keep that.
 * == TWO I2C DEVICE SHAPES (EP11 vs EP12 -- the teaching payload) ==========
 * STORAGE DEVICE (24C02, EP11): the I2C transaction itself carries an
 *   ADDRESS INTO the device. "S 0xA0 word_addr data P": word_addr is a
 *   memory location, and a RANDOM READ is "set the pointer, then fetch".
 *   The device remembers the pointer; reads and writes hit cells.
 * DISPLAY CONTROLLER (SH1107, here): the transaction carries a STREAM.
 *   There is no per-transaction sub-address -- the control byte only says
 *   "commands follow" (0x00) or "pixels follow" (0x40), and WHERE the
 *   pixels land is set beforehand BY COMMANDS (page = 0xB0|n, column =
 *   0x00|lo, 0x10|hi). The controller keeps that cursor and advances it
 *   byte after byte; you paint, you never "read back a cell". Two device
 *   personalities on one identical electrical bus -- the address phase is
 *   the only part the I2C spec itself defines; everything after it is the
 *   device's own grammar. (EP12 discovered this panel family on the 4-pin
 *   header rails while wiring the SPI flash.)
 * == BLIND-WRITE SEMANTICS (deliberate, documented) ========================
 * A soft-I2C master CAN sample the 9th-bit ACK, and we do -- for the probe
 * only. Every other transaction is sent BLIND: bytes are clocked out, ACK
 * is sampled but never gates the flow and never raises an error. Why: the
 * panel is an external dupont-wired module (may be absent), and a display
 * read-back is not part of its grammar (see the two-shapes section). So:
 *   - panel ACKs  -> s_present = 1, text visible on screen;
 *   - panel NACKs -> init/selftest still PASS, report says "(no display:
 *     cmds blind-sent)" -- SKIP semantics, nothing is invented;
 *   - line physically HELD LOW at START (short / swapped wires) -> the
 *     ONE real fault a write-only bus can see -> init returns -2 (after
 *     one 9-pulse + STOP recover attempt, the classic I2C unwedge).
 * == FRAMEBUFFER / TEXT GEOMETRY ==========================================
 * s_fb is 8 pages x 128 columns (page = 8-row band, bit0 = TOP row of the
 * band). A text line sits every 16 px: line L paints pages 2L (glyphs)
 * and leaves page 2L+1 blank -- free line spacing on a 64-px tall panel,
 * 4 lines total. Glyph cell = 5 px wide + 1 px gap = 6 px advance, 16
 * chars max per line = 96 px of the 128 (32 px right margin, like the
 * Radio UI rows). font5x7 encoding (Radio, tools/gen_font.py): one byte
 * per COLUMN, bit 0 = TOP pixel of that column -- which is EXACTLY the
 * panel page-byte orientation, so a glyph column copies into the page
 * with no bit rotation at all. Lower-case folds to upper-case, unknown
 * -> '?' (Radio rules).
 * == HOW TO VERIFY =========================================================
 * menu> oled
 *   with the 4-pin OLED wired (VCC 3V3, GND, SCL->PD9, SDA->PE15):
 *     "PASS oled (ack@0x3C, text on lines 0-1)" and the panel shows
 *     line0 "F407-PERIPH-LAB", line1 "I2C OLED OK" (visual proof; the
 *     framebuffer side is asserted in software, the pixel side is for
 *     your eyes -- that IS the display-device lesson).
 *   bare bus (nothing wired): "PASS oled (no display: cmds blind-sent)"
 *     -- commands went out, NACKed, nothing asserted. Scope on PD9/PE15
 *     still shows the full init waveform (START, 0x78, 0x00, bytes...).
 *   FAIL step1 = lines held low (wiring); step2 = fb/flush defect
 *   (software); step3 = a wait window exhausted (SysTick dead?).
 */
#include "bsp_oled.h"

#include "hal_port.h" /* family seam: pulls in the CMSIS device header */
#include "font5x7.h"  /* data-header exception (spec rule #7; see .h)  */

/* ---- panel constants ------------------------------------------------------ */

#define BSP_OLED_WIDTH 128u
#define BSP_OLED_HEIGHT 64u
#define BSP_OLED_PAGES (BSP_OLED_HEIGHT / 8u) /* 8 pages of 8 rows */

/* I2C identity: 7-bit panel address (Radio OLED_ADDR_PRIMARY/ALT) and the
 * on-wire byte it expands to (addr<<1, R/W=0): 0x78 / 0x7A. */
#define OLED_ADDR_PRIMARY 0x3Cu
#define OLED_ADDR_ALT 0x3Du

/* Control bytes (Radio OLED_CTRL_CMD/DATA): Co=0, D/C#=cmd/data. */
#define OLED_CTRL_CMD 0x00u
#define OLED_CTRL_DATA 0x40u

/* Bus speed and the one long settle the panel demands. */
#define BSP_OLED_I2C_KHZ 400u
#define BSP_OLED_DCDC_SETTLE_MS 100u /* DC-DC pump needs it before 0xAF */
#define OLED_CONTRAST_DEFAULT 0xCFu  /* Radio CONTRAST_DEFAULT */
#define OLED_COL_OFFSET 0u /* some 128-wide panels start at column 2 -- the
                            * single knob to turn if text sits 2 px off */

/* SH1107 command set -- names follow Radio sh1107.c (which follows the
 * U8g2/u8x8 SH1107 lineage). Commands taking an argument are marked in
 * the sequence table; a missing argument would silently swallow the NEXT
 * command byte as its argument. */
#define OLED_CMD_DISPLAY_OFF 0xAEu    /* sleep; config goes in while blank */
#define OLED_CMD_DISPLAY_ON 0xAFu     /* wake, show RAM */
#define OLED_CMD_SET_CLOCK_DIV 0xD5u  /* + arg: internal OSC / frame rate */
#define OLED_CMD_SET_MUX 0xA8u        /* + arg: COM row count - 1 */
#define OLED_CMD_SET_OFFSET 0xD3u     /* + arg: vertical pan of COM0 */
#define OLED_CMD_SET_START_LINE 0x40u /* reg display start line (0..63) */
#define OLED_CMD_DC_ON 0xADu          /* + arg: DC-DC control (SH1107-spec) */
#define OLED_CMD_SET_DISP_START 0xDCu /* + arg: display start column (SH1107-spec) */
#define OLED_CMD_SET_MEM_MODE 0x20u   /* + arg: 0x00 = page addressing */
#define OLED_CMD_SEG_REMAP 0xA1u      /* segment 127->SEG0 map (single byte) */
#define OLED_CMD_COM_SCAN_DEC 0xC8u   /* COM scan COM63 -> COM0 */
#define OLED_CMD_SET_COM_PINS 0xDAu   /* + arg: COM pin hardware config */
#define OLED_CMD_SET_CONTRAST 0x81u   /* + arg: brightness 0..255 */
#define OLED_CMD_SET_PRECHARGE 0xD9u  /* + arg: phase 1/2 precharge periods */
#define OLED_CMD_SET_VCOM 0xDBu       /* + arg: VCOMH deselect level */
#define OLED_CMD_ENTIRE_ON_RESUME 0xA4u /* show RAM (0xA5 would be all-on) */
#define OLED_CMD_NORMAL_DISPLAY 0xA6u /* non-inverted (0xA7 = invert) */
#define OLED_CMD_SET_PAGE 0xB0u       /* | page: page-address cursor (hi nib) */
#define OLED_CMD_COL_LOW 0x00u        /* | lo nibble of the column cursor */
#define OLED_CMD_COL_HIGH 0x10u       /* | hi nibble of the column cursor */

/* Self-test step numbers (the value bsp_oled_selftest returns). */
#define BSP_OLED_ST1_INIT 1u /* pins + init sequence + display-on */
#define BSP_OLED_ST2_TEXT 2u /* clear + 2 lines + fb verify + flush */
#define BSP_OLED_ST3_TIME 3u /* every bounded wait stayed in-window */

/* Spin cap between SysTick wraps (spec #5, same constant as the other
 * bsp modules): SysTick-dead guard, bounded exit instead of a hang. */
#define BSP_OLED_SYSTICK_GUARD 2000000u

/* ---- init sequence (Radio's proven table, one adaptation) ------------------ */

typedef struct {
    uint8_t cmd;
    uint8_t arg;
    uint8_t has_arg;
    uint16_t delay_after_ms;
} oled_cmd_t;

/* Radio sh1107.c s_init_sh1107_128x128 -- the sequence that PROVED the
 * SH1107 command set on real silicon (0xAD 0x8B DC-DC and 0xDC start
 * column are SH1107-specific; an SSD1306 would use 0x8D 0x14 and have no
 * 0xDC). Byte-for-byte EXCEPT the MUX row: Radio's panel is 128 rows
 * (0x7F), ours is 64 rows (0x3F) -- the row count the datasheet mandates
 * for a 128x64 panel. Orientation pair 0xA1 + 0xC8 = the Radio mapping;
 * if a panel ever shows the image upside-down, that pair (0xA1/0xC8 ->
 * 0xA0/0xC0) is the thing to flip -- classic first-bring-up knob. */
static const oled_cmd_t s_init_seq[] = {
    { OLED_CMD_DISPLAY_OFF, 0x00u, 0u, 0u },  /* 0xAE: sleep first */
    { OLED_CMD_SET_CLOCK_DIV, 0x80u, 1u, 0u }, /* 0xD5 0x80: OSC f/1, no divide */
    { OLED_CMD_SET_MUX, 0x3Fu, 1u, 0u }, /* 0xA8 0x3F: 64 COM rows (Radio: 0x7F) */
    { OLED_CMD_SET_OFFSET, 0x00u, 1u, 0u }, /* 0xD3 0x00: COM pan = 0 */
    { OLED_CMD_SET_START_LINE, 0x00u, 0u, 0u }, /* 0x40: start line 0 */
    { OLED_CMD_DC_ON, 0x8Bu, 1u, BSP_OLED_DCDC_SETTLE_MS }, /* 0xAD 0x8B: pump on,
                              * then the 100 ms settle the pump needs */
    { OLED_CMD_SET_DISP_START, 0x00u, 1u, 0u }, /* 0xDC 0x00: start column 0 */
    { OLED_CMD_SEG_REMAP, 0x00u, 0u, 0u }, /* 0xA1: columns remapped */
    { OLED_CMD_COM_SCAN_DEC, 0x00u, 0u, 0u }, /* 0xC8: COMs scanned top-down */
    { OLED_CMD_SET_COM_PINS, 0x12u, 1u, 0u }, /* 0xDA 0x12: alt COM config */
    { OLED_CMD_SET_CONTRAST, OLED_CONTRAST_DEFAULT, 1u, 0u }, /* 0x81 0xCF */
    { OLED_CMD_SET_MEM_MODE, 0x00u, 1u, 0u }, /* 0x20 0x00: page addressing */
    { OLED_CMD_SET_PRECHARGE, 0x22u, 1u, 0u }, /* 0xD9 0x22: 1/2 DCLK phases */
    { OLED_CMD_SET_VCOM, 0x35u, 1u, 0u }, /* 0xDB 0x35: VCOMH = 0.77*Vcc */
    { OLED_CMD_ENTIRE_ON_RESUME, 0x00u, 0u, 0u }, /* 0xA4: RAM drives pixels */
    { OLED_CMD_NORMAL_DISPLAY, 0x00u, 0u, 0u }, /* 0xA6: not inverted */
};
#define OLED_INIT_SEQ_N (sizeof(s_init_seq) / sizeof(s_init_seq[0]))

/* ---- module state ----------------------------------------------------------- */

static uint8_t s_fb[BSP_OLED_PAGES][BSP_OLED_WIDTH]; /* RAM mirror */
static uint16_t s_dirty;    /* one bit per page awaiting a flush */
static uint8_t s_present;   /* panel ACKed its address at probe time */
static uint8_t s_i2c_addr;  /* 0x3C or 0x3D, whichever ACKed */
static uint8_t s_wait_limited; /* a SysTick ms-wait exhausted its window */
static uint8_t s_dwt_ready;    /* cycle counter armed once */
static uint32_t s_half_cycles; /* CPU cycles per half bit (210 @168 MHz) */
static char s_report[48];      /* last selftest report for the menu */

/* ---- report / time helpers ---------------------------------------------------- */

/* Copy a fixed string into s_report (tiny, no libc -- house pattern). */
static void oled_set_report(const char *txt)
{
    char *p = s_report;
    while (*txt != '\0') {
        *p++ = *txt++;
    }
    *p = '\0';
}

/* Arm the DWT cycle counter once (register walk in the file header). */
static void dwt_arm(void)
{
    if (s_dwt_ready != 0u) {
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* bit24: enable trace */
    DWT->CYCCNT = 0u; /* zero the cycle counter ... */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* ... bit0: start counting */
    s_dwt_ready = 1u;
}

/* Busy-wait 'cycles' CPU cycles, bounded even if the DWT is dead: one
 * spin is worth a few cycles, so capping spins at 'cycles' keeps the
 * same order of delay (Radio soft_i2c.c delay_cycles, verbatim bound). */
static void dwt_delay_cycles(uint32_t cycles)
{
    uint32_t start;
    uint32_t spins = 0u;
    uint32_t max_spins;

    if (cycles == 0u) {
        return;
    }
    max_spins = cycles;
    start = DWT->CYCCNT;
    while (((DWT->CYCCNT - start) < cycles) && (spins < max_spins)) {
        spins++;
    }
}

/* Millisecond wait on SysTick COUNTFLAG (1 ms tick), guarded against a
 * dead tick (bounded exit). Flags the exhaustion in s_wait_limited so
 * selftest step 3 can report it -- the spec's "timeout window + error
 * code" discipline; the flag IS the error code here. Same COUNTFLAG
 * pattern as bsp_i2c_eeprom. */
static void systick_wait_ms(uint32_t ms)
{
    uint32_t guard = BSP_OLED_SYSTICK_GUARD;

    while (ms > 0u) {
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) {
            ms--;
            guard = BSP_OLED_SYSTICK_GUARD; /* alive: refill */
        } else if (--guard == 0u) {
            s_wait_limited = 1u; /* tick dead: bounded exit, flagged */
            return;
        }
    }
}

/* ---- wire-level primitives (Radio soft_i2c.c, register-form) --------------- */

/* Writing 1 to an open-drain line = RELEASE (pull-up wins -> HIGH);
 * writing 0 = DRIVE LOW. BSRR is atomic (no RMW race). HAL equivalent:
 * HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET/RESET) -- which ends in the
 * same BSRR write. */
static void scl_release(void) { GPIOD->BSRR = GPIO_BSRR_BS_9; }
static void scl_low(void) { GPIOD->BSRR = GPIO_BSRR_BR_9; }
static void sda_release(void) { GPIOE->BSRR = GPIO_BSRR_BS_15; }
static void sda_low(void) { GPIOE->BSRR = GPIO_BSRR_BR_15; }

/* The bus listener: IDR reads the REAL line (pull-up or panel included),
 * not our intent in ODR. The start check and the ACK slot live on it. */
static int sda_level(void) { return (GPIOE->IDR & GPIO_IDR_IDR_15) != 0u; }
static int scl_level(void) { return (GPIOD->IDR & GPIO_IDR_IDR_9) != 0u; }

/* One half bit period, DWT-timed (see walkthrough for the 210-cycle math
 * and the dead-DWT bound). */
static void bus_half_period(void) { dwt_delay_cycles(s_half_cycles); }

/* START condition: with both lines released-high, SDA falls while SCL is
 * HIGH. Returns 0 when either line reads LOW after release -- someone is
 * holding the bus (short, swapped wires, wedged target): the one fault a
 * write-only master can actually SEE. */
static int bus_start(void)
{
    sda_release();
    bus_half_period();
    scl_release();
    bus_half_period();
    if ((sda_level() == 0) || (scl_level() == 0)) {
        return 0; /* line(s) held low: refuse instead of fighting */
    }
    sda_low();
    bus_half_period(); /* SDA falls, SCL still high = START */
    scl_low();
    bus_half_period();
    return 1;
}

/* STOP condition: SDA rises while SCL is HIGH. Always completes the
 * framed sequence so the bus is never left mid-transaction. */
static void bus_stop(void)
{
    sda_low();
    bus_half_period();
    scl_release();
    bus_half_period();
    sda_release();
    bus_half_period(); /* SDA rises while SCL high = STOP */
}

/* Clock out one byte MSB-first + the 9th-bit ACK slot. Data changes only
 * while SCL is LOW; SCL HIGH is the target's sampling window. In the ACK
 * slot the master lets go of SDA and READS: LOW = the target ACKed.
 * Returns 1 = ACK seen, 0 = NACK (line stayed high) -- the caller decides
 * whether NACK matters (probe: yes; blind display stream: never). */
static int bus_write_byte(uint8_t byte)
{
    uint8_t mask;
    int ack;

    for (mask = 0x80u; mask != 0u; mask >>= 1u) {
        if ((byte & mask) != 0u) {
            sda_release(); /* 1 = release, pull-up shows it */
        } else {
            sda_low(); /* 0 = drive it */
        }
        bus_half_period();
        scl_release(); /* rising edge: target samples SDA now */
        bus_half_period();
        scl_low(); /* falling edge: SDA may change again */
        bus_half_period();
    }
    sda_release(); /* 9th bit: master hands SDA to the target */
    bus_half_period();
    scl_release();
    bus_half_period();
    ack = (sda_level() == 0); /* target pulled low = ACK */
    scl_low();
    bus_half_period();
    return ack;
}

/* Classic I2C unwedge (Radio soft_i2c_recover): a target mid-byte that
 * keeps SDA low blocks every START; 9 SCL pulses clock any half-sent
 * byte out, then a STOP returns the bus to idle. Bounded: 9 x half
 * periods. */
static void i2c_recover(void)
{
    uint8_t i;

    sda_release();
    for (i = 0u; i < 9u; i++) {
        scl_low();
        bus_half_period();
        scl_release();
        bus_half_period();
    }
    bus_stop();
}

/* One transaction: START, address byte, payload, STOP. ACK of every byte
 * is sampled but NEVER gated on (blind-write semantics, file header):
 * returns 1 = transaction clocked out cleanly (bus was free at START),
 * 0 = START refused (line held low -- the only reported fault). */
static int i2c_write(const uint8_t *bytes, uint16_t len)
{
    uint16_t i;

    if (!bus_start()) {
        return 0;
    }
    (void)bus_write_byte((uint8_t)(s_i2c_addr << 1)); /* addr+W; ACK ignored */
    for (i = 0u; i < len; i++) {
        (void)bus_write_byte(bytes[i]); /* ACK ignored: blind by design */
    }
    bus_stop();
    return 1;
}

/* Address-ACK probe (Radio soft_i2c_probe): the ONLY hardware feedback a
 * write-only display bus has. ran=0 + stuck=1 means the line was held
 * LOW even after a recover attempt -> wiring fault, reported as -2. */
static int i2c_probe(uint8_t addr7, int *acked, int *stuck)
{
    int ack;

    *acked = 0;
    *stuck = 0;
    if (!bus_start()) {
        i2c_recover(); /* 9 pulses + STOP, then exactly one retry */
        if (!bus_start()) {
            *stuck = 1;
            return 0;
        }
    }
    ack = bus_write_byte((uint8_t)(addr7 << 1)); /* address only, then STOP */
    bus_stop();
    *acked = ack;
    return 1;
}

/* ---- display command/data layer --------------------------------------------- */

/* Send one command byte: [START] 0x78 0x00 cmd [STOP] -- control byte
 * 0x00 = "commands follow" (walkthrough). Blind: NACK not judged. */
static int panel_cmd(uint8_t cmd)
{
    uint8_t frame[2];

    frame[0] = OLED_CTRL_CMD;
    frame[1] = cmd;
    return i2c_write(frame, 2u);
}

/* Push one whole page: page cursor + column cursor commands, then a
 * 129-byte data transaction (0x40 + 128 px bytes) -- one START/STOP per
 * page, the Radio bus_data shape. Frame is STATIC to keep 129 bytes off
 * the stack; the menu loop is single-threaded so a shared buffer is safe
 * (Radio's own rationale, kept). Returns 0 = bus refused (line stuck). */
static int panel_write_page(uint8_t page, const uint8_t *pixels)
{
    static uint8_t frame[1u + BSP_OLED_WIDTH];
    uint8_t offset = (uint8_t)OLED_COL_OFFSET;
    uint16_t i;

    if (!panel_cmd((uint8_t)(OLED_CMD_SET_PAGE | page))) {
        return 0; /* 0xB0|page: page cursor */
    }
    if (!panel_cmd((uint8_t)(OLED_CMD_COL_LOW | (offset & 0x0Fu)))) {
        return 0; /* 0x00|lo nibble of the column cursor */
    }
    if (!panel_cmd((uint8_t)(OLED_CMD_COL_HIGH | ((offset >> 4) & 0x0Fu)))) {
        return 0; /* 0x10|hi nibble */
    }
    frame[0] = OLED_CTRL_DATA; /* 0x40: "128 data bytes follow" */
    for (i = 0u; i < (uint16_t)BSP_OLED_WIDTH; i++) {
        frame[1u + i] = pixels[i];
    }
    return i2c_write(frame, (uint16_t)(1u + BSP_OLED_WIDTH));
}

/* Flush every dirty page, in order 0..7 (a teaching module flushes
 * synchronously; Radio's round-robin flush_step exists for its
 * cooperative scheduler and is deliberately not carried over). Returns
 * 0 = all pages sent, -1 = bus refused mid-flush (page stays dirty --
 * the next flush retries it, Radio's dirty-until-sent rule). */
static int fb_flush_all(void)
{
    uint8_t page;

    for (page = 0u; page < BSP_OLED_PAGES; page++) {
        if ((s_dirty & (uint16_t)(1u << page)) == 0u) {
            continue; /* clean page: no traffic */
        }
        if (panel_write_page(page, s_fb[page]) == 0) {
            return -1;
        }
        s_dirty &= (uint16_t)~(1u << page);
    }
    return 0;
}

/* ---- public API ------------------------------------------------------------ */

int bsp_oled_init(void)
{
    uint32_t i;
    int acked;
    int stuck;

    /* Kernel clocks: AHB1ENR bit3 GPIODEN + bit4 GPIOEEN, idempotent
     * re-arm; the read-back stalls until the port clocks are live.
     * HAL: __HAL_RCC_GPIOD_CLK_ENABLE()/__GPIOE... */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    (void)RCC->AHB1ENR;

    /* PD9 = SCL, PE15 = SDA: output, OPEN-DRAIN, pull-up, high slew --
     * the register form of the Radio port's HAL_GPIO_Init call (full HAL
     * equivalent quoted in the walkthrough). PE15 mirrors PD9 at field 15.
     * PE must not depend on CubeMX: both pins are GPIO_Analog in the .ioc
     * and stay that way (ownership rule in bsp_oled.h). */
    GPIOD->MODER = (GPIOD->MODER & ~GPIO_MODER_MODER9_Msk) |
                   GPIO_MODER_MODER9_0; /* [19:18]=01 output */
    GPIOD->OTYPER |= GPIO_OTYPER_OT_9;  /* SCL open-drain (wired-AND) */
    GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~GPIO_OSPEEDR_OSPEED9_Msk) |
                     GPIO_OSPEEDR_OSPEED9; /* [19:18]=11 high slew */
    GPIOD->PUPDR = (GPIOD->PUPDR & ~GPIO_PUPDR_PUPD9_Msk) |
                   GPIO_PUPDR_PUPD9_0; /* [19:18]=01 pull-up fallback */
    GPIOE->MODER = (GPIOE->MODER & ~GPIO_MODER_MODER15_Msk) |
                   GPIO_MODER_MODER15_0; /* [31:30]=01 output */
    GPIOE->OTYPER |= GPIO_OTYPER_OT_15;  /* SDA open-drain */
    GPIOE->OSPEEDR = (GPIOE->OSPEEDR & ~GPIO_OSPEEDR_OSPEED15_Msk) |
                     GPIO_OSPEEDR_OSPEED15; /* [31:30]=11 high slew */
    GPIOE->PUPDR = (GPIOE->PUPDR & ~GPIO_PUPDR_PUPD15_Msk) |
                   GPIO_PUPDR_PUPD15_0; /* [31:30]=01 pull-up fallback */

    /* Both lines start RELEASED (BSRR set): bus idle = SCL and SDA HIGH. */
    scl_release();
    sda_release();

    /* Config read-back (spec ST1 evidence, register-level only): both
     * pins output + open-drain + pull-up, and BOTH lines idle HIGH. */
    if (((GPIOD->MODER >> 18) & 0x3u) != 0x1u ||
        ((GPIOE->MODER >> 30) & 0x3u) != 0x1u ||
        ((GPIOD->OTYPER >> 9) & 0x1u) != 0x1u ||
        ((GPIOE->OTYPER >> 15) & 0x1u) != 0x1u ||
        ((GPIOD->PUPDR >> 18) & 0x3u) != 0x1u ||
        ((GPIOE->PUPDR >> 30) & 0x3u) != 0x1u ||
        ((GPIOD->IDR & GPIO_IDR_IDR_9) == 0u) ||
        ((GPIOE->IDR & GPIO_IDR_IDR_15) == 0u)) {
        return -1;
    }

    /* Half bit = SystemCoreClock/(400 kHz*2) = 210 cycles @168 MHz
     * (Radio soft_i2c_init formula; floor at 1 cycle). DWT arms here. */
    dwt_arm();
    s_half_cycles = SystemCoreClock / (BSP_OLED_I2C_KHZ * 2000u);
    if (s_half_cycles == 0u) {
        s_half_cycles = 1u;
    }

    /* Probe the two possible straps (Radio probes 0x3C then 0x3D). NACK
     * on both is NOT an error (blind-write semantics): commands still go
     * out below, selftest reports "(no display...)". Only a line HELD LOW
     * (after one recover attempt, inside i2c_probe) is a real fault. */
    s_present = 0u;
    s_i2c_addr = OLED_ADDR_PRIMARY;
    if (i2c_probe(OLED_ADDR_PRIMARY, &acked, &stuck) == 0) {
        return -2; /* SCL/SDA held low: wiring fault */
    }
    if (acked != 0) {
        s_present = 1u;
    } else if (i2c_probe(OLED_ADDR_ALT, &acked, &stuck) == 0) {
        return -2;
    } else if (acked != 0) {
        s_present = 1u;
        s_i2c_addr = OLED_ADDR_ALT;
    }

    /* SH1107 init sequence, byte-blind, with the DC-DC settle window
     * (SysTick COUNTFLAG + guard; exhaustion flags s_wait_limited). */
    for (i = 0u; i < OLED_INIT_SEQ_N; i++) {
        if (panel_cmd(s_init_seq[i].cmd) == 0) {
            return -2; /* bus refused mid-sequence */
        }
        if ((s_init_seq[i].has_arg != 0u) &&
            (panel_cmd(s_init_seq[i].arg) == 0)) {
            return -2;
        }
        if (s_init_seq[i].delay_after_ms != 0u) {
            systick_wait_ms(s_init_seq[i].delay_after_ms);
        }
    }

    /* Known-state screen: erase the RAM mirror, flush all 8 pages WHILE
     * still blank (Radio pushes the cleared frame before 0xAF so power-on
     * never shows random RAM), then wake the panel. */
    bsp_oled_clear();
    if (fb_flush_all() != 0) {
        return -2;
    }
    if (panel_cmd(OLED_CMD_DISPLAY_ON) == 0) { /* 0xAF: display on */
        return -2;
    }
    return 0;
}

void bsp_oled_clear(void)
{
    uint8_t page;

    for (page = 0u; page < BSP_OLED_PAGES; page++) {
        uint8_t col;
        for (col = 0u; col < BSP_OLED_WIDTH; col++) {
            s_fb[page][col] = 0u;
        }
    }
    s_dirty = (uint16_t)((1u << BSP_OLED_PAGES) - 1u); /* all pages dirty */
}

void bsp_oled_puts(uint8_t line, const char *str)
{
    uint8_t col;       /* character slot 0..15 */
    uint8_t page;      /* panel page holding this line's glyphs = line*2 */

    if ((line >= BSP_OLED_LINES) || (str == 0)) {
        return; /* out-of-grid or NULL: ignore (no error path needed) */
    }
    page = (uint8_t)(line * 2u); /* 16 px line pitch -> text on even pages */

    for (col = 0u; (col < BSP_OLED_COLS) && (*str != '\0'); col++, str++) {
        uint8_t code = (uint8_t)*str;
        const uint8_t *glyph;
        uint8_t gx; /* glyph column 0..4 */
        uint8_t gy; /* glyph row 0..6 = page bit = panel row */

        /* Radio fold: the font is upper-case only; lower-case folds,
         * anything outside the table prints '?'. */
        if ((code >= (uint8_t)'a') && (code <= (uint8_t)'z')) {
            code = (uint8_t)(code - 32u);
        }
        if ((code < FONT5X7_FIRST) || (code > FONT5X7_LAST)) {
            code = (uint8_t)'?';
        }
        glyph = &font5x7[(code - FONT5X7_FIRST) * FONT5X7_W];

        /* Copy the 5 column-bytes straight into the page: font bit0 =
         * top pixel = page bit0 -- no rotation needed (geometry note in
         * the file header). Bit7 of the page byte stays 0: 1 px gap
         * under every glyph. */
        for (gx = 0u; gx < FONT5X7_W; gx++) {
            uint8_t bits = glyph[gx];
            uint16_t px = (uint16_t)((col * (FONT5X7_W + 1u)) + gx);

            for (gy = 0u; gy < FONT5X7_H; gy++) {
                if ((bits & (uint8_t)(1u << gy)) != 0u) {
                    s_fb[page][px] |= (uint8_t)(1u << gy);
                }
            }
        }
    }
    /* Mark this line's page (and only it) for the next flush. */
    s_dirty |= (uint16_t)(1u << page);
}

const char *bsp_oled_last_report(void)
{
    return s_report;
}

/* Non-zero-column counter for one page: the fb-side evidence of "text
 * actually landed where the API said it would". */
static uint16_t fb_nonzero_cols(uint8_t page)
{
    uint8_t col;
    uint16_t count = 0u;

    for (col = 0u; col < BSP_OLED_WIDTH; col++) {
        if (s_fb[page][col] != 0u) {
            count++;
        }
    }
    return count;
}

/* ST2 framebuffer assertion: line 0 and line 1 pages carry ink (15-char
 * and 11-char strings guarantee >=10 and >=5 non-zero columns), every
 * OTHER page is still zero (clear + the blank spacing pages untouched --
 * catches a wild page-index write). */
static int fb_text_ok(void)
{
    uint8_t page;

    if (fb_nonzero_cols(0u) < 10u) {
        return 0;
    }
    if (fb_nonzero_cols(2u) < 5u) {
        return 0;
    }
    for (page = 1u; page < BSP_OLED_PAGES; page++) {
        if (page == 2u) {
            continue; /* line 1's glyph page: ink expected */
        }
        if (fb_nonzero_cols(page) != 0u) {
            return 0;
        }
    }
    return 1;
}

int bsp_oled_selftest(void)
{
    /* Step 1: full init (pin config read-back + probe + sequence + blank
     * flush + display-on). rc -1 = config mismatch, -2 = stuck line.
     * A MISSING panel passes this step by design (blind-write). */
    if (bsp_oled_init() != 0) {
        oled_set_report("(pins stuck low / cfg mismatch)");
        return (int)BSP_OLED_ST1_INIT;
    }

    /* Step 2: the text path. Clear, write the two banner lines, verify
     * the framebuffer landed (software loopback -- the read-back a
     * display bus cannot give you), flush, and require every dirty flag
     * consumed (the whole 8-page frame went out). Worst case wire time:
     * 8 pages x (3 cmds + 129 bytes) x 22.5 us/byte ~= 24 ms + the 100 ms
     * settle in step 1 -- all bounded by construction (fixed bytes at a
     * fixed bit clock; the DWT spin guard bounds each half period even
     * with the counter dead). */
    bsp_oled_clear();
    bsp_oled_puts(0u, "F407-PERIPH-LAB");
    bsp_oled_puts(1u, "I2C OLED OK");
    if (fb_text_ok() == 0) {
        oled_set_report("(fb text pattern bad)");
        return (int)BSP_OLED_ST2_TEXT;
    }
    if (fb_flush_all() != 0) {
        oled_set_report("(flush: line stuck low)");
        return (int)BSP_OLED_ST2_TEXT;
    }
    if (s_dirty != 0u) {
        oled_set_report("(flush dirty left)");
        return (int)BSP_OLED_ST2_TEXT;
    }

    /* Step 3: timeout-window discipline -- every SysTick ms-wait this run
     * (the DC-DC settle) finished inside its window; the flag is the
     * module's single "wait failed" error code. */
    if (s_wait_limited != 0u) {
        oled_set_report("(timeout window exhausted)");
        return (int)BSP_OLED_ST3_TIME;
    }

    /* Verdict suffix distinguishes SKIP (no panel) from the wired case:
     * with a panel, LOOK at it -- the pixel side of a display is verified
     * by eyes, the framebuffer side just got asserted in software. */
    if (s_present != 0u) {
        if (s_i2c_addr == OLED_ADDR_ALT) {
            oled_set_report("(ack@0x3D, text on lines 0-1)");
        } else {
            oled_set_report("(ack@0x3C, text on lines 0-1)");
        }
    } else {
        oled_set_report("(no display: cmds blind-sent)");
    }
    return 0;
}

/** @} */
