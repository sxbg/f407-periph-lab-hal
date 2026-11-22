/**
 ******************************************************************************
 * @file    bsp_oled.c
 * @brief   SH1107 128x64 OLED on a bit-banged soft I2C bus (PD9=SCL,
 *          PE15=SDA) -- teaching module with self-test. (HAL edition of
 *          projects/f407-periph-lab bsp_oled; EP11.)
 ******************************************************************************
 * @addtogroup BSP_OLED
 * @{
 *
 * == HAL EDITION SHAPE =====================================================
 * The register edition implemented the whole bus engine inline here (the
 * scl/sda BSRR helpers, bus_start/bus_write_byte, i2c_recover, and the
 * MODER/OTYPER/OSPEEDR/PUPDR pin config). The HAL edition CALLS the shared
 * bsp/i2c_soft_bus engine instead -- a 1:1 port of Radio soft_i2c.c, kept
 * byte-for-byte UNCHANGED and deliberately REGISTER-FORM: that file is the
 * A3 exemption and the TEACHING SUBJECT (EP11: the soft bus is the point;
 * the "HAL way" -- I2C1 IP + HAL_I2C_Master_Transmit -- is the contrast
 * the teaching draws, and on this silicon the hardware engine stalls
 * non-deterministically while the soft one is field-proven). What is left
 * in THIS file: the SH1107 grammar (control bytes, command table, page
 * writes), the framebuffer/text API, and the self-test -- all of it
 * HAL-clean (HAL clock macros + HAL_GPIO_ReadPin read-backs only).
 *
 * == CUBEMX CONFIG (absent from f407-periph-lab-hal.ioc -- by design) ======
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
 * |                   |                                     | is written by
 * |                   |                                     | i2c_soft_bus_init
 * | PE15 in IOC       | GPIO_Analog, label LCD_D12          | same (LCD 34-pin
 * |                   |                                     | socket rows: a 4-pin
 * |                   |                                     | OLED plugs onto the
 * |                   |                                     | same rails, EP12)
 * | GPIO port clocks  | MX_GPIO_Init arms ALL ports (incl.  | this module re-arms
 * |                   | GPIOD/GPIOE) at boot                | idempotently via
 * |                   |                                     | __HAL_RCC_ macros
 * |                   |                                     | (engine caller
 * |                   |                                     | contract)
 * | "Mode" equiv      | GPIO_Output + Output Open Drain     | Radio port mode
 * |                   |                                     | GPIO_MODE_OUTPUT_OD
 * | "Pull-up/Pull-down"| Pull-up (internal ~40k fallback)   | module usually has
 * |                   |                                     | 4.7-10k externals;
 * |                   |                                     | Radio sets
 * |                   |                                     | GPIO_PULLUP too
 * | "Maximum output
 * |  speed"           | High (OSPEEDR=11)                   | 400 kHz edges
 * | Bus speed         | ~400 kHz (half bit = 210 cycles     | Radio OLED_I2C_KHZ
 * |                   | @168 MHz, DWT-timed in the engine)  | = 400
 * | NVIC / DMA        | none (polling bit-bang)             | simplest teaching
 * |                   |                                     | form of the bus
 *
 * Pin config ownership, restated: PD9/PE15 are GPIO_Analog in the .ioc --
 * MX_GPIO_Init owns NOTHING active on them. i2c_soft_bus_init() is the
 * config owner: output open-drain + pull-up + high slew on both pins,
 * both lines released idle-HIGH. The HAL-flavoured equivalent of what the
 * engine writes (Radio's port, register form):
 *       GPIO_InitTypeDef init = {0};
 *       init.Mode  = GPIO_MODE_OUTPUT_OD;
 *       init.Pull  = GPIO_PULLUP;
 *       init.Speed = GPIO_SPEED_FREQ_HIGH;
 *       init.Pin   = GPIO_PIN_9;  HAL_GPIO_Init(GPIOD, &init);  (SCL)
 *       init.Pin   = GPIO_PIN_15; HAL_GPIO_Init(GPIOE, &init);  (SDA)
 * BSRR-style line control (HAL_GPIO_WritePin / the engine's BSRR writes)
 * is the sanctioned GPIO output pattern here -- on a bit-banged bus the
 * GPIO lines ARE the peripheral.
 *
 * == WIRE BASICS (full walkthrough lives in i2c_soft_bus.c) ================
 * Open-drain wired-AND: writing 1 to a line RELEASES it (pull-up wins ->
 * HIGH), writing 0 drives it LOW; a master can never fight the panel's
 * SDA (or a stretched SCL). IDR reads the REAL line level -- the START
 * check and the ACK slot are built on that truth. Bit timing is DWT-timed
 * in the engine: half bit = SystemCoreClock/(400 kHz*2) = 210 cycles at
 * 168 MHz = 1.25 us; one byte (8 data + 1 ACK) = 22.5 us. Why not SysTick
 * for bit timing: COUNTFLAG granularity is 1 ms -- four hundred bit
 * periods; useless at the wire. SysTick IS used here for the one
 * millisecond window the panel demands (DC-DC settle), COUNTFLAG+guard.
 * Bus protocol: START = SDA falling while SCL HIGH; STOP = SDA rising
 * while SCL HIGH; data changes only while SCL LOW; MSB first; the 9th
 * clock is the ACK slot (target LOW = ACK, line high = NACK). Address
 * bytes: panel 7-bit address 0x3C (or 0x3D, the alt strap) -> on-wire
 * 0x78 / 0x7A (addr<<1, R/W=0); Radio probes 0x3C then 0x3D.
 * SH1107 CONTROL BYTE (the step the I2C IP would hide from you): after
 *   the address byte every transaction opens with one control byte --
 *   bit7 Co = 0, bit6 D/C#: 0x00 = command transaction, 0x40 = data.
 *   A command and its argument ride as separate transactions (Radio).
 *
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
 *
 * == BLIND-WRITE SEMANTICS (deliberate, documented; engine-adapted) =======
 * The bus is write-only as far as the display grammar goes: the ONLY
 * hardware feedback is the 9th-bit ACK, sampled by the engine's probe.
 * NACK policy lives at MODULE level and is unchanged from the register
 * edition: a NACK is NEVER an error. One mechanical difference to be
 * honest about: the engine ABORTS the payload on a NACK (Radio
 * bus_write_byte gates every byte) -- so without a panel attached each
 * transaction shows START + address + STOP on the wire, and with a panel
 * the full byte stream goes out. The register edition clocked every byte
 * out blindly; the observable behaviour (init/selftest PASS without a
 * panel) is identical. So:
 *   - panel ACKs  -> s_present = 1, text visible on screen;
 *   - panel NACKs -> init/selftest still PASS, report says "(no display:
 *     cmds blind-sent)" -- SKIP semantics, nothing is invented;
 *   - line physically HELD LOW at START (short / swapped wires) -> the
 *     ONE real fault a write-only bus can see -> init returns -2 (after
 *     one 9-pulse + STOP recover attempt, the classic I2C unwedge).
 *
 * == FRAMEBUFFER / TEXT GEOMETRY (unchanged) ==============================
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
 * menu> oled (menu wiring re-added by the integrator)
 *   with the 4-pin OLED wired (VCC 3V3, GND, SCL->PD9, SDA->PE15):
 *     "PASS oled (ack@0x3C, text on lines 0-1)" and the panel shows
 *     line0 "F407-PERIPH-LAB", line1 "I2C OLED OK" (visual proof; the
 *     framebuffer side is asserted in software, the pixel side is for
 *     your eyes -- that IS the display-device lesson).
 *   bare bus (nothing wired): "PASS oled (no display: cmds blind-sent)"
 *     -- the address byte of every transaction goes out and NACKs (the
 *     engine then skips the payload and closes with STOP); scope on
 *     PD9/PE15 still shows the START/addr/STOP train at 400 kHz.
 *   FAIL step1 = lines held low (wiring); step2 = fb/flush defect
 *   (software); step3 = a wait window exhausted (SysTick dead?).
 */
#include "bsp_oled.h"

#include "main.h"     /* LCD_D14/LCD_D12 pin+port label macros (CubeMX) */
#include "stm32f4xx_hal.h"

#include "i2c_soft_bus.h" /* the bus engine: bsp/ peer, unchanged, A3-
                           * exempt teaching subject (see bsp_oled.h) */
#include "font5x7.h"  /* data-header exception (spec rule #7; see .h)  */

/* ---- panel constants ------------------------------------------------------ */

#define BSP_OLED_WIDTH 128u
#define BSP_OLED_HEIGHT 64u
#define BSP_OLED_PAGES (BSP_OLED_HEIGHT / 8u) /* 8 pages of 8 rows */

/* I2C identity: 7-bit panel address (Radio OLED_ADDR_PRIMARY/ALT). The
 * engine shifts it onto the wire ((addr<<1)|W = 0x78 / 0x7A). */
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

/* Spin cap between SysTick wraps (spec #5, house guard constant): a
 * SysTick-dead guard, bounded exit instead of a hang. */
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

static i2c_soft_bus_t s_bus;   /* PD9=SCL / PE15=SDA bit-bang instance   */
static uint8_t s_fb[BSP_OLED_PAGES][BSP_OLED_WIDTH]; /* RAM mirror */
static uint16_t s_dirty;    /* one bit per page awaiting a flush */
static uint8_t s_present;   /* panel ACKed its address at probe time */
static uint8_t s_i2c_addr;  /* 0x3C or 0x3D, whichever ACKed */
static uint8_t s_wait_limited; /* a SysTick ms-wait exhausted its window */
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

/* Millisecond wait on SysTick COUNTFLAG (1 ms tick), guarded against a
 * dead tick (bounded exit). Kept LOCAL (instead of the engine's
 * i2c_soft_bus_delay_ms) because step 3 of the selftest needs the
 * exhaustion FLAG, not just the bounded wait. Flags the exhaustion in
 * s_wait_limited -- the spec's "timeout window + error code" discipline;
 * the flag IS the error code here. COUNTFLAG read: read-only (A3
 * exemption). */
static void systick_wait_ms(uint32_t ms)
{
    uint32_t guard = BSP_OLED_SYSTICK_GUARD;

    while (ms > 0u) {
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0u) { /* read-only (A3 exemption) */
            ms--;
            guard = BSP_OLED_SYSTICK_GUARD; /* alive: refill */
        } else if (--guard == 0u) {
            s_wait_limited = 1u; /* tick dead: bounded exit, flagged */
            return;
        }
    }
}

/* ---- bus-level helpers (engine calls + HAL read-backs) ----------------------- */

/* Are SCL (PD9) / SDA (PE15) held LOW right now? HAL_GPIO_ReadPin reads
 * the IDR -- the REAL line level, pull-up and panel included. This is the
 * discriminator between "clean NACK" (lines idle HIGH through the
 * pull-up) and "stuck line" (the one fault a write-only bus can see).
 * Read-only probes (A3 exemption). */
static int lines_stuck(void)
{
    if (HAL_GPIO_ReadPin(LCD_D14_GPIO_Port, LCD_D14_Pin) == GPIO_PIN_RESET) {
        return 1; /* SCL held low */
    }
    if (HAL_GPIO_ReadPin(LCD_D12_GPIO_Port, LCD_D12_Pin) == GPIO_PIN_RESET) {
        return 1; /* SDA held low */
    }
    return 0;
}

/* Address-ACK probe (Radio soft_i2c_probe, engine form): the ONLY
 * hardware feedback a write-only display bus has. Returns 1 = ACKed,
 * 0 = clean NACK (nobody at this address), -1 = line HELD LOW even after
 * one recover attempt (9 pulses + STOP, the classic unwedge) -- wiring
 * fault, reported as -2 by bsp_oled_init. */
static int oled_probe(uint8_t addr7)
{
    if (i2c_soft_bus_probe(&s_bus, addr7)) {
        return 1;
    }
    i2c_soft_bus_recover(&s_bus); /* 9 pulses + STOP, then exactly one retry */
    if (i2c_soft_bus_probe(&s_bus, addr7)) {
        return 1;
    }
    if (lines_stuck() != 0) {
        return -1;
    }
    return 0;
}

/* ---- display command/data layer --------------------------------------------- */

/* Send one command byte as [START] 0x78 0x00 cmd [STOP] -- control byte
 * 0x00 = "commands follow" (walkthrough). NACK policy is the module's
 * blind-write rule, NOT the engine's abort-on-NACK: returns 1 = bytes
 * ACKed, 0 = clean NACK (keep going -- commands still "went out", SKIP
 * semantics), -1 = line held low (the only fatal bus state). */
static int panel_cmd(uint8_t cmd)
{
    uint8_t frame[2];

    frame[0] = OLED_CTRL_CMD;
    frame[1] = cmd;
    if (i2c_soft_bus_write(&s_bus, s_i2c_addr, frame, 2u)) {
        return 1;
    }
    return (lines_stuck() != 0) ? -1 : 0;
}

/* Push one whole page: page cursor + column cursor commands, then a
 * 129-byte data transaction (0x40 + 128 px bytes) -- one START/STOP per
 * page, the Radio bus_data shape. Frame is STATIC to keep 129 bytes off
 * the stack; the menu loop is single-threaded so a shared buffer is safe
 * (Radio's own rationale, kept). Returns 1 = page sent (or blind-sent:
 * NACK tolerated), 0 = bus refused (line held low). */
static int panel_write_page(uint8_t page, const uint8_t *pixels)
{
    static uint8_t frame[1u + BSP_OLED_WIDTH];
    uint8_t offset = (uint8_t)OLED_COL_OFFSET;
    uint16_t i;
    int rc;

    rc = panel_cmd((uint8_t)(OLED_CMD_SET_PAGE | page));
    if (rc < 0) {
        return 0; /* 0xB0|page: page cursor */
    }
    rc = panel_cmd((uint8_t)(OLED_CMD_COL_LOW | (offset & 0x0Fu)));
    if (rc < 0) {
        return 0; /* 0x00|lo nibble of the column cursor */
    }
    rc = panel_cmd((uint8_t)(OLED_CMD_COL_HIGH | ((offset >> 4) & 0x0Fu)));
    if (rc < 0) {
        return 0; /* 0x10|hi nibble */
    }
    frame[0] = OLED_CTRL_DATA; /* 0x40: "128 data bytes follow" */
    for (i = 0u; i < (uint16_t)BSP_OLED_WIDTH; i++) {
        frame[1u + i] = pixels[i];
    }
    if (i2c_soft_bus_write(&s_bus, s_i2c_addr, frame,
                           (uint16_t)(1u + BSP_OLED_WIDTH))) {
        return 1;
    }
    return (lines_stuck() != 0) ? 0 : 1; /* blind NACK is not a refusal */
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
    int rc;
    uint32_t i;

    /* Kernel clocks, HAL form (the register edition's AHB1ENR bit writes;
     * the engine's caller contract: the board layer owns RCC). MX_GPIO_Init
     * arms all ports at boot already -- these are idempotent. */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* PD9 = SCL (CubeMX label LCD_D14), PE15 = SDA (CubeMX label LCD_D12):
     * the ENGINE owns the pin config -- output, OPEN-DRAIN, pull-up, high
     * slew (the register form of the HAL_GPIO_Init call quoted in the
     * walkthrough) -- and releases both lines idle-HIGH, and arms the DWT
     * bit clock (half bit = SystemCoreClock/(kHz*2000), floor 1 cycle).
     * PE must not depend on CubeMX: both pins are GPIO_Analog in the .ioc
     * and stay that way (ownership rule in bsp_oled.h). */
    i2c_soft_bus_init(&s_bus,
                      LCD_D14_GPIO_Port, LCD_D14_Pin, /* SCL = PD9 */
                      LCD_D12_GPIO_Port, LCD_D12_Pin, /* SDA = PE15 */
                      BSP_OLED_I2C_KHZ);

    /* Config read-back (spec ST1 evidence, register-level READS only --
     * A3 exemption; the writes above were the engine's): both pins output
     * (MODER 01) + open-drain (OTYPER 1) + pull-up (PUPDR 01), and BOTH
     * lines idle HIGH -- HAL_GPIO_ReadPin on the IDR. */
    if (((GPIOD->MODER >> 18) & 0x3u) != 0x1u ||  /* read-only (A3 exemption) */
        ((GPIOE->MODER >> 30) & 0x3u) != 0x1u ||
        ((GPIOD->OTYPER >> 9) & 0x1u) != 0x1u ||
        ((GPIOE->OTYPER >> 15) & 0x1u) != 0x1u ||
        ((GPIOD->PUPDR >> 18) & 0x3u) != 0x1u ||
        ((GPIOE->PUPDR >> 30) & 0x3u) != 0x1u ||
        (HAL_GPIO_ReadPin(LCD_D14_GPIO_Port, LCD_D14_Pin) == GPIO_PIN_RESET) ||
        (HAL_GPIO_ReadPin(LCD_D12_GPIO_Port, LCD_D12_Pin) == GPIO_PIN_RESET)) {
        return -1;
    }

    /* Probe the two possible straps (Radio probes 0x3C then 0x3D). NACK
     * on both is NOT an error (blind-write semantics): commands still go
     * out below, selftest reports "(no display...)". Only a line HELD LOW
     * (after one recover attempt, inside oled_probe) is a real fault. */
    s_present = 0u;
    s_i2c_addr = OLED_ADDR_PRIMARY;
    rc = oled_probe(OLED_ADDR_PRIMARY);
    if (rc < 0) {
        return -2; /* SCL/SDA held low: wiring fault */
    }
    if (rc > 0) {
        s_present = 1u;
    } else {
        rc = oled_probe(OLED_ADDR_ALT);
        if (rc < 0) {
            return -2;
        }
        if (rc > 0) {
            s_present = 1u;
            s_i2c_addr = OLED_ADDR_ALT;
        }
    }

    /* SH1107 init sequence, byte-blind at MODULE level (NACKs keep
     * going), with the DC-DC settle window (SysTick COUNTFLAG + guard;
     * exhaustion flags s_wait_limited). A stuck line is the only abort. */
    for (i = 0u; i < OLED_INIT_SEQ_N; i++) {
        if (panel_cmd(s_init_seq[i].cmd) < 0) {
            return -2; /* bus stuck mid-sequence */
        }
        if (s_init_seq[i].has_arg != 0u) {
            if (panel_cmd(s_init_seq[i].arg) < 0) {
                return -2;
            }
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
    if (panel_cmd(OLED_CMD_DISPLAY_ON) < 0) { /* 0xAF: display on */
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
    /* Step 1: full init (engine pin config + read-back + idle-high lines
     * + probe + sequence + blank flush + display-on). rc -1 = config
     * mismatch, -2 = stuck line. A MISSING panel passes this step by
     * design (blind-write at module level). */
    if (bsp_oled_init() != 0) {
        oled_set_report("(pins stuck low / cfg mismatch)");
        return (int)BSP_OLED_ST1_INIT;
    }

    /* Step 2: the text path. Clear, write the two banner lines, verify
     * the framebuffer landed (software loopback -- the read-back a
     * display bus cannot give you), flush, and require every dirty flag
     * consumed (the whole 8-page frame went out). Worst case wire time
     * (panel attached): 8 pages x (3 cmds + 129 bytes) x 22.5 us/byte ~=
     * 24 ms + the 100 ms settle in step 1 -- all bounded by construction
     * (fixed bytes at a fixed bit clock; the engine's DWT spin guard
     * bounds each half period even with the counter dead). */
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
