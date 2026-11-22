/**
 * @file bsp_oled.h
 * @brief SH1107 128x64 OLED on a BIT-BANGED soft I2C bus (PD9=SCL,
 *        PE15=SDA) -- init, 4-line x 16-char text API, self-test.
 *        (HAL edition of projects/f407-periph-lab bsp_oled; EP11.)
 *
 * HAL MAPPING NOTE (register edition -> HAL edition):
 *   - The wire-level bit-bang primitives that lived INLINE in the old
 *     bsp_oled.c (bus_start/bus_write_byte/i2c_recover/... and the
 *     MODER/OTYPER/OSPEEDR/PUPDR pin config) now live in bsp/
 *     i2c_soft_bus.c -- the shared Radio-proven engine, and THE TEACHING
 *     SUBJECT of this module. This module CALLS that engine
 *     (i2c_soft_bus_init/write/probe/recover/delay_ms); it does not
 *     re-implement a single wire transition.
 *   - A3 EXCEPTION (deliberate, documented): i2c_soft_bus.c bit-bangs
 *     the bus with direct GPIO BSRR/IDR/MODER/OTYPER register traffic
 *     for edge-accurate timing. That file IS the teaching subject (the
 *     soft bus is the point of EP11) and is exempt from the A3
 *     HAL-call-only rule -- it is kept byte-for-byte UNCHANGED from the
 *     register edition. bsp_oled.c itself stays HAL-clean: its own GPIO
 *     traffic is HAL_GPIO_ReadPin read-backs and HAL clock macros only.
 *   - The EP11 teaching comparison this module exists for: the hardware
 *     HAL would offer I2C1 IP calls (HAL_I2C_Master_Transmit); the soft
 *     bus makes every START/ACK/stretch visible at the register level
 *     instead -- board finding 2026-11-21: the F4 hardware I2C engine
 *     stalls non-deterministically, the soft engine does not.
 *   - Pin identity UNCHANGED: SCL = PD9, SDA = PE15. In the hal project's
 *     main.h these carry the CubeMX labels LCD_D14 (GPIOD, GPIO_PIN_9)
 *     and LCD_D12 (GPIOE, GPIO_PIN_15) -- the LCD 34-pin socket
 *     placeholder labels, same as in the old project.
 *
 * Teaching module (P3f wave, bsp-module-spec.md template -- see bsp_uart).
 * Full config table / walkthrough / the SH1107 command stream / the
 * "display controller vs storage device" I2C comparison and the
 * blind-write semantics live in the bsp_oled.c file header.
 *
 * Adapted from the Radio reference project (projects/Radio .../adapters/
 * sh1107/ + soft_i2c/): the bit timing, the 0x00/0x40 control-byte
 * convention, the panel init sequence and the 5x7 font encoding are the
 * field-proven Radio code. The bus engine lives in i2c_soft_bus.c/h
 * (sanctioned data/peer-header exception to the "bsp modules do not
 * include each other" rule -- same precedent as font5x7.h).
 *
 * Ownership rule (same precedent as bsp_i2c_eeprom): neither I2C nor the
 * OLED exists as an IP in f407-periph-lab-hal.ioc (knowledge #13:
 * adapter-owned peripherals stay out of the .ioc). PD9/PE15 sit there
 * only as GPIO_Analog placeholders (labels LCD_D14/LCD_D12,
 * anti-collision) -- MX_GPIO_Init configures NOTHING active on them.
 * The pin CONFIG (output open-drain + pull-up + high slew) is owned by
 * i2c_soft_bus_init() and applied at every bsp_oled_init(); the GPIO
 * port clocks are armed here via __HAL_RCC_GPIOD/GPIOE_CLK_ENABLE (the
 * engine's caller contract: the board layer owns RCC). No CubeMX code
 * will ever conflict.
 */
#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdint.h>

/* Text grid the API exposes. 128x64 px panel, one text line every 16 px
 * (4 lines), 16 characters per line -- the classic 16x4 LCD geometry.
 * See the .c header for the pixel math (6 px/char cell, blank page under
 * each glyph page as built-in line spacing). */
#define BSP_OLED_LINES 4u
#define BSP_OLED_COLS 16u

/* Soft-bus init + panel probe + SH1107 init sequence + framebuffer clear
 * + display-on. i2c_soft_bus_init() puts PD9/PE15 into output open-drain
 * + pull-up + high slew and releases both lines; a config read-back and
 * idle-level probe follow; then the panel probe at 0x3C/0x3D (with one
 * 9-pulse unwedge attempt per address) and the init sequence. Returns
 * 0 = sequence sent (see blind-write note -- a MISSING panel is NOT an
 * error here), -1 = pin config read-back mismatch / lines not idling
 * HIGH, -2 = SCL/SDA held low (shorted/wrong wiring; bus recover
 * attempted once per probe first). Idempotent (safe to re-run by the
 * selftest). */
int bsp_oled_init(void);

/* Erase the whole framebuffer (all 8 pages zero) and mark every page for
 * the next flush. Does NOT touch the panel by itself -- screen follows at
 * the next internal flush (init / selftest flush; keep-your-own-pace API,
 * the async per-page flush of the Radio scheduler is out of bsp scope). */
void bsp_oled_clear(void);

/* Write one text line: 'line' 0..3, 'str' truncated at 16 chars. Glyphs
 * come from font5x7 (column-major, bit 0 = top); lower-case folds to
 * upper-case, out-of-range chars print '?' (Radio encoding). Renders into
 * the framebuffer and marks the touched page dirty. */
void bsp_oled_puts(uint8_t line, const char *str);

/* One-line report of the LAST selftest run, for the menu verdict suffix
 * (module itself has no console dependency). Examples:
 *   "(ack@0x3C, text on lines 0-1)"  -- panel answered
 *   "(no display: cmds blind-sent)"  -- SKIP semantics, wires absent
 */
const char *bsp_oled_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed (menu prints "FAIL oled step<N>"; last_report carries the why).
 *   1 = init: pin config read-back + idle-high lines + init sequence +
 *       display-on sent with no stuck-line exception (ACK absence is NOT
 *       a FAIL here -- the bus is write-only/blind at MODULE level; see
 *       .c header)
 *   2 = clear + "F407-PERIPH-LAB" on line 0 + "I2C OLED OK" on line 1,
 *       framebuffer content verified in software, whole frame flushed,
 *       dirty flags consumed
 *   3 = timeout-window discipline: every bounded wait (DC-DC settle,
 *       flush spin guards) finished inside its window
 * PASS both WITH and WITHOUT a panel attached; the report suffix tells
 * the two cases apart ("no display" = SKIP semantics: the address bytes
 * went out (and were NACKed -- the engine skips the payload on NACK),
 * visual check only possible with a wired panel). */
int bsp_oled_selftest(void);

#endif /* BSP_OLED_H */
