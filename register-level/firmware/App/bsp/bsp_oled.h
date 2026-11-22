/**
 * @file bsp_oled.h
 * @brief SH1107 128x64 OLED on a BIT-BANGED soft I2C bus (PD9=SCL,
 *        PE15=SDA) -- init, 4-line x 16-char text API, self-test.
 *
 * Teaching module (P3f wave, bsp-module-spec.md template -- see bsp_uart).
 * Full CubeMX config table / register walkthrough / the SH1107 command
 * stream / the "display controller vs storage device" I2C comparison and
 * the blind-write semantics live in the bsp_oled.c file header.
 *
 * Adapted from the Radio reference project (projects/Radio .../adapters/
 * sh1107/ + soft_i2c/): the bit timing, the 0x00/0x40 control-byte
 * convention, the panel init sequence and the 5x7 font encoding are the
 * field-proven Radio code, flattened from Radio's port+driver+scheduler
 * layering into ONE self-contained bsp module (spec rule #7: bsp modules
 * do not include each other; font5x7.h is the sanctioned data-header
 * exception).
 *
 * Ownership rule (same precedent as bsp_i2c_eeprom): neither I2C nor the
 * OLED exists as an IP in f407-periph-lab.ioc (knowledge #13: adapter-
 * owned peripherals stay out of the .ioc). PD9/PE15 sit there only as
 * GPIO_Analog placeholders (labels LCD_D14/LCD_D12, anti-collision).
 * This module is the FULL owner of the PD9/PE15 pin config (output
 * open-drain + pull-up) and of every bus transition on the pair. No
 * CubeMX code will ever conflict.
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

/* Register-level init: PD9/PE15 clocks + pin config (output open-drain,
 * pull-up, high speed), both lines released, DWT cycle counter armed,
 * panel probe at 0x3C/0x3D, then the SH1107 init sequence + framebuffer
 * clear + display-on. Returns 0 = sequence sent (see blind-write note --
 * a MISSING panel is NOT an error here), -1 = pin config read-back
 * mismatch, -2 = SCL/SDA held low (shorted/wrong wiring; bus recover
 * attempted once first). Idempotent (safe to re-run by the selftest). */
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
 *   1 = init: pin config read-back + init sequence + display-on sent with
 *       no stuck-line exception (ACK absence is NOT a FAIL here -- the
 *       bus is write-only/blind; see .c header)
 *   2 = clear + "F407-PERIPH-LAB" on line 0 + "I2C OLED OK" on line 1,
 *       framebuffer content verified in software, whole frame flushed,
 *       dirty flags consumed
 *   3 = timeout-window discipline: every bounded wait (DC-DC settle,
 *       flush spin guards) finished inside its window
 * PASS both WITH and WITHOUT a panel attached; the report suffix tells
 * the two cases apart ("no display" = SKIP semantics: commands went out
 * blind, visual check only possible with a wired panel). */
int bsp_oled_selftest(void);

#endif /* BSP_OLED_H */
