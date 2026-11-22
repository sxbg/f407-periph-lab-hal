/**
 * @file bsp_i2c_eeprom.h
 * @brief Soft-I2C bus host (PB6=SCL/PB7=SDA) driving the onboard 24C02
 *        EEPROM at device address 0x50 -- init, byte read/write APIs,
 *        self-test.
 *
 * Teaching module (P3 wave, bsp-module-spec.md template -- see bsp_uart).
 * The 24C02 command layer is unchanged; the WIRE ENGINE behind it is the
 * Radio-proven soft bus (i2c_soft_bus, the open-drain bit-bang port) --
 * the F4 hardware I2C host stalled non-deterministically on real hardware
 * (board finding 2026-11-21), the soft engine on the same silicon does
 * not. Full engine walkthrough / electrical contract / timing derivation /
 * verification recipe live in bsp_i2c_eeprom.c + i2c_soft_bus.c headers.
 *
 * Ownership rule: PB6/PB7 are NOT real peripherals in the IOC (knowledge
 * #13: adapter-owned resources stay out) -- they sit there as GPIO_Analog
 * placeholders (labels EE_SCL/EE_SDA, anti-collision only). This module
 * is the FULL owner of the RCC->AHB1ENR GPIOBEN arm and of the PB6/PB7
 * pin config (GPIO output, open-drain, high slew, pull-up -- written by
 * i2c_soft_bus_init). No CubeMX code will ever conflict.
 */
#ifndef BSP_I2C_EEPROM_H
#define BSP_I2C_EEPROM_H

#include <stdint.h>

/* 24C02 geometry (board manual + bsp-module-spec): 256 bytes, word
 * address 0x00..0xFF is ONE byte wide, page size 8 bytes. */
#define BSP_I2C_EEPROM_SIZE 256u
#define BSP_I2C_EEPROM_PAGE 8u /* page-write boundary (see walkthrough) */

/* Soft-bus init: GPIOB clock arm + i2c_soft_bus_init on PB6/PB7 (GPIO
 * output, open-drain, pull-up) at BSP_I2C_EE_BUS_KHZ, then one 9-clock
 * bus recovery (the soft analogue of the old SWRST). Returns 0 = PASS
 * (pin config read-back: MODER output, OTYPER open-drain, PUPDR pull-up
 * on both pins), -1 = mismatch. Idempotent. */
int bsp_i2c_eeprom_init(void);

/* Write one byte to the 24C02 word address 'addr' (0x00..0xFF): START ->
 * (dev<<1)|W -> word address -> data -> STOP. Blocks for the device's
 * internal write cycle afterwards via ACK polling (START+address probes
 * spaced 1 ms apart until the device ACKs -- tWR of the 24C02, up to
 * ~5 ms), so a write followed by a read is safe to back-to-back.
 * Returns 0 = ok, -1 = device NACKed (missing/misaddressed device, or
 * still NACKing when the poll window closed), -2 = bus fault (a line
 * held LOW at START -- short/swapped wires/wedged target). */
int bsp_i2c_eeprom_write_byte(uint8_t addr, uint8_t data);

/* Random read of one byte at word address 'addr': TWO clean transactions
 * (no repeated-Start -- the shape the board finding converged on):
 * START -> (dev<<1)|W -> word address -> STOP, then START -> (dev<<1)|R
 * -> NACK (single-byte read) -> data -> STOP. The pointer-set write
 * carries no data byte, so it never triggers the tWR write cycle.
 * Returns 0 = value delivered in *data, -1 = NACK, -2 = bus fault. */
int bsp_i2c_eeprom_read_byte(uint8_t addr, uint8_t *data);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line (module itself has no console dependency).
 * Examples: "(0x55/0xAA roundtrip ok)" / "(scan: no ack 0x50-0x57)". */
const char *bsp_i2c_eeprom_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed (the menu prints "FAIL i2c_ee step<N>"; bsp_i2c_eeprom_last_report
 * carries the "why"). Steps:
 *   1 = init + config read-back: PB6/PB7 in GPIO output mode with OTYPER
 *       open-drain and PUPDR pull-up (the soft bus electricals)
 *   2 = 7-bit address scan 0x50..0x57 via engine probes (refines the
 *       device address), then write 0x55 @0x10 -> read back -> compare
 *       (a NACK here is "device absent or busy", reported -- the 24C02
 *       is soldered onto this board and is EXPECTED to answer)
 *   3 = write 0xAA @0x11 -> read back -> compare
 *   4 = bus idle: after the last STOP both SCL and SDA read HIGH through
 *       IDR (open-drain released, pull-ups win = recessive state) */
int bsp_i2c_eeprom_selftest(void);

#endif /* BSP_I2C_EEPROM_H */
