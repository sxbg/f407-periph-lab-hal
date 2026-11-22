/**
 * @file bsp_i2c_eeprom.h
 * @brief Hardware I2C1 host (HAL edition: PB6=SCL/PB7=SDA AF4, 100 kHz
 *        standard mode) driving the onboard 24C02 EEPROM at device
 *        address 0x50 -- init verify, byte read/write APIs, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_i2c_eeprom). The
 * 24C02 command layer is unchanged; the WIRE ENGINE behind it changed:
 * the register edition bit-banged a soft open-drain bus (i2c_soft_bus)
 * because the F4 hardware I2C host stalled non-deterministically on
 * real hardware (board finding 2026-11-21, old-project section 3.2).
 * The HAL edition is the TEACHING MAIN LINE: HAL_I2C_Mem_Write /
 * HAL_I2C_Mem_Read / HAL_I2C_IsDeviceReady are the industry-standard
 * shape (one call = one transaction, HAL owns START/STOP/ACK/timeout).
 * KNOWN BOUNDARY carried over from the old project: if the hardware
 * engine's random-read stall reappears during W6 board bring-up, record
 * it in knowledge/ as a known boundary of this board revision -- do NOT
 * resurrect the soft bus inside this module.
 *
 * Ownership rule (inverted from the register edition): CubeMX owns
 * PB6/PB7 (I2C1_MspInit: AF4, open-drain, pull-up) and I2C1's whole
 * register file (CR1/TIMINGR/CCR are MX_I2C1_Init territory). This
 * module only VERIFIES the generated configuration (read-back) and
 * talks the bus through HAL calls. A3 rule: no peripheral control-
 * register writes here; the only raw-register touches are read-only
 * probes, each marked with a read-only (A3 exemption) comment.
 */
#ifndef BSP_I2C_EEPROM_H
#define BSP_I2C_EEPROM_H

#include <stdint.h>

/* 24C02 geometry (board manual + bsp-module-spec): 256 bytes, word
 * address 0x00..0xFF is ONE byte wide, page size 8 bytes. */
#define BSP_I2C_EEPROM_SIZE 256u
#define BSP_I2C_EEPROM_PAGE 8u /* page-write boundary (see walkthrough) */

/* Bus speed the CubeMX configuration promises (100 kHz standard mode,
 * DutyCycle 2). Used by the init read-back only -- the actual TIMINGR/
 * CCR math is HAL_I2C_Init's job. */
#define BSP_I2C_EE_BUS_KHZ 100u

/* Verify the CubeMX-generated configuration (read-back only): hi2c1
 * live, ClockSpeed = BSP_I2C_EE_BUS_KHZ, CR1 PE set, bus lines idle
 * HIGH (pull-ups win). The HAL owns every control register; nothing is
 * written here. Returns 0 = PASS, -1 = mismatch. Idempotent. */
int bsp_i2c_eeprom_init(void);

/* Write one byte to the 24C02 word address 'addr' (0x00..0xFF) via
 * HAL_I2C_Mem_Write (one call = one transaction: START -> (dev<<1)|W ->
 * word address -> data -> STOP). Blocks for the device's internal
 * write cycle afterwards via ACK polling (HAL_I2C_IsDeviceReady probes
 * spaced 1 ms apart until the device ACKs -- tWR of the 24C02, up to
 * ~5 ms), so a write followed by a read is safe to back-to-back.
 * Returns 0 = ok, -1 = device NACKed (missing/misaddressed device, or
 * still NACKing when the poll window closed), -2 = bus fault (HAL
 * timeout / a line held LOW). */
int bsp_i2c_eeprom_write_byte(uint8_t addr, uint8_t data);

/* Random read of one byte at word address 'addr' via HAL_I2C_Mem_Read:
 * the HAL issues the pointer-set write and the read (with its internal
 * repeated-Start) in ONE call. The register edition used two clean
 * transactions to dodge the hardware engine's Sr stall -- that shape
 * became HAL-internal; see the known-boundary note in the file header.
 * Returns 0 = value delivered in *data, -1 = NACK, -2 = bus fault. */
int bsp_i2c_eeprom_read_byte(uint8_t addr, uint8_t *data);

/* One-line report of the LAST selftest run, for the menu to append to its
 * verdict line (module itself has no console dependency).
 * Examples: "(0x55/0xAA roundtrip ok)" / "(scan: no ack 0x50-0x57)". */
const char *bsp_i2c_eeprom_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed (the menu prints "FAIL i2c_ee step<N>"; bsp_i2c_eeprom_last_report
 * carries the "why"). Steps:
 *   1 = init + config read-back: I2C1 enabled (CR1 PE), ClockSpeed =
 *       100 kHz standard mode, PB6/PB7 idle HIGH (the bus electricals,
 *       now CubeMX-owned -- read-only probes)
 *   2 = 7-bit address scan 0x50..0x57 via HAL_I2C_IsDeviceReady
 *       (refines the device address), then write 0x55 @0x10 -> read
 *       back -> compare (a NACK here is "device absent or busy",
 *       reported -- the 24C02 is soldered onto this board and is
 *       EXPECTED to answer)
 *   3 = write 0xAA @0x11 -> read back -> compare
 *   4 = bus idle: after the last STOP both SCL and SDA read HIGH
 *       through HAL_GPIO_ReadPin (open-drain released, pull-ups win =
 *       recessive state) */
int bsp_i2c_eeprom_selftest(void);

#endif /* BSP_I2C_EEPROM_H */
