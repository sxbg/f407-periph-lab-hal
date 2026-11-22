/**
 * @file i2c_soft_bus.h
 * @brief Generic bit-banged open-drain I2C master -- the bus engine behind
 *        bsp_i2c_eeprom (and reusable by any future I2C device module).
 *
 * PORT LINEAGE: a 1:1 port of the Radio project's proven soft_i2c.c
 * (projects/Radio/firmware/App/adapters/soft_i2c/soft_i2c.c) -- the engine
 * that drives the Radio OLED/SI4703 buses on real hardware. Radio timing,
 * edge order and ACK sampling are copied VERBATIM (board finding 2026-11-21:
 * the F4 hardware I2C engine stalls non-deterministically; the soft engine
 * on the same silicon does not). This project renders Radio's HAL calls in
 * register form, exactly as bsp_oled.c already does for its PD9/PE15 bus.
 *
 * Electrical contract (open-drain, wired-AND): a pin is configured as GPIO
 * OUTPUT + OPEN-DRAIN + PULL-UP. Writing 1 to BSRR releases the line (the
 * pull-up wins -> HIGH); writing 0 drives it LOW. IDR reads the REAL line
 * level (pull-up and every attached device included), not our ODR intent --
 * the START check and the ACK slot are built on that truth.
 *
 * Ownership rule (caller contract): the caller must have the GPIO port
 * clock running (RCC->AHB1ENR) BEFORE i2c_soft_bus_init(); the engine is
 * clock-agnostic by design (Radio's HAL_GPIO_Init does not arm clocks
 * either -- the board layer owns RCC).
 */
#ifndef I2C_SOFT_BUS_H
#define I2C_SOFT_BUS_H

#include <stdbool.h>
#include <stdint.h>

#include "hal_port.h" /* family seam: GPIO_TypeDef comes from the CMSIS tree */

/* One bus instance: two pins + the derived half-bit period. Small enough
 * to live as a static in each device module (no dynamic allocation). */
typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t      scl_pin;     /**< line mask, e.g. 0x0040u = PB6        */
    GPIO_TypeDef *sda_port;
    uint16_t      sda_pin;     /**< line mask, e.g. 0x0080u = PB7        */
    uint32_t      half_cycles; /**< half bit period in CPU cycles (DWT)  */
} i2c_soft_bus_t;

/* Configure both pins open-drain with pull-ups, derive the half-bit delay
 * from SystemCoreClock / (khz * 2000) (Radio's own formula; 100 kHz at
 * 168 MHz = 840 cycles = 5.0 us) and release both lines to idle-HIGH.
 * khz must be non-zero (call is a no-op otherwise, like Radio's). */
void i2c_soft_bus_init(i2c_soft_bus_t *bus,
                       GPIO_TypeDef *scl_port, uint16_t scl_pin,
                       GPIO_TypeDef *sda_port, uint16_t sda_pin,
                       uint32_t khz);

/* One transaction: START, (addr7 << 1)|W, 'len' data bytes, STOP. Every
 * byte's ACK is sampled; a NACK stops the payload and the STOP still goes
 * out (the bus is never left mid-transaction). Returns true = all bytes
 * ACKed. Returns false = START refused (a line is held LOW -- short,
 * swapped wires, wedged target) or any NACK on the way. */
bool i2c_soft_bus_write(i2c_soft_bus_t *bus, uint8_t addr7,
                        const uint8_t *data, uint16_t len);

/* One transaction: START, (addr7 << 1)|R, 'len' received bytes (ACK on all
 * but the LAST byte, which the master NACKs per the I2C spec), STOP.
 * Returns true = the address was ACKed; the data bytes themselves have no
 * ACK from the master's side. */
bool i2c_soft_bus_read(i2c_soft_bus_t *bus, uint8_t addr7,
                       uint8_t *data, uint16_t len);

/* Address-only transaction (START, addr7 << 1 | W, STOP): returns true
 * when a target ACKed the address. This is BOTH the "device present"
 * detector and the EEPROM-standard tWR write-cycle poll. */
bool i2c_soft_bus_probe(i2c_soft_bus_t *bus, uint8_t addr7);

/* Classic I2C unwedge: a target caught mid-byte keeps SDA low and blocks
 * every START; 9 SCL pulses clock the stuck byte out, then a STOP returns
 * the bus to idle (Radio I2C_RECOVER_PULSES = 9). Bounded by construction:
 * 9 x two half periods. */
void i2c_soft_bus_recover(i2c_soft_bus_t *bus);

/* Millisecond pacing delay on the SysTick COUNTFLAG (1 ms tick), guarded
 * for a dead tick (bounded exit -- the house pattern of the other bsp
 * modules). Used for ms-scale windows (EEPROM tWR ack-poll pacing); the
 * WIRE-LEVEL half-bit delay is DWT-timed instead -- SysTick's COUNTFLAG
 * granularity is 1 ms, four hundred bit periods, useless at the wire
 * (the finding bsp_oled.c records). Radio's soft_i2c_delay_ms is likewise
 * a free function -- pacing does not need the bus object. */
void i2c_soft_bus_delay_ms(uint32_t ms);

#endif /* I2C_SOFT_BUS_H */
