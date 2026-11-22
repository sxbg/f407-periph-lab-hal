/**
 * @file bsp_spi_flash.h
 * @brief SPI1 register-level host (PA5=SCK/PA6=MISO/PA7=MOSI) + PC13 soft
 *        chip-select, driving the onboard W25Q128 16 MiB SPI NOR flash --
 *        init, 8-bit full-duplex transfer, JEDEC/erase/program/read
 *        command layer, self-test.
 *
 * Teaching module (P3d wave, bsp-module-spec.md template -- see bsp_uart).
 * Full CubeMX config table / register walkthrough (including the CS-vs-BSY
 * timing discipline and the 256-byte page constraint) / verification recipe
 * live in the bsp_spi_flash.c file header.
 *
 * Ownership rule: SPI1 is NOT in the IOC (knowledge #13: adapter-owned
 * peripherals stay out) -- PA5/PA6/PA7 sit there as GPIO_Analog placeholders
 * (labels FLASH_SCK/MISO/MOSI). PC13 IS in the IOC as GPIO_Output high
 * (label FLASH_CS, so MX_GPIO_Init de-asserts CS at boot); from this
 * module's init on, the pin belongs to THIS module (BSRR driven). This
 * module is the FULL owner of RCC->APB2ENR SPI1EN, the PA5/6/7 + PC13 pin
 * config and SPI1's CR1/CR2/SR/DR. No CubeMX code will ever conflict.
 *
 * PARTITION RED LINE (board manual): 0x00000000..0x00A00000-1 = free
 * storage area, 0x00A00000..0x00FFFFFF = FONTS area (factory-burned
 * glyph libraries) -- ERASE AND PROGRAM ARE REFUSED above 0x00A00000.
 */
#ifndef BSP_SPI_FLASH_H
#define BSP_SPI_FLASH_H

#include <stdint.h>

/* W25Q128 geometry and the write-protect boundary. */
#define BSP_SPI_FLASH_TOTAL_SIZE 0x01000000u /* 16 MiB (128 Mbit) */
#define BSP_SPI_FLASH_PAGE_SIZE 256u   /* page program granularity */
#define BSP_SPI_FLASH_SECTOR_SIZE 4096u /* 0x20 sector erase granularity */
/* Address >= this = font region: program/erase REFUSED (hard limit). */
#define BSP_SPI_FLASH_FREE_LIMIT 0x00A00000u

/* Register-level init of SPI1 as a mode-0 8-bit host at 5.25 MHz
 * (APB2 84 MHz / BR=16): clock gate, PA5/PA6/PA7 -> AF5, PC13 -> output
 * high (CS idle de-asserted), CR1 MSTR/SSM/SSI/DFF=8/CPOL=0/CPHA=0/BR=16,
 * SPE=1 last. Returns 0 = PASS (CR1/SR read-back), -1 = mismatch. */
int bsp_spi_flash_init(void);

/* One 8-bit FULL-DUPLEX transfer: write 'tx' into DR, then wait RXNE and
 * return what arrived on MISO during the same 8 clocks. (SPI has no
 * "send only": every transmit shifts a receive in -- dummy 0x00/0xFF
 * bytes are how reads are clocked out of the device.) Returns 0..255. */
uint8_t bsp_spi_flash_xfer8(uint8_t tx);

/* W25Q128 command layer (command/argument shapes in the walkthrough):
 * jedec_id  -> 0x9F, three ID bytes into id[0..2] (EF 40 18 expected).
 *   Returns 0 = read, -1 = timeout.
 * read      -> 0x03 read 'len' bytes from 'addr' into buf (len >= 1).
 *   Returns 0 = read, -1 = bad args (0 len / out of 16 MiB).
 * page_program -> 0x02 program 'len' (1..256) bytes from buf at 'addr'.
 *   'addr' must be page-aligned and the WHOLE range inside ONE page
 *   (a write crossing the page boundary would WRAP to the page start
 *   and corrupt the first bytes -- the API refuses instead). Refuses
 *   any address >= BSP_SPI_FLASH_FREE_LIMIT (font red line).
 *   Returns 0 = done (busy-wait finished), -1 = bad args, -3 = font-
 *   region violation (bus untouched), -2 = busy timeout.
 * sector_erase4k -> 0x20 4 KiB sector erase at the containing sector of
 *   'addr' (must be 4 KiB aligned). Font red line as above.
 *   Returns 0 = done, -1 = bad args, -3 = font-region violation,
 *   -2 = busy timeout. */
int bsp_spi_flash_jedec_id(uint8_t id[3]);
int bsp_spi_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
int bsp_spi_flash_page_program(uint32_t addr, const uint8_t *buf,
                               uint32_t len);
int bsp_spi_flash_sector_erase4k(uint32_t addr);

/* One-line report of the LAST selftest run, for the menu to append to
 * its verdict line. Examples: "(id EF 40 18)" / "(id FF FF FF: dead?)". */
const char *bsp_spi_flash_last_report(void);

/* Self-test. Returns 0 = PASS, otherwise the 1-based step number that
 * failed (bsp_spi_flash_last_report carries the "why"). Steps:
 *   1 = init + config read-back: CR1 has MSTR|SPE|SSM|SSI, BR=/16,
 *       CPOL=0/CPHA=0 (mode 0), DFF=8-bit
 *   2 = JEDEC ID via 0x9F must be EF 40 18 (Winbond, 128 Mbit) -- the
 *       soldered W25Q128 is EXPECTED to answer; FF FF FF means the
 *       part/driver is dead, reported via last_report
 *   3 = free-area page exercise at 0x00010000 (well below the 10 MiB
 *       red line): 4 KiB sector erase -> read back 256 B all 0xFF ->
 *       program 16 B -> read back -> compare, byte for byte */
int bsp_spi_flash_selftest(void);

#endif /* BSP_SPI_FLASH_H */

/* Continuous JEDEC frames at 100 ms period for scope observation
 * (CS=PC13, SCK=PA5, MOSI=PA7). Never returns -- exit by NRST. */
void bsp_spi_flash_scope_demo(void);

/* Pin-probe demo: PA5/PA7 GPIO 1 kHz (5 s) alternating with slow SPI
 * JEDEC frames (2 s), forever -- separates "pin dead" from "SPI dead".
 * Never returns -- exit by NRST. */
void bsp_spi_flash_pin_probe_demo(void);
