/**
 * @file bsp_spi_flash.h
 * @brief SPI1 host (HAL edition: PA5=SCK/PA6=MISO/PA7=MOSI AF5, mode 0,
 *        8-bit MSB-first, soft NSS, prescaler /16 -> 5.25 MHz) + PC13
 *        soft chip-select, driving the onboard W25Q128 16 MiB SPI NOR
 *        flash -- init verify, 8-bit full-duplex transfer, JEDEC/erase/
 *        program/read command layer, self-test.
 *
 * Teaching module (HAL edition of f407-periph-lab bsp_spi_flash). CubeMX
 * owns the peripheral: MX_SPI1_Init() configured the pins (SPI1_MspInit:
 * PA5/PA6/PA7 AF5) and SPI1's CR1 from hspi1.Init (full-duplex master,
 * mode 0, 8-bit MSB-first, NSS soft, prescaler /16 of APB2 84 MHz).
 * This module VERIFIES that configuration (read-back) and drives the
 * bus through HAL_SPI_TransmitReceive / Transmit / Receive. A3 rule: no
 * peripheral control-register writes; read-only probes carry a
 * read-only (A3 exemption) comment.
 *
 * CS ownership: FLASH_CS (PC13) is a plain GPIO output in the IOC
 * (output PP, boot HIGH = deselected; MX_GPIO_Init de-asserts CS at
 * boot). Chip select is NEVER an SPI peripheral pin -- RM0090 28.3.3
 * says so for slaves that must be de-selected between transmissions.
 * This module frames every command with HAL_GPIO_WritePin low/high
 * pairs; the HAL waits TXE/RXNE and the trailing BSY inside each
 * transfer call, so the register edition's CS-vs-BSY polling
 * discipline collapses into "CS low -> HAL frame -> CS high".
 *
 * REMOVED vs the register edition (all were register-level bench
 * diagnostics, all forbidden under the A3 rule now that CubeMX owns
 * the pins; documented here so the diff is honest):
 *   - CR1/MODER/AFRL/OSPEEDR config + assertions -- init verifies the
 *     hspi1.Init fields + CR1 read-back instead (the OSPEEDR-slew and
 *     MISO-must-be-AF board lessons, knowledge #14/#15, are now
 *     CubeMX's guarantees)
 *   - sf_miso_pull_probe (PUPDR pull-toggle test) -- kept as a
 *     read-only MISO level probe in the step-2 failure report
 *   - sf_set_spi_mode + the mode-3/slow-baud fallback engines and the
 *     GPIO bit-bang soft probe (register writes) -- step 2 keeps the
 *     0xAB deep-power-down release + retry (knowledge #13/#14) as the
 *     only fallback, plus raw-ID + MISO-level evidence in the report
 *   - bsp_spi_flash_scope_demo / bsp_spi_flash_pin_probe_demo (infinite
 *     bench loops that re-muxed pins by hand)
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

/* Verify the CubeMX-generated configuration (read-back): hspi1.Init
 * matches the documented mode-0/8-bit/MSB/soft-NSS//16 shape, CR1 has
 * MSTR|SSM|SSI, the CS pin is driven idle HIGH, then send the deep-
 * power-down release preamble (0xAB -- see the walkthrough; a part
 * left in DPD answers ONLY that command). Returns 0 = PASS, -1 =
 * mismatch. */
int bsp_spi_flash_init(void);

/* One 8-bit FULL-DUPLEX exchange via HAL_SPI_TransmitReceive: write
 * 'tx', get back what arrived on MISO during the same 8 clocks. (SPI
 * has no "send only": every transmit shifts a receive in -- dummy
 * 0x00/0xFF bytes are how reads are clocked out of the device.) The
 * HAL waits TXE/RXNE and the trailing BSY, so the returned byte is a
 * fully shifted frame. Returns 0..255 (0 also on HAL timeout -- the
 * command-level result carries the failure). */
uint8_t bsp_spi_flash_xfer8(uint8_t tx);

/* W25Q128 command layer (command shapes in the walkthrough):
 * jedec_id  -> 0x9F, three ID bytes into id[0..2] (EF 40 18 expected).
 *   Returns 0 = read, -1 = bad args (id == 0).
 * read      -> 0x03 read 'len' bytes from 'addr' into buf (len >= 1,
 *   len <= 65535 -- the HAL's 16-bit Size cap). Returns 0 = read,
 *   -1 = bad args, -2 = HAL transfer failed.
 * page_program -> 0x02 program 'len' (1..256) bytes from buf at 'addr'.
 *   'addr' must be page-aligned and the WHOLE range inside ONE page
 *   (a write crossing the page boundary would WRAP to the page start
 *   and corrupt the first bytes -- the API refuses instead). Refuses
 *   any address >= BSP_SPI_FLASH_FREE_LIMIT (font red line).
 *   Returns 0 = done (busy-wait finished), -1 = bad args, -3 = font-
 *   region violation (bus untouched), -2 = busy timeout / HAL error.
 * sector_erase4k -> 0x20 4 KiB sector erase at the containing sector of
 *   'addr' (must be 4 KiB aligned). Font red line as above.
 *   Returns 0 = done, -1 = bad args, -3 = font-region violation,
 *   -2 = busy timeout / HAL error. */
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
 *   1 = init + config read-back: hspi1.Init master/mode-0/8-bit/MSB/
 *       soft-NSS//16, CR1 MSTR|SSM|SSI, CS idle HIGH
 *   2 = JEDEC ID via 0x9F must be EF 40 18 (Winbond, 128 Mbit) -- the
 *       soldered W25Q128 is EXPECTED to answer; on a silent/garbage
 *       answer ONE software 0xAB release frame is sent (knowledge
 *       #13/#14: a deep-power-down part answers ONLY 0xAB) and the ID
 *       retried; the raw IDs + the MISO level go into the report
 *   3 = free-area page exercise at 0x00010000 (well below the 10 MiB
 *       red line): 4 KiB sector erase -> read back 256 B all 0xFF ->
 *       program 16 B -> read back -> compare, byte for byte */
int bsp_spi_flash_selftest(void);

#endif /* BSP_SPI_FLASH_H */
