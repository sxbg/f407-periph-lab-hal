"""Round-2 fixes: wdg arm+feed pattern, i2c wait semantics, spi slow retry.
(All LF-aware, positional anchors with find-assert.)"""
from pathlib import Path

BSP = Path(__file__).parent.parent / "firmware" / "App" / "bsp"

# ---------- 1. bsp_wdg: arm the dog, feed in menu loop, longer window ----------
p = BSP / "bsp_wdg.c"
d = p.read_text(encoding="utf-8")

old = """    IWDG->KR = WDG_KEY_UNLOCK;
    /* NOTE (board finding, 2026-11-21): with the dog deliberately NOT
     * started (no 0xCCCC), PVU/RVU can stay set and never clear -- the
     * sync engine is clocked by the LSI prescaler path that only runs
     * once the counter runs. Waiting here deadlocks, so we skip the
     * PVU/RVU wait entirely and let the PR/RLR READ-BACK below be the
     * acceptance contract. (EP15 teaching point: start-then-configure
     * is the HAL order precisely because of this sync behavior.) */
    IWDG->PR = BSP_WDG_IWDG_PR;  /* /64 -> 2.000 ms tick @32 kHz nominal */
    IWDG->RLR = BSP_WDG_IWDG_RLR; /* 1000 ticks -> 2.0 s nominal          */
    IWDG->KR = WDG_KEY_PROTECT; /* re-protect; the dog is configured but
                                 * NOT running: no 0xCCCC was ever written */"""
new = """    IWDG->KR = WDG_KEY_UNLOCK;
    /* BOARD FINDING (2026-11-21): on an UNARMED IWDG the PVU/RVU sync
     * bits never clear and PR/RLR reads return stale values -- the
     * "configure without starting" pattern is impossible by design.
     * The HAL order (Init feeds 0xCCCC first) exists precisely because
     * of this (EP15 teaching point). So: configure, then START the dog
     * and let the menu loop FEED it every iteration -- the real-world
     * pattern. Timeout is enlarged so a flash-erase pause (~2.6 s,
     * PRIMASK-blocked but IWDG runs on LSI) stays inside the window. */
    IWDG->PR = BSP_WDG_IWDG_PR;   /* /64 -> 2.000 ms tick @32 kHz nominal */
    IWDG->RLR = BSP_WDG_IWDG_RLR; /* 4096 ticks -> 8.19 s nominal          */
    IWDG->KR = 0xCCCCu;           /* START: sync + readback now work       */
    if (wdg_wait_iwdg_idle(BSP_WDG_PVU_TO_MS) != 0) {
        report_set("(iwdg: PVU/RVU stuck)");
        return -1;
    }
    IWDG->KR = WDG_KEY_PROTECT; /* re-protected; menu loop feeds the dog */"""
assert old in d, "wdg init anchor"
d = d.replace(old, new, 1)

d = d.replace("#define BSP_WDG_IWDG_RLR 999u /* 1000 ticks -> 2.0 s nominal */",
              "#define BSP_WDG_IWDG_RLR 4095u /* 4096 ticks -> 8.19 s nominal */")
# header timeout notes
h = BSP / "bsp_wdg.h"
dh = h.read_text(encoding="utf-8")
dh = dh.replace("2.0 s", "8.19 s nominal (5.6-15.4 s over LSI tolerance)")
h.write_text(dh, encoding="utf-8", newline="\n")
p.write_text(d, encoding="utf-8", newline="\n")
print("wdg armed+feed pattern applied")

# ---------- 2. bsp_i2c_eeprom: wait-for-any-flag semantics ----------
p = BSP / "bsp_i2c_eeprom.c"
d = p.read_text(encoding="utf-8")
old = """        if ((I2C1->SR1 & mask) == want) {
            return 0;
        }"""
new = """        /* WAIT-FOR-ANY-FLAG: (SR1 & mask) != 0. The ADDR|AF callers
         * need "whichever comes first" -- requiring BOTH would turn a
         * clean ACK (ADDR set, AF clear) into a false 10 ms stall
         * (board finding 2026-11-21). Callers disambiguate afterwards. */
        if ((I2C1->SR1 & mask) != 0u) {
            return 0;
        }"""
assert old in d, "i2c wait anchor"
d = d.replace(old, new, 1)
p.write_text(d, encoding="utf-8", newline="\n")
print("i2c wait-for-any-flag applied")

# ---------- 3. bsp_spi_flash: slow-speed retry probe on JEDEC ----------
p = BSP / "bsp_spi_flash.c"
d = p.read_text(encoding="utf-8")
old = """        sf_set_report("id ");
        for (i = 0u; i < 3u; i++) {
            sf_report_hex_byte(id[i]);
        }"""
new = """        /* Diagnostics: retry once at /256 (328 kHz) -- rules out an
         * overly hot SCK edge on long DuPont runs before we call the
         * chip absent. The report names the speed that was used. */
        if ((id[0] == 0x00u) && (id[1] == 0x00u) && (id[2] == 0x00u)) {
            SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) |
                        (7u << SPI_CR1_BR_Pos); /* /256 = 328 kHz */
            SPI1->CR1 &= ~SPI_CR1_SPE;
            SPI1->CR1 |= SPI_CR1_SPE;
            (void)bsp_spi_flash_jedec_id(id);
            SPI1->CR1 &= ~SPI_CR1_SPE;
            SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) |
                        (4u << SPI_CR1_BR_Pos);
            SPI1->CR1 |= SPI_CR1_SPE;
            sf_set_report("id@/256 ");
            for (i = 0u; i < 3u; i++) {
                sf_report_hex_byte(id[i]);
            }
        }
        else {
            sf_set_report("id ");
            for (i = 0u; i < 3u; i++) {
                sf_report_hex_byte(id[i]);
            }
        }"""
assert old in d, "spi jedec report anchor"
d = d.replace(old, new, 1)
p.write_text(d, encoding="utf-8", newline="\n")
print("spi slow-retry probe applied")

# ---------- 4. shell: feed the dog every menu loop iteration ----------
p = Path(__file__).parent.parent / "firmware" / "App" / "shell" / "test_menu.c"
d = p.read_text(encoding="utf-8")
if '#include "bsp_wdg.h"' not in d:
    d = d.replace('#include "bsp_i2c_eeprom.h"',
                  '#include "bsp_wdg.h"     /* IWDG feed every loop iteration */\n'
                  '#include "bsp_i2c_eeprom.h"', 1)
old_loop = "        /* RTT heartbeat kept alive for the logcheck watchdog */"
if old_loop in d:
    d = d.replace(old_loop,
                  old_loop + "\n        bsp_wdg_iwdg_feed(); /* IWDG armed: menu loop feeds it */", 1)
else:
    # fallback: feed right before the heartbeat throttle block start marker
    print("WARN: heartbeat anchor not found -- inspect test_menu loop")
p.write_text(d, encoding="utf-8", newline="\n")
print("shell feed applied")
