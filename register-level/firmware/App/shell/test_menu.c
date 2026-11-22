/**
 * @file test_menu.c
 * @brief UART command menu: registry-driven selftest runner on USART1.
 *
 * Design (bsp-module-spec.md "自测试约定"):
 *  - Registry-driven: adding a peripheral = one line in s_module_tests.
 *    Menu numbering stays stable; built-ins (help/reboot) are separate.
 *  - Status semantics match the WorkManger verdicts: PASS <name> /
 *    FAIL <name> step<N> <detail> (the module prints its own detail).
 *  - The loop never returns: RX polling is the main loop, with a throttled
 *    LED heartbeat and the RTT #H heartbeat kept alive for logcheck.
 */
#include "test_menu.h"

#include <stddef.h>

#include "hal_port.h"    /* CMSIS: NVIC_SystemReset */
#include "ports.h"       /* time_port_t (tick_ms) */
#include "adapters.h"    /* adapter_time / adapter_digout (BSRR toggle) */
#include "bsp_uart.h"    /* console I/O + module selftests */
#include "bsp_gpio.h"    /* P3a: LEDs / keys / backlight selftest */
#include "bsp_tim_pwm.h" /* P3a: TIM1/TIM4 1 kHz PWM selftest */
#include "bsp_tim_basic.h" /* P3a: TIM5/6/7 timebase selftest */
#include "bsp_tim_ic.h"  /* P3a: TIM2 input-capture selftest + report */
#include "bsp_tim_encoder.h" /* P3b: TIM3 quadrature encoder + report */
#include "bsp_tim_adv.h" /* P3b: TIM8 complementary PWM + dead time */
#include "bsp_can_loopback.h" /* P3b: CAN1 500k loop-back + report */
#include "bsp_crc_rng.h" /* P3b: CRC32 unit + RNG selftest */
#include "bsp_rtc.h"     /* P3c: RTC LSE calendar + backup registers */
#include "bsp_wdg.h"     /* P3c: IWDG/WWDG config + reset-source flags */
#include "bsp_pwr.h"     /* P3c: Sleep/Stop/Standby (Stop = demo only) */
#include "bsp_flash_iap.h" /* P3c: flash IAP last-sector param page */
#include "bsp_i2c_eeprom.h" /* P3d: I2C1 host + onboard 24C02 */
#include "bsp_spi_flash.h" /* P3d: SPI1 host + onboard W25Q128 */
#include "bsp_sdio_tf.h"   /* P3d: SDIO TF card identification */
#include "bsp_adc.h"     /* P3e: ADC1 SWSTART/EOC poll + Vrefint */
#include "bsp_dac.h"     /* P3e: DAC1 OUT1 software-mode setpoint */
#include "bsp_dma.h"     /* P3e: DMA2 S6 mem2mem + S7 UART TX */
#include "bsp_exti.h"    /* P3e: EXTI1 <- KEY2 press counter */
#include "bsp_oled.h"    /* P3f: SH1107 OLED soft-I2C PD9/PE15 */
#include "rtt_log.h"     /* rtt_heartbeat + rtt_mark (logcheck liveness) */
#include "app_config.h"  /* heartbeat LED port/pin + periods */

/* ---- registry -------------------------------------------------------------- */

typedef int (*bsp_selftest_fn)(void); /* returns 0 = PASS, else step number */
typedef const char *(*bsp_detail_fn)(void); /* verdict-line suffix, or NULL */

typedef struct {
    const char *name;     /* command word typed at the menu */
    bsp_selftest_fn run;  /* the module selftest */
    const char *desc;     /* one-line description for help */
    bsp_detail_fn detail; /* optional: last-run report appended to PASS;
                           * modules that return data instead of printing
                           * use it (bsp_tim_ic_last_report pattern) --
                           * keeps bsp modules free of console includes */
} menu_test_t;

/* Shell-layer CLOSED-LOOP commands (P3e, jumper-wire cross-module proofs).
 * They compose two bsp modules through their public APIs ONLY -- the shell
 * is where cross-module wiring lives (bsp-module-spec rule #7), and they
 * follow the registry calling convention: 0 = PASS, non-zero = FAIL step.
 * Defined below (after the tiny print/wait helpers they share). */
static int cmd_dac_adc_loop(void);
static int cmd_pwm_ic_loop(void);

/* P3 DL-A5 pilot (uart) + P3a wave (gpio / tim_pwm / tim_basic / tim_ic).
 * Later peripherals append here -- the spec keeps menu numbering stable,
 * so new entries go at the END. */
static const menu_test_t s_module_tests[] = {
    { "uart", bsp_uart_selftest,
      "USART1 115200 8N1 (BRR readback, TC TX, RX timeout)", NULL },
    { "gpio", bsp_gpio_selftest,
      "PC5/PB2 LEDs, PA0/PA1 keys, PA15 BL (MODER/BSRR/IDR)", NULL },
    { "tim_pwm", bsp_tim_pwm_selftest,
      "TIM1_CH1=PA8 + TIM4_CH1=PD12 1 kHz PWM (PSC/ARR/CCR/MOE)", NULL },
    { "tim_basic", bsp_tim_basic_selftest,
      "TIM5 1s IRQ + TIM6/TIM7 poll timebases (PSC/ARR/UIF)", NULL },
    { "tim_ic", bsp_tim_ic_selftest,
      "TIM2_CH2=PB3 input capture freq meter (32-bit, 1s window)",
      bsp_tim_ic_last_report },
    { "tim_enc", bsp_tim_encoder_selftest,
      "TIM3 enc PB4/PB5 x4 (SMS=011, CNT wrap, rpm vs PPR)",
      bsp_tim_encoder_last_report },
    { "tim_adv", bsp_tim_adv_selftest,
      "TIM8 CH1=PC6 + CH2=PC7/CH2N=PB14 comp PWM 1us deadtime (DTG/MOE)",
      NULL },
    { "can_lb", bsp_can_loopback_selftest,
      "CAN1 PB8/PB9 500k loopback (INAK, TX->RX frame, TSR flags)",
      bsp_can_loopback_last_report },
    { "crc_rng", bsp_crc_rng_selftest,
      "CRC32 hw (MPEG-2 vectors) + RNG (DRDY/CECS, 8 reads)",
      NULL },
    { "rtc", bsp_rtc_selftest,
      "RTC LSE calendar 1Hz + alarm A + 20x BKPxR (INITF/RSF/BCD)",
      bsp_rtc_last_report },
    { "wdg", bsp_wdg_selftest,
      "IWDG 2s cfg readback + WWDG window + CSR reset flags (RMVF)",
      bsp_wdg_last_report },
    { "pwr", bsp_pwr_selftest,
      "PWR gates + Sleep 100ms via SysTick wake (Stop/Standby = demos)",
      bsp_pwr_last_report },
    { "flash_iap", bsp_flash_iap_selftest,
      "FLASH keys + erase sector7/program param page 0x0807FC00 (~2s)",
      bsp_flash_iap_last_report },
    { "i2c_ee", bsp_i2c_eeprom_selftest,
      "I2C1 100k PB6/PB7 + 24C02@0x50 (SB/ADDR/EV, 0x55/0xAA roundtrip)",
      bsp_i2c_eeprom_last_report },
    { "spi_flash", bsp_spi_flash_selftest,
      "SPI1 mode0 5.25MHz + W25Q128 (JEDEC EF4018, erase/pgm page, CS-BSY)",
      bsp_spi_flash_last_report },
    { "sdio_tf", bsp_sdio_tf_selftest,
      "SDIO CMD0/8/41/2/3/9 identify 400k (OCR CCS, CID mfr, CSD size)",
      bsp_sdio_tf_last_report },
    { "adc", bsp_adc_selftest,
      "ADC1 SWSTART/EOC poll PB0/PC0/PC4+Vrefint (SMPR/SQR/CR2, 480cyc)",
      bsp_adc_last_report },
    { "dac", bsp_dac_selftest,
      "DAC1 OUT1=PA4 software mode (DHR12R1->DOR 0/1650/3300mV roundtrip)",
      NULL },
    { "dma", bsp_dma_selftest,
      "DMA2 S6 mem2mem 64 words + S7/Ch4 USART1_TX line (TCIF/NDTR/DMAT)",
      NULL },
    { "exti", bsp_exti_selftest,
      "EXTI1=KEY2 PA1 falling, interactive x3 in 6s (EXTICR/IMR/FTSR/PR)",
      bsp_exti_last_report },
    { "dac_adc_loop", cmd_dac_adc_loop,
      "WIRE: PA4-PB0 loop: DAC 500/1650/2800mV -> ADC, |delta|<=60mV x3",
      NULL },
    { "pwm_ic_loop", cmd_pwm_ic_loop,
      "WIRE: PA8-PB3 loop: TIM1 1kHz 50% PWM -> TIM2 IC, 950..1050 Hz",
      NULL },
    { "oled", bsp_oled_selftest,
      "SH1107 128x64 soft-I2C PD9/PE15 (EP12)",
      bsp_oled_last_report },
};

#define MENU_TESTS_N ((int)(sizeof(s_module_tests) / sizeof(s_module_tests[0])))

/* ---- demo commands (NOT selftests) ----------------------------------------- */

/* Minimal decimal printer for demo lines (same hand-rolled formatter
 * approach as the FAIL-step printer in menu_exec; no libc in the shell). */
static void menu_put_uint(uint32_t v)
{
    char digits[10];
    unsigned m = 0u;
    do {
        digits[m++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v > 0u);
    while (m > 0u) {
        char one[2];
        one[0] = digits[--m];
        one[1] = '\0';
        bsp_uart_puts(one);
    }
}

/* Bounded millisecond wait on the menu's time adapter (SysTick-backed
 * tick_ms). The guard bounds the case of a dead tick source; without it
 * a frozen tick_ms would spin here forever (spec #5 discipline). */
static void loop_wait_ms(uint32_t ms)
{
    const time_port_t *time = adapter_time();
    const uint32_t start = time->tick_ms();
    uint32_t guard = 2000000u;
    while ((time->tick_ms() - start) < ms) {
        if (--guard == 0u) {
            return; /* tick dead: bounded exit */
        }
    }
}

/* ---- P3e closed loops (jumper-wire cross-module proofs) -------------------- */

/* dac_adc_loop: DAC1 (PA4) -> dupont jumper -> ADC1_IN8 (PB0). The analog
 * closed loop EP08/EP09 promise: what the DAC SETS, the ADC MEASURES.
 * Three setpoints span low/mid/high of the buffered DAC range; both
 * peripherals share VDDA, so the reference cancels and the only residual
 * is quantization (0.8 mV LSB) + noise + INL + the buffer's rail margin
 * -- 60 mV of tolerance is generous for that budget. Steps: 1 = config,
 * 2..4 = the three rows. */
static int cmd_dac_adc_loop(void)
{
    static const uint32_t setpoints[3] = { 500u, 1650u, 2800u };
    int step;

    bsp_uart_puts("[dac_adc_loop] WIRE: PA4 -> PB0 (DAC_OUT1 -> ADC1_IN8)\r\n");
    bsp_uart_puts("[dac_adc_loop] connect the dupont jumper first; no wire"
                  " = every row fails\r\n");
    if ((bsp_dac_init() != 0) || (bsp_adc_init() != 0)) {
        return 1; /* step 1: module config */
    }
    for (step = 0; step < 3; step++) {
        uint32_t delta;
        bsp_dac_set_mv(setpoints[step]);
        loop_wait_ms(5u); /* DAC buffer settling + ADC sample margin */
        {
            /* |read - set| without signed math. */
            const uint32_t read_mv = bsp_adc_read_mv(BSP_ADC_CH_IN8_PB0);
            delta = (read_mv >= setpoints[step])
                        ? (read_mv - setpoints[step])
                        : (setpoints[step] - read_mv);
            bsp_uart_puts("  set ");
            menu_put_uint(setpoints[step]);
            bsp_uart_puts(" mV -> read ");
            menu_put_uint(read_mv);
            bsp_uart_puts(" mV (delta ");
            menu_put_uint(delta);
            bsp_uart_puts(delta <= 60u ? " mV ok)\r\n" : " mV FAIL)\r\n");
        }
        if (delta > 60u) {
            return 2 + step; /* steps 2..4 = rows 1..3 */
        }
    }
    return 0;
}

/* pwm_ic_loop: TIM1_CH1 (PA8) -> dupont jumper -> TIM2_CH2 (PB3). The
 * EP06+EP13 pair: what the PWM OUTPUTS, the capture meter MEASURES.
 * bsp_tim_pwm_init sets 1 kHz 50% (advanced-timer MOE included); the IC
 * meter needs no duty -- two rising edges are a period. The 950..1050
 * window covers the 1 MHz-tick PSC/ARR rounding and the 12 ns capture
 * resolution with room for board clock spread. Steps: 1 = config,
 * 2 = the measurement. */
static int cmd_pwm_ic_loop(void)
{
    bsp_uart_puts("[pwm_ic_loop] WIRE: PA8 -> PB3 (TIM1_CH1 -> TIM2_CH2)\r\n");
    bsp_uart_puts("[pwm_ic_loop] connect the dupont jumper first; no wire"
                  " = 0 Hz = fail\r\n");
    if ((bsp_tim_pwm_init() != 0) || (bsp_tim_ic_init() != 0)) {
        return 1; /* step 1: module config */
    }
    {
        const uint32_t hz = bsp_tim_ic_measure_freq();
        bsp_uart_puts("  measured ");
        menu_put_uint(hz);
        bsp_uart_puts(" Hz (target 1000, window 950..1050)\r\n");
        if ((hz < 950u) || (hz > 1050u)) {
            return 2;
        }
    }
    return 0;
}

/* Danger-capable commands live OUTSIDE s_module_tests on purpose: a
 * selftest must be safe to re-run (spec), while a demo deliberately
 * resets or parks the board. Each entry carries its own warning text,
 * printed BEFORE run() gets a chance to act (bsp_uart_puts TC-drains, so
 * the warning is on the wire before the reset hits). */
typedef void (*menu_demo_fn)(void);

typedef struct {
    const char *name; /* command word typed at the menu */
    const char *desc; /* help line WITH the warning */
    menu_demo_fn run;
} menu_demo_t;

/* Shell-layer composition (bsp-module-spec rule #7: cross-module wiring
 * lives HERE, the bsp modules stay mutually independent): before arming
 * the IWDG, write a "who asked for this reset" marker into RTC backup
 * register 1 (BKP domain survives everything short of a battery pull --
 * EP05/EP15 reset-source teaching). */
static void demo_wdg_reset(void)
{
    (void)bsp_rtc_write_bkp(1u, 0x57444731u); /* "WDG1" ASCII */
    bsp_uart_puts("DEMO: IWDG armed, ~2.0s nominal (1.4-3.8s across the\r\n");
    bsp_uart_puts("DEMO: LSI 17-47kHz spread). NO feed -> reset in seconds.\r\n");
    bsp_uart_puts("DEMO: BKP1=0x57444731 marker written (survives reset).\r\n");
    bsp_uart_puts("DEMO: after reboot: 'wdg' shows flag mechanics, RTT log\r\n");
    bsp_uart_puts("DEMO: carries EV_WATCHDOG_RESET, 'rtc' keeps BKP0/1.\r\n");
    bsp_wdg_reset_demo(); /* NEVER RETURNS */
}

static void demo_spi_probe(void)
{
    bsp_uart_puts("DEMO: PA5/PA7 GPIO 1kHz square 5 s (probe check)\r\n");
    bsp_uart_puts("DEMO: then slow SPI JEDEC frames 2 s, alternating.\r\n");
    bsp_uart_puts("DEMO: scope PA5 -- both phases must be visible.\r\n");
    bsp_spi_flash_pin_probe_demo(); /* NEVER RETURNS */
}

static void demo_spi_scope(void)
{
    bsp_uart_puts("DEMO: HW 9F + soft AB/9F/RDSR1 frames @100 ms -- scope\r\n");
    bsp_uart_puts("DEMO: CH1=PC13(CS) CH2=PA5(SCK) CH3=PA7(MOSI) CH4=PA6(MISO)\r\n");
    bsp_uart_puts("DEMO: runs forever; NRST exits.\r\n");
    bsp_spi_flash_scope_demo(); /* NEVER RETURNS */
}

static void demo_pwr_stop(void)
{
    bsp_uart_puts("DEMO: entering Stop (LPDS regulator, PLL off). Press\r\n");
    bsp_uart_puts("DEMO: KEY1=PA0 to wake (rising edge = press). While\r\n");
    bsp_uart_puts("DEMO: asleep the RTT heartbeat STOPS (logcheck will\r\n");
    bsp_uart_puts("DEMO: call the MCU hung -- expected); if KEY1 fails the\r\n");
    bsp_uart_puts("DEMO: rescue is an SWD reset. Understand before running.\r\n");
    bsp_pwr_enter_stop(); /* sleeps until KEY1; re-runs SystemInit inside */
    bsp_uart_puts("DEMO: woke via EXTI0, wake count ");
    menu_put_uint(bsp_pwr_exti0_wake_count());
    bsp_uart_puts("; SystemInit re-ran (168 MHz back).\r\n");
}

static const menu_demo_t s_menu_demos[] = {
    { "wdg_test", /* the no-feed reset-loop demo (task sheet name) */
      "DEMO resets board: arm IWDG ~2s, never feed -> reset loop",
      demo_wdg_reset },
    { "wdg_reset_demo", /* the same demo under the wave-3 name */
      "DEMO resets board: arm IWDG ~2s, print warnings, wait for reset",
      demo_wdg_reset },
    { "pwr_stop_demo",
      "DEMO enters Stop: press KEY1=PA0 to wake; heartbeat pauses",
      demo_pwr_stop },
    { "spi_scope",
      "DEMO HW 9F + soft AB/9F/RDSR1 frames @100 ms (scope 4 lines)",
      demo_spi_scope },
    { "spi_probe",
      "DEMO PA5/PA7 GPIO 1kHz (5s) then slow SPI (2s), alternates",
      demo_spi_probe },
};

#define MENU_DEMOS_N ((int)(sizeof(s_menu_demos) / sizeof(s_menu_demos[0])))

/* ---- line handling ---------------------------------------------------------- */

#define MENU_LINE_MAX 40u /* commands here are single words; small is fine */

/* Print the command table. Registry order IS the stable menu order: entries
 * are appended at the END, never reordered, so existing positions hold.
 * The demo commands print AFTER the selftests, warnings included inline. */
static void menu_print_help(void)
{
    int i;
    bsp_uart_puts("commands:\r\n");
    bsp_uart_puts("  help   - this list\r\n");
    bsp_uart_puts("  reboot - NVIC_SystemReset (re-run boot diag)\r\n");
    for (i = 0; i < MENU_TESTS_N; i++) {
        bsp_uart_puts("  ");
        bsp_uart_puts(s_module_tests[i].name);
        bsp_uart_puts("  - ");
        bsp_uart_puts(s_module_tests[i].desc);
        bsp_uart_puts("\r\n");
    }
    bsp_uart_puts("demos (NOT selftests -- they reset/park the board):\r\n");
    for (i = 0; i < MENU_DEMOS_N; i++) {
        bsp_uart_puts("  ");
        bsp_uart_puts(s_menu_demos[i].name);
        bsp_uart_puts("  - ");
        bsp_uart_puts(s_menu_demos[i].desc);
        bsp_uart_puts("\r\n");
    }
}

/* Execute one committed line: token = first whitespace-separated word. */
static void menu_exec(const char *line)
{
    char tok[MENU_LINE_MAX];
    unsigned n = 0u;

    /* In-place tokenizer (no libc strtok, no dynamic allocation): copy the
     * first word of the line into tok. */
    while (*line == ' ') {
        line++;
    }
    while ((line[n] != '\0') && (line[n] != ' ') && (n < (MENU_LINE_MAX - 1u))) {
        tok[n] = line[n];
        n++;
    }
    tok[n] = '\0';
    if (n == 0u) {
        return; /* empty line: just re-prompt */
    }

    /* Built-ins first, then the registry (order fixed by the spec). */
    if ((tok[0] == 'h') && (tok[1] == 'e') && (tok[2] == 'l') &&
        (tok[3] == 'p') && (tok[4] == '\0')) {
        menu_print_help();
        return;
    }
    if ((tok[0] == 'r') && (tok[1] == 'e') && (tok[2] == 'b') &&
        (tok[3] == 'o') && (tok[4] == 'o') && (tok[5] == 't') &&
        (tok[6] == '\0')) {
        bsp_uart_puts("rebooting...\r\n"); /* puts TC-drains: safe to reset */
        rtt_mark("MENU: NVIC_SystemReset"); /* post-mortem breadcrumb */
        NVIC_SystemReset(); /* RM0090 ch.7: AIRCR reset; never returns */
    }

    /* Demo commands next: each one prints its own warning BEFORE the
     * run() call (TC-drained by bsp_uart_puts, so the text is on the
     * wire even if the demo resets the board mid-line). wdg demos never
     * return; pwr_stop_demo returns after the KEY1 wake. */
    {
        int di;
        for (di = 0; di < MENU_DEMOS_N; di++) {
            const menu_demo_t *d = &s_menu_demos[di];
            unsigned k = 0u;
            while ((d->name[k] != '\0') && (d->name[k] == tok[k])) {
                k++;
            }
            if ((d->name[k] == '\0') && (tok[k] == '\0')) {
                bsp_uart_puts("demo> ");
                bsp_uart_puts(d->name);
                bsp_uart_puts(": ");
                bsp_uart_puts(d->desc);
                bsp_uart_puts("\r\n");
                d->run();
                return; /* pwr_stop_demo path: back to the prompt */
            }
        }
    }

    {
        int i;
        for (i = 0; i < MENU_TESTS_N; i++) {
            const menu_test_t *t = &s_module_tests[i];
            unsigned k = 0u;
            while ((t->name[k] != '\0') && (t->name[k] == tok[k])) {
                k++;
            }
            if ((t->name[k] == '\0') && (tok[k] == '\0')) {
                const int rc = t->run(); /* module prints its step details */
                if (rc == 0) {
                    bsp_uart_puts("PASS ");
                    bsp_uart_puts(t->name);
                    /* Optional module-provided suffix, e.g.
                     * "PASS tim_ic (no signal, 0 Hz)". The suffix is DATA
                     * returned by the module (bsp_tim_ic pattern), keeping
                     * the console dependency on the shell side only. */
                    if (t->detail != NULL) {
                        const char *d = t->detail();
                        if ((d != NULL) && (d[0] != '\0')) {
                            bsp_uart_puts(" ");
                            bsp_uart_puts(d);
                        }
                    }
                    bsp_uart_puts("\r\n");
                } else {
                    /* Verdict format per spec: FAIL <name> step<N> */
                    bsp_uart_puts("FAIL ");
                    bsp_uart_puts(t->name);
                    bsp_uart_puts(" step");
                    /* decimal step number, tiny local formatter */
                    {
                        char d[4];
                        unsigned m = 0u;
                        unsigned v = (unsigned)rc;
                        do {
                            d[m++] = (char)('0' + (v % 10u));
                            v /= 10u;
                        } while (v > 0u);
                        while (m > 0u) {
                            char one[2];
                            one[0] = d[--m];
                            one[1] = '\0';
                            bsp_uart_puts(one);
                        }
                    }
                    /* FAIL also carries the module report: the raw ID,
                     * NACK step, wait that timed out -- diagnosis data
                     * belongs in the transcript, not behind a rebuild. */
                    if (t->detail != NULL) {
                        const char *d2 = t->detail();
                        if ((d2 != NULL) && (d2[0] != '\0')) {
                            bsp_uart_puts(" ");
                            bsp_uart_puts(d2);
                        }
                    }
                    bsp_uart_puts("\r\n");
                }
                return;
            }
        }
    }

    bsp_uart_puts("unknown: ");
    bsp_uart_puts(tok);
    bsp_uart_puts(" (try 'help')\r\n");
}

/* ---- main loop ---------------------------------------------------------------- */

void test_menu_run(void)
{
    static char line[MENU_LINE_MAX]; /* static: no malloc, small stack use */
    uint32_t len = 0u;
    uint32_t last_was_cr = 0u; /* set on CR commit; the LF of a CRLF pair clears it */
    const time_port_t *time = adapter_time();
    const digout_port_t *led = adapter_digout(APP_HEARTBEAT_PORT,
                                              APP_HEARTBEAT_PIN);
    uint32_t last_led_ms = time->tick_ms();
    uint32_t last_hb_ms = last_led_ms;

    /* The menu OWNS the console from here: register-level init (ownership
     * rule in bsp_uart.h). HAL did GPIO AF7 + clocks; this takes the CRs. */
    (void)bsp_uart_init();

    bsp_uart_puts("\r\n=== f407-periph-lab test menu (P3 DL-A5) ===\r\n");
    menu_print_help();
    bsp_uart_puts("menu> ");

    for (;;) {
        /* 1. RX polling + line assembly (non-blocking). */
        const int c = bsp_uart_getc_nonblock();
        if (c >= 0) {
            const char ch = (char)c;
            if (ch == '\r') {
                if (last_was_cr == 0u) {
                    last_was_cr = 1u; /* a following LF is just the pair tail */
                    line[len] = '\0';
                    bsp_uart_puts("\r\n");
                    menu_exec(line);
                    len = 0u;
                    bsp_uart_puts("menu> ");
                }
                /* CR right after CR: empty line, nothing to commit again */
            } else if (ch == '\n') {
                if (last_was_cr != 0u) {
                    last_was_cr = 0u; /* CRLF pair: LF already handled by CR */
                } else {
                    /* Bare LF (LF-only terminal): commit as a line end too */
                    line[len] = '\0';
                    bsp_uart_puts("\r\n");
                    menu_exec(line);
                    len = 0u;
                    bsp_uart_puts("menu> ");
                }
            } else if ((ch == '\b') || (ch == 0x7Fu)) {
                last_was_cr = 0u;
                if (len > 0u) {
                    len--;
                    bsp_uart_puts("\b \b"); /* erase, space, erase */
                }
            } else if ((ch >= 0x20u) && (ch <= 0x7Eu)) {
                last_was_cr = 0u;
                if (len < (MENU_LINE_MAX - 1u)) {
                    line[len++] = ch;
                    /* Echo the character (single-char puts is cheap here). */
                    {
                        char one[2];
                        one[0] = ch;
                        one[1] = '\0';
                        bsp_uart_puts(one);
                    }
                } /* else: line full, drop silently until CR */
            } /* else: control char, ignore */
        }

        /* 2. Visible LED heartbeat (LED_GREEN = PC5 via APP_HEARTBEAT_*).
         * Throttled: toggling every loop pass would run at MHz rate and the
         * LED would just look permanently half-lit -- 200 ms gives a clear
         * 2.5 Hz blink that says "loop alive" at a glance. */
        const uint32_t now = time->tick_ms();
        if ((now - last_led_ms) >= APP_MENU_HEARTBEAT_MS) {
            last_led_ms = now;
            led->toggle(); /* adapters.c: ODR check + BSRR write */
        }

        /* 3. RTT #H heartbeat: logcheck treats a stopped heartbeat as a hung
         * MCU, so the new main loop inherits the duty from the old blink
         * loop (APP_HEARTBEAT_PERIOD_MS = 1000). */
        if ((now - last_hb_ms) >= APP_HEARTBEAT_PERIOD_MS) {
            last_hb_ms = now;
            rtt_heartbeat();
        }

        /* 4. IWDG feed: the dog is armed (8.19 s nominal, LSI-tolerance
         * 5.6-15.4 s) -- the menu loop feeds it every pass so the board
         * stays alive; stopping the feed is exactly what the
         * wdg_reset_demo command demonstrates. (flash_iap's ~2.6 s
         * erase stays inside the window.) */
        bsp_wdg_iwdg_feed();
    }
}
