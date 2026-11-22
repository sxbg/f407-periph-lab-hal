/**
 * @file test_menu.c
 * @brief UART command menu: registry-driven selftest runner on USART1 (HAL
 *        edition of f407-periph-lab shell/test_menu.c -- same registry
 *        semantics and command names so the two editions diff cleanly).
 *
 * Design (bsp-module-spec.md "自测试约定"):
 *  - Registry-driven: adding a peripheral = one line in s_module_tests.
 *    Menu numbering stays stable; built-ins (help/reboot) are separate.
 *  - Status semantics match the WorkManger verdicts: PASS <name> /
 *    FAIL <name> step<N> (the module returns the step; shell prints).
 *  - The loop never returns: RX polling is the main loop, with a throttled
 *    LED heartbeat, the RTT #H heartbeat, and the WDG refresh helper kept
 *    alive (IWDG is armed at boot since MX_IWDG_Init runs pre-main).
 */
#include "test_menu.h"

#include <stddef.h>

#include "hal_port.h"    /* CMSIS: NVIC_SystemReset */
#include "ports.h"       /* time_port_t (tick_ms) */
#include "adapters.h"    /* adapter_time / adapter_digout (toggle) */
#include "bsp_uart.h"
#include "bsp_gpio.h"
#include "bsp_tim_pwm.h"
#include "bsp_tim_basic.h"
#include "bsp_tim_ic.h"
#include "bsp_tim_encoder.h"
#include "bsp_tim_adv.h"
#include "bsp_can_loopback.h"
#include "bsp_crc_rng.h"
#include "bsp_rtc.h"
#include "bsp_wdg.h"
#include "bsp_pwr.h"
#include "bsp_flash_iap.h"
#include "bsp_i2c_eeprom.h"
#include "bsp_spi_flash.h"
#include "bsp_sdio_tf.h"
#include "bsp_adc.h"
#include "bsp_dac.h"
#include "bsp_dma.h"
#include "bsp_exti.h"
#include "bsp_oled.h"
#include "rtt_log.h"
#include "app_config.h"

/* ---- forward decls ---------------------------------------------------------- */

static void menu_put_uint(unsigned v);

/* ---- registry -------------------------------------------------------------- */

typedef int (*bsp_selftest_fn)(void); /* returns 0 = PASS, else step number */
typedef const char *(*bsp_detail_fn)(void); /* verdict-line suffix, or NULL */

typedef struct {
    const char *name;
    bsp_selftest_fn run;
    const char *desc;
    bsp_detail_fn detail;
} menu_test_t;

/* Shell-layer CLOSED-LOOP commands: they compose two bsp modules through
 * their public APIs ONLY (bsp-module-spec rule #7). 0 = PASS, non-zero. */
static int cmd_dac_adc_loop(void);
static int cmd_pwm_ic_loop(void);

/* Order fixed by the P3 spec; never reorder, append at the END. */
static const menu_test_t s_module_tests[] = {
    { "uart", bsp_uart_selftest,
      "USART1 115200 8N1 (BRR readback, HAL TX, RX timeout)", NULL },
    { "gpio", bsp_gpio_selftest,
      "PC5/PB2 LEDs, PA0/PA1 keys, PA15 BL (HAL write/read)", NULL },
    { "tim_pwm", bsp_tim_pwm_selftest,
      "TIM1_CH1=PA8 + TIM4_CH1=PD12 1 kHz PWM (HAL PWM config/start)", NULL },
    { "tim_basic", bsp_tim_basic_selftest,
      "TIM5 1 Hz timebase, polled update flag (IRQ demo deferred W6)",
      bsp_tim_basic_last_report },
    { "tim_ic", bsp_tim_ic_selftest,
      "TIM2_CH2=PB3 input capture freq meter (32-bit, 1s window)",
      bsp_tim_ic_last_report },
    { "tim_enc", bsp_tim_encoder_selftest,
      "TIM3 encoder PB4/PB5 x4 quadrature (CNT wrap, rpm vs PPR)",
      bsp_tim_encoder_last_report },
    { "tim_adv", bsp_tim_adv_selftest,
      "TIM8 CH1=PC6 + CH2=PC7/CH2N=PB14 comp PWM 1us deadtime", NULL },
    { "can_lb", bsp_can_loopback_selftest,
      "CAN1 PB8/PB9 500k loopback (HAL filter/TX/RX frame)",
      bsp_can_loopback_last_report },
    { "crc_rng", bsp_crc_rng_selftest,
      "CRC32 hw (MPEG-2 vectors) + RNG (DRDY/CECS, reads)", NULL },
    { "rtc", bsp_rtc_selftest,
      "RTC LSE calendar 1Hz + alarm + 20x BKPxR (HAL RTC API)",
      bsp_rtc_last_report },
    { "wdg", bsp_wdg_selftest,
      "IWDG/WWDG armed-dog verification + CSR reset flags (read-only)",
      bsp_wdg_last_report },
    { "pwr", bsp_pwr_selftest,
      "Sleep mode (WFI) + standby flag readback (Stop = demo only)", NULL },
    { "flash_iap", bsp_flash_iap_selftest,
      "Flash IAP last sector 7 param page (HAL erase/program)",
      bsp_flash_iap_last_report },
    { "i2c_ee", bsp_i2c_eeprom_selftest,
      "I2C1 100k + onboard 24C02 (Mem_Write/Read, tWR poll)",
      bsp_i2c_eeprom_last_report },
    { "spi_flash", bsp_spi_flash_selftest,
      "SPI1 5.25MHz + W25Q128 (JEDEC 9F, page program/verify, DPD wake)",
      bsp_spi_flash_last_report },
    { "sdio_tf", bsp_sdio_tf_selftest,
      "SDIO 4-bit TF card identification (CID/CSD parse)",
      bsp_sdio_tf_last_report },
    { "adc", bsp_adc_selftest,
      "ADC1 4-rank scan CH8/10/14+Vrefint, SWSTART poll (mV math)",
      bsp_adc_last_report },
    { "dac", bsp_dac_selftest,
      "DAC OUT1=PA4 software mode (SetValue/GetValue roundtrip)", NULL },
    { "dma", bsp_dma_selftest,
      "USART1_TX via DMA2_Stream7 (HAL_UART_Transmit_DMA completion)",
      NULL },
    { "exti", bsp_exti_selftest,
      "EXTI1 <- KEY2 press counter (6s interactive window)",
      bsp_exti_last_report },
    { "dac_adc_loop", cmd_dac_adc_loop,
      "closed loop: DAC setpoints -> ADC CH10 reads (jumper PA4->PC0)",
      NULL },
    { "pwm_ic_loop", cmd_pwm_ic_loop,
      "closed loop: TIM1 1kHz PWM -> TIM2 capture (jumper PA8->PB3)",
      NULL },
    { "oled", bsp_oled_selftest,
      "SH1107 OLED soft-I2C PD9/PE15 (EP11 bit-bang teaching line)",
      bsp_oled_last_report },
};

#define MENU_TESTS_N ((int)(sizeof(s_module_tests) / sizeof(s_module_tests[0])))

/* ---- demo commands (NOT selftests -- they reset or park the board) ------- */

typedef void (*menu_demo_fn)(void);

typedef struct {
    const char *name;
    const char *desc;
    menu_demo_fn run;
} menu_demo_t;

static void demo_wdg_reset(void)
{
    bsp_uart_puts("DEMO resets board: independent watchdog is armed from "
                  "boot; stop typing and wait ~8 s.\r\n");
    /* The IWDG never gets fed once this loop blocks. */
    for (;;) {
        /* spin: the armed IWDG will reset the board */
    }
}

static void demo_pwr_stop(void)
{
    bsp_uart_puts("DEMO enters Stop: press KEY1 (PA0) to wake; heartbeat "
                  "pauses, SystemInit re-runs on wake.\r\n");
    bsp_pwr_enter_stop();
    bsp_uart_puts("DEMO: woke via EXTI0, wake count ");
    {
        unsigned v = bsp_pwr_exti0_wake_count();
        char buf[12];
        unsigned m = 0u;
        do {
            buf[m++] = (char)('0' + (v % 10u));
            v /= 10u;
        } while (v > 0u);
        while (m > 0u) {
            char one[2];
            one[0] = buf[--m];
            one[1] = '\0';
            bsp_uart_puts(one);
        }
    }
    bsp_uart_puts("; SystemInit re-ran (168 MHz back).\r\n");
}

static const menu_demo_t s_menu_demos[] = {
    { "wdg_test",
      "DEMO resets board: IWDG armed at boot, never feed -> reset loop",
      demo_wdg_reset },
    { "wdg_reset_demo",
      "DEMO resets board: same as wdg_test (wave-3 name kept)",
      demo_wdg_reset },
    { "pwr_stop_demo",
      "DEMO enters Stop: press KEY1 (PA0) to wake; heartbeat pauses",
      demo_pwr_stop },
};

#define MENU_DEMOS_N ((int)(sizeof(s_menu_demos) / sizeof(s_menu_demos[0])))

/* ---- closed-loop commands ---------------------------------------------------- */

/* DAC setpoint -> ADC CH10 (PC0) reads, jumper PA4->PC0 required. */
static int cmd_dac_adc_loop(void)
{
    static const unsigned setpoints_mv[3] = { 500u, 1650u, 2800u };
    int step;

    if ((bsp_dac_init() != 0) || (bsp_adc_init() != 0)) {
        return 1;
    }
    /* Oscilloscope probe point (user-requested 2026-11-22): hold 1650 mV
     * for 500 ms -- scope on PA4 shows the flat level while the ADC reads
     * the same node back through the jumper. */
    bsp_dac_set_mv(1650u);
    HAL_Delay(500);
    {
        const unsigned probe_mv = bsp_adc_read_mv(BSP_ADC_CH_IN10_PC0);
        bsp_uart_puts("  probe: pa4=1650 read(PC0)=");
        menu_put_uint(probe_mv);
        bsp_uart_puts(" mV\r\n");
    }

    for (step = 0; step < 3; step++) {
        const unsigned target = setpoints_mv[step];
        unsigned read_mv;
        unsigned delta;
        bsp_dac_set_mv(target); /* drive PA4, let it settle ~2 tau */
        HAL_Delay(2);
        read_mv = bsp_adc_read_mv(BSP_ADC_CH_IN10_PC0);
        delta = (read_mv > target) ? (read_mv - target)
                                   : (target - read_mv);
        bsp_uart_puts("  target=");
        menu_put_uint(target);
        bsp_uart_puts(" read(PC0)=");
        menu_put_uint(read_mv);
        bsp_uart_puts(" pc1=");
        menu_put_uint(bsp_adc_read_mv(11u));
        bsp_uart_puts(" pc4=");
        menu_put_uint(bsp_adc_read_mv(14u));
        bsp_uart_puts(" mV\r\n");
        if (delta > 60u) {
            return 2 + step;
        }
    }
    return 0;
}

/* TIM1 PWM 1 kHz -> TIM2 capture, jumper PA8->PB3 required. */
static int cmd_pwm_ic_loop(void)
{
    unsigned hz;

    if ((bsp_tim_pwm_init() != 0) || (bsp_tim_ic_init() != 0)) {
        return 1;
    }
    {
        const uint32_t start = adapter_time()->tick_ms();
        while ((adapter_time()->tick_ms() - start) < 1100u) {
            /* let the 1s measurement window run */
        }
    }
    hz = bsp_tim_ic_measure_freq();
    if ((hz < 950u) || (hz > 1050u)) {
        return 2;
    }
    return 0;
}

/* ---- tiny helpers ------------------------------------------------------------ */

static void menu_put_uint(unsigned v)
{
    char d[10];
    unsigned m = 0u;
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

/* ---- line handling ---------------------------------------------------------- */

#define MENU_LINE_MAX 40u

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

static void menu_exec(const char *line)
{
    char tok[MENU_LINE_MAX];
    unsigned n = 0u;

    while (*line == ' ') {
        line++;
    }
    while ((line[n] != '\0') && (line[n] != ' ') && (n < (MENU_LINE_MAX - 1u))) {
        tok[n] = line[n];
        n++;
    }
    tok[n] = '\0';
    if (n == 0u) {
        return;
    }

    /* Built-ins first. */
    if ((tok[0] == 'h') && (tok[1] == 'e') && (tok[2] == 'l') &&
        (tok[3] == 'p') && (tok[4] == '\0')) {
        menu_print_help();
        return;
    }
    if ((tok[0] == 'r') && (tok[1] == 'e') && (tok[2] == 'b') &&
        (tok[3] == 'o') && (tok[4] == 'o') && (tok[5] == 't') &&
        (tok[6] == '\0')) {
        bsp_uart_puts("rebooting...\r\n");
        rtt_mark("MENU: NVIC_SystemReset");
        NVIC_SystemReset();
    }

    /* Demo commands: each prints its own warning BEFORE running. */
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

    /* Registry. */
    {
        int i;
        for (i = 0; i < MENU_TESTS_N; i++) {
            const menu_test_t *t = &s_module_tests[i];
            unsigned k = 0u;
            while ((t->name[k] != '\0') && (t->name[k] == tok[k])) {
                k++;
            }
            if ((t->name[k] == '\0') && (tok[k] == '\0')) {
                const int rc = t->run();
                if (rc == 0) {
                    bsp_uart_puts("PASS ");
                    bsp_uart_puts(t->name);
                    if (t->detail != NULL) {
                        const char *d = t->detail();
                        if ((d != NULL) && (d[0] != '\0')) {
                            bsp_uart_puts(" ");
                            bsp_uart_puts(d);
                        }
                    }
                    bsp_uart_puts("\r\n");
                } else {
                    bsp_uart_puts("FAIL ");
                    bsp_uart_puts(t->name);
                    bsp_uart_puts(" step");
                    menu_put_uint((unsigned)rc);
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
    static char line[MENU_LINE_MAX];
    uint32_t len = 0u;
    uint32_t last_was_cr = 0u;
    const time_port_t *time = adapter_time();
    const digout_port_t *led = adapter_digout(APP_HEARTBEAT_PORT,
                                              APP_HEARTBEAT_PIN);
    uint32_t last_led_ms = time->tick_ms();
    uint32_t last_hb_ms = last_led_ms;

    (void)bsp_uart_init();

    bsp_uart_puts("\r\n=== f407-periph-lab-hal test menu (HAL edition) ===\r\n");
    menu_print_help();
    bsp_uart_puts("menu> ");

    for (;;) {
        const int c = bsp_uart_getc_nonblock();
        if (c >= 0) {
            const char ch = (char)c;
            if (ch == '\r') {
                if (last_was_cr == 0u) {
                    last_was_cr = 1u;
                    line[len] = '\0';
                    bsp_uart_puts("\r\n");
                    menu_exec(line);
                    len = 0u;
                    bsp_uart_puts("menu> ");
                }
            } else if (ch == '\n') {
                if (last_was_cr != 0u) {
                    last_was_cr = 0u;
                } else {
                    line[len] = '\0';
                    bsp_uart_puts("\r\n");
                    menu_exec(line);
                    len = 0u;
                    bsp_uart_puts("menu> ");
                }
            } else if ((ch == 0x08u) || (ch == 0x7Fu)) {
                if (len > 0u) {
                    len--;
                    bsp_uart_puts("\b \b");
                }
            } else if ((ch >= 0x20u) && (ch < 0x7Fu) &&
                       (len < (MENU_LINE_MAX - 1u))) {
                line[len++] = ch;
                {
                    char echo[2];
                    echo[0] = ch;
                    echo[1] = '\0';
                    bsp_uart_puts(echo);
                }
            } else {
                /* drop control chars / overflow silently */
            }
        }

        /* Throttled LED heartbeat + RTT #H + WDG feed (IWDG armed at boot). */
        {
            const uint32_t now = time->tick_ms();
            if ((now - last_led_ms) >= APP_HEARTBEAT_PERIOD_MS) {
                last_led_ms = now;
                led->toggle();
            }
            if ((now - last_hb_ms) >= 1000u) {
                last_hb_ms = now;
                rtt_heartbeat();
            }
            bsp_wdg_refresh();
        }
    }
}
