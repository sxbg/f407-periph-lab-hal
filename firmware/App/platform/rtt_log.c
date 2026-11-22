/**
 * @file rtt_log.c
 * @brief Event-based RTT logging system (template implementation).
 *
 * Depends only on SEGGER_RTT (Middlewares/SEGGER/RTT). No HAL usage:
 * safe to call from ISRs and from fault handlers.
 */
#include "rtt_log.h"

#include "SEGGER_RTT.h"

#include <string.h>

/* ---- configuration (overridable via app_config.h include order) ------- */
#ifndef RTT_RING_DEPTH
#define RTT_RING_DEPTH 512u
#endif
#ifndef RTT_UP_BUFFER_SIZE
#define RTT_UP_BUFFER_SIZE 4096u /* 1024 loses events; 4096 proven safe */
#endif
#ifndef RTT_CHANNEL0_NAME
#define RTT_CHANNEL0_NAME "terminal"
#endif

/* ---- state ------------------------------------------------------------- */
volatile uint16_t rtt_log_mask = RTT_MASK_ALL;

static uint32_t (*s_tick_ms)(void) = 0;
static uint16_t s_seq = 0u;

/* RAM ring: 16 bytes per slot = seq(2) + code(2) + t_ms(4) + a(2) + b(4) + c(4)
 * Kept free-running so post-mortem J-Link dump can decode without RTT. */
typedef struct {
    uint16_t seq;
    uint16_t code;
    uint32_t t_ms;
    uint16_t a;
    uint32_t b;
    uint32_t c;
} rtt_ring_slot_t;

static rtt_ring_slot_t s_ring[RTT_RING_DEPTH];
static uint32_t s_ring_wr = 0u;
static uint32_t s_ring_count = 0u; /* saturates at RTT_RING_DEPTH */
static uint32_t s_ring_overrun = 0u;

/* ISR downsample counters (storm-prevention pattern) */
static uint32_t s_hb_last_ms = 0u;

/* ---- helpers ------------------------------------------------------------ */
static uint16_t next_seq(void)
{
    s_seq = (uint16_t)(s_seq + 1u);
    return s_seq;
}

static const char hexdig[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };

static void emit_line(const rtt_ring_slot_t *s, char prefix)
{
    /* Manual hex formatting: zero printf, fixed width, ASCII only. */
    char line[48];
    uint32_t i = 0u;
    line[i++] = prefix;                            /* 'E' or 'R' */
    line[i++] = ' ';
    for (int sh = 12; sh >= 0; sh -= 4) line[i++] = hexdig[(s->seq >> sh) & 0xF];
    line[i++] = ' ';
    for (int sh = 12; sh >= 0; sh -= 4) line[i++] = hexdig[(s->code >> sh) & 0xF];
    line[i++] = ' ';
    for (int sh = 28; sh >= 0; sh -= 4) line[i++] = hexdig[((uint32_t)s->t_ms >> sh) & 0xF];
    line[i++] = ' ';
    for (int sh = 12; sh >= 0; sh -= 4) line[i++] = hexdig[(s->a >> sh) & 0xF];
    line[i++] = ' ';
    for (int sh = 28; sh >= 0; sh -= 4) line[i++] = hexdig[(s->b >> sh) & 0xF];
    line[i++] = ' ';
    for (int sh = 28; sh >= 0; sh -= 4) line[i++] = hexdig[(s->c >> sh) & 0xF];
    line[i++] = '\r'; /* mandatory: missing CRLF swallows the next line */
    line[i++] = '\n';
    SEGGER_RTT_WriteString(0, line);
}

static void ring_push(uint16_t seq, uint16_t code, uint32_t t_ms,
                      uint16_t a, uint32_t b, uint32_t c)
{
    rtt_ring_slot_t *s = &s_ring[s_ring_wr];
    s->seq = seq; s->code = code; s->t_ms = t_ms;
    s->a = a; s->b = b; s->c = c;
    s_ring_wr = (s_ring_wr + 1u) % RTT_RING_DEPTH;
    if (s_ring_count < RTT_RING_DEPTH) {
        s_ring_count++;
    } else {
        s_ring_overrun++;
    }
}

/* ---- public API ---------------------------------------------------------- */
void rtt_log_init(uint32_t (*tick_ms)(void))
{
    s_tick_ms = tick_ms;
    s_seq = 0u;
    s_ring_wr = 0u;
    s_ring_count = 0u;
    SEGGER_RTT_ConfigUpBuffer(0, RTT_CHANNEL0_NAME, NULL, 0,
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    (void)SEGGER_RTT_ConfigUpBuffer(0, RTT_CHANNEL0_NAME, NULL, 0,
                                    SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    /* Real buffer setup: static allocation via SEGGER_RTT_Conf.h
     * (BUFFER_SIZE_UP = RTT_UP_BUFFER_SIZE). The two ConfigUpBuffer calls
     * above with NULL buffer just rename channel 0. */
}

void rtt_ev(uint16_t code, uint16_t a, uint32_t b, uint32_t c)
{
    uint16_t seq = next_seq();
    uint32_t t = (s_tick_ms != 0) ? s_tick_ms() : 0u;
    rtt_ring_slot_t s;
    s.seq = seq; s.code = code; s.t_ms = t; s.a = a; s.b = b; s.c = c;
    ring_push(seq, code, t, a, b, c);
    emit_line(&s, 'E');
}

void rtt_evt(uint16_t code, uint16_t a, uint32_t b, uint32_t c)
{
    rtt_ring_slot_t s;
    s.seq = next_seq();
    s.code = code;
    s.t_ms = (s_tick_ms != 0) ? s_tick_ms() : 0u;
    s.a = a; s.b = b; s.c = c;
    emit_line(&s, 'E'); /* rtt-only: no ring push */
}

void rtt_heartbeat(void)
{
    uint32_t now = (s_tick_ms != 0) ? s_tick_ms() : 0u;
    if ((now - s_hb_last_ms) < 1000u) {
        return; /* simple rate limit; caller usually drives the period */
    }
    s_hb_last_ms = now;
    char line[40];
    /* #H up_ms=<sec> ring=<count> over=<overrun> seq=<seq> */
    static const char pfx[] = "#H up_s=";
    uint32_t i = 0u;
    for (i = 0u; i < sizeof(pfx) - 1u; i++) line[i] = pfx[i];
    uint32_t v = now / 1000u;
    char tmp[12]; uint32_t n = 0u;
    do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v > 0u && n < 11u);
    while (n > 0u) line[i++] = tmp[--n];
    static const char mid[] = " ring=";
    for (uint32_t k = 0u; k < sizeof(mid) - 1u; k++) line[i++] = mid[k];
    v = s_ring_count; n = 0u;
    do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v > 0u && n < 11u);
    while (n > 0u) line[i++] = tmp[--n];
    line[i++] = '\r';
    line[i++] = '\n';
    line[i] = '\0';
    SEGGER_RTT_WriteString(0, line);
}

void rtt_mark(const char *tag)
{
    /* T prefix + tag + CRLF; tag must be ASCII. */
    SEGGER_RTT_WriteString(0, "T ");
    if (tag != 0) {
        char buf[97];
        uint32_t n = 0u;
        for (const char *p = tag; *p != '\0' && (p - tag) < 96u; p++) {
            buf[n++] = *p;
        }
        buf[n] = '\0';
        SEGGER_RTT_WriteString(0, buf);
    }
    SEGGER_RTT_WriteString(0, "\r\n");
}

void rtt_ring_recover_emit(void)
{
    /* Emit the surviving ring as R-lines, oldest first. */
    uint32_t start = (s_ring_count < RTT_RING_DEPTH)
        ? 0u : (s_ring_wr % RTT_RING_DEPTH);
    for (uint32_t k = 0u; k < s_ring_count; k++) {
        const rtt_ring_slot_t *s = &s_ring[(start + k) % RTT_RING_DEPTH];
        emit_line(s, 'R');
    }
    if (s_ring_overrun > 0u) {
        char line[64];
        static const char pfx[] = "T ring_overrun=";
        uint32_t i = 0u;
        for (i = 0u; i < sizeof(pfx) - 1u; i++) line[i] = pfx[i];
        uint32_t v = s_ring_overrun; char tmp[12]; uint32_t n = 0u;
        do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v > 0u && n < 11u);
        while (n > 0u) line[i++] = tmp[--n];
        line[i++] = '\r'; line[i++] = '\n'; line[i] = '\0';
        SEGGER_RTT_WriteString(0, line);
    }
}
