/**
 * @file rtt_log.h
 * @brief Event-based RTT logging system (template).
 *
 * Design rules (encoded lessons from real projects):
 *  - Event table single source of truth: App/config/rtt_events.json ->
 *    generated header rtt_log_events.h. Never hand-edit the generated header.
 *  - Event line: "E <seq> <code> <t_ms> <a> <b> <c>\r\n" (~37 bytes, zero
 *    printf). Field a is 16-bit only; values that may exceed 0xFFFF MUST go
 *    into b or c (16-bit truncation once produced days of false diagnosis).
 *  - RAM ring 512 x 16B: post-mortem recovery when RTT is disconnected.
 *    Events marked ring+rtt are stored; rtt-only high-rate events are not.
 *  - Runtime subsystem mask (volatile rtt_log_mask): flip via J-Link RAM
 *    write, no rebuild needed.
 *  - Heartbeat #H line every second is the liveness probe; logcheck treats
 *    a stopped heartbeat as a hung MCU.
 *  - Text lines (rtt_mark) MUST end with \r\n or the next line is swallowed.
 *  - ISR discipline: only rtt_evt() from ISRs; downsample high-rate sources
 *    (an unconsumer storm measured ~2300 events/s and drowned everything).
 */
#ifndef RTT_LOG_H
#define RTT_LOG_H

#include <stdint.h>
#include <stddef.h>

#include "rtt_log_events.h" /* generated from rtt_events.json */

/* Runtime subsystem enable mask; write via J-Link to toggle w/o rebuild. */
extern volatile uint16_t rtt_log_mask;

/* Called once from main after clock init. tick_ms returns free-running ms. */
void rtt_log_init(uint32_t (*tick_ms)(void));

/* Store to RAM ring + emit to RTT (survives crash if ring recovered). */
void rtt_ev(uint16_t code, uint16_t a, uint32_t b, uint32_t c);

/* Emit to RTT only (high-rate events; not stored in ring). */
void rtt_evt(uint16_t code, uint16_t a, uint32_t b, uint32_t c);

/* Heartbeat summary line, call from main loop every heartbeat period. */
void rtt_heartbeat(void);

/* Text anchor line (ASCII only, \r\n appended automatically). */
void rtt_mark(const char *tag);

/* Post-mortem: dump the RAM ring as "R ..." lines to RTT (call at boot to
 * recover the previous run's tail, e.g. on watchdog reset with ring intact). */
void rtt_ring_recover_emit(void);

#endif /* RTT_LOG_H */
