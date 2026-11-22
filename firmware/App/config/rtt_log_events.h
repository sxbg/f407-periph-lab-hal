/* GENERATED from rtt_events.json by `asm events-gen`. DO NOT EDIT.
 * Regenerate: python -m autostm32 events-gen
 */

/* source sha256-16: a8ae7d88d3bb3178 */
#define RTT_TABLE_SHA256_16 0xa8ae7d88u

#define RTT_LINE_FMT "E %04x %04x %08x %04x %08x %08x\r\n"

/* Event codes: (subsystem << 8) | index within subsystem table */
#define RTT_EV_BOOT 0x0100u
#define RTT_EV_HEARTBEAT 0x0101u
#define RTT_EV_ERR_HANDLER 0x0102u
#define RTT_EV_ASSERT 0x0103u
#define RTT_EV_CLOCK_FALLBACK 0x0104u
#define RTT_EV_WATCHDOG_RESET 0x0105u
#define RTT_EV_MARK 0x0106u
#define RTT_EV_FAULT_PC 0x0107u
#define RTT_EV_FAULT_CFSR 0x0108u
#define RTT_EV_LED_TOGGLE 0x0200u

/* Runtime subsystem masks (J-Link may write rtt_log_mask directly) */
#define RTT_MASK_ALL 0xFFFFu
#define RTT_MASK_CORE 0x0001u
#define RTT_MASK_APP 0x0002u
#define RTT_MASK_LOG 0x0003u

/* ring channel membership: 1 = ring+rtt (survives crash), 0 = rtt only */
#define RTT_EV_RING_IN_DEFAULT 0u
#define RTT_EV_BOOT_IN_RING 1u
#define RTT_EV_HEARTBEAT_IN_RING 0u
#define RTT_EV_ERR_HANDLER_IN_RING 1u
#define RTT_EV_ASSERT_IN_RING 1u
#define RTT_EV_CLOCK_FALLBACK_IN_RING 1u
#define RTT_EV_WATCHDOG_RESET_IN_RING 1u
#define RTT_EV_MARK_IN_RING 1u
#define RTT_EV_FAULT_PC_IN_RING 1u
#define RTT_EV_FAULT_CFSR_IN_RING 1u
#define RTT_EV_LED_TOGGLE_IN_RING 0u

/*
 * Field width rule (from rtt_events.json field_rule):
 *   field a is 16-bit only; any value that may exceed 0xFFFF MUST go into b or c (16-bit truncation once produced two rounds of false conclusions)
 *
 * void rtt_ev(uint16_t code, uint16_t a, uint32_t b, uint32_t c);
 *   a is 16-bit: values that may exceed 0xFFFF MUST go into b or c.
 */
