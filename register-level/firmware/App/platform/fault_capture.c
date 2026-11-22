/**
 * @file fault_capture.c
 * @brief HardFault现场捕获实现（family-neutral，寄存器经 hal_port.h/CMSIS 通用名）。
 *
 * Fault-context rules: no function calls beyond trivial ones, no RTT write
 * here (the fault may be IN the RTT path). We only park the frame; the
 * replay happens after reset in a healthy context.
 */
#include "fault_capture.h"
#include "hal_port.h"
#include "rtt_log.h"

/* noinit: survives the fault reset (linker keeps it out of .bss init).
 * armcc: __attribute__((section(".noinit"), zero_init)); the scatter file
 * must place UNINIT over RAM. Template note: when the scatter file is not
 * hand-managed, keep this in .bss -- the frame then only survives
 * soft-reset-style faults, still far better than nothing. */
#if defined(__ARMCC_VERSION)
__attribute__((section(".bss.noinit"), zero_init))
#endif
static fault_frame_t s_frame;

static volatile uint32_t s_seq_high = 0; /* increments per capture */

// cppcheck-suppress constParameterPointer ; ASM trampoline owns the calling convention
void fault_capture_handler_c(uint32_t *sp)
{
    s_frame.magic   = FAULT_FRAME_MAGIC;
    s_seq_high++;
    s_frame.seq_and_active = ((s_seq_high & 0xFFFFu) | (1u << 16u));
    s_frame.r0  = sp[0];
    s_frame.r1  = sp[1];
    s_frame.r2  = sp[2];
    s_frame.r3  = sp[3];
    s_frame.r12 = sp[4];
    s_frame.lr  = sp[5];
    s_frame.pc  = sp[6];
    s_frame.psr = sp[7];
#if HAL_HAS_CFSR
    {
        volatile const uint32_t *scb_cfsr  = (volatile const uint32_t *)0xE000ED28u;
        volatile const uint32_t *scb_hfsr  = (volatile const uint32_t *)0xE000ED2Cu;
        volatile const uint32_t *scb_bfar  = (volatile const uint32_t *)0xE000ED38u;
        volatile const uint32_t *scb_mmfar = (volatile const uint32_t *)0xE000ED34u;
        volatile const uint32_t *scb_icsr  = (volatile const uint32_t *)0xE000ED04u;
        s_frame.cfsr  = *scb_cfsr;
        s_frame.hfsr  = *scb_hfsr;
        s_frame.bfar  = *scb_bfar;
        s_frame.mmfar = *scb_mmfar;
        s_frame.icsr  = *scb_icsr;
    }
#else
    {
        volatile uint32_t *scb_icsr = (volatile uint32_t *)0xE000ED04u;
        s_frame.icsr = *scb_icsr;
    }
#endif
    s_frame.reset_flags = RCC->CSR;

    /* Pull the reset line: NVIC_SystemReset is a CMSIS inline (no call).
     * The active=1 write above is INTENTIONALLY overwritten: if the reset
     * request failed, a later capture still sees an inactive (consumed)
     * frame. cppcheck's reassignment warning is a false positive here. */
    // cppcheck-suppress redundantAssignment ; deliberate fail-safe (see above)
    s_frame.seq_and_active &= 0x0000FFFFu; /* active=0: consumed */
    *(volatile uint32_t *)0xE000ED0Cu = 0x05FA0004u; /* AIRCR SYSRESETREQ */
    for (;;) { } /* wait for the reset to land */
}

int fault_capture_pending(void)
{
    return (s_frame.magic == FAULT_FRAME_MAGIC) ? 1 : 0;
}

void fault_capture_report_at_boot(void)
{
    if (s_frame.magic != FAULT_FRAME_MAGIC) {
        return;
    }
    s_frame.magic = 0u; /* consume */

    /* Two FAULT events carry the whole frame (a=16-bit only; PC/CFSR
     * class values go into b/c per the field-width rule). */
    rtt_ev(RTT_EV_FAULT_PC,
           (uint16_t)(s_frame.seq_and_active & 0xFFFFu),
           s_frame.pc,          /* b: faulting PC (symbolized on host) */
           s_frame.lr);         /* c: return address */
#if HAL_HAS_CFSR
    rtt_ev(RTT_EV_FAULT_CFSR,
           (uint16_t)(s_frame.icsr & 0xFFu),
           s_frame.cfsr,        /* b: CFSR (decoded on host) */
           s_frame.bfar);       /* c: BFAR (valid when CFSR.BFARVALID) */
#else
    rtt_ev(RTT_EV_FAULT_CFSR,
           (uint16_t)(s_frame.icsr & 0xFFu),
           0u, 0u);
#endif
}
