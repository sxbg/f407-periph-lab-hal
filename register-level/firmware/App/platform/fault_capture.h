/**
 * @file fault_capture.h
 * @brief HardFault现场捕获：异常帧压入 ring 保留槽，复位后回放为事件。
 *
 * Design (roadmap U2, works on every Cortex-M):
 *  - The exception stack frame (R0-R3, R12, LR, PC, PSR) is uniform
 *    across Cortex-M; CFSR/HFSR/BFAR/MMFAR exist on M3+ only (pack
 *    gate: HAL_HAS_CFSR; M0 falls back to a reduced frame + reset flags).
 *  - The frame lands in a PERSISTENT noinit region: after a fault reset
 *    the frame from the PREVIOUS run is still there, gets replayed as a
 *    FAULT event at boot (see fault_capture_report_at_boot).
 *  - Nothing here may call HAL or printf: this runs in fault context.
 */
#ifndef FAULT_CAPTURE_H
#define FAULT_CAPTURE_H

#include <stdint.h>

#include "hal_port.h"   /* HAL_HAS_CFSR gate (family pack, ADR-0002) */

/* Frame persisted across the fault reset (noinit section).
 * Field order is 4-byte aligned throughout (no implicit padding: armcc
 * A1581W padding warnings are build gate noise). */
typedef struct {
    uint32_t magic;          /* FAULT_FRAME_MAGIC when valid */
    uint32_t seq_and_active; /* low u16 = seq, high u16 = active */
    uint32_t r0, r1, r2, r3; /* stacked (basic frame) */
    uint32_t r12, lr, pc, psr;
#if HAL_HAS_CFSR
    uint32_t cfsr, hfsr, bfar, mmfar, icsr;
#else
    uint32_t icsr;           /* M0: SCB->ICSR carries the active vector */
#endif
    uint32_t reset_flags;    /* RCC->CSR snapshot at capture time */
} fault_frame_t;

#define FAULT_FRAME_MAGIC 0xFA17F0C7u

/* Call once at boot (before any RTT event) to replay a previous fault. */
void fault_capture_report_at_boot(void);

/* Returns 1 when a previous-run frame is pending (consumed by report). */
int fault_capture_pending(void);

#endif /* FAULT_CAPTURE_H */
