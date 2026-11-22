/**
 * @file blink_state.h
 * @brief Domain example: pure heartbeat-blink state machine.
 *
 * Domain rules: no HAL, no register access, no includes beyond this layer.
 * Compiles on the host with `cl /std:c11 /W4 /WX` -- that is the mechanical
 * red line enforced by `asm hosttest`.
 */
#ifndef BLINK_STATE_H
#define BLINK_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t period_ms;      /* full on+off period */
    uint32_t elapsed_ms;     /* internal accumulator */
    bool on;                 /* current logical level */
} blink_state_t;

void blink_init(blink_state_t *s, uint32_t period_ms);

/* Advance by dt_ms; returns true when the logical level changed. */
bool blink_tick(blink_state_t *s, uint32_t dt_ms);

#endif /* BLINK_STATE_H */
