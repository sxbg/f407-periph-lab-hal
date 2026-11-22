#include "blink_state.h"

void blink_init(blink_state_t *s, uint32_t period_ms)
{
    s->period_ms = period_ms > 0u ? period_ms : 1u;
    s->elapsed_ms = 0u;
    s->on = false;
}

bool blink_tick(blink_state_t *s, uint32_t dt_ms)
{
    s->elapsed_ms += dt_ms;
    if (s->elapsed_ms >= s->period_ms) {
        s->elapsed_ms -= s->period_ms;
        s->on = !s->on;
        return true;
    }
    return false;
}
