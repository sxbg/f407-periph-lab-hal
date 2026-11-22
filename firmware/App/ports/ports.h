/**
 * @file ports.h
 * @brief Port interfaces (pure, no HAL). Implementations live in adapters/.
 *
 * The domain layer depends on these interfaces only; host tests supply fakes.
 * This is the dependency-inversion seam between domain and hardware.
 */
#ifndef PORTS_H
#define PORTS_H

#include <stdint.h>
#include <stdbool.h>

/* ---- time port ------------------------------------------------------------ */
typedef struct {
    uint32_t (*tick_ms)(void);       /* free-running milliseconds */
    void     (*delay_ms)(uint32_t);  /* blocking delay (adapters choose impl) */
} time_port_t;

/* ---- digital output port ---------------------------------------------------- */
typedef struct {
    void (*set)(bool level);         /* drive pin high/low */
    void (*toggle)(void);            /* flip output */
} digout_port_t;

/* ---- log port ---------------------------------------------------------------- */
typedef struct {
    void (*write)(const char *s);
    void (*line)(const char *s);
} log_port_t;

#endif /* PORTS_H */
