# Bring-up Log

> Format discipline: one numbered problem = one entry with four sections
> (Symptom / Root cause / Fix / Re-test evidence). Never merge multiple
> defects into one entry; never delete entries (supersede instead).

## Environment (measured)

| Item | Value |
|---|---|
| Board | F407VE dev board v5.5 (板载 CMSIS-DAP) |
| Debugger | J-Link, SWD 4000 kHz |
| Device (J-Link name) | STM32F407VE (板载 DAP 走 Keil CMSIS_AGDI) |
| HSE measured | 25 MHz（原理图 X1 + Radio/ws2812 实盘工程一致；上板 BOOT 日志 clock=168000000Hz 终验）|
| First build | 2026-11-21 Code=24014 RO=2558 RW=88 ZI=20328（16 bsp 模块全量，0 err 0 warn）|

## Problems

<!-- TEMPLATE — copy per problem:
### P001 — <one-line title>

- **Date**: YYYY-MM-DD
- **Symptom**: what was observed (logs, scope, behavior).
- **Root cause**: the mechanism, with evidence (log excerpt / register value).
- **Fix**: what changed (file, function), one commit.
- **Re-test evidence**: the log/RTT capture proving the fix (paste lines).
-->
