# Bring-up Log

> Format discipline: one numbered problem = one entry with four sections
> (Symptom / Root cause / Fix / Re-test evidence). Never merge multiple
> defects into one entry; never delete entries (supersede instead).

## Environment (measured)

| Item | Value |
|---|---|
| Board | (fill: board name/revision) |
| Debugger | J-Link, SWD 4000 kHz |
| Device (J-Link name) | (fill) |
| HSE measured | (fill: some boards differ from schematic — verify) |
| First build | (fill: date, Code/RO/RW/ZI sizes) |

## Problems

<!-- TEMPLATE — copy per problem:
### P001 — <one-line title>

- **Date**: YYYY-MM-DD
- **Symptom**: what was observed (logs, scope, behavior).
- **Root cause**: the mechanism, with evidence (log excerpt / register value).
- **Fix**: what changed (file, function), one commit.
- **Re-test evidence**: the log/RTT capture proving the fix (paste lines).
-->
